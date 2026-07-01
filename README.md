# Server Power BMC Firmware (STM32F407)

一個以 STM32F407VG（STM32F4 Discovery）為平台的 **伺服器 BMC（Baseboard Management Controller）行為模擬韌體**。本專案以 bare-metal 方式（STM32 HAL）實作 BMC 最核心的幾項職責：**電源時序控制（Power Sequencing）**、**溫度監控與風扇散熱控制（Thermal Management）**、**故障保護（Fault Protection）** 以及 **事件記錄（Event Log）**，並透過非阻塞式狀態機（non-blocking state machine）串接整個開機流程。

---

## 功能特色

- **非阻塞式電源時序狀態機**：模擬伺服器上電時 3.3V → 12V → VCORE 逐軌開啟並等待各軌 Power-Good（PG）的流程，任一軌逾時（3 秒）即進入故障保護。
- **雙任務排程架構**：主迴圈同時運行週期性監控任務（Task A，每秒一次）與狀態機任務（Task B），監控在電源時序期間仍持續運作。
- **遲滯式（Hysteresis）風扇控制**：依系統溫度分成關閉 / 50% / 100% 三段，並以遲滯門檻避免臨界溫度時風扇反覆啟停。
- **PWM 風扇驅動含 Kickstart 與 Deadband 映射**：Active-low PWM 輸出，停轉時先短暫 100% 起轉，並將軟體 1~100% 映射到硬體有效工作區間 60~100%。
- **溫度感測（LM75A / I2C）**：透過 I2C 讀取溫度感測器，含感測器離線（讀取失敗）偵測。
- **故障保護與事件記錄**：溫度感測器錯誤、過溫、各電源軌 PG 逾時皆會觸發故障狀態，並寫入環狀事件緩衝區（Event Log）。
- **狀態指示 LED**：以綠 / 黃 / 紅 LED 對應風扇三段狀態。
- **SWO / ITM printf log 輸出**：重導 `_write` 至 ITM，方便透過 SWV 觀察系統即時狀態。

---

## 硬體平台

| 項目 | 說明 |
|------|------|
| MCU | STM32F407VGTx（STM32F4 Discovery） |
| 系統時脈 | HSE + PLL，168 MHz |
| Toolchain | STM32CubeIDE（GCC / HAL） |
| Debug/Log | SWO（ITM Port 0，`printf` 導向 SWV） |

---

## 腳位對應

| 功能 | 腳位 | 方向 / 說明 |
|------|------|------|
| 電源按鈕 | PA0 | Input，高電位觸發開機 |
| 3.3V PG | PB0 | Input（下拉），高電位表示 Power-Good |
| 12V PG | PB1 | Input（下拉），高電位表示 Power-Good |
| VCORE PG | PB2 | Input（下拉），高電位表示 Power-Good |
| 風扇 PWM | TIM1_CH1 | Active-low PWM 輸出 |
| 綠燈（風扇關閉） | PE8 | Output |
| 黃燈（風扇 50%） | PE10 | Output |
| 紅燈（風扇 100%） | PE12 | Output |
| 溫度感測器 | I2C1（LM75A，位址 0x90） | SCL/SDA |

> PB0/PB1/PB2 設為內部下拉，實測時可用外部訊號拉高來模擬各電源軌的 Power-Good。

---

## 狀態機流程

```
STANDBY ──(PA0 按下 & 溫度正常)──> 3V3_ON ──(PB0 PG)──> 12V_ON ──(PB1 PG)──> VCORE_ON ──(PB2 PG)──> SYSTEM_UP
   │                                  │                    │                     │
   │                                  └── PG 逾時 ─────────┴─────────────────────┘
   │                                                       │
   └──(溫度感測器錯誤 / 過溫)──────────────────────────────> FAULT
```

- **STANDBY**：待機，等待電源按鈕。按下時先做溫度檢查（感測器錯誤或過溫則拒絕開機）。
- **3V3_ON / 12V_ON / VCORE_ON**：逐軌開啟並等待各軌 PG；任一軌超過 `PG_TIMEOUT_MS`（3000 ms）未回報即進入 FAULT。
- **SYSTEM_UP**：系統完成開機，啟用散熱管理（Thermal Management）。
- **FAULT**：關閉風扇與 LED、記錄故障碼並停機（需重置板子復原）。

---

## 散熱控制邏輯

風扇採三段遲滯控制（`Thermal_Management`）：

| 目前風扇段位 | 升段條件 | 降段條件 |
|------|------|------|
| 0（關閉） | Temp ≥ 30.0 °C → 50% | — |
| 1（50%） | Temp ≥ 32.0 °C → 100% | Temp ≤ 28.5 °C → 關閉 |
| 2（100%） | — | Temp ≤ 31.0 °C → 50% |

- **過溫保護**：Temp ≥ 35.0 °C 觸發 `FAULT_OVERTEMP`。
- **感測器保護**：讀取失敗（回傳 `-999.0`）觸發 `FAULT_TEMP_SENSOR`。

> 上述溫度門檻為 Demo 用途，可依實測環境調整。

---

## 故障碼

| Fault Code | 說明 |
|------|------|
| `FAULT_NONE` | 無故障 |
| `FAULT_TEMP_SENSOR` | 溫度感測器讀取失敗 |
| `FAULT_OVERTEMP` | 系統過溫 |
| `FAULT_3V3_PG_TIMEOUT` | 3.3V 電源軌 PG 逾時 |
| `FAULT_12V_PG_TIMEOUT` | 12V 電源軌 PG 逾時 |
| `FAULT_VCORE_PG_TIMEOUT` | VCORE 電源軌 PG 逾時 |

故障發生時會寫入 `EventLog_t`（含 timestamp、fault code、發生時的狀態），採 10 筆環狀緩衝區。

---

## 專案結構

```
.
├── Src/                 # 應用程式原始碼（含 main.c）
├── Inc/                 # 標頭檔
├── Drivers/             # STM32 HAL 與 CMSIS 驅動
├── Middlewares/         # STM32 USB Host Library
├── Startup/             # 啟動檔
├── Debug/               # 建置輸出
├── Server_Power_V2.ioc  # STM32CubeMX 專案配置
├── STM32F407VGTX_FLASH.ld
└── STM32F407VGTX_RAM.ld
```

---

## 建置與燒錄

1. 以 **STM32CubeIDE** 開啟本專案（或用 `Server_Power_V2.ioc` 於 CubeMX 檢視/重新產生設定）。
2. 編譯後透過 ST-Link 燒錄至 STM32F4 Discovery。
3. 於 IDE 開啟 **SWV / ITM Data Console**（Core Clock 168 MHz，ITM Port 0）即可觀察 log。

### Log 範例

```
SERVER BMC FIRMWARE INITIALIZED
[STATE] STANDBY. Press PA0 to start boot sequence.
[CMD] Button Pressed. Temp OK 27.500 C. Booting...
[SEQ] 3.3V Enabled. Waiting for PG on PB0...
[SEQ] 3.3V PG OK.
[SEQ] 12V Enabled. Waiting for PG on PB1...
...
>>> SYSTEM FULLY UP. THERMAL MANAGEMENT ENABLED. <<<
[MONITOR], STATE:4, TEMP:27.500, FAN:0%, FAULT:NONE
```

---

## 操作方式

1. 上電後系統進入 **STANDBY**。
2. 拉高 **PA0** 模擬按下電源鍵，開始上電時序。
3. 依序拉高 **PB0 → PB1 → PB2** 模擬各電源軌 Power-Good，完成開機。
4. 進入 **SYSTEM_UP** 後，改變 LM75A 溫度即可觀察風扇分段與 LED 變化。
5. 溫度超過 35 °C 或感測器離線會進入 **FAULT**，重置板子後復原。

---
