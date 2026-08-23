# 基於 STM32F407VG 之伺服器開機時序監控與熱管理系統

> BMC（Baseboard Management Controller）核心功能模擬

以 STM32F407VG 實作簡化版的伺服器管理控制器，模擬主機板開機時序的**監督流程**與**智慧熱管理**。整合 GPIO 時序驗證、I2C 溫度感測、PWM 風扇控制與故障事件記錄。

---

## Demo 影片

| 影片 | 內容 | 連結 |
|---|---|---|
| 01 | 正常開機流程與熱管理（含遲滯控制） | _（待補）_ |
| 02 | 故障偵測：12V Power-Good 逾時 | _（待補）_ |

---

## 專案定位

真實伺服器的電源管理是**兩個角色分工**的：

| 角色 | 由誰執行 | 職責 |
|---|---|---|
| 執行端 | CPLD / FPGA / 專用 sequencer IC<br>（TI UCD90320、ADI ADM1266） | 送 Enable 命令、微秒級控制軌序、反序關機 |
| **監督端** | **BMC**（如 ASPEED AST2500/2600 跑 OpenBMC） | **驗證時序是否正常完成、逾時判定故障、記錄事件、控制散熱、回報管理者** |

**本專案模擬的是監督端。** 電源軌不由 STM32 實際控制，PB0/PB1/PB2 作為三條電軌的 Power-Good 回饋訊號來源，程式負責依序驗證、計時、判定逾時、記錄事件並管理散熱。

---

## 系統架構

```
                    ┌─────────────────────────┐
                    │      STM32F407VG        │
                    │                         │
  PA0  ────────────▶│  電源按鍵                │
                    │                         │
  PB0  ────────────▶│  3.3V  Power-Good       │
  PB1  ────────────▶│  12V   Power-Good       │──┐
  PB2  ────────────▶│  VCORE Power-Good       │  │ 開機時序驗證
                    │                         │  │ (含 timeout 保護)
  PB6/PB7 ◀────────▶│  I2C1 ── LM75A          │──┤
                    │                         │  │ 溫度監控 (1 Hz)
  PA8  ◀────────────│  TIM1_CH1 PWM 1 kHz     │──┤
                    │       └─ L9110 ─ 風扇    │  │ 遲滯風扇控制
                    │                         │  │
  PE8  ◀────────────│  綠 LED  (FAN OFF)      │──┤
  PE10 ◀────────────│  黃 LED  (FAN MID)      │  │ 狀態指示
  PE12 ◀────────────│  紅 LED  (FAN HIGH/FAULT)──┤
                    │                         │  │
       SWO ◀────────│  ITM printf             │──┘ 診斷輸出 + Event Log
                    └─────────────────────────┘
```

---

## 硬體

| 項目 | 型號 / 說明 |
|---|---|
| 開發板 | STM32F4DISCOVERY（STM32F407VGT6，Cortex-M4，168 MHz） |
| 溫度感測器 | LM75A，I2C，11-bit，解析度 0.125 °C |
| 風扇驅動 | L9110 H 橋模組 + DC 風扇 |
| 狀態指示 | 綠 / 黃 / 紅 LED（各串 220 Ω） |
| 診斷輸出 | SWO / ITM（SWV ITM Data Console） |

### 接線

| STM32 | 對象 | 說明 |
|---|---|---|
| PA0 | 板載 User 按鍵（B1） | 電源鍵 |
| PB0 / PB1 / PB2 | 3.3V（杜邦線） | Power-Good 模擬，內部下拉 |
| PB6 / PB7 | LM75A SCL / SDA | I2C1 |
| PA8 | L9110 `INA` | TIM1_CH1 PWM |
| GND | L9110 `INB` | **必須接地，不可浮空** |
| 5V / GND | L9110 VCC / GND | |
| PE8 / PE10 / PE12 | 綠 / 黃 / 紅 LED | 各串 220 Ω 到 GND |

> **注意**：`INA` 與 GND 之間建議並接 10 kΩ 下拉電阻。MCU 上電至韌體初始化完成前 PA8 為浮空狀態，而 L9110 輸入無內部下拉，會被判定為高電位而使馬達啟動。詳見〈開發過程〉。

---

## 功能

### 1. 開機時序驗證

按下電源鍵後，先執行**開機前健康檢查**（pre-boot health check）：確認溫度感測器可通訊、且溫度未超過臨界值，兩者皆通過才啟動時序。

接著依序驗證三條電軌的 Power-Good：

```
3.3V PG  →  12V PG  →  VCORE PG  →  SYSTEM UP
```

每一軌都有獨立的 timeout 保護（5 秒，demo 值）。任一軌逾時即判定該軌啟動失敗，記錄對應故障碼並進入故障鎖定狀態。

### 2. 熱管理（雙門檻遲滯）

每秒讀取一次 LM75A，依溫度切換三段風扇轉速：

| 檔位 | PWM Duty | LED |
|---|---|---|
| `FAN_OFF` | 0 % | 綠 |
| `FAN_MID` | 70 % | 黃 |
| `FAN_HIGH` | 100 % | 紅 |

**升溫與降溫使用不同門檻**，中間為遲滯區：

```
   溫度上升 ────────────────────────────────▶
        32.0      33.0      34.0      35.0      50.0
          │         │         │         │         │
   OFF ───┴───────▶ MID ───────────────▶ HIGH   CRITICAL
          ◀─────────┘         ◀──────────┘
   溫度下降 ◀────────────────────────────────
```

### 3. 故障處理與事件記錄

| 故障碼 | 觸發條件 |
|---|---|
| `TEMP_SENSOR_ERROR` | I2C 連續 3 次讀取失敗 |
| `OVER_TEMPERATURE` | 溫度 ≥ 臨界值 |
| `3V3_PG_TIMEOUT` | 3.3V Power-Good 逾時 |
| `12V_PG_TIMEOUT` | 12V Power-Good 逾時 |
| `VCORE_PG_TIMEOUT` | VCORE Power-Good 逾時 |
| `INVALID_STATE` | 狀態機進入非預期狀態 |

故障發生時記錄至**環形事件緩衝區**（時間戳記、故障當下的系統狀態、故障碼），並自動傾印。故障為鎖定式（latched），需 Reset 才能復原；鎖定期間可按電源鍵重新查詢記錄。

---

## 狀態機

```
        ┌──────────────────────────────────────────┐
        │                                          │
        ▼                                          │ 按鍵
   ┌─────────┐   按鍵 + 健康檢查通過   ┌───────────┐ │
   │ STANDBY │ ──────────────────────▶│ POWER_ON  │ │
   └─────────┘                        └───────────┘ │
        ▲                                   │       │
        │                          三軌 PG 全部通過  │
        │                                   ▼       │
        │                            ┌────────────┐ │
        └────────────────────────────│ SYSTEM_UP  │─┘
                     按鍵            └────────────┘
                                           │
        ┌──────────┐   PG 逾時 / 感測器故障 / 過溫
        │  FAULT   │◀──────────────────────┘
        └──────────┘
             │
         需 Reset
```

> `POWER_ON` 狀態在主迴圈中不會被觀察到——因為時序驗證採阻塞式實作，程式在該狀態時仍在 `Power_On_Sequence()` 內部。此狀態存在的目的是讓事件記錄能標示「故障發生於開機階段」。

---

## 設計說明

### 為什麼順序是 3.3V → 12V → VCORE

**1. 控制電源優先。** 3.3V 供給管理控制器、時序邏輯與 VR 控制器的 bias。負責監督時序的裝置必須先運作。

**2. 電源樹依賴關係。** 12V 是主功率輸入，餵給所有下游 VRM。父軌尚未穩定就啟用子軌，轉換器會因 UVLO 反覆 hiccup。

**3. 核心電源最後。** VCORE 由 12V 降壓產生，且處理器的 I/O 電源必須先就緒；否則電流會經由 I/O 的 ESD 箝位二極體倒灌進未上電的核心軌，造成漏電甚至 latch-up。

此外，逐軌錯開上電可避免各軌突波電流疊加超過電源供應器的過流保護限制。

> 補充：真實 ATX 電源供應器的 3.3V/5V/12V 實際上是同時上升、僅輸出單一合併的 PWR_OK 訊號。逐軌 Power-Good 是**板端 POL 轉換器**的行為，因此本專案定位為「板級電源時序」。

### 為什麼需要遲滯

LM75A 的解析度為 0.125 °C，溫度在單一門檻附近必然會抖動。若採用單一門檻，風扇會每個量測週期就切換一次狀態。

遲滯的本質是：**輸出不只取決於當前輸入，也取決於歷史狀態**。因此程式必須保存 `fan_level` 這個狀態變數——僅調整門檻數值無法達成。

同一個溫度（例如 32.5 °C）可能對應到不同的風扇檔位，取決於溫度是在上升還是下降。硬體上的對應概念為**施密特觸發器（Schmitt trigger）**。

### 為什麼過溫時風扇是 100% 而非關閉

**熱失效安全（thermal fail-safe）**：過溫時應該加強散熱，而非停止散熱。因此 `OVER_TEMPERATURE` 故障會將風扇強制拉到 100%，其餘故障才停止風扇。

### 為什麼 Power-Good 輸入使用內部下拉

**安全預設值原則**：訊號線斷開時輸入讀到低電位，被解讀為「電軌未就緒」而非「電軌正常」。訊號遺失絕不應該看起來像健康狀態。

### 為什麼 PWM 頻率是 1 kHz

本專案的風扇是 DC 馬達透過 L9110 H 橋驅動，PWM 直接切換馬達供電，低頻可提供較好的起動扭力，也在該驅動 IC 的切換能力範圍內。

若改用伺服器常見的 **4-wire PWM 風扇**則完全不同：該類型的 PWM 是送給風扇內部控制器的**指令訊號**、馬達本身持續供電，依 Intel 4-Wire PWM Controlled Fans 規範應設定在 **21–28 kHz**，以避開人耳可聽範圍。

### 為什麼採用阻塞式實作

等待 Power-Good 時程式停留在 `Wait_Power_Good()` 內，最長 5 秒 × 3 軌。

- **優點**：時序流程由上而下閱讀，邏輯清晰。
- **缺點**：該期間主迴圈停擺，溫度監控不會執行。開機途中若發生過熱無法偵測。
- **改進方向**：將等待迴圈改為每次主迴圈評估一次的 `switch(state)`，以 `HAL_GetTick()` 比對進入時間取代阻塞延遲，使時序驗證與溫度監控得以並行。

---

## Console 輸出範例

**正常開機與熱管理**

```
=========================================
 STM32 POWER SEQUENCING AND THERMAL DEMO
=========================================
PG order : 3.3V -> 12V -> VCORE
Fan PWM  : 1 kHz (L9110 H-bridge)
State    : STANDBY
Press PA0 to start.

[BTN ] Power button pressed.
[CHK ] 31.00 C OK.
======== POWER ON SEQUENCE ========
[SEQ ] Waiting for 3.3V  PG ... OK (1852 ms)
[SEQ ] Waiting for 12V   PG ... OK (2431 ms)
[SEQ ] Waiting for VCORE PG ... OK (1975 ms)
======== SYSTEM UP ================

[MON ] t=00:32.104  TEMP= 32.50 C  FAN=  0%  LEVEL=0
[MON ] t=00:33.104  TEMP= 33.25 C  FAN=  0%  LEVEL=0
[THRM] 33.25 C >= 33.0 C -> FAN MID
[FAN ] Duty = 70%
[MON ] t=00:34.104  TEMP= 34.00 C  FAN= 70%  LEVEL=1
...
[MON ] t=00:48.104  TEMP= 32.62 C  FAN= 70%  LEVEL=1   ← 已低於 33.0 但仍維持 MID
[MON ] t=00:52.104  TEMP= 31.87 C  FAN= 70%  LEVEL=1
[THRM] 31.87 C <= 32.0 C -> FAN OFF
[FAN ] Duty = 0%
```

**故障偵測**

```
[SEQ ] Waiting for 3.3V  PG ... OK (12 ms)
[SEQ ] Waiting for 12V   PG ... TIMEOUT after 5001 ms
====================================
[FAULT] 12V_PG_TIMEOUT
[STATE] POWER_ON
[TIME ] 00:29.067
====================================

-------- EVENT LOG (1 entries) --------
[01] t=00:29.067  state=POWER_ON    fault=12V_PG_TIMEOUT
---------------------------------------

[FAULT] System latched in FAULT state.
[FAULT] Press PA0 to print the log, or reset to recover.
```

---

## 開發環境與建置

| 項目 | 版本 / 設定 |
|---|---|
| IDE | STM32CubeIDE |
| HAL | STM32Cube F4 HAL |
| 系統時脈 | HSE 8 MHz → PLL → SYSCLK 168 MHz |
| TIM1 | PSC = 167、ARR = 999 → PWM 1 kHz |
| I2C1 | 100 kHz（標準模式） |

### 建置前設定

1. **浮點 printf**：Project → Properties → C/C++ Build → Settings → MCU Settings → 勾選 `Use float with printf from newlib-nano`
2. **SWV**：Debug Configurations → Debugger → 啟用 Serial Wire Viewer，Core Clock 設 `168.0` MHz
3. 進入 Debug 後開啟 SWV ITM Data Console，勾選 Port 0，並按下 **Start Trace**

### 溫度門檻校準

門檻值需依環境溫度調整。先讀取 `[MON ]` 的室溫值 `T0`，再依下表設定。現行值以室溫約 31 °C 校準：

| 巨集 | 現行值 | 調整方式 |
|---|---|---|
| `TEMP_MID_ON` | 33.0 °C | T0 + 2.0 |
| `TEMP_MID_OFF` | 32.0 °C | T0 + 1.0 |
| `TEMP_HIGH_ON` | 35.0 °C | T0 + 4.0 |
| `TEMP_HIGH_OFF` | 34.0 °C | T0 + 3.0 |
| `TEMP_CRITICAL` | 50.0 °C | 固定值，不隨室溫調整 |

兩段遲滯帶皆為 1.0 °C（MID 33.0/32.0、HIGH 35.0/34.0）。

> `TEMP_CRITICAL` 須明顯高於體溫（約 36 °C），否則以手指加熱測試遲滯時會直接觸發過溫保護。

---

## 開發過程中解決的問題

實作過程中遇到的問題與根因分析，記錄於此。

### 1. H 橋輸入浮空導致馬達反向全速

**現象**：Console 顯示風扇 0%，實際卻全速運轉。

**分析**：L9110 為 H 橋，馬達行為由 `INA`/`INB` 兩支輸入的組合決定：

| INA | INB | 馬達 |
|---|---|---|
| 0 | 0 | 停止 |
| PWM | 0 | 正轉，轉速 = duty |
| 0 | 1 | **反轉全速** |
| 1 | 1 | 煞車 |

當時僅接了 `INA`，`INB` 浮空。L9110 輸入無內部下拉，浮空被判定為高電位，因此 `INA=0, INB=1` 變成反轉全速。

**解法**：`INB` 接地。

### 2. 上電瞬間馬達自行啟動

**現象**：韌體尚未開始執行，風扇即已運轉。

**分析**：STM32 Reset 後 GPIO 預設為輸入高阻抗。PWM 腳位實際由 `HAL_TIM_MspPostInit()`（`stm32f4xx_hal_msp.c`）設定，在該函式執行完成前 PA8 為浮空狀態，同樣被 L9110 判定為高電位。

**解法**：`INA` 與 GND 之間並接 10 kΩ 下拉電阻，使硬體本身具備安全預設值。軟體僅能縮短浮空時間窗（Reset 到初始化完成之間無法由軟體涵蓋），因此此問題必須由硬體解決。

**延伸**：此為**上電預設安全狀態**（power-on safe state）的典型案例。風扇提前運轉影響有限，但若該接腳控制的是加熱器或繼電器，即為安全問題。

### 3. 初始化繞過抽象層導致軟硬體狀態不一致

**現象**：風扇持續全速，且後續所有轉速設定皆無效。

**分析**：初始化時直接以 `__HAL_TIM_SET_COMPARE()` 寫入 compare 暫存器，繞過了 `Set_Fan_Speed()` 內部的 duty 轉換；而該函式又有「數值未變更則不重設」的最佳化，導致錯誤的初始狀態被永久鎖住，之後再也無法修正。

**解法**：初始化改為呼叫 `Set_Fan_Speed()`，並先將 `current_fan_duty` 設為不可能出現的值，確保第一次寫入不會被去重檢查略過。

**心得**：初始化階段應走與平常相同的程式路徑；若函式含有快取或去重最佳化，必須確保開機時至少強制執行一次。

### 4. I2C 匯流排鎖死

**現象**：溫度讀取持續失敗，按 Reset 無法恢復，須拔除電源重新上電。

**分析**：I2C 採開汲極架構，從機亦可將 SDA 拉低。若主機在傳輸中途被重置，從機會停留在等待時脈的狀態並持續拉低 SDA，主機重新啟動後會將匯流排視為忙碌。由於是**從機**持有訊號線，重置主機無法解除。

**解法（本專案）**：整體斷電重新上電。

**產品化解法**：實作 **I2C bus recovery**——將 SCL 切換為一般 GPIO，手動送出 9 個時脈脈衝讓從機完成剩餘位元並釋放 SDA，補上 STOP 條件後重新初始化 I2C 週邊。本專案尚未實作。

### 5. printf 緩衝造成時序觀察失真

**現象**：Power-Good 逾時看起來瞬間發生，無法觀察到 5 秒等待。

**分析**：不含換行符號的 `printf` 會被 newlib 留在緩衝區，直到後續換行或緩衝區填滿才輸出，使得等待期間畫面無任何變化、逾時訊息與前一行一併出現。

**解法**：於 `main()` 開頭呼叫 `setvbuf(stdout, NULL, _IONBF, 0)` 關閉緩衝，並在時序訊息中加入實際等待毫秒數，使 timeout 保護的作用可由 log 直接驗證。

---

## 目前限制與後續規劃

| 項目 | 現況 | 規劃 |
|---|---|---|
| Enable 輸出 | 未實作，僅監督 PG | 增加三支 EN 輸出腳，開機逐軌拉高、關機依 VCORE → 12V → 3.3V 反序 de-assert |
| 執行期 PG 監控 | 進入 `SYSTEM_UP` 後不再檢查 | 持續監看 PG，電軌中途失效時應立即告警 |
| 電壓量測 | 僅讀取數位 PG 訊號 | 以 ADC 或 PMBus 讀取實際電壓，建立上下限告警 |
| 看門狗 | 未啟用 | 導入 IWDG，確保控制器軟體異常時可自動復位 |
| 事件記錄保存 | 僅存於 RAM，斷電即消失 | 寫入 Flash 或外部 EEPROM |
| 時序實作方式 | 阻塞式 | 改為非阻塞狀態機，使時序驗證與溫度監控並行 |
| 風扇轉速回授 | 無 | 以 Input Capture 讀取 tach 訊號，偵測風扇故障 |

---

## 參考資料

本專案的設計概念參考以下業界規範與元件行為模型（**專案本身未實作這些通訊協定**）：

- **TI UCD90320 / ADI ADM1266** — 電源時序控制器 datasheet，rail sequencing 與 Power-Good 逾時保護的行為模型
- **PMBus Power System Management Protocol Specification** — 電源時序參數定義（`TON_DELAY`、`TON_RISE`、`TON_MAX_FAULT_LIMIT`、`POWER_GOOD_ON`）
- **IPMI v2.0 Specification** — System Event Log（SEL）、感測器門檻分級、Chassis Control
- **Intel ATX / ATX12V Power Supply Design Guide** — PSON#、PWR_OK 等電源控制訊號
- **Intel 4-Wire PWM Controlled Fans Specification** — 4-line 風扇 PWM 頻率規範
- **NXP LM75A Datasheet** — 溫度暫存器格式與 I2C 位址
- **STMicroelectronics UM1472 / RM0090** — STM32F4DISCOVERY 使用手冊與 STM32F4 參考手冊
