
#include "main.h"
#include "usb_host.h"
#include <stdio.h>

I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar((*ptr++));
    }
    return len;
}

/* ==============================
 * BMC State Definition
 * ============================== */
typedef enum {
    STATE_STANDBY,      // Standby and waiting for power button
    STATE_3V3_ON,       // Enable / wait for 3.3V power-good
    STATE_12V_ON,       // Enable / wait for 12V power-good
    STATE_VCORE_ON,     // Enable / wait for VCORE power-good
    STATE_SYSTEM_UP,    // System boot completed
    STATE_FAULT         // Fault protection state
} ServerState_t;

/* ==============================
 * Fault Code Definition
 * ============================== */
typedef enum {
    FAULT_NONE = 0,
    FAULT_TEMP_SENSOR,
    FAULT_OVERTEMP,
    FAULT_3V3_PG_TIMEOUT,
    FAULT_12V_PG_TIMEOUT,
    FAULT_VCORE_PG_TIMEOUT
} FaultCode_t;

/* ==============================
 * Simple Event Log
 * ============================== */
#define EVENT_LOG_SIZE 10

typedef struct {
    uint32_t timestamp;
    FaultCode_t fault_code;
    ServerState_t state;
} EventLog_t;

EventLog_t event_log[EVENT_LOG_SIZE];
uint8_t event_log_index = 0;

/* ==============================
 * Global Variables
 * ============================== */
uint8_t current_fan_speed = 0;
FaultCode_t current_fault = FAULT_NONE;
ServerState_t currentState = STATE_STANDBY;
uint32_t state_enter_time = 0;
uint8_t state_entry_printed = 0;

#define PG_TIMEOUT_MS       3000
#define MONITOR_PERIOD_MS   1000

/* ==============================
 * Function Prototypes
 * ============================== */
void Change_State(ServerState_t next_state);
void Set_Fault(FaultCode_t fault);
void Add_Event_Log(FaultCode_t fault, ServerState_t state);
const char* Get_Fault_String(FaultCode_t fault);
void Thermal_Management(float sys_temp);
void Set_Fan_Speed(uint8_t speed_percent);
float Get_System_Temperature(void);

/* ==============================
 * Change State Function
 * ============================== */
void Change_State(ServerState_t next_state) {
    state_enter_time = HAL_GetTick();
    state_entry_printed = 0;
    currentState = next_state;
}

/* ==============================
 * Fault Handling Function
 * ============================== */
void Set_Fault(FaultCode_t fault) {
    current_fault = fault;
    Add_Event_Log(fault, currentState);
    Change_State(STATE_FAULT);
}

/* ==============================
 * Simple Event Log Function
 * ============================== */
void Add_Event_Log(FaultCode_t fault, ServerState_t state) {
    event_log[event_log_index].timestamp = HAL_GetTick();
    event_log[event_log_index].fault_code = fault;
    event_log[event_log_index].state = state;

    event_log_index++;
    if (event_log_index >= EVENT_LOG_SIZE) {
        event_log_index = 0;
    }
}

/* ==============================
 * Fault Code to String
 * ============================== */
const char* Get_Fault_String(FaultCode_t fault) {
    switch (fault) {
        case FAULT_NONE:
            return "NONE";
        case FAULT_TEMP_SENSOR:
            return "TEMP_SENSOR_ERROR";
        case FAULT_OVERTEMP:
            return "OVERTEMP";
        case FAULT_3V3_PG_TIMEOUT:
            return "3V3_PG_TIMEOUT";
        case FAULT_12V_PG_TIMEOUT:
            return "12V_PG_TIMEOUT";
        case FAULT_VCORE_PG_TIMEOUT:
            return "VCORE_PG_TIMEOUT";
        default:
            return "UNKNOWN_FAULT";
    }
}

/* ==============================
 * PWM Fan Control
 * ============================== */
void Set_Fan_Speed(uint8_t speed_percent) {
    if (speed_percent > 100) {
        speed_percent = 100;
    }

    /*
     * 注意：
     * 不要在這裡直接用 current_fan_speed == speed_percent 就 return。
     * 因為系統剛啟動時，軟體變數可能是 0%，
     * 但硬體 PWM compare value 不一定真的已經被設成停止。
     */

    if (speed_percent == 0) {
        current_fan_speed = 0;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);  // Active-low: 1000 = stop
        printf("[FAN] Speed set to 0%%\n");
        return;
    }

    /*
     * 如果不是 0%，而且軟體紀錄已經一樣，
     * 這時才可以避免重複設定。
     */
    if (current_fan_speed == speed_percent) {
        return;
    }

    current_fan_speed = speed_percent;

    /*
     * Kickstart:
     * 如果風扇目前是停止狀態，先給短暫 100% 起轉。
     */
    if (__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) == 1000) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        HAL_Delay(50);
    }

    /*
     * Deadband Mapping:
     * 軟體 1~100% 對應到硬體 60~100%
     */
    uint8_t hardware_power = 60 + ((speed_percent * 40) / 100);

    /*
     * Active-low PWM:
     * compare = 1000 代表停止
     * compare = 0 代表全速
     */
    uint32_t compare_value = 1000 - (hardware_power * 10);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare_value);

    printf("[FAN] Speed set to %d%%\n", current_fan_speed);
}
/* ==============================
 * I2C Temperature Read
 * ============================== */
float Get_System_Temperature(void) {
    uint8_t temp_buffer[2];
    int16_t raw_temp;
    uint16_t LM75A_ADDR = 0x90;

    if (HAL_I2C_Mem_Read(&hi2c1,
                         LM75A_ADDR,
                         0x00,
                         I2C_MEMADD_SIZE_8BIT,
                         temp_buffer,
                         2,
                         1000) == HAL_OK) {

        raw_temp = (temp_buffer[0] << 8) | temp_buffer[1];
        raw_temp = raw_temp >> 5;

        return raw_temp * 0.125f;
    }

    return -999.0f;
}

/* ==============================
 * Fan Hysteresis Thermal Control
 * ==============================
 *
 * Fan Level:
 * 0: fan off
 * 1: fan 50%
 * 2: fan 100%
 *
 * Hysteresis Example:
 * - Temp >= 25C: fan 100%
 * - Temp <= 22C: fan back to 50%
 * - Temp >= 20C: fan 50%
 * - Temp <= 18C: fan off
 */
void Thermal_Management(float sys_temp) {
    static uint8_t fan_level = 0;

    if (sys_temp == -999.0f) {
        Set_Fault(FAULT_TEMP_SENSOR);
        return;
    }

    /*
     * Over-temperature protection.
     * You can adjust this value depending on your demo environment.
     */
    if (sys_temp >= 35.0f) {
        Set_Fault(FAULT_OVERTEMP);
        return;
    }

    switch (fan_level) {
        case 0:
            if (sys_temp >= 30.0f) {
                fan_level = 1;
            }
            break;

        case 1:
            if (sys_temp >= 32.0f) {
                fan_level = 2;
            }
            else if (sys_temp <= 28.5f) {
                fan_level = 0;
            }
            break;

        case 2:
            if (sys_temp <= 31.0f) {
                fan_level = 1;
            }
            break;

        default:
            fan_level = 0;
            break;
    }

    if (fan_level == 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);     // Green LED
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
        Set_Fan_Speed(0);
    }
    else if (fan_level == 1) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);    // Yellow LED
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
        Set_Fan_Speed(50);
    }
    else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);    // Red LED
        Set_Fan_Speed(100);
    }
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2S3_Init();
    MX_SPI1_Init();
    MX_USB_HOST_Init();
    MX_TIM1_Init();

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);  // 先確保停止
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);  // 啟動後再確保一次
    current_fan_speed = 0;

    uint32_t last_monitor_time = HAL_GetTick();

    currentState = STATE_STANDBY;
    current_fault = FAULT_NONE;
    state_enter_time = HAL_GetTick();
    state_entry_printed = 0;

    printf("SERVER BMC FIRMWARE INITIALIZED\n");

    while (1)
    {
        MX_USB_HOST_Process();

        /*
         * ==============================
         * Task A: Periodic Monitor
         * ==============================
         * Runs every 1 second.
         * This task will still run during power sequencing,
         * because the state machine is now non-blocking.
         */
        if (HAL_GetTick() - last_monitor_time >= MONITOR_PERIOD_MS) {
            last_monitor_time = HAL_GetTick();

            float sys_temp = Get_System_Temperature();

            if (sys_temp != -999.0f) {
                printf("[MONITOR], STATE:%d, TEMP:%.3f, FAN:%d%%, FAULT:%s\n",
                       currentState,
                       sys_temp,
                       current_fan_speed,
                       Get_Fault_String(current_fault));
            }
            else {
                printf("[MONITOR], STATE:%d, TEMP:ERROR, FAN:%d%%, FAULT:%s\n",
                       currentState,
                       current_fan_speed,
                       Get_Fault_String(current_fault));
            }

            /*
             * Thermal management only works after system is fully up.
             */
            if (currentState == STATE_SYSTEM_UP) {
                Thermal_Management(sys_temp);
            }
        }

        /*
         * ==============================
         * Task B: Non-blocking State Machine
         * ==============================
         */
        switch (currentState) {

            case STATE_STANDBY:
            {
                if (!state_entry_printed) {
                    printf("[STATE] STANDBY. Press PA0 to start boot sequence.\n");
                    state_entry_printed = 1;
                }

                /*
                 * Wait for power button.
                 */
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                    float check_temp = Get_System_Temperature();

                    if (check_temp == -999.0f) {
                        printf("[REJECT] Temperature sensor error. Boot aborted.\n");
                        Set_Fault(FAULT_TEMP_SENSOR);
                    }
                    else if (check_temp >= 35.0f) {
                        printf("[REJECT] Over-temperature %.3f C. Boot aborted.\n", check_temp);
                        Set_Fault(FAULT_OVERTEMP);
                    }
                    else {
                        printf("[CMD] Button Pressed. Temp OK %.3f C. Booting...\n", check_temp);
                        current_fault = FAULT_NONE;
                        Change_State(STATE_3V3_ON);
                    }

                    /*
                     * Short delay only for button debounce.
                     * This is acceptable because it is short.
                     */
                    HAL_Delay(200);
                }
                break;
            }

            case STATE_3V3_ON:
            {
                if (!state_entry_printed) {
                    printf("[SEQ] 3.3V Enabled. Waiting for PG on PB0...\n");
                    state_entry_printed = 1;
                }

                /*
                 * If PB0 goes high before timeout, go to next rail.
                 */
                if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) {
                    printf("[SEQ] 3.3V PG OK.\n");
                    Change_State(STATE_12V_ON);
                }
                /*
                 * If timeout occurs, enter fault state.
                 */
                else if (HAL_GetTick() - state_enter_time >= PG_TIMEOUT_MS) {
                    printf("[FAULT] 3.3V PG timeout.\n");
                    Set_Fault(FAULT_3V3_PG_TIMEOUT);
                }
                break;
            }

            case STATE_12V_ON:
            {
                if (!state_entry_printed) {
                    printf("[SEQ] 12V Enabled. Waiting for PG on PB1...\n");
                    state_entry_printed = 1;
                }

                if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) {
                    printf("[SEQ] 12V PG OK.\n");
                    Change_State(STATE_VCORE_ON);
                }
                else if (HAL_GetTick() - state_enter_time >= PG_TIMEOUT_MS) {
                    printf("[FAULT] 12V PG timeout.\n");
                    Set_Fault(FAULT_12V_PG_TIMEOUT);
                }
                break;
            }

            case STATE_VCORE_ON:
            {
                if (!state_entry_printed) {
                    printf("[SEQ] VCORE Enabled. Waiting for PG on PB2...\n");
                    state_entry_printed = 1;
                }

                if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) {
                    printf("[SEQ] VCORE PG OK.\n");
                    printf(">>> SYSTEM FULLY UP. THERMAL MANAGEMENT ENABLED. <<<\n");
                    Change_State(STATE_SYSTEM_UP);
                }
                else if (HAL_GetTick() - state_enter_time >= PG_TIMEOUT_MS) {
                    printf("[FAULT] VCORE PG timeout.\n");
                    Set_Fault(FAULT_VCORE_PG_TIMEOUT);
                }
                break;
            }

            case STATE_SYSTEM_UP:
            {
                if (!state_entry_printed) {
                    printf("[STATE] SYSTEM_UP. Sensor monitoring and fan control active.\n");
                    state_entry_printed = 1;
                }

                /*
                 * Thermal management is handled by Task A every 1 second.
                 */
                break;
            }

            case STATE_FAULT:
            {
                if (!state_entry_printed) {
                    printf("[ALARM] FATAL ERROR. FAULT CODE: %s\n",
                           Get_Fault_String(current_fault));

                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);

                    Set_Fan_Speed(0);

                    printf("[EVENT LOG] Latest fault saved. System halted.\n");

                    state_entry_printed = 1;
                }

                /*
                 * Stay in fault state.
                 * You can reset the board to recover.
                 */
                break;
            }

            default:
            {
                Set_Fault(FAULT_NONE);
                break;
            }
        }
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin PE8 PE10 PE12 */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
