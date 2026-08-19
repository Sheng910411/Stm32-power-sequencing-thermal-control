/* =========================================================================
 * STM32F407VG Embedded Power Sequencing and Thermal Management System
 * -------------------------------------------------------------------------
 * Board  : STM32F4DISCOVERY / STM32F407VG
 * Author : Jiang Jia-Sheng
 *
 * -------------------------------------------------------------------------
 * PROJECT SCOPE
 * -------------------------------------------------------------------------
 * In a real server, rail sequencing is usually executed by a CPLD or a
 * dedicated power sequencer IC, while the BMC supervises the process:
 * it verifies that the sequence completes, declares a fault on timeout,
 * records events, and manages cooling.
 *
 * This project models the supervising side. The power rails are NOT
 * physically controlled by the STM32; PB0 / PB1 / PB2 are used to simulate
 * Power-Good feedback signals from three rails.
 *
 * -------------------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------------------
 * 1. GPIO
 *      PA0  : Power button
 *      PB0  : 3.3V  Power-Good input (simulated)
 *      PB1  : 12V   Power-Good input (simulated)
 *      PB2  : VCORE Power-Good input (simulated)
 *      PE8  : Green  LED  (fan off)
 *      PE10 : Yellow LED  (fan mid)
 *      PE12 : Red    LED  (fan high / fault)
 *
 * 2. Power Sequence
 *      Verify Power-Good in the order 3.3V -> 12V -> VCORE.
 *      Each step has timeout protection.
 *
 * 3. I2C Temperature Monitoring
 *      LM75A digital temperature sensor with a retry mechanism.
 *
 * 4. PWM Fan Control
 *      TIM1 CH1 (PE9) -> L9110 H-bridge driver -> DC fan.
 *      Three fan levels with temperature hysteresis.
 *
 * 5. Fault Handling
 *      Fault codes, circular event log, LED indication,
 *      and thermal fail-safe behaviour.
 *
 * -------------------------------------------------------------------------
 * FAN WIRING (L9110 H-bridge module)
 * -------------------------------------------------------------------------
 *      Module INA  <---- PE9  (TIM1_CH1 PWM output)
 *      Module INB  <---- GND        <-- must be grounded, not floating
 *      Module VCC  <---- 5V
 *      Module GND  <---- GND
 *
 *      L9110 truth table:
 *          INA=0   INB=0   -> stop
 *          INA=PWM INB=0   -> forward, speed = duty%      <-- used here
 *          INA=0   INB=1   -> reverse
 *          INA=1   INB=1   -> brake
 *
 * -------------------------------------------------------------------------
 * DESIGN NOTE : BLOCKING IMPLEMENTATION
 * -------------------------------------------------------------------------
 * The power sequence is implemented in a blocking style. While waiting for
 * a Power-Good signal the program stays inside Wait_Power_Good(), for up to
 * PG_TIMEOUT_MS per rail.
 *
 *   Benefit  : the sequence reads top-to-bottom and is easy to follow.
 *   Drawback : the main loop is stalled during that time, so temperature
 *              monitoring does not run while the system is booting.
 *
 * A non-blocking version would replace the wait loop with a switch(state)
 * that is evaluated once per main-loop iteration, using HAL_GetTick()
 * comparisons instead of blocking delays.
 * =========================================================================
 */

#include "main.h"
#include <stdio.h>

/* =========================================================================
 * Peripheral Handles
 * =========================================================================
 */
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;

/* =========================================================================
 * GPIO Definitions
 * =========================================================================
 */

/* Power button */
#define BTN_Pin             GPIO_PIN_0
#define BTN_Port            GPIOA

/* Power-Good inputs (simulated by external jumper wires) */
#define PG_3V3_Pin          GPIO_PIN_0
#define PG_12V_Pin          GPIO_PIN_1
#define PG_VCORE_Pin        GPIO_PIN_2
#define PG_Port             GPIOB

/* Status LEDs */
#define LED_GREEN_Pin       GPIO_PIN_8
#define LED_YELLOW_Pin      GPIO_PIN_10
#define LED_RED_Pin         GPIO_PIN_12
#define LED_Port            GPIOE

/* Fan PWM output is TIM1_CH1 on PE9.
 * The pin itself is configured in HAL_TIM_MspPostInit()
 * inside stm32f4xx_hal_msp.c.
 */

/* =========================================================================
 * System Parameters
 * =========================================================================
 */

/* -------------------------------------------------------------------------
 * LM75A temperature sensor
 *
 * The 7-bit I2C address is 0x48.
 * STM32 HAL expects an 8-bit address with the R/W bit position included,
 * so the value passed to the HAL API is 0x48 << 1 = 0x90.
 * -------------------------------------------------------------------------
 */
#define LM75A_ADDRESS       (0x48 << 1)
#define LM75A_TEMP_REG      0x00
#define I2C_TIMEOUT_MS      70
#define I2C_RETRY_MAX       3       /* transient NACK is common on I2C;    */
                                    /* declare a fault only after 3 fails  */

/* -------------------------------------------------------------------------
 * Power-Good timeout
 *
 * 若在指定時間內 PG 沒有拉高，判定該電源軌啟動失敗。
 *
 * 5000 ms is a demo value chosen so the Power-Good signals can be applied
 * by hand. On a real board a point-of-load regulator is normally expected
 * to assert Power-Good within tens of milliseconds.
 * -------------------------------------------------------------------------
 */
#define PG_TIMEOUT_MS       5000
#define RAIL_DELAY_MS       50      /* short gap between rails */

/* System monitoring period */
#define MONITOR_PERIOD_MS   1000

/* -------------------------------------------------------------------------
 * Temperature thresholds (hysteresis)
 *
 *   Temperature rising                Temperature falling
 *
 *      FAN_OFF                            FAN_HIGH
 *         |                                  |
 *      30.0 C  (TEMP_MID_ON)              31.0 C  (TEMP_HIGH_OFF)
 *         v                                  v
 *      FAN_MID                            FAN_MID
 *         |                                  |
 *      32.0 C  (TEMP_HIGH_ON)             28.5 C  (TEMP_MID_OFF)
 *         v                                  v
 *      FAN_HIGH                           FAN_OFF
 *
 * The ON threshold and the OFF threshold are deliberately different.
 * The gap between them is the hysteresis band.
 *
 * Without it, the LM75A resolution of 0.125 C would make the reading
 * fluctuate around a single threshold and the fan would switch state
 * every measurement cycle.
 *
 * Because the decision depends on the previous fan level as well as the
 * current temperature, a state variable (fan_level) is required.
 *
 * NOTE FOR DEMO:
 *   These values must be calibrated against the ambient temperature of the
 *   demo environment. Read the [MON ] output first, then set:
 *       TEMP_MID_ON   = ambient + 2.0
 *       TEMP_MID_OFF  = ambient + 0.5
 *       TEMP_HIGH_ON  = ambient + 4.0
 *       TEMP_HIGH_OFF = ambient + 3.0
 *       TEMP_CRITICAL = ambient + 12.0
 *   A fingertip is about 36 C, so TEMP_CRITICAL must stay well above it.
 * -------------------------------------------------------------------------
 */
#define TEMP_MID_ON         33.0f
#define TEMP_MID_OFF        32.0f
#define TEMP_HIGH_ON        35.0f
#define TEMP_HIGH_OFF       34.0f
#define TEMP_CRITICAL       50.0f

/* -------------------------------------------------------------------------
 * Fan duty for each level
 *
 * A small DC motor needs a minimum duty to start from standstill.
 * If the fan does not spin up at FAN_DUTY_MID, raise this value.
 * -------------------------------------------------------------------------
 */
#define FAN_DUTY_OFF        0
#define FAN_DUTY_MID        70
#define FAN_DUTY_HIGH       100

/* Event log depth */
#define EVENT_LOG_SIZE      10

/* =========================================================================
 * System State
 * =========================================================================
 */
typedef enum
{
    STATE_STANDBY = 0,
    STATE_POWER_ON,
    STATE_SYSTEM_UP,
    STATE_FAULT
} SystemState_t;

/* =========================================================================
 * Fault Code
 * =========================================================================
 */
typedef enum
{
    FAULT_NONE = 0,
    FAULT_TEMP_SENSOR,
    FAULT_OVERTEMP,
    FAULT_3V3_TIMEOUT,
    FAULT_12V_TIMEOUT,
    FAULT_VCORE_TIMEOUT,
    FAULT_INVALID_STATE
} FaultCode_t;

/* =========================================================================
 * Fan Level
 * =========================================================================
 */
typedef enum
{
    FAN_OFF = 0,
    FAN_MID,
    FAN_HIGH
} FanLevel_t;

/* =========================================================================
 * Event Log Entry
 * =========================================================================
 */
typedef struct
{
    uint32_t      timestamp;
    FaultCode_t   fault;
    SystemState_t state;
} EventLog_t;

/* =========================================================================
 * Global Variables
 * =========================================================================
 */
SystemState_t current_state    = STATE_STANDBY;
FaultCode_t   current_fault    = FAULT_NONE;
FanLevel_t    fan_level        = FAN_OFF;
uint8_t       current_fan_duty = 0;

EventLog_t    event_log[EVENT_LOG_SIZE];
uint8_t       event_log_head   = 0;   /* next write position */
uint8_t       event_log_count  = 0;   /* number of valid entries */

/* =========================================================================
 * Function Prototypes
 * =========================================================================
 */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);

/* Temperature */
HAL_StatusTypeDef Read_Temperature(float *temperature);

/* Fan and LED */
void Set_Fan_Speed(uint8_t duty_percent);
void Set_LED(FanLevel_t level);
void Thermal_Control(float temperature);

/* Power sequence */
HAL_StatusTypeDef Wait_Power_Good(uint16_t pg_pin, uint32_t timeout_ms);
void Power_On_Sequence(void);

/* Fault handling */
void Set_Fault(FaultCode_t fault);

/* Event log */
void Add_Event_Log(FaultCode_t fault, SystemState_t state);
void Dump_Event_Log(void);

/* Debug helpers */
const char *Get_Fault_String(FaultCode_t fault);
const char *Get_State_String(SystemState_t state);

void Error_Handler(void);

/* =========================================================================
 * printf redirection to SWV / ITM
 * -------------------------------------------------------------------------
 * To print floating point values, enable
 *   Project > Properties > C/C++ Build > Settings > MCU Settings
 *   "Use float with printf from newlib-nano"
 * =========================================================================
 */
int _write(int file, char *ptr, int len)
{
    (void)file;

    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }

    return len;
}

/* =========================================================================
 * Read LM75A Temperature
 * -------------------------------------------------------------------------
 * The LM75A temperature register holds 11 bits of data, left aligned:
 *
 *      byte0                 byte1
 *      D15 D14 ... D8        D7 D6 D5 | x x x x x
 *      <-------- 11 valid bits ------>  <- unused ->
 *
 * Steps:
 *   1. combine the two bytes into a 16-bit value
 *   2. shift right by 5 to extract the 11-bit value
 *   3. multiply by 0.125 C per LSB
 *
 * int16_t is used so that the right shift is arithmetic, which keeps
 * negative temperatures correct.
 * =========================================================================
 */
HAL_StatusTypeDef Read_Temperature(float *temperature)
{
    uint8_t data[2];
    int16_t raw;

    if (temperature == NULL)
    {
        return HAL_ERROR;
    }

    for (uint8_t retry = 0; retry < I2C_RETRY_MAX; retry++)
    {
        if (HAL_I2C_Mem_Read(&hi2c1,
                             LM75A_ADDRESS,
                             LM75A_TEMP_REG,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             2,
                             I2C_TIMEOUT_MS) == HAL_OK)
        {
            raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
            raw >>= 5;

            *temperature = raw * 0.125f;

            return HAL_OK;
        }

        HAL_Delay(5);
    }

    return HAL_ERROR;
}

/* =========================================================================
 * Fan PWM Control
 * -------------------------------------------------------------------------
 * TIM1 is on APB2. The APB2 prescaler is 2, so PCLK2 = 84 MHz and the
 * timer clock is doubled to 168 MHz.
 *
 *      PSC = 167   ->  counter clock = 168 MHz / 168 = 1 MHz
 *      ARR = 999   ->  period        = 1 MHz / 1000  = 1 kHz
 *
 * Why 1 kHz:
 *   The fan is a DC motor driven through an L9110 H-bridge, so the PWM
 *   signal switches the motor supply directly. A low frequency gives
 *   better starting torque and stays within the switching capability of
 *   this low-cost driver IC.
 *
 *   A standard 4-wire PWM fan works differently: the PWM signal is a
 *   command input to the fan's own controller while the motor is always
 *   powered. For that type, the Intel 4-Wire PWM Controlled Fans
 *   specification requires 21 to 28 kHz, which is above the audible range.
 *
 * The compare value is derived from the auto-reload register rather than
 * a hard-coded constant, so changing the PWM frequency only requires
 * changing MX_TIM1_Init().
 * =========================================================================
 */
void Set_Fan_Speed(uint8_t duty_percent)
{
    uint32_t arr;
    uint32_t compare;

    if (duty_percent > 100)
    {
        duty_percent = 100;
    }

    /* Skip the update if the duty has not changed.
     * This also prevents the log from being flooded once per second.
     */
    if (duty_percent == current_fan_duty)
    {
        return;
    }

    arr     = __HAL_TIM_GET_AUTORELOAD(&htim1);
    compare = ((uint32_t)(100 - duty_percent) * (arr + 1)) / 100;

    /* At 100% the compare value exceeds ARR, so the compare event never
     * occurs and the output stays high for the whole period. This is the
     * normal way to reach full duty in PWM mode 1.
     */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);

    current_fan_duty = duty_percent;

    printf("[FAN ] Duty = %u%%\n", (unsigned)current_fan_duty);
}

/* =========================================================================
 * Status LED
 * -------------------------------------------------------------------------
 *      Green   -> fan off
 *      Yellow  -> fan mid
 *      Red     -> fan high (and blinking in fault state)
 * =========================================================================
 */
void Set_LED(FanLevel_t level)
{
    HAL_GPIO_WritePin(LED_Port,
                      LED_GREEN_Pin | LED_YELLOW_Pin | LED_RED_Pin,
                      GPIO_PIN_RESET);

    switch (level)
    {
        case FAN_OFF:
            HAL_GPIO_WritePin(LED_Port, LED_GREEN_Pin, GPIO_PIN_SET);
            break;

        case FAN_MID:
            HAL_GPIO_WritePin(LED_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
            break;

        case FAN_HIGH:
            HAL_GPIO_WritePin(LED_Port, LED_RED_Pin, GPIO_PIN_SET);
            break;

        default:
            break;
    }
}

/* =========================================================================
 * Thermal Management
 * -------------------------------------------------------------------------
 * The next fan level depends on two things:
 *
 *      1. the current temperature
 *      2. the previous fan level
 *
 * Depending on the previous level is what creates the hysteresis:
 * the same temperature can map to different fan levels depending on
 * whether the temperature is rising or falling.
 * =========================================================================
 */
void Thermal_Control(float temperature)
{
    /* --- Critical temperature protection has priority --- */
    if (temperature >= TEMP_CRITICAL)
    {
        Set_Fault(FAULT_OVERTEMP);
        return;
    }

    /* --- Level transitions --- */
    if (fan_level == FAN_OFF)
    {
        /* Only one condition matters in this level: rising past MID_ON */
        if (temperature >= TEMP_MID_ON)
        {
            fan_level = FAN_MID;
            printf("[THRM] %.2f C >= %.1f C -> FAN MID\n",
                   temperature, TEMP_MID_ON);
        }
    }
    else if (fan_level == FAN_MID)
    {
        /* Rising uses HIGH_ON, falling uses MID_OFF.
         * The two thresholds are different on purpose.
         */
        if (temperature >= TEMP_HIGH_ON)
        {
            fan_level = FAN_HIGH;
            printf("[THRM] %.2f C >= %.1f C -> FAN HIGH\n",
                   temperature, TEMP_HIGH_ON);
        }
        else if (temperature <= TEMP_MID_OFF)
        {
            fan_level = FAN_OFF;
            printf("[THRM] %.2f C <= %.1f C -> FAN OFF\n",
                   temperature, TEMP_MID_OFF);
        }
    }
    else if (fan_level == FAN_HIGH)
    {
        /* Falls back at HIGH_OFF, which is lower than HIGH_ON */
        if (temperature <= TEMP_HIGH_OFF)
        {
            fan_level = FAN_MID;
            printf("[THRM] %.2f C <= %.1f C -> FAN MID\n",
                   temperature, TEMP_HIGH_OFF);
        }
    }
    else
    {
        /* Should never happen; fall back to a safe level */
        fan_level = FAN_OFF;
    }

    /* --- Apply the level --- */
    Set_LED(fan_level);

    switch (fan_level)
    {
        case FAN_OFF:
            Set_Fan_Speed(FAN_DUTY_OFF);
            break;

        case FAN_MID:
            Set_Fan_Speed(FAN_DUTY_MID);
            break;

        case FAN_HIGH:
            Set_Fan_Speed(FAN_DUTY_HIGH);
            break;

        default:
            Set_Fan_Speed(FAN_DUTY_OFF);
            break;
    }
}

/* =========================================================================
 * Wait for a Power-Good Signal
 * -------------------------------------------------------------------------
 * Waits until the selected Power-Good input goes high.
 * Returns HAL_TIMEOUT if that does not happen within timeout_ms.
 *
 * This function is blocking. It is the only blocking wait in the project.
 *
 * The elapsed-time test uses unsigned subtraction, which is modulo 2^32,
 * so it remains correct when HAL_GetTick() wraps after about 49.7 days.
 * Writing it as (HAL_GetTick() >= start_time + timeout_ms) would fail
 * at the wrap point.
 * =========================================================================
 */
HAL_StatusTypeDef Wait_Power_Good(uint16_t pg_pin, uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();

    while (HAL_GPIO_ReadPin(PG_Port, pg_pin) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - start_time) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }

        /* Avoid polling the GPIO more often than necessary */
        HAL_Delay(10);
    }

    return HAL_OK;
}

/* =========================================================================
 * Power-On Sequence
 * -------------------------------------------------------------------------
 * Verification order:
 *
 *      3.3V PG  ->  12V PG  ->  VCORE PG  ->  SYSTEM UP
 *
 * Why this order:
 *
 *   1. Control power first.
 *      3.3V supplies the management controller, the sequencing logic and
 *      the bias of the voltage-regulator controllers. The device that
 *      supervises the sequence has to be alive before anything else.
 *
 *   2. Power-tree dependency.
 *      12V is the bulk input rail feeding every downstream regulator.
 *      Enabling a child rail before its parent is stable makes the
 *      regulator hiccup on under-voltage lockout.
 *
 *   3. Core power last.
 *      VCORE is stepped down from 12V, and the I/O rails of the processor
 *      must already be present. Otherwise current can flow through the
 *      I/O ESD clamp diodes into an unpowered core rail, causing leakage
 *      or latch-up.
 *
 * Staggering the rails also keeps the combined inrush current within the
 * limits of the power supply.
 * =========================================================================
 */
void Power_On_Sequence(void)
{
    uint32_t t_start;

    current_state = STATE_POWER_ON;
    current_fault = FAULT_NONE;

    printf("\n");
    printf("======== POWER ON SEQUENCE ========\n");

    /* ---------------------------------------------------------------------
     * Step 1 : 3.3V
     * ---------------------------------------------------------------------
     */
    t_start = HAL_GetTick();
    printf("[SEQ ] Waiting for 3.3V  PG ... ");

    if (Wait_Power_Good(PG_3V3_Pin, PG_TIMEOUT_MS) != HAL_OK)
    {
        printf("TIMEOUT after %lu ms\n",
               (unsigned long)(HAL_GetTick() - t_start));
        Set_Fault(FAULT_3V3_TIMEOUT);
        return;
    }

    printf("OK (%lu ms)\n",
           (unsigned long)(HAL_GetTick() - t_start));

    HAL_Delay(RAIL_DELAY_MS);

    /* ---------------------------------------------------------------------
     * Step 2 : 12V
     * ---------------------------------------------------------------------
     */
    t_start = HAL_GetTick();
    printf("[SEQ ] Waiting for 12V   PG ... ");

    if (Wait_Power_Good(PG_12V_Pin, PG_TIMEOUT_MS) != HAL_OK)
    {
        printf("TIMEOUT after %lu ms\n",
               (unsigned long)(HAL_GetTick() - t_start));
        Set_Fault(FAULT_12V_TIMEOUT);
        return;
    }

    printf("OK (%lu ms)\n",
           (unsigned long)(HAL_GetTick() - t_start));

    HAL_Delay(RAIL_DELAY_MS);

    /* ---------------------------------------------------------------------
     * Step 3 : VCORE
     * ---------------------------------------------------------------------
     */
    t_start = HAL_GetTick();
    printf("[SEQ ] Waiting for VCORE PG ... ");

    if (Wait_Power_Good(PG_VCORE_Pin, PG_TIMEOUT_MS) != HAL_OK)
    {
        printf("TIMEOUT after %lu ms\n",
               (unsigned long)(HAL_GetTick() - t_start));
        Set_Fault(FAULT_VCORE_TIMEOUT);
        return;
    }

    printf("OK (%lu ms)\n",
           (unsigned long)(HAL_GetTick() - t_start));

    /* ---------------------------------------------------------------------
     * Boot completed
     * ---------------------------------------------------------------------
     */
    current_state = STATE_SYSTEM_UP;

    printf("======== SYSTEM UP ================\n");
    printf("Press PA0 again to return to STANDBY.\n\n");
}

/* =========================================================================
 * Fault Handling
 * -------------------------------------------------------------------------
 * Thermal fail-safe:
 *   On over-temperature the correct action is to increase cooling, not to
 *   stop it, so the fan is forced to 100%. Every other fault stops the fan.
 * =========================================================================
 */
void Set_Fault(FaultCode_t fault)
{
    current_fault = fault;

    /* Print the banner before taking any action so that the log reads in
     * the correct cause-and-effect order.
     */
    printf("\n");
    printf("====================================\n");
    printf("[FAULT] %s\n", Get_Fault_String(fault));
    printf("[STATE] %s\n", Get_State_String(current_state));
    printf("[TIME ] %lu ms\n", (unsigned long)HAL_GetTick());
    printf("====================================\n");

    /* Record the state in which the fault actually occurred,
     * which is why this happens before current_state is changed.
     */
    Add_Event_Log(fault, current_state);

    if (fault == FAULT_OVERTEMP)
    {
        fan_level = FAN_HIGH;
        Set_Fan_Speed(FAN_DUTY_HIGH);
        Set_LED(FAN_HIGH);
        printf("[SAFE ] Over-temperature -> fan forced to 100%%\n");
    }
    else
    {
        fan_level = FAN_OFF;
        Set_Fan_Speed(FAN_DUTY_OFF);

        /* Clear the LEDs; the main loop will blink the red one. */
        HAL_GPIO_WritePin(LED_Port,
                          LED_GREEN_Pin | LED_YELLOW_Pin | LED_RED_Pin,
                          GPIO_PIN_RESET);
    }

    current_state = STATE_FAULT;

    Dump_Event_Log();

    printf("[FAULT] System latched in FAULT state.\n");
    printf("[FAULT] Press PA0 to print the log, or reset to recover.\n\n");
}

/* =========================================================================
 * Event Log
 * -------------------------------------------------------------------------
 * A circular buffer of fixed size. Once full, the oldest entry is
 * overwritten, so the most recent EVENT_LOG_SIZE events are always kept.
 * This mirrors the idea of the system event log kept by a management
 * controller.
 * =========================================================================
 */
void Add_Event_Log(FaultCode_t fault, SystemState_t state)
{
    event_log[event_log_head].timestamp = HAL_GetTick();
    event_log[event_log_head].fault     = fault;
    event_log[event_log_head].state     = state;

    event_log_head = (event_log_head + 1) % EVENT_LOG_SIZE;

    if (event_log_count < EVENT_LOG_SIZE)
    {
        event_log_count++;
    }
}

void Dump_Event_Log(void)
{
    printf("\n-------- EVENT LOG (%u entries) --------\n",
           (unsigned)event_log_count);

    if (event_log_count == 0)
    {
        printf("(empty)\n");
    }
    else
    {
        uint8_t start;

        /* If the buffer is not full yet, the oldest entry is at index 0.
         * If it is full, the write head points at the oldest entry.
         */
        if (event_log_count == EVENT_LOG_SIZE)
        {
            start = event_log_head;
        }
        else
        {
            start = 0;
        }

        for (uint8_t n = 0; n < event_log_count; n++)
        {
            uint8_t index = (start + n) % EVENT_LOG_SIZE;

            printf("[%02u] t=%8lu ms  state=%-10s  fault=%s\n",
                   (unsigned)(n + 1),
                   (unsigned long)event_log[index].timestamp,
                   Get_State_String(event_log[index].state),
                   Get_Fault_String(event_log[index].fault));
        }
    }

    printf("---------------------------------------\n\n");
}

/* =========================================================================
 * Fault Code to String
 * =========================================================================
 */
const char *Get_Fault_String(FaultCode_t fault)
{
    switch (fault)
    {
        case FAULT_NONE:            return "NONE";
        case FAULT_TEMP_SENSOR:     return "TEMP_SENSOR_ERROR";
        case FAULT_OVERTEMP:        return "OVER_TEMPERATURE";
        case FAULT_3V3_TIMEOUT:     return "3V3_PG_TIMEOUT";
        case FAULT_12V_TIMEOUT:     return "12V_PG_TIMEOUT";
        case FAULT_VCORE_TIMEOUT:   return "VCORE_PG_TIMEOUT";
        case FAULT_INVALID_STATE:   return "INVALID_STATE";
        default:                    return "UNKNOWN_FAULT";
    }
}

/* =========================================================================
 * System State to String
 * =========================================================================
 */
const char *Get_State_String(SystemState_t state)
{
    switch (state)
    {
        case STATE_STANDBY:     return "STANDBY";
        case STATE_POWER_ON:    return "POWER_ON";
        case STATE_SYSTEM_UP:   return "SYSTEM_UP";
        case STATE_FAULT:       return "FAULT";
        default:                return "UNKNOWN_STATE";
    }
}


/* =========================================================================
 * Main
 * =========================================================================
 */
int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
    float    temperature  = 0.0f;
    uint32_t last_monitor = 0;

    /* --- Initialisation --- */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Make sure the fan starts stopped */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    current_fan_duty = 0xFF;
    Set_Fan_Speed(0);
    fan_level        = FAN_OFF;

    current_state = STATE_STANDBY;
    current_fault = FAULT_NONE;

    Set_LED(FAN_OFF);

    printf("\n\n");
    printf("=========================================\n");
    printf(" STM32 POWER SEQUENCING AND THERMAL DEMO\n");
    printf("=========================================\n");
    printf("PG order : 3.3V -> 12V -> VCORE\n");
    printf("State    : STANDBY\n");
    printf("Press PA0 to start.\n\n");

    /* =====================================================================
     * Main loop
     *
     * Note on STATE_POWER_ON:
     *   The power sequence is blocking, so while current_state is
     *   STATE_POWER_ON the program is still inside Power_On_Sequence().
     *   By the time control returns here the state is already SYSTEM_UP
     *   or FAULT, so the main loop never observes STATE_POWER_ON and does
     *   not need a branch for it. The state still exists because the event
     *   log records the state in which a fault occurred.
     * =====================================================================
     */
    while (1)
    {
        /* -----------------------------------------------------------------
         * STATE : STANDBY
         * -----------------------------------------------------------------
         */
        if (current_state == STATE_STANDBY)
        {
            if (HAL_GPIO_ReadPin(BTN_Port, BTN_Pin) == GPIO_PIN_SET)
            {
                /* Simple debounce, then wait for release so that holding
                 * the button is not treated as repeated presses.
                 */
                HAL_Delay(200);
                while (HAL_GPIO_ReadPin(BTN_Port, BTN_Pin) == GPIO_PIN_SET)
                {
                    HAL_Delay(10);
                }

                printf("[BTN ] Power button pressed.\n");

                /* Pre-boot health check: the sensor must respond and the
                 * temperature must be below the critical threshold.
                 */
                if (Read_Temperature(&temperature) != HAL_OK)
                {
                    printf("[CHK ] Temperature sensor error. Boot aborted.\n");
                    Set_Fault(FAULT_TEMP_SENSOR);
                }
                else if (temperature >= TEMP_CRITICAL)
                {
                    printf("[CHK ] %.2f C is too high. Boot aborted.\n",
                           temperature);
                    Set_Fault(FAULT_OVERTEMP);
                }
                else
                {
                    printf("[CHK ] %.2f C OK.\n", temperature);

                    Power_On_Sequence();

                    /* Only arm the monitor timer if the boot succeeded.
                     * Subtracting one period makes the first measurement
                     * happen immediately.
                     */
                    if (current_state == STATE_SYSTEM_UP)
                    {
                        last_monitor = HAL_GetTick() - MONITOR_PERIOD_MS;
                    }
                }
            }
        }

        /* -----------------------------------------------------------------
         * STATE : SYSTEM UP
         * -----------------------------------------------------------------
         */
        else if (current_state == STATE_SYSTEM_UP)
        {
            /* Button press returns the system to standby */
            if (HAL_GPIO_ReadPin(BTN_Port, BTN_Pin) == GPIO_PIN_SET)
            {
                HAL_Delay(200);
                while (HAL_GPIO_ReadPin(BTN_Port, BTN_Pin) == GPIO_PIN_SET)
                {
                    HAL_Delay(10);
                }

                printf("[BTN ] Power-off request.\n");

                Set_Fan_Speed(FAN_DUTY_OFF);
                fan_level = FAN_OFF;
                Set_LED(FAN_OFF);

                current_fault = FAULT_NONE;
                current_state = STATE_STANDBY;

                printf("[STATE] STANDBY\n");
                printf("Press PA0 to start.\n\n");
            }
            /* Periodic temperature monitoring */
            else if ((HAL_GetTick() - last_monitor) >= MONITOR_PERIOD_MS)
            {
                last_monitor = HAL_GetTick();

                if (Read_Temperature(&temperature) != HAL_OK)
                {
                    Set_Fault(FAULT_TEMP_SENSOR);
                }
                else
                {
                    printf("[MON ] t=%8lu ms  TEMP=%6.2f C  "
                           "FAN=%3u%%  LEVEL=%u\n",
                           (unsigned long)HAL_GetTick(),
                           temperature,
                           (unsigned)current_fan_duty,
                           (unsigned)fan_level);

                    Thermal_Control(temperature);
                }
            }
        }

        /* -----------------------------------------------------------------
         * STATE : FAULT
         * -----------------------------------------------------------------
         * The fault is latched. The red LED blinks and the button can be
         * used to print the event log again. A reset is required to
         * recover, which matches how a management controller latches a
         * critical fault rather than silently retrying.
         */
        else if (current_state == STATE_FAULT)
        {
            HAL_GPIO_TogglePin(LED_Port, LED_RED_Pin);
            HAL_Delay(200);

            if (HAL_GPIO_ReadPin(BTN_Port, BTN_Pin) == GPIO_PIN_SET)
            {
                HAL_Delay(200);
                while (HAL_GPIO_ReadPin(BTN_Port, BTN_Pin) == GPIO_PIN_SET)
                {
                    HAL_Delay(10);
                }

                Dump_Event_Log();
            }
        }

        /* -----------------------------------------------------------------
         * Unexpected state
         * -----------------------------------------------------------------
         */
        else
        {
            Set_Fault(FAULT_INVALID_STATE);
        }
    }
}

/* =========================================================================
 * System Clock Configuration
 * -------------------------------------------------------------------------
 *      HSE     = 8 MHz
 *      PLLM    = 8         ->  8 MHz / 8   = 1 MHz
 *      PLLN    = 336       ->  1 MHz * 336 = 336 MHz
 *      PLLP    = 2         ->  336 / 2     = 168 MHz  (SYSCLK)
 *
 *      HCLK    = 168 MHz
 *      APB1    = 42 MHz
 *      APB2    = 84 MHz    ->  TIM1 clock  = 168 MHz
 * =========================================================================
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK
                                     | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1
                                     | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================================
 * I2C1 Initialisation
 * =========================================================================
 */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================================
 * TIM1 PWM Initialisation
 * -------------------------------------------------------------------------
 *      TIM1 clock = 168 MHz
 *      PSC        = 167    ->  counter clock = 1 MHz
 *      ARR        = 999    ->  PWM frequency = 1 kHz
 *
 * If these values are changed, update the .ioc file as well, otherwise
 * regenerating code from CubeMX will overwrite them.
 * =========================================================================
 */
static void MX_TIM1_Init(void)
{
    TIM_MasterConfigTypeDef        sMasterConfig        = {0};
    TIM_OC_InitTypeDef             sConfigOC            = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 167;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configures TIM1_CH1 on PE9. The pin setup lives in
     * stm32f4xx_hal_msp.c and is generated by CubeMX.
     */
    HAL_TIM_MspPostInit(&htim1);
}

/* =========================================================================
 * GPIO Initialisation
 * =========================================================================
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* --- Status LEDs, all off initially --- */
    HAL_GPIO_WritePin(LED_Port,
                      LED_GREEN_Pin | LED_YELLOW_Pin | LED_RED_Pin,
                      GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = LED_GREEN_Pin | LED_YELLOW_Pin | LED_RED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_Port, &GPIO_InitStruct);

    /* --- Power button on PA0 ---
     * The Discovery board already has an external pull-down on this pin,
     * so no internal pull is required. Use GPIO_PULLDOWN if the button
     * is replaced by an external one without a pull-down.
     */
    GPIO_InitStruct.Pin  = BTN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BTN_Port, &GPIO_InitStruct);

    /* --- Power-Good inputs on PB0 / PB1 / PB2 ---
     * An internal pull-down gives a safe default: a disconnected input
     * reads low, which is interpreted as "not ready" rather than "good".
     * A lost signal should never look like a healthy rail.
     */
    GPIO_InitStruct.Pin  = PG_3V3_Pin | PG_12V_Pin | PG_VCORE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(PG_Port, &GPIO_InitStruct);
}

/* =========================================================================
 * Error Handler
 * =========================================================================
 */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
        /* Fatal initialisation error. Stay here until reset. */
    }
}

/* =========================================================================
 * Assert Handler
 * =========================================================================
 */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("[ASSERT] %s : %lu\n", (char *)file, (unsigned long)line);
}
#endif
