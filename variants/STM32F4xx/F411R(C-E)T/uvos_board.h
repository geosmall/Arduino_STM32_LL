#ifndef UVOS_BOARD_H
#define UVOS_BOARD_H

#include <stdbool.h>

uint32_t UVOS_Board_Init( void );
void SystemClock_Config( void );

// ------------------------
// Timers and Channels Used
// ------------------------
/*
   Timer | Channel 1 | Channel 2 | Channel 3 | Channel 4
   ------+-----------+-----------+-----------+----------
   TIM1  |           |           |           |
   TIM2  |           |           |           |
   TIM3  |           |           |           |
   TIM4  |           |           |           |
   TIM5  |           |           |           |
   TIM6  |           |           |           |
   TIM7  |           |           |           |
   TIM8  |           |           |           |
   ------+-----------+-----------+-----------+----------
 */

// ------------------------
// DMA Channels Used
// ------------------------
/* Channel 1  -                                 */
/* Channel 2  - SPI1 RX                         */
/* Channel 3  - SPI1 TX                         */
/* Channel 4  - SPI2 RX                         */
/* Channel 5  - SPI2 TX                         */
/* Channel 6  -                                 */
/* Channel 7  -                                 */
/* Channel 8  -                                 */
/* Channel 9  -                                 */
/* Channel 10 -                                 */
/* Channel 11 -                                 */
/* Channel 12 -                                 */

// ------------------------
// BOOTLOADER_SETTINGS
// ------------------------
#define BOARD_READABLE     true
#define BOARD_WRITABLE     true
#define MAX_DEL_RETRYS     3

// ------------------------
// UVOS_LED
// ------------------------
#define UVOS_LED_HEARTBEAT 0
#define UVOS_LED_ALARM     1
#ifdef UVOS_RFM22B_DEBUG_ON_TELEM
#define UVOS_LED_D1        2
#define UVOS_LED_D2        3
#define UVOS_LED_D3        4
#define UVOS_LED_D4        5

#define D1_LED_ON          UVOS_LED_On(UVOS_LED_D1)
#define D1_LED_OFF         UVOS_LED_Off(UVOS_LED_D1)
#define D1_LED_TOGGLE      UVOS_LED_Toggle(UVOS_LED_D1)

#define D2_LED_ON          UVOS_LED_On(UVOS_LED_D2)
#define D2_LED_OFF         UVOS_LED_Off(UVOS_LED_D2)
#define D2_LED_TOGGLE      UVOS_LED_Toggle(UVOS_LED_D2)

#define D3_LED_ON          UVOS_LED_On(UVOS_LED_D3)
#define D3_LED_OFF         UVOS_LED_Off(UVOS_LED_D3)
#define D3_LED_TOGGLE      UVOS_LED_Toggle(UVOS_LED_D3)

#define D4_LED_ON          UVOS_LED_On(UVOS_LED_D4)
#define D4_LED_OFF         UVOS_LED_Off(UVOS_LED_D4)
#define D4_LED_TOGGLE      UVOS_LED_Toggle(UVOS_LED_D4)
#endif /* UVOS_RFM22B_DEBUG_ON_TELEM */

// ------------------------
// UVOS_SPI
// See also uvos_board.c
// ------------------------
#define UVOS_SPI_MAX_DEVS      3

// ------------------------
// UVOS_WDG
// ------------------------
#define UVOS_WATCHDOG_TIMEOUT  500
#define UVOS_WDG_REGISTER      RTC_BKP_DR4
#define UVOS_WDG_ACTUATOR      0x0001
#define UVOS_WDG_STABILIZATION 0x0002
#define UVOS_WDG_ATTITUDE      0x0004
#define UVOS_WDG_MANUAL        0x0008
#define UVOS_WDG_SENSORS       0x0010

// ------------------------
// UVOS_I2C
// See also uvos_board.c
// ------------------------
#define UVOS_I2C_MAX_DEVS         3
extern uint32_t uvos_i2c_mag_pressure_adapter_id;
#define UVOS_I2C_MAIN_ADAPTER     (uvos_i2c_mag_pressure_adapter_id)
extern uint32_t uvos_i2c_flexiport_adapter_id;
#define UVOS_I2C_FLEXI_ADAPTER    (uvos_i2c_flexiport_adapter_id)
#define UVOS_I2C_ETASV3_ADAPTER   (UVOS_I2C_FLEXI_ADAPTER)
#define UVOS_I2C_MS4525DO_ADAPTER (UVOS_I2C_FLEXI_ADAPTER)

// -------------------------
// UVOS_USART
//
// See also uvos_board.c
// -------------------------
#define UVOS_USART_MAX_DEVS 5

// -------------------------
// UVOS_COM
//
// See also uvos_board.c
// -------------------------
#define UVOS_COM_MAX_DEVS 4
extern uint32_t uvos_com_telem_rf_id;
extern uint32_t uvos_com_rf_id;
extern uint32_t uvos_com_gps_id;
extern uint32_t uvos_com_telem_usb_id;
extern uint32_t uvos_com_bridge_id;
extern uint32_t uvos_com_vcp_id;
extern uint32_t uvos_com_hkosd_id;
extern uint32_t uvos_com_msp_id;
extern uint32_t uvos_com_mavlink_id;

#define UVOS_COM_GPS       (uvos_com_gps_id)
#define UVOS_COM_TELEM_USB (uvos_com_telem_usb_id)
#define UVOS_COM_TELEM_RF  (uvos_com_telem_rf_id)
#define UVOS_COM_RF        (uvos_com_rf_id)
#define UVOS_COM_BRIDGE    (uvos_com_bridge_id)
#define UVOS_COM_VCP       (uvos_com_vcp_id)
#define UVOS_COM_OSDHK     (uvos_com_hkosd_id)
#define UVOS_COM_MSP       (uvos_com_msp_id)
#define UVOS_COM_MAVLINK   (uvos_com_mavlink_id)

#if defined(UVOS_INCLUDE_DEBUG_CONSOLE)
extern uint32_t uvos_com_debug_id;
#define UVOS_COM_DEBUG     (uvos_com_debug_id)
#endif /* UVOS_INCLUDE_DEBUG_CONSOLE */

#if defined(UVOS_INCLUDE_FLASH)
extern uintptr_t uvos_spi_flash_id;
#define UVOS_FLASH_SPI_PORT (uvos_spi_flash_id)
#endif /* UVOS_INCLUDE_FLASH */

// -------------------------
// Packet Handler
// -------------------------
#define RS_ECC_NPARITY          4
#define UVOS_PH_MAX_PACKET      255
#define UVOS_PH_WIN_SIZE        3
#define UVOS_PH_MAX_CONNECTIONS 1
extern uint32_t uvos_packet_handler;
#define UVOS_PACKET_HANDLER     (uvos_packet_handler)

// ------------------------
// TELEMETRY
// ------------------------
#define TELEM_QUEUE_SIZE        80
#define UVOS_TELEM_STACK_SIZE   800

// -------------------------
// System Settings
//
// See also System_stm32f4xx.c
// -------------------------
// These macros are deprecated
// please use UVOS_PERIPHERAL_APBx_CLOCK According to the table below
// #define UVOS_MASTER_CLOCK
// #define UVOS_PERIPHERAL_CLOCK
// #define UVOS_PERIPHERAL_CLOCK

#define UVOS_SYSCLK 96000000
// Peripherals that belongs to APB1 are:
// DAC      |PWR        |CAN1,2
// I2C1,2,3   |UART4,5      |USART3,2
// I2S3Ext    |SPI3/I2S3    |SPI2/I2S2
// I2S2Ext    |IWDG       |WWDG
// RTC/BKP reg
// TIM2,3,4,5,6,7,12,13,14

// Calculated as SYSCLK / APBPresc * (APBPre == 1 ? 1 : 2)
// Default APB1 Prescaler = 2
#define UVOS_PERIPHERAL_APB1_CLOCK UVOS_SYSCLK

// Peripherals belonging to APB2
// SDIO     |EXTI       |SYSCFG     |SPI1
// ADC1,2,3
// USART1,6
// TIM1,8,9,10,11
//
// Default APB2 Prescaler = 1
//
#define UVOS_PERIPHERAL_APB2_CLOCK   UVOS_SYSCLK

// -------------------------
// Interrupt Priorities
// -------------------------
#define UVOS_IRQ_PRIO_LOW            12              // lower than RTOS
#define UVOS_IRQ_PRIO_MID            8               // higher than RTOS
#define UVOS_IRQ_PRIO_HIGH           5               // for SPI, ADC, I2C etc...
#define UVOS_IRQ_PRIO_HIGHEST        4               // for USART etc...

// -------------------------
// Scheduler defines
// -------------------------
#define UVOS_SCHED_UPDATE_HZ         1000

// The maximum number of tasks required at any one time
#define SCH_MAX_TASKS ( 5 )

// Usually set to 1, unless 'Long Tasks' are employed
#define SCH_TICK_COUNT_LIMIT ( 20 )

// ------------------------
// UVOS_RCVR
// See also uvos_board.c
// ------------------------
#define UVOS_RCVR_MAX_DEVS           3
#define UVOS_RCVR_MAX_CHANNELS       12
#define UVOS_GCSRCVR_TIMEOUT_MS      100
#define UVOS_RFM22B_RCVR_TIMEOUT_MS  200
#define UVOS_OPLINK_RCVR_TIMEOUT_MS  100

// -------------------------
// Receiver PPM input
// -------------------------
#define UVOS_PPM_MAX_DEVS            1
#define UVOS_PPM_NUM_INPUTS          16

// -------------------------
// Receiver PWM input
// -------------------------
#define UVOS_PWM_MAX_DEVS            1
#define UVOS_PWM_NUM_INPUTS          8

// -------------------------
// Receiver SPEKTRUM input
// -------------------------
#define UVOS_SPEKTRUM_MAX_DEVS       2
#define UVOS_SPEKTRUM_NUM_INPUTS     12

// -------------------------
// Receiver S.Bus input
// -------------------------
#define UVOS_SBUS_MAX_DEVS           1
#define UVOS_SBUS_NUM_INPUTS         (16 + 2)

// -------------------------
// Receiver HOTT input
// -------------------------
#define UVOS_HOTT_MAX_DEVS           1
#define UVOS_HOTT_NUM_INPUTS         32

// -------------------------
// Receiver EX.Bus input
// -------------------------
#define UVOS_EXBUS_MAX_DEVS          1
#define UVOS_EXBUS_NUM_INPUTS        16

// -------------------------
// Receiver Multiplex SRXL input
// -------------------------
#define UVOS_SRXL_MAX_DEVS           1
#define UVOS_SRXL_NUM_INPUTS         16

// -------------------------
// Receiver DSM input
// -------------------------
#define UVOS_DSM_MAX_DEVS            2
#define UVOS_DSM_NUM_INPUTS          12

// -------------------------
// Receiver FlySky IBus input
// -------------------------
#define UVOS_IBUS_MAX_DEVS           1
#define UVOS_IBUS_NUM_INPUTS         10

// -------------------------
// Servo outputs
// -------------------------
#define UVOS_SERVO_UPDATE_HZ         50
#define UVOS_SERVOS_INITIAL_POSITION 0 /* dont want to start motors, have no pulse till settings loaded */
#define UVOS_SERVO_BANKS             6

// --------------------------
// Timer controller settings
// --------------------------
#define UVOS_TIM_MAX_DEVS            6

// -------------------------
// ADC
// UVOS_ADC_PinGet(0) = Current sensor
// UVOS_ADC_PinGet(1) = Voltage sensor
// UVOS_ADC_PinGet(4) = VREF
// UVOS_ADC_PinGet(5) = Temperature sensor
// -------------------------
#define UVOS_DMA_PIN_CONFIG                                               \
  {                                                                       \
    { GPIOB, GPIO_Pin_1, ADC_Channel_9, false }, /* FLEXI-IO PIN 4     */ \
    { GPIOB, GPIO_Pin_0, ADC_Channel_8, false }, /* FLEXI-IO PIN 5     */ \
    { GPIOA, GPIO_Pin_7, ADC_Channel_7, false }, /* FLEXI-IO PIN 6     */ \
    { GPIOA, GPIO_Pin_6, ADC_Channel_6, false }, /* FLEXI-IO PIN 7     */ \
    { GPIOA, GPIO_Pin_5, ADC_Channel_5, false }, /* FLEXI-IO PIN 8     */ \
                                                                          \
    { GPIOA, GPIO_Pin_0, ADC_Channel_0, false }, /* SERVO PIN 5        */ \
    { GPIOA, GPIO_Pin_1, ADC_Channel_1, false }, /* SERVO PIN 6        */ \
                                                                          \
    { NULL, 0, ADC_Channel_Vrefint, false }, /* Voltage reference  */     \
    { NULL, 0, ADC_Channel_TempSensor, false }, /* Temperature sensor */  \
  }

/* we have to do all this to satisfy the UVOS_ADC_MAX_SAMPLES define in uvos_adc.h */
/* which is annoying because this then determines the rate at which we generate buffer turnover events */
/* the objective here is to get enough buffer space to support 100Hz averaging rate */
#define UVOS_ADC_NUM_CHANNELS     9
#define UVOS_ADC_MAX_OVERSAMPLING 2
#define UVOS_ADC_USE_ADC2         0

#define UVOS_ADC_USE_TEMP_SENSOR
#define UVOS_ADC_TEMPERATURE_PIN  8

// -------------------------
// USB
// -------------------------
#define UVOS_USB_MAX_DEVS         1
#define UVOS_USB_ENABLED          1 /* Should remove all references to this */
#define UVOS_USB_HID_MAX_DEVS     1

#endif /* UVOS_BOARD_H */
