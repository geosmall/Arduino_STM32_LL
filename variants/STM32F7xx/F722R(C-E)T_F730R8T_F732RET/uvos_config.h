#ifndef UVOS_CONFIG_H
#define UVOS_CONFIG_H

/* ST Nucleo F411RE series */
#define NUCLEO_F411RE

/*
 * Below is a complete list of UVOS configurable options.
 * Please do not remove or rearrange them. Only comment out
 * unused options in the list. See main uvos.h header for more
 * details.
 */

#define SKIP_MPU_EXISTS_CHECK

#define UVOS_INCLUDE_DEBUG_CONSOLE
/* #define DEBUG_LEVEL 0 */
// #define UVOS_ENABLE_DEBUG_PINS 

/* UVOS FreeRTOS support */
// #define UVOS_INCLUDE_FREERTOS

/* UVOS task scheduler support */
#define UVOS_INCLUDE_SCHED

/* UVOS system functions */
#define UVOS_INCLUDE_DELAY
// #define UVOS_INCLUDE_INITCALL
#define UVOS_INCLUDE_SYS
// #define UVOS_INCLUDE_TASK_MONITOR

/* UVOS hardware peripherals */
#define UVOS_INCLUDE_IRQ
#define UVOS_INCLUDE_RTC
#define UVOS_INCLUDE_TIM
#define UVOS_INCLUDE_USART
// #define UVOS_INCLUDE_ADC
// #define UVOS_INCLUDE_I2C
#define UVOS_INCLUDE_SPI
#define UVOS_INCLUDE_GPIO
#define UVOS_INCLUDE_EXTI
// #define UVOS_INCLUDE_WDG

/* UVOS sensor interfaces */
#define UVOS_INCLUDE_MPU
// #define UVOS_MPU_ACCEL

/* UVOS receiver drivers */
// #define UVOS_INCLUDE_DSM
// #define UVOS_INCLUDE_SBUS
#define UVOS_INCLUDE_IBUS

/* UVOS abstract receiver interface */
#define UVOS_INCLUDE_RCVR

/* UVOS common peripherals */
#define UVOS_INCLUDE_LED
#define UVOS_INCLUDE_SERVO
#define UVOS_INCLUDE_FLASH
// #define UVOS_INCLUDE_SDCARD
#define UVOS_FLASHFS_LOGFS_MAX_DEVS 5
#define UVOS_INCLUDE_FLASH_LOGFS_SETTINGS

/* UVOS abstract comms interface with options */
#define UVOS_INCLUDE_COM
#define UVOS_INCLUDE_DEBUG_CONSOLE
// #define UVOS_INCLUDE_COM_TELEM
#define UVOS_INCLUDE_COM_FLEXI

#if 0 // GLS

/* PIOS FreeRTOS support */
#define PIOS_INCLUDE_FREERTOS

/* PIOS Callback Scheduler support */
#define PIOS_INCLUDE_CALLBACKSCHEDULER

/* PIOS bootloader helper */
#define PIOS_INCLUDE_BL_HELPER
/* #define PIOS_INCLUDE_BL_HELPER_WRITE_SUPPORT */

/* PIOS system functions */
#define PIOS_INCLUDE_DELAY
#define PIOS_INCLUDE_INITCALL
#define PIOS_INCLUDE_SYS
#define PIOS_INCLUDE_TASK_MONITOR

#define PIOS_INCLUDE_INSTRUMENTATION
#define PIOS_INSTRUMENTATION_MAX_COUNTERS 10

/* PIOS hardware peripherals */
#define PIOS_INCLUDE_IRQ
#define PIOS_INCLUDE_RTC
#define PIOS_INCLUDE_TIM
#define PIOS_INCLUDE_USART
#define PIOS_INCLUDE_ADC
#define PIOS_INCLUDE_I2C
#define PIOS_INCLUDE_SPI
#define PIOS_INCLUDE_GPIO
#define PIOS_INCLUDE_EXTI
#define PIOS_INCLUDE_WDG

/* PIOS USB functions */
#define PIOS_INCLUDE_USB
#define PIOS_INCLUDE_USB_HID
#define PIOS_INCLUDE_USB_CDC
/* #define PIOS_INCLUDE_USB_RCTX */

/* PIOS sensor interfaces */
/* #define PIOS_INCLUDE_ADXL345 */
/* #define PIOS_INCLUDE_BMA180 */
/* #define PIOS_INCLUDE_L3GD20 */
#define PIOS_INCLUDE_MPU6000
#define PIOS_MPU6000_ACCEL
/* #define PIOS_INCLUDE_HMC5843 */
#define PIOS_INCLUDE_HMC5X83
#define PIOS_HMC5X83_HAS_GPIOS
/* #define PIOS_INCLUDE_BMP085 */
#define PIOS_INCLUDE_MS5611
#define PIOS_INCLUDE_MPXV
#define PIOS_INCLUDE_ETASV3
#define PIOS_INCLUDE_MS4525DO
/* #define PIOS_INCLUDE_HCSR04 */

#define PIOS_SENSOR_RATE 500.0f

#define PIOS_INCLUDE_WS2811

/* PIOS receiver drivers */
#define PIOS_INCLUDE_PWM
#define PIOS_INCLUDE_PPM
/* #define PIOS_INCLUDE_PPM_FLEXI */
#define PIOS_INCLUDE_DSM
#define PIOS_INCLUDE_SBUS
#define PIOS_INCLUDE_SRXL
#define PIOS_INCLUDE_HOTT
#define PIOS_INCLUDE_EXBUS
#define PIOS_INCLUDE_IBUS
#define PIOS_INCLUDE_GCSRCVR
#define PIOS_INCLUDE_OPLINKRCVR
#define PIOS_INCLUDE_OPENLRS_RCVR

/* PIOS abstract receiver interface */
#define PIOS_INCLUDE_RCVR

/* PIOS common peripherals */
#define PIOS_INCLUDE_LED
#define PIOS_INCLUDE_IAP
#define PIOS_INCLUDE_SERVO
/* #define PIOS_INCLUDE_I2C_ESC */
/* #define PIOS_INCLUDE_OVERO */
/* #define PIOS_OVERO_SPI */
/* #define PIOS_INCLUDE_SDCARD */
/* #define LOG_FILENAME "startup.log" */
#define PIOS_INCLUDE_FLASH
#define PIOS_INCLUDE_FLASH_INTERNAL
#define PIOS_INCLUDE_FLASH_LOGFS_SETTINGS
#define FLASH_FREERTOS
/* #define PIOS_INCLUDE_FLASH_EEPROM */

#define PIOS_INCLUDE_DEBUGLOG

/* PIOS radio modules */
// #define PIOS_INCLUDE_RFM22B
// #define PIOS_INCLUDE_RFM22B_COM
// #define PIOS_INCLUDE_OPENLRS
/* #define PIOS_INCLUDE_PPM_OUT */
/* #define PIOS_RFM22B_DEBUG_ON_TELEM */

/* PIOS misc peripherals */
/* #define PIOS_INCLUDE_VIDEO */
/* #define PIOS_INCLUDE_WAVE */
/* #define PIOS_INCLUDE_UDP */

/* PIOS abstract comms interface with options */
#define PIOS_INCLUDE_COM
/* #define PIOS_INCLUDE_COM_MSG */
/* #define PIOS_INCLUDE_TELEMETRY_RF */
#define PIOS_INCLUDE_COM_TELEM
#define PIOS_INCLUDE_COM_FLEXI
/* #define PIOS_INCLUDE_COM_AUX */
#define PIOS_TELEM_PRIORITY_QUEUE
#define PIOS_INCLUDE_GPS
/* #define PIOS_GPS_MINIMAL */
#define PIOS_INCLUDE_GPS_NMEA_PARSER
#define PIOS_INCLUDE_GPS_UBX_PARSER
#define PIOS_INCLUDE_GPS_DJI_PARSER
#define PIOS_GPS_SETS_HOMELOCATION

/* Stabilization options */
#define PIOS_QUATERNION_STABILIZATION

/* Performance counters */
#define IDLE_COUNTS_PER_SEC_AT_NO_LOAD 8379692

/* Alarm Thresholds */
#define HEAP_LIMIT_WARNING             1000
#define HEAP_LIMIT_CRITICAL            500
#define IRQSTACK_LIMIT_WARNING         150
#define IRQSTACK_LIMIT_CRITICAL        80
#define CPULOAD_LIMIT_WARNING          80
#define CPULOAD_LIMIT_CRITICAL         95

/* Task stack sizes */
/* #define PIOS_ACTUATOR_STACK_SIZE	1020 */
/* #define PIOS_MANUAL_STACK_SIZE		800 */
#define PIOS_SYSTEM_STACK_SIZE         1536
/* #define PIOS_STABILIZATION_STACK_SIZE	524 */
/* #define PIOS_TELEM_STACK_SIZE		500 */
/* #define PIOS_EVENTDISPATCHER_STACK_SIZE	130 */

/* This can't be too high to stop eventdispatcher thread overflowing */
#define PIOS_EVENTDISAPTCHER_QUEUE 10

#endif // GLS

#endif // UVOS_CONFIG_H
