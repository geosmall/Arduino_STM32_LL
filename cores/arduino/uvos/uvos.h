#pragma once

#define WEAK __attribute__ ((weak))

#ifdef __cplusplus
extern "C" {
#endif

#include <uvos_helpers.h>
#include <uvos_math.h>
#include <uvos_constants.h>

#ifdef USE_SIM_POSIX
/* SimPosix version of this file. This will probably be removed later */
#include <uvos_sim_posix.h>
#else

/* C Lib includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

/* STM32 std lib */
#if defined(STM32F4xx)
#include <stm32f4xx.h>
#include "stm32f4xx_conf.h"
#elif defined(STM32F7xx)
#include <stm32f7xx.h>
#include "stm32f7xx_conf.h"
#else
#error "No Architecture defined"
#endif


/* UVOS Hardware Includes (Common) */
#include <uvos_heap.h>
#include <uvos_sensors.h>

/* UVOS board specific feature selection */
#include "uvos_config.h"

/* UVOS board specific device configuration */
#include "uvos_board.h"

/* UVOS debug interface */
/* #define UVOS_INCLUDE_DEBUG_CONSOLE */
/* #define DEBUG_LEVEL 0 */
#include <uvos_debug.h>
#include <uvos_debuglog.h>

/* UVOS FreeRTOS support */
#ifdef UVOS_INCLUDE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#endif

/* UVOS scheduler support */
#ifdef UVOS_INCLUDE_SCHED
#include "uvos_sched.h"
#endif

#ifdef UVOS_INCLUDE_INITCALL
#include "uvos_initcall.h"
#endif

/* UVOS system functions */
#ifdef UVOS_INCLUDE_DELAY
#include <uvos_delay.h>
// #include <uvos_deltatime.h>
#endif

#ifdef UVOS_INCLUDE_SYS
#include <uvos_sys.h>
#endif

/* UVOS hardware peripherals */
#ifdef UVOS_INCLUDE_IRQ
#include <uvos_irq.h>
#endif

#ifdef UVOS_INCLUDE_RTC
#include <uvos_rtc.h>
#endif

#ifdef UVOS_INCLUDE_TIM
#include <uvos_tim.h>
#endif

#ifdef UVOS_INCLUDE_USART
#include <uvos_usart.h>
#endif

#ifdef UVOS_INCLUDE_I2C
#include <uvos_i2c.h>
#endif

#ifdef UVOS_INCLUDE_SPI
#include <uvos_spi.h>
#endif

#ifdef UVOS_INCLUDE_GPIO
#include <uvos_gpio.h>
#endif

#ifdef UVOS_INCLUDE_EXTI
#include <uvos_exti.h>
#endif

#ifdef UVOS_INCLUDE_IBUS
#include <uvos_ibus.h>
#endif

/* UVOS abstract receiver interface */
#ifdef UVOS_INCLUDE_RCVR
#include <uvos_rcvr.h>
#endif

/* UVOS common peripherals */
#ifdef UVOS_INCLUDE_LED
#include <uvos_led.h>
#endif

#ifdef UVOS_INCLUDE_SERVO
#include <uvos_servo.h>
#endif

#ifdef UVOS_INCLUDE_FS
#include <uvos_fs.h>
#include <ff.h>
#include <lfs.h>
#endif

#ifdef UVOS_INCLUDE_SDCARD
#define LOG_FILENAME "startup.log"
// #include <ff.h>
#include <uvos_sdcard_fatfs.h>
#endif

#ifdef UVOS_INCLUDE_FLASH
/* #define UVOS_INCLUDE_FLASH_LOGFS_SETTINGS */
/* #define FLASH_FREERTOS */
#include <uvos_flash.h>
// #include <lfs.h>
#include <uvos_spif_lfs.h>
#endif

/* UVOS abstract comms interface with options */
#ifdef UVOS_INCLUDE_COM
/* #define UVOS_INCLUDE_COM_MSG */
/* #define UVOS_INCLUDE_TELEMETRY_RF */
/* #define UVOS_INCLUDE_COM_TELEM */
/* #define UVOS_INCLUDE_COM_FLEXI */
/* #define UVOS_INCLUDE_COM_AUX */
/* #define UVOS_TELEM_PRIORITY_QUEUE */
/* #define UVOS_INCLUDE_GPS */
/* #define UVOS_GPS_MINIMAL */
/* #define UVOS_INCLUDE_GPS_NMEA_PARSER */
/* #define UVOS_INCLUDE_GPS_UBX_PARSER */
/* #define UVOS_GPS_SETS_HOMELOCATION */
#include <uvos_com.h>
#endif

#if 0 // GLS

/* STM32 Std Peripherals Lib */
#if defined(STM32F10X)
#include <stm32f10x.h>
#elif defined(STM32F4XX)
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#elif defined(STM32F0)
#include <stm32f0xx.h>
#else
#error "No Architecture defined"
#endif

/* PIOS board specific feature selection */
#include "pios_config.h"

/* PIOS board specific device configuration */
#include "pios_board.h"

/* PIOS debug interface */
/* #define PIOS_INCLUDE_DEBUG_CONSOLE */
/* #define DEBUG_LEVEL 0 */
/* #define PIOS_ENABLE_DEBUG_PINS */
#include <pios_debug.h>
#include <pios_debuglog.h>

/* PIOS common functions */
#include <pios_crc.h>

/* PIOS FreeRTOS support */
#ifdef PIOS_INCLUDE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#endif
#include <stdbool.h>

#include <pios_mem.h>

#include <pios_architecture.h>

#ifdef PIOS_INCLUDE_TASK_MONITOR
#ifndef PIOS_INCLUDE_FREERTOS
#error PiOS Task Monitor requires PIOS_INCLUDE_FREERTOS to be defined
#endif
#include <pios_task_monitor.h>
#endif

/* PIOS CallbackScheduler */
#ifdef PIOS_INCLUDE_CALLBACKSCHEDULER
#ifndef PIOS_INCLUDE_FREERTOS
#error PiOS CallbackScheduler requires PIOS_INCLUDE_FREERTOS to be defined
#endif
#include <pios_callbackscheduler.h>
#endif

/* PIOS bootloader helper */
#ifdef PIOS_INCLUDE_BL_HELPER
/* #define PIOS_INCLUDE_BL_HELPER_WRITE_SUPPORT */
#include <pios_bl_helper.h>
#endif

/* PIOS system functions */
#ifdef PIOS_INCLUDE_DELAY
#include <pios_delay.h>
#include <pios_deltatime.h>
#endif

#ifdef PIOS_INCLUDE_INITCALL
#include "pios_initcall.h"
#endif

#ifdef PIOS_INCLUDE_SYS
#include <pios_sys.h>
#endif

/* PIOS hardware peripherals */
#ifdef PIOS_INCLUDE_IRQ
#include <pios_irq.h>
#endif

#ifdef PIOS_INCLUDE_RTC
#include <pios_rtc.h>
#endif

#ifdef PIOS_INCLUDE_TIM
#include <pios_tim.h>
#endif

#ifdef PIOS_INCLUDE_USART
#include <pios_usart.h>
#endif

#ifdef PIOS_INCLUDE_ADC
#include <pios_adc.h>
#endif

#ifdef PIOS_INCLUDE_I2C
#include <pios_i2c.h>
#endif

#ifdef PIOS_INCLUDE_SPI
#include <pios_spi.h>
#endif

#ifdef PIOS_INCLUDE_GPIO
#include <pios_gpio.h>
#endif

#ifdef PIOS_INCLUDE_EXTI
#include <pios_exti.h>
#endif

#ifdef PIOS_INCLUDE_WDG
#include <pios_wdg.h>
#endif

/* PIOS USB functions */
#ifdef PIOS_INCLUDE_USB
/* #define PIOS_INCLUDE_USB_HID */
/* #define PIOS_INCLUDE_USB_CDC */
/* #define PIOS_INCLUDE_USB_RCTX */
#include <pios_usb.h>
#ifdef PIOS_INCLUDE_USB_HID
#include <pios_usb_hid.h>
#endif
#ifdef PIOS_INCLUDE_USB_RCTX
#include <pios_usb_rctx.h>
#endif
#endif

#ifdef PIOS_INCLUDE_MPXV
/* MPXV5004, MPXV7002 based Airspeed Sensor */
#include <pios_mpxv.h>
#endif

#ifdef PIOS_INCLUDE_ETASV3
/* Eagle Tree Systems Airspeed MicroSensor V3 */
#include <pios_etasv3.h>
#endif

#ifdef PIOS_INCLUDE_MS4525DO
/* PixHawk Airspeed Sensor based on MS4525DO */
#include <pios_ms4525do.h>
#endif


#ifdef PIOS_INCLUDE_HCSR04
/* HC-SR04 Ultrasonic Sensor */
#include <pios_hcsr04.h>
#endif

/* PIOS receiver drivers */
#ifdef PIOS_INCLUDE_PWM
#include <pios_pwm.h>
#endif

#ifdef PIOS_INCLUDE_PPM
#include <pios_ppm.h>
#endif

#ifdef PIOS_INCLUDE_PPM_FLEXI
/* PPM on CC flexi port */
#endif

#ifdef PIOS_INCLUDE_DSM
#include <pios_dsm.h>
#endif

#ifdef PIOS_INCLUDE_SBUS
#include <pios_sbus.h>
#endif

#ifdef PIOS_INCLUDE_HOTT
#include <pios_hott.h>
#endif

#ifdef PIOS_INCLUDE_EXBUS
#include <pios_exbus.h>
#endif

#ifdef PIOS_INCLUDE_SRXL
#include <pios_srxl.h>
#endif

#ifdef PIOS_INCLUDE_IBUS
#include <pios_ibus.h>
#endif

/* PIOS abstract receiver interface */
#ifdef PIOS_INCLUDE_RCVR
#include <pios_rcvr.h>
#endif

/* PIOS common peripherals */
#ifdef PIOS_INCLUDE_LED
#include <pios_led.h>
#endif

#ifdef PIOS_INCLUDE_IAP
#include <pios_iap.h>
#endif

#ifdef PIOS_INCLUDE_SERVO
#include <pios_servo.h>
#endif

#ifdef PIOS_INCLUDE_I2C_ESC
#include <pios_i2c_esc.h>
#endif

#ifdef PIOS_INCLUDE_OVERO
/* #define PIOS_OVERO_SPI */
#include <pios_overo.h>
#endif

#ifdef PIOS_INCLUDE_SDCARD
/* #define LOG_FILENAME "startup.log" */
#include <dosfs.h>
#include <pios_sdcard.h>
#endif

#ifdef PIOS_INCLUDE_FLASH
/* #define PIOS_INCLUDE_FLASH_LOGFS_SETTINGS */
/* #define FLASH_FREERTOS */
#include <pios_flash.h>
#include <pios_flashfs.h>
#endif

/* driver for storage on internal flash */
/* #define PIOS_INCLUDE_FLASH_INTERNAL */

#ifdef PIOS_INCLUDE_FLASH_EEPROM
#include <pios_flash_eeprom.h>
#endif

/* PIOS radio modules */
#ifdef PIOS_INCLUDE_RFM22B
/* #define PIOS_INCLUDE_PPM_OUT */
/* #define PIOS_RFM22B_DEBUG_ON_TELEM */
#include <pios_rfm22b.h>
#ifdef PIOS_INCLUDE_RFM22B_COM
#include <pios_rfm22b_com.h>
#endif
#endif /* PIOS_INCLUDE_RFM22B */

/* PIOS misc peripherals */
#ifdef PIOS_INCLUDE_VIDEO
#include <pios_video.h>
#endif

#ifdef PIOS_INCLUDE_WAVE
#include <pios_wavplay.h>
#endif

#ifdef PIOS_INCLUDE_UDP
#include <pios_udp.h>
#endif

/* PIOS abstract comms interface with options */
#ifdef PIOS_INCLUDE_COM
/* #define PIOS_INCLUDE_COM_MSG */
/* #define PIOS_INCLUDE_TELEMETRY_RF */
/* #define PIOS_INCLUDE_COM_TELEM */
/* #define PIOS_INCLUDE_COM_FLEXI */
/* #define PIOS_INCLUDE_COM_AUX */
/* #define PIOS_TELEM_PRIORITY_QUEUE */
/* #define PIOS_INCLUDE_GPS */
/* #define PIOS_GPS_MINIMAL */
/* #define PIOS_INCLUDE_GPS_NMEA_PARSER */
/* #define PIOS_INCLUDE_GPS_UBX_PARSER */
/* #define PIOS_GPS_SETS_HOMELOCATION */
#include <pios_com.h>
#endif

/* Stabilization options */
/* #define PIOS_QUATERNION_STABILIZATION */

/* Performance counters */
/* #define IDLE_COUNTS_PER_SEC_AT_NO_LOAD 995998 */

#endif /* USE_SIM_POSIX */

#endif // GLS

#ifdef __cplusplus
} // closing brace for extern "C"
#endif

