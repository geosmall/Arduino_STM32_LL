#pragma once

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
#include "stm32f7xx_ll_adc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_cortex.h"
// #include "stm32f7xx_ll_crc.h"
// #include "stm32f7xx_ll_dac.h"
#include "stm32f7xx_ll_dma.h"
// #include "stm32f7xx_ll_dma2d.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_fmc.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_i2c.h"
#include "stm32f7xx_ll_iwdg.h"
// #include "stm32f7xx_ll_lptim.h"
#include "stm32f7xx_ll_pwr.h"
#include "stm32f7xx_ll_rcc.h"
// #include "stm32f7xx_ll_rng.h"
#include "stm32f7xx_ll_rtc.h"
#include "stm32f7xx_ll_sdmmc.h"
#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_tim.h"
#include "stm32f7xx_ll_usart.h"
#include "stm32f7xx_ll_usb.h"
#include "stm32f7xx_ll_utils.h"
// #include "stm32f7xx_ll_wwdg.h"

#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define DEVICE_SIGNATURE_ADDR    0x1FF07A10
#define DEVICE_FLASHSIZE_ADDR    0x1FF07A22

/* If an external clock source is used, then the value of the following define
   should be set to the value of the external clock source, else, if no external
   clock is used, keep this define commented */
/*#define I2S_EXTERNAL_CLOCK_VAL   12288000 */ /* Value of the external clock in Hz */


/* Uncomment the line below to expanse the "assert_param" macro in the
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
 * @brief  The assert_param macro is used for function's parameters check.
 * @param  expr: If expr is false, it calls assert_failed function
 *   which reports the name of the source file and the source
 *   line number of the call that failed.
 *   If expr is true, it returns no value.
 * @retval None
 */
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void assert_failed( uint8_t *file, uint32_t line );
#else
#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */
