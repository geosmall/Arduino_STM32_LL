#if defined(ARDUINO_GENERIC_H7A3VGHX) || defined(ARDUINO_GENERIC_H7A3VGTX) ||\
    defined(ARDUINO_GENERIC_H7A3VIHX) || defined(ARDUINO_GENERIC_H7A3VITX) ||\
    defined(ARDUINO_GENERIC_H7B0VBTX) || defined(ARDUINO_GENERIC_H7B3VIHX) ||\
    defined(ARDUINO_GENERIC_H7B3VITX)


#include "stm32yyxx_ll.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus


__attribute__ ((weak)) void SystemClock_Config( void )
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(32);
  LL_RCC_HSI_SetDivider(LL_RCC_HSI_DIV1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_1);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_1);

  LL_Init1msTick(64000000);

  LL_SetSystemCoreClock(64000000);
}

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* ARDUINO_GENERIC_* */
