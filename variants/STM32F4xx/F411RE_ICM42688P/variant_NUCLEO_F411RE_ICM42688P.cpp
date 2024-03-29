#if defined(ARDUINO_NUCLEO_F411RE_ICM42688P)

#include "stm32yyxx_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config( void )
{
  LL_FLASH_SetLatency( LL_FLASH_LATENCY_3 );
  while ( LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3 ) {
  }
  LL_PWR_SetRegulVoltageScaling( LL_PWR_REGU_VOLTAGE_SCALE1 );
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

  /* Wait till HSE is ready */
  while ( LL_RCC_HSE_IsReady() != 1 ) {

  }
  LL_PWR_EnableBkUpAccess();
  LL_RCC_PLL_ConfigDomain_SYS( LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 96, LL_RCC_PLLP_DIV_2 );
  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  while ( LL_RCC_PLL_IsReady() != 1 ) {

  }
  LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );
  LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_2 );
  LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_1 );
  LL_RCC_SetSysClkSource( LL_RCC_SYS_CLKSOURCE_PLL );

  /* Wait till System clock is ready */
  while ( LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL ) {

  }
  LL_Init1msTick( 96000000 );
  LL_SetSystemCoreClock( 96000000 );
  LL_RCC_SetTIMPrescaler( LL_RCC_TIM_PRESCALER_TWICE );
}

#ifdef __cplusplus
}
#endif

#endif /* ARDUINO_NUCLEO_F411RE_ICM42688P */
