#include "clock.h"
#include "stm32yyxx_ll_cortex.h"

__IO uint32_t clkTick;

void clockInit( void )
{
	clkTick = 0;
}

/**
  * @brief  Function called wto read the current millisecond
  * @param  None
  * @retval None
  */
uint32_t getCurrentMillis( void )
{
	return clkTick;
}

/**
  * @brief  Function called to read the current micro second
  * @param  None
  * @retval None
  */
uint32_t getCurrentMicros( void )
{
	/* Ensure COUNTFLAG is reset by reading SysTick control and status register */
	LL_SYSTICK_IsActiveCounterFlag();
	uint32_t m = clkTick;
	const uint32_t tms = SysTick->LOAD + 1;
	__IO uint32_t u = tms - SysTick->VAL;
	if ( LL_SYSTICK_IsActiveCounterFlag() ) {
		m = clkTick;
		u = tms - SysTick->VAL;
	}
	return ( m * 1000 + ( u * 1000 ) / tms );
}

void noOsSystickHandler()
{

}

void osSystickHandler() __attribute__((weak, alias("noOsSystickHandler")));

#if !defined(USE_USER_SYSTICK)

/**
  * @brief  Function called when the tick interruption falls
  * @param  None
  * @retval None
  */
void SysTick_Handler( void )  __attribute__ ( ( interrupt ) );
void SysTick_Handler( void )
{
	clkTick++;
	osSystickHandler();
}

#endif /* !USE_USER_SYSTICK */
