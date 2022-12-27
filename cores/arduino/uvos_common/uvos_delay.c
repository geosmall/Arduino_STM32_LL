/* Project Includes */
#include <uvos.h>

#if defined(UVOS_INCLUDE_DELAY)

/* cycles per microsecond */
static uint32_t us_ticks;
static uint32_t us_modulo;

/**
 * Initialises the Timer used by UVOS_DELAY functions.
 *
 * \return always zero (success)
 */

int32_t UVOS_DELAY_Init( void )
{
	// RCC_ClocksTypeDef	clocks;
	LL_RCC_ClocksTypeDef clocks;

	/* compute the number of system clocks per microsecond */
	// RCC_GetClocksFreq(&clocks);
	LL_RCC_GetSystemClocksFreq( &clocks );
	us_ticks = clocks.SYSCLK_Frequency / 1000000;
	UVOS_DEBUG_Assert( us_ticks > 1 );

	// Split this into two steps to avoid 64bit maths
	us_modulo = 0xffffffff / us_ticks;
	us_modulo += ( ( 0xffffffff % us_ticks ) + 1 ) / us_ticks;

	// ensure that the us_module is smaller than half of uint32_t max to make modulo operation possible
	UVOS_Assert( us_modulo < 0x80000000 );

	/* turn on access to the DWT registers */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	/* enable the CPU cycle counter */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	return 0;
}

/**
 * Waits for a specific number of uS
 *
 * Example:<BR>
 * \code
 *   // Wait for 500 uS
 *   UVOS_DELAY_Wait_uS(500);
 * \endcode
 * \param[in] uS delay
 * \return < 0 on errors
 */
int32_t UVOS_DELAY_WaituS( uint32_t uS )
{
	uint32_t elapsed = 0;
	uint32_t last_count;

	/* turn on access to the DWT registers */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	last_count = DWT->CYCCNT;

	for ( ;; ) {
		uint32_t current_count;
		uint32_t elapsed_uS;

		/* turn on access to the DWT registers */
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		current_count = DWT->CYCCNT;

		/* measure the time elapsed since the last time we checked */
		elapsed += current_count - last_count;
		last_count = current_count;

		/* convert to microseconds */
		elapsed_uS = elapsed / us_ticks;
		if ( elapsed_uS >= uS )
			break;

		/* reduce the delay by the elapsed time */
		uS -= elapsed_uS;

		/* keep fractional microseconds for the next iteration */
		elapsed %= us_ticks;
	}

	/* No error */
	return 0;
}

/**
 * Waits for a specific number of mS
 *
 * Example:<BR>
 * \code
 *   // Wait for 500 mS
 *   UVOS_DELAY_Wait_mS(500);
 * \endcode
 * \param[in] mS delay (1..65535 milliseconds)
 * \return < 0 on errors
 */
int32_t UVOS_DELAY_WaitmS( uint32_t mS )
{
	while ( mS-- ) {
		UVOS_DELAY_WaituS( 1000 );
	}

	/* No error */
	return 0;
}

/**
 * @brief Query the Delay timer for the current uS
 * @return A microsecond value
 */
uint32_t UVOS_DELAY_GetuS()
{
	/* turn on access to the DWT registers */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	return DWT->CYCCNT / us_ticks;
}

/**
 * @brief Calculate time in microseconds since a previous time
 * @param[in] t previous time
 * @return time in us since previous time t.
 */
uint32_t UVOS_DELAY_GetuSSince( uint32_t t )
{
	return ( UVOS_DELAY_GetuS() + us_modulo - t ) % us_modulo;
}

/**
 * @brief Get the raw delay timer, useful for timing
 * @return Unitless value (uint32 wrap around)
 */
uint32_t UVOS_DELAY_GetRaw()
{
	/* turn on access to the DWT registers */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	return DWT->CYCCNT;
}

/**
 * @brief Compare to raw times to and convert to us
 * @return A microsecond value
 */
uint32_t UVOS_DELAY_DiffuS( uint32_t raw )
{
	/* turn on access to the DWT registers */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	uint32_t diff = DWT->CYCCNT - raw;
	return diff / us_ticks;
}

#endif

/**
  * @}
  * @}
  */
