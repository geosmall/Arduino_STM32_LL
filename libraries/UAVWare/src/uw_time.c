#include <uvos.h>

/* Main Functions */
uint32_t micros( void )
{
  return UVOS_DELAY_GetuS();
}

void UW_time_delay_us( uint32_t uSec )
{
  UVOS_DELAY_WaituS( uSec );
}

void UW_time_delay_ms( uint32_t mSec )
{
  UVOS_DELAY_WaitmS( mSec );
}