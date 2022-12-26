#ifndef UW_TIME_H
#define UW_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Main Functions */
extern uint32_t micros( void );
extern void UW_time_delay_us( uint32_t uSec );
extern void UW_time_delay_ms( uint32_t mSec );

#ifdef __cplusplus
}
#endif

#endif // UW_TIME_H
