#ifndef UVOS_DEBUG_H
#define UVOS_DEBUG_H

#ifdef UVOS_INCLUDE_DEBUG_CONSOLE
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 0
#endif
#define DEBUG_PRINTF(level, ...) \
  { \
    if ((level <= DEBUG_LEVEL) && (UVOS_COM_DEBUG > 0)) { \
      UVOS_COM_SendFormattedStringNonBlocking(UVOS_COM_DEBUG, __VA_ARGS__); \
    } \
  }
#else
#define DEBUG_PRINTF(level, ...)
#endif /* UVOS_INCLUDE_DEBUG_CONSOLE */

extern const char * UVOS_DEBUG_AssertMsg;

#ifdef USE_SIM_POSIX
void UVOS_DEBUG_Init( void );
#else
#include <uvos_tim_priv.h>
void UVOS_DEBUG_Init( const struct uvos_tim_channel * channels, uint8_t num_channels );
#endif

void UVOS_DEBUG_PinHigh( uint8_t pin );
void UVOS_DEBUG_PinLow( uint8_t pin );
void UVOS_DEBUG_PinValue8Bit( uint8_t value );
void UVOS_DEBUG_PinValue4BitL( uint8_t value );
void UVOS_DEBUG_Panic( const char * msg ) __attribute__( ( noreturn ) );

#ifdef DEBUG
#define UVOS_DEBUG_Assert(test) if (!(test)) { UVOS_DEBUG_Panic(UVOS_DEBUG_AssertMsg); }
#define UVOS_Assert(test)       UVOS_DEBUG_Assert(test)
#else
#define UVOS_DEBUG_Assert(test)
#define UVOS_Assert(test) \
  if (!(test)) { while (1) {; } \
  }
#endif

/* Static (compile-time) assertion for use in a function.
   If test evaluates to 0 (ie fails) at compile time then compilation will
   fail with the error: "size of unnamed array is negative" */
#define UVOS_STATIC_ASSERT(test) ((void)sizeof(int[1 - 2 * !(test)]))


#endif /* UVOS_DEBUG_H */

