#include "uvos.h"

// Global variables
const char * UVOS_DEBUG_AssertMsg = "ASSERT FAILED";

#ifdef UVOS_ENABLE_DEBUG_PINS
static const struct uvos_tim_channel * debug_channels;
static uint8_t debug_num_channels;
#endif /* UVOS_ENABLE_DEBUG_PINS */

/**
 * Initialise Debug-features
 */
void UVOS_DEBUG_Init( __attribute__( ( unused ) ) const struct uvos_tim_channel * channels, __attribute__( ( unused ) ) uint8_t num_channels )
{
#ifdef UVOS_ENABLE_DEBUG_PINS
  UVOS_Assert( channels );
  UVOS_Assert( num_channels );

  /* Store away the GPIOs we've been given */
  debug_channels     = channels;
  debug_num_channels = num_channels;

  /* Configure the GPIOs we've been given */
  for ( uint8_t i = 0; i < num_channels; i++ ) {
    const struct uvos_tim_channel * chan = &channels[ i ];

    // Initialise pins as standard output pins
    LL_GPIO_InitTypeDef GPIO_InitStructure;
    LL_GPIO_StructInit( &GPIO_InitStructure );
    GPIO_InitStructure.Pin   = chan->pin.init.Pin;
    GPIO_InitStructure.Mode  = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;

    /* Initialize the GPIO */
    LL_GPIO_Init( chan->pin.gpio, &GPIO_InitStructure );

    /* Set the pin low */
    // GPIO_WriteBit( chan->pin.gpio, chan->pin.init.GPIO_Pin, Bit_RESET );
    LL_GPIO_ResetOutputPin( chan->pin.gpio, chan->pin.init.Pin );
  }
#endif // UVOS_ENABLE_DEBUG_PINS
}

/**
 * Set debug-pin high
 * \param pin 0 for S1 output
 */
void UVOS_DEBUG_PinHigh( __attribute__( ( unused ) ) uint8_t pin )
{
#ifdef UVOS_ENABLE_DEBUG_PINS
  if ( !debug_channels || pin >= debug_num_channels ) {
    return;
  }

  const struct uvos_tim_channel * chan = &debug_channels[ pin ];

  // GPIO_WriteBit( chan->pin.gpio, chan->pin.init.GPIO_Pin, Bit_SET );
  LL_GPIO_SetOutputPin( chan->pin.gpio, chan->pin.init.Pin );

#endif // UVOS_ENABLE_DEBUG_PINS
}

/**
 * Set debug-pin low
 * \param pin 0 for S1 output
 */
void UVOS_DEBUG_PinLow( __attribute__( ( unused ) ) uint8_t pin )
{
#ifdef UVOS_ENABLE_DEBUG_PINS
  if ( !debug_channels || pin >= debug_num_channels ) {
    return;
  }

  const struct uvos_tim_channel * chan = &debug_channels[ pin ];

  // GPIO_WriteBit( chan->pin.gpio, chan->pin.init.GPIO_Pin, Bit_RESET );
  LL_GPIO_ResetOutputPin( chan->pin.gpio, chan->pin.init.Pin );

#endif // UVOS_ENABLE_DEBUG_PINS
}


void UVOS_DEBUG_PinValue8Bit( uint8_t value )
{
#ifdef UVOS_ENABLE_DEBUG_PINS
  if ( !debug_channels ) {
    return;
  }

  uint32_t bsrr_l = ( ( ( ~value ) & 0x0F ) << ( 16 + 6 )   ) | ( ( value & 0x0F ) << 6 );
  uint32_t bsrr_h = ( ( ( ~value ) & 0xF0 ) << ( 16 + 6 - 4 ) ) | ( ( value & 0xF0 ) << ( 6 - 4 ) );

  UVOS_IRQ_Disable();

  /*
   * This is sketchy since it assumes a particular ordering
   * and bitwise layout of the channels provided to the debug code.
   */
  debug_channels[0].pin.gpio->BSRR = bsrr_l;
  debug_channels[4].pin.gpio->BSRR = bsrr_h;

  UVOS_IRQ_Enable();
#endif // UVOS_ENABLE_DEBUG_PINS
}

void UVOS_DEBUG_PinValue4BitL( uint8_t value )
{
#ifdef UVOS_ENABLE_DEBUG_PINS
  if ( !debug_channels ) {
    return;
  }

  /*
   * This is sketchy since it assumes a particular ordering
   * and bitwise layout of the channels provided to the debug code.
   */
  uint32_t bsrr_l = ( ( ~( value & 0x0F ) << ( 16 + 6 ) ) ) | ( ( value & 0x0F ) << 6 );
  debug_channels[0].pin.gpio->BSRR = bsrr_l;

#endif // UVOS_ENABLE_DEBUG_PINS
}


/**
 * Report a serious error and halt
 */
void UVOS_DEBUG_Panic( const char * msg )
{
#if defined(UVOS_INCLUDE_DEBUG_CONSOLE)
  register int * lr asm( "lr" ); // Link-register holds the PC of the caller
  UVOS_COM_SendFormattedStringNonBlocking( UVOS_COM_DEBUG, "\r%s @0x%x\r", msg, lr );
#endif // defined(UVOS_INCLUDE_DEBUG_CONSOLE)

  // Stay put
  while ( 1 ) {
#if defined(UVOS_LED_ALARM)
    UVOS_LED_Toggle( UVOS_LED_ALARM );
#endif // defined(UVOS_LED_ALARM)
    UVOS_DELAY_WaitmS( 100 );
  }
}
