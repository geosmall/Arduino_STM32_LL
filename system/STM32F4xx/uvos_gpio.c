#include "uvos.h"

#ifdef UVOS_INCLUDE_GPIO

#include <uvos_gpio_priv.h>

#define IS_LL_GPIO_PIN(__VALUE__)          (((0x00000000U) < (__VALUE__)) && ((__VALUE__) <= (LL_GPIO_PIN_ALL)))


/**
 * Helper function sets a GPIO pin's alternate function reg
 * @note   Configure gpio alternate function of a dedicated pin from 0 to 7 for a dedicated port.
 * @note   Possible values are from AF0 to AF15 depending on target.
 * @note   Warning: only one pin can be passed as parameter.
 */
void UVOS_GPIO_SetAFPin( GPIO_TypeDef * gpio, uint32_t pin, uint32_t afnum )
{
  // Implementation hints from LL_GPIO_Init()
  uint32_t pinpos = POSITION_VAL( pin );
  uint32_t currentpin = ( pin ) & ( 0x00000001U << pinpos );

  if ( afnum ) {
    // GPIO_PinAFConfig( gpio->pin.gpio, gpio->pin.init.GPIO_Pin, gpio->remap );
    if ( POSITION_VAL( currentpin ) < 0x00000008U ) {
      LL_GPIO_SetAFPin_0_7( gpio, pin, afnum );
    } else {
      LL_GPIO_SetAFPin_8_15( gpio, pin, afnum );
    }
  }

}

/**
 * Initialises all the GPIO's
 */
int32_t UVOS_GPIO_Init( uint32_t * gpios_dev_id, const struct uvos_gpio_cfg * cfg )
{
  UVOS_Assert( cfg );
  *gpios_dev_id = ( uint32_t )cfg;

  for ( uint8_t i = 0; i < cfg->num_gpios; i++ ) {

    const struct uvos_gpio * gpio = &( cfg->gpios[i] );

    /* Check that pin is valid */
    UVOS_Assert( IS_LL_GPIO_PIN( gpio->pin.init.Pin ) );

    if ( gpio->pin.init.Alternate ) {
      UVOS_GPIO_SetAFPin( gpio->pin.gpio, gpio->pin.init.Pin, gpio->pin.init.Alternate );
    }

    LL_GPIO_Init( gpio->pin.gpio, ( LL_GPIO_InitTypeDef * ) &gpio->pin.init );

    UVOS_GPIO_Off( *gpios_dev_id, i );
  }

  return 0;

}

/**
 * Turn on GPIO
 * \param[in] GPIO GPIO id
 */
void UVOS_GPIO_On( uint32_t gpios_dev_id, uint8_t gpio_id )
{
  const struct uvos_gpio_cfg * gpio_cfg = ( const struct uvos_gpio_cfg * )gpios_dev_id;

  UVOS_Assert( gpio_cfg );

  if ( gpio_id >= gpio_cfg->num_gpios ) {
    /* GPIO index out of range */
    return;
  }

  const struct uvos_gpio * gpio = &( gpio_cfg->gpios[gpio_id] );

  if ( gpio->active_low ) {
    LL_GPIO_ResetOutputPin( gpio->pin.gpio, gpio->pin.init.Pin );
  } else {
    LL_GPIO_SetOutputPin( gpio->pin.gpio, gpio->pin.init.Pin );
  }
}

/**
 * Turn off GPIO
 * \param[in] GPIO GPIO id
 */
void UVOS_GPIO_Off( uint32_t gpios_dev_id, uint8_t gpio_id )
{
  const struct uvos_gpio_cfg * gpio_cfg = ( const struct uvos_gpio_cfg * )gpios_dev_id;

  UVOS_Assert( gpio_cfg );

  if ( gpio_id >= gpio_cfg->num_gpios ) {
    /* GPIO index out of range */
    return;
  }

  const struct uvos_gpio * gpio = &( gpio_cfg->gpios[gpio_id] );

  if ( gpio->active_low ) {
    LL_GPIO_SetOutputPin( gpio->pin.gpio, gpio->pin.init.Pin );
  } else {
    LL_GPIO_ResetOutputPin( gpio->pin.gpio, gpio->pin.init.Pin );
  }
}

/**
 * Toggle GPIO on/off
 * \param[in] GPIO GPIO id
 */
void UVOS_GPIO_Toggle( uint32_t gpios_dev_id, uint8_t gpio_id )
{
  const struct uvos_gpio_cfg * gpio_cfg = ( const struct uvos_gpio_cfg * )gpios_dev_id;

  UVOS_Assert( gpio_cfg );

  if ( gpio_id >= gpio_cfg->num_gpios ) {
    /* GPIO index out of range */
    return;
  }

  const struct uvos_gpio * gpio = &( gpio_cfg->gpios[gpio_id] );

  // if ( GPIO_ReadOutputDataBit( gpio->pin.gpio, gpio->pin.init.GPIO_Pin ) == Bit_SET ) {
  //   if ( gpio->active_low ) {
  //     UVOS_GPIO_On( gpios_dev_id, gpio_id );
  //   } else {
  //     UVOS_GPIO_Off( gpios_dev_id, gpio_id );
  //   }
  // } else {
  //   if ( gpio->active_low ) {
  //     UVOS_GPIO_Off( gpios_dev_id, gpio_id );
  //   } else {
  //     UVOS_GPIO_On( gpios_dev_id, gpio_id );
  //   }

  LL_GPIO_TogglePin( gpio->pin.gpio, gpio->pin.init.Pin );

}

#endif /* UVOS_INCLUDE_GPIO */

/**
 * @}
 * @}
 */
