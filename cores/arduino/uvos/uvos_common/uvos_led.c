#include "uvos.h"

#ifdef UVOS_INCLUDE_LED

#include <uvos_gpio_priv.h>
#include <uvos_gpio.h>

static uint32_t uvos_led_gpios_id;

/**
 * Initialises all the LED's
 */
int32_t UVOS_LED_Init( const struct uvos_gpio_cfg * cfg )
{
  UVOS_Assert( cfg );
  return UVOS_GPIO_Init( &uvos_led_gpios_id, cfg );
}

/**
 * Turn on LED
 * \param[in] LED LED id
 */
void UVOS_LED_On( uint32_t led_id )
{
  UVOS_GPIO_On( uvos_led_gpios_id, led_id );
}

/**
 * Turn off LED
 * \param[in] LED LED id
 */
void UVOS_LED_Off( uint32_t led_id )
{
  UVOS_GPIO_Off( uvos_led_gpios_id, led_id );
}

/**
 * Toggle LED on/off
 * \param[in] LED LED id
 */
void UVOS_LED_Toggle( uint32_t led_id )
{
  UVOS_GPIO_Toggle( uvos_led_gpios_id, led_id );
}

#endif /* UVOS_INCLUDE_LED */

/**
 * @}
 * @}
 */
