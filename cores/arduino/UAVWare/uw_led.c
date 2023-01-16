#include <uvos.h>


void UW_led_heartbeat_on( void )
{
  UVOS_LED_On( UVOS_LED_HEARTBEAT );
}

void UW_led_heartbeat_off( void )
{
  UVOS_LED_Off( UVOS_LED_HEARTBEAT );
}

void UW_led_heartbeat_toggle( void )
{
  UVOS_LED_Toggle( UVOS_LED_HEARTBEAT );
}

#if defined(UVOS_LED_ALARM)

void UW_led_alarm_on( void )
{
  UVOS_LED_On( UVOS_LED_ALARM );
}

void UW_led_alarm_off( void )
{
  UVOS_LED_Off( UVOS_LED_ALARM );
}

void UW_led_alarm_toggle( void )
{
  UVOS_LED_Toggle( UVOS_LED_ALARM );
}

#endif // UVOS_LED_ALARM
