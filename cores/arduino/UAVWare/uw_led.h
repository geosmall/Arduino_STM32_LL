#ifndef UW_LED_H
#define UW_LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Main Functions */
extern void UW_led_heartbeat_on( void );
extern void UW_led_heartbeat_off( void );
extern void UW_led_heartbeat_toggle( void );

#if defined(UVOS_LED_ALARM)
extern void UW_led_alarm_on( void );
extern void UW_led_alarm_off( void );
extern void UW_led_alarm_toggle( void );
#endif // UVOS_LED_ALARM

#ifdef __cplusplus
}
#endif

#endif // UW_LED_H