#ifndef UW_ACTUATOR_H
#define UW_ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

// Enumeration options for field BankMode
typedef enum {
    ACTUATOR_BANKMODE_PWM = 0,
    ACTUATOR_BANKMODE_PWMSYNC = 1,
    ACTUATOR_BANKMODE_ONESHOT125 = 2,
} uw_actuator_bank_mode_e;

/* Main Functions */
extern void UW_act_init( void );
extern void UW_act_update_bank_mode( uint8_t bank, uw_actuator_bank_mode_e mode );
extern int32_t UW_act_set( uint8_t actuator, uint16_t cmd );

#ifdef __cplusplus
}
#endif

#endif // UW_ACTUATOR_H
