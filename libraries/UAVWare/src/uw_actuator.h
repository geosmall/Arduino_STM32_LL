#ifndef UW_ACTUATOR_H
#define UW_ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

// Enumeration options for field BankMode
typedef enum {
    ACTUATOR_BANKMODE_PWM=0,
    ACTUATOR_BANKMODE_PWMSYNC=1,
    ACTUATOR_BANKMODE_ONESHOT125=2,
} uw_actuator_bank_mode_e;

/* Main Functions */
extern void UW_act_init( void );

#ifdef __cplusplus
}
#endif

#endif // UW_ACTUATOR_H
