#ifndef UVOS_SERVO_H
#define UVOS_SERVO_H

/* Global types */
typedef enum uvos_servo_bank_mode {
  UVOS_SERVO_BANK_MODE_PWM = 0,
  UVOS_SERVO_BANK_MODE_SINGLE_PULSE = 1
} uvos_servo_bank_mode_e;

/* Public Functions */
extern void UVOS_Servo_SetHz( const uint16_t * speeds, const uint32_t * clock, uint8_t banks );
extern void UVOS_Servo_Set( uint8_t Servo, uint16_t Position );
extern void UVOS_Servo_Update();
extern void UVOS_Servo_SetBankMode( uint8_t bank, uvos_servo_bank_mode_e mode );
extern uint8_t UVOS_Servo_GetPinBank( uint8_t pin );

#endif /* UVOS_SERVO_H */
