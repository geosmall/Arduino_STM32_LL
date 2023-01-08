#include <uvos.h>
#include "uw_actuator.h"

static uw_actuator_bank_mode_e uvos_actuator_bank_mode[ UVOS_SERVO_BANKS ] = { 0 };

#define ACTUATOR_ONESHOT_CLOCK           12000000
#define ACTUATOR_ONESHOT125_PULSE_FACTOR 1.5f
#define ACTUATOR_ONESHOT42_PULSE_FACTOR  0.5f
#define ACTUATOR_MULTISHOT_PULSE_FACTOR  0.24f
#define ACTUATOR_PWM_CLOCK               1000000

static void UW_act_update_rates( void );

void UW_act_init( void )
{
  /* Set all banks to default continuous asyncronous PWM mode */
  for ( uint8_t i = 0; i < UVOS_SERVO_BANKS; i++ ) {
    // UVOS_Servo_SetBankMode( i, UVOS_SERVO_BANK_MODE_PWM );
    UVOS_Servo_SetBankMode( i, UVOS_SERVO_BANK_MODE_SINGLE_PULSE );
    uvos_actuator_bank_mode[ i ] = ACTUATOR_BANKMODE_PWM;
  }
  UW_act_update_rates();
}

/**
 * Set actuator position
 * \param[in] actuator actuator number (0-UVOS_SERVOPORT_ALL_PINS_PWMOUT_XX from board_hw_defs.c.inc)
 * \param[in] Position actuator command in microseconds
 */
int32_t UW_act_set( uint8_t actuator, uint16_t cmd )
{
  return UVOS_Servo_Set( actuator, cmd );
}

void UW_act_update_bank_mode( uint8_t bank, uw_actuator_bank_mode_e mode )
{
  if ( ( bank >= 0 ) && ( bank < UVOS_SERVO_BANKS ) ) {
    uvos_actuator_bank_mode[ bank ] = mode;
    UW_act_update_rates();
  }
}

static void UW_act_update_rates( void )
{

  uint16_t freq[ UVOS_SERVO_BANKS ];
  uint32_t clock[ UVOS_SERVO_BANKS ] = { 0 };

  for ( uint8_t i = 0; i < UVOS_SERVO_BANKS; i++ ) {
    switch ( uvos_actuator_bank_mode[ i ] ) {
    case ACTUATOR_BANKMODE_ONESHOT125:
      freq[ i ]  = 200; // Value must be small enough so CCr isn't update until the UVOS_Servo_Update() is called
      clock[ i ] = ACTUATOR_ONESHOT_CLOCK; // Setup an 12MHz timer clock
      break;
    case ACTUATOR_BANKMODE_PWMSYNC:
      freq[ i ]  = 200;
      clock[ i ] = ACTUATOR_PWM_CLOCK;
      break;
    default: // PWM
      freq[ i ]  = UVOS_SERVO_UPDATE_HZ;
      clock[ i ] = ACTUATOR_PWM_CLOCK;
      break;
    }
  }

  UVOS_Servo_SetHz( freq, clock, UVOS_SERVO_BANKS );

}
