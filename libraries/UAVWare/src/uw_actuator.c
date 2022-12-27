#include <uvos.h>
#include "uw_actuator.h"

static uw_actuator_bank_mode_e uvos_actuator_bank_mode[ UVOS_SERVO_BANKS ] = { 0 };

#define ACTUATOR_ONESHOT_CLOCK           12000000
#define ACTUATOR_ONESHOT125_PULSE_FACTOR 1.5f
#define ACTUATOR_ONESHOT42_PULSE_FACTOR  0.5f
#define ACTUATOR_MULTISHOT_PULSE_FACTOR  0.24f
#define ACTUATOR_PWM_CLOCK               1000000

static void UW_act_update_rate( void );

void UW_act_init( void )
{
  /* TIM1 - Bank0 */
  UVOS_Servo_SetBankMode( 0, UVOS_SERVO_BANK_MODE_PWM );
  uvos_actuator_bank_mode[ 0 ] = ACTUATOR_BANKMODE_PWM;
  /* TIM3 - Bank1 */
  UVOS_Servo_SetBankMode( 1, UVOS_SERVO_BANK_MODE_PWM );
  uvos_actuator_bank_mode[ 1 ] = ACTUATOR_BANKMODE_PWM;

  UW_act_update_rate();

}

static void UW_act_update_rate( void )
{

  uint16_t freq[ UVOS_SERVO_BANKS ];
  uint32_t clock[ UVOS_SERVO_BANKS ] = { 0 };

  for ( uint8_t i = 0; i < UVOS_SERVO_BANKS; i++ ) {
    switch ( uvos_actuator_bank_mode[ i ] ) {
    case ACTUATOR_BANKMODE_ONESHOT125:
      freq[ i ]  = 100; // Value must be small enough so CCr isn't update until the UVOS_Servo_Update() is called
      clock[ i ] = ACTUATOR_ONESHOT_CLOCK; // Setup an 12MHz timer clock
      break;
    case ACTUATOR_BANKMODE_PWMSYNC:
      freq[ i ]  = 100;
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
