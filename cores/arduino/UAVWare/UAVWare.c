#include "UAVWare.h"

int UAVWare_set_imu_ranges( IMUGyroAccelSettingsGyroScaleOptions GyroScale, IMUGyroAccelSettingsAccelScaleOptions AccelScale )
{
  return UW_imu_set_ranges( GyroScale, AccelScale );
}

int UAVWare_init( void )
{
  /* Brings up System using CMSIS functions, initializes periph clock, gpio pins. */
  UVOS_SYS_Init();

  /* board driver init */
  int32_t ret = UVOS_Board_Init();
  if ( ret ) {
#ifdef UVOS_INCLUDE_DEBUG_CONSOLE
    UVOS_DEBUG_Panic( "System initialization Error\r\n" );
#endif // UVOS_INCLUDE_DEBUG_CONSOLE
  } else {
#ifdef UVOS_INCLUDE_DEBUG_CONSOLE
    UVOS_COM_SendString( UVOS_COM_DEBUG, "System initialized\r\n" );
#endif // UVOS_INCLUDE_DEBUG_CONSOLE
  }

  UW_fs_init();
  UW_imu_init();
  UW_act_init();
  UW_sched_init();

  UW_sched_start();

  return 0;
}
