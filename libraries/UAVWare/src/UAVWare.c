#include "UAVWare.h"

int UAVWare_init( void )
{

  /* Brings up System using CMSIS functions, initializes periph clock, gpio pins. */
  UVOS_SYS_Init();

  /* board driver init */
  if ( UVOS_Board_Init() ) {
#ifdef UVOS_INCLUDE_DEBUG_CONSOLE
    UVOS_DEBUG_Panic( "System initialization Error\r\n" );
#endif // UVOS_INCLUDE_DEBUG_CONSOLE
  } else {
#ifdef UVOS_INCLUDE_DEBUG_CONSOLE
    UVOS_COM_SendString( UVOS_COM_DEBUG, "System initialized\r\n" );
#endif // UVOS_INCLUDE_DEBUG_CONSOLE
  }

  UW_fs_init();
  UW_act_init();

  return 0;
}
