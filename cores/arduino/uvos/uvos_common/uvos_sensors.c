#include <uvos_heap.h>
#include <uvos_sensors.h>
#include <string.h>

// private variables

static UVOS_SENSORS_Instance *sensor_list = 0;

UVOS_SENSORS_Instance *UVOS_SENSORS_Register( const UVOS_SENSORS_Driver *driver, UVOS_SENSORS_TYPE type, uintptr_t context )
{
  UVOS_SENSORS_Instance *instance = ( UVOS_SENSORS_Instance * )UVOS_malloc( sizeof( UVOS_SENSORS_Instance ) );

  instance->driver  = driver;
  instance->type    = type;
  instance->context = context;
  instance->next    = NULL;
  LL_APPEND( sensor_list, instance );
  return instance;
}

UVOS_SENSORS_Instance *UVOS_SENSORS_GetList()
{
  return sensor_list;
}

UVOS_SENSORS_Instance *UVOS_SENSORS_GetInstanceByType( const UVOS_SENSORS_Instance *previous_instance, UVOS_SENSORS_TYPE type )
{
  if ( !previous_instance ) {
    previous_instance = sensor_list;
  }
  UVOS_SENSORS_Instance *sensor;

  LL_FOREACH( ( UVOS_SENSORS_Instance * )previous_instance, sensor ) {
    if ( sensor->type & type ) {
      return sensor;
    }
  }
  return NULL;
}
