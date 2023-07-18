#ifndef UVOS_SENSORS_H
#define UVOS_SENSORS_H
#include <uvos.h>
#include <utlist.h>
#include <stdint.h>
#include <vectors.h>
#include "uvos_queue.h"
#include "uvos_debug.h"

// needed for debug APIs.

typedef bool ( *UVOS_SENSORS_test_function )( uintptr_t context );
typedef void ( *UVOS_SENSORS_reset_function )( uintptr_t context );
typedef bool ( *UVOS_SENSORS_poll_function )( uintptr_t context );
typedef void ( *UVOS_SENSORS_fetch_function )( void *samples, uint8_t size, uintptr_t context );
typedef p_uvos_queue_t ( *UVOS_SENSORS_get_queue_function )( uintptr_t context );
/**
 * return an array with current scale for the instance.
 * Instances with multiples sensors returns several value in the same
 * order as they appear in UVOS_SENSORS_TYPE enums.
 */
typedef void ( *UVOS_SENSORS_get_scale_function )( float *, uint8_t size, uintptr_t context );

typedef struct UVOS_SENSORS_Driver {
  UVOS_SENSORS_test_function      test; // called at startup to test the sensor
  UVOS_SENSORS_poll_function      poll; // called to check whether data are available for polled sensors
  UVOS_SENSORS_fetch_function     fetch; // called to fetch data for polled sensors
  UVOS_SENSORS_reset_function     reset; // reset sensor. for example if data are not received in the allotted time
  UVOS_SENSORS_get_queue_function get_queue; // get the queue reference
  UVOS_SENSORS_get_scale_function get_scale; // return scales for the sensors
  bool is_polled;
} UVOS_SENSORS_Driver;

typedef enum UVOS_SENSORS_TYPE {
  UVOS_SENSORS_TYPE_3AXIS_ACCEL      = 0x01,
  UVOS_SENSORS_TYPE_3AXIS_GYRO       = 0x02,
  UVOS_SENSORS_TYPE_3AXIS_GYRO_ACCEL = 0x03,
  UVOS_SENSORS_TYPE_3AXIS_MAG        = 0x04,
  UVOS_SENSORS_TYPE_3AXIS_AUXMAG     = 0x08,
  UVOS_SENSORS_TYPE_1AXIS_BARO       = 0x10,
} UVOS_SENSORS_TYPE;

#define UVOS_SENSORS_TYPE_1D (UVOS_SENSORS_TYPE_1AXIS_BARO)
#define UVOS_SENSORS_TYPE_3D (UVOS_SENSORS_TYPE_3AXIS_ACCEL | UVOS_SENSORS_TYPE_3AXIS_GYRO | UVOS_SENSORS_TYPE_3AXIS_MAG | UVOS_SENSORS_TYPE_3AXIS_AUXMAG)

typedef struct UVOS_SENSORS_Instance {
  const UVOS_SENSORS_Driver    *driver;
  uintptr_t context;
  struct UVOS_SENSORS_Instance *next;
  UVOS_SENSORS_TYPE type;
} UVOS_SENSORS_Instance;

/**
 * A 3d Accel sample with temperature
 */
typedef struct UVOS_SENSORS_3Axis_SensorsWithTemp {
  uint32_t   timestamp;    // UVOS_DELAY_GetRaw() time of sensor read
  uint16_t   count;        // number of sensor instances
  int16_t    temperature;  // Degrees Celsius * 100
  Vector3i16 sample[];     // C99 struct flexible array member
} UVOS_SENSORS_3Axis_SensorsWithTemp;

typedef struct UVOS_SENSORS_1Axis_SensorsWithTemp {
  float sample; // sample
  float temperature; // Degrees Celsius
} UVOS_SENSORS_1Axis_SensorsWithTemp;

/**
 * Register a new sensor instance with sensor subsystem
 * @param driver sensor driver
 * @param type sensor type @ref UVOS_SENSORS_TYPE
 * @param context context to be passed to sensor driver
 * @return the new sensor instance
 */

UVOS_SENSORS_Instance *UVOS_SENSORS_Register( const UVOS_SENSORS_Driver *driver, UVOS_SENSORS_TYPE type, uintptr_t context );
/**
 * return the list of registered sensors.
 * @return the first sensor instance in the list.
 */
UVOS_SENSORS_Instance *UVOS_SENSORS_GetList();

/**
 * Perform sensor test and return true if passed
 * @param sensor instance to test
 * @return true if test passes
 */
static inline bool UVOS_SENSORS_Test( const UVOS_SENSORS_Instance *sensor )
{
  UVOS_Assert( sensor );

  if ( !sensor->driver->test ) {
    return true;
  } else {
    return sensor->driver->test( sensor->context );
  }
}

/**
 * Poll sensor for new values
 * @param sensor instance to poll
 * @return true if sensor has samples available
 */
static inline bool UVOS_SENSORS_Poll( const UVOS_SENSORS_Instance *sensor )
{
  UVOS_Assert( sensor );

  if ( !sensor->driver->poll ) {
    return true;
  } else {
    return sensor->driver->poll( sensor->context );
  }
}
/**
 *
 * @param sensor
 * @param samples
 * @param size
 */
static inline void UVOS_SENSOR_Fetch( const UVOS_SENSORS_Instance *sensor, void *samples, uint8_t size )
{
  UVOS_Assert( sensor );
  sensor->driver->fetch( samples, size, sensor->context );
}

static inline void UVOS_SENSOR_Reset( const UVOS_SENSORS_Instance *sensor )
{
  UVOS_Assert( sensor );
  sensor->driver->reset( sensor->context );
}

/**
 * retrieve the sensor queue
 * @param sensor
 * @return sensor queue or null if not supported
 */
static inline p_uvos_queue_t UVOS_SENSORS_GetQueue( const UVOS_SENSORS_Instance *sensor )
{
  UVOS_Assert( sensor );
  if ( !sensor->driver->get_queue ) {
    return NULL;
  }
  return sensor->driver->get_queue( sensor->context );
}
/**
 * Get the sensor scales.
 * @param sensor sensor instance
 * @param scales float array that will contains scales
 * @param size number of floats within the array
 */
static inline void UVOS_SENSORS_GetScales( const UVOS_SENSORS_Instance *sensor, float *scales, uint8_t size )
{
  UVOS_Assert( sensor );
  sensor->driver->get_scale( scales, size, sensor->context );
}
/**
 * return head of sensor list
 * @return head of sensor list
 */
UVOS_SENSORS_Instance *UVOS_SENSORS_GetList();

/**
 * Return the first occurrence of specified sensor type
 * @param previous_instance last instance found or 0
 * @param type type of sensor to find
 * @return the first occurence found or NULL
 */
UVOS_SENSORS_Instance *UVOS_SENSORS_GetInstanceByType( const UVOS_SENSORS_Instance *previous_instance, UVOS_SENSORS_TYPE type );

#endif /* UVOS_SENSORS_H */
