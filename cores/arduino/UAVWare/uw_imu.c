#include <uvos.h>
#include "uw_imu.h"

#define MAX_SENSORS_PER_INSTANCE 2

#define MAX_SENSOR_DATA_SIZE (sizeof(UVOS_SENSORS_3Axis_SensorsWithTemp) + MAX_SENSORS_PER_INSTANCE * sizeof(Vector3i16))
typedef union {
  UVOS_SENSORS_3Axis_SensorsWithTemp sensorSample3Axis;
  UVOS_SENSORS_1Axis_SensorsWithTemp sensorSample1Axis;
} sensor_data;

static sensor_data *gp_imu_data;
static UVOS_SENSORS_Instance *gp_imu;

/* UVOS Sensors interface saves registered sensor instances in a linked
 * list which can be retrieved with UVOS_SENSORS_GetList().  You can
 * then walk through this list using LL_FOREACH utility macro to find
 * the imu sensor (type defined as UVOS_SENSORS_TYPE_3AXIS_GYRO_ACCEL)
 */
bool UW_imu_init( void )
{
  UVOS_SENSORS_Instance *p_sensor;
  const UVOS_SENSORS_Instance *p_sensors_list;

  gp_imu_data = ( sensor_data * )UVOS_malloc( MAX_SENSOR_DATA_SIZE );
  p_sensors_list = UVOS_SENSORS_GetList();

  // Scan sensor linked list for accgyro
  LL_FOREACH( ( UVOS_SENSORS_Instance * )p_sensors_list, p_sensor ) {
    if ( p_sensor->type & UVOS_SENSORS_TYPE_3AXIS_GYRO_ACCEL ) {
      gp_imu = p_sensor; /* save sensor instance */
    }
  }

  return true;
}

int UW_imu_set_ranges( IMUGyroAccelSettingsGyroScaleOptions GyroScale, IMUGyroAccelSettingsAccelScaleOptions AccelScale )
{
  /* Always use 256Hz HW filter for default */
  IMUGyroAccelSettingsData settings = { GyroScale, AccelScale, IMUGYROACCELSETTINGS_FILTERSETTING_DEFAULT, false };
  IMUGyroAccelSettingsSet( &settings );
  return 0;
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param  ax 16-bit signed integer for accelerometer X-axis value
 *         ay 16-bit signed integer for accelerometer Y-axis value
 *         az 16-bit signed integer for accelerometer Z-axis value
 *         gx 16-bit signed integer for gyroscope X-axis value
 *         gy 16-bit signed integer for gyroscope Y-axis value
 *         gz 16-bit signed integer for gyroscope Z-axis value
 * @retval 0
 */
int UW_imu_read_imu( int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz )
{
  if ( UVOS_SENSORS_Poll( gp_imu ) ) {
    UVOS_SENSOR_Fetch( gp_imu, ( void * )gp_imu_data, MAX_SENSORS_PER_INSTANCE );
    uint32_t timestamp = gp_imu_data->sensorSample3Axis.timestamp;
    *ax = gp_imu_data->sensorSample3Axis.sample[0].x;
    *ay = gp_imu_data->sensorSample3Axis.sample[0].y;
    *az = gp_imu_data->sensorSample3Axis.sample[0].z;
    *gx = gp_imu_data->sensorSample3Axis.sample[1].x;
    *gy = gp_imu_data->sensorSample3Axis.sample[1].y;
    *gz = gp_imu_data->sensorSample3Axis.sample[1].z;
    return 0;
  }
  return -1;
}

int UW_imu_read_imu9( int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz )
{
  UVOS_DEBUG_Panic( "9DOF IMU not implemented yet\r\n" );
  return -1;
}
