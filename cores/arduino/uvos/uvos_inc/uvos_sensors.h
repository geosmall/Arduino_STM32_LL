#ifndef UVOS_SENSOR_H
#define UVOS_SENSOR_H

#include "uvos.h"
#include "stdint.h"
#include "uvos_queue.h"

//! Pios sensor structure for generic gyro data
struct uvos_sensor_gyro_data {
	float x;
	float y;
	float z;
	float temperature;
};

//! Pios sensor structure for generic accel data
struct uvos_sensor_accel_data {
	float x;
	float y;
	float z;
	float temperature;
};

//! Pios sensor structure for generic mag data
struct uvos_sensor_mag_data {
	float x;
	float y;
	float z;
};

//! Pios sensor structure for generic mag data
struct uvos_sensor_optical_flow_data {
	float x_dot;
	float y_dot;
	float z_dot;

	uint8_t quality;
};

//! Pios sensor structure for generic rangefinder data
struct uvos_sensor_rangefinder_data {
	float range;
	uint8_t range_status;
};

//! Pios sensor structure for generic baro data
struct uvos_sensor_baro_data {
	float temperature;
	float pressure;
	float altitude;
};

//! The types of sensors this module supports
enum UVOS_SENSOR_Type {
	UVOS_SENSOR_ACCEL,
	UVOS_SENSOR_GYRO,
	// UVOS_SENSOR_MAG,
	// UVOS_SENSOR_BARO,
	// UVOS_SENSOR_OPTICAL_FLOW,
	// UVOS_SENSOR_RANGEFINDER,
	UVOS_SENSOR_LAST
};

typedef bool ( *UVOS_SENSORS_test_function )( uintptr_t context );
typedef void ( *UVOS_SENSORS_reset_function )( uintptr_t context );
typedef bool ( *UVOS_SENSORS_poll_function )( uintptr_t context );
typedef void ( *UVOS_SENSORS_fetch_function )( void *samples, uint8_t size, uintptr_t context );
typedef struct uvos_queue *( *UVOS_SENSORS_get_queue_function )( uintptr_t context );
typedef void ( *UVOS_SENSORS_get_scale_function )( float *, uint8_t size, uintptr_t context );

struct UVOS_SENSORS_Driver {
	UVOS_SENSORS_test_function      test; // called at startup to test the sensor
	UVOS_SENSORS_poll_function      poll; // called to check whether data are available for polled sensors
	UVOS_SENSORS_fetch_function     fetch; // called to fetch data for polled sensors
	UVOS_SENSORS_reset_function     reset; // reset sensor. for example if data are not received in the allotted time
	UVOS_SENSORS_get_queue_function get_queue; // get the queue reference
	UVOS_SENSORS_get_scale_function get_scale; // return scales for the sensors
};

struct UVOS_SENSORS_Instance {
	const struct UVOS_SENSORS_Driver *driver;
	enum UVOS_SENSOR_Type             type;
};

//! Initialize the PIOS_SENSORS interface
int32_t UVOS_SENSORS_Init();

//! Register a sensor with the PIOS_SENSORS interface
int32_t UVOS_SENSORS_Register(enum UVOS_SENSOR_Type type, struct uvos_queue *queue);

//! Checks if a sensor type is registered with the UVOS_SENSORS interface
bool UVOS_SENSORS_IsRegistered( enum UVOS_SENSOR_Type type );

//! Get the data queue for a sensor type
struct uvos_queue *UVOS_SENSORS_GetQueue( enum UVOS_SENSOR_Type type );

//! Set the maximum gyro rate in deg/s
void UVOS_SENSORS_SetMaxGyro( int32_t rate );

//! Get the maximum gyro rate in deg/s
int32_t UVOS_SENSORS_GetMaxGyro();

#endif /* UVOS_SENSOR_H */
