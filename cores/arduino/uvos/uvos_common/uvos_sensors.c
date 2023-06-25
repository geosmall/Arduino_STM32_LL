#include "uvos_sensors.h"
#include <stddef.h>

//! The list of queue handles
static struct uvos_queue *queues[UVOS_SENSOR_LAST];
static int32_t max_gyro_rate;

//! Initialize the sensors interface
int32_t UVOS_SENSORS_Init()
{
	for (uint32_t i = 0; i < UVOS_SENSOR_LAST; i++)
		queues[i] = NULL;

	return 0;
}

//! Register a sensor with the UVOS_SENSORS interface
int32_t UVOS_SENSORS_Register(enum UVOS_SENSOR_Type type, struct uvos_queue *queue)
{
	if(queues[type] != NULL)
		return -1;

	queues[type] = queue;

	return 0;
}

//! Checks if a sensor type is registered with the UVOS_SENSORS interface
bool UVOS_SENSORS_IsRegistered(enum UVOS_SENSOR_Type type)
{
	if(type >= UVOS_SENSOR_LAST)
		return false;

	if(queues[type] != NULL)
		return true;

	return false;
}

//! Get the data queue for a sensor type
struct uvos_queue *UVOS_SENSORS_GetQueue(enum UVOS_SENSOR_Type type)
{
	if (type >= UVOS_SENSOR_LAST)
		return NULL;

	return queues[type];
}

//! Set the maximum gyro rate in deg/s
void UVOS_SENSORS_SetMaxGyro(int32_t rate)
{
	max_gyro_rate = rate;
}

//! Get the maximum gyro rate in deg/s
int32_t UVOS_SENSORS_GetMaxGyro()
{
		return max_gyro_rate;
}
