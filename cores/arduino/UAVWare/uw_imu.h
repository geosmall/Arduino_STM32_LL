#ifndef UW_MPU_H
#define UW_MPU_H

#include "mpugyroaccelsettings.h"

#define GYRO_SCALE_250DPS MPUGYROACCELSETTINGS_GYROSCALE_SCALE_250
#define GYRO_SCALE_500DPS MPUGYROACCELSETTINGS_GYROSCALE_SCALE_500
#define GYRO_SCALE_1000DPS MPUGYROACCELSETTINGS_GYROSCALE_SCALE_1000
#define GYRO_SCALE_2000DPS MPUGYROACCELSETTINGS_GYROSCALE_SCALE_2000

#define ACCEL_SCALE_2G MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_2G
#define ACCEL_SCALE_4G MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_4G
#define ACCEL_SCALE_8G MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_8G
#define ACCEL_SCALE_16G MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_16G

#define MPUGYROACCELSETTINGS_GYROSCALE_DEFAULT MPUGYROACCELSETTINGS_GYROSCALE_SCALE_2000
#define MPUGYROACCELSETTINGS_ACCELSCALE_DEFAULT MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_8G

#ifdef __cplusplus
extern "C" {
#endif

extern bool UW_imu_init( void );
extern int UW_imu_set_ranges( MPUGyroAccelSettingsGyroScaleOptions GyroScale, MPUGyroAccelSettingsAccelScaleOptions AccelScale );
extern int UW_imu_read_imu( int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz );
extern int UW_imu_read_imu9( int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz );
// extern int UW_imu_set_gyro_range( uvos_mpu_gyro_range_t gyro_range );
// extern int UW_imu_set_accel_range( uvos_mpu_accel_range_t accel_range );

#ifdef __cplusplus
}
#endif

#endif // UW_MPU_H