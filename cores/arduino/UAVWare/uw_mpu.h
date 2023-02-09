#ifndef UW_MPU_H
#define UW_MPU_H

#include <uvos_mpu.h>

#define GYRO_SCALE_250DPS UVOS_GYRO_RANGE_250DPS
#define GYRO_SCALE_500DPS UVOS_GYRO_RANGE_500DPS
#define GYRO_SCALE_1000DPS UVOS_GYRO_RANGE_1000DPS
#define GYRO_SCALE_2000DPS UVOS_GYRO_RANGE_2000DPS

#define ACCEL_SCALE_2G UVOS_ACCEL_RANGE_2G
#define ACCEL_SCALE_4G UVOS_ACCEL_RANGE_4G
#define ACCEL_SCALE_8G UVOS_ACCEL_RANGE_8G
#define ACCEL_SCALE_16G UVOS_ACCEL_RANGE_16G

#ifdef __cplusplus
extern "C" {
#endif

// bool mpu_init( void );
extern int32_t UW_mpu_read_imu( int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz );
extern int32_t UW_mpu_read_imu9( int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz );
extern int32_t UW_mpu_set_gyro_range( uvos_mpu_gyro_range_t gyro_range );
extern int32_t UW_mpu_set_accel_range( uvos_mpu_accel_range_t accel_range );

#ifdef __cplusplus
}
#endif

#endif // UW_MPU_H