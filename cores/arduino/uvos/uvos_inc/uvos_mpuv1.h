#ifndef UVOS_MPUV1_H
#define UVOS_MPUV1_H

#include "uvos_mpu_priv.h"
#include "uvos_mpu_regs.h"

typedef enum uvos_mpuv1_gyro_range {
  UVOS_MPUV1_GYRO_FS_250DPS_BITS = BITS_GYRO_FS_250DPS,
  UVOS_MPUV1_GYRO_FS_500DPS_BITS = BITS_GYRO_FS_500DPS,
  UVOS_MPUV1_GYRO_FS_1000DPS_BITS = BITS_GYRO_FS_1000DPS,
  UVOS_MPUV1_GYRO_FS_2000DPS_BITS = BITS_GYRO_FS_2000DPS,
} uvos_mpuv1_gyro_range_t;

typedef enum uvos_mpuv1_accel_range {
  UVOS_MPUV1_ACCEL_FS_2G_BITS = BITS_ACCEL_FS_2G,
  UVOS_MPUV1_ACCEL_FS_4G_BITS = BITS_ACCEL_FS_4G,
  UVOS_MPUV1_ACCEL_FS_8G_BITS = BITS_ACCEL_FS_8G,
  UVOS_MPUV1_ACCEL_FS_16G_BITS = BITS_ACCEL_FS_16G,
} uvos_mpuv1_accel_range_t;

/*
The sensor register output, FIFO output, and DMP sampling are all based on the Sample Rate.
Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7),
                        1kHz when the DLPF is enabled  (see Register 26, CONFIG register).
Values here assume DLPF = 0, 8kHz output rate.
*/
typedef enum uvos_mpuv1_samplerate_div {
  UVOS_MPUV1_8K_SMPLRT_8000HZ = 0,
  UVOS_MPUV1_8K_SMPLRT_4000HZ = 1,
  UVOS_MPUV1_8K_SMPLRT_2000HZ = 3,
  UVOS_MPUV1_8K_SMPLRT_1000HZ = 7,
  UVOS_MPUV1_8K_SMPLRT_500HZ = 15,
  UVOS_MPUV1_1K_SMPLRT_1000HZ = 0,
  UVOS_MPUV1_1K_SMPLRT_500HZ = 1,
  UVOS_MPUV1_1K_SMPLRT_250HZ = 3,
} uvos_mpuv1_samplerate_div_t;

typedef enum uvos_mpuv1_filter {
  UVOS_MPUV1_LOWPASS_256_HZ_BITS = BITS_DLPF_CFG_256HZ_NOLPF2,
  UVOS_MPUV1_LOWPASS_188_HZ_BITS = BITS_DLPF_CFG_188HZ,
  UVOS_MPUV1_LOWPASS_98_HZ_BITS = BITS_DLPF_CFG_98HZ,
  UVOS_MPUV1_LOWPASS_42_HZ_BITS = BITS_DLPF_CFG_42HZ,
  UVOS_MPUV1_LOWPASS_20_HZ_BITS = BITS_DLPF_CFG_20HZ,
  UVOS_MPUV1_LOWPASS_10_HZ_BITS = BITS_DLPF_CFG_10HZ,
  UVOS_MPUV1_LOWPASS_5_HZ_BITS = BITS_DLPF_CFG_5HZ,
  UVOS_MPUV1_LOWPASS_2100HZ_HZ_NOLPF = BITS_DLPF_CFG_2100HZ_NOLPF,
} uvos_mpuv1_filter_t;

// extern int32_t UVOS_MPUV1_ConfigureRanges( uvos_mpuv1_gyro_range_t gyroRange, uvos_mpuv1_accel_range_t accelRange, uvos_mpuv1_filter_t filterSetting );
extern int32_t UVOS_MPUV1_Hardware_Init( mpu_dev_t *dev );
extern int32_t UVOS_MPUV1_ConfigGyroRange( uvos_mpu_gyro_range_t gyroRange );
extern int32_t UVOS_MPUV1_ConfigAccelRange( uvos_mpu_accel_range_t accelRange );
extern int32_t UVOS_MPUV1_ConfigFilter( uvos_mpuv1_filter_t filterSetting );
extern int32_t UVOS_MPUV1_ConfigSampleRate( uvos_mpuv1_samplerate_div_t sampleRate );

#endif // UVOS_MPUV1_H
