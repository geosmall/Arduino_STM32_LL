#ifndef UVOS_MPU_H
#define UVOS_MPU_H

#include "uvos_mpu_regs.h"

typedef enum uvos_mpu_invensense_type {
  Invalid_IMU_ID = 0,
  Invensense_MPU6000,
  Invensense_MPU6500,
  Invensense_MPU9250,
  Invensense_ICM20608,
  Invensense_ICM20602,
  Invensense_ICM20601,
  Invensense_ICM20789,
  Invensense_ICM20689,
} invensense_type_e;

enum uvos_mpu_gyro_range {
  UVOS_GYRO_FS_250DPS_BITS = BITS_GYRO_FS_250DPS,
  UVOS_GYRO_FS_500DPS_BITS = BITS_GYRO_FS_500DPS,
  UVOS_GYRO_FS_1000DPS_BITS = BITS_GYRO_FS_1000DPS,
  UVOS_GYRO_FS_2000DPS_BITS = BITS_GYRO_FS_2000DPS,
};

enum uvos_mpu_accel_range {
  UVOS_ACCEL_FS_2G_BITS = BITS_ACCEL_FS_2G,
  UVOS_ACCEL_FS_4G_BITS = BITS_ACCEL_FS_4G,
  UVOS_ACCEL_FS_8G_BITS = BITS_ACCEL_FS_8G,
  UVOS_ACCEL_FS_16G_BITS = BITS_ACCEL_FS_16G,
};

enum uvos_mpu_filter {
  UVOS_LOWPASS_256_HZ_BITS = BITS_DLPF_CFG_256HZ_NOLPF2,
  UVOS_LOWPASS_188_HZ_BITS = BITS_DLPF_CFG_188HZ,
  UVOS_LOWPASS_98_HZ_BITS  = BITS_DLPF_CFG_98HZ,
  UVOS_LOWPASS_42_HZ_BITS  = BITS_DLPF_CFG_42HZ,
  UVOS_LOWPASS_20_HZ_BITS  = BITS_DLPF_CFG_20HZ,
  UVOS_LOWPASS_10_HZ_BITS  = BITS_DLPF_CFG_10HZ,
  UVOS_LOWPASS_5_HZ_BITS   = BITS_DLPF_CFG_5HZ,
};

#define UVOS_MPU_MAX_TRIES 5

struct uvos_mpu_cfg {
  // const struct uvos_exti_cfg * exti_cfg; /* Pointer to the EXTI configuration */
  // uint8_t Fifo_store; /* FIFO storage of different readings (See datasheet page 31 for more details) */
  invensense_type_e device_id;
  /* Sample rate divider to use (See datasheet page 32 for more details).*/
  uint8_t Smpl_rate_div_no_dlp; /* used when no dlp is applied (fs=8KHz)*/
  uint8_t Smpl_rate_div_dlp; /* used when dlp is on (fs=1kHz)*/
  // uint8_t interrupt_cfg; /* Interrupt configuration (See datasheet page 35 for more details) */
  // uint8_t interrupt_en; /* Interrupt configuration (See datasheet page 35 for more details) */
  uint8_t User_ctl; /* User control settings (See datasheet page 41 for more details)  */
  uint8_t Pwr_mgmt_clk; /* Power management and clock selection (See datasheet page 32 for more details) */
  enum uvos_mpu_gyro_range gyro_range;
  enum uvos_mpu_accel_range accel_range;
  enum uvos_mpu_filter filter;
  // enum uvos_mpu_orientation orientation;
  SPIPrescalerTypeDef fast_prescaler;
  SPIPrescalerTypeDef std_prescaler;
  uint8_t max_downsample;
};

/* Public Functions */
extern int32_t UVOS_MPU_Init( uint32_t spi_id, uint32_t slave_num, const struct uvos_mpu_cfg * new_cfg );
extern int32_t UVOS_MPU_ConfigureRanges( enum uvos_mpu_gyro_range gyroRange, enum uvos_mpu_accel_range accelRange, enum uvos_mpu_filter filterSetting );
extern int32_t UVOS_MPU_ReadImu( int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz );
extern void UVOS_MPU_SetSpeedFast( const bool fast );

#endif // UVOS_MPU_H
