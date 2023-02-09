#ifndef UVOS_MPU_H
#define UVOS_MPU_H

typedef enum uvos_mpu_invensense_type {
  Invensense_Invalid_ID = 0,
  Invensense_MPU6000,
  Invensense_MPU6500,
  Invensense_MPU9250,
  Invensense_ICM20608,
  Invensense_ICM20602,
  Invensense_ICM20601,
  Invensense_ICM20789,
  Invensense_ICM20689,
  Invensense_ICM40609,
  Invensense_ICM42688,
  Invensense_ICM42605,
  Invensense_ICM40605,
  Invensense_IIM42652,
  Invensense_ICM42670,
  Invensense_ICM45686
} invensense_type_t;

// MPU-6000: ±250, ±500, ±1000, and ±2000 degrees/sec
// ICM-42688-P: ±15.625, ±31.25, ±62.5, ±125, ±250, ±500, ±1000, and ±2000 degrees/sec
typedef enum uvos_mpu_gyro_range {
  Invalid_UVOS_GYRO_RANGE = 0,
  UVOS_GYRO_RANGE_250DPS,
  UVOS_GYRO_RANGE_500DPS,
  UVOS_GYRO_RANGE_1000DPS,
  UVOS_GYRO_RANGE_2000DPS,
} uvos_mpu_gyro_range_t;

// MPU-6000, ICM-42688-P: ±2g, ±4g ±8g and ±16g
typedef enum uvos_mpu_accel_range {
  Invalid_UVOS_ACCEL_RANGE = 0,
  UVOS_ACCEL_RANGE_2G,
  UVOS_ACCEL_RANGE_4G,
  UVOS_ACCEL_RANGE_8G,
  UVOS_ACCEL_RANGE_16G,
} uvos_mpu_accel_range_t;

typedef enum uvos_mpu_spi_speed {
  Invalid_UVOS_MPU_SPI_SPEED = 0,
  UVOS_MPU_SPI_SPEED_SLOW,
  UVOS_MPU_SPI_SPEED_FAST,
} uvos_mpu_spi_speed_t;


typedef struct uvos_mpu_cfg {
  invensense_type_t expected_device_id; /* Device ID expected to be found on board */
  // const struct uvos_exti_cfg *exti_cfg; /* Pointer to the EXTI configuration */
  uint16_t default_samplerate_hz;  /* Sample to use in Hz (See datasheet page 32 for more details) */
  uvos_mpu_gyro_range_t default_gyro_range;
  uvos_mpu_accel_range_t default_accel_range;
  // bool use_magnetometer; /* Use internal or external magnetometer */
  SPIPrescalerTypeDef fast_prescaler;
  SPIPrescalerTypeDef std_prescaler;
} uvos_mpu_cfg_t;

#include "uvos_mpuv1.h"
#include "uvos_mpuv3.h"

#define UVOS_MPU_MAX_TRIES 5

#if 0 // GLS

typedef struct uvos_mpu_cfg {
  // const struct uvos_exti_cfg * exti_cfg; /* Pointer to the EXTI configuration */
  // uint8_t Fifo_store; /* FIFO storage of different readings (See datasheet page 31 for more details) */
  invensense_type_t device_id;
  /* Sample rate divider to use (See datasheet page 32 for more details).*/
  uint8_t Smpl_rate_div_no_dlp; /* used when no dlp is applied (fs=8KHz)*/
  uint8_t Smpl_rate_div_dlp; /* used when dlp is on (fs=1kHz)*/
  // uint8_t interrupt_cfg; /* Interrupt configuration (See datasheet page 35 for more details) */
  // uint8_t interrupt_en; /* Interrupt configuration (See datasheet page 35 for more details) */
  uint8_t User_ctl; /* User control settings (See datasheet page 41 for more details)  */
  uint8_t Pwr_mgmt_clk; /* Power management and clock selection (See datasheet page 32 for more details) */
  uvos_mpu_gyro_range_t gyro_range;
  uvos_mpu_accel_range_t accel_range;
  enum uvos_mpu_filter filter;
  // enum uvos_mpu_orientation orientation;
  SPIPrescalerTypeDef fast_prescaler;
  SPIPrescalerTypeDef std_prescaler;
} uvos_mpu_cfg_t;

#endif // GLS

/* Public Functions */
extern int32_t UVOS_MPU_Init( uint32_t spi_id, uint32_t slave_num, const uvos_mpu_cfg_t *new_cfg );
extern int32_t UVOS_MPU_ReadImu( int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz );
extern int32_t UVOS_MPU_SetSpeed( const uvos_mpu_spi_speed_t spi_speed );
extern int32_t UVOS_MPU_ConfigGyroRange( uvos_mpu_gyro_range_t gyroRange );
extern int32_t UVOS_MPU_ConfigAccelRange( uvos_mpu_accel_range_t accelRange );
extern int32_t UVOS_MPUV1_ConfigFilter( uvos_mpuv1_filter_t filterSetting );
extern int32_t UVOS_MPUV1_ConfigSampleRate( uvos_mpuv1_samplerate_div_t sampleRate );
extern bool UVOS_MPU_data_ready( void );

#endif // UVOS_MPU_H
