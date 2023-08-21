#ifndef UVOS_ICM42688P_H
#define UVOS_ICM42688P_H
#include <uvos_sensors.h>

#include "uvos_icm_regs.h"

// Icm426xx Accelerometer start-up time before having correct data
#define ICM42688P_ACCEL_STARTUP_TIME_US 20000U

// Icm42688p Gyroscope start-up time before having correct data
#define ICM42688P_GYRO_STARTUP_TIME_US 60000U

enum GyroFS {
  dps2000 = 0x00,
  dps1000 = 0x01,
  dps500 = 0x02,
  dps250 = 0x03,
  // dps125 = 0x04,
  // dps62_5 = 0x05,
  // dps31_25 = 0x06,
  // dps15_625 = 0x07
};

enum uvos_icm42688p_gyro_range {
  UVOS_ICM42688P_SCALE_2000_DEG = 0x00,
  UVOS_ICM42688P_SCALE_1000_DEG = 0x01,
  UVOS_ICM42688P_SCALE_500_DEG  = 0x02,
  UVOS_ICM42688P_SCALE_250_DEG  = 0x03,
  // UVOS_ICM42688P_SCALE_125_DEG  = 0x04,
  // UVOS_ICM42688P_SCALE_62_5_DEG = 0x05,
  // UVOS_ICM42688P_SCALE_31_25_DEG = 0x06,
  // UVOS_ICM42688P_SCALE_15_625_DEG = 0x07
};

enum uvos_icm42688p_filter {
  UVOS_ICM42688P_LOWPASS_256_HZ = 0x00,
  UVOS_ICM42688P_LOWPASS_188_HZ = 0x01,
  UVOS_ICM42688P_LOWPASS_98_HZ  = 0x02,
  UVOS_ICM42688P_LOWPASS_42_HZ  = 0x03,
  UVOS_ICM42688P_LOWPASS_20_HZ  = 0x04,
  UVOS_ICM42688P_LOWPASS_10_HZ  = 0x05,
  UVOS_ICM42688P_LOWPASS_5_HZ   = 0x06
};

enum AccelFS {
  gpm16 = 0x00,
  gpm8 = 0x01,
  gpm4 = 0x02,
  gpm2 = 0x03
};

enum uvos_icm42688p_accel_range {
  UVOS_ICM42688P_ACCEL_16G = 0x00,
  UVOS_ICM42688P_ACCEL_8G  = 0x01,
  UVOS_ICM42688P_ACCEL_4G  = 0x02,
  UVOS_ICM42688P_ACCEL_2G  = 0x03
};

enum uvos_icm42688p_odr {
  UVOS_ICM42688P_ODR32k = 0x01, // LN mode only
  UVOS_ICM42688P_ODR16k = 0x02, // LN mode only
  UVOS_ICM42688P_ODR8k = 0x03, // LN mode only
  UVOS_ICM42688P_ODR4k = 0x04, // LN mode only
  UVOS_ICM42688P_ODR2k = 0x05, // LN mode only
  UVOS_ICM42688P_ODR1k = 0x06, // LN mode only
  UVOS_ICM42688P_ODR200 = 0x07,
  UVOS_ICM42688P_ODR100 = 0x08,
  UVOS_ICM42688P_ODR50 = 0x09,
  UVOS_ICM42688P_ODR25 = 0x0A,
  UVOS_ICM42688P_ODR12_5 = 0x0B,
  UVOS_ICM42688P_ODR6a25 = 0x0C, // LP mode only (accel only)
  UVOS_ICM42688P_ODR3a125 = 0x0D, // LP mode only (accel only)
  UVOS_ICM42688P_ODR1a5625 = 0x0E, // LP mode only (accel only)
  UVOS_ICM42688P_ODR500 = 0x0F,
};

typedef enum {
  AAF_CONFIG_42HZ = 0,
  AAF_CONFIG_84HZ,
  AAF_CONFIG_126HZ,
  AAF_CONFIG_170HZ,
  AAF_CONFIG_213HZ,
  AAF_CONFIG_258HZ,
  AAF_CONFIG_536HZ,
  AAF_CONFIG_997HZ,
  AAF_CONFIG_1962HZ,
  AAF_CONFIG_COUNT
} aafConfig_e;

typedef struct aafConfig_s {
  uint8_t delt;
  uint16_t deltSqr;
  uint8_t bitshift;
} aafConfig_t;

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42688[ AAF_CONFIG_COUNT ] = {  // see table in section 5.3
  [AAF_CONFIG_42HZ]   = {  1,    1, 15 },
  [AAF_CONFIG_84HZ]   = {  2,    4, 13 },
  [AAF_CONFIG_126HZ]  = {  3,    9, 12 },
  [AAF_CONFIG_170HZ]  = {  4,   16, 11 },
  [AAF_CONFIG_213HZ]  = {  5,   25, 10 },
  [AAF_CONFIG_258HZ]  = {  6,   36, 10 },
  [AAF_CONFIG_536HZ]  = { 12,  144,  8 },
  [AAF_CONFIG_997HZ]  = { 21,  440,  6 },
  [AAF_CONFIG_1962HZ] = { 37, 1376,  4 },
};

const uint8_t FIFO_EN = 0x23;
const uint8_t UVOS_ICM42688P_FIFO_TEMP_EN = 0x04;
const uint8_t UVOS_ICM42688P_FIFO_GYRO_EN = 0x02;
const uint8_t UVOS_ICM42688P_FIFO_ACCEL_EN = 0x01;
// const uint8_t FIFO_COUNT = 0x2E;
// const uint8_t FIFO_DATA = 0x30;

// BANK 1
// const uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
const uint8_t GYRO_NF_ENABLE = 0x00;
const uint8_t GYRO_NF_DISABLE = 0x01;
const uint8_t GYRO_AAF_ENABLE = 0x00;
const uint8_t GYRO_AAF_DISABLE = 0x02;

// BANK 2
// const uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
const uint8_t ACCEL_AAF_ENABLE = 0x00;
const uint8_t ACCEL_AAF_DISABLE = 0x01;

enum uvos_icm42688p_orientation { // clockwise rotation from board forward
  UVOS_ICM42688P_TOP_0DEG   = 0x00,
  UVOS_ICM42688P_TOP_90DEG  = 0x01,
  UVOS_ICM42688P_TOP_180DEG = 0x02,
  UVOS_ICM42688P_TOP_270DEG = 0x03
};

struct uvos_icm42688p_cfg {
  const struct uvos_exti_cfg *exti_cfg; /* Pointer to the EXTI configuration */

  uint8_t Fifo_store; /* FIFO storage of different readings (See datasheet page 31 for more details) */

  enum uvos_icm42688p_odr Smpl_rate_odr; /* sample output data rate (datasheet v1.7 section 5.6) */
  uint8_t interrupt_cfg; /* Interrupt configuration (datasheet v1.7 section 7 & 14.3) */
  uint8_t interrupt_en; /* Interrupt configuration (datasheet v1.7 section 7 & 14.51) */
  enum uvos_icm42688p_accel_range accel_range;
  enum uvos_icm42688p_gyro_range gyro_range;
  enum uvos_icm42688p_filter filter;
  enum uvos_icm42688p_orientation orientation;
  SPIPrescalerTypeDef fast_prescaler;
  SPIPrescalerTypeDef std_prescaler;
  uint8_t max_downsample;
};

/* Public Functions */
extern int32_t UVOS_ICM42688P_Init( uint32_t spi_id, uint32_t slave_num, const struct uvos_icm42688p_cfg *new_cfg );
extern int32_t UVOS_ICM42688P_ConfigureRanges( enum uvos_icm42688p_gyro_range gyroRange, enum uvos_icm42688p_accel_range accelRange, enum uvos_icm42688p_filter filterSetting );
extern int32_t UVOS_ICM42688P_ReadID();
extern void UVOS_ICM42688P_Register();
extern bool UVOS_ICM42688P_IRQHandler( void );

extern const UVOS_SENSORS_Driver UVOS_ICM42688P_Driver;
#endif /* UVOS_ICM42688P_H */

/**
 * @}
 * @}
 */
