#ifndef UVOS_ICM42688P_H
#define UVOS_ICM42688P_H
#include <uvos_sensors.h>

// #include "uvos_icm_regs.h"

#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

#define ICM426XX_DEVICE_CONFIG                      0x11

#define ICM426XX_RA_PWR_MGMT0                       0x4E  // User Bank 0
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E  // User Bank 1
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05  // User Bank 2
// --- Register & setting for gyro and acc UI Filter --------
#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52  // User Bank 0
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)
// ----------------------------------------------------------

#define ICM426XX_RA_TEMP_DATA1                      0x1D  // User Bank 0
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F  // User Bank 0
#define ICM426XX_RA_GYRO_DATA_X1                    0x25  // User Bank 0

#define ICM426XX_RA_INT_CONFIG                      0x14  // User Bank 0
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63  // User Bank 0
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_STATUS_BIT_READ  ((0 << 5) || (0 << 4)) // Clear on Status Bit Read (default)
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_STATUS_BIT_READ2 ((0 << 5) || (0 << 4)) // Duplicate settings in datasheet, Rev 1.7
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_REG_READ         ((1 << 5) || (0 << 4)) // Clear on Sensor Register Read
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_STATUS_BIT_READ_AND_REG_READ ((1 << 5) || (1 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64   // User Bank 0
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)

#define ICM426XX_RA_INT_SOURCE0                     0x65  // User Bank 0
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

#define ICM426XX_WHO_AM_I                           0x75  // User Bank 0


typedef enum {
  ODR_CONFIG_8K = 0,
  ODR_CONFIG_4K,
  ODR_CONFIG_2K,
  ODR_CONFIG_1K,
  ODR_CONFIG_COUNT
} odrConfig_e;

enum uvos_icm42688p_filter {
  UVOS_ICM42688P_LOWPASS_42_HZ = 0,
  UVOS_ICM42688P_LOWPASS_84_HZ,
  UVOS_ICM42688P_LOWPASS_170_HZ,
  UVOS_ICM42688P_LOWPASS_258_HZ,
  UVOS_ICM42688P_LOWPASS_536_HZ,
  UVOS_ICM42688P_LOWPASS_997_HZ,
  UVOS_ICM42688P_LOWPASS_1962_HZ,
  UVOS_ICM42688P_LOWPASS_COUNT,
};

typedef struct aafConfig_s {
  uint8_t delt;
  uint16_t deltSqr;
  uint8_t bitshift;
} aafConfig_t;

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42688[ UVOS_ICM42688P_LOWPASS_COUNT ] = {  // see table in section 5.3
  [ UVOS_ICM42688P_LOWPASS_42_HZ ]   = {  1,    1, 15 },
  [ UVOS_ICM42688P_LOWPASS_84_HZ ]   = {  2,    4, 13 },
  [ UVOS_ICM42688P_LOWPASS_170_HZ ]  = {  4,   16, 11 },
  [ UVOS_ICM42688P_LOWPASS_258_HZ ]  = {  6,   36, 10 },
  [ UVOS_ICM42688P_LOWPASS_536_HZ ]  = { 12,  144,  8 },
  [ UVOS_ICM42688P_LOWPASS_997_HZ ]  = { 21,  440,  6 },
  [ UVOS_ICM42688P_LOWPASS_1962_HZ ] = { 37, 1376,  4 },
};

enum uvos_icm42688p_gyro_range {
  UVOS_ICM42688P_SCALE_2000_DEG = 0x00,
  UVOS_ICM42688P_SCALE_1000_DEG = 0x01,
  UVOS_ICM42688P_SCALE_500_DEG  = 0x02,
  UVOS_ICM42688P_SCALE_250_DEG  = 0x03,
};

enum uvos_icm42688p_accel_range {
  UVOS_ICM42688P_ACCEL_16G = 0x00,
  UVOS_ICM42688P_ACCEL_8G  = 0x01,
  UVOS_ICM42688P_ACCEL_4G  = 0x02,
  UVOS_ICM42688P_ACCEL_2G  = 0x03,
};

enum uvos_icm42688p_odr {
  UVOS_ICM42688P_ODR32k = 0x01, // LN mode only
  UVOS_ICM42688P_ODR16k = 0x02, // LN mode only
  UVOS_ICM42688P_ODR8k = 0x03, // LN mode only
  UVOS_ICM42688P_ODR4k = 0x04, // LN mode only
  UVOS_ICM42688P_ODR2k = 0x05, // LN mode only
  UVOS_ICM42688P_ODR1k = 0x06, // LN mode only
  // UVOS_ICM42688P_ODR200 = 0x07,
  // UVOS_ICM42688P_ODR100 = 0x08,
  // UVOS_ICM42688P_ODR50 = 0x09,
  // UVOS_ICM42688P_ODR25 = 0x0A,
  // UVOS_ICM42688P_ODR12_5 = 0x0B,
  // UVOS_ICM42688P_ODR6a25 = 0x0C, // LP mode only (accel only)
  // UVOS_ICM42688P_ODR3a125 = 0x0D, // LP mode only (accel only)
  // UVOS_ICM42688P_ODR1a5625 = 0x0E, // LP mode only (accel only)
  // UVOS_ICM42688P_ODR500 = 0x0F,
};

// Icm426xx Accelerometer start-up time before having correct data
#define ICM42688P_ACCEL_STARTUP_TIME_US 20000U

// Icm42688p Gyroscope start-up time before having correct data
#define ICM42688P_GYRO_STARTUP_TIME_US 60000U

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
  enum uvos_icm42688p_filter gyro_filter;
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
