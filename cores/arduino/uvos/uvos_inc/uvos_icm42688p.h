#ifndef UVOS_ICM42688P_H
#define UVOS_ICM42688P_H
#include <uvos_sensors.h>

#include "uvos_icm_regs.h"

#if 0 // gls

/* ICM42688P Addresses */
#define UVOS_ICM42688P_SMPLRT_DIV_REG           0X19
#define UVOS_ICM42688P_DLPF_CFG_REG             0X1A
#define UVOS_ICM42688P_GYRO_CFG_REG             0X1B
#define UVOS_ICM42688P_ACCEL_CFG_REG            0X1C
#define UVOS_ICM42688P_FIFO_EN_REG              0x23
#define UVOS_ICM42688P_INT_CFG_REG              0x37
#define UVOS_ICM42688P_INT_EN_REG               0x38
#define UVOS_ICM42688P_INT_STATUS_REG           0x3A
#define UVOS_ICM42688P_ACCEL_X_OUT_MSB          0x3B
#define UVOS_ICM42688P_ACCEL_X_OUT_LSB          0x3C
#define UVOS_ICM42688P_ACCEL_Y_OUT_MSB          0x3D
#define UVOS_ICM42688P_ACCEL_Y_OUT_LSB          0x3E
#define UVOS_ICM42688P_ACCEL_Z_OUT_MSB          0x3F
#define UVOS_ICM42688P_ACCEL_Z_OUT_LSB          0x40
#define UVOS_ICM42688P_TEMP_OUT_MSB             0x41
#define UVOS_ICM42688P_TEMP_OUT_LSB             0x42
#define UVOS_ICM42688P_GYRO_X_OUT_MSB           0x43
#define UVOS_ICM42688P_GYRO_X_OUT_LSB           0x44
#define UVOS_ICM42688P_GYRO_Y_OUT_MSB           0x45
#define UVOS_ICM42688P_GYRO_Y_OUT_LSB           0x46
#define UVOS_ICM42688P_GYRO_Z_OUT_MSB           0x47
#define UVOS_ICM42688P_GYRO_Z_OUT_LSB           0x48
#define UVOS_ICM42688P_USER_CTRL_REG            0x6A
#define UVOS_ICM42688P_PWR_MGMT_REG             0x6B
#define UVOS_ICM42688P_FIFO_CNT_MSB             0x72
#define UVOS_ICM42688P_FIFO_CNT_LSB             0x73
#define UVOS_ICM42688P_FIFO_REG                 0x74
#define UVOS_ICM42688P_WHOAMI                   0x75

/* FIFO enable for storing different values */
#define UVOS_ICM42688P_FIFO_TEMP_OUT            0x80
#define UVOS_ICM42688P_FIFO_GYRO_X_OUT          0x40
#define UVOS_ICM42688P_FIFO_GYRO_Y_OUT          0x20
#define UVOS_ICM42688P_FIFO_GYRO_Z_OUT          0x10
#define UVOS_ICM42688P_ACCEL_OUT                0x08

/* Interrupt Configuration */
#define UVOS_ICM42688P_INT_ACTL                 0x80
#define UVOS_ICM42688P_INT_OPEN                 0x40
#define UVOS_ICM42688P_INT_LATCH_EN             0x20
#define UVOS_ICM42688P_INT_CLR_ANYRD            0x10

#define UVOS_ICM42688P_INTEN_OVERFLOW           0x10
#define UVOS_ICM42688P_INTEN_DATA_RDY           0x01

/* Interrupt status */
#define UVOS_ICM42688P_INT_STATUS_FIFO_FULL     0x80
#define UVOS_ICM42688P_INT_STATUS_FIFO_OVERFLOW 0x10
#define UVOS_ICM42688P_INT_STATUS_IMU_RDY       0X04
#define UVOS_ICM42688P_INT_STATUS_DATA_RDY      0X01

/* User control functionality */
#define UVOS_ICM42688P_USERCTL_FIFO_EN          0X40
#define UVOS_ICM42688P_USERCTL_I2C_MST_EN       0x20
#define UVOS_ICM42688P_USERCTL_DIS_I2C          0X10
#define UVOS_ICM42688P_USERCTL_FIFO_RST         0X04
#define UVOS_ICM42688P_USERCTL_SIG_COND         0X02
#define UVOS_ICM42688P_USERCTL_GYRO_RST         0X01

/* Power management and clock selection */
#define UVOS_ICM42688P_PWRMGMT_IMU_RST          0X80
#define UVOS_ICM42688P_PWRMGMT_INTERN_CLK       0X00
#define UVOS_ICM42688P_PWRMGMT_PLL_X_CLK        0X01
#define UVOS_ICM42688P_PWRMGMT_PLL_Y_CLK        0X02
#define UVOS_ICM42688P_PWRMGMT_PLL_Z_CLK        0X03
#define UVOS_ICM42688P_PWRMGMT_STOP_CLK         0X07

#endif // gls

enum uvos_icm42688p_range {
  UVOS_ICM42688P_SCALE_250_DEG  = 0x00,
  UVOS_ICM42688P_SCALE_500_DEG  = 0x08,
  UVOS_ICM42688P_SCALE_1000_DEG = 0x10,
  UVOS_ICM42688P_SCALE_2000_DEG = 0x18
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

enum uvos_icm42688p_accel_range {
  UVOS_ICM42688P_ACCEL_2G  = 0x00,
  UVOS_ICM42688P_ACCEL_4G  = 0x08,
  UVOS_ICM42688P_ACCEL_8G  = 0x10,
  UVOS_ICM42688P_ACCEL_16G = 0x18
};

enum uvos_icm42688p_orientation { // clockwise rotation from board forward
  UVOS_ICM42688P_TOP_0DEG   = 0x00,
  UVOS_ICM42688P_TOP_90DEG  = 0x01,
  UVOS_ICM42688P_TOP_180DEG = 0x02,
  UVOS_ICM42688P_TOP_270DEG = 0x03
};

struct uvos_icm42688p_cfg {
  const struct uvos_exti_cfg *exti_cfg; /* Pointer to the EXTI configuration */

  uint8_t Fifo_store; /* FIFO storage of different readings (See datasheet page 31 for more details) */

  /* Sample rate divider to use (See datasheet page 32 for more details).*/
  uint8_t Smpl_rate_div_no_dlp; /* used when no dlp is applied (fs=8KHz)*/
  uint8_t Smpl_rate_div_dlp; /* used when dlp is on (fs=1kHz)*/
  uint8_t interrupt_cfg; /* Interrupt configuration (See datasheet page 35 for more details) */
  uint8_t interrupt_en; /* Interrupt configuration (See datasheet page 35 for more details) */
  uint8_t User_ctl; /* User control settings (See datasheet page 41 for more details)  */
  uint8_t Pwr_mgmt_clk; /* Power management and clock selection (See datasheet page 32 for more details) */
  enum uvos_icm42688p_accel_range accel_range;
  enum uvos_icm42688p_range gyro_range;
  enum uvos_icm42688p_filter filter;
  enum uvos_icm42688p_orientation orientation;
  SPIPrescalerTypeDef fast_prescaler;
  SPIPrescalerTypeDef std_prescaler;
  uint8_t max_downsample;
};

/* Public Functions */
extern int32_t UVOS_ICM42688P_Init( uint32_t spi_id, uint32_t slave_num, const struct uvos_icm42688p_cfg *new_cfg );
extern int32_t UVOS_ICM42688P_ConfigureRanges( enum uvos_icm42688p_range gyroRange, enum uvos_icm42688p_accel_range accelRange, enum uvos_icm42688p_filter filterSetting );
extern int32_t UVOS_ICM42688P_ReadID();
extern void UVOS_ICM42688P_Register();
extern bool UVOS_ICM42688P_IRQHandler( void );

extern const UVOS_SENSORS_Driver UVOS_ICM42688P_Driver;
#endif /* UVOS_ICM42688P_H */

/**
 * @}
 * @}
 */
