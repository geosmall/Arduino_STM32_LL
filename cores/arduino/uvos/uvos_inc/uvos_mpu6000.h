#ifndef UVOS_MPU6000_H
#define UVOS_MPU6000_H
#include <uvos_sensors.h>

#include "uvos_mpu_regs.h"

/* MPU6000 Addresses */
#define UVOS_MPU6000_SMPLRT_DIV_REG           0X19
#define UVOS_MPU6000_DLPF_CFG_REG             0X1A
#define UVOS_MPU6000_GYRO_CFG_REG             0X1B
#define UVOS_MPU6000_ACCEL_CFG_REG            0X1C
#define UVOS_MPU6000_FIFO_EN_REG              0x23
#define UVOS_MPU6000_INT_CFG_REG              0x37
#define UVOS_MPU6000_INT_EN_REG               0x38
#define UVOS_MPU6000_INT_STATUS_REG           0x3A
#define UVOS_MPU6000_ACCEL_X_OUT_MSB          0x3B
#define UVOS_MPU6000_ACCEL_X_OUT_LSB          0x3C
#define UVOS_MPU6000_ACCEL_Y_OUT_MSB          0x3D
#define UVOS_MPU6000_ACCEL_Y_OUT_LSB          0x3E
#define UVOS_MPU6000_ACCEL_Z_OUT_MSB          0x3F
#define UVOS_MPU6000_ACCEL_Z_OUT_LSB          0x40
#define UVOS_MPU6000_TEMP_OUT_MSB             0x41
#define UVOS_MPU6000_TEMP_OUT_LSB             0x42
#define UVOS_MPU6000_GYRO_X_OUT_MSB           0x43
#define UVOS_MPU6000_GYRO_X_OUT_LSB           0x44
#define UVOS_MPU6000_GYRO_Y_OUT_MSB           0x45
#define UVOS_MPU6000_GYRO_Y_OUT_LSB           0x46
#define UVOS_MPU6000_GYRO_Z_OUT_MSB           0x47
#define UVOS_MPU6000_GYRO_Z_OUT_LSB           0x48
#define UVOS_MPU6000_USER_CTRL_REG            0x6A
#define UVOS_MPU6000_PWR_MGMT_REG             0x6B
#define UVOS_MPU6000_FIFO_CNT_MSB             0x72
#define UVOS_MPU6000_FIFO_CNT_LSB             0x73
#define UVOS_MPU6000_FIFO_REG                 0x74
#define UVOS_MPU6000_WHOAMI                   0x75

/* FIFO enable for storing different values */
#define UVOS_MPU6000_FIFO_TEMP_OUT            0x80
#define UVOS_MPU6000_FIFO_GYRO_X_OUT          0x40
#define UVOS_MPU6000_FIFO_GYRO_Y_OUT          0x20
#define UVOS_MPU6000_FIFO_GYRO_Z_OUT          0x10
#define UVOS_MPU6000_ACCEL_OUT                0x08

/* Interrupt Configuration */
#define UVOS_MPU6000_INT_ACTL                 0x80
#define UVOS_MPU6000_INT_OPEN                 0x40
#define UVOS_MPU6000_INT_LATCH_EN             0x20
#define UVOS_MPU6000_INT_CLR_ANYRD            0x10

#define UVOS_MPU6000_INTEN_OVERFLOW           0x10
#define UVOS_MPU6000_INTEN_DATA_RDY           0x01

/* Interrupt status */
#define UVOS_MPU6000_INT_STATUS_FIFO_FULL     0x80
#define UVOS_MPU6000_INT_STATUS_FIFO_OVERFLOW 0x10
#define UVOS_MPU6000_INT_STATUS_IMU_RDY       0X04
#define UVOS_MPU6000_INT_STATUS_DATA_RDY      0X01

/* User control functionality */
#define UVOS_MPU6000_USERCTL_FIFO_EN          0X40
#define UVOS_MPU6000_USERCTL_I2C_MST_EN       0x20
#define UVOS_MPU6000_USERCTL_DIS_I2C          0X10
#define UVOS_MPU6000_USERCTL_FIFO_RST         0X04
#define UVOS_MPU6000_USERCTL_SIG_COND         0X02
#define UVOS_MPU6000_USERCTL_GYRO_RST         0X01

/* Power management and clock selection */
#define UVOS_MPU6000_PWRMGMT_IMU_RST          0X80
#define UVOS_MPU6000_PWRMGMT_INTERN_CLK       0X00
#define UVOS_MPU6000_PWRMGMT_PLL_X_CLK        0X01
#define UVOS_MPU6000_PWRMGMT_PLL_Y_CLK        0X02
#define UVOS_MPU6000_PWRMGMT_PLL_Z_CLK        0X03
#define UVOS_MPU6000_PWRMGMT_STOP_CLK         0X07

enum uvos_mpu6000_range {
  UVOS_MPU6000_SCALE_250_DEG  = 0x00,
  UVOS_MPU6000_SCALE_500_DEG  = 0x08,
  UVOS_MPU6000_SCALE_1000_DEG = 0x10,
  UVOS_MPU6000_SCALE_2000_DEG = 0x18
};

enum uvos_mpu6000_filter {
  UVOS_MPU6000_LOWPASS_256_HZ = 0x00,
  UVOS_MPU6000_LOWPASS_188_HZ = 0x01,
  UVOS_MPU6000_LOWPASS_98_HZ  = 0x02,
  UVOS_MPU6000_LOWPASS_42_HZ  = 0x03,
  UVOS_MPU6000_LOWPASS_20_HZ  = 0x04,
  UVOS_MPU6000_LOWPASS_10_HZ  = 0x05,
  UVOS_MPU6000_LOWPASS_5_HZ   = 0x06
};

enum uvos_mpu6000_accel_range {
  UVOS_MPU6000_ACCEL_2G  = 0x00,
  UVOS_MPU6000_ACCEL_4G  = 0x08,
  UVOS_MPU6000_ACCEL_8G  = 0x10,
  UVOS_MPU6000_ACCEL_16G = 0x18
};

enum uvos_mpu6000_orientation { // clockwise rotation from board forward
  UVOS_MPU6000_TOP_0DEG   = 0x00,
  UVOS_MPU6000_TOP_90DEG  = 0x01,
  UVOS_MPU6000_TOP_180DEG = 0x02,
  UVOS_MPU6000_TOP_270DEG = 0x03
};

struct uvos_mpu6000_cfg {
  const struct uvos_exti_cfg *exti_cfg; /* Pointer to the EXTI configuration */

  uint8_t Fifo_store; /* FIFO storage of different readings (See datasheet page 31 for more details) */

  /* Sample rate divider to use (See datasheet page 32 for more details).*/
  uint8_t Smpl_rate_div_no_dlp; /* used when no dlp is applied (fs=8KHz)*/
  uint8_t Smpl_rate_div_dlp; /* used when dlp is on (fs=1kHz)*/
  uint8_t interrupt_cfg; /* Interrupt configuration (See datasheet page 35 for more details) */
  uint8_t interrupt_en; /* Interrupt configuration (See datasheet page 35 for more details) */
  uint8_t User_ctl; /* User control settings (See datasheet page 41 for more details)  */
  uint8_t Pwr_mgmt_clk; /* Power management and clock selection (See datasheet page 32 for more details) */
  enum uvos_mpu6000_accel_range accel_range;
  enum uvos_mpu6000_range gyro_range;
  enum uvos_mpu6000_filter filter;
  enum uvos_mpu6000_orientation orientation;
  SPIPrescalerTypeDef fast_prescaler;
  SPIPrescalerTypeDef std_prescaler;
  uint8_t max_downsample;
};

/* Public Functions */
extern int32_t UVOS_MPU6000_Init( uint32_t spi_id, uint32_t slave_num, const struct uvos_mpu6000_cfg *new_cfg );
extern int32_t UVOS_MPU6000_ConfigureRanges( enum uvos_mpu6000_range gyroRange, enum uvos_mpu6000_accel_range accelRange, enum uvos_mpu6000_filter filterSetting );
extern int32_t UVOS_MPU6000_ReadID();
extern void UVOS_MPU6000_Register();
extern bool UVOS_MPU6000_IRQHandler( void );

extern const UVOS_SENSORS_Driver UVOS_MPU6000_Driver;
#endif /* UVOS_MPU6000_H */

/**
 * @}
 * @}
 */
