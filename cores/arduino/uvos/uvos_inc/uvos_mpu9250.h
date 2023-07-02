#ifndef UVOS_MPU9250_H
#define UVOS_MPU9250_H
#include <uvos_sensors.h>

/* MPU9250 Addresses */
#define UVOS_MPU9250_SMPLRT_DIV_REG           0X19
#define UVOS_MPU9250_DLPF_CFG_REG             0X1A
#define UVOS_MPU9250_GYRO_CFG_REG             0X1B
#define UVOS_MPU9250_ACCEL_CFG_REG            0X1C
#define UVOS_MPU9250_FIFO_EN_REG              0x23
#define UVOS_MPU9250_I2C_MST_CTRL             0x24
#define UVOS_MPU9250_I2C_SLV0_ADDR            0x25
#define UVOS_MPU9250_I2C_SLV0_REG             0x26
#define UVOS_MPU9250_I2C_SLV0_CTRL            0x27
#define UVOS_MPU9250_I2C_SLV1_ADDR            0x28
#define UVOS_MPU9250_I2C_SLV1_REG             0x29
#define UVOS_MPU9250_I2C_SLV1_CTRL            0x2A
#define UVOS_MPU9250_I2C_SLV4_ADDR            0x31
#define UVOS_MPU9250_I2C_SLV4_REG             0x32
#define UVOS_MPU9250_I2C_SLV4_DO              0x33
#define UVOS_MPU9250_I2C_SLV4_CTRL            0x34
#define UVOS_MPU9250_I2C_SLV4_DI              0x35
#define UVOS_MPU9250_INT_CFG_REG              0x37
#define UVOS_MPU9250_INT_EN_REG               0x38
#define UVOS_MPU9250_INT_STATUS_REG           0x3A
#define UVOS_MPU9250_ACCEL_X_OUT_MSB          0x3B
#define UVOS_MPU9250_ACCEL_X_OUT_LSB          0x3C
#define UVOS_MPU9250_ACCEL_Y_OUT_MSB          0x3D
#define UVOS_MPU9250_ACCEL_Y_OUT_LSB          0x3E
#define UVOS_MPU9250_ACCEL_Z_OUT_MSB          0x3F
#define UVOS_MPU9250_ACCEL_Z_OUT_LSB          0x40
#define UVOS_MPU9250_TEMP_OUT_MSB             0x41
#define UVOS_MPU9250_TEMP_OUT_LSB             0x42
#define UVOS_MPU9250_GYRO_X_OUT_MSB           0x43
#define UVOS_MPU9250_GYRO_X_OUT_LSB           0x44
#define UVOS_MPU9250_GYRO_Y_OUT_MSB           0x45
#define UVOS_MPU9250_GYRO_Y_OUT_LSB           0x46
#define UVOS_MPU9250_GYRO_Z_OUT_MSB           0x47
#define UVOS_MPU9250_GYRO_Z_OUT_LSB           0x48
#define UVOS_MPU9250_EXT_SENS_DATA_00         0x49
#define UVOS_MPU9250_I2C_SLV0_DO              0x63
#define UVOS_MPU9250_I2C_SLV1_DO              0x64
#define UVOS_MPU9250_I2C_MST_DELAY_CTRL       0x67
#define UVOS_MPU9250_USER_CTRL_REG            0x6A
#define UVOS_MPU9250_PWR_MGMT_REG             0x6B
#define UVOS_MPU9250_PWR_MGMT2_REG            0x6C
#define UVOS_MPU9250_FIFO_CNT_MSB             0x72
#define UVOS_MPU9250_FIFO_CNT_LSB             0x73
#define UVOS_MPU9250_FIFO_REG                 0x74
#define UVOS_MPU9250_WHOAMI                   0x75

/* FIFO enable for storing different values */
#define UVOS_MPU9250_FIFO_TEMP_OUT            0x80
#define UVOS_MPU9250_FIFO_GYRO_X_OUT          0x40
#define UVOS_MPU9250_FIFO_GYRO_Y_OUT          0x20
#define UVOS_MPU9250_FIFO_GYRO_Z_OUT          0x10
#define UVOS_MPU9250_ACCEL_OUT                0x08

/* Interrupt Configuration */
#define UVOS_MPU9250_INT_ACTL                 0x80
#define UVOS_MPU9250_INT_OPEN                 0x40
#define UVOS_MPU9250_INT_LATCH_EN             0x20
#define UVOS_MPU9250_INT_CLR_ANYRD            0x10

#define UVOS_MPU9250_INTEN_OVERFLOW           0x10
#define UVOS_MPU9250_INTEN_DATA_RDY           0x01

/* Interrupt status */
#define UVOS_MPU9250_INT_STATUS_FIFO_FULL     0x80
#define UVOS_MPU9250_INT_STATUS_FIFO_OVERFLOW 0x10
#define UVOS_MPU9250_INT_STATUS_IMU_RDY       0X04
#define UVOS_MPU9250_INT_STATUS_DATA_RDY      0X01

/* User control functionality */
#define UVOS_MPU9250_USERCTL_FIFO_EN          0X40
#define UVOS_MPU9250_USERCTL_I2C_MST_EN       0x20
#define UVOS_MPU9250_USERCTL_DIS_I2C          0X10
#define UVOS_MPU9250_USERCTL_FIFO_RST         0X04
#define UVOS_MPU9250_USERCTL_SIG_COND         0X01
#define UVOS_MPU9250_USERCTL_I2C_MST_RST      0X02

/* Power management and clock selection */
#define UVOS_MPU9250_PWRMGMT_IMU_RST          0X80
#define UVOS_MPU9250_PWRMGMT_INTERN_CLK       0X00
#define UVOS_MPU9250_PWRMGMT_PLL_X_CLK        0X01
#define UVOS_MPU9250_PWRMGMT_PLL_Y_CLK        0X02
#define UVOS_MPU9250_PWRMGMT_PLL_Z_CLK        0X03
#define UVOS_MPU9250_PWRMGMT_STOP_CLK         0X07

#define UVOS_MPU9250_PWRMGMT2_DISABLE_GYRO    0x07
#define UVOS_MPU9250_PWRMGMT2_DISABLE_ACCEL   0x38

/* I2C */
#define UVOS_MPU9250_I2C_MST_ENABLE           0x80
#define UVOS_MPU9250_I2C_SLV_ENABLE           0x80
#define UVOS_MPU9250_I2C_MST_CLOCK_400        0x0D
#define UVOS_MPU9250_I2C_MST_P_NSR            0x10
#define UVOS_MPU9250_EXT0_OUT                 0x01

/* AK893 MAG registers */
/* Read-only Register */
#define UVOS_MPU9250_WIA                      0X00
#define UVOS_MPU9250_INFO                     0X01
#define UVOS_MPU9250_ST1                      0X02
#define UVOS_MPU9250_HXL                      0X03
#define UVOS_MPU9250_HXH                      0X04
#define UVOS_MPU9250_HYL                      0X05
#define UVOS_MPU9250_HYH                      0X06
#define UVOS_MPU9250_HZL                      0X07
#define UVOS_MPU9250_HZH                      0X08
#define UVOS_MPU9250_ST2                      0X09
/* Write/read Register */
#define UVOS_MPU9250_CNTL1                    0X0A
#define UVOS_MPU9250_CNTL2                    0X0B
#define UVOS_MPU9250_ASTC                     0X0C
#define UVOS_MPU9250_TS1                      0X0D
#define UVOS_MPU9250_TS2                      0X0E
#define UVOS_MPU9250_I2CDIS                   0X0F
/* Read-only Register */
#define UVOS_MPU9250_ASAX                     0X10
#define UVOS_MPU9250_ASAY                     0X11
#define UVOS_MPU9250_ASAZ                     0X12

/* IDs */
#define UVOS_MPU9250_GYRO_ACC_ID              0x71
#define UVOS_MPU9250_MAG_ID                   0x48

#define UVOS_MPU9250_MAG_DATA_RDY             0x01
#define UVOS_MPU9250_MAG_RESET                0x01
#define UVOS_MPU9250_MAG_POWER_DOWN_MODE      0x00
#define UVOS_MPU9250_MAG_SINGLE_MODE          0x01
#define UVOS_MPU9250_MAG_CONTINUOUS_MODE1     0x02
#define UVOS_MPU9250_MAG_CONTINUOUS_MODE2     0x06
#define UVOS_MPU9250_MAG_FUSE_ROM_MODE        0x0F
#define UVOS_MPU9250_MAG_OUTPUT_16BITS        0x10
#define UVOS_MPU9250_MAG_I2C_ADDR             0x0C
#define UVOS_MPU9250_MAG_I2C_READ_FLAG        0x80
#define UVOS_MPU9250_MAG_ASA_NB_BYTE          3
#define UVOS_MPU9250_MAG_ASAX_IDX             1
#define UVOS_MPU9250_MAG_ASAY_IDX             2
#define UVOS_MPU9250_MAG_ASAZ_IDX             3

#define UVOS_MPU9250_MAG_OK                   0
#define UVOS_MPU9250_ERR_MAG_SET_ADDR         -1
#define UVOS_MPU9250_ERR_MAG_SET_REG          -2
#define UVOS_MPU9250_ERR_MAG_SET_DO           -3
#define UVOS_MPU9250_ERR_MAG_SET_TRIGGER      -4
#define UVOS_MPU9250_ERR_MAG_READ_ID          -5
#define UVOS_MPU9250_ERR_MAG_BAD_ID           -6
#define UVOS_MPU9250_ERR_MAG_READ_ASA         -7

#define UVOS_MPU9250_TEMP_OFFSET              0
#define UVOS_MPU9250_TEMP_SENSITIVITY         321.0f
enum uvos_mpu9250_range {
  UVOS_MPU9250_SCALE_250_DEG  = 0x00,
  UVOS_MPU9250_SCALE_500_DEG  = 0x08,
  UVOS_MPU9250_SCALE_1000_DEG = 0x10,
  UVOS_MPU9250_SCALE_2000_DEG = 0x18
};

enum uvos_mpu9250_filter {
  UVOS_MPU9250_LOWPASS_256_HZ = 0x00,
  UVOS_MPU9250_LOWPASS_188_HZ = 0x01,
  UVOS_MPU9250_LOWPASS_98_HZ  = 0x02,
  UVOS_MPU9250_LOWPASS_42_HZ  = 0x03,
  UVOS_MPU9250_LOWPASS_20_HZ  = 0x04,
  UVOS_MPU9250_LOWPASS_10_HZ  = 0x05,
  UVOS_MPU9250_LOWPASS_5_HZ   = 0x06
};

enum uvos_mpu9250_accel_range {
  UVOS_MPU9250_ACCEL_2G  = 0x00,
  UVOS_MPU9250_ACCEL_4G  = 0x08,
  UVOS_MPU9250_ACCEL_8G  = 0x10,
  UVOS_MPU9250_ACCEL_16G = 0x18
};

enum uvos_mpu9250_orientation { // clockwise rotation from board forward
  UVOS_MPU9250_TOP_0DEG   = 0x00,
  UVOS_MPU9250_TOP_90DEG  = 0x01,
  UVOS_MPU9250_TOP_180DEG = 0x02,
  UVOS_MPU9250_TOP_270DEG = 0x03
};

struct uvos_mpu9250_cfg {
  const struct uvos_exti_cfg *exti_cfg; /* Pointer to the EXTI configuration */

  uint8_t Fifo_store; /* FIFO storage of different readings (See datasheet page 31 for more details) */

  /* Sample rate divider to use (See datasheet page 32 for more details).*/
  uint8_t Smpl_rate_div_no_dlp; /* used when no dlp is applied (fs=8KHz)*/
  uint8_t Smpl_rate_div_dlp; /* used when dlp is on (fs=1kHz)*/
  uint8_t interrupt_cfg; /* Interrupt configuration (See datasheet page 35 for more details) */
  uint8_t interrupt_en; /* Interrupt configuration (See datasheet page 35 for more details) */
  uint8_t User_ctl; /* User control settings (See datasheet page 41 for more details)  */
  uint8_t Pwr_mgmt_clk; /* Power management and clock selection (See datasheet page 32 for more details) */
  enum uvos_mpu9250_accel_range accel_range;
  enum uvos_mpu9250_range gyro_range;
  enum uvos_mpu9250_filter filter;
  enum uvos_mpu9250_orientation orientation;
  SPIPrescalerTypeDef fast_prescaler;
  SPIPrescalerTypeDef std_prescaler;
  uint8_t max_downsample;
};

/* Public Functions */
extern int32_t UVOS_MPU9250_Init( uint32_t spi_id, uint32_t slave_num, const struct uvos_mpu9250_cfg *new_cfg );
extern int32_t UVOS_MPU9250_ConfigureRanges( enum uvos_mpu9250_range gyroRange, enum uvos_mpu9250_accel_range accelRange, enum uvos_mpu9250_filter filterSetting );
extern int32_t UVOS_MPU9250_ReadID();
extern void UVOS_MPU9250_MainRegister();
extern void UVOS_MPU9250_MagRegister();

extern bool UVOS_MPU9250_IRQHandler( void );

extern const UVOS_SENSORS_Driver UVOS_MPU9250_Main_Driver;
extern const UVOS_SENSORS_Driver UVOS_MPU9250_Mag_Driver;

#endif /* UVOS_MPU9250_H */

/**
 * @}
 * @}
 */
