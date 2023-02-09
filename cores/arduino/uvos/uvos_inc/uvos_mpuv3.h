#ifndef UVOS_MPUV3_H
#define UVOS_MPUV3_H

/*
  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==0)
 */
static const float GYRO_SCALE_2000DPS = (0.0174532f / 16.4f);
/*
  gyro as 8.2 LSB/DPS at scale factor of +/- 4000dps (FS_SEL==0)
 */
static const float GYRO_SCALE_4000DPS = (0.0174532f / 8.2f);


// set bit 0x80 in register ID for read on SPI
#define BIT_READ_FLAG                           0x80

// registers we use
#define INV3REG_WHOAMI        0x75
#define INV3REG_FIFO_CONFIG   0x16
#define INV3REG_PWR_MGMT0     0x4e
#define INV3REG_GYRO_CONFIG0  0x4f
#define INV3REG_ACCEL_CONFIG0 0x50
#define INV3REG_GYRO_CONFIG1  0x51
#define INV3REG_GYRO_ACCEL_CONFIG0 0x52
#define INV3REG_ACCEL_CONFIG1 0x53
#define INV3REG_FIFO_CONFIG1  0x5f
#define INV3REG_FIFO_CONFIG2  0x60
#define INV3REG_FIFO_CONFIG3  0x61
#define INV3REG_SIGNAL_PATH_RESET 0x4b
#define INV3REG_INTF_CONFIG0  0x4c
#define INV3REG_FIFO_COUNTH   0x2e
#define INV3REG_FIFO_DATA     0x30
#define INV3REG_BANK_SEL      0x76
#define INV3REG_DEVICE_CONFIG 0x11
#define INV3REG_INT_STATUS    0x2D

// ICM42688 bank1
#define INV3REG_GYRO_CONFIG_STATIC2 0x0B
#define INV3REG_GYRO_CONFIG_STATIC3 0x0C
#define INV3REG_GYRO_CONFIG_STATIC4 0x0D
#define INV3REG_GYRO_CONFIG_STATIC5 0x0E

// ICM42688 bank2
#define INV3REG_ACCEL_CONFIG_STATIC2 0x03
#define INV3REG_ACCEL_CONFIG_STATIC3 0x04
#define INV3REG_ACCEL_CONFIG_STATIC4 0x05

// registers for ICM-42670, multi-bank
#define INV3REG_70_PWR_MGMT0  0x1F
#define INV3REG_70_GYRO_CONFIG0  0x20
#define INV3REG_70_GYRO_CONFIG1  0x23
#define INV3REG_70_ACCEL_CONFIG0 0x21
#define INV3REG_70_ACCEL_CONFIG1 0x24
#define INV3REG_70_FIFO_COUNTH   0x3D
#define INV3REG_70_FIFO_DATA     0x3F
#define INV3REG_70_INTF_CONFIG0  0x35
#define INV3REG_70_MCLK_RDY      0x00
#define INV3REG_70_SIGNAL_PATH_RESET 0x02
#define INV3REG_70_FIFO_CONFIG1  0x28
#define INV3REG_BLK_SEL_W 0x79
#define INV3REG_BLK_SEL_R 0x7C
#define INV3REG_MADDR_W   0x7A
#define INV3REG_MADDR_R   0x7D
#define INV3REG_M_W       0x7B
#define INV3REG_M_R       0x7E
#define INV3REG_BANK_MREG1 0x00
#define INV3REG_BANK_MREG2 0x28
#define INV3REG_BANK_MREG3 0x50

#define INV3REG_MREG1_FIFO_CONFIG5 0x1
#define INV3REG_MREG1_SENSOR_CONFIG3 0x06

#define INV3REG_456_WHOAMI          0x72
#define INV3REG_456_PWR_MGMT0       0x10
#define INV3REG_456_ACCEL_CONFIG0   0x1B
#define INV3REG_456_GYRO_CONFIG0    0x1C
#define INV3REG_456_FIFO_CONFIG0    0x1D
#define INV3REG_456_FIFO_CONFIG2    0x20
#define INV3REG_456_FIFO_CONFIG3    0x21
#define INV3REG_456_FIFO_CONFIG4    0x22
#define INV3REG_456_RTC_CONFIG      0x26
#define INV3REG_456_FIFO_COUNTH     0x12
#define INV3REG_456_FIFO_COUNTL     0x13
#define INV3REG_456_FIFO_DATA       0x14
#define INV3REG_456_INTF_CONFIG0    0x2C
#define INV3REG_456_IOC_PAD_SCENARIO 0x2F
#define INV3REG_456_IOC_PAD_SCENARIO_AUX_OVRD 0x30
#define INV3REG_456_IOC_PAD_SCENARIO_OVRD 0x31
#define INV3REG_456_IREG_ADDRH      0x7C
#define INV3REG_456_IREG_ADDRL      0x7D
#define INV3REG_456_IREG_DATA       0x7E
#define INV3REG_456_REG_MISC2       0x7F

#define INV3BANK_456_IMEM_SRAM_ADDR 0x0000
#define INV3BANK_456_IPREG_BAR_ADDR 0xA000
#define INV3BANK_456_IPREG_TOP1_ADDR 0xA200
#define INV3BANK_456_IPREG_SYS1_ADDR 0xA400
#define INV3BANK_456_IPREG_SYS2_ADDR 0xA500

// WHOAMI values
#define INV3_ID_ICM40605      0x33
#define INV3_ID_ICM40609      0x3b
#define INV3_ID_ICM42605      0x42
#define INV3_ID_ICM42688      0x47
#define INV3_ID_IIM42652      0x6f
#define INV3_ID_ICM42670      0x67
#define INV3_ID_ICM45686      0xE9

#endif // UVOS_MPUV3_H
