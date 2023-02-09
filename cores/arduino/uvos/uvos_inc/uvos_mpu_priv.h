#ifndef UVOS_MPU_PRIV_H
#define UVOS_MPU_PRIV_H

enum uvos_mpu_dev_magic {
	UVOS_MPU_DEV_MAGIC = 0x9da9b3ed,
};

typedef struct mpu_dev {
	enum uvos_mpu_dev_magic magic;

	uint32_t spi_id;
	uint32_t slave_num;

	// QueueHandle_t queue;
	const uvos_mpu_cfg_t *cfg;
	invensense_type_t type;
	uint8_t family;
	uint16_t sample_rate_hz;
	uvos_mpu_gyro_range_t gyro_range;
	uvos_mpu_gyro_range_t accel_range;
	uint8_t mpu_gyro_first_reg;
  uint8_t mpu_accel_first_reg;
} mpu_dev_t;

int32_t UVOS_MPU_SetReg( uint8_t address, uint8_t data );
int32_t UVOS_MPU_SetRegWithVerify( uint8_t address, uint8_t data );
uint32_t UVOS_MPU_SetRegisterBits( uint8_t address, uint8_t mask, uint8_t value );
int32_t UVOS_MPU_GetReg( uint8_t address );
uint32_t UVOS_MPU_GetData( uint8_t address, uint8_t data[], uint8_t size, bool spi_speed );
int32_t UVOS_MPU_ClaimBus( const uvos_mpu_spi_speed_t spi_speed );
int32_t UVOS_MPU_ReleaseBus();

#endif // UVOS_MPU_PRIV_H
