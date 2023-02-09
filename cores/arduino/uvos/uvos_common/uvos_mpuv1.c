#include <uvos.h>
#include "uvos_mpu_priv.h"


static uint8_t _last_stat_user_ctrl; /* Last status from register user control */

int32_t UVOS_MPUV1_Hardware_Init( mpu_dev_t *dev )
{
	if ( dev == NULL ) {
		return -1;
	}

	// MPU POWER ON
	if ( UVOS_MPU_SetRegWithVerify( MPUREG_PWR_MGMT_1, 0x01 ) ) {
		return -2;
	}
	// MPU Gyro and Accel ON
	if ( UVOS_MPU_SetRegWithVerify( MPUREG_PWR_MGMT_2, 0x00 ) ) {
		return -2;
	}
	UVOS_DELAY_WaituS( 10000 );

	/* Configure MPU SPI bus speed to slow speed for config */
	UVOS_MPU_SetSpeed( UVOS_MPU_SPI_SPEED_SLOW );

	// Chip reset
	uint8_t tries;
	for ( tries = 0; tries < UVOS_MPU_MAX_TRIES; tries++ ) {
		_last_stat_user_ctrl = UVOS_MPU_GetReg( MPUREG_USER_CTRL );

		/* First disable the master I2C to avoid hanging the slaves on the
		 * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
		 * is used */
		if ( _last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN ) {
			_last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
			if ( UVOS_MPU_SetReg( MPUREG_USER_CTRL, _last_stat_user_ctrl ) ) {
				return -3;
			}
			UVOS_DELAY_WaitmS( 10 );
		}

		/* reset device, wait 100mS per datasheet */
		UVOS_MPU_SetReg( MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET );
		UVOS_DELAY_WaitmS( 100 );

		/* reset signal path as recommended in datasheet */
		if ( dev->type == Invensense_MPU6000 || dev->type == Invensense_MPU6500 ) {
			if ( UVOS_MPU_SetReg( MPUREG_SIGNAL_PATH_RESET,
			                      BIT_SIGNAL_PATH_RESET_TEMP_RESET | BIT_SIGNAL_PATH_RESET_ACCEL_RESET | BIT_SIGNAL_PATH_RESET_GYRO_RESET ) ) {
				return -3;
			}
			UVOS_DELAY_WaitmS( 100 );
		}

		/* Disable I2C bus if SPI selected (Recommended in Datasheet to be
		 * done just after the device is reset) */
		_last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
		if ( UVOS_MPU_SetReg( MPUREG_USER_CTRL, _last_stat_user_ctrl ) ) {
			return -3;
		}

		// Wake up device and select GyroZ clock. Note that the  Invensense device
		// starts up in sleep mode, and it can take some time for it to come out of sleep
		UVOS_MPU_SetReg( MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO );
		UVOS_DELAY_WaitmS( 5 );

		// check that device has woken up
		if ( UVOS_MPU_GetReg( MPUREG_PWR_MGMT_1 ) == BIT_PWR_MGMT_1_CLK_ZGYRO ) {
			break;
		}

		/* check that device has data available */
		UVOS_DELAY_WaitmS( 10 );
		if ( UVOS_MPU_data_ready() ) {
			break;
		}
	}

	if ( tries >= UVOS_MPU_MAX_TRIES ) {
		return -2;
	}

	if ( dev->type == Invensense_ICM20608 ||
	     dev->type == Invensense_ICM20602 ||
	     dev->type == Invensense_ICM20601 ) {
		// this avoids a sensor bug for these sensors if used, taken from
		// https://github.com/ArduPilot AP_InertialSensor_Invensense.cpp#L345
		UVOS_MPU_SetReg( MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE );
	}

	/* Save register adresses for beginnings of gyro and accel register banks */
	dev->mpu_gyro_first_reg = MPUREG_GYRO_XOUT_H;
	dev->mpu_accel_first_reg = MPUREG_ACCEL_XOUT_H;

	/* Set ranges based on settings defined in uvos_board.c */
	if ( UVOS_MPUV1_ConfigGyroRange( dev->cfg->default_gyro_range ) ) {
		return -4;
	}
	if ( UVOS_MPUV1_ConfigAccelRange( dev->cfg->default_accel_range ) ) {
		return -4;
	}

	/* Here we will config IMU to run continuous at MPU-60x0
	 * max sample rate of 8K with a 256Hz digital low pass filter */
	if ( UVOS_MPUV1_ConfigFilter( BITS_DLPF_CFG_256HZ_NOLPF2 ) ) {
		return -4;
	}
	if ( UVOS_MPUV1_ConfigSampleRate( UVOS_MPUV1_8K_SMPLRT_8000HZ ) ) {
		return -4;
	}

	return 0;
}

/**
 * @brief Configures Gyro range
 * @return 0 if successful, -1 if device has not been initialized
 */
int32_t UVOS_MPUV1_ConfigGyroRange( uvos_mpu_gyro_range_t gyroRange )
{
	int32_t r = UVOS_MPU_SetRegisterBits( MPUREG_GYRO_CONFIG, BITS_GYRO_FS_MASK, gyroRange );
	return r;
}

/**
 * @brief Configures Accel range
 * @return 0 if successful, -1 if device has not been initialized
 */
int32_t UVOS_MPUV1_ConfigAccelRange( uvos_mpu_accel_range_t accelRange )
{
	int32_t r = UVOS_MPU_SetRegisterBits( MPUREG_ACCEL_CONFIG, BITS_ACCEL_FS_MASK, accelRange );
	return r;
}

/**
 * @brief Configures Filter setting
 * @return 0 if successful, -1 if device has not been initialized
 */
int32_t UVOS_MPUV1_ConfigFilter( uvos_mpuv1_filter_t filterSetting )
{
	int32_t r = UVOS_MPU_SetRegWithVerify( MPUREG_CONFIG, filterSetting );
	return r;
}

/**
 * @brief Configures sample rate divider (is based upon digital filtering settings)
 * @return 0 if successful, -1 if device has not been initialized
 */
int32_t UVOS_MPUV1_ConfigSampleRate( uvos_mpuv1_samplerate_div_t sampleRate )
{
	int32_t r = UVOS_MPU_SetRegWithVerify( MPUREG_SMPLRT_DIV, sampleRate );
	return r;
}
