#include <uvos.h>
#include <uvos_mpu.h>

#ifdef UVOS_INCLUDE_MPU

/* Global Variables */

enum uvos_mpu_dev_magic {
	UVOS_MPU_DEV_MAGIC = 0x9da9b3ed,
};

struct mpu_dev {
	enum uvos_mpu_dev_magic magic;

	uint32_t spi_id;
	uint32_t slave_num;

	// QueueHandle_t queue;
	const struct uvos_mpu_cfg * cfg;
	enum uvos_mpu_gyro_range gyro_range;
	enum uvos_mpu_accel_range accel_range;
	enum uvos_mpu_filter filter;
	enum uvos_mpu_invensense_type type;
	// bool spi_speed_fast;
};

#define UVOS_MPU_SPI_SLOW    			false
#define UVOS_MPU_SPI_FAST    			true
#define UVOS_MPU_SAMPLES_BYTES    14
#define UVOS_MPU_SENSOR_FIRST_REG MPUREG_ACCEL_XOUT_H

typedef union {
	uint8_t buffer[UVOS_MPU_SAMPLES_BYTES];
	struct {
		// uint8_t dummy;
		uint8_t Accel_X_h;
		uint8_t Accel_X_l;
		uint8_t Accel_Y_h;
		uint8_t Accel_Y_l;
		uint8_t Accel_Z_h;
		uint8_t Accel_Z_l;
		uint8_t Temperature_h;
		uint8_t Temperature_l;
		uint8_t Gyro_X_h;
		uint8_t Gyro_X_l;
		uint8_t Gyro_Y_h;
		uint8_t Gyro_Y_l;
		uint8_t Gyro_Z_h;
		uint8_t Gyro_Z_l;
	} data;
} mpu_data_t;

// #define GET_SENSOR_DATA(mpudataptr, sensor) (mpudataptr.data.sensor##_h << 8 | mpudataptr.data.sensor##_l)

// ! Global structure for this MPU device
static struct mpu_dev * _mpu_dev;
volatile bool _mpu_configured = false;
static mpu_data_t _mpu_data;
static uint8_t _last_stat_user_ctrl; /* Last status from register user control */

// ! Private functions
static struct mpu_dev * UVOS_MPU_alloc( const struct uvos_mpu_cfg * cfg );
static int32_t UVOS_MPU_Validate( struct mpu_dev * dev );
static uint32_t UVOS_MPU_Config( struct uvos_mpu_cfg const * cfg );
// static uint32_t UVOS_MPU_ReadSensor( void );
static int32_t UVOS_MPU_SetReg( uint8_t address, uint8_t data );
static int32_t UVOS_MPU_SetRegWithVerify( uint8_t address, uint8_t data );
static uint32_t UVOS_MPU_SetRegisterBits( uint8_t address, uint8_t mask, uint8_t value );
static int32_t UVOS_MPU_GetReg( uint8_t address );
static uint32_t UVOS_MPU_GetData( uint8_t address, uint8_t data[], uint8_t size, bool spi_speed );
static int32_t UVOS_MPU_ClaimBus( bool spi_speed );
static int32_t UVOS_MPU_ReleaseBus();
static bool UVOS_MPU_data_ready( void );
static invensense_type_e UVOS_MPU_whoami( void );

/**
 * @brief Allocate a new device
 */
static struct mpu_dev * UVOS_MPU_alloc( const struct uvos_mpu_cfg * cfg )
{
	struct mpu_dev * mpu_dev;

	mpu_dev = ( struct mpu_dev * )UVOS_malloc( sizeof( *mpu_dev ) );
	UVOS_Assert( mpu_dev );

	mpu_dev->magic = UVOS_MPU_DEV_MAGIC;

	// mpu_dev->queue = xQueueCreate( cfg->max_downsample + 1, SENSOR_DATA_SIZE );
	// UVOS_Assert( mpu_dev->queue );

	// queue_data = ( UVOS_SENSORS_3Axis_SensorsWithTemp * )uvos_malloc( SENSOR_DATA_SIZE );
	// UVOS_Assert( queue_data );
	// queue_data->count = SENSOR_COUNT;
	return mpu_dev;
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device, Neg error code otherwise
 */
static int32_t UVOS_MPU_Validate( struct mpu_dev * dev )
{
	if ( dev == NULL ) {
		return -1; // no mpu device
	}
	if ( dev->magic != UVOS_MPU_DEV_MAGIC ) {
		return -2; // wrong magic
	}
	if ( dev->spi_id == 0 ) {
		return -3; // no spi
	}
	return 0;
}

/**
 * @brief Initialize the MPU 3-axis gyro sensor.
 * @return 0 for success, -1 for failure
 */
int32_t UVOS_MPU_Init( uint32_t spi_id, uint32_t slave_num, const struct uvos_mpu_cfg * cfg )
{
	if ( spi_id == 0 ) {
		return -1;
	}

	_mpu_dev = UVOS_MPU_alloc( cfg );
	if ( _mpu_dev == NULL ) {
		return -1;
	}

	_mpu_dev->spi_id  				= spi_id;
	_mpu_dev->slave_num				= slave_num;
	_mpu_dev->cfg 						= cfg;

	/* Configure the MPU Sensor */
	return UVOS_MPU_Config( cfg );
}

/**
 * @brief Initialize the MPU 3-axis gyro/accel sensor
 * \return 0 if successful, non-zero negative value if fail
 * \param[in] uvos_mpu_cfg struct to be used to configure sensor.
 *
 */
static uint32_t UVOS_MPU_Config( struct uvos_mpu_cfg const * cfg )
{

	// MPU POWER ON
	UVOS_MPU_SetReg( MPUREG_PWR_MGMT_1, 0x01 );
	// MPU Gyro and Accel ON
	UVOS_MPU_SetReg( MPUREG_PWR_MGMT_2, 0x00 );
	UVOS_DELAY_WaituS( 10000 );

	/* Configure MPU SPI bus speed to slow speed for config */
	UVOS_MPU_SetSpeedFast( UVOS_MPU_SPI_SLOW );

	invensense_type_e id = UVOS_MPU_whoami();
	// if ( ( id == Invalid_IMU_ID ) || ( id != Invensense_MPU6000 && id != Invensense_ICM20602 ) ) {
	if ( ( id == Invalid_IMU_ID ) || ( id != _mpu_dev->cfg->device_id  ) ) {
		return -1;
	}

	// Chip reset
	uint8_t tries;
	for ( tries = 0; tries < UVOS_MPU_MAX_TRIES; tries++ ) {
		_last_stat_user_ctrl = UVOS_MPU_GetReg( MPUREG_USER_CTRL );

		/* First disable the master I2C to avoid hanging the slaves on the
		 * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
		 * is used */
		if ( _last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN ) {
			_last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
			UVOS_MPU_SetReg( MPUREG_USER_CTRL, _last_stat_user_ctrl );
			UVOS_DELAY_WaitmS( 10 );
		}

		/* reset device, wait 100mS per datasheet */
		UVOS_MPU_SetReg( MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET );
		UVOS_DELAY_WaitmS( 100 );

		/* reset signal path as recommended in datasheet */
		if ( id == Invensense_MPU6000 || id == Invensense_MPU6500 ) {
			UVOS_MPU_SetReg( MPUREG_SIGNAL_PATH_RESET,
			                 BIT_SIGNAL_PATH_RESET_TEMP_RESET | BIT_SIGNAL_PATH_RESET_ACCEL_RESET | BIT_SIGNAL_PATH_RESET_GYRO_RESET );
			UVOS_DELAY_WaitmS( 100 );
		}

		/* Disable I2C bus if SPI selected (Recommended in Datasheet to be
		 * done just after the device is reset) */
		_last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
		UVOS_MPU_SetReg( MPUREG_USER_CTRL, _last_stat_user_ctrl );

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

	if ( id == Invensense_ICM20608 ||
	     id == Invensense_ICM20602 ||
	     id == Invensense_ICM20601 ) {
		// this avoids a sensor bug for these sensors if used, taken from
		// https://github.com/ArduPilot AP_InertialSensor_Invensense.cpp#L345
		UVOS_MPU_SetReg( MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE );
	}

	UVOS_MPU_ConfigureRanges(
	  _mpu_dev->cfg->gyro_range,
	  _mpu_dev->cfg->accel_range,
	  _mpu_dev->cfg->filter
	);

	UVOS_MPU_SetSpeedFast( UVOS_MPU_SPI_FAST );

	_mpu_configured = true;

	return 0;
}

// static bool mpu_set_registerbits( uint8_t address, uint8_t mask, uint8_t value );

/**
 * @brief Configures Gyro, accel and Filter ranges/setings
 * @return 0 if successful, -1 if device has not been initialized
 */
int32_t UVOS_MPU_ConfigureRanges(
  enum uvos_mpu_gyro_range gyroRange,
  enum uvos_mpu_accel_range accelRange,
  enum uvos_mpu_filter filterSetting )
{
	if ( _mpu_dev == NULL ) {
		return -1;
	}

	// update filter settings
	// while ( UVOS_MPU_SetReg( MPUREG_CONFIG, filterSetting ) != 0 ) {
	// 	;
	// }

	// Sample rate divider, chosen upon digital filtering settings
	// while ( UVOS_MPU_SetReg( MPUREG_SMPLRT_DIV,
	//                          filterSetting == UVOS_LOWPASS_256_HZ_BITS ?
	//                          _mpu_dev->cfg->Smpl_rate_div_no_dlp : _mpu_dev->cfg->Smpl_rate_div_dlp ) != 0 ) {
	// 	;
	// }
	// _mpu_dev->filter = filterSetting;

	// Gyro range
	// while ( UVOS_MPU_SetReg( MPUREG_GYRO_CONFIG, gyroRange ) != 0 ) {
	while ( UVOS_MPU_SetRegisterBits( MPUREG_GYRO_CONFIG, BITS_GYRO_FS_MASK, gyroRange ) != 0 ) {
		// while ( mpu_set_registerbits( MPUREG_GYRO_CONFIG, BITS_GYRO_FS_MASK, gyroRange ) != true ) {
		;
	}
	_mpu_dev->gyro_range = gyroRange;

	// Set the accel range
	// while ( UVOS_MPU_SetReg( MPUREG_ACCEL_CONFIG, accelRange ) != 0 ) {
	while ( UVOS_MPU_SetRegisterBits( MPUREG_ACCEL_CONFIG, BITS_ACCEL_FS_MASK, accelRange ) != 0 ) {
		// while ( mpu_set_registerbits( MPUREG_ACCEL_CONFIG, BITS_ACCEL_FS_MASK, accelRange ) != true ) {
		;
	}
	_mpu_dev->accel_range = accelRange;

	return 0;
}

int32_t UVOS_MPU_ReadImu( int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz )
{
	// if ( UVOS_MPU_ReadSensor() != 0 ) {
	// 	return -1;
	// }

	if ( UVOS_MPU_GetData( UVOS_MPU_SENSOR_FIRST_REG, _mpu_data.buffer, sizeof( mpu_data_t ), UVOS_MPU_SPI_FAST ) != 0 ) {
		return -1;
	}

#if 1
	*ax = ( ( ( int16_t )_mpu_data.buffer[0] ) << 8 ) | _mpu_data.buffer[1];
	*ay = ( ( ( int16_t )_mpu_data.buffer[2] ) << 8 ) | _mpu_data.buffer[3];
	*az = ( ( ( int16_t )_mpu_data.buffer[4] ) << 8 ) | _mpu_data.buffer[5];
	*gx = ( ( ( int16_t )_mpu_data.buffer[8] ) << 8 ) | _mpu_data.buffer[9];
	*gy = ( ( ( int16_t )_mpu_data.buffer[10] ) << 8 ) | _mpu_data.buffer[11];
	*gz = ( ( ( int16_t )_mpu_data.buffer[12] ) << 8 ) | _mpu_data.buffer[13];
#else
	*ax = ( ( ( int16_t )_mpu_data.data.Accel_X_h ) << 8 ) | _mpu_data.data.Accel_X_l;
	*ay = ( ( ( int16_t )_mpu_data.data.Accel_Y_h ) << 8 ) | _mpu_data.data.Accel_Y_l;
	*az = ( ( ( int16_t )_mpu_data.data.Accel_Z_h ) << 8 ) | _mpu_data.data.Accel_Z_l;
	*gx = ( ( ( int16_t )_mpu_data.data.Gyro_X_h ) << 8 ) | _mpu_data.data.Gyro_X_l;
	*gy = ( ( ( int16_t )_mpu_data.data.Gyro_Y_h ) << 8 ) | _mpu_data.data.Gyro_Y_l;
	*gz = ( ( ( int16_t )_mpu_data.data.Gyro_Z_h ) << 8 ) | _mpu_data.data.Gyro_Z_l;
#endif

	return 0;
}

// static uint32_t UVOS_MPU_ReadSensor( void )
// {
// 	const uint8_t mpu_send_buf[1 + UVOS_MPU_SAMPLES_BYTES] = { UVOS_MPU_SENSOR_FIRST_REG | 0x80 };

// 	if ( UVOS_MPU_ClaimBus( true ) != 0 ) {
// 		return -1;
// 	}
// 	if ( UVOS_SPI_TransferBlock_PIO( _mpu_dev->spi_id, &mpu_send_buf[0], &_mpu_data.buffer[0], sizeof( mpu_data_t ) ) < 0 ) {
// 		UVOS_MPU_ReleaseBus();
// 		return -2;
// 	}
// 	UVOS_MPU_ReleaseBus();
// 	return 0;
// }


/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return none
 * @param[fast] boolean true = fast SPI, false = slow SPI
 */
void UVOS_MPU_SetSpeedFast( const bool fast )
{
	if ( fast ) {
		UVOS_SPI_SetClockSpeed( _mpu_dev->spi_id, _mpu_dev->cfg->fast_prescaler );
	} else {
		UVOS_SPI_SetClockSpeed( _mpu_dev->spi_id, _mpu_dev->cfg->std_prescaler );
	}
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 * @param[spi_speed] boolean true = fast SPI, false = slow SPI
 */
static int32_t UVOS_MPU_ClaimBus( bool spi_speed )
{
	if ( UVOS_MPU_Validate( _mpu_dev ) != 0 ) {
		return -1;
	}
	if ( UVOS_SPI_ClaimBus( _mpu_dev->spi_id ) != 0 ) {
		return -2;
	}
	UVOS_MPU_SetSpeedFast( spi_speed );
	UVOS_SPI_RC_PinSet( _mpu_dev->spi_id, _mpu_dev->slave_num, 0 );
	return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful
 */
static int32_t UVOS_MPU_ReleaseBus()
{
	if ( UVOS_MPU_Validate( _mpu_dev ) != 0 ) {
		return -1;
	}
	UVOS_SPI_RC_PinSet( _mpu_dev->spi_id, _mpu_dev->slave_num, 1 );
	return UVOS_SPI_ReleaseBus( _mpu_dev->spi_id );
}

/**
 * @brief Read a register from MPU
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t UVOS_MPU_GetReg( uint8_t reg )
{
	uint8_t data;

	if ( UVOS_MPU_ClaimBus( UVOS_MPU_SPI_SLOW ) != 0 ) {
		return -1;
	}

	UVOS_SPI_TransferByte( _mpu_dev->spi_id, ( 0x80 | reg ) ); // request byte
	data = UVOS_SPI_TransferByte( _mpu_dev->spi_id, 0 ); // receive response

	UVOS_MPU_ReleaseBus();
	return data;
}

static uint32_t UVOS_MPU_GetData( uint8_t address, uint8_t data[], uint8_t size, bool spi_speed )
{
	if ( UVOS_MPU_ClaimBus( spi_speed ) != 0 ) {
		return -1;
	}

	UVOS_SPI_TransferByte( _mpu_dev->spi_id, ( address | 0x80 ) );
	for ( uint8_t i = 0; i < size; ++i ) {
		data[ i ] = UVOS_SPI_TransferByte( _mpu_dev->spi_id, 0 );
	}

	UVOS_MPU_ReleaseBus();
	return 0;
}

/**
 * @brief Writes one byte to the MPU
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 failed to send address byte
 * \return -3 failed to send data byte
 */
static int32_t UVOS_MPU_SetReg( uint8_t address, uint8_t data )
{
	if ( UVOS_MPU_ClaimBus( UVOS_MPU_SPI_SLOW ) != 0 ) {
		return -1;
	}
	/* transfer register address to write */
	if ( UVOS_SPI_TransferByte( _mpu_dev->spi_id, 0x7f & address ) != 0 ) {
		UVOS_MPU_ReleaseBus();
		return -2;
	}
	/* transfer data byte */
	if ( UVOS_SPI_TransferByte( _mpu_dev->spi_id, data ) != 0 ) {
		UVOS_MPU_ReleaseBus();
		return -3;
	}

	UVOS_MPU_ReleaseBus();

	return 0;
}

static int32_t UVOS_MPU_SetRegWithVerify( uint8_t address, uint8_t data )
{
	if ( UVOS_MPU_ClaimBus( UVOS_MPU_SPI_SLOW ) != 0 ) {
		return -1;
	}
	/* transfer register address to write */
	if ( UVOS_SPI_TransferByte( _mpu_dev->spi_id, 0x7f & address ) != 0 ) {
		UVOS_MPU_ReleaseBus();
		return -2;
	}
	/* transfer data byte */
	if ( UVOS_SPI_TransferByte( _mpu_dev->spi_id, data ) != 0 ) {
		UVOS_MPU_ReleaseBus();
		return -3;
	}
	UVOS_MPU_ReleaseBus();
	UVOS_DELAY_WaituS( 10 );
	_mpu_data.buffer[0] = UVOS_MPU_GetReg( address ); // read back the register
	// check the read back register against the written register
	if ( _mpu_data.buffer[0] == data ) {
		return 0;
	}
	return -4;
}

/* Write individual bits to MPU register */
static uint32_t UVOS_MPU_SetRegisterBits( uint8_t address, uint8_t mask, uint8_t value )
{
	uint8_t temp = UVOS_MPU_GetReg( address );  // get current register contents
	temp &= ~( mask );                      // zero out temp bits to be written
	temp |= value;                          // bitwise OR bits to be set
	uint32_t r = UVOS_MPU_SetRegWithVerify( address, temp );
	return r;
}

/*
 * Return true if the Invensense has new data available for reading.
 *
 */
static bool UVOS_MPU_data_ready( void )
{
	uint8_t status = UVOS_MPU_GetReg( MPUREG_INT_STATUS );
	return ( status & BIT_RAW_RDY_INT ) != 0;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

#if 0 // GLS

static void mpu_writereg( uint8_t address, uint8_t value )
{
	UVOS_MPU_ClaimBus( false );
	UVOS_SPI_TransferByte( _mpu_dev->spi_id, 0x7f & address );	// transfer register address to write
	UVOS_SPI_TransferByte( _mpu_dev->spi_id, value ); 					// transfer data byte
	UVOS_MPU_ReleaseBus();
	UVOS_DELAY_WaituS( 1 );
}

static uint8_t mpu_readreg( uint8_t address )
{
	UVOS_MPU_ClaimBus( false );
	UVOS_SPI_TransferByte( _mpu_dev->spi_id, address | 0x80 ); 	// request byte
	uint8_t value = UVOS_SPI_TransferByte( _mpu_dev->spi_id, 0 ); 				// receive response
	UVOS_MPU_ReleaseBus();
	UVOS_DELAY_WaituS( 1 );
	return value;
}

static bool mpu_writereg_and_verify( uint8_t address, uint8_t value )
{
	UVOS_MPU_ClaimBus( false );
	UVOS_SPI_TransferByte( _mpu_dev->spi_id, 0x7f & address );	// transfer register address to write
	UVOS_SPI_TransferByte( _mpu_dev->spi_id, value ); 					// transfer data byte
	UVOS_MPU_ReleaseBus();
	UVOS_DELAY_WaituS( 10 );
	_mpu_data.buffer[0] = mpu_readreg( address ); // read back the register
	// check the read back register against the written register
	if ( _mpu_data.buffer[0] == value ) {
		return true;
	}
	return false;
}

static void mpu_readdata( uint8_t address, uint8_t data[], uint8_t size )
{
	UVOS_MPU_ClaimBus( false );
	// UVOS_MPU_sendbyte( address | 0x80 );
	UVOS_SPI_TransferByte( _mpu_dev->spi_id, ( address | 0x80 ) );
	for ( uint8_t i = 0; i < size; ++i ) {
		// data[ i ] = UVOS_MPU_sendzerorecvbyte();
		data[ i ] = UVOS_SPI_TransferByte( _mpu_dev->spi_id, 0 );
	}
	UVOS_MPU_ReleaseBus();
}

/* Write individual bits to MPU register */
static bool mpu_set_registerbits( uint8_t address, uint8_t mask, uint8_t value )
{
	uint8_t temp = mpu_readreg( address );  // get current register contents
	temp &= ~( mask );                      // zero out temp bits to be written
	temp |= value;                          // bitwise OR bits to be set
	bool r = mpu_writereg_and_verify( address, temp );
	return r;
}

#endif // GLS

// Check whoami for sensor type
static invensense_type_e UVOS_MPU_whoami( void )
{
	uint8_t whoami = UVOS_MPU_GetReg( MPUREG_WHOAMI );
	switch ( whoami ) {
	case MPU_WHOAMI_6000:
		return Invensense_MPU6000;
		break;
	case MPU_WHOAMI_6500:
		return Invensense_MPU6500;
		break;
	case MPU_WHOAMI_MPU9250:
	case MPU_WHOAMI_MPU9255:
		return Invensense_MPU9250;
		break;
	case MPU_WHOAMI_20608D:
	case MPU_WHOAMI_20608G:
		return Invensense_ICM20608;
		break;
	case MPU_WHOAMI_20602:
		return Invensense_ICM20602;
		break;
	case MPU_WHOAMI_20601:
		return Invensense_ICM20601;
		break;
	case MPU_WHOAMI_ICM20789:
	case MPU_WHOAMI_ICM20789_R1:
		return Invensense_ICM20789;
		break;
	case MPU_WHOAMI_ICM20689:
		return Invensense_ICM20689;
		break;
	default :
		return Invalid_IMU_ID;
		break;
	}
}

#pragma GCC pop_options

#endif // UVOS_INCLUDE_MPU
