#include <uvos.h>
#include "uvos_mpu.h"
#include "uvos_mpu_priv.h"

#ifdef UVOS_INCLUDE_MPU

#define UVOS_MPU_SAMPLES_BYTES    6

typedef union {
	uint8_t buffer[UVOS_MPU_SAMPLES_BYTES];
	struct {
		// uint8_t dummy;
		uint8_t Data_X_h;
		uint8_t Data_X_l;
		uint8_t Data_Y_h;
		uint8_t Data_Y_l;
		uint8_t Data_Z_h;
		uint8_t Data_Z_l;
	} data;
} mpu_data_t;

// ! Global structure for this MPU device
static mpu_dev_t *_mpu_dev;
static mpu_data_t _mpu_data;
static bool _mpu_configured = false;

// ! Private functions
static struct mpu_dev *UVOS_MPU_alloc( const struct uvos_mpu_cfg *cfg );
static int32_t UVOS_MPU_Validate( mpu_dev_t *dev );
static uint32_t UVOS_MPU_Config( struct uvos_mpu_cfg const *cfg );
static invensense_type_t UVOS_MPU_whoami( void );

/**
 * @brief Allocate a new device
 */
static struct mpu_dev *UVOS_MPU_alloc( const struct uvos_mpu_cfg *cfg )
{
	struct mpu_dev *mpu_dev;

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
static int32_t UVOS_MPU_Validate( mpu_dev_t *dev )
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
int32_t UVOS_MPU_Init( uint32_t spi_id, uint32_t slave_num, const struct uvos_mpu_cfg *cfg )
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
	_mpu_dev->sample_rate_hz  = cfg->default_samplerate_hz;
	_mpu_dev->gyro_range      = cfg->default_gyro_range;
	_mpu_dev->accel_range     = cfg->default_accel_range;

	/* Give IMU some time to settle after power-up */
	UVOS_DELAY_WaituS( 10000 );

	/* Configure IMU SPI bus speed to slow speed for config */
	UVOS_MPU_SetSpeed( UVOS_MPU_SPI_SPEED_SLOW );

	/* Determine IMU sensor type and family, fail out in no valid sensor found */
	invensense_type_t mpu_type = UVOS_MPU_whoami();
	_mpu_dev->type = mpu_type;
	if ( ( _mpu_dev->type == Invensense_Invalid_ID ) || ( _mpu_dev->type != _mpu_dev->cfg->expected_device_id ) ) {
		return -2; // no imu found
	}

	int32_t return_val = 0;
	/* Initialize the IMU hardware registers */
	if ( _mpu_dev->family == 1 ) {
		return_val = UVOS_MPUV1_Hardware_Init( _mpu_dev );
	} else if ( _mpu_dev->family == 2 ) {
		return_val = -1; // IMU family not implemented yet
	} else if ( _mpu_dev->family == 3 ) {
		return_val = UVOS_MPUV3_Hardware_Init( _mpu_dev );
	}

	if ( return_val != 0 ) {
		_mpu_configured = false;
		return -3; // mpu int error
	} else {
		_mpu_configured = true;
	}

	UVOS_MPU_SetSpeed( UVOS_MPU_SPI_SPEED_FAST );

	return 0;
}

int32_t UVOS_MPU_ReadImu( int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz )
{

	/* Get accel data */
	if ( UVOS_MPU_GetData( _mpu_dev->mpu_accel_first_reg, _mpu_data.buffer, sizeof( mpu_data_t ), UVOS_MPU_SPI_SPEED_FAST ) != 0 ) {
		return -1;
	}
	*ax = ( ( ( int16_t )_mpu_data.buffer[0] ) << 8 ) | _mpu_data.buffer[1];
	*ay = ( ( ( int16_t )_mpu_data.buffer[2] ) << 8 ) | _mpu_data.buffer[3];
	*az = ( ( ( int16_t )_mpu_data.buffer[4] ) << 8 ) | _mpu_data.buffer[5];

	/* Get gyro data */
	if ( UVOS_MPU_GetData( _mpu_dev->mpu_gyro_first_reg, _mpu_data.buffer, sizeof( mpu_data_t ), UVOS_MPU_SPI_SPEED_FAST ) != 0 ) {
		return -2;
	}
	*gx = ( ( ( int16_t )_mpu_data.buffer[0] ) << 8 ) | _mpu_data.buffer[1];
	*gy = ( ( ( int16_t )_mpu_data.buffer[2] ) << 8 ) | _mpu_data.buffer[3];
	*gz = ( ( ( int16_t )_mpu_data.buffer[4] ) << 8 ) | _mpu_data.buffer[5];

	return 0;
}

int32_t UVOS_MPU_ConfigGyroRange( uvos_mpu_gyro_range_t gyroRange )
{
	if ( _mpu_dev->family == 1 ) {
		UVOS_MPUV1_ConfigGyroRange( gyroRange );
	}
	_mpu_dev->gyro_range = gyroRange;
	return 0;
}

int32_t UVOS_MPU_ConfigAccelRange( uvos_mpu_accel_range_t accelRange )
{
	if ( _mpu_dev->family == 1 ) {
		UVOS_MPUV1_ConfigAccelRange( accelRange );
	}
	_mpu_dev->accel_range = accelRange;
	return 0;
}

int32_t UVOS_MPU_ConfigImuRate( uint16_t sampleRateHz )
{
	_mpu_dev->sample_rate_hz = sampleRateHz;
	return 0;
}

/*
 * Return true if the Invensense has new data available for reading.
 *
 */
bool UVOS_MPU_data_ready( void )
{
	uint8_t status = UVOS_MPU_GetReg( MPUREG_INT_STATUS );
	return ( status & BIT_RAW_RDY_INT ) != 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return negative value if error
 * @param[fast] UVOS_MPU_SPI_SPEED_FAST = fast SPI, UVOS_MPU_SPI_SPEED_SLOW = slow SPI
 */
int32_t UVOS_MPU_SetSpeed( const uvos_mpu_spi_speed_t spi_speed )
{
	int32_t ret;
	if ( spi_speed == UVOS_MPU_SPI_SPEED_SLOW || spi_speed == UVOS_MPU_SPI_SPEED_FAST ) {
		if ( spi_speed == UVOS_MPU_SPI_SPEED_FAST ) {
			ret = UVOS_SPI_SetClockSpeed( _mpu_dev->spi_id, _mpu_dev->cfg->fast_prescaler );
		} else {
			ret = UVOS_SPI_SetClockSpeed( _mpu_dev->spi_id, _mpu_dev->cfg->std_prescaler );
		}
		return ret;
	} else {
		return -1;
	}
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 * @param[spi_speed] boolean true = fast SPI, false = slow SPI
 */
int32_t UVOS_MPU_ClaimBus( const uvos_mpu_spi_speed_t spi_speed )
{
	if ( UVOS_MPU_Validate( _mpu_dev ) != 0 ) {
		return -1;
	}
	if ( UVOS_SPI_ClaimBus( _mpu_dev->spi_id ) != 0 ) {
		return -2;
	}
	UVOS_MPU_SetSpeed( spi_speed );
	UVOS_SPI_RC_PinSet( _mpu_dev->spi_id, _mpu_dev->slave_num, 0 );
	return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful
 */
int32_t UVOS_MPU_ReleaseBus()
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
int32_t UVOS_MPU_GetReg( uint8_t reg )
{
	uint8_t data;

	if ( UVOS_MPU_ClaimBus( UVOS_MPU_SPI_SPEED_SLOW ) != 0 ) {
		return -1;
	}

	UVOS_SPI_TransferByte( _mpu_dev->spi_id, ( 0x80 | reg ) ); // request byte
	data = UVOS_SPI_TransferByte( _mpu_dev->spi_id, 0 ); // receive response

	UVOS_MPU_ReleaseBus();
	return data;
}

uint32_t UVOS_MPU_GetData( uint8_t address, uint8_t data[], uint8_t size, bool spi_speed )
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
int32_t UVOS_MPU_SetReg( uint8_t address, uint8_t data )
{
	if ( UVOS_MPU_ClaimBus( UVOS_MPU_SPI_SPEED_SLOW ) != 0 ) {
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

int32_t UVOS_MPU_SetRegWithVerify( uint8_t address, uint8_t data )
{
	if ( UVOS_MPU_ClaimBus( UVOS_MPU_SPI_SPEED_SLOW ) != 0 ) {
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
uint32_t UVOS_MPU_SetRegisterBits( uint8_t address, uint8_t mask, uint8_t value )
{
	uint8_t temp = UVOS_MPU_GetReg( address );  // get current register contents
	temp &= ~( mask );                      // zero out temp bits to be written
	temp |= value;                          // bitwise OR bits to be set
	int32_t r = UVOS_MPU_SetRegWithVerify( address, temp );
	return r;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

// Check whoami for sensor type
static invensense_type_t UVOS_MPU_whoami( void )
{
	invensense_type_t mpu_type = Invensense_Invalid_ID;

	uint8_t whoami = UVOS_MPU_GetReg( MPUREG_WHOAMI );

	switch ( whoami ) {
	case MPU_WHOAMI_6000:
		_mpu_dev->family = 1;
		mpu_type = Invensense_MPU6000;
		break;
	case MPU_WHOAMI_6500:
		_mpu_dev->family = 1;
		mpu_type = Invensense_MPU6500;
		break;
	case MPU_WHOAMI_MPU9250:
	case MPU_WHOAMI_MPU9255:
		_mpu_dev->family = 1;
		mpu_type = Invensense_MPU9250;
		break;
	case MPU_WHOAMI_20608D:
	case MPU_WHOAMI_20608G:
		_mpu_dev->family = 1;
		mpu_type = Invensense_ICM20608;
		break;
	case MPU_WHOAMI_20602:
		_mpu_dev->family = 1;
		mpu_type = Invensense_ICM20602;
		break;
	case MPU_WHOAMI_20601:
		_mpu_dev->family = 1;
		mpu_type = Invensense_ICM20601;
		break;
	case MPU_WHOAMI_ICM20789:
	case MPU_WHOAMI_ICM20789_R1:
		_mpu_dev->family = 1;
		mpu_type = Invensense_ICM20789;
		break;
	case MPU_WHOAMI_ICM20689:
		_mpu_dev->family = 1;
		mpu_type = Invensense_ICM20689;
		break;
	case INV3_ID_ICM40609:
		_mpu_dev->family = 3;
		mpu_type = Invensense_ICM40609;
		break;
	case INV3_ID_ICM42688:
		_mpu_dev->family = 3;
		mpu_type = Invensense_ICM42688;
		break;
	case INV3_ID_ICM42605:
		_mpu_dev->family = 3;
		mpu_type = Invensense_ICM42605;
		break;
	case INV3_ID_ICM40605:
		_mpu_dev->family = 3;
		mpu_type = Invensense_ICM40605;
		break;
	case INV3_ID_IIM42652:
		_mpu_dev->family = 3;
		mpu_type = Invensense_IIM42652;
		break;
	case INV3_ID_ICM42670:
		_mpu_dev->family = 3;
		mpu_type = Invensense_ICM42670;
		break;
	default :
		_mpu_dev->family = 0;
		mpu_type = Invensense_Invalid_ID;
		break;
	}

	return mpu_type;
}

#pragma GCC pop_options

#endif // UVOS_INCLUDE_MPU
