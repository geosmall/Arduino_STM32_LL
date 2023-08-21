#include <uvos.h>
#include "uvos_icm42688p.h"
#include "uvos_icm42688p_config.h"

#ifdef UVOS_INCLUDE_ICM42688P
#include <stdint.h>
#include <uvos_constants.h>
#include <uvos_sensors.h>

/* Global Variables */

static const uint8_t ICM42688P_WHOAMI_VALUE = 0x47;

enum uvos_icm42688p_dev_magic {
  UVOS_ICM42688P_DEV_MAGIC = 0x9da9b3ed,
};

// sensor driver interface
bool UVOS_ICM42688P_driver_Test( uintptr_t context );
bool UVOS_ICM42688P_driver_poll( uintptr_t context );
void UVOS_ICM42688P_driver_fetch( void *data, uint8_t size, uintptr_t context );
void UVOS_ICM42688P_driver_Reset( uintptr_t context );
p_uvos_queue_t UVOS_ICM42688P_driver_get_queue( uintptr_t context );
void UVOS_ICM42688P_driver_get_scale( float *scales, uint8_t size, uintptr_t context );

const UVOS_SENSORS_Driver UVOS_ICM42688P_Driver = {
  .test      = UVOS_ICM42688P_driver_Test,
  .poll      = UVOS_ICM42688P_driver_poll,
  .fetch     = UVOS_ICM42688P_driver_fetch,
  .reset     = UVOS_ICM42688P_driver_Reset,
  .get_queue = UVOS_ICM42688P_driver_get_queue,
  .get_scale = UVOS_ICM42688P_driver_get_scale,
  .is_polled = false,
};
//

static uint8_t _bank = 0; ///< current user bank

struct icm42688p_dev {
  uint32_t spi_id;
  uint32_t slave_num;
  p_uvos_queue_t queue;
  const struct uvos_icm42688p_cfg *cfg;
  enum uvos_icm42688p_gyro_range gyro_range;
  enum uvos_icm42688p_accel_range accel_range;
  enum uvos_icm42688p_filter filter;
  enum uvos_icm42688p_dev_magic   magic;
};

#define UVOS_ICM42688P_SAMPLES_BYTES    14
#define UVOS_ICM42688P_SENSOR_FIRST_REG UB0_REG_TEMP_DATA1

typedef union {
  uint8_t buffer[1 + UVOS_ICM42688P_SAMPLES_BYTES];
  struct {
    uint8_t dummy;
    uint8_t Temperature_h;
    uint8_t Temperature_l;
    uint8_t Accel_X_h;
    uint8_t Accel_X_l;
    uint8_t Accel_Y_h;
    uint8_t Accel_Y_l;
    uint8_t Accel_Z_h;
    uint8_t Accel_Z_l;
    uint8_t Gyro_X_h;
    uint8_t Gyro_X_l;
    uint8_t Gyro_Y_h;
    uint8_t Gyro_Y_l;
    uint8_t Gyro_Z_h;
    uint8_t Gyro_Z_l;
  } data;
} icm42688p_data_t;

#define GET_SENSOR_DATA(mpudataptr, sensor) (mpudataptr.data.sensor##_h << 8 | mpudataptr.data.sensor##_l)

// ! Global structure for this device device
static struct icm42688p_dev *dev;
volatile bool icm42688p_configured = false;
static icm42688p_data_t icm42688p_data;
static UVOS_SENSORS_3Axis_SensorsWithTemp *queue_data = 0;
#define SENSOR_COUNT     2
#define SENSOR_DATA_SIZE (sizeof(UVOS_SENSORS_3Axis_SensorsWithTemp) + sizeof(Vector3i16) * SENSOR_COUNT)

// ! Private functions
static struct icm42688p_dev *UVOS_ICM42688P_alloc( const struct uvos_icm42688p_cfg *cfg );
static int32_t UVOS_ICM42688P_Validate( struct icm42688p_dev *dev );
// static void UVOS_ICM42688P_Config( struct uvos_icm42688p_cfg const *cfg );
static int32_t UVOS_ICM42688P_Config( struct uvos_icm42688p_cfg const *cfg );
static int32_t UVOS_ICM42688P_SetReg( uint8_t address, uint8_t buffer );
static int32_t UVOS_ICM42688P_GetReg( uint8_t address );
static void UVOS_ICM42688P_SetSpeed( const bool fast );
static bool UVOS_ICM42688P_HandleData( uint32_t gyro_read_timestamp );
static bool UVOS_ICM42688P_ReadSensor( bool *woken );

static int32_t UVOS_ICM42688P_Test( void );
static int32_t UVOS_ICM42688P_SetAccelRange( enum uvos_icm42688p_accel_range range );
static int32_t UVOS_ICM42688P_SetGyroRange( enum uvos_icm42688p_gyro_range range );

static int32_t UVOS_ICM42688P_SetBank( uint8_t bank );
static void UVOS_ICM42688P_GyroAccOff( void );
static void UVOS_ICM42688P_GyroAccOn( void );
static int32_t UVOS_ICM42688P_SetFilters( bool gyroFilters, bool accFilters );
static int32_t UVOS_ICM42688P_SetAccelODR( enum uvos_icm42688p_odr odr );
static int32_t UVOS_ICM42688P_SetGyroODR( enum uvos_icm42688p_odr odr );
static void UVOS_ICM42688P_Reset();

void UVOS_ICM42688P_Register()
{
  UVOS_SENSORS_Register( &UVOS_ICM42688P_Driver, UVOS_SENSORS_TYPE_3AXIS_GYRO_ACCEL, 0 );
}
/**
 * @brief Allocate a new device
 */
static struct icm42688p_dev *UVOS_ICM42688P_alloc( const struct uvos_icm42688p_cfg *cfg )
{
  struct icm42688p_dev *icm42688p_dev;

  icm42688p_dev = ( struct icm42688p_dev * )UVOS_malloc( sizeof( *icm42688p_dev ) );
  UVOS_Assert( icm42688p_dev );

  icm42688p_dev->magic = UVOS_ICM42688P_DEV_MAGIC;

  // icm42688p_dev->queue = xQueueCreate( cfg->max_downsample + 1, SENSOR_DATA_SIZE );
  icm42688p_dev->queue = UVOS_Queue_Create( cfg->max_downsample + 1, SENSOR_DATA_SIZE );
  UVOS_Assert( icm42688p_dev->queue );

  queue_data = ( UVOS_SENSORS_3Axis_SensorsWithTemp * )UVOS_malloc( SENSOR_DATA_SIZE );
  UVOS_Assert( queue_data );
  queue_data->count = SENSOR_COUNT;
  return icm42688p_dev;
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t UVOS_ICM42688P_Validate( struct icm42688p_dev *vdev )
{
  if ( vdev == NULL ) {
    return -1;
  }
  if ( vdev->magic != UVOS_ICM42688P_DEV_MAGIC ) {
    return -2;
  }
  if ( vdev->spi_id == 0 ) {
    return -3;
  }
  return 0;
}

/**
 * @brief Initialize the ICM42688P 3-axis gyro sensor.
 * @return 0 for success, -1 for failure
 */
int32_t UVOS_ICM42688P_Init( uint32_t spi_id, uint32_t slave_num, const struct uvos_icm42688p_cfg *cfg )
{
  dev = UVOS_ICM42688P_alloc( cfg );
  if ( dev == NULL ) {
    return -1;
  }

  dev->spi_id    = spi_id;
  dev->slave_num = slave_num;
  dev->cfg = cfg;

  /* Configure the ICM42688P Sensor */
  // UVOS_ICM42688P_Config( cfg );
  if ( UVOS_ICM42688P_Config( cfg ) != 0 ) {
    return -1;
  }

  /* Set up EXTI line */
  UVOS_EXTI_Init( cfg->exti_cfg );
  return 0;
}

/**
 * @brief Initialize the ICM42688P 3-axis gyro sensor
 * \return 0 if successful, negative number on error
 * \param[in] UVOS_ICM42688P_ConfigTypeDef struct to be used to configure sensor.
 */
static int32_t UVOS_ICM42688P_Config( struct uvos_icm42688p_cfg const *cfg )
{
  // Last status from register user control
  uint8_t _last_stat_user_ctrl;

  // Turn off ACC and GYRO so they can be configured
  // See section 12.9 in ICM-42688-P datasheet v1.7
  UVOS_ICM42688P_SetBank( 0 );
  UVOS_ICM42688P_GyroAccOff();

  // reset the ICM42688
  UVOS_ICM42688P_Reset();

  if ( UVOS_ICM42688P_Test() ) {
    return -1;
  }

  // turn on accel and gyro in Low Noise (LN) Mode
  if ( UVOS_ICM42688P_SetReg( UB0_REG_PWR_MGMT0, 0x0F ) < 0 ) {
    return -2;
  }

  IMUGyroAccelSettingsData settings;
  IMUGyroAccelSettingsGet( &settings );
  if ( settings.isSet ) {
    UVOS_ICM42688P_ConfigureRanges(
      UVOS_ICM42688P_CONFIG_MAP_GYROSCALE( settings.GyroScale ),
      UVOS_ICM42688P_CONFIG_MAP_ACCELSCALE( settings.AccelScale ),
      UVOS_ICM42688P_CONFIG_MAP_FILTERSETTING( settings.FilterSetting )
    );
  } else {
    UVOS_ICM42688P_ConfigureRanges(
      cfg->gyro_range,
      cfg->accel_range,
      cfg->filter
    );
  }

  // Turn on gyro and acc on again so ODR and FSR can be configured
  UVOS_ICM42688P_GyroAccOn();

  // Set output data rates to 8KHz
  if ( UVOS_ICM42688P_SetAccelODR( UVOS_ICM42688P_ODR8k ) < 0 ) {
    return -3;
  }
  if ( UVOS_ICM42688P_SetGyroODR( UVOS_ICM42688P_ODR8k ) < 0 ) {
    return -3;
  }

  icm42688p_configured = true;
  return 0;
}

/**
 * @brief Configures Gyro, accel and Filter ranges/setings
 * @return 0 if successful, -1 if device has not been initialized
 */
int32_t UVOS_ICM42688P_ConfigureRanges(
  enum uvos_icm42688p_gyro_range gyroRange,
  enum uvos_icm42688p_accel_range accelRange,
  enum uvos_icm42688p_filter filterSetting )
{
  int32_t ret;

  if ( dev == NULL ) {
    return -1;
  }

  // Make sure filters are active
  ret = UVOS_ICM42688P_SetFilters( true, true );
  if ( ret < 0 ) {
    return -2;
  }

  // Sample rate divider, chosen upon digital filtering settings
  // while ( UVOS_ICM42688P_SetReg( UVOS_ICM42688P_SMPLRT_DIV_REG,
  //                                filterSetting == UVOS_ICM42688P_LOWPASS_256_HZ ?
  //                                dev->cfg->Smpl_rate_div_no_dlp : dev->cfg->Smpl_rate_div_dlp ) != 0 ) {
  //   ;
  // }
  // dev->filter = filterSetting;

  // Gyro range
  ret = UVOS_ICM42688P_SetGyroRange( gyroRange );
  if ( ret < 0 ) {
    return -2;
  }
  dev->gyro_range = gyroRange;

  // Set the accel range
  ret = UVOS_ICM42688P_SetAccelRange( accelRange );
  if ( ret < 0 ) {
    return -3;
  }
  dev->accel_range = accelRange;

  return 0;
}

/* sets the accelerometer full scale range to values other than default */
static int32_t UVOS_ICM42688P_SetAccelRange( enum uvos_icm42688p_accel_range range )
{
  // use low speed SPI for register setting
  // _useSPIHS = false;

  // setBank(0);
  UVOS_ICM42688P_SetBank( 0 );

  // read current register value
  // uint8_t reg;
  // if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;
  int32_t ret = UVOS_ICM42688P_GetReg( UB0_REG_ACCEL_CONFIG0 );
  if ( ret < 0 ) {
    return -1;
  }
  uint8_t reg = ( uint8_t )ret;

  // only change FS_SEL in reg
  reg = ( range << 5 ) | ( reg & 0x1F );

  // if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) return -2;
  if ( UVOS_ICM42688P_SetReg( UB0_REG_ACCEL_CONFIG0, reg ) < 0 ) return -2;

  // _accelScale = static_cast<float>(1 << (4 - range)) / 32768.0f;
  // _accelFS = range;

  return 0;
}

/* sets the gyro full scale range to values other than default */
static int32_t UVOS_ICM42688P_SetGyroRange( enum uvos_icm42688p_gyro_range range )
{
  // use low speed SPI for register setting
  // _useSPIHS = false;

  // setBank(0);
  UVOS_ICM42688P_SetBank( 0 );

  // read current register value
  // uint8_t reg;
  // if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;
  int32_t ret = UVOS_ICM42688P_GetReg( UB0_REG_GYRO_CONFIG0 );
  if ( ret < 0 ) {
    return -1;
  }
  uint8_t reg = ( uint8_t )ret;

  // only change FS_SEL in reg
  reg = ( range << 5 ) | ( reg & 0x1F );

  // if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) return -2;
  if ( UVOS_ICM42688P_SetReg( UB0_REG_GYRO_CONFIG0, reg ) < 0 ) return -2;

  // _gyroScale = (2000.0f / static_cast<float>(1 << range)) / 32768.0f;
  // _gyroFS = range;

  return 0;
}

static int32_t UVOS_ICM42688P_SetAccelODR( enum uvos_icm42688p_odr odr )
{
  // use low speed SPI for register setting
  // _useSPIHS = false;

  // setBank(0);
  UVOS_ICM42688P_SetBank( 0 );

  // read current register value
  // uint8_t reg;
  // if ( readRegisters( UB0_REG_ACCEL_CONFIG0, 1, &reg ) < 0 ) return -1;
  int32_t ret = UVOS_ICM42688P_GetReg( UB0_REG_ACCEL_CONFIG0 );
  if ( ret < 0 ) {
    return -1;
  }
  uint8_t reg = ( uint8_t )ret;

  // only change ODR in reg
  reg = odr | ( reg & 0xF0 );

  // if ( writeRegister( UB0_REG_ACCEL_CONFIG0, reg ) < 0 ) return -2;
  if ( UVOS_ICM42688P_SetReg( UB0_REG_ACCEL_CONFIG0, reg ) < 0 ) return -2;

  return 0;
}

static int32_t UVOS_ICM42688P_SetGyroODR( enum uvos_icm42688p_odr odr )
{
  // use low speed SPI for register setting
  // _useSPIHS = false;

  // setBank(0);
  UVOS_ICM42688P_SetBank( 0 );

  // read current register value
  // uint8_t reg;
  // if ( readRegisters( UB0_REG_GYRO_CONFIG0, 1, &reg ) < 0 ) return -1;
  int32_t ret = UVOS_ICM42688P_GetReg( UB0_REG_GYRO_CONFIG0 );
  if ( ret < 0 ) {
    return -1;
  }
  uint8_t reg = ( uint8_t )ret;

  // only change ODR in reg
  reg = odr | ( reg & 0xF0 );

  // if ( writeRegister( UB0_REG_GYRO_CONFIG0, reg ) < 0 ) return -2;
  if ( UVOS_ICM42688P_SetReg( UB0_REG_GYRO_CONFIG0, reg ) < 0 ) return -2;

  return 0;
}

static int32_t UVOS_ICM42688P_SetFilters( bool gyroFilters, bool accFilters )
{
  // if (setBank(1) < 0) return -1;
  UVOS_ICM42688P_SetBank( 1 );

  if ( gyroFilters == true ) {
    if ( UVOS_ICM42688P_SetReg( UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_ENABLE | GYRO_AAF_ENABLE ) < 0 ) {
      return -1;
    }
  } else {
    if ( UVOS_ICM42688P_SetReg( UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE ) < 0 ) {
      return -1;
    }
  }

  // if ( setBank( 2 ) < 0 ) return -4;
  UVOS_ICM42688P_SetBank( 2 );

  if ( accFilters == true ) {
    if ( UVOS_ICM42688P_SetReg( UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE ) < 0 ) {
      return -2;
    }
  } else {
    if ( UVOS_ICM42688P_SetReg( UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE ) < 0 ) {
      return -2;
    }
  }
  // if (setBank(0) < 0) return -7;
  UVOS_ICM42688P_SetBank( 0 );

  return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t UVOS_ICM42688P_ClaimBus( bool fast_spi )
{
  if ( UVOS_ICM42688P_Validate( dev ) != 0 ) {
    return -1;
  }
  if ( UVOS_SPI_ClaimBus( dev->spi_id ) != 0 ) {
    return -2;
  }
  UVOS_ICM42688P_SetSpeed( fast_spi );
  UVOS_SPI_RC_PinSet( dev->spi_id, dev->slave_num, 0 );
  return 0;
}

static void UVOS_ICM42688P_SetSpeed( const bool fast )
{
  if ( fast ) {
    UVOS_SPI_SetClockSpeed( dev->spi_id, dev->cfg->fast_prescaler );
  } else {
    UVOS_SPI_SetClockSpeed( dev->spi_id, dev->cfg->std_prescaler );
  }
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 * @param woken[in,out] If non-NULL, will be set to true if woken was false and a higher priority
 *                      task has is now eligible to run, else unchanged
 */
static int32_t UVOS_ICM42688P_ClaimBusISR( bool *woken, bool fast_spi )
{
  if ( UVOS_ICM42688P_Validate( dev ) != 0 ) {
    return -1;
  }
  if ( UVOS_SPI_ClaimBusISR( dev->spi_id, woken ) != 0 ) {
    return -2;
  }
  UVOS_ICM42688P_SetSpeed( fast_spi );
  UVOS_SPI_RC_PinSet( dev->spi_id, dev->slave_num, 0 );
  return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful
 */
static int32_t UVOS_ICM42688P_ReleaseBus()
{
  if ( UVOS_ICM42688P_Validate( dev ) != 0 ) {
    return -1;
  }
  UVOS_SPI_RC_PinSet( dev->spi_id, dev->slave_num, 1 );
  return UVOS_SPI_ReleaseBus( dev->spi_id );
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful
 * @param woken[in,out] If non-NULL, will be set to true if woken was false and a higher priority
 *                      task has is now eligible to run, else unchanged
 */
static int32_t UVOS_ICM42688P_ReleaseBusISR( bool *woken )
{
  if ( UVOS_ICM42688P_Validate( dev ) != 0 ) {
    return -1;
  }
  UVOS_SPI_RC_PinSet( dev->spi_id, dev->slave_num, 1 );
  return UVOS_SPI_ReleaseBusISR( dev->spi_id, woken );
}

/**
 * @brief Read a register from ICM42688P
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t UVOS_ICM42688P_GetReg( uint8_t reg )
{
  uint8_t data;

  if ( UVOS_ICM42688P_ClaimBus( false ) != 0 ) {
    return -1;
  }

  UVOS_SPI_TransferByte( dev->spi_id, ( 0x80 | reg ) ); // request byte
  data = UVOS_SPI_TransferByte( dev->spi_id, 0 ); // receive response

  UVOS_ICM42688P_ReleaseBus();
  return data;
}

/**
 * @brief Writes one byte to the ICM42688P
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t UVOS_ICM42688P_SetReg( uint8_t reg, uint8_t data )
{
  if ( UVOS_ICM42688P_ClaimBus( false ) != 0 ) {
    return -1;
  }

  if ( UVOS_SPI_TransferByte( dev->spi_id, 0x7f & reg ) != 0 ) {
    UVOS_ICM42688P_ReleaseBus();
    return -2;
  }

  if ( UVOS_SPI_TransferByte( dev->spi_id, data ) != 0 ) {
    UVOS_ICM42688P_ReleaseBus();
    return -3;
  }

  UVOS_ICM42688P_ReleaseBus();

  return 0;
}

/**
 * @brief Perform a dummy read in order to restart interrupt generation
 * \returns 0 if succesful
 */
int32_t UVOS_ICM42688P_DummyReadGyros()
{
  // THIS FUNCTION IS DEPRECATED AND DOES NOT PERFORM A ROTATION
  uint8_t buf[7] = { UB0_REG_GYRO_DATA_X1 | 0x80, 0, 0, 0, 0, 0, 0 };
  uint8_t rec[7];

  if ( UVOS_ICM42688P_ClaimBus( true ) != 0 ) {
    return -1;
  }

  if ( UVOS_SPI_TransferBlock( dev->spi_id, &buf[0], &rec[0], sizeof( buf ), NULL ) < 0 ) {
    return -2;
  }

  UVOS_ICM42688P_ReleaseBus();

  return 0;
}

/*
 * @brief Read the identification bytes from the ICM42688P sensor
 * \return ID read from ICM42688P or -1 if failure
 */
int32_t UVOS_ICM42688P_ReadID()
{
  int32_t icm42688p_id = UVOS_ICM42688P_GetReg( UB0_REG_WHO_AM_I );

  if ( icm42688p_id < 0 ) {
    return -1;
  }
  return icm42688p_id;
}

/**
 * \brief Reads the queue handle
 * \return Handle to the queue or null if invalid device
 */
p_uvos_queue_t UVOS_ICM42688P_GetQueue()
{
  if ( UVOS_ICM42688P_Validate( dev ) != 0 ) {
    return ( p_uvos_queue_t )NULL;
  }

  return dev->queue;
}


static float UVOS_ICM42688P_GetScale()
{
  switch ( dev->gyro_range ) {
  case UVOS_ICM42688P_SCALE_250_DEG:
    return 1.0f / 131.0f;

  case UVOS_ICM42688P_SCALE_500_DEG:
    return 1.0f / 65.5f;

  case UVOS_ICM42688P_SCALE_1000_DEG:
    return 1.0f / 32.8f;

  case UVOS_ICM42688P_SCALE_2000_DEG:
    return 1.0f / 16.4f;
  }
  return 0;
}

static float UVOS_ICM42688P_GetAccelScale()
{
  switch ( dev->accel_range ) {
  case UVOS_ICM42688P_ACCEL_2G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 16384.0f;

  case UVOS_ICM42688P_ACCEL_4G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 8192.0f;

  case UVOS_ICM42688P_ACCEL_8G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 4096.0f;

  case UVOS_ICM42688P_ACCEL_16G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 2048.0f;
  }
  return 0;
}

/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test succeeded
 */
static int32_t UVOS_ICM42688P_Test( void )
{
  /* Verify that ID chip matches */
  int32_t icm42688p_id = UVOS_ICM42688P_ReadID();

  if ( icm42688p_id < 0 ) {
    return -1;
  }

  if ( icm42688p_id != ICM42688P_WHOAMI_VALUE ) {
    return -2;
  }

  return 0;
}

static int32_t UVOS_ICM42688P_SetBank( uint8_t bank )
{
  // if we are already on this bank, bail
  if ( _bank == bank ) return -1;

  _bank = bank;

  return UVOS_ICM42688P_SetReg( REG_BANK_SEL, bank );
}

#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

static void UVOS_ICM42688P_GyroAccOff( void )
{
  UVOS_ICM42688P_SetBank( 0 );

  UVOS_ICM42688P_SetReg( UB0_REG_PWR_MGMT0, ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF );
  /* Per DK-42688-P_SmartMotion_eMD, powering the gyroscope on immediately after powering it off
   * can cause device failure. The gyroscope proof mass can continue vibrating after it has been powered off,
   * and powering it back on immediately can result in unpredictable proof mass movement.
   * After powering the gyroscope off, a period of > 150ms should be allowed to elapse before it is powered back on. */
  UVOS_DELAY_WaituS( 150 * 1000 );
}

// Turn on gyro and acc on in Low Noise mode
static void UVOS_ICM42688P_GyroAccOn( void )
{
  UVOS_ICM42688P_SetBank( 0 );

  UVOS_ICM42688P_SetReg( UB0_REG_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN );

  // Icm42688p gyroscope start-up time before having correct data (accel is shorter)
  UVOS_DELAY_WaituS( ICM42688P_GYRO_STARTUP_TIME_US );
}

static void UVOS_ICM42688P_Reset()
{
  UVOS_ICM42688P_SetBank( 0 );

  UVOS_ICM42688P_SetReg( UB0_REG_DEVICE_CONFIG, 0x01 );

  // Wait ICM42688P_GYRO_STARTUP_TIME_US for soft reset to be effective before any further read
  UVOS_DELAY_WaituS( ICM42688P_GYRO_STARTUP_TIME_US );
}

/**
 * @brief EXTI IRQ Handler.  Read all the data from onboard buffer
 * @return a boleoan to the EXTI IRQ Handler wrapper indicating if a
 *         higher priority task is now eligible to run
 */

bool UVOS_ICM42688P_IRQHandler( void )
{
  uint32_t gyro_read_timestamp = UVOS_DELAY_GetRaw();
  bool woken = false;

  if ( !icm42688p_configured ) {
    return false;
  }

  if ( UVOS_ICM42688P_ReadSensor( &woken ) ) {
    woken |= UVOS_ICM42688P_HandleData( gyro_read_timestamp );
  }

  UVOS_Queue_Send( dev->queue, ( void * )queue_data, 0 );
  return woken;
}

static bool UVOS_ICM42688P_HandleData( uint32_t gyro_read_timestamp )
{
  if ( !queue_data ) {
    return false;
  }

  /* Figure below shows the axis convention used for IMU data and vehicle orientation.
     +X is denoted as out the nose, +Y is out the right side, and +Z is upward.

           +x                   +y
           |                    |
    +-------------+      +-------------+
    |             |      | o           |
    |             |      |             |
    |    +z up    |--+y  |    +z up    |--+x
    |   Vehicle   |      |   ICM42688P   |
    |             |      |             |
    +-------------+      +-------------+

  */

  // Currently we only support rotations on top so switch X/Y accordingly
  switch ( dev->cfg->orientation ) {
  case UVOS_ICM42688P_TOP_0DEG:
    queue_data->sample[0].x = GET_SENSOR_DATA( icm42688p_data, Accel_Y ); // chip +Y
    queue_data->sample[0].y = GET_SENSOR_DATA( icm42688p_data, Accel_X ); // chip +X
    queue_data->sample[1].x = GET_SENSOR_DATA( icm42688p_data, Gyro_Y ); // chip +Y
    queue_data->sample[1].y = GET_SENSOR_DATA( icm42688p_data, Gyro_X ); // chip +X
    break;
  case UVOS_ICM42688P_TOP_90DEG:
    // -1 to bring it back to -32768 +32767 range
    queue_data->sample[0].x = -1 - ( GET_SENSOR_DATA( icm42688p_data, Accel_X ) ); // chip -X
    queue_data->sample[0].y = GET_SENSOR_DATA( icm42688p_data, Accel_Y ); // chip +Y
    queue_data->sample[1].x = -1 - ( GET_SENSOR_DATA( icm42688p_data, Gyro_X ) ); // chip -X
    queue_data->sample[1].y = GET_SENSOR_DATA( icm42688p_data, Gyro_Y ); // chip +Y
    break;
  case UVOS_ICM42688P_TOP_180DEG:
    queue_data->sample[0].x = -1 - ( GET_SENSOR_DATA( icm42688p_data, Accel_Y ) ); // chip -Y
    queue_data->sample[0].y = -1 - ( GET_SENSOR_DATA( icm42688p_data, Accel_X ) ); // chip -X
    queue_data->sample[1].x = -1 - ( GET_SENSOR_DATA( icm42688p_data, Gyro_Y ) ); // chip -Y
    queue_data->sample[1].y = -1 - ( GET_SENSOR_DATA( icm42688p_data, Gyro_X ) ); // chip -X
    break;
  case UVOS_ICM42688P_TOP_270DEG:
    queue_data->sample[0].x = GET_SENSOR_DATA( icm42688p_data, Accel_X ); // chip +X
    queue_data->sample[0].y = -1 - ( GET_SENSOR_DATA( icm42688p_data, Accel_Y ) ); // chip -Y
    queue_data->sample[1].x = GET_SENSOR_DATA( icm42688p_data, Gyro_X ); // chip +X
    queue_data->sample[1].y = -1 - ( GET_SENSOR_DATA( icm42688p_data, Gyro_Y ) ); // chip -Y
    break;
  }
  queue_data->sample[0].z = -1 - ( GET_SENSOR_DATA( icm42688p_data, Accel_Z ) );
  queue_data->sample[1].z = -1 - ( GET_SENSOR_DATA( icm42688p_data, Gyro_Z ) );
  const int16_t temp = GET_SENSOR_DATA( icm42688p_data, Temperature );
  // Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
  queue_data->temperature = 3653 + ( temp * 100 ) / 340;
  queue_data->timestamp   = gyro_read_timestamp;

  // BaseType_t higherPriorityTaskWoken;
  // xQueueSendToBackFromISR( dev->queue, ( void * )queue_data, &higherPriorityTaskWoken );
  // return higherPriorityTaskWoken == pdTRUE;

  // UVOS_Queue_Send( dev->queue, ( void * )queue_data, 0 );
  return true;
}

static bool UVOS_ICM42688P_ReadSensor( bool *woken )
{
  const uint8_t icm42688p_send_buf[1 + UVOS_ICM42688P_SAMPLES_BYTES] = { UVOS_ICM42688P_SENSOR_FIRST_REG | 0x80 };

  if ( UVOS_ICM42688P_ClaimBusISR( woken, true ) != 0 ) {
    return false;
  }
  if ( UVOS_SPI_TransferBlock( dev->spi_id, &icm42688p_send_buf[0], &icm42688p_data.buffer[0], sizeof( icm42688p_data_t ), NULL ) < 0 ) {
    UVOS_ICM42688P_ReleaseBusISR( woken );
    return false;
  }
  UVOS_ICM42688P_ReleaseBusISR( woken );
  return true;
}

// Sensor driver implementation
bool UVOS_ICM42688P_driver_Test( __attribute__( ( unused ) ) uintptr_t context )
{
  return !UVOS_ICM42688P_Test();
}

void UVOS_ICM42688P_driver_Reset( __attribute__( ( unused ) ) uintptr_t context )
{
  UVOS_ICM42688P_DummyReadGyros();
}

void UVOS_ICM42688P_driver_get_scale( float *scales, uint8_t size, __attribute__( ( unused ) ) uintptr_t context )
{
  UVOS_Assert( size >= 2 );
  scales[0] = UVOS_ICM42688P_GetAccelScale();
  scales[1] = UVOS_ICM42688P_GetScale();
}

bool UVOS_ICM42688P_driver_poll( __attribute__( ( unused ) ) uintptr_t context )
{
  const uint8_t icm42688p_send_buf[1 + UVOS_ICM42688P_SAMPLES_BYTES] = { UVOS_ICM42688P_SENSOR_FIRST_REG | 0x80 };

  uint32_t gyro_read_timestamp = UVOS_DELAY_GetRaw();

  if ( UVOS_ICM42688P_ClaimBus( true ) != 0 ) {
    return false;
  }
  if ( UVOS_SPI_TransferBlock( dev->spi_id, &icm42688p_send_buf[0], &icm42688p_data.buffer[0], sizeof( icm42688p_data_t ), NULL ) < 0 ) {
    UVOS_ICM42688P_ReleaseBus();
    return false;
  }
  UVOS_ICM42688P_ReleaseBus();

  return UVOS_ICM42688P_HandleData( gyro_read_timestamp );
}

/**
 * @brief Fetch polled ICM42688P data
 * @return nothing
 * @param data[in,out] If non-NULL, will be set to true if woken was false and a higher priority
 * @param size[in] number of sensor data instances to return
 * @param context[in] driver context, not used
 *     uint32_t   timestamp;       // PIOS_DELAY_GetRaw() time of sensor read
 *     uint16_t   count;           // number of sensor instances
 *     int16_t    temperature;     // Degrees Celsius * 100
 *     Vector3i16 accel_sample[];  // 3x16 bit accel data
 *     Vector3i16 gyro_sample[];   // 3x16 bit gyro data
 */
void UVOS_ICM42688P_driver_fetch( void *data, uint8_t size, __attribute__( ( unused ) ) uintptr_t context )
{
  UVOS_Assert( data );

  if ( !queue_data ) {
    return;
  }
  // void * memcpy ( void * destination, const void * source, size_t num )
  memcpy( data, ( void * )queue_data, SENSOR_DATA_SIZE );
}

p_uvos_queue_t UVOS_ICM42688P_driver_get_queue( __attribute__( ( unused ) ) uintptr_t context )
{
  return dev->queue;
}
#endif /* UVOS_INCLUDE_ICM42688P */
