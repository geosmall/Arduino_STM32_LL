#include "uvos.h"
#include <uvos_mpu6000.h>

#ifdef UVOS_INCLUDE_MPU6000
#include <stdint.h>
#include <uvos_constants.h>
#include <uvos_sensors.h>

/* Global Variables */

enum uvos_mpu6000_dev_magic {
  UVOS_MPU6000_DEV_MAGIC = 0x9da9b3ed,
};

// sensor driver interface
bool UVOS_MPU6000_driver_Test( uintptr_t context );
void UVOS_MPU6000_driver_Reset( uintptr_t context );
void UVOS_MPU6000_driver_get_scale( float *scales, uint8_t size, uintptr_t context );
uvos_queue_ptr_t UVOS_MPU6000_driver_get_queue( uintptr_t context );

const UVOS_SENSORS_Driver UVOS_MPU6000_Driver = {
  .test      = UVOS_MPU6000_driver_Test,
  .poll      = NULL,
  .fetch     = NULL,
  .reset     = UVOS_MPU6000_driver_Reset,
  .get_queue = UVOS_MPU6000_driver_get_queue,
  .get_scale = UVOS_MPU6000_driver_get_scale,
  .is_polled = false,
};
//


struct mpu6000_dev {
  uint32_t spi_id;
  uint32_t slave_num;
  uvos_queue_ptr_t queue;
  const struct uvos_mpu6000_cfg *cfg;
  enum uvos_mpu6000_range gyro_range;
  enum uvos_mpu6000_accel_range accel_range;
  enum uvos_mpu6000_filter filter;
  enum uvos_mpu6000_dev_magic   magic;
};

#define UVOS_MPU6000_SAMPLES_BYTES    14
#define UVOS_MPU6000_SENSOR_FIRST_REG UVOS_MPU6000_ACCEL_X_OUT_MSB

typedef union {
  uint8_t buffer[1 + UVOS_MPU6000_SAMPLES_BYTES];
  struct {
    uint8_t dummy;
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
} mpu6000_data_t;

#define GET_SENSOR_DATA(mpudataptr, sensor) (mpudataptr.data.sensor##_h << 8 | mpudataptr.data.sensor##_l)

// ! Global structure for this device device
static struct mpu6000_dev *dev;
volatile bool mpu6000_configured = false;
static mpu6000_data_t mpu6000_data;
static UVOS_SENSORS_3Axis_SensorsWithTemp *queue_data = 0;
#define SENSOR_COUNT     2
#define SENSOR_DATA_SIZE (sizeof(UVOS_SENSORS_3Axis_SensorsWithTemp) + sizeof(Vector3i16) * SENSOR_COUNT)

// ! Private functions
static struct mpu6000_dev *UVOS_MPU6000_alloc( const struct uvos_mpu6000_cfg *cfg );
static int32_t UVOS_MPU6000_Validate( struct mpu6000_dev *dev );
// static void UVOS_MPU6000_Config( struct uvos_mpu6000_cfg const *cfg );
static int32_t UVOS_MPU6000_Config( struct uvos_mpu6000_cfg const *cfg );
static int32_t UVOS_MPU6000_SetReg( uint8_t address, uint8_t buffer );
static int32_t UVOS_MPU6000_GetReg( uint8_t address );
static void UVOS_MPU6000_SetSpeed( const bool fast );
static bool UVOS_MPU6000_HandleData( uint32_t gyro_read_timestamp );
static bool UVOS_MPU6000_ReadSensor( bool *woken );

static int32_t UVOS_MPU6000_Test( void );

void UVOS_MPU6000_Register()
{
  UVOS_SENSORS_Register( &UVOS_MPU6000_Driver, UVOS_SENSORS_TYPE_3AXIS_GYRO_ACCEL, 0 );
}
/**
 * @brief Allocate a new device
 */
static struct mpu6000_dev *UVOS_MPU6000_alloc( const struct uvos_mpu6000_cfg *cfg )
{
  struct mpu6000_dev *mpu6000_dev;

  mpu6000_dev = ( struct mpu6000_dev * )UVOS_malloc( sizeof( *mpu6000_dev ) );
  UVOS_Assert( mpu6000_dev );

  mpu6000_dev->magic = UVOS_MPU6000_DEV_MAGIC;

  // mpu6000_dev->queue = xQueueCreate( cfg->max_downsample + 1, SENSOR_DATA_SIZE );
  mpu6000_dev->queue = UVOS_Queue_Create( cfg->max_downsample + 1, SENSOR_DATA_SIZE );
  UVOS_Assert( mpu6000_dev->queue );

  queue_data = ( UVOS_SENSORS_3Axis_SensorsWithTemp * )UVOS_malloc( SENSOR_DATA_SIZE );
  UVOS_Assert( queue_data );
  queue_data->count = SENSOR_COUNT;
  return mpu6000_dev;
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t UVOS_MPU6000_Validate( struct mpu6000_dev *vdev )
{
  if ( vdev == NULL ) {
    return -1;
  }
  if ( vdev->magic != UVOS_MPU6000_DEV_MAGIC ) {
    return -2;
  }
  if ( vdev->spi_id == 0 ) {
    return -3;
  }
  return 0;
}

/**
 * @brief Initialize the MPU6000 3-axis gyro sensor.
 * @return 0 for success, -1 for failure
 */
int32_t UVOS_MPU6000_Init( uint32_t spi_id, uint32_t slave_num, const struct uvos_mpu6000_cfg *cfg )
{
  dev = UVOS_MPU6000_alloc( cfg );
  if ( dev == NULL ) {
    return -1;
  }

  dev->spi_id    = spi_id;
  dev->slave_num = slave_num;
  dev->cfg = cfg;

  /* Configure the MPU6000 Sensor */
  // UVOS_MPU6000_Config( cfg );
  if ( UVOS_MPU6000_Config( cfg ) != 0 ) {
    return -1;
  }

  /* Set up EXTI line */
  UVOS_EXTI_Init( cfg->exti_cfg );
  return 0;
}

#if 0 // gls

/**
 * @brief Initialize the MPU6000 3-axis gyro sensor
 * \return none
 * \param[in] UVOS_MPU6000_ConfigTypeDef struct to be used to configure sensor.
 *
 */
static void UVOS_MPU6000_Config( struct uvos_mpu6000_cfg const *cfg )
{
  UVOS_MPU6000_Test();

  // Reset chip
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_PWR_MGMT_REG, UVOS_MPU6000_PWRMGMT_IMU_RST ) != 0 ) {
    ;
  }
  UVOS_DELAY_WaitmS( 50 );

  // Reset chip and fifo
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_USER_CTRL_REG,
                               UVOS_MPU6000_USERCTL_GYRO_RST |
                               UVOS_MPU6000_USERCTL_SIG_COND |
                               UVOS_MPU6000_USERCTL_FIFO_RST ) != 0 ) {
    ;
  }

  // Wait for reset to finish
  while ( UVOS_MPU6000_GetReg( UVOS_MPU6000_USER_CTRL_REG ) &
          ( UVOS_MPU6000_USERCTL_GYRO_RST |
            UVOS_MPU6000_USERCTL_SIG_COND |
            UVOS_MPU6000_USERCTL_FIFO_RST ) ) {
    ;
  }
  UVOS_DELAY_WaitmS( 10 );
  // Power management configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_PWR_MGMT_REG, cfg->Pwr_mgmt_clk ) != 0 ) {
    ;
  }

  // Interrupt configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_INT_CFG_REG, cfg->interrupt_cfg ) != 0 ) {
    ;
  }

  // Interrupt configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_INT_EN_REG, cfg->interrupt_en ) != 0 ) {
    ;
  }

  // FIFO storage
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_FIFO_EN_REG, cfg->Fifo_store ) != 0 ) {
    ;
  }
  UVOS_MPU6000_ConfigureRanges( cfg->gyro_range, cfg->accel_range, cfg->filter );
  // Interrupt configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_USER_CTRL_REG, cfg->User_ctl ) != 0 ) {
    ;
  }

  // Interrupt configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_PWR_MGMT_REG, cfg->Pwr_mgmt_clk ) != 0 ) {
    ;
  }

  // Interrupt configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_INT_CFG_REG, cfg->interrupt_cfg ) != 0 ) {
    ;
  }

  // Interrupt configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_INT_EN_REG, cfg->interrupt_en ) != 0 ) {
    ;
  }
  if ( ( UVOS_MPU6000_GetReg( UVOS_MPU6000_INT_EN_REG ) ) != cfg->interrupt_en ) {
    return;
  }

  mpu6000_configured = true;
}

#endif // gls

/**
 * @brief Initialize the MPU6000 3-axis gyro sensor
 * \return 0 if successful, negative number on error
 * \param[in] UVOS_MPU6000_ConfigTypeDef struct to be used to configure sensor.
 * See - https://github.com/ArduPilot/ardupilot/blob/3388e6f25d4c0955d3af75888a894a9e4f64c036/libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp#L996
 */
static int32_t UVOS_MPU6000_Config( struct uvos_mpu6000_cfg const *cfg )
{
  // Last status from register user control
  uint8_t _last_stat_user_ctrl;

  if ( UVOS_MPU6000_Test() ) {
    return -1;
  }

  // MPU POWER ON
  if ( UVOS_MPU6000_SetReg( MPUREG_PWR_MGMT_1, 0x01 ) ) {
    return -2;
  }
  // MPU Gyro and Accel ON
  if ( UVOS_MPU6000_SetReg( MPUREG_PWR_MGMT_2, 0x00 ) ) {
    return -2;
  }
  UVOS_DELAY_WaituS( 10000 );

  // Chip reset
  uint8_t tries;
  for ( tries = 0; tries < UVOS_MPU_MAX_TRIES; tries++ ) {
    _last_stat_user_ctrl = UVOS_MPU6000_GetReg( MPUREG_USER_CTRL );

    /* First disable the master I2C to avoid hanging the slaves on the
     * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
     * is used */
    if ( _last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN ) {
      _last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
      if ( UVOS_MPU6000_SetReg( MPUREG_USER_CTRL, _last_stat_user_ctrl ) ) {
        return -3;
      }
      UVOS_DELAY_WaituS( 10000 );
    }

    /* reset device, wait 100mS per datasheet */
    UVOS_MPU6000_SetReg( MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET );
    UVOS_DELAY_WaituS( 100000 );

    /* reset signal path as recommended in datasheet */
    if ( UVOS_MPU6000_SetReg( MPUREG_SIGNAL_PATH_RESET,
                              BIT_SIGNAL_PATH_RESET_TEMP_RESET | BIT_SIGNAL_PATH_RESET_ACCEL_RESET | BIT_SIGNAL_PATH_RESET_GYRO_RESET ) ) {
      return -3;
    }
    UVOS_DELAY_WaituS( 100000 );

    /* Disable I2C bus when SPI selected (Recommended in Datasheet to be
     * done just after the device is reset) */
    _last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
    if ( UVOS_MPU6000_SetReg( MPUREG_USER_CTRL, _last_stat_user_ctrl ) ) {
      return -3;
    }

    // Wake up device and select GyroZ clock. Note that the  Invensense device
    // starts up in sleep mode, and it can take some time for it to come out of sleep
    UVOS_MPU6000_SetReg( MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO );
    UVOS_DELAY_WaituS( 5000 );

    // check that device has woken up
    if ( UVOS_MPU6000_GetReg( MPUREG_PWR_MGMT_1 ) == BIT_PWR_MGMT_1_CLK_ZGYRO ) {
      break;
    }

    /* check that device has data available */
    UVOS_DELAY_WaituS( 10000 );
    uint8_t status = UVOS_MPU6000_GetReg( MPUREG_INT_STATUS );
    if ( ( status & BIT_RAW_RDY_INT ) != 0 ) {
      break;
    }
  }

  if ( tries >= UVOS_MPU_MAX_TRIES ) {
    return -4;
  }

  // Power management configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_PWR_MGMT_REG, cfg->Pwr_mgmt_clk ) != 0 ) {
    ;
  }

  // Interrupt configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_INT_CFG_REG, cfg->interrupt_cfg ) != 0 ) {
    ;
  }

  // Interrupt enable
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_INT_EN_REG, cfg->interrupt_en ) != 0 ) {
    ;
  }

  // FIFO storage
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_FIFO_EN_REG, cfg->Fifo_store ) != 0 ) {
    ;
  }

  UVOS_MPU6000_ConfigureRanges( cfg->gyro_range, cfg->accel_range, cfg->filter );

  // User Control Register configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_USER_CTRL_REG, cfg->User_ctl ) != 0 ) {
    ;
  }

  // Power management configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_PWR_MGMT_REG, cfg->Pwr_mgmt_clk ) != 0 ) {
    ;
  }

  // Interrupt configuration
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_INT_CFG_REG, cfg->interrupt_cfg ) != 0 ) {
    ;
  }

  // Interrupt enable and verify
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_INT_EN_REG, cfg->interrupt_en ) != 0 ) {
    ;
  }
  if ( ( UVOS_MPU6000_GetReg( UVOS_MPU6000_INT_EN_REG ) ) != cfg->interrupt_en ) {
    return -5;
  }

  mpu6000_configured = true;
  return 0;
}

/**
 * @brief Configures Gyro, accel and Filter ranges/setings
 * @return 0 if successful, -1 if device has not been initialized
 */
int32_t UVOS_MPU6000_ConfigureRanges(
  enum uvos_mpu6000_range gyroRange,
  enum uvos_mpu6000_accel_range accelRange,
  enum uvos_mpu6000_filter filterSetting )
{
  if ( dev == NULL ) {
    return -1;
  }

  // update filter settings
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_DLPF_CFG_REG, filterSetting ) != 0 ) {
    ;
  }

  // Sample rate divider, chosen upon digital filtering settings
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_SMPLRT_DIV_REG,
                               filterSetting == UVOS_MPU6000_LOWPASS_256_HZ ?
                               dev->cfg->Smpl_rate_div_no_dlp : dev->cfg->Smpl_rate_div_dlp ) != 0 ) {
    ;
  }

  dev->filter = filterSetting;

  // Gyro range
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_GYRO_CFG_REG, gyroRange ) != 0 ) {
    ;
  }

  dev->gyro_range = gyroRange;
  // Set the accel range
  while ( UVOS_MPU6000_SetReg( UVOS_MPU6000_ACCEL_CFG_REG, accelRange ) != 0 ) {
    ;
  }

  dev->accel_range = accelRange;
  return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t UVOS_MPU6000_ClaimBus( bool fast_spi )
{
  if ( UVOS_MPU6000_Validate( dev ) != 0 ) {
    return -1;
  }
  if ( UVOS_SPI_ClaimBus( dev->spi_id ) != 0 ) {
    return -2;
  }
  UVOS_MPU6000_SetSpeed( fast_spi );
  UVOS_SPI_RC_PinSet( dev->spi_id, dev->slave_num, 0 );
  return 0;
}


static void UVOS_MPU6000_SetSpeed( const bool fast )
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
static int32_t UVOS_MPU6000_ClaimBusISR( bool *woken, bool fast_spi )
{
  if ( UVOS_MPU6000_Validate( dev ) != 0 ) {
    return -1;
  }
  if ( UVOS_SPI_ClaimBusISR( dev->spi_id, woken ) != 0 ) {
    return -2;
  }
  UVOS_MPU6000_SetSpeed( fast_spi );
  UVOS_SPI_RC_PinSet( dev->spi_id, dev->slave_num, 0 );
  return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful
 */
static int32_t UVOS_MPU6000_ReleaseBus()
{
  if ( UVOS_MPU6000_Validate( dev ) != 0 ) {
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
static int32_t UVOS_MPU6000_ReleaseBusISR( bool *woken )
{
  if ( UVOS_MPU6000_Validate( dev ) != 0 ) {
    return -1;
  }
  UVOS_SPI_RC_PinSet( dev->spi_id, dev->slave_num, 1 );
  return UVOS_SPI_ReleaseBusISR( dev->spi_id, woken );
}

/**
 * @brief Read a register from MPU6000
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t UVOS_MPU6000_GetReg( uint8_t reg )
{
  uint8_t data;

  if ( UVOS_MPU6000_ClaimBus( false ) != 0 ) {
    return -1;
  }

  UVOS_SPI_TransferByte( dev->spi_id, ( 0x80 | reg ) ); // request byte
  data = UVOS_SPI_TransferByte( dev->spi_id, 0 ); // receive response

  UVOS_MPU6000_ReleaseBus();
  return data;
}

/**
 * @brief Writes one byte to the MPU6000
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t UVOS_MPU6000_SetReg( uint8_t reg, uint8_t data )
{
  if ( UVOS_MPU6000_ClaimBus( false ) != 0 ) {
    return -1;
  }

  if ( UVOS_SPI_TransferByte( dev->spi_id, 0x7f & reg ) != 0 ) {
    UVOS_MPU6000_ReleaseBus();
    return -2;
  }

  if ( UVOS_SPI_TransferByte( dev->spi_id, data ) != 0 ) {
    UVOS_MPU6000_ReleaseBus();
    return -3;
  }

  UVOS_MPU6000_ReleaseBus();

  return 0;
}

/**
 * @brief Perform a dummy read in order to restart interrupt generation
 * \returns 0 if succesful
 */
int32_t UVOS_MPU6000_DummyReadGyros()
{
  // THIS FUNCTION IS DEPRECATED AND DOES NOT PERFORM A ROTATION
  uint8_t buf[7] = { UVOS_MPU6000_GYRO_X_OUT_MSB | 0x80, 0, 0, 0, 0, 0, 0 };
  uint8_t rec[7];

  if ( UVOS_MPU6000_ClaimBus( true ) != 0 ) {
    return -1;
  }

  if ( UVOS_SPI_TransferBlock( dev->spi_id, &buf[0], &rec[0], sizeof( buf ), NULL ) < 0 ) {
    return -2;
  }

  UVOS_MPU6000_ReleaseBus();

  return 0;
}

/*
 * @brief Read the identification bytes from the MPU6000 sensor
 * \return ID read from MPU6000 or -1 if failure
 */
int32_t UVOS_MPU6000_ReadID()
{
  int32_t mpu6000_id = UVOS_MPU6000_GetReg( UVOS_MPU6000_WHOAMI );

  if ( mpu6000_id < 0 ) {
    return -1;
  }
  return mpu6000_id;
}

/**
 * \brief Reads the queue handle
 * \return Handle to the queue or null if invalid device
 */
uvos_queue_ptr_t UVOS_MPU6000_GetQueue()
{
  if ( UVOS_MPU6000_Validate( dev ) != 0 ) {
    return ( uvos_queue_ptr_t )NULL;
  }

  return dev->queue;
}


static float UVOS_MPU6000_GetScale()
{
  switch ( dev->gyro_range ) {
  case UVOS_MPU6000_SCALE_250_DEG:
    return 1.0f / 131.0f;

  case UVOS_MPU6000_SCALE_500_DEG:
    return 1.0f / 65.5f;

  case UVOS_MPU6000_SCALE_1000_DEG:
    return 1.0f / 32.8f;

  case UVOS_MPU6000_SCALE_2000_DEG:
    return 1.0f / 16.4f;
  }
  return 0;
}

static float UVOS_MPU6000_GetAccelScale()
{
  switch ( dev->accel_range ) {
  case UVOS_MPU6000_ACCEL_2G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 16384.0f;

  case UVOS_MPU6000_ACCEL_4G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 8192.0f;

  case UVOS_MPU6000_ACCEL_8G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 4096.0f;

  case UVOS_MPU6000_ACCEL_16G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 2048.0f;
  }
  return 0;
}

/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test succeeded
 */
static int32_t UVOS_MPU6000_Test( void )
{
  /* Verify that ID matches (MPU6000 ID is 0x69) */
  int32_t mpu6000_id = UVOS_MPU6000_ReadID();

  if ( mpu6000_id < 0 ) {
    return -1;
  }

  if ( mpu6000_id != 0x68 ) {
    return -2;
  }

  return 0;
}

/**
 * @brief EXTI IRQ Handler.  Read all the data from onboard buffer
 * @return a boleoan to the EXTI IRQ Handler wrapper indicating if a
 *         higher priority task is now eligible to run
 */

bool UVOS_MPU6000_IRQHandler( void )
{
  uint32_t gyro_read_timestamp = UVOS_DELAY_GetRaw();
  bool woken = false;

  if ( !mpu6000_configured ) {
    return false;
  }

  if ( UVOS_MPU6000_ReadSensor( &woken ) ) {
    woken |= UVOS_MPU6000_HandleData( gyro_read_timestamp );
  }

  return woken;
}

static bool UVOS_MPU6000_HandleData( uint32_t gyro_read_timestamp )
{
  if ( !queue_data ) {
    return false;
  }

  // Rotate the sensor to OP convention.  The datasheet defines X as towards the right
  // and Y as forward.  OP convention transposes this.  Also the Z is defined negatively
  // to our convention

  // Currently we only support rotations on top so switch X/Y accordingly
  switch ( dev->cfg->orientation ) {
  case UVOS_MPU6000_TOP_0DEG:
    queue_data->sample[0].y = GET_SENSOR_DATA( mpu6000_data, Accel_X ); // chip X
    queue_data->sample[0].x = GET_SENSOR_DATA( mpu6000_data, Accel_Y ); // chip Y
    queue_data->sample[1].y = GET_SENSOR_DATA( mpu6000_data, Gyro_X ); // chip X
    queue_data->sample[1].x = GET_SENSOR_DATA( mpu6000_data, Gyro_Y ); // chip Y
    break;
  case UVOS_MPU6000_TOP_90DEG:
    // -1 to bring it back to -32768 +32767 range
    queue_data->sample[0].y = -1 - ( GET_SENSOR_DATA( mpu6000_data, Accel_Y ) ); // chip Y
    queue_data->sample[0].x = GET_SENSOR_DATA( mpu6000_data, Accel_X ); // chip X
    queue_data->sample[1].y = -1 - ( GET_SENSOR_DATA( mpu6000_data, Gyro_Y ) ); // chip Y
    queue_data->sample[1].x = GET_SENSOR_DATA( mpu6000_data, Gyro_X ); // chip X
    break;
  case UVOS_MPU6000_TOP_180DEG:
    queue_data->sample[0].y = -1 - ( GET_SENSOR_DATA( mpu6000_data, Accel_X ) ); // chip X
    queue_data->sample[0].x = -1 - ( GET_SENSOR_DATA( mpu6000_data, Accel_Y ) ); // chip Y
    queue_data->sample[1].y = -1 - ( GET_SENSOR_DATA( mpu6000_data, Gyro_X ) ); // chip X
    queue_data->sample[1].x = -1 - ( GET_SENSOR_DATA( mpu6000_data, Gyro_Y ) ); // chip Y
    break;
  case UVOS_MPU6000_TOP_270DEG:
    queue_data->sample[0].y = GET_SENSOR_DATA( mpu6000_data, Accel_Y ); // chip Y
    queue_data->sample[0].x = -1 - ( GET_SENSOR_DATA( mpu6000_data, Accel_X ) ); // chip X
    queue_data->sample[1].y = GET_SENSOR_DATA( mpu6000_data, Gyro_Y ); // chip Y
    queue_data->sample[1].x = -1 - ( GET_SENSOR_DATA( mpu6000_data, Gyro_X ) ); // chip X
    break;
  }
  queue_data->sample[0].z = -1 - ( GET_SENSOR_DATA( mpu6000_data, Accel_Z ) );
  queue_data->sample[1].z = -1 - ( GET_SENSOR_DATA( mpu6000_data, Gyro_Z ) );
  const int16_t temp = GET_SENSOR_DATA( mpu6000_data, Temperature );
  // Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
  queue_data->temperature = 3653 + ( temp * 100 ) / 340;
  queue_data->timestamp   = gyro_read_timestamp;

  // BaseType_t higherPriorityTaskWoken;
  // xQueueSendToBackFromISR( dev->queue, ( void * )queue_data, &higherPriorityTaskWoken );
  // return higherPriorityTaskWoken == pdTRUE;

  UVOS_Queue_Send( dev->queue, ( void * )queue_data, 0 );
  return true;
}

static bool UVOS_MPU6000_ReadSensor( bool *woken )
{
  const uint8_t mpu6000_send_buf[1 + UVOS_MPU6000_SAMPLES_BYTES] = { UVOS_MPU6000_SENSOR_FIRST_REG | 0x80 };

  if ( UVOS_MPU6000_ClaimBusISR( woken, true ) != 0 ) {
    return false;
  }
  if ( UVOS_SPI_TransferBlock( dev->spi_id, &mpu6000_send_buf[0], &mpu6000_data.buffer[0], sizeof( mpu6000_data_t ), NULL ) < 0 ) {
    UVOS_MPU6000_ReleaseBusISR( woken );
    return false;
  }
  UVOS_MPU6000_ReleaseBusISR( woken );
  return true;
}

// Sensor driver implementation
bool UVOS_MPU6000_driver_Test( __attribute__( ( unused ) ) uintptr_t context )
{
  return !UVOS_MPU6000_Test();
}

void UVOS_MPU6000_driver_Reset( __attribute__( ( unused ) ) uintptr_t context )
{
  UVOS_MPU6000_DummyReadGyros();
}

void UVOS_MPU6000_driver_get_scale( float *scales, uint8_t size, __attribute__( ( unused ) ) uintptr_t contet )
{
  UVOS_Assert( size >= 2 );
  scales[0] = UVOS_MPU6000_GetAccelScale();
  scales[1] = UVOS_MPU6000_GetScale();
}

uvos_queue_ptr_t UVOS_MPU6000_driver_get_queue( __attribute__( ( unused ) ) uintptr_t context )
{
  return dev->queue;
}
#endif /* UVOS_INCLUDE_MPU6000 */

/**
 * @}
 * @}
 */
