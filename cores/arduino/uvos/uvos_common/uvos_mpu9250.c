#include "uvos.h"
#include <uvos_mpu9250.h>

#ifdef UVOS_INCLUDE_MPU9250
#include <stdint.h>
#include <uvos_constants.h>
#include <uvos_sensors.h>
/* Global Variables */

enum uvos_mpu9250_dev_magic {
  UVOS_MPU9250_DEV_MAGIC = 0x9da9b3ed,
};

struct mpu9250_dev {
  uint32_t spi_id;
  uint32_t slave_num;
  p_uvos_queue_t queue;
  const struct uvos_mpu9250_cfg *cfg;
  enum uvos_mpu9250_range gyro_range;
  enum uvos_mpu9250_accel_range accel_range;
  enum uvos_mpu9250_filter filter;
  enum uvos_mpu9250_dev_magic   magic;
  float mag_sens_adj[UVOS_MPU9250_MAG_ASA_NB_BYTE];
};

#ifdef UVOS_MPU9250_ACCEL
#define UVOS_MPU9250_ACCEL_SAMPLES_BYTES (6)
#else
#define UVOS_MPU9250_ACCEL_SAMPLES_BYTES (0)
#endif

#ifdef UVOS_MPU9250_MAG
#define UVOS_MPU9250_MAG_SAMPLES_BYTES   (8)
#else
#define UVOS_MPU9250_MAG_SAMPLES_BYTES   (0)
#endif

#define UVOS_MPU9250_GYRO_SAMPLES_BYTES  (6)
#define UVOS_MPU9250_TEMP_SAMPLES_BYTES  (2)

#define UVOS_MPU9250_SAMPLES_BYTES \
    (UVOS_MPU9250_ACCEL_SAMPLES_BYTES + \
     UVOS_MPU9250_GYRO_SAMPLES_BYTES + \
     UVOS_MPU9250_TEMP_SAMPLES_BYTES + \
     UVOS_MPU9250_MAG_SAMPLES_BYTES)

#ifdef UVOS_MPU9250_ACCEL
#define UVOS_MPU9250_SENSOR_FIRST_REG    UVOS_MPU9250_ACCEL_X_OUT_MSB
#else
#define UVOS_MPU9250_SENSOR_FIRST_REG    UVOS_MPU9250_TEMP_OUT_MSB
#endif

#if defined(UVOS_MPU9250_MAG) && !defined(UVOS_MPU9250_ACCEL)
#error ERROR: UVOS_MPU9250_ACCEL not defined! THIS CONFIGURATION IS NOT SUPPORTED
#endif

typedef union {
  uint8_t buffer[2 + UVOS_MPU9250_SAMPLES_BYTES];
  struct {
    uint8_t dummy;
#ifdef UVOS_MPU9250_ACCEL
    uint8_t Accel_X_h;
    uint8_t Accel_X_l;
    uint8_t Accel_Y_h;
    uint8_t Accel_Y_l;
    uint8_t Accel_Z_h;
    uint8_t Accel_Z_l;
#endif
    uint8_t Temperature_h;
    uint8_t Temperature_l;
    uint8_t Gyro_X_h;
    uint8_t Gyro_X_l;
    uint8_t Gyro_Y_h;
    uint8_t Gyro_Y_l;
    uint8_t Gyro_Z_h;
    uint8_t Gyro_Z_l;
#ifdef UVOS_MPU9250_MAG
    uint8_t st1;
    uint8_t Mag_X_l;
    uint8_t Mag_X_h;
    uint8_t Mag_Y_l;
    uint8_t Mag_Y_h;
    uint8_t Mag_Z_l;
    uint8_t Mag_Z_h;
    uint8_t st2;
#endif
  } data;
} __attribute__( ( __packed__ ) ) mpu9250_data_t;

#define GET_SENSOR_DATA(mpudataptr, sensor) (mpudataptr.data.sensor##_h << 8 | mpudataptr.data.sensor##_l)

static UVOS_SENSORS_3Axis_SensorsWithTemp *queue_data = 0;
static UVOS_SENSORS_3Axis_SensorsWithTemp *mag_data   = 0;
static volatile bool mag_ready = false;
#define SENSOR_COUNT         2
#define SENSOR_DATA_SIZE     (sizeof(UVOS_SENSORS_3Axis_SensorsWithTemp) + sizeof(Vector3i16) * SENSOR_COUNT)
#define MAG_SENSOR_DATA_SIZE (sizeof(UVOS_SENSORS_3Axis_SensorsWithTemp) + sizeof(Vector3i16))
// ! Global structure for this device device
static struct mpu9250_dev *dev;
volatile bool mpu9250_configured = false;
static mpu9250_data_t mpu9250_data;

// ! Private functions
static struct mpu9250_dev *UVOS_MPU9250_alloc( const struct uvos_mpu9250_cfg *cfg );
static int32_t UVOS_MPU9250_Validate( struct mpu9250_dev *dev );
static void UVOS_MPU9250_Config( struct uvos_mpu9250_cfg const *cfg );
static int32_t UVOS_MPU9250_SetReg( uint8_t address, uint8_t buffer );
static int32_t UVOS_MPU9250_GetReg( uint8_t address );
static void UVOS_MPU9250_SetSpeed( const bool fast );
static bool UVOS_MPU9250_HandleData( uint32_t gyro_read_timestamp );
static bool UVOS_MPU9250_ReadSensor( bool *woken );
static int32_t UVOS_MPU9250_Test( void );
#if defined(UVOS_MPU9250_MAG)
static int32_t UVOS_MPU9250_Mag_Test( void );
static int32_t UVOS_MPU9250_Mag_Init( void );
#endif

/* Driver Framework interfaces */
// Gyro/accel interface
bool UVOS_MPU9250_Main_driver_Test( uintptr_t context );
void UVOS_MPU9250_Main_driver_Reset( uintptr_t context );
void UVOS_MPU9250_Main_driver_get_scale( float *scales, uint8_t size, uintptr_t context );
p_uvos_queue_t UVOS_MPU9250_Main_driver_get_queue( uintptr_t context );

const UVOS_SENSORS_Driver UVOS_MPU9250_Main_Driver = {
  .test      = UVOS_MPU9250_Main_driver_Test,
  .poll      = NULL,
  .fetch     = NULL,
  .reset     = UVOS_MPU9250_Main_driver_Reset,
  .get_queue = UVOS_MPU9250_Main_driver_get_queue,
  .get_scale = UVOS_MPU9250_Main_driver_get_scale,
  .is_polled = false,
};

// mag sensor interface
bool UVOS_MPU9250_Mag_driver_Test( uintptr_t context );
void UVOS_MPU9250_Mag_driver_Reset( uintptr_t context );
void UVOS_MPU9250_Mag_driver_get_scale( float *scales, uint8_t size, uintptr_t context );
void UVOS_MPU9250_Mag_driver_fetch( void *, uint8_t size, uintptr_t context );
bool UVOS_MPU9250_Mag_driver_poll( uintptr_t context );

const UVOS_SENSORS_Driver UVOS_MPU9250_Mag_Driver = {
  .test      = UVOS_MPU9250_Mag_driver_Test,
  .poll      = UVOS_MPU9250_Mag_driver_poll,
  .fetch     = UVOS_MPU9250_Mag_driver_fetch,
  .reset     = UVOS_MPU9250_Mag_driver_Reset,
  .get_queue = NULL,
  .get_scale = UVOS_MPU9250_Mag_driver_get_scale,
  .is_polled = true,
};

void UVOS_MPU9250_MainRegister()
{
  UVOS_SENSORS_Register( &UVOS_MPU9250_Main_Driver, UVOS_SENSORS_TYPE_3AXIS_GYRO_ACCEL, 0 );
}

void UVOS_MPU9250_MagRegister()
{
  UVOS_SENSORS_Register( &UVOS_MPU9250_Mag_Driver, UVOS_SENSORS_TYPE_3AXIS_MAG, 0 );
}
/**
 * @brief Allocate a new device
 */
static struct mpu9250_dev *UVOS_MPU9250_alloc( const struct uvos_mpu9250_cfg *cfg )
{
  struct mpu9250_dev *mpu9250_dev;

  mpu9250_dev = ( struct mpu9250_dev * )UVOS_malloc( sizeof( *mpu9250_dev ) );
  UVOS_Assert( mpu9250_dev );

  mpu9250_dev->magic = UVOS_MPU9250_DEV_MAGIC;

  // mpu9250_dev->queue = xQueueCreate( cfg->max_downsample + 1, SENSOR_DATA_SIZE );
  mpu9250_dev->queue = UVOS_Queue_Create( cfg->max_downsample + 1, SENSOR_DATA_SIZE );
  UVOS_Assert( mpu9250_dev->queue );

  queue_data = ( UVOS_SENSORS_3Axis_SensorsWithTemp * )UVOS_malloc( SENSOR_DATA_SIZE );
  UVOS_Assert( queue_data );

  queue_data->count = SENSOR_COUNT;

  mag_data = ( UVOS_SENSORS_3Axis_SensorsWithTemp * )UVOS_malloc( MAG_SENSOR_DATA_SIZE );
  mag_data->count   = 1;
  UVOS_Assert( mag_data );
  return mpu9250_dev;
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t UVOS_MPU9250_Validate( struct mpu9250_dev *vdev )
{
  if ( vdev == NULL ) {
    return -1;
  }
  if ( vdev->magic != UVOS_MPU9250_DEV_MAGIC ) {
    return -2;
  }
  if ( vdev->spi_id == 0 ) {
    return -3;
  }
  return 0;
}

/**
 * @brief Initialize the MPU9250 3-axis gyro sensor.
 * @return 0 for success, -1 for failure
 */
int32_t UVOS_MPU9250_Init( uint32_t spi_id, uint32_t slave_num, const struct uvos_mpu9250_cfg *cfg )
{
  dev = UVOS_MPU9250_alloc( cfg );
  if ( dev == NULL ) {
    return -1;
  }

  dev->spi_id    = spi_id;
  dev->slave_num = slave_num;
  dev->cfg = cfg;

  /* Configure the MPU9250 Sensor */
  UVOS_MPU9250_Config( cfg );

  /* Set up EXTI line */
  UVOS_EXTI_Init( cfg->exti_cfg );
  return 0;
}

/**
 * @brief Initialize the MPU9250 3-axis gyro sensor
 * \return none
 * \param[in] UVOS_MPU9250_ConfigTypeDef struct to be used to configure sensor.
 *
 */
static void UVOS_MPU9250_Config( struct uvos_mpu9250_cfg const *cfg )
{
  uint8_t power;

  while ( UVOS_MPU9250_Test() != 0 ) {
    ;
  }

  // Reset chip
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_PWR_MGMT_REG, UVOS_MPU9250_PWRMGMT_IMU_RST ) != 0 ) {
    ;
  }

  UVOS_DELAY_WaitmS( 100 );

  // Wake up the chip
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_PWR_MGMT_REG, 0 ) != 0 ) {
    ;
  }
  // Reset sensors and fifo
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_USER_CTRL_REG,
                               UVOS_MPU9250_USERCTL_DIS_I2C |
                               UVOS_MPU9250_USERCTL_SIG_COND ) != 0 ) {
    ;
  }
  UVOS_DELAY_WaitmS( 100 );

  // Power management configuration
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_PWR_MGMT_REG, cfg->Pwr_mgmt_clk ) != 0 ) {
    ;
  }

  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_USER_CTRL_REG, cfg->User_ctl ) != 0 ) {
    ;
  }

  // FIFO storage by default, do not include accelerometer and external sense data.
  power = UVOS_MPU9250_PWRMGMT2_DISABLE_ACCEL;

#if defined(UVOS_MPU9250_ACCEL)

  power &= ~UVOS_MPU9250_PWRMGMT2_DISABLE_ACCEL;
#endif

  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_FIFO_EN_REG, cfg->Fifo_store ) != 0 ) {
    ;
  }
  UVOS_MPU9250_SetReg( UVOS_MPU9250_PWR_MGMT2_REG, power );

#if defined(UVOS_MPU9250_ACCEL)
  UVOS_MPU9250_ConfigureRanges( cfg->gyro_range, cfg->accel_range, cfg->filter );
#endif

  // Interrupt configuration
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_INT_CFG_REG, cfg->interrupt_cfg ) != 0 ) {
    ;
  }

#ifdef UVOS_MPU9250_MAG
  UVOS_MPU9250_Mag_Init();
#endif

  // Interrupt enable
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_INT_EN_REG, cfg->interrupt_en ) != 0 ) {
    ;
  }
  if ( ( UVOS_MPU9250_GetReg( UVOS_MPU9250_INT_EN_REG ) ) != cfg->interrupt_en ) {
    return;
  }

  UVOS_MPU9250_GetReg( UVOS_MPU9250_INT_STATUS_REG );

  mpu9250_configured = true;
}
/**
 * @brief Configures Gyro, accel and Filter ranges/setings
 * @return 0 if successful, -1 if device has not been initialized
 */
int32_t UVOS_MPU9250_ConfigureRanges(
  enum uvos_mpu9250_range gyroRange,
  enum uvos_mpu9250_accel_range accelRange,
  enum uvos_mpu9250_filter filterSetting )
{
  if ( dev == NULL ) {
    return -1;
  }

  // update filter settings
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_DLPF_CFG_REG, filterSetting ) != 0 ) {
    ;
  }

  // Sample rate divider, chosen upon digital filtering settings
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_SMPLRT_DIV_REG,
                               filterSetting == UVOS_MPU9250_LOWPASS_256_HZ ?
                               dev->cfg->Smpl_rate_div_no_dlp : dev->cfg->Smpl_rate_div_dlp ) != 0 ) {
    ;
  }

  dev->filter = filterSetting;

  // Gyro range
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_GYRO_CFG_REG, gyroRange ) != 0 ) {
    ;
  }

  dev->gyro_range = gyroRange;
#if defined(UVOS_MPU9250_ACCEL)
  // Set the accel range
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_ACCEL_CFG_REG, accelRange ) != 0 ) {
    ;
  }

  dev->accel_range = accelRange;
#endif
  return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t UVOS_MPU9250_ClaimBus( bool fast_spi )
{
  if ( UVOS_MPU9250_Validate( dev ) != 0 ) {
    return -1;
  }
  if ( UVOS_SPI_ClaimBus( dev->spi_id ) != 0 ) {
    return -2;
  }
  UVOS_MPU9250_SetSpeed( fast_spi );
  UVOS_SPI_RC_PinSet( dev->spi_id, dev->slave_num, 0 );
  return 0;
}


static void UVOS_MPU9250_SetSpeed( const bool fast )
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
static int32_t UVOS_MPU9250_ClaimBusISR( bool *woken, bool fast_spi )
{
  if ( UVOS_MPU9250_Validate( dev ) != 0 ) {
    return -1;
  }
  if ( UVOS_SPI_ClaimBusISR( dev->spi_id, woken ) != 0 ) {
    return -2;
  }
  UVOS_MPU9250_SetSpeed( fast_spi );
  UVOS_SPI_RC_PinSet( dev->spi_id, dev->slave_num, 0 );
  return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful
 */
static int32_t UVOS_MPU9250_ReleaseBus()
{
  if ( UVOS_MPU9250_Validate( dev ) != 0 ) {
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
static int32_t UVOS_MPU9250_ReleaseBusISR( bool *woken )
{
  if ( UVOS_MPU9250_Validate( dev ) != 0 ) {
    return -1;
  }
  UVOS_SPI_RC_PinSet( dev->spi_id, dev->slave_num, 1 );
  return UVOS_SPI_ReleaseBusISR( dev->spi_id, woken );
}

/**
 * @brief Read a register from MPU9250
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t UVOS_MPU9250_GetReg( uint8_t reg )
{
  uint8_t data;

  if ( UVOS_MPU9250_ClaimBus( false ) != 0 ) {
    return -1;
  }

  UVOS_SPI_TransferByte( dev->spi_id, ( 0x80 | reg ) ); // request byte
  data = UVOS_SPI_TransferByte( dev->spi_id, 0 ); // receive response

  UVOS_MPU9250_ReleaseBus();
  return data;
}

/**
 * @brief Writes one byte to the MPU9250
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to send the command
 * \return -3 if unable to receive the response
 */
static int32_t UVOS_MPU9250_SetReg( uint8_t reg, uint8_t data )
{
  int ret = 0;

  if ( UVOS_MPU9250_ClaimBus( false ) != 0 ) {
    return -1;
  }

  UVOS_SPI_TransferByte( dev->spi_id, 0x7f & reg );
  // if (UVOS_SPI_TransferByte(dev->spi_id, 0x7f & reg) != 0) {
  // UVOS_MPU9250_ReleaseBus();
  // return -2;
  // }

  UVOS_SPI_TransferByte( dev->spi_id, data );
  // if (UVOS_SPI_TransferByte(dev->spi_id, data) != 0) {
  // UVOS_MPU9250_ReleaseBus();
  // return -3;
  // }

  UVOS_MPU9250_ReleaseBus();

  return ret;
}


/*
 * @brief Read the identification bytes from the MPU9250 sensor
 * \return ID read from MPU9250 or -1 if failure
 */
int32_t UVOS_MPU9250_ReadID()
{
  int32_t mpu9250_id = UVOS_MPU9250_GetReg( UVOS_MPU9250_WHOAMI );

  if ( mpu9250_id < 0 ) {
    return -1;
  }
  return mpu9250_id;
}

static float UVOS_MPU9250_GetScale()
{
  switch ( dev->gyro_range ) {
  case UVOS_MPU9250_SCALE_250_DEG:
    return 1.0f / 131.0f;

  case UVOS_MPU9250_SCALE_500_DEG:
    return 1.0f / 65.5f;

  case UVOS_MPU9250_SCALE_1000_DEG:
    return 1.0f / 32.8f;

  case UVOS_MPU9250_SCALE_2000_DEG:
    return 1.0f / 16.4f;
  }
  return 0;
}

static float UVOS_MPU9250_GetAccelScale()
{
  switch ( dev->accel_range ) {
  case UVOS_MPU9250_ACCEL_2G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 16384.0f;

  case UVOS_MPU9250_ACCEL_4G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 8192.0f;

  case UVOS_MPU9250_ACCEL_8G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 4096.0f;

  case UVOS_MPU9250_ACCEL_16G:
    return UVOS_CONST_MKS_GRAV_ACCEL_F / 2048.0f;
  }
  return 0;
}

/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test failed
 */
static int32_t UVOS_MPU9250_Test( void )
{
  /* Verify that ID matches */
  int32_t mpu9250_id = UVOS_MPU9250_ReadID();

  if ( mpu9250_id < 0 ) {
    return -1;
  }

  if ( mpu9250_id != UVOS_MPU9250_GYRO_ACC_ID ) {
    return -2;
  }

  return 0;
}

#if defined(UVOS_MPU9250_MAG)
/**
 * @brief Read a mag register from MPU9250
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t UVOS_MPU9250_Mag_GetReg( uint8_t reg )
{
  int32_t data;

  // Set the I2C slave address and read command.
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV4_ADDR, UVOS_MPU9250_MAG_I2C_ADDR |
                               UVOS_MPU9250_MAG_I2C_READ_FLAG ) != UVOS_MPU9250_MAG_OK ) {
    ;
  }

  // Set the address of the register to read.
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV4_REG, reg ) != UVOS_MPU9250_MAG_OK ) {
    ;
  }

  // Trigger the byte transfer.
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV4_CTRL, UVOS_MPU9250_I2C_SLV_ENABLE ) != UVOS_MPU9250_MAG_OK ) {
    ;
  }

  UVOS_DELAY_WaitmS( 1 );

  // Read result.
  data = UVOS_MPU9250_GetReg( UVOS_MPU9250_I2C_SLV4_DI );
  UVOS_DELAY_WaitmS( 1 );
  return data;
}

/**
 * @brief Writes one byte to the MPU9250
 * \param[in] reg Register address
 * \param[in] data Byte to write
 */
static int32_t UVOS_MPU9250_Mag_SetReg( uint8_t reg, uint8_t data )
{
  // Set the I2C slave address.
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV4_ADDR, UVOS_MPU9250_MAG_I2C_ADDR ) != UVOS_MPU9250_MAG_OK ) {
    ;
  }

  // Set the address of the register to write.
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV4_REG, reg ) != UVOS_MPU9250_MAG_OK ) {
    ;
  }

  // Set the byte to write.
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV4_DO, data ) != UVOS_MPU9250_MAG_OK ) {
    ;
  }

  // Trigger the byte transfer.
  while ( UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV4_CTRL, UVOS_MPU9250_I2C_SLV_ENABLE ) != UVOS_MPU9250_MAG_OK ) {
    ;
  }
  UVOS_DELAY_WaitmS( 1 );
  return UVOS_MPU9250_MAG_OK;
}

/**
 * @rief Get ASAx registers from fuse ROM
 *  Hadj = H*((ASA-128)*0.5/128+1)
 * \return 0 if test succeeded
 * \return non-zero value if test failed
 */
static int32_t UVOS_MPU9250_Mag_Sensitivity( void )
{
  int i;

  /* Put mag in power down state before changing mode */
  UVOS_MPU9250_Mag_SetReg( UVOS_MPU9250_CNTL1, UVOS_MPU9250_MAG_POWER_DOWN_MODE );
  UVOS_DELAY_WaitmS( 1 );

  /* Enable fuse ROM for access */
  UVOS_MPU9250_Mag_SetReg( UVOS_MPU9250_CNTL1, UVOS_MPU9250_MAG_FUSE_ROM_MODE );
  UVOS_DELAY_WaitmS( 1 );

  if ( UVOS_MPU9250_ClaimBus( false ) != 0 ) {
    return -1;
  }

  /* Set addres and read flag */
  UVOS_SPI_TransferByte( dev->spi_id, UVOS_MPU9250_I2C_SLV0_ADDR );
  UVOS_SPI_TransferByte( dev->spi_id, UVOS_MPU9250_MAG_I2C_ADDR | UVOS_MPU9250_MAG_I2C_READ_FLAG );

  /* Set the address of the register to read. */
  UVOS_SPI_TransferByte( dev->spi_id, UVOS_MPU9250_I2C_SLV0_REG );
  UVOS_SPI_TransferByte( dev->spi_id, UVOS_MPU9250_ASAX );

  /* Trigger the byte transfer. */
  UVOS_SPI_TransferByte( dev->spi_id, UVOS_MPU9250_I2C_SLV0_CTRL );
  UVOS_SPI_TransferByte( dev->spi_id, UVOS_MPU9250_I2C_SLV_ENABLE | 0x3 );

  UVOS_DELAY_WaitmS( 1 );

  /* Read the mag data from SPI block */
  for ( i = 0; i < 0x3; i++ ) {
    UVOS_SPI_TransferByte( dev->spi_id, ( UVOS_MPU9250_EXT_SENS_DATA_00 | 0x80 ) + i );
    int32_t ret = UVOS_SPI_TransferByte( dev->spi_id, 0x0 );
    if ( ret < 0 ) {
      UVOS_MPU9250_ReleaseBus();
      return -1;
    }
    dev->mag_sens_adj[i] = 1.0f; // 1.0f + ((float)((uint8_t)ret - 128)) / 256.0f;
  }

  UVOS_MPU9250_ReleaseBus();


  /* Put mag in power down state before changing mode */
  UVOS_MPU9250_Mag_SetReg( UVOS_MPU9250_CNTL1, UVOS_MPU9250_MAG_POWER_DOWN_MODE );

  return UVOS_MPU9250_MAG_OK;
}

/**
 * @brief Read a mag register from MPU9250
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t UVOS_MPU9250_Mag_Init( void )
{
  // I2C multi-master init.
  UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_MST_CTRL, UVOS_MPU9250_I2C_MST_P_NSR | UVOS_MPU9250_I2C_MST_CLOCK_400 );
  UVOS_DELAY_WaitmS( 1 );

  // Reset Mag.
  UVOS_MPU9250_Mag_SetReg( UVOS_MPU9250_CNTL2, UVOS_MPU9250_MAG_RESET );
  UVOS_DELAY_WaitmS( 1 );


  // read fuse ROM to get the sensitivity adjustment values.
  if ( UVOS_MPU9250_Mag_Sensitivity() != UVOS_MPU9250_MAG_OK ) {
    ;
  }

  // Confirm Mag ID.
  while ( false && ( UVOS_MPU9250_Mag_Test() != UVOS_MPU9250_MAG_OK ) ) {
    ;
  }

  // Make sure no other registers will be triggered before entering continuous mode.
  UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV4_CTRL, 0x0 );
  UVOS_DELAY_WaitmS( 1 );
  UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV0_DO, 0x0 );
  UVOS_DELAY_WaitmS( 1 );

  // Making sure register are accessible.
  UVOS_MPU9250_Mag_SetReg( UVOS_MPU9250_CNTL1, UVOS_MPU9250_MAG_OUTPUT_16BITS | UVOS_MPU9250_MAG_CONTINUOUS_MODE2 );
  UVOS_DELAY_WaitmS( 1 );

  // Get ST1, the 6 mag data and ST2.
  // This is to save 2 SPI access.
  // Set the I2C slave address and read command.
  UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV0_ADDR, UVOS_MPU9250_MAG_I2C_ADDR | UVOS_MPU9250_MAG_I2C_READ_FLAG );

  // Set the address of the register to read.
  UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV0_REG, UVOS_MPU9250_ST1 );

  // Trigger the byte transfer.
  UVOS_MPU9250_SetReg( UVOS_MPU9250_I2C_SLV0_CTRL, UVOS_MPU9250_I2C_SLV_ENABLE | 0x8 );
  UVOS_DELAY_WaitmS( 1 );

  return UVOS_MPU9250_MAG_OK;
}

/*
 * @brief Read the mag identification bytes from the MPU9250 sensor
 */
int32_t UVOS_MPU9250_Mag_ReadID()
{
  int32_t mpu9250_mag_id = UVOS_MPU9250_Mag_GetReg( UVOS_MPU9250_WIA );

  if ( mpu9250_mag_id < UVOS_MPU9250_MAG_OK ) {
    return UVOS_MPU9250_ERR_MAG_READ_ID;
  }
  return mpu9250_mag_id;
}

/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test failed
 */
static int32_t UVOS_MPU9250_Mag_Test( void )
{
  /* Verify that ID matches */
  int32_t mpu9250_mag_id = UVOS_MPU9250_Mag_ReadID();

  if ( mpu9250_mag_id < UVOS_MPU9250_MAG_OK ) {
    return UVOS_MPU9250_ERR_MAG_READ_ID;
  }

  if ( mpu9250_mag_id != UVOS_MPU9250_MAG_ID ) {
    return UVOS_MPU9250_ERR_MAG_BAD_ID;
  }

  /* TODO: run self-test */

  return UVOS_MPU9250_MAG_OK;
}


/**
 * @brief Read the mag data.
 * \return true if data has been read from mpu
 * \return false on error
 */
static bool UVOS_MPU9250_ReadMag( bool *woken )
{
  if ( UVOS_MPU9250_ClaimBusISR( woken, true ) != 0 ) {
    return false;
  }
  // Trigger the byte transfer.
  UVOS_SPI_TransferByte( dev->spi_id, UVOS_MPU9250_I2C_SLV0_CTRL );
  UVOS_SPI_TransferByte( dev->spi_id, UVOS_MPU9250_I2C_SLV_ENABLE | 0x8 );

  UVOS_MPU9250_ReleaseBusISR( woken );

  return true;
}
#endif /* if defined(UVOS_MPU9250_MAG) */

/**
 * @brief EXTI IRQ Handler.  Read all the data from onboard buffer
 * @return a boolean to the EXTI IRQ Handler wrapper indicating if a
 *         higher priority task is now eligible to run
 */
bool UVOS_MPU9250_IRQHandler( void )
{
  uint32_t gyro_read_timestamp = UVOS_DELAY_GetRaw();
  bool woken = false;

  if ( !mpu9250_configured ) {
    return false;
  }

#if defined(UVOS_MPU9250_MAG)
  UVOS_MPU9250_ReadMag( &woken );
#endif

  if ( UVOS_MPU9250_ReadSensor( &woken ) ) {
    woken |= UVOS_MPU9250_HandleData( gyro_read_timestamp );
  }

  return woken;
}

static bool UVOS_MPU9250_HandleData( uint32_t gyro_read_timestamp )
{
  // Rotate the sensor to OP convention.  The datasheet defines X as towards the right
  // and Y as forward.  OP convention transposes this.  Also the Z is defined negatively
  // to our convention
  if ( !queue_data ) {
    return false;
  }

#ifdef UVOS_MPU9250_MAG
  bool mag_valid = mpu9250_data.data.st1 & UVOS_MPU9250_MAG_DATA_RDY;
#endif

  // Currently we only support rotations on top so switch X/Y accordingly
  switch ( dev->cfg->orientation ) {
  case UVOS_MPU9250_TOP_0DEG:
#ifdef UVOS_MPU9250_ACCEL
    queue_data->sample[0].y = GET_SENSOR_DATA( mpu9250_data, Accel_X ); // chip X
    queue_data->sample[0].x = GET_SENSOR_DATA( mpu9250_data, Accel_Y ); // chip Y
#endif
    queue_data->sample[1].y = GET_SENSOR_DATA( mpu9250_data, Gyro_X ); // chip X
    queue_data->sample[1].x = GET_SENSOR_DATA( mpu9250_data, Gyro_Y ); // chip Y
#ifdef UVOS_MPU9250_MAG
    if ( mag_valid ) {
      mag_data->sample[0].y = GET_SENSOR_DATA( mpu9250_data, Mag_Y ) * dev->mag_sens_adj[1]; // chip Y
      mag_data->sample[0].x = GET_SENSOR_DATA( mpu9250_data, Mag_X ) * dev->mag_sens_adj[0]; // chip X
    }
#endif
    break;
  case UVOS_MPU9250_TOP_90DEG:
    // -1 to bring it back to -32768 +32767 range
#ifdef UVOS_MPU9250_ACCEL
    queue_data->sample[0].y = -1 - ( GET_SENSOR_DATA( mpu9250_data, Accel_Y ) ); // chip Y
    queue_data->sample[0].x = GET_SENSOR_DATA( mpu9250_data, Accel_X ); // chip X
#endif
    queue_data->sample[1].y = -1 - ( GET_SENSOR_DATA( mpu9250_data, Gyro_Y ) ); // chip Y
    queue_data->sample[1].x = GET_SENSOR_DATA( mpu9250_data, Gyro_X ); // chip X
#ifdef UVOS_MPU9250_MAG
    if ( mag_valid ) {
      mag_data->sample[0].y = GET_SENSOR_DATA( mpu9250_data, Mag_X ) * dev->mag_sens_adj[0]; // chip X
      mag_data->sample[0].x = -1 - ( GET_SENSOR_DATA( mpu9250_data, Mag_Y ) ) * dev->mag_sens_adj[1]; // chip Y
    }

#endif
    break;
  case UVOS_MPU9250_TOP_180DEG:
#ifdef UVOS_MPU9250_ACCEL
    queue_data->sample[0].y = -1 - ( GET_SENSOR_DATA( mpu9250_data, Accel_X ) ); // chip X
    queue_data->sample[0].x = -1 - ( GET_SENSOR_DATA( mpu9250_data, Accel_Y ) ); // chip Y
#endif
    queue_data->sample[1].y = -1 - ( GET_SENSOR_DATA( mpu9250_data, Gyro_X ) ); // chip X
    queue_data->sample[1].x = -1 - ( GET_SENSOR_DATA( mpu9250_data, Gyro_Y ) ); // chip Y
#ifdef UVOS_MPU9250_MAG
    if ( mag_valid ) {
      mag_data->sample[0].y = -1 - ( GET_SENSOR_DATA( mpu9250_data, Mag_Y ) ) * dev->mag_sens_adj[1]; // chip Y
      mag_data->sample[0].x = -1 - ( GET_SENSOR_DATA( mpu9250_data, Mag_X ) ) * dev->mag_sens_adj[0]; // chip X
    }
#endif
    break;
  case UVOS_MPU9250_TOP_270DEG:
#ifdef UVOS_MPU9250_ACCEL
    queue_data->sample[0].y = GET_SENSOR_DATA( mpu9250_data, Accel_Y ); // chip Y
    queue_data->sample[0].x = -1 - ( GET_SENSOR_DATA( mpu9250_data, Accel_X ) ); // chip X
#endif
    queue_data->sample[1].y = GET_SENSOR_DATA( mpu9250_data, Gyro_Y ); // chip Y
    queue_data->sample[1].x = -1 - ( GET_SENSOR_DATA( mpu9250_data, Gyro_X ) ); // chip X
#ifdef UVOS_MPU9250_MAG
    if ( mag_valid ) {
      mag_data->sample[0].y = -1 - ( GET_SENSOR_DATA( mpu9250_data, Mag_X ) ) * dev->mag_sens_adj[0]; // chip X
      mag_data->sample[0].x = GET_SENSOR_DATA( mpu9250_data, Mag_Y ) * dev->mag_sens_adj[1]; // chip Y
    }
#endif
    break;
  }
#ifdef UVOS_MPU9250_ACCEL
  queue_data->sample[0].z = -1 - ( GET_SENSOR_DATA( mpu9250_data, Accel_Z ) );
#endif
  queue_data->sample[1].z = -1 - ( GET_SENSOR_DATA( mpu9250_data, Gyro_Z ) );
  const int16_t temp = GET_SENSOR_DATA( mpu9250_data, Temperature );
  queue_data->temperature = 2100 + ( ( float )( temp - UVOS_MPU9250_TEMP_OFFSET ) ) * ( 100.0f / UVOS_MPU9250_TEMP_SENSITIVITY );
  queue_data->timestamp   = gyro_read_timestamp;
  mag_data->temperature   = queue_data->temperature;
#ifdef UVOS_MPU9250_MAG
  if ( mag_valid ) {
    mag_data->sample[0].z = GET_SENSOR_DATA( mpu9250_data, Mag_Z ) * dev->mag_sens_adj[2]; // chip Z
    mag_ready = true;
  }
#endif

  // BaseType_t higherPriorityTaskWoken;
  // xQueueSendToBackFromISR( dev->queue, queue_data, &higherPriorityTaskWoken );
  // return higherPriorityTaskWoken == pdTRUE;

  UVOS_Queue_Send( dev->queue, ( void * )queue_data, 0 );
  return true;
}

static bool UVOS_MPU9250_ReadSensor( bool *woken )
{
  const uint8_t mpu9250_send_buf[1 + UVOS_MPU9250_SAMPLES_BYTES] = { UVOS_MPU9250_SENSOR_FIRST_REG | 0x80 };

  if ( UVOS_MPU9250_ClaimBusISR( woken, true ) != 0 ) {
    return false;
  }
  if ( UVOS_SPI_TransferBlock( dev->spi_id, &mpu9250_send_buf[0], &mpu9250_data.buffer[0], sizeof( mpu9250_data_t ), NULL ) < 0 ) {
    UVOS_MPU9250_ReleaseBusISR( woken );
    return false;
  }
  UVOS_MPU9250_ReleaseBusISR( woken );
  return true;
}

// Sensor driver implementation
bool UVOS_MPU9250_Main_driver_Test( __attribute__( ( unused ) ) uintptr_t context )
{
  return !UVOS_MPU9250_Test();
}

void UVOS_MPU9250_Main_driver_Reset( __attribute__( ( unused ) ) uintptr_t context )
{
  UVOS_MPU9250_GetReg( UVOS_MPU9250_INT_STATUS_REG );
}

void UVOS_MPU9250_Main_driver_get_scale( float *scales, uint8_t size, __attribute__( ( unused ) ) uintptr_t contet )
{
  UVOS_Assert( size >= 2 );
  scales[0] = UVOS_MPU9250_GetAccelScale();
  scales[1] = UVOS_MPU9250_GetScale();
}

p_uvos_queue_t UVOS_MPU9250_Main_driver_get_queue( __attribute__( ( unused ) ) uintptr_t context )
{
  return dev->queue;
}


/* UVOS sensor driver implementation */
bool UVOS_MPU9250_Mag_driver_Test( __attribute__( ( unused ) ) uintptr_t context )
{
  return !UVOS_MPU9250_Test();
}

void UVOS_MPU9250_Mag_driver_Reset( __attribute__( ( unused ) ) uintptr_t context ) {}

void UVOS_MPU9250_Mag_driver_get_scale( float *scales, uint8_t size, __attribute__( ( unused ) )  uintptr_t context )
{
  UVOS_Assert( size > 0 );
  scales[0] = 1;
}

void UVOS_MPU9250_Mag_driver_fetch( void *data, uint8_t size, __attribute__( ( unused ) )  uintptr_t context )
{
  mag_ready = false;
  UVOS_Assert( size > 0 );
  memcpy( data, mag_data, MAG_SENSOR_DATA_SIZE );
}

bool UVOS_MPU9250_Mag_driver_poll( __attribute__( ( unused ) ) uintptr_t context )
{
  return mag_ready;
}

#endif /* UVOS_INCLUDE_MPU9250 */

/**
 * @}
 * @}
 */
