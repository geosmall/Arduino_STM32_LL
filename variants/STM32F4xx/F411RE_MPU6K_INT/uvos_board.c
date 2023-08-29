#include <uvos.h>

#ifdef UVOS_INCLUDE_INSTRUMENTATION
#include <uvos_instrumentation.h>
#endif

/*
 * Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */
#include "board_hw_defs.c.inc"

#if defined( UVOS_INCLUDE_EXTI )

bool UVOS_USER_BTN_IRQHandler( void )
{
  return true;
}

static const struct uvos_exti_cfg uvos_exti_user_btn_cfg __exti_config = {
  .vector = UVOS_USER_BTN_IRQHandler,
  .line   = LL_EXTI_LINE_13,
  .pin    = {
    .gpio = GPIOC,
    .init = {
      .Pin   = LL_GPIO_PIN_13,
      .Mode = LL_GPIO_MODE_INPUT,
      .Speed = LL_GPIO_SPEED_FREQ_HIGH,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_NO,
    },
  },
  .irq                                       = {
    .init                                  = {
      .NVIC_IRQChannel    = EXTI15_10_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
  .exti                                      = {
    .init                                  = {
      .Line_0_31    = LL_EXTI_LINE_13, // matches above GPIO pin
      .LineCommand  = ENABLE,
      .Mode         = LL_EXTI_MODE_IT,
      .Trigger      = LL_EXTI_TRIGGER_RISING,
    },
  },
};

#endif // defined( UVOS_INCLUDE_EXTI )

/**
 * Configuration for the MPU6000 chip
 */
#if defined(UVOS_INCLUDE_MPU6000)
#include "uvos_mpu6000.h"
#include "uvos_mpu6000_config.h"
static const struct uvos_exti_cfg uvos_exti_mpu6000_cfg __exti_config = {
  .vector = UVOS_MPU6000_IRQHandler,
  .line = LL_EXTI_LINE_4,
  .pin = {
    .gpio = GPIOC,
    .init = {
      .Pin   = LL_GPIO_PIN_4,
      .Mode = LL_GPIO_MODE_INPUT,
      .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
      .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
      .Pull = LL_GPIO_PULL_NO,
    },
  },
  .irq = {
    .init = {
      .NVIC_IRQChannel = EXTI4_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_HIGH,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
  .exti = {
    .init = {
      .Line_0_31    = LL_EXTI_LINE_4, // matches above GPIO pin
      .LineCommand  = ENABLE,
      .Mode         = LL_EXTI_MODE_IT,
      .Trigger      = LL_EXTI_TRIGGER_RISING,
    },
  },
};

static const struct uvos_mpu6000_cfg uvos_mpu6000_cfg = {
  .exti_cfg   = &uvos_exti_mpu6000_cfg,
  .Fifo_store = UVOS_MPU6000_FIFO_TEMP_OUT | UVOS_MPU6000_FIFO_GYRO_X_OUT | UVOS_MPU6000_FIFO_GYRO_Y_OUT | UVOS_MPU6000_FIFO_GYRO_Z_OUT,
  // Gyroscope Output Rate = 8kHz when DLPF is disabled
  .Smpl_rate_div_no_dlp = 0,
  // Gyroscope Output Rate = 1kHz when DLPF is enabled (see CONFIG Register 0x1A).
  .Smpl_rate_div_dlp    = 0,
  .interrupt_cfg  = UVOS_MPU6000_INT_CLR_ANYRD,
  .interrupt_en   = UVOS_MPU6000_INTEN_DATA_RDY,
  .User_ctl       = UVOS_MPU6000_USERCTL_DIS_I2C,
  .Pwr_mgmt_clk   = UVOS_MPU6000_PWRMGMT_PLL_Z_CLK,
  .accel_range    = UVOS_MPU6000_ACCEL_8G,
  .gyro_range     = UVOS_MPU6000_SCALE_2000_DEG,
  .filter         = UVOS_MPU6000_LOWPASS_256_HZ,
  .orientation    = UVOS_MPU6000_TOP_0DEG,
  .fast_prescaler = UVOS_SPI_PRESCALER_16,
  .std_prescaler  = UVOS_SPI_PRESCALER_128,
  .max_downsample = 20,
};
#endif /* UVOS_INCLUDE_MPU6000 */

/* One slot per selectable receiver group.
 *  eg. PWM, PPM, GCS, SPEKTRUM1, SPEKTRUM2, SBUS
 * NOTE: No slot in this map for NONE.
 */
uint32_t uvos_rcvr_group_map[ UVOS_RCVR_CHANNELGROUPS_NONE ];

#define UVOS_COM_TELEM_RF_RX_BUF_LEN     512
#define UVOS_COM_TELEM_RF_TX_BUF_LEN     512

#define UVOS_COM_TELEM_USB_RX_BUF_LEN    65
#define UVOS_COM_TELEM_USB_TX_BUF_LEN    65

#define UVOS_COM_MAVLINK_RX_BUF_LEN      64
#define UVOS_COM_MAVLINK_TX_BUF_LEN      128

#if defined(UVOS_INCLUDE_DEBUG_CONSOLE)
#define UVOS_COM_DEBUGCONSOLE_RX_BUF_LEN 64
#define UVOS_COM_DEBUGCONSOLE_TX_BUF_LEN 64
uint32_t uvos_com_debug_id;
#endif /* UVOS_INCLUDE_DEBUG_CONSOLE */

// #if defined(UVOS_INCLUDE_FLASH)
// uintptr_t uvos_spi_flash_id;
// #endif /* UVOS_INCLUDE_FLASH */

// #if defined( UVOS_INCLUDE_SDCARD )
// uintptr_t uvos_spi_sdcard_id;
// #endif /* UVOS_INCLUDE_SDCARD */

// uint32_t uvos_com_gps_id       = 0;
// uint32_t uvos_com_telem_usb_id = 0;
uint32_t uvos_com_telem_rf_id  = 0;
uint32_t uvos_com_rf_id        = 0;
// uint32_t uvos_com_bridge_id    = 0;
// uint32_t uvos_com_overo_id     = 0;
// uint32_t uvos_com_hkosd_id     = 0;
// uint32_t uvos_com_vcp_id       = 0;
// uint32_t uvos_com_msp_id       = 0;
uint32_t uvos_com_mavlink_id   = 0;

uintptr_t uvos_uavo_settings_fs_id;
uintptr_t uvos_user_fs_id;

/* Scheduler timer handle */
// static uint32_t uvos_sched_tim_id     = 0;

/*
 * Setup a com port based on the passed cfg, driver and buffer sizes.
 * tx size = 0 make the port rx only
 * rx size = 0 make the port tx only
 * having both tx and rx size = 0 is not valid and will fail further down in UVOS_COM_Init()
 */
static void UVOS_Board_configure_com( const struct uvos_usart_cfg *usart_port_cfg, uint16_t rx_buf_len, uint16_t tx_buf_len,
                                      const struct uvos_com_driver *com_driver, uint32_t *uvos_com_id )
{
  uint32_t uvos_usart_id;

  if ( UVOS_USART_Init( &uvos_usart_id, usart_port_cfg ) ) {
    UVOS_Assert( 0 );
  }

  uint8_t *rx_buffer = 0, * tx_buffer = 0;

  if ( rx_buf_len > 0 ) {
    rx_buffer = ( uint8_t * )UVOS_malloc( rx_buf_len );
    UVOS_Assert( rx_buffer );
  }

  if ( tx_buf_len > 0 ) {
    tx_buffer = ( uint8_t * )UVOS_malloc( tx_buf_len );
    UVOS_Assert( tx_buffer );
  }

  if ( UVOS_COM_Init( uvos_com_id, com_driver, uvos_usart_id, rx_buffer, rx_buf_len, tx_buffer, tx_buf_len ) ) {
    UVOS_Assert( 0 );
  }
}

static void UVOS_Board_configure_ibus( const struct uvos_usart_cfg *usart_cfg )
{
  uint32_t uvos_usart_ibus_id;

  if ( UVOS_USART_Init( &uvos_usart_ibus_id, usart_cfg ) ) {
    UVOS_Assert( 0 );
  }

  uint32_t uvos_ibus_id;
  if ( UVOS_IBUS_Init( &uvos_ibus_id, &uvos_usart_com_driver, uvos_usart_ibus_id ) ) {
    UVOS_Assert( 0 );
  }

  uint32_t uvos_ibus_rcvr_id;
  if ( UVOS_RCVR_Init( &uvos_ibus_rcvr_id, &uvos_ibus_rcvr_driver, uvos_ibus_id ) ) {
    UVOS_Assert( 0 );
  }

  uvos_rcvr_group_map[ UVOS_RCVR_CHANNELGROUPS_IBUS ] = uvos_ibus_rcvr_id;
}

// void DMA_Transaction_Complete( bool crc_ok, uint8_t crc_val )
// {

// }

/**
 * UVOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from uavware.c
 */
int32_t UVOS_Board_Init( void )
{
  int32_t ret;

#if defined( UVOS_INCLUDE_LED )
  const struct uvos_gpio_cfg *led_cfg  = &uvos_led_cfg;
  UVOS_Assert( led_cfg );
  UVOS_LED_Init( led_cfg );
#endif /* UVOS_INCLUDE_LED */

#if defined( UVOS_INCLUDE_DEBUG_CONSOLE )
  UVOS_Board_configure_com( &uvos_usart_flexi_cfg,
                            UVOS_COM_DEBUGCONSOLE_RX_BUF_LEN,
                            UVOS_COM_DEBUGCONSOLE_TX_BUF_LEN,
                            &uvos_usart_com_driver, &uvos_com_debug_id );
#endif

  /* Set up the SPI interface to the gyro/acelerometer */
  if ( UVOS_SPI_Init( &uvos_spi_gyro_id, &uvos_spi_gyro_cfg ) ) {
    return -1;
  }

  /* Set up the SPI interface to the flash */
  if ( UVOS_SPI_Init( &uvos_spi_storage_id, &uvos_spi_storage_cfg ) ) {
    return -1;
  }

#if defined( UVOS_INCLUDE_FLASH )

  /* Enable and mount the SPI Flash, and retrieve pointer to interface driver */
  ret = UVOS_SPIF_Init( uvos_spi_storage_id );
  if ( ret < 0 ) {
    return -2;
  }

#if defined( ERASE_SYSTEM_FLASH )
  UVOS_Flash_Jedec_EraseChip( uvos_spi_flash_id );
#endif // defined( ERASE_SYSTEM_FLASH )

  /* Set up the file sysytem interface */
  if ( UVOS_FS_Init( &uvos_fs_spif_driver ) ) {
    return -3;
  }

#endif // defined( UVOS_INCLUDE_FLASH )

#if defined( UVOS_INCLUDE_SDCARD )

  /* Enable and mount the SDCard */
  ret = UVOS_SDCARD_Init( uvos_spi_storage_id );
  if ( ret < 0 ) {
    return -2;
  }

  /* Set up the file sysytem interface */
  if ( UVOS_FS_Init( &uvos_fs_sdcard_driver ) ) {
    return -3;
  }

#endif // defined( UVOS_INCLUDE_SDCARD )

#if defined( UVOS_INCLUDE_EXTI )
  UVOS_EXTI_Init( &uvos_exti_user_btn_cfg );
#endif // defined( UVOS_INCLUDE_EXTI )

#if defined( UVOS_INCLUDE_RTC )
  UVOS_RTC_Init( &uvos_rtc_main_cfg );
#endif

  /* Set up scheduler timer */
  UVOS_TIM_InitClock( &tim_11_cfg );

#if defined( UVOS_INCLUDE_SCHED )
  /* Set up scheduler */
  UVOS_SCHED_Init( &uvos_sched_cfg_out );
#endif // defined( UVOS_INCLUDE_SCHED )

  /* Set up pulse timers */
  UVOS_TIM_InitClock( &tim_1_cfg );
  // UVOS_TIM_InitClock( &tim_2_cfg );
  UVOS_TIM_InitClock( &tim_3_cfg );
  // UVOS_TIM_InitClock( &tim_5_cfg );
  // UVOS_TIM_InitClock( &tim_9_cfg );
  // UVOS_TIM_InitClock( &tim_10_cfg );
  // UVOS_TIM_InitClock( &tim_11_cfg );

#if !defined( UVOS_ENABLE_DEBUG_PINS )
  /* uvos_servo_cfg points to the correct configuration based on input port settings */
  UVOS_Servo_Init( &uvos_servo_cfg_out );
#else
  UVOS_DEBUG_Init( uvos_tim_servoport_all_pins, NELEMENTS( uvos_tim_servoport_all_pins ) );
#endif // !defined( UVOS_ENABLE_DEBUG_PINS )

#if defined( UVOS_INCLUDE_IBUS )
  UVOS_Board_configure_ibus( &uvos_usart_ibus_cfg );
#endif // defined( UVOS_INCLUDE_IBUS )

#if defined(UVOS_INCLUDE_MPU6000)
  /* Initialize IMU, initial settings per uvos_mpu6000_cfg defined above */
  ret = UVOS_MPU6000_Init( uvos_spi_gyro_id, 0, &uvos_mpu6000_cfg );
  if ( ret < 0 ) {
    return -4;
  }
  /* Register MPU6000 as a sensor via UVOS_SENSORS_Register() */
  UVOS_MPU6000_Register();
#endif

  return 0;
}
