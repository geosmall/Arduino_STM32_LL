#include "uvos_ibus_priv.h"

#ifdef UVOS_INCLUDE_IBUS

// 1 sync byte, 1 command code byte, 10x channels (uint16_t), 8 unknown bytes, 2 crc bytes
#define UVOS_IBUS_BUFLEN   (1 + 1 + UVOS_IBUS_NUM_INPUTS * 2 + 8 + 2)
#define UVOS_IBUS_SYNCBYTE 0x20
#define UVOS_IBUS_MAGIC    0x84fd9a39

/**
 * @brief IBus receiver driver internal state data with double buffered channel data
 */
struct uvos_ibus_dev {
  uint32_t magic;
  int      buf_pos;
  int      rx_timer;
  int      failsafe_timer;
  uint16_t checksum;
  bool     bank_0_write_enabled;
  uint16_t channel_data_0[ UVOS_IBUS_NUM_INPUTS ];
  uint16_t channel_data_1[ UVOS_IBUS_NUM_INPUTS ];
  uint8_t  rx_buf[ UVOS_IBUS_BUFLEN ];
};

/**
 * @brief Allocates a driver instance
 * @retval uvos_ibus_dev pointer on success, NULL on failure
 */
static struct uvos_ibus_dev * UVOS_IBUS_Alloc( void );
/**
 * @brief Validate a driver instance
 * @param[in] dev device driver instance pointer
 * @retval true on success, false on failure
 */
static bool UVOS_IBUS_Validate( const struct uvos_ibus_dev * ibus_dev );
/**
 * @brief Read a channel from the last received frame
 * @param[in] id Driver instance
 * @param[in] channel 0-based channel index
 * @retval raw channel value, or error value (see uvos_rcvr.h)
 */
static int32_t UVOS_IBUS_Read( uint32_t id, uint8_t channel );
/**
 * @brief Set all channels in the last frame buffer to a given value
 * @param[in] dev Driver instance
 * @param[in] value channel value
 */
static void UVOS_IBUS_SetAllChannels( struct uvos_ibus_dev * ibus_dev, uint16_t value );
/**
 * @brief Serial receive callback
 * @param[in] context Driver instance handle
 * @param[in] buf Receive buffer
 * @param[in] buf_len Number of bytes available
 * @param[out] headroom Number of bytes remaining in internal buffer
 * @param[out] task_woken Did we awake a task?
 * @retval Number of bytes consumed from the buffer
 */
static uint16_t UVOS_IBUS_Receive( uint32_t context, uint8_t * buf, uint16_t buf_len,
                                   uint16_t * headroom, bool * task_woken );
/**
 * @brief Reset the internal receive buffer state
 * @param[in] ibus_dev device driver instance pointer
 */
static void UVOS_IBUS_ResetBuffer( struct uvos_ibus_dev * ibus_dev );
/**
 * @brief Unpack a frame from the internal receive buffer to the channel buffer
 * @param[in] ibus_dev device driver instance pointer
 */
static void UVOS_IBUS_UnpackFrame( struct uvos_ibus_dev * ibus_dev );
/**
 * @brief RTC tick callback
 * @param[in] context Driver instance handle
 */
static void UVOS_IBUS_Supervisor( uint32_t context );

// public
const struct uvos_rcvr_driver uvos_ibus_rcvr_driver = {
  .read = UVOS_IBUS_Read,
};


static struct uvos_ibus_dev * UVOS_IBUS_Alloc( void )
{
  struct uvos_ibus_dev * ibus_dev;

  ibus_dev = ( struct uvos_ibus_dev * )UVOS_malloc( sizeof( *ibus_dev ) );

  if ( !ibus_dev ) {
    return NULL;
  }

  memset( ibus_dev, 0, sizeof( *ibus_dev ) );
  ibus_dev->magic = UVOS_IBUS_MAGIC;

  return ibus_dev;
}

static bool UVOS_IBUS_Validate( const struct uvos_ibus_dev * ibus_dev )
{
  return ibus_dev && ibus_dev->magic == UVOS_IBUS_MAGIC;
}

int32_t UVOS_IBUS_Init( uint32_t * ibus_id, const struct uvos_com_driver * driver, uint32_t lower_id )
{
  struct uvos_ibus_dev * ibus_dev = UVOS_IBUS_Alloc();

  if ( !ibus_dev ) {
    return -1;
  }

  *ibus_id = ( uint32_t )ibus_dev;

  UVOS_IBUS_SetAllChannels( ibus_dev, UVOS_RCVR_INVALID );
  ibus_dev->bank_0_write_enabled = true;

  if ( !UVOS_RTC_RegisterTickCallback( UVOS_IBUS_Supervisor, *ibus_id ) ) {
    UVOS_Assert( 0 );
  }

  ( driver->bind_rx_cb )( lower_id, UVOS_IBUS_Receive, *ibus_id );

  return 0;
}

static int32_t UVOS_IBUS_Read( uint32_t context, uint8_t channel )
{
  if ( channel > UVOS_IBUS_NUM_INPUTS ) {
    return UVOS_RCVR_INVALID;
  }

  struct uvos_ibus_dev * ibus_dev = ( struct uvos_ibus_dev * )context;
  if ( !UVOS_IBUS_Validate( ibus_dev ) ) {
    return UVOS_RCVR_NODRIVER;
  }

  if ( ibus_dev->bank_0_write_enabled ) {
    return ibus_dev->channel_data_1[ channel ];
  } else {
    return ibus_dev->channel_data_0[ channel ];
  }
}

static void UVOS_IBUS_SetAllChannels( struct uvos_ibus_dev * ibus_dev, uint16_t value )
{
  for ( int i = 0; i < UVOS_IBUS_NUM_INPUTS; i++ ) {
    ibus_dev->channel_data_0[ i ] = value;
    ibus_dev->channel_data_1[ i ] = value;
  }
}

static uint16_t UVOS_IBUS_Receive( uint32_t context, uint8_t * buf, uint16_t buf_len,
                                   uint16_t * headroom, bool * task_woken )
{
  struct uvos_ibus_dev * ibus_dev = ( struct uvos_ibus_dev * )context;

  if ( !UVOS_IBUS_Validate( ibus_dev ) ) {
    goto out_fail;
  }

  for ( int i = 0; i < buf_len; i++ ) {
    if ( ibus_dev->buf_pos == 0 && buf[ i ] != UVOS_IBUS_SYNCBYTE ) {
      continue;
    }

    ibus_dev->rx_buf[ ibus_dev->buf_pos++ ] = buf[i];
    if ( ibus_dev->buf_pos <= UVOS_IBUS_BUFLEN - 2 ) {
      ibus_dev->checksum -= buf[ i ];
    } else if ( ibus_dev->buf_pos == UVOS_IBUS_BUFLEN ) {
      UVOS_IBUS_UnpackFrame( ibus_dev );
    }
  }

  ibus_dev->rx_timer = 0;

  *headroom   = UVOS_IBUS_BUFLEN - ibus_dev->buf_pos;
  *task_woken = false;
  return buf_len;

out_fail:
  *headroom   = 0;
  *task_woken = false;
  return 0;
}

static void UVOS_IBUS_ResetBuffer( struct uvos_ibus_dev * ibus_dev )
{
  ibus_dev->checksum = 0xffff;
  ibus_dev->buf_pos  = 0;
}

static void UVOS_IBUS_UnpackFrame( struct uvos_ibus_dev * ibus_dev )
{
  uint16_t rxsum = ibus_dev->rx_buf[ UVOS_IBUS_BUFLEN - 1 ] << 8 |
                   ibus_dev->rx_buf[ UVOS_IBUS_BUFLEN - 2 ];

  if ( ibus_dev->checksum != rxsum ) {
    goto out_fail;
  }

  uint16_t * chan = ( uint16_t * )&ibus_dev->rx_buf[ 2 ];
  for ( int i = 0; i < UVOS_IBUS_NUM_INPUTS; i++ ) {
    if ( ibus_dev->bank_0_write_enabled ) {
      ibus_dev->channel_data_0[ i ] = *chan++;
    } else {
      ibus_dev->channel_data_1[ i ] = *chan++;
    }
  }

  ibus_dev->failsafe_timer = 0;
  ibus_dev->bank_0_write_enabled = !ibus_dev->bank_0_write_enabled;

out_fail:
  UVOS_IBUS_ResetBuffer( ibus_dev );
}

static void UVOS_IBUS_Supervisor( uint32_t context )
{
  struct uvos_ibus_dev * ibus_dev = ( struct uvos_ibus_dev * )context;

  UVOS_Assert( UVOS_IBUS_Validate( ibus_dev ) );

  if ( ++ibus_dev->rx_timer > 3 ) {
    UVOS_IBUS_ResetBuffer( ibus_dev );
  }

  if ( ++ibus_dev->failsafe_timer > 32 ) {
    UVOS_IBUS_SetAllChannels( ibus_dev, UVOS_RCVR_TIMEOUT );
  }
}

#endif // UVOS_INCLUDE_IBUS

/**
 * @}
 * @}
 */
