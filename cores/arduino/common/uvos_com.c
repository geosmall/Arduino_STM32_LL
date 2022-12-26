#include "uvos.h"

#ifdef UVOS_INCLUDE_COM

#include "fifo_buffer.h"
#include <uvos_com_priv.h>

#ifndef UVOS_INCLUDE_FREERTOS
#include "uvos_delay.h" /* UVOS_DELAY_WaitmS */
#endif

enum uvos_com_dev_magic {
  UVOS_COM_DEV_MAGIC = 0xaa55aa55,
};

struct uvos_com_dev {
  enum uvos_com_dev_magic magic;
  uint32_t lower_id;
  const struct uvos_com_driver *driver;

#if defined(UVOS_INCLUDE_FREERTOS)
  xSemaphoreHandle tx_sem;
  xSemaphoreHandle rx_sem;
  xSemaphoreHandle sendbuffer_sem;
#endif

  bool has_rx;
  bool has_tx;

  t_fifo_buffer rx;
  t_fifo_buffer tx;
};

static bool UVOS_COM_validate( struct uvos_com_dev *com_dev )
{
  return com_dev && ( com_dev->magic == UVOS_COM_DEV_MAGIC );
}

#if defined(UVOS_INCLUDE_FREERTOS)
static struct uvos_com_dev *UVOS_COM_alloc( void )
{
  struct uvos_com_dev *com_dev;

  com_dev = ( struct uvos_com_dev * )UVOS_malloc( sizeof( struct uvos_com_dev ) );
  if ( !com_dev ) {
    return NULL;
  }

  memset( com_dev, 0, sizeof( struct uvos_com_dev ) );
  com_dev->magic = UVOS_COM_DEV_MAGIC;
  return com_dev;
}
#else
static struct uvos_com_dev uvos_com_devs[UVOS_COM_MAX_DEVS];
static uint8_t uvos_com_num_devs;
static struct uvos_com_dev *UVOS_COM_alloc( void )
{
  struct uvos_com_dev *com_dev;

  if ( uvos_com_num_devs >= UVOS_COM_MAX_DEVS ) {
    return NULL;
  }

  com_dev = &uvos_com_devs[uvos_com_num_devs++];

  memset( com_dev, 0, sizeof( struct uvos_com_dev ) );
  com_dev->magic = UVOS_COM_DEV_MAGIC;

  return com_dev;
}
#endif /* if defined(UVOS_INCLUDE_FREERTOS) */

static uint16_t UVOS_COM_TxOutCallback( uint32_t context, uint8_t *buf, uint16_t buf_len, uint16_t *headroom, bool *need_yield );
static uint16_t UVOS_COM_RxInCallback( uint32_t context, uint8_t *buf, uint16_t buf_len, uint16_t *headroom, bool *need_yield );
static void UVOS_COM_UnblockRx( struct uvos_com_dev *com_dev, bool *need_yield );
static void UVOS_COM_UnblockTx( struct uvos_com_dev *com_dev, bool *need_yield );

/**
 * Initialises COM layer
 * \param[out] handle
 * \param[in] driver
 * \param[in] id
 * \return < 0 if initialisation failed
 */
int32_t UVOS_COM_Init( uint32_t *com_id, const struct uvos_com_driver *driver, uint32_t lower_id, uint8_t *rx_buffer, uint16_t rx_buffer_len, uint8_t *tx_buffer, uint16_t tx_buffer_len )
{
  UVOS_Assert( com_id );
  UVOS_Assert( driver );

  bool has_rx = ( rx_buffer && rx_buffer_len > 0 );
  bool has_tx = ( tx_buffer && tx_buffer_len > 0 );
  UVOS_Assert( has_rx || has_tx );
  UVOS_Assert( driver->bind_tx_cb || !has_tx );
  UVOS_Assert( driver->bind_rx_cb || !has_rx );

  struct uvos_com_dev *com_dev;

  com_dev = ( struct uvos_com_dev * )UVOS_COM_alloc();
  if ( !com_dev ) {
    goto out_fail;
  }

  com_dev->driver   = driver;
  com_dev->lower_id = lower_id;

  com_dev->has_rx   = has_rx;
  com_dev->has_tx   = has_tx;

  if ( has_rx ) {
    fifoBuf_init( &com_dev->rx, rx_buffer, rx_buffer_len );
#if defined(UVOS_INCLUDE_FREERTOS)
    vSemaphoreCreateBinary( com_dev->rx_sem );
#endif /* UVOS_INCLUDE_FREERTOS */
    ( com_dev->driver->bind_rx_cb )( lower_id, UVOS_COM_RxInCallback, ( uint32_t )com_dev );
    if ( com_dev->driver->rx_start ) {
      /* Start the receiver */
      ( com_dev->driver->rx_start )( com_dev->lower_id,
                                     fifoBuf_getFree( &com_dev->rx ) );
    }
  }

  if ( has_tx ) {
    fifoBuf_init( &com_dev->tx, tx_buffer, tx_buffer_len );
#if defined(UVOS_INCLUDE_FREERTOS)
    vSemaphoreCreateBinary( com_dev->tx_sem );
#endif /* UVOS_INCLUDE_FREERTOS */
    ( com_dev->driver->bind_tx_cb )( lower_id, UVOS_COM_TxOutCallback, ( uint32_t )com_dev );
  }
#if defined(UVOS_INCLUDE_FREERTOS)
  com_dev->sendbuffer_sem = xSemaphoreCreateMutex();
#endif /* UVOS_INCLUDE_FREERTOS */

  *com_id = ( uint32_t )com_dev;
  return 0;

out_fail:
  return -1;
}

#if defined(UVOS_INCLUDE_FREERTOS)
static void UVOS_COM_UnblockRx( struct uvos_com_dev *com_dev, bool *need_yield )
{
  static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR( com_dev->rx_sem, &xHigherPriorityTaskWoken );

  if ( xHigherPriorityTaskWoken != pdFALSE ) {
    *need_yield = true;
  } else {
    *need_yield = false;
  }
}
#else
static void UVOS_COM_UnblockRx( __attribute__( ( unused ) ) struct uvos_com_dev *com_dev, bool *need_yield )
{
  *need_yield = false;
}
#endif

#if defined(UVOS_INCLUDE_FREERTOS)
static void UVOS_COM_UnblockTx( struct uvos_com_dev *com_dev, bool *need_yield )
{
  static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR( com_dev->tx_sem, &xHigherPriorityTaskWoken );

  if ( xHigherPriorityTaskWoken != pdFALSE ) {
    *need_yield = true;
  } else {
    *need_yield = false;
  }
}
#else
static void UVOS_COM_UnblockTx( __attribute__( ( unused ) ) struct uvos_com_dev *com_dev, bool *need_yield )
{
  *need_yield = false;
}
#endif


static uint16_t UVOS_COM_RxInCallback( uint32_t context, uint8_t *buf, uint16_t buf_len, uint16_t *headroom, bool *need_yield )
{
  struct uvos_com_dev *com_dev = ( struct uvos_com_dev * )context;

  bool valid = UVOS_COM_validate( com_dev );

  UVOS_Assert( valid );
  UVOS_Assert( com_dev->has_rx );
  uint16_t bytes_into_fifo;
  if ( buf_len == 1 ) {
    bytes_into_fifo = fifoBuf_putByte( &com_dev->rx, buf[0] );
  } else {
    bytes_into_fifo = fifoBuf_putData( &com_dev->rx, buf, buf_len );
  }
  if ( bytes_into_fifo > 0 ) {
    /* Data has been added to the buffer */
    UVOS_COM_UnblockRx( com_dev, need_yield );
  }

  if ( headroom ) {
    *headroom = fifoBuf_getFree( &com_dev->rx );
  }

  return bytes_into_fifo;
}

static uint16_t UVOS_COM_TxOutCallback( uint32_t context, uint8_t *buf, uint16_t buf_len, uint16_t *headroom, bool *need_yield )
{
  struct uvos_com_dev *com_dev = ( struct uvos_com_dev * )context;

  bool valid = UVOS_COM_validate( com_dev );

  UVOS_Assert( valid );
  UVOS_Assert( buf );
  UVOS_Assert( buf_len );
  UVOS_Assert( com_dev->has_tx );

  uint16_t bytes_from_fifo = fifoBuf_getData( &com_dev->tx, buf, buf_len );

  if ( bytes_from_fifo > 0 ) {
    /* More space has been made in the buffer */
    UVOS_COM_UnblockTx( com_dev, need_yield );
  }

  if ( headroom ) {
    *headroom = fifoBuf_getUsed( &com_dev->tx );
  }

  return bytes_from_fifo;
}

/**
 * Change the port speed without re-initializing
 * \param[in] port COM port
 * \param[in] baud Requested baud rate
 * \return -1 if port not available
 * \return 0 on success
 */
int32_t UVOS_COM_ChangeBaud( uint32_t com_id, uint32_t baud )
{
  struct uvos_com_dev *com_dev = ( struct uvos_com_dev * )com_id;

  if ( !UVOS_COM_validate( com_dev ) ) {
    /* Undefined COM port for this board (see uvos_board.c) */
    return -1;
  }

  /* Invoke the driver function if it exists */
  if ( com_dev->driver->set_baud ) {
    com_dev->driver->set_baud( com_dev->lower_id, baud );
  }

  return 0;
}

/**
 * Set baud rate callback associated with the port
 * \param[in] port COM port
 * \param[in] baud_rate_cb Callback function
 * \param[in] context context to pass to the callback function
 * \return -1 if port not available
 * \return 0 on success
 */
int32_t UVOS_COM_RegisterBaudRateCallback( uint32_t com_id, uvos_com_callback_baud_rate baud_rate_cb, uint32_t context )
{
  struct uvos_com_dev *com_dev = ( struct uvos_com_dev * )com_id;

  if ( !UVOS_COM_validate( com_dev ) ) {
    /* Undefined COM port for this board (see uvos_board.c) */
    return -1;
  }

  /* Invoke the driver function if it exists */
  if ( com_dev->driver->bind_baud_rate_cb ) {
    com_dev->driver->bind_baud_rate_cb( com_dev->lower_id, baud_rate_cb, context );
  }

  return 0;
}

static int32_t UVOS_COM_SendBufferNonBlockingInternal( struct uvos_com_dev *com_dev, const uint8_t *buffer, uint16_t len )
{
  UVOS_Assert( com_dev );
  UVOS_Assert( com_dev->has_tx );
  if ( com_dev->driver->available && !( com_dev->driver->available( com_dev->lower_id ) & COM_AVAILABLE_TX ) ) {
    /*
     * Underlying device is down/unconnected.
     * Dump our fifo contents and act like an infinite data sink.
     * Failure to do this results in stale data in the fifo as well as
     * possibly having the caller block trying to send to a device that's
     * no longer accepting data.
     */
    fifoBuf_clearData( &com_dev->tx );
    return len;
  }

  if ( len > fifoBuf_getFree( &com_dev->tx ) ) {
    /* Buffer cannot accept all requested bytes (retry) */
    return -2;
  }

  uint16_t bytes_into_fifo = fifoBuf_putData( &com_dev->tx, buffer, len );

  if ( bytes_into_fifo > 0 ) {
    /* More data has been put in the tx buffer, make sure the tx is started */
    if ( com_dev->driver->tx_start ) {
      com_dev->driver->tx_start( com_dev->lower_id,
                                 fifoBuf_getUsed( &com_dev->tx ) );
    }
  }
  return bytes_into_fifo;
}

/**
 * Sends a package over given port
 * \param[in] port COM port
 * \param[in] buffer character buffer
 * \param[in] len buffer length
 * \return -1 if port not available
 * \return -2 if non-blocking mode activated: buffer is full
 *            caller should retry until buffer is free again
 * \return -3 another thread is already sending, caller should
 *            retry until com is available again
 * \return number of bytes transmitted on success
 */
int32_t UVOS_COM_SendBufferNonBlocking( uint32_t com_id, const uint8_t *buffer, uint16_t len )
{
  struct uvos_com_dev *com_dev = ( struct uvos_com_dev * )com_id;

  if ( !UVOS_COM_validate( com_dev ) ) {
    /* Undefined COM port for this board (see uvos_board.c) */
    return -1;
  }
#if defined(UVOS_INCLUDE_FREERTOS)
  if ( xSemaphoreTake( com_dev->sendbuffer_sem, 0 ) != pdTRUE ) {
    return -3;
  }
#endif /* UVOS_INCLUDE_FREERTOS */
  int32_t ret = UVOS_COM_SendBufferNonBlockingInternal( com_dev, buffer, len );
#if defined(UVOS_INCLUDE_FREERTOS)
  xSemaphoreGive( com_dev->sendbuffer_sem );
#endif /* UVOS_INCLUDE_FREERTOS */
  return ret;
}


/**
 * Sends a package over given port
 * (blocking function)
 * \param[in] port COM port
 * \param[in] buffer character buffer
 * \param[in] len buffer length
 * \return -1 if port not available
 * \return -2 if mutex can't be taken;
 * \return -3 if data cannot be sent in the max allotted time of 5000msec
 * \return number of bytes transmitted on success
 */
int32_t UVOS_COM_SendBuffer( uint32_t com_id, const uint8_t *buffer, uint16_t len )
{
  struct uvos_com_dev *com_dev = ( struct uvos_com_dev * )com_id;

  if ( !UVOS_COM_validate( com_dev ) ) {
    /* Undefined COM port for this board (see uvos_board.c) */
    return -1;
  }
  UVOS_Assert( com_dev->has_tx );
#if defined(UVOS_INCLUDE_FREERTOS)
  if ( xSemaphoreTake( com_dev->sendbuffer_sem, 5 ) != pdTRUE ) {
    return -2;
  }
#endif /* UVOS_INCLUDE_FREERTOS */
  uint32_t max_frag_len  = fifoBuf_getSize( &com_dev->tx );
  uint32_t bytes_to_send = len;
  while ( bytes_to_send ) {
    uint32_t frag_size;

    if ( bytes_to_send > max_frag_len ) {
      frag_size = max_frag_len;
    } else {
      frag_size = bytes_to_send;
    }
    int32_t rc = UVOS_COM_SendBufferNonBlockingInternal( com_dev, buffer, frag_size );
    if ( rc >= 0 ) {
      bytes_to_send -= rc;
      buffer += rc;
    } else {
      switch ( rc ) {
      case -1:
#if defined(UVOS_INCLUDE_FREERTOS)
        xSemaphoreGive( com_dev->sendbuffer_sem );
#endif /* UVOS_INCLUDE_FREERTOS */
        /* Device is invalid, this will never work */
        return -1;

      case -2:
        /* Device is busy, wait for the underlying device to free some space and retry */
        /* Make sure the transmitter is running while we wait */
        if ( com_dev->driver->tx_start ) {
          ( com_dev->driver->tx_start )( com_dev->lower_id,
                                         fifoBuf_getUsed( &com_dev->tx ) );
        }
#if defined(UVOS_INCLUDE_FREERTOS)
        if ( xSemaphoreTake( com_dev->tx_sem, 5000 ) != pdTRUE ) {
          xSemaphoreGive( com_dev->sendbuffer_sem );
          return -3;
        }
#endif
        continue;
      default:
        /* Unhandled return code */
#if defined(UVOS_INCLUDE_FREERTOS)
        xSemaphoreGive( com_dev->sendbuffer_sem );
#endif /* UVOS_INCLUDE_FREERTOS */
        return rc;
      }
    }
  }
#if defined(UVOS_INCLUDE_FREERTOS)
  xSemaphoreGive( com_dev->sendbuffer_sem );
#endif /* UVOS_INCLUDE_FREERTOS */
  return len;
}

/**
 * Sends a single character over given port
 * \param[in] port COM port
 * \param[in] c character
 * \return -1 if port not available
 * \return -2 buffer is full
 *            caller should retry until buffer is free again
 * \return 0 on success
 */
int32_t UVOS_COM_SendCharNonBlocking( uint32_t com_id, char c )
{
  return UVOS_COM_SendBufferNonBlocking( com_id, ( uint8_t * )&c, 1 );
}

/**
 * Sends a single character over given port
 * (blocking function)
 * \param[in] port COM port
 * \param[in] c character
 * \return -1 if port not available
 * \return 0 on success
 */
int32_t UVOS_COM_SendChar( uint32_t com_id, char c )
{
  return UVOS_COM_SendBuffer( com_id, ( uint8_t * )&c, 1 );
}

/**
 * Sends a string over given port
 * \param[in] port COM port
 * \param[in] str zero-terminated string
 * \return -1 if port not available
 * \return -2 buffer is full
 *         caller should retry until buffer is free again
 * \return 0 on success
 */
int32_t UVOS_COM_SendStringNonBlocking( uint32_t com_id, const char *str )
{
  return UVOS_COM_SendBufferNonBlocking( com_id, ( uint8_t * )str, ( uint16_t )strlen( str ) );
}

/**
 * Sends a string over given port
 * (blocking function)
 * \param[in] port COM port
 * \param[in] str zero-terminated string
 * \return -1 if port not available
 * \return 0 on success
 */
int32_t UVOS_COM_SendString( uint32_t com_id, const char *str )
{
  return UVOS_COM_SendBuffer( com_id, ( uint8_t * )str, strlen( str ) );
}

/**
 * Sends a formatted string (-> printf) over given port
 * \param[in] port COM port
 * \param[in] *format zero-terminated format string - 128 characters supported maximum!
 * \param[in] ... optional arguments,
 *        128 characters supported maximum!
 * \return -2 if non-blocking mode activated: buffer is full
 *         caller should retry until buffer is free again
 * \return 0 on success
 */
int32_t UVOS_COM_SendFormattedStringNonBlocking( uint32_t com_id, const char *format, ... )
{
  uint8_t buffer[128]; // TODO: tmp!!! Provide a streamed COM method later!

  va_list args;

  va_start( args, format );
  vsprintf( ( char * )buffer, format, args );
  return UVOS_COM_SendBufferNonBlocking( com_id, buffer, ( uint16_t )strlen( ( char * )buffer ) );
}

/**
 * Sends a formatted string (-> printf) over given port
 * (blocking function)
 * \param[in] port COM port
 * \param[in] *format zero-terminated format string - 128 characters supported maximum!
 * \param[in] ... optional arguments,
 * \return -1 if port not available
 * \return 0 on success
 */
int32_t UVOS_COM_SendFormattedString( uint32_t com_id, const char *format, ... )
{
  uint8_t buffer[128]; // TODO: tmp!!! Provide a streamed COM method later!
  va_list args;

  va_start( args, format );
  vsprintf( ( char * )buffer, format, args );
  return UVOS_COM_SendBuffer( com_id, buffer, ( uint16_t )strlen( ( char * )buffer ) );
}

/**
 * Transfer bytes from port buffers into another buffer
 * \param[in] port COM port
 * \returns num received bytes from buffer
 */
uint16_t UVOS_COM_ReceiveBuffer( uint32_t com_id, uint8_t *buf, uint16_t buf_len, uint32_t timeout_ms )
{
  UVOS_Assert( buf );
  UVOS_Assert( buf_len );
  uint16_t bytes_from_fifo;

  struct uvos_com_dev *com_dev = ( struct uvos_com_dev * )com_id;

  if ( !UVOS_COM_validate( com_dev ) ) {
    /* Undefined COM port for this board (see uvos_board.c) */
    UVOS_Assert( 0 );
  }
  UVOS_Assert( com_dev->has_rx );

check_again:
  bytes_from_fifo = fifoBuf_getData( &com_dev->rx, buf, buf_len );

  if ( bytes_from_fifo == 0 ) {
    /* No more bytes in receive buffer */
    /* Make sure the receiver is running while we wait */
    if ( com_dev->driver->rx_start ) {
      /* Notify the lower layer that there is now room in the rx buffer */
      ( com_dev->driver->rx_start )( com_dev->lower_id,
                                     fifoBuf_getFree( &com_dev->rx ) );
    }
    if ( timeout_ms > 0 ) {
#if defined(UVOS_INCLUDE_FREERTOS)
      if ( xSemaphoreTake( com_dev->rx_sem, timeout_ms / portTICK_RATE_MS ) == pdTRUE ) {
        /* Make sure we don't come back here again */
        timeout_ms = 0;
        goto check_again;
      }
#else
      UVOS_DELAY_WaitmS( 1 );
      timeout_ms--;
      goto check_again;
#endif
    }
  }

  /* Return num received bytes */
  return bytes_from_fifo;
}

/**
 * Query if a com port has data available for RX in buffer.
 */
int32_t UVOS_COM_ReceiveChar( uint32_t com_id, char *c )
{
  char buf;
  uint16_t bytes_from_fifo;

  struct uvos_com_dev *com_dev = ( struct uvos_com_dev * )com_id;

  if ( !UVOS_COM_validate( com_dev ) ) {
    /* Undefined COM port for this board (see uvos_board.c) */
    UVOS_Assert( 0 );
  }
  UVOS_Assert( com_dev->has_rx );

  /* Note: fifoBuf_getData() returns num bytes copied */
  bytes_from_fifo = fifoBuf_getData( &com_dev->rx, &buf, 1 );
  if ( bytes_from_fifo < 1 ) {
    return -1;
  }

  *c = buf;
  return 0;
}

/**
 * Query if a com port has data available for RX in buffer.
 */
int32_t UVOS_COM_RX_Data_Available ( uint32_t com_id )
{
  struct uvos_com_dev *com_dev = ( struct uvos_com_dev * )com_id;

  if ( !UVOS_COM_validate( com_dev ) ) {
    /* Undefined COM port for this board (see uvos_board.c) */
    UVOS_Assert( 0 );
  }
  UVOS_Assert( com_dev->has_rx );

  /* Note: fifoBuf_getUsed() returns num bytes available in rx buffer */
  return fifoBuf_getUsed( &com_dev->rx );
}

/**
 * Query if a com port is available for use.  That can be
 * used to check a link is established even if the device
 * is valid.
 */
uint32_t UVOS_COM_Available( uint32_t com_id )
{
  struct uvos_com_dev *com_dev = ( struct uvos_com_dev * )com_id;

  if ( !UVOS_COM_validate( com_dev ) ) {
    return COM_AVAILABLE_NONE;
  }

  // If a driver does not provide a query method assume always
  // available if valid
  if ( com_dev->driver->available == NULL ) {
    if ( com_dev->has_rx && com_dev->has_tx ) {
      return COM_AVAILABLE_RXTX;
    } else if ( com_dev->has_rx ) {
      return COM_AVAILABLE_RX;
    } else if ( com_dev->has_tx ) {
      return COM_AVAILABLE_TX;
    }

    return COM_AVAILABLE_NONE; /* can this really happen? */
  }

  return ( com_dev->driver->available )( com_dev->lower_id );
}

#endif /* UVOS_INCLUDE_COM */

/**
 * @}
 * @}
 */
