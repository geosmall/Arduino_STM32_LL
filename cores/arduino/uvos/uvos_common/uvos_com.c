/* Project Includes */
#include "uvos.h"

#if defined(UVOS_INCLUDE_COM)

#include "fifo_buffer.h"
#include <uvos_com_priv.h>

#if !defined(UVOS_INCLUDE_FREERTOS) && !defined(UVOS_INCLUDE_CHIBIOS)
#include "uvos_delay.h"   /* UVOS_DELAY_WaitmS */
#endif

#include "uvos_semaphore.h"
#include "uvos_mutex.h"

enum uvos_com_dev_magic {
  UVOS_COM_DEV_MAGIC = 0xaa55aa55,
};

struct uvos_com_dev {
  enum uvos_com_dev_magic magic;
  uint32_t lower_id;
  const struct uvos_com_driver * driver;

// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
  struct uvos_semaphore *tx_sem;
  struct uvos_semaphore *rx_sem;
  struct uvos_mutex *sendbuffer_mtx;
// #endif

  bool has_rx;
  bool has_tx;

  t_fifo_buffer rx;
  t_fifo_buffer tx;
};

static bool UVOS_COM_validate(struct uvos_com_dev * com_dev)
{
  return (com_dev && (com_dev->magic == UVOS_COM_DEV_MAGIC));
}

static struct uvos_com_dev * UVOS_COM_alloc(void)
{
  struct uvos_com_dev * com_dev;

  com_dev = (struct uvos_com_dev *)UVOS_malloc(sizeof(*com_dev));
  if (!com_dev) return (NULL);

  memset(com_dev, 0, sizeof(*com_dev));
  com_dev->magic = UVOS_COM_DEV_MAGIC;
  return(com_dev);
}

static uint16_t UVOS_COM_TxOutCallback(uint32_t context, uint8_t * buf, uint16_t buf_len, uint16_t * headroom, bool * need_yield);
static uint16_t UVOS_COM_RxInCallback(uint32_t context, uint8_t * buf, uint16_t buf_len, uint16_t * headroom, bool * need_yield);
static void UVOS_COM_UnblockRx(struct uvos_com_dev * com_dev, bool * need_yield);
static void UVOS_COM_UnblockTx(struct uvos_com_dev * com_dev, bool * need_yield);

/**
  * Initialises COM layer
  * \param[out] handle
  * \param[in] driver
  * \param[in] id
  * \return < 0 if initialisation failed
  */
int32_t UVOS_COM_Init(uint32_t * com_id, const struct uvos_com_driver * driver, uint32_t lower_id, uint8_t * rx_buffer, uint16_t rx_buffer_len, uint8_t * tx_buffer, uint16_t tx_buffer_len)
{
  UVOS_Assert(com_id);
  UVOS_Assert(driver);

  bool has_rx = (rx_buffer && rx_buffer_len > 0);
  bool has_tx = (tx_buffer && tx_buffer_len > 0);
  UVOS_Assert(has_rx || has_tx);
  UVOS_Assert(driver->bind_tx_cb || !has_tx);
  UVOS_Assert(driver->bind_rx_cb || !has_rx);

  struct uvos_com_dev * com_dev;

  com_dev = (struct uvos_com_dev *) UVOS_COM_alloc();
  if (!com_dev) goto out_fail;

  com_dev->driver   = driver;
  com_dev->lower_id = lower_id;

  com_dev->has_rx = has_rx;
  com_dev->has_tx = has_tx;

  if (has_rx) {
    fifoBuf_init(&com_dev->rx, rx_buffer, rx_buffer_len);
// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
    com_dev->rx_sem = UVOS_Semaphore_Create();
// #endif  /* UVOS_INCLUDE_FREERTOS */
    (com_dev->driver->bind_rx_cb)(lower_id, UVOS_COM_RxInCallback, (uint32_t)com_dev);
    if (com_dev->driver->rx_start) {
      /* Start the receiver */
      (com_dev->driver->rx_start)(com_dev->lower_id, fifoBuf_getFree(&com_dev->rx));
    }
  }

  if (has_tx) {
    fifoBuf_init(&com_dev->tx, tx_buffer, tx_buffer_len);
// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
    com_dev->tx_sem = UVOS_Semaphore_Create();
// #endif  /* UVOS_INCLUDE_FREERTOS */
    (com_dev->driver->bind_tx_cb)(lower_id, UVOS_COM_TxOutCallback, (uint32_t)com_dev);
  }
// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
  com_dev->sendbuffer_mtx = UVOS_Mutex_Create();
// #endif /* UVOS_INCLUDE_FREERTOS */

  *com_id = (uint32_t)com_dev;
  return(0);

out_fail:
  return(-1);
}

static void UVOS_COM_UnblockRx(struct uvos_com_dev * com_dev, bool * need_yield)
{
// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
  if (UVOS_IRQ_InISR() == true)
    UVOS_Semaphore_Give_FromISR(com_dev->rx_sem, need_yield);
  else
    UVOS_Semaphore_Give(com_dev->rx_sem);
// #endif
}

static void UVOS_COM_UnblockTx(struct uvos_com_dev * com_dev, bool * need_yield)
{
// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
  if (UVOS_IRQ_InISR() == true)
    UVOS_Semaphore_Give_FromISR(com_dev->tx_sem, need_yield);
  else
    UVOS_Semaphore_Give(com_dev->tx_sem);
// #endif
}

static uint16_t UVOS_COM_RxInCallback(uint32_t context, uint8_t * buf, uint16_t buf_len, uint16_t * headroom, bool * need_yield)
{
  struct uvos_com_dev * com_dev = (struct uvos_com_dev *)context;

  bool valid = UVOS_COM_validate(com_dev);
  UVOS_Assert(valid);
  UVOS_Assert(com_dev->has_rx);

  uint16_t bytes_into_fifo = fifoBuf_putData(&com_dev->rx, buf, buf_len);

  if (bytes_into_fifo > 0) {
    /* Data has been added to the buffer */
    UVOS_COM_UnblockRx(com_dev, need_yield);
  }

  if (headroom) {
    *headroom = fifoBuf_getFree(&com_dev->rx);
  }

  return (bytes_into_fifo);
}

/**
 * Callback handler from lower level COM driver, manages COM device fifoBuf
 * This callback gets data from the COM device fifoBuf and copies it into the supplied lower level device buffer.  If
 * bytes copied is greater than zero it will unblock the COM device Tx.
 * \param[in] context lower level COM port device id from which callback is called
 * \param[in] *buf pointer to lower level device buffer
 * \param[in] buf_len size in bytes of lower level device buffer
 * \param[in] *headroom the number of bytes available in the COM device fifoBuf
 * \param[in] *need_yield true if higher priority task has been woken (context switch should be requested before interrupt exits)
 * \return number of bytes copied from COM device fifoBuf into lower level device buffer
 */
static uint16_t UVOS_COM_TxOutCallback( uint32_t context, uint8_t *buf, uint16_t buf_len, uint16_t *headroom, bool *need_yield )
{
  struct uvos_com_dev * com_dev = (struct uvos_com_dev *)context;

  bool valid = UVOS_COM_validate(com_dev);
  UVOS_Assert(valid);
  UVOS_Assert(buf);
  UVOS_Assert(buf_len);
  UVOS_Assert(com_dev->has_tx);

  uint16_t bytes_from_fifo = fifoBuf_getData(&com_dev->tx, buf, buf_len);

  if (bytes_from_fifo > 0) {
    /* More space has been made in the buffer */
    UVOS_COM_UnblockTx(com_dev, need_yield);
  }

  if (headroom) {
    *headroom = fifoBuf_getUsed(&com_dev->tx);
  }

  return (bytes_from_fifo);
}

/**
* Change the port speed without re-initializing
* \param[in] port COM port
* \param[in] baud Requested baud rate
* \return -1 if port not available
* \return 0 on success
*/
int32_t UVOS_COM_ChangeBaud(uint32_t com_id, uint32_t baud)
{
  struct uvos_com_dev * com_dev = (struct uvos_com_dev *)com_id;

  if (!UVOS_COM_validate(com_dev)) {
    /* Undefined COM port for this board (see uvos_board.c) */
    return -1;
  }

  /* Invoke the driver function if it exists */
  if (com_dev->driver->set_baud) {
    com_dev->driver->set_baud(com_dev->lower_id, baud);
  }

  return 0;
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
int32_t UVOS_COM_SendBufferNonBlocking(uint32_t com_id, const uint8_t *buffer, uint16_t len)
{
  struct uvos_com_dev * com_dev = (struct uvos_com_dev *)com_id;

  if (!UVOS_COM_validate(com_dev)) {
    /* Undefined COM port for this board (see uvos_board.c) */
    return -1;
  }

  UVOS_Assert(com_dev->has_tx);

// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
  if (UVOS_Mutex_Lock(com_dev->sendbuffer_mtx, 0) != true) {
    return -3;
  }
// #endif /* defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS) */
  if (com_dev->driver->available && !com_dev->driver->available(com_dev->lower_id)) {
    /*
     * Underlying device is down/unconnected.
     * Dump our fifo contents and act like an infinite data sink.
     * Failure to do this results in stale data in the fifo as well as
     * possibly having the caller block trying to send to a device that's
     * no longer accepting data.
     */
    fifoBuf_clearData(&com_dev->tx);
// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
    UVOS_Mutex_Unlock(com_dev->sendbuffer_mtx);
// #endif /* UVOS_INCLUDE_FREERTOS */

    return len;
  }

  if (len > fifoBuf_getFree(&com_dev->tx)) {
// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
    UVOS_Mutex_Unlock(com_dev->sendbuffer_mtx);
// #endif /* UVOS_INCLUDE_FREERTOS */
    /* Buffer cannot accept all requested bytes (retry) */
    return -2;
  }

  uint16_t bytes_into_fifo = fifoBuf_putData(&com_dev->tx, buffer, len);

  if (bytes_into_fifo > 0) {
    /* More data has been put in the tx buffer, make sure the tx is started */
    if (com_dev->driver->tx_start) {
      com_dev->driver->tx_start(com_dev->lower_id, fifoBuf_getUsed(&com_dev->tx));
    }
  }

// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
  UVOS_Mutex_Unlock(com_dev->sendbuffer_mtx);
// #endif /* UVOS_INCLUDE_FREERTOS */
  return (bytes_into_fifo);
}

/**
* Sends a package over given port
* (blocking function)
* \param[in] port COM port
* \param[in] buffer character buffer
* \param[in] len buffer length
* \return -1 if port not available
* \return number of bytes transmitted on success
*/
int32_t UVOS_COM_SendBuffer(uint32_t com_id, const uint8_t *buffer, uint16_t len)
{
  struct uvos_com_dev * com_dev = (struct uvos_com_dev *)com_id;

  if (!UVOS_COM_validate(com_dev)) {
    /* Undefined COM port for this board (see uvos_board.c) */
    return -1;
  }

  UVOS_Assert(com_dev->has_tx);

  uint32_t max_frag_len = fifoBuf_getSize(&com_dev->tx);
  uint32_t bytes_to_send = len;
  while (bytes_to_send) {
    uint32_t frag_size;

    if (bytes_to_send > max_frag_len) {
      frag_size = max_frag_len;
    } else {
      frag_size = bytes_to_send;
    }
    int32_t ret_code = UVOS_COM_SendBufferNonBlocking(com_id, buffer, frag_size);
    /* ret_code >= 0 is number of bytes transmitted on success */
    if (ret_code >= 0) {
      bytes_to_send -= ret_code;
      buffer += ret_code;
    /* ret_code < 0 is error of some form */
    } else {
      switch (ret_code) {
      case -1:
        /* Device is invalid, this will never send */
        return -1;
      case -2:
        /* Device is busy, wait for the underlying device to free some space and retry */
        /* Make sure the transmitter is running while we wait */
        if (com_dev->driver->tx_start) {
          (com_dev->driver->tx_start)(com_dev->lower_id, fifoBuf_getUsed(&com_dev->tx));
        }
// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
        if (UVOS_Semaphore_Take(com_dev->tx_sem, 5000) != true) {
          return -3;
        }
// #endif
        continue;
      default:
        /* Unhandled return code */
        return ret_code;
      }
    }
  }

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
int32_t UVOS_COM_SendCharNonBlocking(uint32_t com_id, char c)
{
  return UVOS_COM_SendBufferNonBlocking(com_id, (uint8_t *)&c, 1);
}

/**
* Sends a single character over given port
* (blocking function)
* \param[in] port COM port
* \param[in] c character
* \return -1 if port not available
* \return 0 on success
*/
int32_t UVOS_COM_SendChar(uint32_t com_id, char c)
{
  return UVOS_COM_SendBuffer(com_id, (uint8_t *)&c, 1);
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
int32_t UVOS_COM_SendStringNonBlocking(uint32_t com_id, const char *str)
{
  return UVOS_COM_SendBufferNonBlocking(com_id, (uint8_t *)str, (uint16_t)strlen(str));
}

/**
* Sends a string over given port
* (blocking function)
* \param[in] port COM port
* \param[in] str zero-terminated string
* \return -1 if port not available
* \return 0 on success
*/
int32_t UVOS_COM_SendString(uint32_t com_id, const char *str)
{
  return UVOS_COM_SendBuffer(com_id, (uint8_t *)str, strlen(str));
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
int32_t UVOS_COM_SendFormattedStringNonBlocking(uint32_t com_id, const char *format, ...)
{
  uint8_t buffer[128]; // TODO: tmp!!! Provide a streamed COM method later!

  va_list args;

  va_start(args, format);
  vsprintf((char *)buffer, format, args);
  return UVOS_COM_SendBufferNonBlocking(com_id, buffer, (uint16_t)strlen((char *)buffer));
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
int32_t UVOS_COM_SendFormattedString(uint32_t com_id, const char *format, ...)
{
  uint8_t buffer[128]; // TODO: tmp!!! Provide a streamed COM method later!
  va_list args;

  va_start(args, format);
  vsprintf((char *)buffer, format, args);
  return UVOS_COM_SendBuffer(com_id, buffer, (uint16_t)strlen((char *)buffer));
}

/**
* Transfer bytes from port buffers into another buffer
* \param[in] port COM port
* \returns Byte from buffer
*/
uint16_t UVOS_COM_ReceiveBuffer(uint32_t com_id, uint8_t * buf, uint16_t buf_len, uint32_t timeout_ms)
{
  UVOS_Assert(buf);
  UVOS_Assert(buf_len);
  uint16_t bytes_from_fifo;

  struct uvos_com_dev * com_dev = (struct uvos_com_dev *)com_id;

  if (!UVOS_COM_validate(com_dev)) {
    /* Undefined COM port for this board (see uvos_board.c) */
    UVOS_Assert(0);
  }
  UVOS_Assert(com_dev->has_rx);

 check_again:
  bytes_from_fifo = fifoBuf_getData(&com_dev->rx, buf, buf_len);

  if (bytes_from_fifo == 0) {
    /* No more bytes in receive buffer */
    /* Make sure the receiver is running while we wait */
    if (com_dev->driver->rx_start) {
      /* Notify the lower layer that there is now room in the rx buffer */
      (com_dev->driver->rx_start)(com_dev->lower_id,
                fifoBuf_getFree(&com_dev->rx));
    }
    if (timeout_ms > 0) {
// #if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
      if (UVOS_Semaphore_Take(com_dev->rx_sem, timeout_ms) == true) {
        /* Make sure we don't come back here again */
        timeout_ms = 0;
        goto check_again;
      }
// #else
//       UVOS_DELAY_WaitmS(1);
//       timeout_ms--;
//       goto check_again;
// #endif
    }
  }

  /* Return received byte */
  return (bytes_from_fifo);
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
bool UVOS_COM_Available(uint32_t com_id)
{
  struct uvos_com_dev * com_dev = (struct uvos_com_dev *)com_id;

  if (!UVOS_COM_validate(com_dev)) {
    return false;
  }

  // If a driver does not provide a query method assume always
  // available if valid
  if (com_dev->driver->available == NULL)
    return true;

  return (com_dev->driver->available)(com_dev->lower_id);
}

#endif
