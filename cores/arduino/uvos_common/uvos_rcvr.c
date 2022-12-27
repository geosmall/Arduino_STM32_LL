#include "uvos.h"

#ifdef UVOS_INCLUDE_RCVR

#include <uvos_rcvr_priv.h>

enum uvos_rcvr_dev_magic {
  UVOS_RCVR_DEV_MAGIC = 0x99aabbcc,
};

struct uvos_rcvr_dev {
  enum uvos_rcvr_dev_magic magic;
  uint32_t lower_id;
  const struct uvos_rcvr_driver * driver;
};

static bool UVOS_RCVR_validate( struct uvos_rcvr_dev * rcvr_dev )
{
  return rcvr_dev->magic == UVOS_RCVR_DEV_MAGIC;
}

#if defined(UVOS_INCLUDE_FREERTOS)
static struct uvos_rcvr_dev * UVOS_RCVR_alloc( void )
{
  struct uvos_rcvr_dev * rcvr_dev;

  rcvr_dev = ( struct uvos_rcvr_dev * )UVOS_malloc( sizeof( *rcvr_dev ) );
  if ( !rcvr_dev ) {
    return NULL;
  }

  rcvr_dev->magic = UVOS_RCVR_DEV_MAGIC;
  return rcvr_dev;
}
#else
static struct uvos_rcvr_dev uvos_rcvr_devs[ UVOS_RCVR_MAX_DEVS ];
static uint8_t uvos_rcvr_num_devs;
static struct uvos_rcvr_dev * UVOS_RCVR_alloc( void )
{
  struct uvos_rcvr_dev * rcvr_dev;

  if ( uvos_rcvr_num_devs >= UVOS_RCVR_MAX_DEVS ) {
    return NULL;
  }

  rcvr_dev = &uvos_rcvr_devs[uvos_rcvr_num_devs++];
  rcvr_dev->magic = UVOS_RCVR_DEV_MAGIC;

  return rcvr_dev;
}
#endif /* if defined(UVOS_INCLUDE_FREERTOS) */

/**
 * Initialises RCVR layer
 * \param[out] handle
 * \param[in] driver
 * \param[in] id
 * \return < 0 if initialisation failed
 */
int32_t UVOS_RCVR_Init( uint32_t * rcvr_id, const struct uvos_rcvr_driver * driver, uint32_t lower_id )
{
  UVOS_DEBUG_Assert( rcvr_id );
  UVOS_DEBUG_Assert( driver );

  struct uvos_rcvr_dev * rcvr_dev;

  rcvr_dev = ( struct uvos_rcvr_dev * )UVOS_RCVR_alloc();
  if ( !rcvr_dev ) {
    goto out_fail;
  }

  rcvr_dev->driver   = driver;
  rcvr_dev->lower_id = lower_id;

  *rcvr_id = ( uint32_t )rcvr_dev;
  return 0;

out_fail:
  return -1;
}

/**
 * @brief Reads an input channel from the appropriate driver
 * @param[in] rcvr_id driver to read from
 * @param[in] channel channel to read
 * @returns Unitless input value
 *  @retval UVOS_RCVR_TIMEOUT indicates a failsafe or timeout from that channel
 *  @retval UVOS_RCVR_INVALID invalid channel for this driver (usually out of range supported)
 *  @retval UVOS_RCVR_NODRIVER driver was not initialized
 */
int32_t UVOS_RCVR_Read( uint32_t rcvr_id, uint8_t channel )
{
  // Publicly facing API uses channel 1 for first channel
  if ( channel == 0 ) {
    return UVOS_RCVR_INVALID;
  } else {
    channel--;
  }

  if ( rcvr_id == 0 ) {
    return UVOS_RCVR_NODRIVER;
  }

  struct uvos_rcvr_dev * rcvr_dev = ( struct uvos_rcvr_dev * )rcvr_id;

  if ( !UVOS_RCVR_validate( rcvr_dev ) ) {
    /* Undefined RCVR port for this board (see uvos_board.c) */
    UVOS_Assert( 0 );
  }

  UVOS_DEBUG_Assert( rcvr_dev->driver->read );

  return rcvr_dev->driver->read( rcvr_dev->lower_id, channel );
}

/**
 * @brief Reads input quality from the appropriate driver
 * @param[in] rcvr_id driver to read from
 * @returns received signal quality expressed as a %
 *  @retval UVOS_RCVR_NODRIVER driver was not initialized
 */
uint8_t UVOS_RCVR_GetQuality( uint32_t rcvr_id )
{
  if ( rcvr_id == 0 ) {
    return UVOS_RCVR_NODRIVER;
  }

  struct uvos_rcvr_dev * rcvr_dev = ( struct uvos_rcvr_dev * )rcvr_id;

  if ( !UVOS_RCVR_validate( rcvr_dev ) ) {
    /* Undefined RCVR port for this board (see uvos_board.c) */
    /* As no receiver is available assume min */
    return 0;
  }

  if ( !rcvr_dev->driver->get_quality ) {
    /* If no quality is available assume max */
    return 100;
  }

  return rcvr_dev->driver->get_quality( rcvr_dev->lower_id );
}

#if defined( UVOS_INCLUDE_FREERTOS )

/**
 * @brief Get a semaphore that signals when a new sample is available.
 * @param[in] rcvr_id driver to read from
 * @param[in] channel channel to read
 * @returns The semaphore, or NULL if not supported.
 */
xSemaphoreHandle UVOS_RCVR_GetSemaphore( uint32_t rcvr_id, uint8_t channel )
{
  // Publicly facing API uses channel 1 for first channel
  if ( channel == 0 ) {
    return NULL;
  } else {
    channel--;
  }

  if ( rcvr_id == 0 ) {
    return NULL;
  }

  struct uvos_rcvr_dev * rcvr_dev = ( struct uvos_rcvr_dev * )rcvr_id;

  if ( !UVOS_RCVR_validate( rcvr_dev ) ) {
    /* Undefined RCVR port for this board (see uvos_board.c) */
    UVOS_Assert( 0 );
  }

  if ( rcvr_dev->driver->get_semaphore ) {
    return rcvr_dev->driver->get_semaphore( rcvr_dev->lower_id, channel );
  }
  return NULL;
}

#else

/**
 * @brief Get a flag that signals when a new rcvr frame is available.
 * @param[in] rcvr_id driver to read from
 * @returns The flag, or NULL if not supported.
 */
bool UVOS_RCVR_GetFrameAvailableFlag( uint32_t rcvr_id )
{

  if ( rcvr_id == 0 ) {
    return false;
  }

  struct uvos_rcvr_dev * rcvr_dev = ( struct uvos_rcvr_dev * )rcvr_id;

  if ( !UVOS_RCVR_validate( rcvr_dev ) ) {
    /* Undefined RCVR port for this board (see uvos_board.c) */
    UVOS_Assert( 0 );
  }

  if ( rcvr_dev->driver->get_flag ) {
    return rcvr_dev->driver->get_flag( rcvr_dev->lower_id );
  }
  return false;
}

#endif // defined( UVOS_INCLUDE_FREERTOS )

#endif /* UVOS_INCLUDE_RCVR */

/**
 * @}
 * @}
 */
