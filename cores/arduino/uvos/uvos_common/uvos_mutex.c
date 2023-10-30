#include "uvos.h"
#include "uvos_mutex.h"

#if !defined(UVOS_INCLUDE_FREERTOS) && !defined(UVOS_INCLUDE_CHIBIOS) && !defined(UVOS_INCLUDE_IRQ)
#error "uvos_semaphore.c requires either UVOS_INCLUDE_FREERTOS, UVOS_INCLUDE_CHIBIOS or UVOS_INCLUDE_IRQ to be defined"
#endif

#if defined(UVOS_INCLUDE_FREERTOS)

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// portTICK_RATE_MS is in [ms/tick].
// See http://sourceforge.net/tracker/?func=detail&aid=3498382&group_id=111543&atid=659636
#define TICKS2MS(t) ((t) * (portTICK_RATE_MS))
#define MS2TICKS(m) ((m) / (portTICK_RATE_MS))

/**
 *
 * @brief   Creates a non recursive mutex.
 *
 * @returns instance of @p struct uvos_mutex or NULL on failure
 *
 */
struct uvos_mutex *UVOS_Mutex_Create( void )
{
  struct uvos_mutex *mtx = UVOS_malloc_no_dma( sizeof( struct uvos_mutex ) );

  if ( mtx == NULL )
    return NULL;

  mtx->mtx_handle = ( uintptr_t )xSemaphoreCreateMutex();

  return mtx;
}

/**
 *
 * @brief   Locks a non recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_mutex
 * @param[in] timeout_ms   timeout for acquiring the lock in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Mutex_Lock( struct uvos_mutex *mtx, uint32_t timeout_ms )
{
  UVOS_Assert( mtx != NULL );

  portTickType timeout_ticks;
  if ( timeout_ms == UVOS_MUTEX_TIMEOUT_MAX )
    timeout_ticks = portMAX_DELAY;
  else
    timeout_ticks = MS2TICKS( timeout_ms );

  return xSemaphoreTake( mtx->mtx_handle, timeout_ticks ) == pdTRUE;
}

/**
 *
 * @brief   Unlocks a non recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_mutex
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Mutex_Unlock( struct uvos_mutex *mtx )
{
  UVOS_Assert( mtx != NULL );

  return xSemaphoreGive( mtx->mtx_handle ) == pdTRUE;
}

/**
 *
 * @brief   Creates a recursive mutex.
 *
 * @returns instance of @p struct uvos_recursive mutex or NULL on failure
 *
 */
struct uvos_recursive_mutex *UVOS_Recursive_Mutex_Create( void )
{
  struct uvos_recursive_mutex *mtx = UVOS_malloc_no_dma( sizeof( struct uvos_recursive_mutex ) );

  if ( mtx == NULL )
    return NULL;

  mtx->mtx_handle = ( uintptr_t )xSemaphoreCreateRecursiveMutex();

  return mtx;
}

/**
 *
 * @brief   Locks a recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_recursive_mutex
 * @param[in] timeout_ms   timeout for acquiring the lock in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Recursive_Mutex_Lock( struct uvos_recursive_mutex *mtx, uint32_t timeout_ms )
{
  UVOS_Assert( mtx != NULL );

  portTickType timeout_ticks;
  if ( timeout_ms == UVOS_MUTEX_TIMEOUT_MAX )
    timeout_ticks = portMAX_DELAY;
  else
    timeout_ticks = MS2TICKS( timeout_ms );

  return xSemaphoreTakeRecursive( ( xSemaphoreHandle )mtx->mtx_handle, timeout_ticks ) == pdTRUE;
}

/**
 *
 * @brief   Unlocks a recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_recursive_mutex
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Recursive_Mutex_Unlock( struct uvos_recursive_mutex *mtx )
{
  UVOS_Assert( mtx != NULL );

  return xSemaphoreGiveRecursive( ( xSemaphoreHandle )mtx->mtx_handle ) == pdTRUE;
}

#elif defined(UVOS_INCLUDE_CHIBIOS)

/**
 *
 * @brief   Creates a non recursive mutex.
 *
 * @returns instance of @p struct uvos_mutex or NULL on failure
 *
 */
struct uvos_mutex *UVOS_Mutex_Create( void )
{
  struct uvos_mutex *mtx = UVOS_malloc_no_dma( sizeof( struct uvos_mutex ) );

  if ( mtx == NULL )
    return NULL;

  chMtxInit( &mtx->mtx );

  return mtx;
}

/**
 *
 * @brief   Locks a non recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_mutex
 * @param[in] timeout_ms   timeout for acquiring the lock in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Mutex_Lock( struct uvos_mutex *mtx, uint32_t timeout_ms )
{
  UVOS_Assert( mtx != NULL );

  chMtxLock( &mtx->mtx );

  return true;
}

/**
 *
 * @brief   Unlocks a non recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_mutex
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Mutex_Unlock( struct uvos_mutex *mtx )
{
  UVOS_Assert( mtx != NULL );

  chMtxUnlock();

  return true;
}

/**
 *
 * @brief   Creates a recursive mutex.
 *
 * @returns instance of @p struct uvos_recursive mutex or NULL on failure
 *
 */
struct uvos_recursive_mutex *UVOS_Recursive_Mutex_Create( void )
{
  struct uvos_recursive_mutex *mtx = UVOS_malloc_no_dma( sizeof( struct uvos_recursive_mutex ) );

  if ( mtx == NULL )
    return NULL;

  chMtxInit( &mtx->mtx );
  mtx->count = 0;

  return mtx;
}

/**
 *
 * @brief   Locks a recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_recursive_mutex
 * @param[in] timeout_ms   timeout for acquiring the lock in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Recursive_Mutex_Lock( struct uvos_recursive_mutex *mtx, uint32_t timeout_ms )
{
  UVOS_Assert( mtx != NULL );

  chSysLock();

  if ( chThdSelf() != mtx->mtx.m_owner )
    chMtxLockS( &mtx->mtx );

  ++mtx->count;

  chSysUnlock();

  return true;
}

/**
 *
 * @brief   Unlocks a non recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_mutex
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Recursive_Mutex_Unlock( struct uvos_recursive_mutex *mtx )
{
  UVOS_Assert( mtx != NULL );

  chSysLock();

  --mtx->count;

  if ( mtx->count == 0 )
    chMtxUnlockS();

  chSysUnlock();

  return true;
}

#elif defined(UVOS_INCLUDE_IRQ)

struct uvos_mutex *UVOS_Mutex_Create( void )
{
  struct uvos_mutex *mtx = UVOS_malloc_no_dma( sizeof( struct uvos_mutex ) );

  if ( mtx == NULL ) {
    return NULL;
  }

  /* Initial state of a mutex is "unlocked" */
  mtx->mtx_is_locked = false;

  return mtx;
}

/**
 *
 * @brief   Locks a non recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_mutex
 * @param[in] timeout_ms   timeout for acquiring the lock in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Mutex_Lock( struct uvos_mutex *mtx, uint32_t timeout_ms )
{
  UVOS_Assert( mtx != NULL );
  bool mtx_locked_success = false;

  uint32_t start = UVOS_DELAY_GetRaw();

  uint32_t temp_mtx_locked;
  do {
    UVOS_IRQ_Disable();
    temp_mtx_locked = mtx->mtx_is_locked;
    if ( ( temp_mtx_locked ) == false ) {
      mtx->mtx_is_locked = true;
      mtx_locked_success = true;
    }
    UVOS_IRQ_Enable();
  } while ( mtx_locked_success == false && UVOS_DELAY_DiffuS( start ) < timeout_ms * 1000 );

  return mtx_locked_success;
}

/**
 *
 * @brief   Unlocks a non recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_mutex
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Mutex_Unlock( struct uvos_mutex *mtx )
{
  UVOS_Assert( mtx != NULL );
  bool mtx_unlocked_success = true;

  UVOS_IRQ_Disable();

  if ( mtx->mtx_is_locked == true ) {
    mtx->mtx_is_locked = false;
  } else {
    mtx_unlocked_success = false;
  }

  UVOS_IRQ_Enable();

  return mtx_unlocked_success;
}


struct uvos_recursive_mutex *UVOS_Recursive_Mutex_Create( void )
{
  struct uvos_mutex *mtx = UVOS_malloc_no_dma( sizeof( struct uvos_mutex ) );

  if ( mtx == NULL ) {
    return NULL;
  }

  /* Initial state of a mutex is "unlocked" */
  mtx->mtx_is_locked = false;

  return mtx;
}

/**
 *
 * @brief   Locks a non recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_mutex
 * @param[in] timeout_ms   timeout for acquiring the lock in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Recursive_Mutex_Lock( struct uvos_recursive_mutex *mtx, uint32_t timeout_ms )
{
  UVOS_Assert( mtx != NULL );
  bool mtx_locked_success = false;

  uint32_t start = UVOS_DELAY_GetRaw();

  uint32_t temp_mtx_locked;
  do {
    UVOS_IRQ_Disable();
    temp_mtx_locked = mtx->mtx_is_locked;
    if ( ( temp_mtx_locked ) == false ) {
      mtx->mtx_is_locked = true;
      mtx_locked_success = true;
    }
    UVOS_IRQ_Enable();
  } while ( mtx_locked_success == false && UVOS_DELAY_DiffuS( start ) < timeout_ms * 1000 );

  return mtx_locked_success;
}

/**
 *
 * @brief   Unlocks a non recursive mutex.
 *
 * @param[in] mtx          pointer to instance of @p struct uvos_mutex
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Recursive_Mutex_Unlock( struct uvos_recursive_mutex *mtx )
{
  UVOS_Assert( mtx != NULL );
  bool mtx_unlocked_success = true;

  UVOS_IRQ_Disable();

  if ( mtx->mtx_is_locked == true ) {
    mtx->mtx_is_locked = false;
  } else {
    mtx_unlocked_success = false;
  }

  UVOS_IRQ_Enable();

  return mtx_unlocked_success;
}

#endif /* defined(UVOS_INCLUDE_IRQ) */

