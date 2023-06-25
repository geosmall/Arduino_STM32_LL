// Inspired by https://github.com/TauLabs/TauLabs

#include "uvos.h"
#include "uvos_queue.h"

// #if !defined(UVOS_INCLUDE_FREERTOS) && !defined(UVOS_INCLUDE_CHIBIOS)
// #error "uvos_queue.c requires UVOS_INCLUDE_FREERTOS or UVOS_INCLUDE_CHIBIOS"
// #endif

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
 * @brief   Creates a queue.
 *
 * @returns instance of @p struct uvos_queue or NULL on failure
 *
 */
struct uvos_queue *UVOS_Queue_Create( size_t queue_length, size_t item_size )
{
	struct uvos_queue *queuep = UVOS_malloc_no_dma( sizeof( struct uvos_queue ) );

	if ( queuep == NULL )
		return NULL;

	queuep->queue_handle = ( uintptr_t )NULL;

	if ( ( queuep->queue_handle = ( uintptr_t )xQueueCreate( queue_length, item_size ) ) == ( uintptr_t )NULL ) {
		UVOS_free( queuep );
		return NULL;
	}

	return queuep;
}

/**
 *
 * @brief   Destroys an instance of @p struct uvos_queue
 *
 * @param[in] queuep       pointer to instance of @p struct uvos_queue
 *
 */
void UVOS_Queue_Delete( struct uvos_queue *queuep )
{
	vQueueDelete( ( xQueueHandle )queuep->queue_handle );
	UVOS_free( queuep );
}

/**
 *
 * @brief   Appends an item to a queue.
 *
 * @param[in] queuep       pointer to instance of @p struct uvos_queue
 * @param[in] itemp        pointer to item which will be appended to the queue
 * @param[in] timeout_ms   timeout for appending item to queue in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Queue_Send( struct uvos_queue *queuep, const void *itemp, uint32_t timeout_ms )
{
	return xQueueSendToBack( ( xQueueHandle )queuep->queue_handle, itemp, MS2TICKS( timeout_ms ) ) == pdTRUE;
}

/**
 *
 * @brief   Appends an item to a queue from ISR context.
 *
 * @param[in] queuep       pointer to instance of @p struct uvos_queue
 * @param[in] itemp        pointer to item which will be appended to the queue
 * @param[in] timeout_ms   timeout for appending item to queue in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Queue_Send_FromISR( struct uvos_queue *queuep, const void *itemp, bool *wokenp )
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	portBASE_TYPE result = xQueueSendToBackFromISR( ( xQueueHandle )queuep->queue_handle, itemp, &xHigherPriorityTaskWoken );
	*wokenp = *wokenp || xHigherPriorityTaskWoken == pdTRUE;
	return result == pdTRUE;
}

/**
 *
 * @brief   Retrieves an item from the front of a queue.
 *
 * @param[in] queuep       pointer to instance of @p struct uvos_queue
 * @param[in] itemp        pointer to item which will be retrieved
 * @param[in] timeout_ms   timeout for retrieving item from queue in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Queue_Receive( struct uvos_queue *queuep, void *itemp, uint32_t timeout_ms )
{
	return xQueueReceive( ( xQueueHandle )queuep->queue_handle, itemp, MS2TICKS( timeout_ms ) ) == pdTRUE;
}

#elif defined(UVOS_INCLUDE_CHIBIOS)

#if !defined(UVOS_QUEUE_MAX_WAITERS)
#define UVOS_QUEUE_MAX_WAITERS 2
#endif /* !defined(UVOS_QUEUE_MAX_WAITERS) */

/**
 *
 * @brief   Creates a queue.
 *
 * @returns instance of @p struct uvos_queue or NULL on failure
 *
 */
struct uvos_queue *UVOS_Queue_Create( size_t queue_length, size_t item_size )
{
	struct uvos_queue *queuep = UVOS_malloc_no_dma( sizeof( struct uvos_queue ) );
	if ( queuep == NULL )
		return NULL;

	/* Create the memory pool. */
	queuep->mpb = UVOS_malloc_no_dma( item_size * ( queue_length + UVOS_QUEUE_MAX_WAITERS ) );
	if ( queuep->mpb == NULL ) {
		UVOS_free( queuep );
		return NULL;
	}
	chPoolInit( &queuep->mp, item_size, NULL );
	chPoolLoadArray( &queuep->mp, queuep->mpb, queue_length + UVOS_QUEUE_MAX_WAITERS );

	/* Create the mailbox. */
	msg_t *mb_buf = UVOS_malloc_no_dma( sizeof( msg_t ) * queue_length );
	chMBInit( &queuep->mb, mb_buf, queue_length );

	return queuep;
}

/**
 *
 * @brief   Destroys an instance of @p struct uvos_queue
 *
 * @param[in] queuep       pointer to instance of @p struct uvos_queue
 *
 */
void UVOS_Queue_Delete( struct uvos_queue *queuep )
{
	UVOS_free( queuep->mpb );
	UVOS_free( queuep );
}

/**
 *
 * @brief   Appends an item to a queue.
 *
 * @param[in] queuep       pointer to instance of @p struct uvos_queue
 * @param[in] itemp        pointer to item which will be appended to the queue
 * @param[in] timeout_ms   timeout for appending item to queue in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Queue_Send( struct uvos_queue *queuep, const void *itemp, uint32_t timeout_ms )
{
	void *buf = chPoolAlloc( &queuep->mp );
	if ( buf == NULL )
		return false;

	memcpy( buf, itemp, queuep->mp.mp_object_size );

	systime_t timeout;
	if ( timeout_ms == UVOS_QUEUE_TIMEOUT_MAX )
		timeout = TIME_INFINITE;
	else if ( timeout_ms == 0 )
		timeout = TIME_IMMEDIATE;
	else
		timeout = MS2ST( timeout_ms );

	msg_t result = chMBPost( &queuep->mb, ( msg_t )buf, timeout );

	if ( result != RDY_OK ) {
		chPoolFree( &queuep->mp, buf );
		return false;
	}

	return true;
}

/**
 *
 * @brief   Appends an item to a queue from ISR context.
 *
 * @param[in] queuep       pointer to instance of @p struct uvos_queue
 * @param[in] itemp        pointer to item which will be appended to the queue
 * @param[in] timeout_ms   timeout for appending item to queue in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Queue_Send_FromISR( struct uvos_queue *queuep, const void *itemp, bool *wokenp )
{
	chSysLockFromIsr();
	void *buf = chPoolAllocI( &queuep->mp );
	if ( buf == NULL ) {
		chSysUnlockFromIsr();
		return false;
	}

	memcpy( buf, itemp, queuep->mp.mp_object_size );

	msg_t result = chMBPostI( &queuep->mb, ( msg_t )buf );

	if ( result != RDY_OK ) {
		chPoolFreeI( &queuep->mp, buf );
		chSysUnlockFromIsr();
		return false;
	}

	chSysUnlockFromIsr();

	return true;
}

/**
 *
 * @brief   Retrieves an item from the front of a queue.
 *
 * @param[in] queuep       pointer to instance of @p struct uvos_queue
 * @param[in] itemp        pointer to item which will be retrieved
 * @param[in] timeout_ms   timeout for retrieving item from queue in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Queue_Receive( struct uvos_queue *queuep, void *itemp, uint32_t timeout_ms )
{
	msg_t buf;

	systime_t timeout;
	if ( timeout_ms == UVOS_QUEUE_TIMEOUT_MAX )
		timeout = TIME_INFINITE;
	else if ( timeout_ms == 0 )
		timeout = TIME_IMMEDIATE;
	else
		timeout = MS2ST( timeout_ms );

	msg_t result = chMBFetch( &queuep->mb, &buf, timeout );

	if ( result != RDY_OK )
		return false;

	memcpy( itemp, ( void * )buf, queuep->mp.mp_object_size );

	chPoolFree( &queuep->mp, ( void * )buf );

	return true;
}

#else

// Inspired by https://github.com/matthewhartstonge/c-queue/blob/master/queue.c

// #include "uvos_semaphore.h"

// Type by which queues are referenced
// typedef void *QueueHandle;

/* Globals */
// pthread_mutex_t queue_lock;
// typedef struct uvos_semaphore queue_lock_t;
// queue_lock_t queue_lock;

/**
 *
 * @brief   Creates a queue.
 *
 * @returns instance of @p struct uvos_queue or NULL on failure
 *
 */
struct uvos_queue *UVOS_Queue_Create( size_t queue_length, size_t item_size )
{
	struct uvos_queue *queuep = UVOS_malloc_no_dma( sizeof( struct uvos_queue ) );

	if ( queuep == NULL )
		return NULL;

	queuep->queue_length = queue_length;
	queuep->item_size = item_size;
	queuep->queue_handle = ( uintptr_t )NULL;

	// if ( ( queuep->queue_handle = ( uintptr_t )xQueueCreate( queue_length, item_size ) ) == ( uintptr_t )NULL ) {
	if ( ( queuep->queue_handle = ( uintptr_t )UVOS_malloc( queue_length * item_size ) ) == ( uintptr_t )NULL ) {
		UVOS_free( queuep );
		return NULL;
	}
	// void fifoBuf_init(t_fifo_buffer *buf, const void *buffer, const uint16_t buffer_size);
	fifoBuf_init( &queuep->queue, &queuep->queue_handle, ( queue_length * item_size ) );

	return queuep;
}

/**
 *
 * @brief   Appends an item to a queue.
 *
 * @param[in] queuep       pointer to instance of @p struct uvos_queue
 * @param[in] itemp        pointer to item which will be appended to the queue
 * @param[in] timeout_ms   timeout for appending item to queue in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Queue_Send( struct uvos_queue *queuep, const void *itemp, uint32_t timeout_ms )
{
	UNUSED( timeout_ms );
	uint16_t ret;

	ret = fifoBuf_putData( &queuep->queue, itemp, queuep->item_size );
	if ( ret != ( queuep->item_size ) ) {
		return false;
	}
	return true;
}

/**
 *
 * @brief   Retrieves an item from the front of a queue.
 *
 * @param[in] queuep       pointer to instance of @p struct uvos_queue
 * @param[in] itemp        pointer to item which will be retrieved
 * @param[in] timeout_ms   timeout for retrieving item from queue in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Queue_Receive( struct uvos_queue *queuep, void *itemp, uint32_t timeout_ms )
{
	UNUSED( timeout_ms );
	uint16_t ret;

	ret = fifoBuf_getData(  &queuep->queue, itemp, queuep->item_size );
	if ( ret != ( queuep->item_size ) ) {
		return false;
	}
	return true;
}

#endif /* defined(UVOS_INCLUDE_CHIBIOS) */
