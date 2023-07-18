#ifndef UVOS_QUEUE_H_
#define UVOS_QUEUE_H_

// Inspired by https://github.com/TauLabs/TauLabs

#define UVOS_QUEUE_TIMEOUT_MAX 0xffffffff

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(UVOS_INCLUDE_FREERTOS)

struct uvos_queue {
  uintptr_t queue_handle;
};

#elif defined(UVOS_INCLUDE_CHIBIOS)

#include "ch.h"

struct uvos_queue {
  Mailbox mb;
  MemoryPool mp;
  void *mpb;
};

#else

#include "fifo_buffer.h"

struct uvos_queue {
  size_t queue_length;
  size_t item_size;
  t_fifo_buffer fifo;
};

#endif /* defined(UVOS_INCLUDE_FREERTOS) */

typedef struct uvos_queue *p_uvos_queue_t;

/*
 * The following functions implement the concept of a queue usable
 * with UVOS_INCLUDE_FREERTOS or UVOS_INCLUDE_CHIBIOS.
 *
 * for details see
 * http://www.freertos.org/a00018.html
 * http://chibios.sourceforge.net/html/group__mailboxes.html
 *
 */

struct uvos_queue *UVOS_Queue_Create( size_t queue_length, size_t item_size );
void UVOS_Queue_Delete( struct uvos_queue *queuep );
bool UVOS_Queue_Send( struct uvos_queue *queuep, const void *itemp, uint32_t timeout_ms );
bool UVOS_Queue_Send_FromISR( struct uvos_queue *queuep, const void *itemp, bool *wokenp );
bool UVOS_Queue_Receive( struct uvos_queue *queuep, void *itemp, uint32_t timeout_ms );

#endif /* UVOS_QUEUE_H_ */

/**
  * @}
  * @}
  */
