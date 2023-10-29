#ifndef UVOS_MUTEX_H
#define UVOS_MUTEX_H

#define UVOS_MUTEX_TIMEOUT_MAX 0xffffffff

#include <stdint.h>
#include <stdbool.h>

#if defined(UVOS_INCLUDE_FREERTOS)

struct uvos_mutex {
  uintptr_t mtx_handle;
};

struct uvos_recursive_mutex {
  uintptr_t mtx_handle;
};

#elif defined(UVOS_INCLUDE_CHIBIOS)

struct uvos_mutex {
  Mutex mtx;
};

struct uvos_recursive_mutex {
  Mutex mtx;
  uint32_t count;
};

#elif defined(UVOS_INCLUDE_IRQ)

struct uvos_mutex {
  bool mtx_is_locked;
};


#endif /* defined(UVOS_INCLUDE_IRQ) */

/*
 * The following functions implement the concept of a non-recursive mutex usable
 * with UVOS_INCLUDE_FREERTOS or UVOS_INCLUDE_CHIBIOS.
 *
 * Note that this is not the same as:
 * - binary semaphore
 * - semaphore
 * - recursive mutex
 *
 * see FreeRTOS documentation for details: http://www.freertos.org/a00113.html
 * see ChibiOS documentation for details: http://chibios.sourceforge.net/html/group__synchronization.html
 */

struct uvos_mutex *UVOS_Mutex_Create( void );
bool UVOS_Mutex_Lock( struct uvos_mutex *mtx, uint32_t timeout_ms );
bool UVOS_Mutex_Unlock( struct uvos_mutex *mtx );

/*
 * The following functions implement the concept of a recursive mutex usable
 * with UVOS_INCLUDE_FREERTOS or UVOS_INCLUDE_CHIBIOS.
 *
 * Note that this is not the same as:
 * - binary semaphore
 * - semaphore
 * - non-recursive mutex
 *
 * Note that this implementation doesn't prevent priority inversion.
 *
 * see FreeRTOS documentation for details: http://www.freertos.org/a00113.html
 * see ChibiOS documentation for details: http://chibios.sourceforge.net/html/group__synchronization.html
 */

struct uvos_recursive_mutex *UVOS_Recursive_Mutex_Create( void );
bool UVOS_Recursive_Mutex_Lock( struct uvos_recursive_mutex *mtx, uint32_t timeout_ms );
bool UVOS_Recursive_Mutex_Unlock( struct uvos_recursive_mutex *mtx );

#endif /* UVOS_MUTEX_H */

/**
  * @}
  * @}
  */
