#ifndef UVOS_SEMAPHORE_H_
#define UVOS_SEMAPHORE_H_

// Inspired by https://github.com/TauLabs/TauLabs

#define UVOS_SEMAPHORE_TIMEOUT_MAX 0xffffffff

#include <stdint.h>
#include <stdbool.h>

struct uvos_semaphore
{
#if defined(UVOS_INCLUDE_FREERTOS)
	uintptr_t sema_handle;
#elif defined(UVOS_INCLUDE_CHIBIOS)
	BinarySemaphore sema;
#elif defined(UVOS_INCLUDE_IRQ)
	uint32_t sema_count;
#endif /* defined(UVOS_INCLUDE_IRQ) */
};

/*
 * The following functions implement the concept of a binary semaphore usable
 * with UVOS_INCLUDE_FREERTOS, UVOS_INCLUDE_CHIBIOS or UVOS_INCLUDE_IRQ.
 *
 * Note that this is not the same as:
 * - counting semaphore
 * - mutex
 * - recursive mutex
 *
 * see FreeRTOS documentation for details: http://www.freertos.org/a00113.html
 * see ChibiOS documentation for details: http://chibios.sourceforge.net/html/group__synchronization.html
 */

struct uvos_semaphore *UVOS_Semaphore_Create(void);
bool UVOS_Semaphore_Take(struct uvos_semaphore *sema, uint32_t timeout_ms);
bool UVOS_Semaphore_Give(struct uvos_semaphore *sema);

bool UVOS_Semaphore_Take_FromISR(struct uvos_semaphore *sema, bool *woken);
bool UVOS_Semaphore_Give_FromISR(struct uvos_semaphore *sema, bool *woken);

#endif /* UVOS_SEMAPHORE_H_ */

/**
  * @}
  * @}
  */
