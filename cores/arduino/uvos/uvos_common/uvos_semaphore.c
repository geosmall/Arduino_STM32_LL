// Inspired by https://github.com/TauLabs/TauLabs

#include "uvos.h"
#include "uvos_semaphore.h"

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
 * @brief   Creates a binary semaphore.
 *
 * @returns instance of @p struct uvos_semaphore or NULL on failure
 *
 */
struct uvos_semaphore *UVOS_Semaphore_Create(void)
{
	struct uvos_semaphore *sema = UVOS_malloc(sizeof(struct uvos_semaphore));

	if (sema == NULL)
		return NULL;

	/*
	 * The initial state of a binary semaphore is "given".
	 * FreeRTOS executes a "give" upon creation.
	 */
	xSemaphoreHandle temp;
	vSemaphoreCreateBinary(temp);
	sema->sema_handle = (uintptr_t)temp;

	return sema;
}

/**
 *
 * @brief   Takes binary semaphore.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 * @param[in] timeout_ms   timeout for acquiring the lock in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Take(struct uvos_semaphore *sema, uint32_t timeout_ms)
{
	UVOS_Assert(sema != NULL);

	portTickType timeout_ticks;
	if (timeout_ms == UVOS_SEMAPHORE_TIMEOUT_MAX)
		timeout_ticks = portMAX_DELAY;
	else
		timeout_ticks = MS2TICKS(timeout_ms);

	return xSemaphoreTake(sema->sema_handle, timeout_ticks) == pdTRUE;
}

/**
 *
 * @brief   Gives binary semaphore.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Give(struct uvos_semaphore *sema)
{
	UVOS_Assert(sema != NULL);

	return xSemaphoreGive(sema->sema_handle) == pdTRUE;
}

/* Workaround for simulator version of FreeRTOS. */
#if !defined(SIM_POSIX)
/**
 *
 * @brief   Takes binary semaphore from ISR context.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 * @param[out] woken       pointer to bool which will be set true if a context switch is required
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Take_FromISR(struct uvos_semaphore *sema, bool *woken)
{
	UVOS_Assert(sema != NULL);
	UVOS_Assert(woken != NULL);

	signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	bool result = xSemaphoreTakeFromISR(sema->sema_handle, &xHigherPriorityTaskWoken) == pdTRUE;

	*woken = *woken || xHigherPriorityTaskWoken == pdTRUE;

	return result;
}

/**
 *
 * @brief   Gives binary semaphore from ISR context.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 * @param[out] woken       pointer to bool which will be set true if a context switch is required
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Give_FromISR(struct uvos_semaphore *sema, bool *woken)
{
	UVOS_Assert(sema != NULL);
	UVOS_Assert(woken != NULL);

	signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	bool result = xSemaphoreGiveFromISR(sema->sema_handle, &xHigherPriorityTaskWoken) == pdTRUE;

	*woken = *woken || xHigherPriorityTaskWoken == pdTRUE;

	return result;
}
#endif /* !defined(SIM_POSIX) */

#elif defined(UVOS_INCLUDE_CHIBIOS)

/**
 *
 * @brief   Creates a binary semaphore.
 *
 * @returns instance of @p struct uvos_semaphore or NULL on failure
 *
 */
struct uvos_semaphore *UVOS_Semaphore_Create(void)
{
	struct uvos_semaphore *sema = UVOS_malloc(sizeof(struct uvos_semaphore));

	if (sema == NULL)
		return NULL;

	/*
	 * The initial state of a binary semaphore is "given".
	 */
	chBSemInit(&sema->sema, false);

	return sema;
}

/**
 *
 * @brief   Takes binary semaphore.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 * @param[in] timeout_ms   timeout for acquiring the lock in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Take(struct uvos_semaphore *sema, uint32_t timeout_ms)
{
	UVOS_Assert(sema != NULL);

	if (timeout_ms == UVOS_SEMAPHORE_TIMEOUT_MAX)
		return chBSemWait(&sema->sema) == RDY_OK;
	else if (timeout_ms == 0)
		return chBSemWaitTimeout(&sema->sema, TIME_IMMEDIATE) == RDY_OK;
	else
		return chBSemWaitTimeout(&sema->sema, MS2ST(timeout_ms)) == RDY_OK;
}

/**
 *
 * @brief   Gives binary semaphore.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Give(struct uvos_semaphore *sema)
{
	UVOS_Assert(sema != NULL);

	chBSemSignal(&sema->sema);

	return true;
}

/**
 *
 * @brief   Takes binary semaphore from ISR context.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 * @param[out] woken       pointer to bool which will be set true if a context switch is required
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Take_FromISR(struct uvos_semaphore *sema, bool *woken)
{
	/* Waiting on a semaphore within an interrupt is not supported by ChibiOS. */
	UVOS_Assert(false);
	return false;
}

/**
 *
 * @brief   Gives binary semaphore from ISR context.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 * @param[out] woken       pointer to bool which will be set true if a context switch is required
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Give_FromISR(struct uvos_semaphore *sema, bool *woken)
{
	UVOS_Assert(sema != NULL);
	UVOS_Assert(woken != NULL);

	chSysLockFromIsr();
	chBSemSignalI(&sema->sema);
	chSysUnlockFromIsr();

	return true;
}

#elif defined(UVOS_INCLUDE_IRQ)

/**
 *
 * @brief   Creates a binary semaphore.
 *
 * @returns instance of @p struct uvos_semaphore or NULL on failure
 *
 */
struct uvos_semaphore *UVOS_Semaphore_Create(void)
{
	struct uvos_semaphore *sema = UVOS_malloc_no_dma(sizeof(struct uvos_semaphore));

	if (sema == NULL)
		return NULL;

	/*
	 * The initial state of a binary semaphore is "given".
	 */
	sema->sema_count = 1;

	return sema;
}

/**
 *
 * @brief   Takes binary semaphore.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 * @param[in] timeout_ms   timeout for acquiring the lock in milliseconds
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Take(struct uvos_semaphore *sema, uint32_t timeout_ms)
{
	UVOS_Assert(sema != NULL);

	uint32_t start = UVOS_DELAY_GetRaw();

	uint32_t temp_sema_count;
	do {
		UVOS_IRQ_Disable();
		if ((temp_sema_count = sema->sema_count) != 0)
			--sema->sema_count;
		UVOS_IRQ_Enable();
	} while (temp_sema_count == 0 &&
		UVOS_DELAY_DiffuS(start) < timeout_ms * 1000);

	return temp_sema_count != 0;
}

/**
 *
 * @brief   Gives binary semaphore.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Give(struct uvos_semaphore *sema)
{
	UVOS_Assert(sema != NULL);

	bool result = true;

	UVOS_IRQ_Disable();

	if (sema->sema_count == 0)
		++sema->sema_count;
	else
		result = false;

	UVOS_IRQ_Enable();

	return result;
}

/**
 *
 * @brief   Takes binary semaphore from ISR context.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 * @param[out] woken       pointer to bool which will be set true if a context switch is required
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Take_FromISR(struct uvos_semaphore *sema, bool *woken)
{
	UVOS_Assert(sema != NULL);

	bool result = true;

	UVOS_IRQ_Disable();

	if (sema->sema_count != 0)
		--sema->sema_count;
	else
		result = false;

	UVOS_IRQ_Enable();

	return result;
}

/**
 *
 * @brief   Gives binary semaphore from ISR context.
 *
 * @param[in] sema         pointer to instance of @p struct uvos_semaphore
 * @param[out] woken       pointer to bool which will be set true if a context switch is required
 *
 * @returns true on success or false on timeout or failure
 *
 */
bool UVOS_Semaphore_Give_FromISR(struct uvos_semaphore *sema, bool *woken)
{
	UVOS_Assert(sema != NULL);

	bool result = true;

	UVOS_IRQ_Disable();

	if (sema->sema_count == 0)
		++sema->sema_count;
	else
		result = false;

	UVOS_IRQ_Enable();

	return result;
}

#endif /* defined(UVOS_INCLUDE_IRQ) */
