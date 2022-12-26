/* Project Includes */
#include "uvos.h"		/* UVOS_INCLUDE_* */

#include "uvos_heap.h"		/* External API declaration */

#include <stdio.h>		/* NULL */
#include <stdint.h>		/* uintptr_t */
#include <stdbool.h>		/* bool */

#define DEBUG_MALLOC_FAILURES 0
static volatile bool malloc_failed_flag = false;
static void malloc_failed_hook( void )
{
	malloc_failed_flag = true;
#if DEBUG_MALLOC_FAILURES
	static volatile bool wait_here = true;
	while ( wait_here );
	wait_here = true;
#endif
}

bool UVOS_heap_malloc_failed_p( void )
{
	return malloc_failed_flag;
}

struct uvos_heap {
	const uintptr_t start_addr;
	uintptr_t end_addr;
	uintptr_t free_addr;
};

static bool is_ptr_in_heap_p( const struct uvos_heap * heap, void * buf )
{
	uintptr_t buf_addr = ( uintptr_t )buf;

	return ( ( buf_addr >= heap->start_addr ) && ( buf_addr <= heap->end_addr ) );
}

static void * simple_malloc( struct uvos_heap * heap, size_t size )
{
	if ( heap == NULL )
		return NULL;

	void * buf = NULL;
	uint32_t align_pad = ( sizeof( uintptr_t ) - ( size & ( sizeof( uintptr_t ) - 1 ) ) ) % sizeof( uintptr_t );

#if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
	UVOS_Thread_Scheduler_Suspend();
#endif	/* UVOS_INCLUDE_FREERTOS || defined(UVOS_INCLUDE_CHIBIOS) */

	if ( heap->free_addr + size <= heap->end_addr ) {
		buf = ( void * )heap->free_addr;
		heap->free_addr += size + align_pad;
	}

#if defined(UVOS_INCLUDE_FREERTOS) || defined(UVOS_INCLUDE_CHIBIOS)
	UVOS_Thread_Scheduler_Resume();
#endif	/* UVOS_INCLUDE_FREERTOS || defined(UVOS_INCLUDE_CHIBIOS) */

	return buf;
}

static void simple_free( struct uvos_heap * heap, void * buf )
{
	/* This allocator doesn't support free */
}

static size_t simple_get_free_bytes( struct uvos_heap * heap )
{
	if ( heap->free_addr > heap->end_addr )
		return 0;

	return heap->end_addr - heap->free_addr;
}

static void simple_extend_heap( struct uvos_heap * heap, size_t bytes )
{
	heap->end_addr += bytes;
}

/*
 * Standard heap.  All memory in this heap is DMA-safe.
 * Note: Uses underlying FreeRTOS heap when available
 */
extern const void * _eheap;	/* defined in linker script */
extern const void * _sheap;	/* defined in linker script */

static struct uvos_heap uvos_standard_heap = {
	.start_addr = ( const uintptr_t ) & _sheap,
	.end_addr   = ( const uintptr_t ) & _eheap,
	.free_addr  = ( uintptr_t ) & _sheap,
};


void * pvPortMalloc( size_t size ) __attribute__( ( alias ( "UVOS_malloc" ), weak ) );
void * UVOS_malloc( size_t size )
{
	void * buf = simple_malloc( &uvos_standard_heap, size );

	if ( buf == NULL )
		malloc_failed_hook();

	return buf;
}

/*
 * Fast heap.  Memory in this heap is NOT DMA-safe.
 * Note: This should not be used to allocate RAM for task stacks since a task may pass
 *       automatic variables into underlying UVOS functions which *may* use DMA.
 * Note: Falls back to using the standard heap when allocations cannot be satisfied.
 */
#if defined(UVOS_INCLUDE_FASTHEAP)

extern const void * _efastheap;	/* defined in linker script */
extern const void * _sfastheap;	/* defined in linker script */
static struct uvos_heap uvos_nodma_heap = {
	.start_addr = ( const uintptr_t ) & _sfastheap,
	.end_addr   = ( const uintptr_t ) & _efastheap,
	.free_addr  = ( uintptr_t ) & _sfastheap,
};
void * UVOS_malloc_no_dma( size_t size )
{
	void * buf = simple_malloc( &uvos_nodma_heap, size );

	if ( buf == NULL )
		buf = UVOS_malloc( size );

	if ( buf == NULL )
		malloc_failed_hook();

	return buf;
}

#else	/* UVOS_INCLUDE_FASTHEAP */

/* This platform only has a standard heap.  Fall back directly to that */
void * UVOS_malloc_no_dma( size_t size )
{
	return UVOS_malloc( size );
}

#endif	/* UVOS_INCLUDE_FASTHEAP */

void vPortFree( void * buf ) __attribute__( ( alias ( "UVOS_free" ) ) );
void UVOS_free( void * buf )
{
#if defined(UVOS_INCLUDE_FASTHEAP)
	if ( is_ptr_in_heap_p( &uvos_nodma_heap, buf ) )
		return simple_free( &uvos_nodma_heap, buf );
#endif	/* UVOS_INCLUDE_FASTHEAP */

	if ( is_ptr_in_heap_p( &uvos_standard_heap, buf ) )
		return simple_free( &uvos_standard_heap, buf );
}

size_t xPortGetFreeHeapSize( void ) __attribute__( ( alias ( "UVOS_heap_get_free_size" ) ) );
size_t UVOS_heap_get_free_size( void )
{
#if defined(UVOS_INCLUDE_FREERTOS)
	UVOS_Thread_Scheduler_Suspend();
#endif	/* UVOS_INCLUDE_FREERTOS */

	size_t free_bytes = simple_get_free_bytes( &uvos_standard_heap );

#if defined(UVOS_INCLUDE_FREERTOS)
	UVOS_Thread_Scheduler_Resume();
#endif	/* UVOS_INCLUDE_FREERTOS */

	return free_bytes;
}


#if defined(UVOS_INCLUDE_FASTHEAP)

size_t UVOS_fastheap_get_free_size( void )
{
#if defined(UVOS_INCLUDE_FREERTOS)
	UVOS_Thread_Scheduler_Suspend();
#endif	/* UVOS_INCLUDE_FREERTOS */

	size_t free_bytes = simple_get_free_bytes( &uvos_nodma_heap );

#if defined(UVOS_INCLUDE_FREERTOS)
	UVOS_Thread_Scheduler_Resume();
#endif	/* UVOS_INCLUDE_FREERTOS */

	return free_bytes;
}

#else

size_t UVOS_fastheap_get_free_size( void )
{
	return 0;
}

#endif // UVOS_INCLUDE_FASTHEAP

void vPortInitialiseBlocks( void ) __attribute__( ( alias ( "UVOS_heap_initialize_blocks" ) ) );
void UVOS_heap_initialize_blocks( void )
{
	/* NOP for the simple allocator */
}

void xPortIncreaseHeapSize( size_t bytes ) __attribute__( ( alias ( "UVOS_heap_increase_size" ) ) );
void UVOS_heap_increase_size( size_t bytes )
{
#if defined(UVOS_INCLUDE_FREERTOS)
	UVOS_Thread_Scheduler_Suspend();
#endif	/* UVOS_INCLUDE_FREERTOS || defined(UVOS_INCLUDE_CHIBIOS) */

	simple_extend_heap( &uvos_standard_heap, bytes );

#if defined(UVOS_INCLUDE_FREERTOS)
	UVOS_Thread_Scheduler_Resume();
#endif	/* UVOS_INCLUDE_FREERTOS || defined(UVOS_INCLUDE_CHIBIOS) */
}

/* Provide an implementation of _sbrk for library functions.
 * Right now it returns failure always.
 */
void * _sbrk( int incr )
{
	UVOS_Assert( 0 );
	return ( void * ) - 1;
}

/**
 * @}
 * @}
 */
