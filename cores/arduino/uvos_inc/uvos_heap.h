#ifndef UVOS_HEAP_H
#define UVOS_HEAP_H

#include <stdlib.h>		/* size_t */
#include <stdbool.h>		/* bool */

extern bool UVOS_heap_malloc_failed_p(void);

extern void * UVOS_malloc_no_dma(size_t size);
extern void * UVOS_malloc(size_t size);

extern void UVOS_free(void * buf);

extern size_t UVOS_heap_get_free_size(void);
extern size_t UVOS_fastheap_get_free_size(void);
extern void UVOS_heap_initialize_blocks(void);
extern void UVOS_heap_increase_size(size_t bytes);

#endif	/* UVOS_HEAP_H */
