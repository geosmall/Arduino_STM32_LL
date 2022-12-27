#ifndef UVOS_HELPERS_H
#define UVOS_HELPERS_H

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/**
 * @brief return the number of elements contained in the array x.
 * @param[in] x the array
 * @return  number of elements in x.
 *
 */
#define NELEMENTS(x)       (sizeof(x) / sizeof((x)[0]))


/**
 * @brief Compiler barrier: Disables compiler load/store reordering across the barrier
 *
 */
#define COMPILER_BARRIER() asm volatile ("" ::: "memory")

// Memory barriers:
// Note that on single core Cortex M3 & M4, the is generally no need to use a processor memory barrier instruction such as DMB.
// See http://infocenter.arm.com/help/topic/com.arm.doc.dai0321a/DAI0321A_programming_guide_memory_barriers_for_m_profile.pdf
// However, it makes sense to use these if we want to reduce issues if we ever port to a multicore processor in the future.
// An important exception for STM32 is when setting up the DMA engine - see the above reference for details.

/**
 * @brief  Read Acquire memory barrier
 */
#define READ_MEMORY_BARRIER()  COMPILER_BARRIER()
/**
 * @brief Write Release memory barrier
 */
#define WRITE_MEMORY_BARRIER() COMPILER_BARRIER()

/**
 * @brief Full fence memory barrier
 */
#define MEMORY_BARRIER()       COMPILER_BARRIER()

// For future multicore ARM v7 or later:
// The above three macros would be replaced with: asm volatile("dmb":::"memory")


#endif // UVOS_HELPERS_H
