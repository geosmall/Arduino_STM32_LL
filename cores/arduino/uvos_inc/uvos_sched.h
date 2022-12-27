#ifndef UVOS_SCHED_H
#define UVOS_SCHED_H

// ------ Public data type declarations ----------------------------

// User-defined type to store required data for each task
struct uvos_sched_task {
	void ( *pTask ) ( void );  	/* Pointer to the task (must be a 'void (void)' function) */
	uint32_t Delay;  					 	/* Delay (ticks) until the task will (next) be run */
	uint32_t Period;  					/* Interval (ticks) between subsequent runs */
};

// ------ Public function prototypes -----------------------------------------

extern void UVOS_SCHED_init( const TIM_TypeDef * timer );
extern void UVOS_SCHED_start( void );
// extern void UVOS_SCHED_stop( void );
extern int32_t UVOS_SCHED_dispatch_tasks( void );
extern void UVOS_SCHED_tick_handler( uint32_t tim_id, uint32_t context, uint8_t chan_idx, uint16_t count );
extern int32_t UVOS_SCHED_add_task( void ( * pTask )(), const uint32_t DELAY, const uint32_t PERIOD );

// ------ Public constants -------------------------------------------------------------

// Default value for pTask (no task at this location)
#define SCH_NULL_PTR ( ( void (*) ( void ) ) 0 )

#endif /* UVOS_SCHED_H */