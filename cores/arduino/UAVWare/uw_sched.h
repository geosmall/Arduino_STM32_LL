#ifndef UW_SCHED_H
#define UW_SCHED_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* Global Types */

// User-defined type to store required data for each task
struct UW_sched_task {
	void ( *pTask ) ( void );  	/* Pointer to the task (must be a 'void (void)' function) */
	uint32_t Delay;  					 	/* Delay (ticks) until the task will (next) be run */
	uint32_t Period;  					/* Interval (ticks) between subsequent runs */
};


/* Public Functions */
extern void UW_sched_init( void );
extern void UW_sched_start( void );
extern void UW_sched_stop( void );
extern int32_t UW_sched_dispatch_tasks( void );
extern int32_t UW_sched_add_task( void ( *pTask )(), const uint32_t DELAY, const uint32_t PERIOD );

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* UW_SCHED_H */