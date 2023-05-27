#include <uvos.h>
#include "uw_sched.h"

static void UW_sched_tick_handler( uint32_t tim_id, uint32_t context, uint8_t chan_idx, uint16_t count );

/* The array of tasks
 * Check array size in scheduler header file */
static struct UW_sched_task sched_tasks_g[ SCH_MAX_TASKS ];

// Default value for pTask (no task at this location)
#define SCH_NULL_PTR ( ( void (*) ( void ) ) 0 )

/* Tick count and error flag, accessed from both interrupt and userland contexts */
static volatile uint32_t sched_tick_count_g = 0;
static volatile bool sched_overrun_flag_g = false;

/* Callback struct, called from UVOS_TIM timer innterupt */
static const struct uvos_tim_callbacks uvos_sched_tim_callbacks = {
	.overflow = UW_sched_tick_handler,
	.edge     = NULL,
};

/*----------------------------------------------------------------------------*-

  UW_sched_tick_handler()

  This is the scheduler ISR callback.  It is called at a rate
  determined by the timer settings in the UVOS_TIME_init() function
  in uvos_time file.

  PARAMETERS:
     None.

  LONG-TERM DATA:
     sched_tick_count_g (W)

  MCU HARDWARE:
     TIMEBASE_TIM timer.

  PRE-CONDITION CHECKS:
     None.

  POST-CONDITION CHECKS:
     None.

  ERROR DETECTION / ERROR HANDLING:
     Checks sched_tick_count_g value => 'Fail Safe'

  RETURN VALUE:
     None.

-*----------------------------------------------------------------------------*/
static void UW_sched_tick_handler( uint32_t tim_id, uint32_t context, uint8_t chan_idx, uint16_t count )
{

	// Increment tick count
	sched_tick_count_g++;

// #define UVOS_COM_DEBUG_SCHED_TICK
#ifdef UVOS_COM_DEBUG_SCHED_TICK
	UVOS_LED_Toggle( UVOS_LED_HEARTBEAT );
	UVOS_COM_SendChar( UVOS_COM_DEBUG, sched_tick_count_g );
#endif // UVOS_COM_DEBUG_SCHED_TICK

	// check against limit
	if ( sched_tick_count_g > SCH_TICK_COUNT_LIMIT ) {
		// One or more tasks has taken too long to complete
		sched_overrun_flag_g = true;
	}
}

void UW_sched_init( void )
{
	/* Initialaize all task entries to null */
	for ( uint32_t Task_id = 0; Task_id < SCH_MAX_TASKS; Task_id++ ) {
		sched_tasks_g[Task_id].pTask = SCH_NULL_PTR; // Set pTask to "null pointer"
	}

	UVOS_SCHED_RegisterCallbacks( &uvos_sched_tim_callbacks );
}

void UW_sched_start( void )
{
	sched_tick_count_g = 0;
	sched_overrun_flag_g = false;

	UVOS_SCHED_Start();
}

void UW_sched_stop( void )
{
	UVOS_SCHED_Stop();
}

/*----------------------------------------------------------------------------*-

  UW_sched_dispatch_tasks()

  This is the 'dispatcher' function.  When a task (function)
  is due to run, SCH_Dispatch_Tasks() will run it.
  This function must be called (repeatedly) from the main loop.

  Triggers move to "idle" mode when all tasks have been released.

  PARAMETERS:
     None.

  LONG-TERM DATA:
     sched_tasks_g (W)
     sched_tick_count_g (W)

  MCU HARDWARE:
     Triggers move to idle mode.

  PRE-CONDITION CHECKS:
     None.

  POST-CONDITION CHECKS:
     None.

  ERROR DETECTION / ERROR HANDLING:
     None.

  RETURN VALUE:
     sched_overrun_flag_g.

-*----------------------------------------------------------------------------*/
int32_t UW_sched_dispatch_tasks( void )
{
  __disable_irq();
  uint32_t update_required = ( sched_tick_count_g > 0 ); // Check tick count
  __enable_irq();

  while ( update_required ) {

#ifdef UVOS_COM_DEBUG_SCHED_DURATION
    UVOS_LED_On( UVOS_LED_HEARTBEAT );
#endif // UVOS_COM_DEBUG_SCHED_DURATION

    // Go through the task array
    for ( uint32_t Task_id = 0; Task_id < SCH_MAX_TASKS; Task_id++ ) {
      // Check if there is a task at this location
      if ( sched_tasks_g[Task_id].pTask != SCH_NULL_PTR ) {
        if ( --sched_tasks_g[Task_id].Delay == 0 ) {
          ( *sched_tasks_g[Task_id].pTask )(); // Run the task

          // All tasks are periodic: schedule task to run again
          sched_tasks_g[Task_id].Delay = sched_tasks_g[Task_id].Period;
        }
      }
    }

    __disable_irq();
    sched_tick_count_g--;                       // Decrement the count
    update_required = ( sched_tick_count_g > 0 ); // Check again
    __enable_irq();

#ifdef UVOS_COM_DEBUG_SCHED_DURATION
    UVOS_LED_Off( UVOS_LED_HEARTBEAT );
#endif // UVOS_COM_DEBUG_SCHED_DURATION

  }

  // The scheduler enters idle mode at this point
  // __WFI();

  return sched_overrun_flag_g;
}

/*----------------------------------------------------------------------------*-

  UW_sched_add_task()

  Adds a task (function) to the schedule.
  The task will be released periodically by the scheduler.

  PARAMETERS:
     pTask  : The name of the task (function) to be scheduled.
              NOTE: All scheduled functions must be 'void, void' -
              that is, they must take no parameters, and have
              a void return type (in this design).

     DELAY  : The interval (ticks) before the task is first executed.

     PERIOD : Task period (in ticks).  Must be > 0.

  LONG-TERM DATA:
     sched_tasks_g (W)

  MCU HARDWARE:
     None.

  PRE-CONDITION CHECKS:
     1. There is space in the task array.
     2. The task is periodic ('one-shot' tasks are not supported.

  POST-CONDITION CHECKS:
     None.

  ERROR DETECTION / ERROR HANDLING:
     PROCESSOR_Perform_Safe_Shutdown() is called:
     - if the task cannot be added to the schedule (array too small)
     - if an attempt is made to schedule a "one shot" task

  RETURN VALUE:
     Success or failure.

-*----------------------------------------------------------------------------*/
int32_t UW_sched_add_task( void ( *pTask )(), const uint32_t DELAY, const uint32_t PERIOD )
{
  uint32_t Task_id = 0;

  // First find a gap in the array (if there is one)
  while ( ( sched_tasks_g[Task_id].pTask != SCH_NULL_PTR )
          && ( Task_id < SCH_MAX_TASKS ) ) {
    Task_id++;
  }

  // Have we reached the end of the list?
  if ( Task_id == SCH_MAX_TASKS ) {
    // Task list is full - fatal error
    return EXIT_FAILURE;
  }

  // Check for "one shot" tasks
  if ( PERIOD == 0 ) {
    // We do not allow "one shot" tasks (all tasks must be periodic)
    return EXIT_FAILURE;
  }

  // If we're here, there is a space in the task array
  // and the task to be added is periodic
  sched_tasks_g[Task_id].pTask  = pTask;

  sched_tasks_g[Task_id].Delay  = DELAY + 1;
  sched_tasks_g[Task_id].Period = PERIOD;

  return EXIT_SUCCESS;
}