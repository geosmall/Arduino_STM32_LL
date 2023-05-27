#ifndef UVOS_SCHED_H
#define UVOS_SCHED_H

// ------ Public function prototypes -----------------------------------------

extern void UVOS_SCHED_RegisterCallbacks( const struct uvos_tim_callbacks *callbacks );
extern void UVOS_SCHED_Start( void );
extern void UVOS_SCHED_Stop( void );

// ------ Public constants -------------------------------------------------------------

#endif /* UVOS_SCHED_H */