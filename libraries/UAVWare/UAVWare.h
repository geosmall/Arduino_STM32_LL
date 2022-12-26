#ifndef UAVWARE_H
#define UAVWARE_H

/* UVOS Includes */
#include <uvos.h>
#include "uw_actuator.h"
#include "uw_fs.h"
#include "uw_led.h"
#include "uw_mpu.h"
#include "uw_receiver.h"
#include "uw_time.h"


#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

extern void setup( void );
extern void loop( void );
extern int UAVWare_init( void );

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // UAVWARE_H