#ifndef UVOS_RCVR_PRIV_H
#define UVOS_RCVR_PRIV_H

#include <uvos.h>

extern int32_t UVOS_RCVR_Init( uint32_t * rcvr_id, const struct uvos_rcvr_driver * driver, const uint32_t lower_id );

// extern void UVOS_RCVR_IRQ_Handler(uint32_t rcvr_id);

#endif /* UVOS_RCVR_PRIV_H */
