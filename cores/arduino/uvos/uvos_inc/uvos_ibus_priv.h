#ifndef UVOS_IBUS_PRIV_H
#define UVOS_IBUS_PRIV_H

#include <uvos.h>
// #include <uvos_usart_priv.h>

/* IBUS receiver instance configuration */
extern const struct uvos_rcvr_driver uvos_ibus_rcvr_driver;

extern int32_t UVOS_IBUS_Init(uint32_t *ibus_id, const struct uvos_com_driver *driver, uint32_t lower_id);

#endif // UVOS_IBUS_PRIV_H
