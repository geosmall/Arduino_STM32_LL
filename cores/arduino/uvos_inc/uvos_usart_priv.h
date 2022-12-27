#ifndef UVOS_USART_PRIV_H
#define UVOS_USART_PRIV_H

#include <uvos.h>
#include <uvos_stm32.h>
#include "fifo_buffer.h"
#include "uvos_usart.h"

extern const struct uvos_com_driver uvos_usart_com_driver;

struct uvos_usart_cfg {
    USART_TypeDef     *regs;
    // uint32_t remap; /* GPIO_Remap_* */
    LL_USART_InitTypeDef init;
    struct stm32_gpio rx;
    struct stm32_gpio tx;
    // struct stm32_gpio dtr;
    struct stm32_irq  irq;
};

extern int32_t UVOS_USART_Init(uint32_t *usart_id, const struct uvos_usart_cfg *cfg);
extern const struct uvos_usart_cfg *UVOS_USART_GetConfig(uint32_t usart_id);

#endif /* UVOS_USART_PRIV_H */
