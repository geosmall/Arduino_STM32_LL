#ifndef UVOS_RTC_PRIV_H
#define UVOS_RTC_PRIV_H

#include <uvos.h>
#include <uvos_stm32.h>

struct uvos_rtc_cfg {
    uint32_t hse_clkdiv;
    uint32_t WUT_clkdiv;
    uint32_t WUT_reload;
    struct stm32_irq irq;
};

extern void UVOS_RTC_Init(const struct uvos_rtc_cfg *cfg);

extern void UVOS_RTC_irq_handler(void);

#endif /* UVOS_RTC_PRIV_H */
