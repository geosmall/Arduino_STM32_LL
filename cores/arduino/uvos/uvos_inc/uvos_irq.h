#ifndef UVOS_IRQ_H
#define UVOS_IRQ_H

// Inspired by https://github.com/TauLabs/TauLabs

#include <stdint.h>
#include <stdbool.h>

/* Public Functions */
extern int32_t UVOS_IRQ_Disable(void);
extern int32_t UVOS_IRQ_Enable(void);
extern bool UVOS_IRQ_InISR(void);

#endif /* UVOS_IRQ_H */