#ifndef UVOS_EXTI_H
#define UVOS_EXTI_H

/* Public Functions */

#include <uvos_stm32.h>

typedef bool ( *uvos_exti_vector_t )( void );

struct uvos_exti_cfg {
  uvos_exti_vector_t vector;
  uint32_t line; /* use EXTI_LineN macros */
  struct stm32_gpio  pin;
  struct stm32_irq   irq;
  struct stm32_exti  exti;
};

/* must be added to any uvos_exti_cfg definition for it to be valid */
// #define __exti_config __attribute__((section("_exti")))

#define __exti_config

extern int32_t UVOS_EXTI_Init( const struct uvos_exti_cfg * cfg );
extern int32_t UVOS_EXTI_DeInit( const struct uvos_exti_cfg * cfg );

#endif // UVOS_EXTI_H
