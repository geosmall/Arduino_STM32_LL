#ifndef UVOS_SERVO_PRIV_H
#define UVOS_SERVO_PRIV_H

#include <uvos.h>
#include <uvos_stm32.h>
#include <uvos_tim_priv.h>

struct uvos_servo_cfg {
  LL_TIM_InitTypeDef tim_base_init;
  LL_TIM_OC_InitTypeDef tim_oc_init;
  LL_GPIO_InitTypeDef gpio_init;
  // uint32_t remap;
  const struct uvos_tim_channel * channels;
  uint8_t  num_channels;
};

extern int32_t UVOS_Servo_Init( const struct uvos_servo_cfg * cfg );

#endif /* UVOS_SERVO_PRIV_H */
