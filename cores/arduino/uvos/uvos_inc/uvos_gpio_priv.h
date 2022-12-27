#pragma once

#include <uvos.h>
#include <uvos_stm32.h>

struct uvos_gpio {
  struct stm32_gpio pin;
  bool active_low;
};

struct uvos_gpio_cfg {
  const struct uvos_gpio * gpios;
  uint8_t num_gpios;
};

extern int32_t UVOS_GPIO_Init( uint32_t * gpios_dev_id, const struct uvos_gpio_cfg * cfg );

