#ifndef UVOS_SERVO_CONFIG_H_
#define UVOS_SERVO_CONFIG_H_

/**
 * Generic servo pin configuration structure for an STM32F4xx
 */
#if 0 // GLS

#define TIM_SERVO_CHANNEL_CONFIG(_timer, _channel, _gpio, _pin) \
    {                                                     \
        .timer = _timer,                                  \
        .timer_chan = TIM_Channel_##_channel,             \
        .pin   = {                                        \
            .gpio = GPIO##_gpio,                          \
            .init = {                                     \
                .GPIO_Pin   = GPIO_Pin_##_pin,            \
                .GPIO_Speed = GPIO_Speed_2MHz,            \
                .GPIO_Mode  = GPIO_Mode_AF,               \
                .GPIO_OType = GPIO_OType_PP,              \
                .GPIO_PuPd  = GPIO_PuPd_UP                \
            },                                            \
            .pin_source     = GPIO_PinSource##_pin,       \
        },                                                \
        .remap = GPIO_AF_##_timer,                        \
    }

#endif // GLS

#define TIM_SERVO_CHANNEL_CONFIG(_timer, _channel, _gpio, _pin, _alt_function) \
  {                                                   \
    .timer = _timer,                                  \
    .timer_chan = LL_TIM_CHANNEL_CH##_channel,        \
    .pin   = {                                        \
      .gpio = _gpio,                                  \
      .init = {                                       \
        .Pin   = LL_GPIO_PIN_##_pin,                  \
        .Mode = LL_GPIO_MODE_ALTERNATE,               \
        .Speed = LL_GPIO_SPEED_FREQ_LOW,              \
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,        \
        .Pull = LL_GPIO_PULL_UP,                      \
        .Alternate = LL_GPIO_AF_##_alt_function,      \
      },                                              \
    },                                                \
  },

#endif /* UVOS_SERVO_CONFIG_H_ */
