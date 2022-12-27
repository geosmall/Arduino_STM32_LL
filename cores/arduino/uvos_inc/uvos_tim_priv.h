#ifndef UVOS_TIM_PRIV_H
#define UVOS_TIM_PRIV_H

#include <uvos_stm32.h>

// STM32 SPL compatibility ------------------------------------>>>

#define TIM_IT_Update                      ((uint16_t)0x0001)
#define TIM_IT_CC1                         ((uint16_t)0x0002)
#define TIM_IT_CC2                         ((uint16_t)0x0004)
#define TIM_IT_CC3                         ((uint16_t)0x0008)
#define TIM_IT_CC4                         ((uint16_t)0x0010)
#define TIM_IT_COM                         ((uint16_t)0x0020)
#define TIM_IT_Trigger                     ((uint16_t)0x0040)
#define TIM_IT_Break                       ((uint16_t)0x0080)
#define IS_TIM_IT(IT) ((((IT) & (uint16_t)0xFF00) == 0x0000) && ((IT) != 0x0000))

#define UVOS_TIM_ALL_FLAGS TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_Trigger | TIM_IT_Break

#define IS_TIM_GET_IT(IT) (((IT) == TIM_IT_Update) || \
                           ((IT) == TIM_IT_CC1) || \
                           ((IT) == TIM_IT_CC2) || \
                           ((IT) == TIM_IT_CC3) || \
                           ((IT) == TIM_IT_CC4) || \
                           ((IT) == TIM_IT_COM) || \
                           ((IT) == TIM_IT_Trigger) || \
                           ((IT) == TIM_IT_Break))

// STM32 SPL compatibility ------------------------------------<<<

struct uvos_tim_clock_cfg {
  TIM_TypeDef * timer;
  const LL_TIM_InitTypeDef * time_base_init;
  struct stm32_irq irq;
};

struct uvos_tim_channel {
  TIM_TypeDef * timer;
  uint16_t     timer_chan;
  struct stm32_gpio pin;
  // uint32_t    remap;
};

struct uvos_tim_callbacks {
  void ( *overflow )( uint32_t tim_id, uint32_t context, uint8_t chan_idx, uint16_t count );
  void ( *edge )( uint32_t tim_id, uint32_t context, uint8_t chan_idx, uint16_t count );
};

extern int32_t UVOS_TIM_InitClock( const struct uvos_tim_clock_cfg * cfg );
extern int32_t UVOS_TIM_InitTimebase( uint32_t * tim_id, const TIM_TypeDef * timer, const struct uvos_tim_callbacks * callbacks, uint32_t context );
extern int32_t UVOS_TIM_InitChannels( uint32_t * tim_id, const struct uvos_tim_channel * channels, uint8_t num_channels, const struct uvos_tim_callbacks * callbacks, uint32_t context );

#endif /* UVOS_TIM_PRIV_H */