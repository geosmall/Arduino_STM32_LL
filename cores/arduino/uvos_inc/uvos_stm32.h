#pragma once

#include "uvos.h"

#if 0 // GLS

struct stm32_irq {
  void     ( *handler )( uint32_t );
  uint32_t flags;
  NVIC_InitTypeDef init;
};

struct stm32_exti {
  EXTI_InitTypeDef init;
};

struct stm32_dma_chan {
#if defined(STM32F2XX) || defined(STM32F4XX)
  DMA_Stream_TypeDef * channel;
#else
  DMA_Channel_TypeDef * channel;
#endif
  DMA_InitTypeDef     init;
};

struct stm32_dma {
  uint32_t ahb_clk; /* ignored on STM32F2XX */
  struct stm32_irq irq;
  struct stm32_dma_chan rx;
  struct stm32_dma_chan tx;
};

struct stm32_gpio {
  GPIO_TypeDef   *  gpio;
  GPIO_InitTypeDef init;
  uint8_t pin_source;
};

#endif // GLS

struct stm32_irq {
  void ( *handler )( uint32_t );
  uint32_t flags;
  NVIC_InitTypeDef init;
};

struct stm32_exti {
  LL_EXTI_InitTypeDef init;
};

struct stm32_dma_stream {
  DMA_TypeDef * DMAx;       // DMAx Instance
  uint32_t stream;          // @group DMA_LL_EC_STREAM
  LL_DMA_InitTypeDef     init;
};

struct stm32_dma {
  uint32_t ahb_clk; /* ignored on STM32F2XX */
  struct stm32_irq irq;
  struct stm32_dma_stream rx;
  struct stm32_dma_stream tx;
};

struct stm32_gpio {
  GPIO_TypeDef   *  gpio;
  LL_GPIO_InitTypeDef init;
  // uint8_t pin_source;
};
