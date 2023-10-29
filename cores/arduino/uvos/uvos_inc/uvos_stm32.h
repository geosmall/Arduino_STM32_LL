#pragma once

#include "uvos.h"

struct stm32_irq {
  void ( *handler )( uint32_t );
  uint32_t flags;
  NVIC_InitTypeDef init;
};

struct stm32_exti {
  LL_EXTI_InitTypeDef init;
};

struct stm32_dma_stream {
  DMA_TypeDef *DMAx;        // DMAx Instance
  uint32_t stream;          // @group DMA_LL_EC_STREAM
  LL_DMA_InitTypeDef     init;
};

struct stm32_dma {
  // uint32_t ahb_clk; /* ignored on STM32F2XX */
  struct stm32_irq irq;
  struct stm32_dma_stream rx;
  struct stm32_dma_stream tx;
};

struct stm32_dma_tx_only {
  // uint32_t ahb_clk; /* ignored on STM32F2XX */
  struct stm32_irq irq;
  struct stm32_dma_stream tx;
};

struct stm32_gpio {
  GPIO_TypeDef     *gpio;
  LL_GPIO_InitTypeDef init;
  // uint8_t pin_source;
};
