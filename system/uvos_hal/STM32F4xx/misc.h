
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MISC_H
#define __MISC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup MISC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  NVIC Init Structure definition
  */

typedef struct {
  uint8_t NVIC_IRQChannel;                    /*!< Specifies the IRQ channel to be enabled or disabled.
                                                   This parameter can be an enumerator of @ref IRQn_Type
                                                   enumeration (For the complete STM32 Devices IRQ Channels
                                                   list, please refer to stm32f4xx.h file) */

  uint8_t NVIC_IRQChannelPreemptionPriority;  /*!< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref MISC_NVIC_Priority_Table
                                                   A lower priority value indicates a higher priority */

  uint8_t NVIC_IRQChannelSubPriority;         /*!< Specifies the subpriority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref MISC_NVIC_Priority_Table
                                                   A lower priority value indicates a higher priority */

  FunctionalState NVIC_IRQChannelCmd;         /*!< Specifies whether the IRQ channel defined in NVIC_IRQChannel
                                                   will be enabled or disabled.
                                                   This parameter can be set either to ENABLE or DISABLE */
} NVIC_InitTypeDef;

/* Exported constants --------------------------------------------------------*/

/** @defgroup MISC_Exported_Constants
  * @{
  */

/** @defgroup MISC_Vector_Table_Base
  * @{
  */

#define NVIC_VectTab_RAM             ((uint32_t)0x20000000)
#define NVIC_VectTab_FLASH           ((uint32_t)0x08000000)
#define IS_NVIC_VECTTAB(VECTTAB) (((VECTTAB) == NVIC_VectTab_RAM) || \
                                  ((VECTTAB) == NVIC_VectTab_FLASH))
/**
  * @}
  */

/** @defgroup MISC_System_Low_Power
  * @{
  */

#define NVIC_LP_SEVONPEND            ((uint8_t)0x10)
#define NVIC_LP_SLEEPDEEP            ((uint8_t)0x04)
#define NVIC_LP_SLEEPONEXIT          ((uint8_t)0x02)
#define IS_NVIC_LP(LP) (((LP) == NVIC_LP_SEVONPEND) || \
                        ((LP) == NVIC_LP_SLEEPDEEP) || \
                        ((LP) == NVIC_LP_SLEEPONEXIT))
/**
  * @}
  */

/** @defgroup MISC_Preemption_Priority_Group
  * @{
  */

#define NVIC_PriorityGroup_0         ((uint32_t)0x700) /*!< 0 bits for pre-emption priority
                                                            4 bits for subpriority */
#define NVIC_PriorityGroup_1         ((uint32_t)0x600) /*!< 1 bits for pre-emption priority
                                                            3 bits for subpriority */
#define NVIC_PriorityGroup_2         ((uint32_t)0x500) /*!< 2 bits for pre-emption priority
                                                            2 bits for subpriority */
#define NVIC_PriorityGroup_3         ((uint32_t)0x400) /*!< 3 bits for pre-emption priority
                                                            1 bits for subpriority */
#define NVIC_PriorityGroup_4         ((uint32_t)0x300) /*!< 4 bits for pre-emption priority
                                                            0 bits for subpriority */

#define IS_NVIC_PRIORITY_GROUP(GROUP) (((GROUP) == NVIC_PriorityGroup_0) || \
                                       ((GROUP) == NVIC_PriorityGroup_1) || \
                                       ((GROUP) == NVIC_PriorityGroup_2) || \
                                       ((GROUP) == NVIC_PriorityGroup_3) || \
                                       ((GROUP) == NVIC_PriorityGroup_4))

#define IS_NVIC_PREEMPTION_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)

#define IS_NVIC_SUB_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)

#define IS_NVIC_OFFSET(OFFSET)  ((OFFSET) < 0x000FFFFF)

/**
  * @}
  */

/** @defgroup MISC_SysTick_clock_source
  * @{
  */

#define SysTick_CLKSource_HCLK_Div8    ((uint32_t)0xFFFFFFFB)
#define SysTick_CLKSource_HCLK         ((uint32_t)0x00000004)
#define IS_SYSTICK_CLK_SOURCE(SOURCE) (((SOURCE) == SysTick_CLKSource_HCLK) || \
                                       ((SOURCE) == SysTick_CLKSource_HCLK_Div8))
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/* Numerical values from DMA_LISR & DMA_HISR (also match DMA interrupt clear register bit definitions) */
// enum UVOS_DMA_IT {
//   UVOS_DMA_IT_INVALID = 0U,
//   UVOS_DMA_IT_TC      = 0x00000020,
//   UVOS_DMA_IT_HT      = 0x00000010,
//   UVOS_DMA_IT_TE      = 0x00000008,
//   UVOS_DMA_IT_DME     = 0x00000004,
//   UVOS_DMA_IT_FE      = 0x00000001,
// };


/** @defgroup DMA interrupt flag clear register bit definitions (DMA_LIFCR & DMA_HIFCR)
  * @brief    DMA flag definitions
  * @{
  */
#define UVOS_DMA_IFLG_TCIF    ((uint32_t)0x00000020) // DMA_LISR_TCIF0, Pos = 5U
#define UVOS_DMA_IFLG_HTIF    ((uint32_t)0x00000010) // DMA_LISR_HTIF0, Pos = 4U = 
#define UVOS_DMA_IFLG_TEIF    ((uint32_t)0x00000008) // DMA_LISR_TEIF0, Pos = 3U
#define UVOS_DMA_IFLG_DMEIF   ((uint32_t)0x00000004) // DMA_LISR_DMEIF0, Pos = 2U

#define UVOS_DMA_IFLG_FEIF    ((uint32_t)0x00000001) // DMA_LISR_FEIF0, Pos 0U

#define UVOS_DMA_IT_TC        UVOS_DMA_IFLG_TCIF
#define UVOS_DMA_IT_HT        UVOS_DMA_IFLG_HTIF
#define UVOS_DMA_IT_TE        UVOS_DMA_IFLG_TEIF
#define UVOS_DMA_IT_DME       UVOS_DMA_IFLG_DMEIF
#define UVOS_DMA_IT_FE        UVOS_DMA_IFLG_FEIF

#define UVOS_DMA_ALL_IFLG_BITS ((UVOS_DMA_IFLG_TCIF)  | \
                                (UVOS_DMA_IFLG_HTIF)  | \
                                (UVOS_DMA_IFLG_TEIF)  | \
                                (UVOS_DMA_IFLG_DMEIF) | \
                                (UVOS_DMA_IFLG_FEIF))
#define UVOS_DMA_IS_ANY_OF_IFLG_BITS(BITS) ((((BITS) & ~(UVOS_DMA_ALL_IFLG_BITS)) == 0x00) && ((BITS) != 0x00))

#define UVOS_DMA_IS_SINGLE_IFLAG_BIT(IFLAG) (((IFLAG) == UVOS_DMA_IFLG_TCIF) || \
                                             ((IFLAG) == UVOS_DMA_IFLG_HTIF) || \
                                             ((IFLAG) == UVOS_DMA_IFLG_TEIF) || \
                                             ((IFLAG) == UVOS_DMA_IFLG_DMEIF) || \
                                             ((IFLAG) == UVOS_DMA_IFLG_FEIF))


/** @defgroup DMA Stream interrupt_enable bit definitions (DMA_SxCR & DMA_SxFCR)
  * @brief    DMA interrupts definition
  * @{
  */
#define UVOS_DMA_ITEN_TC      ((uint32_t)DMA_SxCR_TCIE)  // 4U = 0x00000010
#define UVOS_DMA_ITEN_HT      ((uint32_t)DMA_SxCR_HTIE)  // 3U = 0x00000008
#define UVOS_DMA_ITEN_TE      ((uint32_t)DMA_SxCR_TEIE)  // 2U = 0x00000004
#define UVOS_DMA_ITEN_DME     ((uint32_t)DMA_SxCR_DMEIE) // 1U = 0x00000002

#define UVOS_DMA_ITEN_FE      ((uint32_t)DMA_SxFCR_FEIE) // 7U = 0x00000080

#define UVOS_DMA_ALL_ITEN_BITS ((UVOS_DMA_ITEN_TC)  | \
                                (UVOS_DMA_ITEN_HT)  | \
                                (UVOS_DMA_ITEN_TE)  | \
                                (UVOS_DMA_ITEN_DME) | \
                                (UVOS_DMA_ITEN_FE))
#define UVOS_DMA_IS_ANY_OF_ITEN_BITS(IT) ((((IT) & ~(UVOS_DMA_ALL_ITEN_BITS)) == 0x00) && ((IT) != 0x00))

#define UVOS_DMA_TRANSFER_ITEN_BITS ((UVOS_DMA_ITEN_TC) | \
                                     (UVOS_DMA_ITEN_HT) | \
                                     (UVOS_DMA_ITEN_TE) | \
                                     (UVOS_DMA_ITEN_DME))
#define UVOS_DMA_IS_TRANSFER_ITEN_BITS(IT) ((((IT) & ~(UVOS_DMA_TRANSFER_ITEN_BITS)) == 0x00) && ((IT) != 0x00))


#define UVOS_DMA_IS_INSTANCE(INSTANCE) (((INSTANCE) == DMA1) || \
                                        ((INSTANCE) == DMA2))

#define UVOS_DMA_IS_STREAM(STREAM) (((STREAM) == LL_DMA_STREAM_0) || \
                                    ((STREAM) == LL_DMA_STREAM_1) || \
                                    ((STREAM) == LL_DMA_STREAM_2) || \
                                    ((STREAM) == LL_DMA_STREAM_3) || \
                                    ((STREAM) == LL_DMA_STREAM_4) || \
                                    ((STREAM) == LL_DMA_STREAM_5) || \
                                    ((STREAM) == LL_DMA_STREAM_6) || \
                                    ((STREAM) == LL_DMA_STREAM_7))


/* Exported functions --------------------------------------------------------*/

void NVIC_PriorityGroupConfig( uint32_t NVIC_PriorityGroup );
void NVIC_Init( NVIC_InitTypeDef *NVIC_InitStruct );
void NVIC_SetVectorTable( uint32_t NVIC_VectTab, uint32_t Offset );
void NVIC_SystemLPConfig( uint8_t LowPowerMode, FunctionalState NewState );
void SysTick_CLKSourceConfig( uint32_t SysTick_CLKSource );
void DMA_ClearFlags( DMA_TypeDef *DMAx, uint32_t Stream, uint32_t DMA_ints );
void DMA_ClearAllFlags( DMA_TypeDef *DMAx, uint32_t Stream );
void DMA_ITConfig( DMA_TypeDef *DMAx, uint32_t Stream, uint32_t DMA_ints, FunctionalState NewState );
ITStatus DMA_GetITStatus( DMA_TypeDef *DMAx, uint32_t Stream, uint32_t DMA_int );

#ifdef __cplusplus
}
#endif

#endif /* __MISC_H */

/**
  * @}
  */

/**
  * @}
  */

