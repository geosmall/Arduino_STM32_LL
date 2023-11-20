/**
  ******************************************************************************
  * @file    misc.c
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   This file provides all the miscellaneous firmware functions (add-on
  *          to CMSIS functions).
  *
  *  @verbatim
  *
  *          ===================================================================
  *                        How to configure Interrupts using driver
  *          ===================================================================
  *
  *            This section provide functions allowing to configure the NVIC interrupts (IRQ).
  *            The Cortex-M4 exceptions are managed by CMSIS functions.
  *
  *            1. Configure the NVIC Priority Grouping using NVIC_PriorityGroupConfig()
  *                function according to the following table.

  *  The table below gives the allowed values of the pre-emption priority and subpriority according
  *  to the Priority Grouping configuration performed by NVIC_PriorityGroupConfig function
  *    ==========================================================================================================================
  *      NVIC_PriorityGroup   | NVIC_IRQChannelPreemptionPriority | NVIC_IRQChannelSubPriority  |       Description
  *    ==========================================================================================================================
  *     NVIC_PriorityGroup_0  |                0                  |            0-15             | 0 bits for pre-emption priority
  *                           |                                   |                             | 4 bits for subpriority
  *    --------------------------------------------------------------------------------------------------------------------------
  *     NVIC_PriorityGroup_1  |                0-1                |            0-7              | 1 bits for pre-emption priority
  *                           |                                   |                             | 3 bits for subpriority
  *    --------------------------------------------------------------------------------------------------------------------------
  *     NVIC_PriorityGroup_2  |                0-3                |            0-3              | 2 bits for pre-emption priority
  *                           |                                   |                             | 2 bits for subpriority
  *    --------------------------------------------------------------------------------------------------------------------------
  *     NVIC_PriorityGroup_3  |                0-7                |            0-1              | 3 bits for pre-emption priority
  *                           |                                   |                             | 1 bits for subpriority
  *    --------------------------------------------------------------------------------------------------------------------------
  *     NVIC_PriorityGroup_4  |                0-15               |            0                | 4 bits for pre-emption priority
  *                           |                                   |                             | 0 bits for subpriority
  *    ==========================================================================================================================
  *
  *            2. Enable and Configure the priority of the selected IRQ Channels using NVIC_Init()
  *
  * @note  When the NVIC_PriorityGroup_0 is selected, IRQ pre-emption is no more possible.
  *        The pending IRQ priority will be managed only by the subpriority.
  *
  * @note  IRQ priority order (sorted by highest to lowest priority):
  *         - Lowest pre-emption priority
  *         - Lowest subpriority
  *         - Lowest hardware priority (IRQ number)
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <uvos.h>
#include "misc.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup MISC
  * @brief MISC driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup MISC_Private_Functions
  * @{
  */

/**
  * @brief  Configures the priority grouping: pre-emption priority and subpriority.
  * @param  NVIC_PriorityGroup: specifies the priority grouping bits length.
  *   This parameter can be one of the following values:
  *     @arg NVIC_PriorityGroup_0: 0 bits for pre-emption priority
  *                                4 bits for subpriority
  *     @arg NVIC_PriorityGroup_1: 1 bits for pre-emption priority
  *                                3 bits for subpriority
  *     @arg NVIC_PriorityGroup_2: 2 bits for pre-emption priority
  *                                2 bits for subpriority
  *     @arg NVIC_PriorityGroup_3: 3 bits for pre-emption priority
  *                                1 bits for subpriority
  *     @arg NVIC_PriorityGroup_4: 4 bits for pre-emption priority
  *                                0 bits for subpriority
  * @note   When the NVIC_PriorityGroup_0 is selected, IRQ pre-emption is no more possible.
  *         The pending IRQ priority will be managed only by the subpriority.
  * @retval None
  */
void NVIC_PriorityGroupConfig( uint32_t NVIC_PriorityGroup )
{
  /* Check the parameters */
  assert_param( IS_NVIC_PRIORITY_GROUP( NVIC_PriorityGroup ) );

  /* Set the PRIGROUP[10:8] bits according to NVIC_PriorityGroup value */
  SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup;
}

/**
  * @brief  Initializes the NVIC peripheral according to the specified
  *         parameters in the NVIC_InitStruct.
  * @note   To configure interrupts priority correctly, the NVIC_PriorityGroupConfig()
  *         function should be called before.
  * @param  NVIC_InitStruct: pointer to a NVIC_InitTypeDef structure that contains
  *         the configuration information for the specified NVIC peripheral.
  * @retval None
  */
void NVIC_Init( NVIC_InitTypeDef *NVIC_InitStruct )
{
  uint8_t tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x0F;

  /* Check the parameters */
  assert_param( IS_FUNCTIONAL_STATE( NVIC_InitStruct->NVIC_IRQChannelCmd ) );
  assert_param( IS_NVIC_PREEMPTION_PRIORITY( NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority ) );
  assert_param( IS_NVIC_SUB_PRIORITY( NVIC_InitStruct->NVIC_IRQChannelSubPriority ) );

  if ( NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE ) {
    /* Compute the Corresponding IRQ Priority --------------------------------*/
    tmppriority = ( 0x700 - ( ( SCB->AIRCR ) & ( uint32_t )0x700 ) ) >> 0x08;
    tmppre = ( 0x4 - tmppriority );
    tmpsub = tmpsub >> tmppriority;

    tmppriority = NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority << tmppre;
    tmppriority |=  ( uint8_t )( NVIC_InitStruct->NVIC_IRQChannelSubPriority & tmpsub );

    tmppriority = tmppriority << 0x04;

    NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel] = tmppriority;

    /* Enable the Selected IRQ Channels --------------------------------------*/
    NVIC->ISER[NVIC_InitStruct->NVIC_IRQChannel >> 0x05] =
      ( uint32_t )0x01 << ( NVIC_InitStruct->NVIC_IRQChannel & ( uint8_t )0x1F );
  } else {
    /* Disable the Selected IRQ Channels -------------------------------------*/
    NVIC->ICER[NVIC_InitStruct->NVIC_IRQChannel >> 0x05] =
      ( uint32_t )0x01 << ( NVIC_InitStruct->NVIC_IRQChannel & ( uint8_t )0x1F );
  }
}

/**
  * @brief  Sets the vector table location and Offset.
  * @param  NVIC_VectTab: specifies if the vector table is in RAM or FLASH memory.
  *   This parameter can be one of the following values:
  *     @arg NVIC_VectTab_RAM: Vector Table in internal SRAM.
  *     @arg NVIC_VectTab_FLASH: Vector Table in internal FLASH.
  * @param  Offset: Vector Table base offset field. This value must be a multiple of 0x200.
  * @retval None
  */
void NVIC_SetVectorTable( uint32_t NVIC_VectTab, uint32_t Offset )
{
  /* Check the parameters */
  assert_param( IS_NVIC_VECTTAB( NVIC_VectTab ) );
  assert_param( IS_NVIC_OFFSET( Offset ) );

  SCB->VTOR = NVIC_VectTab | ( Offset & ( uint32_t )0x1FFFFF80 );
}

/**
  * @brief  Selects the condition for the system to enter low power mode.
  * @param  LowPowerMode: Specifies the new mode for the system to enter low power mode.
  *   This parameter can be one of the following values:
  *     @arg NVIC_LP_SEVONPEND: Low Power SEV on Pend.
  *     @arg NVIC_LP_SLEEPDEEP: Low Power DEEPSLEEP request.
  *     @arg NVIC_LP_SLEEPONEXIT: Low Power Sleep on Exit.
  * @param  NewState: new state of LP condition. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void NVIC_SystemLPConfig( uint8_t LowPowerMode, FunctionalState NewState )
{
  /* Check the parameters */
  assert_param( IS_NVIC_LP( LowPowerMode ) );
  assert_param( IS_FUNCTIONAL_STATE( NewState ) );

  if ( NewState != DISABLE ) {
    SCB->SCR |= LowPowerMode;
  } else {
    SCB->SCR &= ( uint32_t )( ~( uint32_t )LowPowerMode );
  }
}

/**
  * @brief  Configures the SysTick clock source.
  * @param  SysTick_CLKSource: specifies the SysTick clock source.
  *   This parameter can be one of the following values:
  *     @arg SysTick_CLKSource_HCLK_Div8: AHB clock divided by 8 selected as SysTick clock source.
  *     @arg SysTick_CLKSource_HCLK: AHB clock selected as SysTick clock source.
  * @retval None
  */
void SysTick_CLKSourceConfig( uint32_t SysTick_CLKSource )
{
  /* Check the parameters */
  assert_param( IS_SYSTICK_CLK_SOURCE( SysTick_CLKSource ) );
  if ( SysTick_CLKSource == SysTick_CLKSource_HCLK ) {
    SysTick->CTRL |= SysTick_CLKSource_HCLK;
  } else {
    SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;
  }
}

// DMA helper functions --------------------------------------->>>

#define TRANSFER_IT_MASK        (uint32_t)0x0F3C0F3C
#define RESERVED_MASK           (uint32_t)0x0F7D0F7D

static const uint8_t UVOS_DMA_flag_bit_offset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};

/**
  * @brief  Enables or disables the specified DMAy Streamx interrupts.
  * @param  DMAx DMA Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param DMA_ints: specifies the DMA interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg UVOS_DMA_IT_TC:  Stream transfer complete interrupt
  *            @arg UVOS_DMA_IT_HT:  Stream half transfer complete interrupt
  *            @arg UVOS_DMA_IT_TE:  Stream transfer error interrupt
  *            @arg UVOS_DMA_IT_DME: Stream direct mode error interrupt
  *            @arg UVOS_DMA_IT_FE:  Stream FIFO error interrupt
  * @param  NewState: new state of the specified DMA interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DMA_ITConfig( DMA_TypeDef *DMAx, uint32_t Stream, uint32_t DMA_ints, FunctionalState NewState )
{
  /* Check the parameters */
  UVOS_Assert( UVOS_DMA_IS_INSTANCE( DMAx ) );
  UVOS_Assert( UVOS_DMA_IS_STREAM( Stream ) );
  UVOS_Assert( UVOS_DMA_IS_ITEN_BITS( DMA_ints ) );
  UVOS_Assert( IS_FUNCTIONAL_STATE( NewState ) );

  DMA_Stream_TypeDef *DMAy_Streamx = __LL_DMA_GET_STREAM_INSTANCE( DMAx, Stream );

  /* Check if the DMA_ints parameter contains FIFO interrupt */
  if ( ( DMA_ints & UVOS_DMA_ITEN_FE ) != 0 ) {
    if ( NewState != DISABLE ) {
      /* Enable the selected DMA FIFO interrupts */
      DMAy_Streamx->FCR |= ( uint32_t )UVOS_DMA_ITEN_FE;
    } else {
      /* Disable the selected DMA FIFO interrupts */
      DMAy_Streamx->FCR &= ~( uint32_t )UVOS_DMA_ITEN_FE;
    }
  }

  /* Mask off the FIFO interrupt bit */
  CLEAR_BIT( DMA_ints, UVOS_DMA_ITEN_FE );

  /* Check if the DMA_ints parameter contains any of the transfer interrupt bits */
  if ( UVOS_DMA_IS_TRANSFER_ITEN_BITS( DMA_ints ) ) {
    /* Shift (enum UVOS_DMA_IT bits)DMA_ints right by 1 to align with DMA_SxCR bits */
    if ( NewState != DISABLE ) {
      /* Enable the selected DMA transfer interrupts */
      DMAy_Streamx->CR |= ( uint32_t )( DMA_ints >> 1 );
    } else {
      /* Disable the selected DMA transfer interrupts */
      DMAy_Streamx->CR &= ~( uint32_t )( DMA_ints >> 1 );
    }
  }
}

/**
  * @brief  Checks whether the specified DMA Stream interrupt has occurred or not.
  * @param  DMAx DMA Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  DMA_int: specifies the DMA interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg UVOS_DMA_IT_TC:  Stream transfer complete interrupt
  *            @arg UVOS_DMA_IT_HT:  Stream half transfer complete interrupt
  *            @arg UVOS_DMA_IT_TE:  Stream transfer error interrupt
  *            @arg UVOS_DMA_IT_DME: Stream direct mode error interrupt
  *            @arg UVOS_DMA_IT_FE:  Stream FIFO error interrupt
  * @retval The new state of DMA_IT (ITStatus, set to either SET or RESET).
  */
ITStatus DMA_IsActiveFlag( DMA_TypeDef *DMAx, uint32_t Stream, uint32_t DMA_int )
{
  ITStatus bitstatus = RESET;
  uint32_t tmpreg = 0;
  uint32_t enablestatus = 0;

  /* Check the parameters */
  UVOS_Assert( UVOS_DMA_IS_INSTANCE( DMAx ) );
  UVOS_Assert( UVOS_DMA_IS_STREAM( Stream ) );
  UVOS_Assert( UVOS_DMA_IS_SINGLE_IFLG_BIT( DMA_int ) );

  /* Check if the interrupt enable bit is set in the CR or FCR register */
  if ( DMA_int != UVOS_DMA_ITEN_FE ) {
    /* Get the interrupt enable position mask in CR register (shift right 1 to align with DMA_SxCR bits) */
    tmpreg = ( uint32_t )( DMA_int >> 1 );
    /* Check the enable bit in CR register */
    // enablestatus = ( uint32_t )( DMAy_Streamx->CR & tmpreg );
    enablestatus = ( READ_BIT( ( ( DMA_Stream_TypeDef * )( ( uint32_t )( ( uint32_t )DMAx + STREAM_OFFSET_TAB[Stream] ) ) )->CR, tmpreg ) == tmpreg );
  } else {
    /* Check the enable bit in FCR register */
    // enablestatus = ( uint32_t )( DMAy_Streamx->FCR & DMA_IT_FE );
    enablestatus = LL_DMA_IsEnabledIT_FE( DMAx, Stream );
  }

  /* Check if the interrupt pending flag is in LISR or HISR */
  if ( Stream > 3 ) {
    /* Get DMAy HISR register value */
    tmpreg = DMAx->HISR ;
  } else {
    /* Get DMAy LISR register value */
    tmpreg = DMAx->LISR ;
  }

  /* mask all reserved bits */
  tmpreg &= ( uint32_t )RESERVED_MASK;

  /* Shift tmpreg right based on Stream number */
  tmpreg = ( tmpreg >> UVOS_DMA_flag_bit_offset[ Stream ] );

  /* Check the status of the specified DMA interrupt */
  if ( ( ( tmpreg & DMA_int ) != ( uint32_t )RESET ) && ( enablestatus != ( uint32_t )RESET ) ) {
    /* DMA_IT is set */
    bitstatus = SET;
  } else {
    /* DMA_IT is reset */
    bitstatus = RESET;
  }

  /* Return the DMA_IT status */
  return bitstatus;
}

/**
  * @brief  Clears the specified DMA Stream's interrupt pending bits.
  * @param  DMAx DMA Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param DMA_ints: specifies the DMA interrupt sources to be cleared.
  *          This parameter can be any combination of the following values:
  *            @arg UVOS_DMA_IT_TC:  Stream transfer complete interrupt
  *            @arg UVOS_DMA_IT_HT:  Stream half transfer complete interrupt
  *            @arg UVOS_DMA_IT_TE:  Stream transfer error interrupt
  *            @arg UVOS_DMA_IT_DME: Stream direct mode error interrupt
  *            @arg UVOS_DMA_IT_FE:  Stream FIFO error interrupt
  *          Note that UVOS_DMA_IT_XX values are same as UVOS_DMA_IFLG_XXIF
  * @retval None
  */
void DMA_ClearFlags( DMA_TypeDef *DMAx, uint32_t Stream, uint32_t DMA_ints )
{
  /* Check the parameters */
  UVOS_Assert( UVOS_DMA_IS_INSTANCE( DMAx ) );
  UVOS_Assert( UVOS_DMA_IS_STREAM( Stream ) );
  UVOS_Assert( UVOS_DMA_IS_IFLG_BITS ( DMA_ints ) );

  /* Shift DMA_ints to the appropriate offset based on Stream number */
  DMA_ints = ( DMA_ints << UVOS_DMA_flag_bit_offset[ Stream ] );

  /* Check if LIFCR or HIFCR register is targeted */
  if ( Stream > 3 ) {
    DMAx->HIFCR = ( uint32_t )( DMA_ints & RESERVED_MASK );
  } else {
    DMAx->LIFCR = ( uint32_t )( DMA_ints & RESERVED_MASK );
  }
}

/**
  * @brief  Clears all DMA Stream's interrupt pending bits.
  * @param  DMAx DMA Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
void DMA_ClearAllFlags( DMA_TypeDef *DMAx, uint32_t Stream )
{
  /* Check the parameters */
  UVOS_Assert( UVOS_DMA_IS_INSTANCE( DMAx ) );
  UVOS_Assert( UVOS_DMA_IS_STREAM( Stream ) );

  /* Shift DMA_ints to the appropriate offset based on Stream number */
  uint32_t DMA_ints = ( UVOS_DMA_ALL_ITEN_BITS << UVOS_DMA_flag_bit_offset[ Stream ] );

  /* Check if LIFCR or HIFCR register is targeted */
  if ( Stream > 3 ) {
    DMAx->HIFCR = ( uint32_t )( DMA_ints & RESERVED_MASK );
  } else {
    DMAx->LIFCR = ( uint32_t )( DMA_ints & RESERVED_MASK );
  }
}

/**
  * @brief Clear and wait a specified duration for EN bit in DMA_SxCR register to reset (“0”)
  * @param  DMAx DMA Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval 0 on successful EN reset, -1 on timeout.
  */
int32_t DMA_DisableStreamWaitTimeout( DMA_TypeDef *DMAx, uint32_t Stream, uint32_t timeout_us )
{
  /* Check the parameters */
  UVOS_Assert( UVOS_DMA_IS_INSTANCE( DMAx ) );
  UVOS_Assert( UVOS_DMA_IS_STREAM( Stream ) );

  uint32_t start = UVOS_DELAY_GetRaw();

  LL_DMA_DisableStream( DMAx, Stream );
  /* Wait until EN bit in DMA_SxCR register is reset (“0”) or timeout reached. */
  do {
    if ( LL_DMA_IsEnabledStream( DMAx, Stream ) == 0 ) {
      return 0;
    }
  } while ( UVOS_DELAY_DiffuS( start ) < timeout_us );

  /* Timed out */
  return -1;
}

// End DMA helper functions -----------------------------------<<<

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

