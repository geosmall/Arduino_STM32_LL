#include "uvos.h"

#ifdef UVOS_INCLUDE_TIM

#include "uvos_tim_priv.h"

enum uvos_tim_dev_magic {
  UVOS_TIM_DEV_MAGIC = 0x87654098,
};

struct uvos_tim_dev {
  enum uvos_tim_dev_magic magic;

  const TIM_TypeDef *timer;

  const struct uvos_tim_channel *channels;
  uint8_t num_channels;

  const struct uvos_tim_callbacks *callbacks;
  uint32_t context;
};

// STM32 SPL compatibility ------------------------------------>>>

/**
  * @brief  Checks whether the TIM interrupt has occurred or not.
  * @param  TIMx: where x can be 1 to 14 to select the TIM peripheral.
  * @param  TIM_IT: specifies the TIM interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg TIM_IT_Update: TIM update Interrupt source
  *            @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
  *            @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
  *            @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
  *            @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
  *            @arg TIM_IT_COM: TIM Commutation Interrupt source
  *            @arg TIM_IT_Trigger: TIM Trigger Interrupt source
  *            @arg TIM_IT_Break: TIM Break Interrupt source
  *
  * @note   TIM6 and TIM7 can generate only an update interrupt.
  * @note   TIM_IT_COM and TIM_IT_Break are used only with TIM1 and TIM8.
  *
  * @retval The new state of the TIM_IT(SET or RESET).
  */
static ITStatus TIM_GetITStatus( TIM_TypeDef *TIMx, uint16_t TIM_IT )
{
  ITStatus bitstatus = RESET;
  uint16_t itstatus = 0x0, itenable = 0x0;
  /* Check the parameters */
  assert_param( IS_TIM_INSTANCE( TIMx ) );
  assert_param( IS_TIM_GET_IT( TIM_IT ) );

  itstatus = TIMx->SR & TIM_IT;

  itenable = TIMx->DIER & TIM_IT;
  if ( ( itstatus != ( uint16_t )RESET ) && ( itenable != ( uint16_t )RESET ) ) {
    bitstatus = SET;
  } else {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Clears the TIMx's interrupt pending bits.
  * @param  TIMx: where x can be 1 to 14 to select the TIM peripheral.
  * @param  TIM_IT: specifies the pending bit to clear.
  *          This parameter can be any combination of the following values:
  *            @arg TIM_IT_Update: TIM1 update Interrupt source
  *            @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
  *            @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
  *            @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
  *            @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
  *            @arg TIM_IT_COM: TIM Commutation Interrupt source
  *            @arg TIM_IT_Trigger: TIM Trigger Interrupt source
  *            @arg TIM_IT_Break: TIM Break Interrupt source
  *
  * @note   TIM6 and TIM7 can generate only an update interrupt.
  * @note   TIM_IT_COM and TIM_IT_Break are used only with TIM1 and TIM8.
  *
  * @retval None
  */
static inline void TIM_ClearITPendingBit( TIM_TypeDef *TIMx, uint16_t TIM_IT )
{
  /* Check the parameters */
  assert_param( IS_TIM_INSTANCE( TIMx ) );

  /* Clear the IT pending Bit */
  TIMx->SR = ( uint16_t )~TIM_IT;
}

// STM32 SPL compatibility ------------------------------------<<<

// static bool UVOS_TIM_validate( struct uvos_tim_dev * tim_dev )
// {
//   return tim_dev->magic == UVOS_TIM_DEV_MAGIC;
// }

static struct uvos_tim_dev uvos_tim_devs[UVOS_TIM_MAX_DEVS];
static uint8_t uvos_tim_num_devs;
static struct uvos_tim_dev *UVOS_TIM_alloc( void )
{
  struct uvos_tim_dev *tim_dev;

  if ( uvos_tim_num_devs >= UVOS_TIM_MAX_DEVS ) {
    return NULL;
  }

  tim_dev = &uvos_tim_devs[uvos_tim_num_devs++];
  tim_dev->magic = UVOS_TIM_DEV_MAGIC;

  return tim_dev;
}


int32_t UVOS_TIM_InitClock( const struct uvos_tim_clock_cfg *cfg )
{
  UVOS_DEBUG_Assert( cfg );

  /* Configure the dividers for this timer */
  LL_TIM_Init( cfg->timer, ( LL_TIM_InitTypeDef * ) cfg->time_base_init );

  /* Configure internal timer clocks */
  LL_TIM_SetClockSource( cfg->timer, LL_TIM_CLOCKSOURCE_INTERNAL );

  /* Enable timers */
  LL_TIM_EnableCounter( cfg->timer );

  /* Enable Interrupts */
  NVIC_Init( ( NVIC_InitTypeDef * ) & cfg->irq.init );

  // Advanced timers TIM1 & TIM8 (STM32F405 only) need special handling:
  // There are up to 4 separate interrupts handlers for each advanced timer, but
  // pios_tim_clock_cfg has provision for only one irq init, so we take care here
  // to enable additional irq channels that we intend to use.

  // if ( IS_TIM_ADVANCED_INSTANCE( cfg->timer ) )
  if ( cfg->timer == TIM1 ) {
    NVIC_InitTypeDef init = cfg->irq.init;
    init.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
    NVIC_Init( &init );
#if defined(TIM8)
  } else if ( cfg->timer == TIM8 ) {
    NVIC_InitTypeDef init = cfg->irq.init;
    init.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
    NVIC_Init( &init );
#endif // defined(TIM8)
  }

  return 0;
}

int32_t UVOS_TIM_InitTimebase( uint32_t *tim_id, const TIM_TypeDef *timer, const struct uvos_tim_callbacks *callbacks, uint32_t context )
{
  UVOS_Assert( IS_TIM_INSTANCE( timer ) );

  struct uvos_tim_dev *tim_dev;
  tim_dev = ( struct uvos_tim_dev * )UVOS_TIM_alloc();
  if ( !tim_dev ) {
    goto out_fail;
  }

  /* Bind the configuration to the device instance, set channels to NULL for timebase */
  tim_dev->timer        = timer;
  tim_dev->channels     = NULL;
  tim_dev->num_channels = 0;
  tim_dev->callbacks    = callbacks;
  tim_dev->context      = context;

  *tim_id = ( uint32_t )tim_dev;

  return 0;

out_fail:
  return -1;
}

int32_t UVOS_TIM_InitChannels( uint32_t *tim_id, const struct uvos_tim_channel *channels, uint8_t num_channels, const struct uvos_tim_callbacks *callbacks, uint32_t context )
{
  UVOS_Assert( channels );
  UVOS_Assert( num_channels );

  struct uvos_tim_dev *tim_dev;
  tim_dev = ( struct uvos_tim_dev * )UVOS_TIM_alloc();
  if ( !tim_dev ) {
    goto out_fail;
  }

  /* Bind the configuration to the device instance, set timer to NULL for channels */
  tim_dev->timer        = NULL;
  tim_dev->channels     = channels;
  tim_dev->num_channels = num_channels;
  tim_dev->callbacks    = callbacks;
  tim_dev->context      = context;

  /* Configure the pins */
  for ( uint8_t i = 0; i < num_channels; i++ ) {
    const struct uvos_tim_channel *chan = &( channels[i] );
    if ( IS_GPIO_ALL_INSTANCE( chan->pin.gpio ) ) {
      LL_GPIO_Init( chan->pin.gpio, ( LL_GPIO_InitTypeDef * ) & chan->pin.init );
    } else {
      UVOS_Assert( 0 );
    }
  }

  *tim_id = ( uint32_t )tim_dev;

  return 0;

out_fail:
  return -1;
}

// void TimerUpdate_Callback( void )
// {
//   asm( "NOP" );
// }

// void TIM1_TRG_COM_TIM11_IRQHandler( void )
// {
//   /* Check whether update interrupt is pending */
//   // if ( LL_TIM_IsActiveFlag_UPDATE( TIM11 ) == 1 ) {
//   //   /* Clear the update interrupt flag*/
//   //   LL_TIM_ClearFlag_UPDATE( TIM11 );
//   // }

//   if ( READ_BIT( TIM11->SR, TIM_SR_UIF ) == ( TIM_SR_UIF ) ) {
//     WRITE_REG( TIM11->SR, ~( TIM_SR_UIF ) );
//     // CLEAR_BIT(TIM11->SR, TIM_SR_UIF);
//     TimerUpdate_Callback();
//   }
// }

static void UVOS_TIM_generic_irq_handler( TIM_TypeDef *timer )
{
  /* Iterate over all registered clients of the TIM layer to find channels on this timer */
  for ( uint8_t i = 0; i < uvos_tim_num_devs; i++ ) {
    const struct uvos_tim_dev *tim_dev = &uvos_tim_devs[i];

    if ( !tim_dev->channels || tim_dev->num_channels == 0 ) {
      /* No channels to process on this client */
      if ( TIM_GetITStatus( timer, TIM_IT_Update ) == SET ) {
        TIM_ClearITPendingBit( timer, TIM_IT_Update );
        // if ( LL_TIM_IsActiveFlag_UPDATE( timer ) == SET ) {
        //   LL_TIM_ClearFlag_UPDATE( timer );
        // if (READ_BIT(TIM11->SR, TIM_SR_UIF) == (TIM_SR_UIF)) {
        //   WRITE_REG(TIM11->SR, ~(TIM_SR_UIF));
        // if ( LL_TIM_IsActiveFlag_UPDATE( TIM11 ) == 1 ) {
        //   LL_TIM_ClearFlag_UPDATE( TIM11 );
        if ( tim_dev->callbacks->overflow ) {
          ( *tim_dev->callbacks->overflow )( ( uint32_t )tim_dev, tim_dev->context, 0, 0 );
        }
      }
      continue;
    }

    /* Check for an overflow event on this timer */
    bool overflow_event;
    uint16_t overflow_count;

    if ( TIM_GetITStatus( timer, TIM_IT_Update ) == SET ) {
      TIM_ClearITPendingBit( timer, TIM_IT_Update );
      // if ( LL_TIM_IsActiveFlag_UPDATE( timer ) == SET ) {
      //   LL_TIM_ClearFlag_UPDATE( timer );
      overflow_count = timer->ARR;
      overflow_event = true;
    } else {
      overflow_count = 0;
      overflow_event = false;
    }

    for ( uint8_t j = 0; j < tim_dev->num_channels; j++ ) {
      const struct uvos_tim_channel *chan = &tim_dev->channels[j];

      if ( chan->timer != timer ) {
        /* channel is not on this timer */
        continue;
      }

      /* Figure out which interrupt bit we should be looking at */
      uint16_t timer_it;
      switch ( chan->timer_chan ) {
      case LL_TIM_CHANNEL_CH1:
        timer_it = TIM_IT_CC1;
        break;
      case LL_TIM_CHANNEL_CH2:
        timer_it = TIM_IT_CC2;
        break;
      case LL_TIM_CHANNEL_CH3:
        timer_it = TIM_IT_CC3;
        break;
      case LL_TIM_CHANNEL_CH4:
        timer_it = TIM_IT_CC4;
        break;
      default:
        UVOS_Assert( 0 );
        break;
      }

      bool edge_event;
      uint16_t edge_count;
      if ( TIM_GetITStatus( chan->timer, timer_it ) == SET ) {
        TIM_ClearITPendingBit( chan->timer, timer_it );

        /* Read the current counter */
        switch ( chan->timer_chan ) {
        case LL_TIM_CHANNEL_CH1:
          edge_count = LL_TIM_IC_GetCaptureCH1( chan->timer );
          break;
        case LL_TIM_CHANNEL_CH2:
          edge_count = LL_TIM_IC_GetCaptureCH2( chan->timer );
          break;
        case LL_TIM_CHANNEL_CH3:
          edge_count = LL_TIM_IC_GetCaptureCH3( chan->timer );
          break;
        case LL_TIM_CHANNEL_CH4:
          edge_count = LL_TIM_IC_GetCaptureCH4( chan->timer );
          break;
        default:
          UVOS_Assert( 0 );
          break;
        }
        edge_event = true;
      } else {
        edge_event = false;
        edge_count = 0;
      }

      if ( !tim_dev->callbacks ) {
        /* No callbacks registered, we're done with this channel */
        continue;
      }

      /* Generate the appropriate callbacks */
      if ( overflow_event & edge_event ) {
        /*
         * When both edge and overflow happen in the same interrupt, we
         * need a heuristic to determine the order of the edge and overflow
         * events so that the callbacks happen in the right order.  If we
         * get the order wrong, our pulse width calculations could be off by up
         * to ARR ticks.  That could be bad.
         *
         * Heuristic: If the edge_count is < 16 ticks above zero then we assume the
         *            edge happened just after the overflow.
         */

        if ( edge_count < 16 ) {
          /* Call the overflow callback first */
          if ( tim_dev->callbacks->overflow ) {
            ( *tim_dev->callbacks->overflow )( ( uint32_t )tim_dev, tim_dev->context, j, overflow_count );
          }
          /* Call the edge callback second */
          if ( tim_dev->callbacks->edge ) {
            ( *tim_dev->callbacks->edge )( ( uint32_t )tim_dev, tim_dev->context, j, edge_count );
          }
        } else {
          /* Call the edge callback first */
          if ( tim_dev->callbacks->edge ) {
            ( *tim_dev->callbacks->edge )( ( uint32_t )tim_dev, tim_dev->context, j, edge_count );
          }
          /* Call the overflow callback second */
          if ( tim_dev->callbacks->overflow ) {
            ( *tim_dev->callbacks->overflow )( ( uint32_t )tim_dev, tim_dev->context, j, overflow_count );
          }
        }
      } else if ( overflow_event && tim_dev->callbacks->overflow ) {
        ( *tim_dev->callbacks->overflow )( ( uint32_t )tim_dev, tim_dev->context, j, overflow_count );
      } else if ( edge_event && tim_dev->callbacks->edge ) {
        ( *tim_dev->callbacks->edge )( ( uint32_t )tim_dev, tim_dev->context, j, edge_count );
      }
    }
  }
}

/* Bind Interrupt Handlers
 *
 * Map all valid TIM IRQs to the common interrupt handler
 * and give it enough context to properly demux the various timers
 */
void TIM1_CC_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_1_CC_irq_handler" ) ) );
static void UVOS_TIM_1_CC_irq_handler( void )
{
  UVOS_TIM_generic_irq_handler( TIM1 );
}

void TIM2_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_2_irq_handler" ) ) );
static void UVOS_TIM_2_irq_handler( void )
{
  UVOS_TIM_generic_irq_handler( TIM2 );
}

void TIM3_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_3_irq_handler" ) ) );
static void UVOS_TIM_3_irq_handler( void )
{
  UVOS_TIM_generic_irq_handler( TIM3 );
}

void TIM4_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_4_irq_handler" ) ) );
static void UVOS_TIM_4_irq_handler( void )
{
  UVOS_TIM_generic_irq_handler( TIM4 );
}

void TIM5_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_5_irq_handler" ) ) );
static void UVOS_TIM_5_irq_handler( void )
{
  UVOS_TIM_generic_irq_handler( TIM5 );
}

#if defined(TIM6)
void TIM6_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_6_irq_handler" ) ) );
static void UVOS_TIM_6_irq_handler( void )
{
  UVOS_TIM_generic_irq_handler( TIM6 );
}
#endif // defined(TIM6)

#if defined(TIM7)
void TIM7_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_7_irq_handler" ) ) );
static void UVOS_TIM_7_irq_handler( void )
{
  UVOS_TIM_generic_irq_handler( TIM7 );
}
#endif // defined(TIM7)

#if defined(TIM8)
void TIM8_CC_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_8_CC_irq_handler" ) ) );
static void UVOS_TIM_8_CC_irq_handler( void )
{
  UVOS_TIM_generic_irq_handler( TIM8 );
}
#endif // defined(TIM8)

// The rest of advanced timer TIM1 interrupts are shared interrupts
void TIM1_BRK_TIM9_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_9_CC_irq_handler" ) ) );
static void UVOS_TIM_9_CC_irq_handler( void )
{
  if ( TIM_GetITStatus( TIM1, TIM_IT_Break ) ) {
    UVOS_TIM_generic_irq_handler( TIM1 );
  } else if ( TIM_GetITStatus( TIM9, UVOS_TIM_ALL_FLAGS ) ) {
    UVOS_TIM_generic_irq_handler( TIM9 );
  }
}

void TIM1_UP_TIM10_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_10_CC_irq_handler" ) ) );
static void UVOS_TIM_10_CC_irq_handler( void )
{
  if ( TIM_GetITStatus( TIM1, TIM_IT_Update ) ) {
    UVOS_TIM_generic_irq_handler( TIM1 );
  } else if ( TIM_GetITStatus( TIM10, UVOS_TIM_ALL_FLAGS ) ) {
    UVOS_TIM_generic_irq_handler( TIM10 );
  }
}

void TIM1_TRG_COM_TIM11_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_11_CC_irq_handler" ) ) );
static void UVOS_TIM_11_CC_irq_handler( void )
{
  if ( TIM_GetITStatus( TIM1, TIM_IT_COM | TIM_IT_Trigger ) ) {
    UVOS_TIM_generic_irq_handler( TIM1 );
  } else if ( TIM_GetITStatus( TIM11, UVOS_TIM_ALL_FLAGS ) ) {
    UVOS_TIM_generic_irq_handler( TIM11 );
  }
}

#if defined(TIM8)
// The rest of advanced timer TIM8 interrupts are shared interrupts

void TIM8_BRK_TIM12_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM_12_irq_handler" ) ) );
static void UVOS_TIM_12_irq_handler( void )
{
  if ( TIM_GetITStatus( TIM8, TIM_IT_Break ) ) {
    UVOS_TIM_generic_irq_handler( TIM8 );
  } else if ( TIM_GetITStatus( TIM12, UVOS_TIM_ALL_FLAGS ) ) {
    UVOS_TIM_generic_irq_handler( TIM12 );
  }
}

void TIM8_UP_TIM13_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM8_UP_TIM13_IRQHandler" ) ) );
static void UVOS_TIM8_UP_TIM13_IRQHandler( void )
{
  if ( TIM_GetITStatus( TIM8, TIM_IT_Update ) ) {
    UVOS_TIM_generic_irq_handler( TIM8 );
  } else if ( TIM_GetITStatus( TIM13, UVOS_TIM_ALL_FLAGS ) ) {
    UVOS_TIM_generic_irq_handler( TIM13 );
  }
}

void TIM8_TRG_COM_TIM14_IRQHandler( void ) __attribute__( ( alias( "UVOS_TIM8_TRG_COM_TIM14_IRQHandler" ) ) );
static void UVOS_TIM8_TRG_COM_TIM14_IRQHandler( void )
{
  if ( TIM_GetITStatus( TIM8, TIM_IT_COM | TIM_IT_Trigger ) ) {
    UVOS_TIM_generic_irq_handler( TIM8 );
  } else if ( TIM_GetITStatus( TIM14, UVOS_TIM_ALL_FLAGS ) ) {
    UVOS_TIM_generic_irq_handler( TIM14 );
  }
}

#endif // defined(TIM8)

#endif /* UVOS_INCLUDE_TIM */

/**
 * @}
 * @}
 */
