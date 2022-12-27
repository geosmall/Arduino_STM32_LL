#include "uvos.h"

#ifdef UVOS_INCLUDE_SERVO

#include "uvos_servo_priv.h"
#include "uvos_tim_priv.h"

/* Private Function Prototypes */

static const struct uvos_servo_cfg *servo_cfg;

// determine if the related timer will work in synchronous (or OneShot/OneShot125) One Pulse mode.
static uint8_t uvos_servo_bank_mode[ UVOS_SERVO_BANKS ] = { 0 };
// used to skip updates when pulse length is higher than update cycle
static uint16_t uvos_servo_bank_next_update[ UVOS_SERVO_BANKS ] = { 0 };
static uint16_t uvos_servo_bank_max_pulse[ UVOS_SERVO_BANKS ] = { 0 };
// timer associated to each bank
static TIM_TypeDef *uvos_servo_bank_timer[ UVOS_SERVO_BANKS ] = { 0 };

// index of bank used for each pin
static uint8_t *uvos_servo_pin_bank;

#define UVOS_SERVO_TIMER_CLOCK 1000000
#define UVOS_SERVO_SAFE_MARGIN 50
/**
 * Initialise Servos
 */
int32_t UVOS_Servo_Init( const struct uvos_servo_cfg *cfg )
{
  uint32_t tim_id;

  /* Initialize the servo output pins */
  if ( UVOS_TIM_InitChannels( &tim_id, cfg->channels, cfg->num_channels, NULL, 0 ) ) {
    return -1;
  }

  /* Store away the requested configuration */
  servo_cfg = cfg;

  /* Create servo pin bank array, sized to total number of channels */
  uvos_servo_pin_bank = UVOS_malloc( sizeof( uint8_t ) * cfg->num_channels );
  memset( uvos_servo_pin_bank, 0, ( cfg->num_channels ) ); // memset(void *str, int c, size_t n)

  uint8_t bank = 0;

  for ( uint8_t i = 0; ( i < servo_cfg->num_channels ); i++ ) {

    const struct uvos_tim_channel *chan = &servo_cfg->channels[ i ];

    bool new = true;
    /* See if any previous channels use that same timer */
    for ( uint8_t j = 0; ( j < i ) && new; j++ ) {
      new &= chan->timer != servo_cfg->channels[ j ].timer;
    }

    if ( new ) {
      UVOS_Assert( bank < UVOS_SERVO_BANKS );
      for ( uint8_t j = i; j < servo_cfg->num_channels; j++ ) {
        if ( servo_cfg->channels[ j ].timer == chan->timer ) {
          uvos_servo_pin_bank[ j ] = bank;
        }
      }
      uvos_servo_bank_timer[bank] = chan->timer;

      // TIM_ARRPreloadConfig( chan->timer, ENABLE );
      LL_TIM_EnableARRPreload( chan->timer );
      // TIM_CtrlPWMOutputs( chan->timer, ENABLE );
      LL_TIM_EnableAllOutputs( chan->timer );
      // TIM_Cmd( chan->timer, DISABLE );
      LL_TIM_DisableCounter( chan->timer );

      bank++;
    }

    /* Set up for output compare function */
    switch ( chan->timer_chan ) {
    case LL_TIM_CHANNEL_CH1:
      // TIM_OC1Init( chan->timer, &servo_cfg->tim_oc_init );
      LL_TIM_OC_Init( chan->timer, LL_TIM_CHANNEL_CH1, ( LL_TIM_OC_InitTypeDef * ) &servo_cfg->tim_oc_init  );
      // TIM_OC1PreloadConfig( chan->timer, TIM_OCPreload_Enable );
      LL_TIM_OC_EnablePreload( chan->timer, LL_TIM_CHANNEL_CH1 );
      break;
    case LL_TIM_CHANNEL_CH2:
      // TIM_OC2Init( chan->timer, &servo_cfg->tim_oc_init );
      LL_TIM_OC_Init( chan->timer, LL_TIM_CHANNEL_CH2, ( LL_TIM_OC_InitTypeDef * ) &servo_cfg->tim_oc_init  );
      // TIM_OC2PreloadConfig( chan->timer, TIM_OCPreload_Enable );
      LL_TIM_OC_EnablePreload( chan->timer, LL_TIM_CHANNEL_CH2 );
      break;
    case LL_TIM_CHANNEL_CH3:
      // TIM_OC3Init( chan->timer, &servo_cfg->tim_oc_init );
      LL_TIM_OC_Init( chan->timer, LL_TIM_CHANNEL_CH3, ( LL_TIM_OC_InitTypeDef * ) &servo_cfg->tim_oc_init  );
      // TIM_OC3PreloadConfig( chan->timer, TIM_OCPreload_Enable );
      LL_TIM_OC_EnablePreload( chan->timer, LL_TIM_CHANNEL_CH3 );
      break;
    case LL_TIM_CHANNEL_CH4:
      // TIM_OC4Init( chan->timer, &servo_cfg->tim_oc_init );
      LL_TIM_OC_Init( chan->timer, LL_TIM_CHANNEL_CH4, ( LL_TIM_OC_InitTypeDef * ) &servo_cfg->tim_oc_init  );
      // TIM_OC4PreloadConfig( chan->timer, TIM_OCPreload_Enable );
      LL_TIM_OC_EnablePreload( chan->timer, LL_TIM_CHANNEL_CH4 );
      break;
    }
  }

  return 0;
}


/**
  * @brief  Set the servo bank mode
  * @param  bank servo bank member to set
  * @param  mode enum  UVOS_SERVO_BANK_MODE_PWM, UVOS_SERVO_BANK_MODE_SINGLE_PULSE
  * @retval None
  */
void UVOS_Servo_SetBankMode( uint8_t bank, uvos_servo_bank_mode_e mode )
{
  UVOS_Assert( bank < UVOS_SERVO_BANKS );

  uvos_servo_bank_mode[ bank ] = mode;

  if ( uvos_servo_bank_timer[ bank ] ) {
    for ( uint8_t i = 0; ( i < servo_cfg->num_channels ); i++ ) {
      if ( uvos_servo_pin_bank[ i ] == bank ) {
        const struct uvos_tim_channel *chan = &servo_cfg->channels[ i ];
        /* Set up for output compare function */
        switch ( chan->timer_chan ) {
        case LL_TIM_CHANNEL_CH1:
          // TIM_OC1PolarityConfig( chan->timer, TIM_OCPolarity_High );
          LL_TIM_OC_SetPolarity( chan->timer, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH );
          break;
        case LL_TIM_CHANNEL_CH2:
          // TIM_OC2PolarityConfig( chan->timer, TIM_OCPolarity_High );
          LL_TIM_OC_SetPolarity( chan->timer, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH );
          break;
        case LL_TIM_CHANNEL_CH3:
          // TIM_OC3PolarityConfig( chan->timer, TIM_OCPolarity_High );
          LL_TIM_OC_SetPolarity( chan->timer, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH );
          break;
        case LL_TIM_CHANNEL_CH4:
          // TIM_OC4PolarityConfig( chan->timer, TIM_OCPolarity_High );
          LL_TIM_OC_SetPolarity( chan->timer, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH );
          break;
        }
      }
    }

    // Setup the timer accordingly
    // TIM_SelectOnePulseMode( uvos_servo_bank_timer[bank], TIM_OPMode_Repetitive );
    LL_TIM_SetOnePulseMode( uvos_servo_bank_timer[ bank ], LL_TIM_ONEPULSEMODE_REPETITIVE );
    // TIM_Cmd( uvos_servo_bank_timer[bank], ENABLE );
    LL_TIM_EnableCounter( uvos_servo_bank_timer[ bank ] );
  }
}


void UVOS_Servo_Update()
{
  for ( uint8_t i = 0; ( i < UVOS_SERVO_BANKS ); i++ ) {
    const TIM_TypeDef *timer = uvos_servo_bank_timer[ i ];
    if ( timer && uvos_servo_bank_mode[ i ] == UVOS_SERVO_BANK_MODE_SINGLE_PULSE ) {
      // a pulse to be generated is longer than cycle period. skip this update.
      if ( LL_TIM_GetCounter( ( TIM_TypeDef * )timer ) > ( uint32_t )( uvos_servo_bank_next_update[ i ] + UVOS_SERVO_SAFE_MARGIN ) ) {
        // TIM_GenerateEvent( ( TIM_TypeDef * )timer, TIM_EventSource_Update );
        LL_TIM_GenerateEvent_UPDATE( ( TIM_TypeDef * )timer );
        uvos_servo_bank_next_update[ i ] = uvos_servo_bank_max_pulse[ i ];
      }
    }
    uvos_servo_bank_max_pulse[ i ] = 0;
  }
  for ( uint8_t i = 0; ( i < servo_cfg->num_channels ); i++ ) {
    uint8_t bank = uvos_servo_pin_bank[ i ];
    uint8_t mode = uvos_servo_bank_mode[ bank ];
    if ( mode == UVOS_SERVO_BANK_MODE_SINGLE_PULSE ) {
      /* Update the position */
      const struct uvos_tim_channel *chan = &servo_cfg->channels[ i ];

      switch ( chan->timer_chan ) {
      case LL_TIM_CHANNEL_CH1:
        // TIM_SetCompare1( chan->timer, 0 );
        LL_TIM_OC_SetCompareCH1( chan->timer, 0 );
        break;
      case LL_TIM_CHANNEL_CH2:
        // TIM_SetCompare2( chan->timer, 0 );
        LL_TIM_OC_SetCompareCH2( chan->timer, 0 );
        break;
      case LL_TIM_CHANNEL_CH3:
        // TIM_SetCompare3( chan->timer, 0 );
        LL_TIM_OC_SetCompareCH3( chan->timer, 0 );
        break;
      case LL_TIM_CHANNEL_CH4:
        // TIM_SetCompare4( chan->timer, 0 );
        LL_TIM_OC_SetCompareCH4( chan->timer, 0 );
        break;
      }
    }
  }
}
/**
 * @brief  Set the servo update rate (Max 500Hz)
 * @param  speeds array of rates in Hz
 * @param  clock  array of timer clocks in Hz
 * @param  banks  maximum number of banks
 * @retval None
 */
void UVOS_Servo_SetHz( const uint16_t *speeds, const uint32_t *clock, uint8_t banks )
{
  UVOS_Assert( banks <= UVOS_SERVO_BANKS );
  if ( !servo_cfg ) {
    return;
  }

  LL_TIM_InitTypeDef TIM_TimeBaseStructure = servo_cfg->tim_base_init;
  TIM_TimeBaseStructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_TimeBaseStructure.CounterMode   = LL_TIM_COUNTERMODE_UP;

  for ( uint8_t i = 0; i < banks && i < UVOS_SERVO_BANKS; i++ ) {
    const TIM_TypeDef *timer = uvos_servo_bank_timer[ i ];
    if ( timer ) {
      uint32_t new_clock = UVOS_SERVO_TIMER_CLOCK;
      if ( clock[ i ] ) {
        new_clock = clock[ i ];
      }
      // Choose the correct prescaler value for the APB the timer is attached
      if ( timer == TIM1 ||
#if defined (TIM8)
           timer == TIM8 ||
#endif // defined (TIM8)
           timer == TIM9 ||
           timer == TIM10 ||
           timer == TIM11 ) {
        TIM_TimeBaseStructure.Prescaler = ( UVOS_PERIPHERAL_APB2_CLOCK / new_clock ) - 1;
      } else {
        TIM_TimeBaseStructure.Prescaler = ( UVOS_PERIPHERAL_APB1_CLOCK / new_clock ) - 1;
      }
      TIM_TimeBaseStructure.Autoreload = ( ( new_clock / speeds[ i ] ) - 1 );
      // TIM_TimeBaseInit( ( TIM_TypeDef * )timer, &TIM_TimeBaseStructure );
      LL_TIM_Init( ( TIM_TypeDef * )timer, &TIM_TimeBaseStructure );
    }
  }
}

/**
 * Set servo position
 * \param[in] Servo Servo number (0-7)
 * \param[in] Position Servo position in microseconds
 */
void UVOS_Servo_Set( uint8_t servo, uint16_t position )
{
  /* Make sure servo exists */
  if ( !servo_cfg || servo >= servo_cfg->num_channels ) {
    return;
  }


  /* Update the position */
  const struct uvos_tim_channel *chan = &servo_cfg->channels[ servo ];
  uint16_t val    = position;
  uint16_t margin = chan->timer->ARR / 50; // Leave 2% of period as margin to prevent overlaps
  if ( val > ( chan->timer->ARR - margin ) ) {
    val = chan->timer->ARR - margin;
  }

  uint8_t bank = uvos_servo_pin_bank[ servo ];
  if ( uvos_servo_bank_max_pulse[ bank ] < val ) {
    uvos_servo_bank_max_pulse[ bank ] = val;
  }
  switch ( chan->timer_chan ) {
  case LL_TIM_CHANNEL_CH1:
    LL_TIM_OC_SetCompareCH1( chan->timer, val );
    break;
  case LL_TIM_CHANNEL_CH2:
    LL_TIM_OC_SetCompareCH2( chan->timer, val );
    break;
  case LL_TIM_CHANNEL_CH3:
    LL_TIM_OC_SetCompareCH3( chan->timer, val );
    break;
  case LL_TIM_CHANNEL_CH4:
    LL_TIM_OC_SetCompareCH4( chan->timer, val );
    break;
  }
}

uint8_t UVOS_Servo_GetPinBank( uint8_t pin )
{
  if ( pin < servo_cfg->num_channels ) {
    return uvos_servo_pin_bank[ pin ];
  } else {
    return 0;
  }
}

#endif /* UVOS_INCLUDE_SERVO */
