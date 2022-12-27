#include <uvos.h>

#ifdef UVOS_INCLUDE_EXTI

#define EXTI_MAX_LINES 16

static uvos_exti_vector_t uvos_exti_vector[ EXTI_MAX_LINES ];

static uint8_t UVOS_EXTI_line_to_index( uint32_t line )
{
  switch ( line ) {
  case LL_EXTI_LINE_0: return 0;

  case LL_EXTI_LINE_1: return 1;

  case LL_EXTI_LINE_2: return 2;

  case LL_EXTI_LINE_3: return 3;

  case LL_EXTI_LINE_4: return 4;

  case LL_EXTI_LINE_5: return 5;

  case LL_EXTI_LINE_6: return 6;

  case LL_EXTI_LINE_7: return 7;

  case LL_EXTI_LINE_8: return 8;

  case LL_EXTI_LINE_9: return 9;

  case LL_EXTI_LINE_10: return 10;

  case LL_EXTI_LINE_11: return 11;

  case LL_EXTI_LINE_12: return 12;

  case LL_EXTI_LINE_13: return 13;

  case LL_EXTI_LINE_14: return 14;

  case LL_EXTI_LINE_15: return 15;
  }

  UVOS_Assert( 0 );
  return 0xFF;
}

uint32_t UVOS_EXTI_gpio_port_to_exti_source_port( GPIO_TypeDef * gpio_port )
{
  switch ( ( uint32_t )gpio_port ) {
  case ( uint32_t )GPIOA:
    return LL_SYSCFG_EXTI_PORTA;
    break;
  case ( uint32_t )GPIOB:
    return LL_SYSCFG_EXTI_PORTB;
    break;
  case ( uint32_t )GPIOC:
    return LL_SYSCFG_EXTI_PORTC;
    break;
  case ( uint32_t )GPIOD:
    return LL_SYSCFG_EXTI_PORTD;
    break;
  case ( uint32_t )GPIOE:
    return LL_SYSCFG_EXTI_PORTE;
    break;
#if defined(GPIOF)
  case ( uint32_t )GPIOF:
    return LL_SYSCFG_EXTI_PORTF;
    break;
#endif
#if defined(GPIOG)
  case ( uint32_t )GPIOG:
    return LL_SYSCFG_EXTI_PORTG;
    break;
#endif
  default:
    UVOS_Assert( 0 );
    return 0xFF;
  }

  UVOS_Assert( 0 );
  return 0xFF;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

uint32_t UVOS_EXTI_gpio_pin_to_exti_source_pin( uint32_t gpio_pin )
{
  switch ( ( uint32_t )gpio_pin ) {
  case LL_GPIO_PIN_0:
    return LL_SYSCFG_EXTI_LINE0;
    break;
  case LL_GPIO_PIN_1:
    return LL_SYSCFG_EXTI_LINE1;
    break;
  case LL_GPIO_PIN_2:
    return LL_SYSCFG_EXTI_LINE2;
    break;
  case LL_GPIO_PIN_3:
    return LL_SYSCFG_EXTI_LINE3;
    break;
  case LL_GPIO_PIN_4:
    return LL_SYSCFG_EXTI_LINE4;
    break;
  case LL_GPIO_PIN_5:
    return LL_SYSCFG_EXTI_LINE5;
    break;
  case LL_GPIO_PIN_6:
    return LL_SYSCFG_EXTI_LINE6;
    break;
  case LL_GPIO_PIN_7:
    return LL_SYSCFG_EXTI_LINE7;
    break;
  case LL_GPIO_PIN_8:
    return LL_SYSCFG_EXTI_LINE8;
    break;
  case LL_GPIO_PIN_9:
    return LL_SYSCFG_EXTI_LINE9;
    break;
  case LL_GPIO_PIN_10:
    return LL_SYSCFG_EXTI_LINE10;
    break;
  case LL_GPIO_PIN_11:
    return LL_SYSCFG_EXTI_LINE11;
    break;
  case LL_GPIO_PIN_12:
    return LL_SYSCFG_EXTI_LINE12;
    break;
  case LL_GPIO_PIN_13:
    return LL_SYSCFG_EXTI_LINE13;
    break;
  case LL_GPIO_PIN_14:
    return LL_SYSCFG_EXTI_LINE14;
    break;
  case LL_GPIO_PIN_15:
    return LL_SYSCFG_EXTI_LINE15;
    break;
  default:
    UVOS_Assert( 0 );
    return 0;
  }

  UVOS_Assert( 0 );
  return 0;
}

#pragma GCC pop_options

int32_t UVOS_EXTI_Init( const struct uvos_exti_cfg * cfg )
{
  UVOS_Assert( cfg );

  /* Connect this config to the requested vector */
  uint8_t line_index = UVOS_EXTI_line_to_index( cfg->line );

  if ( uvos_exti_vector[ line_index ] ) {
    /* Someone else already has this mapped */
    return -1;
  }

  /* Bind the vector to the exti line */
  uvos_exti_vector[ line_index ] = cfg->vector;

  /* Initialize the GPIO pin */
  LL_GPIO_Init( cfg->pin.gpio, ( LL_GPIO_InitTypeDef * ) & cfg->pin.init );

  /* Set up the EXTI interrupt source */
  uint32_t exti_source_port = UVOS_EXTI_gpio_port_to_exti_source_port( cfg->pin.gpio );
  /* Following is not entirely correct! There is cfg->pin.pin_source to serve this purpose, and GPIO_Pin can also contain more than one bit set */
  uint32_t exti_source_pin  = UVOS_EXTI_gpio_pin_to_exti_source_pin( cfg->pin.init.Pin );
  // SYSCFG_EXTILineConfig( exti_source_port, exti_source_pin );
  // __STATIC_INLINE void LL_SYSCFG_SetEXTISource( uint32_t Port, uint32_t Line )
  LL_SYSCFG_SetEXTISource( exti_source_port, exti_source_pin );

  LL_EXTI_Init( ( LL_EXTI_InitTypeDef * ) & cfg->exti.init );

  /* Enable the interrupt channel */
  NVIC_Init( ( NVIC_InitTypeDef * ) & cfg->irq.init );

  return 0;
}

int32_t UVOS_EXTI_DeInit( const struct uvos_exti_cfg * cfg )
{
  uint8_t line_index = UVOS_EXTI_line_to_index( cfg->line );

  if ( uvos_exti_vector[line_index] == cfg->vector ) {
    LL_EXTI_InitTypeDef disable = cfg->exti.init;
    disable.LineCommand = DISABLE;

    LL_EXTI_Init( &disable );
    uvos_exti_vector[line_index] = 0;

    return 0;
  }

  return -1;
}

static bool UVOS_EXTI_generic_irq_handler( uint8_t line_index )
{
  if ( uvos_exti_vector[ line_index ] ) {
    return uvos_exti_vector[ line_index ]();
  }

  /* Unconfigured interrupt just fired! */
  return false;
}

/* Bind Interrupt Handlers */

#ifdef UVOS_INCLUDE_FREERTOS
#define UVOS_EXTI_HANDLE_LINE( line, woken )                      \
  if ( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_##line ) != RESET ) {       \
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_##line );        \
    woken = UVOS_EXTI_generic_irq_handler( line ) ? pdTRUE : woken; \
  }
#else
#define UVOS_EXTI_HANDLE_LINE( line, woken )                      \
  if ( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_##line ) != RESET ) {       \
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_##line );        \
    UVOS_EXTI_generic_irq_handler( line );            \
  }
#endif

static void UVOS_EXTI_0_irq_handler( void )
{
#ifdef UVOS_INCLUDE_FREERTOS
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#else
  bool __attribute__( ( unused ) ) xHigherPriorityTaskWoken;
#endif
  UVOS_EXTI_HANDLE_LINE( 0, xHigherPriorityTaskWoken );
#ifdef UVOS_INCLUDE_FREERTOS
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
#endif
}
void EXTI0_IRQHandler( void ) __attribute__( ( alias( "UVOS_EXTI_0_irq_handler" ) ) );

static void UVOS_EXTI_1_irq_handler( void )
{
#ifdef UVOS_INCLUDE_FREERTOS
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#else
  bool __attribute__( ( unused ) ) xHigherPriorityTaskWoken;
#endif
  UVOS_EXTI_HANDLE_LINE( 1, xHigherPriorityTaskWoken );
#ifdef UVOS_INCLUDE_FREERTOS
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
#endif
}
void EXTI1_IRQHandler( void ) __attribute__( ( alias( "UVOS_EXTI_1_irq_handler" ) ) );

static void UVOS_EXTI_2_irq_handler( void )
{
#ifdef UVOS_INCLUDE_FREERTOS
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#else
  bool __attribute__( ( unused ) ) xHigherPriorityTaskWoken;
#endif
  UVOS_EXTI_HANDLE_LINE( 2, xHigherPriorityTaskWoken );
#ifdef UVOS_INCLUDE_FREERTOS
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
#endif
}
void EXTI2_IRQHandler( void ) __attribute__( ( alias( "UVOS_EXTI_2_irq_handler" ) ) );

static void UVOS_EXTI_3_irq_handler( void )
{
#ifdef UVOS_INCLUDE_FREERTOS
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#else
  bool __attribute__( ( unused ) ) xHigherPriorityTaskWoken;
#endif
  UVOS_EXTI_HANDLE_LINE( 3, xHigherPriorityTaskWoken );
#ifdef UVOS_INCLUDE_FREERTOS
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
#endif
}
void EXTI3_IRQHandler( void ) __attribute__( ( alias( "UVOS_EXTI_3_irq_handler" ) ) );

static void UVOS_EXTI_4_irq_handler( void )
{
#ifdef UVOS_INCLUDE_FREERTOS
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#else
  bool __attribute__( ( unused ) ) xHigherPriorityTaskWoken;
#endif
  UVOS_EXTI_HANDLE_LINE( 4, xHigherPriorityTaskWoken );
#ifdef UVOS_INCLUDE_FREERTOS
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
#endif
}
void EXTI4_IRQHandler( void ) __attribute__( ( alias( "UVOS_EXTI_4_irq_handler" ) ) );

static void UVOS_EXTI_9_5_irq_handler( void )
{
#ifdef UVOS_INCLUDE_FREERTOS
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#else
  bool __attribute__( ( unused ) ) xHigherPriorityTaskWoken;
#endif
  UVOS_EXTI_HANDLE_LINE( 5, xHigherPriorityTaskWoken );
  UVOS_EXTI_HANDLE_LINE( 6, xHigherPriorityTaskWoken );
  UVOS_EXTI_HANDLE_LINE( 7, xHigherPriorityTaskWoken );
  UVOS_EXTI_HANDLE_LINE( 8, xHigherPriorityTaskWoken );
  UVOS_EXTI_HANDLE_LINE( 9, xHigherPriorityTaskWoken );
#ifdef UVOS_INCLUDE_FREERTOS
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
#endif
}
void EXTI9_5_IRQHandler( void ) __attribute__( ( alias( "UVOS_EXTI_9_5_irq_handler" ) ) );

static void UVOS_EXTI_15_10_irq_handler( void )
{
#ifdef UVOS_INCLUDE_FREERTOS
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#else
  bool __attribute__( ( unused ) ) xHigherPriorityTaskWoken;
#endif
  UVOS_EXTI_HANDLE_LINE( 10, xHigherPriorityTaskWoken );
  UVOS_EXTI_HANDLE_LINE( 11, xHigherPriorityTaskWoken );
  UVOS_EXTI_HANDLE_LINE( 12, xHigherPriorityTaskWoken );
  UVOS_EXTI_HANDLE_LINE( 13, xHigherPriorityTaskWoken );
  UVOS_EXTI_HANDLE_LINE( 14, xHigherPriorityTaskWoken );
  UVOS_EXTI_HANDLE_LINE( 15, xHigherPriorityTaskWoken );
#ifdef UVOS_INCLUDE_FREERTOS
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
#endif
}
void EXTI15_10_IRQHandler( void ) __attribute__( ( alias( "UVOS_EXTI_15_10_irq_handler" ) ) );

#endif /* UVOS_INCLUDE_EXTI */

/**
 * @}
 * @}
 */
