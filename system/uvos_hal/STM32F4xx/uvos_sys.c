#include "uvos.h"

#ifdef UVOS_INCLUDE_SYS

/* Private Function Prototypes */
void NVIC_Configuration( void );
void SysTick_Handler( void );

/* Local Macros */
#define MEM8(addr)  (*((volatile uint8_t *)(addr)))
#define MEM16(addr) (*((volatile uint16_t *)(addr)))
#define MEM32(addr) (*((volatile uint32_t *)(addr)))

/**
 * Initialises all system peripherals
 */
void UVOS_SYS_Init( void )
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_SYSCFG );
  LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_PWR );

  /* Initialise Basic NVIC */
  /* do this early to ensure that we take exceptions in the right place */
  NVIC_Configuration();

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

  /* Configure the system clock */
  SystemClock_Config();

  /* Init the delay system */
  UVOS_DELAY_Init();

  /*
   * Turn on all the peripheral clocks.
   * Micromanaging clocks makes no sense given the power situation in the system, so
   * light up everything we might reasonably use here and just leave it on.
   */
  // LL_AHB1_GRP1_EnableClock(
  //   LL_AHB1_GRP1_PERIPH_GPIOA
  //   | LL_AHB1_GRP1_PERIPH_GPIOB
  //   );

  LL_AHB1_GRP1_EnableClock(
    0
    | LL_AHB1_GRP1_PERIPH_GPIOA
    | LL_AHB1_GRP1_PERIPH_GPIOB
    | LL_AHB1_GRP1_PERIPH_GPIOC
    | LL_AHB1_GRP1_PERIPH_GPIOD
    | LL_AHB1_GRP1_PERIPH_GPIOE
#if defined(GPIOF)
    | LL_AHB1_GRP1_PERIPH_GPIOF
#endif
#if defined(GPIOG)
    | LL_AHB1_GRP1_PERIPH_GPIOG
#endif
#if defined(GPIOH)
    | LL_AHB1_GRP1_PERIPH_GPIOH
#endif
#if defined(GPIOI)
    | LL_AHB1_GRP1_PERIPH_GPIOI
#endif
#if defined(GPIOJ)
    | LL_AHB1_GRP1_PERIPH_GPIOJ
#endif
#if defined(GPIOK)
    | LL_AHB1_GRP1_PERIPH_GPIOK
#endif
#if defined(RCC_AHB1ENR_BKPSRAMEN)
    | LL_AHB1_GRP1_PERIPH_BKPSRAM
#endif
#if defined(RCC_AHB1ENR_CCMDATARAMEN)
    | LL_AHB1_GRP1_PERIPH_CCMDATARAM
#endif
    | LL_AHB1_GRP1_PERIPH_DMA1
    | LL_AHB1_GRP1_PERIPH_DMA2
#if defined(RCC_AHB1ENR_RNGEN)
    // | LL_AHB1_GRP1_PERIPH_RNG          No random number generator
#endif
#if defined(DMA2D)
    // | LL_AHB1_GRP1_PERIPH_DMA2D        No DMA2D graphic Accelerator
#endif
#if defined(ETH)
    // | LL_AHB1_GRP1_PERIPH_ETHMACTX     No ethernet
    // | LL_AHB1_GRP1_PERIPH_ETHMACTX
    // | LL_AHB1_GRP1_PERIPH_ETHMACRX
    // | LL_AHB1_GRP1_PERIPH_ETHMACPTPY
#endif
#if defined(USB_OTG_HS)
    // | LL_AHB1_GRP1_PERIPH_OTGHSULPI    No high-speed USB (requires external PHY)
    // | LL_AHB1_GRP1_PERIPH_OTGHSULPI
#endif
  );

  LL_AHB2_GRP1_EnableClock(
    0
    // | LL_AHB2_GRP1_PERIPH_DCMI         No camera   @todo basic vision support?
    // | LL_AHB2_GRP1_PERIPH_CRYP         No crypto
    // | LL_AHB2_GRP1_PERIPH_AES          No AES algorithm accelerator
    // | LL_AHB2_GRP1_PERIPH_HASH         No hash generator
    // | LL_AHB2_GRP1_PERIPH_RNG          No random numbers
    // | LL_AHB2_GRP1_PERIPH_OTGFS
  );

#if defined(RCC_AHB3_SUPPORT)
  LL_AHB3_GRP1_EnableClock(
    0
    // | LL_AHB3_GRP1_PERIPH_FMC          flexible memory controller
    // | LL_AHB3_GRP1_PERIPH_FSMC         No external static memory
    // | LL_AHB3_GRP1_PERIPH_QSPI         QuadSPI interface
  );
#endif /* RCC_AHB3_SUPPORT */

  LL_APB1_GRP1_EnableClock(
    0
#if defined(TIM2)
    | LL_APB1_GRP1_PERIPH_TIM2
#endif
#if defined(TIM3)
    | LL_APB1_GRP1_PERIPH_TIM3
#endif
#if defined(TIM4)
    | LL_APB1_GRP1_PERIPH_TIM4
    | LL_APB1_GRP1_PERIPH_TIM5
#endif
#if defined(TIM6)
    | LL_APB1_GRP1_PERIPH_TIM6
#endif
#if defined(TIM7)
    | LL_APB1_GRP1_PERIPH_TIM7
#endif
#if defined(TIM12)
    | LL_APB1_GRP1_PERIPH_TIM12
#endif
#if defined(TIM13)
    | LL_APB1_GRP1_PERIPH_TIM13
#endif
#if defined(TIM14)
    | LL_APB1_GRP1_PERIPH_TIM14
#endif
    | LL_APB1_GRP1_PERIPH_WWDG
    | LL_APB1_GRP1_PERIPH_SPI2
    | LL_APB1_GRP1_PERIPH_SPI3
    | LL_APB1_GRP1_PERIPH_USART2
#if defined(USART3)
    | LL_APB1_GRP1_PERIPH_USART3
#endif
#if defined(UART4)
    | LL_APB1_GRP1_PERIPH_UART4
#endif
#if defined(UART5)
    | LL_APB1_GRP1_PERIPH_UART5
#endif
    | LL_APB1_GRP1_PERIPH_I2C1
    | LL_APB1_GRP1_PERIPH_I2C2
#if defined(I2C3)
    | LL_APB1_GRP1_PERIPH_I2C3
#endif
#if defined(CAN1)
    | LL_APB1_GRP1_PERIPH_CAN1
#endif
#if defined(CAN2)
    | LL_APB1_GRP1_PERIPH_CAN2
#endif
    | LL_APB1_GRP1_PERIPH_PWR
#if defined(DAC1)
    | LL_APB1_GRP1_PERIPH_DAC1
#endif
  );

  LL_APB2_GRP1_EnableClock(
    0
    | LL_APB2_GRP1_PERIPH_TIM1
#if defined(TIM8)
    | LL_APB2_GRP1_PERIPH_TIM8
#endif
    | LL_APB2_GRP1_PERIPH_USART1
#if defined(USART6)
    | LL_APB2_GRP1_PERIPH_USART6
#endif
    | LL_APB2_GRP1_PERIPH_ADC1
#if defined(ADC2)
    | LL_APB2_GRP1_PERIPH_ADC2
#endif
#if defined(ADC3)
    | LL_APB2_GRP1_PERIPH_ADC3
#endif
#if defined(SDIO)
    | LL_APB2_GRP1_PERIPH_SDIO
#endif
    | LL_APB2_GRP1_PERIPH_SPI1
    | LL_APB2_GRP1_PERIPH_SYSCFG
    | LL_APB2_GRP1_PERIPH_TIM9
#if defined(TIM10)
    | LL_APB2_GRP1_PERIPH_TIM10
#endif
#if defined(TIM11)
    | LL_APB2_GRP1_PERIPH_TIM11
#endif
  );

  /*
   * Configure all pins as input / pullup to avoid issues with
   * uncommitted pins, excepting special-function pins that we need to
   * remain as-is.
   */
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  LL_GPIO_StructInit( &GPIO_InitStructure );
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP; // default is un-pulled input

  GPIO_InitStructure.Pin  = LL_GPIO_PIN_ALL;
#if (UVOS_USB_ENABLED)
  GPIO_InitStructure.Pin &= ~( LL_GPIO_PIN_11 | LL_GPIO_PIN_12 ); // leave USB D+/D- alone
#endif
  GPIO_InitStructure.Pin &= ~( LL_GPIO_PIN_13 | LL_GPIO_PIN_14 ); // leave JTAG pins alone
  LL_GPIO_Init( GPIOA, &GPIO_InitStructure );

  GPIO_InitStructure.Pin  = LL_GPIO_PIN_ALL;
  LL_GPIO_Init( GPIOB, &GPIO_InitStructure );

  GPIO_InitStructure.Pin  = LL_GPIO_PIN_ALL;
  LL_GPIO_Init( GPIOC, &GPIO_InitStructure );
  LL_GPIO_Init( GPIOD, &GPIO_InitStructure );
  LL_GPIO_Init( GPIOE, &GPIO_InitStructure );
#if defined(GPIOF)
  LL_GPIO_Init( GPIOF, &GPIO_InitStructure );
#endif
#if defined(GPIOG)
  LL_GPIO_Init( GPIOG, &GPIO_InitStructure );
#endif
#if defined(GPIOH)
  LL_GPIO_Init( GPIOH, &GPIO_InitStructure );
#endif
#if defined(GPIOI)
  LL_GPIO_Init( GPIOI, &GPIO_InitStructure );
#endif

}

/**
 * Returns the CPU's flash size (in bytes)
 */
uint32_t UVOS_SYS_getCPUFlashSize( void )
{
  return ( uint32_t )MEM16( DEVICE_FLASHSIZE_ADDR ) * 1024; // it might be possible to locate in the OTP area, but haven't looked and not documented
}

/**
 * Returns the serial number as a string
 * param[out] str pointer to a string which can store at least 32 digits + zero terminator!
 * (24 digits returned for STM32)
 * return < 0 if feature not supported
 */
int32_t UVOS_SYS_SerialNumberGetBinary( uint8_t * array )
{
  int i;

  /* Stored in the so called "electronic signature" */
  for ( i = 0; i < UVOS_SYS_SERIAL_NUM_BINARY_LEN; ++i ) {
    uint8_t b = MEM8( DEVICE_SIGNATURE_ADDR + i );

    array[i] = b;
  }

  /* No error */
  return 0;
}

/**
 * Returns the serial number as a string
 * param[out] str pointer to a string which can store at least 32 digits + zero terminator!
 * (24 digits returned for STM32)
 * return < 0 if feature not supported
 */
int32_t UVOS_SYS_SerialNumberGet( char * str )
{
  int i;

  /* Stored in the so called "electronic signature" */
  for ( i = 0; i < UVOS_SYS_SERIAL_NUM_ASCII_LEN; ++i ) {
    uint8_t b = MEM8( DEVICE_SIGNATURE_ADDR + ( i / 2 ) );
    if ( !( i & 1 ) ) {
      b >>= 4;
    }
    b &= 0x0f;

    str[i] = ( ( b > 9 ) ? ( 'A' - 10 ) : '0' ) + b;
  }
  str[i] = '\0';

  /* No error */
  return 0;
}

/**
 * Configures Vector Table base location and SysTick
 */
void NVIC_Configuration( void )
{
  /* Set the Vector Table base address as specified in .ld file */
  // extern void *uvos_isr_vector_table_base;

  // NVIC_SetVectorTable( ( uint32_t )&uvos_isr_vector_table_base, 0x0 );

  /* 4 bits for Interrupt priorities so no sub priorities */
  // NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

  /* 4 bits for Interrupt priorities so no sub priorities */
  NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );
  // NVIC_SetPriorityGrouping( 0 );

  /* Configure HCLK clock as SysTick clock source. */
  // SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
  LL_SYSTICK_SetClkSource( LL_SYSTICK_CLKSOURCE_HCLK );

}

#endif /* UVOS_INCLUDE_SYS */