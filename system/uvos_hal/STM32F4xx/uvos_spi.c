#include <uvos.h>

#ifdef UVOS_INCLUDE_SPI

#include "uvos_spi_priv.h"

#define SPI_MAX_BLOCK_PIO 128

static bool UVOS_SPI_validate( __attribute__( ( unused ) ) struct uvos_spi_dev * com_dev )
{
  /* Should check device magic here */
  return true;
}

#if defined(UVOS_INCLUDE_FREERTOS)
static struct uvos_spi_dev * UVOS_SPI_alloc( void )
{
  return UVOS_malloc( sizeof( struct uvos_spi_dev ) );
}
#else
static struct uvos_spi_dev uvos_spi_devs[ UVOS_SPI_MAX_DEVS ];
static uint8_t uvos_spi_num_devs;

/* Module level allocate function, allocates instance from static uvos_spi_devs[] pool */
static struct uvos_spi_dev * UVOS_SPI_alloc( void )
{
  if ( uvos_spi_num_devs >= UVOS_SPI_MAX_DEVS ) {
    return NULL;
  }

  return &uvos_spi_devs[ uvos_spi_num_devs++ ];
}
#endif

// STM32 SPL compatibility ------------------------------------>>>

/**
  * @brief  Configures internally by software the NSS pin for the selected SPI.
  * @param  SPIx: where x can be 1, 2, 3, 4, 5 or 6 to select the SPI peripheral.
  * @param  SPI_NSSInternalSoft: specifies the SPI NSS internal state.
  *          This parameter can be one of the following values:
  *            @arg SPI_NSSInternalSoft_Set: Set NSS pin internally
  *            @arg SPI_NSSInternalSoft_Reset: Reset NSS pin internally
  * @retval None
  */
void SPI_NSSInternalSoftwareConfig( SPI_TypeDef * SPIx, uint16_t SPI_NSSInternalSoft )
{
  /* Check the parameters */
  // assert_param(IS_SPI_ALL_PERIPH(SPIx));
  // assert_param(IS_SPI_NSS_INTERNAL(SPI_NSSInternalSoft));
  if ( SPI_NSSInternalSoft != SPI_NSSInternalSoft_Reset ) {
    /* Set NSS pin internally by software */
    SPIx->CR1 |= SPI_NSSInternalSoft_Set;
  } else {
    /* Reset NSS pin internally by software */
    SPIx->CR1 &= SPI_NSSInternalSoft_Reset;
  }
}

/**
  * @brief  Enables or disables the SS output for the selected SPI.
  * @param  SPIx: where x can be 1, 2, 3, 4, 5 or 6 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx SS output.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPI_SSOutputCmd( SPI_TypeDef * SPIx, FunctionalState NewState )
{
  /* Check the parameters */
  // assert_param(IS_SPI_ALL_PERIPH(SPIx));
  // assert_param(IS_FUNCTIONAL_STATE(NewState));
  if ( NewState != DISABLE ) {
    /* Enable the selected SPI SS output */
    SPIx->CR2 |= ( uint16_t )SPI_CR2_SSOE;
  } else {
    /* Disable the selected SPI SS output */
    SPIx->CR2 &= ( uint16_t )~( ( uint16_t )SPI_CR2_SSOE );
  }
}

// STM32 SPL compatibility ------------------------------------<<<

/**
 * Initialises SPI pins
 * \param[in] mode currently only mode 0 supported
 * \return < 0 if initialisation failed
 */
int32_t UVOS_SPI_Init( uint32_t * spi_id, const struct uvos_spi_cfg * cfg )
{
  uint32_t init_ssel = 0;

  UVOS_Assert( spi_id );
  UVOS_Assert( cfg );

  struct uvos_spi_dev * spi_dev;

  spi_dev = ( struct uvos_spi_dev * )UVOS_SPI_alloc();
  if ( !spi_dev ) {
    goto out_fail;
  }

  /* Bind the configuration to the device instance */
  spi_dev->cfg = cfg;

#if defined(UVOS_INCLUDE_FREERTOS)
  vSemaphoreCreateBinary( spi_dev->busy );
  xSemaphoreGive( spi_dev->busy );
#endif

  /* Disable callback function */
  spi_dev->callback = NULL;

  /* Set rx/tx dummy bytes to a known value */
  spi_dev->rx_dummy_byte = 0xFF;
  spi_dev->tx_dummy_byte = 0xFF;

  switch ( spi_dev->cfg->init.NSS ) {
  case LL_SPI_NSS_SOFT:
    if ( spi_dev->cfg->init.Mode == LL_SPI_MODE_MASTER ) {
      /* We're a master in soft NSS mode, make sure we see NSS high at all times. */
      SPI_NSSInternalSoftwareConfig( spi_dev->cfg->regs, SPI_NSSInternalSoft_Set );
      /* Init as many slave selects as the config advertises. */
      init_ssel = spi_dev->cfg->slave_count;
    } else {
      /* We're a slave in soft NSS mode, make sure we see NSS low at all times. */
      SPI_NSSInternalSoftwareConfig( spi_dev->cfg->regs, SPI_NSSInternalSoft_Reset );
    }
    break;

  case LL_SPI_NSS_HARD_OUTPUT:
    /* only legal for single-slave config */
    UVOS_Assert( spi_dev->cfg->slave_count == 1 );
    init_ssel = 1;
    SPI_SSOutputCmd( spi_dev->cfg->regs, ( spi_dev->cfg->init.Mode == LL_SPI_NSS_HARD_OUTPUT ) ? ENABLE : DISABLE );
    break;

  default:
    UVOS_Assert( 0 );
  }

  /* Initialize the GPIO pins */
  /* note __builtin_ctz() due to the difference between GPIO_PinX and GPIO_PinSourceX */
  /* Initialize the SPI related GPIO pins */
  LL_GPIO_Init( spi_dev->cfg->sclk.gpio, ( LL_GPIO_InitTypeDef * ) & ( spi_dev->cfg->sclk.init ) );
  LL_GPIO_Init( spi_dev->cfg->mosi.gpio, ( LL_GPIO_InitTypeDef * ) & ( spi_dev->cfg->mosi.init ) );
  LL_GPIO_Init( spi_dev->cfg->miso.gpio, ( LL_GPIO_InitTypeDef * ) & ( spi_dev->cfg->miso.init ) );

  if ( spi_dev->cfg->init.NSS == LL_SPI_NSS_SOFT ) {
    for ( uint32_t i = 0; i < init_ssel; i++ ) {
      /* Since we're driving the SSEL pin in software, ensure that the slave is deselected */
      /* XXX multi-slave support - maybe have another SPI_NSS_ mode? */
      LL_GPIO_SetOutputPin( spi_dev->cfg->ssel[i].gpio, spi_dev->cfg->ssel[i].init.Pin );
      LL_GPIO_Init( spi_dev->cfg->ssel[i].gpio,  ( LL_GPIO_InitTypeDef * ) & ( spi_dev->cfg->ssel[i].init ) );
    }
  }

  /* Save DMA Rx/Tx instances */
  DMA_TypeDef * DMAx_rx = spi_dev->cfg->dma.rx.DMAx;
  DMA_TypeDef * DMAx_tx = spi_dev->cfg->dma.tx.DMAx;

  /* Configure DMA for SPI Rx */
  LL_DMA_DeInit( DMAx_rx, spi_dev->cfg->dma.rx.stream );
  LL_DMA_DisableStream( DMAx_rx, spi_dev->cfg->dma.rx.stream );
  LL_DMA_Init( DMAx_rx, spi_dev->cfg->dma.rx.stream, ( LL_DMA_InitTypeDef * ) & ( spi_dev->cfg->dma.rx.init ) );

  /* Configure DMA for SPI Tx */
  LL_DMA_DeInit( DMAx_tx, spi_dev->cfg->dma.tx.stream );
  LL_DMA_DisableStream( DMAx_tx, spi_dev->cfg->dma.tx.stream );
  LL_DMA_Init( DMAx_tx, spi_dev->cfg->dma.tx.stream, ( LL_DMA_InitTypeDef * ) & ( spi_dev->cfg->dma.tx.init ) );

  /* Initialize the SPI block */
  LL_SPI_DeInit( spi_dev->cfg->regs );
  LL_SPI_Init( spi_dev->cfg->regs, ( LL_SPI_InitTypeDef * ) & ( spi_dev->cfg->init ) );

  /* Configure CRC calculation */
  if ( spi_dev->cfg->use_crc ) {
    LL_SPI_EnableCRC( spi_dev->cfg->regs );
  } else {
    LL_SPI_DisableCRC( spi_dev->cfg->regs );
  }

  /* Enable SPI */
  LL_SPI_Enable( spi_dev->cfg->regs );

  /* Enable SPI interrupts to DMA */
  LL_SPI_EnableDMAReq_TX( spi_dev->cfg->regs );
  LL_SPI_EnableDMAReq_RX( spi_dev->cfg->regs );

  /* Must store this before enabling interrupt */
  *spi_id = ( uint32_t )spi_dev;

  /* Configure DMA interrupt */
  NVIC_Init( ( NVIC_InitTypeDef * ) & ( spi_dev->cfg->dma.irq.init ) );
  // NVIC_SetPriority( spi_dev->cfg->dma.irq.init.NVIC_IRQChannel,
  //                   NVIC_EncodePriority( NVIC_GetPriorityGrouping(),
  //                                        spi_dev->cfg->dma.irq.init.NVIC_IRQChannelPreemptionPriority,
  //                                        spi_dev->cfg->dma.irq.init.NVIC_IRQChannelSubPriority ) );
  // if ( spi_dev->cfg->dma.irq.init.NVIC_IRQChannelCmd != DISABLE ) {
  //   NVIC_EnableIRQ( spi_dev->cfg->dma.irq.init.NVIC_IRQChannel );
  // } else {
  //   NVIC_DisableIRQ( spi_dev->cfg->dma.irq.init.NVIC_IRQChannel );
  // }

  return 0;

out_fail:
  return -1;
}

/**
 * (Re-)initialises SPI peripheral clock rate
 *
 * \param[in] spi SPI number (0 or 1)
 * \param[in] spi_prescaler configures the SPI speed:
 * <UL>
 *   <LI>UVOS_SPI_PRESCALER_2: sets clock rate 27.7~ nS @ 72 MHz (36 MBit/s) (only supported for spi==0, spi1 uses 4 instead)
 *   <LI>UVOS_SPI_PRESCALER_4: sets clock rate 55.5~ nS @ 72 MHz (18 MBit/s)
 *   <LI>UVOS_SPI_PRESCALER_8: sets clock rate 111.1~ nS @ 72 MHz (9 MBit/s)
 *   <LI>UVOS_SPI_PRESCALER_16: sets clock rate 222.2~ nS @ 72 MHz (4.5 MBit/s)
 *   <LI>UVOS_SPI_PRESCALER_32: sets clock rate 444.4~ nS @ 72 MHz (2.25 MBit/s)
 *   <LI>UVOS_SPI_PRESCALER_64: sets clock rate 888.8~ nS @ 72 MHz (1.125 MBit/s)
 *   <LI>UVOS_SPI_PRESCALER_128: sets clock rate 1.7~ nS @ 72 MHz (0.562 MBit/s)
 *   <LI>UVOS_SPI_PRESCALER_256: sets clock rate 3.5~ nS @ 72 MHz (0.281 MBit/s)
 * </UL>
 * \return 0 if no error
 * \return -1 if disabled SPI port selected
 * \return -3 if invalid spi_prescaler selected
 */
int32_t UVOS_SPI_SetClockSpeed( uint32_t spi_id, SPIPrescalerTypeDef spi_prescaler )
{
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;

  bool valid = UVOS_SPI_validate( spi_dev );

  UVOS_Assert( valid )

  /* Validate selected prescaler */
  if ( spi_prescaler >= 8 ) {
    return -3;
  }

  /* Adjust the prescaler for the peripheral's clock */
  uint32_t BaudRate = ( ( uint32_t )spi_prescaler & 7 ) << 3;

  /* Write the new prescaler value */
  LL_SPI_SetBaudRatePrescaler( spi_dev->cfg->regs, BaudRate );

  UVOS_SPI_TransferByte( spi_id, 0xFF );
  return 0;
}

/**
 * Claim the SPI bus semaphore.  Calling the SPI functions does not require this
 * \param[in] spi SPI number (0 or 1)
 * \return 0 if no error
 * \return -1 if timeout before claiming semaphore
 */
int32_t UVOS_SPI_ClaimBus( uint32_t spi_id )
{
#if defined(UVOS_INCLUDE_FREERTOS)
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;

  bool valid = UVOS_SPI_validate( spi_dev );
  UVOS_Assert( valid )

  if ( xSemaphoreTake( spi_dev->busy, 0xffff ) != pdTRUE ) {
    return -1;
  }
#else
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;
  uint32_t timeout = 0xffff;
  while ( ( UVOS_SPI_Busy( spi_id ) || spi_dev->busy ) && --timeout ) {
    ;
  }
  if ( timeout == 0 ) { // timed out
    return -1;
  }

  UVOS_IRQ_Disable();
  if ( spi_dev->busy ) {
    return -1;
  }
  spi_dev->busy = 1;
  UVOS_IRQ_Enable();
#endif /* if defined(UVOS_INCLUDE_FREERTOS) */
  return 0;
}

/**
 * Claim the SPI bus semaphore from an ISR.  Has no timeout.
 * \param[in] spi SPI number (0 or 1)
 * \param woken[in,out] If non-NULL, will be set to true if woken was false and a higher priority
 *                      task has is now eligible to run, else unchanged
 * \return 0 if no error
 * \return -1 if timeout before claiming semaphore
 */
int32_t UVOS_SPI_ClaimBusISR( uint32_t spi_id, bool * woken )
{
#if defined(UVOS_INCLUDE_FREERTOS)
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;
  signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

  bool valid = UVOS_SPI_validate( spi_dev );
  UVOS_Assert( valid )

  if ( xSemaphoreTakeFromISR( spi_dev->busy, &higherPriorityTaskWoken ) != pdTRUE ) {
    return -1;
  }
  if ( woken ) {
    *woken = *woken || ( higherPriorityTaskWoken == pdTRUE );
  }
  return 0;

#else
  if ( woken ) {
    *woken = false;
  }
  return UVOS_SPI_ClaimBus( spi_id );

#endif
}

/**
 * Release the SPI bus semaphore.  Calling the SPI functions does not require this
 * \param[in] spi SPI number (0 or 1)
 * \return 0 if no error
 */
int32_t UVOS_SPI_ReleaseBus( uint32_t spi_id )
{
#if defined(UVOS_INCLUDE_FREERTOS)
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;

  bool valid = UVOS_SPI_validate( spi_dev );
  UVOS_Assert( valid )

  xSemaphoreGive( spi_dev->busy );
#else
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;
  UVOS_IRQ_Disable();
  spi_dev->busy = 0;
  UVOS_IRQ_Enable();
#endif
  return 0;
}

/**
 * Release the SPI bus semaphore from ISR.  Calling the SPI functions does not require this
 * \param[in] spi SPI number (0 or 1)
 * \param woken[in,out] If non-NULL, will be set to true if woken was false and a higher priority
 *                      task has is now eligible to run, else unchanged
 * \return 0 if no error
 */
int32_t UVOS_SPI_ReleaseBusISR( uint32_t spi_id, bool * woken )
{
#if defined(UVOS_INCLUDE_FREERTOS)
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;
  signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

  bool valid = UVOS_SPI_validate( spi_dev );
  UVOS_Assert( valid )

  xSemaphoreGiveFromISR( spi_dev->busy, &higherPriorityTaskWoken );
  if ( woken ) {
    *woken = *woken || ( higherPriorityTaskWoken == pdTRUE );
  }
  return 0;

#else
  if ( woken ) {
    *woken = false;
  }
  return UVOS_SPI_ReleaseBus( spi_id );

#endif
}

/**
 * Controls the RC (Register Clock alias Chip Select) pin of a SPI port
 * \param[in] spi SPI number (0 or 1)
 * \param[in] pin_value 0 or 1
 * \return 0 if no error
 */
int32_t UVOS_SPI_RC_PinSet( uint32_t spi_id, uint32_t slave_id, uint8_t pin_value )
{
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;

  bool valid = UVOS_SPI_validate( spi_dev );

  UVOS_Assert( valid )
  UVOS_Assert( slave_id <= spi_dev->cfg->slave_count )

  /* XXX multi-slave support? */
  if ( pin_value ) {
    LL_GPIO_SetOutputPin( spi_dev->cfg->ssel[slave_id].gpio, spi_dev->cfg->ssel[slave_id].init.Pin );
  } else {
    LL_GPIO_ResetOutputPin( spi_dev->cfg->ssel[slave_id].gpio, spi_dev->cfg->ssel[slave_id].init.Pin );
  }

  return 0;
}

/**
 * Transfers a byte to SPI output and reads back the return value from SPI input
 * \param[in] spi SPI number (0 or 1)
 * \param[in] b the byte which should be transfered
 */
int32_t UVOS_SPI_TransferByte( uint32_t spi_id, uint8_t b )
{
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;

  bool valid = UVOS_SPI_validate( spi_dev );

  UVOS_Assert( valid )

// uint8_t dummy;
  uint8_t rx_byte;

  /*
   * Procedure taken from STM32F40xxx Reference Manual section
   * 25.3.5 'Data transmission and reception procedures'
   */

  /* Make sure the RXNE flag is cleared by reading the DR register */
  /*dummy =*/ ( void )spi_dev->cfg->regs->DR;

  /* Start the transfer */
  spi_dev->cfg->regs->DR = b;

  /* Wait until there is a byte to read */
  while ( !( spi_dev->cfg->regs->SR & SPI_SR_RXNE ) );

  /* Read the rx'd byte */
  rx_byte = spi_dev->cfg->regs->DR;

  /* Wait until the TXE goes high */
  while ( !( spi_dev->cfg->regs->SR & SPI_SR_TXE ) );

  /* Wait for SPI transfer to have fully completed */
  while ( spi_dev->cfg->regs->SR & SPI_SR_BSY );

  /* Return received byte */
  return rx_byte;
}

/**
 * Transfers a block of bytes via DMA.
 * \param[in] spi SPI number (0 or 1)
 * \param[in] send_buffer pointer to buffer which should be sent.<BR>
 * If NULL, 0xff (all-one) will be sent.
 * \param[in] receive_buffer pointer to buffer which should get the received values.<BR>
 * If NULL, received bytes will be discarded.
 * \param[in] len number of bytes which should be transfered
 * \param[in] callback pointer to callback function which will be executed
 * from DMA channel interrupt once the transfer is finished.
 * If NULL, no callback function will be used, and UVOS_SPI_TransferBlock() will
 * block until the transfer is finished.
 * \return >= 0 if no error during transfer
 * \return -1 if disabled SPI port selected
 * \return -3 if function has been called during an ongoing DMA transfer
 */
static int32_t UVOS_SPI_TransferBlock_DMA( uint32_t spi_id, const uint8_t * send_buffer, uint8_t * receive_buffer, uint16_t len, void * callback )
{
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;

  bool valid = UVOS_SPI_validate( spi_dev );

  UVOS_Assert( valid )

  LL_DMA_InitTypeDef dma_init;

  /* Exit if ongoing transfer */
  if ( LL_DMA_GetDataLength( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream ) ) {
    return -3;
  }

  /* Disable the DMA channels */
  LL_DMA_DisableStream( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream );
  LL_DMA_DisableStream( spi_dev->cfg->dma.tx.DMAx, spi_dev->cfg->dma.tx.stream );

  while ( LL_DMA_IsEnabledStream( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream ) );
  while ( LL_DMA_IsEnabledStream( spi_dev->cfg->dma.tx.DMAx, spi_dev->cfg->dma.tx.stream ) );

  /* Disable the SPI peripheral */
  /* Initialize the SPI block */
  LL_SPI_DeInit( spi_dev->cfg->regs );
  LL_SPI_Init( spi_dev->cfg->regs, ( LL_SPI_InitTypeDef * ) & ( spi_dev->cfg->init ) );
  LL_SPI_Disable( spi_dev->cfg->regs );

  /* Enable SPI interrupts to DMA */
  LL_SPI_EnableDMAReq_TX( spi_dev->cfg->regs );
  LL_SPI_EnableDMAReq_RX( spi_dev->cfg->regs );

  /* Set callback function */
  spi_dev->callback = callback;

  /*
   * Configure Rx channel
   */

  /* Start with the default configuration for this peripheral */
  dma_init = spi_dev->cfg->dma.rx.init;
  LL_DMA_DeInit( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream );
  if ( receive_buffer != NULL ) {
    /* Enable memory addr. increment - bytes written into receive buffer */
    dma_init.MemoryOrM2MDstAddress = ( uint32_t )receive_buffer;
    dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  } else {
    /* Disable memory addr. increment - bytes written into dummy buffer */
    spi_dev->rx_dummy_byte = 0xFF;
    dma_init.MemoryOrM2MDstAddress = ( uint32_t )&spi_dev->rx_dummy_byte;
    dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT;
  }
  if ( spi_dev->cfg->use_crc ) {
    /* Make sure the CRC error flag is cleared before we start */
    LL_SPI_ClearFlag_CRCERR( spi_dev->cfg->regs );
  }

  dma_init.NbData = len;
  LL_DMA_Init( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream, ( LL_DMA_InitTypeDef * ) & ( dma_init ) );

  /*
   * Configure Tx channel
   */

  /* Start with the default configuration for this peripheral */
  dma_init = spi_dev->cfg->dma.tx.init;
  LL_DMA_DeInit( spi_dev->cfg->dma.tx.DMAx, spi_dev->cfg->dma.tx.stream );
  if ( send_buffer != NULL ) {
    /* Enable memory addr. increment - bytes written into receive buffer */
    dma_init.MemoryOrM2MDstAddress = ( uint32_t )send_buffer;
    dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  } else {
    /* Disable memory addr. increment - bytes written into dummy buffer */
    spi_dev->tx_dummy_byte = 0xFF;
    dma_init.MemoryOrM2MDstAddress = ( uint32_t )&spi_dev->tx_dummy_byte;
    dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT;
  }

  if ( spi_dev->cfg->use_crc ) {
    /* The last byte of the payload will be replaced with the CRC8 */
    dma_init.NbData = len - 1;
  } else {
    dma_init.NbData = len;
  }

  LL_DMA_Init( spi_dev->cfg->dma.tx.DMAx, spi_dev->cfg->dma.tx.stream, ( LL_DMA_InitTypeDef * ) & ( dma_init ) );

  /* Enable DMA interrupt if callback function active */
  // DMA_ITConfig( spi_dev->cfg->dma.rx.channel, DMA_IT_TC, ( callback != NULL ) ? ENABLE : DISABLE );
  if ( callback != NULL ) {
    LL_DMA_EnableIT_TC( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream );
  } else {
    LL_DMA_DisableIT_TC( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream );
  }
  // LL_DMA_EnableIT_TE( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream );

  /* Flush out the CRC registers */
  LL_SPI_DisableCRC( spi_dev->cfg->regs );
  ( void )LL_SPI_GetRxCRC( spi_dev->cfg->regs );
  LL_SPI_ClearFlag_CRCERR( spi_dev->cfg->regs );

  /* Make sure to flush out the receive buffer */
  ( void )LL_SPI_ReceiveData8( spi_dev->cfg->regs );

  if ( spi_dev->cfg->use_crc ) {
    /* Need a 0->1 transition to reset the CRC logic */
    LL_SPI_EnableCRC( spi_dev->cfg->regs );
  }

  /* Start DMA transfers */
  LL_DMA_EnableStream( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream );
  LL_DMA_EnableStream( spi_dev->cfg->dma.tx.DMAx, spi_dev->cfg->dma.tx.stream );

  /* Reenable the SPI device */
  LL_SPI_Enable( spi_dev->cfg->regs );

  if ( callback ) {
    /* User has requested a callback, don't wait for the transfer to complete. */
    return 0;
  }

  /* Wait until all bytes have been transmitted/received */
  while ( LL_DMA_GetDataLength( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream ) ) {
#if defined(UVOS_INCLUDE_FREERTOS)
    if ( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING ) {
      vTaskDelay( 0 );
    }
#endif
    ;
  }

  /* Wait for the final bytes of the transfer to complete, including CRC byte(s). */
  while ( !LL_SPI_IsActiveFlag_TXE( spi_dev->cfg->regs ) );

  /* Wait for the final bytes of the transfer to complete, including CRC byte(s). */
  while ( LL_SPI_IsActiveFlag_BSY( spi_dev->cfg->regs ) );

  /* Check the CRC on the transfer if enabled. */
  if ( spi_dev->cfg->use_crc ) {
    /* Check the SPI CRC error flag */
    if ( LL_SPI_IsActiveFlag_CRCERR( spi_dev->cfg->regs ) ) {
      return -4;
    }
  }

  /* No error */
  return 0;

}

/**
 * Transfers a block of bytes via Programmed I/O (PIO).
 *
 * https://en.wikipedia.org/wiki/Programmed_input%E2%80%93output
 * PIO stands for Programmed Input/Output, a protocol for data transfer.
 * Uses the CPU to manage bytewise transfer, as opposed to DMA which uses the DMA
 * peripheral to manage the transfer.
 *
 * \param[in] spi_id SPI device handle
 * \param[in] send_buffer pointer to buffer which should be sent.<BR>
 * If NULL, 0xff (all-one) will be sent.
 * \param[in] receive_buffer pointer to buffer which should get the received values.<BR>
 * If NULL, received bytes will be discarded.
 * \param[in] len number of bytes which should be transfered
 * \return >= 0 if no error during transfer
 * \return -1 if disabled SPI port selected
 * \return -3 if function has been called during an ongoing DMA transfer
 */
static int32_t UVOS_SPI_TransferBlock_PIO( uint32_t spi_id, const uint8_t * send_buffer, uint8_t * receive_buffer, uint16_t len )
{
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;
  uint8_t b;

  bool valid = UVOS_SPI_validate( spi_dev );

  UVOS_Assert( valid )

  /* Exit if ongoing transfer */
  if ( LL_DMA_GetDataLength( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream ) ) {
    return -3;
  }

  /* Make sure the RXNE flag is cleared by reading the DR register */
  b = spi_dev->cfg->regs->DR;

  while ( len-- ) {
    /* get the byte to send */
    b = send_buffer ? *( send_buffer++ ) : 0xff;

    /* Start the transfer */
    spi_dev->cfg->regs->DR = b;

    /* Wait until there is a byte to read */
    while ( !( spi_dev->cfg->regs->SR & SPI_SR_RXNE ) );

    /* Read the rx'd byte */
    b = spi_dev->cfg->regs->DR;

    /* save the received byte */
    if ( receive_buffer ) {
      *( receive_buffer++ ) = b;
    }

    /* Wait until the TXE goes high */
    while ( !( spi_dev->cfg->regs->SR & SPI_SR_TXE ) );
  }

  /* Wait for SPI transfer to have fully completed */
  while ( spi_dev->cfg->regs->SR & SPI_SR_BSY );

  return 0;
}

/**
 * Transfers a block of bytes via PIO or DMA.
 * \param[in] spi_id SPI device handle
 * \param[in] send_buffer pointer to buffer which should be sent.<BR>
 * If NULL, 0xff (all-one) will be sent.
 * \param[in] receive_buffer pointer to buffer which should get the received values.<BR>
 * If NULL, received bytes will be discarded.
 * \param[in] len number of bytes which should be transfered
 * \param[in] callback pointer to callback function which will be executed
 * from DMA channel interrupt once the transfer is finished.
 * If NULL, no callback function will be used, and UVOS_SPI_TransferBlock() will
 * block until the transfer is finished.
 * \return >= 0 if no error during transfer
 * \return -1 if disabled SPI port selected
 * \return -3 if function has been called during an ongoing DMA transfer
 */
int32_t UVOS_SPI_TransferBlock( uint32_t spi_id, const uint8_t * send_buffer, uint8_t * receive_buffer, uint16_t len, void * callback )
{
  if ( callback || len > SPI_MAX_BLOCK_PIO ) {
    return UVOS_SPI_TransferBlock_DMA( spi_id, send_buffer, receive_buffer, len, callback );
  }
  return UVOS_SPI_TransferBlock_PIO( spi_id, send_buffer, receive_buffer, len );
}

/**
 * Check if a transfer is in progress
 * \param[in] spi SPI number (0 or 1)
 * \return >= 0 if no transfer is in progress
 * \return -1 if disabled SPI port selected
 * \return -2 if unsupported SPI port selected
 * \return -3 if function has been called during an ongoing DMA transfer
 */
int32_t UVOS_SPI_Busy( uint32_t spi_id )
{
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;

  bool valid = UVOS_SPI_validate( spi_dev );

  UVOS_Assert( valid )

  /* DMA buffer has data or SPI transmit register not empty or SPI is busy */
  // if ( DMA_GetCurrDataCounter( spi_dev->cfg->dma.rx.channel ) ||
  //      !SPI_I2S_GetFlagStatus( spi_dev->cfg->regs, SPI_I2S_FLAG_TXE ) ||
  //      SPI_I2S_GetFlagStatus( spi_dev->cfg->regs, SPI_I2S_FLAG_BSY ) ) {
  //   return -3;
  // }
  if ( LL_DMA_GetDataLength( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.rx.stream ) ||
       !LL_SPI_IsActiveFlag_TXE( spi_dev->cfg->regs ) ||
       LL_SPI_IsActiveFlag_BSY( spi_dev->cfg->regs  ) ) {
    return -3;
  }

  return 0;
}

void UVOS_SPI_IRQ_Handler( uint32_t spi_id )
{
  struct uvos_spi_dev * spi_dev = ( struct uvos_spi_dev * )spi_id;

  bool valid = UVOS_SPI_validate( spi_dev );

  UVOS_Assert( valid )

  // FIXME XXX Only RX channel or better clear flags for both channels?
  DMA_ClearFlags( spi_dev->cfg->dma.rx.DMAx, spi_dev->cfg->dma.irq.flags );

  // if ( spi_dev->cfg->init.Mode == LL_SPI_MODE_MASTER ) {
  if ( LL_SPI_GetMode( spi_dev->cfg->regs ) == LL_SPI_MODE_MASTER ) {
    /* Wait for the final bytes of the transfer to complete, including CRC byte(s). */
    while ( !( LL_SPI_IsActiveFlag_TXE( spi_dev->cfg->regs ) ) );

    /* Wait for the final bytes of the transfer to complete, including CRC byte(s). */
    while ( LL_SPI_IsActiveFlag_BSY( spi_dev->cfg->regs ) );
  }

  if ( spi_dev->callback != NULL ) {
    bool crc_ok = true;
    uint8_t crc_val;

    if ( LL_SPI_IsActiveFlag_CRCERR( spi_dev->cfg->regs ) ) {
      crc_ok = false;
      LL_SPI_ClearFlag_CRCERR( spi_dev->cfg->regs );
    }
    crc_val = ( uint8_t )( spi_dev->cfg->regs->RXCRCR );
    spi_dev->callback( crc_ok, crc_val );
  }
}

#endif /* UVOS_INCLUDE_SPI */


