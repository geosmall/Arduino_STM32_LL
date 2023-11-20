#include "uvos.h"

#ifdef UVOS_INCLUDE_USART

#include <uvos_usart_priv.h>

/* Provide a COM driver */
static void UVOS_USART_ChangeBaud( uint32_t usart_id, uint32_t baud );
static void UVOS_USART_RegisterRxCallback( uint32_t usart_id, uvos_com_callback rx_in_cb, uint32_t context );
static void UVOS_USART_RegisterTxCallback( uint32_t usart_id, uvos_com_callback tx_out_cb, uint32_t context );
static void UVOS_USART_TxStart( uint32_t usart_id, uint16_t tx_bytes_avail );
static void UVOS_USART_RxStart( uint32_t usart_id, uint16_t rx_bytes_avail );

const struct uvos_com_driver uvos_usart_com_driver = {
  .set_baud      = UVOS_USART_ChangeBaud,
  // .set_ctrl_line = UVOS_USART_SetCtrlLine,
  .tx_start      = UVOS_USART_TxStart,
  .rx_start      = UVOS_USART_RxStart,
  .bind_tx_cb    = UVOS_USART_RegisterTxCallback,
  .bind_rx_cb    = UVOS_USART_RegisterRxCallback,
};

enum uvos_usart_dev_magic {
  UVOS_USART_DEV_MAGIC = 0x4152834A,
};

#define UVOS_USART_DMA_BUFFER_SIZE 64

struct uvos_usart_dev {
  enum uvos_usart_dev_magic   magic;
  const struct uvos_usart_cfg *cfg;

  uvos_com_callback rx_in_cb;
  uint32_t rx_in_context;
  uvos_com_callback tx_out_cb;
  uint32_t tx_out_context;
  uint8_t dma_buffer[UVOS_USART_DMA_BUFFER_SIZE];
  bool dma_tx_is_active;
};

static uint16_t get_dma_data_from_higher_level_fifo( struct uvos_usart_dev *usart_dev, bool *need_yield );

static bool UVOS_USART_validate( struct uvos_usart_dev *usart_dev )
{
  return usart_dev->magic == UVOS_USART_DEV_MAGIC;
}

#if defined(UVOS_INCLUDE_FREERTOS)
static struct uvos_usart_dev *UVOS_USART_alloc( void )
{
  struct uvos_usart_dev *usart_dev;

  usart_dev = ( struct uvos_usart_dev * )uvos_malloc( sizeof( struct uvos_usart_dev ) );
  if ( !usart_dev ) {
    return NULL;
  }

  memset( usart_dev, 0, sizeof( struct uvos_usart_dev ) );
  usart_dev->magic = UVOS_USART_DEV_MAGIC;
  return usart_dev;
}
#else
static struct uvos_usart_dev uvos_usart_devs[ UVOS_USART_MAX_DEVS ];
static uint8_t uvos_usart_num_devs;
static struct uvos_usart_dev *UVOS_USART_alloc( void )
{
  struct uvos_usart_dev *usart_dev;

  if ( uvos_usart_num_devs >= UVOS_USART_MAX_DEVS ) {
    return NULL;
  }

  usart_dev = &uvos_usart_devs[ uvos_usart_num_devs++ ];

  memset( usart_dev, 0, sizeof( struct uvos_usart_dev ) );
  usart_dev->magic = UVOS_USART_DEV_MAGIC;

  return usart_dev;
}
#endif /* if defined(UVOS_INCLUDE_FREERTOS) */

/* Bind Interrupt Handlers
 *
 * Map all valid USART IRQs to the common interrupt handler
 * and provide storage for a 32-bit device id IRQ to map
 * each physical IRQ to a specific registered device instance.
 */
static void UVOS_USART_generic_irq_handler( uint32_t usart_id );

static uint32_t UVOS_USART_1_id;
void USART1_IRQHandler( void ) __attribute__( ( alias( "UVOS_USART_1_irq_handler" ) ) );
static void UVOS_USART_1_irq_handler( void )
{
  UVOS_USART_generic_irq_handler( UVOS_USART_1_id );
}

static uint32_t UVOS_USART_2_id;
void USART2_IRQHandler( void ) __attribute__( ( alias( "UVOS_USART_2_irq_handler" ) ) );
static void UVOS_USART_2_irq_handler( void )
{
  UVOS_USART_generic_irq_handler( UVOS_USART_2_id );
}

static uint32_t UVOS_USART_3_id;
void USART3_IRQHandler( void ) __attribute__( ( alias( "UVOS_USART_3_irq_handler" ) ) );
static void UVOS_USART_3_irq_handler( void )
{
  UVOS_USART_generic_irq_handler( UVOS_USART_3_id );
}

static uint32_t UVOS_USART_4_id;
void USART4_IRQHandler( void ) __attribute__( ( alias( "UVOS_USART_4_irq_handler" ) ) );
static void UVOS_USART_4_irq_handler( void )
{
  UVOS_USART_generic_irq_handler( UVOS_USART_4_id );
}

static uint32_t UVOS_USART_5_id;
void USART5_IRQHandler( void ) __attribute__( ( alias( "UVOS_USART_5_irq_handler" ) ) );
static void UVOS_USART_5_irq_handler( void )
{
  UVOS_USART_generic_irq_handler( UVOS_USART_5_id );
}

static uint32_t UVOS_USART_6_id;
void USART6_IRQHandler( void ) __attribute__( ( alias( "UVOS_USART_6_irq_handler" ) ) );
static void UVOS_USART_6_irq_handler( void )
{
  UVOS_USART_generic_irq_handler( UVOS_USART_6_id );
}

/**
 * Initialise a single USART device
 */
int32_t UVOS_USART_Init( uint32_t *usart_id, const struct uvos_usart_cfg *cfg )
{
  UVOS_Assert( usart_id );
  UVOS_Assert( cfg );

  struct uvos_usart_dev *usart_dev;

  usart_dev = ( struct uvos_usart_dev * )UVOS_USART_alloc();
  if ( !usart_dev ) {
    goto out_fail;
  }

  /* Bind the configuration to the device instance */
  usart_dev->cfg = cfg;

  /* Disable callback functions */
  usart_dev->rx_in_cb = NULL;
  usart_dev->tx_out_cb = NULL;

  /* Initialize the USART Rx and Tx pins */
  LL_GPIO_Init( usart_dev->cfg->rx.gpio, ( LL_GPIO_InitTypeDef * )&usart_dev->cfg->rx.init );
  LL_GPIO_Init( usart_dev->cfg->tx.gpio, ( LL_GPIO_InitTypeDef * )&usart_dev->cfg->tx.init );

  /* Configure the USART */
  LL_USART_Init( usart_dev->cfg->regs, ( LL_USART_InitTypeDef * )&usart_dev->cfg->init );

  /* Save DMA Tx instance */
  DMA_TypeDef *DMAx_tx = usart_dev->cfg->dma_tx.tx.DMAx;

  /* Configure DMA for USART Tx */
  LL_DMA_DeInit( DMAx_tx, usart_dev->cfg->dma_tx.tx.stream );
  LL_DMA_DisableStream( DMAx_tx, usart_dev->cfg->dma_tx.tx.stream );
  LL_DMA_Init( DMAx_tx, usart_dev->cfg->dma_tx.tx.stream, ( LL_DMA_InitTypeDef * ) & ( usart_dev->cfg->dma_tx.tx.init ) );
  usart_dev->dma_tx_is_active = false;

  /* Set DMA memory address to the USART Tx device's buffer */
  LL_DMA_SetMemoryAddress( usart_dev->cfg->dma_tx.tx.DMAx,
                           usart_dev->cfg->dma_tx.tx.stream,
                           ( uint32_t ) & ( usart_dev->dma_buffer ) );

  *usart_id = ( uint32_t )usart_dev;

  /* Configure USART Interrupts */
  switch ( ( uint32_t )usart_dev->cfg->regs ) {
  case ( uint32_t )USART1:
    UVOS_USART_1_id = ( uint32_t )usart_dev;
    break;
  case ( uint32_t )USART2:
    UVOS_USART_2_id = ( uint32_t )usart_dev;
    break;
#if !defined(STM32F401xC) && !defined(STM32F401xE) && !defined(STM32F411xE)
  case ( uint32_t )USART3:
    UVOS_USART_3_id = ( uint32_t )usart_dev;
    break;
  case ( uint32_t )UART4:
    UVOS_USART_4_id = ( uint32_t )usart_dev;
    break;
  case ( uint32_t )UART5:
    UVOS_USART_5_id = ( uint32_t )usart_dev;
    break;
#endif // !defined(STM32F401xC) && !defined(STM32F401xE) && !defined(STM32F411xE)
  case ( uint32_t )USART6:
    UVOS_USART_6_id = ( uint32_t )usart_dev;
    break;
  }

  /* Configure USART RXNE Interrupt */
  NVIC_Init( ( NVIC_InitTypeDef * ) & ( usart_dev->cfg->irq.init ) );
  LL_USART_EnableIT_RXNE( usart_dev->cfg->regs );

  if ( usart_dev->cfg->use_dma_tx ) {
    /* Configure USART DMA transfer complete (TC) Interrupt */
    NVIC_Init( ( NVIC_InitTypeDef * ) & ( usart_dev->cfg->dma_tx.irq.init ) );
    LL_DMA_EnableIT_TC( usart_dev->cfg->dma_tx.tx.DMAx, usart_dev->cfg->dma_tx.tx.stream );
  } else {
    /* Configure USART TXE Interrupt */
    LL_USART_EnableIT_TXE( usart_dev->cfg->regs );
  }

  // FIXME XXX Clear / reset uart here - sends NUL char else

  /* Enable USART */
  // USART_Cmd( usart_dev->cfg->regs, ENABLE );
  LL_USART_Enable( usart_dev->cfg->regs );

  return 0;

out_fail:
  return -1;
}

static void UVOS_USART_RxStart( uint32_t usart_id, __attribute__( ( unused ) ) uint16_t rx_bytes_avail )
{
  struct uvos_usart_dev *usart_dev = ( struct uvos_usart_dev * )usart_id;

  bool valid = UVOS_USART_validate( usart_dev );

  UVOS_Assert( valid );

  LL_USART_EnableIT_RXNE( usart_dev->cfg->regs );
}

static void UVOS_USART_TxStart( uint32_t usart_id, __attribute__( ( unused ) ) uint16_t tx_bytes_avail )
{
  struct uvos_usart_dev *usart_dev = ( struct uvos_usart_dev * )usart_id;

  bool valid = UVOS_USART_validate( usart_dev );

  UVOS_Assert( valid );

  uint16_t bytes_to_send = 0;
  bool tx_need_yield = false;

  if ( usart_dev->cfg->use_dma_tx ) {
    if ( usart_dev->dma_tx_is_active == false ) {
      /* Start an active DMA stream transfer */
      usart_dev->dma_tx_is_active = true;
      /* Retrieve bytes to send from higher level device */
      bytes_to_send = get_dma_data_from_higher_level_fifo( usart_dev, &tx_need_yield );

      if ( bytes_to_send > 0 ) {
        /* Per STM32F4 DMA AN4031, 4.2 Clear all DMA stream flags in DMA_xIFCR register before starting new transfer. */
        DMA_ClearAllFlags( usart_dev->cfg->dma_tx.tx.DMAx, usart_dev->cfg->dma_tx.tx.stream );
        /* Per 4.3 Enable DMA stream used, then enable the peripheral DMA request enable bit. */
        LL_DMA_EnableStream( usart_dev->cfg->dma_tx.tx.DMAx, usart_dev->cfg->dma_tx.tx.stream );
        LL_USART_EnableDMAReq_TX( usart_dev->cfg->regs );
      }
    }
  } else {
    /* Enable USART TXE Interrupt */
    LL_USART_EnableIT_TXE( usart_dev->cfg->regs );
  }
}

/**
 * Changes the baud rate of the USART peripheral without re-initialising.
 * \param[in] usart_id USART name (GPS, TELEM, AUX)
 * \param[in] baud Requested baud rate
 */
static void UVOS_USART_ChangeBaud( uint32_t usart_id, uint32_t baud )
{
  struct uvos_usart_dev *usart_dev = ( struct uvos_usart_dev * )usart_id;

  bool valid = UVOS_USART_validate( usart_dev );

  UVOS_Assert( valid );

  LL_USART_InitTypeDef USART_InitStructure;

  /* Start with a copy of the default configuration for the peripheral */
  USART_InitStructure = usart_dev->cfg->init;

  /* Adjust the baud rate */
  USART_InitStructure.BaudRate = baud;

  /* Write back the new configuration */
  LL_USART_Init( usart_dev->cfg->regs, &USART_InitStructure );
}

static void UVOS_USART_RegisterRxCallback( uint32_t usart_id, uvos_com_callback rx_in_cb, uint32_t context )
{
  struct uvos_usart_dev *usart_dev = ( struct uvos_usart_dev * )usart_id;

  bool valid = UVOS_USART_validate( usart_dev );

  UVOS_Assert( valid );

  /*
   * Order is important in these assignments since ISR uses _cb
   * field to determine if it's ok to dereference _cb and _context
   */
  usart_dev->rx_in_context = context;
  usart_dev->rx_in_cb = rx_in_cb;
}

static void UVOS_USART_RegisterTxCallback( uint32_t usart_id, uvos_com_callback tx_out_cb, uint32_t context )
{
  struct uvos_usart_dev *usart_dev = ( struct uvos_usart_dev * )usart_id;

  bool valid = UVOS_USART_validate( usart_dev );

  UVOS_Assert( valid );

  /*
   * Order is important in these assignments since ISR uses _cb
   * field to determine if it's ok to dereference _cb and _context
   */
  usart_dev->tx_out_context = context;
  usart_dev->tx_out_cb = tx_out_cb;
}

static void UVOS_USART_generic_irq_handler( uint32_t usart_id )
{
  struct uvos_usart_dev *usart_dev = ( struct uvos_usart_dev * )usart_id;

  bool valid = UVOS_USART_validate( usart_dev );

  UVOS_Assert( valid );

  /* Force read of dr after sr to make sure to clear error flags */
  volatile uint16_t sr = usart_dev->cfg->regs->SR;
  volatile uint8_t dr  = usart_dev->cfg->regs->DR;

  /* Check if RXNE flag is set */
  bool rx_need_yield   = false;
  if ( sr & USART_SR_RXNE ) {
    uint8_t byte = dr;
    if ( usart_dev->rx_in_cb ) {
      ( void )( usart_dev->rx_in_cb )( usart_dev->rx_in_context, &byte, 1, NULL, &rx_need_yield );
    }
  }

  /* Check if TXE flag is set */
  bool tx_need_yield = false;
  if ( sr & USART_SR_TXE ) {
    if ( usart_dev->tx_out_cb ) {
      uint8_t b;
      uint16_t bytes_to_send;

      bytes_to_send = ( usart_dev->tx_out_cb )( usart_dev->tx_out_context, &b, 1, NULL, &tx_need_yield );

      if ( bytes_to_send > 0 ) {
        /* Send the byte we've been given */
        usart_dev->cfg->regs->DR = b;
      } else {
        /* No bytes to send, disable TXE interrupt */
        // USART_ITConfig( usart_dev->cfg->regs, USART_IT_TXE, DISABLE );
        LL_USART_DisableIT_TXE( usart_dev->cfg->regs );
      }
    } else {
      /* No bytes to send, disable TXE interrupt */
      // USART_ITConfig( usart_dev->cfg->regs, USART_IT_TXE, DISABLE );
      LL_USART_DisableIT_TXE( usart_dev->cfg->regs );
    }
  }

#if defined(UVOS_INCLUDE_FREERTOS)
  portEND_SWITCHING_ISR( ( rx_need_yield || tx_need_yield ) ? pdTRUE : pdFALSE );
#endif /* UVOS_INCLUDE_FREERTOS */
}

/**
 * Call higher level tx callback handler to retrieve any waiting fifo data
 * If bytes copied is greater than zero this will unblock the higher level device Tx.
 * \param[in] *usart_dev pointer to usart device
 * \param[in] *need_yield true if higher priority task has been woken (if multi-task RTOS is used)
 * \return number of bytes copied from higher level device fifoBuf
 */
static uint16_t get_dma_data_from_higher_level_fifo( struct uvos_usart_dev *usart_dev, bool *need_yield )
{
  uint16_t bytes_to_send = 0;

  if ( usart_dev->cfg->use_dma_tx ) {
    /* Call Tx callback to get data from fifo */
    if ( usart_dev->tx_out_cb ) {
      bytes_to_send = ( usart_dev->tx_out_cb )(
                        usart_dev->tx_out_context,
                        usart_dev->dma_buffer,
                        UVOS_USART_DMA_BUFFER_SIZE,
                        NULL,
                        need_yield );
    } else {
      return 0;
    }

    if ( bytes_to_send > 0 ) {
      /* Set the DMA stream NDTR  */
      LL_DMA_SetDataLength( usart_dev->cfg->dma_tx.tx.DMAx, usart_dev->cfg->dma_tx.tx.stream, bytes_to_send );
    }
  }

  return bytes_to_send;
}

void UVOS_USART_generic_dma_irq_handler( uint32_t usart_id )
{
  struct uvos_usart_dev *usart_dev = ( struct uvos_usart_dev * )usart_id;

  bool valid = UVOS_USART_validate( usart_dev );

  UVOS_Assert( valid );

  bool tx_need_yield = false;
  uint16_t bytes_to_send = 0;

  if ( DMA_IsActiveFlag( usart_dev->cfg->dma_tx.tx.DMAx, usart_dev->cfg->dma_tx.tx.stream, UVOS_DMA_IT_TC ) ) {
    /* Clear pending interrupt flags */
    DMA_ClearFlags( usart_dev->cfg->dma_tx.tx.DMAx, usart_dev->cfg->dma_tx.tx.stream, UVOS_DMA_IT_TC );

    /* See if there are more bytes to send from higher level device */
    bytes_to_send = get_dma_data_from_higher_level_fifo( usart_dev, &tx_need_yield );

    if ( bytes_to_send > 0 ) {
      usart_dev->dma_tx_is_active = true;
      /* Per STM32F4 DMA AN4031, 4.2 Clear all DMA stream flags in DMA_xIFCR register before starting new transfer. */
      DMA_ClearAllFlags( usart_dev->cfg->dma_tx.tx.DMAx, usart_dev->cfg->dma_tx.tx.stream );
      /* Per 4.3 Enable DMA stream used, then enable the peripheral DMA request enable bit. */
      LL_DMA_EnableStream( usart_dev->cfg->dma_tx.tx.DMAx, usart_dev->cfg->dma_tx.tx.stream );
      LL_USART_EnableDMAReq_TX( usart_dev->cfg->regs );
    } else {
      /* No bytes to send, disable TX DMA transfer per Per STM32F4 DMA AN4031, 4.1. */
      LL_DMA_DisableStream( usart_dev->cfg->dma_tx.tx.DMAx, usart_dev->cfg->dma_tx.tx.stream );
      while ( LL_DMA_IsEnabledStream( usart_dev->cfg->dma_tx.tx.DMAx, usart_dev->cfg->dma_tx.tx.stream ) != RESET );
      LL_USART_DisableDMAReq_TX( usart_dev->cfg->regs );
      usart_dev->dma_tx_is_active = false;
    }
  }
#if defined(PIOS_INCLUDE_FREERTOS)
  portEND_SWITCHING_ISR( tx_need_yield ? pdTRUE : pdFALSE );
#endif  /* defined(PIOS_INCLUDE_FREERTOS) */
}

#endif /* UVOS_INCLUDE_USART */
