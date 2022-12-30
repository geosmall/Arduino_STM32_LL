
/* I/O mapping
 *      I/O |   PIN     |    Channel    |   Alternate function
 *      I1  |           |   TIM2_CH3    |   x
 *      i2  |           |   TIM3_CH4    |   ADC1_9
 *      I3  |           |   TIM3_CH3    |   ADC1_8
 *      I4  |           |   TIM3_CH2    |   SPI1_MOSI/ADC1_7
 *      I5  |           |   TIM3_CH1    |   SPI1_MISO/ADC1_6
 *      I6  |           |   TIM2_CH1    |   SPI1_SCK/ADC1_5
 *------------------------------------
 *      o1  |   PA8     |   TIM1_CH1    |
 *      o2  |   PA9     |   TIM1_CH2    |   USART1_TX
 *      o3  |   PA10    |   TIM1_CH3    |   USART1_RX
 *      o4  |   PB0     |   TIM3_CH3    |   ADC1_8
 *      o5  |   PB4     |   TIM3_CH1    |   SPI1_MISO
 *      o6  |   PB5     |   TIM3_CH2    |   SPI1_MOSI

Servo out 1: PA8  - TIM1_CH1 (29)
Servo out 2: PA9  - TIM1_CH2 (30)
Servo out 3: PA10 - TIM1_CH3 (31)
Servo out 4: PB0  - TIM3_CH3 (18)
Servo out 5: PB4  - TIM3_CH1 (40)
Servo out 6: PB5  - TIM3_CH2 (41)

 */
#define MAIN_USART_REGS                  USART2
#define MAIN_USART_REMAP                 GPIO_AF_USART2
#define MAIN_USART_IRQ                   USART2_IRQn
#define MAIN_USART_RX_GPIO               GPIOA
#define MAIN_USART_RX_PIN                GPIO_Pin_3
#define MAIN_USART_TX_GPIO               GPIOA
#define MAIN_USART_TX_PIN                GPIO_Pin_2
// Inverter for SBUS handling
#define MAIN_USART_INVERTER_GPIO         GPIOC
#define MAIN_USART_INVERTER_PIN          GPIO_Pin_15
#define MAIN_USART_INVERTER_CLOCK_FUNC   RCC_AHB1PeriphClockCmd
#define MAIN_USART_INVERTER_CLOCK_PERIPH RCC_AHB1Periph_GPIOC

#define FLEXI_USART_REGS                 USART1
#define FLEXI_USART_REMAP                GPIO_AF_USART1
#define FLEXI_USART_IRQ                  USART1_IRQn
#define FLEXI_USART_RX_GPIO              GPIOB
#define FLEXI_USART_RX_PIN               GPIO_Pin_7
#define FLEXI_USART_TX_GPIO              GPIOB
#define FLEXI_USART_TX_PIN               GPIO_Pin_6
// ReceiverPort pin 3
#define FLEXI_USART_DTR_GPIO             GPIOB
#define FLEXI_USART_DTR_PIN              GPIO_Pin_10

#if defined(UVOS_INCLUDE_LED)

#include <uvos_led_priv.h>
static const struct uvos_gpio uvos_leds[] = {
  [ UVOS_LED_HEARTBEAT ] = {
    .pin               =             {
      .gpio = GPIOB,
      .init =             {
        .Pin   = LL_GPIO_PIN_8,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .Mode  = LL_GPIO_MODE_OUTPUT,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull  = LL_GPIO_PULL_NO
      },
    },
    .active_low         = true
  },
  [ UVOS_LED_ALARM ] =     {
    .pin               =             {
      .gpio = GPIOB,
      .init =             {
        .Pin   = LL_GPIO_PIN_9,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .Mode  = LL_GPIO_MODE_OUTPUT,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull  = LL_GPIO_PULL_NO
      },
    },
    .active_low         = true
  },
};

static const struct uvos_gpio_cfg uvos_led_cfg = {
  .gpios     = uvos_leds,
  .num_gpios = NELEMENTS( uvos_leds ),
};

#endif /* UVOS_INCLUDE_LED */

#if defined( UVOS_INCLUDE_SPI )
#include <uvos_spi_priv.h>

/*
 * SPI1 Interface
 * Used for MPU6000 gyro and accelerometer
 */
void UVOS_SPI_gyro_irq_handler( void );
void DMA2_Stream2_IRQHandler( void ) __attribute__( ( alias( "UVOS_SPI_gyro_irq_handler" ) ) );
void DMA2_Stream3_IRQHandler( void ) __attribute__( ( alias( "UVOS_SPI_gyro_irq_handler" ) ) );
static const struct uvos_spi_cfg uvos_spi_gyro_cfg = {
  .regs  = SPI1,
  .init  = {
    .TransferDirection = LL_SPI_FULL_DUPLEX,
    .Mode = LL_SPI_MODE_MASTER,
    .DataWidth = LL_SPI_DATAWIDTH_8BIT,
    .ClockPolarity = LL_SPI_POLARITY_LOW,
    .ClockPhase = LL_SPI_PHASE_1EDGE,
    .NSS = LL_SPI_NSS_SOFT,
    .BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV128,
    .BitOrder = LL_SPI_MSB_FIRST,
    .CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
    .CRCPoly = 10,
  },
  .use_crc = false,
  .dma     = {
    .irq                                       = {
      .flags = ( DMA_LISR_TCIF2 | DMA_LISR_TEIF2 | DMA_LISR_HTIF2 ),
      .init  = {
        .NVIC_IRQChannel    = DMA2_Stream2_IRQn,
        .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_HIGH,
        .NVIC_IRQChannelSubPriority        = 0,
        .NVIC_IRQChannelCmd = ENABLE,
      },
    },

    .rx                                        = {
      .DMAx = DMA2,
      .stream = LL_DMA_STREAM_2,
      .init    = {
        .Channel = LL_DMA_CHANNEL_3,
        .PeriphOrM2MSrcAddress = ( uint32_t ) & ( SPI1->DR ),
        .MemoryOrM2MDstAddress = ( uint32_t ) NULL,
        .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
        .Mode = LL_DMA_MODE_NORMAL,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
        .NbData = ( uint32_t ) NULL,
        .Priority = LL_DMA_PRIORITY_LOW,
        .FIFOMode = LL_DMA_FIFOMODE_DISABLE,
        /* .FIFOThreshold */
        .MemBurst = LL_DMA_MBURST_SINGLE,
        .PeriphBurst = LL_DMA_PBURST_SINGLE,
      },
    },
    .tx                                        = {
      .DMAx = DMA2,
      .stream = LL_DMA_STREAM_3,
      .init    = {
        .Channel = LL_DMA_CHANNEL_3,
        .PeriphOrM2MSrcAddress = ( uint32_t ) & ( SPI1->DR ),
        .MemoryOrM2MDstAddress = ( uint32_t ) NULL,
        .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
        .Mode = LL_DMA_MODE_NORMAL,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
        .NbData = ( uint32_t ) NULL,
        .Priority = LL_DMA_PRIORITY_LOW,
        .FIFOMode = LL_DMA_FIFOMODE_DISABLE,
        /* .FIFOThreshold */
        .MemBurst = LL_DMA_MBURST_SINGLE,
        .PeriphBurst = LL_DMA_PBURST_SINGLE,
      },
    },
  },
  .sclk                                          = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_5,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_NO,
      .Alternate = LL_GPIO_AF_5,
    },
  },
  .miso                                          = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_6,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_NO,
      .Alternate = LL_GPIO_AF_5,
    },
  },
  .mosi                                          = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_7,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_NO,
      .Alternate = LL_GPIO_AF_5,
    },
  },
  .slave_count                                   = 1,
  .ssel                                          = {
    {
      .gpio = GPIOA,
      .init = {
        .Pin   = LL_GPIO_PIN_4,
        .Mode = LL_GPIO_MODE_OUTPUT,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_NO,
      }
    }
  }
};

static uint32_t uvos_spi_gyro_id;
void UVOS_SPI_gyro_irq_handler( void )
{
  /* Call into the generic code to handle the IRQ for this specific device */
  UVOS_SPI_IRQ_Handler( uvos_spi_gyro_id );
}

/*
 * SPI2 Interface
 * Used for Flash chip
 */
void UVOS_SPI_telem_flash_irq_handler( void );
void DMA1_Stream3_IRQHandler( void ) __attribute__( ( alias( "UVOS_SPI_telem_flash_irq_handler" ) ) );
void DMA1_Stream4_IRQHandler( void ) __attribute__( ( alias( "UVOS_SPI_telem_flash_irq_handler" ) ) );
static const struct uvos_spi_cfg uvos_spi_telem_flash_cfg = {
  .regs  = SPI2,
  .init  = {
    .TransferDirection = LL_SPI_FULL_DUPLEX,
    .Mode = LL_SPI_MODE_MASTER,
    .DataWidth = LL_SPI_DATAWIDTH_8BIT,
    .ClockPolarity = LL_SPI_POLARITY_LOW,
    .ClockPhase = LL_SPI_PHASE_1EDGE,
    .NSS = LL_SPI_NSS_SOFT,
    .BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8,
    .BitOrder = LL_SPI_MSB_FIRST,
    .CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
    .CRCPoly = 7,
  },
  .use_crc = false,
  .dma     = {
    .irq                                       = {
      // Note this is the stream ID that triggers interrupts (in this case RX)
      .flags = ( DMA_LISR_TCIF0 | DMA_LISR_TEIF0 | DMA_LISR_HTIF0 ),
      .init  = {
        .NVIC_IRQChannel    = DMA1_Stream3_IRQn,
        .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_HIGH,
        .NVIC_IRQChannelSubPriority        = 0,
        .NVIC_IRQChannelCmd = ENABLE,
      },
    },

    .rx                                        = {
      .DMAx = DMA1,
      .stream = LL_DMA_STREAM_3,
      .init    = {
        .Channel = LL_DMA_CHANNEL_0,
        .PeriphOrM2MSrcAddress = ( uint32_t ) & ( SPI2->DR ),
        .MemoryOrM2MDstAddress = ( uint32_t ) NULL,
        .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
        .Mode = LL_DMA_MODE_NORMAL,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
        .NbData = ( uint32_t ) NULL,
        .Priority = LL_DMA_PRIORITY_MEDIUM,
        .FIFOMode = LL_DMA_FIFOMODE_DISABLE,
        .FIFOThreshold = LL_DMA_FIFOTHRESHOLD_FULL,
        .MemBurst = LL_DMA_MBURST_SINGLE,
        .PeriphBurst = LL_DMA_PBURST_SINGLE,
      },
    },
    .tx                                        = {
      .DMAx = DMA1,
      .stream = LL_DMA_STREAM_4,
      .init    = {
        .Channel = LL_DMA_CHANNEL_0,
        .PeriphOrM2MSrcAddress = ( uint32_t ) & ( SPI2->DR ),
        .MemoryOrM2MDstAddress = ( uint32_t ) NULL,
        .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
        .Mode = LL_DMA_MODE_NORMAL,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
        .NbData = ( uint32_t ) NULL,
        .Priority = LL_DMA_PRIORITY_MEDIUM,
        .FIFOMode = LL_DMA_FIFOMODE_DISABLE,
        .FIFOThreshold = LL_DMA_FIFOTHRESHOLD_FULL,
        .MemBurst = LL_DMA_MBURST_SINGLE,
        .PeriphBurst = LL_DMA_PBURST_SINGLE,
      },
    },
  },
  .sclk                                          = {
    .gpio = GPIOB,
    .init = {
      .Pin   = LL_GPIO_PIN_13,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_NO,
      .Alternate = LL_GPIO_AF_5,
    },
  },
  .miso                                          = {
    .gpio = GPIOB,
    .init = {
      .Pin   = LL_GPIO_PIN_14,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_NO,
      .Alternate = LL_GPIO_AF_5,
    },
  },
  .mosi                                          = {
    .gpio = GPIOB,
    .init = {
      .Pin   = LL_GPIO_PIN_15,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_NO,
      .Alternate = LL_GPIO_AF_5,
    },
  },
  .slave_count                                   = 1,
  .ssel                                          = {
    {
      // Flash
      .gpio = GPIOB,
      .init = {
        .Pin   = LL_GPIO_PIN_12,
        .Mode = LL_GPIO_MODE_OUTPUT,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_UP,
      },
    },
  },
};

static uint32_t uvos_spi_telem_flash_id;
void UVOS_SPI_telem_flash_irq_handler( void )
{
  /* Call into the generic code to handle the IRQ for this specific device */
  UVOS_SPI_IRQ_Handler( uvos_spi_telem_flash_id );
}

#endif /* UVOS_INCLUDE_SPI */

#if defined( UVOS_INCLUDE_FLASH )
#include "uvos_flashfs_logfs_priv.h"
#include "uvos_flash_jedec_priv.h"
// #include "uvos_flash_internal_priv.h"

static const struct flashfs_logfs_cfg flashfs_external_user_cfg = {
  .fs_magic      = 0x99abceff,
  .total_fs_size = 0x001C0000, /* 2M bytes (32 sectors = entire chip) */
  .arena_size    = 0x000E0000, /* biggest possible arena size fssize/2 */
  .slot_size     = 0x00000100, /* 256 bytes */

  .start_offset  = 0x00040000, /* start offset */
  .sector_size   = 0x00010000, /* 64K bytes */
  .page_size     = 0x00000100, /* 256 bytes */
};

static const struct flashfs_logfs_cfg flashfs_external_system_cfg = {
  .fs_magic      = 0x99bbcdef,
  .total_fs_size = 0x00040000, /* 2M bytes (32 sectors = entire chip) */
  .arena_size    = 0x00010000, /* 256 * slot size */
  .slot_size     = 0x00000100, /* 256 bytes */

  .start_offset  = 0,          /* start at the beginning of the chip */
  .sector_size   = 0x00010000, /* 64K bytes */
  .page_size     = 0x00000100, /* 256 bytes */
};

#endif /* UVOS_INCLUDE_FLASH */

#include <uvos_usart_priv.h>

#ifdef UVOS_INCLUDE_COM_TELEM

/*
 * MAIN USART
 */
static const struct uvos_usart_cfg uvos_usart_main_cfg = {
  .regs  = USART1,
  .init  = {
    .BaudRate            = 57600,
    .DataWidth           = LL_USART_DATAWIDTH_8B,
    .StopBits            = LL_USART_STOPBITS_1,
    .Parity              = LL_USART_PARITY_NONE,
    .TransferDirection   = LL_USART_DIRECTION_TX_RX,
    .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
  },
  .irq                                       = {
    .init                                  = {
      .NVIC_IRQChannel    = USART1_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
  .rx                                        = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_10,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_UP,
      .Alternate = LL_GPIO_AF_7,
    },
  },
  .tx                                        = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_9,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_UP,
      .Alternate = LL_GPIO_AF_7,
    },
  },
};
#endif /* UVOS_INCLUDE_COM_TELEM */

#ifdef UVOS_INCLUDE_COM_FLEXI
/*
 * FLEXI PORT
 */
static const struct uvos_usart_cfg uvos_usart_flexi_cfg = {
  .regs  = USART2,
  .init  = {
    .BaudRate            = 57600,
    .DataWidth           = LL_USART_DATAWIDTH_8B,
    .StopBits            = LL_USART_STOPBITS_1,
    .Parity              = LL_USART_PARITY_NONE,
    .TransferDirection   = LL_USART_DIRECTION_TX_RX,
    .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
  },
  .irq                                       = {
    .init                                  = {
      .NVIC_IRQChannel    = USART2_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
  .rx                                        = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_3,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_UP,
      .Alternate = LL_GPIO_AF_7,
    },
  },
  .tx                                        = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_2,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_UP,
      .Alternate = LL_GPIO_AF_7,
    },
  },
};

#endif /* UVOS_INCLUDE_COM_FLEXI */

#if defined(UVOS_INCLUDE_IBUS)
/*
 * IBUS on Flexi Port USART
 */
#include <uvos_ibus_priv.h>

static const struct uvos_usart_cfg uvos_usart_ibus_flexi_cfg = {
  .regs  = USART1,
  .init  = {
    .BaudRate            = 115200,
    .DataWidth           = LL_USART_DATAWIDTH_8B,
    .StopBits            = LL_USART_STOPBITS_1,
    .Parity              = LL_USART_PARITY_NONE,
    .TransferDirection   = LL_USART_DIRECTION_RX,
    .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
  },
  .irq                                       = {
    .init                                  = {
      .NVIC_IRQChannel    = USART1_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_HIGH,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
  .rx                                        = {
    .gpio = GPIOB,
    .init = {
      .Pin   = LL_GPIO_PIN_7,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_UP,
      .Alternate = LL_GPIO_AF_7,
    },
  },
  .tx                                        = {
    .gpio = GPIOB,
    .init = {
      .Pin   = LL_GPIO_PIN_6,
      .Mode = LL_GPIO_MODE_OUTPUT,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_NO,
      .Alternate = LL_GPIO_AF_7,
    },
  },
};

#endif /* UVOS_INCLUDE_IBUS */

#if defined(UVOS_INCLUDE_RTC)
/*
 * Realtime Clock (RTC)
 */
#include <uvos_rtc_priv.h>

void UVOS_RTC_IRQ_Handler( void );
void RTC_WKUP_IRQHandler() __attribute__( ( alias( "UVOS_RTC_IRQ_Handler" ) ) );
static const struct uvos_rtc_cfg uvos_rtc_main_cfg = {
  .hse_clkdiv  = LL_RCC_RTC_HSE_DIV_8, // Divide 8 Mhz crystal down to 1
  .WUT_clkdiv = LL_RTC_WAKEUPCLOCK_DIV_16, // Divide 1 MHz by 16 gives 62.5 khz clock
  .WUT_reload = 100, // Every 100 cycles gives 625 Hz
  .irq                                       = {
    .init                                  = {
      .NVIC_IRQChannel    = RTC_WKUP_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

void UVOS_RTC_IRQ_Handler( void )
{
  UVOS_RTC_irq_handler();
}

#endif /* if defined(PIOS_INCLUDE_RTC) */

#include "uvos_tim_priv.h"

static const LL_TIM_InitTypeDef tim_1_8_9_10_time_base = {
  .Prescaler         = ( UVOS_PERIPHERAL_APB2_CLOCK / 1000000 ) - 1,
  .CounterMode       = LL_TIM_COUNTERMODE_UP,
  .Autoreload        = ( ( 1000000 / UVOS_SERVO_UPDATE_HZ ) - 1 ),
  .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1,
  .RepetitionCounter = 0x0000,
};
static const LL_TIM_InitTypeDef tim_2_3_4_5_time_base = {
  .Prescaler         = ( UVOS_PERIPHERAL_APB1_CLOCK / 1000000 ) - 1,
  .CounterMode       = LL_TIM_COUNTERMODE_UP,
  .Autoreload        = ( ( 1000000 / UVOS_SERVO_UPDATE_HZ ) - 1 ),
  .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1,
  .RepetitionCounter = 0x0000,
};
static const LL_TIM_InitTypeDef tim_11_time_base = {
  .Prescaler         = ( UVOS_PERIPHERAL_APB2_CLOCK / 1000000 ) - 1,
  .CounterMode       = LL_TIM_COUNTERMODE_UP,
  .Autoreload        = ( ( 1000000 / UVOS_SCHED_UPDATE_HZ ) - 1 ),
  .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1,
  .RepetitionCounter = 0x0000,
};

static const struct uvos_tim_clock_cfg tim_1_cfg = {
  .timer = TIM1,
  .time_base_init                            = &tim_1_8_9_10_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM1_UP_TIM10_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

static const struct uvos_tim_clock_cfg tim_2_cfg = {
  .timer = TIM2,
  .time_base_init                            = &tim_2_3_4_5_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM1_UP_TIM10_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

static const struct uvos_tim_clock_cfg tim_3_cfg = {
  .timer = TIM3,
  .time_base_init                            = &tim_2_3_4_5_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM3_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

static const struct uvos_tim_clock_cfg tim_5_cfg = {
  .timer = TIM5,
  .time_base_init                            = &tim_2_3_4_5_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM5_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

static const struct uvos_tim_clock_cfg tim_9_cfg = {
  .timer = TIM9,
  .time_base_init                            = &tim_1_8_9_10_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM1_BRK_TIM9_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

static const struct uvos_tim_clock_cfg tim_10_cfg = {
  .timer = TIM10,
  .time_base_init                            = &tim_1_8_9_10_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM1_UP_TIM10_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

static const struct uvos_tim_clock_cfg tim_11_cfg = {
  .timer = TIM11,
  .time_base_init                            = &tim_11_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM1_TRG_COM_TIM11_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

/**
 * Pios servo configuration structures
 * Using TIM1, TIM3
 *      o1  |   PA8    |  TIM1_CH1   |   AF01   |
 *      o2  |   PA9    |  TIM1_CH2   |   AF01   |  USART1_TX
 *      o3  |   PA10   |  TIM1_CH3   |   AF01   |  USART1_RX
 *      o4  |   PB0    |  TIM3_CH3   |   AF02   |  ADC1_8
 *      o5  |   PB4    |  TIM3_CH1   |   AF02   |  SPI1_MISO
 *      o6  |   PB5    |  TIM3_CH2   |   AF02   |  SPI1_MOSI
 */
#include <uvos_servo_priv.h>
#include <uvos_servo_config.h>
static const struct uvos_tim_channel uvos_tim_servoport_all_pins[] = {
  {
    .timer = TIM1,
    .timer_chan = LL_TIM_CHANNEL_CH1,
    .pin   = {
      .gpio = GPIOA,
      .init = {
        .Pin   = LL_GPIO_PIN_8,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_UP,
        .Alternate = LL_GPIO_AF_1,
      },
    },
  },
  {
    .timer = TIM1,
    .timer_chan = LL_TIM_CHANNEL_CH2,
    .pin   = {
      .gpio = GPIOA,
      .init = {
        .Pin   = LL_GPIO_PIN_9,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_UP,
        .Alternate = LL_GPIO_AF_1,
      },
    },
  },
  {
    .timer = TIM1,
    .timer_chan = LL_TIM_CHANNEL_CH3,
    .pin   = {
      .gpio = GPIOA,
      .init = {
        .Pin   = LL_GPIO_PIN_10,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_UP,
        .Alternate = LL_GPIO_AF_1,
      },
    },
  },
  {
    .timer = TIM3,
    .timer_chan = LL_TIM_CHANNEL_CH3,
    .pin   = {
      .gpio = GPIOB,
      .init = {
        .Pin   = LL_GPIO_PIN_0,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_UP,
        .Alternate = LL_GPIO_AF_2,
      },
    },
  },
  {
    .timer = TIM3,
    .timer_chan = LL_TIM_CHANNEL_CH1,
    .pin   = {
      .gpio = GPIOB,
      .init = {
        .Pin   = LL_GPIO_PIN_4,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_UP,
        .Alternate = LL_GPIO_AF_2,
      },
    },
  },
  {
    .timer = TIM3,
    .timer_chan = LL_TIM_CHANNEL_CH2,
    .pin   = {
      .gpio = GPIOB,
      .init = {
        .Pin   = LL_GPIO_PIN_5,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_UP,
        .Alternate = LL_GPIO_AF_2,
      },
    },
  },
};

#define UVOS_SERVOPORT_ALL_PINS_PWMOUT        6

const struct uvos_servo_cfg uvos_servo_cfg_out = {
  .tim_oc_init          = {
    // .TIM_OCMode       = TIM_OCMode_PWM1,
    // .TIM_OutputState  = TIM_OutputState_Enable,
    // .TIM_OutputNState = TIM_OutputNState_Disable,
    // .TIM_Pulse        = PIOS_SERVOS_INITIAL_POSITION,
    // .TIM_OCPolarity   = TIM_OCPolarity_High,
    // .TIM_OCNPolarity  = TIM_OCPolarity_High,
    // .TIM_OCIdleState  = TIM_OCIdleState_Reset,
    // .TIM_OCNIdleState = TIM_OCNIdleState_Reset,
    .OCMode = LL_TIM_OCMODE_PWM1,
    .OCState = LL_TIM_OCSTATE_ENABLE,
    .OCNState = LL_TIM_OCSTATE_DISABLE,
    .CompareValue = UVOS_SERVOS_INITIAL_POSITION,
    .OCPolarity = LL_TIM_OCPOLARITY_HIGH,
    .OCNPolarity = LL_TIM_OCPOLARITY_HIGH,
    .OCIdleState = LL_TIM_OCIDLESTATE_LOW,
    .OCNIdleState = LL_TIM_OCIDLESTATE_LOW,
  },
  .channels     = uvos_tim_servoport_all_pins,
  .num_channels = UVOS_SERVOPORT_ALL_PINS_PWMOUT,
};

#if defined(UVOS_INCLUDE_RCVR)

#include "uvos_rcvr_priv.h"

#endif /* UVOS_INCLUDE_RCVR */