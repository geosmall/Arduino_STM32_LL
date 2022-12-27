#pragma once

#include <uvos.h>
#include <uvos_stm32.h>

#if 0
/**
  * @brief LL SPI Init structures definition
  */
typedef struct {
  uint32_t TransferDirection;       /*!< Specifies the SPI unidirectional or bidirectional data mode. */
  uint32_t Mode;                    /*!< Specifies the SPI mode (Master/Slave). */
  uint32_t DataWidth;               /*!< Specifies the SPI data width. */
  uint32_t ClockPolarity;           /*!< Specifies the serial clock steady state. */
  uint32_t ClockPhase;              /*!< Specifies the clock active edge for the bit capture. */
  uint32_t NSS;                     /*!< Specifies whether the NSS signal is managed by hardware (NSS pin) or by software using the SSI bit. */
  uint32_t BaudRate;                /*!< Specifies the BaudRate prescaler value which will be used to configure the transmit and receive SCK clock. */
  uint32_t BitOrder;                /*!< Specifies whether data transfers start from MSB or LSB bit. */
  uint32_t CRCCalculation;          /*!< Specifies if the CRC calculation is enabled or not. */
  uint32_t CRCPoly;                 /*!< Specifies the polynomial used for the CRC calculation. */
} LL_SPI_InitTypeDef;
#endif

#define SPI_NSSInternalSoft_Set         ((uint16_t)0x0100)
#define SPI_NSSInternalSoft_Reset       ((uint16_t)0xFEFF)

struct uvos_spi_cfg {
  SPI_TypeDef     *    regs;
  // uint32_t            remap;         /* @defgroup GPIO_LL_EC_AF Alternate Function */
  LL_SPI_InitTypeDef  init;
  bool                use_crc;
  struct stm32_dma    dma;
  struct stm32_gpio   sclk;
  struct stm32_gpio   miso;
  struct stm32_gpio   mosi;
  uint32_t            slave_count;
  struct stm32_gpio   ssel[];
};

struct uvos_spi_dev {
  const struct        uvos_spi_cfg * cfg;
  void                ( *callback )( uint8_t, uint8_t );
  uint8_t             tx_dummy_byte;
  uint8_t             rx_dummy_byte;
#if defined(UVOS_INCLUDE_FREERTOS)
  xSemaphoreHandle    busy;
#else
  uint8_t             busy;
#endif
};

void SPI_NSSInternalSoftwareConfig( SPI_TypeDef * SPIx, uint16_t SPI_NSSInternalSoft );
void SPI_SSOutputCmd( SPI_TypeDef * SPIx, FunctionalState NewState );

extern int32_t UVOS_SPI_Init( uint32_t * spi_id, const struct uvos_spi_cfg * cfg );



