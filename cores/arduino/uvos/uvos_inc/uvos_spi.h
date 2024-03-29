#ifndef UVOS_SPI_H
#define UVOS_SPI_H

/* Global types */
// typedef enum {
//     UVOS_SPI_PRESCALER_2   = 0,
//     UVOS_SPI_PRESCALER_4   = 1,
//     UVOS_SPI_PRESCALER_8   = 2,
//     UVOS_SPI_PRESCALER_16  = 3,
//     UVOS_SPI_PRESCALER_32  = 4,
//     UVOS_SPI_PRESCALER_64  = 5,
//     UVOS_SPI_PRESCALER_128 = 6,
//     UVOS_SPI_PRESCALER_256 = 7
// } SPIPrescalerTypeDef;
typedef enum {
    UVOS_SPI_PRESCALER_2   = LL_SPI_BAUDRATEPRESCALER_DIV2,
    UVOS_SPI_PRESCALER_4   = LL_SPI_BAUDRATEPRESCALER_DIV4,
    UVOS_SPI_PRESCALER_8   = LL_SPI_BAUDRATEPRESCALER_DIV8,
    UVOS_SPI_PRESCALER_16  = LL_SPI_BAUDRATEPRESCALER_DIV16,
    UVOS_SPI_PRESCALER_32  = LL_SPI_BAUDRATEPRESCALER_DIV32,
    UVOS_SPI_PRESCALER_64  = LL_SPI_BAUDRATEPRESCALER_DIV64,
    UVOS_SPI_PRESCALER_128 = LL_SPI_BAUDRATEPRESCALER_DIV128,
    UVOS_SPI_PRESCALER_256 = LL_SPI_BAUDRATEPRESCALER_DIV256
} SPIPrescalerTypeDef;

#define IS_SPI_PRESCALER(INSTANCE) (((INSTANCE) == LL_SPI_BAUDRATEPRESCALER_DIV2)   || \
                                    ((INSTANCE) == LL_SPI_BAUDRATEPRESCALER_DIV4)   || \
                                    ((INSTANCE) == LL_SPI_BAUDRATEPRESCALER_DIV8)   || \
                                    ((INSTANCE) == LL_SPI_BAUDRATEPRESCALER_DIV16)  || \
                                    ((INSTANCE) == LL_SPI_BAUDRATEPRESCALER_DIV32)  || \
                                    ((INSTANCE) == LL_SPI_BAUDRATEPRESCALER_DIV64)  || \
                                    ((INSTANCE) == LL_SPI_BAUDRATEPRESCALER_DIV128) || \
                                    ((INSTANCE) == LL_SPI_BAUDRATEPRESCALER_DIV256))


/* Public Functions */
extern int32_t UVOS_SPI_SetClockSpeed(uint32_t spi_id, SPIPrescalerTypeDef spi_prescaler);
extern int32_t UVOS_SPI_SetClockSpeedHz( uint32_t spi_id, uint32_t spi_speed );
extern int32_t UVOS_SPI_RC_PinSet(uint32_t spi_id, uint32_t slave_id, uint8_t pin_value);
extern int32_t UVOS_SPI_TransferByte(uint32_t spi_id, uint8_t b);
extern int32_t UVOS_SPI_TransferBlock(uint32_t spi_id, const uint8_t *send_buffer, uint8_t *receive_buffer, uint16_t len, void *callback);
// extern int32_t UVOS_SPI_TransferBlock_DMA( uint32_t spi_id, const uint8_t * send_buffer, uint8_t * receive_buffer, uint16_t len, void * callback );
// extern int32_t UVOS_SPI_TransferBlock_PIO( uint32_t spi_id, const uint8_t * send_buffer, uint8_t * receive_buffer, uint16_t len );
extern int32_t UVOS_SPI_Busy(uint32_t spi_id);
extern int32_t UVOS_SPI_ClaimBus(uint32_t spi_id);
extern int32_t UVOS_SPI_ClaimBusISR(uint32_t spi_id, bool *woken);
extern int32_t UVOS_SPI_ReleaseBus(uint32_t spi_id);
extern int32_t UVOS_SPI_ReleaseBusISR(uint32_t spi_id, bool *woken);
extern void    UVOS_SPI_IRQ_Handler(uint32_t spi_id);

#endif // UVOS_SPI_H
