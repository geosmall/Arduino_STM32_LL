#ifndef UVOS_FLASH_JEDEC_H
#define UVOS_FLASH_JEDEC_H

#include "uvos_flash.h" /* API definition for flash drivers */

extern const struct uvos_flash_driver uvos_jedec_flash_driver;

#define JEDEC_MANUFACTURER_ST       0x20
#define JEDEC_MANUFACTURER_MICRON   0x20
#define JEDEC_MANUFACTURER_NUMORIX  0x20
#define JEDEC_MANUFACTURER_MACRONIX 0xC2
#define JEDEC_MANUFACTURER_WINBOND  0xEF

struct uvos_flash_jedec_cfg {
  uint8_t expect_manufacturer;
  uint8_t expect_memorytype;
  uint8_t expect_capacity;
  uint8_t sector_erase;
  uint8_t chip_erase;
  uint8_t fast_read;
  uint8_t fast_read_dummy_bytes;
};

int32_t UVOS_Flash_Jedec_Init( uintptr_t * flash_id, uint32_t spi_id, uint32_t slave_num );
int32_t UVOS_Flash_Jedec_EraseChip( uintptr_t flash_id );
int32_t UVOS_Flash_Jedec_EraseSector( uintptr_t flash_id, uint32_t addr );
int32_t UVOS_Flash_Jedec_WriteChunks( uintptr_t flash_id, uint32_t addr, struct uvos_flash_chunk chunks[], uint32_t num );
int32_t UVOS_Flash_Jedec_WriteData( uintptr_t flash_id, uint32_t addr, uint8_t * data, uint16_t len );
int32_t UVOS_Flash_Jedec_ReadData( uintptr_t flash_id, uint32_t addr, uint8_t * data, uint16_t len );

#endif /* UVOS_FLASH_JEDEC_H */
