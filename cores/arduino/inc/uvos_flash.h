#ifndef UVOS_FLASH_H
#define UVOS_FLASH_H

#include <stdint.h>

struct uvos_flash_chunk {
  uint8_t * addr;
  uint32_t len;
};

struct uvos_flash_driver {
  int32_t ( *start_transaction )( uintptr_t flash_id );
  int32_t ( *end_transaction )( uintptr_t flash_id );
  int32_t ( *erase_chip )( uintptr_t flash_id );
  int32_t ( *erase_sector )( uintptr_t flash_id, uint32_t addr );
  int32_t ( *write_data )( uintptr_t flash_id, uint32_t addr, uint8_t * data, uint16_t len );
  int32_t ( *write_chunks )( uintptr_t flash_id, uint32_t addr, struct uvos_flash_chunk chunks[], uint32_t num_chunks );
  int32_t ( *rewrite_data )( uintptr_t flash_id, uint32_t addr, uint8_t * data, uint16_t len );
  int32_t ( *rewrite_chunks )( uintptr_t flash_id, uint32_t addr, struct uvos_flash_chunk chunks[], uint32_t num_chunks );
  int32_t ( *read_data )( uintptr_t flash_id, uint32_t addr, uint8_t * data, uint16_t len );
};

#endif /* UVOS_FLASH_H */