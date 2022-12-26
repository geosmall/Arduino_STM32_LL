#ifndef UVOS_FLASHFS_LOGFS_PRIV_H
#define UVOS_FLASHFS_LOGFS_PRIV_H

#include <stdint.h>
#include "uvos_flash.h" /* struct uvos_flash_driver */

struct flashfs_logfs_cfg {
  uint32_t fs_magic;
  uint32_t total_fs_size; /* Total size of all generations of the filesystem */
  uint32_t arena_size; /* Max size of one generation of the filesystem */
  uint32_t slot_size; /* Max size of a "file" */

  uint32_t start_offset; /* Offset into flash where this filesystem starts */
  uint32_t sector_size; /* Size of a flash erase block */
  uint32_t page_size; /* Maximum flash burst write size */
};

int32_t UVOS_FLASHFS_Logfs_Init( uintptr_t * fs_id, const struct flashfs_logfs_cfg * cfg, const struct uvos_flash_driver * driver, uintptr_t flash_id );

int32_t UVOS_FLASHFS_Logfs_Destroy( uintptr_t fs_id );

#endif /* UVOS_FLASHFS_LOGFS_PRIV_H */
