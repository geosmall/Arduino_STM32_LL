#ifndef UVOS_FLASHFS_OBJLIST_H
#define UVOS_FLASHFS_OBJLIST_H

#include "uavw.h"
#include "uavobjectmanager.h"
#include <uvos_flashfs.h>
struct flashfs_cfg {
  uint32_t table_magic;
  uint32_t obj_magic;
  uint32_t obj_table_start;
  uint32_t obj_table_end;
  uint32_t sector_size;
  uint32_t chip_size;
};

int32_t UVOS_FLASHFS_Init( uintptr_t * fs_id, const struct flashfs_cfg * cfg, const struct uvos_flash_driver * driver, uintptr_t flash_id );

#endif /* UVOS_FLASHFS_OBJLIST_H */
