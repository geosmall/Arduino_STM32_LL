#ifndef UVOS_FLASHFS_H
#define UVOS_FLASHFS_H

#include <stdint.h>

struct UVOS_FLASHFS_Stats {
  uint16_t num_free_slots; /* slots in free state */
  uint16_t num_active_slots; /* slots in active state */
};

// define logfs subdirectory of flash device
#define UVOS_LOGFS_DIR "logfs"

int32_t UVOS_FLASHFS_Format( uintptr_t fs_id );
int32_t UVOS_FLASHFS_ObjSave( uintptr_t fs_id, uint32_t obj_id, uint16_t obj_inst_id, uint8_t * obj_data, uint16_t obj_size );
int32_t UVOS_FLASHFS_ObjLoad( uintptr_t fs_id, uint32_t obj_id, uint16_t obj_inst_id, uint8_t * obj_data, uint16_t obj_size );
int32_t UVOS_FLASHFS_ObjDelete( uintptr_t fs_id, uint32_t obj_id, uint16_t obj_inst_id );
int32_t UVOS_FLASHFS_GetStats( uintptr_t fs_id, struct UVOS_FLASHFS_Stats * stats );

#endif /* UVOS_FLASHFS_H */