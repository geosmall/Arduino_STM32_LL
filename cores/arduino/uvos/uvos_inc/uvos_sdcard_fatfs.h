#ifndef UVOS_SDCARD_H
#define UVOS_SDCARD_H

#include "uvos_fs.h"

// SD Card volume information structure
typedef struct {
  uint32_t total_sectors;
  uint32_t free_sectors;
  uint32_t vol_total_Kbytes;
  uint32_t vol_free_Kbytes;
} SDCARD_vol_info_t;

/* Public Functions */

/* Prototypes */
extern int32_t UVOS_SDCARD_Init( uint32_t spi_id );

#endif /* UVOS_SDCARD_H */
