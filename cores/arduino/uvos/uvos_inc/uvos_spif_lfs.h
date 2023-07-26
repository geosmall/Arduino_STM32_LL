#ifndef UVOS_SPIF_H
#define UVOS_SPIF_H

#include "uvos_fs.h"

// SPI Flash volume information structure
typedef struct {
  uint32_t total_sectors;
  uint32_t free_sectors;
  uint32_t vol_total_Kbytes;
  uint32_t vol_free_Kbytes;
} SPIF_vol_info_t;

/* Public Functions */

/* Prototypes */
extern int32_t UVOS_SPIF_Init( uint32_t spi_id );
extern int32_t UVOS_SPIF_Format( void );

#endif /* UVOS_SPIF_H */