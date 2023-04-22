#ifndef UVOS_SDCARD_H
#define UVOS_SDCARD_H

#include "uvos_fs_driver.h"

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
extern int32_t UVOS_SDCARD_MountFS( void );
extern bool UVOS_SDCARD_IsMounted( void );
extern int32_t UVOS_SDCARD_GetVolInfo( uvos_fs_vol_info_t *vol_info );

extern int32_t UVOS_SDCARD_Open( uintptr_t *fp, const char *path, uvos_fopen_mode_t mode );
extern int32_t UVOS_SDCARD_Read( uintptr_t *fp, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read );
extern int32_t UVOS_SDCARD_Write( uintptr_t *fp, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written );
extern int32_t UVOS_SDCARD_Seek( uintptr_t *fp, uint32_t offset );
extern uint32_t UVOS_SDCARD_Tell( uintptr_t *fp );
extern int32_t UVOS_SDCARD_Close( uintptr_t *fp );
extern int32_t UVOS_SDCARD_Remove( const char *path );

#endif /* UVOS_SDCARD_H */
