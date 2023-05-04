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
extern int32_t UVOS_SDCARD_UnmountFS( void );
extern bool UVOS_SDCARD_IsMounted( void );
extern int32_t UVOS_SDCARD_GetVolInfo( uvos_fs_vol_info_t *vol_info );

extern int32_t UVOS_SDCARD_File_Open( uvos_fs_file_t *fp, const char *path, uvos_fopen_mode_t mode );
extern int32_t UVOS_SDCARD_File_Read( uvos_fs_file_t *fp, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read );
extern int32_t UVOS_SDCARD_File_Write( uvos_fs_file_t *fp, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written );
extern int32_t UVOS_SDCARD_File_Seek( uvos_fs_file_t *fp, int32_t offset );
extern uint32_t UVOS_SDCARD_File_Tell( uvos_fs_file_t *fp );
extern int32_t UVOS_SDCARD_File_Close( uvos_fs_file_t *fp );
extern int32_t UVOS_SDCARD_File_Remove( const char *path );

extern int32_t UVOS_SDCARD_Dir_Open( uvos_fs_dir_t *dp, const char *path );
extern int32_t UVOS_SDCARD_Dir_Close( uvos_fs_dir_t *dp );
extern int32_t UVOS_SDCARD_Dir_Read( uvos_fs_dir_t *dp, uvos_file_info_t *file_info );

#endif /* UVOS_SDCARD_H */
