#ifndef UVOS_SDCARD_H
#define UVOS_SDCARD_H

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
extern int32_t UVOS_SDCARD_GetVolInfo( SDCARD_vol_info_t *vol_info );

extern int32_t UVOS_SDCARD_Open( FIL *fp, const char *path, int32_t mode );
extern int32_t UVOS_SDCARD_Read( FIL *fp, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read );
extern int32_t UVOS_SDCARD_Write( FIL *fp, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written );
extern int32_t UVOS_SDCARD_Seek( FIL *fp, uint32_t offset );
extern uint32_t UVOS_SDCARD_Tell( FIL *fp );
extern int32_t UVOS_SDCARD_Close( FIL *fp );
extern int32_t UVOS_SDCARD_Remove( const char *path );

#endif /* UVOS_SDCARD_H */
