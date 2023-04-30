#ifndef UW_FS_OLD_H
#define UW_FS_OLD_H

#include <stdint.h>
#include <stdbool.h>

#define FS_OLD_ERR_OK                          0L /*!< OK */
#define FS_OLD_ERR_FAILED                     -1L /*!< Requested functionality or process failed. */
#define FS_OLD_ERR_RANGE                      -2l /*!< Request out of range */

#ifdef __cplusplus
extern "C" {
#endif

extern int UW_fs_old_init( void );
extern int UW_fs_old_read_file( const char * srcPath, uint8_t * buf, size_t bufSize );
extern int UW_fs_old_write_file( const char * filePath , const uint8_t * buf, size_t bufSize );
extern int UW_fs_old_format( void );
extern int UW_fs_old_mount( void );
extern int UW_fs_old_unmount( void );
extern int UW_fs_old_dir( const char * path );
extern int UW_fs_old_file_size( const char * filePath );
extern int UW_fs_old_remove_file( const char * filePath );
extern int UW_fs_old_run_benchmark( void );
extern int UW_fs_old_print_hex_file( const char * filePath );

#ifdef __cplusplus
}
#endif

#endif /* UW_FS_OLD_H */
