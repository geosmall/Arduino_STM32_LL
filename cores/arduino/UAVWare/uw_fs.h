#ifndef UW_FS_H
#define UW_FS_H

#include <stdint.h>
#include <stdbool.h>

#define FS_ERR_OK                          0L /*!< OK */
#define FS_ERR_FAILED                     -1L /*!< Requested functionality or process failed. */
#define FS_ERR_RANGE                      -2l /*!< Request out of range */

#ifdef __cplusplus
extern "C" {
#endif

extern int UW_fs_init( void );
extern int UW_fs_read_file( const char * srcPath, uint8_t * buf, size_t bufSize );
extern int UW_fs_write_file( const char * filePath , const uint8_t * buf, size_t bufSize );
extern int UW_fs_format( void );
extern int UW_fs_mount( void );
extern int UW_fs_unmount( void );
extern int UW_fs_dir( const char * path );
extern int UW_fs_file_size( const char * filePath );
extern int UW_fs_remove_file( const char * filePath );
extern int UW_fs_run_benchmark( void );
extern int UW_fs_print_hex_file( const char * filePath );

#ifdef __cplusplus
}
#endif

#endif /* UW_FS_H */
