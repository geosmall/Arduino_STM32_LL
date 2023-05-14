#ifndef UW_FS_H
#define UW_FS_H

#include <stdint.h>
#include <stdbool.h>
#include "uvos_fs.h"

// A POSIX-like file system interface that can be implemented by different file systems
// such as FatFS and LittleFS. The interface provides basic operations for mounting,
// unmounting, opening, reading, writing, seeking, closing, and deleting files,
// as well as listing and creating directories.

#ifdef __cplusplus
extern "C" {
#endif

/* POSIX fopen(3) — Linux manual pag
   https://man7.org/linux/man-pages/man3/fopen.3.html

   FILE *fopen(const char *restrict pathname, const char *restrict mode);

   The argument [mode] points to a string beginning with one of the
   following sequences (possibly followed by additional characters,
   as described below):

   r      Open text file for reading.  The stream is positioned at
          the beginning of the file.

   r+     Open for reading and writing.  The stream is positioned at
          the beginning of the file.

   w      Truncate file to zero length or create text file for
          writing.  The stream is positioned at the beginning of the
          file.

   w+     Open for reading and writing.  The file is created if it
          does not exist, otherwise it is truncated.  The stream is
          positioned at the beginning of the file.

   a      Open for appending (writing at end of file).  The file is
          created if it does not exist.  The stream is positioned at
          the end of the file.

   a+     Open for reading and appending (writing at end of file).
          The file is created if it does not exist.  Output is
          always appended to the end of the file.  POSIX is silent
          on what the initial read position is when using this mode.
          For glibc, the initial file position for reading is at the
          beginning of the file, but for Android/BSD/MacOS, the
          initial file position for reading is at the end of the
          file.
*/

// Storage volume information structure
typedef struct {
  uint32_t vol_total_Kbytes;
  uint32_t vol_free_Kbytes;
  uvos_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
} UW_fs_vol_info_t;

// Possible error codes, these are negative to allow
// valid positive return values
typedef enum {
  FS_ERR_OK                        =   0, /*!< OK */
  FS_ERR_MOUNT_FAILED              =  -1, /*!< Attempt to mount File system not failed */
  FS_ERR_NOT_VALID                 =  -2, /*!< File system not mounted or invalid */
  FS_ERR_FAILED                    =  -3, /*!< Requested functionality or process failed. */
  FS_ERR_NO_VOL_INFO               =  -4, /*!< Could not retieve volume info */
  FS_ERR_RANGE                     =  -5, /*!< Request out of range */
} fs_error_t;

struct uvos_fs_file typedef UW_fs_file_t;
struct uvos_fs_dir typedef UW_fs_dir_t;
struct uvos_file_info typedef UW_fs_file_info_t;


// Initialize and mount a logical file system
// Returns 0 on success, or negative error code on failure
extern int UW_fs_init( void );

/* Check that file system is mounted and of valid type */
// Returns true on success, or false on failure
extern bool UW_fs_is_valid( void );

// Get volume info (total and free Kbytes)
// Returns 0 on success, or negative error code on failure
extern int UW_fs_get_vol_info( UW_fs_vol_info_t *vol_info );

// Reads in a file from file system into provided buffer
// Returns 0 on success, or negative error code on failure
extern int UW_fs_read_file( const char *srcPath, uint8_t *buf, size_t bufSize );

// Writrs a file to file system from provided buffer
// Returns 0 on success, or -1 on failure
extern int UW_fs_write_file( const char *filePath , const uint8_t *buf, size_t bufSize );

// Delete a file or directory
// Returns 0 on success, or negative error code on failure
extern int UW_fs_remove( const char *path );

// Open a directory for listing its contents
// Returns 0 on success, or negative error code on failure
extern int UW_fs_dir_open( UW_fs_dir_t *dir, const char *path );

// Close a directory
// Returns 0 on success, or negative error code on failure
extern int UW_fs_dir_close( UW_fs_dir_t *dir );

// Read the next entry in a directory
// Returns a positive value on success, 0 at the end of directory, or a negative error code on failure
extern int UW_fs_dir_read( UW_fs_dir_t *dir, UW_fs_file_info_t *dir_info );

// Create a directory
// Returns 0 on success, or negative error code on failure
extern int UW_fs_mkdir( const char *path );

#ifdef __cplusplus
}
#endif

#endif // UW_FS_H