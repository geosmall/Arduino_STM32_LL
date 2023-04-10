#ifndef UW_FS_H
#define UW_FS_H

#include <stdint.h>
#include <stdbool.h>
#include "ff.h"
#include "lfs.h"

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

#if defined ( UVOS_INCLUDE_SDCARD ) // SD Card uses FatFs
typedef enum {
  FOPEN_MODE_R   = FA_READ, // POSIX "r"
  FOPEN_MODE_W   = FA_CREATE_ALWAYS | FA_WRITE, // POSIX "w"
  FOPEN_MODE_A   = FA_OPEN_APPEND | FA_WRITE, // POSIX "a"
  FOPEN_MODE_RP  = FA_READ | FA_WRITE, // POSIX "r+"
  FOPEN_MODE_WP  = FA_CREATE_ALWAYS | FA_WRITE | FA_READ,  // POSIX "w+"
  FOPEN_MODE_AP  = FA_OPEN_APPEND | FA_WRITE | FA_READ, // POSIX "a+"
  FOPEN_MODE_WX  = FA_CREATE_NEW | FA_WRITE, // POSIX "wx"
  FOPEN_MODE_WPX = FA_CREATE_NEW | FA_WRITE | FA_READ // POSIX "w+x"
} fopen_mode_t;
#elif defined ( UVOS_INCLUDE_FLASH ) // SPI Flash uses LittleFS
typedef enum {
  FOPEN_MODE_R   = LFS_O_RDONLY,              // Open for reading only
  FOPEN_MODE_W   = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, // Open for writing only, create or truncate
  FOPEN_MODE_A   = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND, // Open for writing only, create or append
  FOPEN_MODE_RP  = LFS_O_RDWR,                // Open for reading and writing
  FOPEN_MODE_WP  = LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC, // Open for reading and writing, create or truncate
  FOPEN_MODE_AP  = LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND, // Open for reading and writing, create or append
  // FOPEN_MODE_RX  = LFS_O_RDONLY | LFS_O_EXCL, // Open for reading only, fail if exists
  FOPEN_MODE_WX  = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL, // Open for writing only, fail if exists
  // FOPEN_MODE_AX  = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL | LFS_O_APPEND, // Open for writing only, fail if exists or append
  // FOPEN_MODE_RPX = LFS_O_RDWR   | LFS_O_EXCL, // Open for reading and writing, fail if exists
  FOPEN_MODE_WPX = LFS_O_RDWR   | LFS_O_CREAT | LFS_O_EXCL, // Open for reading and writing, fail if exists or truncate
  // FOPEN_MODE_APX = LFS_O_RDWR   | LFS_O_CREAT | LFS_O_EXCL | LFS_O_APPEND  // Open for reading and writing, fail if exists or append
} fopen_mode_t;
#endif // defined ( UVOS_INCLUDE_SDCARD )

// Define the file system types
typedef enum {
  FS_TYPE_INVALID = 0,
  FS_TYPE_FATFS,
  FS_TYPE_LITTLEFS
} UW_fs_type_t;

// SD Card volume information structure
typedef struct {
  uint32_t vol_total_Kbytes;
  uint32_t vol_free_Kbytes;
} UW_fs_vol_info_t;

// Define the file object
typedef struct {
  union {
    FIL fatfile; // FatFS file object
    lfs_file_t lfsfile; // LittleFS file object
  };
  UW_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
} UW_fs_file_t;

// A file or directory information structure
typedef struct {
  char name[256]; // The name of the file or directory
  uint32_t size; // The size of the file in bytes
  bool is_dir; // Whether the entry is a directory or not
} UW_fs_info_t;


// Initialize and mount a logical file system
// Returns 0 on success, or -1 on failure
extern int UW_fs_init( void );

// Get the mounted FS type
// Returns FS_TYPE_INVALID = 0, FS_TYPE_FATFS = 1 or FS_TYPE_LITTLEFS = 2
extern UW_fs_type_t UW_fs_get_type( void );

// Get volume info (total and free Kbytes)
// Returns 0 on success, or -1 on failure
extern int UW_fs_get_vol_info( UW_fs_vol_info_t *vol_info );

// Open a file in a given mode per fopen_mode_t
// Returns 0 if file open is successful, -1 if unsuccessful
extern int UW_fs_open( UW_fs_file_t *file, const char *path, fopen_mode_t mode );

// Read data from a file into a buffer
// Returns the number of bytes read, or -1 on failure
extern int UW_fs_read( UW_fs_file_t *file, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read );

// Write data from a buffer into a file
// Returns the number of bytes written, or -1 on failure
extern int UW_fs_write( UW_fs_file_t *file, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written );

// Seek to a position in a file
// Returns 0 on success, or -1 on failure
extern int UW_fs_seek( UW_fs_file_t *file, uint32_t offset );

// Get the current position in a file
// Returns the position, or -1 on failure
extern uint32_t UW_fs_tell( UW_fs_file_t *file );

// Close a file
// Returns 0 on success, or -1 on failure
extern int UW_fs_close( UW_fs_file_t *file );

// Delete a file
// Returns 0 on success, or -1 on failure
extern int UW_fs_remove( const char *path );

#if 0 // GLS

// Open a directory for listing its contents
// Returns a directory handle on success, or NULL on failure
extern UW_fs_dir_t UW_fs_opendir( const char *path );

// Read the next entry in a directory
// Returns a pointer to an info structure on success, or NULL on failure or end of directory
extern UW_fs_info_t *UW_fs_readdir( UW_fs_dir_t dir );

// Close a directory
// Returns 0 on success, or -1 on failure
extern int UW_fs_closedir( UW_fs_dir_t dir );

// Create a directory
// Returns 0 on success, or -1 on failure
extern int UW_fs_mkdir( const char *path );

#endif // GLS

#ifdef __cplusplus
}
#endif

#endif // UW_FS_H