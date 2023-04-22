#ifndef UVOS_FLASHFS_H
#define UVOS_FLASHFS_H

#include <stdint.h>
#include "ff.h"
#include "lfs.h"

// define logfs subdirectory of flash device
#define UVOS_LOGFS_DIR "logfs"

// int32_t UVOS_FLASHFS_Format( uintptr_t fs_id );
// int32_t UVOS_FLASHFS_ObjSave( uintptr_t fs_id, uint32_t obj_id, uint16_t obj_inst_id, uint8_t * obj_data, uint16_t obj_size );
// int32_t UVOS_FLASHFS_ObjLoad( uintptr_t fs_id, uint32_t obj_id, uint16_t obj_inst_id, uint8_t * obj_data, uint16_t obj_size );
// int32_t UVOS_FLASHFS_ObjDelete( uintptr_t fs_id, uint32_t obj_id, uint16_t obj_inst_id );
// int32_t UVOS_FLASHFS_GetStats( uintptr_t fs_id, struct UVOS_FLASHFS_Stats * stats );

typedef enum {
  FOPEN_MODE_INVALID = 0,
  FOPEN_MODE_R, // POSIX "r"
  FOPEN_MODE_W, // POSIX "w"
  FOPEN_MODE_A, // POSIX "a"
  FOPEN_MODE_RP, // POSIX "r+"
  FOPEN_MODE_WP,  // POSIX "w+"
  FOPEN_MODE_AP, // POSIX "a+"
  FOPEN_MODE_WX, // POSIX "wx"
  FOPEN_MODE_WPX, // POSIX "w+x"
} uvos_fopen_mode_t;

typedef enum {
  FATFS_FOPEN_MODE_R   = FA_READ, // POSIX "r"
  FATFS_FOPEN_MODE_W   = FA_CREATE_ALWAYS | FA_WRITE, // POSIX "w"
  FATFS_FOPEN_MODE_A   = FA_OPEN_APPEND | FA_WRITE, // POSIX "a"
  FATFS_FOPEN_MODE_RP  = FA_READ | FA_WRITE, // POSIX "r+"
  FATFS_FOPEN_MODE_WP  = FA_CREATE_ALWAYS | FA_WRITE | FA_READ,  // POSIX "w+"
  FATFS_FOPEN_MODE_AP  = FA_OPEN_APPEND | FA_WRITE | FA_READ, // POSIX "a+"
  FATFS_FOPEN_MODE_WX  = FA_CREATE_NEW | FA_WRITE, // POSIX "wx"
  FATFS_FOPEN_MODE_WPX = FA_CREATE_NEW | FA_WRITE | FA_READ // POSIX "w+x"
} FatFS_fopen_mode_t;

typedef enum {
  LITTLEFS_FOPEN_MODE_R   = LFS_O_RDONLY,              // Open for reading only
  LITTLEFS_FOPEN_MODE_W   = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, // Open for writing only, create or truncate
  LITTLEFS_FOPEN_MODE_A   = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND, // Open for writing only, create or append
  LITTLEFS_FOPEN_MODE_RP  = LFS_O_RDWR,                // Open for reading and writing
  LITTLEFS_FOPEN_MODE_WP  = LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC, // Open for reading and writing, create or truncate
  LITTLEFS_FOPEN_MODE_AP  = LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND, // Open for reading and writing, create or append
  // LITTLEFS_FOPEN_MODE_RX  = LFS_O_RDONLY | LFS_O_EXCL, // Open for reading only, fail if exists
  LITTLEFS_FOPEN_MODE_WX  = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL, // Open for writing only, fail if exists
  // LITTLEFS_FOPEN_MODE_AX  = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL | LFS_O_APPEND, // Open for writing only, fail if exists or append
  // LITTLEFS_FOPEN_MODE_RPX = LFS_O_RDWR   | LFS_O_EXCL, // Open for reading and writing, fail if exists
  LITTLEFS_FOPEN_MODE_WPX = LFS_O_RDWR   | LFS_O_CREAT | LFS_O_EXCL, // Open for reading and writing, fail if exists or truncate
  // LITTLEFS_FOPEN_MODE_APX = LFS_O_RDWR   | LFS_O_CREAT | LFS_O_EXCL | LFS_O_APPEND  // Open for reading and writing, fail if exists or append
} LittleFS_fopen_mode_t;

// Define the file system types
typedef enum {
  FS_TYPE_INVALID = 0,
  FS_TYPE_FATFS,
  FS_TYPE_LITTLEFS
} uvos_fs_type_t;

// Storage volume information structure
typedef struct {
  uint32_t vol_total_Kbytes;
  uint32_t vol_free_Kbytes;
  uvos_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
} uvos_fs_vol_info_t;

// Define a file object that works with both FatFS and LittleFS
typedef struct {
  union {
    FIL fatfile; // FatFS file object
    lfs_file_t lfsfile; // LittleFS file object
  };
  uvos_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
} uvos_fs_file_t;

struct uvos_fs_driver_t {
  bool ( *is_mounted )( void );
  int32_t ( *get_vol_info )( uvos_fs_vol_info_t *vol_info );
  int32_t ( *file_open )( uintptr_t *fp, const char *path, uvos_fopen_mode_t mode );
  int32_t ( *file_read )( uintptr_t *fp, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read );
  int32_t ( *file_write )( uintptr_t *fp, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written );
  int32_t ( *file_seek )( uintptr_t *fp, uint32_t offset );
  uint32_t ( *file_tell )( uintptr_t *fp );
  int32_t ( *file_close )( uintptr_t *fp );
  int32_t ( *file_remove )( const char *path );
};

#endif /* UVOS_FLASHFS_H */