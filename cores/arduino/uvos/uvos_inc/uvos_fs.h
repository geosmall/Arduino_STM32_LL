#ifndef UVOS_FS_H
#define UVOS_FS_H

#include <uvos.h>
#include <ff.h>
#include <lfs.h>

// Define null terminated UVOS file name size as [8].[3] + /0 = 13
#define UVOS_FILE_NAME_Z 13

// Define logfs subdirectory of flash device
#define UVOS_LOGFS_DIR "logfs"

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


// Declare the file system types
typedef enum {
  FS_TYPE_INVALID = 0,
  FS_TYPE_FATFS,
  FS_TYPE_LITTLEFS
} uvos_fs_type_t;

// Storage volume information structure
struct uvos_fs_vol_info {
  uint32_t vol_total_Kbytes;
  uint32_t vol_free_Kbytes;
  uvos_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
};

// Declare file object compatible with both FatFS and LittleFS
struct uvos_fs_file {
  // Union of FIL and lfs_file_t must be first member of struct
  union {
    FIL fatfile; // FatFS file object
    lfs_file_t lfsfile; // LittleFS file object
  };
  uvos_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
};

// Declare dir object compatible with both FatFS and LittleFS
struct uvos_fs_dir {
  // Union of DIR and lfs_dir_t must be first member of struct
  union {
    DIR fatdir; // FatFS dir object
    lfs_dir_t lfsdir; // LittleFS dir object
  };
  uvos_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
};

// Maximum size of a file in bytes is limited by LittleFS implementation.
// LittleFS is limited on disk to <= 4294967296. However, above 2147483647 the
// functions lfs_file_seek, lfs_file_size, and lfs_file_tell will return
// incorrect values due to using signed integers.
#define UVOS_FS_FILE_MAX 2147483647

// Declare file or directory information structure
struct uvos_file_info {
  char name[UVOS_FILE_NAME_Z]; // The name of the file or directory
  uint32_t size; // Size of the file, only valid for files (not dirs) up to UVOS_FS_FILE_MAX
  bool is_dir; // Whether the entry is a directory or not
};

/* Public Functions */
// extern int32_t UVOS_FS_Init( const struct uvos_fs_driver *fs_driver );
extern bool UVOS_FS_IsValid( void );
extern int32_t UVOS_FS_Deinit( void );
extern int32_t UVOS_FS_FileOpen( struct uvos_fs_file *file, const char *path, uvos_fopen_mode_t mode );
extern int32_t UVOS_FS_FileRead( struct uvos_fs_file *file, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read );
extern int32_t UVOS_FS_FileWrite( struct uvos_fs_file *file, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written );
extern int32_t UVOS_FS_FileSeek( struct uvos_fs_file *file, uint32_t offset );
extern int32_t UVOS_FS_FileTell( struct uvos_fs_file *file );
extern int32_t UVOS_FS_FileClose( struct uvos_fs_file *file );
extern int32_t UVOS_FS_FileSize( const char *path );
extern int32_t UVOS_FS_Remove( const char *path );
extern int32_t UVOS_FS_DirOpen( struct uvos_fs_dir *dir, const char *path );
extern int32_t UVOS_FS_DirClose( struct uvos_fs_dir *dir );
extern int32_t UVOS_FS_DirRead( struct uvos_fs_dir *dir, struct uvos_file_info *dir_info );
extern int32_t UVOS_FS_Mkdir( const char *path );
extern int32_t UVOS_FS_GetVolumeInfo( struct uvos_fs_vol_info *info );

#endif /* UVOS_FS_H */