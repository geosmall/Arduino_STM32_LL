#ifndef UVOS_FS_H
#define UVOS_FS_H

#include <uvos.h>
#include <ff.h>
#include <lfs.h>

// Define null terminated UVOS file name size as [8].[3] + /0 = 13
#define UVOS_FILE_NAME_Z 13

// define logfs subdirectory of flash device
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

// Define file object compatible with both FatFS and LittleFS
typedef struct {
  // Union of FIL and lfs_file_t must be first member of struct
  union {
    FIL fatfile; // FatFS file object
    lfs_file_t lfsfile; // LittleFS file object
  };
  uvos_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
} uvos_fs_file_t;

// Define dir object compatible with both FatFS and LittleFS
typedef struct {
  // Union of DIR and lfs_dir_t must be first member of struct
  union {
    DIR fatdir; // FatFS dir object
    lfs_dir_t lfsdir; // LittleFS dir object
  };
  uvos_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
} uvos_fs_dir_t;

// Define file or directory information structure
typedef struct {
  char name[UVOS_FILE_NAME_Z]; // The name of the file or directory
  uint64_t size; // Size of the file, only valid for files (not dirs)
  bool is_dir; // Whether the entry is a directory or not
} uvos_file_info_t;

struct uvos_fs_driver {
  int32_t ( *mount_fs )( void );
  int32_t ( *unmount_fs )( void );
  bool ( *is_mounted )( void );
  int32_t ( *get_vol_info )( uvos_fs_vol_info_t *vol_info );
  int32_t ( *file_open )( uvos_fs_file_t *fp, const char *path, uvos_fopen_mode_t mode );
  int32_t ( *file_read )( uvos_fs_file_t *fp, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read );
  int32_t ( *file_write )( uvos_fs_file_t *fp, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written );
  int32_t ( *file_seek )( uvos_fs_file_t *fp, int32_t offset );
  uint32_t ( *file_tell )( uvos_fs_file_t *fp );
  int32_t ( *file_close )( uvos_fs_file_t *fp );
  int32_t ( *file_remove )( const char *path );
  int32_t ( *dir_open )( uvos_fs_dir_t *dp, const char *path );
  int32_t ( *dir_close )( uvos_fs_dir_t *dp );
  int32_t ( *dir_read )( uvos_fs_dir_t *dp, uvos_file_info_t *file_info );
};

/* Public Functions */
// extern int32_t UVOS_FS_Init( uintptr_t *fs_driver );
extern int32_t UVOS_FS_Init( const struct uvos_fs_driver *fs_driver );

#endif /* UVOS_FS_H */