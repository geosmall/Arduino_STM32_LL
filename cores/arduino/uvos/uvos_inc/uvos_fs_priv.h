#ifndef UVOS_FS_PRIV_H
#define UVOS_FS_PRIV_H

#include <uvos.h>
#include <uvos_fs.h>

extern const struct uvos_fs_driver uvos_fs_spif_driver;
extern const struct uvos_fs_driver uvos_fs_sdcard_driver;


typedef enum {
  FATFS_FOPEN_MODE_INVALID = 0,
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
  LFS_FOPEN_MODE_INVALID = 0,
  LFS_FOPEN_MODE_R     = LFS_O_RDONLY,              // Open for reading only
  LFS_FOPEN_MODE_W     = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, // Open for writing only, create or truncate
  // LFS_FOPEN_MODE_W     = LFS_O_WRONLY | LFS_O_CREAT, // Open for writing only, create
  LFS_FOPEN_MODE_A     = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND, // Open for writing only, create or append
  LFS_FOPEN_MODE_RP    = LFS_O_RDWR,                // Open for reading and writing
  LFS_FOPEN_MODE_WP    = LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC, // Open for reading and writing, create or truncate
  LFS_FOPEN_MODE_AP    = LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND, // Open for reading and writing, create or append
  // LFS_FOPEN_MODE_RX  = LFS_O_RDONLY | LFS_O_EXCL, // Open for reading only, fail if exists
  LFS_FOPEN_MODE_WX    = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL, // Open for writing only, fail if exists
  // LFS_FOPEN_MODE_AX  = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL | LFS_O_APPEND, // Open for writing only, fail if exists or append
  // LFS_FOPEN_MODE_RPX = LFS_O_RDWR   | LFS_O_EXCL, // Open for reading and writing, fail if exists
  LFS_FOPEN_MODE_WPX   = LFS_O_RDWR   | LFS_O_CREAT | LFS_O_EXCL, // Open for reading and writing, fail if exists or truncate
  // LFS_FOPEN_MODE_APX = LFS_O_RDWR   | LFS_O_CREAT | LFS_O_EXCL | LFS_O_APPEND  // Open for reading and writing, fail if exists or append
} LittleFS_fopen_mode_t;

#endif /* UVOS_FS_PRIV_H */