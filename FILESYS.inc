c
// A header file that uses an array of function pointers to provide a file system interface
#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H

// Define the file system operations
typedef struct {
  int (*open)(const char *path, int flags); // Open a file
  int (*close)(int fd); // Close a file
  int (*read)(int fd, void *buf, size_t count); // Read from a file
  int (*write)(int fd, const void *buf, size_t count); // Write to a file
  int (*lseek)(int fd, off_t offset, int whence); // Move the file pointer
  int (*stat)(const char *path, struct stat *buf); // Get file status
  int (*unlink)(const char *path); // Delete a file
} fs_ops_t;

// Declare an array of function pointers for the file system interface
extern fs_ops_t fs_ops[];

// Define some constants for the array indices
#define FS_OPEN 0
#define FS_CLOSE 1
#define FS_READ 2
#define FS_WRITE 3
#define FS_LSEEK 4
#define FS_STAT 5
#define FS_UNLINK 6

#endif // FILE_SYSTEM_H


// ==========================================================================================

/**
 * @brief An enum that translates from POSIX fopen mode flags to LittleFS lfs_file_open modes.
 *
 * This enum can be used to convert a string of fopen mode flags (such as "r+", "w", "a+") to
 * an integer value that can be passed to the lfs_file_open function of the LittleFS library.
 * The enum values are bitwise ORed combinations of the LFS_O_RDONLY, LFS_O_WRONLY, LFS_O_RDWR,
 * LFS_O_CREAT, LFS_O_EXCL, and LFS_O_APPEND constants defined by LittleFS.
 *
 * For example, to open a file for reading and writing, creating it if it does not exist,
 * one can use the following code:
 *
 *     #include "lfs.h"
 *     #include "fopen_modes.h"
 *
 *     lfs_t lfs; // The LittleFS filesystem object
 *     lfs_file_t file; // The LittleFS file object
 *     const char* path = "/foo/bar.txt"; // The path of the file to open
 *     const char* mode = "w+"; // The fopen mode string
 *
 *     // Convert the mode string to an enum value
 *     fopen_mode_t fopen_mode = fopen_mode_from_string(mode);
 *
 *     // Open the file using the enum value
 *     int err = lfs_file_open(&lfs, &file, path, fopen_mode);
 *
 */
typedef enum {
    FOPEN_MODE_R   = LFS_O_RDONLY,              // Open for reading only
    FOPEN_MODE_W   = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, // Open for writing only, create or truncate
    FOPEN_MODE_A   = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND, // Open for writing only, create or append
    FOPEN_MODE_RP  = LFS_O_RDWR,                // Open for reading and writing
    FOPEN_MODE_WP  = LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC, // Open for reading and writing, create or truncate
    FOPEN_MODE_AP  = LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND, // Open for reading and writing, create or append
    FOPEN_MODE_RX  = LFS_O_RDONLY | LFS_O_EXCL, // Open for reading only, fail if exists
    FOPEN_MODE_WX  = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL, // Open for writing only, fail if exists
    FOPEN_MODE_AX  = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL | LFS_O_APPEND, // Open for writing only, fail if exists or append
    FOPEN_MODE_RPX = LFS_O_RDWR   | LFS_O_EXCL, // Open for reading and writing, fail if exists
    FOPEN_MODE_WPX = LFS_O_RDWR   | LFS_O_CREAT | LFS_O_EXCL, // Open for reading and writing, fail if exists or truncate
    FOPEN_MODE_APX = LFS_O_RDWR   | LFS_O_CREAT | LFS_O_EXCL | LFS_O_APPEND  // Open for reading and writing, fail if exists or append
} fopen_mode_t;

/**
 * @brief A function that converts a string of fopen mode flags to an enum value.
 *
 * This function takes a null-terminated string of fopen mode flags (such as "r+", "w", "a+")
 * and returns the corresponding enum value defined by fopen_mode_t. If the string is invalid,
 * the function returns -1.
 *
 * @param mode The string of fopen mode flags.
 * @return The enum value of the mode flags or -1 if invalid.
 */
fopen_mode_t fopen_mode_from_string(const char* mode);

//===============================================================================

// A minimal POSIX file system API in the style of a C header file
// suitable for abstracting both FatFS and LittleFS.

// A file system object that holds the information of a mounted file system
typedef struct {
  union {
    FATFS fatfs; // FatFS file system object
    lfs_t lfs;   // LittleFS file system object
  };
  int type; // 0 for FatFS, 1 for LittleFS
} fs_t;

// A file object that holds the information of an opened file
typedef struct {
  union {
    FIL fatfile; // FatFS file object
    lfs_file_t lfsfile; // LittleFS file object
  };
  int type; // 0 for FatFS, 1 for LittleFS
} file_t;

// An enum for flags that specify access mode and open mode
// suitable for use with both FatFS f_open and LittleFS lfs_file_open
typedef enum {
  FS_READ = 0x01,   // Open for reading
  FS_WRITE = 0x02,  // Open for writing
  FS_RDWR = 0x03,   // Open a file as read and write
  FS_CREATE = 0x04, // Create a new file if not exist
  FS_APPEND = 0x08, // Move to the end of the file on open
  FS_TRUNC = 0x10   // Truncate the file on open
} fs_flags_t;

// A function that mounts a file system to a logical drive
int fs_mount(fs_t *fs, const char *path, int type);

// A function that unmounts a file system from a logical drive
int fs_unmount(fs_t *fs);

// A function that opens a file with specified flags
int fs_open(file_t *file, fs_t *fs, const char *path, fs_flags_t flags);

// A function that closes an opened file
int fs_close(file_t *file);

// A function that reads data from an opened file
int fs_read(file_t *file, void *buff, size_t size, size_t *read);

// A function that writes data to an opened file
int fs_write(file_t *file, const void *buff, size_t size, size_t *written);

// A function that moves the file pointer of an opened file
int fs_seek(file_t *file, size_t offset);

// A function that returns the current position of the file pointer of an opened file
size_t fs_tell(file_t *file);

// A function that returns the size of an opened file
size_t fs_size(file_t *file);

// A function that checks if the end of an opened file is reached
int fs_eof(file_t *file);

//===============================================================================

// This is a file system module file in C that works with both FatFS and LittleFS
// It uses void pointers to the file system structures

#include "fs.h" // The header file for this module
#include "ff.h" // The header file for FatFS
#include "lfs.h" // The header file for LittleFS

// Define the file system types
#define FS_TYPE_FATFS 1
#define FS_TYPE_LITTLEFS 2

// Define the file system structure
typedef struct {
    int type; // The type of the file system (FS_TYPE_FATFS or FS_TYPE_LITTLEFS)
    void *fs; // A pointer to the file system structure (FATFS or lfs_t)
} fs_t;

// Define the file structure
typedef struct {
    int type; // The type of the file system (FS_TYPE_FATFS or FS_TYPE_LITTLEFS)
    void *file; // A pointer to the file structure (FIL or lfs_file_t)
} fs_file_t;

// Initialize a file system object
int fs_init(fs_t *fs, int type) {
    if (fs == NULL) return -1; // Check if the pointer is valid
    fs->type = type; // Set the type of the file system
    if (type == FS_TYPE_FATFS) { // If it is FatFS
        fs->fs = malloc(sizeof(FATFS)); // Allocate memory for the FATFS structure
        if (fs->fs == NULL) return -2; // Check if the allocation succeeded
        return f_mount((FATFS *)fs->fs, "", 1); // Mount the FatFS volume
    } else if (type == FS_TYPE_LITTLEFS) { // If it is LittleFS
        fs->fs = malloc(sizeof(lfs_t)); // Allocate memory for the lfs_t structure
        if (fs->fs == NULL) return -2; // Check if the allocation succeeded
        return lfs_mount((lfs_t *)fs->fs, NULL); // Mount the LittleFS volume with default configuration
    } else {
        return -3; // Invalid file system type
    }
}

// Deinitialize a file system object
int fs_deinit(fs_t *fs) {
    if (fs == NULL) return -1; // Check if the pointer is valid
    int res = 0; // The result of the operation
    if (fs->type == FS_TYPE_FATFS) { // If it is FatFS
        res = f_unmount(""); // Unmount the FatFS volume
        free(fs->fs); // Free the memory for the FATFS structure
    } else if (fs->type == FS_TYPE_LITTLEFS) { // If it is LittleFS
        res = lfs_unmount((lfs_t *)fs->fs); // Unmount the LittleFS volume
        free(fs->fs); // Free the memory for the lfs_t structure
    } else {
        res = -3; // Invalid file system type
    }
    fs->type = 0; // Reset the type of the file system
    fs->fs = NULL; // Reset


// File system module file in C that works with both FatFS and LittleFS

#include "ff.h" // FatFS header file
#include "lfs.h" // LittleFS header file

// Define the file system object
typedef struct {
  union {
    FATFS fatfs; // FatFS object
    lfs_t lfs; // LittleFS object
  };
  int type; // File system type: 0 for FatFS, 1 for LittleFS
} fs_t;

// Define the file object
typedef struct {
  union {
    FIL fatfile; // FatFS file object
    lfs_file_t lfsfile; // LittleFS file object
  };
  int type; // File system type: 0 for FatFS, 1 for LittleFS
} file_t;

// Define the common file system functions
int fs_mount(fs_t *fs, const char *path, int type); // Mount a file system
int fs_unmount(fs_t *fs); // Unmount a file system
int fs_open(file_t *file, fs_t *fs, const char *path, int mode); // Open a file
int fs_close(file_t *file); // Close a file
int fs_read(file_t *file, void *buff, UINT btr, UINT *br); // Read from a file
int fs_write(file_t *file, const void *buff, UINT btw, UINT *bw); // Write to a file
int fs_lseek(file_t *file, FSIZE_t ofs); // Move the file pointer
int fs_truncate(file_t *file); // Truncate the file
int fs_sync(file_t *file); // Flush the cached data of the writing file

// Implement the common file system functions using FatFS or LittleFS functions depending on the type

int fs_mount(fs_t *fs, const char *path, int type) {
  int res;
  if (type == 0) { // FatFS
    res = f_mount(&fs->fatfs, path, 1); // Mount a logical drive with force mount option
  } else if (type == 1) { // LittleFS
    // Define the configuration structure for LittleFS
    static const struct lfs_config cfg = {
      // TODO: fill in the configuration parameters such as read/write/erase/prog functions and block size/count/cycle
    };
    res = lfs_mount(&fs->lfs, &cfg); // Mount a file system with the given configuration
  } else {
    res = -1; // Invalid type
  }
  fs->type = type; // Set the file system type
  return res;
}

int fs_unmount(fs_t *fs) {
  int res;
  if (fs->type == 0) { // FatFS
    res = f_unmount(""); // Unmount a logical drive
  } else if (fs->type == 1) { // LittleFS
    res = lfs_unmount(&fs->lfs); // Unmount a file system
  } else {
    res = -1; // Invalid type
  }
  return res;
}

int fs_open(file_t *file, fs_t *fs, const char *path, int mode) {
  int res;
  if (fs->type == 0) { // FatFS
    res = f_open(&file->fatfile, path, mode); // Open or create a file with the given mode
  } else if (fs->type == 1) { // LittleFS
    res = lfs_file_open(&fs->lfs, &file->lfsfile, path, mode); // Open or create a file with the given mode
  } else {
    res = -1; // Invalid type
  }
  file->type = fs->type; // Set the file system type
  return res;
}

int fs_close(file_t *file) {
  int res;
  if (file->type == 0) { // FatFS
    res = f_close(&file->fatfile); // Close an open file object
  } else if (file->type == 1) { // LittleFS
    res = lfs_file_close(&file->lfsfile); // Close an open file object
  } else {
    res = -1




// Write me a minimal POSIX file system API in the style of a C header file suitable for abstracting both FatFS and LittleFS

/* A minimal POSIX file system API in the style of a C header file */

#ifndef MIN_FS_H
#define MIN_FS_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* File descriptor operations */
int open(const char *path, int flags, ...); /* Open a file */
int close(int fd); /* Close a file */
ssize_t read(int fd, void *buf, size_t count); /* Read from a file */
ssize_t write(int fd, const void *buf, size_t count); /* Write to a file */
off_t lseek(int fd, off_t offset, int whence); /* Move the file pointer */

/* File status operations */
int fstat(int fd, struct stat *buf); /* Get file status by file descriptor */
int stat(const char *path, struct stat *buf); /* Get file status by file name */
int access(const char *path, int mode); /* Check file access permissions */

/* Directory operations */
int mkdir(const char *path, mode_t mode); /* Create a directory */
int rmdir(const char *path); /* Remove a directory */
DIR *opendir(const char *name); /* Open a directory stream */
struct dirent *readdir(DIR *dirp); /* Read a directory entry */
int closedir(DIR *dirp); /* Close a directory stream */

#endif /* MIN_FS_H */

// Write me a minimal POSIX file system API in the style of a C header file suitable for abstracting both FatFS and LittleFS

#ifndef POSIX_FS_H
#define POSIX_FS_H

#include <stdint.h>
#include <stdbool.h>

// A POSIX file system interface that can be implemented by different file systems
// such as FatFS and LittleFS. The interface provides basic operations for opening,
// reading, writing, seeking, closing, and deleting files, as well as listing and
// creating directories.

// A file handle type
typedef void* posix_file_t;

// A directory handle type
typedef void* posix_dir_t;

// A file or directory information structure
typedef struct {
  char name[256]; // The name of the file or directory
  uint32_t size; // The size of the file in bytes
  bool is_dir; // Whether the entry is a directory or not
} posix_info_t;

// Open a file with a given mode (r, w, a, r+, w+, a+)
// Returns a file handle on success, or NULL on failure
posix_file_t posix_open(const char* path, const char* mode);

// Read data from a file into a buffer
// Returns the number of bytes read, or -1 on failure
int posix_read(posix_file_t file, void* buf, int size);

// Write data from a buffer into a file
// Returns the number of bytes written, or -1 on failure
int posix_write(posix_file_t file, const void* buf, int size);

// Seek to a position in a file
// Returns 0 on success, or -1 on failure
int posix_seek(posix_file_t file, int offset, int whence);

// Get the current position in a file
// Returns the position, or -1 on failure
int posix_tell(posix_file_t file);

// Close a file
// Returns 0 on success, or -1 on failure
int posix_close(posix_file_t file);

// Delete a file
// Returns 0 on success, or -1 on failure
int posix_remove(const char* path);

// Open a directory for listing its contents
// Returns a directory handle on success, or NULL on failure
posix_dir_t posix_opendir(const char* path);

// Read the next entry in a directory
// Returns a pointer to an info structure on success, or NULL on failure or end of directory
posix_info_t* posix_readdir(posix_dir_t dir);

// Close a directory
// Returns 0 on success, or -1 on failure
int posix_closedir(posix_dir_t dir);

// Create a directory
// Returns 0 on success, or -1 on failure
int posix_mkdir(const char* path);

#endif // POSIX_FS_H


/*--------------------------------------------------------------*/
/* POSIX Module Application Interface                           */
/*--------------------------------------------------------------*/

int open (const char *file, int flags, int mode)
int close (int fd)
int read (int fd, void *buf, size_t len)
int write (int fd, const void *buf, size_t len)
off_t lseek (int fd, off_t offset, int whence)
int rename (const char *old, const char *new)
int unlink (const char *pathname)
int stat (const char *file, struct stat *buf)
int fstat (int fildes, struct stat *buf)
int statfs (const char *path, struct statfs *buf)
int mkdir (const char *path, mode_t mode)
int rmdir (const char *pathname)
DIR * opendir (const char *name)
struct dirent * readdir (DIR *d)
long telldir (DIR *d)
void seekdir (DIR *d, off_t offset)
void rewinddir (DIR *d)
int closedir (DIR *d)
char * getcwd (char *buf, size_t size)


/*--------------------------------------------------------------*/
/* FatFs Module Application Interface                           */
/*--------------------------------------------------------------*/

FRESULT f_open (FIL* fp, const TCHAR* path, BYTE mode);				/* Open or create a file */
FRESULT f_close (FIL* fp);											/* Close an open file object */
FRESULT f_read (FIL* fp, void* buff, UINT btr, UINT* br);			/* Read data from the file */
FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw);	/* Write data to the file */
FRESULT f_lseek (FIL* fp, FSIZE_t ofs);								/* Move file pointer of the file object */
FRESULT f_truncate (FIL* fp);										/* Truncate the file */
FRESULT f_sync (FIL* fp);											/* Flush cached data of the writing file */
FRESULT f_opendir (DIR* dp, const TCHAR* path);						/* Open a directory */
FRESULT f_closedir (DIR* dp);										/* Close an open directory */
FRESULT f_readdir (DIR* dp, FILINFO* fno);							/* Read a directory item */
FRESULT f_findfirst (DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern);	/* Find first file */
FRESULT f_findnext (DIR* dp, FILINFO* fno);							/* Find next file */
FRESULT f_mkdir (const TCHAR* path);								/* Create a sub directory */
FRESULT f_unlink (const TCHAR* path);								/* Delete an existing file or directory */
FRESULT f_rename (const TCHAR* path_old, const TCHAR* path_new);	/* Rename/Move a file or directory */
FRESULT f_stat (const TCHAR* path, FILINFO* fno);					/* Get file status */
FRESULT f_chmod (const TCHAR* path, BYTE attr, BYTE mask);			/* Change attribute of a file/dir */
FRESULT f_utime (const TCHAR* path, const FILINFO* fno);			/* Change timestamp of a file/dir */
FRESULT f_chdir (const TCHAR* path);								/* Change current directory */
FRESULT f_chdrive (const TCHAR* path);								/* Change current drive */
FRESULT f_getcwd (TCHAR* buff, UINT len);							/* Get current directory */
FRESULT f_getfree (const TCHAR* path, DWORD* nclst, FATFS** fatfs);	/* Get number of free clusters on the drive */
FRESULT f_getlabel (const TCHAR* path, TCHAR* label, DWORD* vsn);	/* Get volume label */
FRESULT f_setlabel (const TCHAR* label);							/* Set volume label */
FRESULT f_forward (FIL* fp, UINT(*func)(const BYTE*,UINT), UINT btf, UINT* bf);	/* Forward data to the stream */
FRESULT f_expand (FIL* fp, FSIZE_t fsz, BYTE opt);					/* Allocate a contiguous block to the file */
FRESULT f_mount (FATFS* fs, const TCHAR* path, BYTE opt);			/* Mount/Unmount a logical drive */
FRESULT f_mkfs (const TCHAR* path, const MKFS_PARM* opt, void* work, UINT len);	/* Create a FAT volume */
FRESULT f_fdisk (BYTE pdrv, const LBA_t ptbl[], void* work);		/* Divide a physical drive into some partitions */
FRESULT f_setcp (WORD cp);											/* Set current code page */
int f_putc (TCHAR c, FIL* fp);										/* Put a character to the file */
int f_puts (const TCHAR* str, FIL* cp);								/* Put a string to the file */
int f_printf (FIL* fp, const TCHAR* str, ...);						/* Put a formatted string to the file */
TCHAR* f_gets (TCHAR* buff, int len, FIL* fp);						/* Get a string from the file */

/* Some API fucntions are implemented as macro */

#define f_eof(fp) ((int)((fp)->fptr == (fp)->obj.objsize))
#define f_error(fp) ((fp)->err)
#define f_tell(fp) ((fp)->fptr)
#define f_size(fp) ((fp)->obj.objsize)
#define f_rewind(fp) f_lseek((fp), 0)
#define f_rewinddir(dp) f_readdir((dp), 0)
#define f_rmdir(path) f_unlink(path)
#define f_unmount(path) f_mount(0, path, 0)


/*--------------------------------------------------------------*/
/* LFS Module Application Interface                             */
/*--------------------------------------------------------------*/

/// LFS Filesystem functions ///

int lfs_format(lfs_t *lfs, const struct lfs_config *config); // Format a block device with the littlefs
int lfs_mount(lfs_t *lfs, const struct lfs_config *config); // Mounts a littlefs
int lfs_unmount(lfs_t *lfs); // Unmounts a littlefs
int lfs_remove(lfs_t *lfs, const char *path); // Removes a file or directory
int lfs_rename(lfs_t *lfs, const char *oldpath, const char *newpath); // Rename or move a file or directory
int lfs_stat(lfs_t *lfs, const char *path, struct lfs_info *info); // Find info about a file or directory
lfs_ssize_t lfs_getattr(lfs_t *lfs, const char *path, uint8_t type, void *buffer, lfs_size_t size); // Get a custom attribute
int lfs_setattr(lfs_t *lfs, const char *path, uint8_t type, const void *buffer, lfs_size_t size); // Set custom attributes
int lfs_removeattr(lfs_t *lfs, const char *path, uint8_t type); // Removes a custom attribute

/// File operations ///

int lfs_file_open(lfs_t *lfs, lfs_file_t *file, const char *path, int flags); // Open a file
int lfs_file_opencfg(lfs_t *lfs, lfs_file_t *file, const char *path, int flags, const struct lfs_file_config *config); // Open a file with extra configuration
int lfs_file_close(lfs_t *lfs, lfs_file_t *file); // Close a file
int lfs_file_sync(lfs_t *lfs, lfs_file_t *file); // Synchronize a file on storage
lfs_ssize_t lfs_file_read(lfs_t *lfs, lfs_file_t *file, void *buffer, lfs_size_t size); // Read data from file
lfs_ssize_t lfs_file_write(lfs_t *lfs, lfs_file_t *file, const void *buffer, lfs_size_t size); // Write data to file
lfs_soff_t lfs_file_seek(lfs_t *lfs, lfs_file_t *file, lfs_soff_t off, int whence); // Change the position of the file
int lfs_file_truncate(lfs_t *lfs, lfs_file_t *file, lfs_off_t size); // Truncates the size of the file to the specified size
lfs_soff_t lfs_file_tell(lfs_t *lfs, lfs_file_t *file); // Return the position of the file
int lfs_file_rewind(lfs_t *lfs, lfs_file_t *file); // Change the position of the file to the beginning of the file
lfs_soff_t lfs_file_size(lfs_t *lfs, lfs_file_t *file); // Return the size of the file

/// Directory operations ///

int lfs_mkdir(lfs_t *lfs, const char *path); // Returns a negative error code on failure.
int lfs_dir_open(lfs_t *lfs, lfs_dir_t *dir, const char *path); // Open a directory
int lfs_dir_close(lfs_t *lfs, lfs_dir_t *dir); // Close a directory
int lfs_dir_read(lfs_t *lfs, lfs_dir_t *dir, struct lfs_info *info); // Read an entry in the directory
int lfs_dir_seek(lfs_t *lfs, lfs_dir_t *dir, lfs_off_t off); // Change the position of the directory
lfs_soff_t lfs_dir_tell(lfs_t *lfs, lfs_dir_t *dir); // Return the position of the directory
int lfs_dir_rewind(lfs_t *lfs, lfs_dir_t *dir); // Change the position of the directory to the beginning of the directory

/// Filesystem-level filesystem operations ///

lfs_ssize_t lfs_fs_size(lfs_t *lfs); // Finds the current size of the filesystem
int lfs_fs_traverse(lfs_t *lfs, int (*cb)(void*, lfs_block_t), void *data); // Traverse through all blocks in use by the filesystem
int lfs_migrate(lfs_t *lfs, const struct lfs_config *cfg); // Attempts to migrate a previous version of littlefs