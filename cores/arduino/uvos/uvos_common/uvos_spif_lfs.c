#include "uvos.h"
#include "uvos_fs.h"
#include "uvos_fs_priv.h"
#include "uvos_spi_priv.h"
#include "uvos_flash_jedec_priv.h"

#ifdef UVOS_INCLUDE_FLASH

static int32_t UVOS_SPIF_MountFS( void );
static int32_t UVOS_SPIF_UnmountFS( void );
static bool UVOS_SPIF_IsMounted( void );
// static int32_t UVOS_SPIF_Format( void );
static int32_t UVOS_SPIF_GetVolInfo( struct uvos_fs_vol_info *vol_info );

static int32_t UVOS_SPIF_File_Open( struct uvos_fs_file *fp, const char *path, uvos_fopen_mode_t mode );
static int32_t UVOS_SPIF_File_Read( struct uvos_fs_file *fp, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read );
static int32_t UVOS_SPIF_File_Write( struct uvos_fs_file *fp, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written );
static int32_t UVOS_SPIF_File_Seek( struct uvos_fs_file *fp, uint32_t offset );
static int32_t UVOS_SPIF_File_Tell( struct uvos_fs_file *fp );
static int32_t UVOS_SPIF_File_Close( struct uvos_fs_file *fp );
static int32_t UVOS_SPIF_File_Size(  const char *path );

static int32_t UVOS_SPIF_Remove( const char *path );

static int32_t UVOS_SPIF_Dir_Open( struct uvos_fs_dir *dp, const char *path );
static int32_t UVOS_SPIF_Dir_Close( struct uvos_fs_dir *dp );
static int32_t UVOS_SPIF_Dir_Read( struct uvos_fs_dir *dp, struct uvos_file_info *file_info );
static int32_t UVOS_SPIF_Mkdir( const char *path );

/* Provide a file system driver */
const struct uvos_fs_driver uvos_fs_spif_driver = {
  .mount_fs = UVOS_SPIF_MountFS,
  .unmount_fs = UVOS_SPIF_UnmountFS,
  .is_mounted = UVOS_SPIF_IsMounted,
  .get_vol_info = UVOS_SPIF_GetVolInfo,
  .file_open = UVOS_SPIF_File_Open,
  .file_read = UVOS_SPIF_File_Read,
  .file_write = UVOS_SPIF_File_Write,
  .file_seek = UVOS_SPIF_File_Seek,
  .file_tell = UVOS_SPIF_File_Tell,
  .file_close = UVOS_SPIF_File_Close,
  .file_size = UVOS_SPIF_File_Size,
  .remove = UVOS_SPIF_Remove,
  .dir_open = UVOS_SPIF_Dir_Open,
  .dir_close = UVOS_SPIF_Dir_Close,
  .dir_read = UVOS_SPIF_Dir_Read,
  .mkdir = UVOS_SPIF_Mkdir,
};

#define FS_FILE_NAME_SIZE  32 /* Length of file name, used in FS buffers */
#define LFS_PAGE_SIZE 256
#define LFS_LOOKAHEAD_SIZE 512
// #define PRN_BUFFER_SIZE  128

// LittleFS interface functions
static int block_device_read( const struct lfs_config *c, lfs_block_t block,  lfs_off_t off, void *buffer, lfs_size_t size );
static int block_device_prog( const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size );
static int block_device_erase( const struct lfs_config *c, lfs_block_t block );
static int block_device_sync( const struct lfs_config *c );

// UVOS to LittleFS file open mode translation
static LittleFS_fopen_mode_t UVOS_SDCARD_fopen_mode_str_to_enum( uvos_fopen_mode_t mode );


// Optional statically allocated read buffer. Must be cache_size.
// By default lfs_malloc is used to allocate this buffer.
uint8_t read_buf[ LFS_PAGE_SIZE ];

// Optional statically allocated program buffer. Must be cache_size.
// By default lfs_malloc is used to allocate this buffer.
uint8_t prog_buf[ LFS_PAGE_SIZE ];

// Optional statically allocated lookahead buffer. Must be lookahead_size
// and aligned to a 32-bit boundary. By default lfs_malloc is used to
// allocate this buffer.
uint8_t lookahead_buf[ LFS_LOOKAHEAD_SIZE ] __attribute__ ( ( aligned ( 4 ) ) );

// From https://github.com/littlefs-project/littlefs/blob/master/SPEC.md :
// In addition to the logical block_size (which usually matches the erase block size),
// littlefs also uses a program block size and read block size.
// These determine the alignment of block device operations,
// but don't need to be consistent for portability.
// For W25Q128FV: Page Program (02h) is 256b page, and Sector Erase (20h) = 4096b sector

// configuration of the file system is provided by this struct
const struct lfs_config FS_cfg = {
  // block device operations
  .read  = block_device_read,
  .prog  = block_device_prog,
  .erase = block_device_erase,
  .sync  = block_device_sync,
  // chip info
  .read_size = LFS_PAGE_SIZE,
  .prog_size = LFS_PAGE_SIZE,
  .block_size = 4096,
  .block_count = 4096,
  .block_cycles = 500,
  .cache_size = LFS_PAGE_SIZE,
  .lookahead_size = LFS_LOOKAHEAD_SIZE,
  .read_buffer = read_buf,
  .prog_buffer = prog_buf,
  .lookahead_buffer = lookahead_buf,
};


/* Local Definitions */
#if !defined( SPIF_MUTEX_TAKE )
#define SPIF_MUTEX_TAKE            {}
#define SPIF_MUTEX_GIVE            {}
#endif

#define FCLK_SLOW() { UVOS_SPI_SetClockSpeed( UVOS_SPIF_SPI, UVOS_SPI_PRESCALER_64 ); }
#define FCLK_FAST() { UVOS_SPI_SetClockSpeed( UVOS_SPIF_SPI, UVOS_SPI_PRESCALER_8 ); }

#define CS_HIGH() { UVOS_SPI_RC_PinSet( UVOS_SPIF_SPI, 0, 1 ); }/* spi_id, slave_id, pin_value */
#define CS_LOW()  { UVOS_SPI_RC_PinSet( UVOS_SPIF_SPI, 0, 0 ); }/* spi_id, slave_id, pin_value */

#define WAIT_READY_MS 500 /* Time (mSec) to wait for card ready */


// Define card status values
typedef enum {
  SPIF_STATUS_NOT_MOUNTED = 0,
  SPIF_STATUS_MOUNTED = 1
} uvos_spif_status_t;

static bool spif_mounted = false;

// Variables for LittleFS
static lfs_t FS_lfs;
static int lfs_res; //Result after LittleFS operations

// Variables for SPI Flash interface
static uint32_t UVOS_SPIF_SPI;
static SPI_TypeDef *spif_spi_regs;
// static uintptr_t uvos_spif_jedec_id;
uintptr_t uvos_spif_jedec_id;

#ifdef LFS_NO_MALLOC
static uint8_t file_buffer[ LFS_PAGE_SIZE ];
static struct lfs_file_config file_cfg = {
  .buffer = file_buffer
};
#endif // LFS_NO_MALLOC

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/**
 * Initialises SPI pins and peripheral to access MMC/SD Card
 * \param[in] mode currently only mode 0 supported
 * \return < 0 if initialisation failed
 */
int32_t UVOS_SPIF_Init( uint32_t spi_id )
{
  SPIF_MUTEX_TAKE;

  spif_mounted = false;

  UVOS_SPIF_SPI = spi_id;
  struct uvos_spi_dev *spi_dev = ( struct uvos_spi_dev * )UVOS_SPIF_SPI;

  /* Save away the SPI instance if valid */
  bool valid = IS_SPI_ALL_INSTANCE( spi_dev->cfg->regs );
  if ( !valid ) {
    return -1;
  }
  spif_spi_regs = spi_dev->cfg->regs;

  /* Init SPI port for slow frequency access (ca. 0.3 MBit/s) */
  FCLK_SLOW();

  UVOS_SPI_TransferByte( spi_id, 0xFF );

  if ( UVOS_Flash_Jedec_Init( &uvos_spif_jedec_id, spi_id, 0 ) ) {
    return -2;
  }

  SPIF_MUTEX_GIVE;

  /* No error */
  return 0;
}

/**
 * Mounts the file system
 * param[in] void
 * return 0 No errors
 * return -1 SPI Flash mount of LittleFS unsuccessful
 */
static int32_t UVOS_SPIF_MountFS( void )
{
  // Open the file system
  lfs_res = lfs_mount( &FS_lfs, &FS_cfg );
  if ( lfs_res != LFS_ERR_OK ) {
    spif_mounted = false;
    return -1;
  }

  /* No errors */
  spif_mounted = true;
  return 0;
}

/**
 * Unmounts the file system
 * param[in] void
 * return 0 No errors
 * return -1 SPI Flash mount of LittleFS unsuccessful
 */
static int32_t UVOS_SPIF_UnmountFS( void )
{
  // Open the file system
  lfs_res = lfs_unmount( &FS_lfs );
  if ( lfs_res != LFS_ERR_OK ) {
    return -1;
  }

  /* No errors */
  spif_mounted = false;
  return 0;
}

/**
 * Check if the SPI Flash has been mounted
 * @return false (0) if no
 * @return true (non-zero) if yes
 */
static bool UVOS_SPIF_IsMounted( void )
{
  return spif_mounted;
}

/**
 * Format the file system
 * param[in] void
 * return 0 No errors
 * return -1 SPI Flash format of LittleFS unsuccessful
 */
int32_t UVOS_SPIF_Format( void )
{
  /* Format the file system (clobbers littlefs object, leaves file system mounted) */
  lfs_res = lfs_format( &FS_lfs, &FS_cfg );
  if ( lfs_res != LFS_ERR_OK ) {
    return -1;
  }

  /* No errors */
  spif_mounted = false;
  return 0;
}

/**
 * Checks that file system is mounted, fills in SPIF vol_info
 * return 0 if successful, -1 if unsuccessful
 */
static int32_t UVOS_SPIF_GetVolInfo( struct uvos_fs_vol_info *vol_info )
{
  // Free space (assumes 4Kb/block, for Kb divide by 4096/1024 = 4)
  lfs_ssize_t blocks_used = lfs_fs_size( &FS_lfs );
  if ( blocks_used < 0 ) {
    return -1;
  }
  uint32_t used_Kb = ( blocks_used * FS_lfs.cfg->block_size ) / 1024;

  vol_info->vol_total_Kbytes = ( FS_lfs.cfg->block_count * FS_lfs.cfg->block_size ) / 1024;
  vol_info->vol_free_Kbytes = vol_info->vol_total_Kbytes - used_Kb;
  vol_info->type = FS_TYPE_LITTLEFS;

  return 0;
}

/**
 * Opens or creates a LittleFS based file on SPI Flash device
 * param[in] LittleFS *fp Pointer to the blank file object structure
 * param[in] path Pointer to null-terminated string that specifies the file name to open or create.
 * param[in] mode Flags that specifies the type of access and open method for the file
 * return 0 if file open is successful, -1 if unsuccessful
 */
static int32_t UVOS_SPIF_File_Open( struct uvos_fs_file *fp, const char *path, uvos_fopen_mode_t mode )
{
  LittleFS_fopen_mode_t lfs_flags = UVOS_SDCARD_fopen_mode_str_to_enum( mode );

  /* open source file */
#ifdef LFS_NO_MALLOC
  lfs_res = lfs_file_opencfg( &FS_lfs, ( lfs_file_t * )fp, path, lfs_flags, &file_cfg );
#else
  lfs_res = lfs_file_open( &FS_lfs, ( lfs_file_t * )fp, path, lfs_flags );
#endif
  if ( lfs_res != LFS_ERR_OK ) {
    return -1;
  }

  return 0;
}

static int32_t UVOS_SPIF_File_Read( struct uvos_fs_file *fp, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read )
{
  lfs_res = lfs_file_read( &FS_lfs, ( lfs_file_t * )fp, buf, bytes_to_read );
  if ( lfs_res < 0 ) {
    return -1;
  }
  if ( lfs_res != bytes_to_read ) {
    return -2;
  }

  *bytes_read = lfs_res;
  return 0;
}

static int32_t UVOS_SPIF_File_Write( struct uvos_fs_file *fp, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written )
{
  lfs_ssize_t lfs_write_res;

  lfs_write_res = lfs_file_write( &FS_lfs, ( lfs_file_t * )fp, buf, bytes_to_write );
  if ( lfs_write_res < 0 ) {
    return -1;
  }
  if ( lfs_write_res != bytes_to_write ) {
    return -2;
  }

  *bytes_written = lfs_write_res;
  return 0;
}

static int32_t UVOS_SPIF_File_Seek( struct uvos_fs_file *fp, uint32_t offset )
{
  lfs_soff_t lfs_seek_res;

  if ( offset < 0 ) {
    return -1;
  }
  /* Seek offset is from top of file as with FatFS f_lseek */
  lfs_seek_res = lfs_file_seek( &FS_lfs, ( lfs_file_t * )fp, offset, LFS_SEEK_SET );
  if ( lfs_seek_res < 0 ) {
    return -2;
  }
  return 0;
}

static int32_t UVOS_SPIF_File_Tell( struct uvos_fs_file *fp )
{
  lfs_soff_t lfs_tell_res;

  lfs_tell_res = lfs_file_tell( &FS_lfs, ( lfs_file_t * )fp );
  if ( lfs_tell_res < 0 ) {
    return -1;
  }

  return 0;
}

static int32_t UVOS_SPIF_File_Close( struct uvos_fs_file *fp )
{
  lfs_res = lfs_file_close( &FS_lfs, ( lfs_file_t * )fp );
  if ( lfs_res < 0 ) {
    return -1;
  }
  return 0;
}

// Returns the size of the file, or a negative error code on failure.
static int32_t UVOS_SPIF_File_Size( const char *path )
{
  lfs_file_t fil;

  /* open source file */
#ifdef LFS_NO_MALLOC
  lfs_res = lfs_file_opencfg( &FS_lfs, &fil, path, LFS_O_RDONLY, &file_cfg );
#else
  lfs_res = lfs_file_open( &FS_lfs, &fil, path, LFS_O_RDONLY );
#endif
  if ( lfs_res < 0 ) {
    return -1;
  }

  // lfs_soff_t defined as int32_t in lfs.h
  // lfs_file_size returns a negative error code on fail
  lfs_soff_t size = lfs_file_size( &FS_lfs, &fil );
  if ( size < 0 ) {
    return -2;
  }

  /* clean up */
  lfs_res = lfs_file_close( &FS_lfs, &fil );
  if ( lfs_res < 0 ) {
    return -3;
  }

  return size;
}

static int32_t UVOS_SPIF_Remove( const char *path )
{
  // If removing a directory, directory must be empty.
  // Returns a negative error code on failure.
  lfs_res = lfs_remove( &FS_lfs, path );
  if ( lfs_res < 0 ) {
    return -1;
  }
  return 0;
}

static int32_t UVOS_SPIF_Dir_Open( struct uvos_fs_dir *dp, const char *path )
{
  lfs_res = lfs_dir_open( &FS_lfs, ( lfs_dir_t * )dp, path );
  if ( lfs_res < 0 ) {
    return -1;
  }
  return 0;
}

static int32_t UVOS_SPIF_Dir_Close( struct uvos_fs_dir *dp )
{
  lfs_res = lfs_dir_close( &FS_lfs, ( lfs_dir_t * )dp );
  if ( lfs_res < 0 ) {
    return -1;
  }
  return 0;
}

/* Read an entry in the dir info structure, based on the specified file or directory.
   Returns a positive value on success, 0 at the end of directory,
   or a negative error code on failure. */
static int32_t UVOS_SPIF_Dir_Read( struct uvos_fs_dir *dp, struct uvos_file_info *file_info )
{
  struct lfs_info finfo;

  lfs_res = lfs_dir_read( &FS_lfs, ( lfs_dir_t * )dp, &finfo );
  if ( lfs_res < 0 ) {
    return -1;
  }

  if ( finfo.type == LFS_TYPE_DIR ) {
    file_info->is_dir = true;
  } else {
    file_info->is_dir = false;
  }

  file_info->size = finfo.size;

  // Copy and null terminate dir/file name with strncpy(dst, src, num);
  strncpy( file_info->name, finfo.name, UVOS_FILE_NAME_Z );
  file_info->name[UVOS_FILE_NAME_Z - 1] = '\0';

  if ( lfs_res > 0 ) {
    return true; // success
  } else {
    return 0; // end of directory
  }
}

static int32_t UVOS_SPIF_Mkdir( const char *path )
{
  lfs_res = lfs_mkdir( &FS_lfs, path );
  if ( lfs_res < 0 ) {
    return -1;
  }
  return 0;
}

/*--------------------------------------------------------------------------

   Private Functions

---------------------------------------------------------------------------*/


/* Convert UVOS file open enum to LittleFS mode */
static inline LittleFS_fopen_mode_t UVOS_SDCARD_fopen_mode_str_to_enum( uvos_fopen_mode_t mode )
{
  LittleFS_fopen_mode_t ret_val;

  if ( mode == FOPEN_MODE_INVALID ) {
    return LFS_FOPEN_MODE_INVALID;
  }

  // Compare the mode with valid values and return the corresponding enum value
  if ( mode == FOPEN_MODE_R ) {
    ret_val = LFS_FOPEN_MODE_R; // POSIX "r"
  } else if ( mode == FOPEN_MODE_W ) {
    ret_val = LFS_FOPEN_MODE_W;  // POSIX "w"
  } else if ( mode == FOPEN_MODE_A ) {
    ret_val = LFS_FOPEN_MODE_A;  // POSIX "a"
  } else if ( mode == FOPEN_MODE_RP ) {
    ret_val = LFS_FOPEN_MODE_RP;  // POSIX "r+"
  } else if ( mode == FOPEN_MODE_WP ) {
    ret_val = LFS_FOPEN_MODE_WP;  // POSIX "w+"
  } else if ( mode == FOPEN_MODE_AP ) {
    ret_val = LFS_FOPEN_MODE_AP;  // POSIX "a+"
  } else if ( mode == FOPEN_MODE_WX ) {
    ret_val = LFS_FOPEN_MODE_WX;  // POSIX "wx"
  } else if ( mode == FOPEN_MODE_WPX ) {
    ret_val = LFS_FOPEN_MODE_WPX;  // POSIX "w+x"
  } else {
    ret_val = LFS_FOPEN_MODE_INVALID;  // Mode is invalid
  }

  return ret_val;
}

/*--------------------------------------------------------------------------

   LittleFS device I/O User Defined Functions (see lfs.h)

---------------------------------------------------------------------------*/

static int block_device_read( const struct lfs_config *c, lfs_block_t block,  lfs_off_t off, void *buffer, lfs_size_t size )
{
  lfs_res = UVOS_Flash_Jedec_ReadData( uvos_spif_jedec_id, block * c->block_size + off, buffer, size );

  if ( lfs_res ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_prog( const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size )
{
  lfs_res = UVOS_Flash_Jedec_WriteData( uvos_spif_jedec_id, block * c->block_size + off, ( uint8_t * )buffer, size );

  if ( lfs_res ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_erase( const struct lfs_config *c, lfs_block_t block )
{
  lfs_res = UVOS_Flash_Jedec_EraseSector( uvos_spif_jedec_id, block * c->block_size );

  if ( lfs_res ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_sync( const struct lfs_config *c )
{
  return LFS_ERR_OK;
}

#endif /* UVOS_INCLUDE_FLASH */
