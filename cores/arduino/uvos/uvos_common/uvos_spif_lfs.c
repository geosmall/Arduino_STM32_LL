#include "uvos.h"
#include "uvos_spi_priv.h"
#include "uvos_fs_driver.h"
#include "uvos_flash_jedec_priv.h"

#ifdef UVOS_INCLUDE_FLASH

#define FS_FILE_NAME_SIZE  32 /* Length of file name, used in FS buffers */
#define LFS_PAGE_SIZE 256
#define LFS_LOOKAHEAD_SIZE 512
// #define PRN_BUFFER_SIZE  128

static int block_device_read( const struct lfs_config *c, lfs_block_t block,  lfs_off_t off, void *buffer, lfs_size_t size );
static int block_device_prog( const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size );
static int block_device_erase( const struct lfs_config *c, lfs_block_t block );
static int block_device_sync( const struct lfs_config *c );

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

// #define FCLK_SLOW()  { LL_SPI_SetBaudRatePrescaler(SD_SPIx, LL_SPI_BAUDRATEPRESCALER_DIV128); }
#define FCLK_SLOW() { UVOS_SPI_SetClockSpeed( UVOS_SPIF_SPI, UVOS_SPI_PRESCALER_128 ); }
// #define FCLK_FAST()  { LL_SPI_SetBaudRatePrescaler(SD_SPIx, LL_SPI_BAUDRATEPRESCALER_DIV8); }
#define FCLK_FAST() { UVOS_SPI_SetClockSpeed( UVOS_SPIF_SPI, UVOS_SPI_PRESCALER_8 ); }

// #define CS_HIGH()      { LL_GPIO_SetOutputPin(SD_CS_GPIO_Port, SD_CS_Pin); }
#define CS_HIGH() { UVOS_SPI_RC_PinSet( UVOS_SPIF_SPI, 0, 1 ); }/* spi_id, slave_id, pin_value */
// #define CS_LOW()     { LL_GPIO_ResetOutputPin(SD_CS_GPIO_Port, SD_CS_Pin); }
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
static int fres; //Result after LittleFS operations

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

#pragma GCC push_options
#pragma GCC optimize ("O0")

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
int32_t UVOS_SPIF_MountFS( void )
{
  // Open the file system
  fres = lfs_mount( &FS_lfs, &FS_cfg );
  if ( fres != LFS_ERR_OK ) {
    spif_mounted = false;
    return -1;
  }

  /* No errors */
  spif_mounted = true;
  return 0;
}

/**
 * Check if the SD card has been mounted
 * @return 0 if no
 * @return 1 if yes
 */
bool UVOS_SPIF_IsMounted( void )
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
  // Format the file system
  fres = lfs_format( &FS_lfs, &FS_cfg );
  if ( fres != LFS_ERR_OK ) {
    return -1;
  }

  /* No errors */
  return 0;
}

/**
 * Checks that file system is mounted, fills in SPIF vol_info
 * return 0 if successful, -1 if unsuccessful
 */
int32_t UVOS_SPIF_GetVolInfo( uvos_fs_vol_info_t *vol_info )
{
  // Free space (assumes 4Kb/block, for Kb divide by 4096/1024 = 4)
  // uint32_t total_Kb = ( FS_lfs.cfg->block_count * FS_lfs.cfg->block_size ) / 1024;
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
int32_t UVOS_SPIF_Open( uintptr_t *fp, const char *path, uvos_fopen_mode_t mode )
{
  // fres = f_open( ( FIL * )fp, path, ( BYTE )mode );
  // if ( fres != FR_OK ) {
  //   return -1;
  // }

  /* open source file */
#ifdef LFS_NO_MALLOC
  fres = lfs_file_opencfg( &FS_lfs, ( lfs_file_t * )fp, path, LFS_O_RDONLY, &file_cfg );
#else
  fres = lfs_file_open( &FS_lfs, ( lfs_file_t * )fp, path, LFS_O_RDONLY );
#endif
  if ( fres != LFS_ERR_OK ) {
    return -1;
  }

  return 0;
}

int32_t UVOS_SPIF_Read( uintptr_t *fp, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read )
{
  // fres = f_read( ( FIL * )fp, buf, bytes_to_read, ( UINT * )bytes_read );
  // if ( fres != FR_OK ) {
  //   return -1;
  // }
  fres = lfs_file_read( &FS_lfs, ( lfs_file_t * )fp, buf, bytes_to_read );
  if ( fres < 0 ) {
    return -1;
  }
  if ( fres != bytes_to_read ) {
    return -2;
  }

  *bytes_read = fres;
  return 0;
}

int32_t UVOS_SPIF_Write( uintptr_t *fp, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written )
{
  return 0;
}

int32_t UVOS_SPIF_Seek( uintptr_t *fp, uint32_t offset )
{
  return 0;
}

uint32_t UVOS_SPIF_Tell( uintptr_t *fp )
{
  return 0;
}

int32_t UVOS_SPIF_Close( uintptr_t *fp )
{
  return 0;
}

int32_t UVOS_SPIF_Remove( const char *path )
{
  return 0;
}


/*--------------------------------------------------------------------------

   Private Functions

---------------------------------------------------------------------------*/


/* Convert UVOS file open enum to LittleFS mode */
static int UVOS_SDCARD_fopen_mode_str_to_enum( uvos_fopen_mode_t mode )
{
  if ( mode == FOPEN_MODE_INVALID ) {
    return -1;
  }

  // Compare the mode with valid values and return the corresponding enum value
  if ( mode == FOPEN_MODE_R ) {
    return ( LITTLEFS_FOPEN_MODE_R ); // POSIX "r"
  } else if ( mode == FOPEN_MODE_W ) {
    return ( LITTLEFS_FOPEN_MODE_W );  // POSIX "w"
  } else if ( mode == FOPEN_MODE_A ) {
    return ( LITTLEFS_FOPEN_MODE_A );  // POSIX "a"
  } else if ( mode == FOPEN_MODE_RP ) {
    return ( LITTLEFS_FOPEN_MODE_RP );  // POSIX "r+"
  } else if ( mode == FOPEN_MODE_WP ) {
    return ( LITTLEFS_FOPEN_MODE_WP );  // POSIX "w+"
  } else if ( mode == FOPEN_MODE_AP ) {
    return ( LITTLEFS_FOPEN_MODE_AP );  // POSIX "a+"
  } else if ( mode == FOPEN_MODE_WX ) {
    return ( LITTLEFS_FOPEN_MODE_WX );  // POSIX "wx"
  } else if ( mode == FOPEN_MODE_WPX ) {
    return ( LITTLEFS_FOPEN_MODE_WPX );  // POSIX "w+x"
  } else {
    // Mode is invalid
    return -1;
  }
}

/*--------------------------------------------------------------------------

   LittleFS device I/O User Defined Functions (see lfs.h)

---------------------------------------------------------------------------*/

static int block_device_read( const struct lfs_config *c, lfs_block_t block,  lfs_off_t off, void *buffer, lfs_size_t size )
{
  fres = UVOS_Flash_Jedec_ReadData( uvos_spif_jedec_id, block * c->block_size + off, buffer, size );

  if ( fres ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_prog( const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size )
{
  fres = UVOS_Flash_Jedec_WriteData( uvos_spif_jedec_id, block * c->block_size + off, ( uint8_t * )buffer, size );

  if ( fres ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_erase( const struct lfs_config *c, lfs_block_t block )
{
  fres = UVOS_Flash_Jedec_EraseSector( uvos_spif_jedec_id, block * c->block_size );

  if ( fres ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_sync( const struct lfs_config *c )
{
  return LFS_ERR_OK;
}

#pragma GCC pop_options

#endif /* UVOS_INCLUDE_FLASH */
