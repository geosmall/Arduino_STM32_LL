#include "uvos.h"
#include "uvos_spi_priv.h"
#include "uvos_flash_jedec_priv.h"

#ifdef UVOS_INCLUDE_FLASH

#define FS_FILE_NAME_SIZE  32 /* Length of file name, used in FS buffers */
#define LFS_CACHE_SIZE 256
#define LFS_LOOKAHEAD_SIZE 512
#define PRN_BUFFER_SIZE  128

static int block_device_read( const struct lfs_config * c, lfs_block_t block,  lfs_off_t off, void * buffer, lfs_size_t size );
static int block_device_prog( const struct lfs_config * c, lfs_block_t block, lfs_off_t off, const void * buffer, lfs_size_t size );
static int block_device_erase( const struct lfs_config * c, lfs_block_t block );
static int block_device_sync( const struct lfs_config * c );

// Optional statically allocated read buffer. Must be cache_size.
// By default lfs_malloc is used to allocate this buffer.
uint8_t read_buf[ LFS_CACHE_SIZE ];

// Optional statically allocated program buffer. Must be cache_size.
// By default lfs_malloc is used to allocate this buffer.
uint8_t prog_buf[ LFS_CACHE_SIZE ];

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
  .read_size = LFS_CACHE_SIZE,
  .prog_size = LFS_CACHE_SIZE,
  .block_size = 4096,
  .block_count = 4096,
  .block_cycles = 500,
  .cache_size = LFS_CACHE_SIZE,
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

// Variables for SD Card SPI interface
static uint32_t UVOS_SPIF_SPI;
static SPI_TypeDef *spif_spi_regs;

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

  UVOS_SPI_TransferByte( UVOS_SPIF_SPI, 0xFF );
  SPIF_MUTEX_GIVE;

  /* No error */
  return 0;
}

/**
 * Mounts the file system
 * param[in] void
 * return 0 No errors
 * return -1 SPI Flash mount of FatFs unsuccessful
 */
int32_t UVOS_SPIF_MountFS( void )
{
  // Open the file system
  fres = lfs_mount( &FS_lfs, &FS_cfg );
  if ( fres != LFS_ERR_OK ) {
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

/*--------------------------------------------------------------------------

   Private Functions

---------------------------------------------------------------------------*/

static int block_device_read( const struct lfs_config * c, lfs_block_t block,  lfs_off_t off, void * buffer, lfs_size_t size )
{
  fres = UVOS_Flash_Jedec_ReadData( UVOS_FLASH_SPI_PORT, block * c->block_size + off, buffer, size );

  if ( fres ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_prog( const struct lfs_config * c, lfs_block_t block, lfs_off_t off, const void * buffer, lfs_size_t size )
{
  fres = UVOS_Flash_Jedec_WriteData( UVOS_FLASH_SPI_PORT, block * c->block_size + off, ( uint8_t * )buffer, size );

  if ( fres ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_erase( const struct lfs_config * c, lfs_block_t block )
{
  fres = UVOS_Flash_Jedec_EraseSector( UVOS_FLASH_SPI_PORT, block * c->block_size );

  if ( fres ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_sync( const struct lfs_config * c )
{
  return LFS_ERR_OK;
}

#pragma GCC pop_options

#endif /* UVOS_INCLUDE_FLASH */
