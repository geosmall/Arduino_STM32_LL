#include "uvos.h"
#include "uvos_fs.h"
#include "uvos_fs_priv.h"
#include "uvos_spi_priv.h"
#include "diskio.h" // FatFS interface

#ifdef UVOS_INCLUDE_SDCARD

static int32_t UVOS_SDCARD_MountFS( void );
static int32_t UVOS_SDCARD_UnmountFS( void );
static bool UVOS_SDCARD_IsMounted( void );
static int32_t UVOS_SDCARD_GetVolInfo( struct uvos_fs_vol_info *vol_info );

static int32_t UVOS_SDCARD_File_Open( struct uvos_fs_file *fp, const char *path, uvos_fopen_mode_t mode );
static int32_t UVOS_SDCARD_File_Read( struct uvos_fs_file *fp, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read );
static int32_t UVOS_SDCARD_File_Write( struct uvos_fs_file *fp, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written );
static int32_t UVOS_SDCARD_File_Seek( struct uvos_fs_file *fp, int32_t offset );
static uint32_t UVOS_SDCARD_File_Tell( struct uvos_fs_file *fp );
static int32_t UVOS_SDCARD_File_Close( struct uvos_fs_file *fp );

static int32_t UVOS_SDCARD_Remove( const char *path );

static int32_t UVOS_SDCARD_Dir_Open( struct uvos_fs_dir *dp, const char *path );
static int32_t UVOS_SDCARD_Dir_Close( struct uvos_fs_dir *dp );
static int32_t UVOS_SDCARD_Dir_Read( struct uvos_fs_dir *dp, struct uvos_file_info *file_info );
static int32_t UVOS_SDCARD_Mkdir( const char *path );

/* Provide a file system driver */
const struct uvos_fs_driver uvos_fs_sdcard_driver = {
  .mount_fs = UVOS_SDCARD_MountFS,
  .unmount_fs = UVOS_SDCARD_UnmountFS,
  .is_mounted = UVOS_SDCARD_IsMounted,
  .get_vol_info = UVOS_SDCARD_GetVolInfo,
  .file_open = UVOS_SDCARD_File_Open,
  .file_read = UVOS_SDCARD_File_Read,
  .file_write = UVOS_SDCARD_File_Write,
  .file_seek = UVOS_SDCARD_File_Seek,
  .file_tell = UVOS_SDCARD_File_Tell,
  .file_close = UVOS_SDCARD_File_Close,
  .remove = UVOS_SDCARD_Remove,
  .dir_open = UVOS_SDCARD_Dir_Open,
  .dir_close = UVOS_SDCARD_Dir_Close,
  .dir_read = UVOS_SDCARD_Dir_Read,
  .mkdir = UVOS_SDCARD_Mkdir,
};

/* Local Definitions */
#if !defined( SDCARD_MUTEX_TAKE )
#define SDCARD_MUTEX_TAKE            {}
#define SDCARD_MUTEX_GIVE            {}
#endif

// #define FCLK_SLOW()  { LL_SPI_SetBaudRatePrescaler(SD_SPIx, LL_SPI_BAUDRATEPRESCALER_DIV128); }
#define FCLK_SLOW() { UVOS_SPI_SetClockSpeed( UVOS_SDCARD_SPI, UVOS_SPI_PRESCALER_128 ); }
// #define FCLK_FAST()  { LL_SPI_SetBaudRatePrescaler(SD_SPIx, LL_SPI_BAUDRATEPRESCALER_DIV8); }
#define FCLK_FAST() { UVOS_SPI_SetClockSpeed( UVOS_SDCARD_SPI, UVOS_SPI_PRESCALER_8 ); }

#define CS_HIGH() { UVOS_SPI_RC_PinSet( UVOS_SDCARD_SPI, 0, 1 ); }/* spi_id, slave_id, pin_value */
#define CS_LOW()  { UVOS_SPI_RC_PinSet( UVOS_SDCARD_SPI, 0, 0 ); }/* spi_id, slave_id, pin_value */

#define MMC_CD        1 /* Assumes card detect (yes:true, no:false, default:true) */
#define MMC_WP        0 /* Assume write off protected (yes:true, no:false, default:false) */
#define WAIT_READY_MS 500 /* Time (mSec) to wait for card ready */

// _BV Converts a bit number into a Byte Value (BV).
#define _BV(bit)      ( 1 << ( bit ) ) // : Special function registers

/* MMC/SD command */
#define CMD0  (0)     /* GO_IDLE_STATE */
#define CMD1  (1)     /* SEND_OP_COND (MMC) */
#define ACMD41  (0x80+41) /* SEND_OP_COND (SDC) */
#define CMD8  (8)     /* SEND_IF_COND */
#define CMD9  (9)     /* SEND_CSD */
#define CMD10 (10)    /* SEND_CID */
#define CMD12 (12)    /* STOP_TRANSMISSION */
#define ACMD13  (0x80+13) /* SD_STATUS (SDC) */
#define CMD16 (16)    /* SET_BLOCKLEN */
#define CMD17 (17)    /* READ_SINGLE_BLOCK */
#define CMD18 (18)    /* READ_MULTIPLE_BLOCK */
#define CMD23 (23)    /* SET_BLOCK_COUNT (MMC) */
#define ACMD23  (0x80+23) /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24 (24)    /* WRITE_BLOCK */
#define CMD25 (25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD32 (32)    /* ERASE_ER_BLK_START */
#define CMD33 (33)    /* ERASE_ER_BLK_END */
#define CMD38 (38)    /* ERASE */
#define CMD55 (55)    /* APP_CMD */
#define CMD58 (58)    /* READ_OCR */

// Define card status values
typedef enum {
  SDCARD_STATUS_NOT_MOUNTED = 0,
  SDCARD_STATUS_MOUNTED = 1
} uvos_sdcard_status_t;

static bool sdcard_mounted = false;
static volatile DSTATUS Stat = STA_NOINIT;  /* Physical drive status */
static BYTE CardType; /* Card type flags */
static volatile UINT Timer1;    /* 1kHz decrement timer stopped at zero (disk_timerproc()) */

// Variables for FatFs
static FATFS FatFs;  //Fatfs handle
static FRESULT fres; //Result after FatFS operations

// Variables for SD Card SPI interface
static uint32_t UVOS_SDCARD_SPI;
static SPI_TypeDef *sdcard_spi_regs;

static int UVOS_SDCARD_fopen_mode_str_to_enum( uvos_fopen_mode_t mode );

static void disk_timerproc ( void );
void UserSystickCallback( void )
{
  disk_timerproc();
}

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/**
 * Initialises SPI pins and peripheral to access MMC/SD Card
 * \param[in] mode currently only mode 0 supported
 * \return < 0 if initialisation failed
 */
int32_t UVOS_SDCARD_Init( uint32_t spi_id )
{
  SDCARD_MUTEX_TAKE;

  sdcard_mounted = false;

  UVOS_SDCARD_SPI = spi_id;
  struct uvos_spi_dev *spi_dev = ( struct uvos_spi_dev * )UVOS_SDCARD_SPI;

  /* Save away the SPI instance if valid */
  bool valid = IS_SPI_ALL_INSTANCE( spi_dev->cfg->regs );
  if ( !valid ) {
    return -1;
  }
  sdcard_spi_regs = spi_dev->cfg->regs;

  /* Init SPI port for slow frequency access (ca. 0.3 MBit/s) */
  FCLK_SLOW();

  UVOS_SPI_TransferByte( UVOS_SDCARD_SPI, 0xFF );
  SDCARD_MUTEX_GIVE;

  /* No error */
  return 0;
}

/**
 * Mounts the file system
 * param[in] void
 * return 0 No errors
 * return -1 SDCard mount of FatFs unsuccessful
 */
static int32_t UVOS_SDCARD_MountFS( void )
{
  // Open the file system
  fres = f_mount( &FatFs, "", 1 ); // 1=mount now
  if ( fres != FR_OK ) {
    sdcard_mounted = false;
    return -1;
  }

  /* No errors */
  sdcard_mounted = true;
  return 0;
}

/**
 * Unmounts the file system
 * param[in] void
 * return 0 No errors
 * return -1 SDCard unmount of FatFs unsuccessful
 */
static int32_t UVOS_SDCARD_UnmountFS( void )
{
  // Open the file system
  fres = f_unmount( "" );
  if ( fres != FR_OK ) {
    return -1;
  }

  /* No errors */
  sdcard_mounted = false;
  return 0;
}

/**
 * Check if the SD card has been mounted
 * @return 0 if no
 * @return 1 if yes
 */
static bool UVOS_SDCARD_IsMounted( void )
{
  return sdcard_mounted;
}

/**
 * Checks that file system is mounted, fills in SDCARD vol_info
 * return 0 if successful, -1 if unsuccessful
 */
static int32_t UVOS_SDCARD_GetVolInfo( struct uvos_fs_vol_info *vol_info )
{
  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS *fs_ptr = &FatFs;

  if ( !sdcard_mounted ) {
    return -1;
  }

  fres = f_getfree( "", &free_clusters, &fs_ptr );
  if ( fres != FR_OK ) {
    return -1;
  }

  //Formula comes from ChaN documentation http://elm-chan.org/fsw/ff/doc/getfree.html
  total_sectors = ( fs_ptr->n_fatent - 2 ) * fs_ptr->csize;
  free_sectors = free_clusters * fs_ptr->csize;

  // Free space (assumes 512 bytes/sector, for Kb divide by 1024/512 = 2)
  vol_info->vol_total_Kbytes = total_sectors / 2;
  vol_info->vol_free_Kbytes = free_sectors / 2;
  vol_info->type = FS_TYPE_FATFS;

  return 0;
}

/**
 * Opens or creates a FatFs based file on SD Card
 * param[in] FatFs FIL *fp Pointer to the blank file object structure
 * param[in] path Pointer to null-terminated string that specifies the file name to open or create.
 * param[in] mode Flags that specifies the type of access and open method for the file
 * return 0 if file open is successful, -1 if unsuccessful
 */
static int32_t UVOS_SDCARD_File_Open( struct uvos_fs_file *fp, const char *path, uvos_fopen_mode_t mode )
{
  fres = f_open( ( FIL * )fp, path, ( BYTE )UVOS_SDCARD_fopen_mode_str_to_enum( mode ) );
  if ( fres != FR_OK ) {
    return -1;
  }
  return 0;
}

static int32_t UVOS_SDCARD_File_Read( struct uvos_fs_file *fp, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read )
{
  fres = f_read( ( FIL * )fp, buf, bytes_to_read, ( UINT * )bytes_read );
  if ( fres != FR_OK ) {
    return -1;
  }
  return 0;
}

static int32_t UVOS_SDCARD_File_Write( struct uvos_fs_file *fp, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written )
{
  fres = f_write( ( FIL * )fp, buf, bytes_to_write, ( UINT * )bytes_written );
  if ( fres != FR_OK ) {
    return -1;
  }
  return 0;
}

static int32_t UVOS_SDCARD_File_Seek( struct uvos_fs_file *fp, int32_t offset )
{
  fres = f_lseek( ( FIL * )fp, offset );
  if ( fres != FR_OK ) {
    return -1;
  }
  return 0;
}

static uint32_t UVOS_SDCARD_File_Tell( struct uvos_fs_file *fp )
{
  return ( uint32_t )f_tell( ( FIL * )fp );
}

static int32_t UVOS_SDCARD_File_Close( struct uvos_fs_file *fp )
{
  fres = f_close( ( FIL * )fp );
  if ( fres != FR_OK ) {
    return -1;
  }
  return 0;
}

static int32_t UVOS_SDCARD_Remove( const char *path )
{
  // If removing a directory, directory must be empty and not read-only
  // Returns a negative error code on failure.
  fres = f_unlink( path );
  if ( fres != FR_OK ) {
    return -1;
  }
  return 0;
}

static int32_t UVOS_SDCARD_Dir_Open( struct uvos_fs_dir *dp, const char *path )
{
  fres = f_opendir( ( DIR * )dp, path );
  if ( fres != FR_OK ) {
    return -1;
  }
  return 0;
}

static int32_t UVOS_SDCARD_Dir_Close( struct uvos_fs_dir *dp )
{
  fres = f_closedir( ( DIR * )dp );
  if ( fres != FR_OK ) {
    return -1;
  }
  return 0;
}

/* Read an entry in the dir info structure, based on the specified file or directory.
   Returns a positive value on success, 0 at the end of directory,
   or a negative error code on failure. */
static int32_t UVOS_SDCARD_Dir_Read( struct uvos_fs_dir *dp, struct uvos_file_info *file_info )
{
  FILINFO finfo;

  fres = f_readdir( ( DIR * )dp, &finfo );
  if ( fres != FR_OK ) {
    return -1;
  }

  if ( finfo.fattrib & AM_DIR ) {
    file_info->is_dir = true;
  } else {
    file_info->is_dir = false;
  }

  file_info->size = finfo.fsize;

  // Copy and null terminate dir/file name with strncpy(dst, src, num);
  strncpy( file_info->name, finfo.fname, UVOS_FILE_NAME_Z );
  file_info->name[UVOS_FILE_NAME_Z - 1] = '\0';

  if ( finfo.fname[0] != '\0' ) {
    return true; // success
  } else {
    return 0; // fname is NULL, end of directory
  }
}

static int32_t UVOS_SDCARD_Mkdir( const char *path )
{
  fres = f_mkdir( path );
  if ( fres != FR_OK ) {
    return -1;
  }
  return 0;
}

/*--------------------------------------------------------------------------

   Private Functions

---------------------------------------------------------------------------*/

/* Convert UVOS file open enum to FatFs mode */
static int UVOS_SDCARD_fopen_mode_str_to_enum( uvos_fopen_mode_t mode )
{
  if ( mode == FOPEN_MODE_INVALID ) {
    return -1;
  }

  // Compare the mode with valid values and return the corresponding enum value
  if ( mode == FOPEN_MODE_R ) {
    return ( FATFS_FOPEN_MODE_R ); // POSIX "r"
  } else if ( mode == FOPEN_MODE_W ) {
    return ( FATFS_FOPEN_MODE_W );  // POSIX "w"
  } else if ( mode == FOPEN_MODE_A ) {
    return ( FATFS_FOPEN_MODE_A );  // POSIX "a"
  } else if ( mode == FOPEN_MODE_RP ) {
    return ( FATFS_FOPEN_MODE_RP );  // POSIX "r+"
  } else if ( mode == FOPEN_MODE_WP ) {
    return ( FATFS_FOPEN_MODE_WP );  // POSIX "w+"
  } else if ( mode == FOPEN_MODE_AP ) {
    return ( FATFS_FOPEN_MODE_AP );  // POSIX "a+"
  } else if ( mode == FOPEN_MODE_WX ) {
    return ( FATFS_FOPEN_MODE_WX );  // POSIX "wx"
  } else if ( mode == FOPEN_MODE_WPX ) {
    return ( FATFS_FOPEN_MODE_WPX );  // POSIX "w+x"
  } else {
    // Mode is invalid
    return -1;
  }
}

/* Exchange a byte */
static BYTE xchg_spi (
  BYTE dat  /* Data to send */
)
{
  return UVOS_SPI_TransferByte( UVOS_SDCARD_SPI, dat );
}

/* Receive multiple byte */
static void rcvr_spi_multi (
  BYTE *buff,   /* Pointer to data buffer */
  UINT brx    /* Number of bytes to receive (even number) */
)
{
  UVOS_SPI_TransferBlock( UVOS_SDCARD_SPI, NULL, buff, brx, NULL );
}

#if FF_FS_READONLY == 0

/* Send multiple byte */
static void xmit_spi_multi (
  const BYTE *buff, /* Pointer to the data */
  UINT btx      /* Number of bytes to send (even number) */
)
{
  UVOS_SPI_TransferBlock( UVOS_SDCARD_SPI, buff, NULL, btx, NULL );
}

#endif // FF_FS_READONLY == 0

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static int wait_ready ( /* 1:Ready, 0:Timeout */
  UINT wait_time_ms     /* Timeout [ms] */
)
{
  BYTE d;
  uint32_t tick_start;
  uint32_t tick_elapsed;

  /* Init tick_start for timeout management */
  tick_start =  UVOS_SYS_GetTick();

  do {
    d = xchg_spi( 0xFF );
    /* This loop takes a time. Insert rot_rdq() here for multitask envilonment. */
    tick_elapsed = UVOS_SYS_GetTick() - tick_start;
  } while ( d != 0xFF && ( tick_elapsed < wait_time_ms ) ); /* Wait for card goes ready or timeout */

  return ( d == 0xFF ) ? 1 : 0;
}

/*-----------------------------------------------------------------------*/
/* Deselect card and release SPI                                         */
/*-----------------------------------------------------------------------*/

static void sdcard_deselect ( void )
{
  CS_HIGH();    /* Set CS# high */
  xchg_spi( 0xFF ); /* Dummy clock (force DO hi-z for multiple slave SPI) */
}

/*-----------------------------------------------------------------------*/
/* Select card and wait for ready                                        */
/*-----------------------------------------------------------------------*/

static int sdcard_select ( void )  /* 1:OK, 0:Timeout */
{
  CS_LOW();   /* Set CS# low */
  xchg_spi( 0xFF ); /* Dummy clock (force DO enabled) */
  if ( wait_ready( WAIT_READY_MS ) ) return 1;  /* Wait for card ready */

  sdcard_deselect();
  return 0; /* Timeout */
}

/*-----------------------------------------------------------------------*/
/* Receive a data packet from the MMC                                    */
/*-----------------------------------------------------------------------*/

static int rcvr_datablock ( /* 1:OK, 0:Error */
  BYTE *buff,       /* Data buffer */
  UINT brx        /* Data block length (byte) */
)
{
  BYTE token;


  Timer1 = 200;
  do {              /* Wait for DataStart token in timeout of 200ms */
    token = xchg_spi( 0xFF );
    /* This loop will take a time. Insert rot_rdq() here for multitask envilonment. */
  } while ( ( token == 0xFF ) && Timer1 );
  if ( token != 0xFE ) return 0;    /* Function fails if invalid DataStart token or timeout */

  rcvr_spi_multi( buff, brx );    /* Store trailing data to the buffer */
  xchg_spi( 0xFF ); xchg_spi( 0xFF );     /* Discard CRC */

  return 1;           /* Function succeeded */
}

/*-----------------------------------------------------------------------*/
/* Send a data packet to the MMC                                         */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0
static int xmit_datablock ( /* 1:OK, 0:Failed */
  const BYTE *buff,   /* Ponter to 512 byte data to be sent */
  BYTE token        /* Token */
)
{
  BYTE resp;


  if ( !wait_ready( WAIT_READY_MS ) ) return 0;   /* Wait for card ready */

  xchg_spi( token );          /* Send token */
  if ( token != 0xFD ) {        /* Send data if token is other than StopTran */
    xmit_spi_multi( buff, 512 );    /* Data */
    xchg_spi( 0xFF ); xchg_spi( 0xFF ); /* Dummy CRC */

    resp = xchg_spi( 0xFF );        /* Receive data resp */
    if ( ( resp & 0x1F ) != 0x05 ) return 0;  /* Function fails if the data packet was not accepted */
  }
  return 1;
}
#endif

/*-----------------------------------------------------------------------*/
/* Send a command packet to the MMC                                      */
/*-----------------------------------------------------------------------*/

static BYTE send_cmd (  /* Return value: R1 resp (bit7==1:Failed to send) */
  BYTE cmd,     /* Command index */
  DWORD arg     /* Argument */
)
{
  BYTE n, res;


  if ( cmd & 0x80 ) { /* Send a CMD55 prior to ACMD<n> */
    cmd &= 0x7F;
    res = send_cmd( CMD55, 0 );
    if ( res > 1 ) return res;
  }

  /* Select the card and wait for ready except to stop multiple block read */
  if ( cmd != CMD12 ) {
    sdcard_deselect();
    if ( !sdcard_select() ) return 0xFF;
  }

  /* Send command packet */
  xchg_spi( 0x40 | cmd );       /* Start + command index */
  xchg_spi( ( BYTE )( arg >> 24 ) );    /* Argument[31..24] */
  xchg_spi( ( BYTE )( arg >> 16 ) );    /* Argument[23..16] */
  xchg_spi( ( BYTE )( arg >> 8 ) );     /* Argument[15..8] */
  xchg_spi( ( BYTE )arg );        /* Argument[7..0] */
  n = 0x01;             /* Dummy CRC + Stop */
  if ( cmd == CMD0 ) n = 0x95;      /* Valid CRC for CMD0(0) */
  if ( cmd == CMD8 ) n = 0x87;      /* Valid CRC for CMD8(0x1AA) */
  xchg_spi( n );

  /* Receive command resp */
  if ( cmd == CMD12 ) xchg_spi( 0xFF ); /* Diacard following one byte when CMD12 */
  n = 10;               /* Wait for response (10 bytes max) */
  do {
    res = xchg_spi( 0xFF );
  } while ( ( res & 0x80 ) && --n );

  return res;             /* Return received response */
}

/*--------------------------------------------------------------------------

   FatFS disk I/O User Defined Functions (see diskio.h)

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize disk                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
  BYTE drv    /* Physical drive number (0) */
)
{
  BYTE n, cmd, ty, ocr[4];


  if ( drv ) return STA_NOINIT;     /* Supports only drive 0 */
  // init_spi();              /* Initialize SPI */

  if ( Stat & STA_NODISK ) return Stat; /* Is card existing in the soket? */

  FCLK_SLOW();
  for ( n = 10; n; n-- ) xchg_spi( 0xFF );  /* Send 80 dummy clocks */

  ty = 0;
  if ( send_cmd( CMD0, 0 ) == 1 ) {     /* Put the card SPI/Idle state */
    Timer1 = 1000;            /* Initialization timeout = 1 sec */
    if ( send_cmd( CMD8, 0x1AA ) == 1 ) { /* SDv2? */
      for ( n = 0; n < 4; n++ ) ocr[n] = xchg_spi( 0xFF );  /* Get 32 bit return value of R7 resp */
      if ( ocr[2] == 0x01 && ocr[3] == 0xAA ) {       /* Is the card supports vcc of 2.7-3.6V? */
        while ( Timer1 && send_cmd( ACMD41, 1UL << 30 ) ) ; /* Wait for end of initialization with ACMD41(HCS) */
        if ( Timer1 && send_cmd( CMD58, 0 ) == 0 ) {    /* Check CCS bit in the OCR */
          for ( n = 0; n < 4; n++ ) ocr[n] = xchg_spi( 0xFF );
          ty = ( ocr[0] & 0x40 ) ? CT_SDC2 | CT_BLOCK : CT_SDC2;  /* Card id SDv2 */
        }
      }
    } else {  /* Not SDv2 card */
      if ( send_cmd( ACMD41, 0 ) <= 1 )   { /* SDv1 or MMC? */
        ty = CT_SDC1; cmd = ACMD41; /* SDv1 (ACMD41(0)) */
      } else {
        ty = CT_MMC3; cmd = CMD1; /* MMCv3 (CMD1(0)) */
      }
      while ( Timer1 && send_cmd( cmd, 0 ) ) ;    /* Wait for end of initialization */
      if ( !Timer1 || send_cmd( CMD16, 512 ) != 0 ) /* Set block length: 512 */
        ty = 0;
    }
  }
  CardType = ty;  /* Card type */
  sdcard_deselect();

  if ( ty ) {     /* OK */
    FCLK_FAST();      /* Set fast clock */
    Stat &= ~STA_NOINIT;  /* Clear STA_NOINIT flag */
  } else {      /* Failed */
    Stat = STA_NOINIT;
  }

  return Stat;
}

/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
  BYTE drv    /* Physical drive number (0) */
)
{
  if ( drv ) return STA_NOINIT;   /* Supports only drive 0 */

  return Stat;  /* Return disk status */
}

/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
  BYTE drv,   /* Physical drive number (0) */
  BYTE *buff,   /* Pointer to the data buffer to store read data */
  LBA_t sector, /* Start sector number (LBA) */
  UINT count    /* Number of sectors to read (1..128) */
)
{
  DWORD sect = ( DWORD )sector;


  if ( drv || !count ) return RES_PARERR;   /* Check parameter */
  if ( Stat & STA_NOINIT ) return RES_NOTRDY; /* Check if drive is ready */

  if ( !( CardType & CT_BLOCK ) ) sect *= 512;  /* LBA ot BA conversion (byte addressing cards) */

  if ( count == 1 ) { /* Single sector read */
    if ( ( send_cmd( CMD17, sect ) == 0 ) /* READ_SINGLE_BLOCK */
         && rcvr_datablock( buff, 512 ) ) {
      count = 0;
    }
  } else {        /* Multiple sector read */
    if ( send_cmd( CMD18, sect ) == 0 ) { /* READ_MULTIPLE_BLOCK */
      do {
        if ( !rcvr_datablock( buff, 512 ) ) break;
        buff += 512;
      } while ( --count );
      send_cmd( CMD12, 0 );       /* STOP_TRANSMISSION */
    }
  }
  sdcard_deselect();

  return count ? RES_ERROR : RES_OK;  /* Return result */
}

/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0
DRESULT disk_write (
  BYTE drv,     /* Physical drive number (0) */
  const BYTE *buff, /* Ponter to the data to write */
  LBA_t sector,   /* Start sector number (LBA) */
  UINT count      /* Number of sectors to write (1..128) */
)
{
  DWORD sect = ( DWORD )sector;


  if ( drv || !count ) return RES_PARERR;   /* Check parameter */
  if ( Stat & STA_NOINIT ) return RES_NOTRDY; /* Check drive status */
  if ( Stat & STA_PROTECT ) return RES_WRPRT; /* Check write protect */

  if ( !( CardType & CT_BLOCK ) ) sect *= 512;  /* LBA ==> BA conversion (byte addressing cards) */

  if ( count == 1 ) { /* Single sector write */
    if ( ( send_cmd( CMD24, sect ) == 0 ) /* WRITE_BLOCK */
         && xmit_datablock( buff, 0xFE ) ) {
      count = 0;
    }
  } else {        /* Multiple sector write */
    if ( CardType & CT_SDC ) send_cmd( ACMD23, count ); /* Predefine number of sectors */
    if ( send_cmd( CMD25, sect ) == 0 ) { /* WRITE_MULTIPLE_BLOCK */
      do {
        if ( !xmit_datablock( buff, 0xFC ) ) break;
        buff += 512;
      } while ( --count );
      if ( !xmit_datablock( 0, 0xFD ) ) count = 1;  /* STOP_TRAN token */
    }
  }
  sdcard_deselect();

  return count ? RES_ERROR : RES_OK;  /* Return result */
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
  BYTE drv,   /* Physical drive number (0) */
  BYTE cmd,   /* Control command code */
  void *buff    /* Pointer to the conrtol data */
)
{
  DRESULT res;
  BYTE n, csd[16];
  DWORD st, ed, csize;
  LBA_t *dp;


  if ( drv ) return RES_PARERR;         /* Check parameter */
  if ( Stat & STA_NOINIT ) return RES_NOTRDY; /* Check if drive is ready */

  res = RES_ERROR;

  switch ( cmd ) {
  case CTRL_SYNC :    /* Wait for end of internal write process of the drive */
    if ( sdcard_select() ) res = RES_OK;
    break;

  case GET_SECTOR_COUNT : /* Get drive capacity in unit of sector (DWORD) */
    if ( ( send_cmd( CMD9, 0 ) == 0 ) && rcvr_datablock( csd, 16 ) ) {
      if ( ( csd[0] >> 6 ) == 1 ) { /* SDC CSD ver 2 */
        csize = csd[9] + ( ( WORD )csd[8] << 8 ) + ( ( DWORD )( csd[7] & 63 ) << 16 ) + 1;
        *( LBA_t * )buff = csize << 10;
      } else {          /* SDC CSD ver 1 or MMC */
        n = ( csd[5] & 15 ) + ( ( csd[10] & 128 ) >> 7 ) + ( ( csd[9] & 3 ) << 1 ) + 2;
        csize = ( csd[8] >> 6 ) + ( ( WORD )csd[7] << 2 ) + ( ( WORD )( csd[6] & 3 ) << 10 ) + 1;
        *( LBA_t * )buff = csize << ( n - 9 );
      }
      res = RES_OK;
    }
    break;

  case GET_BLOCK_SIZE : /* Get erase block size in unit of sector (DWORD) */
    if ( CardType & CT_SDC2 ) { /* SDC ver 2+ */
      if ( send_cmd( ACMD13, 0 ) == 0 ) { /* Read SD status */
        xchg_spi( 0xFF );
        if ( rcvr_datablock( csd, 16 ) ) {        /* Read partial block */
          for ( n = 64 - 16; n; n-- ) xchg_spi( 0xFF ); /* Purge trailing data */
          *( DWORD * )buff = 16UL << ( csd[10] >> 4 );
          res = RES_OK;
        }
      }
    } else {          /* SDC ver 1 or MMC */
      if ( ( send_cmd( CMD9, 0 ) == 0 ) && rcvr_datablock( csd, 16 ) ) {  /* Read CSD */
        if ( CardType & CT_SDC1 ) { /* SDC ver 1.XX */
          *( DWORD * )buff = ( ( ( csd[10] & 63 ) << 1 ) + ( ( WORD )( csd[11] & 128 ) >> 7 ) + 1 ) << ( ( csd[13] >> 6 ) - 1 );
        } else {          /* MMC */
          *( DWORD * )buff = ( ( WORD )( ( csd[10] & 124 ) >> 2 ) + 1 ) * ( ( ( csd[11] & 3 ) << 3 ) + ( ( csd[11] & 224 ) >> 5 ) + 1 );
        }
        res = RES_OK;
      }
    }
    break;

  case CTRL_TRIM :  /* Erase a block of sectors (used when _USE_ERASE == 1) */
    if ( !( CardType & CT_SDC ) ) break;        /* Check if the card is SDC */
    if ( disk_ioctl( drv, MMC_GET_CSD, csd ) ) break; /* Get CSD */
    if ( !( csd[10] & 0x40 ) ) break;         /* Check if ERASE_BLK_EN = 1 */
    dp = buff; st = ( DWORD )dp[0]; ed = ( DWORD )dp[1];  /* Load sector block */
    if ( !( CardType & CT_BLOCK ) ) {
      st *= 512; ed *= 512;
    }
    if ( send_cmd( CMD32, st ) == 0 && send_cmd( CMD33, ed ) == 0 && send_cmd( CMD38, 0 ) == 0 && wait_ready( 30000 ) ) { /* Erase sector block */
      res = RES_OK; /* FatFs does not check result of this command */
    }
    break;

  /* Following commands are never used by FatFs module */

  case MMC_GET_TYPE:    /* Get MMC/SDC type (BYTE) */
    *( BYTE * )buff = CardType;
    res = RES_OK;
    break;

  case MMC_GET_CSD:   /* Read CSD (16 bytes) */
    if ( send_cmd( CMD9, 0 ) == 0 && rcvr_datablock( ( BYTE * )buff, 16 ) ) { /* READ_CSD */
      res = RES_OK;
    }
    break;

  case MMC_GET_CID:   /* Read CID (16 bytes) */
    if ( send_cmd( CMD10, 0 ) == 0 && rcvr_datablock( ( BYTE * )buff, 16 ) ) {  /* READ_CID */
      res = RES_OK;
    }
    break;

  case MMC_GET_OCR:   /* Read OCR (4 bytes) */
    if ( send_cmd( CMD58, 0 ) == 0 ) {  /* READ_OCR */
      for ( n = 0; n < 4; n++ ) *( ( ( BYTE * )buff ) + n ) = xchg_spi( 0xFF );
      res = RES_OK;
    }
    break;

  case MMC_GET_SDSTAT:  /* Read SD status (64 bytes) */
    if ( send_cmd( ACMD13, 0 ) == 0 ) { /* SD_STATUS */
      xchg_spi( 0xFF );
      if ( rcvr_datablock( ( BYTE * )buff, 64 ) ) res = RES_OK;
    }
    break;

  default:
    res = RES_PARERR;
  }

  sdcard_deselect();

  return res;
}

/*-----------------------------------------------------------------------*/
/* Device timer function                                                 */
/*-----------------------------------------------------------------------*/
/* This function must be called from timer interrupt routine in period
/  of 1 ms to generate card control timing.
*/

static void disk_timerproc ( void )
{
  WORD n;
  BYTE s;

  n = Timer1;           /* 1kHz decrement timer stopped at 0 */
  if ( n ) Timer1 = --n;

  s = Stat;
  if ( MMC_WP ) { /* Write protected */
    s |= STA_PROTECT;
  } else {    /* Write enabled */
    s &= ~STA_PROTECT;
  }
  if ( MMC_CD ) { /* Card is in socket */
    s &= ~STA_NODISK;
  } else {    /* Socket empty */
    s |= ( STA_NODISK | STA_NOINIT );
  }
  Stat = s;
}

#endif /* UVOS_INCLUDE_SDCARD */

/**
 * @}
 * @}
 */
