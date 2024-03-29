#include "uvos.h"
#include "uvos_spi_priv.h"
#include "diskio.h" // FatFS interface

#ifdef UVOS_INCLUDE_SDCARD

/* Global Variables */

/* Comment out in order to not use UVOS_SPI transfer functions */
#define USE_UVOS_SPI

/* Local Definitions */
#if !defined( SDCARD_MUTEX_TAKE )
#define SDCARD_MUTEX_TAKE            {}
#define SDCARD_MUTEX_GIVE            {}
#endif

static uint32_t UVOS_SDCARD_SPI;
static SPI_TypeDef *sdcard_spi_regs;
// #define SD_SPIx   SPI2

// #define FCLK_SLOW()  { LL_SPI_SetBaudRatePrescaler(SD_SPIx, LL_SPI_BAUDRATEPRESCALER_DIV128); }
#define FCLK_SLOW() { UVOS_SPI_SetClockSpeed( UVOS_SDCARD_SPI, UVOS_SPI_PRESCALER_128 ); }
// #define FCLK_FAST()  { LL_SPI_SetBaudRatePrescaler(SD_SPIx, LL_SPI_BAUDRATEPRESCALER_DIV8); }
#define FCLK_FAST() { UVOS_SPI_SetClockSpeed( UVOS_SDCARD_SPI, UVOS_SPI_PRESCALER_8 ); }

// #define CS_HIGH()      { LL_GPIO_SetOutputPin(SD_CS_GPIO_Port, SD_CS_Pin); }
#define CS_HIGH() { UVOS_SPI_RC_PinSet( UVOS_SDCARD_SPI, 0, 1 ); }/* spi_id, slave_id, pin_value */
// #define CS_LOW()     { LL_GPIO_ResetOutputPin(SD_CS_GPIO_Port, SD_CS_Pin); }
#define CS_LOW()  { UVOS_SPI_RC_PinSet( UVOS_SDCARD_SPI, 0, 0 ); }/* spi_id, slave_id, pin_value */

#define MMC_CD        1 /* Assumes card detect (yes:true, no:false, default:true) */
#define MMC_WP        0 /* Assume write off protected (yes:true, no:false, default:false) */
// #define SPIx_CR1      SD_SPIx->CR1
// #define SPIx_SR       SD_SPIx->SR
// #define SPIx_DR       SD_SPIx->DR
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

static int32_t sdcard_mounted;
static volatile DSTATUS Stat = STA_NOINIT;  /* Physical drive status */
static volatile UINT Timer1, Timer2;    /* 1kHz decrement timer stopped at zero (disk_timerproc()) */
static BYTE CardType; /* Card type flags */

#pragma GCC push_options
#pragma GCC optimize ("O0")

void disk_timerproc ( void );

void UserSystickCallback( void )
{
  disk_timerproc();
}

/* Exchange a byte */
static BYTE xchg_spi (
  BYTE dat  /* Data to send */
)
{
  return UVOS_SPI_TransferByte( UVOS_SDCARD_SPI, dat );
}

#if defined (USE_UVOS_SPI)

/* Receive multiple byte */
// static void rcvr_spi_multi (
void rcvr_spi_multi (
  BYTE *buff,   /* Pointer to data buffer */
  UINT brx    /* Number of bytes to receive (even number) */
)
{
  UVOS_SPI_TransferBlock( UVOS_SDCARD_SPI, NULL, buff, brx, NULL );
}

#if FF_FS_READONLY == 0

/* Send multiple byte */
void xmit_spi_multi (
  const BYTE *buff, /* Pointer to the data */
  UINT btx      /* Number of bytes to send (even number) */
)
{
  UVOS_SPI_TransferBlock( UVOS_SDCARD_SPI, buff, NULL, btx, NULL );
}

#endif // FF_FS_READONLY == 0

#else // defined (USE_UVOS_SPI)

static const UINT flags = ( LL_SPI_SR_BSY | LL_SPI_SR_TXE | LL_SPI_SR_RXNE );
static const UINT complete = ( LL_SPI_SR_TXE | LL_SPI_SR_RXNE );

/* Receive multiple byte */
// static void rcvr_spi_multi (
void rcvr_spi_multi (
  BYTE *buff,   /* Pointer to data buffer */
  UINT brx    /* Number of bytes to receive (even number) */
)
{
  WORD d;

  /* Put SPI into 16-bit mode */
  LL_SPI_SetDataWidth( sdcard_spi_regs, LL_SPI_DATAWIDTH_16BIT );

  /* Receive the data block into buffer */
  LL_SPI_TransmitData16( sdcard_spi_regs, 0xFFFF );
  brx -= 2;
  do {
    while ( ( sdcard_spi_regs->SR & flags ) != complete );  /* Wait for end of the SPI transaction */
    d = LL_SPI_ReceiveData16( sdcard_spi_regs );  /* Get received word */
    LL_SPI_TransmitData16( sdcard_spi_regs, 0xFFFF );  /* Start next transaction */
    buff[1] = d; buff[0] = d >> 8;  /* Store received data */
    buff += 2;
  } while ( brx -= 2 );
  while ( ( sdcard_spi_regs->SR & flags ) != complete );  /* Wait for end of the SPI transaction */
  d = LL_SPI_ReceiveData16( sdcard_spi_regs );  /* Get last word received */
  buff[1] = d; buff[0] = d >> 8;  /* Store it */

  /* Put SPI into 8-bit mode */
  LL_SPI_SetDataWidth( sdcard_spi_regs, LL_SPI_DATAWIDTH_8BIT );
}

#if FF_FS_READONLY == 0

/* Send multiple byte */
// static void xmit_spi_multi (
void xmit_spi_multi (
  const BYTE *buff, /* Pointer to the data */
  UINT btx      /* Number of bytes to send (even number) */
)
{
  WORD d;

  struct uvos_spi_dev *spi_dev = ( struct uvos_spi_dev * )UVOS_SDCARD_SPI;
  SPI_TypeDef *sdcard_spi_regs = spi_dev->cfg->regs;

  /* Put SPI into 16-bit mode */
  LL_SPI_SetDataWidth( spi_dev->cfg->regs, LL_SPI_DATAWIDTH_16BIT );

  /* Transmit the data block from buffer */
  d = buff[0] << 8 | buff[1];
  buff += 2;
  LL_SPI_TransmitData16( spi_dev->cfg->regs, d );
  btx -= 2;
  do {
    d = buff[0] << 8 | buff[1]; buff += 2;  /* Word to send next */
    while ( ( sdcard_spi_regs->SR & flags ) != complete );  /* Wait for end of the SPI transaction */
    LL_SPI_ReceiveData16( sdcard_spi_regs );  /* Discard received word */
    LL_SPI_TransmitData16( sdcard_spi_regs, d );  /* Start next transaction */
  } while ( btx -= 2 );
  while ( ( sdcard_spi_regs->SR & flags ) != complete );  /* Wait for end of the SPI transaction */
  LL_SPI_ReceiveData16( sdcard_spi_regs );  /* Discard received word */

  /* Put SPI into 8-bit mode */
  LL_SPI_SetDataWidth( sdcard_spi_regs, LL_SPI_DATAWIDTH_8BIT );
}

#endif // FF_FS_READONLY == 0

#endif // defined (USE_UVOS_SPI)


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
  tick_start =  UVOS_SYS_GetCurrentMillis();

  do {
    d = xchg_spi( 0xFF );
    /* This loop takes a time. Insert rot_rdq() here for multitask envilonment. */
    tick_elapsed = UVOS_SYS_GetCurrentMillis() - tick_start;
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


  if ( !wait_ready( 500 ) ) return 0;   /* Wait for card ready */

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

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

/**
 * Initialises SPI pins and peripheral to access MMC/SD Card
 * \param[in] mode currently only mode 0 supported
 * \return < 0 if initialisation failed
 */
DSTATUS disk_spi_init( uint32_t spi_id )
{
  SDCARD_MUTEX_TAKE;

  sdcard_mounted  = 0;

  UVOS_SDCARD_SPI = spi_id;
  struct uvos_spi_dev *spi_dev = ( struct uvos_spi_dev * )UVOS_SDCARD_SPI;

  /* Save away the SPI instance if valid */
  bool valid = IS_SPI_ALL_INSTANCE( spi_dev->cfg->regs );
  if ( !valid ) {
    return RES_PARERR;
  }
  sdcard_spi_regs = spi_dev->cfg->regs;

  /* Init SPI port for slow frequency access (ca. 0.3 MBit/s) */
  FCLK_SLOW();

  UVOS_SPI_TransferByte( UVOS_SDCARD_SPI, 0xFF );
  SDCARD_MUTEX_GIVE;

  /* No error */
  return RES_OK;
}

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

void disk_timerproc ( void )
{
  WORD n;
  BYTE s;


  n = Timer1;           /* 1kHz decrement timer stopped at 0 */
  if ( n ) Timer1 = --n;
  n = Timer2;
  if ( n ) Timer2 = --n;

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

#pragma GCC pop_options

#endif /* UVOS_INCLUDE_SDCARD */

/**
 * @}
 * @}
 */
