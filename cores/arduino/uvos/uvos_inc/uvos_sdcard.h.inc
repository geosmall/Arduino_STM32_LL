#ifndef UVOS_SDCARD_H
#define UVOS_SDCARD_H

/* Public Functions */
typedef struct {
  uint8_t  CSDStruct;      /* CSD structure */
  uint8_t  SysSpecVersion; /* System specification version */
  uint8_t  Reserved1;      /* Reserved */
  uint8_t  TAAC;           /* Data read access-time 1 */
  uint8_t  NSAC;           /* Data read access-time 2 in CLK cycles */
  uint8_t  MaxBusClkFrec;  /* Max. bus clock frequency */
  uint16_t CardComdClasses; /* Card command classes */
  uint8_t  RdBlockLen;     /* Max. read data block length */
  uint8_t  PartBlockRead;  /* Partial blocks for read allowed */
  uint8_t  WrBlockMisalign;        /* Write block misalignment */
  uint8_t  RdBlockMisalign;        /* Read block misalignment */
  uint8_t  DSRImpl;        /* DSR implemented */
  uint8_t  Reserved2;      /* Reserved */
  uint16_t DeviceSize; /* Device Size */
  uint8_t  MaxRdCurrentVDDMin;     /* Max. read current @ VDD min */
  uint8_t  MaxRdCurrentVDDMax;     /* Max. read current @ VDD max */
  uint8_t  MaxWrCurrentVDDMin;     /* Max. write current @ VDD min */
  uint8_t  MaxWrCurrentVDDMax;     /* Max. write current @ VDD max */
  uint8_t  DeviceSizeMul;  /* Device size multiplier */
  uint8_t  EraseGrSize;    /* Erase group size */
  uint8_t  EraseGrMul;     /* Erase group size multiplier */
  uint8_t  WrProtectGrSize;        /* Write protect group size */
  uint8_t  WrProtectGrEnable;      /* Write protect group enable */
  uint8_t  ManDeflECC;     /* Manufacturer default ECC */
  uint8_t  WrSpeedFact;    /* Write speed factor */
  uint8_t  MaxWrBlockLen;  /* Max. write data block length */
  uint8_t  WriteBlockPaPartial;    /* Partial blocks for write allowed */
  uint8_t  Reserved3;      /* Reserved */
  uint8_t  ContentProtectAppli;    /* Content protection application */
  uint8_t  FileFormatGrouop;       /* File format group */
  uint8_t  CopyFlag;       /* Copy flag (OTP) */
  uint8_t  PermWrProtect;  /* Permanent write protection */
  uint8_t  TempWrProtect;  /* Temporary write protection */
  uint8_t  FileFormat;     /* File Format */
  uint8_t  ECC;            /* ECC code */
  uint8_t  msd_CRC;        /* CRC */
  uint8_t  Reserved4;      /* always 1 */
} SDCARDCsdTypeDef;

/* Structure taken from Mass Storage Driver example provided by STM */
typedef struct {
  uint8_t  ManufacturerID; /* ManufacturerID */
  uint16_t OEM_AppliID; /* OEM/Application ID */
  char     ProdName[6];       /* Product Name */
  uint8_t  ProdRev;        /* Product Revision */
  uint32_t ProdSN; /* Product Serial Number */
  uint8_t  Reserved1;      /* Reserved1 */
  uint16_t ManufactDate; /* Manufacturing Date */
  uint8_t  msd_CRC;        /* CRC */
  uint8_t  Reserved2;      /* always 1 */
} SDCARDCidTypeDef;
#ifndef USE_SIM_POSIX
/* Global Variables */
extern VOLINFO UVOS_SDCARD_VolInfo;
extern uint8_t UVOS_SDCARD_Sector[SECTOR_SIZE];
#endif
/* Prototypes */
extern int32_t UVOS_SDCARD_Init( uint32_t spi_id );
extern int32_t UVOS_SDCARD_PowerOn( void );
extern int32_t UVOS_SDCARD_PowerOff( void );
extern int32_t UVOS_SDCARD_CheckAvailable( uint8_t was_available );
extern int32_t UVOS_SDCARD_SendSDCCmd( uint8_t cmd, uint32_t addr, uint8_t crc );
extern int32_t UVOS_SDCARD_SectorRead( uint32_t sector, uint8_t *buffer );
extern int32_t UVOS_SDCARD_SectorWrite( uint32_t sector, uint8_t *buffer );
extern int32_t UVOS_SDCARD_CIDRead( SDCARDCidTypeDef *cid );
extern int32_t UVOS_SDCARD_CSDRead( SDCARDCsdTypeDef *csd );

extern int32_t UVOS_SDCARD_StartupLog( void );
extern int32_t UVOS_SDCARD_IsMounted();
extern int32_t UVOS_SDCARD_MountFS( uint32_t StartupLog );
extern int32_t UVOS_SDCARD_GetFree( void );
#ifndef USE_SIM_POSIX
extern int32_t UVOS_SDCARD_ReadBuffer( PFILEINFO fileinfo, uint8_t *buffer, uint32_t len );
extern int32_t UVOS_SDCARD_ReadLine( PFILEINFO fileinfo, uint8_t *buffer, uint32_t max_len );
#endif
extern int32_t UVOS_SDCARD_FileCopy( char *Source, char *Destination );
extern int32_t UVOS_SDCARD_FileDelete( char *Filename );

#endif /* UVOS_SDCARD_H */
