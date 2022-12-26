#ifndef UVOS_FLASH_JEDEC_CATALOG_H
#define UVOS_FLASH_JEDEC_CATALOG_H

#include <uvos_flash_jedec_priv.h>

// this structure contains the catalog of all "known" flash chip used
const struct uvos_flash_jedec_cfg uvos_flash_jedec_catalog[] = {
  {
    // m25p16
    .expect_manufacturer = JEDEC_MANUFACTURER_ST,
    .expect_memorytype   = 0x20,
    .expect_capacity     = 0x15,
    .sector_erase = 0xD8,
    .chip_erase   = 0xC7,
    .fast_read    = 0x0B,
    .fast_read_dummy_bytes = 1,
  },
  {
    // m25px16
    .expect_manufacturer = JEDEC_MANUFACTURER_ST,
    .expect_memorytype   = 0x71,
    .expect_capacity     = 0x15,
    .sector_erase = 0xD8,
    .chip_erase   = 0xC7,
    .fast_read    = 0x0B,
    .fast_read_dummy_bytes = 1,
  },
  {
    // w25x
    .expect_manufacturer = JEDEC_MANUFACTURER_WINBOND,
    .expect_memorytype   = 0x30,
    .expect_capacity     = 0x13,
    .sector_erase = 0x20,
    .chip_erase   = 0x60,
    .fast_read    = 0x0B,
    .fast_read_dummy_bytes = 1,
  },
  {
    // 25q16
    .expect_manufacturer = JEDEC_MANUFACTURER_WINBOND,
    .expect_memorytype   = 0x40,
    .expect_capacity     = 0x15,
    .sector_erase = 0x20,
    .chip_erase   = 0x60,
    .fast_read    = 0x0B,
    .fast_read_dummy_bytes = 1,
  },
  {
    // 25q128
    .expect_manufacturer = JEDEC_MANUFACTURER_WINBOND,
    .expect_memorytype   = 0x40,
    .expect_capacity     = 0x18,
    .sector_erase = 0x20,
    .chip_erase   = 0x60,
    .fast_read    = 0x0B,
    .fast_read_dummy_bytes = 1,
  },
  {
    // 25q512
    .expect_manufacturer = JEDEC_MANUFACTURER_MICRON,
    .expect_memorytype   = 0xBA,
    .expect_capacity     = 0x20,
    .sector_erase = 0xD8,
    .chip_erase   = 0xC7,
    .fast_read    = 0x0B,
    .fast_read_dummy_bytes = 1,
  },
  {
    // 25q256
    .expect_manufacturer = JEDEC_MANUFACTURER_NUMORIX,
    .expect_memorytype   = 0xBA,
    .expect_capacity     = 0x19,
    .sector_erase = 0xD8,
    .chip_erase   = 0xC7,
    .fast_read    = 0x0B,
    .fast_read_dummy_bytes = 1,
  },
  {
    // 25q128
    .expect_manufacturer = JEDEC_MANUFACTURER_MICRON,
    .expect_memorytype   = 0xBA,
    .expect_capacity     = 0x18,
    .sector_erase = 0xD8,
    .chip_erase   = 0xC7,
    .fast_read    = 0x0B,
    .fast_read_dummy_bytes = 1,
  },
};
const uint32_t uvos_flash_jedec_catalog_size = NELEMENTS( uvos_flash_jedec_catalog );

#endif /* UVOS_FLASH_JEDEC_CATALOG_H */