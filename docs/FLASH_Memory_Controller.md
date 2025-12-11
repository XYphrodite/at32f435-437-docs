---
title: FLASH - Flash Memory Controller
category: Memory
complexity: Intermediate
mcu: AT32F435/437
peripheral: FLASH
keywords: [flash, eeprom, program, erase, protection, slib, fap, epp, usd]
---

# FLASH - Flash Memory Controller

## Overview

The Flash Memory Controller manages the embedded Flash memory, providing programming and erasing capabilities with various protection mechanisms. The AT32F435/437 features a dual-bank Flash architecture with configurable SRAM/Flash partitioning, security library (SLIB) for IP protection, and hardware CRC calculation. The controller supports byte, halfword, and word programming with sector, block, or mass erase operations.

### Key Features

| Feature | Specification |
|---------|---------------|
| Architecture | Dual-bank (Bank1 + Bank2) |
| Total Capacity | Up to 4032 KB (4MB variant) or 1024 KB |
| Sector Size | 4 KB (4MB variant) or 2 KB |
| Block Size | 64 KB |
| Programming Width | Byte (8-bit), Halfword (16-bit), Word (32-bit) |
| Erase Granularity | Sector, Block, Bank, Mass |
| Protection | FAP (read), EPP (erase/program), SLIB (code) |
| User Data | USD with FAP, SSB, EPP, EOPB0, DATA0/1 |
| Special Features | CRC calculation, NZW boost, configurable SRAM |

---

## Memory Map

### AT32F435/437 4MB Variant (xMxx)

| Region | Start Address | End Address | Size |
|--------|---------------|-------------|------|
| Bank1 | 0x0800_0000 | 0x081F_FFFF | 2 MB |
| Bank2 | 0x0820_0000 | 0x083E_FFFF | ~2 MB |
| System Memory | 0x1FFF_0000 | 0x1FFF_7FFF | 32 KB |
| User System Data | 0x1FFF_C000 | 0x1FFF_C0FF | 256 B |

### AT32F435/437 1MB Variant (xGxx)

| Region | Start Address | End Address | Size |
|--------|---------------|-------------|------|
| Bank1 | 0x0800_0000 | 0x0807_FFFF | 512 KB |
| Bank2 | 0x0808_0000 | 0x080F_FFFF | 512 KB |
| System Memory | 0x1FFF_0000 | 0x1FFF_7FFF | 32 KB |
| User System Data | 0x1FFF_C000 | 0x1FFF_C0FF | 256 B |

### Zero-Wait Area (NZW) / Extended SRAM

The EOPB0 byte in USD controls the SRAM/Flash ZW area partitioning:

| EOPB0 Value | SRAM Size | Flash ZW Size |
|-------------|-----------|---------------|
| 0x00 | 512 KB | 128 KB |
| 0x01 | 448 KB | 192 KB |
| 0x02 | 384 KB | 256 KB |
| 0x03 | 320 KB | 320 KB |
| 0x04 | 256 KB | 384 KB |
| 0x05 | 192 KB | 448 KB |
| 0x06 | 128 KB | 512 KB |

> **Note:** 256 KB and smaller MCUs only support EOPB0 values 0x00, 0x01, 0x02.

---

## Architecture

```
                         ┌─────────────────────────────────────────────────────┐
                         │               FLASH Controller                       │
                         │                                                      │
  ┌─────────────────┐   │   ┌───────────┐        ┌───────────────────┐        │
  │   AHB Bus       │───┼──►│  Unlock   │        │    Status         │        │
  │   (Program)     │   │   │  Logic    │        │    Registers      │        │
  └─────────────────┘   │   │ KEY1/KEY2 │        │  STS, STS2        │        │
                         │   └─────┬─────┘        └─────────┬─────────┘        │
                         │         │                        │                  │
                         │   ┌─────▼─────────────────────────▼─────────┐       │
  ┌─────────────────┐   │   │            Control Logic                 │       │
  │   I-Code Bus    │───┼──►│    CTRL (Bank1)  │  CTRL2 (Bank2)       │       │
  │   (Fetch)       │   │   │                                          │       │
  └─────────────────┘   │   └─────────────────┬────────────────────────┘       │
                         │                     │                                │
                         │         ┌───────────┴───────────┐                   │
                         │         ▼                       ▼                   │
                         │   ┌───────────┐           ┌───────────┐             │
                         │   │  Bank 1   │           │  Bank 2   │             │
                         │   │           │           │           │             │
                         │   │ Sectors   │           │ Sectors   │             │
                         │   │ 0x0800... │           │ 0x0808... │             │
                         │   │           │           │ or        │             │
                         │   │           │           │ 0x0820... │             │
                         │   └───────────┘           └───────────┘             │
                         │                                                      │
                         │   ┌─────────────────────────────────────────┐       │
                         │   │      User System Data (USD)             │       │
                         │   │  FAP │ SSB │ DATA │ EPP │ EOPB0 │ SLIB  │       │
                         │   │  0x1FFFC000                              │       │
                         │   └─────────────────────────────────────────┘       │
                         │                                                      │
                         │   ┌──────────────────┐    ┌──────────────────┐      │
                         │   │  SLIB (Security  │    │  CRC Calculator  │      │
                         │   │  Library)        │    │                  │      │
                         │   └──────────────────┘    └──────────────────┘      │
                         └─────────────────────────────────────────────────────┘
```

---

## Register Map

### Bank1 Registers

| Register | Offset | Description |
|----------|--------|-------------|
| PSR | 0x00 | Prefetch/Status Register |
| UNLOCK | 0x04 | Flash unlock register |
| USD_UNLOCK | 0x08 | USD unlock register |
| STS | 0x0C | Status register |
| CTRL | 0x10 | Control register |
| ADDR | 0x14 | Address register |
| USD | 0x1C | User System Data register |
| EPPS0 | 0x20 | EPP status 0 |
| EPPS1 | 0x2C | EPP status 1 |

### Bank2 Registers

| Register | Offset | Description |
|----------|--------|-------------|
| UNLOCK2 | 0x44 | Bank2 unlock register |
| STS2 | 0x4C | Bank2 status register |
| CTRL2 | 0x50 | Bank2 control register |
| ADDR2 | 0x54 | Bank2 address register |

### Other Registers

| Register | Offset | Description |
|----------|--------|-------------|
| CONTR | 0x58 | Continue read control |
| DIVR | 0x60 | Clock divider register |
| SLIB_STS2 | 0xC8 | SLIB instruction start sector |
| SLIB_STS0 | 0xCC | SLIB status 0 |
| SLIB_STS1 | 0xD0 | SLIB status 1 |
| SLIB_PWD_CLR | 0xD4 | SLIB password clear |
| SLIB_MISC_STS | 0xD8 | SLIB misc status |
| SLIB_SET_PWD | 0xDC | SLIB set password |
| SLIB_SET_RANGE0 | 0xE0 | SLIB range 0 |
| SLIB_SET_RANGE1 | 0xE4 | SLIB range 1 |
| SLIB_UNLOCK | 0xF0 | SLIB unlock |
| CRC_CTRL | 0xF4 | CRC control |
| CRC_CHKR | 0xF8 | CRC check result |

---

## Status Flags

| Flag | Description |
|------|-------------|
| FLASH_OBF_FLAG | Operation busy flag |
| FLASH_ODF_FLAG | Operation done flag |
| FLASH_PRGMERR_FLAG | Program error flag |
| FLASH_EPPERR_FLAG | Erase/program protection error flag |
| FLASH_BANK1_xxx | Bank1 specific flags |
| FLASH_BANK2_xxx | Bank2 specific flags |
| FLASH_USDERR_FLAG | User system data error flag |

---

## Operation Status

```c
typedef enum {
  FLASH_OPERATE_BUSY    = 0x00,  /* Flash operation in progress */
  FLASH_PROGRAM_ERROR   = 0x01,  /* Programming error */
  FLASH_EPP_ERROR       = 0x02,  /* Erase/program protection error */
  FLASH_OPERATE_DONE    = 0x03,  /* Operation completed successfully */
  FLASH_OPERATE_TIMEOUT = 0x04   /* Operation timeout */
} flash_status_type;
```

---

## Unlock Keys

| Key | Value | Purpose |
|-----|-------|---------|
| FLASH_UNLOCK_KEY1 | 0x45670123 | First unlock key |
| FLASH_UNLOCK_KEY2 | 0xCDEF89AB | Second unlock key |
| FAP_RELIEVE_KEY | 0x00A5 | Disable flash access protection |
| SLIB_UNLOCK_KEY | 0xA35F6D24 | SLIB configuration unlock |

---

## User System Data (USD)

The USD area stores important configuration and protection settings:

| Field | Offset | Description |
|-------|--------|-------------|
| FAP | 0x00 | Flash Access Protection (0xA5 = disabled) |
| SSB | 0x02 | System Setting Byte |
| DATA0 | 0x04 | User data byte 0 |
| DATA1 | 0x06 | User data byte 1 |
| EPP0-3 | 0x08-0x0E | Erase/Program Protection bits 0-31 |
| EOPB0 | 0x10 | Extended Option Byte 0 (SRAM config) |
| EPP4-7 | 0x14-0x1A | Erase/Program Protection bits 32-63 |
| QSPIKEY | 0x2C-0x3A | QSPI encryption key (8 halfwords) |

### SSB (System Setting Byte) Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0 | WDT_ATO_EN | WDT auto start (0=enabled, 1=disabled) |
| 1 | DEPSLP_RST | Reset on deep sleep (0=reset, 1=no reset) |
| 2 | STDBY_RST | Reset on standby (0=reset, 1=no reset) |
| 3 | BTOPT | Boot option (0=Bank2/1, 1=Bank1 only) |
| 5 | WDT_DEPSLP | WDT in deep sleep (0=stop, 1=continue) |
| 6 | WDT_STDBY | WDT in standby (0=stop, 1=continue) |

---

## API Reference

### Unlock/Lock Functions

```c
/**
  * @brief  Unlock both flash banks for program/erase operations
  * @retval none
  */
void flash_unlock(void);

/**
  * @brief  Unlock flash Bank1 only
  * @retval none
  */
void flash_bank1_unlock(void);

/**
  * @brief  Unlock flash Bank2 only
  * @retval none
  */
void flash_bank2_unlock(void);

/**
  * @brief  Lock both flash banks (protect from unwanted writes)
  * @retval none
  */
void flash_lock(void);

/**
  * @brief  Lock flash Bank1 only
  * @retval none
  */
void flash_bank1_lock(void);

/**
  * @brief  Lock flash Bank2 only
  * @retval none
  */
void flash_bank2_lock(void);
```

### Erase Functions

```c
/**
  * @brief  Erase a flash sector (4KB or 2KB depending on model)
  * @param  sector_address: address within the sector to erase
  * @retval flash_status_type
  */
flash_status_type flash_sector_erase(uint32_t sector_address);

/**
  * @brief  Erase a flash block (64KB)
  * @param  block_address: address within the block to erase
  * @retval flash_status_type
  */
flash_status_type flash_block_erase(uint32_t block_address);

/**
  * @brief  Erase Bank1 completely
  * @retval flash_status_type
  */
flash_status_type flash_bank1_erase(void);

/**
  * @brief  Erase Bank2 completely
  * @retval flash_status_type
  */
flash_status_type flash_bank2_erase(void);

/**
  * @brief  Mass erase all internal flash (both banks)
  * @retval flash_status_type
  */
flash_status_type flash_internal_all_erase(void);

/**
  * @brief  Erase user system data (preserves FAP)
  * @retval flash_status_type
  * @note   EOPB0 resets to 0xFF, SRAM size may change
  */
flash_status_type flash_user_system_data_erase(void);
```

### Program Functions

```c
/**
  * @brief  Program a word (32-bit) at specified address
  * @param  address: target address (word-aligned recommended)
  * @param  data: 32-bit data to program
  * @retval flash_status_type
  */
flash_status_type flash_word_program(uint32_t address, uint32_t data);

/**
  * @brief  Program a halfword (16-bit) at specified address
  * @param  address: target address (halfword-aligned recommended)
  * @param  data: 16-bit data to program
  * @retval flash_status_type
  */
flash_status_type flash_halfword_program(uint32_t address, uint16_t data);

/**
  * @brief  Program a byte (8-bit) at specified address
  * @param  address: target address
  * @param  data: 8-bit data to program
  * @retval flash_status_type
  * @note   Cannot be used to program SPIM flash
  */
flash_status_type flash_byte_program(uint32_t address, uint8_t data);

/**
  * @brief  Program a byte in User System Data area
  * @param  address: target USD address
  * @param  data: 8-bit data to program
  * @retval flash_status_type
  */
flash_status_type flash_user_system_data_program(uint32_t address, uint8_t data);
```

### Status Functions

```c
/**
  * @brief  Get flash flag status
  * @param  flash_flag: flag to check (FLASH_xxx_FLAG)
  * @retval flag_status: SET or RESET
  */
flag_status flash_flag_get(uint32_t flash_flag);

/**
  * @brief  Clear flash flag(s)
  * @param  flash_flag: flag(s) to clear
  * @retval none
  */
void flash_flag_clear(uint32_t flash_flag);

/**
  * @brief  Get current flash operation status
  * @retval flash_status_type
  */
flash_status_type flash_operation_status_get(void);

/**
  * @brief  Get Bank1 operation status
  * @retval flash_status_type
  */
flash_status_type flash_bank1_operation_status_get(void);

/**
  * @brief  Get Bank2 operation status
  * @retval flash_status_type
  */
flash_status_type flash_bank2_operation_status_get(void);

/**
  * @brief  Wait for flash operation to complete
  * @param  time_out: timeout counter
  * @retval flash_status_type
  */
flash_status_type flash_operation_wait_for(uint32_t time_out);

/**
  * @brief  Wait for Bank1 operation to complete
  * @param  time_out: timeout counter
  * @retval flash_status_type
  */
flash_status_type flash_bank1_operation_wait_for(uint32_t time_out);

/**
  * @brief  Wait for Bank2 operation to complete
  * @param  time_out: timeout counter
  * @retval flash_status_type
  */
flash_status_type flash_bank2_operation_wait_for(uint32_t time_out);
```

### Protection Functions

```c
/**
  * @brief  Enable or disable Flash Access Protection
  * @param  new_state: TRUE (enable FAP) or FALSE (disable FAP)
  * @retval flash_status_type
  * @warning Enabling FAP protects flash from reading; disabling causes mass erase
  */
flash_status_type flash_fap_enable(confirm_state new_state);

/**
  * @brief  Get FAP status
  * @retval flag_status: SET (protected) or RESET (not protected)
  */
flag_status flash_fap_status_get(void);

/**
  * @brief  Set Erase/Program Protection for sectors
  * @param  sector_bits: pointer to 2 x uint32_t with protection bits
  *         - sector_bits[0]: bits 0-31 (4KB sectors)
  *         - sector_bits[1]: bits 32-63 (128KB sectors)
  *         - 1 = protected, 0 = not protected
  * @retval flash_status_type
  */
flash_status_type flash_epp_set(uint32_t *sector_bits);

/**
  * @brief  Get EPP status
  * @param  sector_bits: pointer to receive 2 x uint32_t status
  * @retval none
  */
void flash_epp_status_get(uint32_t *sector_bits);
```

### System Setting Functions

```c
/**
  * @brief  Set System Setting Byte
  * @param  usd_ssb: combination of USD_xxx settings
  * @retval flash_status_type
  */
flash_status_type flash_ssb_set(uint8_t usd_ssb);

/**
  * @brief  Get SSB status
  * @retval uint8_t: SSB value
  */
uint8_t flash_ssb_status_get(void);

/**
  * @brief  Configure EOPB0 (SRAM/Flash partitioning)
  * @param  data: FLASH_EOPB0_SRAM_xxxK value
  * @retval flash_status_type
  */
flash_status_type flash_eopb0_config(flash_usd_eopb0_type data);
```

### SLIB (Security Library) Functions

```c
/**
  * @brief  Enable Security Library protection
  * @param  pwd: 32-bit password
  * @param  start_sector: SLIB start sector number
  * @param  inst_start_sector: I-bus area start sector (0xFFFF to disable)
  * @param  end_sector: SLIB end sector number
  * @retval flash_status_type
  */
flash_status_type flash_slib_enable(uint32_t pwd, uint16_t start_sector, 
                                     uint16_t inst_start_sector, uint16_t end_sector);

/**
  * @brief  Disable SLIB with password
  * @param  pwd: 32-bit password (must match enabled password)
  * @retval error_status: SUCCESS or ERROR
  */
error_status flash_slib_disable(uint32_t pwd);

/**
  * @brief  Get SLIB state
  * @retval flag_status: SET (enabled) or RESET (disabled)
  */
flag_status flash_slib_state_get(void);

/**
  * @brief  Get remaining SLIB password attempts (256 max)
  * @retval uint32_t: remaining attempts
  */
uint32_t flash_slib_remaining_count_get(void);

/**
  * @brief  Get SLIB start sector
  * @retval uint16_t: sector number
  */
uint16_t flash_slib_start_sector_get(void);

/**
  * @brief  Get SLIB instruction start sector
  * @retval uint16_t: sector number
  */
uint16_t flash_slib_inststart_sector_get(void);

/**
  * @brief  Get SLIB end sector
  * @retval uint16_t: sector number
  */
uint16_t flash_slib_end_sector_get(void);
```

### Special Functions

```c
/**
  * @brief  Calculate CRC of flash sectors
  * @param  start_sector: starting sector number
  * @param  sector_cnt: number of sectors
  * @retval uint32_t: CRC result
  */
uint32_t flash_crc_calibrate(uint32_t start_sector, uint32_t sector_cnt);

/**
  * @brief  Enable/disable Non-Zero Wait area boost
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void flash_nzw_boost_enable(confirm_state new_state);

/**
  * @brief  Enable/disable continue read mode
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void flash_continue_read_enable(confirm_state new_state);

/**
  * @brief  Enable flash interrupts
  * @param  flash_int: FLASH_xxx_INT flags
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void flash_interrupt_enable(uint32_t flash_int, confirm_state new_state);

/* Macro for setting flash clock divider */
#define flash_clock_divider_set(div)  (FLASH->divr_bit.fdiv = div)
/* div can be: FLASH_CLOCK_DIV_2, FLASH_CLOCK_DIV_3, FLASH_CLOCK_DIV_4 */
```

---

## Code Examples

### Example 1: Basic Write and Read

Write data to flash and read it back:

```c
#include "at32f435_437.h"

#define FLASH_TEST_ADDR  (0x08000000 + 0x10000)  /* 64KB offset */
#define TEST_DATA_SIZE   64

uint32_t write_buffer[TEST_DATA_SIZE];
uint32_t read_buffer[TEST_DATA_SIZE];

/**
  * @brief  Write array of words to flash
  */
flash_status_type flash_write_words(uint32_t address, uint32_t *data, uint32_t count)
{
    flash_status_type status;
    uint32_t i;
    
    for(i = 0; i < count; i++)
    {
        status = flash_word_program(address + (i * 4), data[i]);
        if(status != FLASH_OPERATE_DONE)
            return status;
    }
    return FLASH_OPERATE_DONE;
}

/**
  * @brief  Read array of words from flash
  */
void flash_read_words(uint32_t address, uint32_t *data, uint32_t count)
{
    uint32_t i;
    for(i = 0; i < count; i++)
    {
        data[i] = *(__IO uint32_t*)(address + (i * 4));
    }
}

int main(void)
{
    flash_status_type status;
    uint32_t i;
    
    system_clock_config();
    
    /* Initialize test data */
    for(i = 0; i < TEST_DATA_SIZE; i++)
    {
        write_buffer[i] = 0xDEADBEEF + i;
    }
    
    /* Unlock flash */
    flash_unlock();
    
    /* Erase sector first (flash can only write 1->0) */
    status = flash_sector_erase(FLASH_TEST_ADDR);
    if(status != FLASH_OPERATE_DONE)
    {
        flash_lock();
        while(1);  /* Error handling */
    }
    
    /* Write data */
    status = flash_write_words(FLASH_TEST_ADDR, write_buffer, TEST_DATA_SIZE);
    if(status != FLASH_OPERATE_DONE)
    {
        flash_lock();
        while(1);  /* Error handling */
    }
    
    /* Lock flash */
    flash_lock();
    
    /* Read back and verify */
    flash_read_words(FLASH_TEST_ADDR, read_buffer, TEST_DATA_SIZE);
    
    for(i = 0; i < TEST_DATA_SIZE; i++)
    {
        if(read_buffer[i] != write_buffer[i])
        {
            while(1);  /* Verification failed */
        }
    }
    
    /* Success */
    while(1);
}
```

---

### Example 2: Sector-Aware Write with Read-Modify-Write

Handle writes that span sectors or require preserving existing data:

```c
#include "at32f435_437.h"

/* Sector size depends on chip variant */
#if defined (AT32F437xM) || defined (AT32F435xM)
#define SECTOR_SIZE  4096
#else
#define SECTOR_SIZE  2048
#endif

uint16_t sector_buffer[SECTOR_SIZE / 2];

/**
  * @brief  Get sector start address
  */
uint32_t get_sector_address(uint32_t address)
{
    return (address / SECTOR_SIZE) * SECTOR_SIZE;
}

/**
  * @brief  Write halfwords with automatic erase when needed
  */
error_status flash_write_safe(uint32_t address, uint16_t *data, uint16_t count)
{
    uint32_t sector_addr;
    uint16_t sector_offset;
    uint16_t write_count;
    uint16_t i;
    flash_status_type status;
    confirm_state need_erase;
    
    flash_unlock();
    
    while(count > 0)
    {
        sector_addr = get_sector_address(address);
        sector_offset = (address - sector_addr) / 2;
        write_count = (SECTOR_SIZE / 2) - sector_offset;
        if(write_count > count)
            write_count = count;
        
        /* Read entire sector */
        for(i = 0; i < SECTOR_SIZE / 2; i++)
        {
            sector_buffer[i] = *(__IO uint16_t*)(sector_addr + i * 2);
        }
        
        /* Check if erase is needed (can only write 1->0) */
        need_erase = FALSE;
        for(i = 0; i < write_count; i++)
        {
            if((sector_buffer[sector_offset + i] & data[i]) != data[i])
            {
                need_erase = TRUE;
                break;
            }
        }
        
        /* Merge new data into buffer */
        for(i = 0; i < write_count; i++)
        {
            sector_buffer[sector_offset + i] = data[i];
        }
        
        if(need_erase)
        {
            /* Erase sector */
            status = flash_sector_erase(sector_addr);
            if(status != FLASH_OPERATE_DONE)
            {
                flash_lock();
                return ERROR;
            }
            
            /* Write entire sector back */
            for(i = 0; i < SECTOR_SIZE / 2; i++)
            {
                status = flash_halfword_program(sector_addr + i * 2, sector_buffer[i]);
                if(status != FLASH_OPERATE_DONE)
                {
                    flash_lock();
                    return ERROR;
                }
            }
        }
        else
        {
            /* Just write new data */
            for(i = 0; i < write_count; i++)
            {
                status = flash_halfword_program(address + i * 2, data[i]);
                if(status != FLASH_OPERATE_DONE)
                {
                    flash_lock();
                    return ERROR;
                }
            }
        }
        
        address += write_count * 2;
        data += write_count;
        count -= write_count;
    }
    
    flash_lock();
    return SUCCESS;
}
```

---

### Example 3: User System Data Programming

Program user data bytes in USD area:

```c
#include "at32f435_437.h"

/**
  * @brief  Write user data to USD DATA0 and DATA1
  */
flash_status_type write_user_data(uint8_t data0, uint8_t data1)
{
    flash_status_type status;
    
    /* Erase USD first (preserves FAP status) */
    status = flash_user_system_data_erase();
    if(status != FLASH_OPERATE_DONE)
        return status;
    
    /* Program DATA0 */
    status = flash_user_system_data_program((uint32_t)&USD->data0, data0);
    if(status != FLASH_OPERATE_DONE)
        return status;
    
    /* Program DATA1 */
    status = flash_user_system_data_program((uint32_t)&USD->data1, data1);
    
    return status;
}

/**
  * @brief  Read user data from USD
  */
void read_user_data(uint8_t *data0, uint8_t *data1)
{
    *data0 = (uint8_t)(FLASH->usd_bit.user_d0);
    *data1 = (uint8_t)(FLASH->usd_bit.user_d1);
}

/**
  * @brief  Configure watchdog auto-start setting
  */
flash_status_type configure_wdt_autostart(confirm_state enable)
{
    uint8_t ssb = flash_ssb_status_get();
    
    if(enable)
        ssb &= ~USD_WDT_ATO_DISABLE;  /* Clear bit = enable */
    else
        ssb |= USD_WDT_ATO_DISABLE;   /* Set bit = disable */
    
    return flash_ssb_set(ssb);
}
```

---

### Example 4: Erase/Program Protection (EPP)

Protect specific sectors from accidental modification:

```c
#include "at32f435_437.h"

/**
  * @brief  Protect first 32KB of flash (sectors 0-7 for 4KB sectors)
  */
flash_status_type protect_bootloader_area(void)
{
    uint32_t epp_bits[2] = {0, 0};
    
    /* Get current protection status */
    flash_epp_status_get(epp_bits);
    
    /* Set protection for sectors 0-7 (bits 0-7) */
    /* Note: EPP register uses inverted logic (0 = protected) */
    /* But the API inverts it for us (1 = protect) */
    epp_bits[0] |= 0x000000FF;  /* Protect sectors 0-7 */
    
    return flash_epp_set(epp_bits);
}

/**
  * @brief  Check if a sector is protected
  */
confirm_state is_sector_protected(uint8_t sector)
{
    uint32_t epp_bits[2];
    flash_epp_status_get(epp_bits);
    
    if(sector < 32)
    {
        /* EPPS0: bits 0-31 protect 4KB sectors */
        return ((epp_bits[0] & (1 << sector)) == 0) ? TRUE : FALSE;
    }
    else
    {
        /* EPPS1: bits 0-31 protect 128KB blocks */
        sector -= 32;
        return ((epp_bits[1] & (1 << sector)) == 0) ? TRUE : FALSE;
    }
}

/**
  * @brief  Remove all sector protection (requires USD erase)
  */
flash_status_type remove_all_protection(void)
{
    /* Erasing USD removes EPP settings */
    return flash_user_system_data_erase();
}
```

---

### Example 5: Security Library (SLIB) for IP Protection

Protect code from unauthorized access:

```c
#include "at32f435_437.h"

#define SLIB_PASSWORD     0x12345678
#define SLIB_START_SECTOR 8       /* Start at sector 8 */
#define SLIB_INST_SECTOR  8       /* I-bus starts at same sector */
#define SLIB_END_SECTOR   15      /* End at sector 15 */

/**
  * @brief  Enable SLIB protection for proprietary code
  * @note   After enabling, protected code can execute but cannot be read
  */
flash_status_type enable_code_protection(void)
{
    /* Check if SLIB is already enabled */
    if(flash_slib_state_get() == SET)
    {
        return FLASH_OPERATE_DONE;  /* Already protected */
    }
    
    /* Enable SLIB */
    return flash_slib_enable(SLIB_PASSWORD, 
                             SLIB_START_SECTOR, 
                             SLIB_INST_SECTOR, 
                             SLIB_END_SECTOR);
}

/**
  * @brief  Disable SLIB protection (for development)
  * @note   Limited attempts (256) - chip becomes permanently locked if exceeded
  */
error_status disable_code_protection(void)
{
    uint32_t remaining;
    
    /* Check remaining attempts */
    remaining = flash_slib_remaining_count_get();
    if(remaining == 0)
    {
        return ERROR;  /* No attempts left - chip is locked */
    }
    
    /* Attempt to disable with password */
    return flash_slib_disable(SLIB_PASSWORD);
}

/**
  * @brief  Get SLIB status information
  */
void get_slib_info(void)
{
    if(flash_slib_state_get() == SET)
    {
        printf("SLIB is ENABLED\n");
        printf("Start sector: %d\n", flash_slib_start_sector_get());
        printf("Inst sector:  %d\n", flash_slib_inststart_sector_get());
        printf("End sector:   %d\n", flash_slib_end_sector_get());
        printf("Attempts left: %lu\n", flash_slib_remaining_count_get());
    }
    else
    {
        printf("SLIB is DISABLED\n");
    }
}
```

---

### Example 6: Flash CRC Verification

Calculate CRC to verify flash contents:

```c
#include "at32f435_437.h"

/**
  * @brief  Calculate and store CRC during manufacturing
  */
uint32_t calculate_firmware_crc(void)
{
    /* Calculate CRC of sectors 0-31 (128KB for 4KB sectors) */
    return flash_crc_calibrate(0, 32);
}

/**
  * @brief  Verify firmware integrity at runtime
  */
error_status verify_firmware_integrity(uint32_t expected_crc)
{
    uint32_t calculated_crc;
    
    calculated_crc = flash_crc_calibrate(0, 32);
    
    if(calculated_crc == expected_crc)
        return SUCCESS;
    else
        return ERROR;
}

/**
  * @brief  Store CRC in last sector of application area
  */
flash_status_type store_crc(uint32_t crc_address, uint32_t crc_value)
{
    flash_status_type status;
    
    flash_unlock();
    
    /* Erase sector containing CRC */
    status = flash_sector_erase(crc_address);
    if(status == FLASH_OPERATE_DONE)
    {
        status = flash_word_program(crc_address, crc_value);
    }
    
    flash_lock();
    return status;
}
```

---

## Configuration Checklist

### Basic Flash Operations

- [ ] Call `flash_unlock()` before any program/erase operation
- [ ] Erase target sector before programming (flash writes 1→0 only)
- [ ] Wait for operation completion or check status
- [ ] Call `flash_lock()` after operations complete
- [ ] Handle errors appropriately

### User System Data

- [ ] USD modifications require `flash_user_system_data_erase()` first
- [ ] FAP status is preserved during USD erase
- [ ] EOPB0 changes require system reset to take effect
- [ ] SSB changes may require reset

### Protection Mechanisms

- [ ] FAP prevents external read access (via debug interface)
- [ ] Disabling FAP triggers mass erase
- [ ] EPP prevents accidental sector modification
- [ ] SLIB protects IP with password (limited attempts)

---

## Troubleshooting

### Common Issues

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Program fails | Flash not unlocked | Call `flash_unlock()` first |
| | Target not erased | Erase sector before programming |
| | EPP protection enabled | Check and clear EPP settings |
| Erase fails | Flash locked | Unlock flash |
| | EPP protection | Remove EPP from USD |
| | SLIB protection | Disable SLIB with password |
| Data corruption | Interrupted operation | Add power-fail protection |
| | Improper alignment | Use aligned addresses |
| Timeout | Clock configuration | Check flash clock divider |
| | Large operation | Increase timeout value |
| Cannot read USD | FAP enabled | Reset or disable FAP |

### Timing Considerations

```c
/* Default timeout values */
#define ERASE_TIMEOUT       0x80000000  /* Sector/bank erase */
#define PROGRAMMING_TIMEOUT 0x00100000  /* Word/byte program */
#define OPERATION_TIMEOUT   0x10000000  /* General operations */
```

### Flash Clock Configuration

For high-frequency operation, adjust the flash clock divider:

```c
/* At system clock > 200 MHz */
flash_clock_divider_set(FLASH_CLOCK_DIV_3);

/* At system clock > 250 MHz */
flash_clock_divider_set(FLASH_CLOCK_DIV_4);
```

---

## Related Peripherals

| Peripheral | Relationship |
|------------|-------------|
| [CRM](CRM_Clock_Reset_Management.md) | Provides flash clock |
| [PWC](PWC_Power_Controller.md) | Low-power mode affects flash access |
| [DEBUG](DEBUG_MCU_Debug_Support.md) | Debug access affected by FAP |
| [QSPI](QSPI_Quad_SPI.md) | External flash via QSPI with encryption key |

---

## References

- AT32F435/437 Reference Manual - Chapter: Embedded Flash
- AT32F435/437 Datasheet - Flash memory specifications
- Application Note AN0145 - Flash Programming Guide
- Application Note AN0044 - Security Library Usage

