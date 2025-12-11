---
title: SRAM - Extended Internal Memory Configuration
category: System
complexity: Intermediate
mcu: AT32F435/437
peripheral: SRAM
keywords: [sram, memory, eopb0, flash, zw, partition, option byte, extend]
---

# SRAM - Extended Internal Memory Configuration

## Overview

The AT32F435/437 features a flexible internal memory architecture that allows partitioning between SRAM and Zero-Wait (ZW) Flash memory. The MCU provides a total of 640KB of configurable memory space that can be allocated between SRAM (128KB to 512KB) and ZW Flash area (128KB to 512KB) using the Extended Option Programming Byte 0 (EOPB0) in the User System Data (USD).

### Key Features

| Feature | Specification |
|---------|---------------|
| Total Configurable Memory | 640 KB |
| SRAM Range | 128 KB to 512 KB |
| ZW Flash Range | 128 KB to 512 KB (in 64KB steps) |
| Configuration Storage | EOPB0 in User System Data (USD) |
| Configuration Options | 7 predefined sizes |
| Default Configuration | 384 KB SRAM, 256 KB ZW Flash |

### Memory Configuration Options

| EOPB0 Value | Enumeration | SRAM Size | ZW Flash Size |
|-------------|-------------|-----------|---------------|
| 0x00 | `FLASH_EOPB0_SRAM_512K` | 512 KB | 128 KB |
| 0x01 | `FLASH_EOPB0_SRAM_448K` | 448 KB | 192 KB |
| 0x02 | `FLASH_EOPB0_SRAM_384K` | 384 KB (default) | 256 KB |
| 0x03 | `FLASH_EOPB0_SRAM_320K` | 320 KB | 320 KB |
| 0x04 | `FLASH_EOPB0_SRAM_256K` | 256 KB | 384 KB |
| 0x05 | `FLASH_EOPB0_SRAM_192K` | 192 KB | 448 KB |
| 0x06 | `FLASH_EOPB0_SRAM_128K` | 128 KB | 512 KB |

> **Note:** For 256KB and below Flash capacity MCU variants, only `FLASH_EOPB0_SRAM_384K`, `FLASH_EOPB0_SRAM_448K`, and `FLASH_EOPB0_SRAM_512K` are supported.

## Memory Architecture

```
                        AT32F435/437 Memory Map
    ┌─────────────────────────────────────────────────────────────────┐
    │                                                                  │
    │   0x2000 0000  ┌─────────────────────────────────────┐          │
    │                │                                     │          │
    │                │           SRAM                      │          │
    │                │     (128KB - 512KB based on EOPB0)  │          │
    │                │                                     │          │
    │   SRAM End     ├─────────────────────────────────────┤          │
    │   (varies)     │                                     │          │
    │                │       Reserved / Not accessible     │          │
    │                │                                     │          │
    │   0x200A 0000  └─────────────────────────────────────┘          │
    │                                                                  │
    │   ─────────────────────────────────────────────────────────     │
    │                                                                  │
    │   0x0800 0000  ┌─────────────────────────────────────┐          │
    │                │                                     │          │
    │                │      Main Flash (Bank 1 + Bank 2)   │          │
    │                │        (256KB - 4032KB variants)    │          │
    │                │                                     │          │
    │                ├─────────────────────────────────────┤          │
    │                │                                     │          │
    │                │      ZW (Zero-Wait) Flash Area      │          │
    │                │    (128KB - 512KB based on EOPB0)   │          │
    │                │                                     │          │
    │   Flash End    └─────────────────────────────────────┘          │
    │                                                                  │
    └─────────────────────────────────────────────────────────────────┘
```

## SRAM Address Ranges by Configuration

| Configuration | SRAM Start | SRAM End | Size |
|---------------|------------|----------|------|
| 512K SRAM | 0x2000 0000 | 0x2007 FFFF | 512 KB |
| 448K SRAM | 0x2000 0000 | 0x2006 FFFF | 448 KB |
| 384K SRAM | 0x2000 0000 | 0x2005 FFFF | 384 KB |
| 320K SRAM | 0x2000 0000 | 0x2004 FFFF | 320 KB |
| 256K SRAM | 0x2000 0000 | 0x2003 FFFF | 256 KB |
| 192K SRAM | 0x2000 0000 | 0x2002 FFFF | 192 KB |
| 128K SRAM | 0x2000 0000 | 0x2001 FFFF | 128 KB |

## User System Data (USD) Structure

The EOPB0 byte is stored in the User System Data area of Flash memory:

```c
/**
  * @brief user system data
  */
typedef struct
{
  __IO uint16_t fap;        /* Flash Access Protection */
  __IO uint16_t ssb;        /* System Setting Byte */
  __IO uint16_t data0;      /* User data 0 */
  __IO uint16_t data1;      /* User data 1 */
  __IO uint16_t epp0;       /* Erase/Program Protection 0 */
  __IO uint16_t epp1;       /* Erase/Program Protection 1 */
  __IO uint16_t epp2;       /* Erase/Program Protection 2 */
  __IO uint16_t epp3;       /* Erase/Program Protection 3 */
  __IO uint16_t eopb0;      /* Extended Option Programming Byte 0 (SRAM config) */
  __IO uint16_t reserved1;
  __IO uint16_t epp4;       /* Erase/Program Protection 4 */
  __IO uint16_t epp5;       /* Erase/Program Protection 5 */
  __IO uint16_t epp6;       /* Erase/Program Protection 6 */
  __IO uint16_t epp7;       /* Erase/Program Protection 7 */
  __IO uint16_t reserved2[12];
  __IO uint16_t qspikey[8]; /* QSPI decryption key */
} usd_type;

#define USD  ((usd_type *) USD_BASE)
```

## Configuration Type

```c
/**
  * @brief  flash usd eopb0 type
  */
typedef enum
{
  FLASH_EOPB0_SRAM_512K = 0x00,  /* SRAM 512KB, ZW Flash 128KB */
  FLASH_EOPB0_SRAM_448K = 0x01,  /* SRAM 448KB, ZW Flash 192KB */
  FLASH_EOPB0_SRAM_384K = 0x02,  /* SRAM 384KB, ZW Flash 256KB (default) */
  FLASH_EOPB0_SRAM_320K = 0x03,  /* SRAM 320KB, ZW Flash 320KB */
  FLASH_EOPB0_SRAM_256K = 0x04,  /* SRAM 256KB, ZW Flash 384KB */
  FLASH_EOPB0_SRAM_192K = 0x05,  /* SRAM 192KB, ZW Flash 448KB */
  FLASH_EOPB0_SRAM_128K = 0x06   /* SRAM 128KB, ZW Flash 512KB */
} flash_usd_eopb0_type;
```

## API Reference

### EOPB0 Configuration

```c
/**
  * @brief  Config the extend SRAM byte EOPB0 in user system data.
  * @note   The 256KB and below capacity MCU only support FLASH_EOPB0_SRAM_384K,
  *         FLASH_EOPB0_SRAM_448K or FLASH_EOPB0_SRAM_512K.
  * @param  data: the EOPB0 value.
  *         This parameter can be one of:
  *         - FLASH_EOPB0_SRAM_512K
  *         - FLASH_EOPB0_SRAM_448K
  *         - FLASH_EOPB0_SRAM_384K
  *         - FLASH_EOPB0_SRAM_320K
  *         - FLASH_EOPB0_SRAM_256K
  *         - FLASH_EOPB0_SRAM_192K
  *         - FLASH_EOPB0_SRAM_128K
  * @retval Flash status
  */
flash_status_type flash_eopb0_config(flash_usd_eopb0_type data);
```

### Supporting Flash Functions

```c
/* Unlock flash for programming */
void flash_unlock(void);

/* Lock flash after programming */
void flash_lock(void);

/* Erase User System Data (required before modifying EOPB0) */
flash_status_type flash_user_system_data_erase(void);
```

## Implementation Requirements

### Modified Startup File

When extending SRAM, the startup file must be modified to call the SRAM extension function **before** `SystemInit` and `main` are called. This is critical because:

1. The default stack pointer may be outside the valid SRAM range
2. The SRAM configuration must be set before any memory operations

**Keil MDK Startup Assembly (startup_at32f435_437_ext_ram.s):**

```asm
; Reset handler
Reset_Handler   PROC
                EXPORT  Reset_Handler                       [WEAK]
                IMPORT  __main
                IMPORT  SystemInit
; Add for extend SRAM
                IMPORT  extend_sram
                MOV32   R0, #0x20001000    ; Set temporary stack in safe SRAM area
                MOV     SP, R0
                LDR     R0, =extend_sram   ; Call SRAM extension function
                BLX     R0
                MOV32   R0, #0x08000000    ; Restore stack from vector table
                LDR     SP, [R0]
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP
```

### SRAM Extension Function

```c
/**
  * @brief  Extend SRAM size by configuring EOPB0
  * @param  none
  * @retval none
  */
void extend_sram(void)
{
  /* Check if SRAM has been set to expected size */
  if(((USD->eopb0) & 0x07) != EXTEND_SRAM)
  {
    /* Unlock flash for programming */
    flash_unlock();
    
    /* Erase user system data bytes (required before modification) */
    flash_user_system_data_erase();
    
    /* Change SRAM size configuration */
    flash_eopb0_config(EXTEND_SRAM);
    
    /* System reset to apply new configuration */
    nvic_system_reset();
  }
}
```

### Project Configuration Define

Add the desired SRAM configuration as a project define:

```c
/* In project settings or header file */
#define EXTEND_SRAM  FLASH_EOPB0_SRAM_512K  /* For 512KB SRAM */
```

## Code Examples

### Example 1: Extend SRAM to Maximum (512KB)

```c
/**
 * @brief  Configure maximum SRAM (512KB)
 * @note   Requires modified startup file
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/* Target SRAM configuration */
#define EXTEND_SRAM  FLASH_EOPB0_SRAM_512K

/**
 * @brief  Extend SRAM to 512KB
 * @note   Called from startup before SystemInit
 */
void extend_sram(void)
{
  /* Check if SRAM is already configured correctly */
  if(((USD->eopb0) & 0x07) != EXTEND_SRAM)
  {
    /* Unlock flash */
    flash_unlock();
    
    /* Erase USD (required to modify option bytes) */
    flash_user_system_data_erase();
    
    /* Configure EOPB0 for 512KB SRAM */
    flash_eopb0_config(FLASH_EOPB0_SRAM_512K);
    
    /* Reset to apply changes */
    nvic_system_reset();
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();

  /* Verify SRAM configuration */
  if(((USD->eopb0) & 0x07) == EXTEND_SRAM)
  {
    /* SRAM extended successfully */
    at32_led_on(LED4);
    
    /* Now you can use up to 512KB of SRAM */
    /* Address range: 0x20000000 - 0x2007FFFF */
  }

  while(1)
  {
  }
}
```

### Example 2: Dynamic SRAM Configuration Check

```c
/**
 * @brief  Check and display current SRAM configuration
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include <stdio.h>

typedef struct {
  flash_usd_eopb0_type config;
  const char* name;
  uint32_t sram_kb;
  uint32_t zw_flash_kb;
} sram_config_info_t;

const sram_config_info_t sram_configs[] = {
  { FLASH_EOPB0_SRAM_512K, "512K SRAM / 128K ZW", 512, 128 },
  { FLASH_EOPB0_SRAM_448K, "448K SRAM / 192K ZW", 448, 192 },
  { FLASH_EOPB0_SRAM_384K, "384K SRAM / 256K ZW", 384, 256 },
  { FLASH_EOPB0_SRAM_320K, "320K SRAM / 320K ZW", 320, 320 },
  { FLASH_EOPB0_SRAM_256K, "256K SRAM / 384K ZW", 256, 384 },
  { FLASH_EOPB0_SRAM_192K, "192K SRAM / 448K ZW", 192, 448 },
  { FLASH_EOPB0_SRAM_128K, "128K SRAM / 512K ZW", 128, 512 },
};

void print_sram_config(void)
{
  uint8_t eopb0_val = (USD->eopb0) & 0x07;
  
  printf("Current EOPB0 value: 0x%02X\r\n", eopb0_val);
  
  if(eopb0_val <= FLASH_EOPB0_SRAM_128K)
  {
    printf("Configuration: %s\r\n", sram_configs[eopb0_val].name);
    printf("SRAM Size: %lu KB\r\n", sram_configs[eopb0_val].sram_kb);
    printf("ZW Flash Size: %lu KB\r\n", sram_configs[eopb0_val].zw_flash_kb);
    printf("SRAM Range: 0x20000000 - 0x%08lX\r\n", 
           0x20000000 + (sram_configs[eopb0_val].sram_kb * 1024) - 1);
  }
  else
  {
    printf("Invalid EOPB0 configuration!\r\n");
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  uart_print_init(115200);
  
  printf("\r\n=== AT32F435/437 SRAM Configuration ===\r\n");
  print_sram_config();
  
  while(1)
  {
  }
}
```

### Example 3: Balanced Configuration (320KB each)

```c
/**
 * @brief  Configure balanced SRAM/ZW Flash (320KB each)
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define EXTEND_SRAM  FLASH_EOPB0_SRAM_320K

void extend_sram(void)
{
  if(((USD->eopb0) & 0x07) != EXTEND_SRAM)
  {
    flash_unlock();
    flash_user_system_data_erase();
    flash_eopb0_config(FLASH_EOPB0_SRAM_320K);
    nvic_system_reset();
  }
}

/* Large buffer utilizing extended SRAM */
#define BUFFER_SIZE  (256 * 1024)  /* 256KB buffer */
uint8_t large_buffer[BUFFER_SIZE] __attribute__((section(".bss")));

int main(void)
{
  uint32_t i;
  
  system_clock_config();
  at32_board_init();

  /* Verify extended SRAM is available */
  if(((USD->eopb0) & 0x07) == FLASH_EOPB0_SRAM_320K)
  {
    /* Test large buffer access */
    for(i = 0; i < BUFFER_SIZE; i++)
    {
      large_buffer[i] = (uint8_t)i;
    }
    
    /* Verify data */
    for(i = 0; i < BUFFER_SIZE; i++)
    {
      if(large_buffer[i] != (uint8_t)i)
      {
        /* Error - memory test failed */
        at32_led_on(LED2);
        while(1);
      }
    }
    
    /* Success */
    at32_led_on(LED4);
  }

  while(1)
  {
  }
}
```

### Example 4: Safe SRAM Reconfiguration with Backup

```c
/**
 * @brief  Safely change SRAM configuration with option byte backup
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/* Backup structure for USD contents */
typedef struct {
  uint16_t fap;
  uint16_t ssb;
  uint16_t data0;
  uint16_t data1;
} usd_backup_t;

usd_backup_t usd_backup;

void backup_usd(void)
{
  usd_backup.fap = USD->fap;
  usd_backup.ssb = USD->ssb;
  usd_backup.data0 = USD->data0;
  usd_backup.data1 = USD->data1;
}

flash_status_type restore_usd_and_set_eopb0(flash_usd_eopb0_type new_sram_config)
{
  flash_status_type status;
  
  /* Unlock flash */
  flash_unlock();
  
  /* Erase USD */
  status = flash_user_system_data_erase();
  if(status != FLASH_OPERATE_DONE) return status;
  
  /* Restore user data bytes */
  status = flash_user_system_data_program((uint32_t)&USD->data0, 
                                          (uint8_t)usd_backup.data0);
  if(status != FLASH_OPERATE_DONE) return status;
  
  status = flash_user_system_data_program((uint32_t)&USD->data1, 
                                          (uint8_t)usd_backup.data1);
  if(status != FLASH_OPERATE_DONE) return status;
  
  /* Set new EOPB0 */
  status = flash_eopb0_config(new_sram_config);
  
  flash_lock();
  
  return status;
}

void change_sram_config(flash_usd_eopb0_type new_config)
{
  uint8_t current = (USD->eopb0) & 0x07;
  
  if(current != new_config)
  {
    /* Backup current USD settings */
    backup_usd();
    
    /* Apply new configuration with backup restoration */
    if(restore_usd_and_set_eopb0(new_config) == FLASH_OPERATE_DONE)
    {
      /* Reset to apply */
      nvic_system_reset();
    }
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();

  /* Change to 512KB SRAM configuration */
  change_sram_config(FLASH_EOPB0_SRAM_512K);

  /* If we reach here, configuration is already correct */
  at32_led_on(LED4);
  
  while(1)
  {
  }
}
```

## Linker Script Modifications

When using extended SRAM, update the linker script to reflect the new memory size:

### Keil MDK Scatter File Example

```
LR_IROM1 0x08000000 0x00200000  {    ; 2MB Flash
  ER_IROM1 0x08000000 0x00200000  {
    *.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
  }
  
  ; For 512KB SRAM configuration
  RW_IRAM1 0x20000000 0x00080000  {  ; 512KB SRAM
    .ANY (+RW +ZI)
  }
}
```

### GCC Linker Script Example

```
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  SRAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 512K  /* Adjust for EOPB0 setting */
}
```

## Configuration Checklist

- [ ] Determine required SRAM size based on application needs
- [ ] Verify MCU variant supports desired configuration
- [ ] Create modified startup file with `extend_sram` call
- [ ] Define `EXTEND_SRAM` macro with target configuration
- [ ] Implement `extend_sram()` function in application
- [ ] Update linker script with correct SRAM size
- [ ] Test memory access across entire configured SRAM range
- [ ] Verify application boots correctly after reset

## Troubleshooting

### System Hangs at Startup

1. Ensure startup file calls `extend_sram()` before `SystemInit`
2. Verify temporary stack (0x20001000) is in valid SRAM range
3. Check that EOPB0 value is valid (0x00-0x06)

### SRAM Configuration Not Applied

1. System reset is required after modifying EOPB0
2. Ensure `flash_user_system_data_erase()` is called before `flash_eopb0_config()`
3. Verify flash is unlocked before programming

### Memory Access Fault

1. Check that linker script matches EOPB0 configuration
2. Verify stack and heap sizes fit within configured SRAM
3. Ensure no code accesses memory beyond configured SRAM end

### USD Erase Affects Other Settings

1. Back up USD contents before erasing
2. Restore FAP, SSB, user data bytes after erasing
3. Use the safe reconfiguration example as a template

## Use Cases

| SRAM Config | ZW Flash | Best For |
|-------------|----------|----------|
| 512K SRAM | 128K ZW | Large data buffers, image processing, audio processing |
| 448K SRAM | 192K ZW | Graphics applications, large lookup tables |
| 384K SRAM | 256K ZW | Balanced applications (default) |
| 320K SRAM | 320K ZW | Equal SRAM and fast code execution needs |
| 256K SRAM | 384K ZW | More code in ZW area, moderate SRAM needs |
| 192K SRAM | 448K ZW | Code-heavy applications with fast execution |
| 128K SRAM | 512K ZW | Maximum code in ZW area, minimal SRAM needs |

## Related Documents

- **[FLASH Memory Controller](FLASH_Memory_Controller.md)** - Flash operations and USD programming
- **[CRM Clock Reset Management](CRM_Clock_Reset_Management.md)** - System reset
- **AN0026** - AT32F435/437 Extended SRAM Application Note

## Related Peripherals

- **[FLASH](FLASH_Memory_Controller.md)** - USD and EOPB0 programming
- **[CRM](CRM_Clock_Reset_Management.md)** - System reset functionality
- **[SCFG](SCFG_System_Configuration.md)** - Memory remapping options

