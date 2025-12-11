---
title: XMC - External Memory Controller
category: Memory Interface
complexity: Advanced
mcu: AT32F435/437
peripheral: XMC
keywords: [xmc, external memory, sdram, sram, nor, nand, psram, lcd, pccard]
---

# XMC - External Memory Controller

## Overview

The XMC (External Memory Controller) provides a flexible interface to various external memory types including SRAM, PSRAM, NOR Flash, NAND Flash, SDRAM, PC Card, and LCD displays. It supports multiple memory banks with configurable timing parameters, bus widths, and access modes.

### Key Features

| Feature | Specification |
|---------|---------------|
| Memory Banks | 6 banks (Bank1-4 + SDRAM Bank5/6) |
| Data Bus Width | 8-bit or 16-bit |
| Address Space | Up to 256 MB per bank |
| SDRAM Support | Up to 32 MB per bank |
| NOR/SRAM Banks | 4 subbanks (64 MB each) |
| NAND Banks | 2 banks with ECC |
| PC Card Support | 1 bank |
| ECC | Hardware ECC for NAND |

### Supported Memory Types

| Memory Type | Bank | Base Address | Size |
|-------------|------|--------------|------|
| NOR/PSRAM/SRAM Subbank 1 | Bank1 | 0x6000_0000 | 64 MB |
| NOR/PSRAM/SRAM Subbank 2 | Bank1 | 0x6400_0000 | 64 MB |
| NOR/PSRAM/SRAM Subbank 3 | Bank1 | 0x6800_0000 | 64 MB |
| NOR/PSRAM/SRAM Subbank 4 | Bank1 | 0x6C00_0000 | 64 MB |
| NAND Flash | Bank2 | 0x7000_0000 | 64 MB |
| NAND Flash | Bank3 | 0x8000_0000 | 64 MB |
| PC Card | Bank4 | 0x9000_0000 | 64 MB |
| SDRAM Bank 1 | Bank5 | 0xC000_0000 | 256 MB |
| SDRAM Bank 2 | Bank6 | 0xD000_0000 | 256 MB |

---

## Architecture

### Block Diagram

```
                    ┌─────────────────────────────────────────────────────────────────────┐
                    │                      XMC Controller                                  │
                    │                                                                      │
                    │   ┌─────────────────────────────────────────────────────────────┐   │
                    │   │                    AHB Bus Interface                         │   │
                    │   │                 (Read/Write Requests)                        │   │
                    │   └───────────────────────────┬─────────────────────────────────┘   │
                    │                               │                                      │
                    │              ┌────────────────┼────────────────┐                     │
                    │              │                │                │                     │
                    │              ▼                ▼                ▼                     │
                    │   ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐      │
                    │   │   Bank 1     │  │  Bank 2/3/4  │  │    Bank 5/6          │      │
                    │   │  NOR/SRAM    │  │  NAND/PCCARD │  │     SDRAM            │      │
                    │   │  Controller  │  │  Controller  │  │    Controller        │      │
                    │   │              │  │              │  │                      │      │
                    │   │  4 Subbanks  │  │  ECC Engine  │  │  Auto-Refresh        │      │
                    │   │  Async/Sync  │  │  Ready/Busy  │  │  Commands            │      │
                    │   └──────┬───────┘  └──────┬───────┘  └──────────┬───────────┘      │
                    │          │                 │                     │                   │
                    │          ▼                 ▼                     ▼                   │
                    │   ┌─────────────────────────────────────────────────────────────┐   │
                    │   │                   Timing Generator                           │   │
                    │   │   Address Setup, Data Setup, Hold, Bus Turnaround           │   │
                    │   └───────────────────────────┬─────────────────────────────────┘   │
                    │                               │                                      │
                    └───────────────────────────────┼──────────────────────────────────────┘
                                                    │
                                                    ▼
                              ┌──────────────────────────────────────────────┐
                              │              External Memory Bus              │
                              │                                               │
                              │   Address[25:0]  Data[15:0]  Control Signals │
                              │   NE[3:0]        NWE         NOE             │
                              │   NBL[1:0]       NWAIT       CLK             │
                              │   SDNE[1:0]      SDCKE[1:0]  SDNRAS/NCAS     │
                              └──────────────────────────────────────────────┘
```

---

## Memory Map

```
0xFFFF_FFFF ┌────────────────────────────┐
            │        Reserved             │
0xD000_0000 ├────────────────────────────┤
            │  SDRAM Bank 2 (256 MB)     │  ◀── Bank 6
0xC000_0000 ├────────────────────────────┤
            │  SDRAM Bank 1 (256 MB)     │  ◀── Bank 5
            ├────────────────────────────┤
            │        Reserved             │
0x9000_0000 ├────────────────────────────┤
            │  PC Card (64 MB)           │  ◀── Bank 4
0x8000_0000 ├────────────────────────────┤
            │  NAND Bank 3 (64 MB)       │  ◀── Bank 3
0x7000_0000 ├────────────────────────────┤
            │  NAND Bank 2 (64 MB)       │  ◀── Bank 2
0x6C00_0000 ├────────────────────────────┤
            │  NOR/SRAM Subbank 4        │  ◀── Bank 1, Subbank 4
0x6800_0000 ├────────────────────────────┤
            │  NOR/SRAM Subbank 3        │  ◀── Bank 1, Subbank 3
0x6400_0000 ├────────────────────────────┤
            │  NOR/SRAM Subbank 2        │  ◀── Bank 1, Subbank 2
0x6000_0000 ├────────────────────────────┤
            │  NOR/SRAM Subbank 1        │  ◀── Bank 1, Subbank 1
            └────────────────────────────┘
```

---

## Configuration Types

### NOR/SRAM Subbank Selection

```c
typedef enum
{
  XMC_BANK1_NOR_SRAM1 = 0x00,  /* Subbank 1: 0x6000_0000 */
  XMC_BANK1_NOR_SRAM2 = 0x01,  /* Subbank 2: 0x6400_0000 */
  XMC_BANK1_NOR_SRAM3 = 0x02,  /* Subbank 3: 0x6800_0000 */
  XMC_BANK1_NOR_SRAM4 = 0x03   /* Subbank 4: 0x6C00_0000 */
} xmc_nor_sram_subbank_type;
```

### Memory Device Type

```c
typedef enum
{
  XMC_DEVICE_SRAM  = 0x00,  /* SRAM/ROM */
  XMC_DEVICE_PSRAM = 0x04,  /* PSRAM (Cellular RAM) */
  XMC_DEVICE_NOR   = 0x08   /* NOR Flash */
} xmc_memory_type;
```

### Data Bus Width

```c
typedef enum
{
  XMC_BUSTYPE_8_BITS  = 0x00,  /* 8-bit data bus */
  XMC_BUSTYPE_16_BITS = 0x10   /* 16-bit data bus */
} xmc_data_width_type;
```

### Access Mode

```c
typedef enum
{
  XMC_ACCESS_MODE_A = 0x00000000,  /* SRAM/CRAM */
  XMC_ACCESS_MODE_B = 0x10000000,  /* NOR Flash */
  XMC_ACCESS_MODE_C = 0x20000000,  /* NOR Flash */
  XMC_ACCESS_MODE_D = 0x30000000   /* Asynchronous */
} xmc_access_mode_type;
```

### SDRAM Column Address Bits

```c
typedef enum
{
  XMC_COLUMN_8  = 0x00,  /* 8-bit column (256 columns) */
  XMC_COLUMN_9  = 0x01,  /* 9-bit column (512 columns) */
  XMC_COLUMN_10 = 0x02,  /* 10-bit column (1024 columns) */
  XMC_COLUMN_11 = 0x03   /* 11-bit column (2048 columns) */
} xmc_sdram_column_type;
```

### SDRAM Row Address Bits

```c
typedef enum
{
  XMC_ROW_11 = 0x00,  /* 11-bit row (2048 rows) */
  XMC_ROW_12 = 0x01,  /* 12-bit row (4096 rows) */
  XMC_ROW_13 = 0x02   /* 13-bit row (8192 rows) */
} xmc_sdram_row_type;
```

### SDRAM CAS Latency

```c
typedef enum
{
  XMC_CAS_1 = 0x01,  /* 1 clock cycle */
  XMC_CAS_2 = 0x02,  /* 2 clock cycles */
  XMC_CAS_3 = 0x03   /* 3 clock cycles */
} xmc_sdram_cas_type;
```

### SDRAM Clock Divider

```c
typedef enum
{
  XMC_NO_CLK   = 0x00,  /* Clock disabled */
  XMC_CLKDIV_2 = 0x02,  /* HCLK / 2 */
  XMC_CLKDIV_3 = 0x03,  /* HCLK / 3 */
  XMC_CLKDIV_4 = 0x01   /* HCLK / 4 */
} xmc_sdram_clkdiv_type;
```

### SDRAM Commands

```c
typedef enum
{
  XMC_CMD_NORMAL       = 0x00,  /* Normal operation */
  XMC_CMD_CLK          = 0x01,  /* Clock enable */
  XMC_CMD_PRECHARG_ALL = 0x02,  /* Precharge all banks */
  XMC_CMD_AUTO_REFRESH = 0x03,  /* Auto refresh */
  XMC_CMD_LOAD_MODE    = 0x04,  /* Load mode register */
  XMC_CMD_SELF_REFRESH = 0x05,  /* Self refresh */
  XMC_CMD_POWER_DOWN   = 0x06   /* Power down */
} xmc_command_type;
```

### ECC Page Size

```c
typedef enum
{
  XMC_ECC_PAGESIZE_256_BYTES  = 0x00000000,
  XMC_ECC_PAGESIZE_512_BYTES  = 0x00020000,
  XMC_ECC_PAGESIZE_1024_BYTES = 0x00040000,
  XMC_ECC_PAGESIZE_2048_BYTES = 0x00060000,
  XMC_ECC_PAGESIZE_4096_BYTES = 0x00080000,
  XMC_ECC_PAGESIZE_8192_BYTES = 0x000A0000
} xmc_ecc_pagesize_type;
```

---

## API Reference

### NOR/SRAM Functions

```c
/* Reset NOR/SRAM bank configuration */
void xmc_nor_sram_reset(xmc_nor_sram_subbank_type xmc_subbank);

/* Initialize NOR/SRAM bank */
void xmc_nor_sram_init(xmc_norsram_init_type* xmc_norsram_init_struct);

/* Configure NOR/SRAM timing */
void xmc_nor_sram_timing_config(xmc_norsram_timing_init_type* xmc_rw_timing_struct,
                                xmc_norsram_timing_init_type* xmc_w_timing_struct);

/* Initialize default parameters */
void xmc_norsram_default_para_init(xmc_norsram_init_type* xmc_nor_sram_init_struct);
void xmc_norsram_timing_default_para_init(xmc_norsram_timing_init_type* xmc_rw_timing_struct,
                                          xmc_norsram_timing_init_type* xmc_w_timing_struct);

/* Enable/disable NOR/SRAM bank */
void xmc_nor_sram_enable(xmc_nor_sram_subbank_type xmc_subbank, confirm_state new_state);

/* Configure extended timing */
void xmc_ext_timing_config(xmc_nor_sram_subbank_type xmc_sub_bank, 
                           uint16_t w2w_timing, uint16_t r2r_timing);
```

### NAND Functions

```c
/* Reset NAND bank */
void xmc_nand_reset(xmc_class_bank_type xmc_bank);

/* Initialize NAND */
void xmc_nand_init(xmc_nand_init_type* xmc_nand_init_struct);

/* Configure NAND timing */
void xmc_nand_timing_config(xmc_nand_pccard_timinginit_type* xmc_common_spacetiming_struct,
                            xmc_nand_pccard_timinginit_type* xmc_attribute_spacetiming_struct);

/* Initialize default parameters */
void xmc_nand_default_para_init(xmc_nand_init_type* xmc_nand_init_struct);
void xmc_nand_timing_default_para_init(xmc_nand_pccard_timinginit_type* xmc_common_spacetiming_struct,
                                       xmc_nand_pccard_timinginit_type* xmc_attribute_spacetiming_struct);

/* Enable/disable NAND bank */
void xmc_nand_enable(xmc_class_bank_type xmc_bank, confirm_state new_state);

/* Enable/disable ECC */
void xmc_nand_ecc_enable(xmc_class_bank_type xmc_bank, confirm_state new_state);

/* Get ECC value */
uint32_t xmc_ecc_get(xmc_class_bank_type xmc_bank);
```

### SDRAM Functions

```c
/* Reset SDRAM bank */
void xmc_sdram_reset(xmc_sdram_bank_type xmc_bank);

/* Initialize SDRAM */
void xmc_sdram_init(xmc_sdram_init_type *xmc_sdram_init_struct, 
                    xmc_sdram_timing_type *xmc_sdram_timing_struct);

/* Initialize default parameters */
void xmc_sdram_default_para_init(xmc_sdram_init_type *xmc_sdram_init_struct, 
                                 xmc_sdram_timing_type *xmc_sdram_timing_struct);

/* Send SDRAM command */
void xmc_sdram_cmd(xmc_sdram_cmd_type *xmc_sdram_cmd_struct);

/* Get SDRAM status */
uint32_t xmc_sdram_status_get(xmc_sdram_bank_type xmc_bank);

/* Set refresh counter */
void xmc_sdram_refresh_counter_set(uint32_t counter);

/* Set auto-refresh count */
void xmc_sdram_auto_refresh_set(uint32_t number);
```

### PC Card Functions

```c
/* Reset PC Card */
void xmc_pccard_reset(void);

/* Initialize PC Card */
void xmc_pccard_init(xmc_pccard_init_type* xmc_pccard_init_struct);

/* Configure PC Card timing */
void xmc_pccard_timing_config(xmc_nand_pccard_timinginit_type* xmc_common_spacetiming_struct,
                              xmc_nand_pccard_timinginit_type* xmc_attribute_spacetiming_struct,
                              xmc_nand_pccard_timinginit_type* xmc_iospace_timing_struct);

/* Enable/disable PC Card */
void xmc_pccard_enable(confirm_state new_state);
```

### Interrupt and Flag Functions

```c
/* Enable/disable interrupt */
void xmc_interrupt_enable(xmc_class_bank_type xmc_bank, 
                          xmc_interrupt_sources_type xmc_int, 
                          confirm_state new_state);

/* Get flag status */
flag_status xmc_flag_status_get(xmc_class_bank_type xmc_bank, 
                                xmc_interrupt_flag_type xmc_flag);

/* Get interrupt flag status */
flag_status xmc_interrupt_flag_status_get(xmc_class_bank_type xmc_bank, 
                                          xmc_interrupt_flag_type xmc_flag);

/* Clear flag */
void xmc_flag_clear(xmc_class_bank_type xmc_bank, xmc_interrupt_flag_type xmc_flag);
```

---

## Pin Configuration

### SDRAM Pins

| Signal | Pin | Description |
|--------|-----|-------------|
| XMC_A0-A12 | PF0-PF5, PF12-PF15, PG0-PG2 | Address lines |
| XMC_D0-D15 | PD14, PD15, PD0, PD1, PE7-PE15, PD8-PD10 | Data lines |
| XMC_CLK | PG8 | SDRAM clock |
| XMC_CKE0 | PC3 | Clock enable (Bank 1) |
| XMC_CKE1 | PB5 | Clock enable (Bank 2) |
| XMC_NE0 | PC2 | Chip select (Bank 1) |
| XMC_NE1 | PB6 | Chip select (Bank 2) |
| XMC_BA0-BA1 | PG4, PG5 | Bank address |
| XMC_NRAS | PF11 | Row address strobe |
| XMC_NCAS | PG15 | Column address strobe |
| XMC_SDNWE | PC0 | Write enable |
| XMC_NBL0 | PE0 | Lower byte enable |
| XMC_NBL1 | PE1 | Upper byte enable |

### NOR/SRAM Pins

| Signal | Pin | Description |
|--------|-----|-------------|
| XMC_A0-A25 | Various | Address lines |
| XMC_D0-D15 | Various | Data lines |
| XMC_NE1-NE4 | PD7, PG9, PG10, PG12 | Chip selects |
| XMC_NOE | PD4 | Output enable |
| XMC_NWE | PD5 | Write enable |
| XMC_NWAIT | PD6 | Wait signal |
| XMC_NBL0-NBL1 | PE0, PE1 | Byte lane enables |

### LCD Interface (8-bit)

| Signal | Pin | Description |
|--------|-----|-------------|
| XMC_D0-D7 | PD14, PD15, PD0, PD1, PE7-PE10 | Data bus |
| XMC_NE1 | PD7 | Chip select |
| XMC_NOE | PD4 | Read strobe (RD) |
| XMC_NWE | PD5 | Write strobe (WR) |
| XMC_A0 | PF0 | Register select (RS) |

---

## Code Examples

### Example 1: SRAM Interface

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define SRAM_BANK_ADDR  ((uint32_t)0x60000000)

void sram_gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    /* Enable GPIO clocks */
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOG_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    
    /* Configure data pins D0-D15 */
    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_8 |
                                  GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_14 | GPIO_PINS_15;
    gpio_init(GPIOD, &gpio_init_struct);
    
    gpio_init_struct.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 |
                                  GPIO_PINS_10 | GPIO_PINS_11 | GPIO_PINS_12 |
                                  GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
    gpio_init(GPIOE, &gpio_init_struct);
    
    /* Configure address and control pins */
    gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_7;
    gpio_init(GPIOD, &gpio_init_struct);
    
    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
    gpio_init(GPIOE, &gpio_init_struct);
    
    /* Configure MUX functions */
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE0, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE1, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE4, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE5, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE7, GPIO_MUX_12);
    /* ... continue for all pins ... */
}

void sram_init(void)
{
    xmc_norsram_init_type xmc_norsram_init_struct;
    xmc_norsram_timing_init_type rw_timing_struct;
    xmc_norsram_timing_init_type w_timing_struct;
    
    /* Enable XMC clock */
    crm_periph_clock_enable(CRM_XMC_PERIPH_CLOCK, TRUE);
    
    /* Configure GPIO */
    sram_gpio_config();
    
    /* Initialize default parameters */
    xmc_norsram_default_para_init(&xmc_norsram_init_struct);
    xmc_norsram_timing_default_para_init(&rw_timing_struct, &w_timing_struct);
    
    /* Configure SRAM parameters */
    xmc_norsram_init_struct.subbank = XMC_BANK1_NOR_SRAM1;
    xmc_norsram_init_struct.data_addr_multiplex = XMC_DATA_ADDR_MUX_DISABLE;
    xmc_norsram_init_struct.device = XMC_DEVICE_SRAM;
    xmc_norsram_init_struct.bus_type = XMC_BUSTYPE_16_BITS;
    xmc_norsram_init_struct.burst_mode_enable = XMC_BURST_MODE_DISABLE;
    xmc_norsram_init_struct.asynwait_enable = XMC_ASYN_WAIT_DISABLE;
    xmc_norsram_init_struct.wait_signal_lv = XMC_WAIT_SIGNAL_LEVEL_LOW;
    xmc_norsram_init_struct.wrapped_mode_enable = XMC_WRAPPED_MODE_DISABLE;
    xmc_norsram_init_struct.wait_signal_config = XMC_WAIT_SIGNAL_SYN_BEFORE;
    xmc_norsram_init_struct.write_enable = XMC_WRITE_OPERATION_ENABLE;
    xmc_norsram_init_struct.wait_signal_enable = XMC_WAIT_SIGNAL_DISABLE;
    xmc_norsram_init_struct.write_timing_enable = XMC_WRITE_TIMING_DISABLE;
    xmc_norsram_init_struct.write_burst_syn = XMC_WRITE_BURST_SYN_DISABLE;
    
    /* Configure timing */
    rw_timing_struct.subbank = XMC_BANK1_NOR_SRAM1;
    rw_timing_struct.write_timing_enable = XMC_WRITE_TIMING_DISABLE;
    rw_timing_struct.addr_setup_time = 0x02;
    rw_timing_struct.addr_hold_time = 0x00;
    rw_timing_struct.data_setup_time = 0x08;
    rw_timing_struct.bus_latency_time = 0x00;
    rw_timing_struct.clk_psc = 0x00;
    rw_timing_struct.data_latency_time = 0x00;
    rw_timing_struct.mode = XMC_ACCESS_MODE_A;
    
    xmc_nor_sram_init(&xmc_norsram_init_struct);
    xmc_nor_sram_timing_config(&rw_timing_struct, &w_timing_struct);
    xmc_nor_sram_enable(XMC_BANK1_NOR_SRAM1, TRUE);
}

void sram_write_buffer(uint16_t* buffer, uint32_t addr, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
    {
        *(volatile uint16_t*)(SRAM_BANK_ADDR + addr + i * 2) = buffer[i];
    }
}

void sram_read_buffer(uint16_t* buffer, uint32_t addr, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
    {
        buffer[i] = *(volatile uint16_t*)(SRAM_BANK_ADDR + addr + i * 2);
    }
}

int main(void)
{
    uint16_t tx_buf[256], rx_buf[256];
    
    system_clock_config();
    at32_board_init();
    sram_init();
    
    /* Fill TX buffer */
    for (int i = 0; i < 256; i++)
        tx_buf[i] = i + 0x1234;
    
    /* Write and read back */
    sram_write_buffer(tx_buf, 0x0000, 256);
    sram_read_buffer(rx_buf, 0x0000, 256);
    
    /* Verify */
    uint8_t pass = 1;
    for (int i = 0; i < 256; i++)
    {
        if (tx_buf[i] != rx_buf[i])
        {
            pass = 0;
            break;
        }
    }
    
    if (pass)
        at32_led_on(LED2);
    else
        at32_led_on(LED4);
    
    while (1) {}
}
```

---

### Example 2: SDRAM Interface

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define SDRAM_BANK_ADDR  ((uint32_t)0xC0000000)

/* SDRAM Mode Register definitions */
#define SDRAM_BURST_LEN_1             ((uint16_t)0x0000)
#define SDRAM_BURST_LEN_2             ((uint16_t)0x0001)
#define SDRAM_BURST_LEN_4             ((uint16_t)0x0002)
#define SDRAM_BURST_LEN_8             ((uint16_t)0x0004)
#define SDRAM_BURST_SEQUENTIAL        ((uint16_t)0x0000)
#define SDRAM_CAS_LATENCY_2           ((uint16_t)0x0020)
#define SDRAM_CAS_LATENCY_3           ((uint16_t)0x0030)
#define SDRAM_WR_BURST_SINGLE         ((uint16_t)0x0200)

void sdram_gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOG_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    
    /* Configure all SDRAM pins with MUX_12 */
    /* Address A0-A12, Data D0-D15, Control signals */
    /* ... (detailed pin configuration) ... */
}

void sdram_init(void)
{
    xmc_sdram_init_type xmc_sdram_init_struct;
    xmc_sdram_timing_type xmc_sdram_timing_struct;
    xmc_sdram_cmd_type xmc_sdram_cmd_struct;
    
    crm_periph_clock_enable(CRM_XMC_PERIPH_CLOCK, TRUE);
    sdram_gpio_config();
    
    /* Initialize default parameters */
    xmc_sdram_default_para_init(&xmc_sdram_init_struct, &xmc_sdram_timing_struct);
    
    /* Configure SDRAM parameters (W9825G6KH-6) */
    xmc_sdram_init_struct.sdram_bank = XMC_SDRAM_BANK1;
    xmc_sdram_init_struct.internel_banks = XMC_INBK_4;
    xmc_sdram_init_struct.clkdiv = XMC_CLKDIV_2;  /* HCLK/2 */
    xmc_sdram_init_struct.write_protection = FALSE;
    xmc_sdram_init_struct.burst_read = TRUE;
    xmc_sdram_init_struct.read_delay = XMC_READ_DELAY_1;
    xmc_sdram_init_struct.column_address = XMC_COLUMN_9;  /* 512 columns */
    xmc_sdram_init_struct.row_address = XMC_ROW_13;       /* 8192 rows */
    xmc_sdram_init_struct.cas = XMC_CAS_3;
    xmc_sdram_init_struct.width = XMC_MEM_WIDTH_16;
    
    /* Configure timing (for 144MHz HCLK, SDRAM CLK = 72MHz) */
    xmc_sdram_timing_struct.tmrd = XMC_DELAY_CYCLE_2;  /* Mode to active */
    xmc_sdram_timing_struct.txsr = XMC_DELAY_CYCLE_7;  /* Exit self-refresh */
    xmc_sdram_timing_struct.tras = XMC_DELAY_CYCLE_4;  /* Self-refresh */
    xmc_sdram_timing_struct.trc = XMC_DELAY_CYCLE_7;   /* Refresh to active */
    xmc_sdram_timing_struct.twr = XMC_DELAY_CYCLE_2;   /* Write recovery */
    xmc_sdram_timing_struct.trp = XMC_DELAY_CYCLE_2;   /* Precharge */
    xmc_sdram_timing_struct.trcd = XMC_DELAY_CYCLE_2;  /* Row to column */
    
    xmc_sdram_init(&xmc_sdram_init_struct, &xmc_sdram_timing_struct);
    
    /* SDRAM Initialization Sequence */
    
    /* Step 1: Clock enable */
    xmc_sdram_cmd_struct.cmd = XMC_CMD_CLK;
    xmc_sdram_cmd_struct.cmd_banks = XMC_CMD_BANK1;
    xmc_sdram_cmd_struct.auto_refresh = 1;
    xmc_sdram_cmd_struct.data = 0;
    xmc_sdram_cmd(&xmc_sdram_cmd_struct);
    
    delay_us(200);  /* Wait 200us */
    
    /* Step 2: Precharge all */
    xmc_sdram_cmd_struct.cmd = XMC_CMD_PRECHARG_ALL;
    xmc_sdram_cmd(&xmc_sdram_cmd_struct);
    
    /* Step 3: Auto-refresh (8 cycles) */
    xmc_sdram_cmd_struct.cmd = XMC_CMD_AUTO_REFRESH;
    xmc_sdram_cmd_struct.auto_refresh = 8;
    xmc_sdram_cmd(&xmc_sdram_cmd_struct);
    
    /* Step 4: Load mode register */
    xmc_sdram_cmd_struct.cmd = XMC_CMD_LOAD_MODE;
    xmc_sdram_cmd_struct.auto_refresh = 1;
    xmc_sdram_cmd_struct.data = SDRAM_BURST_LEN_1 | SDRAM_BURST_SEQUENTIAL |
                                 SDRAM_CAS_LATENCY_3 | SDRAM_WR_BURST_SINGLE;
    xmc_sdram_cmd(&xmc_sdram_cmd_struct);
    
    /* Set refresh rate: 64ms / 8192 rows = 7.8us
     * Refresh counter = (7.8us * 72MHz) - 20 = 541
     */
    xmc_sdram_refresh_counter_set(541);
}

void sdram_write_buffer(uint16_t* buffer, uint32_t addr, uint32_t count)
{
    volatile uint16_t* sdram = (volatile uint16_t*)(SDRAM_BANK_ADDR + addr);
    for (uint32_t i = 0; i < count; i++)
    {
        sdram[i] = buffer[i];
    }
}

void sdram_read_buffer(uint16_t* buffer, uint32_t addr, uint32_t count)
{
    volatile uint16_t* sdram = (volatile uint16_t*)(SDRAM_BANK_ADDR + addr);
    for (uint32_t i = 0; i < count; i++)
    {
        buffer[i] = sdram[i];
    }
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    sdram_init();
    
    uint16_t tx_buf[1024], rx_buf[1024];
    
    /* Fill and test */
    for (int i = 0; i < 1024; i++)
        tx_buf[i] = i + 0x3212;
    
    sdram_write_buffer(tx_buf, 0x8000, 1024);
    sdram_read_buffer(rx_buf, 0x8000, 1024);
    
    /* Verify */
    uint8_t pass = 1;
    for (int i = 0; i < 1024; i++)
    {
        if (tx_buf[i] != rx_buf[i])
        {
            pass = 0;
            break;
        }
    }
    
    if (pass)
    {
        at32_led_on(LED2);
        at32_led_on(LED3);
        at32_led_on(LED4);
    }
    
    while (1) {}
}
```

---

### Example 3: NOR Flash Interface

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define NOR_BANK_ADDR  ((uint32_t)0x60000000)

/* NOR Flash commands (CFI interface) */
#define NOR_CMD_READ_ARRAY         0x00F0
#define NOR_CMD_READ_ID            0x0090
#define NOR_CMD_UNLOCK_1           0x00AA
#define NOR_CMD_UNLOCK_2           0x0055
#define NOR_CMD_ERASE_SETUP        0x0080
#define NOR_CMD_ERASE_CONFIRM      0x0030
#define NOR_CMD_PROGRAM            0x00A0
#define NOR_CMD_READ_STATUS        0x0070

typedef struct
{
    uint16_t manufacturer_id;
    uint16_t device_id1;
    uint16_t device_id2;
    uint16_t device_id3;
} nor_id_type;

void nor_gpio_config(void)
{
    /* Configure GPIO for NOR Flash interface */
    /* Similar to SRAM configuration */
}

void nor_init(void)
{
    xmc_norsram_init_type xmc_norsram_init_struct;
    xmc_norsram_timing_init_type rw_timing_struct;
    xmc_norsram_timing_init_type w_timing_struct;
    
    crm_periph_clock_enable(CRM_XMC_PERIPH_CLOCK, TRUE);
    nor_gpio_config();
    
    xmc_norsram_default_para_init(&xmc_norsram_init_struct);
    xmc_norsram_timing_default_para_init(&rw_timing_struct, &w_timing_struct);
    
    /* Configure for NOR Flash */
    xmc_norsram_init_struct.subbank = XMC_BANK1_NOR_SRAM1;
    xmc_norsram_init_struct.data_addr_multiplex = XMC_DATA_ADDR_MUX_DISABLE;
    xmc_norsram_init_struct.device = XMC_DEVICE_NOR;
    xmc_norsram_init_struct.bus_type = XMC_BUSTYPE_16_BITS;
    xmc_norsram_init_struct.burst_mode_enable = XMC_BURST_MODE_DISABLE;
    xmc_norsram_init_struct.asynwait_enable = XMC_ASYN_WAIT_DISABLE;
    xmc_norsram_init_struct.wait_signal_lv = XMC_WAIT_SIGNAL_LEVEL_LOW;
    xmc_norsram_init_struct.wrapped_mode_enable = XMC_WRAPPED_MODE_DISABLE;
    xmc_norsram_init_struct.wait_signal_config = XMC_WAIT_SIGNAL_SYN_BEFORE;
    xmc_norsram_init_struct.write_enable = XMC_WRITE_OPERATION_ENABLE;
    xmc_norsram_init_struct.wait_signal_enable = XMC_WAIT_SIGNAL_DISABLE;
    xmc_norsram_init_struct.write_timing_enable = XMC_WRITE_TIMING_ENABLE;
    xmc_norsram_init_struct.write_burst_syn = XMC_WRITE_BURST_SYN_DISABLE;
    
    /* Read timing */
    rw_timing_struct.subbank = XMC_BANK1_NOR_SRAM1;
    rw_timing_struct.write_timing_enable = XMC_WRITE_TIMING_ENABLE;
    rw_timing_struct.addr_setup_time = 0x06;
    rw_timing_struct.addr_hold_time = 0x01;
    rw_timing_struct.data_setup_time = 0x0C;
    rw_timing_struct.bus_latency_time = 0x00;
    rw_timing_struct.clk_psc = 0x00;
    rw_timing_struct.data_latency_time = 0x00;
    rw_timing_struct.mode = XMC_ACCESS_MODE_B;
    
    /* Write timing */
    w_timing_struct.subbank = XMC_BANK1_NOR_SRAM1;
    w_timing_struct.write_timing_enable = XMC_WRITE_TIMING_ENABLE;
    w_timing_struct.addr_setup_time = 0x06;
    w_timing_struct.addr_hold_time = 0x01;
    w_timing_struct.data_setup_time = 0x0C;
    w_timing_struct.bus_latency_time = 0x00;
    w_timing_struct.clk_psc = 0x00;
    w_timing_struct.data_latency_time = 0x00;
    w_timing_struct.mode = XMC_ACCESS_MODE_B;
    
    xmc_nor_sram_init(&xmc_norsram_init_struct);
    xmc_nor_sram_timing_config(&rw_timing_struct, &w_timing_struct);
    xmc_nor_sram_enable(XMC_BANK1_NOR_SRAM1, TRUE);
}

#define NOR_WRITE(addr, data)  *(volatile uint16_t*)(NOR_BANK_ADDR + addr) = data
#define NOR_READ(addr)         *(volatile uint16_t*)(NOR_BANK_ADDR + addr)

void nor_read_id(nor_id_type* id)
{
    NOR_WRITE(0x0AAA, NOR_CMD_UNLOCK_1);
    NOR_WRITE(0x0554, NOR_CMD_UNLOCK_2);
    NOR_WRITE(0x0AAA, NOR_CMD_READ_ID);
    
    id->manufacturer_id = NOR_READ(0x0000);
    id->device_id1 = NOR_READ(0x0002);
    id->device_id2 = NOR_READ(0x001C);
    id->device_id3 = NOR_READ(0x001E);
    
    NOR_WRITE(0x0000, NOR_CMD_READ_ARRAY);  /* Return to read mode */
}

void nor_erase_block(uint32_t block_addr)
{
    NOR_WRITE(0x0AAA, NOR_CMD_UNLOCK_1);
    NOR_WRITE(0x0554, NOR_CMD_UNLOCK_2);
    NOR_WRITE(0x0AAA, NOR_CMD_ERASE_SETUP);
    NOR_WRITE(0x0AAA, NOR_CMD_UNLOCK_1);
    NOR_WRITE(0x0554, NOR_CMD_UNLOCK_2);
    NOR_WRITE(block_addr, NOR_CMD_ERASE_CONFIRM);
    
    /* Wait for erase to complete (poll DQ7) */
    while ((NOR_READ(block_addr) & 0x80) != 0x80);
}

void nor_program_word(uint32_t addr, uint16_t data)
{
    NOR_WRITE(0x0AAA, NOR_CMD_UNLOCK_1);
    NOR_WRITE(0x0554, NOR_CMD_UNLOCK_2);
    NOR_WRITE(0x0AAA, NOR_CMD_PROGRAM);
    NOR_WRITE(addr, data);
    
    /* Wait for program to complete */
    while ((NOR_READ(addr) & 0x80) != (data & 0x80));
}

int main(void)
{
    nor_id_type nor_id;
    
    system_clock_config();
    at32_board_init();
    nor_init();
    
    /* Read NOR ID */
    nor_read_id(&nor_id);
    
    /* Erase a block */
    nor_erase_block(0x8000);
    
    /* Program data */
    for (int i = 0; i < 256; i++)
    {
        nor_program_word(0x8000 + i * 2, 0x1234 + i);
    }
    
    /* Verify */
    uint8_t pass = 1;
    for (int i = 0; i < 256; i++)
    {
        if (NOR_READ(0x8000 + i * 2) != (0x1234 + i))
        {
            pass = 0;
            break;
        }
    }
    
    if (pass)
        at32_led_on(LED2);
    
    while (1) {}
}
```

---

### Example 4: LCD Interface (8-bit 8080)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define LCD_BANK_ADDR  ((uint32_t)0x60000000)
#define LCD_CMD        (*(volatile uint16_t*)LCD_BANK_ADDR)
#define LCD_DATA       (*(volatile uint16_t*)(LCD_BANK_ADDR + 0x02))

void lcd_gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    
    /* D0-D7 (8-bit mode) */
    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_14 | GPIO_PINS_15;
    gpio_init(GPIOD, &gpio_init_struct);
    
    gpio_init_struct.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
    gpio_init(GPIOE, &gpio_init_struct);
    
    /* Control: NOE, NWE, NE1, A0 */
    gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_7;
    gpio_init(GPIOD, &gpio_init_struct);
    
    gpio_init_struct.gpio_pins = GPIO_PINS_0;  /* A0 = RS */
    gpio_init(GPIOF, &gpio_init_struct);
    
    /* Configure MUX */
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE0, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE1, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE4, GPIO_MUX_12);  /* NOE */
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE5, GPIO_MUX_12);  /* NWE */
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE7, GPIO_MUX_12);  /* NE1 */
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE14, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE15, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE7, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE8, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE9, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE10, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOF, GPIO_PINS_SOURCE0, GPIO_MUX_12);  /* A0 */
}

void lcd_xmc_init(void)
{
    xmc_norsram_init_type xmc_norsram_init_struct;
    xmc_norsram_timing_init_type rw_timing_struct;
    xmc_norsram_timing_init_type w_timing_struct;
    
    crm_periph_clock_enable(CRM_XMC_PERIPH_CLOCK, TRUE);
    lcd_gpio_config();
    
    xmc_norsram_default_para_init(&xmc_norsram_init_struct);
    xmc_norsram_timing_default_para_init(&rw_timing_struct, &w_timing_struct);
    
    xmc_norsram_init_struct.subbank = XMC_BANK1_NOR_SRAM1;
    xmc_norsram_init_struct.data_addr_multiplex = XMC_DATA_ADDR_MUX_DISABLE;
    xmc_norsram_init_struct.device = XMC_DEVICE_SRAM;
    xmc_norsram_init_struct.bus_type = XMC_BUSTYPE_8_BITS;  /* 8-bit mode */
    xmc_norsram_init_struct.burst_mode_enable = XMC_BURST_MODE_DISABLE;
    xmc_norsram_init_struct.asynwait_enable = XMC_ASYN_WAIT_DISABLE;
    xmc_norsram_init_struct.wait_signal_lv = XMC_WAIT_SIGNAL_LEVEL_LOW;
    xmc_norsram_init_struct.wrapped_mode_enable = XMC_WRAPPED_MODE_DISABLE;
    xmc_norsram_init_struct.wait_signal_config = XMC_WAIT_SIGNAL_SYN_BEFORE;
    xmc_norsram_init_struct.write_enable = XMC_WRITE_OPERATION_ENABLE;
    xmc_norsram_init_struct.wait_signal_enable = XMC_WAIT_SIGNAL_DISABLE;
    xmc_norsram_init_struct.write_timing_enable = XMC_WRITE_TIMING_DISABLE;
    xmc_norsram_init_struct.write_burst_syn = XMC_WRITE_BURST_SYN_DISABLE;
    
    rw_timing_struct.subbank = XMC_BANK1_NOR_SRAM1;
    rw_timing_struct.write_timing_enable = XMC_WRITE_TIMING_DISABLE;
    rw_timing_struct.addr_setup_time = 0x01;
    rw_timing_struct.addr_hold_time = 0x00;
    rw_timing_struct.data_setup_time = 0x05;
    rw_timing_struct.bus_latency_time = 0x00;
    rw_timing_struct.clk_psc = 0x00;
    rw_timing_struct.data_latency_time = 0x00;
    rw_timing_struct.mode = XMC_ACCESS_MODE_A;
    
    xmc_nor_sram_init(&xmc_norsram_init_struct);
    xmc_nor_sram_timing_config(&rw_timing_struct, &w_timing_struct);
    xmc_nor_sram_enable(XMC_BANK1_NOR_SRAM1, TRUE);
}

void lcd_write_cmd(uint8_t cmd)
{
    LCD_CMD = cmd;
}

void lcd_write_data(uint8_t data)
{
    LCD_DATA = data;
}

void lcd_init(void)
{
    lcd_xmc_init();
    
    /* LCD reset sequence */
    /* ... LCD-specific initialization ... */
    
    /* Example for ILI9341 */
    lcd_write_cmd(0x01);  /* Software reset */
    delay_ms(50);
    
    lcd_write_cmd(0x11);  /* Sleep out */
    delay_ms(120);
    
    lcd_write_cmd(0x29);  /* Display ON */
}

void lcd_set_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    /* Set column address */
    lcd_write_cmd(0x2A);
    lcd_write_data(x >> 8);
    lcd_write_data(x & 0xFF);
    lcd_write_data(x >> 8);
    lcd_write_data(x & 0xFF);
    
    /* Set row address */
    lcd_write_cmd(0x2B);
    lcd_write_data(y >> 8);
    lcd_write_data(y & 0xFF);
    lcd_write_data(y >> 8);
    lcd_write_data(y & 0xFF);
    
    /* Write pixel */
    lcd_write_cmd(0x2C);
    lcd_write_data(color >> 8);
    lcd_write_data(color & 0xFF);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    lcd_init();
    
    /* Fill screen with color */
    for (int y = 0; y < 240; y++)
    {
        for (int x = 0; x < 320; x++)
        {
            lcd_set_pixel(x, y, 0xF800);  /* Red */
        }
    }
    
    while (1) {}
}
```

---

## Configuration Checklist

### General Setup
- [ ] Enable XMC clock: `crm_periph_clock_enable(CRM_XMC_PERIPH_CLOCK, TRUE)`
- [ ] Configure all required GPIO pins as alternate function (MUX_12)
- [ ] Set appropriate drive strength for high-speed signals

### NOR/SRAM/PSRAM
- [ ] Select appropriate subbank
- [ ] Configure bus width (8 or 16 bit)
- [ ] Set device type (SRAM, PSRAM, or NOR)
- [ ] Configure timing parameters based on memory datasheet
- [ ] Enable write operations if needed

### SDRAM
- [ ] Configure column/row address bits
- [ ] Set CAS latency
- [ ] Configure clock divider
- [ ] Set timing parameters (tMRD, tXSR, tRAS, tRC, tWR, tRP, tRCD)
- [ ] Execute initialization sequence (CLK, PRECHARG_ALL, AUTO_REFRESH, LOAD_MODE)
- [ ] Set refresh counter

### NAND
- [ ] Configure timing for command and data phases
- [ ] Enable ECC if needed
- [ ] Configure ECC page size

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| No response from memory | GPIO not configured | Check MUX settings (MUX_12) |
| Data corruption | Timing too fast | Increase setup/hold times |
| SDRAM not initializing | Wrong init sequence | Follow exact command sequence |
| Refresh errors | Counter too high | Calculate correct refresh rate |
| NOR program fails | Block not erased | Erase before programming |
| LCD garbled display | Wrong timing | Adjust XMC timing for display |
| Memory access fault | Wrong address | Verify bank base address |

---

## Performance Tips

- Use burst mode for SDRAM/NOR to improve throughput
- Enable read delay for high-speed SDRAM
- Use DMA for large data transfers
- Configure appropriate clock divider for memory speed
- Place frequently accessed code/data in fast memory regions

---

## Related Peripherals

| Peripheral | Relationship |
|------------|--------------|
| GPIO | Pin configuration for address/data/control |
| CRM | XMC clock enable |
| DMA | High-speed data transfers |
| DVP | Frame buffer in SDRAM |

---

## References

- AT32F435/437 Reference Manual - XMC Chapter
- Application Note AN0089 - SDRAM Interface
- Application Note AN0082 - External Memory Examples
- Memory device datasheets (W9825G6KH, IS62WV51216, etc.)

