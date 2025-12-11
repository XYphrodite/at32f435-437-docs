---
title: QSPI - Quad SPI Interface
category: Peripheral
complexity: Advanced
mcu: AT32F435/437
peripheral: QSPI
keywords: [qspi, spi, flash, xip, quad, dual, dma, memory-mapped, nor]
---

# QSPI - Quad SPI Interface

## Overview

The Quad SPI (QSPI) peripheral provides a high-speed serial interface for communicating with external SPI NOR flash memory devices. It supports Single, Dual, and Quad I/O modes for efficient data transfer, and offers two operating modes: Command-Port mode for explicit flash operations and XIP (eXecute-In-Place) mode for memory-mapped access allowing code execution directly from external flash.

### Key Features

| Feature | Specification |
|---------|---------------|
| QSPI Interfaces | 2 (QSPI1, QSPI2) |
| I/O Modes | Single (1-bit), Dual (2-bit), Quad (4-bit) |
| Operating Modes | Command-Port, XIP (memory-mapped) |
| Clock Division | 2, 3, 4, 5, 6, 8, 10, 12 |
| SCK Modes | Mode 0 (CPOL=0, CPHA=0), Mode 3 (CPOL=1, CPHA=1) |
| FIFO Depth | 32 words (128 bytes) |
| DMA Support | Yes (TX and RX) |
| Address Length | 0-4 bytes |
| XIP Cache | Built-in (bypassable) |
| Max Speed | Up to 144 MHz SCK |

### Operation Mode Notation

| Mode | Instruction | Address | Data | Description |
|------|-------------|---------|------|-------------|
| 1-1-1 | 1-bit | 1-bit | 1-bit | Standard SPI |
| 1-1-2 | 1-bit | 1-bit | 2-bit | Dual output |
| 1-1-4 | 1-bit | 1-bit | 4-bit | Quad output |
| 1-2-2 | 1-bit | 2-bit | 2-bit | Dual I/O |
| 1-4-4 | 1-bit | 4-bit | 4-bit | Quad I/O |
| 2-2-2 | 2-bit | 2-bit | 2-bit | Full dual |
| 4-4-4 | 4-bit | 4-bit | 4-bit | QPI mode |

## Architecture

```
                        ┌────────────────────────────────────────────────────────┐
                        │                     QSPI Controller                     │
                        │                                                        │
┌─────────────┐         │  ┌─────────────────────────────────────────────────┐  │
│ AHB Bus     │─────────│──│           Mode Controller                       │  │
│             │         │  │  ┌────────────────┐  ┌──────────────────────┐  │  │
└─────────────┘         │  │  │ Command-Port   │  │      XIP Mode        │  │  │
                        │  │  │ (explicit ops) │  │ (memory-mapped)      │  │  │
                        │  │  └───────┬────────┘  └──────────┬───────────┘  │  │
                        │  │          │                      │              │  │
                        │  └──────────┴──────────────────────┴──────────────┘  │
                        │                        │                              │
                        │  ┌─────────────────────▼─────────────────────────┐   │
                        │  │              Command Engine                    │   │
                        │  │  ┌────────────┐  ┌──────────┐  ┌───────────┐  │   │
                        │  │  │ Instruction │  │ Address  │  │   Data    │  │   │
                        │  │  │ CMD_W3.INSC │  │ CMD_W0   │  │  Counter  │  │   │
                        │  │  └────────────┘  └──────────┘  │  CMD_W2   │  │   │
                        │  │                                └───────────┘  │   │
                        │  └────────────────────┬──────────────────────────┘   │
                        │                       │                              │
                        │  ┌────────────────────▼──────────────────────────┐   │
                        │  │               FIFO Controller                  │   │
                        │  │  ┌──────────────────────────────────────────┐ │   │
                        │  │  │            TX/RX FIFO (32 words)         │ │   │
                        │  │  └──────────────────────────────────────────┘ │   │
                        │  │  ┌─────────────┐         ┌─────────────────┐  │   │
                        │  │  │ TXFIFORDY   │         │    RXFIFORDY    │  │   │
                        │  │  └─────────────┘         └─────────────────┘  │   │
                        │  └────────────────────┬──────────────────────────┘   │
                        │                       │                              │
                        │  ┌────────────────────▼──────────────────────────┐   │
                        │  │             Serial Engine                      │   │
                        │  │  ┌───────────────┐  ┌─────────────────────┐   │   │
                        │  │  │ Clock Gen     │  │ Mode Selector       │   │   │
                        │  │  │ CTRL.CLKDIV   │  │ 1-1-1 to 4-4-4      │   │   │
                        │  │  └───────────────┘  └─────────────────────┘   │   │
                        │  └────────────────────────────────────────────────┘   │
                        │                                                        │
                        │     ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐  │
                        │     │ CS  │ │ SCK │ │ IO0 │ │ IO1 │ │ IO2 │ │ IO3 │  │
                        └─────┴──┬──┴─┴──┬──┴─┴──┬──┴─┴──┬──┴─┴──┬──┴─┴──┬──┴──┘
                                 │       │       │       │       │       │
                                 ▼       ▼       ▼       ▼       ▼       ▼
                        ┌────────────────────────────────────────────────────────┐
                        │              External SPI NOR Flash                    │
                        └────────────────────────────────────────────────────────┘
```

## Memory Map

| Unit | Base Address | Size | Description |
|------|--------------|------|-------------|
| QSPI1 Registers | 0xA0001000 | - | QSPI1 control registers |
| QSPI2 Registers | 0xA0002000 | - | QSPI2 control registers |
| QSPI1 Memory | 0x90000000 | Up to 256MB | QSPI1 XIP address space |
| QSPI2 Memory | 0xB0000000 | Up to 256MB | QSPI2 XIP address space |

**Note:** In XIP mode, the external flash can be accessed directly via the memory address like internal memory.

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| CMD_W0 | 0x00 | Command word 0 (address) |
| CMD_W1 | 0x04 | Command word 1 (address length, dummy cycles, instruction length) |
| CMD_W2 | 0x08 | Command word 2 (data counter) |
| CMD_W3 | 0x0C | Command word 3 (mode, instruction code, flags) |
| CTRL | 0x10 | Control register |
| FIFOSTS | 0x18 | FIFO status register |
| CTRL2 | 0x20 | Control register 2 (DMA, interrupt) |
| CMDSTS | 0x24 | Command status register |
| RSTS | 0x28 | Read status register |
| FSIZE | 0x2C | Flash size register |
| XIP_CMD_W0 | 0x30 | XIP read command |
| XIP_CMD_W1 | 0x34 | XIP write command |
| XIP_CMD_W2 | 0x38 | XIP mode selection |
| XIP_CMD_W3 | 0x3C | XIP cache control |
| CTRL3 | 0x40 | Control register 3 (sampling phase) |
| DT | 0x100 | Data register (TX/RX FIFO access) |

### CTRL Register (0x10)

| Bits | Name | Description |
|------|------|-------------|
| 2:0 | CLKDIV | Clock division |
| 4 | SCKMODE | SCK mode: 0=Mode 0, 1=Mode 3 |
| 7 | XIPIDLE | XIP idle status |
| 8 | ABORT | Abort current operation |
| 18:16 | BUSY | Flash WIP bit position in status register |
| 19 | XIPRCMDF | XIP read command flush |
| 20 | XIPSEL | XIP mode enable |
| 21 | KEYEN | Encryption enable |

### CTRL2 Register (0x20)

| Bits | Name | Description |
|------|------|-------------|
| 0 | DMAEN | DMA enable |
| 1 | CMDIE | Command complete interrupt enable |
| 9:8 | TXFIFO_THOD | TX FIFO DMA threshold |
| 13:12 | RXFIFO_THOD | RX FIFO DMA threshold |

## Configuration Types

### Clock Division

| Enum | Value | Division |
|------|-------|----------|
| `QSPI_CLK_DIV_2` | 0x00 | AHB clock / 2 |
| `QSPI_CLK_DIV_3` | 0x04 | AHB clock / 3 |
| `QSPI_CLK_DIV_4` | 0x01 | AHB clock / 4 |
| `QSPI_CLK_DIV_5` | 0x05 | AHB clock / 5 |
| `QSPI_CLK_DIV_6` | 0x02 | AHB clock / 6 |
| `QSPI_CLK_DIV_8` | 0x03 | AHB clock / 8 |
| `QSPI_CLK_DIV_10` | 0x06 | AHB clock / 10 |
| `QSPI_CLK_DIV_12` | 0x07 | AHB clock / 12 |

### Operation Mode

| Enum | Value | Mode | Description |
|------|-------|------|-------------|
| `QSPI_OPERATE_MODE_111` | 0x00 | 1-1-1 | Standard SPI |
| `QSPI_OPERATE_MODE_112` | 0x01 | 1-1-2 | Dual output |
| `QSPI_OPERATE_MODE_114` | 0x02 | 1-1-4 | Quad output |
| `QSPI_OPERATE_MODE_122` | 0x03 | 1-2-2 | Dual I/O |
| `QSPI_OPERATE_MODE_144` | 0x04 | 1-4-4 | Quad I/O |
| `QSPI_OPERATE_MODE_222` | 0x05 | 2-2-2 | Full dual |
| `QSPI_OPERATE_MODE_444` | 0x06 | 4-4-4 | QPI mode |

### Address Length

| Enum | Value | Length |
|------|-------|--------|
| `QSPI_CMD_ADRLEN_0_BYTE` | 0x00 | No address |
| `QSPI_CMD_ADRLEN_1_BYTE` | 0x01 | 1 byte |
| `QSPI_CMD_ADRLEN_2_BYTE` | 0x02 | 2 bytes |
| `QSPI_CMD_ADRLEN_3_BYTE` | 0x03 | 3 bytes (24-bit) |
| `QSPI_CMD_ADRLEN_4_BYTE` | 0x04 | 4 bytes (32-bit) |

### Instruction Length

| Enum | Value | Length |
|------|-------|--------|
| `QSPI_CMD_INSLEN_0_BYTE` | 0x00 | No instruction |
| `QSPI_CMD_INSLEN_1_BYTE` | 0x01 | 1 byte |
| `QSPI_CMD_INSLEN_2_BYTE` | 0x02 | 2 bytes (repeated) |

### SCK Mode

| Enum | Value | Description |
|------|-------|-------------|
| `QSPI_SCK_MODE_0` | 0x00 | CPOL=0, CPHA=0 (idle low, sample on rising) |
| `QSPI_SCK_MODE_3` | 0x01 | CPOL=1, CPHA=1 (idle high, sample on falling) |

### DMA FIFO Threshold

| Enum | Value | Threshold |
|------|-------|-----------|
| `QSPI_DMA_FIFO_THOD_WORD08` | 0x00 | 8 words |
| `QSPI_DMA_FIFO_THOD_WORD16` | 0x01 | 16 words |
| `QSPI_DMA_FIFO_THOD_WORD24` | 0x02 | 24 words |

### Read Status Configuration

| Enum | Value | Description |
|------|-------|-------------|
| `QSPI_RSTSC_HW_AUTO` | 0x00 | Hardware automatically reads status |
| `QSPI_RSTSC_SW_ONCE` | 0x01 | Software reads status once |

## Command Structure

The `qspi_cmd_type` structure defines a QSPI command:

```c
typedef struct
{
    confirm_state         pe_mode_enable;          /* Performance enhance mode */
    uint8_t               pe_mode_operate_code;    /* PE mode code */
    uint8_t               instruction_code;        /* Flash command (0x03, 0xEB, etc.) */
    qspi_cmd_inslen_type  instruction_length;      /* 0, 1, or 2 bytes */
    uint32_t              address_code;            /* Target address */
    qspi_cmd_adrlen_type  address_length;          /* 0-4 bytes */
    uint32_t              data_counter;            /* Data bytes to transfer */
    uint8_t               second_dummy_cycle_num;  /* Dummy cycles (0-32) */
    qspi_operate_mode_type operation_mode;         /* 1-1-1 to 4-4-4 */
    qspi_read_status_conf_type read_status_config; /* Auto or manual status */
    confirm_state         read_status_enable;      /* Enable status read */
    confirm_state         write_data_enable;       /* Write operation */
} qspi_cmd_type;
```

## XIP Structure

The `qspi_xip_type` structure defines XIP mode configuration:

```c
typedef struct
{
    uint8_t               read_instruction_code;        /* Read command */
    qspi_xip_addrlen_type read_address_length;          /* 3 or 4 bytes */
    qspi_operate_mode_type read_operation_mode;         /* Read I/O mode */
    uint8_t               read_second_dummy_cycle_num;  /* Read dummy cycles */
    uint8_t               write_instruction_code;       /* Write command */
    qspi_xip_addrlen_type write_address_length;         /* 3 or 4 bytes */
    qspi_operate_mode_type write_operation_mode;        /* Write I/O mode */
    uint8_t               write_second_dummy_cycle_num; /* Write dummy cycles */
    qspi_xip_write_sel_type write_select_mode;          /* Write mode D/T */
    uint8_t               write_time_counter;           /* Mode T counter */
    uint8_t               write_data_counter;           /* Mode D counter */
    qspi_xip_read_sel_type read_select_mode;            /* Read mode D/T */
    uint8_t               read_time_counter;            /* Mode T counter */
    uint8_t               read_data_counter;            /* Mode D counter */
} qspi_xip_type;
```

## Flags

| Flag Macro | Description |
|------------|-------------|
| `QSPI_CMDSTS_FLAG` | Command complete status |
| `QSPI_RXFIFORDY_FLAG` | RX FIFO has data ready |
| `QSPI_TXFIFORDY_FLAG` | TX FIFO can accept data |

## Pin Configuration

### QSPI1 Pins

| Signal | Pin (Option 1) | MUX | Pin (Option 2) | MUX |
|--------|----------------|-----|----------------|-----|
| CS | PG6 | AF10 | PB10 | AF9 |
| SCK | PF10 | AF9 | PB2 | AF9 |
| IO0 | PF8 | AF10 | PD11 | AF9 |
| IO1 | PF9 | AF10 | PD12 | AF9 |
| IO2 | PF7 | AF9 | PE2 | AF9 |
| IO3 | PF6 | AF9 | PD13 | AF9 |

### QSPI2 Pins

| Signal | Pin | MUX |
|--------|-----|-----|
| CS | PG12 | AF5 |
| SCK | PG13 | AF5 |
| IO0 | PG14 | AF5 |
| IO1 | PG15 | AF5 |
| IO2 | PG9 | AF5 |
| IO3 | PG10 | AF5 |

## Common Flash Commands

| Command | Code | Mode | Description |
|---------|------|------|-------------|
| Read ID | 0x9F | 1-1-1 | Read JEDEC ID |
| Read | 0x03 | 1-1-1 | Standard read |
| Fast Read | 0x0B | 1-1-1 | Fast read with dummy |
| Dual Output Read | 0x3B | 1-1-2 | Dual output fast read |
| Quad Output Read | 0x6B | 1-1-4 | Quad output fast read |
| Dual I/O Read | 0xBB | 1-2-2 | Dual I/O fast read |
| Quad I/O Read | 0xEB | 1-4-4 | Quad I/O fast read |
| Page Program | 0x02 | 1-1-1 | Standard program |
| Quad Page Program | 0x32 | 1-1-4 | Quad page program |
| Sector Erase | 0x20 | 1-1-1 | 4KB sector erase |
| Block Erase | 0xD8 | 1-1-1 | 64KB block erase |
| Write Enable | 0x06 | 1-1-1 | Enable write operations |
| Read Status | 0x05 | 1-1-1 | Read status register |
| Reset Enable | 0x66 | 1-1-1 | Enable software reset |
| Reset | 0x99 | 1-1-1 | Execute software reset |

## API Reference

### Initialization

#### `qspi_reset`

```c
void qspi_reset(qspi_type* qspi_x);
```

Reset QSPI peripheral to default values.

---

#### `qspi_clk_division_set`

```c
void qspi_clk_division_set(qspi_type* qspi_x, qspi_clk_div_type new_clkdiv);
```

Set the QSPI clock division.

| Parameter | Description |
|-----------|-------------|
| `qspi_x` | QSPI1 or QSPI2 |
| `new_clkdiv` | Clock divider: `QSPI_CLK_DIV_2` to `QSPI_CLK_DIV_12` |

---

#### `qspi_sck_mode_set`

```c
void qspi_sck_mode_set(qspi_type* qspi_x, qspi_clk_mode_type new_mode);
```

Set the SCK clock mode.

| Parameter | Description |
|-----------|-------------|
| `qspi_x` | QSPI1 or QSPI2 |
| `new_mode` | `QSPI_SCK_MODE_0` or `QSPI_SCK_MODE_3` |

---

#### `qspi_busy_config`

```c
void qspi_busy_config(qspi_type* qspi_x, qspi_busy_pos_type busy_pos);
```

Configure the position of WIP (Write In Progress) bit in flash status register.

| Parameter | Description |
|-----------|-------------|
| `qspi_x` | QSPI1 or QSPI2 |
| `busy_pos` | `QSPI_BUSY_OFFSET_0` to `QSPI_BUSY_OFFSET_7` |

---

#### `qspi_auto_ispc_enable`

```c
void qspi_auto_ispc_enable(qspi_type* qspi_x);
```

Enable automatic input sampling phase correction for high-speed operation.

---

### Command Operations

#### `qspi_cmd_operation_kick`

```c
void qspi_cmd_operation_kick(qspi_type* qspi_x, qspi_cmd_type* qspi_cmd_struct);
```

Start a QSPI command operation.

| Parameter | Description |
|-----------|-------------|
| `qspi_x` | QSPI1 or QSPI2 |
| `qspi_cmd_struct` | Pointer to command configuration structure |

---

### Data Transfer

#### `qspi_byte_read`

```c
uint8_t qspi_byte_read(qspi_type* qspi_x);
```

Read one byte from the data register.

---

#### `qspi_half_word_read`

```c
uint16_t qspi_half_word_read(qspi_type* qspi_x);
```

Read one half-word (16 bits) from the data register.

---

#### `qspi_word_read`

```c
uint32_t qspi_word_read(qspi_type* qspi_x);
```

Read one word (32 bits) from the data register.

---

#### `qspi_byte_write`

```c
void qspi_byte_write(qspi_type* qspi_x, uint8_t value);
```

Write one byte to the data register.

---

#### `qspi_half_word_write`

```c
void qspi_half_word_write(qspi_type* qspi_x, uint16_t value);
```

Write one half-word (16 bits) to the data register.

---

#### `qspi_word_write`

```c
void qspi_word_write(qspi_type* qspi_x, uint32_t value);
```

Write one word (32 bits) to the data register.

---

### XIP Mode

#### `qspi_xip_init`

```c
void qspi_xip_init(qspi_type* qspi_x, qspi_xip_type* xip_init_struct);
```

Initialize XIP mode parameters.

| Parameter | Description |
|-----------|-------------|
| `qspi_x` | QSPI1 or QSPI2 |
| `xip_init_struct` | Pointer to XIP configuration structure |

---

#### `qspi_xip_enable`

```c
void qspi_xip_enable(qspi_type* qspi_x, confirm_state new_state);
```

Enable or disable XIP (memory-mapped) mode.

| Parameter | Description |
|-----------|-------------|
| `qspi_x` | QSPI1 or QSPI2 |
| `new_state` | TRUE for XIP mode, FALSE for Command-Port mode |

**Note:** In XIP mode, the flash is accessible via `QSPI1_MEM_BASE` (0x90000000) or `QSPI2_MEM_BASE` (0xB0000000).

---

#### `qspi_xip_cache_bypass_set`

```c
void qspi_xip_cache_bypass_set(qspi_type* qspi_x, confirm_state new_state);
```

Enable or disable the XIP cache bypass.

| Parameter | Description |
|-----------|-------------|
| `qspi_x` | QSPI1 or QSPI2 |
| `new_state` | TRUE to bypass cache, FALSE to enable cache |

---

### DMA Support

#### `qspi_dma_enable`

```c
void qspi_dma_enable(qspi_type* qspi_x, confirm_state new_state);
```

Enable or disable DMA transfers.

---

#### `qspi_dma_rx_threshold_set`

```c
void qspi_dma_rx_threshold_set(qspi_type* qspi_x, qspi_dma_fifo_thod_type new_threshold);
```

Set the RX FIFO DMA threshold.

---

#### `qspi_dma_tx_threshold_set`

```c
void qspi_dma_tx_threshold_set(qspi_type* qspi_x, qspi_dma_fifo_thod_type new_threshold);
```

Set the TX FIFO DMA threshold.

---

### Flags and Interrupts

#### `qspi_flag_get`

```c
flag_status qspi_flag_get(qspi_type* qspi_x, uint32_t flag);
```

Get the status of a QSPI flag.

---

#### `qspi_flag_clear`

```c
void qspi_flag_clear(qspi_type* qspi_x, uint32_t flag);
```

Clear a QSPI flag.

---

#### `qspi_interrupt_enable`

```c
void qspi_interrupt_enable(qspi_type* qspi_x, confirm_state new_state);
```

Enable or disable the command complete interrupt.

---

#### `qspi_interrupt_flag_get`

```c
flag_status qspi_interrupt_flag_get(qspi_type* qspi_x, uint32_t flag);
```

Get the interrupt flag status (checks if interrupt is both pending and enabled).

---

### Security

#### `qspi_encryption_enable`

```c
void qspi_encryption_enable(qspi_type* qspi_x, confirm_state new_state);
```

Enable or disable data encryption.

---

## Code Examples

### Example 1: Basic Command-Port Read/Write with Polling

Basic flash operations using polling mode.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include <string.h>

#define TEST_SIZE                4096
#define FLASH_PAGE_SIZE          256
#define QSPI_FIFO_DEPTH         (32 * 4)

uint8_t wbuf[TEST_SIZE];
uint8_t rbuf[TEST_SIZE];

/* Command configurations for EN25QH128A flash */
static const qspi_cmd_type cmd_read = {
    FALSE, 0, 0xEB, QSPI_CMD_INSLEN_1_BYTE, 0, QSPI_CMD_ADRLEN_3_BYTE,
    0, 6, QSPI_OPERATE_MODE_144, QSPI_RSTSC_HW_AUTO, FALSE, FALSE
};

static const qspi_cmd_type cmd_write = {
    FALSE, 0, 0x32, QSPI_CMD_INSLEN_1_BYTE, 0, QSPI_CMD_ADRLEN_3_BYTE,
    0, 0, QSPI_OPERATE_MODE_114, QSPI_RSTSC_HW_AUTO, FALSE, TRUE
};

static const qspi_cmd_type cmd_erase = {
    FALSE, 0, 0x20, QSPI_CMD_INSLEN_1_BYTE, 0, QSPI_CMD_ADRLEN_3_BYTE,
    0, 0, QSPI_OPERATE_MODE_111, QSPI_RSTSC_HW_AUTO, FALSE, TRUE
};

static const qspi_cmd_type cmd_wren = {
    FALSE, 0, 0x06, QSPI_CMD_INSLEN_1_BYTE, 0, QSPI_CMD_ADRLEN_0_BYTE,
    0, 0, QSPI_OPERATE_MODE_111, QSPI_RSTSC_HW_AUTO, FALSE, TRUE
};

static const qspi_cmd_type cmd_rdsr = {
    FALSE, 0, 0x05, QSPI_CMD_INSLEN_1_BYTE, 0, QSPI_CMD_ADRLEN_0_BYTE,
    0, 0, QSPI_OPERATE_MODE_111, QSPI_RSTSC_HW_AUTO, TRUE, FALSE
};

void qspi_gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_QSPI1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOG_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    
    /* IO0 (PF8), IO1 (PF9) */
    gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9;
    gpio_init(GPIOF, &gpio_init_struct);
    gpio_pin_mux_config(GPIOF, GPIO_PINS_SOURCE8, GPIO_MUX_10);
    gpio_pin_mux_config(GPIOF, GPIO_PINS_SOURCE9, GPIO_MUX_10);
    
    /* IO2 (PF7), IO3 (PF6), SCK (PF10) */
    gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7 | GPIO_PINS_10;
    gpio_init(GPIOF, &gpio_init_struct);
    gpio_pin_mux_config(GPIOF, GPIO_PINS_SOURCE6, GPIO_MUX_9);
    gpio_pin_mux_config(GPIOF, GPIO_PINS_SOURCE7, GPIO_MUX_9);
    gpio_pin_mux_config(GPIOF, GPIO_PINS_SOURCE10, GPIO_MUX_9);
    
    /* CS (PG6) */
    gpio_init_struct.gpio_pins = GPIO_PINS_6;
    gpio_init(GPIOG, &gpio_init_struct);
    gpio_pin_mux_config(GPIOG, GPIO_PINS_SOURCE6, GPIO_MUX_10);
}

void qspi_cmd_send(qspi_cmd_type* cmd)
{
    qspi_cmd_operation_kick(QSPI1, cmd);
    while(qspi_flag_get(QSPI1, QSPI_CMDSTS_FLAG) == RESET);
    qspi_flag_clear(QSPI1, QSPI_CMDSTS_FLAG);
}

void qspi_write_enable(void)
{
    qspi_cmd_send((qspi_cmd_type*)&cmd_wren);
}

void qspi_busy_check(void)
{
    qspi_cmd_send((qspi_cmd_type*)&cmd_rdsr);
}

void qspi_sector_erase(uint32_t addr)
{
    qspi_cmd_type cmd = cmd_erase;
    
    qspi_write_enable();
    cmd.address_code = addr;
    qspi_cmd_send(&cmd);
    qspi_busy_check();
}

void qspi_data_read(uint32_t addr, uint32_t len, uint8_t* buf)
{
    qspi_cmd_type cmd = cmd_read;
    uint32_t i, chunk;
    
    cmd.address_code = addr;
    cmd.data_counter = len;
    qspi_cmd_operation_kick(QSPI1, &cmd);
    
    while(len > 0)
    {
        chunk = (len >= QSPI_FIFO_DEPTH) ? QSPI_FIFO_DEPTH : len;
        
        while(qspi_flag_get(QSPI1, QSPI_RXFIFORDY_FLAG) == RESET);
        
        for(i = 0; i < chunk; i++)
        {
            *buf++ = qspi_byte_read(QSPI1);
        }
        len -= chunk;
    }
    
    while(qspi_flag_get(QSPI1, QSPI_CMDSTS_FLAG) == RESET);
    qspi_flag_clear(QSPI1, QSPI_CMDSTS_FLAG);
}

void qspi_data_write(uint32_t addr, uint32_t len, uint8_t* buf)
{
    qspi_cmd_type cmd = cmd_write;
    uint32_t i, page_len;
    
    while(len > 0)
    {
        qspi_write_enable();
        
        /* Calculate bytes to page boundary */
        page_len = FLASH_PAGE_SIZE - (addr % FLASH_PAGE_SIZE);
        if(len < page_len) page_len = len;
        
        cmd.address_code = addr;
        cmd.data_counter = page_len;
        qspi_cmd_operation_kick(QSPI1, &cmd);
        
        for(i = 0; i < page_len; i++)
        {
            while(qspi_flag_get(QSPI1, QSPI_TXFIFORDY_FLAG) == RESET);
            qspi_byte_write(QSPI1, *buf++);
        }
        
        while(qspi_flag_get(QSPI1, QSPI_CMDSTS_FLAG) == RESET);
        qspi_flag_clear(QSPI1, QSPI_CMDSTS_FLAG);
        
        qspi_busy_check();
        
        addr += page_len;
        len -= page_len;
    }
}

int main(void)
{
    uint16_t i, err = 0;
    
    system_clock_config();
    at32_board_init();
    
    /* Initialize test data */
    for(i = 0; i < TEST_SIZE; i++)
    {
        wbuf[i] = (uint8_t)i;
        rbuf[i] = 0;
    }
    
    /* Configure QSPI GPIO */
    qspi_gpio_config();
    
    /* Configure QSPI */
    qspi_xip_enable(QSPI1, FALSE);  /* Command-port mode */
    qspi_clk_division_set(QSPI1, QSPI_CLK_DIV_4);
    qspi_sck_mode_set(QSPI1, QSPI_SCK_MODE_0);
    qspi_busy_config(QSPI1, QSPI_BUSY_OFFSET_0);
    qspi_auto_ispc_enable(QSPI1);
    
    /* Erase sector */
    qspi_sector_erase(0);
    
    /* Verify erase */
    qspi_data_read(0, TEST_SIZE, rbuf);
    for(i = 0; i < TEST_SIZE; i++)
    {
        if(rbuf[i] != 0xFF) { err = 1; break; }
    }
    
    /* Program data */
    qspi_data_write(0, TEST_SIZE, wbuf);
    
    /* Read back and verify */
    qspi_data_read(0, TEST_SIZE, rbuf);
    if(memcmp(rbuf, wbuf, TEST_SIZE)) err = 1;
    
    while(1)
    {
        if(err == 0)
            at32_led_toggle(LED3);
        else
            at32_led_toggle(LED2);
        delay_ms(300);
    }
}
```

---

### Example 2: XIP Memory-Mapped Mode

Configure QSPI for XIP mode to access flash as memory.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include <string.h>

#define TEST_SIZE    4096

ALIGNED_HEAD uint8_t wbuf[TEST_SIZE] ALIGNED_TAIL;
ALIGNED_HEAD uint8_t rbuf[TEST_SIZE] ALIGNED_TAIL;

/* XIP configuration for EN25QH128A */
static const qspi_xip_type xip_config = {
    .read_instruction_code       = 0x6B,  /* Quad Output Read */
    .read_address_length         = QSPI_XIP_ADDRLEN_3_BYTE,
    .read_operation_mode         = QSPI_OPERATE_MODE_114,
    .read_second_dummy_cycle_num = 8,
    .write_instruction_code      = 0x32,  /* Quad Page Program */
    .write_address_length        = QSPI_XIP_ADDRLEN_3_BYTE,
    .write_operation_mode        = QSPI_OPERATE_MODE_114,
    .write_second_dummy_cycle_num = 0,
    .write_select_mode           = QSPI_XIPW_SEL_MODED,
    .write_time_counter          = 0x7F,
    .write_data_counter          = 0x1F,
    .read_select_mode            = QSPI_XIPR_SEL_MODET,
    .read_time_counter           = 0x7F,
    .read_data_counter           = 0x1F
};

extern void qspi_data_write(uint32_t addr, uint32_t total_len, uint8_t* buf);
extern void qspi_erase(uint32_t sec_addr);

void qspi_xip_mode_init(void)
{
    /* Switch to command-port mode first */
    qspi_xip_enable(QSPI1, FALSE);
    
    /* Configure XIP parameters */
    qspi_xip_init(QSPI1, (qspi_xip_type*)&xip_config);
    
    /* Enable XIP mode */
    qspi_xip_enable(QSPI1, TRUE);
}

int main(void)
{
    uint16_t i, err = 0;
    
    system_clock_config();
    at32_board_init();
    
    /* Initialize test data */
    for(i = 0; i < TEST_SIZE; i++)
    {
        wbuf[i] = (uint8_t)i;
        rbuf[i] = 0;
    }
    
    /* Configure QSPI GPIO (same as Example 1) */
    qspi_gpio_config();
    
    /* Configure QSPI basics */
    qspi_xip_enable(QSPI1, FALSE);
    qspi_clk_division_set(QSPI1, QSPI_CLK_DIV_4);
    qspi_sck_mode_set(QSPI1, QSPI_SCK_MODE_0);
    qspi_busy_config(QSPI1, QSPI_BUSY_OFFSET_0);
    qspi_auto_ispc_enable(QSPI1);
    
    /* Program data using command-port mode */
    qspi_erase(0);
    qspi_data_write(0, TEST_SIZE, wbuf);
    
    /* Switch to XIP mode */
    qspi_xip_mode_init();
    
    /* Read via memory-mapped access! */
    memcpy(rbuf, (uint8_t*)QSPI1_MEM_BASE, TEST_SIZE);
    
    /* Or access directly like regular memory: */
    /* uint8_t value = *(volatile uint8_t*)(QSPI1_MEM_BASE + offset); */
    
    /* Verify */
    if(memcmp(rbuf, wbuf, TEST_SIZE)) err = 1;
    
    while(1)
    {
        if(err == 0)
            at32_led_toggle(LED3);
        else
            at32_led_toggle(LED2);
        delay_ms(300);
    }
}
```

---

### Example 3: DMA-Based Data Transfer

Use DMA for efficient data transfer.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include <string.h>

#define TEST_SIZE    4096

ALIGNED_HEAD uint8_t wbuf[TEST_SIZE] ALIGNED_TAIL;
ALIGNED_HEAD uint8_t rbuf[TEST_SIZE] ALIGNED_TAIL;

void qspi_dma_init(dma_dir_type dir, uint8_t* buf, uint32_t length)
{
    dma_init_type dma_init_struct;
    
    dma_reset(DMA2_CHANNEL1);
    dma_default_para_init(&dma_init_struct);
    
    dma_init_struct.buffer_size = length / 4;  /* Word count */
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init_struct.direction = dir;
    dma_init_struct.memory_base_addr = (uint32_t)buf;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)(&QSPI1->dt);
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    
    dma_init(DMA2_CHANNEL1, &dma_init_struct);
    
    /* Configure DMAMUX */
    dmamux_init(DMA2MUX_CHANNEL1, DMAMUX_DMAREQ_ID_QSPI1);
    dmamux_enable(DMA2, TRUE);
    
    dma_channel_enable(DMA2_CHANNEL1, TRUE);
}

void qspi_dma_read(uint32_t addr, uint32_t len, uint8_t* buf)
{
    qspi_cmd_type cmd = {
        FALSE, 0, 0xEB, QSPI_CMD_INSLEN_1_BYTE, addr, QSPI_CMD_ADRLEN_3_BYTE,
        len, 6, QSPI_OPERATE_MODE_144, QSPI_RSTSC_HW_AUTO, FALSE, FALSE
    };
    
    /* Enable QSPI DMA */
    qspi_dma_enable(QSPI1, TRUE);
    qspi_dma_rx_threshold_set(QSPI1, QSPI_DMA_FIFO_THOD_WORD08);
    
    /* Configure DMA */
    qspi_dma_init(DMA_DIR_PERIPHERAL_TO_MEMORY, buf, len);
    
    /* Start command */
    qspi_cmd_operation_kick(QSPI1, &cmd);
    while(qspi_flag_get(QSPI1, QSPI_CMDSTS_FLAG) == RESET);
    qspi_flag_clear(QSPI1, QSPI_CMDSTS_FLAG);
    
    /* Wait DMA complete */
    while(dma_flag_get(DMA2_FDT1_FLAG) == RESET);
    dma_flag_clear(DMA2_FDT1_FLAG);
    
    qspi_dma_enable(QSPI1, FALSE);
}

void qspi_dma_write(uint32_t addr, uint32_t len, uint8_t* buf)
{
    qspi_cmd_type cmd = {
        FALSE, 0, 0x32, QSPI_CMD_INSLEN_1_BYTE, addr, QSPI_CMD_ADRLEN_3_BYTE,
        len, 0, QSPI_OPERATE_MODE_114, QSPI_RSTSC_HW_AUTO, FALSE, TRUE
    };
    
    qspi_write_enable();
    
    /* Enable QSPI DMA */
    qspi_dma_enable(QSPI1, TRUE);
    qspi_dma_tx_threshold_set(QSPI1, QSPI_DMA_FIFO_THOD_WORD08);
    
    /* Configure DMA */
    qspi_dma_init(DMA_DIR_MEMORY_TO_PERIPHERAL, buf, len);
    
    /* Start command */
    qspi_cmd_operation_kick(QSPI1, &cmd);
    while(qspi_flag_get(QSPI1, QSPI_CMDSTS_FLAG) == RESET);
    qspi_flag_clear(QSPI1, QSPI_CMDSTS_FLAG);
    
    /* Wait DMA complete */
    while(dma_flag_get(DMA2_FDT1_FLAG) == RESET);
    dma_flag_clear(DMA2_FDT1_FLAG);
    
    qspi_dma_enable(QSPI1, FALSE);
    qspi_busy_check();
}

int main(void)
{
    uint16_t i, err = 0;
    
    system_clock_config();
    at32_board_init();
    
    /* Enable DMA clock */
    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
    
    for(i = 0; i < TEST_SIZE; i++)
    {
        wbuf[i] = (uint8_t)i;
        rbuf[i] = 0;
    }
    
    qspi_gpio_config();
    
    qspi_xip_enable(QSPI1, FALSE);
    qspi_clk_division_set(QSPI1, QSPI_CLK_DIV_4);
    qspi_sck_mode_set(QSPI1, QSPI_SCK_MODE_0);
    qspi_busy_config(QSPI1, QSPI_BUSY_OFFSET_0);
    qspi_auto_ispc_enable(QSPI1);
    
    /* Erase and program using DMA */
    qspi_sector_erase(0);
    qspi_dma_write(0, TEST_SIZE, wbuf);
    
    /* Read using DMA */
    qspi_dma_read(0, TEST_SIZE, rbuf);
    
    if(memcmp(rbuf, wbuf, TEST_SIZE)) err = 1;
    
    while(1)
    {
        at32_led_toggle(err ? LED2 : LED3);
        delay_ms(300);
    }
}
```

---

### Example 4: Interrupt-Based Command Completion

Use interrupts to detect command completion.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

volatile uint8_t cmd_complete = 0;

void QSPI1_IRQHandler(void)
{
    if(qspi_interrupt_flag_get(QSPI1, QSPI_CMDSTS_FLAG) != RESET)
    {
        qspi_flag_clear(QSPI1, QSPI_CMDSTS_FLAG);
        cmd_complete = 1;
    }
}

void qspi_cmd_send_async(qspi_cmd_type* cmd)
{
    cmd_complete = 0;
    qspi_cmd_operation_kick(QSPI1, cmd);
}

void qspi_wait_complete(void)
{
    while(cmd_complete == 0);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    qspi_gpio_config();
    
    qspi_xip_enable(QSPI1, FALSE);
    qspi_clk_division_set(QSPI1, QSPI_CLK_DIV_4);
    qspi_sck_mode_set(QSPI1, QSPI_SCK_MODE_0);
    qspi_busy_config(QSPI1, QSPI_BUSY_OFFSET_0);
    qspi_auto_ispc_enable(QSPI1);
    
    /* Enable interrupt */
    nvic_irq_enable(QSPI1_IRQn, 0, 0);
    qspi_interrupt_enable(QSPI1, TRUE);
    
    /* Send command asynchronously */
    qspi_cmd_type cmd_rdid = {
        FALSE, 0, 0x9F, QSPI_CMD_INSLEN_1_BYTE, 0, QSPI_CMD_ADRLEN_0_BYTE,
        3, 0, QSPI_OPERATE_MODE_111, QSPI_RSTSC_HW_AUTO, FALSE, FALSE
    };
    
    qspi_cmd_send_async(&cmd_rdid);
    
    /* Do other work... */
    at32_led_on(LED2);
    
    /* Wait for completion */
    qspi_wait_complete();
    
    /* Read ID bytes */
    uint8_t mfr_id = qspi_byte_read(QSPI1);
    uint8_t dev_id1 = qspi_byte_read(QSPI1);
    uint8_t dev_id2 = qspi_byte_read(QSPI1);
    
    at32_led_on(LED3);
    
    while(1);
}
```

---

## Configuration Checklist

### Command-Port Mode Setup

- [ ] Enable QSPI and GPIO clocks
- [ ] Configure GPIO pins with correct MUX
- [ ] Disable XIP mode: `qspi_xip_enable(QSPI1, FALSE)`
- [ ] Set clock division: `qspi_clk_division_set()`
- [ ] Set SCK mode: `qspi_sck_mode_set()`
- [ ] Configure busy bit position: `qspi_busy_config()`
- [ ] Enable auto ISPC for high speed: `qspi_auto_ispc_enable()`

### XIP Mode Setup

- [ ] Configure flash for Quad mode (if needed)
- [ ] Initialize XIP parameters: `qspi_xip_init()`
- [ ] Enable XIP mode: `qspi_xip_enable(QSPI1, TRUE)`
- [ ] Access flash via `QSPI1_MEM_BASE` (0x90000000)

### DMA Transfer Setup

- [ ] Enable DMA clock
- [ ] Configure DMAMUX for QSPI
- [ ] Set FIFO threshold: `qspi_dma_rx/tx_threshold_set()`
- [ ] Enable QSPI DMA: `qspi_dma_enable()`
- [ ] Ensure buffer is word-aligned

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| No response from flash | Wrong GPIO MUX | Verify pin configuration |
| Read data all 0xFF | Flash not erased state | Check if data was programmed |
| Read data incorrect | Wrong dummy cycles | Check flash datasheet |
| XIP access hangs | Flash not in Quad mode | Enable QE bit in flash |
| DMA transfer incomplete | Buffer not aligned | Align buffer to 4 bytes |
| High-speed errors | Timing issues | Enable `qspi_auto_ispc_enable()` |
| Program fails | Write not enabled | Call Write Enable before program |
| Erase incomplete | Busy check missing | Wait for WIP bit to clear |

---

## Supported Flash Devices

The examples support common SPI NOR flash devices:

| Manufacturer | Part Number | Size | Interface |
|-------------|-------------|------|-----------|
| Eon | EN25QH128A | 16MB | SPI/Dual/Quad |
| ESMT | ESMT32M | 32MB | SPI/Dual/Quad |
| Winbond | W25Q128 | 16MB | SPI/Dual/Quad |
| GigaDevice | GD25Q128 | 16MB | SPI/Dual/Quad |

---

## Related Peripherals

- **[DMA](DMA_Direct_Memory_Access.md)** - DMA transfers for QSPI
- **[GPIO](GPIO_General_Purpose_IO.md)** - Pin configuration
- **[CRM](CRM_Clock_Reset_Management.md)** - Clock configuration

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-01 | Initial release |

