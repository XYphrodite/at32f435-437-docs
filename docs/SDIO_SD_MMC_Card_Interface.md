---
title: SDIO - SD/MMC Card Interface
category: Peripheral
complexity: Advanced
mcu: AT32F435/437
peripheral: SDIO
keywords: [sdio, sd, mmc, emmc, card, dma, fatfs, storage, memory]
---

# SDIO - SD/MMC Card Interface

## Overview

The Secure Digital Input/Output (SDIO) interface provides a high-speed connection to SD memory cards, MMC cards, and eMMC devices. It supports multiple bus widths, DMA transfers, and both block and stream data transfer modes, making it ideal for mass storage and file system applications.

### Key Features

| Feature | Specification |
|---------|---------------|
| SDIO Interfaces | 2 (SDIO1, SDIO2) |
| Bus Width | 1-bit, 4-bit, 8-bit (MMC only) |
| Transfer Modes | Block, Stream |
| Clock Frequency | Up to 50 MHz |
| Clock Division | 0-1023 (SDIO_CK = AHB_CK / [CLKDIV + 2]) |
| Response Types | None, Short (48-bit), Long (136-bit) |
| DMA Support | Yes |
| Hardware Flow Control | Yes |
| FIFO | 32 x 32-bit words |

### Supported Card Types

| Type | Description |
|------|-------------|
| SD V1.1 | Standard Capacity SD Card Version 1.1 |
| SD V2.0 | Standard Capacity SD Card Version 2.0 |
| SDHC | High Capacity SD Card (up to 32GB) |
| MMC | MultiMediaCard |
| MMC V4.2+ | High Speed MMC |
| eMMC | Embedded MMC (High Capacity) |

## Architecture

```
                    ┌────────────────────────────────────────────────────────────┐
                    │                     SDIO Controller                        │
                    │                                                            │
┌─────────────┐     │  ┌────────────────────────────────────────────────────┐   │
│ AHB Bus     │─────│──│              Register Interface                    │   │
│             │     │  │  PWRCTRL │ CLKCTRL │ ARG │ CMDCTRL │ RSP1-4       │   │
└─────────────┘     │  │  DTTMR │ DTLEN │ DTCTRL │ DTCNT │ STS │ INTEN     │   │
                    │  └────────────────────────────────────────────────────┘   │
                    │                                                            │
                    │  ┌────────────────────┐  ┌──────────────────────────────┐ │
                    │  │   Command Path     │  │        Data Path             │ │
                    │  │   State Machine    │  │    State Machine             │ │
                    │  │      (CPSM)        │  │      (DPSM)                  │ │
                    │  │                    │  │                              │ │
                    │  │ ┌──────────────┐   │  │  ┌──────────────────────┐   │ │
                    │  │ │ CMD Register │   │  │  │   TX/RX FIFO         │   │ │
                    │  │ │ RSP Register │   │  │  │   32 x 32-bit        │   │ │
                    │  │ └──────────────┘   │  │  └──────────────────────┘   │ │
                    │  └─────────┬──────────┘  └────────────┬─────────────────┘ │
                    │            │                          │                   │
                    │  ┌─────────▼──────────────────────────▼─────────────────┐ │
                    │  │              Clock Generator                          │ │
                    │  │    SDIO_CK = AHB_CK / (CLKDIV + 2)                   │ │
                    │  │    Clock Edge: Rising / Falling                       │ │
                    │  │    Power Saving: Clock gating when idle               │ │
                    │  └──────────────────────────────────────────────────────┘ │
                    │                                                            │
                    │      ┌─────┐  ┌─────┐  ┌─────┐  ┌─────────────────────┐  │
                    │      │ CLK │  │ CMD │  │ D0  │  │  D1   D2   D3   D7  │  │
                    └──────┴──┬──┴──┴──┬──┴──┴──┬──┴──┴───────┬─────────────┴──┘
                              │        │        │             │
                              ▼        ▼        ▼             ▼
                    ┌─────────────────────────────────────────────────────────────┐
                    │                 SD/MMC/eMMC Card                            │
                    └─────────────────────────────────────────────────────────────┘
```

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| PWRCTRL | 0x00 | Power control |
| CLKCTRL | 0x04 | Clock control |
| ARGU | 0x08 | Command argument |
| CMDCTRL | 0x0C | Command control |
| RSPCMD | 0x10 | Response command |
| RSP1 | 0x14 | Response register 1 |
| RSP2 | 0x18 | Response register 2 |
| RSP3 | 0x1C | Response register 3 |
| RSP4 | 0x20 | Response register 4 |
| DTTMR | 0x24 | Data timeout timer |
| DTLEN | 0x28 | Data length |
| DTCTRL | 0x2C | Data control |
| DTCNT | 0x30 | Data counter |
| STS | 0x34 | Status register |
| INTCLR | 0x38 | Interrupt clear |
| INTEN | 0x3C | Interrupt enable |
| BUFCNT | 0x48 | FIFO counter |
| BUF | 0x80 | Data FIFO |

### CLKCTRL Register (0x04)

| Bits | Name | Description |
|------|------|-------------|
| 7:0 | CLKDIV_L | Clock divider low bits |
| 8 | CLKOEN | Clock output enable |
| 9 | PWRSVEN | Power saving mode enable |
| 10 | BYPSEN | Clock divider bypass |
| 12:11 | BUSWS | Bus width selection |
| 13 | CLKEGS | Clock edge selection |
| 14 | HFCEN | Hardware flow control enable |
| 16:15 | CLKDIV_H | Clock divider high bits |

### DTCTRL Register (0x2C)

| Bits | Name | Description |
|------|------|-------------|
| 0 | TFREN | Data transfer enable |
| 1 | TFRDIR | Transfer direction: 0=write, 1=read |
| 2 | TFRMODE | Transfer mode: 0=block, 1=stream |
| 3 | DMAEN | DMA enable |
| 7:4 | BLKSIZE | Block size (2^n bytes) |
| 8 | RDWTSTART | Read wait start |
| 9 | RDWTSTOP | Read wait stop |
| 10 | RDWTMODE | Read wait mode |
| 11 | IOEN | SD I/O enable |

## Configuration Types

### Power State

| Enum | Value | Description |
|------|-------|-------------|
| `SDIO_POWER_OFF` | 0x00 | Power off, clock stopped |
| `SDIO_POWER_ON` | 0x03 | Power on, card is clocked |

### Bus Width

| Enum | Value | Lines | Description |
|------|-------|-------|-------------|
| `SDIO_BUS_WIDTH_D1` | 0x00 | 1 | 1-bit bus (D0 only) |
| `SDIO_BUS_WIDTH_D4` | 0x01 | 4 | 4-bit bus (D0-D3) |
| `SDIO_BUS_WIDTH_D8` | 0x02 | 8 | 8-bit bus (D0-D7, MMC only) |

### Clock Edge

| Enum | Value | Description |
|------|-------|-------------|
| `SDIO_CLOCK_EDGE_RISING` | 0x00 | Clock on rising edge |
| `SDIO_CLOCK_EDGE_FALLING` | 0x01 | Clock on falling edge |

### Response Type

| Enum | Value | Description |
|------|-------|-------------|
| `SDIO_RESPONSE_NO` | 0x00 | No response |
| `SDIO_RESPONSE_SHORT` | 0x01 | Short response (48-bit) |
| `SDIO_RESPONSE_LONG` | 0x03 | Long response (136-bit) |

### Data Block Size

| Enum | Value | Size |
|------|-------|------|
| `SDIO_DATA_BLOCK_SIZE_1B` | 0x00 | 1 byte |
| `SDIO_DATA_BLOCK_SIZE_2B` | 0x01 | 2 bytes |
| `SDIO_DATA_BLOCK_SIZE_4B` | 0x02 | 4 bytes |
| `SDIO_DATA_BLOCK_SIZE_8B` | 0x03 | 8 bytes |
| `SDIO_DATA_BLOCK_SIZE_16B` | 0x04 | 16 bytes |
| `SDIO_DATA_BLOCK_SIZE_32B` | 0x05 | 32 bytes |
| `SDIO_DATA_BLOCK_SIZE_64B` | 0x06 | 64 bytes |
| `SDIO_DATA_BLOCK_SIZE_128B` | 0x07 | 128 bytes |
| `SDIO_DATA_BLOCK_SIZE_256B` | 0x08 | 256 bytes |
| `SDIO_DATA_BLOCK_SIZE_512B` | 0x09 | 512 bytes |
| `SDIO_DATA_BLOCK_SIZE_1024B` | 0x0A | 1024 bytes |
| `SDIO_DATA_BLOCK_SIZE_2048B` | 0x0B | 2048 bytes |
| `SDIO_DATA_BLOCK_SIZE_4096B` | 0x0C | 4096 bytes |

### Transfer Mode

| Enum | Value | Description |
|------|-------|-------------|
| `SDIO_DATA_BLOCK_TRANSFER` | 0x00 | Block transfer mode |
| `SDIO_DATA_STREAM_TRANSFER` | 0x01 | Stream transfer mode (MMC only) |

### Transfer Direction

| Enum | Value | Description |
|------|-------|-------------|
| `SDIO_DATA_TRANSFER_TO_CARD` | 0x00 | Write data to card |
| `SDIO_DATA_TRANSFER_TO_CONTROLLER` | 0x01 | Read data from card |

## Flags and Interrupts

### Status Flags

| Flag | Description |
|------|-------------|
| `SDIO_CMDFAIL_FLAG` | Command response CRC error |
| `SDIO_DTFAIL_FLAG` | Data block CRC error |
| `SDIO_CMDTIMEOUT_FLAG` | Command response timeout |
| `SDIO_DTTIMEOUT_FLAG` | Data timeout |
| `SDIO_TXERRU_FLAG` | Transmit FIFO underrun |
| `SDIO_RXERRO_FLAG` | Receive FIFO overrun |
| `SDIO_CMDRSPCMPL_FLAG` | Command response received |
| `SDIO_CMDCMPL_FLAG` | Command sent (no response expected) |
| `SDIO_DTCMPL_FLAG` | Data transfer complete |
| `SDIO_SBITERR_FLAG` | Start bit error |
| `SDIO_DTBLKCMPL_FLAG` | Data block transfer complete |
| `SDIO_DOCMD_FLAG` | Command in progress |
| `SDIO_DOTX_FLAG` | Data transmit in progress |
| `SDIO_DORX_FLAG` | Data receive in progress |
| `SDIO_TXBUFH_FLAG` | TX FIFO half empty |
| `SDIO_RXBUFH_FLAG` | RX FIFO half full |
| `SDIO_TXBUFF_FLAG` | TX FIFO full |
| `SDIO_RXBUFF_FLAG` | RX FIFO full |
| `SDIO_TXBUFE_FLAG` | TX FIFO empty |
| `SDIO_RXBUFE_FLAG` | RX FIFO empty |
| `SDIO_TXBUF_FLAG` | TX FIFO has data |
| `SDIO_RXBUF_FLAG` | RX FIFO has data |
| `SDIO_SDIOIF_FLAG` | SDIO interrupt |

## Pin Configuration

### SDIO1 Pins

| Signal | Pin | MUX | Description |
|--------|-----|-----|-------------|
| CLK | PC12 | AF12 | Clock output |
| CMD | PD2 | AF12 | Command line |
| D0 | PC8 | AF12 | Data line 0 |
| D1 | PC9 | AF12 | Data line 1 |
| D2 | PC10 | AF12 | Data line 2 |
| D3 | PC11 | AF12 | Data line 3 |
| D4 | PB8 | AF12 | Data line 4 (8-bit) |
| D5 | PB9 | AF12 | Data line 5 (8-bit) |
| D6 | PC6 | AF12 | Data line 6 (8-bit) |
| D7 | PC7 | AF12 | Data line 7 (8-bit) |

### SDIO2 Pins

| Signal | Pin | MUX |
|--------|-----|-----|
| CLK | PD6 | AF11 |
| CMD | PD7 | AF11 |
| D0 | PG9 | AF11 |
| D1 | PG10 | AF11 |
| D2 | PB3 | AF10 |
| D3 | PB4 | AF10 |

## Command Structure

```c
typedef struct
{
    uint32_t         argument;   /* Command argument */
    uint8_t          cmd_index;  /* Command index (0-63) */
    sdio_reponse_type rsp_type;  /* Response type */
    sdio_wait_type   wait_type;  /* Wait mode */
} sdio_command_struct_type;
```

## Data Structure

```c
typedef struct
{
    uint32_t                      timeout;            /* Data timeout */
    uint32_t                      data_length;        /* Total data length */
    sdio_block_size_type          block_size;         /* Block size */
    sdio_transfer_mode_type       transfer_mode;      /* Block or stream */
    sdio_transfer_direction_type  transfer_direction; /* Read or write */
} sdio_data_struct_type;
```

## API Reference

### Initialization

#### `sdio_reset`

```c
void sdio_reset(sdio_type *sdio_x);
```

Reset SDIO peripheral to default values.

---

#### `sdio_power_set`

```c
void sdio_power_set(sdio_type *sdio_x, sdio_power_state_type power_state);
```

Set the power state of SDIO.

---

#### `sdio_power_status_get`

```c
sdio_power_state_type sdio_power_status_get(sdio_type *sdio_x);
```

Get the current power state.

---

### Clock Configuration

#### `sdio_clock_config`

```c
void sdio_clock_config(sdio_type *sdio_x, uint16_t clk_div, sdio_edge_phase_type clk_edg);
```

Configure SDIO clock division and edge.

| Parameter | Description |
|-----------|-------------|
| `sdio_x` | SDIO1 or SDIO2 |
| `clk_div` | Clock divider (0-1023), SDIO_CK = AHB_CK / (clk_div + 2) |
| `clk_edg` | Clock edge selection |

---

#### `sdio_bus_width_config`

```c
void sdio_bus_width_config(sdio_type *sdio_x, sdio_bus_width_type width);
```

Configure bus width (1-bit, 4-bit, or 8-bit).

---

#### `sdio_clock_enable`

```c
void sdio_clock_enable(sdio_type *sdio_x, confirm_state new_state);
```

Enable or disable clock output.

---

#### `sdio_clock_bypass`

```c
void sdio_clock_bypass(sdio_type *sdio_x, confirm_state new_state);
```

Enable or disable clock divider bypass.

---

#### `sdio_power_saving_mode_enable`

```c
void sdio_power_saving_mode_enable(sdio_type *sdio_x, confirm_state new_state);
```

Enable or disable power saving mode (clock gating when idle).

---

#### `sdio_flow_control_enable`

```c
void sdio_flow_control_enable(sdio_type *sdio_x, confirm_state new_state);
```

Enable or disable hardware flow control.

---

### Command Operations

#### `sdio_command_config`

```c
void sdio_command_config(sdio_type *sdio_x, sdio_command_struct_type *command_struct);
```

Configure a command to send.

---

#### `sdio_command_state_machine_enable`

```c
void sdio_command_state_machine_enable(sdio_type *sdio_x, confirm_state new_state);
```

Enable or disable the Command Path State Machine (CPSM).

---

#### `sdio_command_response_get`

```c
uint8_t sdio_command_response_get(sdio_type *sdio_x);
```

Get the command index of the last received response.

---

#### `sdio_response_get`

```c
uint32_t sdio_response_get(sdio_type *sdio_x, sdio_rsp_index_type reg_index);
```

Get response data from RSP1-4 registers.

---

### Data Operations

#### `sdio_data_config`

```c
void sdio_data_config(sdio_type *sdio_x, sdio_data_struct_type *data_struct);
```

Configure data transfer parameters.

---

#### `sdio_data_state_machine_enable`

```c
void sdio_data_state_machine_enable(sdio_type *sdio_x, confirm_state new_state);
```

Enable or disable the Data Path State Machine (DPSM).

---

#### `sdio_data_read`

```c
uint32_t sdio_data_read(sdio_type *sdio_x);
```

Read one word from the FIFO.

---

#### `sdio_data_write`

```c
void sdio_data_write(sdio_type *sdio_x, uint32_t data);
```

Write one word to the FIFO.

---

#### `sdio_data_counter_get`

```c
uint32_t sdio_data_counter_get(sdio_type *sdio_x);
```

Get remaining data bytes to transfer.

---

#### `sdio_buffer_counter_get`

```c
uint32_t sdio_buffer_counter_get(sdio_type *sdio_x);
```

Get FIFO word count.

---

### DMA and Interrupts

#### `sdio_dma_enable`

```c
void sdio_dma_enable(sdio_type *sdio_x, confirm_state new_state);
```

Enable or disable DMA.

---

#### `sdio_interrupt_enable`

```c
void sdio_interrupt_enable(sdio_type *sdio_x, uint32_t int_opt, confirm_state new_state);
```

Enable or disable interrupts.

---

#### `sdio_flag_get`

```c
flag_status sdio_flag_get(sdio_type *sdio_x, uint32_t flag);
```

Get flag status.

---

#### `sdio_interrupt_flag_get`

```c
flag_status sdio_interrupt_flag_get(sdio_type *sdio_x, uint32_t flag);
```

Get interrupt flag status.

---

#### `sdio_flag_clear`

```c
void sdio_flag_clear(sdio_type *sdio_x, uint32_t flag);
```

Clear flags.

---

## Code Examples

### Example 1: SD Card Initialization and Block Read/Write

Basic SD card operations with polling mode.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include <string.h>

#define BLOCK_SIZE  512

uint8_t tx_buffer[BLOCK_SIZE];
uint8_t rx_buffer[BLOCK_SIZE];

/* SDIO GPIO configuration */
void sdio_gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SDIO1_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    
    /* PC8-PC12: D0-D3, CLK */
    gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10 | 
                                 GPIO_PINS_11 | GPIO_PINS_12;
    gpio_init(GPIOC, &gpio_init_struct);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE8, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE9, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE10, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE11, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE12, GPIO_MUX_12);
    
    /* PD2: CMD */
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init(GPIOD, &gpio_init_struct);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE2, GPIO_MUX_12);
}

/* Initialize SDIO for 400kHz identification */
void sdio_low_speed_init(void)
{
    uint32_t clk_div;
    
    sdio_reset(SDIO1);
    
    /* 400kHz for card identification */
    clk_div = (system_core_clock / 400000) - 2;
    if(clk_div > 0x3FF) clk_div = 0x3FF;
    
    sdio_clock_config(SDIO1, clk_div, SDIO_CLOCK_EDGE_FALLING);
    sdio_bus_width_config(SDIO1, SDIO_BUS_WIDTH_D1);
    sdio_flow_control_enable(SDIO1, FALSE);
    sdio_clock_bypass(SDIO1, FALSE);
    sdio_power_saving_mode_enable(SDIO1, FALSE);
    
    sdio_power_set(SDIO1, SDIO_POWER_ON);
    sdio_clock_enable(SDIO1, TRUE);
}

/* Send command and wait for response */
sd_error_status_type sdio_send_cmd(uint8_t cmd_idx, uint32_t arg, 
                                    sdio_reponse_type rsp_type)
{
    sdio_command_struct_type cmd;
    uint32_t timeout = 0xFFFF;
    
    cmd.argument = arg;
    cmd.cmd_index = cmd_idx;
    cmd.rsp_type = rsp_type;
    cmd.wait_type = SDIO_WAIT_FOR_NO;
    
    sdio_command_config(SDIO1, &cmd);
    sdio_command_state_machine_enable(SDIO1, TRUE);
    
    if(rsp_type == SDIO_RESPONSE_NO)
    {
        while(timeout--)
        {
            if(sdio_flag_get(SDIO1, SDIO_CMDCMPL_FLAG) != RESET)
                break;
        }
    }
    else
    {
        while(timeout--)
        {
            if(sdio_flag_get(SDIO1, SDIO_CMDRSPCMPL_FLAG) != RESET)
                break;
            if(sdio_flag_get(SDIO1, SDIO_CMDTIMEOUT_FLAG) != RESET)
            {
                sdio_flag_clear(SDIO1, SDIO_CMDTIMEOUT_FLAG);
                return SD_CMD_RSP_TIMEOUT;
            }
        }
    }
    
    sdio_flag_clear(SDIO1, SDIO_CMDCMPL_FLAG | SDIO_CMDRSPCMPL_FLAG);
    return SD_OK;
}

/* Read single block */
sd_error_status_type sd_read_block(uint8_t *buf, uint32_t block_addr)
{
    sdio_data_struct_type data;
    uint32_t count = 0;
    uint32_t *p32 = (uint32_t *)buf;
    
    /* CMD16: Set block length */
    sdio_send_cmd(16, BLOCK_SIZE, SDIO_RESPONSE_SHORT);
    
    /* Configure data transfer */
    data.block_size = SDIO_DATA_BLOCK_SIZE_512B;
    data.data_length = BLOCK_SIZE;
    data.timeout = 0xFFFFFFFF;
    data.transfer_direction = SDIO_DATA_TRANSFER_TO_CONTROLLER;
    data.transfer_mode = SDIO_DATA_BLOCK_TRANSFER;
    
    sdio_data_config(SDIO1, &data);
    sdio_data_state_machine_enable(SDIO1, TRUE);
    
    /* CMD17: Read single block */
    sdio_send_cmd(17, block_addr, SDIO_RESPONSE_SHORT);
    
    /* Read data from FIFO */
    while(count < BLOCK_SIZE / 4)
    {
        if(sdio_flag_get(SDIO1, SDIO_RXBUF_FLAG) != RESET)
        {
            p32[count++] = sdio_data_read(SDIO1);
        }
        
        if(sdio_flag_get(SDIO1, SDIO_DTTIMEOUT_FLAG) != RESET)
        {
            sdio_flag_clear(SDIO1, SDIO_DTTIMEOUT_FLAG);
            return SD_DATA_TIMEOUT;
        }
    }
    
    while(sdio_flag_get(SDIO1, SDIO_DTCMPL_FLAG) == RESET);
    sdio_flag_clear(SDIO1, SDIO_DTCMPL_FLAG);
    
    return SD_OK;
}

/* Write single block */
sd_error_status_type sd_write_block(const uint8_t *buf, uint32_t block_addr)
{
    sdio_data_struct_type data;
    uint32_t count = 0;
    uint32_t *p32 = (uint32_t *)buf;
    
    /* CMD16: Set block length */
    sdio_send_cmd(16, BLOCK_SIZE, SDIO_RESPONSE_SHORT);
    
    /* CMD24: Write single block */
    sdio_send_cmd(24, block_addr, SDIO_RESPONSE_SHORT);
    
    /* Configure data transfer */
    data.block_size = SDIO_DATA_BLOCK_SIZE_512B;
    data.data_length = BLOCK_SIZE;
    data.timeout = 0xFFFFFFFF;
    data.transfer_direction = SDIO_DATA_TRANSFER_TO_CARD;
    data.transfer_mode = SDIO_DATA_BLOCK_TRANSFER;
    
    sdio_data_config(SDIO1, &data);
    sdio_data_state_machine_enable(SDIO1, TRUE);
    
    /* Write data to FIFO */
    while(count < BLOCK_SIZE / 4)
    {
        if(sdio_flag_get(SDIO1, SDIO_TXBUFH_FLAG) != RESET)
        {
            sdio_data_write(SDIO1, p32[count++]);
        }
    }
    
    while(sdio_flag_get(SDIO1, SDIO_DTCMPL_FLAG) == RESET)
    {
        if(sdio_flag_get(SDIO1, SDIO_DTTIMEOUT_FLAG) != RESET)
        {
            sdio_flag_clear(SDIO1, SDIO_DTTIMEOUT_FLAG);
            return SD_DATA_TIMEOUT;
        }
    }
    
    sdio_flag_clear(SDIO1, SDIO_DTCMPL_FLAG);
    return SD_OK;
}

int main(void)
{
    uint16_t i;
    
    system_clock_config();
    at32_board_init();
    
    /* Initialize test data */
    for(i = 0; i < BLOCK_SIZE; i++)
    {
        tx_buffer[i] = (uint8_t)i;
        rx_buffer[i] = 0;
    }
    
    sdio_gpio_config();
    sdio_low_speed_init();
    
    /* Card identification sequence would go here... */
    /* For complete initialization, use the SD library */
    
    while(1)
    {
        at32_led_toggle(LED3);
        delay_ms(500);
    }
}
```

---

### Example 2: Complete SD Card Driver with DMA

Using the provided SD card library with DMA transfers.

```c
#include "at32_sdio.h"
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include <string.h>

#define BLOCK_SIZE      512
#define BLOCKS_NUMBER   64
#define BUFFER_SIZE     (BLOCK_SIZE * BLOCKS_NUMBER)

uint8_t write_buffer[BUFFER_SIZE];
uint8_t read_buffer[BUFFER_SIZE];

void nvic_config(void)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(SDIO1_IRQn, 1, 0);
}

int main(void)
{
    sd_error_status_type status;
    uint16_t i;
    uint8_t err = 0;
    
    system_clock_config();
    at32_board_init();
    nvic_config();
    uart_print_init(115200);
    
    /* Initialize test data */
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        write_buffer[i] = (uint8_t)i;
        read_buffer[i] = 0;
    }
    
    printf("SD Card Test Start\r\n");
    
    /* Initialize SD card */
    status = sd_init();
    if(status != SD_OK)
    {
        printf("SD Init Failed: %d\r\n", status);
        while(1);
    }
    
    printf("SD Init OK\r\n");
    printf("Card Type: %d\r\n", sd_card_info.card_type);
    printf("Card Capacity: %u MB\r\n", (uint32_t)(sd_card_info.card_capacity >> 20));
    
    /* Set 4-bit bus width */
    status = sd_wide_bus_operation_config(SDIO_BUS_WIDTH_D4);
    if(status != SD_OK)
    {
        printf("Bus Width Config Failed\r\n");
    }
    
    /* Single block write/read test */
    printf("\r\nSingle Block Test...\r\n");
    
    status = sd_block_write(write_buffer, 0, BLOCK_SIZE);
    if(status != SD_OK)
    {
        printf("Write Failed: %d\r\n", status);
        err = 1;
    }
    
    status = sd_block_read(read_buffer, 0, BLOCK_SIZE);
    if(status != SD_OK)
    {
        printf("Read Failed: %d\r\n", status);
        err = 1;
    }
    
    if(memcmp(write_buffer, read_buffer, BLOCK_SIZE) != 0)
    {
        printf("Data Mismatch!\r\n");
        err = 1;
    }
    else
    {
        printf("Single Block Test OK\r\n");
    }
    
    /* Multi-block write/read test */
    printf("\r\nMulti Block Test...\r\n");
    memset(read_buffer, 0, BUFFER_SIZE);
    
    status = sd_mult_blocks_write(write_buffer, 0, BLOCK_SIZE, BLOCKS_NUMBER);
    if(status != SD_OK)
    {
        printf("Multi Write Failed: %d\r\n", status);
        err = 1;
    }
    
    status = sd_mult_blocks_read(read_buffer, 0, BLOCK_SIZE, BLOCKS_NUMBER);
    if(status != SD_OK)
    {
        printf("Multi Read Failed: %d\r\n", status);
        err = 1;
    }
    
    if(memcmp(write_buffer, read_buffer, BUFFER_SIZE) != 0)
    {
        printf("Multi Block Data Mismatch!\r\n");
        err = 1;
    }
    else
    {
        printf("Multi Block Test OK\r\n");
    }
    
    printf("\r\nTest %s\r\n", err ? "FAILED" : "PASSED");
    
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

### Example 3: Interrupt-Based Data Transfer

Using interrupts for data transfer completion.

```c
#include "at32_sdio.h"
#include "at32f435_437_board.h"

volatile uint8_t transfer_complete = 0;
volatile sd_error_status_type transfer_status = SD_OK;

void SDIO1_IRQHandler(void)
{
    /* Data transfer complete */
    if(sdio_interrupt_flag_get(SDIO1, SDIO_DTCMPL_FLAG) != RESET)
    {
        sdio_flag_clear(SDIO1, SDIO_DTCMPL_FLAG);
        transfer_complete = 1;
        transfer_status = SD_OK;
    }
    
    /* Data CRC error */
    if(sdio_interrupt_flag_get(SDIO1, SDIO_DTFAIL_FLAG) != RESET)
    {
        sdio_flag_clear(SDIO1, SDIO_DTFAIL_FLAG);
        transfer_complete = 1;
        transfer_status = SD_DATA_FAIL;
    }
    
    /* Data timeout */
    if(sdio_interrupt_flag_get(SDIO1, SDIO_DTTIMEOUT_FLAG) != RESET)
    {
        sdio_flag_clear(SDIO1, SDIO_DTTIMEOUT_FLAG);
        transfer_complete = 1;
        transfer_status = SD_DATA_TIMEOUT;
    }
    
    /* RX overrun */
    if(sdio_interrupt_flag_get(SDIO1, SDIO_RXERRO_FLAG) != RESET)
    {
        sdio_flag_clear(SDIO1, SDIO_RXERRO_FLAG);
        transfer_complete = 1;
        transfer_status = SD_RX_OVERRUN;
    }
    
    /* TX underrun */
    if(sdio_interrupt_flag_get(SDIO1, SDIO_TXERRU_FLAG) != RESET)
    {
        sdio_flag_clear(SDIO1, SDIO_TXERRU_FLAG);
        transfer_complete = 1;
        transfer_status = SD_TX_UNDERRUN;
    }
    
    /* Disable interrupts after completion */
    if(transfer_complete)
    {
        sdio_interrupt_enable(SDIO1, SDIO_DTCMPL_INT | SDIO_DTFAIL_INT | 
                             SDIO_DTTIMEOUT_INT | SDIO_RXERRO_INT | 
                             SDIO_TXERRU_INT, FALSE);
    }
}

sd_error_status_type sd_read_block_it(uint8_t *buf, uint32_t block_addr)
{
    transfer_complete = 0;
    transfer_status = SD_OK;
    
    /* Enable interrupts */
    sdio_interrupt_enable(SDIO1, SDIO_DTCMPL_INT | SDIO_DTFAIL_INT | 
                         SDIO_DTTIMEOUT_INT | SDIO_RXERRO_INT, TRUE);
    
    /* Start read operation */
    /* ... configure and start transfer ... */
    
    /* Wait for completion */
    while(!transfer_complete);
    
    return transfer_status;
}
```

---

## Configuration Checklist

### Basic SD Card Setup

- [ ] Enable GPIO and SDIO clocks
- [ ] Configure GPIO pins for SDIO (CLK, CMD, D0-D3)
- [ ] Reset SDIO: `sdio_reset()`
- [ ] Set clock ≤400kHz for identification: `sdio_clock_config()`
- [ ] Power on: `sdio_power_set(SDIO_POWER_ON)`
- [ ] Enable clock output: `sdio_clock_enable(TRUE)`
- [ ] Send CMD0, CMD8, ACMD41 for card identification
- [ ] Get CID, CSD, RCA
- [ ] Switch to higher clock speed (up to 25MHz SD, 50MHz SDHC)
- [ ] Enable 4-bit bus width

### DMA Transfer Setup

- [ ] Enable DMA clock
- [ ] Configure DMA channel for SDIO
- [ ] Enable SDIO DMA: `sdio_dma_enable(TRUE)`
- [ ] Configure DMAMUX for SDIO

### Interrupt Setup

- [ ] Configure NVIC for SDIO interrupt
- [ ] Enable desired interrupts: `sdio_interrupt_enable()`

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Card not detected | Clock too fast | Use ≤400kHz for identification |
| Command timeout | No card / bad connection | Check card insertion and wiring |
| Data CRC error | Signal integrity | Use stronger drive, shorter traces |
| DMA transfer fails | Buffer alignment | Align buffer to 4 bytes |
| 4-bit mode fails | Card doesn't support | Check SCR register for support |
| eMMC not working | Wrong initialization | Use MMC-specific commands |
| Transfer incomplete | Flow control needed | Enable hardware flow control |

---

## Performance Guidelines

| Mode | Typical Speed |
|------|---------------|
| 1-bit @ 25MHz | ~3 MB/s |
| 4-bit @ 25MHz | ~12 MB/s |
| 4-bit @ 50MHz | ~24 MB/s |
| 8-bit @ 50MHz (MMC) | ~48 MB/s |

**Note:** Actual performance depends on card quality, signal integrity, and DMA efficiency.

---

## Related Peripherals

- **[DMA](DMA_Direct_Memory_Access.md)** - DMA transfers for SDIO
- **[GPIO](GPIO_General_Purpose_IO.md)** - Pin configuration
- **[CRM](CRM_Clock_Reset_Management.md)** - Clock configuration

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-01 | Initial release |

