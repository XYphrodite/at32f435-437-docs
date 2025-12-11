---
title: SPI - Serial Peripheral Interface
category: Peripheral
complexity: Intermediate
mcu: AT32F435/437
peripheral: SPI
keywords: [spi, serial, master, slave, dma, interrupt, crc, ti, i2s, full-duplex, half-duplex]
---

# SPI - Serial Peripheral Interface

## Overview

The Serial Peripheral Interface (SPI) provides a synchronous serial communication interface for high-speed data transfer with external devices. Each SPI peripheral can operate as master or slave in full-duplex, simplex, or half-duplex modes, with support for hardware CRC calculation, TI mode, and DMA transfers. The SPI peripherals also share hardware with I2S interfaces for audio applications.

### Key Features

| Feature | Specification |
|---------|---------------|
| SPI Interfaces | 4 (SPI1, SPI2, SPI3, SPI4) |
| I2S Extension | I2S2EXT, I2S3EXT (for full-duplex I2S) |
| Operating Modes | Master / Slave |
| Transfer Modes | Full-duplex, Simplex RX-only, Half-duplex |
| Frame Size | 8-bit or 16-bit |
| Bit Order | MSB-first or LSB-first |
| Clock Polarity | Idle Low (CPOL=0) or Idle High (CPOL=1) |
| Clock Phase | 1st Edge (CPHA=0) or 2nd Edge (CPHA=1) |
| Clock Division | 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1024 |
| CS Mode | Hardware or Software |
| CRC | Hardware 16-bit CRC with programmable polynomial |
| TI Mode | TI synchronous serial interface supported |
| DMA | TX and RX DMA supported |

### SPI Mode Configuration (Clock Polarity & Phase)

| Mode | CPOL | CPHA | Clock Idle | Data Capture |
|------|------|------|------------|--------------|
| Mode 0 | 0 | 0 | Low | Rising edge |
| Mode 1 | 0 | 1 | Low | Falling edge |
| Mode 2 | 1 | 0 | High | Falling edge |
| Mode 3 | 1 | 1 | High | Rising edge |

## Architecture

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                      SPI Controller                          │
                    │                                                              │
┌─────────────┐     │  ┌─────────────────────────────────────────────────────┐    │
│  APB Bus    │─────│──│              Register Interface                     │    │
│             │     │  │  CTRL1 │ CTRL2 │ STS │ DT │ CPOLY │ RCRC │ TCRC    │    │
└─────────────┘     │  └─────────────────────────────────────────────────────┘    │
                    │                                                              │
                    │  ┌──────────────────┐    ┌─────────────────────────────┐    │
                    │  │   Control Logic  │    │       Shift Register         │    │
                    │  │                  │    │                              │    │
                    │  │ ┌──────────────┐ │    │  ┌──────┐    ┌───────────┐  │    │
                    │  │ │ Clock Gen    │ │    │  │ TX   │────│ 16-bit    │  │    │
                    │  │ │ MCLK/DIV     │ │    │  │Buffer│    │ Shift Reg │──│────│──▶ MOSI/MISO
                    │  │ └──────────────┘ │    │  └──────┘    └───────────┘  │    │
                    │  │                  │    │                    ▲        │    │
                    │  │ ┌──────────────┐ │    │  ┌──────┐          │        │    │
                    │  │ │ CRC Engine   │ │    │  │ RX   │◀─────────┘        │    │
                    │  │ │ Polynomial   │ │    │  │Buffer│                   │    │
                    │  │ └──────────────┘ │    │  └──────┘                   │    │
                    │  └──────────────────┘    └─────────────────────────────┘    │
                    │                                                              │
                    │        ┌───────┐    ┌───────┐    ┌───────┐    ┌───────┐    │
                    │        │  SCK  │    │ MOSI  │    │ MISO  │    │  CS   │    │
                    └────────┴───┬───┴────┴───┬───┴────┴───┬───┴────┴───┬───┴────┘
                                 │            │            │            │
                                 ▼            ▼            ▼            ▼
                    ┌─────────────────────────────────────────────────────────────┐
                    │                     External SPI Device                     │
                    └─────────────────────────────────────────────────────────────┘
```

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| CTRL1 | 0x00 | Control register 1 |
| CTRL2 | 0x04 | Control register 2 |
| STS | 0x08 | Status register |
| DT | 0x0C | Data register |
| CPOLY | 0x10 | CRC polynomial register |
| RCRC | 0x14 | RX CRC register |
| TCRC | 0x18 | TX CRC register |
| I2SCTRL | 0x1C | I2S control register |
| I2SCLK | 0x20 | I2S clock register |

### CTRL1 Register (0x00)

| Bits | Name | Description |
|------|------|-------------|
| 0 | CLKPHA | Clock phase: 0=1st edge, 1=2nd edge |
| 1 | CLKPOL | Clock polarity: 0=idle low, 1=idle high |
| 2 | MSTEN | Master enable: 0=slave, 1=master |
| 5:3 | MDIV_L | Master clock divider low bits |
| 6 | SPIEN | SPI enable |
| 7 | LTF | LSB transmit first |
| 8 | SWCSIL | Software CS internal level |
| 9 | SWCSEN | Software CS mode enable |
| 10 | ORA | Only receive active (simplex RX) |
| 11 | FBN | Frame bit number: 0=8-bit, 1=16-bit |
| 12 | NTC | Next transmit CRC |
| 13 | CCEN | CRC calculation enable |
| 14 | SLBTD | Single line bidirectional direction |
| 15 | SLBEN | Single line bidirectional enable |

### CTRL2 Register (0x04)

| Bits | Name | Description |
|------|------|-------------|
| 0 | DMAREN | DMA receive enable |
| 1 | DMATEN | DMA transmit enable |
| 2 | HWCSOE | Hardware CS output enable |
| 4 | TIEN | TI mode enable |
| 5 | ERRIE | Error interrupt enable |
| 6 | RDBFIE | RX data buffer full interrupt enable |
| 7 | TDBEIE | TX data buffer empty interrupt enable |
| 8 | MDIV_H | Master clock divider high bit |
| 9 | MDIV3EN | Master clock divide by 3 enable |

### STS Register (0x08)

| Bits | Name | Description |
|------|------|-------------|
| 0 | RDBF | Receive data buffer full |
| 1 | TDBE | Transmit data buffer empty |
| 2 | ACS | Audio channel state (I2S) |
| 3 | TUERR | Transmitter underload error (I2S) |
| 4 | CCERR | CRC calculation error |
| 5 | MMERR | Master mode error |
| 6 | ROERR | Receiver overflow error |
| 7 | BF | Busy flag |
| 8 | CSPAS | CS pulse abnormal setting |

## Configuration Types

### Transmission Mode

| Enum | Value | Description |
|------|-------|-------------|
| `SPI_TRANSMIT_FULL_DUPLEX` | 0x00 | Dual-line full-duplex |
| `SPI_TRANSMIT_SIMPLEX_RX` | 0x01 | Dual-line simplex receive-only |
| `SPI_TRANSMIT_HALF_DUPLEX_RX` | 0x02 | Single-line half-duplex receive |
| `SPI_TRANSMIT_HALF_DUPLEX_TX` | 0x03 | Single-line half-duplex transmit |

### Master/Slave Mode

| Enum | Value | Description |
|------|-------|-------------|
| `SPI_MODE_SLAVE` | 0x00 | Slave mode |
| `SPI_MODE_MASTER` | 0x01 | Master mode |

### Clock Division

| Enum | Value | Division |
|------|-------|----------|
| `SPI_MCLK_DIV_2` | 0x00 | ÷2 |
| `SPI_MCLK_DIV_3` | 0x0A | ÷3 |
| `SPI_MCLK_DIV_4` | 0x01 | ÷4 |
| `SPI_MCLK_DIV_8` | 0x02 | ÷8 |
| `SPI_MCLK_DIV_16` | 0x03 | ÷16 |
| `SPI_MCLK_DIV_32` | 0x04 | ÷32 |
| `SPI_MCLK_DIV_64` | 0x05 | ÷64 |
| `SPI_MCLK_DIV_128` | 0x06 | ÷128 |
| `SPI_MCLK_DIV_256` | 0x07 | ÷256 |
| `SPI_MCLK_DIV_512` | 0x08 | ÷512 |
| `SPI_MCLK_DIV_1024` | 0x09 | ÷1024 |

### Frame Bit Number

| Enum | Value | Description |
|------|-------|-------------|
| `SPI_FRAME_8BIT` | 0x00 | 8-bit frame |
| `SPI_FRAME_16BIT` | 0x01 | 16-bit frame |

### First Bit Transmission

| Enum | Value | Description |
|------|-------|-------------|
| `SPI_FIRST_BIT_MSB` | 0x00 | MSB first |
| `SPI_FIRST_BIT_LSB` | 0x01 | LSB first |

### Clock Polarity

| Enum | Value | Description |
|------|-------|-------------|
| `SPI_CLOCK_POLARITY_LOW` | 0x00 | Idle low |
| `SPI_CLOCK_POLARITY_HIGH` | 0x01 | Idle high |

### Clock Phase

| Enum | Value | Description |
|------|-------|-------------|
| `SPI_CLOCK_PHASE_1EDGE` | 0x00 | Capture on 1st clock edge |
| `SPI_CLOCK_PHASE_2EDGE` | 0x01 | Capture on 2nd clock edge |

### CS Mode

| Enum | Value | Description |
|------|-------|-------------|
| `SPI_CS_HARDWARE_MODE` | 0x00 | Hardware CS control |
| `SPI_CS_SOFTWARE_MODE` | 0x01 | Software CS control |

## Flags and Interrupts

### Status Flags

| Flag | Description |
|------|-------------|
| `SPI_I2S_RDBF_FLAG` | Receive data buffer full |
| `SPI_I2S_TDBE_FLAG` | Transmit data buffer empty |
| `I2S_ACS_FLAG` | Audio channel state (I2S only) |
| `I2S_TUERR_FLAG` | Transmitter underload error (I2S) |
| `SPI_CCERR_FLAG` | CRC calculation error |
| `SPI_MMERR_FLAG` | Master mode error |
| `SPI_I2S_ROERR_FLAG` | Receiver overflow error |
| `SPI_I2S_BF_FLAG` | Busy flag |
| `SPI_CSPAS_FLAG` | CS pulse abnormal setting |

### Interrupts

| Interrupt | Description |
|-----------|-------------|
| `SPI_I2S_ERROR_INT` | Error interrupt (ROERR, MMERR, CCERR, TUERR, CSPAS) |
| `SPI_I2S_RDBF_INT` | Receive data buffer full interrupt |
| `SPI_I2S_TDBE_INT` | Transmit data buffer empty interrupt |

## DMA Requests

| SPI | TX Request | RX Request |
|-----|------------|------------|
| SPI1 | DMAMUX_DMAREQ_ID_SPI1_TX | DMAMUX_DMAREQ_ID_SPI1_RX |
| SPI2 | DMAMUX_DMAREQ_ID_SPI2_TX | DMAMUX_DMAREQ_ID_SPI2_RX |
| SPI3 | DMAMUX_DMAREQ_ID_SPI3_TX | DMAMUX_DMAREQ_ID_SPI3_RX |
| SPI4 | DMAMUX_DMAREQ_ID_SPI4_TX | DMAMUX_DMAREQ_ID_SPI4_RX |

## SPI Init Structure

```c
typedef struct
{
    spi_transmission_mode_type   transmission_mode;      /* Full/Half-duplex/Simplex */
    spi_master_slave_mode_type   master_slave_mode;      /* Master or Slave */
    spi_mclk_freq_div_type       mclk_freq_division;     /* Clock division factor */
    spi_first_bit_type           first_bit_transmission; /* MSB or LSB first */
    spi_frame_bit_num_type       frame_bit_num;          /* 8-bit or 16-bit frame */
    spi_clock_polarity_type      clock_polarity;         /* Idle low or high */
    spi_clock_phase_type         clock_phase;            /* 1st or 2nd edge capture */
    spi_cs_mode_type             cs_mode_selection;      /* Hardware or Software CS */
} spi_init_type;
```

## API Reference

### Initialization

#### `spi_i2s_reset`

```c
void spi_i2s_reset(spi_type *spi_x);
```

Reset SPI peripheral via CRM reset register.

---

#### `spi_default_para_init`

```c
void spi_default_para_init(spi_init_type* spi_init_struct);
```

Initialize SPI init structure with default values.

**Default Values:**
- transmission_mode: `SPI_TRANSMIT_FULL_DUPLEX`
- master_slave_mode: `SPI_MODE_SLAVE`
- mclk_freq_division: `SPI_MCLK_DIV_2`
- first_bit_transmission: `SPI_FIRST_BIT_MSB`
- frame_bit_num: `SPI_FRAME_8BIT`
- clock_polarity: `SPI_CLOCK_POLARITY_LOW`
- clock_phase: `SPI_CLOCK_PHASE_1EDGE`
- cs_mode_selection: `SPI_CS_SOFTWARE_MODE`

---

#### `spi_init`

```c
void spi_init(spi_type* spi_x, spi_init_type* spi_init_struct);
```

Initialize SPI with specified parameters.

---

#### `spi_enable`

```c
void spi_enable(spi_type* spi_x, confirm_state new_state);
```

Enable or disable SPI.

---

### Configuration

#### `spi_ti_mode_enable`

```c
void spi_ti_mode_enable(spi_type* spi_x, confirm_state new_state);
```

Enable or disable TI synchronous serial interface mode.

**Note:** When TI mode is enabled, clock_polarity, clock_phase, and cs_mode_selection settings are ignored.

---

#### `spi_frame_bit_num_set`

```c
void spi_frame_bit_num_set(spi_type* spi_x, spi_frame_bit_num_type bit_num);
```

Set frame size (8-bit or 16-bit).

---

#### `spi_half_duplex_direction_set`

```c
void spi_half_duplex_direction_set(spi_type* spi_x, spi_half_duplex_direction_type direction);
```

Set half-duplex direction (TX or RX).

---

#### `spi_software_cs_internal_level_set`

```c
void spi_software_cs_internal_level_set(spi_type* spi_x, spi_software_cs_level_type level);
```

Set software CS internal level.

---

#### `spi_hardware_cs_output_enable`

```c
void spi_hardware_cs_output_enable(spi_type* spi_x, confirm_state new_state);
```

Enable or disable hardware CS output (master mode only).

---

### CRC Operations

#### `spi_crc_enable`

```c
void spi_crc_enable(spi_type* spi_x, confirm_state new_state);
```

Enable or disable hardware CRC calculation.

---

#### `spi_crc_polynomial_set`

```c
void spi_crc_polynomial_set(spi_type* spi_x, uint16_t crc_poly);
```

Set CRC polynomial value.

---

#### `spi_crc_polynomial_get`

```c
uint16_t spi_crc_polynomial_get(spi_type* spi_x);
```

Get CRC polynomial value.

---

#### `spi_crc_next_transmit`

```c
void spi_crc_next_transmit(spi_type* spi_x);
```

Set next transmit to be CRC value (call after last data byte).

---

#### `spi_crc_value_get`

```c
uint16_t spi_crc_value_get(spi_type* spi_x, spi_crc_direction_type crc_direction);
```

Get RX or TX CRC value.

---

### Data Operations

#### `spi_i2s_data_transmit`

```c
void spi_i2s_data_transmit(spi_type* spi_x, uint16_t tx_data);
```

Write data to transmit buffer.

---

#### `spi_i2s_data_receive`

```c
uint16_t spi_i2s_data_receive(spi_type* spi_x);
```

Read data from receive buffer.

---

### DMA and Interrupts

#### `spi_i2s_dma_transmitter_enable`

```c
void spi_i2s_dma_transmitter_enable(spi_type* spi_x, confirm_state new_state);
```

Enable or disable TX DMA.

---

#### `spi_i2s_dma_receiver_enable`

```c
void spi_i2s_dma_receiver_enable(spi_type* spi_x, confirm_state new_state);
```

Enable or disable RX DMA.

---

#### `spi_i2s_interrupt_enable`

```c
void spi_i2s_interrupt_enable(spi_type* spi_x, uint32_t spi_i2s_int, confirm_state new_state);
```

Enable or disable interrupts.

---

#### `spi_i2s_flag_get`

```c
flag_status spi_i2s_flag_get(spi_type* spi_x, uint32_t spi_i2s_flag);
```

Get flag status.

---

#### `spi_i2s_interrupt_flag_get`

```c
flag_status spi_i2s_interrupt_flag_get(spi_type* spi_x, uint32_t spi_i2s_flag);
```

Get interrupt flag status.

---

#### `spi_i2s_flag_clear`

```c
void spi_i2s_flag_clear(spi_type* spi_x, uint32_t spi_i2s_flag);
```

Clear flag.

---

## Code Examples

### Example 1: Full-Duplex Master/Slave Polling

Basic SPI communication between master and slave using polling.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define BUFFER_SIZE  32

uint8_t master_tx[BUFFER_SIZE] = {0x01, 0x02, 0x03, /* ... */};
uint8_t slave_tx[BUFFER_SIZE]  = {0x51, 0x52, 0x53, /* ... */};
uint8_t master_rx[BUFFER_SIZE], slave_rx[BUFFER_SIZE];

/* GPIO configuration */
void gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    
    /* Master CS (software controlled) */
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_4;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_bits_set(GPIOA, GPIO_PINS_4);  /* CS high initially */
    
    /* Master SCK, MISO, MOSI */
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
    gpio_init_struct.gpio_pins = GPIO_PINS_5;  /* SCK */
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE5, GPIO_MUX_5);
    
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_6;  /* MISO */
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE6, GPIO_MUX_5);
    
    gpio_init_struct.gpio_pins = GPIO_PINS_7;  /* MOSI */
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_5);
}

/* SPI configuration */
void spi_config(void)
{
    spi_init_type spi_init_struct;
    
    /* Master SPI1 */
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    
    spi_init(SPI1, &spi_init_struct);
    spi_enable(SPI1, TRUE);
}

/* Transfer one byte */
uint8_t spi_transfer_byte(spi_type *spi_x, uint8_t data)
{
    while(spi_i2s_flag_get(spi_x, SPI_I2S_TDBE_FLAG) == RESET);
    spi_i2s_data_transmit(spi_x, data);
    
    while(spi_i2s_flag_get(spi_x, SPI_I2S_RDBF_FLAG) == RESET);
    return (uint8_t)spi_i2s_data_receive(spi_x);
}

int main(void)
{
    uint32_t i;
    
    system_clock_config();
    at32_board_init();
    
    gpio_config();
    spi_config();
    
    /* Select slave (CS low) */
    gpio_bits_reset(GPIOA, GPIO_PINS_4);
    
    /* Transfer data */
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        master_rx[i] = spi_transfer_byte(SPI1, master_tx[i]);
    }
    
    /* Wait for SPI idle */
    while(spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) != RESET);
    
    /* Deselect slave (CS high) */
    gpio_bits_set(GPIOA, GPIO_PINS_4);
    
    while(1) { }
}
```

---

### Example 2: Full-Duplex with DMA

High-performance SPI transfer using DMA.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define BUFFER_SIZE  32

uint8_t master_tx[BUFFER_SIZE] = {0x01, 0x02, 0x03, /* ... */};
uint8_t master_rx[BUFFER_SIZE];
volatile uint8_t transfer_complete = 0;

/* DMA configuration */
void dma_config(void)
{
    dma_init_type dma_init_struct;
    
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    dmamux_enable(DMA1, TRUE);
    
    /* TX DMA Channel */
    dma_reset(DMA1_CHANNEL1);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = BUFFER_SIZE;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init_struct.memory_base_addr = (uint32_t)master_tx;
    dma_init_struct.peripheral_base_addr = (uint32_t)&(SPI1->dt);
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_SPI1_TX);
    
    /* RX DMA Channel */
    dma_reset(DMA1_CHANNEL2);
    dma_init_struct.memory_base_addr = (uint32_t)master_rx;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init(DMA1_CHANNEL2, &dma_init_struct);
    dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_SPI1_RX);
}

/* SPI configuration with DMA */
void spi_config(void)
{
    spi_init_type spi_init_struct;
    
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    
    spi_init(SPI1, &spi_init_struct);
    
    /* Enable SPI DMA */
    spi_i2s_dma_transmitter_enable(SPI1, TRUE);
    spi_i2s_dma_receiver_enable(SPI1, TRUE);
    
    spi_enable(SPI1, TRUE);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    gpio_config();  /* Configure SPI pins */
    dma_config();
    spi_config();
    
    /* Select slave */
    gpio_bits_reset(GPIOA, GPIO_PINS_4);
    
    /* Start DMA transfer */
    dma_channel_enable(DMA1_CHANNEL2, TRUE);  /* RX first */
    dma_channel_enable(DMA1_CHANNEL1, TRUE);  /* TX starts transfer */
    
    /* Wait for transfer complete */
    while(dma_flag_get(DMA1_FDT2_FLAG) == RESET);
    
    /* Wait for SPI idle */
    while(spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) != RESET);
    
    /* Deselect slave */
    gpio_bits_set(GPIOA, GPIO_PINS_4);
    
    while(1) { }
}
```

---

### Example 3: Half-Duplex with Interrupt

Single-wire communication using interrupts.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define BUFFER_SIZE  32

uint8_t tx_buffer[BUFFER_SIZE] = {0x01, 0x02, 0x03, /* ... */};
volatile uint32_t tx_index = 0;

/* SPI configuration for half-duplex TX */
void spi_config(void)
{
    spi_init_type spi_init_struct;
    
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_HALF_DUPLEX_TX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_16;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    
    spi_init(SPI1, &spi_init_struct);
    
    /* Enable TX buffer empty interrupt */
    spi_i2s_interrupt_enable(SPI1, SPI_I2S_TDBE_INT, TRUE);
}

/* SPI1 interrupt handler */
void SPI1_IRQHandler(void)
{
    if(spi_i2s_interrupt_flag_get(SPI1, SPI_I2S_TDBE_FLAG) != RESET)
    {
        spi_i2s_data_transmit(SPI1, tx_buffer[tx_index++]);
        
        if(tx_index >= BUFFER_SIZE)
        {
            /* Disable interrupt when done */
            spi_i2s_interrupt_enable(SPI1, SPI_I2S_TDBE_INT, FALSE);
        }
    }
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(SPI1_IRQn, 0, 0);
    
    gpio_config();
    spi_config();
    
    /* Select slave */
    gpio_bits_reset(GPIOA, GPIO_PINS_4);
    
    /* Enable SPI - interrupt will handle data transmission */
    spi_enable(SPI1, TRUE);
    
    /* Wait for transfer complete */
    while(tx_index < BUFFER_SIZE);
    
    /* Wait for SPI idle */
    while(spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) != RESET);
    
    /* Deselect slave */
    gpio_bits_set(GPIOA, GPIO_PINS_4);
    
    while(1) { }
}
```

---

### Example 4: Hardware CRC Transfer

SPI transfer with automatic CRC calculation and verification.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define BUFFER_SIZE  32
#define CRC_POLYNOMIAL  0x07

uint16_t master_tx[BUFFER_SIZE] = {0x0102, 0x0304, /* ... */};
uint16_t master_rx[BUFFER_SIZE];
uint16_t slave_tx[BUFFER_SIZE] = {0x5152, 0x5354, /* ... */};
uint16_t slave_rx[BUFFER_SIZE];

void spi_config(void)
{
    spi_init_type spi_init_struct;
    
    /* Master SPI3 */
    crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, TRUE);
    
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    
    spi_init(SPI3, &spi_init_struct);
    
    /* Configure CRC */
    spi_crc_polynomial_set(SPI3, CRC_POLYNOMIAL);
    spi_crc_enable(SPI3, TRUE);
    
    spi_enable(SPI3, TRUE);
    
    /* Slave SPI2 - similar configuration */
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
    spi_init_struct.master_slave_mode = SPI_MODE_SLAVE;
    spi_init_struct.cs_mode_selection = SPI_CS_HARDWARE_MODE;
    spi_init(SPI2, &spi_init_struct);
    spi_crc_polynomial_set(SPI2, CRC_POLYNOMIAL);
    spi_crc_enable(SPI2, TRUE);
    spi_enable(SPI2, TRUE);
}

int main(void)
{
    uint32_t i;
    error_status status = SUCCESS;
    
    system_clock_config();
    at32_board_init();
    
    gpio_config();
    spi_config();
    
    /* Select slave */
    gpio_bits_reset(GPIOA, GPIO_PINS_4);
    
    /* Transfer data (N-1 bytes) */
    for(i = 0; i < BUFFER_SIZE - 1; i++)
    {
        while(spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) == RESET);
        spi_i2s_data_transmit(SPI2, slave_tx[i]);
        while(spi_i2s_flag_get(SPI3, SPI_I2S_TDBE_FLAG) == RESET);
        spi_i2s_data_transmit(SPI3, master_tx[i]);
        
        while(spi_i2s_flag_get(SPI2, SPI_I2S_RDBF_FLAG) == RESET);
        slave_rx[i] = spi_i2s_data_receive(SPI2);
        while(spi_i2s_flag_get(SPI3, SPI_I2S_RDBF_FLAG) == RESET);
        master_rx[i] = spi_i2s_data_receive(SPI3);
    }
    
    /* Last byte + CRC */
    while(spi_i2s_flag_get(SPI3, SPI_I2S_TDBE_FLAG) == RESET);
    while(spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) == RESET);
    
    spi_i2s_data_transmit(SPI2, slave_tx[BUFFER_SIZE-1]);
    spi_crc_next_transmit(SPI2);  /* Next transmit is CRC */
    
    spi_i2s_data_transmit(SPI3, master_tx[BUFFER_SIZE-1]);
    spi_crc_next_transmit(SPI3);  /* Next transmit is CRC */
    
    /* Receive last data */
    while(spi_i2s_flag_get(SPI3, SPI_I2S_RDBF_FLAG) == RESET);
    master_rx[BUFFER_SIZE-1] = spi_i2s_data_receive(SPI3);
    while(spi_i2s_flag_get(SPI2, SPI_I2S_RDBF_FLAG) == RESET);
    slave_rx[BUFFER_SIZE-1] = spi_i2s_data_receive(SPI2);
    
    /* Receive CRC (automatically verified) */
    while(spi_i2s_flag_get(SPI3, SPI_I2S_RDBF_FLAG) == RESET);
    while(spi_i2s_flag_get(SPI2, SPI_I2S_RDBF_FLAG) == RESET);
    
    /* Wait idle */
    while(spi_i2s_flag_get(SPI3, SPI_I2S_BF_FLAG) != RESET);
    while(spi_i2s_flag_get(SPI2, SPI_I2S_BF_FLAG) != RESET);
    
    /* Deselect slave */
    gpio_bits_set(GPIOA, GPIO_PINS_4);
    
    /* Check CRC errors */
    if(spi_i2s_flag_get(SPI3, SPI_CCERR_FLAG) != RESET)
    {
        spi_i2s_flag_clear(SPI3, SPI_CCERR_FLAG);
        status = ERROR;
    }
    if(spi_i2s_flag_get(SPI2, SPI_CCERR_FLAG) != RESET)
    {
        spi_i2s_flag_clear(SPI2, SPI_CCERR_FLAG);
        status = ERROR;
    }
    
    if(status == SUCCESS)
        at32_led_on(LED2);
    else
        at32_led_on(LED3);
    
    while(1) { }
}
```

---

### Example 5: TI Mode with DMA

TI synchronous serial interface mode.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define BUFFER_SIZE  32

uint8_t master_tx[BUFFER_SIZE] = {0x01, 0x02, 0x03, /* ... */};
uint8_t master_rx[BUFFER_SIZE];

void spi_config_ti_mode(void)
{
    spi_init_type spi_init_struct;
    
    crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, TRUE);
    
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.cs_mode_selection = SPI_CS_HARDWARE_MODE;
    
    /* Note: clock_polarity and clock_phase are ignored in TI mode */
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    
    spi_init(SPI3, &spi_init_struct);
    
    /* Enable TI mode - overrides CPOL/CPHA/CS settings */
    spi_ti_mode_enable(SPI3, TRUE);
    
    /* Enable DMA */
    spi_i2s_dma_transmitter_enable(SPI3, TRUE);
    spi_i2s_dma_receiver_enable(SPI3, TRUE);
    
    spi_enable(SPI3, TRUE);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    gpio_config();  /* Configure pins including hardware CS */
    dma_config();   /* Configure TX/RX DMA channels */
    spi_config_ti_mode();
    
    /* Start DMA transfer */
    dma_channel_enable(DMA1_CHANNEL2, TRUE);  /* RX */
    dma_channel_enable(DMA1_CHANNEL1, TRUE);  /* TX */
    
    /* Wait for completion */
    while(dma_flag_get(DMA1_FDT2_FLAG) == RESET);
    while(spi_i2s_flag_get(SPI3, SPI_I2S_BF_FLAG) != RESET);
    
    while(1) { }
}
```

---

## Configuration Checklist

### Basic SPI Setup

- [ ] Enable GPIO clock
- [ ] Enable SPI peripheral clock: `crm_periph_clock_enable()`
- [ ] Configure GPIO pins (SCK, MISO, MOSI, CS)
- [ ] Set GPIO alternate function: `gpio_pin_mux_config()`
- [ ] Initialize SPI parameters: `spi_init()`
- [ ] Enable SPI: `spi_enable(TRUE)`

### Master Mode

- [ ] Set `master_slave_mode = SPI_MODE_MASTER`
- [ ] Configure clock division for desired speed
- [ ] For software CS: set `cs_mode_selection = SPI_CS_SOFTWARE_MODE`
- [ ] For software CS: set SWCSIL high when master

### Slave Mode

- [ ] Set `master_slave_mode = SPI_MODE_SLAVE`
- [ ] For hardware CS: set `cs_mode_selection = SPI_CS_HARDWARE_MODE`
- [ ] Enable before master starts communication

### DMA Setup

- [ ] Enable DMA clock
- [ ] Configure TX DMA channel (memory → peripheral)
- [ ] Configure RX DMA channel (peripheral → memory)
- [ ] Configure DMAMUX for SPI TX/RX requests
- [ ] Enable SPI DMA: `spi_i2s_dma_transmitter_enable()`, `spi_i2s_dma_receiver_enable()`

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| No clock output | SPI not enabled | Enable SPI after configuration |
| Data corruption | Clock/phase mismatch | Match CPOL/CPHA with slave device |
| Master mode error | Multi-master conflict | Ensure only one master on bus |
| Overflow error | Read buffer too slow | Use DMA or increase handling priority |
| CRC error | Data corruption | Check connections, reduce speed |
| Slave not responding | CS not controlled | Use hardware CS or manage software CS |
| TI mode not working | Wrong GPIO config | Use hardware CS mode with TI |

---

## Performance Guidelines

| Configuration | Typical Max Speed |
|---------------|-------------------|
| SPI1/SPI4 (APB2) | PCLK2/2 (up to 144 MHz at 288 MHz) |
| SPI2/SPI3 (APB1) | PCLK1/2 (up to 72 MHz at 144 MHz) |

**Note:** Actual maximum speed depends on signal integrity, trace length, and slave device capabilities.

---

## Related Peripherals

- **[DMA](DMA_Direct_Memory_Access.md)** - DMA transfers for SPI
- **[GPIO](GPIO_General_Purpose_IO.md)** - Pin configuration
- **[I2S](I2S_Inter_IC_Sound.md)** - Audio mode on SPI peripherals
- **[CRM](CRM_Clock_Reset_Management.md)** - Clock configuration

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-01 | Initial release |

