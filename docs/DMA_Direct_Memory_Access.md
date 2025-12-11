---
title: "DMA - Direct Memory Access Controller"
type: "peripheral-documentation"
purpose: "context7-knowledge-source"
mcu_family: "AT32F435/437"
peripheral: "DMA"
version: "1.0.0"
last_updated: "2024-12-11"
tags:
  - dma
  - dmamux
  - memory-transfer
  - peripheral-transfer
  - circular-mode
  - interrupt
  - synchronization
  - request-generator
  - context7
related_peripherals:
  - ADC
  - DAC
  - USART
  - SPI
  - I2C
  - TMR
  - SDIO
  - QSPI
---

# DMA - Direct Memory Access Controller

## Overview

The **Direct Memory Access (DMA)** controller in AT32F435/437 MCUs enables high-speed data transfers between memory and peripherals without CPU intervention, significantly improving system performance and efficiency.

**Key Features:**
- 🔄 **2 DMA controllers** (DMA1, DMA2) with 7 channels each (14 total)
- ⚡ **High-speed transfers** without CPU overhead
- 🎯 **DMAMUX** - Flexible request multiplexer for any peripheral to any channel
- 📈 **Multiple transfer modes** (Memory-to-Memory, Peripheral-to-Memory, Memory-to-Peripheral)
- 🔁 **Circular mode** for continuous data streaming
- 📊 **Data width options** (8-bit, 16-bit, 32-bit)
- 🔢 **4 priority levels** per channel
- 🔔 **3 interrupt types** (Full transfer, Half transfer, Error)
- 🔗 **Synchronization** support with external signals
- ⚙️ **Request generator** for event-triggered DMA

---

## Architecture

```
                              ┌─────────────────────────────────────────────────────────────┐
                              │                     DMA Controller                          │
  Peripheral                  │                                                             │
  DMA Requests                │  ┌─────────────┐    ┌─────────────────────────────────┐   │
  ─────────────               │  │   DMAMUX    │    │        DMA Channels             │   │
  ADC1 ──────────────────────►│  │             │    │  ┌─────────────────────────┐   │   │
  DAC1/2 ────────────────────►│  │  Request    │───►│  │ Channel 1               │   │   │──► Memory
  SPI1-4 ────────────────────►│  │  Selector   │    │  │ - Priority              │   │   │
  I2C1-3 ────────────────────►│  │             │    │  │ - Direction             │   │   │──► Peripheral
  USART1-8 ──────────────────►│  │  (per DMA)  │    │  │ - Data Width            │   │   │
  TMR1-8,20 ─────────────────►│  │             │    │  │ - Address Increment     │   │   │
  SDIO1/2 ───────────────────►│  │  Sync &     │    │  │ - Loop Mode             │   │   │
  QSPI1/2 ───────────────────►│  │  Generator  │    │  └─────────────────────────┘   │   │
  DVP ───────────────────────►│  └─────────────┘    │           ...                   │   │
                              │                      │  ┌─────────────────────────┐   │   │
  External Sync               │  ┌─────────────┐    │  │ Channel 7               │   │   │
  ─────────────               │  │ Sync/Gen    │───►│  │ (same structure)        │   │   │
  EXINT0-15 ─────────────────►│  │ Control     │    │  └─────────────────────────┘   │   │
  DMAMUX Events ─────────────►│  └─────────────┘    └─────────────────────────────────┘   │
                              │                                                             │
                              │  Interrupts: FDT (Full), HDT (Half), DTERR (Error)         │
                              └─────────────────────────────────────────────────────────────┘
```

---

## Hardware Resources

### DMA Controllers

| Controller | Base Address | Channels | DMAMUX Base |
|------------|--------------|----------|-------------|
| DMA1 | 0x40020000 | 7 (DMA1_CHANNEL1-7) | 0x40020104 |
| DMA2 | 0x40020400 | 7 (DMA2_CHANNEL1-7) | 0x40020504 |

### Channel Resources per DMA

| Channel | IRQ Vector | DMAMUX Channel | Generator |
|---------|------------|----------------|-----------|
| Channel 1 | DMA1/2_Channel1_IRQn | DMA1/2MUX_CHANNEL1 | - |
| Channel 2 | DMA1/2_Channel2_IRQn | DMA1/2MUX_CHANNEL2 | - |
| Channel 3 | DMA1/2_Channel3_IRQn | DMA1/2MUX_CHANNEL3 | - |
| Channel 4 | DMA1/2_Channel4_IRQn | DMA1/2MUX_CHANNEL4 | - |
| Channel 5 | DMA1/2_Channel5_IRQn | DMA1/2MUX_CHANNEL5 | - |
| Channel 6 | DMA1/2_Channel6_IRQn | DMA1/2MUX_CHANNEL6 | - |
| Channel 7 | DMA1/2_Channel7_IRQn | DMA1/2MUX_CHANNEL7 | - |
| Generator 1-4 | - | - | DMA1/2MUX_GENERATOR1-4 |

---

## Transfer Directions

| Direction | Enum | Description |
|-----------|------|-------------|
| Peripheral → Memory | `DMA_DIR_PERIPHERAL_TO_MEMORY` | Read from peripheral, write to memory |
| Memory → Peripheral | `DMA_DIR_MEMORY_TO_PERIPHERAL` | Read from memory, write to peripheral |
| Memory → Memory | `DMA_DIR_MEMORY_TO_MEMORY` | Copy between memory locations (peripheral_addr = source, memory_addr = destination) |

---

## Data Width Options

### Peripheral Data Width

| Width | Enum | Bytes per Transfer |
|-------|------|-------------------|
| 8-bit | `DMA_PERIPHERAL_DATA_WIDTH_BYTE` | 1 |
| 16-bit | `DMA_PERIPHERAL_DATA_WIDTH_HALFWORD` | 2 |
| 32-bit | `DMA_PERIPHERAL_DATA_WIDTH_WORD` | 4 |

### Memory Data Width

| Width | Enum | Bytes per Transfer |
|-------|------|-------------------|
| 8-bit | `DMA_MEMORY_DATA_WIDTH_BYTE` | 1 |
| 16-bit | `DMA_MEMORY_DATA_WIDTH_HALFWORD` | 2 |
| 32-bit | `DMA_MEMORY_DATA_WIDTH_WORD` | 4 |

**Note:** Peripheral and memory widths can differ. DMA handles packing/unpacking automatically.

---

## Priority Levels

| Priority | Enum | Arbitration Order |
|----------|------|-------------------|
| Low | `DMA_PRIORITY_LOW` | Lowest |
| Medium | `DMA_PRIORITY_MEDIUM` | Medium-Low |
| High | `DMA_PRIORITY_HIGH` | Medium-High |
| Very High | `DMA_PRIORITY_VERY_HIGH` | Highest |

**Arbitration:** When multiple channels request simultaneously, priority level determines order. Same priority channels use channel number (lower = higher priority).

---

## DMAMUX - Request Multiplexer

DMAMUX allows any peripheral DMA request to be routed to any DMA channel, providing maximum flexibility.

### DMAMUX Request IDs

#### Analog Peripherals
| Request ID | Peripheral |
|------------|------------|
| `DMAMUX_DMAREQ_ID_ADC1` | ADC1 |
| `DMAMUX_DMAREQ_ID_ADC2` | ADC2 |
| `DMAMUX_DMAREQ_ID_ADC3` | ADC3 |
| `DMAMUX_DMAREQ_ID_DAC1` | DAC1 |
| `DMAMUX_DMAREQ_ID_DAC2` | DAC2 |

#### Communication Peripherals
| Request ID | Peripheral |
|------------|------------|
| `DMAMUX_DMAREQ_ID_SPI1_RX/TX` | SPI1 RX/TX |
| `DMAMUX_DMAREQ_ID_SPI2_RX/TX` | SPI2 RX/TX |
| `DMAMUX_DMAREQ_ID_SPI3_RX/TX` | SPI3 RX/TX |
| `DMAMUX_DMAREQ_ID_SPI4_RX/TX` | SPI4 RX/TX |
| `DMAMUX_DMAREQ_ID_I2C1_RX/TX` | I2C1 RX/TX |
| `DMAMUX_DMAREQ_ID_I2C2_RX/TX` | I2C2 RX/TX |
| `DMAMUX_DMAREQ_ID_I2C3_RX/TX` | I2C3 RX/TX |
| `DMAMUX_DMAREQ_ID_USART1_RX/TX` | USART1 RX/TX |
| `DMAMUX_DMAREQ_ID_USART2_RX/TX` | USART2 RX/TX |
| `DMAMUX_DMAREQ_ID_USART3_RX/TX` | USART3 RX/TX |
| `DMAMUX_DMAREQ_ID_UART4_RX/TX` | UART4 RX/TX |
| `DMAMUX_DMAREQ_ID_UART5_RX/TX` | UART5 RX/TX |
| `DMAMUX_DMAREQ_ID_USART6_RX/TX` | USART6 RX/TX |
| `DMAMUX_DMAREQ_ID_UART7_RX/TX` | UART7 RX/TX |
| `DMAMUX_DMAREQ_ID_UART8_RX/TX` | UART8 RX/TX |

#### Timer Peripherals
| Request ID | Peripheral |
|------------|------------|
| `DMAMUX_DMAREQ_ID_TMR1_CH1/2/3/4` | TMR1 Channels |
| `DMAMUX_DMAREQ_ID_TMR1_OVERFLOW` | TMR1 Overflow |
| `DMAMUX_DMAREQ_ID_TMR1_TRIG` | TMR1 Trigger |
| `DMAMUX_DMAREQ_ID_TMR1_HALL` | TMR1 Hall |
| `DMAMUX_DMAREQ_ID_TMR2_CH1/2/3/4` | TMR2 Channels |
| `DMAMUX_DMAREQ_ID_TMR2_OVERFLOW` | TMR2 Overflow |
| `DMAMUX_DMAREQ_ID_TMR3_CH1/2/3/4` | TMR3 Channels |
| `DMAMUX_DMAREQ_ID_TMR3_OVERFLOW` | TMR3 Overflow |
| `DMAMUX_DMAREQ_ID_TMR4_CH1/2/3/4` | TMR4 Channels |
| `DMAMUX_DMAREQ_ID_TMR4_OVERFLOW` | TMR4 Overflow |
| `DMAMUX_DMAREQ_ID_TMR5_CH1/2/3/4` | TMR5 Channels |
| `DMAMUX_DMAREQ_ID_TMR5_OVERFLOW` | TMR5 Overflow |
| `DMAMUX_DMAREQ_ID_TMR6_OVERFLOW` | TMR6 Overflow |
| `DMAMUX_DMAREQ_ID_TMR7_OVERFLOW` | TMR7 Overflow |
| `DMAMUX_DMAREQ_ID_TMR8_CH1/2/3/4` | TMR8 Channels |
| `DMAMUX_DMAREQ_ID_TMR8_OVERFLOW` | TMR8 Overflow |
| `DMAMUX_DMAREQ_ID_TMR20_CH1/2/3/4` | TMR20 Channels |
| `DMAMUX_DMAREQ_ID_TMR20_OVERFLOW` | TMR20 Overflow |

#### Storage & Special Peripherals
| Request ID | Peripheral |
|------------|------------|
| `DMAMUX_DMAREQ_ID_SDIO1` | SDIO1 |
| `DMAMUX_DMAREQ_ID_SDIO2` | SDIO2 |
| `DMAMUX_DMAREQ_ID_QSPI1` | QSPI1 |
| `DMAMUX_DMAREQ_ID_QSPI2` | QSPI2 |
| `DMAMUX_DMAREQ_ID_DVP` | Digital Video Port |

#### Request Generators
| Request ID | Source |
|------------|--------|
| `DMAMUX_DMAREQ_ID_REQ_G1` | Generator 1 |
| `DMAMUX_DMAREQ_ID_REQ_G2` | Generator 2 |
| `DMAMUX_DMAREQ_ID_REQ_G3` | Generator 3 |
| `DMAMUX_DMAREQ_ID_REQ_G4` | Generator 4 |

---

## DMAMUX Synchronization

Synchronization allows DMA requests to be gated by external signals, enabling precise timing control.

### Synchronization Sources

| Sync ID | Source |
|---------|--------|
| `DMAMUX_SYNC_ID_EXINT0` - `DMAMUX_SYNC_ID_EXINT15` | External interrupt lines 0-15 |
| `DMAMUX_SYNC_ID_DMAMUX_CH1_EVT` - `DMAMUX_SYNC_ID_DMAMUX_CH7_EVT` | DMAMUX channel events |

### Synchronization Polarity

| Polarity | Enum | Description |
|----------|------|-------------|
| Disabled | `DMAMUX_SYNC_POLARITY_DISABLE` | No synchronization |
| Rising | `DMAMUX_SYNC_POLARITY_RISING` | Sync on rising edge |
| Falling | `DMAMUX_SYNC_POLARITY_FALLING` | Sync on falling edge |
| Both | `DMAMUX_SYNC_POLARITY_RISING_FALLING` | Sync on both edges |

---

## DMAMUX Request Generator

The request generator creates DMA requests from external events (EXINT lines, DMAMUX events).

### Generator Trigger Sources

| Gen ID | Source |
|--------|--------|
| `DMAMUX_GEN_ID_EXINT0` - `DMAMUX_GEN_ID_EXINT15` | External interrupt lines 0-15 |
| `DMAMUX_GEN_ID_DMAMUX_CH1_EVT` - `DMAMUX_GEN_ID_DMAMUX_CH7_EVT` | DMAMUX channel events |

### Generator Polarity

| Polarity | Enum | Description |
|----------|------|-------------|
| Disabled | `DMAMUX_GEN_POLARITY_DISABLE` | Generator disabled |
| Rising | `DMAMUX_GEN_POLARITY_RISING` | Generate on rising edge |
| Falling | `DMAMUX_GEN_POLARITY_FALLING` | Generate on falling edge |
| Both | `DMAMUX_GEN_POLARITY_RISING_FALLING` | Generate on both edges |

---

## Interrupts and Flags

### DMA Interrupts

| Interrupt | Macro | Description |
|-----------|-------|-------------|
| Full Transfer | `DMA_FDT_INT` | All data transferred |
| Half Transfer | `DMA_HDT_INT` | Half data transferred |
| Transfer Error | `DMA_DTERR_INT` | Bus error during transfer |

### DMA Flags (per channel)

| Flag Type | DMA1 Example | DMA2 Example |
|-----------|--------------|--------------|
| Global | `DMA1_GL1_FLAG` | `DMA2_GL1_FLAG` |
| Full Transfer | `DMA1_FDT1_FLAG` | `DMA2_FDT1_FLAG` |
| Half Transfer | `DMA1_HDT1_FLAG` | `DMA2_HDT1_FLAG` |
| Error | `DMA1_DTERR1_FLAG` | `DMA2_DTERR1_FLAG` |

### DMAMUX Flags

| Flag | Description |
|------|-------------|
| `DMAMUX_SYNC_OV1_FLAG` - `DMAMUX_SYNC_OV7_FLAG` | Synchronization overrun |
| `DMAMUX_GEN_TRIG_OV1_FLAG` - `DMAMUX_GEN_TRIG_OV4_FLAG` | Generator trigger overrun |

---

## API Reference

### DMA Initialization

```c
/**
 * @brief  Reset DMA channel to default state
 * @param  dmax_channely: DMA1_CHANNEL1-7 or DMA2_CHANNEL1-7
 */
void dma_reset(dma_channel_type *dmax_channely);

/**
 * @brief  Initialize dma_init_type with default values
 * @param  dma_init_struct: Pointer to init structure
 */
void dma_default_para_init(dma_init_type *dma_init_struct);

/**
 * @brief  Initialize DMA channel
 * @param  dmax_channely: DMA channel to configure
 * @param  dma_init_struct: Configuration parameters
 */
void dma_init(dma_channel_type *dmax_channely, dma_init_type *dma_init_struct);
```

### DMA Control

```c
/**
 * @brief  Enable or disable DMA channel
 * @param  dmax_channely: DMA channel
 * @param  new_state: TRUE to enable, FALSE to disable
 */
void dma_channel_enable(dma_channel_type *dmax_channely, confirm_state new_state);

/**
 * @brief  Set number of data items to transfer
 * @param  dmax_channely: DMA channel
 * @param  data_number: Number of items (0-65535)
 */
void dma_data_number_set(dma_channel_type *dmax_channely, uint16_t data_number);

/**
 * @brief  Get remaining data items to transfer
 * @param  dmax_channely: DMA channel
 * @return Remaining items count
 */
uint16_t dma_data_number_get(dma_channel_type *dmax_channely);

/**
 * @brief  Enable or disable DMA interrupts
 * @param  dmax_channely: DMA channel
 * @param  dma_int: DMA_FDT_INT, DMA_HDT_INT, DMA_DTERR_INT
 * @param  new_state: TRUE to enable, FALSE to disable
 */
void dma_interrupt_enable(dma_channel_type *dmax_channely, uint32_t dma_int, confirm_state new_state);
```

### DMA Flags

```c
/**
 * @brief  Get DMA flag status
 * @param  dmax_flag: Flag to check (e.g., DMA1_FDT1_FLAG)
 * @return SET or RESET
 */
flag_status dma_flag_get(uint32_t dmax_flag);

/**
 * @brief  Clear DMA flag
 * @param  dmax_flag: Flag to clear
 */
void dma_flag_clear(uint32_t dmax_flag);

/**
 * @brief  Get DMA interrupt flag status
 * @param  dmax_flag: Interrupt flag to check
 * @return SET or RESET
 */
flag_status dma_interrupt_flag_get(uint32_t dmax_flag);
```

### DMAMUX Functions

```c
/**
 * @brief  Enable or disable DMAMUX
 * @param  dma_x: DMA1 or DMA2
 * @param  new_state: TRUE to enable, FALSE to disable
 */
void dmamux_enable(dma_type *dma_x, confirm_state new_state);

/**
 * @brief  Configure DMAMUX channel request source
 * @param  dmamux_channelx: DMAMUX channel
 * @param  dmamux_req_sel: Request source ID
 */
void dmamux_init(dmamux_channel_type *dmamux_channelx, dmamux_requst_id_sel_type dmamux_req_sel);

/**
 * @brief  Configure DMAMUX synchronization
 * @param  dmamux_channelx: DMAMUX channel
 * @param  dmamux_sync_init_struct: Sync configuration
 */
void dmamux_sync_config(dmamux_channel_type *dmamux_channelx, dmamux_sync_init_type *dmamux_sync_init_struct);

/**
 * @brief  Configure DMAMUX request generator
 * @param  dmamux_gen_x: Generator (DMA1/2MUX_GENERATOR1-4)
 * @param  dmamux_gen_init_struct: Generator configuration
 */
void dmamux_generator_config(dmamux_generator_type *dmamux_gen_x, dmamux_gen_init_type *dmamux_gen_init_struct);
```

---

## Code Examples

### Example 1: Memory-to-Memory Transfer (Flash to SRAM)

Basic DMA transfer copying data from Flash to SRAM.

```c
#include "at32f435_437.h"

#define BUFFER_SIZE  32

/* Source data in Flash (const) */
const uint32_t src_buffer[BUFFER_SIZE] = {
    0x01020304, 0x05060708, 0x090A0B0C, 0x0D0E0F10,
    0x11121314, 0x15161718, 0x191A1B1C, 0x1D1E1F20,
    0x21222324, 0x25262728, 0x292A2B2C, 0x2D2E2F30,
    0x31323334, 0x35363738, 0x393A3B3C, 0x3D3E3F40,
    0x41424344, 0x45464748, 0x494A4B4C, 0x4D4E4F50,
    0x51525354, 0x55565758, 0x595A5B5C, 0x5D5E5F60,
    0x61626364, 0x65666768, 0x696A6B6C, 0x6D6E6F70,
    0x71727374, 0x75767778, 0x797A7B7C, 0x7D7E7F80
};

/* Destination buffer in SRAM */
uint32_t dst_buffer[BUFFER_SIZE];

volatile uint8_t transfer_complete = 0;

void DMA1_Channel1_IRQHandler(void)
{
    if (dma_flag_get(DMA1_FDT1_FLAG) != RESET)
    {
        transfer_complete = 1;
        dma_flag_clear(DMA1_FDT1_FLAG);
    }
}

void dma_memory_to_memory_init(void)
{
    dma_init_type dma_init_struct;
    
    /* Enable DMA1 clock */
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    
    /* Reset and configure DMA channel */
    dma_reset(DMA1_CHANNEL1);
    
    dma_init_struct.buffer_size = BUFFER_SIZE;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)dst_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)src_buffer;  /* Source for M2M */
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
    dma_init_struct.peripheral_inc_enable = TRUE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    
    /* Enable transfer complete interrupt */
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
    
    /* Configure NVIC */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(DMA1_Channel1_IRQn, 1, 0);
    
    /* Start transfer */
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    
    /* Wait for completion */
    while (!transfer_complete);
}
```

### Example 2: Timer-Triggered DMA to GPIO

Output data pattern to GPIO using timer-triggered DMA with DMAMUX.

```c
#include "at32f435_437.h"

#define BUFFER_SIZE  16

uint16_t gpio_pattern[BUFFER_SIZE] = {
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0010, 0x0020, 0x0040, 0x0080,
    0x0100, 0x0200, 0x0400, 0x0800,
    0x1000, 0x2000, 0x4000, 0x8000
};

volatile uint8_t transfer_complete = 0;

void DMA2_Channel1_IRQHandler(void)
{
    if (dma_flag_get(DMA2_FDT1_FLAG) != RESET)
    {
        transfer_complete = 1;
        dma_flag_clear(DMA2_FDT1_FLAG);
    }
}

void dma_gpio_pattern_init(void)
{
    gpio_init_type gpio_init_struct = {0};
    dma_init_type dma_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    
    /* Configure GPIOC as output */
    gpio_init_struct.gpio_pins = GPIO_PINS_ALL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOC, &gpio_init_struct);
    
    /* Configure Timer 2 */
    tmr_base_init(TMR2, 0xFF, 0);
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, TRUE);
    
    /* Configure DMA */
    dma_reset(DMA2_CHANNEL1);
    dma_init_struct.buffer_size = BUFFER_SIZE;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)gpio_pattern;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&GPIOC->odt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = FALSE;  /* Set TRUE for continuous output */
    dma_init(DMA2_CHANNEL1, &dma_init_struct);
    
    /* Enable interrupt */
    dma_interrupt_enable(DMA2_CHANNEL1, DMA_FDT_INT, TRUE);
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(DMA2_Channel1_IRQn, 1, 0);
    
    /* Enable DMAMUX and configure request source */
    dmamux_enable(DMA2, TRUE);
    dmamux_init(DMA2MUX_CHANNEL1, DMAMUX_DMAREQ_ID_TMR2_OVERFLOW);
    
    /* Enable DMA channel */
    dma_channel_enable(DMA2_CHANNEL1, TRUE);
    
    /* Start timer */
    tmr_counter_enable(TMR2, TRUE);
}
```

### Example 3: DMAMUX Synchronization with External Interrupt

DMA transfer synchronized with external signal (button press).

```c
#include "at32f435_437.h"

#define BUFFER_SIZE  16

uint16_t src_buffer[BUFFER_SIZE] = {
    0x0001, 0x0002, 0x0003, 0x0004,
    0x0005, 0x0006, 0x0007, 0x0008,
    0x0009, 0x000A, 0x000B, 0x000C,
    0x000D, 0x000E, 0x000F, 0x0010
};
uint16_t dst_buffer[BUFFER_SIZE];

void EXINT1_IRQHandler(void)
{
    if (exint_interrupt_flag_get(EXINT_LINE_1) != RESET)
    {
        /* LED toggle on each sync event */
        at32_led_toggle(LED2);
        exint_flag_clear(EXINT_LINE_1);
    }
}

void DMA2_Channel4_IRQHandler(void)
{
    if (dma_flag_get(DMA2_FDT4_FLAG) != RESET)
    {
        /* Transfer complete */
        at32_led_on(LED3);
        dma_flag_clear(DMA2_FDT4_FLAG);
    }
}

void dma_sync_init(void)
{
    gpio_init_type gpio_init_struct = {0};
    dma_init_type dma_init_struct;
    dmamux_sync_init_type dmamux_sync_init_struct;
    exint_init_type exint_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    
    /* Configure PA1 as external interrupt input */
    gpio_init_struct.gpio_pins = GPIO_PINS_1;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Configure EXINT */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE1);
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_1;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);
    
    nvic_irq_enable(EXINT1_IRQn, 1, 0);
    
    /* Configure DMA */
    dma_reset(DMA2_CHANNEL4);
    dma_init_struct.buffer_size = BUFFER_SIZE;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)dst_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)src_buffer;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = TRUE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA2_CHANNEL4, &dma_init_struct);
    
    /* Configure DMAMUX synchronization */
    dmamux_sync_default_para_init(&dmamux_sync_init_struct);
    dmamux_sync_init_struct.sync_request_number = 4;  /* 4 requests per sync event */
    dmamux_sync_init_struct.sync_signal_sel = DMAMUX_SYNC_ID_EXINT1;
    dmamux_sync_init_struct.sync_polarity = DMAMUX_SYNC_POLARITY_RISING;
    dmamux_sync_init_struct.sync_event_enable = TRUE;
    dmamux_sync_init_struct.sync_enable = TRUE;
    dmamux_sync_config(DMA2MUX_CHANNEL4, &dmamux_sync_init_struct);
    
    /* Enable DMA interrupt */
    dma_interrupt_enable(DMA2_CHANNEL4, DMA_FDT_INT, TRUE);
    nvic_irq_enable(DMA2_Channel4_IRQn, 1, 0);
    
    /* Enable DMAMUX with timer overflow as base request */
    dmamux_enable(DMA2, TRUE);
    dmamux_init(DMA2MUX_CHANNEL4, DMAMUX_DMAREQ_ID_TMR1_OVERFLOW);
    
    /* Configure Timer 1 */
    tmr_base_init(TMR1, 5000, 0);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
    tmr_dma_request_enable(TMR1, TMR_OVERFLOW_DMA_REQUEST, TRUE);
    
    /* Enable DMA and Timer */
    dma_channel_enable(DMA2_CHANNEL4, TRUE);
    tmr_counter_enable(TMR1, TRUE);
    
    /* Now: Every rising edge on PA1 allows 4 timer overflow requests through */
}
```

### Example 4: DMAMUX Request Generator

Generate DMA requests from external interrupt events.

```c
#include "at32f435_437.h"

#define BUFFER_SIZE  16

uint16_t src_buffer[BUFFER_SIZE] = {
    0x0001, 0x0002, 0x0003, 0x0004,
    0x0005, 0x0006, 0x0007, 0x0008,
    0x0009, 0x000A, 0x000B, 0x000C,
    0x000D, 0x000E, 0x000F, 0x0010
};
uint16_t dst_buffer[BUFFER_SIZE];

void dma_generator_init(void)
{
    gpio_init_type gpio_init_struct = {0};
    dma_init_type dma_init_struct;
    dmamux_gen_init_type dmamux_gen_init_struct;
    exint_init_type exint_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA1 as input */
    gpio_init_struct.gpio_pins = GPIO_PINS_1;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Configure EXINT for generator trigger */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE1);
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_1;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);
    
    nvic_irq_enable(EXINT1_IRQn, 1, 0);
    
    /* Configure DMA */
    dma_reset(DMA2_CHANNEL4);
    dma_init_struct.buffer_size = BUFFER_SIZE;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)dst_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)src_buffer;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = TRUE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA2_CHANNEL4, &dma_init_struct);
    
    /* Configure Request Generator 1 */
    dmamux_generator_default_para_init(&dmamux_gen_init_struct);
    dmamux_gen_init_struct.gen_polarity = DMAMUX_GEN_POLARITY_RISING;
    dmamux_gen_init_struct.gen_request_number = 4;  /* Generate 4 DMA requests per trigger */
    dmamux_gen_init_struct.gen_signal_sel = DMAMUX_GEN_ID_EXINT1;
    dmamux_gen_init_struct.gen_enable = TRUE;
    dmamux_generator_config(DMA2MUX_GENERATOR1, &dmamux_gen_init_struct);
    
    /* Enable DMA interrupt */
    dma_interrupt_enable(DMA2_CHANNEL4, DMA_FDT_INT, TRUE);
    nvic_irq_enable(DMA2_Channel4_IRQn, 1, 0);
    
    /* Enable DMAMUX and connect to Generator 1 */
    dmamux_enable(DMA2, TRUE);
    dmamux_init(DMA2MUX_CHANNEL4, DMAMUX_DMAREQ_ID_REQ_G1);
    
    /* Enable DMA channel */
    dma_channel_enable(DMA2_CHANNEL4, TRUE);
    
    /* Now: Every rising edge on PA1 generates 4 DMA transfer requests */
}
```

### Example 5: Circular Mode for Continuous ADC Sampling

```c
#include "at32f435_437.h"

#define ADC_BUFFER_SIZE  64

uint16_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint8_t buffer_half_full = 0;
volatile uint8_t buffer_full = 0;

void DMA1_Channel1_IRQHandler(void)
{
    if (dma_flag_get(DMA1_HDT1_FLAG) != RESET)
    {
        /* First half complete - process adc_buffer[0..31] */
        buffer_half_full = 1;
        dma_flag_clear(DMA1_HDT1_FLAG);
    }
    
    if (dma_flag_get(DMA1_FDT1_FLAG) != RESET)
    {
        /* Second half complete - process adc_buffer[32..63] */
        buffer_full = 1;
        dma_flag_clear(DMA1_FDT1_FLAG);
    }
}

void dma_adc_circular_init(void)
{
    dma_init_type dma_init_struct;
    
    /* Enable DMA1 clock */
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    
    /* Configure DMA for ADC */
    dma_reset(DMA1_CHANNEL1);
    dma_init_struct.buffer_size = ADC_BUFFER_SIZE;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)adc_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&ADC1->odt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.loop_mode_enable = TRUE;  /* Circular mode! */
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    
    /* Enable both half and full transfer interrupts for double-buffering */
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT | DMA_HDT_INT, TRUE);
    
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0);  /* High priority for ADC */
    
    /* Enable DMAMUX for ADC1 */
    dmamux_enable(DMA1, TRUE);
    dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_ADC1);
    
    /* Enable DMA channel */
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    
    /* Now configure and enable ADC with DMA... */
}

void process_adc_data(void)
{
    if (buffer_half_full)
    {
        buffer_half_full = 0;
        /* Process adc_buffer[0] to adc_buffer[ADC_BUFFER_SIZE/2 - 1] */
    }
    
    if (buffer_full)
    {
        buffer_full = 0;
        /* Process adc_buffer[ADC_BUFFER_SIZE/2] to adc_buffer[ADC_BUFFER_SIZE - 1] */
    }
}
```

---

## Configuration Checklist

### Basic DMA Setup
- [ ] Enable DMA peripheral clock: `crm_periph_clock_enable(CRM_DMA1/2_PERIPH_CLOCK, TRUE)`
- [ ] Reset DMA channel: `dma_reset(DMAx_CHANNELy)`
- [ ] Configure init structure with all parameters
- [ ] Call `dma_init()` to apply configuration
- [ ] Enable interrupts if needed: `dma_interrupt_enable()`
- [ ] Configure NVIC for DMA IRQ
- [ ] Enable channel: `dma_channel_enable(DMAx_CHANNELy, TRUE)`

### DMAMUX Setup (Required for Peripheral DMA)
- [ ] Enable DMAMUX: `dmamux_enable(DMAx, TRUE)`
- [ ] Set request source: `dmamux_init(DMAxMUX_CHANNELy, DMAMUX_DMAREQ_ID_xxx)`
- [ ] Enable peripheral's DMA request (e.g., `adc_dma_mode_enable()`, `tmr_dma_request_enable()`)

### Synchronization Setup (Optional)
- [ ] Configure sync init structure
- [ ] Call `dmamux_sync_config()`
- [ ] Configure external interrupt if using EXINT as sync source

### Generator Setup (Optional)
- [ ] Configure generator init structure
- [ ] Call `dmamux_generator_config()`
- [ ] Use `DMAMUX_DMAREQ_ID_REQ_Gx` as request source

---

## Troubleshooting

### No DMA Transfer

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Transfer never starts | Channel not enabled | Call `dma_channel_enable()` |
| No peripheral requests | DMAMUX not enabled | Call `dmamux_enable(DMAx, TRUE)` |
| Wrong DMAMUX ID | Incorrect request source | Verify DMAMUX_DMAREQ_ID matches peripheral |
| Peripheral DMA disabled | Peripheral not configured | Enable peripheral's DMA request |

### Transfer Incomplete

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Partial data | Buffer size mismatch | Match buffer_size with actual data |
| Stops mid-transfer | Error occurred | Check DTERR flag, verify addresses |
| Circular stops | Loop mode disabled | Set `loop_mode_enable = TRUE` |

### Data Corruption

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Wrong values | Data width mismatch | Match peripheral and memory widths |
| Address errors | Missing increment | Enable appropriate inc_enable flags |
| Overlap issues | Unprotected buffer | Use volatile, check ISR timing |

### DMAMUX Issues

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Sync not working | Wrong polarity | Check sync signal polarity |
| Generator overrun | Too many triggers | Increase request_number or slow trigger |
| Wrong channel | Incorrect MUX channel | Match DMAx_CHANNELy with DMAxMUX_CHANNELy |

---

## Performance Considerations

| Factor | Recommendation |
|--------|---------------|
| **Priority** | Use higher priority for time-critical transfers |
| **Data Width** | Use 32-bit when possible for fastest transfer |
| **Circular Mode** | Use for continuous streaming (ADC, DAC, UART) |
| **Half-Transfer INT** | Enable for double-buffering processing |
| **Memory Alignment** | Align buffers to data width for best performance |
| **Bus Contention** | Spread high-bandwidth DMA across DMA1 and DMA2 |

---

## Related Documentation

### Official Artery Documents
- **[Reference Manual](../RM_AT32F435_437_V2.07_EN.pdf)** - DMA and DMAMUX Chapters
- **[AN0103]** - DMA Application Note (referenced in examples)

### Related Peripherals
- **[ADC](ADC_Analog_to_Digital_Converter.md)** - ADC with DMA
- **[DAC](DAC_Digital_to_Analog_Converter.md)** - DAC with DMA
- **[TMR](TMR_Timer.md)** - Timer DMA requests
- **[USART](USART_Universal_Serial.md)** - UART with DMA

### Example Projects
- `project/at_start_f435/examples/dma/flash_to_sram/`
- `project/at_start_f435/examples/dma/dmamux_data_to_gpio/`
- `project/at_start_f435/examples/dma/dmamux_synchronization_exint/`
- `project/at_start_f435/examples/dma/dmamux_genertor_exint/`

---

**Status:** ✅ Complete  
**Last Updated:** December 2024  
**Firmware Library Version:** v2.2.2


