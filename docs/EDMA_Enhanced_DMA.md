---
title: EDMA (Enhanced DMA)
mcu: AT32F435/437
peripheral: EDMA
version: 2.0.9
---

# EDMA Enhanced Direct Memory Access

## Overview

The AT32F435/437 EDMA (Enhanced DMA) is an advanced DMA controller providing high-speed data transfers between memory and peripherals without CPU intervention. It features 8 independent streams, each with configurable FIFO, burst transfers, double-buffer mode, linked list mode, and 2D transfer capabilities. The integrated EDMAMUX allows flexible routing of any peripheral request to any stream.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                            EDMA Architecture                                     │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  ┌────────────────────────────────────────────────────────────────────────────┐ │
│  │                              EDMAMUX                                        │ │
│  │                    (DMA Request Multiplexer)                                │ │
│  │                                                                             │ │
│  │  ┌──────────────────────────────────────────────────────────────────────┐  │ │
│  │  │                    Peripheral Request Sources                         │  │ │
│  │  │  ADC1-3  DAC1-2  SPI1-4  I2C1-3  USART1-8  TMR1-8  SDIO1-2  QSPI1-2 │  │ │
│  │  │  I2S2-3  DVP     TMR20   and more...                                  │  │ │
│  │  └───────────────────────────────────┬──────────────────────────────────┘  │ │
│  │                                      │                                      │ │
│  │                                      ▼                                      │ │
│  │  ┌─────────────────────────────────────────────────────────────────────┐   │ │
│  │  │                  Request Routing Matrix                              │   │ │
│  │  │     Any peripheral request → Any EDMA stream (1-8)                  │   │ │
│  │  └──────────────────────────────────┬──────────────────────────────────┘   │ │
│  │                                     │                                       │ │
│  │  ┌──────────────────────────────────┴─────────────────────────────────┐   │ │
│  │  │  Synchronization │ Request Generator │ Event Generation            │   │ │
│  │  │   (EXINT0-15)   │   (4 channels)    │  (per channel)              │   │ │
│  │  └────────────────────────────────────────────────────────────────────┘   │ │
│  └────────────────────────────────────────────────────────────────────────────┘ │
│                                     │                                            │
│                                     ▼                                            │
│  ┌────────────────────────────────────────────────────────────────────────────┐ │
│  │                         EDMA Controller                                     │ │
│  │                                                                             │ │
│  │  ┌────────────────────────────────────────────────────────────────────┐   │ │
│  │  │                    8 Independent Streams                            │   │ │
│  │  │                                                                     │   │ │
│  │  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐              │   │ │
│  │  │  │ Stream 1 │ │ Stream 2 │ │ Stream 3 │ │ Stream 4 │              │   │ │
│  │  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘              │   │ │
│  │  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐              │   │ │
│  │  │  │ Stream 5 │ │ Stream 6 │ │ Stream 7 │ │ Stream 8 │              │   │ │
│  │  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘              │   │ │
│  │  └────────────────────────────────────────────────────────────────────┘   │ │
│  │                                                                             │ │
│  │  Per-Stream Features:                                                       │ │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐          │ │
│  │  │    FIFO     │ │   Double    │ │  Link List  │ │ 2D Transfer │          │ │
│  │  │ 4x32-bit    │ │   Buffer    │ │   (Scatter  │ │    Mode     │          │ │
│  │  │ Threshold   │ │    Mode     │ │   Gather)   │ │  (Stride)   │          │ │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘          │ │
│  │                                                                             │ │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐          │ │
│  │  │   Burst     │ │ Circular/   │ │  Priority   │ │   Flow      │          │ │
│  │  │  4/8/16     │ │   Normal    │ │  Control    │ │  Control    │          │ │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘          │ │
│  └────────────────────────────────────────────────────────────────────────────┘ │
│                                     │                                            │
│                                     ▼                                            │
│  ┌────────────────────────────────────────────────────────────────────────────┐ │
│  │                         AHB Bus Matrix                                      │ │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐           │ │
│  │  │   Flash    │  │    SRAM    │  │ Peripherals│  │ External   │           │ │
│  │  │  Memory    │  │  (up to    │  │ (APB1/2)   │  │  Memory    │           │ │
│  │  │            │  │   512KB)   │  │            │  │  (XMC)     │           │ │
│  │  └────────────┘  └────────────┘  └────────────┘  └────────────┘           │ │
│  └────────────────────────────────────────────────────────────────────────────┘ │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Key Features

| Feature | Specification |
|---------|---------------|
| Streams | 8 independent streams |
| FIFO | 4 x 32-bit per stream, configurable threshold |
| Burst Mode | Single, 4, 8, or 16 beats |
| Data Width | 8, 16, or 32-bit (memory and peripheral) |
| Priority | 4 levels (Low, Medium, High, Very High) |
| Double Buffer | Automatic buffer switching |
| Link List | Scatter-gather with descriptor chains |
| 2D Transfer | Row/column stride addressing |
| Transfer Count | 0-65535 per transfer |
| Directions | Memory-to-Memory, Memory-to-Peripheral, Peripheral-to-Memory |

### EDMAMUX Features

| Feature | Specification |
|---------|---------------|
| Request Sources | 100+ peripheral request IDs |
| Synchronization | 24 sources (EXINT0-15, DMAMUX events) |
| Request Generator | 4 channels with trigger input |
| Event Generation | Per-channel event output |

---

## Register Map

### EDMA Base Registers

| Register | Offset | Description |
|----------|--------|-------------|
| `STS1` | 0x00 | Status register for streams 1-4 |
| `STS2` | 0x04 | Status register for streams 5-8 |
| `CLR1` | 0x08 | Clear register for streams 1-4 |
| `CLR2` | 0x0C | Clear register for streams 5-8 |
| `LLCTRL` | 0xD0 | Link list control register |
| `S2DCTRL` | 0xF4 | 2D mode control register |
| `MUXSEL` | 0x13C | EDMAMUX enable register |

### Per-Stream Registers (offset = 0x10 + 0x18 × n)

| Register | Offset | Description |
|----------|--------|-------------|
| `CTRL` | +0x00 | Stream control register |
| `DTCNT` | +0x04 | Data transfer count |
| `PADDR` | +0x08 | Peripheral address |
| `M0ADDR` | +0x0C | Memory 0 address |
| `M1ADDR` | +0x10 | Memory 1 address (double buffer) |
| `FCTRL` | +0x14 | FIFO control register |

---

## Configuration Options

### Transfer Direction

| Enum | Value | Description |
|------|-------|-------------|
| `EDMA_DIR_PERIPHERAL_TO_MEMORY` | 0x00 | Peripheral → Memory |
| `EDMA_DIR_MEMORY_TO_PERIPHERAL` | 0x01 | Memory → Peripheral |
| `EDMA_DIR_MEMORY_TO_MEMORY` | 0x02 | Memory → Memory |

### Data Width

| Enum | Value | Description |
|------|-------|-------------|
| `EDMA_PERIPHERAL_DATA_WIDTH_BYTE` | 0x00 | 8-bit peripheral |
| `EDMA_PERIPHERAL_DATA_WIDTH_HALFWORD` | 0x01 | 16-bit peripheral |
| `EDMA_PERIPHERAL_DATA_WIDTH_WORD` | 0x02 | 32-bit peripheral |
| `EDMA_MEMORY_DATA_WIDTH_BYTE` | 0x00 | 8-bit memory |
| `EDMA_MEMORY_DATA_WIDTH_HALFWORD` | 0x01 | 16-bit memory |
| `EDMA_MEMORY_DATA_WIDTH_WORD` | 0x02 | 32-bit memory |

### Priority Levels

| Enum | Value | Description |
|------|-------|-------------|
| `EDMA_PRIORITY_LOW` | 0x00 | Lowest priority |
| `EDMA_PRIORITY_MEDIUM` | 0x01 | Medium priority |
| `EDMA_PRIORITY_HIGH` | 0x02 | High priority |
| `EDMA_PRIORITY_VERY_HIGH` | 0x03 | Highest priority |

### FIFO Threshold

| Enum | Value | Description |
|------|-------|-------------|
| `EDMA_FIFO_THRESHOLD_1QUARTER` | 0x00 | 1/4 full (4 bytes) |
| `EDMA_FIFO_THRESHOLD_HALF` | 0x01 | 1/2 full (8 bytes) |
| `EDMA_FIFO_THRESHOLD_3QUARTER` | 0x02 | 3/4 full (12 bytes) |
| `EDMA_FIFO_THRESHOLD_FULL` | 0x03 | Full (16 bytes) |

### Burst Mode

| Enum | Value | Description |
|------|-------|-------------|
| `EDMA_MEMORY_SINGLE` | 0x00 | Single transfer |
| `EDMA_MEMORY_BURST_4` | 0x01 | 4-beat burst |
| `EDMA_MEMORY_BURST_8` | 0x02 | 8-beat burst |
| `EDMA_MEMORY_BURST_16` | 0x03 | 16-beat burst |

---

## Interrupts and Flags

### Interrupt Types

| Interrupt | Description |
|-----------|-------------|
| `EDMA_FDT_INT` | Full Data Transfer complete |
| `EDMA_HDT_INT` | Half Data Transfer complete |
| `EDMA_DTERR_INT` | Data Transfer Error |
| `EDMA_DMERR_INT` | Direct Mode Error |
| `EDMA_FERR_INT` | FIFO Error |

### Stream Flags (per stream 1-8)

| Flag Pattern | Example (Stream 1) | Description |
|--------------|-------------------|-------------|
| `EDMA_FDTn_FLAG` | `EDMA_FDT1_FLAG` | Full transfer complete |
| `EDMA_HDTn_FLAG` | `EDMA_HDT1_FLAG` | Half transfer complete |
| `EDMA_DTERRn_FLAG` | `EDMA_DTERR1_FLAG` | Transfer error |
| `EDMA_DMERRn_FLAG` | `EDMA_DMERR1_FLAG` | Direct mode error |
| `EDMA_FERRn_FLAG` | `EDMA_FERR1_FLAG` | FIFO error |

---

## EDMAMUX Request IDs (Partial List)

### ADC/DAC

| Request ID | Description |
|------------|-------------|
| `EDMAMUX_DMAREQ_ID_ADC1` | ADC1 conversion complete |
| `EDMAMUX_DMAREQ_ID_ADC2` | ADC2 conversion complete |
| `EDMAMUX_DMAREQ_ID_ADC3` | ADC3 conversion complete |
| `EDMAMUX_DMAREQ_ID_DAC1` | DAC1 output |
| `EDMAMUX_DMAREQ_ID_DAC2` | DAC2 output |

### SPI/I2S

| Request ID | Description |
|------------|-------------|
| `EDMAMUX_DMAREQ_ID_SPI1_RX` | SPI1 receive |
| `EDMAMUX_DMAREQ_ID_SPI1_TX` | SPI1 transmit |
| `EDMAMUX_DMAREQ_ID_SPI2_RX` | SPI2 receive |
| `EDMAMUX_DMAREQ_ID_SPI2_TX` | SPI2 transmit |
| `EDMAMUX_DMAREQ_ID_SPI3_RX` | SPI3 receive |
| `EDMAMUX_DMAREQ_ID_SPI3_TX` | SPI3 transmit |
| `EDMAMUX_DMAREQ_ID_SPI4_RX` | SPI4 receive |
| `EDMAMUX_DMAREQ_ID_SPI4_TX` | SPI4 transmit |

### USART/UART

| Request ID | Description |
|------------|-------------|
| `EDMAMUX_DMAREQ_ID_USART1_RX` | USART1 receive |
| `EDMAMUX_DMAREQ_ID_USART1_TX` | USART1 transmit |
| `EDMAMUX_DMAREQ_ID_USART2_RX` | USART2 receive |
| `EDMAMUX_DMAREQ_ID_USART2_TX` | USART2 transmit |
| ... | (USART3, UART4-8, USART6) |

### Timers

| Request ID | Description |
|------------|-------------|
| `EDMAMUX_DMAREQ_ID_TMR1_CH1` | Timer 1 Channel 1 |
| `EDMAMUX_DMAREQ_ID_TMR1_OVERFLOW` | Timer 1 Overflow |
| `EDMAMUX_DMAREQ_ID_TMR1_TRIG` | Timer 1 Trigger |
| ... | (TMR2-8, TMR20) |

### Other Peripherals

| Request ID | Description |
|------------|-------------|
| `EDMAMUX_DMAREQ_ID_I2C1_RX/TX` | I2C1 RX/TX |
| `EDMAMUX_DMAREQ_ID_SDIO1` | SDIO1 |
| `EDMAMUX_DMAREQ_ID_QSPI1` | QSPI1 |
| `EDMAMUX_DMAREQ_ID_DVP` | DVP (camera interface) |
| `EDMAMUX_DMAREQ_ID_REQ_G1-4` | Generator channels |

---

## API Reference

### Core EDMA Functions

| Function | Description |
|----------|-------------|
| `edma_reset(stream)` | Reset stream to default state |
| `edma_init(stream, config)` | Initialize stream with configuration |
| `edma_default_para_init(config)` | Initialize config struct with defaults |
| `edma_stream_enable(stream, state)` | Enable/disable stream |
| `edma_data_number_set(stream, count)` | Set transfer count |
| `edma_data_number_get(stream)` | Get remaining count |

### Interrupt Functions

| Function | Description |
|----------|-------------|
| `edma_interrupt_enable(stream, int, state)` | Enable/disable interrupt |
| `edma_flag_get(flag)` | Get flag status |
| `edma_interrupt_flag_get(flag)` | Get interrupt flag status |
| `edma_flag_clear(flag)` | Clear flag |

### Double Buffer Functions

| Function | Description |
|----------|-------------|
| `edma_double_buffer_mode_init(stream, mem1, target)` | Configure double buffer |
| `edma_double_buffer_mode_enable(stream, state)` | Enable double buffer mode |
| `edma_memory_addr_set(stream, addr, target)` | Set memory address |
| `edma_memory_target_get(stream)` | Get current buffer target |

### 2D Transfer Functions

| Function | Description |
|----------|-------------|
| `edma_2d_init(stream_2d, src_stride, dst_stride, xcnt, ycnt)` | Configure 2D mode |
| `edma_2d_enable(stream_2d, state)` | Enable 2D mode |

### Link List Functions

| Function | Description |
|----------|-------------|
| `edma_link_list_init(stream_ll, pointer)` | Set link list pointer |
| `edma_link_list_enable(stream_ll, state)` | Enable link list mode |

### EDMAMUX Functions

| Function | Description |
|----------|-------------|
| `edmamux_enable(state)` | Enable EDMAMUX |
| `edmamux_init(channel, req_id)` | Route request to channel |
| `edmamux_sync_config(channel, config)` | Configure synchronization |
| `edmamux_generator_config(gen, config)` | Configure request generator |

---

## Complete Examples

### Example 1: Memory-to-Memory Transfer

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * EDMA Memory-to-Memory Transfer Example
 * 
 * Transfers 32 words from Flash to SRAM using EDMA Stream 1.
 * Demonstrates basic EDMA configuration without peripheral involvement.
 ******************************************************************************/

#define BUFFER_SIZE  32

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

uint32_t dst_buffer[BUFFER_SIZE];
volatile uint32_t transfer_complete = 0;

void EDMA_Stream1_IRQHandler(void)
{
  if(edma_interrupt_flag_get(EDMA_FDT1_FLAG) != RESET)
  {
    transfer_complete = 1;
    edma_flag_clear(EDMA_FDT1_FLAG);
  }
}

int main(void)
{
  edma_init_type edma_config;
  
  system_clock_config();
  at32_board_init();

  /* Enable EDMA clock */
  crm_periph_clock_enable(CRM_EDMA_PERIPH_CLOCK, TRUE);

  /* Configure EDMA Stream 1 for memory-to-memory transfer */
  edma_default_para_init(&edma_config);
  edma_config.direction = EDMA_DIR_MEMORY_TO_MEMORY;
  edma_config.buffer_size = BUFFER_SIZE;
  
  /* For M2M: peripheral_base_addr = source, memory0_base_addr = destination */
  edma_config.peripheral_base_addr = (uint32_t)src_buffer;
  edma_config.peripheral_inc_enable = TRUE;
  edma_config.peripheral_data_width = EDMA_PERIPHERAL_DATA_WIDTH_WORD;
  
  edma_config.memory0_base_addr = (uint32_t)dst_buffer;
  edma_config.memory_inc_enable = TRUE;
  edma_config.memory_data_width = EDMA_MEMORY_DATA_WIDTH_WORD;
  
  edma_config.priority = EDMA_PRIORITY_HIGH;
  edma_config.loop_mode_enable = FALSE;
  edma_config.fifo_mode_enable = FALSE;
  edma_config.memory_burst_mode = EDMA_MEMORY_SINGLE;
  edma_config.peripheral_burst_mode = EDMA_PERIPHERAL_SINGLE;
  
  edma_init(EDMA_STREAM1, &edma_config);

  /* Enable transfer complete interrupt */
  edma_interrupt_enable(EDMA_STREAM1, EDMA_FDT_INT, TRUE);
  
  /* Configure NVIC */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  nvic_irq_enable(EDMA_Stream1_IRQn, 1, 0);

  /* Start transfer */
  edma_stream_enable(EDMA_STREAM1, TRUE);

  /* Wait for completion */
  while(transfer_complete == 0);

  /* Verify transfer */
  for(uint32_t i = 0; i < BUFFER_SIZE; i++)
  {
    if(src_buffer[i] != dst_buffer[i])
    {
      at32_led_on(LED4);  /* Error */
      while(1);
    }
  }
  
  at32_led_on(LED2);  /* Success */
  at32_led_on(LED3);

  while(1);
}
```

---

### Example 2: Link List Mode (Scatter-Gather)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * EDMA Link List Mode Example
 * 
 * Transfers multiple non-contiguous buffers using linked descriptors.
 * Each descriptor points to the next, creating a chain of transfers.
 * 
 * Use Case: Sending multiple UART messages without CPU intervention.
 ******************************************************************************/

#define LIST_COUNT  3
#define MSG_SIZE    35

/* Link list descriptor structure (must match hardware format) */
typedef struct {
  uint32_t ctrl_dtcnt;  /* Control + data count */
  uint32_t paddr;       /* Peripheral address */
  uint32_t m0addr;      /* Memory address */
  uint32_t llp;         /* Link to next descriptor (0 = end) */
} ll_descriptor_t;

/* Message buffers */
const int8_t messages[LIST_COUNT][MSG_SIZE] = {
  {'T','h','i','s',' ','i','s',' ','E','D','M','A',' '},
  {'L','i','n','k',' ','L','i','s','t',' ','M','o','d','e',' ','t','e','s','t','!'},
  {'\r','\n','T','r','a','n','s','f','e','r',' ','c','o','m','p','l','e','t','e','!','\r','\n'}
};

/* Descriptors must be 16-byte aligned! */
__ALIGNED(16) ll_descriptor_t descriptors[LIST_COUNT];

void init_descriptors(void)
{
  for(uint32_t i = 0; i < LIST_COUNT; i++)
  {
    /*=========================================================================
     * Descriptor format:
     * ctrl_dtcnt[31:16] = control bits (direction, increment, width, etc.)
     * ctrl_dtcnt[15:0]  = data count
     * 
     * 0x20090000 = Memory-to-Peripheral, Memory Inc, 8-bit width
     *=========================================================================*/
    descriptors[i].ctrl_dtcnt = 0x20090000 | sizeof(messages[i]);
    descriptors[i].paddr = (uint32_t)(&USART1->dt);
    descriptors[i].m0addr = (uint32_t)(&messages[i][0]);
    
    /* Chain to next descriptor, or NULL for last */
    if(i == (LIST_COUNT - 1))
      descriptors[i].llp = 0;  /* End of list */
    else
      descriptors[i].llp = (uint32_t)(&descriptors[i + 1]);
  }
}

void EDMA_Stream1_IRQHandler(void)
{
  if(edma_interrupt_flag_get(EDMA_FDT1_FLAG) != RESET)
  {
    at32_led_on(LED2);  /* Transfer complete */
    edma_flag_clear(EDMA_FDT1_FLAG);
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  
  /* Initialize USART1 for TX */
  /* ... USART init code ... */
  usart_dma_transmitter_enable(USART1, TRUE);

  /* Enable EDMA clock */
  crm_periph_clock_enable(CRM_EDMA_PERIPH_CLOCK, TRUE);

  /* Initialize link list descriptors */
  init_descriptors();

  /* Enable link list mode for Stream 1 */
  edma_link_list_enable(EDMA_STREAM1_LL, TRUE);
  edma_link_list_init(EDMA_STREAM1_LL, (uint32_t)descriptors);

  /* Configure EDMAMUX for USART1 TX */
  edmamux_enable(TRUE);
  edmamux_init(EDMAMUX_CHANNEL1, EDMAMUX_DMAREQ_ID_USART1_TX);

  /* Enable interrupt */
  edma_interrupt_enable(EDMA_STREAM1, EDMA_FDT_INT, TRUE);
  nvic_irq_enable(EDMA_Stream1_IRQn, 1, 0);

  /* Start transfer - hardware follows the descriptor chain */
  edma_stream_enable(EDMA_STREAM1, TRUE);

  while(1);
}
```

---

### Example 3: 2D Transfer Mode

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * EDMA 2D Transfer Mode Example
 * 
 * Transfers data with row/column stride for 2D array operations.
 * Useful for: Image processing, matrix operations, framebuffer manipulation.
 * 
 * This example extracts a 2-column strip from a 4-column source array.
 ******************************************************************************/

#define BUFFER_SIZE  32

/* Source: 8 rows × 4 columns = 32 words */
const uint32_t src_buffer[BUFFER_SIZE] = {
  0x01020304, 0x05060708, 0x090A0B0C, 0x0D0E0F10,  /* Row 0 */
  0x11121314, 0x15161718, 0x191A1B1C, 0x1D1E1F20,  /* Row 1 */
  0x21222324, 0x25262728, 0x292A2B2C, 0x2D2E2F30,  /* Row 2 */
  0x31323334, 0x35363738, 0x393A3B3C, 0x3D3E3F40,  /* Row 3 */
  0x41424344, 0x45464748, 0x494A4B4C, 0x4D4E4F50,  /* Row 4 */
  0x51525354, 0x55565758, 0x595A5B5C, 0x5D5E5F60,  /* Row 5 */
  0x61626364, 0x65666768, 0x696A6B6C, 0x6D6E6F70,  /* Row 6 */
  0x71727374, 0x75767778, 0x797A7B7C, 0x7D7E7F80   /* Row 7 */
};

/* Destination: 8 rows × 2 columns = 16 words */
uint32_t dst_buffer[BUFFER_SIZE];
volatile uint32_t transfer_complete = 0;

void EDMA_Stream1_IRQHandler(void)
{
  if(edma_interrupt_flag_get(EDMA_FDT1_FLAG) != RESET)
  {
    transfer_complete = 1;
    edma_flag_clear(EDMA_FDT1_FLAG);
  }
}

int main(void)
{
  edma_init_type edma_config;
  
  system_clock_config();
  at32_board_init();
  uart_print_init(115200);

  crm_periph_clock_enable(CRM_EDMA_PERIPH_CLOCK, TRUE);

  /* Configure basic EDMA parameters */
  edma_default_para_init(&edma_config);
  edma_config.direction = EDMA_DIR_MEMORY_TO_MEMORY;
  edma_config.buffer_size = BUFFER_SIZE;
  edma_config.peripheral_base_addr = (uint32_t)src_buffer;
  edma_config.peripheral_inc_enable = TRUE;
  edma_config.peripheral_data_width = EDMA_PERIPHERAL_DATA_WIDTH_WORD;
  edma_config.memory0_base_addr = (uint32_t)dst_buffer;
  edma_config.memory_inc_enable = TRUE;
  edma_config.memory_data_width = EDMA_MEMORY_DATA_WIDTH_WORD;
  edma_config.fifo_mode_enable = TRUE;
  edma_config.fifo_threshold = EDMA_FIFO_THRESHOLD_FULL;
  edma_config.priority = EDMA_PRIORITY_HIGH;
  edma_config.loop_mode_enable = FALSE;
  
  edma_init(EDMA_STREAM1, &edma_config);

  edma_interrupt_enable(EDMA_STREAM1, EDMA_FDT_INT, TRUE);
  nvic_irq_enable(EDMA_Stream1_IRQn, 1, 0);

  /*=========================================================================
   * 2D Mode Configuration
   * 
   * Extract 2 columns from 4-column source (copy columns 0-1, skip 2-3)
   * 
   * Parameters:
   * - src_stride = 0x10 (16 bytes = 4 words per row, skip to next row)
   * - dst_stride = 0x08 (8 bytes = 2 words per row)
   * - xcnt = 2 (copy 2 words per row)
   * - ycnt = 8 (8 rows total)
   *=========================================================================*/
  edma_2d_init(EDMA_STREAM1_2D, 0x10, 0x08, 0x02, 0x08);
  edma_2d_enable(EDMA_STREAM1_2D, TRUE);

  edma_stream_enable(EDMA_STREAM1, TRUE);

  while(transfer_complete == 0);

  /* Print results */
  printf("Source (4 columns × 8 rows):\r\n");
  for(int i = 0; i < 32; i++)
  {
    printf("%08X ", src_buffer[i]);
    if((i + 1) % 4 == 0) printf("\r\n");
  }

  printf("\r\nDestination (2 columns × 8 rows):\r\n");
  for(int i = 0; i < 16; i++)
  {
    printf("%08X ", dst_buffer[i]);
    if((i + 1) % 2 == 0) printf("\r\n");
  }

  while(1);
}
```

---

### Example 4: Burst Mode with Timer PWM

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * EDMA Burst Mode with Timer PWM Example
 * 
 * Uses burst transfers to update all 4 PWM channels simultaneously.
 * Timer overflow triggers DMA to update duty cycle values.
 * 
 * This creates smooth PWM breathing effect on 4 channels.
 ******************************************************************************/

uint32_t duty_buffer[4] = {10, 10, 10, 10};

void TMR1_OVF_TMR10_IRQHandler(void)
{
  if(tmr_interrupt_flag_get(TMR1, TMR_OVF_FLAG) == SET)
  {
    /* Update duty cycle values for breathing effect */
    duty_buffer[0] += 10;
    duty_buffer[1] += 10;
    duty_buffer[2] += 10;
    duty_buffer[3] += 10;

    if(duty_buffer[0] >= 500)
    {
      duty_buffer[0] = 10;
      duty_buffer[1] = 10;
      duty_buffer[2] = 10;
      duty_buffer[3] = 10;
    }

    tmr_flag_clear(TMR1, TMR_OVF_FLAG);
  }
}

int main(void)
{
  edma_init_type edma_config;
  gpio_init_type gpio_config;
  tmr_output_config_type tmr_oc_config;
  
  system_clock_config();
  at32_board_init();

  /* Enable clocks */
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_EDMA_PERIPH_CLOCK, TRUE);

  /* Configure PA8-11 as TMR1 CH1-4 */
  gpio_config.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
  gpio_config.gpio_mode = GPIO_MODE_MUX;
  gpio_config.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_config.gpio_pull = GPIO_PULL_NONE;
  gpio_config.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOA, &gpio_config);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_1);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_1);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_1);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_1);

  /*=========================================================================
   * EDMA Configuration for Burst Transfer
   * 
   * Key points:
   * - Transfer 4 words to 4 consecutive timer compare registers
   * - Use burst mode for atomic update of all channels
   * - Peripheral address increment allows writing to C1DT, C2DT, C3DT, C4DT
   *=========================================================================*/
  edma_default_para_init(&edma_config);
  edma_config.direction = EDMA_DIR_MEMORY_TO_PERIPHERAL;
  edma_config.buffer_size = 4;
  edma_config.peripheral_base_addr = (uint32_t)&(TMR1->c1dt);
  edma_config.peripheral_inc_enable = TRUE;
  edma_config.peripheral_data_width = EDMA_PERIPHERAL_DATA_WIDTH_WORD;
  edma_config.memory0_base_addr = (uint32_t)duty_buffer;
  edma_config.memory_inc_enable = TRUE;
  edma_config.memory_data_width = EDMA_MEMORY_DATA_WIDTH_WORD;
  
  /* BURST MODE: Transfer 4 beats at once */
  edma_config.peripheral_burst_mode = EDMA_PERIPHERAL_BURST_4;
  edma_config.memory_burst_mode = EDMA_MEMORY_BURST_4;
  edma_config.fifo_mode_enable = TRUE;
  edma_config.fifo_threshold = EDMA_FIFO_THRESHOLD_FULL;
  
  edma_config.priority = EDMA_PRIORITY_HIGH;
  edma_config.loop_mode_enable = TRUE;  /* Continuous transfers */
  
  edma_init(EDMA_STREAM1, &edma_config);

  /* Route TMR1 overflow to EDMA Stream 1 */
  edmamux_enable(TRUE);
  edmamux_init(EDMAMUX_CHANNEL1, EDMAMUX_DMAREQ_ID_TMR1_OVERFLOW);

  edma_stream_enable(EDMA_STREAM1, TRUE);

  /* Configure TMR1 */
  tmr_base_init(TMR1, 500, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  /* Configure PWM channels */
  tmr_output_default_para_init(&tmr_oc_config);
  tmr_oc_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_oc_config.oc_output_state = TRUE;
  tmr_oc_config.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
  
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_oc_config);
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_oc_config);
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_oc_config);
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &tmr_oc_config);

  tmr_output_enable(TMR1, TRUE);
  tmr_dma_request_enable(TMR1, TMR_OVERFLOW_DMA_REQUEST, TRUE);
  tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);
  
  nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 1, 0);
  
  tmr_counter_enable(TMR1, TRUE);

  while(1);
}
```

---

### Example 5: EDMAMUX Synchronization

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * EDMAMUX Synchronization Example
 * 
 * Synchronizes DMA transfers to external interrupt events.
 * DMA only transfers when EXINT1 rising edge is detected.
 * 
 * Use Case: Transfer data only when external sensor signals data ready.
 ******************************************************************************/

#define BUFFER_SIZE  16

uint16_t src_buffer[BUFFER_SIZE] = {
  0x0001, 0x0002, 0x0003, 0x0004,
  0x0005, 0x0006, 0x0007, 0x0008,
  0x0009, 0x000A, 0x000B, 0x000C,
  0x000D, 0x000E, 0x000F, 0x0010
};
uint16_t dst_buffer[BUFFER_SIZE];

void EDMA_Stream4_IRQHandler(void)
{
  if(edma_interrupt_flag_get(EDMA_FDT4_FLAG) != RESET)
  {
    at32_led_on(LED2);  /* Transfer complete */
    at32_led_on(LED3);
    edma_flag_clear(EDMA_FDT4_FLAG);
  }
}

void EXINT1_IRQHandler(void)
{
  if(exint_interrupt_flag_get(EXINT_LINE_1) != RESET)
  {
    at32_led_toggle(LED4);  /* Show sync event */
    exint_flag_clear(EXINT_LINE_1);
  }
}

int main(void)
{
  edma_init_type edma_config;
  edmamux_sync_init_type sync_config;
  gpio_init_type gpio_config;
  exint_init_type exint_config;
  
  system_clock_config();
  at32_board_init();

  /* Enable clocks */
  crm_periph_clock_enable(CRM_EDMA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  /* Configure PA1 as EXINT1 input */
  gpio_config.gpio_pins = GPIO_PINS_1;
  gpio_config.gpio_mode = GPIO_MODE_INPUT;
  gpio_config.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_config);

  /* Configure EXINT1 */
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE1);
  exint_default_para_init(&exint_config);
  exint_config.line_enable = TRUE;
  exint_config.line_mode = EXINT_LINE_INTERRUPT;
  exint_config.line_select = EXINT_LINE_1;
  exint_config.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_config);
  nvic_irq_enable(EXINT1_IRQn, 1, 0);

  /* Configure EDMA Stream 4 */
  edma_default_para_init(&edma_config);
  edma_config.direction = EDMA_DIR_PERIPHERAL_TO_MEMORY;
  edma_config.buffer_size = BUFFER_SIZE;
  edma_config.peripheral_base_addr = (uint32_t)src_buffer;
  edma_config.peripheral_inc_enable = TRUE;
  edma_config.peripheral_data_width = EDMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  edma_config.memory0_base_addr = (uint32_t)dst_buffer;
  edma_config.memory_inc_enable = TRUE;
  edma_config.memory_data_width = EDMA_MEMORY_DATA_WIDTH_HALFWORD;
  edma_config.priority = EDMA_PRIORITY_HIGH;
  edma_config.loop_mode_enable = FALSE;
  
  edma_init(EDMA_STREAM4, &edma_config);

  /*=========================================================================
   * EDMAMUX Synchronization Configuration
   * 
   * - Sync signal: EXINT1 rising edge
   * - Transfer 4 items per sync event
   * - 4 sync events needed for complete transfer (16 items total)
   *=========================================================================*/
  edmamux_sync_default_para_init(&sync_config);
  sync_config.sync_enable = TRUE;
  sync_config.sync_event_enable = TRUE;
  sync_config.sync_polarity = EDMAMUX_SYNC_POLARITY_RISING;
  sync_config.sync_request_number = 4;  /* 4 transfers per sync */
  sync_config.sync_signal_sel = EDMAMUX_SYNC_ID_EXINT1;
  
  edmamux_sync_config(EDMAMUX_CHANNEL4, &sync_config);

  /* Enable EDMAMUX and route TMR1 overflow as DMA trigger */
  edmamux_enable(TRUE);
  edmamux_init(EDMAMUX_CHANNEL4, EDMAMUX_DMAREQ_ID_TMR1_OVERFLOW);

  edma_interrupt_enable(EDMA_STREAM4, EDMA_FDT_INT, TRUE);
  nvic_irq_enable(EDMA_Stream4_IRQn, 1, 0);

  /* Configure TMR1 as DMA request source */
  tmr_base_init(TMR1, 5000, 0);
  tmr_dma_request_enable(TMR1, TMR_OVERFLOW_DMA_REQUEST, TRUE);

  edma_stream_enable(EDMA_STREAM4, TRUE);
  tmr_counter_enable(TMR1, TRUE);

  /* Now each rising edge on PA1 will trigger 4 DMA transfers */
  /* Press button connected to PA1 four times to complete transfer */

  while(1);
}
```

---

### Example 6: Double Buffer Mode

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * EDMA Double Buffer Mode Example
 * 
 * Uses two memory buffers that automatically swap on transfer completion.
 * While DMA fills one buffer, CPU can process the other.
 * 
 * Use Case: Continuous ADC sampling, audio streaming, etc.
 ******************************************************************************/

#define BUFFER_SIZE  256

uint16_t buffer0[BUFFER_SIZE];
uint16_t buffer1[BUFFER_SIZE];
volatile uint8_t current_buffer = 0;
volatile uint32_t buffer_ready = 0;

void EDMA_Stream1_IRQHandler(void)
{
  if(edma_interrupt_flag_get(EDMA_FDT1_FLAG) != RESET)
  {
    /* Check which buffer just completed */
    if(edma_memory_target_get(EDMA_STREAM1) == EDMA_MEMORY_0)
    {
      /* Now filling buffer 0, buffer 1 is ready to process */
      current_buffer = 1;
    }
    else
    {
      /* Now filling buffer 1, buffer 0 is ready to process */
      current_buffer = 0;
    }
    buffer_ready = 1;
    
    edma_flag_clear(EDMA_FDT1_FLAG);
  }
}

void process_buffer(uint16_t *buffer)
{
  /* Process ADC data - example: find max value */
  uint16_t max_val = 0;
  for(int i = 0; i < BUFFER_SIZE; i++)
  {
    if(buffer[i] > max_val)
      max_val = buffer[i];
  }
  /* Use max_val... */
}

int main(void)
{
  edma_init_type edma_config;
  
  system_clock_config();
  at32_board_init();

  crm_periph_clock_enable(CRM_EDMA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);

  /* Configure ADC... */
  /* adc_init(...); */

  /* Configure EDMA Stream 1 */
  edma_default_para_init(&edma_config);
  edma_config.direction = EDMA_DIR_PERIPHERAL_TO_MEMORY;
  edma_config.buffer_size = BUFFER_SIZE;
  edma_config.peripheral_base_addr = (uint32_t)&(ADC1->odt);
  edma_config.peripheral_inc_enable = FALSE;
  edma_config.peripheral_data_width = EDMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  edma_config.memory0_base_addr = (uint32_t)buffer0;
  edma_config.memory_inc_enable = TRUE;
  edma_config.memory_data_width = EDMA_MEMORY_DATA_WIDTH_HALFWORD;
  edma_config.priority = EDMA_PRIORITY_HIGH;
  edma_config.loop_mode_enable = TRUE;  /* Circular mode required */
  edma_config.fifo_mode_enable = TRUE;
  edma_config.fifo_threshold = EDMA_FIFO_THRESHOLD_HALF;
  
  edma_init(EDMA_STREAM1, &edma_config);

  /*=========================================================================
   * Double Buffer Mode Configuration
   * 
   * - memory0_base_addr: First buffer (set in edma_init)
   * - buffer1: Second buffer address
   * - Start with EDMA_MEMORY_0
   * 
   * Hardware automatically swaps buffers on each transfer completion.
   *=========================================================================*/
  edma_double_buffer_mode_init(EDMA_STREAM1, (uint32_t)buffer1, EDMA_MEMORY_0);
  edma_double_buffer_mode_enable(EDMA_STREAM1, TRUE);

  /* Route ADC1 to EDMA Stream 1 */
  edmamux_enable(TRUE);
  edmamux_init(EDMAMUX_CHANNEL1, EDMAMUX_DMAREQ_ID_ADC1);

  edma_interrupt_enable(EDMA_STREAM1, EDMA_FDT_INT, TRUE);
  nvic_irq_enable(EDMA_Stream1_IRQn, 1, 0);

  edma_stream_enable(EDMA_STREAM1, TRUE);
  
  /* Start ADC conversion */
  /* adc_software_start_conversion_enable(ADC1, TRUE); */

  while(1)
  {
    if(buffer_ready)
    {
      buffer_ready = 0;
      
      /* Process the completed buffer while DMA fills the other */
      if(current_buffer == 0)
        process_buffer(buffer0);
      else
        process_buffer(buffer1);
    }
  }
}
```

---

## Implementation Checklist

### Basic EDMA Setup
- [ ] Enable EDMA clock (`CRM_EDMA_PERIPH_CLOCK`)
- [ ] Reset stream (`edma_reset()`)
- [ ] Configure transfer parameters
- [ ] Enable EDMAMUX if using peripherals
- [ ] Configure interrupts if needed
- [ ] Enable stream

### For Peripheral Transfers
- [ ] Enable EDMAMUX (`edmamux_enable(TRUE)`)
- [ ] Route request (`edmamux_init(channel, request_id)`)
- [ ] Enable peripheral's DMA request

### For Double Buffer
- [ ] Enable circular/loop mode
- [ ] Initialize double buffer mode
- [ ] Handle buffer swap in interrupt

### For Link List Mode
- [ ] Create descriptor array (16-byte aligned!)
- [ ] Initialize descriptor chain
- [ ] Enable link list mode
- [ ] Set initial link list pointer

### For 2D Mode
- [ ] Enable FIFO mode
- [ ] Configure 2D parameters (xcnt, ycnt, strides)
- [ ] Enable 2D mode

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| No transfer occurs | Stream not enabled | Call `edma_stream_enable()` |
| Transfer stalls | EDMAMUX not enabled | Call `edmamux_enable(TRUE)` |
| Wrong data | Incorrect data width | Match peripheral and memory widths |
| FIFO error | Burst/threshold mismatch | Ensure burst size fits threshold |
| Link list fails | Alignment issue | Align descriptors to 16 bytes |
| Transfer error | Bus error | Check address alignment |
| Double buffer issues | Loop mode disabled | Enable `loop_mode_enable` |

### FIFO/Burst Configuration Rules

| Memory Width | Burst Size | Min FIFO Threshold |
|--------------|------------|-------------------|
| Byte | 4 | 1/4 (4 bytes) |
| Byte | 8 | 1/2 (8 bytes) |
| Byte | 16 | Full (16 bytes) |
| Halfword | 4 | 1/2 (8 bytes) |
| Halfword | 8 | Full (16 bytes) |
| Word | 4 | Full (16 bytes) |

---

## Performance Considerations

| Mode | Throughput | Use Case |
|------|------------|----------|
| Single | Normal | Low-speed peripherals |
| Burst 4 | 4x single | Medium-speed transfers |
| Burst 8 | 8x single | High-speed transfers |
| Burst 16 | 16x single | Maximum throughput |

### Priority Arbitration

When multiple streams request simultaneously:
1. Very High > High > Medium > Low
2. Within same priority: Lower stream number wins

---

## See Also

- [DMA Documentation](./DMA_Direct_Memory_Access.md) - Standard DMA controller
- [DVP Documentation](./DVP_Digital_Video_Port.md) - Camera interface using EDMA
- [ADC Documentation](./ADC_Analog_to_Digital_Converter.md) - ADC with DMA
- Application Note AN0090 - EDMA Application Guide
- AT32F435_437 Reference Manual - EDMA Chapter

