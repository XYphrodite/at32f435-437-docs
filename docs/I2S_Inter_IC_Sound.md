---
title: I2S - Inter-IC Sound
category: Peripheral
complexity: Intermediate
mcu: AT32F435/437
peripheral: I2S
keywords: [i2s, audio, sound, spi, dma, phillips, pcm, msb, lsb, codec]
---

# I2S - Inter-IC Sound

## Overview

The I2S (Inter-IC Sound) peripheral provides a high-quality digital audio interface for connecting to external audio codecs, DACs, ADCs, and other audio devices. I2S is integrated with the SPI peripheral and shares the same hardware, allowing switching between SPI and I2S modes. The AT32F435/437 supports standard audio protocols including I2S Phillips, MSB/LSB-justified, and PCM standards. Full-duplex operation is enabled through I2S2EXT and I2S3EXT extension interfaces.

### Key Features

| Feature | Specification |
|---------|---------------|
| I2S Interfaces | 4 (SPI1, SPI2, SPI3, SPI4 can operate as I2S) |
| Full-Duplex Extensions | I2S2EXT (with SPI2), I2S3EXT (with SPI3) |
| Audio Protocols | Phillips, MSB-justified, LSB-justified, PCM short/long |
| Sampling Frequencies | 8K, 11.025K, 16K, 22.05K, 32K, 44.1K, 48K, 96K, 192K Hz |
| Data Formats | 16/24/32-bit data in 16/32-bit channel frames |
| Operation Modes | Master TX, Master RX, Slave TX, Slave RX |
| Master Clock | MCLK output support (256×Fs or 128×Fs for PCM) |
| DMA | TX and RX support |
| Interrupts | TDBE, RDBF, Errors |

---

## Architecture

```
                         ┌──────────────────────────────────────────────────────────┐
                         │                    SPI/I2S Peripheral                     │
                         │                                                           │
  ┌───────────────┐     │   ┌─────────────────────────────────────────────────────┐│
  │   APB1/APB2   │─────┼──►│              Mode Select (I2SMSEL)                   ││
  │     Bus       │     │   │         ┌────────────────────────────────┐          ││
  └───────────────┘     │   │         │                                │          ││
                         │   │    ┌────▼────┐                    ┌─────▼─────┐    ││
                         │   │    │   SPI   │                    │    I2S    │    ││
                         │   │    │  Mode   │                    │   Mode    │    ││
                         │   │    └────┬────┘                    └─────┬─────┘    ││
                         │   │         │                               │          ││
                         │   └─────────┼───────────────────────────────┼──────────┘│
                         │             │                               │           │
                         │   ┌─────────┼───────────────────────────────┼─────────┐ │
                         │   │         │         I2S Core              │         │ │
                         │   │         │                               │         │ │
                         │   │   ┌─────▼─────────────────────────▼─────┐         │ │
                         │   │   │         Clock Generator             │         │ │
                         │   │   │   ┌─────────────────────────────┐   │         │ │
                         │   │   │   │  I2SDIV, I2SODD → Fs calc   │   │         │ │
                         │   │   │   └─────────────────────────────┘   │         │ │
                         │   │   └────────────────┬────────────────────┘         │ │
                         │   │                    │                               │ │
                         │   │   ┌────────────────▼────────────────────┐         │ │
                         │   │   │       Protocol Controller           │         │ │
                         │   │   │  Phillips / MSB / LSB / PCM         │         │ │
                         │   │   └────────────────┬────────────────────┘         │ │
                         │   │                    │                               │ │
                         │   │   ┌────────────────▼────────────────────┐         │ │
                         │   │   │      TX/RX Shift Register           │         │ │
                         │   │   │         (16-bit DT)                 │         │ │
                         │   │   └────────────────┬────────────────────┘         │ │
                         │   │                    │                               │ │
                         │   └────────────────────┼───────────────────────────────┘ │
                         │                        │                                  │
                         │              ┌─────────┴─────────┐                       │
                         │              │                   │                       │
                         │         ┌────▼────┐        ┌─────▼────┐                  │
                         │         │  I2Sx   │        │ I2SxEXT  │──► Full-Duplex  │
                         │         │ (main)  │        │  (ext)   │                  │
                         │         └────┬────┘        └─────┬────┘                  │
                         │              │                   │                       │
                         └──────────────┼───────────────────┼───────────────────────┘
                                        │                   │
                                   ┌────┴────┐         ┌────┴────┐
                                   │SD (Data)│         │EXT_SD   │
                                   │WS (L/R) │         │(Data)   │
                                   │CK (CLK) │         └─────────┘
                                   │MCK      │
                                   └─────────┘
```

---

## I2S Signal Pins

| Signal | Direction (Master TX) | Direction (Slave RX) | Description |
|--------|----------------------|----------------------|-------------|
| WS | Output | Input | Word Select (Left/Right channel) |
| CK | Output | Input | Serial Clock (bit clock) |
| SD | Output | Input | Serial Data |
| MCK | Output | - | Master Clock (optional, for external codec) |
| EXT_SD | Input | Output | Extension data for full-duplex |

---

## Configuration Types

### Audio Protocol

```c
typedef enum {
  I2S_AUDIO_PROTOCOL_PHILLIPS   = 0x00,  /* I2S Phillips standard */
  I2S_AUDIO_PROTOCOL_MSB        = 0x01,  /* MSB-justified standard */
  I2S_AUDIO_PROTOCOL_LSB        = 0x02,  /* LSB-justified standard */
  I2S_AUDIO_PROTOCOL_PCM_SHORT  = 0x03,  /* PCM standard - short frame */
  I2S_AUDIO_PROTOCOL_PCM_LONG   = 0x04   /* PCM standard - long frame */
} i2s_audio_protocol_type;
```

### Audio Sampling Frequency

```c
typedef enum {
  I2S_AUDIO_FREQUENCY_DEFAULT = 2,       /* Default (I2SDIV=2) */
  I2S_AUDIO_FREQUENCY_8K      = 8000,    /* 8 kHz */
  I2S_AUDIO_FREQUENCY_11_025K = 11025,   /* 11.025 kHz */
  I2S_AUDIO_FREQUENCY_16K     = 16000,   /* 16 kHz */
  I2S_AUDIO_FREQUENCY_22_05K  = 22050,   /* 22.05 kHz */
  I2S_AUDIO_FREQUENCY_32K     = 32000,   /* 32 kHz */
  I2S_AUDIO_FREQUENCY_44_1K   = 44100,   /* 44.1 kHz (CD quality) */
  I2S_AUDIO_FREQUENCY_48K     = 48000,   /* 48 kHz (DVD quality) */
  I2S_AUDIO_FREQUENCY_96K     = 96000,   /* 96 kHz (high-res) */
  I2S_AUDIO_FREQUENCY_192K    = 192000   /* 192 kHz (high-res) */
} i2s_audio_sampling_freq_type;
```

### Data/Channel Format

```c
typedef enum {
  I2S_DATA_16BIT_CHANNEL_16BIT = 0x01,  /* 16-bit data in 16-bit channel frame */
  I2S_DATA_16BIT_CHANNEL_32BIT = 0x02,  /* 16-bit data in 32-bit channel frame */
  I2S_DATA_24BIT_CHANNEL_32BIT = 0x03,  /* 24-bit data in 32-bit channel frame */
  I2S_DATA_32BIT_CHANNEL_32BIT = 0x04   /* 32-bit data in 32-bit channel frame */
} i2s_data_channel_format_type;
```

### Operation Mode

```c
typedef enum {
  I2S_MODE_SLAVE_TX  = 0x00,  /* Slave transmission mode */
  I2S_MODE_SLAVE_RX  = 0x01,  /* Slave reception mode */
  I2S_MODE_MASTER_TX = 0x02,  /* Master transmission mode */
  I2S_MODE_MASTER_RX = 0x03   /* Master reception mode */
} i2s_operation_mode_type;
```

### Clock Polarity

```c
typedef enum {
  I2S_CLOCK_POLARITY_LOW  = 0x00,  /* CK idle low */
  I2S_CLOCK_POLARITY_HIGH = 0x01   /* CK idle high */
} i2s_clock_polarity_type;
```

---

## Initialization Structure

```c
typedef struct {
  i2s_operation_mode_type        operation_mode;        /* Master/Slave TX/RX */
  i2s_audio_protocol_type        audio_protocol;        /* Phillips/MSB/LSB/PCM */
  i2s_audio_sampling_freq_type   audio_sampling_freq;   /* Sample rate */
  i2s_data_channel_format_type   data_channel_format;   /* Data bits / channel bits */
  i2s_clock_polarity_type        clock_polarity;        /* CK idle state */
  confirm_state                  mclk_output_enable;    /* MCLK output enable */
} i2s_init_type;
```

---

## Register Map (I2S-specific)

| Register | Offset | Description |
|----------|--------|-------------|
| I2SCTRL | 0x1C | I2S control register |
| I2SCLK | 0x20 | I2S clock prescaler register |

### I2SCTRL Register Fields

| Field | Bits | Description |
|-------|------|-------------|
| I2SCBN | [0] | Channel bit number (0: 16-bit, 1: 32-bit) |
| I2SDBN | [2:1] | Data bit number (00: 16-bit, 01: 24-bit, 10: 32-bit) |
| I2SCLKPOL | [3] | Clock polarity |
| STDSEL | [5:4] | Standard selection (00: Phillips, 01: MSB, 10: LSB, 11: PCM) |
| PCMFSSEL | [7] | PCM frame sync (0: short, 1: long) |
| OPERSEL | [9:8] | Operation mode (00: Slave TX, 01: Slave RX, 10: Master TX, 11: Master RX) |
| I2SEN | [10] | I2S enable |
| I2SMSEL | [11] | I2S mode select (0: SPI mode, 1: I2S mode) |

### I2SCLK Register Fields

| Field | Bits | Description |
|-------|------|-------------|
| I2SDIV_L | [7:0] | I2S linear prescaler (low 8 bits) |
| I2SODD | [8] | Odd factor for division |
| I2SMCLKOE | [9] | Master clock output enable |
| I2SDIV_H | [11:10] | I2S linear prescaler (high 2 bits) |

---

## Flags

| Flag | Description |
|------|-------------|
| SPI_I2S_RDBF_FLAG | Receive data buffer full |
| SPI_I2S_TDBE_FLAG | Transmit data buffer empty |
| I2S_ACS_FLAG | Audio channel state (0: Left, 1: Right) |
| I2S_TUERR_FLAG | Transmitter underload error |
| SPI_I2S_ROERR_FLAG | Receiver overrun error |
| SPI_I2S_BF_FLAG | Busy flag |

---

## Interrupts

| Interrupt | Description |
|-----------|-------------|
| SPI_I2S_TDBE_INT | Transmit data buffer empty |
| SPI_I2S_RDBF_INT | Receive data buffer full |
| SPI_I2S_ERROR_INT | Error interrupt (underrun, overrun) |

---

## DMA Requests

| DMA Request | DMAMUX ID | Description |
|-------------|-----------|-------------|
| SPI1_TX | DMAMUX_DMAREQ_ID_SPI1_TX | SPI1/I2S1 transmit |
| SPI1_RX | DMAMUX_DMAREQ_ID_SPI1_RX | SPI1/I2S1 receive |
| SPI2_TX | DMAMUX_DMAREQ_ID_SPI2_TX | SPI2/I2S2 transmit |
| SPI2_RX | DMAMUX_DMAREQ_ID_SPI2_RX | SPI2/I2S2 receive |
| I2S2_EXT_TX | DMAMUX_DMAREQ_ID_I2S2_EXT_TX | I2S2EXT transmit |
| I2S2_EXT_RX | DMAMUX_DMAREQ_ID_I2S2_EXT_RX | I2S2EXT receive |
| SPI3_TX | DMAMUX_DMAREQ_ID_SPI3_TX | SPI3/I2S3 transmit |
| SPI3_RX | DMAMUX_DMAREQ_ID_SPI3_RX | SPI3/I2S3 receive |
| I2S3_EXT_TX | DMAMUX_DMAREQ_ID_I2S3_EXT_TX | I2S3EXT transmit |
| I2S3_EXT_RX | DMAMUX_DMAREQ_ID_I2S3_EXT_RX | I2S3EXT receive |
| SPI4_TX | DMAMUX_DMAREQ_ID_SPI4_TX | SPI4/I2S4 transmit |
| SPI4_RX | DMAMUX_DMAREQ_ID_SPI4_RX | SPI4/I2S4 receive |

---

## API Reference

### Initialization Functions

```c
/**
  * @brief  Reset SPI/I2S peripheral
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @retval none
  */
void spi_i2s_reset(spi_type *spi_x);

/**
  * @brief  Initialize i2s_init_type structure with default values
  * @param  i2s_init_struct: pointer to structure
  * @retval none
  * @note   Default: Slave TX, Phillips, 2 (default freq), 16-bit/16-bit, low polarity, no MCLK
  */
void i2s_default_para_init(i2s_init_type* i2s_init_struct);

/**
  * @brief  Initialize I2S peripheral
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @param  i2s_init_struct: pointer to initialization structure
  * @retval none
  */
void i2s_init(spi_type* spi_x, i2s_init_type* i2s_init_struct);

/**
  * @brief  Enable/disable I2S peripheral
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2s_enable(spi_type* spi_x, confirm_state new_state);
```

### Data Transfer Functions

```c
/**
  * @brief  Transmit 16-bit data
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @param  tx_data: data to transmit (0x0000-0xFFFF)
  * @retval none
  */
void spi_i2s_data_transmit(spi_type* spi_x, uint16_t tx_data);

/**
  * @brief  Receive 16-bit data
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @retval Received 16-bit data
  */
uint16_t spi_i2s_data_receive(spi_type* spi_x);
```

### DMA Functions

```c
/**
  * @brief  Enable/disable I2S DMA transmitter
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void spi_i2s_dma_transmitter_enable(spi_type* spi_x, confirm_state new_state);

/**
  * @brief  Enable/disable I2S DMA receiver
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void spi_i2s_dma_receiver_enable(spi_type* spi_x, confirm_state new_state);
```

### Interrupt Functions

```c
/**
  * @brief  Enable/disable I2S interrupt
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @param  spi_i2s_int: SPI_I2S_TDBE_INT, SPI_I2S_RDBF_INT, or SPI_I2S_ERROR_INT
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void spi_i2s_interrupt_enable(spi_type* spi_x, uint32_t spi_i2s_int, confirm_state new_state);
```

### Flag Functions

```c
/**
  * @brief  Get I2S flag status
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @param  spi_i2s_flag: flag to check
  * @retval SET or RESET
  */
flag_status spi_i2s_flag_get(spi_type* spi_x, uint32_t spi_i2s_flag);

/**
  * @brief  Get I2S interrupt flag status
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @param  spi_i2s_flag: flag to check
  * @retval SET or RESET
  */
flag_status spi_i2s_interrupt_flag_get(spi_type* spi_x, uint32_t spi_i2s_flag);

/**
  * @brief  Clear I2S flag
  * @param  spi_x: SPI1, SPI2, SPI3, SPI4, I2S2EXT, or I2S3EXT
  * @param  spi_i2s_flag: flag(s) to clear
  * @retval none
  */
void spi_i2s_flag_clear(spi_type* spi_x, uint32_t spi_i2s_flag);
```

---

## Code Examples

### Example 1: Half-Duplex Master TX with DMA

```c
#include "at32f435_437.h"

#define BUFFER_SIZE  32

uint16_t i2s_tx_buffer[BUFFER_SIZE] = {
    0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F10,
    0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E, 0x1F20,
    0x2122, 0x2324, 0x2526, 0x2728, 0x292A, 0x2B2C, 0x2D2E, 0x2F30,
    0x3132, 0x3334, 0x3536, 0x3738, 0x393A, 0x3B3C, 0x3D3E, 0x3F40
};

/**
  * @brief  Configure GPIO for I2S3 (Master TX)
  */
void i2s_gpio_init(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    
    /* I2S3_WS (PA4) */
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_4;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE4, GPIO_MUX_6);
    
    /* I2S3_CK (PC10) */
    gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init(GPIOC, &gpio_init_struct);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE10, GPIO_MUX_6);
    
    /* I2S3_SD (PC12) */
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_12;
    gpio_init(GPIOC, &gpio_init_struct);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE12, GPIO_MUX_6);
    
    /* I2S3_MCK (PC7) - optional */
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_7;
    gpio_init(GPIOC, &gpio_init_struct);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE7, GPIO_MUX_6);
}

/**
  * @brief  Configure DMA for I2S3 TX
  */
void i2s_dma_init(void)
{
    dma_init_type dma_init_struct;
    
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    dmamux_enable(DMA1, TRUE);
    
    dma_reset(DMA1_CHANNEL1);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = BUFFER_SIZE;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init_struct.memory_base_addr = (uint32_t)i2s_tx_buffer;
    dma_init_struct.peripheral_base_addr = (uint32_t)&SPI3->dt;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    
    dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_SPI3_TX);
}

/**
  * @brief  Configure I2S3 as Master TX
  */
void i2s_config(void)
{
    i2s_init_type i2s_init_struct;
    
    crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, TRUE);
    
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = I2S_DATA_16BIT_CHANNEL_32BIT;
    i2s_init_struct.mclk_output_enable = TRUE;
    i2s_init_struct.audio_sampling_freq = I2S_AUDIO_FREQUENCY_48K;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = I2S_MODE_MASTER_TX;
    i2s_init(SPI3, &i2s_init_struct);
    
    /* Enable DMA TX */
    spi_i2s_dma_transmitter_enable(SPI3, TRUE);
}

int main(void)
{
    system_clock_config();
    
    i2s_gpio_init();
    i2s_dma_init();
    i2s_config();
    
    /* Start DMA transfer */
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    
    /* Enable I2S */
    i2s_enable(SPI3, TRUE);
    
    /* Wait for transfer complete */
    while(dma_flag_get(DMA1_FDT1_FLAG) == RESET);
    
    /* Wait for I2S idle */
    while(spi_i2s_flag_get(SPI3, SPI_I2S_BF_FLAG) != RESET);
    
    while(1);
}
```

---

### Example 2: Half-Duplex Slave RX with Interrupt

```c
#include "at32f435_437.h"

#define BUFFER_SIZE  32

uint16_t i2s_rx_buffer[BUFFER_SIZE];
volatile uint32_t rx_index = 0;

/**
  * @brief  Configure I2S2 as Slave RX
  */
void i2s_slave_init(void)
{
    i2s_init_type i2s_init_struct;
    
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
    
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = I2S_DATA_16BIT_CHANNEL_32BIT;
    i2s_init_struct.mclk_output_enable = FALSE;
    i2s_init_struct.audio_sampling_freq = I2S_AUDIO_FREQUENCY_48K;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = I2S_MODE_SLAVE_RX;
    i2s_init(SPI2, &i2s_init_struct);
    
    /* Enable RDBF interrupt */
    spi_i2s_interrupt_enable(SPI2, SPI_I2S_RDBF_INT, TRUE);
    
    /* Configure NVIC */
    nvic_irq_enable(SPI2_I2S2EXT_IRQn, 0, 0);
}

/**
  * @brief  SPI2/I2S2 interrupt handler
  */
void SPI2_I2S2EXT_IRQHandler(void)
{
    if(spi_i2s_interrupt_flag_get(SPI2, SPI_I2S_RDBF_FLAG) != RESET)
    {
        if(rx_index < BUFFER_SIZE)
        {
            i2s_rx_buffer[rx_index++] = spi_i2s_data_receive(SPI2);
        }
        else
        {
            /* Buffer full - disable interrupt */
            spi_i2s_interrupt_enable(SPI2, SPI_I2S_RDBF_INT, FALSE);
        }
    }
}

int main(void)
{
    system_clock_config();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    /* ... GPIO init for I2S2 pins ... */
    i2s_slave_init();
    
    /* Enable I2S */
    i2s_enable(SPI2, TRUE);
    
    /* Wait for all data received */
    while(rx_index < BUFFER_SIZE);
    
    /* Data is now in i2s_rx_buffer */
    while(1);
}
```

---

### Example 3: Full-Duplex with I2S2 + I2S2EXT

```c
#include "at32f435_437.h"

#define BUFFER_SIZE  32

uint16_t i2s_tx_buffer[BUFFER_SIZE];
uint16_t i2s_rx_buffer[BUFFER_SIZE];

/**
  * @brief  Configure full-duplex I2S: I2S2 (TX) + I2S2EXT (RX)
  */
void i2s_fullduplex_init(void)
{
    i2s_init_type i2s_init_struct;
    
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
    
    /* I2S2 as Master TX (main interface) */
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = I2S_DATA_16BIT_CHANNEL_32BIT;
    i2s_init_struct.mclk_output_enable = TRUE;
    i2s_init_struct.audio_sampling_freq = I2S_AUDIO_FREQUENCY_48K;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = I2S_MODE_MASTER_TX;
    i2s_init(SPI2, &i2s_init_struct);
    
    /* I2S2EXT as Slave RX (extension interface) */
    i2s_init_struct.operation_mode = I2S_MODE_SLAVE_RX;
    i2s_init(I2S2EXT, &i2s_init_struct);
    
    /* Enable DMA */
    spi_i2s_dma_transmitter_enable(SPI2, TRUE);
    spi_i2s_dma_receiver_enable(I2S2EXT, TRUE);
}

/**
  * @brief  Configure GPIO for full-duplex
  */
void i2s_fullduplex_gpio_init(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    
    /* I2S2_WS (PD0) */
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_0;
    gpio_init(GPIOD, &gpio_init_struct);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE0, GPIO_MUX_7);
    
    /* I2S2_CK (PD1) */
    gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
    gpio_init_struct.gpio_pins = GPIO_PINS_1;
    gpio_init(GPIOD, &gpio_init_struct);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE1, GPIO_MUX_6);
    
    /* I2S2_SD - TX (PD4) */
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_4;
    gpio_init(GPIOD, &gpio_init_struct);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE4, GPIO_MUX_6);
    
    /* I2S2EXT_SD - RX (PC2) */
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init(GPIOC, &gpio_init_struct);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE2, GPIO_MUX_6);
}

/**
  * @brief  Configure DMA for full-duplex
  */
void i2s_fullduplex_dma_init(void)
{
    dma_init_type dma_init_struct;
    
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    dmamux_enable(DMA1, TRUE);
    
    /* DMA for I2S2 TX */
    dma_reset(DMA1_CHANNEL1);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = BUFFER_SIZE;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init_struct.memory_base_addr = (uint32_t)i2s_tx_buffer;
    dma_init_struct.peripheral_base_addr = (uint32_t)&SPI2->dt;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_SPI2_TX);
    
    /* DMA for I2S2EXT RX */
    dma_reset(DMA1_CHANNEL2);
    dma_init_struct.memory_base_addr = (uint32_t)i2s_rx_buffer;
    dma_init_struct.peripheral_base_addr = (uint32_t)&I2S2EXT->dt;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init(DMA1_CHANNEL2, &dma_init_struct);
    dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_I2S2_EXT_RX);
}

int main(void)
{
    uint32_t i;
    
    system_clock_config();
    
    /* Fill TX buffer */
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        i2s_tx_buffer[i] = i;
    }
    
    i2s_fullduplex_gpio_init();
    i2s_fullduplex_dma_init();
    i2s_fullduplex_init();
    
    /* Enable DMA channels */
    dma_channel_enable(DMA1_CHANNEL2, TRUE);  /* RX first */
    dma_channel_enable(DMA1_CHANNEL1, TRUE);  /* TX */
    
    /* Enable I2S */
    i2s_enable(SPI2, TRUE);
    i2s_enable(I2S2EXT, TRUE);
    
    /* Wait for RX complete */
    while(dma_flag_get(DMA1_FDT2_FLAG) == RESET);
    
    /* Data transmitted and received */
    while(1);
}
```

---

### Example 4: Polling Mode Master TX

```c
#include "at32f435_437.h"

#define BUFFER_SIZE  32

uint16_t i2s_tx_buffer[BUFFER_SIZE] = {
    0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F10,
    0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E, 0x1F20,
    0x2122, 0x2324, 0x2526, 0x2728, 0x292A, 0x2B2C, 0x2D2E, 0x2F30,
    0x3132, 0x3334, 0x3536, 0x3738, 0x393A, 0x3B3C, 0x3D3E, 0x3F40
};

/**
  * @brief  Transmit data using polling
  */
void i2s_transmit_polling(uint16_t *data, uint32_t size)
{
    uint32_t i;
    
    for(i = 0; i < size; i++)
    {
        /* Wait for TX buffer empty */
        while(spi_i2s_flag_get(SPI3, SPI_I2S_TDBE_FLAG) == RESET);
        
        /* Send data */
        spi_i2s_data_transmit(SPI3, data[i]);
    }
    
    /* Wait for last byte transmitted */
    while(spi_i2s_flag_get(SPI3, SPI_I2S_BF_FLAG) != RESET);
}

int main(void)
{
    i2s_init_type i2s_init_struct;
    
    system_clock_config();
    
    /* ... GPIO init ... */
    
    crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, TRUE);
    
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = I2S_DATA_16BIT_CHANNEL_32BIT;
    i2s_init_struct.mclk_output_enable = TRUE;
    i2s_init_struct.audio_sampling_freq = I2S_AUDIO_FREQUENCY_48K;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = I2S_MODE_MASTER_TX;
    i2s_init(SPI3, &i2s_init_struct);
    
    i2s_enable(SPI3, TRUE);
    
    /* Transmit data */
    i2s_transmit_polling(i2s_tx_buffer, BUFFER_SIZE);
    
    while(1);
}
```

---

### Example 5: Switching Between SPI and I2S Modes

```c
#include "at32f435_437.h"

/**
  * @brief  Configure as I2S mode
  */
void config_i2s_mode(void)
{
    i2s_init_type i2s_init_struct;
    
    crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, TRUE);
    spi_i2s_reset(SPI3);
    
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = I2S_DATA_16BIT_CHANNEL_32BIT;
    i2s_init_struct.mclk_output_enable = FALSE;
    i2s_init_struct.audio_sampling_freq = I2S_AUDIO_FREQUENCY_48K;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = I2S_MODE_MASTER_TX;
    i2s_init(SPI3, &i2s_init_struct);
    
    i2s_enable(SPI3, TRUE);
}

/**
  * @brief  Configure as SPI mode
  */
void config_spi_mode(void)
{
    spi_init_type spi_init_struct;
    
    crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, TRUE);
    spi_i2s_reset(SPI3);  /* Reset to switch modes */
    
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_HALF_DUPLEX_TX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI3, &spi_init_struct);
    
    spi_enable(SPI3, TRUE);
}

int main(void)
{
    system_clock_config();
    
    /* ... GPIO init ... */
    
    /* Start with I2S mode */
    config_i2s_mode();
    /* ... do I2S communication ... */
    
    /* Disable and reset before switching */
    i2s_enable(SPI3, FALSE);
    spi_i2s_reset(SPI3);
    
    /* Switch to SPI mode */
    config_spi_mode();
    /* ... do SPI communication ... */
    
    while(1);
}
```

---

## Audio Protocol Timing

### I2S Phillips Standard

```
     ┌─────┐     ┌─────┐     ┌─────┐     ┌─────┐
CK ──┘     └─────┘     └─────┘     └─────┘     └─────
          ┌─────────────────────┐
WS ───────┘                     └───────────────────
     │◄── Left Channel ──────►│◄── Right Channel ──►│
SD ──┼─D15─┼─D14─┼─...─┼─D0──┼─D15─┼─D14─┼─...─┼─D0──┼
     │     │     │     │     │     │     │     │     │
```
- Data changes on falling edge of CK
- WS low = Left channel, WS high = Right channel
- MSB first, one bit delay after WS transition

### MSB-Justified Standard

```
     ┌─────┐     ┌─────┐     ┌─────┐     ┌─────┐
CK ──┘     └─────┘     └─────┘     └─────┘     └─────
     ┌───────────────────────┐
WS ──┘                       └───────────────────────
     │◄── Left Channel ─────►│◄── Right Channel ───►│
SD ──┼─D15─┼─D14─┼─...─┼─D0──┼─D15─┼─D14─┼─...─┼─D0──┼
```
- No bit delay after WS transition
- Data aligned to MSB

### LSB-Justified Standard

```
     ┌─────┐     ┌─────┐     ┌─────┐     ┌─────┐
CK ──┘     └─────┘     └─────┘     └─────┘     └─────
     ┌───────────────────────┐
WS ──┘                       └───────────────────────
     │◄── Left Channel ─────►│◄── Right Channel ───►│
SD ──┼──...──┼─D15─┼─...─┼─D0┼──...──┼─D15─┼─...─┼─D0┼
```
- Data aligned to LSB (end of channel frame)

---

## Pin Configuration

### SPI2/I2S2

| Pin | Function | GPIO MUX |
|-----|----------|----------|
| PB10 | I2S2_CK | GPIO_MUX_5 |
| PB12 | I2S2_WS | GPIO_MUX_5 |
| PB15 | I2S2_SD | GPIO_MUX_5 |
| PC2 | I2S2EXT_SD | GPIO_MUX_6 |
| PD0 | I2S2_WS | GPIO_MUX_7 |
| PD1 | I2S2_CK | GPIO_MUX_6 |
| PD4 | I2S2_SD | GPIO_MUX_6 |

### SPI3/I2S3

| Pin | Function | GPIO MUX |
|-----|----------|----------|
| PA4 | I2S3_WS | GPIO_MUX_6 |
| PC7 | I2S3_MCK | GPIO_MUX_6 |
| PC10 | I2S3_CK | GPIO_MUX_6 |
| PC11 | I2S3EXT_SD | GPIO_MUX_5 |
| PC12 | I2S3_SD | GPIO_MUX_6 |

---

## Configuration Checklist

### Half-Duplex Configuration
- [ ] Enable GPIO and SPI peripheral clocks
- [ ] Configure I2S pins (WS, CK, SD, optionally MCK)
- [ ] Call `i2s_default_para_init()` and set parameters
- [ ] Call `i2s_init()` with configured structure
- [ ] For DMA: configure DMA channel and DMAMUX
- [ ] For interrupts: enable NVIC and interrupt sources
- [ ] Enable I2S with `i2s_enable()`

### Full-Duplex Configuration
- [ ] Configure main I2S (SPI2 or SPI3) as Master TX or Master RX
- [ ] Configure extension (I2S2EXT or I2S3EXT) as Slave RX or Slave TX
- [ ] Configure separate DMA channels for main and extension
- [ ] Configure EXT_SD pin in addition to standard pins
- [ ] Enable both main and extension interfaces

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| No audio output | Wrong pin mux | Verify GPIO MUX configuration |
| | Clock not running | Check MCLK output, verify clock settings |
| | Wrong protocol | Match protocol with codec (Phillips/MSB/LSB) |
| Distorted audio | Sample rate mismatch | Verify audio_sampling_freq matches source |
| | Buffer underrun | Use DMA for continuous streaming |
| Left/Right swapped | WS polarity | Check I2S_CLOCK_POLARITY setting |
| Only left channel | Half-duplex mode | For full-duplex, use I2SxEXT |
| DMA not working | Wrong DMAMUX ID | Use correct SPI/I2S/EXT_TX/EXT_RX ID |

---

## Related Peripherals

| Peripheral | Relationship |
|------------|-------------|
| [SPI](SPI_Serial_Peripheral_Interface.md) | Shares hardware with I2S |
| [GPIO](GPIO_General_Purpose_IO.md) | Pin configuration |
| [CRM](CRM_Clock_Reset_Management.md) | Clock enable and I2S clock source |
| [DMA](DMA_Direct_Memory_Access.md) | Audio data streaming |
| [DMAMUX](DMA_Direct_Memory_Access.md) | DMA request routing |

---

## References

- AT32F435/437 Reference Manual - Chapter: SPI/I2S
- AT32F435/437 Datasheet - I2S specifications and pin mapping
- I2S Bus Specification (Philips Semiconductors)
- Application Note AN0098 - I2S Audio Interface Guide

