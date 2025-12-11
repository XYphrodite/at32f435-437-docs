---
title: "DAC - Digital to Analog Converter Peripheral"
type: "peripheral-documentation"
purpose: "context7-knowledge-source"
mcu_family: "AT32F435/437"
peripheral: "DAC"
version: "1.0.0"
last_updated: "2024-12-11"
tags:
  - dac
  - analog
  - waveform
  - dma
  - triangle-wave
  - noise-generator
  - signal-generation
  - peripheral
  - context7
related_peripherals:
  - DMA
  - TMR
  - GPIO
  - DMAMUX
---

# DAC - Digital to Analog Converter Peripheral

## Overview

The **Digital to Analog Converter (DAC)** peripheral in AT32F435/437 MCUs provides high-quality digital-to-analog conversion for generating analog signals, waveforms, and DC voltage levels from digital data.

**Key Features:**
- 🎛️ **2 independent DAC channels** (DAC1, DAC2)
- 📊 **12-bit resolution** (with 8-bit mode support)
- ⚡ **High-speed conversion** with hardware triggering
- 🔄 **DMA support** for automatic waveform generation
- 📈 **Built-in waveform generators** (Triangle, Noise)
- ⚙️ **Multiple trigger sources** (Timers, Software, External)
- 🔗 **Dual-channel mode** for synchronized output
- 📤 **Output buffer** for driving external loads
- 🛡️ **DMA underrun detection** with interrupt support

---

## Architecture

```
                          ┌─────────────────────────────────────────────────────────┐
 Digital Data Input        │                     DAC Peripheral                      │
 ─────────────────         │                                                         │
                           │  ┌─────────────────────────────────────────────────┐   │
 DAC1 Data Registers ─────►│  │ DAC1 Channel                                    │   │
 (D1DTH12R/L, D1DTH8R)     │  │ ┌──────────┐    ┌──────────────┐    ┌────────┐ │   │
                           │  │ │ Data     │    │ 12-bit       │    │ Output │ │   │──► PA4
 ─────────────────────────►│  │ │ Register │───►│ DAC Core     │───►│ Buffer │ │   │
 Trigger Sources           │  │ └──────────┘    └──────────────┘    └────────┘ │   │
 (TMR, SW, EXINT)          │  │       ↑                ↑                       │   │
                           │  │       │         ┌──────┴──────┐                │   │
                           │  │       │         │ Wave Gen    │                │   │
                           │  │       │         │ Triangle/   │                │   │
                           │  │       │         │ Noise       │                │   │
                           │  │       │         └─────────────┘                │   │
                           │  └───────┼────────────────────────────────────────┘   │
                           │          │                                             │
                           │  ┌───────┼────────────────────────────────────────┐   │
 DAC2 Data Registers ─────►│  │ DAC2 Channel                                    │   │
 (D2DTH12R/L, D2DTH8R)     │  │ ┌──────────┐    ┌──────────────┐    ┌────────┐ │   │
                           │  │ │ Data     │    │ 12-bit       │    │ Output │ │   │──► PA5
 ─────────────────────────►│  │ │ Register │───►│ DAC Core     │───►│ Buffer │ │   │
 Trigger Sources           │  │ └──────────┘    └──────────────┘    └────────┘ │   │
                           │  └─────────────────────────────────────────────────┘   │
                           │                                                         │
 DMA Request ◄─────────────│  DMAMUX_DMAREQ_ID_DAC1, DMAMUX_DMAREQ_ID_DAC2          │
                           └─────────────────────────────────────────────────────────┘
```

---

## Hardware Resources

### DAC Channels

| Channel | Base Address | Output Pin | DMA Request ID |
|---------|--------------|------------|----------------|
| DAC1 | 0x40007400 | PA4 | DMAMUX_DMAREQ_ID_DAC1 |
| DAC2 | 0x40007400 | PA5 | DMAMUX_DMAREQ_ID_DAC2 |

### Data Holding Registers

| Register | Address | Description |
|----------|---------|-------------|
| D1DTH12R | 0x40007408 | DAC1 12-bit right-aligned data |
| D1DTH12L | 0x4000740C | DAC1 12-bit left-aligned data |
| D1DTH8R | 0x40007410 | DAC1 8-bit right-aligned data |
| D2DTH12R | 0x40007414 | DAC2 12-bit right-aligned data |
| D2DTH12L | 0x40007418 | DAC2 12-bit left-aligned data |
| D2DTH8R | 0x4000741C | DAC2 8-bit right-aligned data |
| DDTH12R | 0x40007420 | Dual DAC 12-bit right-aligned data |
| DDTH12L | 0x40007424 | Dual DAC 12-bit left-aligned data |
| DDTH8R | 0x40007428 | Dual DAC 8-bit right-aligned data |

---

## Resolution and Data Alignment

### Data Formats

| Format | Bits | Max Value | Alignment |
|--------|------|-----------|-----------|
| 12-bit Right | [11:0] | 4095 | Bits 0-11 |
| 12-bit Left | [15:4] | 4095 | Bits 4-15 |
| 8-bit Right | [7:0] | 255 | Bits 0-7 |

### Output Voltage Calculation

```
V_out = V_REF × (DAC_DATA / 4095)    // For 12-bit mode
V_out = V_REF × (DAC_DATA / 255)     // For 8-bit mode
```

**Example:** With V_REF = 3.3V and DAC_DATA = 2048:
```
V_out = 3.3V × (2048 / 4095) ≈ 1.65V
```

---

## Trigger Sources

The DAC conversion can be triggered by multiple sources:

| Trigger | Enum Value | Description |
|---------|------------|-------------|
| Timer 6 TRGOUT | DAC_TMR6_TRGOUT_EVENT | Timer 6 overflow event |
| Timer 8 TRGOUT | DAC_TMR8_TRGOUT_EVENT | Timer 8 overflow event |
| Timer 7 TRGOUT | DAC_TMR7_TRGOUT_EVENT | Timer 7 overflow event |
| Timer 5 TRGOUT | DAC_TMR5_TRGOUT_EVENT | Timer 5 overflow event |
| Timer 2 TRGOUT | DAC_TMR2_TRGOUT_EVENT | Timer 2 overflow event |
| Timer 4 TRGOUT | DAC_TMR4_TRGOUT_EVENT | Timer 4 overflow event |
| External Line 9 | DAC_EXTERNAL_INTERRUPT_LINE_9 | EXINT9 signal |
| Software | DAC_SOFTWARE_TRIGGER | Manual trigger via register |

---

## Wave Generation Modes

### No Wave Generation (DAC_WAVE_GENERATE_NONE)

Direct DAC output from data register. Used for:
- Arbitrary waveform generation via DMA
- Static DC voltage output
- Custom signal generation

### Triangle Wave (DAC_WAVE_GENERATE_TRIANGLE)

Hardware-generated triangle waveform. The amplitude is controlled by the mask/amplitude selection.

**Triangle Wave Generation Formula:**
```
Output = Base_Value + Triangle_Counter

Where Triangle_Counter:
- Increments on each trigger until max amplitude
- Then decrements until 0
- Repeats cycle
```

### Noise Wave (DAC_WAVE_GENERATE_NOISE)

Hardware-generated pseudo-random noise using LFSR (Linear Feedback Shift Register).

**Noise Generation:**
- Uses internal LFSR generator
- Amplitude controlled by mask bits
- Output = Base_Value + (LFSR_Value & Mask)

---

## Amplitude/Mask Selection

The `dac_mask_amplitude_type` controls both:
- **Triangle mode:** Maximum amplitude of triangle wave
- **Noise mode:** LFSR bit mask (unmasked bits)

| Enum | Triangle Amplitude | LFSR Mask | Max Output Addition |
|------|-------------------|-----------|---------------------|
| DAC_LSFR_BIT0_AMPLITUDE_1 | 1 | bit[0] | 1 |
| DAC_LSFR_BIT10_AMPLITUDE_3 | 3 | bit[1:0] | 3 |
| DAC_LSFR_BIT20_AMPLITUDE_7 | 7 | bit[2:0] | 7 |
| DAC_LSFR_BIT30_AMPLITUDE_15 | 15 | bit[3:0] | 15 |
| DAC_LSFR_BIT40_AMPLITUDE_31 | 31 | bit[4:0] | 31 |
| DAC_LSFR_BIT50_AMPLITUDE_63 | 63 | bit[5:0] | 63 |
| DAC_LSFR_BIT60_AMPLITUDE_127 | 127 | bit[6:0] | 127 |
| DAC_LSFR_BIT70_AMPLITUDE_255 | 255 | bit[7:0] | 255 |
| DAC_LSFR_BIT80_AMPLITUDE_511 | 511 | bit[8:0] | 511 |
| DAC_LSFR_BIT90_AMPLITUDE_1023 | 1023 | bit[9:0] | 1023 |
| DAC_LSFR_BITA0_AMPLITUDE_2047 | 2047 | bit[10:0] | 2047 |
| DAC_LSFR_BITB0_AMPLITUDE_4095 | 4095 | bit[11:0] | 4095 |

---

## API Reference

### Initialization Functions

```c
/**
 * @brief  Reset DAC peripheral to default state
 */
void dac_reset(void);

/**
 * @brief  Enable or disable DAC channel
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 * @param  new_state: TRUE to enable, FALSE to disable
 */
void dac_enable(dac_select_type dac_select, confirm_state new_state);

/**
 * @brief  Enable or disable DAC output buffer
 * @note   Buffer provides lower output impedance
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 * @param  new_state: TRUE to enable buffer, FALSE to disable
 */
void dac_output_buffer_enable(dac_select_type dac_select, confirm_state new_state);
```

### Trigger Configuration

```c
/**
 * @brief  Enable or disable DAC trigger
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 * @param  new_state: TRUE to enable trigger, FALSE for auto conversion
 */
void dac_trigger_enable(dac_select_type dac_select, confirm_state new_state);

/**
 * @brief  Select DAC trigger source
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 * @param  dac_trigger_source: Trigger source enum
 */
void dac_trigger_select(dac_select_type dac_select, dac_trigger_type dac_trigger_source);

/**
 * @brief  Generate software trigger for single channel
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 */
void dac_software_trigger_generate(dac_select_type dac_select);

/**
 * @brief  Generate software trigger for both channels simultaneously
 */
void dac_dual_software_trigger_generate(void);
```

### Wave Generation

```c
/**
 * @brief  Configure wave generation mode
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 * @param  dac_wave: DAC_WAVE_GENERATE_NONE, DAC_WAVE_GENERATE_NOISE,
 *                   or DAC_WAVE_GENERATE_TRIANGLE
 */
void dac_wave_generate(dac_select_type dac_select, dac_wave_type dac_wave);

/**
 * @brief  Select mask/amplitude for wave generation
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 * @param  dac_mask_amplitude: Amplitude/mask selection enum
 */
void dac_mask_amplitude_select(dac_select_type dac_select, 
                                dac_mask_amplitude_type dac_mask_amplitude);
```

### Data Functions

```c
/**
 * @brief  Set DAC1 output data
 * @param  dac1_aligned: Data alignment (DAC1_12BIT_RIGHT, DAC1_12BIT_LEFT, DAC1_8BIT_RIGHT)
 * @param  dac1_data: 12-bit or 8-bit data value
 */
void dac_1_data_set(dac1_aligned_data_type dac1_aligned, uint16_t dac1_data);

/**
 * @brief  Set DAC2 output data
 * @param  dac2_aligned: Data alignment (DAC2_12BIT_RIGHT, DAC2_12BIT_LEFT, DAC2_8BIT_RIGHT)
 * @param  dac2_data: 12-bit or 8-bit data value
 */
void dac_2_data_set(dac2_aligned_data_type dac2_aligned, uint16_t dac2_data);

/**
 * @brief  Set dual DAC output data simultaneously
 * @param  dac_dual: Data alignment for dual mode
 * @param  data1: DAC1 data value
 * @param  data2: DAC2 data value
 */
void dac_dual_data_set(dac_dual_data_type dac_dual, uint16_t data1, uint16_t data2);

/**
 * @brief  Get DAC output data register value
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 * @return Current output data value (12-bit)
 */
uint16_t dac_data_output_get(dac_select_type dac_select);
```

### DMA Functions

```c
/**
 * @brief  Enable or disable DAC DMA request
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 * @param  new_state: TRUE to enable DMA, FALSE to disable
 */
void dac_dma_enable(dac_select_type dac_select, confirm_state new_state);

/**
 * @brief  Enable or disable DMA underrun interrupt
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 * @param  new_state: TRUE to enable interrupt, FALSE to disable
 */
void dac_udr_enable(dac_select_type dac_select, confirm_state new_state);

/**
 * @brief  Get DMA underrun flag status
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 * @return SET if underrun occurred, RESET otherwise
 */
flag_status dac_udr_flag_get(dac_select_type dac_select);

/**
 * @brief  Clear DMA underrun flag
 * @param  dac_select: DAC1_SELECT or DAC2_SELECT
 */
void dac_udr_flag_clear(dac_select_type dac_select);
```

---

## Code Examples

### Example 1: Basic DC Voltage Output

Simple DAC output of a constant voltage level.

```c
#include "at32f435_437.h"

void dac_basic_init(void)
{
    gpio_init_type gpio_init_struct = {0};
    
    /* Enable peripheral clocks */
    crm_periph_clock_enable(CRM_DAC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA4 as analog (DAC1 output) */
    gpio_init_struct.gpio_pins = GPIO_PINS_4;
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Disable output buffer for direct output */
    dac_output_buffer_enable(DAC1_SELECT, FALSE);
    
    /* Enable DAC1 */
    dac_enable(DAC1_SELECT, TRUE);
    
    /* Set output to ~1.65V (half of 3.3V) */
    dac_1_data_set(DAC1_12BIT_RIGHT, 2048);
}

void dac_set_voltage(float voltage, float vref)
{
    uint16_t dac_value;
    
    /* Clamp voltage to valid range */
    if (voltage < 0) voltage = 0;
    if (voltage > vref) voltage = vref;
    
    /* Calculate DAC value */
    dac_value = (uint16_t)((voltage / vref) * 4095);
    
    dac_1_data_set(DAC1_12BIT_RIGHT, dac_value);
}
```

### Example 2: Triangle Wave Generation

Hardware-generated triangle waveform on two channels with different amplitudes.

```c
#include "at32f435_437.h"

void dac_triangle_wave_init(void)
{
    gpio_init_type gpio_init_struct = {0};
    crm_clocks_freq_type crm_clocks;
    
    /* Enable peripheral clocks */
    crm_periph_clock_enable(CRM_DAC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA4, PA5 as analog outputs */
    gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Get system clock for timer calculation */
    crm_clocks_freq_get(&crm_clocks);
    
    /* Configure Timer 2 for 10kHz trigger rate */
    /* Formula: (sysclk / prescaler) / period = trigger_freq */
    tmr_base_init(TMR2, 99, (crm_clocks.sclk_freq / 1000000 - 1));
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    tmr_primary_mode_select(TMR2, TMR_PRIMARY_SEL_OVERFLOW);
    
    /* Configure DAC1 - Triangle wave, amplitude 1023 */
    dac_trigger_select(DAC1_SELECT, DAC_TMR2_TRGOUT_EVENT);
    dac_trigger_enable(DAC1_SELECT, TRUE);
    dac_wave_generate(DAC1_SELECT, DAC_WAVE_GENERATE_TRIANGLE);
    dac_mask_amplitude_select(DAC1_SELECT, DAC_LSFR_BIT90_AMPLITUDE_1023);
    dac_output_buffer_enable(DAC1_SELECT, FALSE);
    
    /* Configure DAC2 - Triangle wave, amplitude 2047 */
    dac_trigger_select(DAC2_SELECT, DAC_TMR2_TRGOUT_EVENT);
    dac_trigger_enable(DAC2_SELECT, TRUE);
    dac_wave_generate(DAC2_SELECT, DAC_WAVE_GENERATE_TRIANGLE);
    dac_mask_amplitude_select(DAC2_SELECT, DAC_LSFR_BITA0_AMPLITUDE_2047);
    dac_output_buffer_enable(DAC2_SELECT, FALSE);
    
    /* Set base value (DC offset) for both channels */
    dac_dual_data_set(DAC_DUAL_12BIT_RIGHT, 0x100, 0x100);
    
    /* Enable DAC channels */
    dac_enable(DAC1_SELECT, TRUE);
    dac_enable(DAC2_SELECT, TRUE);
    
    /* Start timer */
    tmr_counter_enable(TMR2, TRUE);
}
```

### Example 3: Noise Wave Generation

Pseudo-random noise output using software triggers.

```c
#include "at32f435_437.h"

void dac_noise_wave_init(void)
{
    gpio_init_type gpio_init_struct = {0};
    
    /* Enable peripheral clocks */
    crm_periph_clock_enable(CRM_DAC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA4 as analog output */
    gpio_init_struct.gpio_pins = GPIO_PINS_4;
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Configure DAC1 for noise generation */
    dac_trigger_select(DAC1_SELECT, DAC_SOFTWARE_TRIGGER);
    dac_trigger_enable(DAC1_SELECT, TRUE);
    dac_wave_generate(DAC1_SELECT, DAC_WAVE_GENERATE_NOISE);
    dac_mask_amplitude_select(DAC1_SELECT, DAC_LSFR_BITB0_AMPLITUDE_4095);
    dac_output_buffer_enable(DAC1_SELECT, TRUE);
    
    /* Enable DAC1 */
    dac_enable(DAC1_SELECT, TRUE);
}

void dac_noise_generate(void)
{
    /* Generate new noise sample on each call */
    dac_software_trigger_generate(DAC1_SELECT);
}

/* Main loop example */
void noise_main_loop(void)
{
    while (1)
    {
        dac_software_trigger_generate(DAC1_SELECT);
        delay_us(1);  /* Adjust for desired noise bandwidth */
    }
}
```

### Example 4: DMA Sine Wave Generation

Automatic sine wave output using DMA circular mode.

```c
#include "at32f435_437.h"

/* 32-sample sine wave lookup table (12-bit values) */
const uint16_t sine_wave_12bit[32] = {
    2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056,
    4095, 4056, 3939, 3750, 3495, 3185, 2831, 2447,
    2047, 1647, 1263, 909,  599,  344,  155,  38,
    0,    38,   155,  344,  599,  909,  1263, 1647
};

/* Dual channel sine wave buffer */
uint32_t dual_sine_buffer[32];

void dac_dma_sinewave_init(void)
{
    gpio_init_type gpio_init_struct = {0};
    dma_init_type dma_init_struct;
    crm_clocks_freq_type crm_clocks;
    uint32_t idx;
    
    /* Enable peripheral clocks */
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DAC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA4, PA5 as analog */
    gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Fill dual channel buffer: DAC2 in upper 16 bits, DAC1 in lower 16 bits */
    for (idx = 0; idx < 32; idx++)
    {
        dual_sine_buffer[idx] = (sine_wave_12bit[idx] << 16) | sine_wave_12bit[idx];
    }
    
    /* Get system clock */
    crm_clocks_freq_get(&crm_clocks);
    
    /* Configure Timer 2 for 10kHz trigger (sine freq = 10000/32 = 312.5 Hz) */
    tmr_base_init(TMR2, 99, (crm_clocks.sclk_freq / 1000000 - 1));
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    tmr_primary_mode_select(TMR2, TMR_PRIMARY_SEL_OVERFLOW);
    
    /* Configure DAC channels */
    dac_trigger_select(DAC1_SELECT, DAC_TMR2_TRGOUT_EVENT);
    dac_trigger_select(DAC2_SELECT, DAC_TMR2_TRGOUT_EVENT);
    dac_trigger_enable(DAC1_SELECT, TRUE);
    dac_trigger_enable(DAC2_SELECT, TRUE);
    dac_wave_generate(DAC1_SELECT, DAC_WAVE_GENERATE_NONE);
    dac_wave_generate(DAC2_SELECT, DAC_WAVE_GENERATE_NONE);
    dac_output_buffer_enable(DAC1_SELECT, FALSE);
    dac_output_buffer_enable(DAC2_SELECT, FALSE);
    dac_dma_enable(DAC1_SELECT, TRUE);
    dac_dma_enable(DAC2_SELECT, TRUE);
    
    /* Configure DMA for dual DAC */
    dma_reset(DMA1_CHANNEL1);
    dma_init_struct.buffer_size = 32;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)dual_sine_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = DAC_DUAL_12BIT_RIGHT;  /* 0x40007420 */
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = TRUE;  /* Circular mode */
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    
    /* Configure DMAMUX */
    dmamux_enable(DMA1, TRUE);
    dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_DAC2);
    
    /* Enable DMA channel */
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    
    /* Enable DAC channels */
    dac_enable(DAC1_SELECT, TRUE);
    dac_enable(DAC2_SELECT, TRUE);
    
    /* Start timer - waveform generation begins automatically */
    tmr_counter_enable(TMR2, TRUE);
}
```

### Example 5: DMA Escalator (Staircase) Waveform

8-bit escalator waveform using single channel DMA.

```c
#include "at32f435_437.h"

/* 6-step escalator pattern */
const uint8_t escalator_8bit[6] = {0x00, 0x33, 0x66, 0x99, 0xCC, 0xFF};

void dac_dma_escalator_init(void)
{
    gpio_init_type gpio_init_struct = {0};
    dma_init_type dma_init_struct;
    crm_clocks_freq_type crm_clocks;
    
    /* Enable peripheral clocks */
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DAC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA4 as analog */
    gpio_init_struct.gpio_pins = GPIO_PINS_4;
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Get system clock */
    crm_clocks_freq_get(&crm_clocks);
    
    /* Configure Timer 2 for 1kHz trigger */
    tmr_base_init(TMR2, 999, (crm_clocks.sclk_freq / 1000000 - 1));
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    tmr_primary_mode_select(TMR2, TMR_PRIMARY_SEL_OVERFLOW);
    
    /* Configure DAC1 */
    dac_trigger_select(DAC1_SELECT, DAC_TMR2_TRGOUT_EVENT);
    dac_trigger_enable(DAC1_SELECT, TRUE);
    dac_wave_generate(DAC1_SELECT, DAC_WAVE_GENERATE_NONE);
    dac_output_buffer_enable(DAC1_SELECT, FALSE);
    dac_dma_enable(DAC1_SELECT, TRUE);
    
    /* Configure DMA for 8-bit data */
    dma_reset(DMA1_CHANNEL2);
    dma_init_struct.buffer_size = 6;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)escalator_8bit;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = DAC1_8BIT_RIGHT;  /* 0x40007410 */
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = TRUE;
    dma_init(DMA1_CHANNEL2, &dma_init_struct);
    
    /* Configure DMAMUX */
    dmamux_enable(DMA1, TRUE);
    dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_DAC1);
    
    /* Enable DMA and DAC */
    dma_channel_enable(DMA1_CHANNEL2, TRUE);
    dac_enable(DAC1_SELECT, TRUE);
    
    /* Start timer */
    tmr_counter_enable(TMR2, TRUE);
}
```

### Example 6: Dual Channel Square Wave

Synchronized square wave on both DAC channels.

```c
#include "at32f435_437.h"

/* Square wave: high and low values for both channels */
uint32_t dual_square_12bit[2] = {
    (0xFFF | (0xFFF << 16)),  /* Both channels high */
    0                          /* Both channels low */
};

void dac_dma_squarewave_init(void)
{
    gpio_init_type gpio_init_struct = {0};
    dma_init_type dma_init_struct;
    
    /* Enable peripheral clocks */
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DAC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA4, PA5 as analog */
    gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Configure both DAC channels with software trigger */
    dac_trigger_select(DAC1_SELECT, DAC_SOFTWARE_TRIGGER);
    dac_trigger_select(DAC2_SELECT, DAC_SOFTWARE_TRIGGER);
    dac_trigger_enable(DAC1_SELECT, TRUE);
    dac_trigger_enable(DAC2_SELECT, TRUE);
    dac_wave_generate(DAC1_SELECT, DAC_WAVE_GENERATE_NONE);
    dac_wave_generate(DAC2_SELECT, DAC_WAVE_GENERATE_NONE);
    dac_output_buffer_enable(DAC1_SELECT, TRUE);
    dac_output_buffer_enable(DAC2_SELECT, TRUE);
    dac_dma_enable(DAC1_SELECT, TRUE);
    dac_dma_enable(DAC2_SELECT, TRUE);
    
    /* Configure DMA for dual DAC */
    dma_reset(DMA1_CHANNEL1);
    dma_init_struct.buffer_size = 2;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)dual_square_12bit;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = DAC_DUAL_12BIT_RIGHT;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = TRUE;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    
    /* Configure DMAMUX */
    dmamux_enable(DMA1, TRUE);
    dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_DAC2);
    
    /* Enable DMA and DAC */
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    dac_enable(DAC1_SELECT, TRUE);
    dac_enable(DAC2_SELECT, TRUE);
}

void squarewave_main_loop(void)
{
    while (1)
    {
        delay_us(100);  /* Square wave period = 200us = 5kHz */
        dac_software_trigger_generate(DAC1_SELECT);
        dac_software_trigger_generate(DAC2_SELECT);
    }
}
```

---

## Configuration Checklist

### Basic DAC Setup
- [ ] Enable DAC peripheral clock: `crm_periph_clock_enable(CRM_DAC_PERIPH_CLOCK, TRUE)`
- [ ] Enable GPIO clock for output pin(s)
- [ ] Configure output pin(s) as analog mode
- [ ] Configure output buffer (enable/disable based on load requirements)
- [ ] Enable DAC channel(s): `dac_enable(DAC_SELECT, TRUE)`

### Trigger-Based Output
- [ ] Enable timer peripheral clock (if using timer trigger)
- [ ] Configure timer for desired trigger rate
- [ ] Select trigger source: `dac_trigger_select()`
- [ ] Enable trigger: `dac_trigger_enable(DAC_SELECT, TRUE)`
- [ ] Start timer (if applicable)

### DMA-Based Waveform
- [ ] Enable DMA peripheral clock
- [ ] Prepare waveform data buffer
- [ ] Configure DMA channel (direction, data width, circular mode)
- [ ] Configure DMAMUX for DAC DMA request
- [ ] Enable DMA: `dac_dma_enable(DAC_SELECT, TRUE)`
- [ ] Enable DMA channel
- [ ] Consider DMA underrun interrupt for error handling

### Wave Generation (Triangle/Noise)
- [ ] Select wave generation mode: `dac_wave_generate()`
- [ ] Select amplitude/mask: `dac_mask_amplitude_select()`
- [ ] Set base value (DC offset) in data register

---

## Troubleshooting

### No Output Signal

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| No output voltage | DAC not enabled | Call `dac_enable()` |
| Output stuck at 0V | Wrong GPIO config | Set GPIO to analog mode |
| Output stuck at Vref | Data register overflow | Check 12-bit max value (4095) |

### Incorrect Waveform

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Wrong frequency | Timer misconfigured | Verify timer prescaler and period |
| Clipped waveform | Amplitude + offset > 4095 | Reduce base value or amplitude |
| Noisy output | Output buffer disabled | Enable output buffer for high-Z loads |

### DMA Issues

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| No DMA transfer | DMAMUX not configured | Call `dmamux_enable()` and `dmamux_init()` |
| Glitches in waveform | DMA underrun | Increase trigger period or check buffer |
| Wrong channel | Incorrect DMA request ID | Use DMAMUX_DMAREQ_ID_DAC1 or DAC2 |

### Buffer Considerations

| Load Type | Buffer Setting | Notes |
|-----------|---------------|-------|
| High impedance (>1MΩ) | Disabled | Direct output, lower power |
| Low impedance (<1kΩ) | Enabled | Buffer provides drive strength |
| Capacitive load | Enabled | Buffer helps with stability |

---

## Performance Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Resolution | 12-bit | 4096 levels |
| Settling time | ~3µs | To 0.5 LSB |
| Output impedance (buffer off) | ~15kΩ | High-Z output |
| Output impedance (buffer on) | <1Ω | Low-Z output |
| Max output current (buffer) | ±1mA | Per channel |
| Output voltage range | 0 to Vref | Typically 0-3.3V |

---

## Related Documentation

### Official Artery Documents
- **[Reference Manual](../RM_AT32F435_437_V2.07_EN.pdf)** - DAC Chapter
- **[Datasheet](../DS_AT32F435_437_V2.20_EN.pdf)** - Electrical specifications
- **[AN0101]** - DAC Application Note (referenced in examples)

### Related Peripherals
- **[DMA](DMA_Direct_Memory_Access.md)** - For automatic data transfer
- **[TMR](TMR_Timer.md)** - For trigger signal generation
- **[GPIO](GPIO_General_Purpose_IO.md)** - For pin configuration
- **[ADC](ADC_Analog_to_Digital_Converter.md)** - Complementary peripheral

### Example Projects
- `project/at_start_f435/examples/dac/two_dac_trianglewave/`
- `project/at_start_f435/examples/dac/double_mode_dma_sinewave/`
- `project/at_start_f435/examples/dac/one_dac_dma_escalator/`
- `project/at_start_f435/examples/dac/one_dac_noisewave/`
- `project/at_start_f435/examples/dac/double_mode_dma_squarewave/`

---

**Status:** ✅ Complete  
**Last Updated:** December 2024  
**Firmware Library Version:** v2.2.2


