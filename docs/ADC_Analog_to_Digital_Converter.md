---
title: "ADC - Analog to Digital Converter Peripheral"
type: "peripheral-documentation"
purpose: "context7-knowledge-source"
mcu_family: "AT32F435/437"
peripheral: "ADC"
version: "1.0.0"
last_updated: "2024-11-26"
tags:
  - adc
  - analog
  - conversion
  - dma
  - temperature-sensor
  - voltage-monitoring
  - oversampling
  - peripheral
  - context7
related_peripherals:
  - DMA
  - EDMA
  - TMR
  - GPIO
---

# ADC - Analog to Digital Converter Peripheral

## Overview

The **Analog to Digital Converter (ADC)** peripheral in AT32F435/437 MCUs provides high-performance analog-to-digital conversion with extensive features for precise voltage measurement and signal acquisition.

**Key Features:**
- 🎯 **3 ADC units** (ADC1, ADC2, ADC3) with up to 19 channels each
- 📊 **12-bit resolution** (configurable: 12/10/8/6-bit)
- ⚡ **Up to 5.33 MSPS** conversion rate
- 🔄 **DMA/EDMA support** for continuous data transfer
- 🌡️ **Internal temperature sensor** and VREFINT channels
- 🔋 **VBAT monitoring** (battery voltage)
- 📈 **Hardware oversampling** (2x to 256x)
- ⚙️ **Multiple trigger sources** (software, timers, external)
- 🔔 **Voltage monitoring** (analog watchdog)
- 🔗 **Combine modes** for multi-ADC operation

---

## Architecture

```
                           ┌─────────────────────────────────────────────────────┐
 Analog Inputs             │                    ADC Peripheral                    │
 ─────────────────         │                                                     │
 CH0-CH15 (ext) ──────────►│  ┌──────────┐    ┌──────────────┐    ┌──────────┐  │
 CH16 (temp) ─────────────►│  │ Analog   │    │ SAR          │    │ Digital  │  │──► DMA
 CH17 (VREFINT) ──────────►│  │ MUX      │───►│ Converter    │───►│ Filter   │  │
 CH18 (VBAT/4) ───────────►│  └──────────┘    │ 12/10/8/6bit │    │ Oversamp │  │──► ODT Register
                           │                   └──────────────┘    └──────────┘  │
                           │                                                     │
 Trigger Sources           │  ┌──────────────────────────────────────────────┐  │
 ─────────────────         │  │ Sequence Controller                          │  │
 Software ─────────────────►│  │ - Ordinary: up to 16 channels               │  │
 TMR1-8, TMR20 ───────────►│  │ - Preempt: up to 4 channels (higher priority)│  │
 EXINT11/15 ──────────────►│  └──────────────────────────────────────────────┘  │
                           │                                                     │
                           │  ┌──────────────────────────────────────────────┐  │
                           │  │ Voltage Monitor (Analog Watchdog)            │  │──► VMOR Interrupt
                           │  │ - High/Low threshold comparison              │  │
                           │  └──────────────────────────────────────────────┘  │
                           └─────────────────────────────────────────────────────┘
```

---

## ADC Units and Channels

### Hardware Resources

| ADC Unit | Base Address | Channels | Special Channels |
|----------|--------------|----------|------------------|
| ADC1 | 0x40012400 | CH0-CH18 | Temp, VREFINT, VBAT |
| ADC2 | 0x40012500 | CH0-CH18 | Temp, VREFINT, VBAT |
| ADC3 | 0x40012600 | CH0-CH18 | Temp, VREFINT, VBAT |

### Channel Mapping

| Channel | Description | Typical Pin |
|---------|-------------|-------------|
| CH0-CH15 | External analog inputs | GPIO pins |
| CH16 | Internal temperature sensor | Internal |
| CH17 | VREFINT (internal reference) | Internal |
| CH18 | VBAT/4 (battery voltage) | Internal |

---

## Conversion Modes

### Ordinary Channels (Regular Group)

- Up to **16 channels** in sequence
- Configurable trigger sources
- DMA support for automatic data transfer
- Continuous or single conversion mode

### Preempt Channels (Injected Group)

- Up to **4 channels** in sequence
- **Higher priority** than ordinary channels
- Can interrupt ordinary conversion
- Individual data registers (PDT1-PDT4)
- Offset subtraction support

### Conversion Sequence

```
Ordinary Sequence:  CH4 → CH5 → CH6 → ... → CH_N (up to 16)
                      ↓     ↓     ↓           ↓
                    ODT   ODT   ODT         ODT (single register)
                      ↓     ↓     ↓           ↓
                    DMA   DMA   DMA         DMA

Preempt Sequence:   CH7 → CH8 → CH9 → CH10 (up to 4)
                      ↓     ↓     ↓     ↓
                    PDT1  PDT2  PDT3  PDT4 (individual registers)
```

---

## Resolution Options

| Resolution | Conversion Time | Max Value | Use Case |
|------------|-----------------|-----------|----------|
| 12-bit | Longest | 4095 | High precision |
| 10-bit | Medium | 1023 | Balanced |
| 8-bit | Short | 255 | Fast sampling |
| 6-bit | Shortest | 63 | Very fast sampling |

```c
/* Set ADC resolution */
adc_resolution_set(ADC1, ADC_RESOLUTION_12B);  // 12-bit
adc_resolution_set(ADC1, ADC_RESOLUTION_10B);  // 10-bit
adc_resolution_set(ADC1, ADC_RESOLUTION_8B);   // 8-bit
adc_resolution_set(ADC1, ADC_RESOLUTION_6B);   // 6-bit
```

---

## Sample Time Configuration

| Sample Time | Cycles | Use Case |
|-------------|--------|----------|
| ADC_SAMPLETIME_2_5 | 2.5 | Fast, low impedance source |
| ADC_SAMPLETIME_6_5 | 6.5 | General purpose |
| ADC_SAMPLETIME_12_5 | 12.5 | Medium impedance |
| ADC_SAMPLETIME_24_5 | 24.5 | Higher impedance |
| ADC_SAMPLETIME_47_5 | 47.5 | High impedance |
| ADC_SAMPLETIME_92_5 | 92.5 | Very high impedance |
| ADC_SAMPLETIME_247_5 | 247.5 | Internal channels |
| ADC_SAMPLETIME_640_5 | 640.5 | Temperature sensor |

**Total Conversion Time** = Sample Time + 12.5 cycles (for 12-bit)

---

## API Reference

### Initialization

```c
/* Reset ADC */
void adc_reset(void);

/* Initialize default parameters */
void adc_base_default_para_init(adc_base_config_type *adc_base_struct);
void adc_common_default_para_init(adc_common_config_type *adc_common_struct);

/* Apply configuration */
void adc_base_config(adc_type *adc_x, adc_base_config_type *adc_base_struct);
void adc_common_config(adc_common_config_type *adc_common_struct);

/* Enable/Disable ADC */
void adc_enable(adc_type *adc_x, confirm_state new_state);
```

### Calibration

```c
/* Initialize calibration */
void adc_calibration_init(adc_type *adc_x);
flag_status adc_calibration_init_status_get(adc_type *adc_x);

/* Start calibration */
void adc_calibration_start(adc_type *adc_x);
flag_status adc_calibration_status_get(adc_type *adc_x);

/* Set calibration value manually */
void adc_calibration_value_set(adc_type *adc_x, uint8_t adc_calibration_value);
```

### Channel Configuration

```c
/* Configure ordinary channel */
void adc_ordinary_channel_set(adc_type *adc_x, 
                               adc_channel_select_type adc_channel,
                               uint8_t adc_sequence, 
                               adc_sampletime_select_type adc_sampletime);

/* Configure preempt channel */
void adc_preempt_channel_length_set(adc_type *adc_x, uint8_t adc_channel_length);
void adc_preempt_channel_set(adc_type *adc_x, 
                              adc_channel_select_type adc_channel,
                              uint8_t adc_sequence, 
                              adc_sampletime_select_type adc_sampletime);

/* Set preempt offset value */
void adc_preempt_offset_value_set(adc_type *adc_x, 
                                   adc_preempt_channel_type adc_preempt_channel,
                                   uint16_t adc_offset_value);
```

### Trigger Configuration

```c
/* Ordinary channel trigger */
void adc_ordinary_conversion_trigger_set(adc_type *adc_x, 
                                          adc_ordinary_trig_select_type adc_ordinary_trig,
                                          adc_ordinary_trig_edge_type adc_ordinary_trig_edge);

/* Preempt channel trigger */
void adc_preempt_conversion_trigger_set(adc_type *adc_x, 
                                         adc_preempt_trig_select_type adc_preempt_trig,
                                         adc_preempt_trig_edge_type adc_preempt_trig_edge);

/* Software trigger */
void adc_ordinary_software_trigger_enable(adc_type *adc_x, confirm_state new_state);
void adc_preempt_software_trigger_enable(adc_type *adc_x, confirm_state new_state);
```

### Data Reading

```c
/* Read ordinary conversion data */
uint16_t adc_ordinary_conversion_data_get(adc_type *adc_x);

/* Read preempt conversion data */
uint16_t adc_preempt_conversion_data_get(adc_type *adc_x, 
                                          adc_preempt_channel_type adc_preempt_channel);

/* Read combined data (for multi-ADC modes) */
uint32_t adc_combine_ordinary_conversion_data_get(void);
```

### DMA Configuration

```c
/* Enable DMA mode */
void adc_dma_mode_enable(adc_type *adc_x, confirm_state new_state);

/* Enable DMA request repeat (for continuous mode) */
void adc_dma_request_repeat_enable(adc_type *adc_x, confirm_state new_state);
```

### Voltage Monitoring (Analog Watchdog)

```c
/* Enable voltage monitoring */
void adc_voltage_monitor_enable(adc_type *adc_x, 
                                 adc_voltage_monitoring_type adc_voltage_monitoring);

/* Set threshold values */
void adc_voltage_monitor_threshold_value_set(adc_type *adc_x, 
                                              uint16_t adc_high_threshold,
                                              uint16_t adc_low_threshold);

/* Select monitored channel */
void adc_voltage_monitor_single_channel_select(adc_type *adc_x, 
                                                adc_channel_select_type adc_channel);
```

### Oversampling

```c
/* Enable oversampling */
void adc_ordinary_oversample_enable(adc_type *adc_x, confirm_state new_state);
void adc_preempt_oversample_enable(adc_type *adc_x, confirm_state new_state);

/* Configure oversampling ratio and shift */
void adc_oversample_ratio_shift_set(adc_type *adc_x, 
                                     adc_oversample_ratio_type adc_oversample_ratio,
                                     adc_oversample_shift_type adc_oversample_shift);

/* Configure oversampling trigger and restart modes */
void adc_ordinary_oversample_trig_enable(adc_type *adc_x, confirm_state new_state);
void adc_ordinary_oversample_restart_set(adc_type *adc_x, 
                                          adc_ordinary_oversample_restart_type restart);
```

### Flags and Interrupts

```c
/* Enable interrupts */
void adc_interrupt_enable(adc_type *adc_x, uint32_t adc_int, confirm_state new_state);

/* Check flags */
flag_status adc_flag_get(adc_type *adc_x, uint8_t adc_flag);
flag_status adc_interrupt_flag_get(adc_type *adc_x, uint8_t adc_flag);

/* Clear flags */
void adc_flag_clear(adc_type *adc_x, uint32_t adc_flag);
```

---

## Complete Examples

### Example 1: Basic Software Trigger with DMA

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

__IO uint16_t adc1_values[3] = {0};

void gpio_config(void)
{
  gpio_init_type gpio_init_struct;
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_6;
  gpio_init(GPIOA, &gpio_init_struct);
}

void dma_config(void)
{
  dma_init_type dma_init_struct;
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  
  dma_reset(DMA1_CHANNEL1);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = 3;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)adc1_values;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = TRUE;
  dma_init(DMA1_CHANNEL1, &dma_init_struct);

  /* Configure DMAMUX */
  dmamux_enable(DMA1, TRUE);
  dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_ADC1);
  
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

void adc_config(void)
{
  adc_common_config_type adc_common_struct;
  adc_base_config_type adc_base_struct;
  
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);

  /* Common configuration */
  adc_common_default_para_init(&adc_common_struct);
  adc_common_struct.combine_mode = ADC_INDEPENDENT_MODE;
  adc_common_struct.div = ADC_HCLK_DIV_4;
  adc_common_struct.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
  adc_common_struct.tempervintrv_state = FALSE;
  adc_common_struct.vbat_state = FALSE;
  adc_common_config(&adc_common_struct);

  /* Base configuration */
  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = TRUE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 3;
  adc_base_config(ADC1, &adc_base_struct);
  
  adc_resolution_set(ADC1, ADC_RESOLUTION_12B);

  /* Configure ordinary channels */
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 1, ADC_SAMPLETIME_47_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_5, 2, ADC_SAMPLETIME_47_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_6, 3, ADC_SAMPLETIME_47_5);

  /* Software trigger (no external trigger) */
  adc_ordinary_conversion_trigger_set(ADC1, ADC_ORDINARY_TRIG_TMR1CH1, 
                                       ADC_ORDINARY_TRIG_EDGE_NONE);

  /* Enable DMA */
  adc_dma_mode_enable(ADC1, TRUE);
  adc_dma_request_repeat_enable(ADC1, TRUE);

  /* Enable ADC */
  adc_enable(ADC1, TRUE);
  while(adc_flag_get(ADC1, ADC_RDY_FLAG) == RESET);

  /* Calibration */
  adc_calibration_init(ADC1);
  while(adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while(adc_calibration_status_get(ADC1));
}

int main(void)
{
  system_clock_config();
  gpio_config();
  dma_config();
  adc_config();

  /* Start continuous conversion */
  adc_ordinary_software_trigger_enable(ADC1, TRUE);

  while(1)
  {
    /* adc1_values[] is continuously updated by DMA */
    printf("CH4=%d, CH5=%d, CH6=%d\r\n", 
           adc1_values[0], adc1_values[1], adc1_values[2]);
    delay_ms(1000);
  }
}
```

### Example 2: Internal Temperature Sensor

```c
#define ADC_VREF         (3.3)
#define ADC_TEMP_BASE    (1.27)
#define ADC_TEMP_SLOPE   (-0.00413)

__IO uint16_t adc1_temp_value = 0;

void adc_temp_config(void)
{
  adc_common_config_type adc_common_struct;
  adc_base_config_type adc_base_struct;
  
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);

  /* Enable temperature sensor */
  adc_common_default_para_init(&adc_common_struct);
  adc_common_struct.combine_mode = ADC_INDEPENDENT_MODE;
  adc_common_struct.div = ADC_HCLK_DIV_4;
  adc_common_struct.tempervintrv_state = TRUE;  /* Enable temp sensor */
  adc_common_config(&adc_common_struct);

  /* Single channel, single conversion */
  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = FALSE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 1;
  adc_base_config(ADC1, &adc_base_struct);
  
  adc_resolution_set(ADC1, ADC_RESOLUTION_12B);

  /* Configure temperature sensor channel (CH16) with long sample time */
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_16, 1, ADC_SAMPLETIME_640_5);

  /* Enable ADC and calibrate */
  adc_enable(ADC1, TRUE);
  while(adc_flag_get(ADC1, ADC_RDY_FLAG) == RESET);
  
  adc_calibration_init(ADC1);
  while(adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while(adc_calibration_status_get(ADC1));
}

float read_temperature(void)
{
  /* Trigger conversion */
  adc_ordinary_software_trigger_enable(ADC1, TRUE);
  
  /* Wait for conversion complete */
  while(adc_flag_get(ADC1, ADC_OCCE_FLAG) == RESET);
  adc_flag_clear(ADC1, ADC_OCCE_FLAG);
  
  /* Read value */
  adc1_temp_value = adc_ordinary_conversion_data_get(ADC1);
  
  /* Calculate temperature */
  /* Formula: T = (V_BASE - V_SENSE) / SLOPE + 25°C */
  float v_sense = (float)adc1_temp_value * ADC_VREF / 4096.0;
  float temperature = (ADC_TEMP_BASE - v_sense) / ADC_TEMP_SLOPE + 25.0;
  
  return temperature;
}
```

### Example 3: Voltage Monitoring (Analog Watchdog)

```c
__IO uint32_t vmor_flag_count = 0;

void ADC1_2_3_IRQHandler(void)
{
  /* Voltage monitor out of range interrupt */
  if(adc_interrupt_flag_get(ADC1, ADC_VMOR_FLAG) != RESET)
  {
    vmor_flag_count++;
    adc_flag_clear(ADC1, ADC_VMOR_FLAG);
    
    /* Handle voltage out of range condition */
    at32_led_toggle(LED3);
  }
}

void adc_voltage_monitor_config(void)
{
  /* ... basic ADC configuration ... */

  /* Configure voltage monitoring */
  /* High threshold: 0x100 (256), Low threshold: 0x000 (0) */
  adc_voltage_monitor_threshold_value_set(ADC1, 0x100, 0x000);
  
  /* Monitor single ordinary channel (CH5) */
  adc_voltage_monitor_single_channel_select(ADC1, ADC_CHANNEL_5);
  adc_voltage_monitor_enable(ADC1, ADC_VMONITOR_SINGLE_ORDINARY);
  
  /* Enable voltage monitoring interrupt */
  adc_interrupt_enable(ADC1, ADC_VMOR_INT, TRUE);
  nvic_irq_enable(ADC1_2_3_IRQn, 0, 0);
}
```

### Example 4: Hardware Oversampling

```c
void adc_oversampling_config(void)
{
  /* ... basic ADC configuration ... */

  /* Configure oversampling: 8x ratio, shift right by 3 bits */
  /* Result = Sum of 8 samples >> 3 = Average of 8 samples */
  adc_oversample_ratio_shift_set(ADC1, ADC_OVERSAMPLE_RATIO_8, ADC_OVERSAMPLE_SHIFT_3);

  /* Disable triggered oversampling (continuous mode) */
  adc_ordinary_oversample_trig_enable(ADC1, FALSE);
  
  /* Set restart mode: continue after preempt interrupt */
  adc_ordinary_oversample_restart_set(ADC1, ADC_OVERSAMPLE_CONTINUE);

  /* Enable ordinary oversampling */
  adc_ordinary_oversample_enable(ADC1, TRUE);
  
  /* Optionally enable preempt oversampling */
  adc_preempt_oversample_enable(ADC1, TRUE);
}
```

### Example 5: Preempt (Injected) Channels

```c
__IO uint16_t preempt_values[4] = {0};
__IO uint16_t preempt_count = 0;

void ADC1_2_3_IRQHandler(void)
{
  /* Preempt conversion complete interrupt */
  if(adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET)
  {
    /* Read preempt channel data */
    preempt_values[0] = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_1);
    preempt_values[1] = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_2);
    preempt_values[2] = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_3);
    preempt_count++;
    
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
  }
}

void adc_preempt_config(void)
{
  /* ... basic ADC configuration ... */

  /* Configure preempt channels */
  adc_preempt_channel_length_set(ADC1, 3);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_7, 1, ADC_SAMPLETIME_6_5);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_8, 2, ADC_SAMPLETIME_6_5);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_9, 3, ADC_SAMPLETIME_6_5);

  /* Configure preempt trigger (software trigger) */
  adc_preempt_conversion_trigger_set(ADC1, ADC_PREEMPT_TRIG_TMR1CH4, 
                                      ADC_PREEMPT_TRIG_EDGE_NONE);

  /* Disable auto preempt (manual trigger only) */
  adc_preempt_auto_mode_enable(ADC1, FALSE);

  /* Enable preempt conversion complete interrupt */
  adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
  nvic_irq_enable(ADC1_2_3_IRQn, 0, 0);
}

void trigger_preempt_conversion(void)
{
  /* Preempt conversion will interrupt ordinary conversion */
  adc_preempt_software_trigger_enable(ADC1, TRUE);
}
```

---

## Combine Modes (Multi-ADC)

### Available Modes

| Mode | Description | ADCs Used |
|------|-------------|-----------|
| `ADC_INDEPENDENT_MODE` | Each ADC operates independently | Any |
| `ADC_ORDINARY_SMLT_ONLY_ONESLAVE_MODE` | Ordinary simultaneous (ADC1+ADC2) | ADC1, ADC2 |
| `ADC_ORDINARY_SMLT_ONLY_TWOSLAVE_MODE` | Ordinary simultaneous (ADC1+ADC2+ADC3) | All |
| `ADC_ORDINARY_SHIFT_ONLY_ONESLAVE_MODE` | Ordinary interleaved (ADC1+ADC2) | ADC1, ADC2 |
| `ADC_ORDINARY_SHIFT_ONLY_TWOSLAVE_MODE` | Ordinary interleaved (ADC1+ADC2+ADC3) | All |
| `ADC_PREEMPT_SMLT_ONLY_*` | Preempt simultaneous | Multi |
| `ADC_PREEMPT_INTERLTRIG_ONLY_*` | Preempt interleaved | Multi |

### DMA Modes for Combined Operation

| DMA Mode | Description |
|----------|-------------|
| `ADC_COMMON_DMAMODE_1` | Each request transfers half-word |
| `ADC_COMMON_DMAMODE_2` | Each request transfers two half-words |
| `ADC_COMMON_DMAMODE_3` | Each request transfers two bytes |
| `ADC_COMMON_DMAMODE_4` | Each request transfers three bytes |
| `ADC_COMMON_DMAMODE_5` | Alternating transfer sizes |

---

## Trigger Sources

### Ordinary Channel Triggers

| Trigger Source | Description |
|----------------|-------------|
| `ADC_ORDINARY_TRIG_TMR1CH1` | Timer 1 Channel 1 |
| `ADC_ORDINARY_TRIG_TMR1CH2` | Timer 1 Channel 2 |
| `ADC_ORDINARY_TRIG_TMR1CH3` | Timer 1 Channel 3 |
| `ADC_ORDINARY_TRIG_TMR1TRGOUT` | Timer 1 TRGO |
| `ADC_ORDINARY_TRIG_TMR2CH2-4` | Timer 2 Channels |
| `ADC_ORDINARY_TRIG_TMR3CH1/TRGOUT` | Timer 3 |
| `ADC_ORDINARY_TRIG_TMR4CH4/TRGOUT` | Timer 4 |
| `ADC_ORDINARY_TRIG_TMR5CH1-3` | Timer 5 Channels |
| `ADC_ORDINARY_TRIG_TMR8CH1/TRGOUT` | Timer 8 |
| `ADC_ORDINARY_TRIG_TMR20*` | Timer 20 |
| `ADC_ORDINARY_TRIG_EXINT11` | External interrupt line 11 |

### Preempt Channel Triggers

| Trigger Source | Description |
|----------------|-------------|
| `ADC_PREEMPT_TRIG_TMR1CH4/TRGOUT` | Timer 1 |
| `ADC_PREEMPT_TRIG_TMR2CH1/TRGOUT` | Timer 2 |
| `ADC_PREEMPT_TRIG_TMR3CH2-4/TRGOUT` | Timer 3 |
| `ADC_PREEMPT_TRIG_TMR4CH1-4/TRGOUT` | Timer 4 |
| `ADC_PREEMPT_TRIG_TMR5CH4/TRGOUT` | Timer 5 |
| `ADC_PREEMPT_TRIG_TMR8CH2-4/TRGOUT` | Timer 8 |
| `ADC_PREEMPT_TRIG_TMR20*` | Timer 20 |
| `ADC_PREEMPT_TRIG_EXINT15` | External interrupt line 15 |

---

## Flags and Interrupts

### Status Flags

| Flag | Description |
|------|-------------|
| `ADC_VMOR_FLAG` | Voltage monitoring out of range |
| `ADC_OCCE_FLAG` | Ordinary channels conversion end |
| `ADC_PCCE_FLAG` | Preempt channels conversion end |
| `ADC_PCCS_FLAG` | Preempt channel conversion start |
| `ADC_OCCS_FLAG` | Ordinary channel conversion start |
| `ADC_OCCO_FLAG` | Ordinary channel conversion overflow |
| `ADC_RDY_FLAG` | ADC ready for conversion |

### Interrupts

| Interrupt | Description |
|-----------|-------------|
| `ADC_OCCE_INT` | Ordinary channels conversion end |
| `ADC_VMOR_INT` | Voltage monitoring out of range |
| `ADC_PCCE_INT` | Preempt channels conversion end |
| `ADC_OCCO_INT` | Ordinary channel conversion overflow |

---

## Implementation Checklist

### Basic Setup
- [ ] Enable ADC peripheral clock (`CRM_ADC1_PERIPH_CLOCK`)
- [ ] Configure GPIO pins as analog input (`GPIO_MODE_ANALOG`)
- [ ] Initialize common ADC configuration
- [ ] Initialize base ADC configuration
- [ ] Set resolution (12/10/8/6-bit)
- [ ] Configure channel sequence and sample times
- [ ] Configure trigger source and edge
- [ ] Enable ADC and wait for ready flag
- [ ] Perform calibration

### DMA Setup (Optional)
- [ ] Enable DMA peripheral clock
- [ ] Configure DMA channel (direction, size, increment)
- [ ] Configure DMAMUX for ADC
- [ ] Enable DMA channel
- [ ] Enable ADC DMA mode

### Advanced Features
- [ ] Configure voltage monitoring if needed
- [ ] Configure oversampling if needed
- [ ] Configure preempt channels if needed
- [ ] Enable required interrupts
- [ ] Configure NVIC for ADC interrupts

---

## Voltage Conversion Formulas

### ADC Value to Voltage

```c
/* For 12-bit resolution with 3.3V reference */
float voltage = (float)adc_value * 3.3 / 4096.0;

/* For 10-bit resolution */
float voltage = (float)adc_value * 3.3 / 1024.0;
```

### Temperature Calculation

```c
/* AT32F435/437 internal temperature sensor */
#define ADC_VREF      3.3      /* Reference voltage */
#define TEMP_V25      1.27     /* Voltage at 25°C */
#define TEMP_SLOPE    -0.00413 /* mV/°C slope */

float v_sense = (float)adc_value * ADC_VREF / 4096.0;
float temp_c = (TEMP_V25 - v_sense) / TEMP_SLOPE + 25.0;
```

### VBAT Measurement

```c
/* VBAT is divided by 4 internally */
float vbat = (float)adc_value * 3.3 * 4.0 / 4096.0;
```

---

## Troubleshooting

### Conversion Not Starting

- Verify ADC clock is enabled
- Check ADC is enabled (`adc_enable()`)
- Verify trigger configuration
- Wait for ADC ready flag after enable

### Incorrect Values

- Verify GPIO is configured as analog input
- Check sample time is sufficient for source impedance
- Verify calibration was performed
- Check reference voltage

### DMA Not Transferring

- Verify DMA clock is enabled
- Check DMAMUX configuration
- Verify DMA channel is enabled
- Check ADC DMA mode is enabled

### Overflow Errors

- Increase DMA buffer size
- Reduce conversion rate
- Use higher DMA priority
- Process data faster

---

## Related Documentation

- **Reference Manual:** RM_AT32F435_437 - Chapter on ADC
- **Datasheet:** DS_AT32F435_437 - Electrical characteristics
- **Example Code:** `project/at_start_f435/examples/adc/`

---

**Last Updated:** November 2024  
**Firmware Library:** v2.2.2  
**Status:** ✅ Production Ready

**Tags:** #ADC #analog #conversion #DMA #temperature #voltage-monitoring #Context7

