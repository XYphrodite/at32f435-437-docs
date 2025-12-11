---
title: TMR - Timer
category: Peripheral
complexity: Intermediate
mcu: AT32F435/437
peripheral: TMR
keywords: [timer, pwm, input capture, output compare, encoder, brake, dead-time, dma, complementary]
---

# TMR - Timer

## Overview

The Timer (TMR) peripheral provides versatile timing, counting, PWM generation, and input capture capabilities. The AT32F435/437 features 15 timer instances categorized as Advanced Control Timers (TMR1, TMR8, TMR20), General Purpose Timers (TMR2-5, TMR9-14), and Basic Timers (TMR6, TMR7). Timers support various operating modes including PWM output, input capture, encoder interface, one-pulse mode, and master/slave synchronization.

### Timer Classification

| Category | Timers | Resolution | Channels | Complementary | Features |
|----------|--------|------------|----------|---------------|----------|
| Advanced | TMR1, TMR8, TMR20 | 16-bit | 4+1 | Yes (3 pairs) | Dead-time, Brake, 6-step commutation |
| General Purpose (32-bit) | TMR2, TMR5 | 32-bit | 4 | No | Encoder, External clock |
| General Purpose (16-bit) | TMR3, TMR4 | 16-bit | 4 | No | Encoder, External clock |
| General Purpose (1ch) | TMR9-14 | 16-bit | 1-2 | No | Basic PWM/Capture |
| Basic | TMR6, TMR7 | 16-bit | 0 | No | Time base, DAC trigger |

### Key Features

| Feature | Specification |
|---------|---------------|
| Timer Count | 15 instances |
| Counter Width | 16-bit (most) / 32-bit (TMR2, TMR5) |
| Prescaler | 16-bit (1 to 65536) |
| Counter Modes | Up, Down, Center-aligned |
| PWM Modes | Edge-aligned, Center-aligned |
| Input Capture | Rising/Falling/Both edges |
| Output Compare | 8 modes |
| Encoder Interface | Modes A, B, C |
| Dead-time | 8-bit generator |
| Brake Input | Hardware protection |
| DMA | Per channel and overflow |
| Synchronization | Master/Slave chaining |

## Architecture

```
                                       ┌─────────────────────────────────────────────────┐
                                       │              TMR Controller                      │
┌─────────────┐                        │                                                  │
│   CK_INT    │───────┐               │  ┌────────────────────────────────────────────┐  │
│ (APB Clock) │       │               │  │           Time Base Unit                    │  │
└─────────────┘       │               │  │                                             │  │
                      ▼               │  │  ┌──────┐   ┌──────┐   ┌──────────────┐    │  │
              ┌───────────────┐       │  │  │ DIV  │───│ CNT  │───│ Auto-reload  │    │  │
              │   Prescaler   │───────│──│  │ PSC  │   │ CVAL │   │    PR        │    │  │
              │   (DIV)       │       │  │  └──────┘   └──────┘   └──────────────┘    │  │
              └───────────────┘       │  │       │          │             │           │  │
                      ▲               │  │       ▼          ▼             ▼           │  │
┌─────────────┐       │               │  │  ┌────────────────────────────────────┐    │  │
│ External    │───────┤               │  │  │         Overflow/Underflow         │    │  │
│ Clock       │       │               │  └──┴────────────────────────────────────┴────┘  │
└─────────────┘       │               │                                                  │
                      │               │  ┌────────────────────────────────────────────┐  │
┌─────────────┐       │               │  │           Capture/Compare Unit             │  │
│ Trigger     │───────┘               │  │                                             │  │
│ Input       │                       │  │  ┌────────┐  ┌────────┐  ┌────────┐       │  │
└─────────────┘                       │  │  │ CH1    │  │ CH2    │  │ CH3/4  │       │  │
                                      │  │  │ C1DT   │  │ C2DT   │  │ C3/4DT │       │  │
                                      │  │  └────────┘  └────────┘  └────────┘       │  │
                                      │  │       │           │           │            │  │
                                      │  │       ▼           ▼           ▼            │  │
                                      │  │  ┌────────────────────────────────────┐   │  │
                                      │  │  │     Output Control / Dead-time     │   │  │
                                      │  │  └────────────────────────────────────┘   │  │
                                      │  └────────────────────────────────────────────┘  │
                                      │                                                  │
                                      │       ┌───────────────────────────────────┐     │
                                      │       │    Brake & Protection Logic       │     │
                                      │       └───────────────────────────────────┘     │
                                      │                                                  │
                                      │  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐  │
                                      │  │CH1   │ │CH1N  │ │CH2   │ │CH2N  │ │ ...  │  │
                                      └──┴──────┴─┴──────┴─┴──────┴─┴──────┴─┴──────┴──┘
                                              │      │       │       │
                                              ▼      ▼       ▼       ▼
                                           Output   Output  Output  Output
                                           Pins     Pins    Pins    Pins
```

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| CTRL1 | 0x00 | Control register 1 |
| CTRL2 | 0x04 | Control register 2 |
| STCTRL | 0x08 | Subordinate mode control register |
| IDEN | 0x0C | Interrupt/DMA enable register |
| ISTS | 0x10 | Interrupt status register |
| SWEVT | 0x14 | Software event register |
| CM1 | 0x18 | Channel mode register 1 |
| CM2 | 0x1C | Channel mode register 2 |
| CCTRL | 0x20 | Channel control register |
| CVAL | 0x24 | Counter value register |
| DIV | 0x28 | Prescaler (divider) register |
| PR | 0x2C | Period (auto-reload) register |
| RPR | 0x30 | Repetition period register |
| C1DT | 0x34 | Channel 1 data register |
| C2DT | 0x38 | Channel 2 data register |
| C3DT | 0x3C | Channel 3 data register |
| C4DT | 0x40 | Channel 4 data register |
| BRK | 0x44 | Brake register |
| DMACTRL | 0x48 | DMA control register |
| DMADT | 0x4C | DMA data register |
| RMP | 0x50 | Input remap register |
| CM3 | 0x70 | Channel mode register 3 (CH5) |
| C5DT | 0x74 | Channel 5 data register |

## Configuration Types

### Counter Mode

| Enum | Value | Description |
|------|-------|-------------|
| `TMR_COUNT_UP` | 0x00 | Count up |
| `TMR_COUNT_DOWN` | 0x01 | Count down |
| `TMR_COUNT_TWO_WAY_1` | 0x02 | Center-aligned mode 1 |
| `TMR_COUNT_TWO_WAY_2` | 0x04 | Center-aligned mode 2 |
| `TMR_COUNT_TWO_WAY_3` | 0x06 | Center-aligned mode 3 |

### Clock Division

| Enum | Value | Description |
|------|-------|-------------|
| `TMR_CLOCK_DIV1` | 0x00 | Division by 1 |
| `TMR_CLOCK_DIV2` | 0x01 | Division by 2 |
| `TMR_CLOCK_DIV4` | 0x02 | Division by 4 |

### Output Control Mode

| Enum | Value | Description |
|------|-------|-------------|
| `TMR_OUTPUT_CONTROL_OFF` | 0x00 | Frozen |
| `TMR_OUTPUT_CONTROL_HIGH` | 0x01 | Active on match |
| `TMR_OUTPUT_CONTROL_LOW` | 0x02 | Inactive on match |
| `TMR_OUTPUT_CONTROL_SWITCH` | 0x03 | Toggle on match |
| `TMR_OUTPUT_CONTROL_FORCE_LOW` | 0x04 | Force inactive |
| `TMR_OUTPUT_CONTROL_FORCE_HIGH` | 0x05 | Force active |
| `TMR_OUTPUT_CONTROL_PWM_MODE_A` | 0x06 | PWM mode A (edge-aligned) |
| `TMR_OUTPUT_CONTROL_PWM_MODE_B` | 0x07 | PWM mode B (inverted) |

### Output Polarity

| Enum | Value | Description |
|------|-------|-------------|
| `TMR_OUTPUT_ACTIVE_HIGH` | 0x00 | Active high |
| `TMR_OUTPUT_ACTIVE_LOW` | 0x01 | Active low |

### Input Polarity

| Enum | Value | Description |
|------|-------|-------------|
| `TMR_INPUT_RISING_EDGE` | 0x00 | Rising edge |
| `TMR_INPUT_FALLING_EDGE` | 0x01 | Falling edge |
| `TMR_INPUT_BOTH_EDGE` | 0x03 | Both edges |

### Channel Select

| Enum | Value | Description |
|------|-------|-------------|
| `TMR_SELECT_CHANNEL_1` | 0x00 | Channel 1 |
| `TMR_SELECT_CHANNEL_1C` | 0x01 | Channel 1 complementary |
| `TMR_SELECT_CHANNEL_2` | 0x02 | Channel 2 |
| `TMR_SELECT_CHANNEL_2C` | 0x03 | Channel 2 complementary |
| `TMR_SELECT_CHANNEL_3` | 0x04 | Channel 3 |
| `TMR_SELECT_CHANNEL_3C` | 0x05 | Channel 3 complementary |
| `TMR_SELECT_CHANNEL_4` | 0x06 | Channel 4 |
| `TMR_SELECT_CHANNEL_5` | 0x07 | Channel 5 |

### Encoder Mode

| Enum | Value | Description |
|------|-------|-------------|
| `TMR_ENCODER_MODE_A` | 0x01 | Count on TI1 edges |
| `TMR_ENCODER_MODE_B` | 0x02 | Count on TI2 edges |
| `TMR_ENCODER_MODE_C` | 0x03 | Count on both TI1 and TI2 edges |

### Subordinate Mode

| Enum | Value | Description |
|------|-------|-------------|
| `TMR_SUB_MODE_DIABLE` | 0x00 | Disabled |
| `TMR_SUB_ENCODER_MODE_A` | 0x01 | Encoder mode A |
| `TMR_SUB_ENCODER_MODE_B` | 0x02 | Encoder mode B |
| `TMR_SUB_ENCODER_MODE_C` | 0x03 | Encoder mode C |
| `TMR_SUB_RESET_MODE` | 0x04 | Reset on trigger |
| `TMR_SUB_HANG_MODE` | 0x05 | Gated mode |
| `TMR_SUB_TRIGGER_MODE` | 0x06 | Trigger mode |
| `TMR_SUB_EXTERNAL_CLOCK_MODE_A` | 0x07 | External clock mode A |

## Flags and Interrupts

### Status Flags

| Flag | Description |
|------|-------------|
| `TMR_OVF_FLAG` | Overflow flag |
| `TMR_C1_FLAG` | Channel 1 capture/compare flag |
| `TMR_C2_FLAG` | Channel 2 capture/compare flag |
| `TMR_C3_FLAG` | Channel 3 capture/compare flag |
| `TMR_C4_FLAG` | Channel 4 capture/compare flag |
| `TMR_C5_FLAG` | Channel 5 capture/compare flag |
| `TMR_HALL_FLAG` | Hall sensor event flag |
| `TMR_TRIGGER_FLAG` | Trigger flag |
| `TMR_BRK_FLAG` | Brake flag |
| `TMR_C1_RECAPTURE_FLAG` | Channel 1 recapture flag |
| `TMR_C2_RECAPTURE_FLAG` | Channel 2 recapture flag |
| `TMR_C3_RECAPTURE_FLAG` | Channel 3 recapture flag |
| `TMR_C4_RECAPTURE_FLAG` | Channel 4 recapture flag |

### Interrupt Sources

| Interrupt | Description |
|-----------|-------------|
| `TMR_OVF_INT` | Overflow interrupt |
| `TMR_C1_INT` | Channel 1 interrupt |
| `TMR_C2_INT` | Channel 2 interrupt |
| `TMR_C3_INT` | Channel 3 interrupt |
| `TMR_C4_INT` | Channel 4 interrupt |
| `TMR_HALL_INT` | Hall sensor interrupt |
| `TMR_TRIGGER_INT` | Trigger interrupt |
| `TMR_BRK_INT` | Brake interrupt |

### DMA Requests

| DMA Request | Description |
|-------------|-------------|
| `TMR_OVERFLOW_DMA_REQUEST` | Overflow DMA request |
| `TMR_C1_DMA_REQUEST` | Channel 1 DMA request |
| `TMR_C2_DMA_REQUEST` | Channel 2 DMA request |
| `TMR_C3_DMA_REQUEST` | Channel 3 DMA request |
| `TMR_C4_DMA_REQUEST` | Channel 4 DMA request |
| `TMR_HALL_DMA_REQUEST` | Hall DMA request |
| `TMR_TRIGGER_DMA_REQUEST` | Trigger DMA request |

## Init Structures

### Output Config Structure

```c
typedef struct
{
    tmr_output_control_mode_type oc_mode;           /* Output channel mode */
    confirm_state                oc_idle_state;     /* Idle state */
    confirm_state                occ_idle_state;    /* Complementary idle state */
    tmr_output_polarity_type     oc_polarity;       /* Output polarity */
    tmr_output_polarity_type     occ_polarity;      /* Complementary polarity */
    confirm_state                oc_output_state;   /* Output enable */
    confirm_state                occ_output_state;  /* Complementary output enable */
} tmr_output_config_type;
```

### Input Config Structure

```c
typedef struct
{
    tmr_channel_select_type          input_channel_select;   /* Channel selection */
    tmr_input_polarity_type          input_polarity_select;  /* Edge polarity */
    tmr_input_direction_mapped_type  input_mapped_select;    /* Direct/Indirect mapping */
    uint8_t                          input_filter_value;     /* Digital filter (0-15) */
} tmr_input_config_type;
```

### Brake/Dead-time Config Structure

```c
typedef struct
{
    uint8_t                  deadtime;            /* Dead-time value (0-255) */
    tmr_brk_polarity_type    brk_polarity;        /* Brake input polarity */
    tmr_wp_level_type        wp_level;            /* Write protection level */
    confirm_state            auto_output_enable;  /* Automatic output enable */
    confirm_state            fcsoen_state;        /* Freeze state on enable */
    confirm_state            fcsodis_state;       /* Freeze state on disable */
    confirm_state            brk_enable;          /* Brake enable */
} tmr_brkdt_config_type;
```

## API Reference

### Initialization and Reset

#### `tmr_reset`

```c
void tmr_reset(tmr_type *tmr_x);
```

Reset timer via CRM reset register.

---

#### `tmr_base_init`

```c
void tmr_base_init(tmr_type* tmr_x, uint32_t tmr_pr, uint32_t tmr_div);
```

Initialize timer time base: period and prescaler.

**Parameters:**
- `tmr_pr`: Period value (auto-reload)
- `tmr_div`: Prescaler value (0 = divide by 1)

**Timer Frequency:** `f_TMR = f_CLK / ((DIV + 1) * (PR + 1))`

---

#### `tmr_counter_enable`

```c
void tmr_counter_enable(tmr_type *tmr_x, confirm_state new_state);
```

Enable or disable timer counter.

---

#### `tmr_output_enable`

```c
void tmr_output_enable(tmr_type *tmr_x, confirm_state new_state);
```

Enable main output (required for advanced timers).

---

### Counter Configuration

#### `tmr_cnt_dir_set`

```c
void tmr_cnt_dir_set(tmr_type *tmr_x, tmr_count_mode_type tmr_cnt_dir);
```

Set counter direction/mode.

---

#### `tmr_counter_value_set` / `tmr_counter_value_get`

```c
void tmr_counter_value_set(tmr_type *tmr_x, uint32_t tmr_cnt_value);
uint32_t tmr_counter_value_get(tmr_type *tmr_x);
```

Set/get current counter value.

---

#### `tmr_period_value_set` / `tmr_period_value_get`

```c
void tmr_period_value_set(tmr_type *tmr_x, uint32_t tmr_pr_value);
uint32_t tmr_period_value_get(tmr_type *tmr_x);
```

Set/get period (auto-reload) value.

---

#### `tmr_div_value_set` / `tmr_div_value_get`

```c
void tmr_div_value_set(tmr_type *tmr_x, uint32_t tmr_div_value);
uint32_t tmr_div_value_get(tmr_type *tmr_x);
```

Set/get prescaler value.

---

#### `tmr_32_bit_function_enable`

```c
void tmr_32_bit_function_enable(tmr_type *tmr_x, confirm_state new_state);
```

Enable 32-bit mode (TMR2, TMR5 only).

---

### Output Channel Configuration

#### `tmr_output_default_para_init`

```c
void tmr_output_default_para_init(tmr_output_config_type *tmr_output_struct);
```

Initialize output config structure with defaults.

---

#### `tmr_output_channel_config`

```c
void tmr_output_channel_config(tmr_type *tmr_x, tmr_channel_select_type tmr_channel,
                               tmr_output_config_type *tmr_output_struct);
```

Configure output channel.

---

#### `tmr_output_channel_mode_select`

```c
void tmr_output_channel_mode_select(tmr_type *tmr_x, tmr_channel_select_type tmr_channel,
                                    tmr_output_control_mode_type oc_mode);
```

Select output channel mode.

---

#### `tmr_channel_value_set` / `tmr_channel_value_get`

```c
void tmr_channel_value_set(tmr_type *tmr_x, tmr_channel_select_type tmr_channel,
                           uint32_t tmr_channel_value);
uint32_t tmr_channel_value_get(tmr_type *tmr_x, tmr_channel_select_type tmr_channel);
```

Set/get channel compare/capture value.

---

#### `tmr_output_channel_buffer_enable`

```c
void tmr_output_channel_buffer_enable(tmr_type *tmr_x, tmr_channel_select_type tmr_channel,
                                      confirm_state new_state);
```

Enable channel preload (buffered update).

---

#### `tmr_output_channel_polarity_set`

```c
void tmr_output_channel_polarity_set(tmr_type *tmr_x, tmr_channel_select_type tmr_channel,
                                     tmr_polarity_active_type oc_polarity);
```

Set output channel polarity.

---

#### `tmr_channel_enable`

```c
void tmr_channel_enable(tmr_type *tmr_x, tmr_channel_select_type tmr_channel, confirm_state new_state);
```

Enable or disable channel output.

---

### Input Capture Configuration

#### `tmr_input_default_para_init`

```c
void tmr_input_default_para_init(tmr_input_config_type *tmr_input_struct);
```

Initialize input config structure with defaults.

---

#### `tmr_input_channel_init`

```c
void tmr_input_channel_init(tmr_type *tmr_x, tmr_input_config_type *input_struct,
                            tmr_channel_input_divider_type divider_factor);
```

Initialize input capture channel.

---

#### `tmr_pwm_input_config`

```c
void tmr_pwm_input_config(tmr_type *tmr_x, tmr_input_config_type *input_struct,
                          tmr_channel_input_divider_type divider_factor);
```

Configure PWM input mode (two channels for frequency/duty cycle).

---

#### `tmr_input_channel_filter_set`

```c
void tmr_input_channel_filter_set(tmr_type *tmr_x, tmr_channel_select_type tmr_channel,
                                  uint16_t filter_value);
```

Set input channel digital filter.

---

### Encoder Interface

#### `tmr_encoder_mode_config`

```c
void tmr_encoder_mode_config(tmr_type *tmr_x, tmr_encoder_mode_type encoder_mode,
                             tmr_input_polarity_type ic1_polarity,
                             tmr_input_polarity_type ic2_polarity);
```

Configure encoder interface mode.

---

### Brake and Dead-time

#### `tmr_brkdt_default_para_init`

```c
void tmr_brkdt_default_para_init(tmr_brkdt_config_type *tmr_brkdt_struct);
```

Initialize brake/dead-time config structure with defaults.

---

#### `tmr_brkdt_config`

```c
void tmr_brkdt_config(tmr_type *tmr_x, tmr_brkdt_config_type *brkdt_struct);
```

Configure brake and dead-time.

---

### Master/Slave Synchronization

#### `tmr_primary_mode_select`

```c
void tmr_primary_mode_select(tmr_type *tmr_x, tmr_primary_select_type primary_mode);
```

Select master mode trigger output.

---

#### `tmr_sub_mode_select`

```c
void tmr_sub_mode_select(tmr_type *tmr_x, tmr_sub_mode_select_type sub_mode);
```

Select subordinate (slave) mode.

---

#### `tmr_trigger_input_select`

```c
void tmr_trigger_input_select(tmr_type *tmr_x, sub_tmr_input_sel_type trigger_select);
```

Select trigger input source.

---

### DMA and Interrupts

#### `tmr_interrupt_enable`

```c
void tmr_interrupt_enable(tmr_type *tmr_x, uint32_t tmr_interrupt, confirm_state new_state);
```

Enable or disable timer interrupts.

---

#### `tmr_dma_request_enable`

```c
void tmr_dma_request_enable(tmr_type *tmr_x, tmr_dma_request_type dma_request, confirm_state new_state);
```

Enable or disable DMA requests.

---

#### `tmr_flag_get` / `tmr_interrupt_flag_get`

```c
flag_status tmr_flag_get(tmr_type *tmr_x, uint32_t tmr_flag);
flag_status tmr_interrupt_flag_get(tmr_type *tmr_x, uint32_t tmr_flag);
```

Get flag status.

---

#### `tmr_flag_clear`

```c
void tmr_flag_clear(tmr_type *tmr_x, uint32_t tmr_flag);
```

Clear flag.

---

#### `tmr_event_sw_trigger`

```c
void tmr_event_sw_trigger(tmr_type *tmr_x, tmr_event_trigger_type tmr_event);
```

Generate software event.

---

## Code Examples

### Example 1: Basic Timer with Overflow Interrupt

Periodic interrupt at 1 Hz (1 second).

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

crm_clocks_freq_type crm_clocks_freq;

void TMR1_OVF_TMR10_IRQHandler(void)
{
    if(tmr_interrupt_flag_get(TMR1, TMR_OVF_FLAG) == SET)
    {
        at32_led_toggle(LED2);
        tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    }
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    crm_clocks_freq_get(&crm_clocks_freq);
    
    /* Enable TMR1 clock */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    
    /* Configure timer for 1 Hz (1 second overflow)
       Timer clock = APB2 * 2 = 288 MHz
       Period = 10000, Prescaler = (288MHz/10000/1Hz) - 1 = 28799 */
    tmr_base_init(TMR1, 9999, (crm_clocks_freq.apb2_freq * 2 / 10000) - 1);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
    
    /* Enable overflow interrupt */
    tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);
    
    /* Configure NVIC */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 1, 0);
    
    /* Start timer */
    tmr_counter_enable(TMR1, TRUE);
    
    while(1) { }
}
```

---

### Example 2: PWM Output (4 Channels)

Generate 4 PWM signals with different duty cycles.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

tmr_output_config_type tmr_oc_init;
crm_clocks_freq_type crm_clocks_freq;

void gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    
    /* TMR3 CH1 (PA6), CH2 (PA7), CH3 (PB0), CH4 (PB1) */
    gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
    gpio_init(GPIOB, &gpio_init_struct);
    
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE6, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_2);
}

int main(void)
{
    uint16_t period = 999;  /* 1000 steps (0-999) */
    
    system_clock_config();
    at32_board_init();
    crm_clocks_freq_get(&crm_clocks_freq);
    
    gpio_config();
    
    /* Enable TMR3 clock */
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
    
    /* Configure TMR3 for ~36 kHz PWM (APB1*2/4000 prescaler at 144 MHz) */
    tmr_base_init(TMR3, period, (crm_clocks_freq.apb1_freq * 2 / 36000000) - 1);
    tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
    
    /* Configure PWM channels */
    tmr_output_default_para_init(&tmr_oc_init);
    tmr_oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_oc_init.oc_output_state = TRUE;
    
    /* CH1: 50% duty cycle */
    tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_1, &tmr_oc_init);
    tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_1, 500);
    tmr_output_channel_buffer_enable(TMR3, TMR_SELECT_CHANNEL_1, TRUE);
    
    /* CH2: 37.5% duty cycle */
    tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_2, &tmr_oc_init);
    tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_2, 375);
    tmr_output_channel_buffer_enable(TMR3, TMR_SELECT_CHANNEL_2, TRUE);
    
    /* CH3: 25% duty cycle */
    tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_3, &tmr_oc_init);
    tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_3, 250);
    tmr_output_channel_buffer_enable(TMR3, TMR_SELECT_CHANNEL_3, TRUE);
    
    /* CH4: 12.5% duty cycle */
    tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_4, &tmr_oc_init);
    tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_4, 125);
    tmr_output_channel_buffer_enable(TMR3, TMR_SELECT_CHANNEL_4, TRUE);
    
    /* Enable period buffer */
    tmr_period_buffer_enable(TMR3, TRUE);
    
    /* Start timer */
    tmr_counter_enable(TMR3, TRUE);
    
    while(1) { }
}
```

---

### Example 3: Input Capture (Frequency Measurement)

Measure external signal frequency.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

crm_clocks_freq_type crm_clocks_freq;
volatile uint32_t measured_frequency = 0;
volatile uint16_t capture1 = 0, capture2 = 0;
volatile uint8_t capture_number = 0;

void TMR3_GLOBAL_IRQHandler(void)
{
    if(tmr_interrupt_flag_get(TMR3, TMR_C2_FLAG) == SET)
    {
        tmr_flag_clear(TMR3, TMR_C2_FLAG);
        
        if(capture_number == 0)
        {
            capture1 = tmr_channel_value_get(TMR3, TMR_SELECT_CHANNEL_2);
            capture_number = 1;
        }
        else
        {
            capture2 = tmr_channel_value_get(TMR3, TMR_SELECT_CHANNEL_2);
            
            uint32_t period;
            if(capture2 > capture1)
                period = capture2 - capture1;
            else
                period = (0x10000 - capture1) + capture2;
            
            measured_frequency = crm_clocks_freq.sclk_freq / period;
            capture_number = 0;
        }
    }
}

int main(void)
{
    gpio_init_type gpio_init_struct;
    tmr_input_config_type tmr_input_config;
    
    system_clock_config();
    at32_board_init();
    crm_clocks_freq_get(&crm_clocks_freq);
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA7 as TMR3_CH2 input */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_7;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_2);
    
    /* Configure TMR3 for input capture */
    tmr_base_init(TMR3, 0xFFFF, 0);  /* Max period, no prescaler */
    tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
    
    /* Configure channel 2 input capture */
    tmr_input_default_para_init(&tmr_input_config);
    tmr_input_config.input_channel_select = TMR_SELECT_CHANNEL_2;
    tmr_input_config.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    tmr_input_config.input_polarity_select = TMR_INPUT_RISING_EDGE;
    tmr_input_config.input_filter_value = 0;
    tmr_input_channel_init(TMR3, &tmr_input_config, TMR_CHANNEL_INPUT_DIV_1);
    
    /* Enable capture interrupt */
    tmr_interrupt_enable(TMR3, TMR_C2_INT, TRUE);
    
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(TMR3_GLOBAL_IRQn, 1, 0);
    
    /* Start timer */
    tmr_counter_enable(TMR3, TRUE);
    
    while(1)
    {
        /* measured_frequency contains the result */
    }
}
```

---

### Example 4: Encoder Interface

Quadrature encoder decoding.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

int main(void)
{
    gpio_init_type gpio_init_struct;
    int32_t encoder_position;
    
    system_clock_config();
    at32_board_init();
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA0 (TMR2_CH1) and PA1 (TMR2_CH2) as encoder inputs */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE0, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE1, GPIO_MUX_1);
    
    /* Enable 32-bit mode for TMR2 */
    tmr_32_bit_function_enable(TMR2, TRUE);
    
    /* Configure TMR2 */
    tmr_base_init(TMR2, 0xFFFFFFFF, 0);
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    
    /* Configure encoder mode C (count on both edges of both channels) */
    tmr_encoder_mode_config(TMR2, TMR_ENCODER_MODE_C, 
                            TMR_INPUT_RISING_EDGE, 
                            TMR_INPUT_RISING_EDGE);
    
    /* Start timer */
    tmr_counter_enable(TMR2, TRUE);
    
    while(1)
    {
        /* Read encoder position */
        encoder_position = (int32_t)tmr_counter_value_get(TMR2);
        
        /* Use encoder_position value */
    }
}
```

---

### Example 5: Complementary PWM with Dead-time

Motor control with brake protection.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

crm_clocks_freq_type crm_clocks_freq;

void gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    
    /* TMR1 CH1/CH1N, CH2/CH2N, CH3/CH3N */
    gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
    gpio_init(GPIOB, &gpio_init_struct);
    
    /* TMR1 BKIN (brake input) */
    gpio_init_struct.gpio_pins = GPIO_PINS_12;
    gpio_init(GPIOB, &gpio_init_struct);
    
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE13, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE14, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE15, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE12, GPIO_MUX_1);
}

int main(void)
{
    tmr_output_config_type tmr_output_struct;
    tmr_brkdt_config_type tmr_brkdt_struct;
    uint16_t period;
    
    system_clock_config();
    at32_board_init();
    crm_clocks_freq_get(&crm_clocks_freq);
    
    gpio_config();
    
    /* Enable TMR1 clock */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    
    /* Configure for ~20 kHz PWM */
    period = ((crm_clocks_freq.apb2_freq * 2) / 20000) - 1;
    tmr_base_init(TMR1, period, 0);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
    
    /* Configure output channels */
    tmr_output_default_para_init(&tmr_output_struct);
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.occ_output_state = TRUE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state = FALSE;
    tmr_output_struct.occ_idle_state = FALSE;
    
    /* Channel 1: 50% duty */
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, period / 2);
    
    /* Channel 2: 25% duty */
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, period / 4);
    
    /* Channel 3: 12.5% duty */
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, period / 8);
    
    /* Configure brake and dead-time */
    tmr_brkdt_default_para_init(&tmr_brkdt_struct);
    tmr_brkdt_struct.deadtime = 50;              /* ~175ns at 288MHz */
    tmr_brkdt_struct.brk_enable = TRUE;
    tmr_brkdt_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_HIGH;
    tmr_brkdt_struct.auto_output_enable = TRUE;
    tmr_brkdt_struct.fcsoen_state = TRUE;
    tmr_brkdt_struct.fcsodis_state = TRUE;
    tmr_brkdt_struct.wp_level = TMR_WP_LEVEL_1;
    tmr_brkdt_config(TMR1, &tmr_brkdt_struct);
    
    /* Enable main output */
    tmr_output_enable(TMR1, TRUE);
    
    /* Start timer */
    tmr_counter_enable(TMR1, TRUE);
    
    while(1) { }
}
```

---

### Example 6: PWM with DMA Update

Automatic duty cycle update via DMA.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

crm_clocks_freq_type crm_clocks_freq;
uint16_t duty_cycle_values[3] = {0, 0, 0};

int main(void)
{
    tmr_output_config_type tmr_output_struct;
    dma_init_type dma_init_struct;
    uint16_t period;
    
    system_clock_config();
    at32_board_init();
    crm_clocks_freq_get(&crm_clocks_freq);
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    
    /* Configure GPIO for TMR1_CH3 and CH3N */
    /* ... GPIO configuration ... */
    
    /* Configure PWM for ~20 kHz */
    period = ((crm_clocks_freq.apb2_freq * 2) / 20000) - 1;
    
    /* Precompute duty cycle values */
    duty_cycle_values[0] = (period * 50) / 100;   /* 50% */
    duty_cycle_values[1] = (period * 375) / 1000; /* 37.5% */
    duty_cycle_values[2] = (period * 25) / 100;   /* 25% */
    
    tmr_base_init(TMR1, period, 0);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
    
    /* Configure channel 3 */
    tmr_output_default_para_init(&tmr_output_struct);
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.occ_output_state = TRUE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, duty_cycle_values[0]);
    
    /* Enable overflow DMA request */
    tmr_dma_request_enable(TMR1, TMR_OVERFLOW_DMA_REQUEST, TRUE);
    
    /* Configure DMA */
    dma_reset(DMA1_CHANNEL1);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = 3;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)duty_cycle_values;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&TMR1->c3dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.loop_mode_enable = TRUE;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    
    dmamux_enable(DMA1, TRUE);
    dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_TMR1_OVERFLOW);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    
    /* Enable main output */
    tmr_output_enable(TMR1, TRUE);
    
    /* Start timer */
    tmr_counter_enable(TMR1, TRUE);
    
    while(1) { }
}
```

---

## Configuration Checklist

### Basic Timer Setup

- [ ] Enable GPIO clock for timer pins
- [ ] Enable timer peripheral clock: `crm_periph_clock_enable()`
- [ ] Configure GPIO pins as alternate function
- [ ] Set GPIO MUX configuration: `gpio_pin_mux_config()`
- [ ] Initialize time base: `tmr_base_init()`
- [ ] Set counter direction: `tmr_cnt_dir_set()`

### PWM Output

- [ ] Configure output channel: `tmr_output_channel_config()`
- [ ] Set compare value: `tmr_channel_value_set()`
- [ ] Enable channel buffer: `tmr_output_channel_buffer_enable()`
- [ ] For advanced timers: `tmr_output_enable(TRUE)`
- [ ] Start counter: `tmr_counter_enable(TRUE)`

### Input Capture

- [ ] Configure input channel: `tmr_input_channel_init()`
- [ ] Enable capture interrupt: `tmr_interrupt_enable()`
- [ ] Configure NVIC
- [ ] Start counter: `tmr_counter_enable(TRUE)`

### Encoder Mode

- [ ] Configure encoder: `tmr_encoder_mode_config()`
- [ ] For TMR2/TMR5: `tmr_32_bit_function_enable(TRUE)`
- [ ] Start counter: `tmr_counter_enable(TRUE)`

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| No PWM output | Main output not enabled | Call `tmr_output_enable(TRUE)` for advanced timers |
| PWM frequency wrong | Incorrect prescaler/period | Verify clock source and calculation |
| Encoder counts wrong | Polarity mismatch | Check input polarity settings |
| DMA not working | DMAMUX not configured | Enable DMAMUX and set correct request ID |
| Brake not triggering | Wrong polarity | Check brake input polarity setting |
| Complementary outputs same | Dead-time too large | Reduce dead-time value |

---

## Related Peripherals

- **[GPIO](GPIO_General_Purpose_IO.md)** - Timer pin configuration
- **[DMA](DMA_Direct_Memory_Access.md)** - Timer DMA transfers
- **[ADC](ADC_Analog_to_Digital_Converter.md)** - Timer-triggered conversions
- **[DAC](DAC_Digital_to_Analog_Converter.md)** - Timer-triggered waveforms
- **[IRTMR](IRTMR_Infrared_Timer.md)** - Infrared modulation (TMR10/TMR11)

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-01 | Initial release |

