---
title: IRTMR - Infrared Timer
category: Peripheral
complexity: Basic
mcu: AT32F435/437
peripheral: IRTMR
keywords: [irtmr, infrared, ir, remote, modulation, tmr10, tmr11, carrier, nec, rc5]
---

# IRTMR - Infrared Timer

## Overview

The Infrared Timer (IRTMR) provides hardware support for generating infrared remote control signals. It combines the outputs of TMR10 (carrier frequency) and TMR11 (data envelope) through an AND gate to produce a modulated infrared signal suitable for driving IR LEDs. This eliminates the need for software bit-banging and ensures precise timing for IR protocols like NEC, RC5, SIRC, and others.

### Key Features

| Feature | Specification |
|---------|---------------|
| Carrier Source | TMR10 Channel 1 PWM output |
| Envelope Source | TMR11 Channel 1 PWM output |
| Modulation | Internal AND gate (carrier × envelope) |
| Output Pin | PB9 (IRTMR_OUT) |
| Polarity | Configurable (normal or inverted) |
| Carrier Frequency | Programmable via TMR10 (typ. 30-56 kHz) |
| Common IR Frequencies | 36 kHz, 38 kHz, 40 kHz, 56 kHz |
| Configuration | Via SCFG peripheral |

---

## Architecture

```
                    ┌──────────────────────────────────────────────────────┐
                    │                  IRTMR Module                         │
                    │                                                       │
  ┌─────────────┐   │   ┌───────────────────────────────────────────────┐  │
  │   TMR10     │   │   │                                               │  │
  │             │   │   │     ┌─────────┐                               │  │
  │  Carrier    │───┼──►│     │         │                               │  │
  │  Generator  │   │   │     │   AND   │────►┌─────────┐              │  │
  │  (38kHz)    │   │   │     │   Gate  │     │Polarity │──────►PB9    │  │
  │             │   │   │     │         │     │ Select  │     (IRTMR)  │  │
  └─────────────┘   │   │     └────┬────┘     └─────────┘              │  │
                    │   │          │                                    │  │
  ┌─────────────┐   │   │          │                                    │  │
  │   TMR11     │   │   │     ┌────┴────┐                               │  │
  │             │   │   │     │         │                               │  │
  │  Envelope   │───┼──►│     │   AND   │                               │  │
  │  Generator  │   │   │     │   Gate  │                               │  │
  │  (Data)     │   │   │     │         │                               │  │
  └─────────────┘   │   │     └─────────┘                               │  │
                    │   │                                               │  │
                    │   └───────────────────────────────────────────────┘  │
                    │                                                       │
                    │   Configuration via SCFG->CFG1                        │
                    │   - IR_SRC_SEL: Source selection (TMR10)             │
                    │   - IR_POL: Output polarity                          │
                    └──────────────────────────────────────────────────────┘
```

---

## Signal Generation

### Modulated IR Signal

```
TMR10 (Carrier 38kHz):
    ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐
    │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │ │
  ──┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └──

TMR11 (Envelope):
    ┌───────────────────┐                 ┌────────┐
    │                   │                 │        │
  ──┘                   └─────────────────┘        └─────────────

IRTMR Output (Carrier AND Envelope):
    ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐                   ┌─┐ ┌─┐ ┌─┐
    │ │ │ │ │ │ │ │ │ │                   │ │ │ │ │ │
  ──┘ └─┘ └─┘ └─┘ └─┘ └───────────────────┘ └─┘ └─┘ └────────────
    │◄── Burst ──────►│◄── Space ────────►│◄─ Burst ─►│
```

---

## Pin Configuration

| Pin | Function | GPIO MUX | Description |
|-----|----------|----------|-------------|
| PB8 | TMR10_CH1 | GPIO_MUX_3 | Timer 10 channel 1 output (carrier, optional debug) |
| PB9 | IRTMR_OUT | GPIO_MUX_0 | Modulated infrared output |

---

## Configuration Types

### IR Signal Source

```c
typedef enum {
  SCFG_IR_SOURCE_TMR10 = 0x00  /* Infrared signal source: TMR10 */
} scfg_ir_source_type;
```

### IR Output Polarity

```c
typedef enum {
  SCFG_IR_POLARITY_NO_AFFECTE = 0x00,  /* Output polarity unchanged */
  SCFG_IR_POLARITY_REVERSE    = 0x01   /* Output polarity inverted */
} scfg_ir_polarity_type;
```

---

## Register Configuration (SCFG->CFG1)

| Field | Bits | Description |
|-------|------|-------------|
| IR_SRC_SEL | [7:6] | IR signal source selection (0x00 = TMR10) |
| IR_POL | [5] | IR output polarity (0 = normal, 1 = inverted) |

---

## API Reference

### SCFG Infrared Configuration

```c
/**
  * @brief  Configure infrared output
  * @param  source: IR signal source
  *         - SCFG_IR_SOURCE_TMR10
  * @param  polarity: Output polarity
  *         - SCFG_IR_POLARITY_NO_AFFECTE
  *         - SCFG_IR_POLARITY_REVERSE
  * @retval none
  */
void scfg_infrared_config(scfg_ir_source_type source, scfg_ir_polarity_type polarity);
```

### Timer Configuration (for TMR10/TMR11)

```c
/* Timer base initialization */
void tmr_base_init(tmr_type *tmr_x, uint32_t arr, uint32_t div);
void tmr_cnt_dir_set(tmr_type *tmr_x, tmr_count_mode_type cnt_dir);
void tmr_clock_source_div_set(tmr_type *tmr_x, tmr_clock_division_type div);

/* Output compare configuration */
void tmr_output_default_para_init(tmr_output_config_type *init_struct);
void tmr_output_channel_config(tmr_type *tmr_x, tmr_channel_select_type channel,
                                tmr_output_config_type *init_struct);
void tmr_channel_value_set(tmr_type *tmr_x, tmr_channel_select_type channel, uint32_t value);
void tmr_output_channel_buffer_enable(tmr_type *tmr_x, tmr_channel_select_type channel,
                                       confirm_state new_state);
void tmr_period_buffer_enable(tmr_type *tmr_x, confirm_state new_state);

/* Timer enable */
void tmr_counter_enable(tmr_type *tmr_x, confirm_state new_state);
```

---

## Common IR Carrier Frequencies

| Protocol | Carrier Frequency | Duty Cycle |
|----------|-------------------|------------|
| NEC | 38 kHz | 33% |
| RC5/RC6 | 36 kHz | 25-33% |
| SIRC (Sony) | 40 kHz | 33% |
| Sharp | 38 kHz | 33% |
| Samsung | 38 kHz | 33% |
| Panasonic | 36.7 kHz | 33% |

---

## Timer Configuration Calculations

### Carrier Frequency (TMR10)

```
Carrier Period = (ARR + 1) × (PSC + 1) / Timer_Clock

For 38 kHz carrier with 1 MHz timer clock:
  Period = 1,000,000 / 38,000 ≈ 26.3 µs
  ARR = 26 (for period of 27 cycles)

For 33% duty cycle:
  CCR = (ARR + 1) × 0.33 ≈ 9
```

### Data Envelope (TMR11)

The envelope timing depends on the IR protocol:

| Protocol | Logical 0 | Logical 1 | Leader |
|----------|-----------|-----------|--------|
| NEC | 562.5µs burst + 562.5µs space | 562.5µs burst + 1687.5µs space | 9ms burst + 4.5ms space |
| RC5 | 889µs burst + 889µs space | 889µs space + 889µs burst | N/A (Manchester) |
| SIRC | 600µs burst + 600µs space | 1200µs burst + 600µs space | 2400µs burst + 600µs space |

---

## Code Examples

### Example 1: Basic IRTMR Configuration

```c
#include "at32f435_437.h"

/**
  * @brief  Configure clocks for IRTMR
  */
void irtmr_clock_config(void)
{
    /* Enable TMR10, TMR11, GPIOB, and SCFG clocks */
    crm_periph_clock_enable(CRM_TMR10_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR11_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
}

/**
  * @brief  Configure GPIO for IRTMR
  */
void irtmr_gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    
    /* PB8: TMR10_CH1 (optional - for debugging carrier) */
    gpio_init_struct.gpio_pins = GPIO_PINS_8;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE8, GPIO_MUX_3);
    
    /* PB9: IRTMR_OUT (modulated output) */
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9, GPIO_MUX_0);
}

/**
  * @brief  Configure IRTMR for 38kHz carrier
  */
void irtmr_config(void)
{
    tmr_output_config_type tmr_oc_struct;
    crm_clocks_freq_type clocks;
    uint16_t prescaler;
    
    crm_clocks_freq_get(&clocks);
    
    /* Configure SCFG for IR output */
    scfg_infrared_config(SCFG_IR_SOURCE_TMR10, SCFG_IR_POLARITY_NO_AFFECTE);
    
    /* Calculate prescaler for 1 MHz timer clock */
    prescaler = (clocks.apb1_freq * 2 / 1000000) - 1;
    
    /* TMR10: 38 kHz carrier (26 µs period, ~33% duty) */
    tmr_base_init(TMR10, 25, prescaler);  /* Period = 26 cycles = 26 µs */
    tmr_cnt_dir_set(TMR10, TMR_COUNT_UP);
    
    tmr_output_default_para_init(&tmr_oc_struct);
    tmr_oc_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_oc_struct.oc_output_state = TRUE;
    tmr_oc_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_channel_config(TMR10, TMR_SELECT_CHANNEL_1, &tmr_oc_struct);
    tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1, 8);  /* ~33% duty */
    tmr_output_channel_buffer_enable(TMR10, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_period_buffer_enable(TMR10, TRUE);
    
    /* TMR11: Envelope (initially disabled - all high) */
    tmr_base_init(TMR11, 999, prescaler);  /* 1 ms period */
    tmr_cnt_dir_set(TMR11, TMR_COUNT_UP);
    
    tmr_output_default_para_init(&tmr_oc_struct);
    tmr_oc_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_oc_struct.oc_output_state = TRUE;
    tmr_oc_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_channel_config(TMR11, TMR_SELECT_CHANNEL_1, &tmr_oc_struct);
    tmr_channel_value_set(TMR11, TMR_SELECT_CHANNEL_1, 500);  /* 50% initial */
    tmr_output_channel_buffer_enable(TMR11, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_period_buffer_enable(TMR11, TRUE);
    
    /* Enable both timers */
    tmr_counter_enable(TMR10, TRUE);
    tmr_counter_enable(TMR11, TRUE);
}

int main(void)
{
    system_clock_config();
    
    irtmr_clock_config();
    irtmr_gpio_config();
    irtmr_config();
    
    while(1)
    {
        /* IR output is now active on PB9 */
    }
}
```

---

### Example 2: NEC Protocol Transmission

```c
#include "at32f435_437.h"

/* NEC timing constants (in microseconds) */
#define NEC_LEADER_BURST    9000
#define NEC_LEADER_SPACE    4500
#define NEC_BIT_BURST       562
#define NEC_ZERO_SPACE      562
#define NEC_ONE_SPACE       1687
#define NEC_REPEAT_SPACE    2250

volatile uint32_t ir_bit_index = 0;
volatile uint32_t ir_data = 0;
volatile uint8_t ir_sending = 0;

/**
  * @brief  Delay in microseconds using SysTick
  */
void delay_us(uint32_t us)
{
    uint32_t ticks = (SystemCoreClock / 1000000) * us;
    uint32_t start = SysTick->VAL;
    
    while((start - SysTick->VAL) < ticks);
}

/**
  * @brief  Enable IR burst (carrier on)
  */
void ir_burst_on(void)
{
    /* Set TMR11 to output high (enable carrier) */
    tmr_channel_value_set(TMR11, TMR_SELECT_CHANNEL_1, 999);
}

/**
  * @brief  Disable IR burst (carrier off)
  */
void ir_burst_off(void)
{
    /* Set TMR11 to output low (disable carrier) */
    tmr_channel_value_set(TMR11, TMR_SELECT_CHANNEL_1, 0);
}

/**
  * @brief  Send NEC protocol leader
  */
void nec_send_leader(void)
{
    ir_burst_on();
    delay_us(NEC_LEADER_BURST);
    ir_burst_off();
    delay_us(NEC_LEADER_SPACE);
}

/**
  * @brief  Send NEC protocol bit
  * @param  bit: 0 or 1
  */
void nec_send_bit(uint8_t bit)
{
    ir_burst_on();
    delay_us(NEC_BIT_BURST);
    ir_burst_off();
    
    if(bit)
    {
        delay_us(NEC_ONE_SPACE);
    }
    else
    {
        delay_us(NEC_ZERO_SPACE);
    }
}

/**
  * @brief  Send NEC protocol byte (LSB first)
  * @param  data: byte to send
  */
void nec_send_byte(uint8_t data)
{
    uint8_t i;
    
    for(i = 0; i < 8; i++)
    {
        nec_send_bit(data & 0x01);
        data >>= 1;
    }
}

/**
  * @brief  Send complete NEC frame
  * @param  address: 8-bit device address
  * @param  command: 8-bit command
  */
void nec_send_command(uint8_t address, uint8_t command)
{
    /* Leader */
    nec_send_leader();
    
    /* Address */
    nec_send_byte(address);
    
    /* Inverted address */
    nec_send_byte(~address);
    
    /* Command */
    nec_send_byte(command);
    
    /* Inverted command */
    nec_send_byte(~command);
    
    /* Stop bit */
    ir_burst_on();
    delay_us(NEC_BIT_BURST);
    ir_burst_off();
}

/**
  * @brief  Send NEC repeat code
  */
void nec_send_repeat(void)
{
    ir_burst_on();
    delay_us(NEC_LEADER_BURST);
    ir_burst_off();
    delay_us(NEC_REPEAT_SPACE);
    ir_burst_on();
    delay_us(NEC_BIT_BURST);
    ir_burst_off();
}

int main(void)
{
    system_clock_config();
    
    /* ... clock and GPIO configuration ... */
    
    /* ... IRTMR configuration for 38kHz ... */
    
    /* Initially turn off carrier */
    ir_burst_off();
    
    while(1)
    {
        /* Send power button command (example) */
        nec_send_command(0x00, 0x45);  /* Address 0x00, Command 0x45 */
        
        /* Wait before next transmission */
        delay_ms(1000);
    }
}
```

---

### Example 3: Using Interrupt for IR Transmission

```c
#include "at32f435_437.h"

#define IR_STATE_IDLE       0
#define IR_STATE_LEADER     1
#define IR_STATE_DATA       2
#define IR_STATE_STOP       3

volatile uint8_t ir_state = IR_STATE_IDLE;
volatile uint32_t ir_data = 0;
volatile uint8_t ir_bit_count = 0;
volatile uint8_t ir_burst_phase = 0;

/**
  * @brief  Configure TMR11 for interrupt-driven IR
  */
void ir_timer_config(void)
{
    /* TMR11 generates interrupts for timing control */
    tmr_base_init(TMR11, 561, 0);  /* 562 µs base timing */
    tmr_cnt_dir_set(TMR11, TMR_COUNT_UP);
    
    tmr_interrupt_enable(TMR11, TMR_OVF_INT, TRUE);
    nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 0, 0);  /* TMR11 shares this IRQ */
}

/**
  * @brief  Start NEC transmission
  * @param  data: 32-bit NEC frame (address + inv_address + command + inv_command)
  */
void ir_start_transmission(uint32_t data)
{
    ir_data = data;
    ir_bit_count = 0;
    ir_burst_phase = 1;  /* Start with burst */
    ir_state = IR_STATE_LEADER;
    
    /* Set period for leader burst (9ms = 16 × 562µs) */
    tmr_base_init(TMR11, 8999, (SystemCoreClock / 1000000) - 1);
    
    /* Enable carrier */
    ir_burst_on();
    
    /* Start timer */
    tmr_counter_enable(TMR11, TRUE);
}

/**
  * @brief  TMR11 overflow interrupt handler
  */
void TMR1_OVF_TMR10_IRQHandler(void)
{
    if(tmr_flag_get(TMR11, TMR_OVF_FLAG) != RESET)
    {
        tmr_flag_clear(TMR11, TMR_OVF_FLAG);
        
        switch(ir_state)
        {
            case IR_STATE_LEADER:
                if(ir_burst_phase)
                {
                    /* End of leader burst, start space (4.5ms) */
                    ir_burst_off();
                    tmr_base_init(TMR11, 4499, (SystemCoreClock / 1000000) - 1);
                    ir_burst_phase = 0;
                }
                else
                {
                    /* End of leader space, start data */
                    ir_state = IR_STATE_DATA;
                    ir_burst_phase = 1;
                    ir_burst_on();
                    tmr_base_init(TMR11, 561, (SystemCoreClock / 1000000) - 1);
                }
                break;
                
            case IR_STATE_DATA:
                if(ir_burst_phase)
                {
                    /* End of bit burst, start space */
                    ir_burst_off();
                    ir_burst_phase = 0;
                    
                    if(ir_data & (1 << ir_bit_count))
                    {
                        /* Logic 1: 1687µs space */
                        tmr_base_init(TMR11, 1686, (SystemCoreClock / 1000000) - 1);
                    }
                    else
                    {
                        /* Logic 0: 562µs space */
                        tmr_base_init(TMR11, 561, (SystemCoreClock / 1000000) - 1);
                    }
                }
                else
                {
                    /* End of space */
                    ir_bit_count++;
                    
                    if(ir_bit_count < 32)
                    {
                        /* More bits to send */
                        ir_burst_phase = 1;
                        ir_burst_on();
                        tmr_base_init(TMR11, 561, (SystemCoreClock / 1000000) - 1);
                    }
                    else
                    {
                        /* All bits sent, send stop bit */
                        ir_state = IR_STATE_STOP;
                        ir_burst_phase = 1;
                        ir_burst_on();
                        tmr_base_init(TMR11, 561, (SystemCoreClock / 1000000) - 1);
                    }
                }
                break;
                
            case IR_STATE_STOP:
                /* Stop bit complete */
                ir_burst_off();
                tmr_counter_enable(TMR11, FALSE);
                ir_state = IR_STATE_IDLE;
                break;
        }
    }
}

int main(void)
{
    system_clock_config();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    /* ... IRTMR and GPIO configuration ... */
    
    ir_timer_config();
    
    while(1)
    {
        if(ir_state == IR_STATE_IDLE)
        {
            /* Build NEC frame: address=0x00, command=0x45 */
            uint32_t frame = 0x00 | (0xFF << 8) | (0x45 << 16) | (0xBA << 24);
            ir_start_transmission(frame);
        }
        
        delay_ms(500);
    }
}
```

---

### Example 4: RC5 Protocol (Manchester Encoding)

```c
#include "at32f435_437.h"

/* RC5 timing: half-bit period = 889 µs */
#define RC5_HALF_BIT_US     889
#define RC5_BIT_US          (RC5_HALF_BIT_US * 2)

/**
  * @brief  Send RC5 half-bit
  * @param  level: 0 = space, 1 = burst
  */
void rc5_send_half_bit(uint8_t level)
{
    if(level)
    {
        ir_burst_on();
    }
    else
    {
        ir_burst_off();
    }
    delay_us(RC5_HALF_BIT_US);
}

/**
  * @brief  Send RC5 Manchester encoded bit
  * @param  bit: 0 or 1
  * @note   Logic 1 = space then burst, Logic 0 = burst then space
  */
void rc5_send_bit(uint8_t bit)
{
    if(bit)
    {
        /* Logic 1: low-to-high transition */
        rc5_send_half_bit(0);
        rc5_send_half_bit(1);
    }
    else
    {
        /* Logic 0: high-to-low transition */
        rc5_send_half_bit(1);
        rc5_send_half_bit(0);
    }
}

/**
  * @brief  Send RC5 frame
  * @param  toggle: toggle bit (0 or 1)
  * @param  address: 5-bit device address
  * @param  command: 6-bit command
  */
void rc5_send_command(uint8_t toggle, uint8_t address, uint8_t command)
{
    uint8_t i;
    
    /* Two start bits (always 1) */
    rc5_send_bit(1);
    rc5_send_bit(1);
    
    /* Toggle bit */
    rc5_send_bit(toggle & 0x01);
    
    /* 5-bit address (MSB first) */
    for(i = 0; i < 5; i++)
    {
        rc5_send_bit((address >> (4 - i)) & 0x01);
    }
    
    /* 6-bit command (MSB first) */
    for(i = 0; i < 6; i++)
    {
        rc5_send_bit((command >> (5 - i)) & 0x01);
    }
    
    ir_burst_off();
}

int main(void)
{
    static uint8_t toggle = 0;
    
    system_clock_config();
    
    /* Configure IRTMR for 36 kHz (RC5 carrier) */
    /* ... configuration code ... */
    
    while(1)
    {
        rc5_send_command(toggle, 0x00, 0x0C);  /* Address 0, Command 12 (Power) */
        toggle ^= 1;  /* Toggle bit changes each press */
        
        delay_ms(500);
    }
}
```

---

## Configuration Checklist

- [ ] Enable CRM clocks: TMR10, TMR11, GPIOB, SCFG
- [ ] Configure PB9 as GPIO_MUX_0 (IRTMR_OUT)
- [ ] Optionally configure PB8 as GPIO_MUX_3 (TMR10_CH1 for debugging)
- [ ] Call `scfg_infrared_config()` to set source and polarity
- [ ] Configure TMR10 for carrier frequency (e.g., 38 kHz)
- [ ] Configure TMR11 for envelope/modulation timing
- [ ] Enable both timers

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| No IR output | GPIO not configured | Verify PB9 is set to GPIO_MUX_0 |
| | SCFG clock not enabled | Enable CRM_SCFG_PERIPH_CLOCK |
| | Timer not running | Check tmr_counter_enable() calls |
| Weak signal | Wrong duty cycle | Adjust TMR10 CCR for ~33% duty |
| | Low drive strength | Use GPIO_DRIVE_STRENGTH_STRONGER |
| Wrong frequency | Incorrect prescaler/ARR | Recalculate timer values |
| Device not responding | Wrong protocol | Verify carrier frequency and timing |
| | Inverted polarity | Try SCFG_IR_POLARITY_REVERSE |
| Intermittent | Timing jitter | Use DMA or disable interrupts during TX |

---

## Performance Specifications

| Parameter | Value |
|-----------|-------|
| Carrier accuracy | ±1% (depends on system clock) |
| Minimum burst | ~1 carrier cycle (~26 µs at 38 kHz) |
| Output current | Up to 8 mA (GPIO) or more with transistor driver |
| Recommended LED current | 50-100 mA with external driver |

---

## Related Peripherals

| Peripheral | Relationship |
|------------|-------------|
| [TMR](TMR_Timer.md) | TMR10/TMR11 provide carrier and envelope |
| [SCFG](SCFG_System_Configuration.md) | Configures IRTMR source and polarity |
| [GPIO](GPIO_General_Purpose_IO.md) | Pin configuration |
| [CRM](CRM_Clock_Reset_Management.md) | Clock enable |

---

## References

- AT32F435/437 Reference Manual - Chapter: System Configuration (SCFG)
- AT32F435/437 Reference Manual - Chapter: General-purpose Timers (TMR)
- Application Note AN0108 - Infrared Remote Control Interface
- NEC Infrared Transmission Protocol
- Philips RC5 Protocol Specification

