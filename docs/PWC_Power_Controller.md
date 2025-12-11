---
title: PWC - Power Controller
category: Peripheral
complexity: Intermediate
mcu: AT32F435/437
peripheral: PWC
keywords: [pwc, power, sleep, deepsleep, standby, ldo, pvm, wakeup, low-power]
---

# PWC - Power Controller

## Overview

The Power Controller (PWC) manages the power supply domains and low-power operating modes of the AT32F435/437 microcontroller. It provides three low-power modes (Sleep, Deep Sleep, Standby) with different trade-offs between power consumption and wake-up latency. Additionally, PWC includes a Power Voltage Monitor (PVM) for brownout detection, LDO voltage control for performance optimization, and battery-powered domain access for ERTC and backup registers.

### Key Features

| Feature | Specification |
|---------|---------------|
| Low-Power Modes | Sleep, Deep Sleep, Standby |
| Wakeup Pins | 2 (PA0, PC13) |
| PVM Thresholds | 2.3V ~ 2.9V (7 levels) |
| LDO Output Voltages | 1.1V, 1.2V, 1.3V |
| Battery Domain | ERTC + Backup Registers access |
| Entry Methods | WFI (Wait for Interrupt), WFE (Wait for Event) |

### Power Mode Comparison

| Mode | Core | Clocks | Voltage Regulator | Wake-up Time | Wake-up Sources |
|------|------|--------|-------------------|--------------|-----------------|
| **Run** | ON | ON | ON | - | - |
| **Sleep** | OFF | ON | ON | ~1 μs | Any interrupt |
| **Deep Sleep** | OFF | OFF | ON/Low-Power | ~3 μs | EXINT, ERTC events |
| **Standby** | OFF | OFF | OFF | ~50 μs | WKUP pins, ERTC alarm, reset |

## Architecture

```
                        ┌──────────────────────────────────────────────┐
                        │              Power Controller (PWC)          │
                        │                                              │
┌─────────────┐         │  ┌─────────────────────────────────────┐    │
│   VDD       │─────────│──│       Power Voltage Monitor (PVM)   │    │
│  (Supply)   │         │  │  ┌─────────────────────────────┐    │    │
└─────────────┘         │  │  │  Threshold: 2.3V ~ 2.9V     │    │    │
                        │  │  │  PVMSEL[2:0]                │    │    │
                        │  │  └───────────────┬─────────────┘    │    │
                        │  │                  │ PVMOF            │    │
                        │  │                  ▼                  │    │
                        │  │         ┌───────────────┐           │────│──► EXINT Line 16
                        │  │         │  Comparator   │           │    │
                        │  └─────────┴───────────────┴───────────┘    │
                        │                                              │
                        │  ┌─────────────────────────────────────┐    │
                        │  │        LDO Voltage Regulator        │    │
                        │  │  ┌─────────────────────────────┐    │    │
                        │  │  │  LDOOVSEL[2:0]              │    │    │
                        │  │  │  • 1.3V → 288 MHz           │────│──► Core Supply
                        │  │  │  • 1.2V → 240 MHz           │    │
                        │  │  │  • 1.1V → 144 MHz           │    │
                        │  └──┴─────────────────────────────┴────┘    │
                        │                                              │
                        │  ┌─────────────────────────────────────┐    │
                        │  │        Low-Power Mode Control       │    │
                        │  │  ┌───────────────────────────────┐  │    │
                        │  │  │  VRSEL  → Regulator mode      │  │    │
                        │  │  │  LPSEL  → Standby select      │  │    │
                        │  │  │  BPWEN  → Battery domain      │  │    │
                        │  └──┴───────────────────────────────┴──┘    │
                        │                                              │
                        │  ┌─────────────────────────────────────┐    │
                        │  │         Wakeup Pin Control          │    │
┌─────────────┐         │  │  ┌───────────────────────────────┐  │    │
│ PA0 (WKUP1) │─────────│──│──│  SWPEN1 → Wakeup Pin 1        │  │    │
└─────────────┘         │  │  │  SWPEN2 → Wakeup Pin 2        │──│──► Standby Wakeup
┌─────────────┐         │  │  └───────────────────────────────┘  │    │
│ PC13(WKUP2) │─────────│──│                                      │    │
└─────────────┘         │  └─────────────────────────────────────┘    │
                        │                                              │
                        │  ┌─────────────────────────────────────┐    │
                        │  │            Status Flags             │    │
                        │  │  • SWEF  (Standby Wakeup Event)     │    │
                        │  │  • SEF   (Standby Entry Flag)       │    │
                        │  │  • PVMOF (PVM Output)               │    │
                        │  └─────────────────────────────────────┘    │
                        └──────────────────────────────────────────────┘
```

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| CTRL | 0x00 | PWC control register |
| CTRLSTS | 0x04 | PWC control/status register |
| LDOOV | 0x10 | LDO output voltage register |

### CTRL Register (0x00)

| Bits | Name | Description |
|------|------|-------------|
| 0 | VRSEL | Voltage regulator mode: 0=ON, 1=Low-power (deep sleep) |
| 1 | LPSEL | Low power mode select: 0=Deep sleep, 1=Standby |
| 2 | CLSWEF | Clear standby wakeup event flag (write 1 to clear) |
| 3 | CLSEF | Clear standby entry flag (write 1 to clear) |
| 4 | PVMEN | Power voltage monitor enable |
| 7:5 | PVMSEL | PVM threshold selection (001=2.3V to 111=2.9V) |
| 8 | BPWEN | Battery powered domain write enable |

### CTRLSTS Register (0x04)

| Bits | Name | Description |
|------|------|-------------|
| 0 | SWEF | Standby wakeup event flag (read-only) |
| 1 | SEF | Standby entry flag (read-only) |
| 2 | PVMOF | PVM output flag (read-only): 1=VDD < threshold |
| 8 | SWPEN1 | Standby wakeup pin 1 (PA0) enable |
| 9 | SWPEN2 | Standby wakeup pin 2 (PC13) enable |

### LDOOV Register (0x10)

| Bits | Name | Description |
|------|------|-------------|
| 2:0 | LDOOVSEL | LDO output voltage selection |

## Configuration Types

### PVM Voltage Thresholds

| Enum | Value | Threshold |
|------|-------|-----------|
| `PWC_PVM_VOLTAGE_2V3` | 0x01 | 2.3V |
| `PWC_PVM_VOLTAGE_2V4` | 0x02 | 2.4V |
| `PWC_PVM_VOLTAGE_2V5` | 0x03 | 2.5V |
| `PWC_PVM_VOLTAGE_2V6` | 0x04 | 2.6V |
| `PWC_PVM_VOLTAGE_2V7` | 0x05 | 2.7V |
| `PWC_PVM_VOLTAGE_2V8` | 0x06 | 2.8V |
| `PWC_PVM_VOLTAGE_2V9` | 0x07 | 2.9V |

### LDO Output Voltages

| Enum | Value | Voltage | Max System Clock | Notes |
|------|-------|---------|------------------|-------|
| `PWC_LDO_OUTPUT_1V3` | 0x01 | 1.3V | 288 MHz | VDD > 3.0V, -40~85°C |
| `PWC_LDO_OUTPUT_1V2` | 0x00 | 1.2V | 240 MHz | Default |
| `PWC_LDO_OUTPUT_1V1` | 0x04 | 1.1V | 144 MHz | Low power |

### Sleep Entry Methods

| Enum | Value | Description |
|------|-------|-------------|
| `PWC_SLEEP_ENTER_WFI` | 0x00 | Wait for Interrupt |
| `PWC_SLEEP_ENTER_WFE` | 0x01 | Wait for Event |

### Deep Sleep Entry Methods

| Enum | Value | Description |
|------|-------|-------------|
| `PWC_DEEP_SLEEP_ENTER_WFI` | 0x00 | Wait for Interrupt |
| `PWC_DEEP_SLEEP_ENTER_WFE` | 0x01 | Wait for Event |

### Voltage Regulator Modes

| Enum | Value | Description |
|------|-------|-------------|
| `PWC_REGULATOR_ON` | 0x00 | Regulator ON in deep sleep |
| `PWC_REGULATOR_LOW_POWER` | 0x01 | Regulator low-power in deep sleep |

## Wakeup Sources

### Sleep Mode Wakeup

- Any enabled peripheral interrupt
- Any EXINT line interrupt
- SysTick interrupt

### Deep Sleep Mode Wakeup

- EXINT lines (GPIO, ERTC, PVM, etc.)
- ERTC Alarm (via EXINT Line 17)
- ERTC Wakeup Timer (via EXINT Line 22)
- ERTC Tamper/Timestamp (via EXINT Line 21)

### Standby Mode Wakeup

| Source | Pin/Event | Configuration |
|--------|-----------|---------------|
| WKUP Pin 1 | PA0 (rising edge) | `pwc_wakeup_pin_enable(PWC_WAKEUP_PIN_1, TRUE)` |
| WKUP Pin 2 | PC13 (rising edge) | `pwc_wakeup_pin_enable(PWC_WAKEUP_PIN_2, TRUE)` |
| ERTC Alarm A | Internal | ERTC Alarm A interrupt enabled |
| ERTC Alarm B | Internal | ERTC Alarm B interrupt enabled |
| Reset | NRST pin | External reset |

## Flags

| Flag Macro | Bit | Description | Clearable |
|------------|-----|-------------|-----------|
| `PWC_WAKEUP_FLAG` | SWEF | Standby wakeup event occurred | Yes |
| `PWC_STANDBY_FLAG` | SEF | MCU was in standby mode | Yes |
| `PWC_PVM_OUTPUT_FLAG` | PVMOF | VDD below PVM threshold | No (read-only) |

## API Reference

### Initialization

#### `pwc_reset`

```c
void pwc_reset(void);
```

Reset PWC peripheral to default values.

---

### Battery Domain Access

#### `pwc_battery_powered_domain_access`

```c
void pwc_battery_powered_domain_access(confirm_state new_state);
```

Enable or disable write access to battery-powered domain (ERTC, backup registers).

| Parameter | Description |
|-----------|-------------|
| `new_state` | TRUE to enable access, FALSE to disable |

**Note:** Must be called before configuring ERTC or backup registers.

---

### Power Voltage Monitor

#### `pwc_pvm_level_select`

```c
void pwc_pvm_level_select(pwc_pvm_voltage_type pvm_voltage);
```

Set the PVM threshold voltage level.

| Parameter | Description |
|-----------|-------------|
| `pvm_voltage` | Threshold: `PWC_PVM_VOLTAGE_2V3` to `PWC_PVM_VOLTAGE_2V9` |

---

#### `pwc_power_voltage_monitor_enable`

```c
void pwc_power_voltage_monitor_enable(confirm_state new_state);
```

Enable or disable the Power Voltage Monitor.

| Parameter | Description |
|-----------|-------------|
| `new_state` | TRUE to enable PVM, FALSE to disable |

---

### Low-Power Modes

#### `pwc_sleep_mode_enter`

```c
void pwc_sleep_mode_enter(pwc_sleep_enter_type pwc_sleep_enter);
```

Enter sleep mode. CPU clock is stopped, peripherals continue running.

| Parameter | Description |
|-----------|-------------|
| `pwc_sleep_enter` | Entry method: `PWC_SLEEP_ENTER_WFI` or `PWC_SLEEP_ENTER_WFE` |

**Behavior:**
- Clears SLEEPDEEP bit in SCB->SCR
- Executes WFI or WFE instruction
- Returns after wakeup interrupt/event

---

#### `pwc_deep_sleep_mode_enter`

```c
void pwc_deep_sleep_mode_enter(pwc_deep_sleep_enter_type pwc_deep_sleep_enter);
```

Enter deep sleep mode. All clocks except LEXT/LICK are stopped.

| Parameter | Description |
|-----------|-------------|
| `pwc_deep_sleep_enter` | Entry method: `PWC_DEEP_SLEEP_ENTER_WFI` or `PWC_DEEP_SLEEP_ENTER_WFE` |

**Behavior:**
- Sets SLEEPDEEP bit in SCB->SCR
- Executes WFI or WFE instruction
- Clears SLEEPDEEP bit after wakeup
- System clock switches to HICK after wakeup

---

#### `pwc_voltage_regulate_set`

```c
void pwc_voltage_regulate_set(pwc_regulator_type pwc_regulator);
```

Configure voltage regulator behavior during deep sleep.

| Parameter | Description |
|-----------|-------------|
| `pwc_regulator` | `PWC_REGULATOR_ON` or `PWC_REGULATOR_LOW_POWER` |

**Note:** Low-power regulator reduces power consumption but increases wakeup time.

---

#### `pwc_standby_mode_enter`

```c
void pwc_standby_mode_enter(void);
```

Enter standby mode. Lowest power consumption, most state is lost.

**Behavior:**
- Clears wakeup event flag
- Sets LPSEL bit for standby
- Sets SLEEPDEEP bit
- Executes WFI in infinite loop
- **Does not return** - MCU resets on wakeup

**What is preserved:**
- Battery-powered domain (ERTC, backup registers)
- Wakeup pins state

**What is lost:**
- All RAM contents
- All register values (except battery domain)
- Peripheral states

---

### Wakeup Pin Control

#### `pwc_wakeup_pin_enable`

```c
void pwc_wakeup_pin_enable(uint32_t pin_num, confirm_state new_state);
```

Enable or disable standby wakeup pins.

| Parameter | Description |
|-----------|-------------|
| `pin_num` | `PWC_WAKEUP_PIN_1` (PA0), `PWC_WAKEUP_PIN_2` (PC13), or both |
| `new_state` | TRUE to enable, FALSE to disable |

---

### Flag Operations

#### `pwc_flag_get`

```c
flag_status pwc_flag_get(uint32_t pwc_flag);
```

Get status of a PWC flag.

| Parameter | Description |
|-----------|-------------|
| `pwc_flag` | Flag to check: `PWC_WAKEUP_FLAG`, `PWC_STANDBY_FLAG`, `PWC_PVM_OUTPUT_FLAG` |

| Return | Description |
|--------|-------------|
| `SET` | Flag is set |
| `RESET` | Flag is not set |

---

#### `pwc_flag_clear`

```c
void pwc_flag_clear(uint32_t pwc_flag);
```

Clear PWC flags.

| Parameter | Description |
|-----------|-------------|
| `pwc_flag` | Flags to clear: `PWC_WAKEUP_FLAG`, `PWC_STANDBY_FLAG` (can be ORed) |

**Note:** `PWC_PVM_OUTPUT_FLAG` cannot be cleared (read-only hardware status).

---

### LDO Voltage Control

#### `pwc_ldo_output_voltage_set` (Macro)

```c
pwc_ldo_output_voltage_set(val);
```

Set the LDO output voltage level.

| Parameter | Description |
|-----------|-------------|
| `val` | `PWC_LDO_OUTPUT_1V3`, `PWC_LDO_OUTPUT_1V2`, `PWC_LDO_OUTPUT_1V1` |

**Important Sequence for LDO Change:**
1. Switch system clock to HICK before changing LDO
2. Set LDO voltage
3. Switch system clock back to desired source (PLL)

---

## Code Examples

### Example 1: Sleep Mode with Timer Wakeup

Enter sleep mode and wake up periodically using TMR2 overflow interrupt.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void tmr2_config(void)
{
    crm_clocks_freq_type crm_clocks;
    
    /* Enable TMR2 clock */
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    
    /* Get system clock */
    crm_clocks_freq_get(&crm_clocks);
    
    /* Configure for 1Hz (1 second period) */
    /* (systemclock/(systemclock/10000))/10000 = 1Hz */
    tmr_base_init(TMR2, 9999, (crm_clocks.sclk_freq / 10000 - 1));
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
    
    /* Enable overflow interrupt */
    tmr_interrupt_enable(TMR2, TMR_OVF_INT, TRUE);
    nvic_irq_enable(TMR2_GLOBAL_IRQn, 0, 0);
    
    /* Start timer */
    tmr_counter_enable(TMR2, TRUE);
}

int main(void)
{
    __IO uint32_t systick_index = 0;
    
    system_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    /* Enable PWC clock */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    
    /* Configure TMR2 for wakeup */
    tmr2_config();
    
    at32_led_on(LED2);
    
    while(1)
    {
        at32_led_off(LED2);
        
        /* Save and disable SysTick */
        systick_index = SysTick->CTRL;
        systick_index &= ~((uint32_t)0xFFFFFFFE);
        SysTick->CTRL &= (uint32_t)0xFFFFFFFE;
        
        /* Enter sleep mode - wakeup on TMR2 interrupt */
        pwc_sleep_mode_enter(PWC_SLEEP_ENTER_WFI);
        
        /* Restore SysTick after wakeup */
        SysTick->CTRL |= systick_index;
        
        /* Woken up by TMR2 */
        at32_led_on(LED2);
        delay_ms(500);
    }
}

/* TMR2 interrupt handler */
void TMR2_GLOBAL_IRQHandler(void)
{
    if(tmr_flag_get(TMR2, TMR_OVF_FLAG) == SET)
    {
        tmr_flag_clear(TMR2, TMR_OVF_FLAG);
        /* Wakeup processing done in main loop */
    }
}
```

---

### Example 2: Sleep Mode with USART Wakeup

Enter sleep mode and wake up when USART1 receives data.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void usart1_config(void)
{
    gpio_init_type gpio_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA9 (TX) and PA10 (RX) */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_7);
    
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_7);
    
    /* Enable USART1 interrupt */
    nvic_irq_enable(USART1_IRQn, 0, 0);
    
    /* Configure USART */
    usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(USART1, USART_PARITY_NONE);
    usart_transmitter_enable(USART1, TRUE);
    usart_receiver_enable(USART1, TRUE);
    usart_hardware_flow_control_set(USART1, USART_HARDWARE_FLOW_NONE);
    
    /* Enable receive buffer full interrupt */
    usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
    usart_enable(USART1, TRUE);
}

int main(void)
{
    __IO uint32_t systick_index = 0;
    
    system_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    /* Enable PWC clock */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    
    /* Configure USART1 */
    usart1_config();
    
    printf("Sleep mode demo - send any character to wake\r\n");
    
    while(1)
    {
        at32_led_off(LED2);
        printf("Entering sleep mode...\r\n");
        
        /* Save and disable SysTick */
        systick_index = SysTick->CTRL;
        systick_index &= ~((uint32_t)0xFFFFFFFE);
        SysTick->CTRL &= (uint32_t)0xFFFFFFFE;
        
        /* Enter sleep mode - wakeup on USART receive */
        pwc_sleep_mode_enter(PWC_SLEEP_ENTER_WFI);
        
        /* Restore SysTick */
        SysTick->CTRL |= systick_index;
        
        /* Woken up by USART1 */
        printf("Woke up from sleep!\r\n\r\n");
        at32_led_on(LED2);
        delay_ms(500);
    }
}

void USART1_IRQHandler(void)
{
    if(usart_flag_get(USART1, USART_RDBF_FLAG) != RESET)
    {
        /* Read data to clear flag and wakeup */
        volatile uint8_t data = usart_data_receive(USART1);
        (void)data;
    }
}
```

---

### Example 3: Deep Sleep with ERTC Wakeup Timer

Enter deep sleep mode with low-power regulator, wake up via ERTC wakeup timer.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void ertc_config(void)
{
    /* Enable PWC clock and allow battery domain access */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_battery_powered_domain_access(TRUE);
    
    /* Reset battery domain */
    crm_battery_powered_domain_reset(TRUE);
    crm_battery_powered_domain_reset(FALSE);
    
    /* Enable LEXT and wait for stability */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_LEXT, TRUE);
    while(crm_flag_get(CRM_LEXT_STABLE_FLAG) == RESET);
    
    /* Configure ERTC clock source */
    crm_ertc_clock_select(CRM_ERTC_CLOCK_LEXT);
    crm_ertc_clock_enable(TRUE);
    
    /* Initialize ERTC */
    ertc_reset();
    ertc_wait_update();
    
    /* Configure prescaler for 1Hz: LEXT(32768) / (127+1) / (255+1) = 1Hz */
    ertc_divider_set(127, 255);
    ertc_hour_mode_set(ERTC_HOUR_MODE_24);
}

void ertc_wakeup_timer_config(uint32_t seconds)
{
    exint_init_type exint_init_struct;
    
    /* Configure EXINT line 22 for ERTC wakeup */
    exint_init_struct.line_select   = EXINT_LINE_22;
    exint_init_struct.line_enable   = TRUE;
    exint_init_struct.line_mode     = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);
    
    /* Set wakeup timer clock to 1Hz (CK_B with 16-bit counter) */
    ertc_wakeup_clock_set(ERTC_WAT_CLK_CK_B_16BITS);
    
    /* Set wakeup counter (seconds - 1 because 0 = 1 tick) */
    ertc_wakeup_counter_set(seconds - 1);
    
    /* Enable interrupt */
    nvic_irq_enable(ERTC_WKUP_IRQn, 0, 0);
    ertc_interrupt_enable(ERTC_WAT_INT, TRUE);
    
    /* Enable wakeup timer */
    ertc_wakeup_enable(TRUE);
}

void system_clock_recover(void)
{
    /* Re-enable HEXT */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    while(crm_hext_stable_wait() == ERROR);
    
    /* Re-enable PLL */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while(crm_flag_get(CRM_PLL_STABLE_FLAG) == RESET);
    
    /* Switch back to PLL */
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL);
}

int main(void)
{
    crm_clocks_freq_type crm_clocks = {0};
    __IO uint32_t systick_index = 0;
    
    system_clock_config();
    crm_clocks_freq_get(&crm_clocks);
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    /* Enable PWC clock */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    
    at32_led_on(LED2);
    
    /* Configure ERTC */
    ertc_config();
    
    /* Configure wakeup timer for 5 seconds */
    ertc_wakeup_timer_config(5);
    
    /* Save and disable SysTick */
    systick_index = SysTick->CTRL;
    systick_index &= ~((uint32_t)0xFFFFFFFE);
    SysTick->CTRL &= (uint32_t)0xFFFFFFFE;
    
    /* Switch to HICK before changing LDO */
    crm_sysclk_switch(CRM_SCLK_HICK);
    while(crm_sysclk_switch_status_get() != CRM_SCLK_HICK);
    
    /* Set LDO to low-power mode */
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V1);
    
    /* Configure low-power regulator */
    pwc_voltage_regulate_set(PWC_REGULATOR_LOW_POWER);
    
    /* Enter deep sleep */
    pwc_deep_sleep_mode_enter(PWC_DEEP_SLEEP_ENTER_WFI);
    
    /* === Wakeup occurs here === */
    
    /* Restore SysTick */
    SysTick->CTRL |= systick_index;
    
    at32_led_on(LED3);
    
    /* Wait for clock stability after wakeup */
    delay_us(120);
    
    /* Restore LDO voltage before increasing clock */
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
    
    /* Restore system clock */
    system_clock_recover();
    
    while(1)
    {
        at32_led_toggle(LED2);
        delay_ms(500);
    }
}

void ERTC_WKUP_IRQHandler(void)
{
    if(ertc_flag_get(ERTC_WATF_FLAG) != RESET)
    {
        ertc_flag_clear(ERTC_WATF_FLAG);
        exint_flag_clear(EXINT_LINE_22);
    }
}
```

---

### Example 4: Standby Mode with Wakeup Pin

Enter standby mode and wake up via PA0 (WKUP1) rising edge.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

int main(void)
{
    system_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    /* Enable PWC clock */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    
    /* Check if woke from standby */
    if(pwc_flag_get(PWC_STANDBY_FLAG) != RESET)
    {
        /* Woke up from standby mode */
        pwc_flag_clear(PWC_STANDBY_FLAG);
        at32_led_on(LED2);  /* Indicate standby wakeup */
    }
    
    if(pwc_flag_get(PWC_WAKEUP_FLAG) != RESET)
    {
        /* Wakeup event occurred */
        pwc_flag_clear(PWC_WAKEUP_FLAG);
        at32_led_on(LED3);  /* Indicate wakeup event */
    }
    
    at32_led_on(LED4);  /* Normal operation indicator */
    
    /* Wait to see LED status */
    delay_ms(1000);
    
    /* Enable wakeup pin 1 (PA0) */
    pwc_wakeup_pin_enable(PWC_WAKEUP_PIN_1, TRUE);
    
    /* Enter standby mode - this function does not return!
     * MCU will reset when PA0 rises */
    pwc_standby_mode_enter();
    
    /* Never reached */
    while(1);
}
```

---

### Example 5: Standby Mode with ERTC Alarm

Enter standby mode and wake up after ERTC alarm triggers.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void ertc_config(void)
{
    /* Enable PWC clock and battery domain access */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_battery_powered_domain_access(TRUE);
    
    /* Reset battery domain */
    crm_battery_powered_domain_reset(TRUE);
    crm_battery_powered_domain_reset(FALSE);
    
    /* Enable LEXT */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_LEXT, TRUE);
    while(crm_flag_get(CRM_LEXT_STABLE_FLAG) == RESET);
    
    /* Configure ERTC */
    crm_ertc_clock_select(CRM_ERTC_CLOCK_LEXT);
    crm_ertc_clock_enable(TRUE);
    
    ertc_reset();
    ertc_wait_update();
    ertc_divider_set(127, 255);  /* 1Hz */
    ertc_hour_mode_set(ERTC_HOUR_MODE_24);
    
    /* Set initial time: 06:20:00 */
    ertc_date_set(21, 6, 11, 5);
    ertc_time_set(6, 20, 0, ERTC_AM);
}

void ertc_alarm_config(uint32_t seconds_from_now)
{
    ertc_time_type current_time;
    
    /* Mask date, hour, minute - only match seconds */
    ertc_alarm_mask_set(ERTC_ALA, 
        ERTC_ALARM_MASK_DATE_WEEK | ERTC_ALARM_MASK_HOUR | ERTC_ALARM_MASK_MIN);
    ertc_alarm_week_date_select(ERTC_ALA, ERTC_SLECT_DATE);
    
    /* Get current time */
    ertc_calendar_get(&current_time);
    
    /* Disable alarm, set new value, re-enable */
    ertc_alarm_enable(ERTC_ALA, FALSE);
    ertc_alarm_set(ERTC_ALA, 
        current_time.day, 
        current_time.hour, 
        current_time.min, 
        (current_time.sec + seconds_from_now) % 60, 
        current_time.ampm);
    ertc_alarm_enable(ERTC_ALA, TRUE);
    
    /* Enable alarm interrupt */
    ertc_interrupt_enable(ERTC_ALA_INT, TRUE);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Enable PWC clock */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    
    /* Check wakeup source */
    if(pwc_flag_get(PWC_STANDBY_FLAG) != RESET)
    {
        pwc_flag_clear(PWC_STANDBY_FLAG);
        at32_led_on(LED2);
    }
    if(pwc_flag_get(PWC_WAKEUP_FLAG) != RESET)
    {
        pwc_flag_clear(PWC_WAKEUP_FLAG);
        at32_led_on(LED3);
    }
    
    delay_sec(1);
    
    /* Configure ERTC */
    ertc_config();
    at32_led_on(LED4);
    
    /* Set alarm for 3 seconds from now */
    ertc_alarm_config(3);
    
    delay_sec(1);
    
    /* Enter standby mode - will reset on alarm */
    pwc_standby_mode_enter();
    
    while(1);
}
```

---

### Example 6: Power Voltage Monitor (PVM)

Monitor supply voltage and trigger interrupt when it drops below threshold.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

volatile uint8_t pvm_triggered = 0;

void pvm_exint_config(void)
{
    exint_init_type exint_init_struct;
    
    /* Configure EXINT line 16 for PVM */
    exint_init_struct.line_select   = EXINT_LINE_16;
    exint_init_struct.line_enable   = TRUE;
    exint_init_struct.line_mode     = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_polarity = EXINT_TRIGGER_BOTH_EDGE;  /* Rising and falling */
    exint_init(&exint_init_struct);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    /* Enable PWC clock */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    
    at32_led_on(LED2);
    at32_led_on(LED3);
    at32_led_on(LED4);
    
    /* Set PVM threshold to 2.9V */
    pwc_pvm_level_select(PWC_PVM_VOLTAGE_2V9);
    
    /* Enable PVM */
    pwc_power_voltage_monitor_enable(TRUE);
    
    /* Configure EXINT for PVM */
    pvm_exint_config();
    
    /* Enable PVM interrupt */
    nvic_irq_enable(PVM_IRQn, 0, 0);
    
    while(1)
    {
        /* Check PVM output flag */
        if(pwc_flag_get(PWC_PVM_OUTPUT_FLAG) == SET)
        {
            /* VDD is below threshold! */
            at32_led_off(LED2);
            at32_led_on(LED3);
            
            /* Take protective action:
             * - Save critical data
             * - Disable non-essential peripherals
             * - Enter low-power mode
             */
        }
        else
        {
            /* VDD is above threshold */
            at32_led_on(LED2);
            at32_led_off(LED3);
        }
        
        delay_ms(100);
    }
}

void PVM_IRQHandler(void)
{
    exint_flag_clear(EXINT_LINE_16);
    pvm_triggered = 1;
    
    /* PVM state changed - check PWC_PVM_OUTPUT_FLAG for current state */
    at32_led_toggle(LED4);
}
```

---

### Example 7: Deep Sleep with LDO Optimization

Optimize power consumption in deep sleep by reducing LDO voltage.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/* Store system clock configuration */
crm_clocks_freq_type saved_clocks = {0};

void save_clock_config(void)
{
    crm_clocks_freq_get(&saved_clocks);
}

void system_clock_recover(void)
{
    /* Enable HEXT */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    while(crm_hext_stable_wait() == ERROR);
    
    /* Enable PLL */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while(crm_flag_get(CRM_PLL_STABLE_FLAG) == RESET);
    
    /* Switch to PLL */
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL);
}

void enter_low_power_deepsleep(void)
{
    __IO uint32_t systick_ctrl = 0;
    
    /* Save and disable SysTick */
    systick_ctrl = SysTick->CTRL & 0x01;
    SysTick->CTRL &= ~0x01;
    
    /* Step 1: Switch to HICK before reducing LDO voltage */
    crm_sysclk_switch(CRM_SCLK_HICK);
    while(crm_sysclk_switch_status_get() != CRM_SCLK_HICK);
    
    /* Step 2: Reduce LDO voltage for minimum power */
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V1);
    
    /* Step 3: Enable low-power voltage regulator */
    pwc_voltage_regulate_set(PWC_REGULATOR_LOW_POWER);
    
    /* Step 4: Enter deep sleep */
    pwc_deep_sleep_mode_enter(PWC_DEEP_SLEEP_ENTER_WFI);
    
    /* === Wakeup point === */
    
    /* Step 5: Wait for clock stability (required after wakeup) */
    /* Check if debug is enabled or revision A silicon */
    if(((DEBUGMCU->ctrl & 0x00000007) != 0) || (DEBUGMCU->ser_id_bit.rev_id == 0))
    {
        /* Wait ~120us for LICK stability */
        if((CRM->misc1_bit.hick_to_sclk == TRUE) && (CRM->misc1_bit.hickdiv == TRUE))
        {
            /* HICK is 48MHz */
            delay_us(((120 * 6 * HICK_VALUE) / saved_clocks.sclk_freq) + 1);
        }
        else
        {
            /* HICK is 8MHz */
            delay_us(((120 * HICK_VALUE) / saved_clocks.sclk_freq) + 1);
        }
    }
    
    /* Step 6: Restore LDO voltage BEFORE increasing clock speed */
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
    
    /* Step 7: Restore system clock */
    system_clock_recover();
    
    /* Restore SysTick */
    SysTick->CTRL |= systick_ctrl;
}

int main(void)
{
    exint_init_type exint_init_struct;
    
    system_clock_config();
    save_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure PA0 as EXINT wakeup source */
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_0;
    gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
    gpio_init(GPIOA, &gpio_init_struct);
    
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE0);
    
    exint_init_struct.line_select = EXINT_LINE_0;
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);
    
    nvic_irq_enable(EXINT0_IRQn, 0, 0);
    
    at32_led_on(LED2);
    
    while(1)
    {
        at32_led_off(LED3);
        
        /* Enter optimized deep sleep */
        enter_low_power_deepsleep();
        
        /* Woken up! */
        at32_led_on(LED3);
        delay_ms(1000);
    }
}

void EXINT0_IRQHandler(void)
{
    exint_flag_clear(EXINT_LINE_0);
}
```

---

## Configuration Checklist

### Before Sleep Mode

- [ ] Enable PWC clock: `crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE)`
- [ ] Configure wakeup interrupt source
- [ ] Enable peripheral interrupts for wakeup
- [ ] Save SysTick configuration if using delay functions
- [ ] Consider disabling unused peripheral clocks

### Before Deep Sleep Mode

- [ ] Enable PWC clock
- [ ] Configure EXINT for wakeup source (GPIO, ERTC, etc.)
- [ ] Optionally switch to HICK and reduce LDO voltage
- [ ] Configure regulator mode: `pwc_voltage_regulate_set()`
- [ ] Save SysTick configuration
- [ ] Plan clock recovery sequence for after wakeup

### Before Standby Mode

- [ ] Enable PWC clock
- [ ] Clear wakeup flags: `pwc_flag_clear(PWC_WAKEUP_FLAG | PWC_STANDBY_FLAG)`
- [ ] Enable wakeup pins if needed: `pwc_wakeup_pin_enable()`
- [ ] Configure ERTC alarm if needed (persists through standby)
- [ ] Save critical data (RAM content will be lost!)
- [ ] Note: MCU will reset on wakeup

### LDO Voltage Change Sequence

1. Switch system clock to HICK
2. Set new LDO voltage
3. Wait for stability (if reducing clock)
4. Switch back to desired clock source (if increasing)

**Important:** Never run at high clock speeds with reduced LDO voltage!

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Won't enter sleep | Pending interrupt | Check NVIC for pending interrupts |
| Immediate wakeup | EXINT already triggered | Clear EXINT flags before sleep |
| Deep sleep wakeup fails | Wrong EXINT configuration | Verify EXINT line and polarity |
| Standby doesn't wake | WKUP pin not enabled | Call `pwc_wakeup_pin_enable()` |
| Clock issues after wakeup | Clock not recovered | Run `system_clock_recover()` |
| Hard fault after deep sleep | LDO voltage too low | Increase LDO before high clock |
| ERTC not working | Battery domain locked | Enable `pwc_battery_powered_domain_access()` |
| PVM always triggered | Threshold too high | Lower PVM threshold |
| Wakeup flags not set | Flags already cleared | Check flags before clearing |

---

## Power Consumption Guidelines

| Mode | Typical Current | Notes |
|------|-----------------|-------|
| Run @ 288MHz | ~50-80 mA | Maximum performance |
| Run @ 144MHz | ~30-40 mA | With LDO at 1.1V |
| Sleep | ~10-15 mA | Peripherals active |
| Deep Sleep (reg ON) | ~100-500 μA | Fast wakeup |
| Deep Sleep (reg LP) | ~10-50 μA | Slower wakeup |
| Standby | ~2-5 μA | ERTC running |

---

## Related Peripherals

- **[CRM](CRM_Clock_Reset_Management.md)** - Clock source configuration for low-power modes
- **[ERTC](ERTC_Enhanced_Real_Time_Clock.md)** - Wakeup timer, alarms for standby wakeup
- **[EXINT](EXINT_External_Interrupt.md)** - Wakeup sources for deep sleep
- **[GPIO](GPIO_General_Purpose_IO.md)** - Wakeup pin configuration
- **[DEBUG](DEBUG_MCU_Debug_Support.md)** - Debug behavior in low-power modes

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-01 | Initial release |

