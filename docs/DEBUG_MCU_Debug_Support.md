---
title: DEBUG (MCU Debug Support)
mcu: AT32F435/437
peripheral: DEBUG
version: 2.0.9
---

# DEBUG MCU Debug Support

## Overview

The AT32F435/437 DEBUG module provides hardware support for debugging and development. It allows peripherals to be frozen (paused) when the CPU is halted by the debugger, preventing unexpected behavior during step-by-step debugging. Additionally, it supports debug operation during low-power modes and provides device identification.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          DEBUG Module Architecture                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                         Debug Interface                                 │ │
│  │                                                                         │ │
│  │   ┌──────────────┐    ┌──────────────────────────────────────────┐    │ │
│  │   │   JTAG/SWD   │───▶│           Core Debug Unit                │    │ │
│  │   │   Interface  │    │    (Cortex-M4 CoreSight DAP)             │    │ │
│  │   └──────────────┘    └──────────────────────────────────────────┘    │ │
│  │                                        │                               │ │
│  │                                        ▼                               │ │
│  │                        ┌───────────────────────────┐                  │ │
│  │                        │      HALT Signal          │                  │ │
│  │                        │   (CPU Stopped by Debug)  │                  │ │
│  │                        └─────────────┬─────────────┘                  │ │
│  └──────────────────────────────────────┼─────────────────────────────────┘ │
│                                         │                                    │
│                                         ▼                                    │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                        DEBUG Control Registers                          │ │
│  │                                                                         │ │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐        │ │
│  │  │    IDCODE       │  │     CTRL        │  │    SER_ID       │        │ │
│  │  │  Device ID      │  │  Low Power Cfg  │  │   Series ID     │        │ │
│  │  │  (PID)          │  │  Sleep/Deep/    │  │   Revision      │        │ │
│  │  │                 │  │  Standby Debug  │  │                 │        │ │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘        │ │
│  │                                                                         │ │
│  │  ┌─────────────────────────────┐  ┌─────────────────────────────┐     │ │
│  │  │       APB1_FRZ              │  │       APB2_FRZ              │     │ │
│  │  │  APB1 Peripheral Freeze     │  │  APB2 Peripheral Freeze     │     │ │
│  │  │                             │  │                             │     │ │
│  │  │  • TMR2-7, TMR12-14        │  │  • TMR1, TMR8, TMR20        │     │ │
│  │  │  • ERTC, WDT, WWDT         │  │  • TMR9, TMR10, TMR11       │     │ │
│  │  │  • I2C1-3 SMBus Timeout    │  │                             │     │ │
│  │  │  • CAN1, CAN2              │  │                             │     │ │
│  │  └─────────────────────────────┘  └─────────────────────────────┘     │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                         │                                    │
│                                         ▼                                    │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                    Peripheral Freeze Control                            │ │
│  │                                                                         │ │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐         │ │
│  │  │  TMRx   │ │  ERTC   │ │  WDT    │ │  I2C    │ │  CAN    │         │ │
│  │  │  Pause  │ │  Pause  │ │  Pause  │ │  Pause  │ │  Pause  │         │ │
│  │  └─────────┘ └─────────┘ └─────────┘ └─────────┘ └─────────┘         │ │
│  │                                                                         │ │
│  │         When CPU halted + FRZ enabled → Peripheral counter stops        │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Key Features

| Feature | Specification |
|---------|---------------|
| Device Identification | 32-bit Product ID (PID) |
| Series Identification | Series ID and Revision ID |
| Low Power Debug | Sleep, Deep Sleep, Standby modes |
| APB1 Peripherals | 18 freeze options (timers, watchdogs, I2C, CAN) |
| APB2 Peripherals | 6 freeze options (advanced timers) |
| Debug Interface | JTAG and SWD supported |

---

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| `PID` | 0x00 | Product/Device ID register |
| `CTRL` | 0x04 | Debug control register |
| `APB1_FRZ` | 0x08 | APB1 peripheral freeze register |
| `APB2_FRZ` | 0x0C | APB2 peripheral freeze register |
| `SER_ID` | 0x20 | Series and revision ID register |

### CTRL Register Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0 | SLEEP_DEBUG | Keep debug active in Sleep mode |
| 1 | DEEPSLEEP_DEBUG | Keep debug active in Deep Sleep mode |
| 2 | STANDBY_DEBUG | Keep debug active in Standby mode |

### APB1_FRZ Register Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0 | TMR2_PAUSE | Freeze Timer 2 |
| 1 | TMR3_PAUSE | Freeze Timer 3 |
| 2 | TMR4_PAUSE | Freeze Timer 4 |
| 3 | TMR5_PAUSE | Freeze Timer 5 |
| 4 | TMR6_PAUSE | Freeze Timer 6 |
| 5 | TMR7_PAUSE | Freeze Timer 7 |
| 6 | TMR12_PAUSE | Freeze Timer 12 |
| 7 | TMR13_PAUSE | Freeze Timer 13 |
| 8 | TMR14_PAUSE | Freeze Timer 14 |
| 10 | ERTC_PAUSE | Freeze Enhanced RTC |
| 11 | WWDT_PAUSE | Freeze Window Watchdog |
| 12 | WDT_PAUSE | Freeze Watchdog Timer |
| 15 | ERTC_512_PAUSE | Freeze ERTC 512Hz output |
| 24 | I2C1_SMBUS_TIMEOUT | Freeze I2C1 SMBus timeout |
| 25 | CAN1_PAUSE | Freeze CAN1 |
| 26 | CAN2_PAUSE | Freeze CAN2 |
| 27 | I2C2_SMBUS_TIMEOUT | Freeze I2C2 SMBus timeout |
| 28 | I2C3_SMBUS_TIMEOUT | Freeze I2C3 SMBus timeout |

### APB2_FRZ Register Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0 | TMR1_PAUSE | Freeze Timer 1 |
| 1 | TMR8_PAUSE | Freeze Timer 8 |
| 6 | TMR20_PAUSE | Freeze Timer 20 |
| 16 | TMR9_PAUSE | Freeze Timer 9 |
| 17 | TMR10_PAUSE | Freeze Timer 10 |
| 18 | TMR11_PAUSE | Freeze Timer 11 |

---

## Freeze Macros Reference

### Low Power Mode Macros

| Macro | Value | Description |
|-------|-------|-------------|
| `DEBUG_SLEEP` | 0x00000001 | Debug in Sleep mode |
| `DEBUG_DEEPSLEEP` | 0x00000002 | Debug in Deep Sleep mode |
| `DEBUG_STANDBY` | 0x00000004 | Debug in Standby mode |

### APB1 Peripheral Macros

| Macro | Value | Description |
|-------|-------|-------------|
| `DEBUG_TMR2_PAUSE` | 0x00000001 | Timer 2 freeze |
| `DEBUG_TMR3_PAUSE` | 0x00000002 | Timer 3 freeze |
| `DEBUG_TMR4_PAUSE` | 0x00000004 | Timer 4 freeze |
| `DEBUG_TMR5_PAUSE` | 0x00000008 | Timer 5 freeze |
| `DEBUG_TMR6_PAUSE` | 0x00000010 | Timer 6 freeze |
| `DEBUG_TMR7_PAUSE` | 0x00000020 | Timer 7 freeze |
| `DEBUG_TMR12_PAUSE` | 0x00000040 | Timer 12 freeze |
| `DEBUG_TMR13_PAUSE` | 0x00000080 | Timer 13 freeze |
| `DEBUG_TMR14_PAUSE` | 0x00000100 | Timer 14 freeze |
| `DEBUG_ERTC_PAUSE` | 0x00000400 | Enhanced RTC freeze |
| `DEBUG_WWDT_PAUSE` | 0x00000800 | Window Watchdog freeze |
| `DEBUG_WDT_PAUSE` | 0x00001000 | Watchdog Timer freeze |
| `DEBUG_ERTC_512_PAUSE` | 0x00008000 | ERTC 512Hz freeze |
| `DEBUG_I2C1_SMBUS_TIMEOUT` | 0x01000000 | I2C1 SMBus timeout freeze |
| `DEBUG_CAN1_PAUSE` | 0x02000000 | CAN1 freeze |
| `DEBUG_CAN2_PAUSE` | 0x04000000 | CAN2 freeze |
| `DEBUG_I2C2_SMBUS_TIMEOUT` | 0x08000000 | I2C2 SMBus timeout freeze |
| `DEBUG_I2C3_SMBUS_TIMEOUT` | 0x10000000 | I2C3 SMBus timeout freeze |

### APB2 Peripheral Macros

| Macro | Value | Description |
|-------|-------|-------------|
| `DEBUG_TMR1_PAUSE` | 0x00000001 | Timer 1 freeze |
| `DEBUG_TMR8_PAUSE` | 0x00000002 | Timer 8 freeze |
| `DEBUG_TMR20_PAUSE` | 0x00000040 | Timer 20 freeze |
| `DEBUG_TMR9_PAUSE` | 0x00010000 | Timer 9 freeze |
| `DEBUG_TMR10_PAUSE` | 0x00020000 | Timer 10 freeze |
| `DEBUG_TMR11_PAUSE` | 0x00040000 | Timer 11 freeze |

---

## API Reference

### Device Identification

| Function | Description |
|----------|-------------|
| `debug_device_id_get()` | Get 32-bit device/product ID |

### Low Power Debug Configuration

| Function | Description |
|----------|-------------|
| `debug_low_power_mode_set(mode, state)` | Enable/disable debug in low power modes |

**Parameters:**
- `mode`: `DEBUG_SLEEP`, `DEBUG_DEEPSLEEP`, or `DEBUG_STANDBY`
- `state`: `TRUE` to enable, `FALSE` to disable

### Peripheral Freeze Configuration

| Function | Description |
|----------|-------------|
| `debug_apb1_periph_mode_set(periph, state)` | Freeze/unfreeze APB1 peripherals during debug |
| `debug_apb2_periph_mode_set(periph, state)` | Freeze/unfreeze APB2 peripherals during debug |

**Parameters:**
- `periph`: One or more `DEBUG_xxx_PAUSE` macros (can be OR'd together)
- `state`: `TRUE` to freeze, `FALSE` to run normally

---

## Complete Examples

### Example 1: Timer Freeze During Debug

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * Timer Debug Freeze Example
 * 
 * Demonstrates how to freeze TMR1 during debug breakpoints.
 * When the CPU is halted by the debugger, TMR1 counter will pause,
 * allowing step-by-step debugging without missing timer events.
 * 
 * Hardware: AT-START-F435 board
 * LED3: Toggles on TMR1 overflow interrupt
 ******************************************************************************/

crm_clocks_freq_type crm_clocks_freq_struct = {0};
uint16_t counter = 0;

/**
 * @brief TMR1 overflow interrupt handler
 */
void TMR1_OVF_TMR10_IRQHandler(void)
{
  if(tmr_interrupt_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
  {
    at32_led_toggle(LED3);
    tmr_flag_clear(TMR1, TMR_OVF_FLAG);
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();

  /* Get system clock frequency */
  crm_clocks_freq_get(&crm_clocks_freq_struct);

  /* Turn on LEDs to indicate running */
  at32_led_on(LED2);
  at32_led_on(LED3);
  at32_led_on(LED4);

  /* Enable TMR1 clock */
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  /*=========================================================================
   * KEY: Enable TMR1 debug freeze mode
   * 
   * When you set a breakpoint and the CPU halts:
   * - WITHOUT this: TMR1 continues counting, interrupts may fire unexpectedly
   * - WITH this: TMR1 counter pauses, resumes when stepping/continuing
   *=========================================================================*/
  debug_apb2_periph_mode_set(DEBUG_TMR1_PAUSE, TRUE);

  /* Configure TMR1: count up to 10000 */
  tmr_base_init(TMR1, 10000, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  /* Enable overflow interrupt */
  tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);

  /* Configure NVIC */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 1, 0);

  /* Start TMR1 */
  tmr_counter_enable(TMR1, TRUE);

  while(1)
  {
    /*
     * In debug mode with breakpoint here, TMR1 counter will pause.
     * You can read the counter value multiple times - it won't change
     * while halted at the breakpoint.
     */
    counter = tmr_counter_value_get(TMR1);
    counter = tmr_counter_value_get(TMR1);
    counter = tmr_counter_value_get(TMR1);
    counter = tmr_counter_value_get(TMR1);
    counter = tmr_counter_value_get(TMR1);
  }
}
```

---

### Example 2: Watchdog Freeze During Debug

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * Watchdog Debug Freeze Example
 * 
 * When debugging code with watchdog enabled, the WDT would reset the MCU
 * during breakpoints if not frozen. This example shows how to prevent
 * watchdog resets while debugging.
 * 
 * CRITICAL: Always freeze WDT during debug sessions to avoid resets!
 ******************************************************************************/

int main(void)
{
  system_clock_config();
  at32_board_init();

  at32_led_on(LED2);

  /*=========================================================================
   * IMPORTANT: Enable watchdog freeze BEFORE enabling the watchdog
   * 
   * This prevents WDT resets when:
   * - Hitting a breakpoint
   * - Single-stepping through code
   * - Pausing execution in the debugger
   *=========================================================================*/
  debug_apb1_periph_mode_set(DEBUG_WDT_PAUSE, TRUE);

  /* Also freeze Window Watchdog if used */
  debug_apb1_periph_mode_set(DEBUG_WWDT_PAUSE, TRUE);

  /* Enable WDT clock */
  crm_periph_clock_enable(CRM_WDT_PERIPH_CLOCK, TRUE);

  /* Configure and enable WDT (example configuration) */
  wdt_register_write_enable(TRUE);
  wdt_divider_set(WDT_CLK_DIV_64);
  wdt_reload_value_set(0xFFF);  /* ~4 seconds timeout */
  wdt_register_write_enable(FALSE);
  wdt_enable();

  while(1)
  {
    /* Main application loop */
    at32_led_toggle(LED3);
    delay_ms(500);

    /* Feed the watchdog */
    wdt_counter_reload();

    /*
     * Set a breakpoint here and pause for >4 seconds.
     * Without DEBUG_WDT_PAUSE, the MCU would reset.
     * With DEBUG_WDT_PAUSE, the watchdog is frozen and
     * you can debug indefinitely.
     */
  }
}
```

---

### Example 3: Multiple Peripheral Freeze

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * Multiple Peripheral Freeze Example
 * 
 * Demonstrates freezing multiple peripherals simultaneously during debug.
 * Useful for complex applications using multiple timers and communication.
 ******************************************************************************/

void debug_freeze_init(void)
{
  /*=========================================================================
   * APB1 Peripherals - Basic Timers, Watchdogs, I2C, CAN
   * 
   * You can OR multiple macros together for a single call
   *=========================================================================*/
  debug_apb1_periph_mode_set(
    DEBUG_TMR2_PAUSE |      /* Freeze TMR2 */
    DEBUG_TMR3_PAUSE |      /* Freeze TMR3 */
    DEBUG_TMR4_PAUSE |      /* Freeze TMR4 */
    DEBUG_TMR5_PAUSE |      /* Freeze TMR5 */
    DEBUG_WDT_PAUSE  |      /* Freeze Watchdog - CRITICAL */
    DEBUG_WWDT_PAUSE |      /* Freeze Window Watchdog */
    DEBUG_I2C1_SMBUS_TIMEOUT | /* Freeze I2C1 SMBus timeout */
    DEBUG_CAN1_PAUSE,       /* Freeze CAN1 */
    TRUE
  );

  /*=========================================================================
   * APB2 Peripherals - Advanced Timers
   *=========================================================================*/
  debug_apb2_periph_mode_set(
    DEBUG_TMR1_PAUSE |      /* Freeze TMR1 (motor control timer) */
    DEBUG_TMR8_PAUSE |      /* Freeze TMR8 */
    DEBUG_TMR9_PAUSE |      /* Freeze TMR9 */
    DEBUG_TMR10_PAUSE |     /* Freeze TMR10 */
    DEBUG_TMR11_PAUSE,      /* Freeze TMR11 */
    TRUE
  );
}

int main(void)
{
  system_clock_config();
  at32_board_init();

  /* Initialize all debug freeze settings first */
  debug_freeze_init();

  /* Now enable peripheral clocks and configure */
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_WDT_PERIPH_CLOCK, TRUE);

  /* ... rest of peripheral configuration ... */

  while(1)
  {
    /* Application code */
    at32_led_toggle(LED2);
    delay_ms(500);
  }
}
```

---

### Example 4: Low Power Mode Debug

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * Low Power Debug Example
 * 
 * By default, debug connection is lost when entering low power modes.
 * This example shows how to maintain debug capability in Sleep/DeepSleep.
 * 
 * WARNING: This increases power consumption in low power modes!
 * Only use during development, disable for production.
 ******************************************************************************/

void debug_low_power_init(void)
{
  /*=========================================================================
   * Enable debug in Sleep mode
   * - Debug interface remains active
   * - Can wake MCU via debugger
   * - Higher power consumption than normal Sleep
   *=========================================================================*/
  debug_low_power_mode_set(DEBUG_SLEEP, TRUE);

  /*=========================================================================
   * Enable debug in Deep Sleep mode
   * - Debug clocks kept running
   * - Significant power penalty
   * - Useful for debugging sleep/wake cycles
   *=========================================================================*/
  debug_low_power_mode_set(DEBUG_DEEPSLEEP, TRUE);

  /*=========================================================================
   * Enable debug in Standby mode (if needed)
   * - Highest power penalty
   * - Usually not recommended
   *=========================================================================*/
  // debug_low_power_mode_set(DEBUG_STANDBY, TRUE);  /* Uncomment if needed */
}

int main(void)
{
  system_clock_config();
  at32_board_init();

  /* Enable low power debug BEFORE entering low power modes */
  debug_low_power_init();

  at32_led_on(LED2);

  while(1)
  {
    at32_led_off(LED3);
    
    /*
     * Enter Sleep mode (WFI instruction)
     * With DEBUG_SLEEP enabled, you can:
     * - Halt the MCU while in Sleep
     * - Inspect registers
     * - Single-step through wake-up code
     */
    __WFI();
    
    /* Woke up from Sleep */
    at32_led_on(LED3);
    delay_ms(500);
  }
}
```

---

### Example 5: Device Identification

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include <stdio.h>

/*******************************************************************************
 * Device Identification Example
 * 
 * Demonstrates reading device ID and series information.
 * Useful for runtime device detection and firmware compatibility checks.
 ******************************************************************************/

void print_device_info(void)
{
  uint32_t device_id;
  uint32_t series_id;
  
  /* Get device ID (32-bit product identifier) */
  device_id = debug_device_id_get();
  
  /* Get series ID from SER_ID register */
  series_id = DEBUGMCU->ser_id;
  
  printf("\n=== AT32F435/437 Device Information ===\n");
  printf("Device ID (PID): 0x%08X\n", (unsigned int)device_id);
  printf("Series ID: 0x%02X\n", (unsigned int)((series_id >> 8) & 0xFF));
  printf("Revision ID: %d\n", (unsigned int)(series_id & 0x07));
  
  /*
   * Common AT32F435/437 Device IDs:
   * - AT32F435: Check specific ID in datasheet
   * - AT32F437: Check specific ID in datasheet
   */
  
  /* Example: Check if this is an AT32F435 series */
  if((device_id & 0xFFF) == 0x0XX) /* Replace with actual ID */
  {
    printf("Detected: AT32F435 series\n");
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  uart_print_init(115200);

  /* Print device information */
  print_device_info();

  /* Flash LED to indicate running */
  while(1)
  {
    at32_led_toggle(LED2);
    delay_ms(500);
  }
}
```

---

### Example 6: PWM Debug Without Glitches

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * PWM Debug Freeze Example
 * 
 * When debugging motor control or LED PWM applications, timer freeze
 * prevents PWM outputs from being held in unpredictable states.
 * 
 * Without freeze: PWM output might be stuck HIGH or LOW at breakpoint
 * With freeze: PWM output state is preserved, resumes cleanly
 ******************************************************************************/

void pwm_debug_init(void)
{
  /* Enable TMR1 clock */
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
  
  /*=========================================================================
   * CRITICAL for PWM applications:
   * Freeze TMR1 during debug to prevent output glitches
   *=========================================================================*/
  debug_apb2_periph_mode_set(DEBUG_TMR1_PAUSE, TRUE);

  /* Configure TMR1 for PWM output */
  tmr_base_init(TMR1, 1000 - 1, 0);  /* 1kHz PWM @ 1MHz timer clock */
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  /* Configure output compare for PWM mode */
  tmr_output_default_para_init(&tmr_output_struct);
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  
  /* Set 50% duty cycle */
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 500);

  /* Enable main output (required for advanced timers) */
  tmr_output_enable(TMR1, TRUE);

  /* Start timer */
  tmr_counter_enable(TMR1, TRUE);
}

int main(void)
{
  system_clock_config();
  at32_board_init();

  pwm_debug_init();

  while(1)
  {
    /*
     * When you hit a breakpoint here:
     * - TMR1 counter freezes
     * - PWM output holds its current state
     * - When you continue, PWM resumes smoothly
     * - No glitches or missed cycles
     */
    delay_ms(100);
  }
}
```

---

## Use Cases

### Recommended Freeze Settings by Application Type

| Application | Recommended Freezes |
|-------------|---------------------|
| Motor Control | TMR1, TMR8, TMR20, WDT |
| PWM Lighting | TMR1-5, WDT |
| Communication | CAN1, CAN2, I2C1-3, WDT |
| Real-Time Clock | ERTC, ERTC_512 |
| General Purpose | TMR2-5, WDT, WWDT |
| Safety Critical | WDT, WWDT (always) |

### Development vs Production

```c
/* Development build - enable all debug features */
#ifdef DEBUG_BUILD
void debug_init(void)
{
  /* Enable all timer freezes */
  debug_apb1_periph_mode_set(
    DEBUG_TMR2_PAUSE | DEBUG_TMR3_PAUSE | DEBUG_TMR4_PAUSE |
    DEBUG_TMR5_PAUSE | DEBUG_TMR6_PAUSE | DEBUG_TMR7_PAUSE |
    DEBUG_WDT_PAUSE | DEBUG_WWDT_PAUSE,
    TRUE
  );
  
  debug_apb2_periph_mode_set(
    DEBUG_TMR1_PAUSE | DEBUG_TMR8_PAUSE | DEBUG_TMR20_PAUSE |
    DEBUG_TMR9_PAUSE | DEBUG_TMR10_PAUSE | DEBUG_TMR11_PAUSE,
    TRUE
  );
  
  /* Enable low power debug */
  debug_low_power_mode_set(DEBUG_SLEEP | DEBUG_DEEPSLEEP, TRUE);
}
#else
/* Production build - no debug overhead */
#define debug_init()  /* Empty */
#endif
```

---

## Implementation Checklist

### Basic Debug Setup
- [ ] Call freeze functions early in `main()` before peripheral init
- [ ] Always freeze WDT when using watchdog
- [ ] Freeze timers used for time-critical operations

### For Motor Control / PWM
- [ ] Freeze TMR1/TMR8/TMR20 (advanced timers)
- [ ] Test PWM output behavior at breakpoints
- [ ] Verify motor doesn't move unexpectedly when halted

### For Communication Protocols
- [ ] Freeze I2C SMBus timeout if using I2C
- [ ] Freeze CAN if using CAN bus
- [ ] Consider communication timeout handling

### For Low Power Applications
- [ ] Enable Sleep/DeepSleep debug during development
- [ ] Disable low power debug for production builds
- [ ] Document power impact of debug settings

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| MCU resets at breakpoints | Watchdog not frozen | Add `debug_apb1_periph_mode_set(DEBUG_WDT_PAUSE, TRUE)` |
| Timer values change during halt | Timer not frozen | Enable appropriate `DEBUG_TMRx_PAUSE` |
| Lost debug connection in Sleep | Low power debug disabled | Enable `debug_low_power_mode_set(DEBUG_SLEEP, TRUE)` |
| PWM glitches at breakpoints | Timer output not frozen | Freeze the timer generating PWM |
| I2C timeout errors in debug | SMBus timeout not frozen | Enable `DEBUG_I2Cx_SMBUS_TIMEOUT` |
| CAN frame loss during debug | CAN not frozen | Enable `DEBUG_CANx_PAUSE` |
| ERTC time jumps after debug | ERTC not frozen | Enable `DEBUG_ERTC_PAUSE` |

---

## Debug Interface Pins

| Interface | Pins | Description |
|-----------|------|-------------|
| JTAG | TMS, TCK, TDI, TDO, nTRST | Full JTAG (5 pins) |
| SWD | SWDIO, SWCLK | Serial Wire Debug (2 pins) |
| SWO | SWO | Serial Wire Output (trace) |

### Pin Configuration Notes

```c
/*
 * Debug pins are configured automatically by hardware.
 * If you need to use debug pins as GPIO (release mode):
 */

/* Release JTAG pins for GPIO use (keep SWD) */
gpio_pin_remap_config(SWJTAG_MUX_010, TRUE);  /* SWD only */

/* Release all debug pins for GPIO (no debug!) */
gpio_pin_remap_config(SWJTAG_MUX_100, TRUE);  /* Full release */
```

---

## See Also

- [CRM Documentation](./CRM_Clock_Reset_Management.md) - Clock configuration for debug
- [TMR Documentation](./TMR_Timer.md) - Timer peripheral details
- [WDT Documentation](./WDT_Watchdog_Timer.md) - Watchdog configuration
- [PWC Documentation](./PWC_Power_Control.md) - Low power modes
- AT32F435_437 Reference Manual - Debug Support Chapter

