---
title: WDT - Independent Watchdog Timer
category: Safety
complexity: Basic
mcu: AT32F435/437
peripheral: WDT
keywords: [wdt, watchdog, reset, timeout, lick, safety, fault recovery]
---

# WDT - Independent Watchdog Timer

## Overview

The WDT (Watchdog Timer) is an independent watchdog that provides system-level fault recovery. It is clocked by its own dedicated low-speed internal RC oscillator (LICK) and remains active even when the main clock fails. The WDT is designed to detect and recover from software faults, resetting the microcontroller when the software fails to reload the watchdog counter within a configurable timeout period.

### Key Features

| Feature | Specification |
|---------|---------------|
| Clock Source | LICK (~40 kHz) |
| Counter Resolution | 12-bit down counter |
| Reload Value Range | 0 to 4095 |
| Prescaler Dividers | 4, 8, 16, 32, 64, 128, 256 |
| Timeout Range | ~100 µs to ~26.2 s |
| Window Mode | Supported (optional) |
| Clock Independence | Runs from dedicated LICK |
| Operation in Low Power | Active in Standby mode |

### WDT vs WWDT Comparison

| Feature | WDT (Independent) | WWDT (Window) |
|---------|-------------------|---------------|
| Clock Source | LICK (~40 kHz) | PCLK1 / 4096 |
| Counter Bits | 12-bit | 7-bit (6-bit usable) |
| Timeout Range | ~100 µs to ~26.2 s | ~68 µs to ~139 ms |
| Window Mode | Optional | Mandatory |
| Early Wakeup Interrupt | No | Yes |
| Standby Operation | Yes | No |
| Clock Fail Safe | Yes | No |
| Typical Use | System-level watchdog | Application timing |

---

## Architecture

### Block Diagram

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                          WDT                                 │
                    │                                                              │
                    │   ┌──────────────────────────────────────────────────────┐  │
                    │   │                   Clock System                        │  │
                    │   │   LICK (~40 kHz) ─────────────────┐                   │  │
                    │   │                                   │                   │  │
                    │   │   ┌───────────────────────────────▼────────────────┐  │  │
                    │   │   │              Prescaler (DIV)                    │  │  │
                    │   │   │    ÷4, ÷8, ÷16, ÷32, ÷64, ÷128, ÷256          │  │  │
                    │   │   └───────────────────────────────┬────────────────┘  │  │
                    │   └───────────────────────────────────┼───────────────────┘  │
                    │                                       │                      │
                    │                                       ▼                      │
                    │   ┌───────────────────────────────────────────────────────┐  │
                    │   │              12-bit Down Counter (RLD)                 │  │
                    │   │   ┌─────────────────────────────────────────────────┐ │  │
                    │   │   │  Load Value ──▶ Counter ──▶ Decrement on CLK   │ │  │
                    │   │   │                     │                           │ │  │
                    │   │   │                     ▼                           │ │  │
                    │   │   │              Compare = 0x000?                   │ │  │
                    │   │   └─────────────────────┬───────────────────────────┘ │  │
                    │   └─────────────────────────┼─────────────────────────────┘  │
                    │                             │                                │
                    │                             ▼ Counter = 0                    │
                    │   ┌───────────────────────────────────────────────────────┐  │
                    │   │                    Reset Generator                     │  │
                    │   │   Underflow ─────▶ System Reset (WDT Reset Flag)      │  │
                    │   └───────────────────────────────────────────────────────┘  │
                    │                                                              │
                    │   ┌───────────────────────────────────────────────────────┐  │
                    │   │              Window Comparator (Optional)              │  │
                    │   │   WIN Value ◀── Compare ──▶ Reload too early?         │  │
                    │   │                    │                                   │  │
                    │   │                    ▼                                   │  │
                    │   │   If reload when CNT > WIN ──▶ Reset                  │  │
                    │   └───────────────────────────────────────────────────────┘  │
                    └──────────────────────────────────────────────────────────────┘
```

### Timeout Calculation

```
Timeout = (RLD + 1) × Prescaler / LICK_Frequency

Where:
- RLD = Reload value (0 to 4095)
- Prescaler = 4, 8, 16, 32, 64, 128, or 256
- LICK_Frequency ≈ 40,000 Hz
```

### Timeout Examples

| Prescaler | RLD Value | Timeout (ms) | Timeout (s) |
|-----------|-----------|--------------|-------------|
| 4 | 4095 | 409.6 | 0.41 |
| 8 | 4095 | 819.2 | 0.82 |
| 16 | 4095 | 1638.4 | 1.64 |
| 32 | 4095 | 3276.8 | 3.28 |
| 64 | 4095 | 6553.6 | 6.55 |
| 128 | 4095 | 13107.2 | 13.11 |
| 256 | 4095 | 26214.4 | 26.21 |
| 4 | 2999 | 300 | 0.30 |
| 4 | 999 | 100 | 0.10 |

---

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| CMD | 0x00 | Command register |
| DIV | 0x04 | Prescaler divider |
| RLD | 0x08 | Reload value |
| STS | 0x0C | Status register |
| WIN | 0x10 | Window value |

### Command Register Values

| Command | Value | Description |
|---------|-------|-------------|
| `WDT_CMD_UNLOCK` | 0x5555 | Enable write access to DIV, RLD, WIN |
| `WDT_CMD_LOCK` | 0x0000 | Disable write access (lock) |
| `WDT_CMD_RELOAD` | 0xAAAA | Reload counter with RLD value |
| `WDT_CMD_ENABLE` | 0xCCCC | Enable WDT and start counting |

### Status Flags

| Flag | Bit | Description |
|------|-----|-------------|
| `WDT_DIVF_UPDATE_FLAG` | 0 | Prescaler update complete |
| `WDT_RLDF_UPDATE_FLAG` | 1 | Reload value update complete |
| `WDT_WINF_UPDATE_FLAG` | 2 | Window value update complete |

---

## Configuration Types

### Prescaler Division

```c
typedef enum
{
  WDT_CLK_DIV_4   = 0x00,  /* LICK / 4   = 10,000 Hz */
  WDT_CLK_DIV_8   = 0x01,  /* LICK / 8   = 5,000 Hz  */
  WDT_CLK_DIV_16  = 0x02,  /* LICK / 16  = 2,500 Hz  */
  WDT_CLK_DIV_32  = 0x03,  /* LICK / 32  = 1,250 Hz  */
  WDT_CLK_DIV_64  = 0x04,  /* LICK / 64  = 625 Hz    */
  WDT_CLK_DIV_128 = 0x05,  /* LICK / 128 = 312.5 Hz  */
  WDT_CLK_DIV_256 = 0x06   /* LICK / 256 = 156.25 Hz */
} wdt_division_type;
```

---

## API Reference

### Enable and Control

```c
/* Enable WDT - starts counting down */
void wdt_enable(void);

/* Reload counter - must call before timeout */
void wdt_counter_reload(void);
```

### Configuration Functions

```c
/* Enable/disable register write access */
void wdt_register_write_enable(confirm_state new_state);

/* Set prescaler divider */
void wdt_divider_set(wdt_division_type division);

/* Set reload value (0-4095) */
void wdt_reload_value_set(uint16_t reload_value);

/* Set window value for window mode (0-4095) */
void wdt_window_counter_set(uint16_t window_cnt);
```

### Status Functions

```c
/* Get flag status */
flag_status wdt_flag_get(uint16_t wdt_flag);
```

---

## Code Examples

### Example 1: Basic WDT Reset Protection

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/**
 * @brief  Basic WDT configuration with 300ms timeout
 *         LED3 toggles normally, button press causes WDT reset
 */
int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Check if reset was caused by WDT */
    if (crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WDT_RESET_FLAG);
        at32_led_on(LED4);  /* Indicate WDT reset occurred */
    }
    else
    {
        at32_led_off(LED4);
    }
    
    /* Enable write access to WDT registers */
    wdt_register_write_enable(TRUE);
    
    /* Set prescaler: LICK / 4 = 10,000 Hz */
    wdt_divider_set(WDT_CLK_DIV_4);
    
    /* Set reload value for 300ms timeout
     * timeout = (RLD + 1) * prescaler / LICK
     * 0.3 = (RLD + 1) * 4 / 40000
     * RLD = 2999
     */
    wdt_reload_value_set(3000 - 1);
    
    /* Initial counter reload */
    wdt_counter_reload();
    
    /* Enable WDT - cannot be disabled once enabled! */
    wdt_enable();
    
    while (1)
    {
        /* Feed the watchdog - must be called before timeout */
        wdt_counter_reload();
        
        at32_led_toggle(LED3);
        delay_ms(200);  /* Less than 300ms timeout */
        
        /* Button press simulates software hang (no reload) */
        if (at32_button_press() == USER_BUTTON)
        {
            while (1);  /* Infinite loop - WDT will reset */
        }
    }
}
```

---

### Example 2: WDT with Standby Mode Wakeup

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/**
 * @brief  WDT wakes MCU from Standby mode
 *         WDT continues running in Standby
 */
int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Enable PWC clock for standby mode */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    
    /* Check if wakeup was from WDT */
    if (crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WDT_RESET_FLAG);
        at32_led_on(LED4);  /* WDT caused wakeup */
        delay_ms(1000);
    }
    else
    {
        at32_led_off(LED4);
    }
    
    /* Configure WDT with 300ms timeout */
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_4);
    wdt_reload_value_set(3000 - 1);
    wdt_counter_reload();
    wdt_enable();
    
    /* Brief delay to show LED */
    delay_ms(100);
    
    /* Enter Standby mode
     * WDT will continue running and wake up MCU
     */
    pwc_standby_mode_enter();
    
    /* Code never reaches here after standby entry */
    while (1)
    {
    }
}
```

---

### Example 3: Long Timeout WDT (26 seconds)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/**
 * @brief  Maximum WDT timeout configuration (~26 seconds)
 *         Suitable for tasks that take a long time
 */
int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Check for WDT reset */
    if (crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WDT_RESET_FLAG);
        at32_led_on(LED4);
    }
    
    /* Configure for maximum timeout */
    wdt_register_write_enable(TRUE);
    
    /* Maximum prescaler: LICK / 256 */
    wdt_divider_set(WDT_CLK_DIV_256);
    
    /* Maximum reload value: 4095
     * timeout = 4096 * 256 / 40000 = 26.21 seconds
     */
    wdt_reload_value_set(4095);
    
    wdt_counter_reload();
    wdt_enable();
    
    while (1)
    {
        /* Perform long-running task */
        perform_long_task();  /* Must complete within 26 seconds */
        
        /* Feed watchdog after task completes */
        wdt_counter_reload();
        
        at32_led_toggle(LED3);
    }
}

void perform_long_task(void)
{
    /* Simulate task taking up to 20 seconds */
    for (int i = 0; i < 20; i++)
    {
        delay_ms(1000);
        at32_led_toggle(LED2);
    }
}
```

---

### Example 4: Window Mode WDT

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/**
 * @brief  WDT with window mode - reload only valid in window
 *         Reloading too early also causes reset
 */
int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Check for WDT reset */
    if (crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WDT_RESET_FLAG);
        at32_led_on(LED4);
    }
    
    wdt_register_write_enable(TRUE);
    
    /* Set prescaler: LICK / 32 = 1250 Hz */
    wdt_divider_set(WDT_CLK_DIV_32);
    
    /* Set reload value: 1000ms timeout
     * timeout = 1250 * 32 / 40000 = 1.0 second
     */
    wdt_reload_value_set(1250 - 1);
    
    /* Wait for RLD update to complete */
    while (wdt_flag_get(WDT_RLDF_UPDATE_FLAG) != RESET);
    
    /* Set window value: reload valid after 500ms
     * window = 625 * 32 / 40000 = 0.5 second
     * Reload only valid when counter < window (625)
     */
    wdt_window_counter_set(625);
    
    /* Wait for window update to complete */
    while (wdt_flag_get(WDT_WINF_UPDATE_FLAG) != RESET);
    
    wdt_counter_reload();
    wdt_enable();
    
    while (1)
    {
        /* Wait for window to open (500ms after last reload) */
        delay_ms(600);  /* Within 500ms-1000ms window */
        
        /* Reload in valid window */
        wdt_counter_reload();
        
        at32_led_toggle(LED3);
    }
}
```

---

### Example 5: WDT with State Machine Task Monitoring

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

typedef enum
{
    STATE_INIT,
    STATE_IDLE,
    STATE_PROCESSING,
    STATE_ERROR
} app_state_type;

volatile app_state_type current_state = STATE_INIT;
volatile uint32_t task_counter = 0;

/**
 * @brief  Monitor application state machine with WDT
 *         Each state must complete and feed watchdog
 */
int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Check for WDT reset - indicates state machine hung */
    if (crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WDT_RESET_FLAG);
        at32_led_on(LED4);
        current_state = STATE_ERROR;
    }
    
    /* Configure WDT: 500ms timeout */
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_4);
    wdt_reload_value_set(5000 - 1);  /* 500ms */
    wdt_counter_reload();
    wdt_enable();
    
    while (1)
    {
        switch (current_state)
        {
            case STATE_INIT:
                /* Initialization tasks */
                at32_led_on(LED2);
                delay_ms(100);
                at32_led_off(LED2);
                wdt_counter_reload();  /* Feed WDT */
                current_state = STATE_IDLE;
                break;
                
            case STATE_IDLE:
                /* Idle - wait for trigger */
                at32_led_toggle(LED3);
                delay_ms(100);
                wdt_counter_reload();
                
                if (at32_button_press() == USER_BUTTON)
                {
                    current_state = STATE_PROCESSING;
                }
                break;
                
            case STATE_PROCESSING:
                /* Process task - must complete within timeout */
                for (int i = 0; i < 5; i++)
                {
                    at32_led_toggle(LED2);
                    delay_ms(50);
                    wdt_counter_reload();  /* Feed during long task */
                }
                task_counter++;
                current_state = STATE_IDLE;
                break;
                
            case STATE_ERROR:
                /* Error recovery */
                at32_led_on(LED4);
                delay_ms(200);
                wdt_counter_reload();
                current_state = STATE_INIT;
                break;
        }
    }
}
```

---

### Example 6: Multiple Task Watchdog with Software Timer

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define TASK_COUNT 3
#define TASK_TIMEOUT_MS 200

typedef struct
{
    uint32_t last_feed;
    uint8_t active;
} task_watchdog_type;

task_watchdog_type task_wdt[TASK_COUNT];
volatile uint32_t system_tick = 0;

void TMR2_GLOBAL_IRQHandler(void)
{
    if (tmr_flag_get(TMR2, TMR_OVF_FLAG) != RESET)
    {
        tmr_flag_clear(TMR2, TMR_OVF_FLAG);
        system_tick++;
    }
}

void timer_init(void)
{
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    
    tmr_base_init(TMR2, 1000 - 1, system_core_clock / 1000000 - 1);
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    tmr_interrupt_enable(TMR2, TMR_OVF_INT, TRUE);
    
    nvic_irq_enable(TMR2_GLOBAL_IRQn, 0, 0);
    tmr_counter_enable(TMR2, TRUE);
}

void task_wdt_feed(uint8_t task_id)
{
    if (task_id < TASK_COUNT)
    {
        task_wdt[task_id].last_feed = system_tick;
    }
}

uint8_t task_wdt_check_all(void)
{
    for (int i = 0; i < TASK_COUNT; i++)
    {
        if (task_wdt[i].active)
        {
            if ((system_tick - task_wdt[i].last_feed) > TASK_TIMEOUT_MS)
            {
                return 0;  /* Task timeout */
            }
        }
    }
    return 1;  /* All tasks OK */
}

/**
 * @brief  Software task watchdog with hardware WDT backup
 *         Individual tasks must feed their software watchdog
 *         Main loop feeds hardware WDT only if all tasks are healthy
 */
int main(void)
{
    system_clock_config();
    at32_board_init();
    timer_init();
    
    /* Initialize task watchdogs */
    for (int i = 0; i < TASK_COUNT; i++)
    {
        task_wdt[i].last_feed = 0;
        task_wdt[i].active = 1;
    }
    
    /* Configure hardware WDT: 500ms backup timeout */
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_4);
    wdt_reload_value_set(5000 - 1);
    wdt_counter_reload();
    wdt_enable();
    
    while (1)
    {
        /* Task 0: LED toggle */
        at32_led_toggle(LED2);
        task_wdt_feed(0);
        
        /* Task 1: Button check */
        if (at32_button_press() == USER_BUTTON)
        {
            /* Simulate task 2 hanging */
            task_wdt[2].active = 0;
        }
        task_wdt_feed(1);
        
        /* Task 2: Processing */
        at32_led_toggle(LED3);
        task_wdt_feed(2);
        
        /* Check all software watchdogs before feeding hardware WDT */
        if (task_wdt_check_all())
        {
            wdt_counter_reload();  /* All OK - feed hardware WDT */
        }
        /* If any task failed, hardware WDT will reset system */
        
        delay_ms(50);
    }
}
```

---

## Configuration Checklist

### Basic Setup
- [ ] No clock enable required (uses dedicated LICK)
- [ ] Call `wdt_register_write_enable(TRUE)` before configuration
- [ ] Set prescaler with `wdt_divider_set()`
- [ ] Set reload value with `wdt_reload_value_set()`
- [ ] Call `wdt_counter_reload()` before `wdt_enable()`
- [ ] Call `wdt_enable()` to start watchdog

### Important Notes
- [ ] WDT cannot be disabled once enabled (until reset)
- [ ] Always reload counter before timeout expires
- [ ] LICK accuracy is ±10%, plan for worst case
- [ ] WDT runs in Standby mode and will wake/reset MCU
- [ ] Check `CRM_WDT_RESET_FLAG` on startup to detect WDT resets

### Window Mode (Optional)
- [ ] Configure window value with `wdt_window_counter_set()`
- [ ] Wait for `WDT_WINF_UPDATE_FLAG` before enabling
- [ ] Reload only when counter is below window value

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Unexpected resets | Timeout too short | Increase reload value or prescaler |
| No reset on hang | WDT not enabled | Verify `wdt_enable()` called |
| Config not applied | Write protection | Call `wdt_register_write_enable(TRUE)` |
| Timeout varies | LICK accuracy ±10% | Use conservative timeout values |
| Window reset | Reload too early | Wait for window to open |
| Can't disable WDT | By design | WDT can only be disabled by system reset |
| Standby wakeup | WDT active in standby | Use longer timeout or different power mode |

---

## Performance Specifications

| Parameter | Min | Typical | Max | Unit |
|-----------|-----|---------|-----|------|
| LICK Frequency | 36 | 40 | 44 | kHz |
| Minimum Timeout | ~90 | 100 | ~110 | µs |
| Maximum Timeout | ~23.8 | 26.2 | ~29 | s |
| Counter Reload Time | - | 5 | - | LICK cycles |
| Write Access Latency | - | 5 | - | LICK cycles |

---

## Related Peripherals

| Peripheral | Relationship |
|------------|--------------|
| WWDT | Alternative window watchdog (shorter timeouts) |
| CRM | Reset flag detection |
| PWC | Low-power modes where WDT continues |
| DEBUG | Can freeze WDT during debug |

---

## References

- AT32F435/437 Reference Manual - WDT Chapter
- Application Note AN0045 - Watchdog Timer Usage
- AT32F435/437 Datasheet - LICK Specifications

