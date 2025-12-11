---
title: WWDT - Window Watchdog Timer
category: Safety
complexity: Intermediate
mcu: AT32F435/437
peripheral: WWDT
keywords: [wwdt, watchdog, window, reset, timeout, pclk1, safety, early wakeup]
---

# WWDT - Window Watchdog Timer

## Overview

The WWDT (Window Watchdog Timer) is a programmable countdown timer that generates a system reset when the counter reaches a preset value (0x40) or when the counter is reloaded outside a valid time window. Unlike the independent WDT, the WWDT is clocked from the APB1 clock (PCLK1), providing precise timing but not operating during clock failures. The WWDT features an Early Wakeup Interrupt (EWI) that can be used for recovery before reset.

### Key Features

| Feature | Specification |
|---------|---------------|
| Clock Source | PCLK1 / 4096 |
| Counter Resolution | 7-bit (6-bit usable: 0x40-0x7F) |
| Counter Range | 0x40 to 0x7F (64 to 127) |
| Prescaler Dividers | 1, 2, 4, 8 (effective: 4096, 8192, 16384, 32768) |
| Timeout Range | ~68 µs to ~139 ms (at 144 MHz PCLK1) |
| Window Mode | Mandatory - must reload in valid window |
| Early Wakeup Interrupt | Supported |
| Standby Operation | Not active |

### Timing Characteristics (PCLK1 = 144 MHz)

| Prescaler | Counter Ticks | Min Timeout | Max Timeout |
|-----------|---------------|-------------|-------------|
| DIV_4096 | ~35.2 kHz | ~1.8 ms | ~3.6 ms |
| DIV_8192 | ~17.6 kHz | ~3.6 ms | ~7.3 ms |
| DIV_16384 | ~8.8 kHz | ~7.3 ms | ~14.5 ms |
| DIV_32768 | ~4.4 kHz | ~14.5 ms | ~29.1 ms |

---

## Architecture

### Block Diagram

```
                    ┌──────────────────────────────────────────────────────────────────┐
                    │                            WWDT                                   │
                    │                                                                   │
                    │   ┌───────────────────────────────────────────────────────────┐  │
                    │   │                     Clock System                           │  │
                    │   │                                                            │  │
                    │   │   PCLK1 ──▶ ÷4096 ──▶ Prescaler ──▶ Counter Clock         │  │
                    │   │              base      (÷1,2,4,8)                          │  │
                    │   └───────────────────────────────────────┬───────────────────┘  │
                    │                                           │                      │
                    │                                           ▼                      │
                    │   ┌───────────────────────────────────────────────────────────┐  │
                    │   │              7-bit Down Counter (T[6:0])                   │  │
                    │   │   ┌─────────────────────────────────────────────────────┐ │  │
                    │   │   │  0x7F ──▶ Countdown ──▶ 0x40 ──▶ Reset if underflow │ │  │
                    │   │   │    │                       │                         │ │  │
                    │   │   │    │   ┌───────────────────┘                         │ │  │
                    │   │   │    │   │                                             │ │  │
                    │   │   │    ▼   ▼                                             │ │  │
                    │   │   │  T[6] bit = 0 ──▶ Counter reaches 0x3F ──▶ Reset    │ │  │
                    │   │   └─────────────────────────────────────────────────────┘ │  │
                    │   └─────────────────────────────────────────────────────────────┘  │
                    │                             │                                      │
                    │              ┌──────────────┼──────────────┐                      │
                    │              │              │              │                      │
                    │              ▼              ▼              ▼                      │
                    │   ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
                    │   │   Window     │  │    Early     │  │    Reset             │   │
                    │   │  Comparator  │  │   Wakeup     │  │   Generator          │   │
                    │   │              │  │  Interrupt   │  │                      │   │
                    │   │ If reload    │  │  at T[6:0]   │  │  T[6:0]=0x3F ──▶    │   │
                    │   │ when T>W     │  │  = 0x40      │  │  System Reset        │   │
                    │   │ ──▶ Reset   │  │              │  │                      │   │
                    │   └──────────────┘  └──────────────┘  └──────────────────────┘   │
                    │                                                                   │
                    └───────────────────────────────────────────────────────────────────┘

                    Valid Reload Window:
                    
                    Counter  0x7F ────────────────────────────────────────────▶ 0x40 ──▶ Reset
                             │                                                    │
                             │    Invalid (early reload causes reset)             │
                             │    ◀─────────────────────────────▶                │
                             │              T > W                                 │
                             │                                                    │
                             │              Valid Reload Window                   │
                             │              ◀────────────────────▶               │
                             │                   W ≥ T > 0x3F                     │
                             │                                                    │
                             └────────────────────────────────────────────────────┘
```

### Timeout and Window Calculation

```
Timeout = (T[5:0] + 1) × Prescaler × 4096 / PCLK1

Where:
- T[5:0] = Lower 6 bits of counter (0 to 63)
- Prescaler = 1, 2, 4, or 8
- PCLK1 = APB1 clock frequency

Window = (T[6:0] - W[6:0]) × Prescaler × 4096 / PCLK1

Where:
- T[6:0] = Counter reload value (0x40 to 0x7F)
- W[6:0] = Window value (0x40 to 0x7F)
```

---

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| CTRL | 0x00 | Control register (counter + enable) |
| CFG | 0x04 | Configuration (window + prescaler + EWI enable) |
| STS | 0x08 | Status register (EWI flag) |

### Control Register (CTRL)

| Bits | Field | Description |
|------|-------|-------------|
| 6:0 | CNT | 7-bit counter value |
| 7 | WWDTEN | WWDT enable (set-only) |

### Configuration Register (CFG)

| Bits | Field | Description |
|------|-------|-------------|
| 6:0 | WIN | 7-bit window value |
| 8:7 | DIV | Prescaler selection |
| 9 | RLDIEN | Early wakeup interrupt enable |

### Status Register (STS)

| Bits | Field | Description |
|------|-------|-------------|
| 0 | RLDF | Early wakeup interrupt flag (write 0 to clear) |

---

## Configuration Types

### Prescaler Division

```c
typedef enum
{
  WWDT_PCLK1_DIV_4096  = 0x00,  /* PCLK1 / 4096 / 1 */
  WWDT_PCLK1_DIV_8192  = 0x01,  /* PCLK1 / 4096 / 2 */
  WWDT_PCLK1_DIV_16384 = 0x02,  /* PCLK1 / 4096 / 4 */
  WWDT_PCLK1_DIV_32768 = 0x03   /* PCLK1 / 4096 / 8 */
} wwdt_division_type;
```

---

## API Reference

### Reset and Enable

```c
/* Reset WWDT registers to default */
void wwdt_reset(void);

/* Enable WWDT with initial counter value */
void wwdt_enable(uint8_t wwdt_cnt);
```

### Configuration Functions

```c
/* Set prescaler divider */
void wwdt_divider_set(wwdt_division_type division);

/* Set window counter value (0x40-0x7F) */
void wwdt_window_counter_set(uint8_t window_cnt);

/* Set counter value (reload) */
void wwdt_counter_set(uint8_t wwdt_cnt);

/* Enable early wakeup interrupt */
void wwdt_interrupt_enable(void);
```

### Status Functions

```c
/* Get early wakeup flag status */
flag_status wwdt_flag_get(void);

/* Get interrupt flag status */
flag_status wwdt_interrupt_flag_get(void);

/* Clear early wakeup flag */
void wwdt_flag_clear(void);
```

---

## Code Examples

### Example 1: Basic WWDT with Window Protection

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/**
 * @brief  Basic WWDT configuration with window mode
 *         Counter: 0x7F, Window: 0x6F
 *         Must reload when counter is between 0x6F and 0x40
 */
int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Check if reset was caused by WWDT */
    if (crm_flag_get(CRM_WWDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WWDT_RESET_FLAG);
        at32_led_on(LED4);  /* Indicate WWDT reset */
    }
    else
    {
        at32_led_off(LED4);
    }
    
    /* Enable WWDT clock (on APB1) */
    crm_periph_clock_enable(CRM_WWDT_PERIPH_CLOCK, TRUE);
    
    /* Set prescaler: PCLK1 / 32768
     * At PCLK1 = 144 MHz: counter clock = 144M / 4096 / 8 = 4394.5 Hz
     */
    wwdt_divider_set(WWDT_PCLK1_DIV_32768);
    
    /* Set window value: 0x6F
     * Reload is valid only when counter <= 0x6F
     * Window time = (0x7F - 0x6F) * 32768 / 144MHz = 3.64 ms
     */
    wwdt_window_counter_set(0x6F);
    
    /* Enable WWDT with counter = 0x7F
     * Timeout = (0x7F - 0x3F) * 32768 / 144MHz = 14.56 ms
     * Valid reload window: 3.64 ms to 14.56 ms after reload
     */
    wwdt_enable(0x7F);
    
    while (1)
    {
        at32_led_toggle(LED3);
        
        /* Wait for window to open (must be > 3.64 ms) */
        delay_ms(6);
        
        /* Reload counter - must be in valid window */
        wwdt_counter_set(0x7F);
        
        /* Button press causes infinite loop - WWDT will reset */
        if (at32_button_press() == USER_BUTTON)
        {
            while (1);
        }
    }
}
```

---

### Example 2: WWDT with Early Wakeup Interrupt

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

volatile uint8_t ewi_triggered = 0;

/**
 * @brief  WWDT Early Wakeup Interrupt Handler
 *         Called when counter reaches 0x40 - last chance to reload
 */
void WWDT_IRQHandler(void)
{
    if (wwdt_interrupt_flag_get() != RESET)
    {
        /* Clear the flag */
        wwdt_flag_clear();
        
        /* Emergency reload - prevent reset */
        wwdt_counter_set(0x7F);
        
        ewi_triggered = 1;
        at32_led_toggle(LED4);  /* Indicate EWI occurred */
    }
}

/**
 * @brief  WWDT with Early Wakeup Interrupt for last-chance recovery
 */
int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Check for WWDT reset */
    if (crm_flag_get(CRM_WWDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WWDT_RESET_FLAG);
        at32_led_on(LED2);
    }
    
    /* Enable WWDT clock */
    crm_periph_clock_enable(CRM_WWDT_PERIPH_CLOCK, TRUE);
    
    /* Configure NVIC for WWDT interrupt */
    nvic_irq_enable(WWDT_IRQn, 0, 0);
    
    /* Set prescaler */
    wwdt_divider_set(WWDT_PCLK1_DIV_32768);
    
    /* Set window value */
    wwdt_window_counter_set(0x50);
    
    /* Enable early wakeup interrupt */
    wwdt_interrupt_enable();
    
    /* Enable WWDT */
    wwdt_enable(0x7F);
    
    while (1)
    {
        at32_led_toggle(LED3);
        
        /* Normal operation - reload in window */
        delay_ms(10);
        wwdt_counter_set(0x7F);
        
        /* Reset EWI flag */
        if (ewi_triggered)
        {
            ewi_triggered = 0;
        }
        
        /* Simulate sporadic long operation */
        if (at32_button_press() == USER_BUTTON)
        {
            /* Long delay - will trigger EWI but not reset
             * thanks to EWI handler reloading counter
             */
            delay_ms(50);
        }
    }
}
```

---

### Example 3: Precise Timing Window Configuration

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/**
 * @brief  Calculate and configure precise WWDT timing
 *         Target: 10ms window, 20ms timeout
 */

/* Calculate WWDT parameters for desired timing
 * PCLK1 = 144 MHz (AHB/2)
 * Base divider = 4096
 */
#define PCLK1_FREQ       144000000
#define WWDT_BASE_DIV    4096

/* For DIV_32768 (4096 * 8):
 * Counter tick = 144MHz / 32768 = 4394.5 Hz
 * Tick period = 1 / 4394.5 = 0.2275 ms per tick
 */
#define WWDT_PRESCALER   8
#define TICK_PERIOD_US   ((WWDT_PRESCALER * WWDT_BASE_DIV * 1000000UL) / PCLK1_FREQ)

/* Timeout = 20ms -> ticks needed = 20000 / 227.5 = 88 ticks
 * Counter range is 0x40 to 0x7F (64 ticks max)
 * Use 64 ticks = 14.56 ms timeout
 */
#define TIMEOUT_TICKS    64
#define COUNTER_VALUE    (0x3F + TIMEOUT_TICKS)  /* 0x7F */

/* Window = 5ms -> window ticks = 5000 / 227.5 = 22 ticks
 * Window value = counter - window_ticks = 0x7F - 22 = 0x69
 */
#define WINDOW_TICKS     22
#define WINDOW_VALUE     (COUNTER_VALUE - WINDOW_TICKS)

int main(void)
{
    uint32_t tick_us;
    uint32_t timeout_us;
    uint32_t window_us;
    
    system_clock_config();
    at32_board_init();
    
    /* Calculate actual timing */
    tick_us = TICK_PERIOD_US;
    timeout_us = TIMEOUT_TICKS * tick_us;
    window_us = WINDOW_TICKS * tick_us;
    
    /* Print timing info (if UART available) */
    /* printf("Tick: %lu us, Timeout: %lu us, Window: %lu us\n", 
           tick_us, timeout_us, window_us); */
    
    /* Check for WWDT reset */
    if (crm_flag_get(CRM_WWDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WWDT_RESET_FLAG);
        at32_led_on(LED4);
    }
    
    crm_periph_clock_enable(CRM_WWDT_PERIPH_CLOCK, TRUE);
    
    /* Configure with calculated values */
    wwdt_divider_set(WWDT_PCLK1_DIV_32768);
    wwdt_window_counter_set(WINDOW_VALUE);
    wwdt_enable(COUNTER_VALUE);
    
    while (1)
    {
        /* Reload in valid window: after window_us, before timeout_us */
        delay_ms(8);  /* ~8ms > 5ms window, < 14.56ms timeout */
        wwdt_counter_set(COUNTER_VALUE);
        
        at32_led_toggle(LED3);
    }
}
```

---

### Example 4: WWDT for Critical Task Monitoring

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

typedef enum
{
    TASK_IDLE,
    TASK_ADC_SAMPLE,
    TASK_PROCESS,
    TASK_TRANSMIT
} task_phase_type;

volatile task_phase_type current_phase = TASK_IDLE;
volatile uint32_t phase_start_time = 0;

#define PHASE_TIMEOUT_MS  5  /* Each phase must complete within 5ms */

/**
 * @brief  WWDT monitors critical task phases
 *         Each phase must complete and reload WWDT
 */
int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Check reset source */
    if (crm_flag_get(CRM_WWDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WWDT_RESET_FLAG);
        /* Log which phase caused timeout */
        at32_led_on(LED4);
    }
    
    crm_periph_clock_enable(CRM_WWDT_PERIPH_CLOCK, TRUE);
    
    /* Configure WWDT: ~7.3ms timeout with 2ms window */
    wwdt_divider_set(WWDT_PCLK1_DIV_16384);
    wwdt_window_counter_set(0x70);  /* Window starts ~2ms after reload */
    wwdt_enable(0x7F);
    
    while (1)
    {
        switch (current_phase)
        {
            case TASK_IDLE:
                /* Wait for trigger */
                delay_ms(1);
                current_phase = TASK_ADC_SAMPLE;
                break;
                
            case TASK_ADC_SAMPLE:
                /* Simulate ADC sampling */
                at32_led_on(LED2);
                delay_ms(1);
                at32_led_off(LED2);
                
                /* Phase complete - enter window and reload */
                delay_ms(2);  /* Wait for window */
                wwdt_counter_set(0x7F);
                
                current_phase = TASK_PROCESS;
                break;
                
            case TASK_PROCESS:
                /* Simulate data processing */
                at32_led_on(LED3);
                delay_ms(2);
                at32_led_off(LED3);
                
                /* Reload WWDT */
                delay_ms(2);
                wwdt_counter_set(0x7F);
                
                current_phase = TASK_TRANSMIT;
                break;
                
            case TASK_TRANSMIT:
                /* Simulate transmission */
                at32_led_toggle(LED2);
                delay_ms(1);
                
                /* Reload WWDT */
                delay_ms(2);
                wwdt_counter_set(0x7F);
                
                current_phase = TASK_IDLE;
                break;
        }
        
        /* Button simulates a phase hanging */
        if (at32_button_press() == USER_BUTTON)
        {
            while (1);  /* Hang - WWDT will reset */
        }
    }
}
```

---

### Example 5: WWDT Combined with System Diagnostics

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

typedef struct
{
    uint32_t wwdt_reset_count;
    uint32_t ewi_count;
    uint32_t last_counter_value;
    uint8_t  ewi_recovery_enabled;
} wwdt_diagnostics_type;

/* Store in backup SRAM or battery-backed register if available */
wwdt_diagnostics_type wwdt_diag = {0};

void WWDT_IRQHandler(void)
{
    if (wwdt_interrupt_flag_get() != RESET)
    {
        wwdt_flag_clear();
        wwdt_diag.ewi_count++;
        wwdt_diag.last_counter_value = WWDT->ctrl_bit.cnt;
        
        if (wwdt_diag.ewi_recovery_enabled)
        {
            /* Emergency reload */
            wwdt_counter_set(0x7F);
            at32_led_on(LED4);
        }
        /* If recovery disabled, system will reset */
    }
}

/**
 * @brief  WWDT with diagnostics and configurable EWI recovery
 */
int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Check reset source and update diagnostics */
    if (crm_flag_get(CRM_WWDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WWDT_RESET_FLAG);
        wwdt_diag.wwdt_reset_count++;
        at32_led_on(LED2);  /* Indicate WWDT reset */
    }
    
    /* Enable EWI recovery by default */
    wwdt_diag.ewi_recovery_enabled = 1;
    
    crm_periph_clock_enable(CRM_WWDT_PERIPH_CLOCK, TRUE);
    nvic_irq_enable(WWDT_IRQn, 0, 0);
    
    wwdt_divider_set(WWDT_PCLK1_DIV_32768);
    wwdt_window_counter_set(0x50);
    wwdt_interrupt_enable();
    wwdt_enable(0x7F);
    
    while (1)
    {
        /* Normal watchdog feeding */
        delay_ms(10);
        wwdt_counter_set(0x7F);
        at32_led_off(LED4);
        
        at32_led_toggle(LED3);
        
        /* Button toggles EWI recovery mode */
        if (at32_button_press() == USER_BUTTON)
        {
            wwdt_diag.ewi_recovery_enabled = !wwdt_diag.ewi_recovery_enabled;
            
            if (wwdt_diag.ewi_recovery_enabled)
            {
                at32_led_off(LED2);  /* Recovery enabled */
            }
            else
            {
                at32_led_on(LED2);   /* Recovery disabled - will reset on EWI */
            }
            
            /* Long delay to trigger EWI */
            delay_ms(50);
        }
    }
}
```

---

### Example 6: Dual Watchdog Configuration (WDT + WWDT)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/**
 * @brief  Use both WDT and WWDT for layered protection
 *         WDT: Long timeout (1s) - catches complete system hangs
 *         WWDT: Short timeout (15ms) - catches task-level issues
 */

void WWDT_IRQHandler(void)
{
    if (wwdt_interrupt_flag_get() != RESET)
    {
        wwdt_flag_clear();
        wwdt_counter_set(0x7F);  /* Emergency reload */
        at32_led_toggle(LED4);
    }
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Check reset sources */
    if (crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WDT_RESET_FLAG);
        at32_led_on(LED2);  /* WDT reset - major failure */
    }
    
    if (crm_flag_get(CRM_WWDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WWDT_RESET_FLAG);
        at32_led_on(LED4);  /* WWDT reset - task timeout */
    }
    
    /* ===== Configure WDT (Independent Watchdog) ===== */
    /* 1 second timeout - backup watchdog */
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_256);  /* LICK / 256 */
    wdt_reload_value_set(156 - 1);     /* ~1 second */
    wdt_counter_reload();
    wdt_enable();
    
    /* ===== Configure WWDT (Window Watchdog) ===== */
    /* 15ms timeout - primary watchdog */
    crm_periph_clock_enable(CRM_WWDT_PERIPH_CLOCK, TRUE);
    nvic_irq_enable(WWDT_IRQn, 1, 0);
    
    wwdt_divider_set(WWDT_PCLK1_DIV_32768);
    wwdt_window_counter_set(0x50);
    wwdt_interrupt_enable();
    wwdt_enable(0x7F);
    
    while (1)
    {
        /* Fast task loop - feed WWDT frequently */
        for (int i = 0; i < 10; i++)
        {
            delay_ms(8);
            wwdt_counter_set(0x7F);  /* Feed WWDT */
            at32_led_toggle(LED3);
        }
        
        /* Feed WDT less frequently */
        wdt_counter_reload();
        
        /* Button simulates different failure modes */
        if (at32_button_press() == USER_BUTTON)
        {
            /* Long delay - WDT will eventually reset */
            /* WWDT EWI keeps it alive until WDT times out */
            while (1)
            {
                delay_ms(100);
            }
        }
    }
}
```

---

## Configuration Checklist

### Basic Setup
- [ ] Enable WWDT clock: `crm_periph_clock_enable(CRM_WWDT_PERIPH_CLOCK, TRUE)`
- [ ] Set prescaler with `wwdt_divider_set()`
- [ ] Set window value with `wwdt_window_counter_set()`
- [ ] Enable WWDT with `wwdt_enable(counter_value)`

### Window Mode Configuration
- [ ] Counter value must be between 0x40 and 0x7F
- [ ] Window value must be ≤ counter value
- [ ] Reload only when counter ≤ window value
- [ ] Reload too early causes immediate reset

### Early Wakeup Interrupt (Optional)
- [ ] Enable interrupt: `wwdt_interrupt_enable()`
- [ ] Configure NVIC: `nvic_irq_enable(WWDT_IRQn, priority, 0)`
- [ ] Clear flag in ISR: `wwdt_flag_clear()`
- [ ] Emergency reload in ISR if needed

### Important Notes
- [ ] WWDT cannot be disabled once enabled
- [ ] Dependent on PCLK1 - not fail-safe for clock issues
- [ ] Does NOT run in Standby mode
- [ ] Check `CRM_WWDT_RESET_FLAG` on startup

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Reset immediately | Reload too early | Wait for window to open |
| Reset after config | Window > Counter | Set window ≤ counter |
| Unexpected resets | Timeout too short | Increase prescaler |
| EWI not firing | Interrupt not enabled | Call `wwdt_interrupt_enable()` |
| NVIC not configured | Enable WWDT_IRQn | |
| Variable timing | PCLK1 varies | Account for all clock configs |
| No reset in standby | By design | WWDT stops in low power modes |

---

## Performance Specifications (PCLK1 = 144 MHz)

| Parameter | DIV_4096 | DIV_8192 | DIV_16384 | DIV_32768 |
|-----------|----------|----------|-----------|-----------|
| Counter Clock | 35.2 kHz | 17.6 kHz | 8.8 kHz | 4.4 kHz |
| Min Timeout | 1.8 ms | 3.6 ms | 7.3 ms | 14.6 ms |
| Max Timeout | 3.6 ms | 7.3 ms | 14.6 ms | 29.1 ms |
| Tick Period | 28.4 µs | 56.9 µs | 113.8 µs | 227.6 µs |

---

## Related Peripherals

| Peripheral | Relationship |
|------------|--------------|
| WDT | Independent watchdog (longer timeouts, standby operation) |
| CRM | Clock source (PCLK1), reset flag detection |
| NVIC | Early wakeup interrupt |
| DEBUG | Can freeze WWDT during debug |

---

## References

- AT32F435/437 Reference Manual - WWDT Chapter
- Application Note AN0045 - Watchdog Timer Usage
- AT32F435/437 Datasheet - APB1 Clock Specifications

