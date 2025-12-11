---
title: EXINT - External Interrupt Controller
category: Interrupt
complexity: Basic
mcu: AT32F435/437
peripheral: EXINT
keywords: [exint, external interrupt, event, gpio, edge trigger, software trigger]
---

# EXINT - External Interrupt Controller

## Overview

The External Interrupt/Event Controller (EXINT) manages external and internal interrupt/event signals. It provides 23 lines that can be independently configured as interrupt or event sources with programmable trigger polarity. Lines 0-15 are connected to GPIO pins through SCFG multiplexing, while lines 16-22 handle internal peripheral events like ERTC, USB wakeup, and PVD.

### Key Features

| Feature | Specification |
|---------|---------------|
| Total Lines | 23 (EXINT_LINE_0 to EXINT_LINE_22) |
| GPIO Lines | Lines 0-15 (multiplexed from all GPIO ports) |
| Internal Lines | Lines 16-22 (dedicated peripheral events) |
| Trigger Modes | Rising edge, Falling edge, Both edges |
| Output Modes | Interrupt request or Event (for wakeup) |
| Software Trigger | Yes, on any line |
| Pending Register | Read/clear interrupt flags |

---

## Architecture

```
                    ┌─────────────────────────────────────────────────┐
                    │              EXINT Controller                   │
                    │                                                 │
   GPIO Ports       │   SCFG          ┌──────────┐                    │
   ┌─────┐         │   MUX            │  Edge    │     ┌─────────┐    │
   │GPIOA├─PA0────►│───┬───►LINE0────►│ Detect   ├────►│ INTEN   │───►│ NVIC
   │GPIOB├─PB0────►│───┤              │          │     │ Mask    │    │
   │GPIOC├─PC0────►│───┤     LINE1───►│ POLCFG1  │     └───┬────┘     │
   │GPIOD├─PD0────►│───┤     ...      │ (Rising) │         │          │
   │GPIOE├─PE0────►│───┤     LINE15──►│          │    ┌────▼────┐     │
   │GPIOF├─PF0────►│───┤              │ POLCFG2  │    │  INTSTS │     │
   │GPIOG├─PG0────►│───┤              │(Falling) │    │ Pending │     │
   │GPIOH├─PH0────►│───┘              └────┬─────┘    └─────────┘     │
   └─────┘         │                       │                          │
                   │                  ┌────▼────┐     ┌─────────┐     │
  Internal Events  │                  │  SWTRG  │     │  EVTEN  │────►│ Event
  ┌─────────────┐  │    LINE16───────►│Software │────►│  Mask   │     │ Output
  │ PVD Output  ├──┼───►LINE17        │ Trigger │     └─────────┘     │ (Wakeup)
  │ ERTC Alarm  ├──┼───►LINE18        └─────────┘                     │
  │ USB FS Wake ├──┼───►LINE19                                        │
  │ ETH Wakeup  ├──┼───►LINE20                                        │
  │ USB HS Wake ├──┼───►LINE21                                        │
  │ ERTC Tamper ├──┼───►LINE22                                        │
  │ ERTC Wakeup ├──┼───►...                                           │
  └─────────────┘  │                                                  │
                    └─────────────────────────────────────────────────┘
```

---

## EXINT Line Mapping

### GPIO Lines (0-15)

Each EXINT line 0-15 can be connected to any GPIO port through SCFG multiplexing:

| EXINT Line | Available GPIO Pins | SCFG Register |
|------------|---------------------|---------------|
| LINE_0 | PA0, PB0, PC0, PD0, PE0, PF0, PG0, PH0 | EXINTC1 |
| LINE_1 | PA1, PB1, PC1, PD1, PE1, PF1, PG1, PH1 | EXINTC1 |
| LINE_2 | PA2, PB2, PC2, PD2, PE2, PF2, PG2, PH2 | EXINTC1 |
| LINE_3 | PA3, PB3, PC3, PD3, PE3, PF3, PG3, PH3 | EXINTC1 |
| LINE_4 | PA4, PB4, PC4, PD4, PE4, PF4, PG4, PH4 | EXINTC2 |
| LINE_5 | PA5, PB5, PC5, PD5, PE5, PF5, PG5, PH5 | EXINTC2 |
| LINE_6 | PA6, PB6, PC6, PD6, PE6, PF6, PG6, PH6 | EXINTC2 |
| LINE_7 | PA7, PB7, PC7, PD7, PE7, PF7, PG7, PH7 | EXINTC2 |
| LINE_8 | PA8, PB8, PC8, PD8, PE8, PF8, PG8, PH8 | EXINTC3 |
| LINE_9 | PA9, PB9, PC9, PD9, PE9, PF9, PG9, PH9 | EXINTC3 |
| LINE_10 | PA10, PB10, PC10, PD10, PE10, PF10, PG10, PH10 | EXINTC3 |
| LINE_11 | PA11, PB11, PC11, PD11, PE11, PF11, PG11, PH11 | EXINTC3 |
| LINE_12 | PA12, PB12, PC12, PD12, PE12, PF12, PG12, PH12 | EXINTC4 |
| LINE_13 | PA13, PB13, PC13, PD13, PE13, PF13, PG13, PH13 | EXINTC4 |
| LINE_14 | PA14, PB14, PC14, PD14, PE14, PF14, PG14, PH14 | EXINTC4 |
| LINE_15 | PA15, PB15, PC15, PD15, PE15, PF15, PG15, PH15 | EXINTC4 |

### Internal Peripheral Lines (16-22)

| EXINT Line | Connected Peripheral | Description |
|------------|---------------------|-------------|
| LINE_16 | PVD | Programmable Voltage Detector output |
| LINE_17 | ERTC Alarm | ERTC Alarm A and B event |
| LINE_18 | USB OTG FS | USB OTG FS wakeup event |
| LINE_19 | Ethernet | Ethernet wakeup event |
| LINE_20 | USB OTG HS | USB OTG HS wakeup event |
| LINE_21 | ERTC Tamper/Timestamp | ERTC tamper and timestamp events |
| LINE_22 | ERTC Wakeup | ERTC wakeup timer event |

---

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| INTEN | 0x00 | Interrupt enable register |
| EVTEN | 0x04 | Event enable register |
| POLCFG1 | 0x08 | Rising edge trigger configuration |
| POLCFG2 | 0x0C | Falling edge trigger configuration |
| SWTRG | 0x10 | Software trigger register |
| INTSTS | 0x14 | Interrupt/event pending status |

### Register Bit Definitions

**INTEN - Interrupt Enable Register (0x00)**

| Bits | Name | Description |
|------|------|-------------|
| 22:0 | INTENx | Interrupt enable for line x (1 = enabled) |
| 31:23 | Reserved | - |

**EVTEN - Event Enable Register (0x04)**

| Bits | Name | Description |
|------|------|-------------|
| 22:0 | EVTENx | Event enable for line x (1 = enabled) |
| 31:23 | Reserved | - |

**POLCFG1 - Rising Edge Trigger Register (0x08)**

| Bits | Name | Description |
|------|------|-------------|
| 22:0 | RPx | Rising edge trigger for line x (1 = enabled) |
| 31:23 | Reserved | - |

**POLCFG2 - Falling Edge Trigger Register (0x0C)**

| Bits | Name | Description |
|------|------|-------------|
| 22:0 | FPx | Falling edge trigger for line x (1 = enabled) |
| 31:23 | Reserved | - |

**SWTRG - Software Trigger Register (0x10)**

| Bits | Name | Description |
|------|------|-------------|
| 22:0 | SWTx | Software trigger for line x (write 1 to trigger) |
| 31:23 | Reserved | - |

**INTSTS - Interrupt Status Register (0x14)**

| Bits | Name | Description |
|------|------|-------------|
| 22:0 | LINEx | Interrupt pending for line x (write 1 to clear) |
| 31:23 | Reserved | - |

---

## NVIC IRQ Mapping

| EXINT Line(s) | IRQ Handler | IRQn |
|---------------|-------------|------|
| LINE_0 | EXINT0_IRQHandler | EXINT0_IRQn |
| LINE_1 | EXINT1_IRQHandler | EXINT1_IRQn |
| LINE_2 | EXINT2_IRQHandler | EXINT2_IRQn |
| LINE_3 | EXINT3_IRQHandler | EXINT3_IRQn |
| LINE_4 | EXINT4_IRQHandler | EXINT4_IRQn |
| LINE_5 to LINE_9 | EXINT9_5_IRQHandler | EXINT9_5_IRQn |
| LINE_10 to LINE_15 | EXINT15_10_IRQHandler | EXINT15_10_IRQn |

> **Note:** Lines 5-9 share a single interrupt vector, as do lines 10-15. Your ISR must check which specific line(s) triggered the interrupt.

---

## Configuration Options

### Line Mode

```c
typedef enum {
  EXINT_LINE_INTERRUPT = 0x00,  /* Generate interrupt request to NVIC */
  EXINT_LINE_EVENT     = 0x01   /* Generate event pulse (for wakeup) */
} exint_line_mode_type;
```

### Trigger Polarity

```c
typedef enum {
  EXINT_TRIGGER_RISING_EDGE  = 0x00,  /* Trigger on rising edge */
  EXINT_TRIGGER_FALLING_EDGE = 0x01,  /* Trigger on falling edge */
  EXINT_TRIGGER_BOTH_EDGE    = 0x02   /* Trigger on both edges */
} exint_polarity_config_type;
```

### Line Definitions

```c
#define EXINT_LINE_NONE   ((uint32_t)0x000000)
#define EXINT_LINE_0      ((uint32_t)0x000001)
#define EXINT_LINE_1      ((uint32_t)0x000002)
#define EXINT_LINE_2      ((uint32_t)0x000004)
#define EXINT_LINE_3      ((uint32_t)0x000008)
#define EXINT_LINE_4      ((uint32_t)0x000010)
#define EXINT_LINE_5      ((uint32_t)0x000020)
#define EXINT_LINE_6      ((uint32_t)0x000040)
#define EXINT_LINE_7      ((uint32_t)0x000080)
#define EXINT_LINE_8      ((uint32_t)0x000100)
#define EXINT_LINE_9      ((uint32_t)0x000200)
#define EXINT_LINE_10     ((uint32_t)0x000400)
#define EXINT_LINE_11     ((uint32_t)0x000800)
#define EXINT_LINE_12     ((uint32_t)0x001000)
#define EXINT_LINE_13     ((uint32_t)0x002000)
#define EXINT_LINE_14     ((uint32_t)0x004000)
#define EXINT_LINE_15     ((uint32_t)0x008000)
#define EXINT_LINE_16     ((uint32_t)0x010000)  /* PVD */
#define EXINT_LINE_17     ((uint32_t)0x020000)  /* ERTC Alarm */
#define EXINT_LINE_18     ((uint32_t)0x040000)  /* USB OTG FS Wakeup */
#define EXINT_LINE_19     ((uint32_t)0x080000)  /* Ethernet Wakeup */
#define EXINT_LINE_20     ((uint32_t)0x100000)  /* USB OTG HS Wakeup */
#define EXINT_LINE_21     ((uint32_t)0x200000)  /* ERTC Tamper/Timestamp */
#define EXINT_LINE_22     ((uint32_t)0x400000)  /* ERTC Wakeup */
```

---

## API Reference

### Initialization Functions

```c
/**
  * @brief  Reset EXINT registers to default values
  * @param  none
  * @retval none
  */
void exint_reset(void);

/**
  * @brief  Initialize exint_init_type structure with default values
  * @param  exint_struct: pointer to exint_init_type structure
  * @retval none
  * @note   Default: disabled, event mode, falling edge, no line selected
  */
void exint_default_para_init(exint_init_type *exint_struct);

/**
  * @brief  Initialize EXINT according to specified parameters
  * @param  exint_struct: pointer to exint_init_type structure
  *         - line_enable: TRUE or FALSE
  *         - line_mode: EXINT_LINE_INTERRUPT or EXINT_LINE_EVENT
  *         - line_select: EXINT_LINE_x (can be OR'd for multiple lines)
  *         - line_polarity: EXINT_TRIGGER_RISING_EDGE, FALLING_EDGE, or BOTH_EDGE
  * @retval none
  */
void exint_init(exint_init_type *exint_struct);
```

### Flag Functions

```c
/**
  * @brief  Clear EXINT pending flag
  * @param  exint_line: EXINT_LINE_x (can be OR'd for multiple lines)
  * @retval none
  */
void exint_flag_clear(uint32_t exint_line);

/**
  * @brief  Get EXINT pending flag status
  * @param  exint_line: single EXINT_LINE_x value
  * @retval flag_status: SET or RESET
  */
flag_status exint_flag_get(uint32_t exint_line);

/**
  * @brief  Get EXINT interrupt flag (checks both pending and enabled)
  * @param  exint_line: single EXINT_LINE_x value
  * @retval flag_status: SET or RESET
  * @note   Use this in ISR to verify interrupt source
  */
flag_status exint_interrupt_flag_get(uint32_t exint_line);
```

### Control Functions

```c
/**
  * @brief  Generate software interrupt/event on specified line
  * @param  exint_line: EXINT_LINE_x value
  * @retval none
  * @note   Sets pending flag and triggers interrupt if enabled
  */
void exint_software_interrupt_event_generate(uint32_t exint_line);

/**
  * @brief  Enable or disable EXINT interrupt for specified line(s)
  * @param  exint_line: EXINT_LINE_x (can be OR'd for multiple lines)
  * @param  new_state: TRUE (enable) or FALSE (disable)
  * @retval none
  */
void exint_interrupt_enable(uint32_t exint_line, confirm_state new_state);

/**
  * @brief  Enable or disable EXINT event for specified line(s)
  * @param  exint_line: EXINT_LINE_x (can be OR'd for multiple lines)
  * @param  new_state: TRUE (enable) or FALSE (disable)
  * @retval none
  */
void exint_event_enable(uint32_t exint_line, confirm_state new_state);
```

### SCFG Functions (for GPIO mapping)

```c
/**
  * @brief  Configure GPIO port source for EXINT line
  * @param  port_source: SCFG_PORT_SOURCE_GPIOx (A through H)
  * @param  pin_source: SCFG_PINS_SOURCEx (0 through 15)
  * @retval none
  * @note   Requires CRM_SCFG_PERIPH_CLOCK enabled
  */
void scfg_exint_line_config(scfg_port_source_type port_source, 
                            scfg_pins_source_type pin_source);
```

### SCFG Port Source Definitions

```c
typedef enum {
  SCFG_PORT_SOURCE_GPIOA = 0x00,
  SCFG_PORT_SOURCE_GPIOB = 0x01,
  SCFG_PORT_SOURCE_GPIOC = 0x02,
  SCFG_PORT_SOURCE_GPIOD = 0x03,
  SCFG_PORT_SOURCE_GPIOE = 0x04,
  SCFG_PORT_SOURCE_GPIOF = 0x05,
  SCFG_PORT_SOURCE_GPIOG = 0x06,
  SCFG_PORT_SOURCE_GPIOH = 0x07
} scfg_port_source_type;

typedef enum {
  SCFG_PINS_SOURCE0  = 0x00,
  SCFG_PINS_SOURCE1  = 0x01,
  /* ... */
  SCFG_PINS_SOURCE15 = 0x0F
} scfg_pins_source_type;
```

---

## Code Examples

### Example 1: Basic GPIO Interrupt (Rising Edge)

Configure PA0 to generate an interrupt on rising edge:

```c
#include "at32f435_437.h"

volatile uint32_t interrupt_count = 0;

/**
  * @brief  Configure EXINT Line 0 for PA0 rising edge interrupt
  */
void exint_line0_config(void)
{
    exint_init_type exint_init_struct;
    gpio_init_type gpio_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    
    /* Configure PA0 as input with pull-down */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_0;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Connect PA0 to EXINT Line 0 */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE0);
    
    /* Configure EXINT Line 0 */
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_0;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);
    
    /* Enable NVIC interrupt */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(EXINT0_IRQn, 1, 0);
}

/**
  * @brief  EXINT Line 0 interrupt handler
  */
void EXINT0_IRQHandler(void)
{
    if(exint_interrupt_flag_get(EXINT_LINE_0) != RESET)
    {
        interrupt_count++;
        /* Your interrupt handling code here */
        
        /* Clear pending flag (REQUIRED!) */
        exint_flag_clear(EXINT_LINE_0);
    }
}

int main(void)
{
    system_clock_config();
    exint_line0_config();
    
    while(1)
    {
        /* Main loop */
    }
}
```

---

### Example 2: Button with Debounce (Both Edges)

Configure a button on PC13 with both edge detection:

```c
#include "at32f435_437.h"

volatile uint32_t button_press_count = 0;
volatile uint32_t last_press_time = 0;

#define DEBOUNCE_TIME_MS  50

/**
  * @brief  Configure button on PC13 with both edge detection
  */
void button_exint_config(void)
{
    exint_init_type exint_init_struct;
    gpio_init_type gpio_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    
    /* Configure PC13 as input with pull-up (button pulls to GND) */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_13;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOC, &gpio_init_struct);
    
    /* Connect PC13 to EXINT Line 13 */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE13);
    
    /* Configure EXINT Line 13 for both edges */
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_13;
    exint_init_struct.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
    exint_init(&exint_init_struct);
    
    /* Enable NVIC interrupt (lines 10-15 share one vector) */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(EXINT15_10_IRQn, 2, 0);
}

/**
  * @brief  Get current system tick in milliseconds
  */
extern uint32_t system_tick_ms;

/**
  * @brief  EXINT Lines 10-15 interrupt handler
  */
void EXINT15_10_IRQHandler(void)
{
    /* Check if Line 13 triggered */
    if(exint_interrupt_flag_get(EXINT_LINE_13) != RESET)
    {
        uint32_t current_time = system_tick_ms;
        
        /* Simple debounce check */
        if((current_time - last_press_time) > DEBOUNCE_TIME_MS)
        {
            /* Check if button is pressed (LOW) or released (HIGH) */
            if(gpio_input_data_bit_read(GPIOC, GPIO_PINS_13) == RESET)
            {
                /* Button pressed */
                button_press_count++;
            }
            else
            {
                /* Button released */
            }
            last_press_time = current_time;
        }
        
        exint_flag_clear(EXINT_LINE_13);
    }
    
    /* Check other lines 10-15 if configured */
    if(exint_interrupt_flag_get(EXINT_LINE_10) != RESET)
    {
        /* Handle Line 10 */
        exint_flag_clear(EXINT_LINE_10);
    }
    /* ... check other lines as needed */
}
```

---

### Example 3: Software Trigger with Timer

Use timer overflow to trigger software interrupt:

```c
#include "at32f435_437.h"

volatile uint32_t sw_trigger_count = 0;

/**
  * @brief  Configure EXINT Line 4 for software trigger
  */
void exint_software_trigger_config(void)
{
    exint_init_type exint_init_struct;
    
    /* Enable SCFG clock (needed for EXINT) */
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    
    /* Configure EXINT Line 4 for interrupt */
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_4;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);
    
    /* Clear any pending flag */
    exint_flag_clear(EXINT_LINE_4);
    
    /* Enable NVIC */
    nvic_irq_enable(EXINT4_IRQn, 1, 0);
}

/**
  * @brief  Configure TMR1 for 1Hz overflow
  */
void tmr1_config(void)
{
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    
    /* Configure for 1 second period */
    /* Assuming 288 MHz system clock with APB2 timer clock */
    tmr_base_init(TMR1, 10000-1, 28800-1);  /* 288MHz/28800/10000 = 1Hz */
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
    
    /* Enable overflow interrupt */
    tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);
    nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 0, 0);
}

/**
  * @brief  TMR1 overflow handler - generates software EXINT trigger
  */
void TMR1_OVF_TMR10_IRQHandler(void)
{
    if(tmr_interrupt_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
    {
        /* Generate software interrupt on EXINT Line 4 */
        exint_software_interrupt_event_generate(EXINT_LINE_4);
        
        tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    }
}

/**
  * @brief  EXINT Line 4 handler (triggered by software)
  */
void EXINT4_IRQHandler(void)
{
    if(exint_interrupt_flag_get(EXINT_LINE_4) != RESET)
    {
        sw_trigger_count++;
        /* Handle software triggered interrupt */
        
        exint_flag_clear(EXINT_LINE_4);
    }
}

int main(void)
{
    system_clock_config();
    
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    
    exint_software_trigger_config();
    tmr1_config();
    
    /* Start timer */
    tmr_counter_enable(TMR1, TRUE);
    
    while(1)
    {
        /* sw_trigger_count increments every second */
    }
}
```

---

### Example 4: Multiple Lines with Shared Handler

Configure multiple EXINT lines (5-9) sharing one interrupt handler:

```c
#include "at32f435_437.h"

typedef struct {
    uint32_t line5_count;
    uint32_t line6_count;
    uint32_t line7_count;
    uint32_t line8_count;
    uint32_t line9_count;
} exint_counters_t;

volatile exint_counters_t counters = {0};

/**
  * @brief  Configure multiple EXINT lines on different ports
  */
void exint_multi_line_config(void)
{
    exint_init_type exint_init_struct;
    gpio_init_type gpio_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    
    /* Configure GPIO pins as inputs */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    
    /* PA5, PB6, PC7, PA8, PB9 as inputs */
    gpio_init_struct.gpio_pins = GPIO_PINS_5 | GPIO_PINS_8;
    gpio_init(GPIOA, &gpio_init_struct);
    
    gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_9;
    gpio_init(GPIOB, &gpio_init_struct);
    
    gpio_init_struct.gpio_pins = GPIO_PINS_7;
    gpio_init(GPIOC, &gpio_init_struct);
    
    /* Map GPIO to EXINT lines */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE5);  /* Line 5 = PA5 */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, SCFG_PINS_SOURCE6);  /* Line 6 = PB6 */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE7);  /* Line 7 = PC7 */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE8);  /* Line 8 = PA8 */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, SCFG_PINS_SOURCE9);  /* Line 9 = PB9 */
    
    /* Configure all lines at once (same settings) */
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
    
    /* Configure each line individually if different polarity needed */
    exint_init_struct.line_select = EXINT_LINE_5;
    exint_init(&exint_init_struct);
    
    exint_init_struct.line_select = EXINT_LINE_6;
    exint_init(&exint_init_struct);
    
    exint_init_struct.line_select = EXINT_LINE_7;
    exint_init(&exint_init_struct);
    
    exint_init_struct.line_select = EXINT_LINE_8;
    exint_init(&exint_init_struct);
    
    exint_init_struct.line_select = EXINT_LINE_9;
    exint_init(&exint_init_struct);
    
    /* Enable shared NVIC interrupt for lines 5-9 */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(EXINT9_5_IRQn, 1, 0);
}

/**
  * @brief  Shared handler for EXINT lines 5-9
  */
void EXINT9_5_IRQHandler(void)
{
    /* Check and handle each possible line */
    if(exint_interrupt_flag_get(EXINT_LINE_5) != RESET)
    {
        counters.line5_count++;
        exint_flag_clear(EXINT_LINE_5);
    }
    
    if(exint_interrupt_flag_get(EXINT_LINE_6) != RESET)
    {
        counters.line6_count++;
        exint_flag_clear(EXINT_LINE_6);
    }
    
    if(exint_interrupt_flag_get(EXINT_LINE_7) != RESET)
    {
        counters.line7_count++;
        exint_flag_clear(EXINT_LINE_7);
    }
    
    if(exint_interrupt_flag_get(EXINT_LINE_8) != RESET)
    {
        counters.line8_count++;
        exint_flag_clear(EXINT_LINE_8);
    }
    
    if(exint_interrupt_flag_get(EXINT_LINE_9) != RESET)
    {
        counters.line9_count++;
        exint_flag_clear(EXINT_LINE_9);
    }
}
```

---

### Example 5: Event Mode for Low Power Wakeup

Use EXINT event mode to wake from sleep:

```c
#include "at32f435_437.h"

/**
  * @brief  Configure PA0 as wakeup source using event mode
  */
void exint_wakeup_config(void)
{
    exint_init_type exint_init_struct;
    gpio_init_type gpio_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    
    /* Configure PA0 as input */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_0;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Connect PA0 to EXINT Line 0 */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE0);
    
    /* Configure EXINT Line 0 in EVENT mode (not interrupt) */
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_EVENT;  /* Event mode for wakeup */
    exint_init_struct.line_select = EXINT_LINE_0;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);
    
    /* Note: No NVIC enable needed for event mode */
}

/**
  * @brief  Enter sleep mode and wait for wakeup event
  */
void enter_sleep_mode(void)
{
    /* Clear any pending EXINT flags */
    exint_flag_clear(EXINT_LINE_0);
    
    /* Enter sleep mode - will wake on PA0 rising edge event */
    /* Using WFE (Wait For Event) instruction */
    __WFE();
    
    /* Execution continues here after wakeup */
}

int main(void)
{
    system_clock_config();
    
    /* Configure LED for status indication */
    /* ... LED init code ... */
    
    /* Configure wakeup source */
    exint_wakeup_config();
    
    while(1)
    {
        /* Turn off LED before sleep */
        /* gpio_bits_reset(GPIOC, GPIO_PINS_13); */
        
        /* Enter sleep mode */
        enter_sleep_mode();
        
        /* Woke up - turn on LED */
        /* gpio_bits_set(GPIOC, GPIO_PINS_13); */
        
        /* Do some work after wakeup */
        delay_ms(1000);
    }
}
```

---

### Example 6: ERTC Alarm Wakeup (Internal Line)

Configure ERTC Alarm to generate wakeup interrupt:

```c
#include "at32f435_437.h"

volatile flag_status alarm_triggered = RESET;

/**
  * @brief  Configure EXINT Line 17 for ERTC Alarm
  */
void exint_ertc_alarm_config(void)
{
    exint_init_type exint_init_struct;
    
    /* EXINT Line 17 is internally connected to ERTC Alarm */
    /* No GPIO or SCFG configuration needed */
    
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_17;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);
    
    /* ERTC Alarm interrupt is handled by ERTC_IRQHandler */
    /* But EXINT flag must also be cleared */
}

/**
  * @brief  Configure ERTC Alarm A to trigger in 10 seconds
  */
void ertc_alarm_config(void)
{
    ertc_alarm_type alarm_struct;
    ertc_time_type time_struct;
    
    /* Enable PWC and backup domain access */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_battery_powered_domain_access(TRUE);
    
    /* Enable ERTC */
    crm_ertc_clock_select(CRM_ERTC_CLOCK_LEXT);
    crm_ertc_clock_enable(TRUE);
    
    /* ... ERTC init and time set code ... */
    
    /* Get current time and set alarm 10 seconds later */
    ertc_time_get(&time_struct);
    
    /* Configure Alarm A */
    alarm_struct.alarm_mask = ERTC_ALARM_MASK_DATE_WEEK;  /* Ignore date */
    alarm_struct.week_day_sel = ERTC_SLECT_DATE;
    alarm_struct.alarm_x = ERTC_ALA;
    alarm_struct.hour = time_struct.hour;
    alarm_struct.min = time_struct.min;
    alarm_struct.sec = (time_struct.sec + 10) % 60;
    if(alarm_struct.sec < time_struct.sec) alarm_struct.min++;
    alarm_struct.ampm = time_struct.ampm;
    
    ertc_alarm_set(ERTC_ALA, &alarm_struct);
    ertc_alarm_enable(ERTC_ALA, TRUE);
    ertc_interrupt_enable(ERTC_ALA_INT, TRUE);
    
    /* Enable NVIC for ERTC Alarm */
    nvic_irq_enable(ERTC_IRQn, 0, 0);
}

/**
  * @brief  ERTC interrupt handler
  */
void ERTC_IRQHandler(void)
{
    if(ertc_interrupt_flag_get(ERTC_ALA_FLAG) != RESET)
    {
        alarm_triggered = SET;
        
        /* Clear ERTC flag */
        ertc_flag_clear(ERTC_ALA_FLAG);
        
        /* Clear EXINT Line 17 flag (REQUIRED for internal lines!) */
        exint_flag_clear(EXINT_LINE_17);
    }
}
```

---

## Configuration Checklist

### GPIO External Interrupt Setup

- [ ] Enable GPIO port clock (`CRM_GPIOx_PERIPH_CLOCK`)
- [ ] Enable SCFG clock (`CRM_SCFG_PERIPH_CLOCK`)
- [ ] Configure GPIO pin as input with appropriate pull
- [ ] Map GPIO to EXINT line (`scfg_exint_line_config()`)
- [ ] Initialize EXINT structure (`exint_default_para_init()`)
- [ ] Configure line mode (interrupt or event)
- [ ] Configure trigger polarity (rising, falling, both)
- [ ] Apply configuration (`exint_init()`)
- [ ] Clear any pending flags (`exint_flag_clear()`)
- [ ] Enable NVIC interrupt (for interrupt mode)
- [ ] Implement ISR with flag clearing

### Internal Peripheral Event Setup

- [ ] Configure source peripheral (ERTC, USB, etc.)
- [ ] Enable EXINT line in interrupt mode
- [ ] Configure trigger polarity
- [ ] Enable NVIC interrupt
- [ ] Clear both peripheral AND EXINT flags in ISR

---

## Troubleshooting

### Common Issues

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Interrupt not triggering | SCFG clock not enabled | Enable `CRM_SCFG_PERIPH_CLOCK` |
| | GPIO not mapped to EXINT | Call `scfg_exint_line_config()` |
| | NVIC not enabled | Call `nvic_irq_enable()` |
| | Wrong trigger polarity | Check signal edge vs. configuration |
| Continuous interrupts | Flag not cleared in ISR | Add `exint_flag_clear()` at end of ISR |
| | Hardware bounce | Add debounce logic or RC filter |
| Shared ISR missing events | Not checking all lines | Check all possible lines in shared handler |
| Wakeup not working | Using interrupt mode | Use event mode for WFE wakeup |
| | EXINT not configured before sleep | Configure before entering low-power |

### Debugging Tips

1. **Verify Configuration**
   ```c
   /* Read register values for debugging */
   uint32_t inten = EXINT->inten;   /* Check interrupt enable */
   uint32_t evten = EXINT->evten;   /* Check event enable */
   uint32_t pol1 = EXINT->polcfg1;  /* Check rising edge config */
   uint32_t pol2 = EXINT->polcfg2;  /* Check falling edge config */
   ```

2. **Check Pending Status**
   ```c
   if(exint_flag_get(EXINT_LINE_0) == SET)
   {
       /* Interrupt is pending */
   }
   ```

3. **Use `exint_interrupt_flag_get()` in ISR**
   This function checks both the pending flag AND that interrupts are enabled, avoiding false triggers.

4. **For Lines 5-9 and 10-15**
   Always check which specific line triggered the interrupt since they share handlers.

---

## Related Peripherals

| Peripheral | Relationship |
|------------|-------------|
| [GPIO](GPIO_General_Purpose_IO.md) | Source pins for EXINT lines 0-15 |
| [SCFG](SCFG_System_Configuration.md) | Multiplexes GPIO to EXINT lines |
| [NVIC](NVIC_Nested_Vectored_Interrupt.md) | Handles EXINT interrupt requests |
| [ERTC](ERTC_Enhanced_Real_Time_Clock.md) | Connected to lines 17, 21, 22 |
| [PWC](PWC_Power_Controller.md) | PVD connected to line 16, low-power wakeup |
| [USB](USB_Universal_Serial_Bus.md) | Wakeup events on lines 18, 20 |

---

## References

- AT32F435/437 Reference Manual - Chapter: EXINT
- AT32F435/437 Datasheet - Interrupt and event mapping
- Application Note AN0104 - External Interrupt Configuration

