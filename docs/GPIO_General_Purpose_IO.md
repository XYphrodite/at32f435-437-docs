---
title: GPIO - General Purpose Input/Output
category: Peripheral
complexity: Basic
mcu: AT32F435/437
peripheral: GPIO
keywords: [gpio, input, output, push-pull, open-drain, alternate function, mux]
---

# GPIO - General Purpose Input/Output

## Overview

The General Purpose Input/Output (GPIO) controller provides flexible digital I/O capabilities with configurable modes, output types, drive strength, and pull resistors. Each GPIO port contains 16 pins that can be independently configured as input, output, alternate function (mux), or analog. The AT32F435/437 features 8 GPIO ports (A-H) with atomic set/clear operations, write protection, and high-drive capability.

### Key Features

| Feature | Specification |
|---------|---------------|
| GPIO Ports | 8 ports: GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH |
| Pins per Port | 16 (GPIO_PINS_0 to GPIO_PINS_15) |
| Modes | Input, Output, Alternate Function (MUX), Analog |
| Output Types | Push-Pull, Open-Drain |
| Pull Resistors | None, Pull-Up, Pull-Down |
| Drive Strength | Stronger, Moderate |
| High Drive | HDRV register for extra current capability |
| Atomic Operations | SCR (set), CLR (clear) registers |
| Write Protection | Lock pin configuration until reset |
| Alternate Functions | 16 mux options (MUX0 - MUX15) per pin |

---

## Architecture

```
                          ┌───────────────────────────────────────────────────┐
                          │                  GPIO Port                        │
                          │                                                   │
   ┌─────────────────┐    │    ┌─────────────┐      ┌──────────────────┐      │
   │   AHB1 Bus      │────┼───►│   CFGR      │      │    Output        │      │
   │                 │    │    │   Mode      │      │    Driver        │      │
   └─────────────────┘    │    │   Config    │      │                  │      │
                          │    └──────┬──────┘      │  ┌────────────┐  │      │
                          │           │             │  │  Push-Pull │  │      │    ┌──────┐
                          │    ┌──────▼──────┐      │  │    or      │──┼──────┼───►│ Pin  │
                          │    │   PULL      │      │  │ Open-Drain │  │      │    └──────┘
                          │    │  Pull-up/   │──────┼─►│            │  │      │        │
                          │    │  Pull-down  │      │  └────────────┘  │      │        │
                          │    └─────────────┘      │                  │      │        │
                          │                         │  ┌────────────┐  │      │        │
                          │    ┌─────────────┐      │  │   ODRVR    │  │      │        │
                          │    │   OMODE     │──────┼─►│   Drive    │  │      │        │
                          │    │  Output     │      │  │  Strength  │  │      │        │
                          │    │   Type      │      │  └────────────┘  │      │        │
                          │    └─────────────┘      │                  │      │        │
                          │                         │  ┌────────────┐  │      │        │
                          │    ┌─────────────┐      │  │   HDRV     │  │      │        │
                          │    │   ODT       │──────┼─►│   Huge     │  │      │        │
                          │    │  Output     │      │  │   Drive    │  │      │        │
                          │    │   Data      │      │  └────────────┘  │      │        │
                          │    └─────────────┘      └──────────────────┘      │        │
                          │          ▲                                        │        │
                          │          │                                        │        │
                          │    ┌─────┴─────┐   ┌─────────────┐                │        │
                          │    │   SCR     │   │    IDT      │◄───────────────┼────────┘
                          │    │Set/Clear  │   │   Input     │                │
                          │    │  Register │   │   Data      │                │
                          │    └───────────┘   └─────────────┘                │
                          │                                                   │
                          │    ┌─────────────────────────────────┐            │
                          │    │    MUXL / MUXH                   │           │
                          │    │    Alternate Function Select     │───────────┼───► Peripheral
                          │    │    (MUX0 - MUX15)                │           │     (SPI, USART,
                          │    └─────────────────────────────────┘            │      TMR, etc.)
                          │                                                   │
                          │    ┌─────────────┐                                │
                          │    │    WPR      │                                │
                          │    │   Write     │                                │
                          │    │ Protection  │                                │
                          │    └─────────────┘                                │
                          └───────────────────────────────────────────────────┘
```

---

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| CFGR | 0x00 | Mode configuration (2 bits per pin) |
| OMODE | 0x04 | Output type (1 bit per pin) |
| ODRVR | 0x08 | Output drive strength (2 bits per pin) |
| PULL | 0x0C | Pull-up/pull-down (2 bits per pin) |
| IDT | 0x10 | Input data register (read-only) |
| ODT | 0x14 | Output data register |
| SCR | 0x18 | Set/Clear register (lower 16 bits set, upper 16 bits clear) |
| WPR | 0x1C | Write protection register |
| MUXL | 0x20 | Alternate function for pins 0-7 (4 bits per pin) |
| MUXH | 0x24 | Alternate function for pins 8-15 (4 bits per pin) |
| CLR | 0x28 | Clear register (write 1 to clear pin) |
| HDRV | 0x3C | Huge drive register (1 bit per pin) |

---

## Pin Definitions

```c
#define GPIO_PINS_0       0x0001  /* Pin 0 */
#define GPIO_PINS_1       0x0002  /* Pin 1 */
#define GPIO_PINS_2       0x0004  /* Pin 2 */
#define GPIO_PINS_3       0x0008  /* Pin 3 */
#define GPIO_PINS_4       0x0010  /* Pin 4 */
#define GPIO_PINS_5       0x0020  /* Pin 5 */
#define GPIO_PINS_6       0x0040  /* Pin 6 */
#define GPIO_PINS_7       0x0080  /* Pin 7 */
#define GPIO_PINS_8       0x0100  /* Pin 8 */
#define GPIO_PINS_9       0x0200  /* Pin 9 */
#define GPIO_PINS_10      0x0400  /* Pin 10 */
#define GPIO_PINS_11      0x0800  /* Pin 11 */
#define GPIO_PINS_12      0x1000  /* Pin 12 */
#define GPIO_PINS_13      0x2000  /* Pin 13 */
#define GPIO_PINS_14      0x4000  /* Pin 14 */
#define GPIO_PINS_15      0x8000  /* Pin 15 */
#define GPIO_PINS_ALL     0xFFFF  /* All pins */
```

---

## Configuration Types

### GPIO Mode

```c
typedef enum {
  GPIO_MODE_INPUT   = 0x00,  /* Input mode (reset state) */
  GPIO_MODE_OUTPUT  = 0x01,  /* General purpose output mode */
  GPIO_MODE_MUX     = 0x02,  /* Alternate function mode */
  GPIO_MODE_ANALOG  = 0x03   /* Analog mode (ADC/DAC) */
} gpio_mode_type;
```

### Output Type

```c
typedef enum {
  GPIO_OUTPUT_PUSH_PULL  = 0x00,  /* Push-pull output */
  GPIO_OUTPUT_OPEN_DRAIN = 0x01   /* Open-drain output */
} gpio_output_type;
```

### Pull Configuration

```c
typedef enum {
  GPIO_PULL_NONE = 0x00,  /* No pull (floating for input) */
  GPIO_PULL_UP   = 0x01,  /* Pull-up resistor enabled */
  GPIO_PULL_DOWN = 0x02   /* Pull-down resistor enabled */
} gpio_pull_type;
```

### Drive Strength

```c
typedef enum {
  GPIO_DRIVE_STRENGTH_STRONGER  = 0x01,  /* Stronger sourcing/sinking */
  GPIO_DRIVE_STRENGTH_MODERATE  = 0x02   /* Moderate sourcing/sinking */
} gpio_drive_type;
```

### Alternate Function (MUX)

```c
typedef enum {
  GPIO_MUX_0  = 0x00,  /* AF0 - System */
  GPIO_MUX_1  = 0x01,  /* AF1 - TMR1/TMR2 */
  GPIO_MUX_2  = 0x02,  /* AF2 - TMR3/TMR4/TMR5 */
  GPIO_MUX_3  = 0x03,  /* AF3 - TMR8/TMR9/TMR10/TMR11 */
  GPIO_MUX_4  = 0x04,  /* AF4 - I2C1/I2C2/I2C3 */
  GPIO_MUX_5  = 0x05,  /* AF5 - SPI1/SPI2/I2S2/I2S3 */
  GPIO_MUX_6  = 0x06,  /* AF6 - SPI2/SPI3/I2S2/I2S3 */
  GPIO_MUX_7  = 0x07,  /* AF7 - USART1/USART2/USART3 */
  GPIO_MUX_8  = 0x08,  /* AF8 - UART4/UART5/USART6 */
  GPIO_MUX_9  = 0x09,  /* AF9 - CAN1/CAN2/TMR12/TMR13/TMR14 */
  GPIO_MUX_10 = 0x0A,  /* AF10 - USB OTG FS/HS */
  GPIO_MUX_11 = 0x0B,  /* AF11 - EMAC */
  GPIO_MUX_12 = 0x0C,  /* AF12 - XMC/SDIO/USB OTG HS */
  GPIO_MUX_13 = 0x0D,  /* AF13 - DVP */
  GPIO_MUX_14 = 0x0E,  /* AF14 - LCD */
  GPIO_MUX_15 = 0x0F   /* AF15 - EVENTOUT */
} gpio_mux_sel_type;
```

### Pin Source (for MUX configuration)

```c
typedef enum {
  GPIO_PINS_SOURCE0  = 0x00,
  GPIO_PINS_SOURCE1  = 0x01,
  GPIO_PINS_SOURCE2  = 0x02,
  /* ... */
  GPIO_PINS_SOURCE15 = 0x0F
} gpio_pins_source_type;
```

---

## Initialization Structure

```c
typedef struct {
  uint32_t         gpio_pins;            /* Pin selection (GPIO_PINS_x) */
  gpio_output_type gpio_out_type;        /* Output type */
  gpio_pull_type   gpio_pull;            /* Pull configuration */
  gpio_mode_type   gpio_mode;            /* Mode selection */
  gpio_drive_type  gpio_drive_strength;  /* Drive strength */
} gpio_init_type;
```

---

## API Reference

### Initialization Functions

```c
/**
  * @brief  Reset GPIO port to default state
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @retval none
  */
void gpio_reset(gpio_type *gpio_x);

/**
  * @brief  Initialize GPIO pin(s) with specified configuration
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @param  gpio_init_struct: pointer to initialization structure
  * @retval none
  */
void gpio_init(gpio_type *gpio_x, gpio_init_type *gpio_init_struct);

/**
  * @brief  Fill gpio_init_type structure with default values
  * @param  gpio_init_struct: pointer to structure to initialize
  * @retval none
  * @note   Default: all pins, input mode, push-pull, no pull, stronger drive
  */
void gpio_default_para_init(gpio_init_type *gpio_init_struct);
```

### Input Functions

```c
/**
  * @brief  Read single pin input state
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @param  pins: GPIO pin (single GPIO_PINS_x value)
  * @retval flag_status: SET (high) or RESET (low)
  */
flag_status gpio_input_data_bit_read(gpio_type *gpio_x, uint16_t pins);

/**
  * @brief  Read entire port input data
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @retval uint16_t: 16-bit port input value
  */
uint16_t gpio_input_data_read(gpio_type *gpio_x);
```

### Output Functions

```c
/**
  * @brief  Read single pin output state
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @param  pins: GPIO pin (single GPIO_PINS_x value)
  * @retval flag_status: SET (high) or RESET (low)
  */
flag_status gpio_output_data_bit_read(gpio_type *gpio_x, uint16_t pins);

/**
  * @brief  Read entire port output data
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @retval uint16_t: 16-bit port output value
  */
uint16_t gpio_output_data_read(gpio_type *gpio_x);

/**
  * @brief  Set pin(s) to high level (atomic)
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @param  pins: GPIO pin(s) - can be OR'd combination
  * @retval none
  */
void gpio_bits_set(gpio_type *gpio_x, uint16_t pins);

/**
  * @brief  Reset pin(s) to low level (atomic)
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @param  pins: GPIO pin(s) - can be OR'd combination
  * @retval none
  */
void gpio_bits_reset(gpio_type *gpio_x, uint16_t pins);

/**
  * @brief  Write specified state to pin(s)
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @param  pins: GPIO pin(s) - can be OR'd combination
  * @param  bit_state: TRUE (high) or FALSE (low)
  * @retval none
  */
void gpio_bits_write(gpio_type *gpio_x, uint16_t pins, confirm_state bit_state);

/**
  * @brief  Write entire port output data
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @param  port_value: 16-bit value to write
  * @retval none
  */
void gpio_port_write(gpio_type *gpio_x, uint16_t port_value);
```

### Configuration Functions

```c
/**
  * @brief  Lock pin configuration (until reset)
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @param  pins: GPIO pin(s) to lock
  * @retval none
  * @note   Once locked, CFGR, OMODE, ODRVR, PULL, MUXL, MUXH cannot be changed
  */
void gpio_pin_wp_config(gpio_type *gpio_x, uint16_t pins);

/**
  * @brief  Enable/disable huge drive capability
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @param  pins: GPIO pin(s)
  * @param  new_state: TRUE (enable) or FALSE (disable)
  * @retval none
  */
void gpio_pins_huge_driven_config(gpio_type *gpio_x, uint16_t pins, confirm_state new_state);

/**
  * @brief  Configure pin alternate function
  * @param  gpio_x: GPIO port (GPIOA...GPIOH)
  * @param  gpio_pin_source: GPIO_PINS_SOURCEx (single pin)
  * @param  gpio_mux: GPIO_MUX_x alternate function
  * @retval none
  * @note   Pin must also be configured as GPIO_MODE_MUX
  */
void gpio_pin_mux_config(gpio_type *gpio_x, gpio_pins_source_type gpio_pin_source, 
                         gpio_mux_sel_type gpio_mux);
```

---

## Code Examples

### Example 1: Basic Output (LED Toggle)

Configure a pin as output and toggle it:

```c
#include "at32f435_437.h"

/**
  * @brief  Configure PC13 as push-pull output for LED
  */
void led_gpio_init(void)
{
    gpio_init_type gpio_init_struct;
    
    /* Enable GPIOC clock */
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    
    /* Initialize structure with defaults */
    gpio_default_para_init(&gpio_init_struct);
    
    /* Configure PC13 as output */
    gpio_init_struct.gpio_pins = GPIO_PINS_13;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOC, &gpio_init_struct);
}

/**
  * @brief  Toggle LED
  */
void led_toggle(void)
{
    GPIOC->odt ^= GPIO_PINS_13;  /* XOR to toggle */
}

int main(void)
{
    system_clock_config();
    led_gpio_init();
    
    while(1)
    {
        led_toggle();
        delay_ms(500);
    }
}
```

---

### Example 2: Fast I/O Toggle using SCR/CLR

Maximum speed GPIO toggling using atomic registers:

```c
#include "at32f435_437.h"

/**
  * @brief  Configure PA1 for high-speed toggle
  */
void fast_io_init(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_1;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
}

int main(void)
{
    system_clock_config();
    fast_io_init();
    
    while(1)
    {
        /* Using SCR/CLR for fastest possible toggle */
        /* Each operation is atomic and takes 1 clock cycle */
        
        GPIOA->scr = GPIO_PINS_1;  /* Set PA1 high */
        GPIOA->clr = GPIO_PINS_1;  /* Set PA1 low */
        
        GPIOA->scr = GPIO_PINS_1;
        GPIOA->clr = GPIO_PINS_1;
        
        GPIOA->scr = GPIO_PINS_1;
        GPIOA->clr = GPIO_PINS_1;
        
        /* This generates a high-frequency square wave */
    }
}
```

---

### Example 3: Button Input with Debounce

Configure a pin as input with pull-up for button reading:

```c
#include "at32f435_437.h"

#define BUTTON_PIN   GPIO_PINS_0
#define BUTTON_PORT  GPIOA
#define LED_PIN      GPIO_PINS_13
#define LED_PORT     GPIOC

/**
  * @brief  Configure button and LED GPIO
  */
void button_led_init(void)
{
    gpio_init_type gpio_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    
    /* Configure button as input with pull-up */
    gpio_init_struct.gpio_pins = BUTTON_PIN;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;  /* Button pulls to GND */
    gpio_init(BUTTON_PORT, &gpio_init_struct);
    
    /* Configure LED as output */
    gpio_init_struct.gpio_pins = LED_PIN;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(LED_PORT, &gpio_init_struct);
}

/**
  * @brief  Read button state with debounce
  */
confirm_state button_pressed(void)
{
    if(gpio_input_data_bit_read(BUTTON_PORT, BUTTON_PIN) == RESET)
    {
        delay_ms(20);  /* Debounce delay */
        if(gpio_input_data_bit_read(BUTTON_PORT, BUTTON_PIN) == RESET)
        {
            return TRUE;
        }
    }
    return FALSE;
}

int main(void)
{
    system_clock_config();
    button_led_init();
    
    while(1)
    {
        if(button_pressed())
        {
            gpio_bits_set(LED_PORT, LED_PIN);  /* LED on */
        }
        else
        {
            gpio_bits_reset(LED_PORT, LED_PIN);  /* LED off */
        }
    }
}
```

---

### Example 4: Alternate Function (USART TX)

Configure GPIO for peripheral alternate function:

```c
#include "at32f435_437.h"

/**
  * @brief  Configure PA9 as USART1 TX, PA10 as USART1 RX
  */
void usart1_gpio_init(void)
{
    gpio_init_type gpio_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    
    /* Configure PA9 (TX) as alternate function push-pull */
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;  /* Must be MUX mode */
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Set alternate function to USART1 (AF7) */
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_7);
    
    /* Configure PA10 (RX) as alternate function input */
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;  /* Pull-up for idle high */
    gpio_init(GPIOA, &gpio_init_struct);
    
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_7);
}
```

---

### Example 5: SWJ-DP Pin Remapping

Release debug pins (JTAG/SWD) for GPIO use:

```c
#include "at32f435_437.h"

/**
  * @brief  Configure PA13 (SWDIO) and PA14 (SWCLK) as GPIO
  * @note   After this, debugging will not work!
  */
void swj_disable_for_gpio(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    
    /* Configure PA13 (JTMS/SWDIO) as general output */
    gpio_init_struct.gpio_pins = GPIO_PINS_13;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Configure PA14 (JTCK/SWCLK) as USART2 TX (alternate function) */
    gpio_init_struct.gpio_pins = GPIO_PINS_14;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE14, GPIO_MUX_8);  /* USART2 */
}

/**
  * @brief  Toggle PA13 pin
  */
void gpio_toggle_pa13(void)
{
    GPIOA->odt ^= GPIO_PINS_13;
}
```

---

### Example 6: Open-Drain Output (I2C-like)

Configure open-drain output with external pull-up:

```c
#include "at32f435_437.h"

/**
  * @brief  Configure PB6/PB7 as open-drain for bit-bang I2C
  */
void bitbang_i2c_init(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    
    /* SCL and SDA as open-drain */
    gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;  /* PB6=SCL, PB7=SDA */
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;  /* Open-drain! */
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;  /* External pull-up required */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOB, &gpio_init_struct);
    
    /* Set both high (released) initially */
    gpio_bits_set(GPIOB, GPIO_PINS_6 | GPIO_PINS_7);
}

/**
  * @brief  Set SCL line (PB6)
  */
void i2c_scl_high(void) { gpio_bits_set(GPIOB, GPIO_PINS_6); }
void i2c_scl_low(void)  { gpio_bits_reset(GPIOB, GPIO_PINS_6); }

/**
  * @brief  Set SDA line (PB7)
  */
void i2c_sda_high(void) { gpio_bits_set(GPIOB, GPIO_PINS_7); }
void i2c_sda_low(void)  { gpio_bits_reset(GPIOB, GPIO_PINS_7); }

/**
  * @brief  Read SDA line state
  */
flag_status i2c_sda_read(void) { return gpio_input_data_bit_read(GPIOB, GPIO_PINS_7); }
```

---

### Example 7: Write Protection

Lock GPIO configuration to prevent accidental changes:

```c
#include "at32f435_437.h"

/**
  * @brief  Configure and lock safety-critical GPIO
  */
void safety_gpio_init(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    
    /* Configure PB0-PB3 for motor control */
    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_3;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;  /* Safe state = low */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOB, &gpio_init_struct);
    
    /* Set all low initially */
    gpio_bits_reset(GPIOB, GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_3);
    
    /* LOCK the configuration - cannot be changed until reset */
    gpio_pin_wp_config(GPIOB, GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_3);
    
    /* After this, any attempt to change mode/type/pull/drive will fail */
}
```

---

### Example 8: Analog Mode (ADC Input)

Configure GPIO for ADC input:

```c
#include "at32f435_437.h"

/**
  * @brief  Configure PA0 as analog input for ADC
  */
void adc_gpio_init(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    
    /* Configure PA0 as analog */
    gpio_init_struct.gpio_pins = GPIO_PINS_0;
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;  /* Analog mode */
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;    /* No pull for analog */
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* Pin is now disconnected from digital circuitry */
    /* Ready for ADC channel 0 sampling */
}
```

---

## Alternate Function Mapping (Common)

| MUX | Function |
|-----|----------|
| GPIO_MUX_0 | System (MCO, JTAG, TRACE, etc.) |
| GPIO_MUX_1 | TMR1, TMR2 |
| GPIO_MUX_2 | TMR3, TMR4, TMR5 |
| GPIO_MUX_3 | TMR8, TMR9, TMR10, TMR11 |
| GPIO_MUX_4 | I2C1, I2C2, I2C3 |
| GPIO_MUX_5 | SPI1, SPI2, I2S2, I2S3 |
| GPIO_MUX_6 | SPI2, SPI3, I2S2, I2S3 |
| GPIO_MUX_7 | USART1, USART2, USART3 |
| GPIO_MUX_8 | UART4, UART5, USART6, UART7, UART8 |
| GPIO_MUX_9 | CAN1, CAN2, TMR12, TMR13, TMR14 |
| GPIO_MUX_10 | USB OTG FS/HS |
| GPIO_MUX_11 | EMAC |
| GPIO_MUX_12 | XMC, SDIO, USB OTG HS |
| GPIO_MUX_13 | DVP |
| GPIO_MUX_14 | LCD |
| GPIO_MUX_15 | EVENTOUT |

> **Note:** Refer to the datasheet for complete pin-specific alternate function tables.

---

## Configuration Checklist

### Output Configuration
- [ ] Enable GPIO port clock (`CRM_GPIOx_PERIPH_CLOCK`)
- [ ] Initialize `gpio_init_type` structure
- [ ] Set `gpio_mode` to `GPIO_MODE_OUTPUT`
- [ ] Choose `gpio_out_type` (push-pull or open-drain)
- [ ] Select `gpio_pull` if needed
- [ ] Set `gpio_drive_strength`
- [ ] Call `gpio_init()`

### Input Configuration
- [ ] Enable GPIO port clock
- [ ] Set `gpio_mode` to `GPIO_MODE_INPUT`
- [ ] Configure `gpio_pull` (none, up, or down)
- [ ] Call `gpio_init()`

### Alternate Function Configuration
- [ ] Enable GPIO port clock
- [ ] Enable peripheral clock
- [ ] Set `gpio_mode` to `GPIO_MODE_MUX`
- [ ] Configure output type, pull, drive as needed
- [ ] Call `gpio_init()`
- [ ] Call `gpio_pin_mux_config()` with correct AF

### Analog Configuration
- [ ] Enable GPIO port clock
- [ ] Set `gpio_mode` to `GPIO_MODE_ANALOG`
- [ ] Set `gpio_pull` to `GPIO_PULL_NONE`
- [ ] Call `gpio_init()`

---

## Troubleshooting

### Common Issues

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Pin not responding | Clock not enabled | Enable `CRM_GPIOx_PERIPH_CLOCK` |
| | Pin locked | Check WPR register, reset MCU |
| Wrong alternate function | Incorrect MUX value | Check datasheet for correct AF |
| | Mode not set to MUX | Set `gpio_mode = GPIO_MODE_MUX` |
| Output weak | Drive strength too low | Use `GPIO_DRIVE_STRENGTH_STRONGER` |
| | Need more current | Enable HDRV with `gpio_pins_huge_driven_config()` |
| Input floating | No pull configured | Add pull-up or pull-down |
| ADC not working | Pin not in analog mode | Set `gpio_mode = GPIO_MODE_ANALOG` |
| Open-drain not working | External pull-up missing | Add external pull-up resistor |

### Direct Register Access

For maximum performance, use direct register access:

```c
/* Set pin(s) high - atomic, 1 cycle */
GPIOx->scr = GPIO_PINS_x;

/* Set pin(s) low - atomic, 1 cycle */
GPIOx->clr = GPIO_PINS_x;

/* Toggle pin(s) - read-modify-write */
GPIOx->odt ^= GPIO_PINS_x;

/* Read input */
uint16_t value = GPIOx->idt;

/* Read output */
uint16_t value = GPIOx->odt;
```

---

## Related Peripherals

| Peripheral | Relationship |
|------------|-------------|
| [CRM](CRM_Clock_Reset_Management.md) | GPIO port clock enable |
| [EXINT](EXINT_External_Interrupt.md) | External interrupt from GPIO pins |
| [SCFG](SCFG_System_Configuration.md) | EXINT line to GPIO mapping |
| [ADC](ADC_Analog_to_Digital_Converter.md) | Analog input through GPIO |
| [DAC](DAC_Digital_to_Analog_Converter.md) | Analog output through GPIO |
| All Peripherals | Alternate function mapping |

---

## References

- AT32F435/437 Reference Manual - Chapter: GPIO
- AT32F435/437 Datasheet - Pin definitions and alternate functions
- Application Note AN0095 - GPIO Configuration Guide

