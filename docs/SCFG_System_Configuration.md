---
title: SCFG - System Configuration
category: Peripheral
complexity: Basic
mcu: AT32F435/437
peripheral: SCFG
keywords: [scfg, system, configuration, memory, mapping, exint, emac, infrared, ultra-driven]
---

# SCFG - System Configuration

## Overview

The System Configuration (SCFG) peripheral provides system-level configuration options including memory mapping, external interrupt line selection, Ethernet MAC interface selection, infrared signal configuration, XMC address swap, and ultra-high drive capability for selected GPIO pins.

### Key Features

| Feature | Description |
|---------|-------------|
| Memory Mapping | Remap address 0x00000000 to different memory regions |
| EXINT Line Config | Select GPIO port source for each external interrupt line |
| EMAC Interface | Select MII or RMII mode for Ethernet MAC |
| Infrared Config | Configure infrared signal source and polarity |
| XMC Address Swap | Swap XMC peripheral address ranges |
| Ultra-Driven Pins | Enable extra high drive strength on selected pins |

## Architecture

```
                    ┌──────────────────────────────────────────────────────────┐
                    │                  SCFG Controller                         │
                    │                                                          │
                    │  ┌────────────────────────────────────────────────────┐  │
                    │  │              CFG1 Register                         │  │
                    │  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐  │  │
                    │  │  │ MEM_MAP_SEL │ │  IR_POL/SRC │ │  SWAP_XMC   │  │  │
                    │  │  │  [2:0]      │ │  [5:7]      │ │  [11:10]    │  │  │
                    │  │  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘  │  │
                    │  │         │               │               │         │  │
                    │  └─────────┼───────────────┼───────────────┼─────────┘  │
                    │            │               │               │            │
                    │            ▼               ▼               ▼            │
                    │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │
                    │  │ Memory Map  │  │  Infrared   │  │ XMC Address     │  │
                    │  │ Selection   │  │  Output     │  │ Remapping       │  │
                    │  └─────────────┘  └─────────────┘  └─────────────────┘  │
                    │                                                          │
                    │  ┌────────────────────────────────────────────────────┐  │
                    │  │              CFG2 Register                         │  │
                    │  │  ┌─────────────────────────────────────────────┐   │  │
                    │  │  │          MII_RMII_SEL [23]                  │   │  │
                    │  │  └──────────────────────┬──────────────────────┘   │  │
                    │  └─────────────────────────┼──────────────────────────┘  │
                    │                            │                             │
                    │                            ▼                             │
                    │  ┌───────────────────────────────────────────────────┐   │
                    │  │              EMAC Interface                        │   │
                    │  │         (MII / RMII Selection)                     │   │
                    │  └───────────────────────────────────────────────────┘   │
                    │                                                          │
                    │  ┌────────────────────────────────────────────────────┐  │
                    │  │         EXINTC1-4 Registers                       │  │
                    │  │  ┌──────────────────────────────────────────────┐ │  │
                    │  │  │ EXINT0-3 │ EXINT4-7 │ EXINT8-11 │ EXINT12-15 │ │  │
                    │  │  └────────────────────────────────────────────────┘ │  │
                    │  │         GPIO Port Selection for Each Line           │  │
                    │  └────────────────────────────────────────────────────┘  │
                    │                                                          │
                    │  ┌────────────────────────────────────────────────────┐  │
                    │  │              UHDRV Register                        │  │
                    │  │  ┌─────────────────────────────────────────────┐   │  │
                    │  │  │ PB3 │ PB9 │ PB10 │ PD12-15 │ PF14 │ PF15   │   │  │
                    │  │  └─────────────────────────────────────────────┘   │  │
                    │  │           Ultra-High Drive Enable                  │  │
                    │  └────────────────────────────────────────────────────┘  │
                    │                                                          │
                    └──────────────────────────────────────────────────────────┘
```

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| CFG1 | 0x00 | Configuration register 1 (memory map, IR, XMC swap) |
| CFG2 | 0x04 | Configuration register 2 (EMAC interface) |
| EXINTC1 | 0x08 | External interrupt configuration 1 (EXINT0-3) |
| EXINTC2 | 0x0C | External interrupt configuration 2 (EXINT4-7) |
| EXINTC3 | 0x10 | External interrupt configuration 3 (EXINT8-11) |
| EXINTC4 | 0x14 | External interrupt configuration 4 (EXINT12-15) |
| UHDRV | 0x2C | Ultra-high drive register |

### CFG1 Register (0x00)

| Bits | Name | Description |
|------|------|-------------|
| 2:0 | MEM_MAP_SEL | Memory mapping selection |
| 5 | IR_POL | Infrared output polarity |
| 7:6 | IR_SRC_SEL | Infrared source selection |
| 11:10 | SWAP_XMC | XMC address mapping swap |

### CFG2 Register (0x04)

| Bits | Name | Description |
|------|------|-------------|
| 23 | MII_RMII_SEL | EMAC interface: 0=MII, 1=RMII |

### EXINTCn Registers (0x08-0x14)

Each 4-bit field selects the GPIO port for the corresponding EXINT line:

| Value | Port |
|-------|------|
| 0x0 | GPIOA |
| 0x1 | GPIOB |
| 0x2 | GPIOC |
| 0x3 | GPIOD |
| 0x4 | GPIOE |
| 0x5 | GPIOF |
| 0x6 | GPIOG |
| 0x7 | GPIOH |

### UHDRV Register (0x2C)

| Bit | Name | Pin |
|-----|------|-----|
| 0 | PB3_UH | PB3 ultra-high drive |
| 1 | PB9_UH | PB9 ultra-high drive |
| 2 | PB10_UH | PB10 ultra-high drive |
| 5 | PD12_UH | PD12 ultra-high drive |
| 6 | PD13_UH | PD13 ultra-high drive |
| 7 | PD14_UH | PD14 ultra-high drive |
| 8 | PD15_UH | PD15 ultra-high drive |
| 9 | PF14_UH | PF14 ultra-high drive |
| 10 | PF15_UH | PF15 ultra-high drive |

## Configuration Types

### Memory Mapping Selection

| Enum | Value | Description |
|------|-------|-------------|
| `SCFG_MEM_MAP_MAIN_MEMORY` | 0x00 | Address 0x00000000 maps to main flash (0x08000000) |
| `SCFG_MEM_MAP_BOOT_MEMORY` | 0x01 | Address 0x00000000 maps to boot ROM (0x1FFF0000) |
| `SCFG_MEM_MAP_XMC_BANK1` | 0x02 | Address 0x00000000 maps to XMC bank 1 |
| `SCFG_MEM_MAP_INTERNAL_SRAM` | 0x03 | Address 0x00000000 maps to internal SRAM (0x20000000) |
| `SCFG_MEM_MAP_XMC_SDRAM_BANK1` | 0x04 | Address 0x00000000 maps to XMC SDRAM bank 1 |

### XMC Address Swap

| Enum | Value | Description |
|------|-------|-------------|
| `SCFG_XMC_SWAP_NONE` | 0x00 | No address swap |
| `SCFG_XMC_SWAP_MODE1` | 0x01 | SDRAM @ 0x60000000/0x70000000, NOR/PSRAM/SRAM/NAND2 @ 0xC0000000/0xD0000000 |
| `SCFG_XMC_SWAP_MODE2` | 0x02 | QSPI2 @ 0x80000000, NAND3 @ 0xB0000000 |
| `SCFG_XMC_SWAP_MODE3` | 0x03 | Combination of Mode1 and Mode2 |

### EMAC Interface

| Enum | Value | Description |
|------|-------|-------------|
| `SCFG_EMAC_SELECT_MII` | 0x00 | Media Independent Interface (MII) |
| `SCFG_EMAC_SELECT_RMII` | 0x01 | Reduced Media Independent Interface (RMII) |

### Infrared Source

| Enum | Value | Description |
|------|-------|-------------|
| `SCFG_IR_SOURCE_TMR10` | 0x00 | TMR10 as infrared signal source |

### Infrared Polarity

| Enum | Value | Description |
|------|-------|-------------|
| `SCFG_IR_POLARITY_NO_AFFECTE` | 0x00 | No polarity change |
| `SCFG_IR_POLARITY_REVERSE` | 0x01 | Reverse polarity |

### Port Source

| Enum | Value | Port |
|------|-------|------|
| `SCFG_PORT_SOURCE_GPIOA` | 0x00 | Port A |
| `SCFG_PORT_SOURCE_GPIOB` | 0x01 | Port B |
| `SCFG_PORT_SOURCE_GPIOC` | 0x02 | Port C |
| `SCFG_PORT_SOURCE_GPIOD` | 0x03 | Port D |
| `SCFG_PORT_SOURCE_GPIOE` | 0x04 | Port E |
| `SCFG_PORT_SOURCE_GPIOF` | 0x05 | Port F |
| `SCFG_PORT_SOURCE_GPIOG` | 0x06 | Port G |
| `SCFG_PORT_SOURCE_GPIOH` | 0x07 | Port H |

### Pin Source

| Enum | Value | Pin |
|------|-------|-----|
| `SCFG_PINS_SOURCE0` | 0x00 | Pin 0 |
| `SCFG_PINS_SOURCE1` | 0x01 | Pin 1 |
| ... | ... | ... |
| `SCFG_PINS_SOURCE15` | 0x0F | Pin 15 |

### Ultra-Driven Pins

| Enum | Pin | Description |
|------|-----|-------------|
| `SCFG_ULTRA_DRIVEN_PB3` | PB3 | SPI1_SCK, I2S1_CK |
| `SCFG_ULTRA_DRIVEN_PB9` | PB9 | SPI2_NSS, I2S2_WS |
| `SCFG_ULTRA_DRIVEN_PB10` | PB10 | SPI2_SCK, I2S2_CK |
| `SCFG_ULTRA_DRIVEN_PD12` | PD12 | XMC_A17, TMR4_CH1 |
| `SCFG_ULTRA_DRIVEN_PD13` | PD13 | XMC_A18, TMR4_CH2 |
| `SCFG_ULTRA_DRIVEN_PD14` | PD14 | XMC_D0 |
| `SCFG_ULTRA_DRIVEN_PD15` | PD15 | XMC_D1 |
| `SCFG_ULTRA_DRIVEN_PF14` | PF14 | XMC_A8 |
| `SCFG_ULTRA_DRIVEN_PF15` | PF15 | XMC_A9 |

## API Reference

### Reset

#### `scfg_reset`

```c
void scfg_reset(void);
```

Reset SCFG peripheral to default values.

---

### Memory Mapping

#### `scfg_mem_map_set`

```c
void scfg_mem_map_set(scfg_mem_map_type mem_map);
```

Configure the memory address mapping at 0x00000000.

| Parameter | Description |
|-----------|-------------|
| `mem_map` | Memory mapping selection |

**Memory Map Options:**

| Option | Result |
|--------|--------|
| `SCFG_MEM_MAP_MAIN_MEMORY` | 0x00000000 → 0x08000000 (Main Flash) |
| `SCFG_MEM_MAP_BOOT_MEMORY` | 0x00000000 → 0x1FFF0000 (System Boot ROM) |
| `SCFG_MEM_MAP_XMC_BANK1` | 0x00000000 → XMC Bank 1 |
| `SCFG_MEM_MAP_INTERNAL_SRAM` | 0x00000000 → 0x20000000 (SRAM) |
| `SCFG_MEM_MAP_XMC_SDRAM_BANK1` | 0x00000000 → XMC SDRAM Bank 1 |

---

### XMC Configuration

#### `scfg_xmc_mapping_swap_set`

```c
void scfg_xmc_mapping_swap_set(scfg_xmc_swap_type xmc_swap);
```

Configure XMC address mapping swap.

| Parameter | Description |
|-----------|-------------|
| `xmc_swap` | XMC swap mode selection |

---

### External Interrupt Configuration

#### `scfg_exint_line_config`

```c
void scfg_exint_line_config(scfg_port_source_type port_source, scfg_pins_source_type pin_source);
```

Select the GPIO port for an external interrupt line.

| Parameter | Description |
|-----------|-------------|
| `port_source` | GPIO port (A-H) |
| `pin_source` | Pin number (0-15) for EXINT line |

**Example:** To connect PA5 to EXINT line 5:
```c
scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE5);
```

---

### EMAC Configuration

#### `scfg_emac_interface_set`

```c
void scfg_emac_interface_set(scfg_emac_interface_type mode);
```

Select the Ethernet MAC interface mode.

| Parameter | Description |
|-----------|-------------|
| `mode` | `SCFG_EMAC_SELECT_MII` or `SCFG_EMAC_SELECT_RMII` |

**Note:** Must be called before enabling EMAC peripheral.

---

### Infrared Configuration

#### `scfg_infrared_config`

```c
void scfg_infrared_config(scfg_ir_source_type source, scfg_ir_polarity_type polarity);
```

Configure infrared signal source and polarity.

| Parameter | Description |
|-----------|-------------|
| `source` | Signal source (TMR10) |
| `polarity` | Output polarity |

---

### Ultra-Driven Pins

#### `scfg_pins_ultra_driven_enable`

```c
void scfg_pins_ultra_driven_enable(scfg_ultra_driven_pins_type value, confirm_state new_state);
```

Enable or disable ultra-high drive strength for a pin.

| Parameter | Description |
|-----------|-------------|
| `value` | Pin to configure |
| `new_state` | TRUE to enable, FALSE to disable |

---

## Code Examples

### Example 1: Memory Mapping to Boot ROM

Remap address 0x00000000 to boot memory for verification.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    /* Enable SCFG clock */
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    
    /* Remap 0x00000000 to boot memory (0x1FFF0000) */
    scfg_mem_map_set(SCFG_MEM_MAP_BOOT_MEMORY);
    
    /* Verify: read from 0x00000000 should match 0x1FFF0000 */
    if((*(__IO uint32_t *)0x00000000) == (*(__IO uint32_t *)0x1FFF0000))
    {
        /* Mapping successful */
        at32_led_on(LED2);
        at32_led_on(LED3);
        at32_led_on(LED4);
    }
    
    while(1)
    {
    }
}
```

---

### Example 2: Memory Mapping to SRAM (Execute from RAM)

Remap address 0x00000000 to internal SRAM for executing code from RAM.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/* Function to be executed from RAM */
__attribute__((section(".ram_code")))
void ram_function(void)
{
    at32_led_toggle(LED3);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    
    /* Remap 0x00000000 to internal SRAM */
    scfg_mem_map_set(SCFG_MEM_MAP_INTERNAL_SRAM);
    
    /* Verify mapping */
    if((*(__IO uint32_t *)0x00000000) == (*(__IO uint32_t *)0x20000000))
    {
        at32_led_on(LED2);
    }
    
    while(1)
    {
        ram_function();
        delay_ms(500);
    }
}
```

---

### Example 3: External Interrupt Line Configuration

Configure multiple GPIO ports for external interrupts.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void exint_config(void)
{
    exint_init_type exint_init_struct;
    
    /* Enable SCFG clock */
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    
    /* Configure GPIO pins as input */
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    
    /* PA0 for EXINT0 */
    gpio_init_struct.gpio_pins = GPIO_PINS_0;
    gpio_init(GPIOA, &gpio_init_struct);
    
    /* PB5 for EXINT5 */
    gpio_init_struct.gpio_pins = GPIO_PINS_5;
    gpio_init(GPIOB, &gpio_init_struct);
    
    /* PC13 for EXINT13 (user button) */
    gpio_init_struct.gpio_pins = GPIO_PINS_13;
    gpio_init(GPIOC, &gpio_init_struct);
    
    /* Connect GPIO to EXINT lines via SCFG */
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE0);
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, SCFG_PINS_SOURCE5);
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE13);
    
    /* Configure EXINT */
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
    exint_init_struct.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
    
    exint_init_struct.line_select = EXINT_LINE_0;
    exint_init(&exint_init_struct);
    
    exint_init_struct.line_select = EXINT_LINE_5;
    exint_init(&exint_init_struct);
    
    exint_init_struct.line_select = EXINT_LINE_13;
    exint_init(&exint_init_struct);
    
    /* Enable NVIC interrupts */
    nvic_irq_enable(EXINT0_IRQn, 1, 0);
    nvic_irq_enable(EXINT9_5_IRQn, 1, 0);
    nvic_irq_enable(EXINT15_10_IRQn, 1, 0);
}

void EXINT0_IRQHandler(void)
{
    if(exint_flag_get(EXINT_LINE_0) != RESET)
    {
        at32_led_toggle(LED2);
        exint_flag_clear(EXINT_LINE_0);
    }
}

void EXINT9_5_IRQHandler(void)
{
    if(exint_flag_get(EXINT_LINE_5) != RESET)
    {
        at32_led_toggle(LED3);
        exint_flag_clear(EXINT_LINE_5);
    }
}

void EXINT15_10_IRQHandler(void)
{
    if(exint_flag_get(EXINT_LINE_13) != RESET)
    {
        at32_led_toggle(LED4);
        exint_flag_clear(EXINT_LINE_13);
    }
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    exint_config();
    
    while(1)
    {
    }
}
```

---

### Example 4: EMAC Interface Selection

Configure Ethernet MAC for RMII mode.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void emac_gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    
    /* IMPORTANT: Set EMAC interface BEFORE enabling EMAC clock */
    scfg_emac_interface_set(SCFG_EMAC_SELECT_RMII);
    
    /* Now enable EMAC clock */
    crm_periph_clock_enable(CRM_EMAC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_EMACTX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_EMACRX_PERIPH_CLOCK, TRUE);
    
    /* Configure RMII pins */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    
    /* RMII_REF_CLK (PA1), RMII_MDIO (PA2), RMII_CRS_DV (PA7) */
    gpio_init_struct.gpio_pins = GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_7;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE1, GPIO_MUX_11);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_11);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_11);
    
    /* RMII_TX_EN (PB11), RMII_TXD0 (PB12), RMII_TXD1 (PB13) */
    gpio_init_struct.gpio_pins = GPIO_PINS_11 | GPIO_PINS_12 | GPIO_PINS_13;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE11, GPIO_MUX_11);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE12, GPIO_MUX_11);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE13, GPIO_MUX_11);
    
    /* RMII_MDC (PC1), RMII_RXD0 (PC4), RMII_RXD1 (PC5) */
    gpio_init_struct.gpio_pins = GPIO_PINS_1 | GPIO_PINS_4 | GPIO_PINS_5;
    gpio_init(GPIOC, &gpio_init_struct);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE1, GPIO_MUX_11);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE4, GPIO_MUX_11);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE5, GPIO_MUX_11);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    emac_gpio_config();
    
    /* Continue with EMAC initialization... */
    
    while(1)
    {
    }
}
```

---

### Example 5: Ultra-High Drive for SPI/I2S

Enable ultra-high drive strength for high-speed SPI/I2S signals.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void spi_ultra_drive_config(void)
{
    /* Enable SCFG clock */
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    
    /* Configure SPI2 pins */
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    
    /* PB9 (SPI2_NSS/I2S2_WS), PB10 (SPI2_SCK/I2S2_CK) */
    gpio_init_struct.gpio_pins = GPIO_PINS_9 | GPIO_PINS_10;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9, GPIO_MUX_5);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE10, GPIO_MUX_5);
    
    /* Enable ultra-high drive for SPI2 clock and NSS pins */
    scfg_pins_ultra_driven_enable(SCFG_ULTRA_DRIVEN_PB9, TRUE);
    scfg_pins_ultra_driven_enable(SCFG_ULTRA_DRIVEN_PB10, TRUE);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    spi_ultra_drive_config();
    
    /* Continue with SPI/I2S configuration... */
    
    while(1)
    {
    }
}
```

---

### Example 6: Infrared Signal Configuration

Configure infrared output using TMR10.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void infrared_config(void)
{
    /* Enable clocks */
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR10_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR11_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    
    /* Configure IR output pin (PB9) */
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9, GPIO_MUX_0);  /* IR_OUT */
    
    /* Configure infrared: TMR10 source, no polarity inversion */
    scfg_infrared_config(SCFG_IR_SOURCE_TMR10, SCFG_IR_POLARITY_NO_AFFECTE);
    
    /* Configure TMR10 for 38kHz carrier */
    tmr_base_init(TMR10, 3157, 0);  /* 120MHz / 3158 ≈ 38kHz */
    tmr_cnt_dir_set(TMR10, TMR_COUNT_UP);
    tmr_output_channel_config(TMR10, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_PWM_MODE_A);
    tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1, 1578);  /* 50% duty */
    tmr_output_channel_buffer_enable(TMR10, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_output_enable(TMR10, TRUE);
    
    /* Configure TMR11 for envelope modulation */
    tmr_base_init(TMR11, 60000, 1999);  /* Low frequency for envelope */
    tmr_cnt_dir_set(TMR11, TMR_COUNT_UP);
    tmr_output_channel_config(TMR11, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_PWM_MODE_A);
    tmr_channel_value_set(TMR11, TMR_SELECT_CHANNEL_1, 30000);  /* 50% */
    tmr_output_channel_buffer_enable(TMR11, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_output_enable(TMR11, TRUE);
    
    /* Enable timers */
    tmr_counter_enable(TMR10, TRUE);
    tmr_counter_enable(TMR11, TRUE);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    infrared_config();
    
    while(1)
    {
    }
}
```

---

### Example 7: XMC Address Swap for SDRAM

Configure XMC address swap for SDRAM at alternate address.

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void xmc_swap_config(void)
{
    /* Enable SCFG clock */
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    
    /* Swap XMC addresses:
     * SDRAM moves to 0x60000000 and 0x70000000
     * NOR/PSRAM/SRAM/NAND2 moves to 0xC0000000 and 0xD0000000
     */
    scfg_xmc_mapping_swap_set(SCFG_XMC_SWAP_MODE1);
    
    /* Now configure XMC SDRAM at the new address */
    /* ... XMC configuration code ... */
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    xmc_swap_config();
    
    /* Access SDRAM at 0x60000000 instead of default address */
    volatile uint32_t *sdram = (volatile uint32_t *)0x60000000;
    
    /* Test write/read */
    sdram[0] = 0x12345678;
    if(sdram[0] == 0x12345678)
    {
        at32_led_on(LED3);
    }
    
    while(1)
    {
    }
}
```

---

## Configuration Checklist

### Basic Setup

- [ ] Enable SCFG clock: `crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE)`
- [ ] Configure required features

### Memory Mapping

- [ ] Call `scfg_mem_map_set()` early in initialization if needed
- [ ] Verify mapping with test read

### EXINT Configuration

- [ ] Enable SCFG clock first
- [ ] Call `scfg_exint_line_config()` before EXINT initialization
- [ ] Each EXINT line can only be connected to one GPIO port at a time

### EMAC Configuration

- [ ] Call `scfg_emac_interface_set()` BEFORE enabling EMAC clocks
- [ ] Select MII or RMII based on PHY connection

### Ultra-Driven Pins

- [ ] Only specific pins support ultra-high drive
- [ ] Use for high-speed signals (SPI, I2S, XMC)

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| SCFG functions have no effect | SCFG clock not enabled | Enable `CRM_SCFG_PERIPH_CLOCK` |
| EXINT not triggering | Wrong port/pin configured | Verify `scfg_exint_line_config()` parameters |
| EMAC not working | Interface set after clock enable | Set EMAC interface before enabling EMAC clocks |
| Memory mapping fails | Invalid configuration | Check supported mapping options |
| Ultra-drive has no effect | Pin not supported | Only PB3, PB9, PB10, PD12-15, PF14-15 supported |

---

## Related Peripherals

- **[EXINT](EXINT_External_Interrupt.md)** - External interrupt controller
- **[GPIO](GPIO_General_Purpose_IO.md)** - GPIO configuration
- **[IRTMR](IRTMR_Infrared_Timer.md)** - Infrared timer
- **[CRM](CRM_Clock_Reset_Management.md)** - Clock management

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-01 | Initial release |

