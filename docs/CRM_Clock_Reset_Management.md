---
title: CRM (Clock and Reset Management)
mcu: AT32F435/437
peripheral: CRM
version: 2.0.9
---

# CRM Clock and Reset Management

## Overview

The CRM (Clock and Reset Management) peripheral controls all system clocks, peripheral clocks, and reset signals for the AT32F435/437 microcontrollers. It manages clock sources, PLL configuration, clock distribution, and provides clock monitoring through the Clock Failure Detection (CFD) feature.

## Clock Tree Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              AT32F435/437 Clock Tree                                 │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│  ┌─────────────┐                                                                     │
│  │    HICK     │──┬─────────────────────────────────────────────────┐               │
│  │   (8 MHz)   │  │                                                 │               │
│  └─────────────┘  │  ┌──────────────────────────────────────────┐   │               │
│                   ├──┤           PLL Configuration               │   │               │
│  ┌─────────────┐  │  │  ┌───────┐  ┌───────┐  ┌───────┐        │   │               │
│  │    HEXT     │──┘  │  │ ÷MS   │──│ ×NS   │──│ ÷FR   │───────────────┐            │
│  │ (8-25 MHz)  │     │  │(1-15) │  │(31-500)│ │(1-32) │        │   │   │            │
│  └─────────────┘     │  └───────┘  └───────┘  └───────┘        │   │   │            │
│                      │  PLL_CLK = (Source × NS) / (MS × FR)    │   │   │            │
│                      └──────────────────────────────────────────┘   │   │            │
│                                                                     │   │            │
│                                                                     │   │            │
│                      ┌──────────────────────────────────────────────┘   │            │
│                      │                                                   │            │
│                      ▼                                                   ▼            │
│                 ┌─────────┐   SCLK Switch   ┌─────────────────────────────┐          │
│                 │  HICK   │◄───────────────►│          SCLK               │          │
│                 │  HEXT   │    (MUX)        │   (System Clock)            │          │
│                 │  PLL    │                 │   Max: 288 MHz              │          │
│                 └─────────┘                 └──────────┬──────────────────┘          │
│                                                        │                             │
│                                                        ▼                             │
│                                              ┌─────────────────┐                     │
│                                              │     AHB Bus     │                     │
│                                              │  ÷1,2,4,8,16,   │                     │
│                                              │   64,128,256,512│                     │
│                                              │  Max: 288 MHz   │                     │
│                                              └────────┬────────┘                     │
│                                                       │                              │
│                      ┌────────────────────────────────┼────────────────────────────┐ │
│                      │                                │                            │ │
│                      ▼                                ▼                            ▼ │
│               ┌─────────────┐               ┌─────────────┐               ┌────────┐│
│               │  APB1 Bus   │               │  APB2 Bus   │               │Periph  ││
│               │  ÷1,2,4,8,16│               │  ÷1,2,4,8,16│               │Clocks  ││
│               │ Max: 144 MHz│               │ Max: 144 MHz│               │        ││
│               └──────┬──────┘               └──────┬──────┘               └────────┘│
│                      │                             │                                 │
│                      ▼                             ▼                                 │
│  ┌────────────────────────────────┐ ┌────────────────────────────────┐              │
│  │ TMR2-7,12-14, USART2-3,        │ │ TMR1,8,9-11,20, USART1,6,      │              │
│  │ UART4-5,7-8, SPI2-3, I2C1-3,   │ │ SPI1,4, ADC1-3, ACC,           │              │
│  │ CAN1-2, DAC, PWC               │ │ SCFG, TMR20                     │              │
│  └────────────────────────────────┘ └────────────────────────────────┘              │
│                                                                                      │
│  ┌─────────────┐                    ┌─────────────┐                                 │
│  │    LEXT     │────────────────────│    ERTC     │                                 │
│  │ (32.768 kHz)│                    │   Clock     │                                 │
│  └─────────────┘                    └─────────────┘                                 │
│                                                                                      │
│  ┌─────────────┐                    ┌─────────────┐                                 │
│  │    LICK     │────────────────────│    WDT      │                                 │
│  │  (~40 kHz)  │                    │   Clock     │                                 │
│  └─────────────┘                    └─────────────┘                                 │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## Key Features

| Feature | Specification |
|---------|---------------|
| Max System Clock (SCLK) | 288 MHz |
| Max AHB Clock | 288 MHz |
| Max APB1/APB2 Clock | 144 MHz |
| Clock Sources | HICK (8 MHz), HEXT (8-25 MHz), PLL, LEXT (32.768 kHz), LICK (~40 kHz) |
| PLL Multiplier (NS) | 31-500 |
| PLL Dividers | MS (1-15), FR (1, 2, 4, 8, 16, 32) |
| Clock Outputs | CLKOUT1 (PA8), CLKOUT2 (PC9) |
| Clock Failure Detection | Yes (CFD for HEXT) |

---

## Clock Sources

### HICK (High-speed Internal Clock)
- Frequency: 8 MHz (can output 48 MHz for USB)
- Startup time: Fast
- Accuracy: ±1% (typical)
- Use case: Default clock, low-power applications

### HEXT (High-speed External Crystal)
- Frequency: 8-25 MHz
- Startup time: 1-5 ms (crystal dependent)
- Accuracy: High (crystal dependent)
- Use case: Precision timing, USB, high-speed operation

### PLL (Phase-Locked Loop)
- Input: HICK or HEXT
- Output: Up to 288 MHz
- Formula: `PLL_CLK = (Source × NS) / (MS × FR)`

### LEXT (Low-speed External Crystal)
- Frequency: 32.768 kHz
- Use case: RTC, low-power timing

### LICK (Low-speed Internal Clock)
- Frequency: ~40 kHz
- Use case: Watchdog timer, backup clock

---

## PLL Configuration

### PLL Formula

```
PLL_OUT = (PLL_SOURCE × PLL_NS) / (PLL_MS × PLL_FR)
```

Where:
- **PLL_SOURCE**: HICK (8 MHz) or HEXT (8-25 MHz)
- **PLL_NS**: Multiplier (31-500)
- **PLL_MS**: Pre-divider (1-15)
- **PLL_FR**: Post-divider (1, 2, 4, 8, 16, 32)

### PLL FR Values

| Enum | Value | Division |
|------|-------|----------|
| `CRM_PLL_FR_1` | 0x00 | ÷1 |
| `CRM_PLL_FR_2` | 0x01 | ÷2 |
| `CRM_PLL_FR_4` | 0x02 | ÷4 |
| `CRM_PLL_FR_8` | 0x03 | ÷8 |
| `CRM_PLL_FR_16` | 0x04 | ÷16 |
| `CRM_PLL_FR_32` | 0x05 | ÷32 |

### Common PLL Configurations

| Target SCLK | Source | NS | MS | FR | Calculation |
|-------------|--------|----|----|----|-----------------------------|
| 288 MHz | HEXT 8 MHz | 144 | 1 | 4 | (8 × 144) / (1 × 4) = 288 |
| 288 MHz | HICK 8 MHz | 144 | 1 | 4 | (8 × 144) / (1 × 4) = 288 |
| 200 MHz | HEXT 8 MHz | 100 | 1 | 4 | (8 × 100) / (1 × 4) = 200 |
| 144 MHz | HEXT 8 MHz | 72 | 1 | 4 | (8 × 72) / (1 × 4) = 144 |
| 96 MHz | HEXT 8 MHz | 96 | 1 | 8 | (8 × 96) / (1 × 8) = 96 |
| 64 MHz | HICK 8 MHz | 64 | 1 | 8 | (8 × 64) / (1 × 8) = 64 |

---

## Bus Dividers

### AHB Divider

| Enum | Value | Division |
|------|-------|----------|
| `CRM_AHB_DIV_1` | 0x00 | ÷1 |
| `CRM_AHB_DIV_2` | 0x08 | ÷2 |
| `CRM_AHB_DIV_4` | 0x09 | ÷4 |
| `CRM_AHB_DIV_8` | 0x0A | ÷8 |
| `CRM_AHB_DIV_16` | 0x0B | ÷16 |
| `CRM_AHB_DIV_64` | 0x0C | ÷64 |
| `CRM_AHB_DIV_128` | 0x0D | ÷128 |
| `CRM_AHB_DIV_256` | 0x0E | ÷256 |
| `CRM_AHB_DIV_512` | 0x0F | ÷512 |

### APB1/APB2 Divider

| Enum | Value | Division |
|------|-------|----------|
| `CRM_APB1_DIV_1` / `CRM_APB2_DIV_1` | 0x00 | ÷1 |
| `CRM_APB1_DIV_2` / `CRM_APB2_DIV_2` | 0x04 | ÷2 |
| `CRM_APB1_DIV_4` / `CRM_APB2_DIV_4` | 0x05 | ÷4 |
| `CRM_APB1_DIV_8` / `CRM_APB2_DIV_8` | 0x06 | ÷8 |
| `CRM_APB1_DIV_16` / `CRM_APB2_DIV_16` | 0x07 | ÷16 |

---

## Peripheral Clock Enable

### AHB1 Peripherals

```c
CRM_GPIOA_PERIPH_CLOCK    CRM_GPIOB_PERIPH_CLOCK    CRM_GPIOC_PERIPH_CLOCK
CRM_GPIOD_PERIPH_CLOCK    CRM_GPIOE_PERIPH_CLOCK    CRM_GPIOF_PERIPH_CLOCK
CRM_GPIOG_PERIPH_CLOCK    CRM_GPIOH_PERIPH_CLOCK    CRM_CRC_PERIPH_CLOCK
CRM_EDMA_PERIPH_CLOCK     CRM_DMA1_PERIPH_CLOCK     CRM_DMA2_PERIPH_CLOCK
CRM_OTGFS2_PERIPH_CLOCK   CRM_EMAC_PERIPH_CLOCK (437 only)
```

### AHB2 Peripherals

```c
CRM_DVP_PERIPH_CLOCK      CRM_OTGFS1_PERIPH_CLOCK   CRM_SDIO1_PERIPH_CLOCK
```

### AHB3 Peripherals

```c
CRM_XMC_PERIPH_CLOCK      CRM_QSPI1_PERIPH_CLOCK    CRM_QSPI2_PERIPH_CLOCK
CRM_SDIO2_PERIPH_CLOCK
```

### APB1 Peripherals

```c
CRM_TMR2_PERIPH_CLOCK     CRM_TMR3_PERIPH_CLOCK     CRM_TMR4_PERIPH_CLOCK
CRM_TMR5_PERIPH_CLOCK     CRM_TMR6_PERIPH_CLOCK     CRM_TMR7_PERIPH_CLOCK
CRM_TMR12_PERIPH_CLOCK    CRM_TMR13_PERIPH_CLOCK    CRM_TMR14_PERIPH_CLOCK
CRM_WWDT_PERIPH_CLOCK     CRM_SPI2_PERIPH_CLOCK     CRM_SPI3_PERIPH_CLOCK
CRM_USART2_PERIPH_CLOCK   CRM_USART3_PERIPH_CLOCK   CRM_UART4_PERIPH_CLOCK
CRM_UART5_PERIPH_CLOCK    CRM_I2C1_PERIPH_CLOCK     CRM_I2C2_PERIPH_CLOCK
CRM_I2C3_PERIPH_CLOCK     CRM_CAN1_PERIPH_CLOCK     CRM_CAN2_PERIPH_CLOCK
CRM_PWC_PERIPH_CLOCK      CRM_DAC_PERIPH_CLOCK      CRM_UART7_PERIPH_CLOCK
CRM_UART8_PERIPH_CLOCK
```

### APB2 Peripherals

```c
CRM_TMR1_PERIPH_CLOCK     CRM_TMR8_PERIPH_CLOCK     CRM_USART1_PERIPH_CLOCK
CRM_USART6_PERIPH_CLOCK   CRM_ADC1_PERIPH_CLOCK     CRM_ADC2_PERIPH_CLOCK
CRM_ADC3_PERIPH_CLOCK     CRM_SPI1_PERIPH_CLOCK     CRM_SPI4_PERIPH_CLOCK
CRM_SCFG_PERIPH_CLOCK     CRM_TMR9_PERIPH_CLOCK     CRM_TMR10_PERIPH_CLOCK
CRM_TMR11_PERIPH_CLOCK    CRM_TMR20_PERIPH_CLOCK    CRM_ACC_PERIPH_CLOCK
```

---

## API Reference

### Clock Source Control

| Function | Description |
|----------|-------------|
| `crm_clock_source_enable(source, state)` | Enable/disable clock source |
| `crm_hext_stable_wait()` | Wait for HEXT stabilization |
| `crm_flag_get(flag)` | Get clock status flag |

### PLL Configuration

| Function | Description |
|----------|-------------|
| `crm_pll_config(source, ns, ms, fr)` | Configure PLL parameters |
| `crm_pll_parameter_calculate(source, target_freq, *ms, *ns, *fr)` | Auto-calculate PLL parameters |

### System Clock

| Function | Description |
|----------|-------------|
| `crm_sysclk_switch(source)` | Switch system clock source |
| `crm_sysclk_switch_status_get()` | Get current SCLK source |
| `crm_clocks_freq_get(*clocks_struct)` | Get all clock frequencies |

### Bus Dividers

| Function | Description |
|----------|-------------|
| `crm_ahb_div_set(div)` | Set AHB bus divider |
| `crm_apb1_div_set(div)` | Set APB1 bus divider |
| `crm_apb2_div_set(div)` | Set APB2 bus divider |

### Peripheral Clock Control

| Function | Description |
|----------|-------------|
| `crm_periph_clock_enable(periph, state)` | Enable/disable peripheral clock |
| `crm_periph_reset(periph, state)` | Reset peripheral |
| `crm_periph_lowpower_mode_enable(periph, state)` | Control peripheral clock in sleep mode |

### Clock Output

| Function | Description |
|----------|-------------|
| `crm_clock_out1_set(source)` | Configure CLKOUT1 source |
| `crm_clock_out2_set(source)` | Configure CLKOUT2 source |
| `crm_clkout_div_set(index, div1, div2)` | Set clock output dividers |

### Clock Failure Detection

| Function | Description |
|----------|-------------|
| `crm_clock_failure_detection_enable(state)` | Enable/disable CFD |
| `crm_interrupt_enable(int_type, state)` | Enable clock interrupts |

### Miscellaneous

| Function | Description |
|----------|-------------|
| `crm_reset()` | Reset CRM to default values |
| `crm_auto_step_mode_enable(state)` | Enable auto-step for clock switching |
| `crm_usb_clock_div_set(div)` | Set USB clock divider |
| `crm_usb_clock_source_select(source)` | Select USB clock source |
| `crm_hick_sclk_frequency_select(freq)` | Select HICK frequency (8/48 MHz) |

---

## Complete Examples

### Example 1: System Clock Switch (HICK ↔ HEXT)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * System Clock Switch Example
 * 
 * Demonstrates switching between HICK-based and HEXT-based PLL configurations.
 * Press user button to toggle between 64 MHz (HICK) and 96 MHz (HEXT).
 ******************************************************************************/

/* Configure PA8 as CLKOUT1 to observe system clock */
static void clkout_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_8;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* Output PLL/4 on CLKOUT1 */
  crm_clkout_div_set(CRM_CLKOUT_INDEX_1, CRM_CLKOUT_DIV1_4, CRM_CLKOUT_DIV2_1);
  crm_clock_out1_set(CRM_CLKOUT1_PLL);
}

/**
 * Configure 64 MHz system clock using HICK as PLL source
 * SCLK = (8 MHz × 64) / (1 × 8) = 64 MHz
 */
static void sclk_64m_hick_config(void)
{
  /* Reset CRM to default state */
  crm_reset();

  /* Enable power controller clock */
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

  /* Configure LDO voltage for target frequency */
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);

  /* Set flash divider for safe operation during clock change */
  flash_clock_divider_set(FLASH_CLOCK_DIV_3);

  /* Enable HICK */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET);

  /* Configure PLL: 8 MHz × 64 / (1 × 8) = 64 MHz */
  crm_pll_config(CRM_PLL_SOURCE_HICK, 64, 1, CRM_PLL_FR_8);

  /* Enable PLL */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

  /* Configure bus dividers */
  crm_ahb_div_set(CRM_AHB_DIV_1);    /* AHB = 64 MHz */
  crm_apb2_div_set(CRM_APB2_DIV_2); /* APB2 = 32 MHz */
  crm_apb1_div_set(CRM_APB1_DIV_2); /* APB1 = 32 MHz */

  /* Enable auto-step mode for safe switching */
  crm_auto_step_mode_enable(TRUE);

  /* Switch to PLL */
  crm_sysclk_switch(CRM_SCLK_PLL);
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL);

  /* Disable auto-step mode */
  crm_auto_step_mode_enable(FALSE);

  /* Update global clock variable */
  system_core_clock_update();

  /* Optimize flash divider for final frequency */
  flash_clock_divider_set(FLASH_CLOCK_DIV_2);

  /* Re-initialize delay functions */
  delay_init();

  /* Configure clock output */
  clkout_config();
}

/**
 * Configure 96 MHz system clock using HEXT as PLL source
 * SCLK = (8 MHz × 96) / (1 × 8) = 96 MHz
 */
static void sclk_96m_hext_config(void)
{
  crm_reset();

  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
  flash_clock_divider_set(FLASH_CLOCK_DIV_3);

  /* Enable HEXT */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
  while(crm_hext_stable_wait() == ERROR);

  /* Configure PLL: 8 MHz × 96 / (1 × 8) = 96 MHz */
  crm_pll_config(CRM_PLL_SOURCE_HEXT, 96, 1, CRM_PLL_FR_8);

  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

  crm_ahb_div_set(CRM_AHB_DIV_1);
  crm_apb2_div_set(CRM_APB2_DIV_2);
  crm_apb1_div_set(CRM_APB1_DIV_2);

  crm_auto_step_mode_enable(TRUE);
  crm_sysclk_switch(CRM_SCLK_PLL);
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL);
  crm_auto_step_mode_enable(FALSE);

  system_core_clock_update();
  flash_clock_divider_set(FLASH_CLOCK_DIV_2);
  delay_init();
  clkout_config();
}

/* Toggle between clock configurations */
static void switch_system_clock(void)
{
  if(CRM->pllcfg_bit.pllrcs == RESET)  /* Currently using HICK */
  {
    sclk_96m_hext_config();  /* Switch to HEXT */
  }
  else  /* Currently using HEXT */
  {
    sclk_64m_hick_config();  /* Switch to HICK */
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  clkout_config();

  while(1)
  {
    if(at32_button_press() == USER_BUTTON)
    {
      switch_system_clock();
      at32_led_toggle(LED4);  /* Indicate clock switch */
    }
    at32_led_toggle(LED2);
    delay_ms(100);
  }
}
```

---

### Example 2: PLL Parameter Auto-Calculate

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * PLL Parameter Auto-Calculate Example
 * 
 * Uses crm_pll_parameter_calculate() to automatically determine
 * optimal PLL parameters for a target frequency.
 ******************************************************************************/

error_status calculate_status = ERROR;

static void clkout_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_8;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  crm_clkout_div_set(CRM_CLKOUT_INDEX_1, CRM_CLKOUT_DIV1_4, CRM_CLKOUT_DIV2_1);
  crm_clock_out1_set(CRM_CLKOUT1_PLL);
}

/**
 * Configure 200 MHz system clock with auto-calculated PLL parameters
 */
void system_clock_config_200mhz(void)
{
  uint16_t pll_ns, pll_ms, pll_fr;

  crm_reset();

  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
  flash_clock_divider_set(FLASH_CLOCK_DIV_3);

  /* Enable HEXT */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
  while(crm_hext_stable_wait() == ERROR);

  /*
   * Auto-calculate PLL parameters for 200 MHz
   * The function returns optimal NS, MS, FR values
   */
  calculate_status = crm_pll_parameter_calculate(
    CRM_PLL_SOURCE_HEXT,  /* PLL source */
    200000000,            /* Target frequency: 200 MHz */
    &pll_ms,              /* Output: MS divider */
    &pll_ns,              /* Output: NS multiplier */
    &pll_fr               /* Output: FR divider */
  );

  /* Configure PLL with calculated parameters */
  crm_pll_config(CRM_PLL_SOURCE_HEXT, pll_ns, pll_ms, (crm_pll_fr_type)pll_fr);

  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

  crm_ahb_div_set(CRM_AHB_DIV_1);
  crm_apb2_div_set(CRM_APB2_DIV_2);  /* APB2 = 100 MHz */
  crm_apb1_div_set(CRM_APB1_DIV_2);  /* APB1 = 100 MHz */

  crm_auto_step_mode_enable(TRUE);
  crm_sysclk_switch(CRM_SCLK_PLL);
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL);
  crm_auto_step_mode_enable(FALSE);

  system_core_clock_update();
}

int main(void)
{
  system_clock_config_200mhz();
  at32_board_init();
  clkout_config();

  while(1)
  {
    if(calculate_status == ERROR)
    {
      at32_led_toggle(LED2);  /* Error - calculation failed */
    }
    else
    {
      at32_led_toggle(LED4);  /* Success - 200 MHz achieved */
    }
    delay_ms(500);
  }
}
```

---

### Example 3: Clock Failure Detection (CFD)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * Clock Failure Detection Example
 * 
 * Demonstrates automatic failover from HEXT to HICK when external
 * crystal failure is detected. The CFD monitors HEXT and triggers
 * an NMI interrupt on failure.
 ******************************************************************************/

static void clkout_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_8;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  crm_clkout_div_set(CRM_CLKOUT_INDEX_1, CRM_CLKOUT_DIV1_4, CRM_CLKOUT_DIV2_1);
  crm_clock_out1_set(CRM_CLKOUT1_PLL);
}

/**
 * Fallback clock configuration: 288 MHz from HICK
 * Called when HEXT failure is detected
 */
static void sclk_288m_hick_config(void)
{
  crm_reset();

  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
  flash_clock_divider_set(FLASH_CLOCK_DIV_3);

  /* Enable HICK as backup */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET);

  /* Configure PLL for 288 MHz: 8 × 144 / 4 = 288 MHz */
  crm_pll_config(CRM_PLL_SOURCE_HICK, 144, 1, CRM_PLL_FR_4);

  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

  crm_ahb_div_set(CRM_AHB_DIV_1);
  crm_apb2_div_set(CRM_APB2_DIV_2);
  crm_apb1_div_set(CRM_APB1_DIV_2);

  crm_auto_step_mode_enable(TRUE);
  crm_sysclk_switch(CRM_SCLK_PLL);
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL);
  crm_auto_step_mode_enable(FALSE);

  system_core_clock_update();
  delay_init();
  clkout_config();
}

/**
 * Clock Failure Detection Handler
 * Called from NMI_Handler when HEXT failure is detected
 */
void clock_failure_detection_handler(void)
{
  /* Check if CFD interrupt flag is set */
  if(crm_flag_get(CRM_CLOCK_FAILURE_INT_FLAG) != RESET)
  {
    /* Disable CFD to prevent repeated interrupts */
    crm_clock_failure_detection_enable(FALSE);

    /* Switch to HICK-based configuration */
    sclk_288m_hick_config();

    /* Clear the CFD flag */
    crm_flag_clear(CRM_CLOCK_FAILURE_INT_FLAG);

    /* Optionally: Set error flag, log event, alert user */
  }
}

int main(void)
{
  /* Normal startup with HEXT-based clock */
  system_clock_config();
  at32_board_init();
  clkout_config();

  /* Enable Clock Failure Detection */
  crm_clock_failure_detection_enable(TRUE);

  while(1)
  {
    at32_led_toggle(LED2);
    delay_ms(200);
  }
}

/**
 * NMI Handler - called on clock failure
 * Add to interrupt handler file (at32f435_437_int.c)
 */
void NMI_Handler(void)
{
  clock_failure_detection_handler();
}
```

---

## Clock Configuration Checklist

### Basic Clock Setup
- [ ] Reset CRM (`crm_reset()`)
- [ ] Enable PWC clock (`CRM_PWC_PERIPH_CLOCK`)
- [ ] Set LDO voltage for target frequency
- [ ] Set flash divider for safe operation
- [ ] Enable clock source (HICK/HEXT)
- [ ] Wait for clock source stability

### PLL Configuration
- [ ] Configure PLL parameters (NS, MS, FR)
- [ ] Enable PLL
- [ ] Wait for PLL stability

### System Clock Switch
- [ ] Configure bus dividers (AHB, APB1, APB2)
- [ ] Enable auto-step mode
- [ ] Switch system clock source
- [ ] Wait for switch completion
- [ ] Disable auto-step mode
- [ ] Update `system_core_clock` variable
- [ ] Optimize flash divider
- [ ] Re-initialize delay functions

### Peripheral Setup
- [ ] Enable required peripheral clocks
- [ ] Configure clock outputs if needed

---

## Status Flags

| Flag | Description |
|------|-------------|
| `CRM_HICK_STABLE_FLAG` | HICK oscillator stable |
| `CRM_HEXT_STABLE_FLAG` | HEXT crystal stable |
| `CRM_PLL_STABLE_FLAG` | PLL locked |
| `CRM_LEXT_STABLE_FLAG` | LEXT crystal stable |
| `CRM_LICK_STABLE_FLAG` | LICK oscillator stable |
| `CRM_CLOCK_FAILURE_INT_FLAG` | Clock failure detected |

## Reset Flags

| Flag | Description |
|------|-------------|
| `CRM_POR_RESET_FLAG` | Power-on reset |
| `CRM_SW_RESET_FLAG` | Software reset |
| `CRM_WDT_RESET_FLAG` | Watchdog timer reset |
| `CRM_WWDT_RESET_FLAG` | Window watchdog reset |
| `CRM_LOWPOWER_RESET_FLAG` | Low-power reset |
| `CRM_NRST_RESET_FLAG` | External reset (NRST pin) |

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| System hangs after PLL switch | Wrong flash divider | Set flash divider before clock change |
| HEXT doesn't stabilize | Crystal problem | Check crystal, load capacitors |
| Clock frequency incorrect | Wrong PLL parameters | Verify NS/MS/FR calculation |
| Peripheral not working | Clock not enabled | Enable peripheral clock |
| USB not working | Wrong USB clock divider | Configure USB divider for 48 MHz |
| Low-power mode issues | Peripheral clock disabled | Enable low-power clock for peripheral |
| System unstable at high freq | LDO voltage too low | Set appropriate LDO voltage |

### Maximum Frequencies

| Bus/Peripheral | Maximum Frequency |
|----------------|-------------------|
| SCLK (System) | 288 MHz |
| AHB | 288 MHz |
| APB1 | 144 MHz |
| APB2 | 144 MHz |
| USB | 48 MHz |
| ADC | 72 MHz (HCLK/4 max) |

---

## See Also

- [ACC Documentation](./ACC_Auto_Clock_Calibration.md) - Auto clock calibration using USB SOF
- [Cortex-M4 Documentation](./Cortex_M4_Core_Features.md) - SysTick timer configuration
- AT32F435_437 Reference Manual - CRM Chapter

