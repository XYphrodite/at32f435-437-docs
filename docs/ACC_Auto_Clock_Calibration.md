---
title: "ACC - Auto Clock Calibration Peripheral"
type: "peripheral-documentation"
purpose: "context7-knowledge-source"
mcu_family: "AT32F435/437"
peripheral: "ACC"
version: "1.0.0"
last_updated: "2024-11-26"
tags:
  - acc
  - clock-calibration
  - hick
  - usb
  - sof
  - oscillator
  - peripheral
  - context7
related_peripherals:
  - CRM
  - USB
  - HICK
---

# ACC - Auto Clock Calibration Peripheral

## Overview

The **Auto Clock Calibration (ACC)** peripheral in AT32F435/437 MCUs provides automatic calibration of the internal High-speed Internal Clock (HICK) using a reference signal, typically the USB Start of Frame (SOF) signal. This enables precise 48MHz clock generation for USB applications without an external crystal oscillator.

**Key Features:**
- 🎯 Automatic HICK frequency calibration
- 📡 Uses USB SOF (Start of Frame) as reference signal
- ⚡ Two calibration modes: HICKCAL (coarse) and HICKTRIM (fine)
- 🔔 Interrupt support for calibration ready and signal lost events
- 📊 Configurable calibration step size and tolerance ranges

---

## Use Cases

### Primary Applications

1. **USB without External Crystal**
   - Run USB device/host at 48MHz using internal HICK oscillator
   - Self-calibrating system using USB SOF signal feedback

2. **Cost Reduction**
   - Eliminate external crystal oscillator in USB applications
   - Reduce BOM cost and PCB space

3. **Dynamic Frequency Adjustment**
   - Real-time oscillator tuning during operation
   - Compensation for temperature and voltage variations

---

## Architecture

```
                    ┌─────────────────────────────────────────┐
                    │              ACC Peripheral              │
                    │                                         │
  USB SOF Signal ──►│  ┌─────────┐    ┌──────────────────┐   │
  (OTG1 or OTG2)    │  │Reference│    │  Calibration     │   │
                    │  │ Counter │───►│  Logic           │   │──► HICK Trim/Cal
                    │  └─────────┘    │                  │   │    Adjustment
                    │                 │  C1 < count < C3 │   │
  HICK Clock ──────►│  ┌─────────┐    │       ▼          │   │
                    │  │ HICK    │───►│  Adjust HICKCAL  │   │
                    │  │ Counter │    │  or HICKTRIM     │   │
                    │  └─────────┘    └──────────────────┘   │
                    │                                         │
                    │  Interrupts: CALRDY, RSLOST            │
                    └─────────────────────────────────────────┘
```

### Calibration Process

1. **Reference Signal:** USB SOF arrives every 1ms (1kHz)
2. **Counting:** ACC counts HICK cycles between SOF signals
3. **Comparison:** Count compared against C1, C2, C3 values
4. **Adjustment:** HICKCAL or HICKTRIM adjusted based on comparison
5. **Convergence:** Process repeats until count is within tolerance (C1 < count < C3)

---

## Registers

### STS - Status Register (Offset: 0x00)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | CALRDY | Calibration ready flag. Set when HICK is calibrated (C1 < count < C3) |
| 1 | RSLOST | Reference signal lost flag. Set when USB SOF signal is missing |

### CTRL1 - Control Register 1 (Offset: 0x04)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | CALON | Calibration enable. 1 = ACC enabled |
| 1 | ENTRIM | Enable trim mode. 0 = HICKCAL mode, 1 = HICKTRIM mode |
| 4 | EIEN | Reference signal lost interrupt enable |
| 5 | CALRDYIEN | Calibration ready interrupt enable |
| 11:8 | STEP | Calibration step value (1-15) |

### CTRL2 - Control Register 2 (Offset: 0x08)

| Bit | Name | Description |
|-----|------|-------------|
| 7:0 | HICKCAL | HICK calibration value (read/write in HICKCAL mode) |
| 13:8 | HICKTRIM | HICK trim value (read/write in HICKTRIM mode) |

### C1, C2, C3 - Calibration Registers (Offset: 0x0C, 0x10, 0x14)

| Register | Purpose | Typical Value |
|----------|---------|---------------|
| C1 | Lower threshold | C2 - tolerance |
| C2 | Target count | 8000 (for 48MHz HICK / 1kHz SOF) |
| C3 | Upper threshold | C2 + tolerance |

**Note:** When HICK is 48MHz and SOF is 1kHz, the ideal count between SOF signals is 48,000,000 / 1000 = 48,000. However, the counter operates at HICK/6, so C2 = 48000/6 = 8000.

---

## API Reference

### Enable/Disable Calibration

```c
/**
 * @brief  Enable or disable ACC calibration mode
 * @param  acc_trim: Calibration type
 *         - ACC_CAL_HICKCAL: Coarse calibration (±40kHz/step)
 *         - ACC_CAL_HICKTRIM: Fine calibration (±20kHz/step)
 * @param  new_state: TRUE to enable, FALSE to disable
 */
void acc_calibration_mode_enable(uint16_t acc_trim, confirm_state new_state);
```

### Step Configuration

```c
/**
 * @brief  Set calibration step value
 * @param  step_value: Step increment (1-15)
 *         Higher value = faster convergence, lower accuracy
 */
void acc_step_set(uint8_t step_value);
```

### SOF Source Selection

```c
/**
 * @brief  Select USB SOF signal source
 * @param  sof_sel: SOF source
 *         - ACC_SOF_OTG1: Use OTG1 (OTGFS1) SOF signal
 *         - ACC_SOF_OTG2: Use OTG2 (OTGFS2) SOF signal
 */
void acc_sof_select(uint16_t sof_sel);
```

### Interrupt Control

```c
/**
 * @brief  Enable or disable ACC interrupts
 * @param  acc_int: Interrupt source
 *         - ACC_CALRDYIEN_INT: Calibration ready interrupt
 *         - ACC_EIEN_INT: Reference signal lost interrupt
 * @param  new_state: TRUE to enable, FALSE to disable
 */
void acc_interrupt_enable(uint16_t acc_int, confirm_state new_state);
```

### Calibration Value Access

```c
/* Read current trim/cal values */
uint8_t acc_hicktrim_get(void);  // Get HICKTRIM value (6-bit)
uint8_t acc_hickcal_get(void);   // Get HICKCAL value (8-bit)

/* Set calibration thresholds */
void acc_write_c1(uint16_t acc_c1_value);  // Lower threshold
void acc_write_c2(uint16_t acc_c2_value);  // Target count
void acc_write_c3(uint16_t acc_c3_value);  // Upper threshold

/* Read calibration thresholds */
uint16_t acc_read_c1(void);
uint16_t acc_read_c2(void);
uint16_t acc_read_c3(void);
```

### Flag Management

```c
/**
 * @brief  Check ACC flag status
 * @param  acc_flag: Flag to check
 *         - ACC_CALRDY_FLAG: Calibration ready
 *         - ACC_RSLOST_FLAG: Reference signal lost
 * @return flag_status: SET or RESET
 */
flag_status acc_flag_get(uint16_t acc_flag);
flag_status acc_interrupt_flag_get(uint16_t acc_flag);

/**
 * @brief  Clear ACC flags
 * @param  acc_flag: Flag(s) to clear (can be OR'd)
 */
void acc_flag_clear(uint16_t acc_flag);
```

---

## Calibration Modes

### HICKCAL Mode (Coarse)

- **Resolution:** ~40kHz per step
- **Use Case:** Initial calibration or large frequency errors
- **Tolerance:** C1/C3 typically ±20 counts from C2

```c
#define ACC_CAL  // Use HICKCAL mode

// Configure for coarse calibration
acc_write_c1(8000 - 20);  // 7980
acc_write_c2(8000);       // 8000 (target)
acc_write_c3(8000 + 20);  // 8020

// Enable HICKCAL calibration
acc_calibration_mode_enable(ACC_CAL_HICKCAL, TRUE);
```

### HICKTRIM Mode (Fine)

- **Resolution:** ~20kHz per step
- **Use Case:** Fine-tuning, higher accuracy requirements
- **Tolerance:** C1/C3 typically ±10 counts from C2

```c
#define ACC_TRIM  // Use HICKTRIM mode

// Configure for fine calibration
acc_write_c1(8000 - 10);  // 7990
acc_write_c2(8000);       // 8000 (target)
acc_write_c3(8000 + 10);  // 8010

// Enable HICKTRIM calibration
acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
```

### Mode Comparison

| Feature | HICKCAL | HICKTRIM |
|---------|---------|----------|
| Resolution | ~40kHz/step | ~20kHz/step |
| Convergence Speed | Faster | Slower |
| Final Accuracy | Lower | Higher |
| Typical Tolerance | ±20 counts | ±10 counts |
| Use Case | Initial/coarse | Fine-tuning |

---

## Complete Example: USB CDC with ACC Calibration

### System Configuration

```c
/* Clock configuration for 288MHz using HICK */
void system_clock_config(void)
{
  crm_reset();
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
  flash_clock_divider_set(FLASH_CLOCK_DIV_3);

  /* Enable HICK (High-speed Internal Clock) */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET) {}

  /* Configure PLL from HICK: 288MHz */
  /* SCLK = (HICK / 6 * PLL_NS) / (PLL_MS * PLL_FR) */
  /* SCLK = (48MHz / 6 * 144) / (1 * 4) = 288MHz */
  crm_pll_config(CRM_PLL_SOURCE_HICK, 144, 1, CRM_PLL_FR_4);
  
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET) {}

  crm_ahb_div_set(CRM_AHB_DIV_1);      // 288MHz
  crm_apb2_div_set(CRM_APB2_DIV_2);    // 144MHz
  crm_apb1_div_set(CRM_APB1_DIV_2);    // 144MHz

  crm_auto_step_mode_enable(TRUE);
  crm_sysclk_switch(CRM_SCLK_PLL);
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL) {}
  crm_auto_step_mode_enable(FALSE);
  
  system_core_clock_update();
}
```

### ACC Initialization

```c
/* Select calibration mode */
#define ACC_TRIM  // Use fine calibration mode

void acc_init(void)
{
  /* Enable ACC peripheral clock */
  crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);

  /* Enable interrupts */
  acc_interrupt_enable(ACC_CALRDYIEN_INT, TRUE);  // Calibration ready
  acc_interrupt_enable(ACC_EIEN_INT, TRUE);       // Reference signal lost

  /* Configure NVIC */
  nvic_irq_enable(ACC_IRQn, 1, 0);

  /* Set calibration thresholds */
  uint32_t acc_c2_value = 8000;  // Target count for 48MHz HICK
  
  #ifdef ACC_CAL
    /* HICKCAL mode: coarse tolerance */
    acc_write_c1(acc_c2_value - 20);
    acc_write_c2(acc_c2_value);
    acc_write_c3(acc_c2_value + 20);
  #else
    /* HICKTRIM mode: fine tolerance */
    acc_write_c1(acc_c2_value - 10);
    acc_write_c2(acc_c2_value);
    acc_write_c3(acc_c2_value + 10);
  #endif

  /* Select USB SOF source */
  acc_sof_select(ACC_SOF_OTG1);  // Use OTG1 SOF signal

  /* Start calibration */
  #ifdef ACC_CAL
    acc_calibration_mode_enable(ACC_CAL_HICKCAL, TRUE);
  #else
    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
  #endif
}
```

### ACC Interrupt Handler

```c
void ACC_IRQHandler(void)
{
  /* Calibration ready - HICK is now calibrated */
  if(acc_interrupt_flag_get(ACC_CALRDY_FLAG) != RESET)
  {
    /* LED indication: calibration successful */
    at32_led_toggle(LED2);
    
    /* Optional: Read calibration value for debugging */
    // uint8_t trim_value = acc_hicktrim_get();
    
    /* Clear flag */
    acc_flag_clear(ACC_CALRDY_FLAG);
  }

  /* Reference signal lost - USB disconnected or error */
  if(acc_interrupt_flag_get(ACC_RSLOST_FLAG) != RESET)
  {
    /* LED indication: signal lost */
    at32_led_toggle(LED3);
    
    /* Clear flag */
    acc_flag_clear(ACC_RSLOST_FLAG);
  }
}
```

### USB Clock Configuration

```c
typedef enum
{
  USB_CLK_HICK = 0,  // Use HICK as USB clock source
  USB_CLK_HEXT = 1   // Use HEXT as USB clock source
} usb_clk48_s;

void usb_clock48m_select(usb_clk48_s clk_s)
{
  if(clk_s == USB_CLK_HICK)
  {
    /* Use HICK directly - requires ACC calibration */
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);
  }
  else
  {
    /* Derive from HEXT via divider */
    switch(system_core_clock)
    {
      case 48000000:  crm_usb_clock_div_set(CRM_USB_DIV_1);   break;
      case 72000000:  crm_usb_clock_div_set(CRM_USB_DIV_1_5); break;
      case 96000000:  crm_usb_clock_div_set(CRM_USB_DIV_2);   break;
      case 120000000: crm_usb_clock_div_set(CRM_USB_DIV_2_5); break;
      case 144000000: crm_usb_clock_div_set(CRM_USB_DIV_3);   break;
      case 168000000: crm_usb_clock_div_set(CRM_USB_DIV_3_5); break;
      case 192000000: crm_usb_clock_div_set(CRM_USB_DIV_4);   break;
      case 216000000: crm_usb_clock_div_set(CRM_USB_DIV_4_5); break;
      case 240000000: crm_usb_clock_div_set(CRM_USB_DIV_5);   break;
      case 264000000: crm_usb_clock_div_set(CRM_USB_DIV_5_5); break;
      case 288000000: crm_usb_clock_div_set(CRM_USB_DIV_6);   break;
      default: break;
    }
  }
}
```

### Main Application

```c
int main(void)
{
  /* Initialize system */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  system_clock_config();
  at32_board_init();

  /* Configure USB GPIO */
  usb_gpio_config();

  /* Enable USB clock */
  crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
  
  /* Select USB clock from HICK (requires ACC) */
  usb_clock48m_select(USB_CLK_HICK);

  /* Enable USB interrupt */
  nvic_irq_enable(OTGFS1_IRQn, 0, 0);

  /* Initialize USB device */
  usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            0,  // USB_ID for OTG1
            &cdc_class_handler,
            &cdc_desc_handler);

  /* Initialize ACC calibration */
  acc_init();

  /* Main loop */
  while(1)
  {
    /* USB VCP loopback example */
    uint16_t data_len = usb_vcp_get_rxdata(&otg_core_struct.dev, usb_buffer);
    if(data_len > 0)
    {
      usb_vcp_send_data(&otg_core_struct.dev, usb_buffer, data_len);
    }
  }
}
```

---

## Implementation Checklist

### Setup Steps

- [ ] Enable CRM_ACC_PERIPH_CLOCK
- [ ] Enable USB peripheral clock (OTG1 or OTG2)
- [ ] Configure USB GPIO pins
- [ ] Select USB clock source as HICK
- [ ] Initialize USB peripheral
- [ ] Configure ACC calibration thresholds (C1, C2, C3)
- [ ] Select SOF source (ACC_SOF_OTG1 or ACC_SOF_OTG2)
- [ ] Enable ACC interrupts (optional)
- [ ] Enable ACC calibration

### Runtime Monitoring

- [ ] Monitor CALRDY flag for successful calibration
- [ ] Handle RSLOST flag for USB disconnect scenarios
- [ ] Optionally log trim values for debugging

---

## Troubleshooting

### Calibration Not Converging

**Symptoms:** CALRDY flag never sets, oscillator drifting

**Solutions:**
1. Verify USB connection is active (SOF signals present)
2. Check C1/C2/C3 values are appropriate
3. Increase tolerance (difference between C1/C3 and C2)
4. Try HICKCAL mode instead of HICKTRIM

### Reference Signal Lost (RSLOST)

**Symptoms:** RSLOST flag frequently triggering

**Solutions:**
1. Verify USB cable connection
2. Check USB host is sending SOF signals
3. Ensure correct SOF source selection (OTG1 vs OTG2)
4. Verify USB peripheral is properly initialized

### USB Enumeration Failure

**Symptoms:** USB device not recognized by host

**Solutions:**
1. Wait for CALRDY before USB operations
2. Verify 48MHz clock is generated correctly
3. Check USB D+/D- GPIO configuration
4. Verify VBUS detection (or use USB_VBUS_IGNORE)

### Clock Frequency Drift

**Symptoms:** USB communication errors, timing issues

**Solutions:**
1. Use HICKTRIM mode for better accuracy
2. Reduce calibration tolerance (smaller C1-C3 range)
3. Monitor temperature effects on HICK
4. Consider using HEXT for critical applications

---

## Related Documentation

- **Reference Manual:** RM_AT32F435_437 - Chapter on ACC
- **Application Note:** AN0107 - USB Clock Calibration
- **Example Code:** `project/at_start_f435/examples/acc/calibration/`

---

## API Summary

| Function | Description |
|----------|-------------|
| `acc_calibration_mode_enable()` | Enable/disable calibration |
| `acc_step_set()` | Set calibration step size |
| `acc_sof_select()` | Select SOF signal source |
| `acc_interrupt_enable()` | Enable/disable interrupts |
| `acc_hicktrim_get()` | Read HICKTRIM value |
| `acc_hickcal_get()` | Read HICKCAL value |
| `acc_write_c1/c2/c3()` | Set calibration thresholds |
| `acc_read_c1/c2/c3()` | Read calibration thresholds |
| `acc_flag_get()` | Check status flags |
| `acc_interrupt_flag_get()` | Check interrupt flags |
| `acc_flag_clear()` | Clear status flags |

---

**Last Updated:** November 2024  
**Firmware Library:** v2.2.2  
**Status:** ✅ Production Ready

**Tags:** #ACC #clock-calibration #HICK #USB #SOF #Context7

