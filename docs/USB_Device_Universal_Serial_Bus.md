---
title: USB Device - Universal Serial Bus Device Mode
category: Communication
complexity: Advanced
mcu: AT32F435/437
peripheral: USB OTG
keywords: [usb, otg, device, cdc, vcp, msc, hid, audio, winusb, composite]
---

# USB Device - Universal Serial Bus Device Mode

## Overview

The AT32F435/437 features two independent USB OTG (On-The-Go) Full-Speed controllers (OTGFS1 and OTGFS2), each supporting Device mode, Host mode, and DRD (Dual-Role Device) mode. The USB device middleware provides a complete USB device stack with support for multiple standard USB device classes.

### Key Features

| Feature | Specification |
|---------|---------------|
| USB Controllers | 2 × USB OTG Full-Speed (OTGFS1, OTGFS2) |
| USB Standard | USB 2.0 Full-Speed (12 Mbps) |
| Operating Modes | Device, Host, DRD |
| Endpoints | 8 IN + 8 OUT per controller |
| Endpoint Types | Control, Bulk, Interrupt, Isochronous |
| Max Packet Size | 64 bytes (EP0), up to 512 bytes (others) |
| FIFO Size | 320 words (1280 bytes) total |
| Clock Source | HEXT or HICK (48 MHz) |
| Power | Bus-powered or Self-powered |

### Supported USB Device Classes

| Class | Description | Use Case |
|-------|-------------|----------|
| **CDC (VCP)** | Communication Device Class | Virtual COM Port |
| **MSC** | Mass Storage Class | USB Flash Drive |
| **HID** | Human Interface Device | Keyboard, Mouse, Custom HID |
| **Audio** | Audio Class | Microphone, Speaker |
| **Printer** | Printer Class | USB Printer |
| **WinUSB** | Microsoft WinUSB | Custom USB Device (driverless) |
| **Composite** | Multiple Classes | VCP+MSC, VCP+Keyboard, Audio+HID |

## USB Hardware Architecture

```
                        AT32F435/437 USB OTG Architecture
    ┌─────────────────────────────────────────────────────────────────┐
    │                                                                 │
    │   USB OTG Controller (OTGFS1 or OTGFS2)                         │
    │   ┌─────────────────────────────────────────────────────────┐   │
    │   │                                                         │   │
    │   │  ┌──────────────┐    ┌──────────────┐                   │   │
    │   │  │  USB PHY     │    │  Control     │                   │   │
    │   │  │  (Built-in)  │◄──►│  Logic       │                   │   │
    │   │  └──────┬───────┘    └──────────────┘                   │   │
    │   │         │                    │                          │   │
    │   │         ▼                    ▼                          │   │
    │   │  ┌─────────────────────────────────────────────────┐    │   │
    │   │  │              Endpoint FIFOs (320 words)         │    │   │
    │   │  │  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐    │    │   │
    │   │  │  │ RX     │ │ TX EP0 │ │ TX EP1 │ │ TX EPn │    │    │   │
    │   │  │  │ FIFO   │ │ FIFO   │ │ FIFO   │ │ FIFO   │    │    │   │
    │   │  │  └────────┘ └────────┘ └────────┘ └────────┘    │    │   │
    │   │  └─────────────────────────────────────────────────┘    │   │
    │   │                         │                               │   │
    │   │                         ▼                               │   │
    │   │              AHB Bus Interface                          │   │
    │   │                                                         │   │
    │   └─────────────────────────────────────────────────────────┘   │
    │                             │                                   │
    │                             ▼                                   │
    │                      Cortex-M4 CPU                              │
    │                                                                 │
    └─────────────────────────────────────────────────────────────────┘
```

## USB Middleware Architecture

```
    ┌─────────────────────────────────────────────────────────────────┐
    │                      Application Layer                          │
    │   (main.c, user application code)                               │
    └───────────────────────────┬─────────────────────────────────────┘
                                │
    ┌───────────────────────────▼─────────────────────────────────────┐
    │                    USB Class Drivers                             │
    │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐   │
    │  │  CDC    │ │  MSC    │ │  HID    │ │ Audio   │ │ WinUSB  │   │
    │  │ (VCP)   │ │ (Disk)  │ │(KB/Mse) │ │(Spk/Mic)│ │ (Cust)  │   │
    │  └─────────┘ └─────────┘ └─────────┘ └─────────┘ └─────────┘   │
    │                                                                  │
    │  Composite: CDC+MSC, CDC+Keyboard, Audio+HID                    │
    └───────────────────────────┬─────────────────────────────────────┘
                                │
    ┌───────────────────────────▼─────────────────────────────────────┐
    │                  USB Device Core (usbd_core)                     │
    │   - Descriptor handling                                          │
    │   - Standard request processing                                  │
    │   - Endpoint management                                          │
    │   - State machine                                                │
    └───────────────────────────┬─────────────────────────────────────┘
                                │
    ┌───────────────────────────▼─────────────────────────────────────┐
    │                USB Hardware Abstraction (usb_core)               │
    │   - FIFO read/write                                             │
    │   - Interrupt handling                                          │
    │   - PHY control                                                 │
    └───────────────────────────┬─────────────────────────────────────┘
                                │
    ┌───────────────────────────▼─────────────────────────────────────┐
    │            Hardware Driver (at32f435_437_usb)                   │
    │   - Register access                                             │
    │   - Low-level operations                                        │
    └─────────────────────────────────────────────────────────────────┘
```

## Pin Configuration

### OTGFS1 Pins

| Pin | Function | Description |
|-----|----------|-------------|
| PA11 | OTG1_FS_DM | USB Data Minus |
| PA12 | OTG1_FS_DP | USB Data Plus |
| PA9 | OTG1_FS_VBUS | VBUS Sense (optional) |
| PA10 | OTG1_FS_ID | OTG ID (for DRD mode) |
| PA8 | OTG1_FS_SOF | SOF Output (optional) |

### OTGFS2 Pins

| Pin | Function | Description |
|-----|----------|-------------|
| PB14 | OTG2_FS_DM | USB Data Minus |
| PB15 | OTG2_FS_DP | USB Data Plus |
| PB13 | OTG2_FS_VBUS | VBUS Sense (optional) |
| PB12 | OTG2_FS_ID | OTG ID (for DRD mode) |

## USB Device Events

```c
/**
  * @brief usb device event types
  */
typedef enum
{
  USBD_NOP_EVENT,          /* No operation event */
  USBD_RESET_EVENT,        /* USB bus reset detected */
  USBD_SUSPEND_EVENT,      /* USB suspend detected */
  USBD_WAKEUP_EVENT,       /* USB wakeup detected */
  USBD_DISCONNECT_EVENT,   /* USB disconnect detected */
  USBD_INISOINCOM_EVENT,   /* Incomplete isochronous IN */
  USBD_OUTISOINCOM_EVENT,  /* Incomplete isochronous OUT */
  USBD_ERR_EVENT           /* USB error event */
} usbd_event_type;
```

## USB Connection States

```c
/**
  * @brief usb device connection states
  */
typedef enum
{
  USB_CONN_STATE_DEFAULT    = 1,  /* Default state after reset */
  USB_CONN_STATE_ADDRESSED,       /* Device has address assigned */
  USB_CONN_STATE_CONFIGURED,      /* Device is configured */
  USB_CONN_STATE_SUSPENDED        /* Device is suspended */
} usbd_conn_state;
```

## USB Class Codes

```c
/* USB Standard Class Codes */
#define USB_CLASS_CODE_AUDIO    0x01  /* Audio Class */
#define USB_CLASS_CODE_CDC      0x02  /* CDC (Communication Device Class) */
#define USB_CLASS_CODE_HID      0x03  /* HID (Human Interface Device) */
#define USB_CLASS_CODE_PRINTER  0x07  /* Printer Class */
#define USB_CLASS_CODE_MSC      0x08  /* Mass Storage Class */
#define USB_CLASS_CODE_HUB      0x09  /* Hub Class */
#define USB_CLASS_CODE_CDCDATA  0x0A  /* CDC Data Class */
#define USB_CLASS_CODE_VIDEO    0x0E  /* Video Class */
#define USB_CLASS_CODE_VENDOR   0xFF  /* Vendor Specific */
```

## Core API Reference

### USB Device Initialization

```c
/**
  * @brief  Initialize USB device core
  * @param  udev: USB device core handle
  * @param  usb_reg: USB register base address
  * @param  class_handler: USB class handler
  * @param  desc_handler: USB descriptor handler
  * @param  core_id: USB core ID (USB_OTG1_ID or USB_OTG2_ID)
  * @retval USB status
  */
usb_sts_type usbd_core_init(usbd_core_type *udev,
                            usb_reg_type *usb_reg,
                            usbd_class_handler *class_handler,
                            usbd_desc_handler *desc_handler,
                            uint8_t core_id);
```

### USB Endpoint Operations

```c
/* Open an endpoint */
void usbd_ept_open(usbd_core_type *udev, uint8_t ept_addr, 
                   uint8_t ept_type, uint16_t maxpacket);

/* Close an endpoint */
void usbd_ept_close(usbd_core_type *udev, uint8_t ept_addr);

/* Send data on IN endpoint */
void usbd_ept_send(usbd_core_type *udev, uint8_t ept_num, 
                   uint8_t *buffer, uint16_t len);

/* Receive data on OUT endpoint */
void usbd_ept_recv(usbd_core_type *udev, uint8_t ept_num, 
                   uint8_t *buffer, uint16_t len);

/* Set endpoint STALL */
void usbd_set_stall(usbd_core_type *udev, uint8_t ept_addr);

/* Clear endpoint STALL */
void usbd_clear_stall(usbd_core_type *udev, uint8_t ept_addr);

/* Get received data length */
uint32_t usbd_get_recv_len(usbd_core_type *udev, uint8_t ept_addr);
```

### USB Device Control

```c
/* Connect device (enable pull-up) */
void usbd_connect(usbd_core_type *udev);

/* Disconnect device (disable pull-up) */
void usbd_disconnect(usbd_core_type *udev);

/* Set device address */
void usbd_set_device_addr(usbd_core_type *udev, uint8_t address);

/* Get connection state */
usbd_conn_state usbd_connect_state_get(usbd_core_type *udev);

/* Remote wakeup */
void usbd_remote_wakeup(usbd_core_type *udev);

/* Enter suspend mode */
void usbd_enter_suspend(usbd_core_type *udev);
```

### USB Control Transfer

```c
/* Send data on control endpoint */
void usbd_ctrl_send(usbd_core_type *udev, uint8_t *buffer, uint16_t len);

/* Receive data on control endpoint */
void usbd_ctrl_recv(usbd_core_type *udev, uint8_t *buffer, uint16_t len);

/* Send ZLP status */
void usbd_ctrl_send_status(usbd_core_type *udev);

/* Receive ZLP status */
void usbd_ctrl_recv_status(usbd_core_type *udev);

/* STALL control endpoint (unsupported request) */
void usbd_ctrl_unsupport(usbd_core_type *udev);
```

## CDC (Virtual COM Port) Class API

```c
/**
  * @brief  Get received data from USB host
  * @param  udev: USB device handle
  * @param  recv_data: Buffer for received data
  * @retval Number of bytes received
  */
uint16_t usb_vcp_get_rxdata(void *udev, uint8_t *recv_data);

/**
  * @brief  Send data to USB host
  * @param  udev: USB device handle
  * @param  send_data: Data buffer to send
  * @param  len: Data length
  * @retval SUCCESS or ERROR
  */
error_status usb_vcp_send_data(void *udev, uint8_t *send_data, uint16_t len);
```

## Code Examples

### Example 1: USB Virtual COM Port (CDC)

```c
/**
 * @brief  USB CDC Virtual COM Port Example
 * @note   Creates a USB-to-UART bridge
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"

/* USB global struct */
otg_core_type otg_core_struct;
uint8_t usb_buffer[256];

void usb_clock48m_select(usb_clk48_s clk_s);
void usb_gpio_config(void);

int main(void)
{
  uint16_t data_len;
  
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  system_clock_config();
  at32_board_init();
  
  /* Configure USB GPIO */
  usb_gpio_config();
  
  /* Enable OTGFS clock */
  crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
  
  /* Select USB 48MHz clock source from HEXT */
  usb_clock48m_select(USB_CLK_HEXT);
  
  /* Enable USB interrupt */
  nvic_irq_enable(OTGFS1_IRQn, 0, 0);
  
  /* Initialize USB device */
  usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_OTG1_ID,
            &cdc_class_handler,
            &cdc_desc_handler);
  
  while(1)
  {
    /* Get data received from USB host */
    data_len = usb_vcp_get_rxdata(&otg_core_struct.dev, usb_buffer);
    
    if(data_len > 0)
    {
      /* Echo data back to host */
      usb_vcp_send_data(&otg_core_struct.dev, usb_buffer, data_len);
    }
  }
}

void usb_gpio_config(void)
{
  gpio_init_type gpio_init_struct;
  
  /* Enable GPIO clock */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  
  /* Configure PA11 (DM) and PA12 (DP) */
  gpio_init_struct.gpio_pins = GPIO_PINS_11 | GPIO_PINS_12;
  gpio_init(GPIOA, &gpio_init_struct);
  
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_10);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_10);
}

void usb_clock48m_select(usb_clk48_s clk_s)
{
  if(clk_s == USB_CLK_HEXT)
  {
    switch(system_core_clock)
    {
      case 144000000:
        crm_usb_clock_div_set(CRM_USB_DIV_3);
        break;
      case 192000000:
        crm_usb_clock_div_set(CRM_USB_DIV_4);
        break;
      case 288000000:
        crm_usb_clock_div_set(CRM_USB_DIV_6);
        break;
      default:
        break;
    }
  }
}

/* USB interrupt handler */
void OTGFS1_IRQHandler(void)
{
  usbd_irq_handler(&otg_core_struct);
}

/* Required delay functions */
void usb_delay_ms(uint32_t ms) { delay_ms(ms); }
void usb_delay_us(uint32_t us) { delay_us(us); }
```

### Example 2: USB Mass Storage Class (MSC)

```c
/**
 * @brief  USB Mass Storage Class Example
 * @note   Exposes internal Flash or external media as USB disk
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "msc_class.h"
#include "msc_desc.h"
#include "flash_disk.h"

otg_core_type otg_core_struct;

int main(void)
{
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  system_clock_config();
  at32_board_init();
  
  /* Initialize storage media */
  flash_disk_init();
  
  /* Configure USB GPIO */
  usb_gpio_config();
  
  /* Enable OTGFS clock */
  crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
  
  /* Select USB 48MHz clock */
  crm_usb_clock_div_set(CRM_USB_DIV_3);
  
  /* Enable USB interrupt */
  nvic_irq_enable(OTGFS1_IRQn, 0, 0);
  
  /* Initialize USB MSC device */
  usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_OTG1_ID,
            &msc_class_handler,
            &msc_desc_handler);
  
  while(1)
  {
    /* MSC operations handled in interrupt */
  }
}
```

### Example 3: USB HID Keyboard

```c
/**
 * @brief  USB HID Keyboard Example
 * @note   Sends keyboard key presses to host
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "keyboard_class.h"
#include "keyboard_desc.h"

otg_core_type otg_core_struct;

/* HID keyboard report structure */
typedef struct {
  uint8_t modifier;   /* Modifier keys (Ctrl, Shift, Alt, GUI) */
  uint8_t reserved;
  uint8_t keycode[6]; /* Up to 6 simultaneous keys */
} hid_keyboard_report_t;

hid_keyboard_report_t keyboard_report;

void send_key(uint8_t keycode)
{
  /* Clear report */
  memset(&keyboard_report, 0, sizeof(keyboard_report));
  
  /* Set key pressed */
  keyboard_report.keycode[0] = keycode;
  
  /* Send key press */
  usb_hid_keyboard_send(&otg_core_struct.dev, 
                        (uint8_t*)&keyboard_report, 
                        sizeof(keyboard_report));
  
  delay_ms(50);
  
  /* Send key release */
  memset(&keyboard_report, 0, sizeof(keyboard_report));
  usb_hid_keyboard_send(&otg_core_struct.dev, 
                        (uint8_t*)&keyboard_report, 
                        sizeof(keyboard_report));
}

int main(void)
{
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  system_clock_config();
  at32_board_init();
  
  usb_gpio_config();
  crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
  crm_usb_clock_div_set(CRM_USB_DIV_3);
  nvic_irq_enable(OTGFS1_IRQn, 0, 0);
  
  /* Initialize USB HID Keyboard */
  usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_OTG1_ID,
            &keyboard_class_handler,
            &keyboard_desc_handler);
  
  while(1)
  {
    /* Wait for button press */
    if(at32_button_press() == USER_BUTTON)
    {
      /* Send 'A' key (keycode 0x04) */
      send_key(0x04);
    }
    delay_ms(100);
  }
}
```

### Example 4: USB HID Mouse

```c
/**
 * @brief  USB HID Mouse Example
 * @note   Sends mouse movements and button clicks
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "mouse_class.h"
#include "mouse_desc.h"

otg_core_type otg_core_struct;

/* HID mouse report structure */
typedef struct {
  uint8_t buttons;   /* Button states */
  int8_t x;          /* X movement (-127 to 127) */
  int8_t y;          /* Y movement (-127 to 127) */
  int8_t wheel;      /* Scroll wheel */
} hid_mouse_report_t;

hid_mouse_report_t mouse_report;

void move_mouse(int8_t x, int8_t y)
{
  mouse_report.buttons = 0;
  mouse_report.x = x;
  mouse_report.y = y;
  mouse_report.wheel = 0;
  
  usb_hid_mouse_send(&otg_core_struct.dev, 
                     (uint8_t*)&mouse_report, 
                     sizeof(mouse_report));
}

void click_mouse(uint8_t button)
{
  /* Button: 0x01=Left, 0x02=Right, 0x04=Middle */
  mouse_report.buttons = button;
  mouse_report.x = 0;
  mouse_report.y = 0;
  mouse_report.wheel = 0;
  
  /* Press */
  usb_hid_mouse_send(&otg_core_struct.dev, 
                     (uint8_t*)&mouse_report, 
                     sizeof(mouse_report));
  delay_ms(50);
  
  /* Release */
  mouse_report.buttons = 0;
  usb_hid_mouse_send(&otg_core_struct.dev, 
                     (uint8_t*)&mouse_report, 
                     sizeof(mouse_report));
}

int main(void)
{
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  system_clock_config();
  at32_board_init();
  
  usb_gpio_config();
  crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
  crm_usb_clock_div_set(CRM_USB_DIV_3);
  nvic_irq_enable(OTGFS1_IRQn, 0, 0);
  
  /* Initialize USB HID Mouse */
  usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_OTG1_ID,
            &mouse_class_handler,
            &mouse_desc_handler);
  
  while(1)
  {
    if(at32_button_press() == USER_BUTTON)
    {
      /* Move mouse in a square pattern */
      move_mouse(50, 0);   delay_ms(100);
      move_mouse(0, 50);   delay_ms(100);
      move_mouse(-50, 0);  delay_ms(100);
      move_mouse(0, -50);  delay_ms(100);
    }
    delay_ms(100);
  }
}
```

### Example 5: USB Audio Class

```c
/**
 * @brief  USB Audio Class Example
 * @note   USB microphone and speaker device
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "audio_class.h"
#include "audio_desc.h"
#include "audio_codec.h"

otg_core_type otg_core_struct;

/* Audio sample rate */
#define AUDIO_SAMPLE_RATE    48000
#define AUDIO_CHANNELS       2
#define AUDIO_BIT_DEPTH      16

int main(void)
{
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  system_clock_config();
  at32_board_init();
  
  /* Initialize audio codec (e.g., WM8988) */
  audio_codec_init(AUDIO_SAMPLE_RATE, AUDIO_BIT_DEPTH, AUDIO_CHANNELS);
  
  /* Configure I2S for audio streaming */
  audio_i2s_config();
  
  usb_gpio_config();
  crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
  crm_usb_clock_div_set(CRM_USB_DIV_3);
  nvic_irq_enable(OTGFS1_IRQn, 0, 0);
  
  /* Initialize USB Audio device */
  usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_OTG1_ID,
            &audio_class_handler,
            &audio_desc_handler);
  
  while(1)
  {
    /* Audio streaming handled via I2S DMA and USB interrupts */
  }
}
```

### Example 6: Composite CDC + MSC

```c
/**
 * @brief  Composite USB Device (CDC + MSC)
 * @note   Virtual COM Port + Mass Storage in single device
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "cdc_msc_class.h"
#include "cdc_msc_desc.h"

otg_core_type otg_core_struct;
uint8_t usb_buffer[256];

int main(void)
{
  uint16_t data_len;
  
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  system_clock_config();
  at32_board_init();
  
  /* Initialize storage for MSC */
  flash_disk_init();
  
  usb_gpio_config();
  crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
  crm_usb_clock_div_set(CRM_USB_DIV_3);
  nvic_irq_enable(OTGFS1_IRQn, 0, 0);
  
  /* Initialize Composite CDC+MSC device */
  usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_OTG1_ID,
            &cdc_msc_class_handler,
            &cdc_msc_desc_handler);
  
  while(1)
  {
    /* Handle CDC data */
    data_len = usb_vcp_get_rxdata(&otg_core_struct.dev, usb_buffer);
    if(data_len > 0)
    {
      usb_vcp_send_data(&otg_core_struct.dev, usb_buffer, data_len);
    }
    
    /* MSC handled in interrupt */
  }
}
```

### Example 7: WinUSB Device

```c
/**
 * @brief  WinUSB Device Example
 * @note   Custom USB device with automatic Windows driver installation
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "winusb_class.h"
#include "winusb_desc.h"

otg_core_type otg_core_struct;
uint8_t rx_buffer[512];
uint8_t tx_buffer[512];

int main(void)
{
  uint16_t rx_len;
  
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  system_clock_config();
  at32_board_init();
  
  usb_gpio_config();
  crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
  crm_usb_clock_div_set(CRM_USB_DIV_3);
  nvic_irq_enable(OTGFS1_IRQn, 0, 0);
  
  /* Initialize WinUSB device */
  usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_OTG1_ID,
            &winusb_class_handler,
            &winusb_desc_handler);
  
  while(1)
  {
    /* Receive data from host */
    rx_len = winusb_get_rxdata(&otg_core_struct.dev, rx_buffer);
    
    if(rx_len > 0)
    {
      /* Process received data and prepare response */
      process_command(rx_buffer, rx_len, tx_buffer);
      
      /* Send response to host */
      winusb_send_data(&otg_core_struct.dev, tx_buffer, tx_len);
    }
  }
}
```

## USB Clock Configuration

The USB peripheral requires a 48 MHz clock. Configure using HEXT or HICK:

```c
void usb_clock48m_select(usb_clk48_s clk_s)
{
  if(clk_s == USB_CLK_HICK)
  {
    /* Use HICK with ACC calibration */
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);
    
    /* ACC calibration values */
    acc_write_c1(7980);
    acc_write_c2(8000);
    acc_write_c3(8020);
    acc_sof_select(ACC_SOF_OTG1);
    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
  }
  else /* USB_CLK_HEXT */
  {
    /* Calculate divider based on system clock */
    switch(system_core_clock)
    {
      case 48000000:  crm_usb_clock_div_set(CRM_USB_DIV_1);   break;
      case 72000000:  crm_usb_clock_div_set(CRM_USB_DIV_1_5); break;
      case 96000000:  crm_usb_clock_div_set(CRM_USB_DIV_2);   break;
      case 144000000: crm_usb_clock_div_set(CRM_USB_DIV_3);   break;
      case 192000000: crm_usb_clock_div_set(CRM_USB_DIV_4);   break;
      case 240000000: crm_usb_clock_div_set(CRM_USB_DIV_5);   break;
      case 288000000: crm_usb_clock_div_set(CRM_USB_DIV_6);   break;
      default: break;
    }
  }
}
```

## Configuration Checklist

- [ ] Enable GPIO clock for USB pins
- [ ] Configure USB D+/D- pins as alternate function
- [ ] Enable OTGFS peripheral clock
- [ ] Configure USB 48 MHz clock (from HEXT or HICK)
- [ ] Set NVIC priority group
- [ ] Enable USB interrupt in NVIC
- [ ] Initialize USB device core with class/descriptor handlers
- [ ] Implement `usb_delay_ms()` and `usb_delay_us()` functions
- [ ] Implement USB interrupt handler

## Troubleshooting

### Device Not Recognized

1. Verify USB D+/D- pin configuration (PA11/PA12 or PB14/PB15)
2. Check USB 48 MHz clock configuration
3. Ensure pull-up resistor on D+ is enabled (via soft connect)
4. Verify USB cable connectivity

### Enumeration Fails

1. Check descriptor configuration (VID/PID, string descriptors)
2. Verify endpoint configuration matches class requirements
3. Ensure FIFO allocation is correct
4. Check interrupt priority configuration

### Data Transfer Issues

1. Verify endpoint addresses match between class and descriptor
2. Check buffer sizes and alignment
3. Ensure proper handling of ZLP (Zero-Length Packets)
4. Verify flow control (TX/RX complete flags)

### Power Issues

1. Configure VBUS sensing if using bus-powered mode
2. Set appropriate max power in configuration descriptor
3. Enable power-on GPIO if using external VBUS switch

## Performance Specifications

| Parameter | Full-Speed | Notes |
|-----------|------------|-------|
| Max Transfer Rate | 12 Mbps | USB 2.0 Full-Speed |
| Control Transfer | 64 bytes | EP0 max packet |
| Bulk Transfer | 64 bytes | Max packet size |
| Interrupt Transfer | 64 bytes | Max packet size |
| Isochronous Transfer | 1023 bytes | Max packet size |
| Total FIFO | 320 words | 1280 bytes shared |

## Related Documents

- **AN0097** - AT32 USB Application Note
- **USB 2.0 Specification** - Universal Serial Bus Specification
- **USB CDC Specification** - Communication Device Class
- **USB MSC Specification** - Mass Storage Class
- **USB HID Specification** - Human Interface Device Class

## Related Peripherals

- **[CRM](CRM_Clock_Reset_Management.md)** - USB clock configuration
- **[GPIO](GPIO_General_Purpose_IO.md)** - USB pin configuration
- **[EXINT](EXINT_External_Interrupt.md)** - USB wakeup interrupt
- **[I2S](I2S_Inter_IC_Sound.md)** - Audio class codec interface
- **[SDIO](SDIO_SD_MMC_Card_Interface.md)** - MSC storage backend

