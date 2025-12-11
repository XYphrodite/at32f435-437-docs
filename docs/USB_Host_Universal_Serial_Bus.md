---
title: USB Host - Universal Serial Bus Host Mode
category: Communication
complexity: Advanced
mcu: AT32F435/437
peripheral: USB OTG
keywords: [usb, otg, host, cdc, msc, hid, keyboard, mouse, fat32, mass storage]
---

# USB Host - Universal Serial Bus Host Mode

## Overview

The AT32F435/437 features two independent USB OTG (On-The-Go) Full-Speed controllers (OTGFS1 and OTGFS2), each supporting Host mode operation. The USB host middleware provides a complete USB host stack with support for multiple standard USB device classes, enabling the MCU to communicate with USB devices such as keyboards, mice, USB flash drives, and CDC devices.

### Key Features

| Feature | Specification |
|---------|---------------|
| USB Controllers | 2 × USB OTG Full-Speed (OTGFS1, OTGFS2) |
| USB Standard | USB 2.0 Full-Speed (12 Mbps) |
| Operating Modes | Host, Device, DRD |
| Host Channels | 16 per controller |
| Maximum Endpoints | 5 per interface |
| Maximum Interfaces | 5 per device |
| Supported Classes | CDC, MSC, HID (Keyboard/Mouse) |
| File System | FatFS integration for MSC |

### Supported Device Classes

| Class | Description | Use Case |
|-------|-------------|----------|
| **CDC** | Communication Device Class | USB-to-Serial adapters, modems |
| **MSC** | Mass Storage Class | USB flash drives, card readers |
| **HID** | Human Interface Device | Keyboards, mice, gamepads |

---

## Architecture

### Hardware Block Diagram

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                    USB OTG Controller                        │
                    │  ┌─────────────────────────────────────────────────────────┐│
                    │  │                    Host Controller                       ││
                    │  │  ┌───────────┐  ┌───────────┐  ┌───────────────────────┐││
                    │  │  │  Channel  │  │  Channel  │  │     Host Channel      │││
                    │  │  │  Manager  │─▶│  FIFO     │─▶│     State Machine     │││
                    │  │  │  (16 ch)  │  │  Memory   │  │                       │││
                    │  │  └───────────┘  └───────────┘  └───────────────────────┘││
                    │  │        │              │                    │             ││
                    │  │        ▼              ▼                    ▼             ││
                    │  │  ┌───────────────────────────────────────────────────┐  ││
                    │  │  │              USB Protocol Engine                   │  ││
                    │  │  │  (Enumeration, Control, Bulk, Interrupt, Isoc)    │  ││
                    │  │  └───────────────────────────────────────────────────┘  ││
                    │  └─────────────────────────────────────────────────────────┘│
                    │                              │                               │
                    │                              ▼                               │
                    │  ┌───────────────────────────────────────────────────────┐  │
                    │  │                      USB PHY                           │  │
                    │  │  DP ─────────────────────────────────────▶ USB_DP     │  │
                    │  │  DM ─────────────────────────────────────▶ USB_DM     │  │
                    │  │  VBUS ───────────────────────────────────▶ USB_VBUS   │  │
                    │  │  ID ─────────────────────────────────────▶ USB_ID     │  │
                    │  └───────────────────────────────────────────────────────┘  │
                    └─────────────────────────────────────────────────────────────┘
                                                    │
                                              To USB Device
```

### Middleware Architecture

```
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                         Application Layer                                │
    │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────────────┐│
    │  │   User      │ │   FatFS     │ │   Keyboard  │ │   CDC Application   ││
    │  │ Application │ │ File System │ │   /Mouse    │ │                     ││
    │  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └──────────┬──────────┘│
    └─────────┼───────────────┼───────────────┼──────────────────┼────────────┘
              │               │               │                  │
    ┌─────────┼───────────────┼───────────────┼──────────────────┼────────────┐
    │         ▼               ▼               ▼                  ▼            │
    │  ┌─────────────────────────────────────────────────────────────────┐   │
    │  │                     Class Driver Layer                          │   │
    │  │  ┌───────────┐  ┌───────────────┐  ┌───────────────────────────┐│   │
    │  │  │ MSC Class │  │   HID Class   │  │        CDC Class          ││   │
    │  │  │(Bot/SCSI) │  │(KB/Mouse/Raw) │  │   (Line Coding/Data)      ││   │
    │  │  └───────────┘  └───────────────┘  └───────────────────────────┘│   │
    │  └─────────────────────────────────────────────────────────────────┘   │
    │                                 │                                       │
    │                                 ▼                                       │
    │  ┌─────────────────────────────────────────────────────────────────┐   │
    │  │                    USB Host Core (usbh_core)                     │   │
    │  │  • Device Enumeration    • Descriptor Parsing                   │   │
    │  │  • Control Transfers     • Channel Management                   │   │
    │  │  • State Machine         • User Callbacks                       │   │
    │  └─────────────────────────────────────────────────────────────────┘   │
    │                                 │                                       │
    │                                 ▼                                       │
    │  ┌─────────────────────────────────────────────────────────────────┐   │
    │  │                  USB Core Driver (usb_core)                      │   │
    │  │  • Register Access       • FIFO Management                      │   │
    │  │  • Interrupt Handling    • Power Management                     │   │
    │  └─────────────────────────────────────────────────────────────────┘   │
    │                                 │                                       │
    │                                 ▼                                       │
    │  ┌─────────────────────────────────────────────────────────────────┐   │
    │  │              Hardware Driver (at32f435_437_usb)                  │   │
    │  └─────────────────────────────────────────────────────────────────┘   │
    └─────────────────────────────────────────────────────────────────────────┘
```

---

## Pin Configuration

### OTGFS1 Pins (USB OTG 1)

| Pin | Function | Direction | Description |
|-----|----------|-----------|-------------|
| PA11 | OTG1_DM | Bidirectional | USB Data Minus |
| PA12 | OTG1_DP | Bidirectional | USB Data Plus |
| PA9 | OTG1_VBUS | Input | VBUS sensing |
| PA10 | OTG1_ID | Input | OTG ID detection |
| PA8 | OTG1_SOF | Output | Start of Frame (optional) |

### OTGFS2 Pins (USB OTG 2)

| Pin | Function | Direction | Description |
|-----|----------|-----------|-------------|
| PB14 | OTG2_DM | Bidirectional | USB Data Minus |
| PB15 | OTG2_DP | Bidirectional | USB Data Plus |
| PB13 | OTG2_VBUS | Input | VBUS sensing |
| PB12 | OTG2_ID | Input | OTG ID detection |

---

## Host State Machine

### Global States

| State | Value | Description |
|-------|-------|-------------|
| `USBH_IDLE` | 0 | Waiting for device connection |
| `USBH_PORT_EN` | 1 | Port enabled |
| `USBH_ATTACHED` | 2 | Device attached |
| `USBH_DISCONNECT` | 3 | Device disconnected |
| `USBH_DEV_SPEED` | 4 | Getting device speed |
| `USBH_ENUMERATION` | 5 | Enumerating device |
| `USBH_CLASS_REQUEST` | 6 | Class-specific requests |
| `USBH_CLASS` | 7 | Class handler running |
| `USBH_CTRL_XFER` | 8 | Control transfer in progress |
| `USBH_USER_HANDLER` | 9 | User handler active |
| `USBH_SUSPEND` | 10 | Entering suspend mode |
| `USBH_SUSPENDED` | 11 | In suspend mode |
| `USBH_WAKEUP` | 12 | Waking up from suspend |
| `USBH_UNSUPPORT` | 13 | Unsupported device |
| `USBH_ERROR_STATE` | 14 | Error state |

### Enumeration States

| State | Description |
|-------|-------------|
| `ENUM_IDLE` | Initial state |
| `ENUM_GET_MIN_DESC` | Get device descriptor (8 bytes) |
| `ENUM_GET_FULL_DESC` | Get full device descriptor (18 bytes) |
| `ENUM_SET_ADDR` | Set device address |
| `ENUM_GET_CFG` | Get configuration descriptor header |
| `ENUM_GET_FULL_CFG` | Get full configuration descriptor |
| `ENUM_GET_MFC_STRING` | Get manufacturer string |
| `ENUM_GET_PRODUCT_STRING` | Get product string |
| `ENUM_GET_SERIALNUM_STRING` | Get serial number string |
| `ENUM_SET_CONFIG` | Set configuration |
| `ENUM_COMPLETE` | Enumeration complete |

---

## Channel States

### Channel Status Types

| Status | Description |
|--------|-------------|
| `HCH_IDLE` | Channel idle |
| `HCH_XFRC` | Transfer completed |
| `HCH_HALTED` | Channel halted |
| `HCH_NAK` | NAK received |
| `HCH_NYET` | NYET received |
| `HCH_STALL` | STALL received |
| `HCH_XACTERR` | Transaction error |
| `HCH_BBLERR` | Babble error |
| `HCH_DATATGLERR` | Data toggle error |

### URB (USB Request Block) States

| State | Description |
|-------|-------------|
| `URB_IDLE` | Request idle |
| `URB_DONE` | Request completed |
| `URB_NOTREADY` | Request not ready |
| `URB_NYET` | NYET status |
| `URB_ERROR` | Request error |
| `URB_STALL` | STALL received |

---

## USB Host Core API

### Initialization Functions

```c
/* Initialize USB host core */
usb_sts_type usbh_core_init(usbh_core_type *uhost,
                            usb_reg_type *usb_reg,
                            usbh_class_handler_type *class_handler,
                            usbh_user_handler_type *user_handler,
                            uint8_t core_id);

/* Reset default configuration */
usb_sts_type usbh_cfg_default_init(usbh_core_type *uhost);
```

### Main Loop Handler

```c
/* Must be called in main loop - processes USB host state machine */
usb_sts_type usbh_loop_handler(usbh_core_type *uhost);
```

### Channel Management

```c
/* Allocate a host channel */
uint16_t usbh_alloc_channel(usbh_core_type *uhost, uint8_t ept_addr);

/* Get free channel */
uint16_t usbh_get_free_channel(usbh_core_type *uhost);

/* Free a host channel */
void usbh_free_channel(usbh_core_type *uhost, uint8_t index);

/* Open a host channel */
void usbh_hc_open(usbh_core_type *uhost,
                  uint8_t chn,
                  uint8_t ept_num,
                  uint8_t dev_address,
                  uint8_t type,
                  uint16_t maxpacket,
                  uint8_t speed);

/* Disable a host channel */
void usbh_ch_disable(usbh_core_type *uhost, uint8_t chn);
```

### Transfer Functions

```c
/* Bulk transfers */
usb_sts_type usbh_bulk_recv(usbh_core_type *uhost, uint8_t hc_num,
                            uint8_t *buffer, uint16_t length);
usb_sts_type usbh_bulk_send(usbh_core_type *uhost, uint8_t hc_num,
                            uint8_t *buffer, uint16_t length);

/* Interrupt transfers */
usb_sts_type usbh_interrupt_recv(usbh_core_type *uhost, uint8_t hc_num,
                                 uint8_t *buffer, uint16_t length);
usb_sts_type usbh_interrupt_send(usbh_core_type *uhost, uint8_t hc_num,
                                 uint8_t *buffer, uint16_t length);

/* Isochronous transfers */
usb_sts_type usbh_isoc_recv(usbh_core_type *uhost, uint8_t hc_num,
                            uint8_t *buffer, uint16_t length);
usb_sts_type usbh_isoc_send(usbh_core_type *uhost, uint8_t hc_num,
                            uint8_t *buffer, uint16_t length);

/* Control transfer helper */
usb_sts_type usbh_in_out_request(usbh_core_type *uhost, uint8_t hc_num);
```

### Status Functions

```c
/* Get URB status */
urb_sts_type usbh_get_urb_status(usbh_core_type *uhost, uint8_t ch_num);

/* Set data toggle */
usb_sts_type usbh_set_toggle(usbh_core_type *uhost, uint8_t hc_num, uint8_t toggle);

/* Check control transfer result */
usb_sts_type usbh_ctrl_result_check(usbh_core_type *uhost,
                                    ctrl_ept0_sts_type next_ctrl_state,
                                    uint8_t next_enum_state);
```

### Power Management

```c
/* Enter suspend mode */
void usbh_enter_suspend(usbh_core_type *uhost);

/* Resume from suspend */
void usbh_resume(usbh_core_type *uhost);

/* Reset port */
void usbh_reset_port(usbh_core_type *uhost);

/* Control VBUS */
void usbh_active_vbus(usbh_core_type *uhost, confirm_state state);

/* Allocate address for device */
uint8_t usbh_alloc_address(void);
```

---

## User Handler Interface

### User Handler Structure

```c
typedef struct
{
  usb_sts_type (*user_init)(void);                                    /* Init callback */
  usb_sts_type (*user_reset)(void);                                   /* Reset callback */
  usb_sts_type (*user_attached)(void);                                /* Device attached */
  usb_sts_type (*user_disconnect)(void);                              /* Device disconnected */
  usb_sts_type (*user_speed)(uint8_t speed);                          /* Speed detected */
  usb_sts_type (*user_mfc_string)(void *);                            /* Manufacturer string */
  usb_sts_type (*user_product_string)(void *);                        /* Product string */
  usb_sts_type (*user_serial_string)(void *);                         /* Serial string */
  usb_sts_type (*user_enumeration_done)(void);                        /* Enumeration done */
  usb_sts_type (*user_application)(void);                             /* Application handler */
  usb_sts_type (*user_active_vbus)(void *uhost, confirm_state state); /* VBUS control */
  usb_sts_type (*user_not_support)(void);                             /* Unsupported device */
} usbh_user_handler_type;
```

---

## CDC Class (Communication Device Class)

### CDC States

| State | Description |
|-------|-------------|
| `CDC_IDLE_STATE` | Idle |
| `CDC_SET_LINE_CODING_STATE` | Setting line coding |
| `CDC_GET_LAST_LINE_CODING_STATE` | Getting line coding |
| `CDC_TRANSFER_DATA` | Data transfer active |
| `CDC_ERROR_STATE` | Error state |

### CDC Data States

| State | Description |
|-------|-------------|
| `CDC_IDLE` | Data idle |
| `CDC_SEND_DATA` | Sending data |
| `CDC_SEND_DATA_WAIT` | Waiting for send completion |
| `CDC_RECEIVE_DATA` | Receiving data |
| `CDC_RECEIVE_DATA_WAIT` | Waiting for receive completion |

### Line Coding Structure

```c
typedef struct
{
  uint32_t data_baudrate;  /* Baud rate */
  uint8_t  char_format;    /* Stop bits: 0=1, 1=1.5, 2=2 */
  uint8_t  parity_type;    /* Parity: 0=None, 1=Odd, 2=Even, 3=Mark, 4=Space */
  uint8_t  data_bits;      /* Data bits: 5, 6, 7, 8, or 16 */
} cdc_line_coding_type;
```

### CDC API

```c
/* Start data transmission */
void cdc_start_transmission(usbh_core_type *phost, uint8_t *data, uint32_t len);

/* Start data reception */
void cdc_start_reception(usbh_core_type *uhost, uint8_t *data, uint32_t len);

/* Transmission complete callback (implement in application) */
void cdc_transmit_complete(usbh_core_type *uhost);

/* Reception complete callback (implement in application) */
void cdc_receive_complete(usbh_core_type *uhost);

/* Class handler */
extern usbh_class_handler_type uhost_cdc_class_handler;
extern usbh_cdc_type usbh_cdc;
```

---

## MSC Class (Mass Storage Class)

### MSC States

| State | Description |
|-------|-------------|
| `USBH_MSC_STATE_IDLE` | Idle |
| `USBH_MSC_STATE_GET_LUN` | Getting LUN count |
| `USBH_MSC_STATE_ERROR` | Error state |
| `USBH_MSC_STATE_COMPLETE` | Complete |

### MSC API

```c
/* Check if MSC device is ready */
msc_error_type usbh_msc_is_ready(void *uhost, uint8_t lun);

/* Read from MSC device */
usb_sts_type usbh_msc_read(void *uhost, uint32_t address, uint32_t len,
                           uint8_t *buffer, uint8_t lun);

/* Write to MSC device */
usb_sts_type usbh_msc_write(void *uhost, uint32_t address, uint32_t len,
                            uint8_t *buffer, uint8_t lun);

/* Read/Write handle (internal) */
usb_sts_type usbh_msc_rw_handle(void *uhost, uint32_t address, uint32_t len,
                                uint8_t *buffer, uint8_t lun);

/* Initialize BOT/SCSI */
usb_sts_type msc_bot_scsi_init(usbh_msc_type *msc_struct);

/* Class handler */
extern usbh_class_handler_type uhost_msc_class_handler;
extern usbh_msc_type usbh_msc;
```

### FatFS Integration

The MSC class can be integrated with FatFS for file system operations:

```c
/* Disk I/O functions for FatFS (implement in usbh_msc_diskio.c) */
DSTATUS disk_initialize(BYTE pdrv);
DSTATUS disk_status(BYTE pdrv);
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count);
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count);
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff);
```

---

## HID Class (Human Interface Device)

### HID Protocol Codes

| Code | Description |
|------|-------------|
| `USB_HID_NONE_PROTOCOL_CODE` (0x00) | None/Custom |
| `USB_HID_KEYBOARD_PROTOCOL_CODE` (0x01) | Keyboard |
| `USB_HID_MOUSE_PROTOCOL_CODE` (0x02) | Mouse |

### HID States

| State | Description |
|-------|-------------|
| `USB_HID_STATE_IDLE` | Idle |
| `USB_HID_STATE_GET_DESC` | Getting HID descriptor |
| `USB_HID_STATE_GET_REPORT` | Getting report |
| `USB_HID_STATE_SET_IDLE` | Setting idle rate |
| `USB_HID_STATE_SET_PROTOCOL` | Setting protocol |
| `USB_HID_STATE_COMPLETE` | Complete |

### HID Process States

| State | Description |
|-------|-------------|
| `USB_HID_INIT` | Initialization |
| `USB_HID_GET` | Getting data |
| `USB_HID_SEND` | Sending data |
| `USB_HID_POLL` | Polling |
| `USB_HID_BUSY` | Busy |
| `USB_HID_ERROR` | Error |

### Keyboard API

```c
/* Keyboard modifier keys */
#define KEYBOARD_LEFT_CTRL   0x01
#define KEYBOARD_LEFT_SHIFT  0x02
#define KEYBOARD_LEFT_ALT    0x04
#define KEYBOARD_LEFT_GUI    0x08
#define KEYBOARD_RIGHT_CTRL  0x10
#define KEYBOARD_RIGHT_SHIFT 0x20
#define KEYBOARD_RIGHT_ALT   0x40
#define KEYBOARD_RIGHT_GUI   0x80

#define KEYBOARD_MAX_NB_PRESSED  6

/* Decode keyboard report */
void usbh_hid_keyboard_decode(uint8_t *data);
```

### Mouse API

```c
/* Mouse button definitions */
#define MOUSE_BUTTON_LEFT   0x00
#define MOUSE_BUTTON_RIGHT  0x01
#define MOUSE_BUTTON_MIDDLE 0x02

/* Mouse data structure */
typedef struct
{
  uint8_t button;  /* Button states */
  uint8_t x;       /* X movement */
  uint8_t y;       /* Y movement */
  uint8_t z;       /* Scroll wheel */
} usb_hid_mouse_type;

/* Decode mouse report */
void usbh_hid_mouse_decode(uint8_t *mouse_data);
```

### HID Class Handler

```c
extern usbh_class_handler_type uhost_hid_class_handler;
```

---

## Class Handler Interface

### Class Handler Structure

```c
typedef struct
{
  usb_sts_type (*init_handler)(void *uhost);     /* Class initialization */
  usb_sts_type (*reset_handler)(void *uhost);    /* Class reset */
  usb_sts_type (*request_handler)(void *uhost);  /* Class-specific requests */
  usb_sts_type (*process_handler)(void *uhost);  /* Class process handler */
  void *pdata;                                   /* Class private data */
} usbh_class_handler_type;
```

---

## Code Examples

### Example 1: HID Host (Keyboard/Mouse)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbh_int.h"
#include "usbh_hid_class.h"
#include "usbh_user.h"

otg_core_type otg_core_struct;

/* User handler callbacks */
static usb_sts_type user_init(void) { return USB_OK; }
static usb_sts_type user_reset(void) { return USB_OK; }
static usb_sts_type user_attached(void)
{
    printf("USB Device Attached\r\n");
    return USB_OK;
}
static usb_sts_type user_disconnect(void)
{
    printf("USB Device Disconnected\r\n");
    return USB_OK;
}
static usb_sts_type user_speed(uint8_t speed)
{
    if (speed == USB_PRTSPD_FULL_SPEED)
        printf("Full-Speed Device\r\n");
    else if (speed == USB_PRTSPD_LOW_SPEED)
        printf("Low-Speed Device\r\n");
    return USB_OK;
}
static usb_sts_type user_mfc_string(void *string)
{
    printf("Manufacturer: %s\r\n", (char *)string);
    return USB_OK;
}
static usb_sts_type user_product_string(void *string)
{
    printf("Product: %s\r\n", (char *)string);
    return USB_OK;
}
static usb_sts_type user_serial_string(void *string)
{
    printf("Serial: %s\r\n", (char *)string);
    return USB_OK;
}
static usb_sts_type user_enumeration_done(void)
{
    printf("Enumeration Done\r\n");
    return USB_OK;
}
static usb_sts_type user_application(void)
{
    /* HID data is handled by HID class callbacks */
    return USB_OK;
}
static usb_sts_type user_active_vbus(void *uhost, confirm_state state)
{
    /* Control VBUS power if needed */
    return USB_OK;
}
static usb_sts_type user_not_support(void)
{
    printf("Device Not Supported\r\n");
    return USB_OK;
}

usbh_user_handler_type usbh_user_handle =
{
    user_init,
    user_reset,
    user_attached,
    user_disconnect,
    user_speed,
    user_mfc_string,
    user_product_string,
    user_serial_string,
    user_enumeration_done,
    user_application,
    user_active_vbus,
    user_not_support,
};

void usb_gpio_config(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);
    
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    
    /* DP and DM */
    gpio_init_struct.gpio_pins = GPIO_PINS_11 | GPIO_PINS_12;
    gpio_init(GPIOA, &gpio_init_struct);
    
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_10);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_10);
}

void usb_clock48m_select(void)
{
    /* Configure USB clock from HEXT */
    switch (system_core_clock)
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

int main(void)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    system_clock_config();
    at32_board_init();
    
    /* Configure USB GPIO */
    usb_gpio_config();
    
    /* Enable USB clock */
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
    usb_clock48m_select();
    
    /* Enable USB interrupt */
    nvic_irq_enable(OTGFS1_IRQn, 0, 0);
    
    /* Initialize USB host with HID class */
    usbh_init(&otg_core_struct,
              USB_FULL_SPEED_CORE_ID,
              USB_ID1,
              &uhost_hid_class_handler,
              &usbh_user_handle);
    
    while (1)
    {
        /* Process USB host state machine */
        usbh_loop_handler(&otg_core_struct.host);
    }
}

void OTGFS1_IRQHandler(void)
{
    usbh_irq_handler(&otg_core_struct);
}

void usb_delay_ms(uint32_t ms)
{
    delay_ms(ms);
}

void usb_delay_us(uint32_t us)
{
    delay_us(us);
}
```

---

### Example 2: MSC Host with FatFS

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbh_int.h"
#include "usbh_msc_class.h"
#include "usbh_user.h"
#include "ff.h"

otg_core_type otg_core_struct;
FATFS fs;
FIL file;

typedef enum
{
    USR_IDLE,
    USR_APP,
    USR_FINISH
} msc_usr_state;

msc_usr_state usr_state = USR_IDLE;

static usb_sts_type user_application(void)
{
    FRESULT res;
    uint32_t bytes_written, bytes_read;
    uint8_t write_data[] = "Hello from AT32F435 USB Host!";
    uint8_t read_data[64] = {0};
    
    switch (usr_state)
    {
        case USR_IDLE:
            usr_state = USR_APP;
            break;
            
        case USR_APP:
            /* Mount file system */
            res = f_mount(&fs, "", 0);
            if (res != FR_OK)
            {
                printf("Mount failed: %d\r\n", res);
                usr_state = USR_FINISH;
                break;
            }
            printf("File system mounted\r\n");
            
            /* Create and write file */
            res = f_open(&file, "0:AT32TEST.TXT", FA_CREATE_ALWAYS | FA_WRITE);
            if (res == FR_OK)
            {
                res = f_write(&file, write_data, sizeof(write_data), &bytes_written);
                if (res == FR_OK && bytes_written > 0)
                {
                    printf("Written %lu bytes\r\n", bytes_written);
                }
                f_close(&file);
            }
            
            /* Read file back */
            res = f_open(&file, "0:AT32TEST.TXT", FA_READ);
            if (res == FR_OK)
            {
                res = f_read(&file, read_data, sizeof(read_data) - 1, &bytes_read);
                if (res == FR_OK && bytes_read > 0)
                {
                    read_data[bytes_read] = '\0';
                    printf("Read: %s\r\n", read_data);
                }
                f_close(&file);
            }
            
            /* Unmount */
            f_mount(NULL, "", 0);
            usr_state = USR_FINISH;
            break;
            
        case USR_FINISH:
            /* Done - wait for device removal */
            break;
    }
    
    return USB_OK;
}

static usb_sts_type user_disconnect(void)
{
    usr_state = USR_IDLE;
    printf("USB Device Disconnected\r\n");
    return USB_OK;
}

/* ... other user handlers same as Example 1 ... */

int main(void)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    system_clock_config();
    at32_board_init();
    
    usb_gpio_config();
    
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
    usb_clock48m_select();
    nvic_irq_enable(OTGFS1_IRQn, 0, 0);
    
    /* Initialize USB host with MSC class */
    usbh_init(&otg_core_struct,
              USB_FULL_SPEED_CORE_ID,
              USB_ID1,
              &uhost_msc_class_handler,
              &usbh_user_handle);
    
    while (1)
    {
        usbh_loop_handler(&otg_core_struct.host);
    }
}
```

---

### Example 3: CDC Host (Virtual COM Port)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbh_int.h"
#include "usbh_cdc_class.h"
#include "usbh_user.h"

otg_core_type otg_core_struct;
uint8_t tx_buffer[64] = "AT32 USB Host CDC Test\r\n";
uint8_t rx_buffer[64];

/* Called when transmission completes */
void cdc_transmit_complete(usbh_core_type *uhost)
{
    printf("TX Complete\r\n");
    
    /* Start reception after transmission */
    cdc_start_reception(&otg_core_struct.host, rx_buffer, sizeof(rx_buffer));
}

/* Called when reception completes */
void cdc_receive_complete(usbh_core_type *uhost)
{
    usbh_cdc_type *pcdc = (usbh_cdc_type *)uhost->class_handler->pdata;
    uint32_t rx_len = uhost->hch[pcdc->data_interface.in_channel].trans_count;
    
    if (rx_len > 0)
    {
        printf("RX (%lu bytes): ", rx_len);
        for (uint32_t i = 0; i < rx_len; i++)
        {
            printf("%02X ", rx_buffer[i]);
        }
        printf("\r\n");
    }
    
    /* Continue reception */
    cdc_start_reception(&otg_core_struct.host, rx_buffer, sizeof(rx_buffer));
}

static usb_sts_type user_enumeration_done(void)
{
    printf("CDC Device Ready\r\n");
    return USB_OK;
}

static usb_sts_type user_application(void)
{
    static uint8_t started = 0;
    
    /* Start communication once when device is ready */
    if (!started)
    {
        started = 1;
        
        /* Configure line coding (115200, 8N1) */
        usbh_cdc.linecoding.line_coding_b.data_baudrate = 115200;
        usbh_cdc.linecoding.line_coding_b.data_bits = 8;
        usbh_cdc.linecoding.line_coding_b.char_format = 0;  /* 1 stop bit */
        usbh_cdc.linecoding.line_coding_b.parity_type = 0;  /* No parity */
        
        /* Start transmission */
        cdc_start_transmission(&otg_core_struct.host, tx_buffer, strlen((char *)tx_buffer));
    }
    
    return USB_OK;
}

int main(void)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    system_clock_config();
    at32_board_init();
    
    usb_gpio_config();
    
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
    usb_clock48m_select();
    nvic_irq_enable(OTGFS1_IRQn, 0, 0);
    
    /* Initialize USB host with CDC class */
    usbh_init(&otg_core_struct,
              USB_FULL_SPEED_CORE_ID,
              USB_ID1,
              &uhost_cdc_class_handler,
              &usbh_user_handle);
    
    while (1)
    {
        usbh_loop_handler(&otg_core_struct.host);
        
        /* Button press triggers new transmission */
        if (at32_button_press() == USER_BUTTON)
        {
            cdc_start_transmission(&otg_core_struct.host, tx_buffer, strlen((char *)tx_buffer));
        }
    }
}
```

---

### Example 4: Dual USB Host (Two OTG Controllers)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbh_int.h"
#include "usbh_msc_class.h"
#include "usbh_hid_class.h"
#include "usbh_user.h"
#include "ff.h"

/* Two separate USB OTG cores */
otg_core_type otg1_core_struct;  /* MSC on OTG1 */
otg_core_type otg2_core_struct;  /* HID on OTG2 */

/* Separate user handlers for each port */
usbh_user_handler_type usbh_user_handle_otg1;
usbh_user_handler_type usbh_user_handle_otg2;

void usb_gpio_config_otg1(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);
    
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = GPIO_PINS_11 | GPIO_PINS_12;
    gpio_init(GPIOA, &gpio_init_struct);
    
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_10);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_10);
}

void usb_gpio_config_otg2(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);
    
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = GPIO_PINS_14 | GPIO_PINS_15;
    gpio_init(GPIOB, &gpio_init_struct);
    
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE14, GPIO_MUX_12);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE15, GPIO_MUX_12);
}

int main(void)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    system_clock_config();
    at32_board_init();
    
    /* Configure GPIO for both OTG controllers */
    usb_gpio_config_otg1();
    usb_gpio_config_otg2();
    
    /* Enable clocks for both USB controllers */
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_OTGFS2_PERIPH_CLOCK, TRUE);
    
    /* Configure USB 48MHz clock */
    crm_usb_clock_div_set(CRM_USB_DIV_3);  /* For 144MHz system clock */
    
    /* Enable interrupts for both controllers */
    nvic_irq_enable(OTGFS1_IRQn, 0, 0);
    nvic_irq_enable(OTGFS2_IRQn, 1, 0);
    
    /* Initialize OTG1 as MSC host */
    usbh_init(&otg1_core_struct,
              USB_FULL_SPEED_CORE_ID,
              USB_ID1,
              &uhost_msc_class_handler,
              &usbh_user_handle_otg1);
    
    /* Initialize OTG2 as HID host */
    usbh_init(&otg2_core_struct,
              USB_FULL_SPEED_CORE_ID,
              USB_ID2,
              &uhost_hid_class_handler,
              &usbh_user_handle_otg2);
    
    while (1)
    {
        /* Process both USB host state machines */
        usbh_loop_handler(&otg1_core_struct.host);
        usbh_loop_handler(&otg2_core_struct.host);
    }
}

void OTGFS1_IRQHandler(void)
{
    usbh_irq_handler(&otg1_core_struct);
}

void OTGFS2_IRQHandler(void)
{
    usbh_irq_handler(&otg2_core_struct);
}
```

---

### Example 5: Host Suspend/Resume

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbh_int.h"
#include "usbh_hid_class.h"
#include "usbh_user.h"

otg_core_type otg_core_struct;

void button_exint_init(void)
{
    exint_init_type exint_init_struct;
    
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE0);
    
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_0;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);
    
    nvic_irq_enable(EXINT0_IRQn, 0, 0);
}

void EXINT0_IRQHandler(void)
{
    delay_ms(50);  /* Debounce */
    exint_flag_clear(EXINT_LINE_0);
    
    /* Toggle suspend/resume on button press */
    if (otg_core_struct.host.global_state == USBH_CLASS)
    {
        /* Enter suspend mode */
        otg_core_struct.host.global_state = USBH_SUSPEND;
        printf("Entering Suspend Mode\r\n");
    }
    else if (otg_core_struct.host.global_state == USBH_SUSPENDED)
    {
        /* Wake up from suspend */
        otg_core_struct.host.global_state = USBH_WAKEUP;
        printf("Waking Up\r\n");
    }
}

#ifdef USB_LOW_POWER_WAKUP
void usb_low_power_wakeup_config(void)
{
    exint_init_type exint_init_struct;
    
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    exint_default_para_init(&exint_init_struct);
    
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_18;  /* OTG1 wakeup line */
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);
    
    nvic_irq_enable(OTGFS1_WKUP_IRQn, 0, 0);
}

void OTGFS1_WKUP_IRQHandler(void)
{
    exint_flag_clear(EXINT_LINE_18);
    printf("USB Wakeup Event\r\n");
}
#endif

int main(void)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    system_clock_config();
    at32_board_init();
    
    button_exint_init();
    usb_gpio_config();
    
#ifdef USB_LOW_POWER_WAKUP
    usb_low_power_wakeup_config();
#endif
    
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
    usb_clock48m_select();
    nvic_irq_enable(OTGFS1_IRQn, 0, 0);
    
    usbh_init(&otg_core_struct,
              USB_FULL_SPEED_CORE_ID,
              USB_ID1,
              &uhost_hid_class_handler,
              &usbh_user_handle);
    
    while (1)
    {
        usbh_loop_handler(&otg_core_struct.host);
    }
}
```

---

## Configuration Checklist

### Hardware Setup
- [ ] Configure USB D+ and D- pins as alternate function
- [ ] Configure VBUS sensing pin (if used)
- [ ] Configure ID pin for OTG mode (if used)
- [ ] Configure VBUS power switch GPIO (if external power control)
- [ ] Ensure proper USB connector and ESD protection

### Clock Configuration
- [ ] Enable USB peripheral clock
- [ ] Configure USB 48MHz clock source (HEXT division or HICK with ACC)
- [ ] Verify 48MHz ±0.25% accuracy for USB compliance

### Interrupt Configuration
- [ ] Enable USB OTG interrupt in NVIC
- [ ] Configure wakeup interrupt (if low-power mode used)
- [ ] Set appropriate interrupt priorities

### Software Configuration
- [ ] Initialize USB host core with class handler
- [ ] Implement user handler callbacks
- [ ] Call `usbh_loop_handler()` in main loop
- [ ] Implement delay functions (`usb_delay_ms`, `usb_delay_us`)

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Device not detected | VBUS not enabled | Check VBUS power supply and enable VBUS |
| Enumeration fails | Clock frequency error | Verify 48MHz USB clock accuracy |
| Device disconnects randomly | ESD/EMI issues | Add proper ESD protection, check cable quality |
| Transfer errors | Interrupt latency | Increase USB interrupt priority |
| HID device not working | Unsupported protocol | Check if device uses Boot protocol |
| MSC mount fails | FAT format issue | Ensure device is formatted as FAT32 |
| CDC no data | Line coding mismatch | Match baud rate and format settings |
| Low-power wakeup fails | EXINT not configured | Enable wakeup EXINT line |

---

## Related Peripherals

| Peripheral | Relationship |
|------------|--------------|
| GPIO | USB pin configuration |
| CRM | USB clock configuration |
| EXINT | USB wakeup, button interrupts |
| NVIC | Interrupt priority management |
| PWC | Low-power mode with USB wakeup |
| DMA | Not directly used (FIFO-based) |

---

## References

- AT32F435/437 Reference Manual - USB OTG Chapter
- USB 2.0 Specification
- USB CDC Class Specification
- USB MSC Class (Bulk-Only Transport)
- USB HID Class Specification
- Application Note AN0094 - USB Examples

---

## Middleware File Structure

```
middlewares/
├── usb_drivers/
│   ├── inc/
│   │   ├── usb_conf.h         # USB configuration
│   │   ├── usb_core.h         # Core driver header
│   │   ├── usb_std.h          # USB standard definitions
│   │   ├── usbh_core.h        # Host core header
│   │   └── usbh_int.h         # Host interrupt handler
│   └── src/
│       ├── usb_core.c         # Core driver implementation
│       ├── usbh_core.c        # Host core implementation
│       └── usbh_int.c         # Host interrupt handler
│
└── usbh_class/
    ├── usbh_cdc/
    │   ├── usbh_cdc_class.h   # CDC class header
    │   └── usbh_cdc_class.c   # CDC class implementation
    ├── usbh_hid/
    │   ├── usbh_hid_class.h   # HID class header
    │   ├── usbh_hid_class.c   # HID class implementation
    │   ├── usbh_hid_keyboard.h
    │   ├── usbh_hid_keyboard.c
    │   ├── usbh_hid_mouse.h
    │   └── usbh_hid_mouse.c
    └── usbh_msc/
        ├── usbh_msc_class.h   # MSC class header
        ├── usbh_msc_class.c   # MSC class implementation
        ├── usbh_msc_bot_scsi.h
        └── usbh_msc_bot_scsi.c
```


