---
title: CAN (Controller Area Network)
mcu: AT32F435/437
peripheral: CAN
version: 2.0.9
---

# CAN Controller Area Network

## Overview

The AT32F435/437 features two independent CAN controllers (CAN1, CAN2) compliant with CAN 2.0A and CAN 2.0B specifications. Each controller supports communication rates up to 1 Mbps and provides robust message filtering with 28 configurable filter banks.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         CAN Controller Block                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────┐     ┌──────────────────────────────────────────────────┐  │
│  │   APB Bus    │────▶│              Control & Status                    │  │
│  │  Interface   │     │  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐    │  │
│  └──────────────┘     │  │ MCTRL  │ │  MSTS  │ │  TSTS  │ │  ESTS  │    │  │
│         │             │  └────────┘ └────────┘ └────────┘ └────────┘    │  │
│         ▼             └──────────────────────────────────────────────────┘  │
│  ┌──────────────┐                                                           │
│  │   Bit Timing │     ┌──────────────────────────────────────────────────┐  │
│  │   Generator  │────▶│              TX Mailboxes (3)                    │  │
│  │    (BTMG)    │     │  ┌──────────┐ ┌──────────┐ ┌──────────┐         │  │
│  └──────────────┘     │  │ Mailbox 0│ │ Mailbox 1│ │ Mailbox 2│         │  │
│         │             │  │ TMI/TMC  │ │ TMI/TMC  │ │ TMI/TMC  │         │  │
│         ▼             │  │ TMDTL/H  │ │ TMDTL/H  │ │ TMDTL/H  │         │  │
│  ┌──────────────┐     │  └──────────┘ └──────────┘ └──────────┘         │  │
│  │   CAN Core   │     └──────────────────────────────────────────────────┘  │
│  │  Protocol    │              │                                            │
│  │   Engine     │              ▼ Priority Scheduler                         │
│  └──────────────┘     ┌──────────────────────────────────────────────────┐  │
│         │             │              Filter Bank (0-27)                   │  │
│         │             │  ┌────────────────────────────────────────────┐  │  │
│         ▼             │  │  32-bit Mask Mode   │  32-bit List Mode   │  │  │
│  ┌──────────────┐     │  │  16-bit Mask Mode   │  16-bit List Mode   │  │  │
│  │  CAN_TX Pin  │◀───│  └────────────────────────────────────────────┘  │  │
│  │  CAN_RX Pin  │────▶│              ▼                                   │  │
│  └──────────────┘     │  ┌──────────────────────────────────────────────┐│  │
│                       │  │         RX FIFOs (2 x 3 messages)            ││  │
│                       │  │  ┌───────────┐         ┌───────────┐        ││  │
│                       │  │  │  FIFO 0   │         │  FIFO 1   │        ││  │
│                       │  │  │ RFI/RFC   │         │ RFI/RFC   │        ││  │
│                       │  │  │ RFDTL/H   │         │ RFDTL/H   │        ││  │
│                       │  │  └───────────┘         └───────────┘        ││  │
│                       │  └──────────────────────────────────────────────┘│  │
│                       └──────────────────────────────────────────────────┘  │
│                                                                              │
│                       ┌──────────────────────────────────────────────────┐  │
│                       │              Interrupt Controller                 │  │
│                       │  TX Complete │ RX Message │ Error │ Mode Change  │  │
│                       └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Key Features

| Feature | Specification |
|---------|---------------|
| CAN Controllers | 2 (CAN1, CAN2) |
| Protocol | CAN 2.0A/2.0B |
| Max Baudrate | 1 Mbps |
| TX Mailboxes | 3 per controller |
| RX FIFOs | 2 per controller (3 messages each) |
| Filter Banks | 28 total (can be split between CAN1/CAN2) |
| Filter Modes | ID Mask, ID List |
| Filter Width | 16-bit, 32-bit |
| Error Handling | TEC/REC counters, bus-off recovery |

---

## Operating Modes

### Mode Types

| Mode | Enum Value | Description |
|------|------------|-------------|
| Communication | `CAN_MODE_COMMUNICATE` | Normal TX/RX on bus |
| Loopback | `CAN_MODE_LOOPBACK` | Internal loopback for testing |
| Listen-Only | `CAN_MODE_LISTENONLY` | Receive only, no ACK |
| Loopback + Listen | `CAN_MODE_LISTENONLY_LOOPBACK` | Combined mode |

### Power Modes

| Mode | Enum Value | Description |
|------|------------|-------------|
| Freeze | `CAN_OPERATINGMODE_FREEZE` | Initialization mode |
| Doze | `CAN_OPERATINGMODE_DOZE` | Low-power sleep |
| Communicate | `CAN_OPERATINGMODE_COMMUNICATE` | Active mode |

---

## Bit Timing & Baud Rate

### Bit Timing Segments

```
                          Nominal Bit Time
├───────────────────────────────────────────────────────────────────┤
│ SYNC_SEG │        BTS1 (1-16 TQ)        │  BTS2 (1-8 TQ)  │ RSAW  │
│  (1 TQ)  │    Propagation + Phase1      │     Phase2      │       │
├──────────┼──────────────────────────────┼─────────────────┼───────┤
                            ▲
                     Sample Point
```

### Baud Rate Calculation

```
Baudrate = PCLK / (baudrate_div × (1 + BTS1 + BTS2))
```

**Example:** For 500 kbps with PCLK = 144 MHz:
```
500,000 = 144,000,000 / (12 × (1 + 8 + 3))
500,000 = 144,000,000 / (12 × 12)
500,000 = 144,000,000 / 144 ✓
```

### Time Quantum Options

| BTS1 Options | BTS2 Options | RSAW Options |
|--------------|--------------|--------------|
| `CAN_BTS1_1TQ` to `CAN_BTS1_16TQ` | `CAN_BTS2_1TQ` to `CAN_BTS2_8TQ` | `CAN_RSAW_1TQ` to `CAN_RSAW_4TQ` |

---

## Message Filtering

### Filter Modes

| Mode | Enum | Description |
|------|------|-------------|
| ID Mask | `CAN_FILTER_MODE_ID_MASK` | ID & Mask comparison (accepts range) |
| ID List | `CAN_FILTER_MODE_ID_LIST` | Exact ID matching (accepts specific IDs) |

### Filter Width

| Width | Enum | IDs per Filter |
|-------|------|----------------|
| 32-bit | `CAN_FILTER_32BIT` | 1 (Mask) or 2 (List) |
| 16-bit | `CAN_FILTER_16BIT` | 2 (Mask) or 4 (List) |

### Filter Register Layout

#### 32-bit Mode (Extended ID)

```
┌────────────────────────────────────────────────────────────────┐
│ filter_id_high (16-bit)  │ filter_id_low (16-bit)              │
│ EID[28:13]               │ EID[12:0] << 3 | IDE | RTR | 0     │
└────────────────────────────────────────────────────────────────┘
```

#### 32-bit Mode (Standard ID)

```
┌────────────────────────────────────────────────────────────────┐
│ filter_id_high (16-bit)  │ filter_id_low (16-bit)              │
│ SID[10:0] << 5           │ 0                                   │
└────────────────────────────────────────────────────────────────┘
```

### ID Calculation Helpers

```c
/* Extended ID (29-bit) to filter registers */
#define EXT_ID_TO_FILTER_HIGH(id)   ((((id) << 3) >> 16) & 0xFFFF)
#define EXT_ID_TO_FILTER_LOW(id)    (((id) << 3) & 0xFFFF) | 0x04  /* IDE bit set */

/* Standard ID (11-bit) to filter registers */
#define STD_ID_TO_FILTER_HIGH(id)   ((id) << 5)
#define STD_ID_TO_FILTER_LOW(id)    (0)
```

---

## API Reference

### Initialization Functions

| Function | Description |
|----------|-------------|
| `can_reset(can_x)` | Reset CAN peripheral |
| `can_default_para_init(&struct)` | Initialize base config to defaults |
| `can_base_init(can_x, &struct)` | Configure CAN base parameters |
| `can_baudrate_default_para_init(&struct)` | Initialize baudrate config |
| `can_baudrate_set(can_x, &struct)` | Set CAN baud rate |
| `can_filter_default_para_init(&struct)` | Initialize filter config |
| `can_filter_init(can_x, &struct)` | Configure message filter |

### Transmission Functions

| Function | Description |
|----------|-------------|
| `can_message_transmit(can_x, &tx_msg)` | Queue message for TX (returns mailbox) |
| `can_transmit_status_get(can_x, mailbox)` | Get TX mailbox status |
| `can_transmit_cancel(can_x, mailbox)` | Cancel pending transmission |

### Reception Functions

| Function | Description |
|----------|-------------|
| `can_message_receive(can_x, fifo, &rx_msg)` | Read message from FIFO |
| `can_receive_fifo_release(can_x, fifo)` | Release FIFO (pop message) |
| `can_receive_message_pending_get(can_x, fifo)` | Get message count in FIFO |

### Mode Control Functions

| Function | Description |
|----------|-------------|
| `can_operating_mode_set(can_x, mode)` | Set operating mode |
| `can_doze_mode_enter(can_x)` | Enter low-power mode |
| `can_doze_mode_exit(can_x)` | Exit low-power mode |
| `can_debug_transmission_prohibit(can_x, state)` | Freeze TX in debug |

### Error Handling Functions

| Function | Description |
|----------|-------------|
| `can_error_type_record_get(can_x)` | Get last error type |
| `can_receive_error_counter_get(can_x)` | Get REC value |
| `can_transmit_error_counter_get(can_x)` | Get TEC value |

### Interrupt Functions

| Function | Description |
|----------|-------------|
| `can_interrupt_enable(can_x, int_type, state)` | Enable/disable interrupt |
| `can_interrupt_flag_get(can_x, flag)` | Check interrupt flag |
| `can_flag_get(can_x, flag)` | Check status flag |
| `can_flag_clear(can_x, flag)` | Clear status flag |

---

## Data Structures

### Base Configuration

```c
typedef struct {
  can_mode_type mode_selection;              /* CAN_MODE_COMMUNICATE, etc. */
  confirm_state ttc_enable;                  /* Time-triggered communication */
  confirm_state aebo_enable;                 /* Auto exit bus-off */
  confirm_state aed_enable;                  /* Auto exit doze mode */
  confirm_state prsf_enable;                 /* Prohibit retransmit on fail */
  can_msg_discarding_rule_type mdrsel_selection;  /* Overflow discard rule */
  can_msg_sending_rule_type mmssr_selection;      /* TX priority rule */
} can_base_type;
```

### Baud Rate Configuration

```c
typedef struct {
  uint16_t baudrate_div;      /* Prescaler: 1-4096 */
  can_rsaw_type rsaw_size;    /* Resync adjust width: 1-4 TQ */
  can_bts1_type bts1_size;    /* Bit segment 1: 1-16 TQ */
  can_bts2_type bts2_size;    /* Bit segment 2: 1-8 TQ */
} can_baudrate_type;
```

### Filter Configuration

```c
typedef struct {
  confirm_state filter_activate_enable;      /* Enable this filter */
  can_filter_mode_type filter_mode;          /* Mask or List mode */
  can_filter_fifo_type filter_fifo;          /* Route to FIFO0 or FIFO1 */
  uint8_t filter_number;                     /* Filter index: 0-27 */
  can_filter_bit_width_type filter_bit;      /* 16-bit or 32-bit */
  uint16_t filter_id_high;                   /* ID register high */
  uint16_t filter_id_low;                    /* ID register low */
  uint16_t filter_mask_high;                 /* Mask/ID2 register high */
  uint16_t filter_mask_low;                  /* Mask/ID2 register low */
} can_filter_init_type;
```

### TX Message Structure

```c
typedef struct {
  uint32_t standard_id;          /* 11-bit standard ID (0-0x7FF) */
  uint32_t extended_id;          /* 29-bit extended ID (0-0x1FFFFFFF) */
  can_identifier_type id_type;   /* CAN_ID_STANDARD or CAN_ID_EXTENDED */
  can_trans_frame_type frame_type;  /* CAN_TFT_DATA or CAN_TFT_REMOTE */
  uint8_t dlc;                   /* Data length: 0-8 */
  uint8_t data[8];               /* Payload data */
} can_tx_message_type;
```

### RX Message Structure

```c
typedef struct {
  uint32_t standard_id;          /* Received standard ID */
  uint32_t extended_id;          /* Received extended ID */
  can_identifier_type id_type;   /* ID type indicator */
  can_trans_frame_type frame_type;  /* Frame type indicator */
  uint8_t dlc;                   /* Received data length */
  uint8_t data[8];               /* Received payload */
  uint8_t filter_index;          /* Matching filter number */
} can_rx_message_type;
```

---

## Interrupt Flags

### Available Interrupts

| Interrupt | Macro | Description |
|-----------|-------|-------------|
| TX Complete | `CAN_TCIEN_INT` | Transmission completed |
| RX FIFO0 Message | `CAN_RF0MIEN_INT` | Message available in FIFO0 |
| RX FIFO0 Full | `CAN_RF0FIEN_INT` | FIFO0 is full |
| RX FIFO0 Overflow | `CAN_RF0OIEN_INT` | FIFO0 overflow occurred |
| RX FIFO1 Message | `CAN_RF1MIEN_INT` | Message available in FIFO1 |
| RX FIFO1 Full | `CAN_RF1FIEN_INT` | FIFO1 is full |
| RX FIFO1 Overflow | `CAN_RF1OIEN_INT` | FIFO1 overflow occurred |
| Error Active | `CAN_EAIEN_INT` | Error active state |
| Error Passive | `CAN_EPIEN_INT` | Error passive state |
| Bus-Off | `CAN_BOIEN_INT` | Bus-off condition |
| Error Type Record | `CAN_ETRIEN_INT` | Error type recorded |
| Error Occur | `CAN_EOIEN_INT` | Error occurrence |
| Quit Doze | `CAN_QDZIEN_INT` | Exited doze mode |
| Enter Doze | `CAN_EDZIEN_INT` | Entered doze mode |

### Status Flags

| Flag | Macro | Description |
|------|-------|-------------|
| Error Active | `CAN_EAF_FLAG` | In error active state |
| Error Passive | `CAN_EPF_FLAG` | In error passive state |
| Bus-Off | `CAN_BOF_FLAG` | In bus-off state |
| Error Type | `CAN_ETR_FLAG` | Error type recorded |
| TX Mailbox Empty | `CAN_TMEF_FLAG` | TX mailbox available |
| RX FIFO0 Message | `CAN_RF0MN_FLAG` | Messages pending in FIFO0 |
| RX FIFO1 Message | `CAN_RF1MN_FLAG` | Messages pending in FIFO1 |

---

## Error Types

| Error | Enum | Description |
|-------|------|-------------|
| No Error | `CAN_ERRORRECORD_NOERR` | No error detected |
| Stuff Error | `CAN_ERRORRECORD_STUFFERR` | Bit stuffing violation |
| Form Error | `CAN_ERRORRECORD_FORMERR` | Fixed form bit error |
| ACK Error | `CAN_ERRORRECORD_ACKERR` | No acknowledgment received |
| Bit Recessive | `CAN_ERRORRECORD_BITRECESSIVEERR` | Recessive bit error |
| Bit Dominant | `CAN_ERRORRECORD_BITDOMINANTERR` | Dominant bit error |
| CRC Error | `CAN_ERRORRECORD_CRCERR` | CRC mismatch |
| Software Set | `CAN_ERRORRECORD_SOFTWARESETERR` | Software-triggered error |

---

## Complete Examples

### Example 1: Communication Mode (Normal TX/RX)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * CAN Communication Mode Example
 * 
 * Demonstrates normal CAN bus communication with interrupt-driven reception.
 * Transmits 8-byte messages and receives via FIFO0 interrupt.
 * 
 * Hardware: CAN1 on PB8 (RX) / PB9 (TX) via GPIO_MUX_9
 * Baudrate: 500 kbps (with 144 MHz PCLK)
 ******************************************************************************/

/* GPIO Configuration for CAN pins */
static void can_gpio_config(void)
{
  gpio_init_type gpio_init_struct;
  
  /* Enable GPIO clock */
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);

  /* Configure CAN TX (PB9) and RX (PB8) pins */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* Map pins to CAN1 alternate function */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE8, GPIO_MUX_9);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9, GPIO_MUX_9);
}

/* CAN Controller Configuration */
error_status can_configuration(void)
{
  can_base_type can_base_struct;
  can_baudrate_type can_baudrate_struct;
  can_filter_init_type can_filter_init_struct;
  
  /* IMPORTANT: CAN requires accurate clock source (HEXT recommended)
     HICK oscillator tolerance exceeds CAN protocol's 1.58% limit */
  if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
  {
    return ERROR;  /* Clock source not suitable for CAN */
  }
  
  /* Enable CAN peripheral clock */
  crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);

  /* Configure CAN base parameters */
  can_default_para_init(&can_base_struct);
  can_base_struct.mode_selection = CAN_MODE_COMMUNICATE;  /* Normal mode */
  can_base_struct.ttc_enable = FALSE;           /* No time-triggered */
  can_base_struct.aebo_enable = TRUE;           /* Auto exit bus-off */
  can_base_struct.aed_enable = TRUE;            /* Auto exit doze */
  can_base_struct.prsf_enable = FALSE;          /* Allow retransmission */
  can_base_struct.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
  can_base_struct.mmssr_selection = CAN_SENDING_BY_ID;  /* TX by ID priority */
  can_base_init(CAN1, &can_base_struct);

  /* Configure baud rate: 500 kbps
     Formula: Baudrate = PCLK / (div × (1 + BTS1 + BTS2))
     500k = 144M / (12 × (1 + 8 + 3)) = 144M / 144 */
  can_baudrate_struct.baudrate_div = 12;
  can_baudrate_struct.rsaw_size = CAN_RSAW_3TQ;
  can_baudrate_struct.bts1_size = CAN_BTS1_8TQ;
  can_baudrate_struct.bts2_size = CAN_BTS2_3TQ;
  if(can_baudrate_set(CAN1, &can_baudrate_struct) != SUCCESS)
  {
    return ERROR;
  }

  /* Configure filter 0: Accept all messages (mask = 0) */
  can_filter_init_struct.filter_activate_enable = TRUE;
  can_filter_init_struct.filter_mode = CAN_FILTER_MODE_ID_MASK;
  can_filter_init_struct.filter_fifo = CAN_FILTER_FIFO0;
  can_filter_init_struct.filter_number = 0;
  can_filter_init_struct.filter_bit = CAN_FILTER_32BIT;
  can_filter_init_struct.filter_id_high = 0;
  can_filter_init_struct.filter_id_low = 0;
  can_filter_init_struct.filter_mask_high = 0;  /* All bits = don't care */
  can_filter_init_struct.filter_mask_low = 0;
  can_filter_init(CAN1, &can_filter_init_struct);

  /* Enable interrupts */
  nvic_irq_enable(CAN1_SE_IRQn, 0x00, 0x00);
  nvic_irq_enable(CAN1_RX0_IRQn, 0x00, 0x00);
  can_interrupt_enable(CAN1, CAN_RF0MIEN_INT, TRUE);   /* RX message */
  can_interrupt_enable(CAN1, CAN_ETRIEN_INT, TRUE);    /* Error type */
  can_interrupt_enable(CAN1, CAN_EOIEN_INT, TRUE);     /* Error occur */
  
  return SUCCESS;
}

/* Transmit a CAN message */
static void can_transmit_data(void)
{
  uint8_t transmit_mailbox;
  can_tx_message_type tx_message_struct;
  
  /* Configure message */
  tx_message_struct.standard_id = 0x400;
  tx_message_struct.extended_id = 0;
  tx_message_struct.id_type = CAN_ID_STANDARD;
  tx_message_struct.frame_type = CAN_TFT_DATA;
  tx_message_struct.dlc = 8;
  tx_message_struct.data[0] = 0x11;
  tx_message_struct.data[1] = 0x22;
  tx_message_struct.data[2] = 0x33;
  tx_message_struct.data[3] = 0x44;
  tx_message_struct.data[4] = 0x55;
  tx_message_struct.data[5] = 0x66;
  tx_message_struct.data[6] = 0x77;
  tx_message_struct.data[7] = 0x88;
  
  /* Send message and wait for completion */
  transmit_mailbox = can_message_transmit(CAN1, &tx_message_struct);
  while(can_transmit_status_get(CAN1, (can_tx_mailbox_num_type)transmit_mailbox) 
        != CAN_TX_STATUS_SUCCESSFUL);
}

/* RX FIFO0 Interrupt Handler */
void CAN1_RX0_IRQHandler(void)
{
  can_rx_message_type rx_message_struct;
  
  if(can_interrupt_flag_get(CAN1, CAN_RF0MN_FLAG) != RESET)
  {
    can_message_receive(CAN1, CAN_RX_FIFO0, &rx_message_struct);
    
    /* Process received message based on ID */
    if(rx_message_struct.standard_id == 0x400)
    {
      at32_led_toggle(LED2);  /* Expected message */
    }
    else
    {
      at32_led_toggle(LED3);  /* Other message */
    }
  }
}

/* Error Status Interrupt Handler */
void CAN1_SE_IRQHandler(void)
{
  __IO uint32_t err_index = 0;
  
  if(can_interrupt_flag_get(CAN1, CAN_ETR_FLAG) != RESET)
  {
    err_index = CAN1->ests & 0x70;
    can_flag_clear(CAN1, CAN_ETR_FLAG);
    
    /* Handle stuff error (common on bus issues) */
    if(err_index == 0x00000010)
    {
      /* Recovery: restart CAN or send highest priority frame */
    }
  }
}

/* Main Function */
int main(void)
{
  system_clock_config();
  at32_board_init();
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  
  can_gpio_config();
  
  if(can_configuration() == ERROR)
  {
    /* Clock initialization error - halt */
    while(1) { }
  }
  
  while(1)
  {
    can_transmit_data();
    at32_led_toggle(LED4);
    delay_sec(1);
  }
}
```

---

### Example 2: Loopback Mode (Self-Test)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * CAN Loopback Mode Example
 * 
 * Internal loopback for testing without external CAN transceiver.
 * TX messages are internally routed to RX - no bus connection needed.
 * 
 * Use case: Hardware validation, software debugging
 ******************************************************************************/

static void can_gpio_config(void)
{
  gpio_init_type gpio_init_struct;
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE8, GPIO_MUX_9);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9, GPIO_MUX_9);
}

error_status can_configuration(void)
{
  can_base_type can_base_struct;
  can_baudrate_type can_baudrate_struct;
  can_filter_init_type can_filter_init_struct;
  
  if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
  {
    return ERROR;
  }
  
  crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);

  can_default_para_init(&can_base_struct);
  /* KEY DIFFERENCE: Loopback mode for internal testing */
  can_base_struct.mode_selection = CAN_MODE_LOOPBACK;
  can_base_struct.ttc_enable = FALSE;
  can_base_struct.aebo_enable = TRUE;
  can_base_struct.aed_enable = TRUE;
  can_base_struct.prsf_enable = FALSE;
  can_base_struct.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
  can_base_struct.mmssr_selection = CAN_SENDING_BY_ID;
  can_base_init(CAN1, &can_base_struct);

  can_baudrate_struct.baudrate_div = 12;
  can_baudrate_struct.rsaw_size = CAN_RSAW_3TQ;
  can_baudrate_struct.bts1_size = CAN_BTS1_8TQ;
  can_baudrate_struct.bts2_size = CAN_BTS2_3TQ;
  if(can_baudrate_set(CAN1, &can_baudrate_struct) != SUCCESS)
  {
    return ERROR;
  }

  can_filter_init_struct.filter_activate_enable = TRUE;
  can_filter_init_struct.filter_mode = CAN_FILTER_MODE_ID_MASK;
  can_filter_init_struct.filter_fifo = CAN_FILTER_FIFO0;
  can_filter_init_struct.filter_number = 0;
  can_filter_init_struct.filter_bit = CAN_FILTER_32BIT;
  can_filter_init_struct.filter_id_high = 0;
  can_filter_init_struct.filter_id_low = 0;
  can_filter_init_struct.filter_mask_high = 0;
  can_filter_init_struct.filter_mask_low = 0;
  can_filter_init(CAN1, &can_filter_init_struct);

  nvic_irq_enable(CAN1_SE_IRQn, 0x00, 0x00);
  nvic_irq_enable(CAN1_RX0_IRQn, 0x00, 0x00);
  can_interrupt_enable(CAN1, CAN_RF0MIEN_INT, TRUE);
  can_interrupt_enable(CAN1, CAN_ETRIEN_INT, TRUE);
  can_interrupt_enable(CAN1, CAN_EOIEN_INT, TRUE);
  
  return SUCCESS;
}

static void can_transmit_data(void)
{
  uint8_t transmit_mailbox;
  can_tx_message_type tx_message_struct;
  
  tx_message_struct.standard_id = 0x400;
  tx_message_struct.extended_id = 0;
  tx_message_struct.id_type = CAN_ID_STANDARD;
  tx_message_struct.frame_type = CAN_TFT_DATA;
  tx_message_struct.dlc = 8;
  tx_message_struct.data[0] = 0x11;
  tx_message_struct.data[1] = 0x22;
  tx_message_struct.data[2] = 0x33;
  tx_message_struct.data[3] = 0x44;
  tx_message_struct.data[4] = 0x55;
  tx_message_struct.data[5] = 0x66;
  tx_message_struct.data[6] = 0x77;
  tx_message_struct.data[7] = 0x88;
  
  transmit_mailbox = can_message_transmit(CAN1, &tx_message_struct);
  while(can_transmit_status_get(CAN1, (can_tx_mailbox_num_type)transmit_mailbox) 
        != CAN_TX_STATUS_SUCCESSFUL);
}

/* In loopback mode, TX messages appear in RX FIFO */
void CAN1_RX0_IRQHandler(void)
{
  can_rx_message_type rx_message_struct;
  
  if(can_interrupt_flag_get(CAN1, CAN_RF0MN_FLAG) != RESET)
  {
    can_message_receive(CAN1, CAN_RX_FIFO0, &rx_message_struct);
    
    if(rx_message_struct.standard_id == 0x400)
    {
      at32_led_toggle(LED2);  /* Loopback successful! */
    }
  }
}

void CAN1_SE_IRQHandler(void)
{
  __IO uint32_t err_index = 0;
  
  if(can_interrupt_flag_get(CAN1, CAN_ETR_FLAG) != RESET)
  {
    err_index = CAN1->ests & 0x70;
    can_flag_clear(CAN1, CAN_ETR_FLAG);
    
    if(err_index == 0x00000010)
    {
      /* Stuff error - should not occur in loopback */
    }
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  
  can_gpio_config();
  
  if(can_configuration() == ERROR)
  {
    while(1) { }
  }
  
  while(1)
  {
    can_transmit_data();
    at32_led_toggle(LED4);
    delay_sec(1);
  }
}
```

---

### Example 3: ID List Filter (Accept Specific IDs Only)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * CAN ID List Filter Example
 * 
 * Demonstrates precise message filtering using ID List mode.
 * Only messages with exact matching IDs are accepted.
 * 
 * Filter configuration:
 * - Filter 0: Extended IDs 0x18F5F100, 0x18F5F200 (32-bit list)
 * - Filter 1: Standard IDs 0x04F6, 0x04F7 (32-bit list)
 * 
 * Messages with IDs 0x18F5F300, 0x04F8 will be rejected.
 ******************************************************************************/

/* Extended identifiers (29-bit) */
#define FILTER_EXT_ID1    ((uint32_t)0x18F5F100)  /* Accepted */
#define FILTER_EXT_ID2    ((uint32_t)0x18F5F200)  /* Accepted */
#define FILTER_EXT_ID3    ((uint32_t)0x18F5F300)  /* Rejected by filter */

/* Standard identifiers (11-bit) */
#define FILTER_STD_ID1    ((uint16_t)0x04F6)      /* Accepted */
#define FILTER_STD_ID2    ((uint16_t)0x04F7)      /* Accepted */
#define FILTER_STD_ID3    ((uint16_t)0x04F8)      /* Rejected by filter */

uint8_t test_result = 0;

static void can_gpio_config(void)
{
  gpio_init_type gpio_init_struct;
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE8, GPIO_MUX_9);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9, GPIO_MUX_9);
}

error_status can_configuration(void)
{
  can_base_type can_base_struct;
  can_baudrate_type can_baudrate_struct;
  can_filter_init_type can_filter_init_struct;
  
  if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
  {
    return ERROR;
  }
  
  crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);

  can_default_para_init(&can_base_struct);
  can_base_struct.mode_selection = CAN_MODE_COMMUNICATE;
  can_base_struct.ttc_enable = FALSE;
  can_base_struct.aebo_enable = TRUE;
  can_base_struct.aed_enable = TRUE;
  can_base_struct.prsf_enable = FALSE;
  can_base_struct.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
  can_base_struct.mmssr_selection = CAN_SENDING_BY_ID;
  can_base_init(CAN1, &can_base_struct);

  can_baudrate_struct.baudrate_div = 12;
  can_baudrate_struct.rsaw_size = CAN_RSAW_3TQ;
  can_baudrate_struct.bts1_size = CAN_BTS1_8TQ;
  can_baudrate_struct.bts2_size = CAN_BTS2_3TQ;
  if(can_baudrate_set(CAN1, &can_baudrate_struct) != SUCCESS)
  {
    return ERROR;
  }

  /***************************************************************************
   * Filter 0: Two Extended IDs in 32-bit List Mode
   * 
   * In 32-bit List mode, filter_id holds first ID, filter_mask holds second ID.
   * Extended ID format: (ID << 3) with IDE bit set (bit 2 = 1)
   ***************************************************************************/
  can_filter_init_struct.filter_activate_enable = TRUE;
  can_filter_init_struct.filter_mode = CAN_FILTER_MODE_ID_LIST;  /* Exact match */
  can_filter_init_struct.filter_fifo = CAN_FILTER_FIFO0;
  can_filter_init_struct.filter_number = 0;
  can_filter_init_struct.filter_bit = CAN_FILTER_32BIT;
  /* First Extended ID: 0x18F5F100 */
  can_filter_init_struct.filter_id_high = (((FILTER_EXT_ID1 << 3) >> 16) & 0xFFFF);
  can_filter_init_struct.filter_id_low = ((FILTER_EXT_ID1 << 3) & 0xFFFF) | 0x04;
  /* Second Extended ID: 0x18F5F200 */
  can_filter_init_struct.filter_mask_high = (((FILTER_EXT_ID2 << 3) >> 16) & 0xFFFF);
  can_filter_init_struct.filter_mask_low = ((FILTER_EXT_ID2 << 3) & 0xFFFF) | 0x04;
  can_filter_init(CAN1, &can_filter_init_struct);

  /***************************************************************************
   * Filter 1: Two Standard IDs in 32-bit List Mode
   * 
   * Standard ID format: (ID << 5) in high 16 bits, low 16 bits = 0
   ***************************************************************************/
  can_filter_init_struct.filter_activate_enable = TRUE;
  can_filter_init_struct.filter_mode = CAN_FILTER_MODE_ID_LIST;
  can_filter_init_struct.filter_fifo = CAN_FILTER_FIFO0;
  can_filter_init_struct.filter_number = 1;
  can_filter_init_struct.filter_bit = CAN_FILTER_32BIT;
  /* First Standard ID: 0x04F6 */
  can_filter_init_struct.filter_id_high = FILTER_STD_ID1 << 5;
  can_filter_init_struct.filter_id_low = 0;
  /* Second Standard ID: 0x04F7 */
  can_filter_init_struct.filter_mask_high = FILTER_STD_ID2 << 5;
  can_filter_init_struct.filter_mask_low = 0;
  can_filter_init(CAN1, &can_filter_init_struct);

  nvic_irq_enable(CAN1_SE_IRQn, 0x00, 0x00);
  nvic_irq_enable(CAN1_RX0_IRQn, 0x00, 0x00);
  can_interrupt_enable(CAN1, CAN_RF0MIEN_INT, TRUE);
  can_interrupt_enable(CAN1, CAN_ETRIEN_INT, TRUE);
  can_interrupt_enable(CAN1, CAN_EOIEN_INT, TRUE);
  
  return SUCCESS;
}

/* Transmit messages with different IDs - only 4 should pass filter */
static void can_transmit_data(void)
{
  uint8_t transmit_mailbox;
  can_tx_message_type tx_message_struct;
  
  tx_message_struct.dlc = 8;
  tx_message_struct.data[0] = 0x11;
  tx_message_struct.data[1] = 0x22;
  tx_message_struct.data[2] = 0x33;
  tx_message_struct.data[3] = 0x44;
  tx_message_struct.data[4] = 0x55;
  tx_message_struct.data[5] = 0x66;
  tx_message_struct.data[6] = 0x77;
  tx_message_struct.data[7] = 0x88;

  /* Standard ID 1 - ACCEPTED */
  tx_message_struct.standard_id = FILTER_STD_ID1;
  tx_message_struct.extended_id = 0;
  tx_message_struct.id_type = CAN_ID_STANDARD;
  tx_message_struct.frame_type = CAN_TFT_DATA;
  transmit_mailbox = can_message_transmit(CAN1, &tx_message_struct);
  while(can_transmit_status_get(CAN1, (can_tx_mailbox_num_type)transmit_mailbox) 
        != CAN_TX_STATUS_SUCCESSFUL);

  /* Standard ID 2 - ACCEPTED */
  tx_message_struct.standard_id = FILTER_STD_ID2;
  transmit_mailbox = can_message_transmit(CAN1, &tx_message_struct);
  while(can_transmit_status_get(CAN1, (can_tx_mailbox_num_type)transmit_mailbox) 
        != CAN_TX_STATUS_SUCCESSFUL);

  /* Standard ID 3 - REJECTED (not in filter list) */
  tx_message_struct.standard_id = FILTER_STD_ID3;
  transmit_mailbox = can_message_transmit(CAN1, &tx_message_struct);
  while(can_transmit_status_get(CAN1, (can_tx_mailbox_num_type)transmit_mailbox) 
        != CAN_TX_STATUS_SUCCESSFUL);

  /* Extended ID 1 - ACCEPTED */
  tx_message_struct.standard_id = 0;
  tx_message_struct.extended_id = FILTER_EXT_ID1;
  tx_message_struct.id_type = CAN_ID_EXTENDED;
  transmit_mailbox = can_message_transmit(CAN1, &tx_message_struct);
  while(can_transmit_status_get(CAN1, (can_tx_mailbox_num_type)transmit_mailbox) 
        != CAN_TX_STATUS_SUCCESSFUL);

  /* Extended ID 2 - ACCEPTED */
  tx_message_struct.extended_id = FILTER_EXT_ID2;
  transmit_mailbox = can_message_transmit(CAN1, &tx_message_struct);
  while(can_transmit_status_get(CAN1, (can_tx_mailbox_num_type)transmit_mailbox) 
        != CAN_TX_STATUS_SUCCESSFUL);

  /* Extended ID 3 - REJECTED (not in filter list) */
  tx_message_struct.extended_id = FILTER_EXT_ID3;
  transmit_mailbox = can_message_transmit(CAN1, &tx_message_struct);
  while(can_transmit_status_get(CAN1, (can_tx_mailbox_num_type)transmit_mailbox) 
        != CAN_TX_STATUS_SUCCESSFUL);
}

/* Only 4 messages should be received (2 rejected by filter) */
void CAN1_RX0_IRQHandler(void)
{
  can_rx_message_type rx_message_struct;
  
  if(can_interrupt_flag_get(CAN1, CAN_RF0MN_FLAG) != RESET)
  {
    if(test_result == 4)
    {
      test_result = 0;  /* Reset counter */
    }

    can_message_receive(CAN1, CAN_RX_FIFO0, &rx_message_struct);

    /* Verify received message matches expected filter list */
    if((rx_message_struct.id_type == CAN_ID_STANDARD) && 
       (rx_message_struct.standard_id == FILTER_STD_ID1))
      test_result++;
    else if((rx_message_struct.id_type == CAN_ID_STANDARD) && 
            (rx_message_struct.standard_id == FILTER_STD_ID2))
      test_result++;
    else if((rx_message_struct.id_type == CAN_ID_EXTENDED) && 
            (rx_message_struct.extended_id == FILTER_EXT_ID1))
      test_result++;
    else if((rx_message_struct.id_type == CAN_ID_EXTENDED) && 
            (rx_message_struct.extended_id == FILTER_EXT_ID2))
      test_result++;
  }
}

void CAN1_SE_IRQHandler(void)
{
  __IO uint32_t err_index = 0;
  
  if(can_interrupt_flag_get(CAN1, CAN_ETR_FLAG) != RESET)
  {
    err_index = CAN1->ests & 0x70;
    can_flag_clear(CAN1, CAN_ETR_FLAG);
    
    if(err_index == 0x00000010)
    {
      /* Stuff error handling */
    }
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  
  can_gpio_config();
  
  if(can_configuration() == ERROR)
  {
    while(1) { }
  }
  
  can_transmit_data();  /* Send 6 messages, expect 4 received */
  
  while(1)
  {
    /* test_result should equal 4 (2 messages filtered out) */
    if(test_result == 4)
    {
      at32_led_toggle(LED2);
      at32_led_toggle(LED3);
      at32_led_toggle(LED4);
      delay_sec(1);
    }
  }
}
```

---

## Common Baud Rate Configurations

| Baud Rate | PCLK (MHz) | Divider | BTS1 | BTS2 | Sample Point |
|-----------|------------|---------|------|------|--------------|
| 1 Mbps | 144 | 6 | 14 TQ | 9 TQ | 62.5% |
| 500 kbps | 144 | 12 | 8 TQ | 3 TQ | 75% |
| 250 kbps | 144 | 24 | 8 TQ | 3 TQ | 75% |
| 125 kbps | 144 | 48 | 8 TQ | 3 TQ | 75% |
| 100 kbps | 144 | 60 | 8 TQ | 3 TQ | 75% |
| 50 kbps | 144 | 120 | 8 TQ | 3 TQ | 75% |

---

## Implementation Checklist

### Hardware Setup
- [ ] Connect CAN transceiver (e.g., TJA1050, SN65HVD230)
- [ ] Add 120Ω termination resistor at bus ends
- [ ] Verify HEXT crystal oscillator is stable
- [ ] Configure GPIO pins with proper alternate function

### Software Configuration
- [ ] Enable CRM clocks for CAN and GPIO peripherals
- [ ] Configure GPIO pins in MUX mode with correct AF
- [ ] Initialize CAN base parameters
- [ ] Set baud rate with proper timing segments
- [ ] Configure at least one filter to accept messages
- [ ] Enable NVIC interrupts for RX and errors
- [ ] Verify clock source stability (HEXT required)

### Filter Configuration
- [ ] Choose appropriate filter mode (mask vs list)
- [ ] Calculate ID/mask values with correct bit shifts
- [ ] Assign filters to appropriate FIFOs
- [ ] Activate filters after configuration

---

## Troubleshooting

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| TX stuck pending | No ACK from bus | Check transceiver, termination, other nodes |
| No RX messages | Filter misconfigured | Use mask=0 to accept all, then narrow |
| Stuff errors | Clock mismatch | Verify HEXT stable, check baud rate |
| Bus-off state | Too many errors | Check wiring, termination, ground |
| ACK errors | Only node on bus | Add second node or use loopback mode |
| Form errors | Signal integrity | Check cable length, shielding |

### Clock Source Warning

> ⚠️ **IMPORTANT:** CAN protocol requires clock accuracy within ±1.58%. The internal HICK oscillator does NOT meet this requirement. Always use HEXT (external crystal) for CAN communication.

```c
/* Always verify HEXT before CAN initialization */
if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
{
  return ERROR;  /* Cannot use HICK for CAN */
}
```

---

## GPIO Pin Mapping

| CAN | TX Pin Options | RX Pin Options | AF |
|-----|----------------|----------------|-----|
| CAN1 | PA12, PB9, PD1, PH13 | PA11, PB8, PD0, PH14 | AF9 |
| CAN2 | PB6, PB13 | PB5, PB12 | AF9 |

---

## See Also

- [ADC Documentation](./ADC_Analog_to_Digital_Converter.md)
- [ACC Documentation](./ACC_Auto_Clock_Calibration.md)
- AT32F435_437 Reference Manual - CAN Chapter

