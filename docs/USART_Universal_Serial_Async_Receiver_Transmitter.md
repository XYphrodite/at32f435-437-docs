---
title: USART - Universal Synchronous Asynchronous Receiver Transmitter
category: Peripheral
complexity: Intermediate
mcu: AT32F435/437
peripheral: USART
keywords: [usart, uart, serial, rs232, rs485, irda, smartcard, lin, dma, interrupt, half-duplex]
---

# USART - Universal Synchronous Asynchronous Receiver Transmitter

## Overview

The Universal Synchronous Asynchronous Receiver Transmitter (USART) provides flexible serial communication interfaces supporting asynchronous (UART), synchronous, half-duplex, LIN, IrDA, Smartcard, and RS485 modes. The AT32F435/437 features up to 8 USART/UART interfaces with hardware flow control, multi-processor communication, and DMA support.

### Key Features

| Feature | Specification |
|---------|---------------|
| USART Interfaces | 4 (USART1, USART2, USART3, USART6) |
| UART Interfaces | 4 (UART4, UART5, UART7, UART8*) |
| Data Bits | 7, 8, or 9 bits |
| Stop Bits | 0.5, 1, 1.5, or 2 bits |
| Parity | None, Even, or Odd |
| Hardware Flow Control | RTS/CTS (USART1-3) |
| DMA Support | TX and RX for all interfaces |
| Special Modes | Synchronous, Half-duplex, LIN, IrDA, Smartcard, RS485 |

> **Note:** UART8 is only available on AT32F435Zx/Vx/Rx and AT32F437Zx/Vx/Rx variants.

## Architecture

```
                              USART Block Diagram
    ┌─────────────────────────────────────────────────────────────────────┐
    │                                                                     │
    │  ┌────────────┐   ┌─────────────┐   ┌────────────┐   ┌──────────┐   │
    │  │   Baud     │   │  Transmit   │   │  TX Shift  │──►│   TX     │──►│──► TX Pin
    │  │   Rate     │──►│  Data Reg   │──►│  Register  │   │  Control │   │
    │  │  Generator │   │    (DT)     │   │            │   │          │   │
    │  └────────────┘   └─────────────┘   └────────────┘   └──────────┘   │
    │       │                                                             │
    │       │           ┌─────────────┐   ┌────────────┐   ┌──────────┐   │
    │       │           │  Receive    │◄──│  RX Shift  │◄──│   RX     │◄──│◄── RX Pin
    │       └──────────►│  Data Reg   │   │  Register  │   │  Control │   │
    │                   │    (DT)     │   │            │   │          │   │
    │                   └─────────────┘   └────────────┘   └──────────┘   │
    │                         │                                  ▲        │
    │                         ▼                                  │        │
    │  ┌─────────────────────────────────────────────────────────┴──────┐ │
    │  │                    Control Logic                               │ │
    │  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │ │
    │  │  │  Parity  │  │   LIN    │  │  IrDA    │  │ Smartcard│        │ │
    │  │  │  Check   │  │  Mode    │  │  Mode    │  │   Mode   │        │ │
    │  │  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │ │
    │  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │ │
    │  │  │  RS485   │  │  Half    │  │  Sync    │  │   HW     │        │ │
    │  │  │  Mode    │  │  Duplex  │  │  Mode    │  │ Flow Ctrl│        │ │
    │  │  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │ │
    │  └────────────────────────────────────────────────────────────────┘ │
    │                         │                                           │
    │  ┌──────────────────────┴────────────────────────────────────────┐  │
    │  │                   Interrupt & DMA Control                     │  │
    │  │   TDBE │ TDC │ RDBF │ IDLE │ PERR │ FERR │ NERR │ ROERR │ BF  │  │
    │  └───────────────────────────────────────────────────────────────┘  │
    │                         │               │                           │
    │                         ▼               ▼                           │
    │                      NVIC            DMA                            │
    └─────────────────────────────────────────────────────────────────────┘
```

## USART vs UART Features

| Feature | USART1/2/3/6 | UART4/5/7/8 |
|---------|--------------|-------------|
| Asynchronous Mode | ✓ | ✓ |
| Synchronous Mode | ✓ | ✗ |
| Hardware Flow Control | ✓ (1/2/3 only) | ✗ |
| Smartcard Mode | ✓ | ✗ |
| IrDA Mode | ✓ | ✓ |
| LIN Mode | ✓ | ✓ |
| RS485 Mode | ✓ (1/2/3 only) | ✗ |
| Half-Duplex | ✓ | ✓ |
| Multi-Processor | ✓ | ✓ |
| DMA Support | ✓ | ✓ |

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| STS | 0x00 | Status register |
| DT | 0x04 | Data register |
| BAUDR | 0x08 | Baud rate register |
| CTRL1 | 0x0C | Control register 1 |
| CTRL2 | 0x10 | Control register 2 |
| CTRL3 | 0x14 | Control register 3 |
| GDIV | 0x18 | Guard time and prescaler register |

### Status Register (STS) Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0 | PERR | Parity error |
| 1 | FERR | Framing error |
| 2 | NERR | Noise error |
| 3 | ROERR | Receiver overflow error |
| 4 | IDLEF | Idle frame detected flag |
| 5 | RDBF | Receive data buffer full |
| 6 | TDC | Transmit data complete |
| 7 | TDBE | Transmit data buffer empty |
| 8 | BFF | Break frame flag |
| 9 | CTSCF | CTS change flag |

### Control Register 1 (CTRL1) Key Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0 | SBF | Send break frame |
| 1 | RM | Receiver mute |
| 2 | REN | Receiver enable |
| 3 | TEN | Transmitter enable |
| 4 | IDLEIEN | IDLE interrupt enable |
| 5 | RDBFIEN | RDBF interrupt enable |
| 6 | TDCIEN | TDC interrupt enable |
| 7 | TDBEIEN | TDBE interrupt enable |
| 8 | PERRIEN | Parity error interrupt enable |
| 9 | PSEL | Parity selection (0=even, 1=odd) |
| 10 | PEN | Parity enable |
| 11 | WUM | Wakeup mode |
| 12 | DBN_L | Data bit number (low) |
| 13 | UEN | USART enable |
| 16:20 | TCDT | Transmit complete delay time |
| 21:25 | TSDT | Transmit start delay time |
| 28 | DBN_H | Data bit number (high) |

### Control Register 2 (CTRL2) Key Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0:3 | ID_L | Address of the USART node (low) |
| 4 | IDBN | ID bit number |
| 5 | BFBN | Break frame bit number |
| 6 | BFIEN | Break frame interrupt enable |
| 8 | LBCP | Last bit clock pulse |
| 9 | CLKPHA | Clock phase |
| 10 | CLKPOL | Clock polarity |
| 11 | CLKEN | Clock output enable |
| 12:13 | STOPBN | Stop bit number |
| 14 | LINEN | LIN mode enable |
| 15 | TRPSWAP | TX/RX pin swap |
| 28:31 | ID_H | Address of the USART node (high) |

### Control Register 3 (CTRL3) Key Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0 | ERRIEN | Error interrupt enable |
| 1 | IRDAEN | IrDA mode enable |
| 2 | IRDALP | IrDA low-power |
| 3 | SLBEN | Single-line bidirectional half-duplex |
| 4 | SCNACKEN | Smartcard NACK enable |
| 5 | SCMEN | Smartcard mode enable |
| 6 | DMAREN | DMA receiver enable |
| 7 | DMATEN | DMA transmitter enable |
| 8 | RTSEN | RTS enable |
| 9 | CTSEN | CTS enable |
| 10 | CTSCFIEN | CTS change flag interrupt enable |
| 14 | RS485EN | RS485 mode enable |
| 15 | DEP | DE polarity |

## Configuration Types

### Parity Selection

```c
typedef enum
{
  USART_PARITY_NONE  = 0x00,  /* No parity */
  USART_PARITY_EVEN  = 0x01,  /* Even parity */
  USART_PARITY_ODD   = 0x02   /* Odd parity */
} usart_parity_selection_type;
```

### Data Bit Number

```c
typedef enum
{
  USART_DATA_7BITS = 0x00,  /* 7-bit data */
  USART_DATA_8BITS = 0x01,  /* 8-bit data */
  USART_DATA_9BITS = 0x02   /* 9-bit data */
} usart_data_bit_num_type;
```

### Stop Bit Number

```c
typedef enum
{
  USART_STOP_1_BIT   = 0x00,  /* 1 stop bit */
  USART_STOP_0_5_BIT = 0x01,  /* 0.5 stop bit */
  USART_STOP_2_BIT   = 0x02,  /* 2 stop bits */
  USART_STOP_1_5_BIT = 0x03   /* 1.5 stop bits */
} usart_stop_bit_num_type;
```

### Hardware Flow Control

```c
typedef enum
{
  USART_HARDWARE_FLOW_NONE    = 0x00,  /* No flow control */
  USART_HARDWARE_FLOW_RTS     = 0x01,  /* RTS only */
  USART_HARDWARE_FLOW_CTS     = 0x02,  /* CTS only */
  USART_HARDWARE_FLOW_RTS_CTS = 0x03   /* RTS and CTS */
} usart_hardware_flow_control_type;
```

### Wakeup Mode (Multi-Processor)

```c
typedef enum
{
  USART_WAKEUP_BY_IDLE_FRAME   = 0x00,  /* Wakeup by idle line */
  USART_WAKEUP_BY_MATCHING_ID  = 0x01   /* Wakeup by address mark */
} usart_wakeup_mode_type;
```

### Clock Configuration (Synchronous Mode)

```c
typedef enum
{
  USART_CLOCK_POLARITY_LOW  = 0x00,  /* Clock low when idle */
  USART_CLOCK_POLARITY_HIGH = 0x01   /* Clock high when idle */
} usart_clock_polarity_type;

typedef enum
{
  USART_CLOCK_PHASE_1EDGE = 0x00,  /* First clock transition captures data */
  USART_CLOCK_PHASE_2EDGE = 0x01   /* Second clock transition captures data */
} usart_clock_phase_type;

typedef enum
{
  USART_CLOCK_LAST_BIT_NONE   = 0x00,  /* No clock pulse for last bit */
  USART_CLOCK_LAST_BIT_OUTPUT = 0x01   /* Clock pulse for last bit */
} usart_lbcp_type;
```

### Break Frame Detection (LIN Mode)

```c
typedef enum
{
  USART_BREAK_10BITS = 0x00,  /* 10-bit break detection */
  USART_BREAK_11BITS = 0x01   /* 11-bit break detection */
} usart_break_bit_num_type;
```

### DE Polarity (RS485 Mode)

```c
typedef enum
{
  USART_DE_POLARITY_HIGH = 0x00,  /* DE active high */
  USART_DE_POLARITY_LOW  = 0x01   /* DE active low */
} usart_de_polarity_type;
```

### ID Bit Number (Multi-Processor)

```c
typedef enum
{
  USART_ID_FIXED_4_BIT       = 0x00,  /* Fixed 4-bit ID */
  USART_ID_RELATED_DATA_BIT  = 0x01   /* ID length matches data length */
} usart_identification_bit_num_type;
```

## Flags

| Flag | Description |
|------|-------------|
| `USART_PERR_FLAG` | Parity error |
| `USART_FERR_FLAG` | Framing error |
| `USART_NERR_FLAG` | Noise error |
| `USART_ROERR_FLAG` | Receiver overflow error |
| `USART_IDLEF_FLAG` | IDLE line detected |
| `USART_RDBF_FLAG` | Receive data buffer full |
| `USART_TDC_FLAG` | Transmit data complete |
| `USART_TDBE_FLAG` | Transmit data buffer empty |
| `USART_BFF_FLAG` | Break frame flag |
| `USART_CTSCF_FLAG` | CTS change flag |

## Interrupts

| Interrupt | Description |
|-----------|-------------|
| `USART_IDLE_INT` | IDLE line detection interrupt |
| `USART_RDBF_INT` | Receive data buffer full interrupt |
| `USART_TDC_INT` | Transmit data complete interrupt |
| `USART_TDBE_INT` | Transmit data buffer empty interrupt |
| `USART_PERR_INT` | Parity error interrupt |
| `USART_BF_INT` | Break frame interrupt |
| `USART_ERR_INT` | Error interrupt (FERR, NERR, ROERR) |
| `USART_CTSCF_INT` | CTS change flag interrupt |

## DMA Requests

| USART | TX Request | RX Request |
|-------|------------|------------|
| USART1 | `DMAMUX_DMAREQ_ID_USART1_TX` | `DMAMUX_DMAREQ_ID_USART1_RX` |
| USART2 | `DMAMUX_DMAREQ_ID_USART2_TX` | `DMAMUX_DMAREQ_ID_USART2_RX` |
| USART3 | `DMAMUX_DMAREQ_ID_USART3_TX` | `DMAMUX_DMAREQ_ID_USART3_RX` |
| UART4 | `DMAMUX_DMAREQ_ID_UART4_TX` | `DMAMUX_DMAREQ_ID_UART4_RX` |
| UART5 | `DMAMUX_DMAREQ_ID_UART5_TX` | `DMAMUX_DMAREQ_ID_UART5_RX` |
| USART6 | `DMAMUX_DMAREQ_ID_USART6_TX` | `DMAMUX_DMAREQ_ID_USART6_RX` |
| UART7 | `DMAMUX_DMAREQ_ID_UART7_TX` | `DMAMUX_DMAREQ_ID_UART7_RX` |
| UART8 | `DMAMUX_DMAREQ_ID_UART8_TX` | `DMAMUX_DMAREQ_ID_UART8_RX` |

## Pin Configuration

### USART1

| Function | Pin Options |
|----------|-------------|
| TX | PA9, PB6 |
| RX | PA10, PB7 |
| CK | PA8 |
| CTS | PA11 |
| RTS | PA12 |

### USART2

| Function | Pin Options |
|----------|-------------|
| TX | PA2, PD5 |
| RX | PA3, PD6 |
| CK | PA4, PD7 |
| CTS | PA0, PD3 |
| RTS | PA1, PD4 |
| DE | PA1 (RS485) |

### USART3

| Function | Pin Options |
|----------|-------------|
| TX | PB10, PC10, PD8 |
| RX | PB11, PC11, PD9 |
| CK | PB12, PC12, PD10 |
| CTS | PB13, PD11 |
| RTS | PB14, PD12 |

### USART6

| Function | Pin Options |
|----------|-------------|
| TX | PC6, PG14 |
| RX | PC7, PG9 |
| CK | PC8, PG7 |

## API Reference

### Initialization and Configuration

```c
/* Reset USART to default state */
void usart_reset(usart_type* usart_x);

/* Initialize USART with basic parameters */
void usart_init(usart_type* usart_x, uint32_t baud_rate, 
                usart_data_bit_num_type data_bit, 
                usart_stop_bit_num_type stop_bit);

/* Configure parity */
void usart_parity_selection_config(usart_type* usart_x, 
                                   usart_parity_selection_type parity);

/* Enable/disable USART */
void usart_enable(usart_type* usart_x, confirm_state new_state);

/* Enable/disable transmitter */
void usart_transmitter_enable(usart_type* usart_x, confirm_state new_state);

/* Enable/disable receiver */
void usart_receiver_enable(usart_type* usart_x, confirm_state new_state);
```

### Data Transmission and Reception

```c
/* Transmit single data */
void usart_data_transmit(usart_type* usart_x, uint16_t data);

/* Receive single data */
uint16_t usart_data_receive(usart_type* usart_x);

/* Send break frame */
void usart_break_send(usart_type* usart_x);
```

### Synchronous Mode (USART only)

```c
/* Configure clock parameters */
void usart_clock_config(usart_type* usart_x, 
                        usart_clock_polarity_type clk_pol,
                        usart_clock_phase_type clk_pha, 
                        usart_lbcp_type clk_lb);

/* Enable/disable clock output */
void usart_clock_enable(usart_type* usart_x, confirm_state new_state);
```

### Half-Duplex Mode

```c
/* Enable/disable single-line half-duplex mode */
void usart_single_line_halfduplex_select(usart_type* usart_x, confirm_state new_state);

/* Swap TX/RX pins */
void usart_transmit_receive_pin_swap(usart_type* usart_x, confirm_state new_state);
```

### Hardware Flow Control

```c
/* Configure hardware flow control */
void usart_hardware_flow_control_set(usart_type* usart_x, 
                                     usart_hardware_flow_control_type flow_state);
```

### Multi-Processor Communication

```c
/* Set wakeup ID */
void usart_wakeup_id_set(usart_type* usart_x, uint8_t usart_id);

/* Set wakeup mode */
void usart_wakeup_mode_set(usart_type* usart_x, usart_wakeup_mode_type wakeup_mode);

/* Enable/disable receiver mute */
void usart_receiver_mute_enable(usart_type* usart_x, confirm_state new_state);

/* Set ID bit number */
void usart_id_bit_num_set(usart_type* usart_x, 
                          usart_identification_bit_num_type id_bit_num);
```

### LIN Mode

```c
/* Enable/disable LIN mode */
void usart_lin_mode_enable(usart_type* usart_x, confirm_state new_state);

/* Set break detection length */
void usart_break_bit_num_set(usart_type* usart_x, 
                             usart_break_bit_num_type break_bit);
```

### IrDA Mode

```c
/* Enable/disable IrDA mode */
void usart_irda_mode_enable(usart_type* usart_x, confirm_state new_state);

/* Enable/disable IrDA low-power mode */
void usart_irda_low_power_enable(usart_type* usart_x, confirm_state new_state);

/* Set IrDA/Smartcard division */
void usart_irda_smartcard_division_set(usart_type* usart_x, uint8_t div_val);
```

### Smartcard Mode

```c
/* Enable/disable smartcard mode */
void usart_smartcard_mode_enable(usart_type* usart_x, confirm_state new_state);

/* Enable/disable NACK transmission */
void usart_smartcard_nack_set(usart_type* usart_x, confirm_state new_state);

/* Set guard time */
void usart_smartcard_guard_time_set(usart_type* usart_x, uint8_t guard_time_val);
```

### RS485 Mode

```c
/* Enable/disable RS485 mode */
void usart_rs485_mode_enable(usart_type* usart_x, confirm_state new_state);

/* Set DE polarity */
void usart_de_polarity_set(usart_type* usart_x, usart_de_polarity_type de_polarity);

/* Configure delay times */
void usart_rs485_delay_time_config(usart_type* usart_x, 
                                   uint8_t start_delay_time, 
                                   uint8_t complete_delay_time);
```

### DMA Control

```c
/* Enable/disable DMA transmitter */
void usart_dma_transmitter_enable(usart_type* usart_x, confirm_state new_state);

/* Enable/disable DMA receiver */
void usart_dma_receiver_enable(usart_type* usart_x, confirm_state new_state);
```

### Interrupt Control

```c
/* Enable/disable interrupt */
void usart_interrupt_enable(usart_type* usart_x, uint32_t usart_int, 
                            confirm_state new_state);
```

### Flag Management

```c
/* Get flag status */
flag_status usart_flag_get(usart_type* usart_x, uint32_t flag);

/* Get interrupt flag status */
flag_status usart_interrupt_flag_get(usart_type* usart_x, uint32_t flag);

/* Clear flag */
void usart_flag_clear(usart_type* usart_x, uint32_t flag);
```

## Code Examples

### Example 1: Basic Polling Mode

```c
/**
 * @brief  Basic USART polling communication
 * @note   USART2 TX/RX with polling
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

uint8_t tx_buffer[] = "Hello, USART!";
uint8_t rx_buffer[16];

void usart_config(void)
{
  gpio_init_type gpio_init_struct;

  /* Enable clocks */
  crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  /* Configure TX (PA2) and RX (PA3) pins */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_2 | GPIO_PINS_3;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_7);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE3, GPIO_MUX_7);

  /* Configure USART2: 115200 baud, 8N1 */
  usart_init(USART2, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_parity_selection_config(USART2, USART_PARITY_NONE);
  usart_transmitter_enable(USART2, TRUE);
  usart_receiver_enable(USART2, TRUE);
  usart_enable(USART2, TRUE);
}

void usart_send_string(usart_type* usart_x, uint8_t* str)
{
  while(*str)
  {
    while(usart_flag_get(usart_x, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(usart_x, *str++);
  }
  /* Wait for transmission complete */
  while(usart_flag_get(usart_x, USART_TDC_FLAG) == RESET);
}

uint8_t usart_receive_byte(usart_type* usart_x)
{
  while(usart_flag_get(usart_x, USART_RDBF_FLAG) == RESET);
  return (uint8_t)usart_data_receive(usart_x);
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  usart_config();

  /* Send greeting */
  usart_send_string(USART2, tx_buffer);

  /* Echo received data */
  while(1)
  {
    uint8_t data = usart_receive_byte(USART2);
    while(usart_flag_get(USART2, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(USART2, data);
  }
}
```

### Example 2: Interrupt-Driven Communication

```c
/**
 * @brief  USART communication using interrupts
 * @note   USART2 <-> USART3 bidirectional transfer
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define BUFFER_SIZE 64

uint8_t usart2_tx_buffer[] = "USART2 -> USART3";
uint8_t usart3_tx_buffer[] = "USART3 -> USART2";
uint8_t usart2_rx_buffer[BUFFER_SIZE];
uint8_t usart3_rx_buffer[BUFFER_SIZE];
volatile uint8_t usart2_tx_index = 0;
volatile uint8_t usart3_tx_index = 0;
volatile uint8_t usart2_rx_index = 0;
volatile uint8_t usart3_rx_index = 0;

void usart_config(void)
{
  gpio_init_type gpio_init_struct;

  /* Enable clocks */
  crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  /* USART2: PA2(TX), PA3(RX) */
  gpio_init_struct.gpio_pins = GPIO_PINS_2 | GPIO_PINS_3;
  gpio_init(GPIOA, &gpio_init_struct);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_7);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE3, GPIO_MUX_7);

  /* USART3: PB10(TX), PB11(RX) */
  gpio_init_struct.gpio_pins = GPIO_PINS_10 | GPIO_PINS_11;
  gpio_init(GPIOB, &gpio_init_struct);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE10, GPIO_MUX_7);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE11, GPIO_MUX_7);

  /* Configure NVIC */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  nvic_irq_enable(USART2_IRQn, 0, 0);
  nvic_irq_enable(USART3_IRQn, 0, 0);

  /* Configure USART2 */
  usart_init(USART2, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART2, TRUE);
  usart_receiver_enable(USART2, TRUE);
  usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
  usart_enable(USART2, TRUE);

  /* Configure USART3 */
  usart_init(USART3, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART3, TRUE);
  usart_receiver_enable(USART3, TRUE);
  usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);
  usart_enable(USART3, TRUE);

  /* Start transmission */
  usart_interrupt_enable(USART2, USART_TDBE_INT, TRUE);
  usart_interrupt_enable(USART3, USART_TDBE_INT, TRUE);
}

void USART2_IRQHandler(void)
{
  if(usart_interrupt_flag_get(USART2, USART_TDBE_FLAG) != RESET)
  {
    if(usart2_tx_buffer[usart2_tx_index] != '\0')
    {
      usart_data_transmit(USART2, usart2_tx_buffer[usart2_tx_index++]);
    }
    else
    {
      usart_interrupt_enable(USART2, USART_TDBE_INT, FALSE);
    }
  }

  if(usart_interrupt_flag_get(USART2, USART_RDBF_FLAG) != RESET)
  {
    usart2_rx_buffer[usart2_rx_index++] = usart_data_receive(USART2);
  }
}

void USART3_IRQHandler(void)
{
  if(usart_interrupt_flag_get(USART3, USART_TDBE_FLAG) != RESET)
  {
    if(usart3_tx_buffer[usart3_tx_index] != '\0')
    {
      usart_data_transmit(USART3, usart3_tx_buffer[usart3_tx_index++]);
    }
    else
    {
      usart_interrupt_enable(USART3, USART_TDBE_INT, FALSE);
    }
  }

  if(usart_interrupt_flag_get(USART3, USART_RDBF_FLAG) != RESET)
  {
    usart3_rx_buffer[usart3_rx_index++] = usart_data_receive(USART3);
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  usart_config();

  while(1)
  {
    /* Check transfer complete */
    if(usart2_rx_index >= sizeof(usart3_tx_buffer) - 1 &&
       usart3_rx_index >= sizeof(usart2_tx_buffer) - 1)
    {
      at32_led_toggle(LED2);
      delay_ms(500);
    }
  }
}
```

### Example 3: DMA Transfer

```c
/**
 * @brief  USART DMA transfer
 * @note   Efficient data transfer using DMA
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define TX_BUFFER_SIZE 60

uint8_t tx_buffer[] = "USART DMA transfer example - high speed data transmission";
uint8_t rx_buffer[TX_BUFFER_SIZE];

void dma_config(void)
{
  dma_init_type dma_init_struct;

  /* Enable DMA1 clock */
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  dmamux_enable(DMA1, TRUE);

  /* DMA1 Channel1 for USART2 TX */
  dma_reset(DMA1_CHANNEL1);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = TX_BUFFER_SIZE;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = (uint32_t)tx_buffer;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART2->dt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA1_CHANNEL1, &dma_init_struct);
  dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_USART2_TX);

  /* DMA1 Channel2 for USART2 RX */
  dma_reset(DMA1_CHANNEL2);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = TX_BUFFER_SIZE;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)rx_buffer;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART2->dt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA1_CHANNEL2, &dma_init_struct);
  dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_USART2_RX);
}

void usart_config(void)
{
  gpio_init_type gpio_init_struct;

  /* Enable clocks */
  crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  /* Configure pins */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_2 | GPIO_PINS_3;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_7);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE3, GPIO_MUX_7);

  /* Configure USART2 */
  usart_init(USART2, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART2, TRUE);
  usart_receiver_enable(USART2, TRUE);
  usart_dma_transmitter_enable(USART2, TRUE);
  usart_dma_receiver_enable(USART2, TRUE);
  usart_enable(USART2, TRUE);
}

int main(void)
{
  system_clock_config();
  at32_board_init();

  usart_config();
  dma_config();

  /* Start DMA transfers */
  dma_channel_enable(DMA1_CHANNEL2, TRUE);  /* RX first */
  dma_channel_enable(DMA1_CHANNEL1, TRUE);  /* Then TX */

  /* Wait for transfer complete */
  while(dma_flag_get(DMA1_FDT1_FLAG) == RESET || 
        dma_flag_get(DMA1_FDT2_FLAG) == RESET);

  /* Verify data */
  while(1)
  {
    at32_led_toggle(LED2);
    delay_ms(500);
  }
}
```

### Example 4: Half-Duplex Mode

```c
/**
 * @brief  Single-wire half-duplex communication
 * @note   Uses only TX pin for bidirectional transfer
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

uint8_t master_tx[] = "Master sends";
uint8_t slave_tx[] = "Slave replies";
uint8_t master_rx[16];
uint8_t slave_rx[16];

void half_duplex_config(void)
{
  gpio_init_type gpio_init_struct;

  /* Enable clocks */
  crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  
  /* Configure TX pins as open-drain with pull-up */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;

  /* USART2 TX: PA2 */
  gpio_init_struct.gpio_pins = GPIO_PINS_2;
  gpio_init(GPIOA, &gpio_init_struct);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_7);

  /* USART3 TX: PB10 */
  gpio_init_struct.gpio_pins = GPIO_PINS_10;
  gpio_init(GPIOB, &gpio_init_struct);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE10, GPIO_MUX_7);

  /* Configure USART2 (Master) */
  usart_init(USART2, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART2, TRUE);
  usart_receiver_enable(USART2, TRUE);
  usart_single_line_halfduplex_select(USART2, TRUE);
  usart_enable(USART2, TRUE);

  /* Configure USART3 (Slave) */
  usart_init(USART3, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART3, TRUE);
  usart_receiver_enable(USART3, TRUE);
  usart_single_line_halfduplex_select(USART3, TRUE);
  usart_enable(USART3, TRUE);
}

void transfer_data(usart_type* tx_usart, uint8_t* tx_buf, uint8_t tx_len,
                   usart_type* rx_usart, uint8_t* rx_buf)
{
  uint8_t i;
  for(i = 0; i < tx_len; i++)
  {
    while(usart_flag_get(tx_usart, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(tx_usart, tx_buf[i]);
    while(usart_flag_get(rx_usart, USART_RDBF_FLAG) == RESET);
    rx_buf[i] = usart_data_receive(rx_usart);
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  half_duplex_config();

  /* Master -> Slave */
  transfer_data(USART2, master_tx, sizeof(master_tx) - 1, USART3, slave_rx);

  /* Slave -> Master */
  transfer_data(USART3, slave_tx, sizeof(slave_tx) - 1, USART2, master_rx);

  while(1)
  {
    at32_led_toggle(LED2);
    delay_ms(500);
  }
}
```

### Example 5: RS485 Mode

```c
/**
 * @brief  RS485 half-duplex communication
 * @note   Automatic DE (Driver Enable) control
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define RS485_BAUDRATE 9600
#define BUFFER_SIZE    128

uint8_t rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_count = 0;

void rs485_config(void)
{
  gpio_init_type gpio_init_struct;

  /* Enable clocks */
  crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  /* Configure TX (PA2), RX (PA3), DE (PA1) */
  gpio_init_struct.gpio_pins = GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_3;
  gpio_init(GPIOA, &gpio_init_struct);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE1, GPIO_MUX_7);  /* DE */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_7);  /* TX */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE3, GPIO_MUX_7);  /* RX */

  /* Configure USART2 for RS485 */
  usart_init(USART2, RS485_BAUDRATE, USART_DATA_8BITS, USART_STOP_1_BIT);
  
  /* RS485 specific configuration */
  usart_rs485_delay_time_config(USART2, 2, 2);  /* Start/complete delay */
  usart_de_polarity_set(USART2, USART_DE_POLARITY_HIGH);
  usart_rs485_mode_enable(USART2, TRUE);

  /* Enable RX interrupt */
  usart_flag_clear(USART2, USART_RDBF_FLAG);
  usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);

  usart_receiver_enable(USART2, TRUE);
  usart_transmitter_enable(USART2, TRUE);
  usart_enable(USART2, TRUE);

  /* Configure NVIC */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  nvic_irq_enable(USART2_IRQn, 1, 0);
}

void rs485_send(uint8_t* data, uint8_t len)
{
  while(len--)
  {
    while(usart_flag_get(USART2, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(USART2, *data++);
  }
  /* Wait for transmit complete */
  while(usart_flag_get(USART2, USART_TDC_FLAG) == RESET);
}

void USART2_IRQHandler(void)
{
  if(usart_interrupt_flag_get(USART2, USART_RDBF_FLAG) != RESET)
  {
    if(rx_count < BUFFER_SIZE)
    {
      rx_buffer[rx_count++] = usart_data_receive(USART2);
    }
    else
    {
      usart_data_receive(USART2);  /* Discard overflow */
    }
  }
}

int main(void)
{
  char msg[] = "RS485 Test\r\n";

  system_clock_config();
  at32_board_init();
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  rs485_config();

  /* Send initial message */
  rs485_send((uint8_t*)msg, sizeof(msg) - 1);

  while(1)
  {
    /* Check for IDLE line (frame received) */
    if(usart_flag_get(USART2, USART_IDLEF_FLAG) != RESET)
    {
      usart_data_receive(USART2);  /* Clear IDLE flag */
      
      /* Echo received data */
      usart_interrupt_enable(USART2, USART_RDBF_INT, FALSE);
      rs485_send(rx_buffer, rx_count);
      rx_count = 0;
      usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
    }
  }
}
```

### Example 6: Hardware Flow Control

```c
/**
 * @brief  RTS/CTS hardware flow control
 * @note   Prevents data loss at high speeds
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define BUFFER_SIZE 256

uint8_t rx_buffer[BUFFER_SIZE];

void hw_flow_config(void)
{
  gpio_init_type gpio_init_struct;

  /* Enable clocks */
  crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  /* Configure PD3(CTS), PD4(RTS), PD5(TX), PD6(RX) */
  gpio_init_struct.gpio_pins = GPIO_PINS_3 | GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_6;
  gpio_init(GPIOD, &gpio_init_struct);
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE3, GPIO_MUX_7);  /* CTS */
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE4, GPIO_MUX_7);  /* RTS */
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE5, GPIO_MUX_7);  /* TX */
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE6, GPIO_MUX_7);  /* RX */

  /* Configure USART2 with hardware flow control */
  usart_init(USART2, 921600, USART_DATA_8BITS, USART_STOP_1_BIT);  /* High speed */
  usart_hardware_flow_control_set(USART2, USART_HARDWARE_FLOW_RTS_CTS);
  usart_transmitter_enable(USART2, TRUE);
  usart_receiver_enable(USART2, TRUE);
  usart_enable(USART2, TRUE);
}

int main(void)
{
  uint16_t i;
  uint16_t rx_count = 0;

  system_clock_config();
  at32_board_init();
  hw_flow_config();

  /* Receive data with flow control */
  while(rx_count < BUFFER_SIZE)
  {
    while(usart_flag_get(USART2, USART_RDBF_FLAG) == RESET);
    rx_buffer[rx_count++] = usart_data_receive(USART2);
  }

  /* Echo back */
  for(i = 0; i < rx_count; i++)
  {
    while(usart_flag_get(USART2, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(USART2, rx_buffer[i]);
  }

  while(1)
  {
    at32_led_toggle(LED2);
    delay_ms(500);
  }
}
```

### Example 7: IrDA Mode

```c
/**
 * @brief  IrDA infrared communication
 * @note   For IR transceivers
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void irda_config(void)
{
  gpio_init_type gpio_init_struct;

  /* Enable clocks */
  crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_2 | GPIO_PINS_3;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_7);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE3, GPIO_MUX_7);

  /* Configure USART2 for IrDA */
  usart_init(USART2, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_irda_smartcard_division_set(USART2, 0x01);
  usart_irda_mode_enable(USART2, TRUE);
  usart_transmitter_enable(USART2, TRUE);
  usart_receiver_enable(USART2, TRUE);
  usart_enable(USART2, TRUE);
}

void irda_send_byte(uint8_t data)
{
  while(usart_flag_get(USART2, USART_TDBE_FLAG) == RESET);
  usart_data_transmit(USART2, data);
}

uint8_t irda_receive_byte(void)
{
  while(usart_flag_get(USART2, USART_RDBF_FLAG) == RESET);
  return (uint8_t)usart_data_receive(USART2);
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  irda_config();

  /* Transmit mode */
  #if defined(TRANSMIT)
  irda_send_byte(0x55);
  at32_led_on(LED4);
  #endif

  /* Receive mode */
  #if defined(RECEIVE)
  if(irda_receive_byte() == 0x55)
  {
    at32_led_on(LED4);
  }
  #endif

  while(1)
  {
  }
}
```

### Example 8: Smartcard Mode (ISO 7816-3)

```c
/**
 * @brief  ISO 7816-3 Smartcard interface
 * @note   T=0 protocol with NACK
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#define SC_USART          USART2
#define SC_USART_DIV      20    /* CLK = APB1/(2*20) -> 1-5 MHz */
#define SC_F_DIV_D        372   /* F/D = 372 per ISO 7816-3 */

uint8_t atr_buffer[40];

void smartcard_config(void)
{
  gpio_init_type gpio_init_struct;
  crm_clocks_freq_type clocks;
  uint32_t sc_clock, sc_baud;

  /* Enable clocks */
  crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);

  /* Configure CK pin (PA4) - clock to card */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_4;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE4, GPIO_MUX_7);

  /* Configure TX pin (PA2) - I/O line (open-drain) */
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_pins = GPIO_PINS_2;
  gpio_init(GPIOA, &gpio_init_struct);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_7);

  /* Calculate baud rate */
  crm_clocks_freq_get(&clocks);
  sc_clock = clocks.apb1_freq / (2 * SC_USART_DIV);
  sc_baud = sc_clock / SC_F_DIV_D;

  /* Configure smartcard clock and guard time */
  usart_irda_smartcard_division_set(SC_USART, SC_USART_DIV);
  usart_smartcard_guard_time_set(SC_USART, 0x02);
  usart_clock_enable(SC_USART, TRUE);

  /* Configure USART for smartcard */
  usart_init(SC_USART, sc_baud, USART_DATA_9BITS, USART_STOP_1_5_BIT);
  usart_parity_selection_config(SC_USART, USART_PARITY_EVEN);
  usart_transmitter_enable(SC_USART, TRUE);
  usart_receiver_enable(SC_USART, TRUE);

  /* Enable parity error interrupt */
  usart_interrupt_enable(SC_USART, USART_PERR_INT, TRUE);
  usart_enable(SC_USART, TRUE);

  /* Enable NACK and smartcard mode */
  usart_smartcard_nack_set(SC_USART, TRUE);
  usart_smartcard_mode_enable(SC_USART, TRUE);
}

uint8_t read_atr(uint8_t* buffer, uint8_t max_len)
{
  uint8_t i;
  uint32_t timeout;

  for(i = 0; i < max_len; i++)
  {
    timeout = 0xA000;
    while((usart_flag_get(SC_USART, USART_RDBF_FLAG) == RESET) && timeout)
    {
      timeout--;
    }
    if(timeout == 0) break;
    buffer[i] = usart_data_receive(SC_USART);
  }
  return i;
}

int main(void)
{
  uint8_t atr_len;

  system_clock_config();
  at32_board_init();
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  smartcard_config();

  /* Wait for card insertion, then read ATR */
  /* (Card detection and reset logic would be here) */

  atr_len = read_atr(atr_buffer, 40);

  if(atr_len > 0)
  {
    at32_led_on(LED2);  /* Card detected */
  }

  while(1)
  {
  }
}
```

## Configuration Checklist

- [ ] Enable USART and GPIO peripheral clocks
- [ ] Configure TX/RX pins as alternate function (AF7 typically)
- [ ] Set correct baud rate for target device
- [ ] Configure data bits (7/8/9)
- [ ] Configure stop bits (1/0.5/2/1.5)
- [ ] Configure parity (none/even/odd)
- [ ] Enable transmitter and/or receiver
- [ ] Configure interrupts if using interrupt mode
- [ ] Configure DMA if using DMA mode
- [ ] Configure special mode (RS485/IrDA/Smartcard/LIN) if needed
- [ ] Enable USART peripheral

## Troubleshooting

### No Data Transmission

1. Verify TX pin is configured as alternate function
2. Check that USART clock is enabled
3. Ensure transmitter is enabled (`usart_transmitter_enable`)
4. Verify USART is enabled (`usart_enable`)
5. Check baud rate divisor calculation

### No Data Reception

1. Verify RX pin is configured as alternate function
2. Ensure receiver is enabled (`usart_receiver_enable`)
3. Check for overflow errors (ROERR)
4. Verify sender baud rate matches

### Framing Errors

1. Check baud rate on both ends
2. Verify stop bit configuration matches
3. Check for signal integrity issues
4. Ensure proper ground connection

### Parity Errors

1. Verify parity settings match on both ends
2. Check for electrical noise
3. Reduce baud rate if persistent

### Overflow Errors

1. Increase receive buffer processing speed
2. Use DMA for high-speed reception
3. Implement hardware flow control
4. Reduce sender transmission rate

### RS485 Issues

1. Verify DE polarity matches transceiver
2. Check delay time settings
3. Ensure proper bus termination
4. Verify only one transmitter active at a time

### Smartcard Issues

1. Verify clock frequency is 1-5 MHz
2. Check guard time setting
3. Ensure F/D ratio is correct (372 typical)
4. Verify card power supply

## Performance Guidelines

| Baud Rate | Typical Use Case | DMA Recommended |
|-----------|------------------|-----------------|
| 9600 | Legacy devices, RS485 | No |
| 115200 | General purpose | Optional |
| 460800 | High-speed serial | Yes |
| 921600 | Maximum throughput | Yes |
| 4.5 Mbps | Max (at 288 MHz APB) | Yes |

## Related Peripherals

- **[GPIO](GPIO_General_Purpose_IO.md)** - Pin configuration for USART signals
- **[DMA](DMA_Direct_Memory_Access.md)** - Efficient data transfers
- **[CRM](CRM_Clock_Reset_Manager.md)** - Clock configuration
- **[NVIC](NVIC_Interrupt_Controller.md)** - Interrupt management
- **[TMR](TMR_Timer.md)** - Baud rate generation, timeout handling

