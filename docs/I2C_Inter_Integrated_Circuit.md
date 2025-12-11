---
title: I2C - Inter-Integrated Circuit
category: Peripheral
complexity: Intermediate
mcu: AT32F435/437
peripheral: I2C
keywords: [i2c, iic, twi, smbus, eeprom, master, slave, dma]
---

# I2C - Inter-Integrated Circuit

## Overview

The I2C (Inter-Integrated Circuit) peripheral provides a multi-master, multi-slave serial communication interface compliant with I2C and SMBus protocols. The AT32F435/437 features three independent I2C controllers supporting Standard Mode (100 kHz), Fast Mode (400 kHz), and Fast Mode Plus (1 MHz). Each controller supports master/slave operation, 7/10-bit addressing, DMA transfers, digital noise filtering, clock stretching, and SMBus with PEC calculation.

### Key Features

| Feature | Specification |
|---------|---------------|
| I2C Units | 3 (I2C1, I2C2, I2C3) |
| Speed Modes | Standard (100 kHz), Fast (400 kHz), Fast Mode Plus (1 MHz) |
| Addressing | 7-bit and 10-bit |
| Operation Modes | Master, Slave, Multi-master |
| Own Addresses | 2 (OA1: 7/10-bit, OA2: 7-bit with mask) |
| Max Transfer | 255 bytes per reload |
| Digital Filter | 0-15 I2C clock cycles |
| DMA | TX and RX support |
| SMBus | Host, Device, Alert, PEC, Timeout |
| Interrupts | TX, RX, Address, ACK Fail, Stop, TDC, Error |

---

## Architecture

```
                              ┌─────────────────────────────────────────────────┐
                              │                 I2C Controller                   │
                              │                                                  │
  ┌─────────────────┐        │   ┌───────────────────────────────────────────┐ │
  │    APB1 Bus     │────────┼──►│           Control Logic                    │ │
  └─────────────────┘        │   │                                            │ │
                              │   │  ┌─────────┐  ┌─────────┐  ┌───────────┐ │ │
                              │   │  │ Master  │  │  Slave  │  │  Arbiter  │ │ │
                              │   │  │  FSM    │  │   FSM   │  │           │ │ │
                              │   │  └────┬────┘  └────┬────┘  └─────┬─────┘ │ │
                              │   │       │            │              │       │ │
                              │   └───────┼────────────┼──────────────┼───────┘ │
                              │           │            │              │         │
                              │   ┌───────▼────────────▼──────────────▼───────┐ │
                              │   │              Clock Control                 │ │
                              │   │    ┌────────────────────────────────┐     │ │
                              │   │    │  CLKCTRL: SCLL, SCLH, SDAD,   │     │ │
                              │   │    │          SCLD, DIVH, DIVL      │     │ │
                              │   │    └────────────────────────────────┘     │ │
                              │   └───────────────────────────────────────────┘ │
                              │                                                  │
                              │   ┌──────────────┐     ┌──────────────────────┐ │
                              │   │   TXDT       │     │      Digital         │ │
                              │   │  (TX Data)   │────►│      Filter          │─┼─► SDA
                              │   └──────────────┘     │   (0-15 clocks)      │ │
                              │                        └──────────────────────┘ │
                              │   ┌──────────────┐              │               │
                              │   │   RXDT       │◄─────────────┘               │
                              │   │  (RX Data)   │                              │
                              │   └──────────────┘     ┌──────────────────────┐ │
                              │                        │    SCL Generation    │ │
                              │   ┌──────────────┐     │    & Stretching      │─┼─► SCL
                              │   │  Address     │     └──────────────────────┘ │
                              │   │  OA1, OA2    │                              │
                              │   └──────────────┘                              │
                              │                                                  │
                              │   ┌──────────────┐     ┌──────────────────────┐ │
                              │   │   SMBus      │     │   Timeout            │ │
                              │   │   PEC Calc   │     │   Detection          │ │
                              │   └──────────────┘     └──────────────────────┘ │
                              │                                                  │
  ┌─────────────────┐        │   ┌──────────────────────────────────────────┐  │
  │      DMA        │◄───────┼───│  DMA Request: DMATEN, DMAREN             │  │
  └─────────────────┘        │   └──────────────────────────────────────────┘  │
                              │                                                  │
  ┌─────────────────┐        │   ┌──────────────────────────────────────────┐  │
  │      NVIC       │◄───────┼───│  Interrupts: EVT, ERR                    │  │
  └─────────────────┘        │   └──────────────────────────────────────────┘  │
                              └─────────────────────────────────────────────────┘
```

---

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| CTRL1 | 0x00 | Control register 1 (enable, interrupts, DMA, filter, SMBus) |
| CTRL2 | 0x04 | Control register 2 (address, direction, count, start/stop) |
| OADDR1 | 0x08 | Own address 1 (7/10-bit mode) |
| OADDR2 | 0x0C | Own address 2 (7-bit with mask) |
| CLKCTRL | 0x10 | Clock control (SCLL, SCLH, SDAD, SCLD, DIVH, DIVL) |
| TIMEOUT | 0x14 | Timeout configuration |
| STS | 0x18 | Status register (flags) |
| CLR | 0x1C | Flag clear register |
| PEC | 0x20 | PEC value register |
| RXDT | 0x24 | Receive data register |
| TXDT | 0x28 | Transmit data register |

---

## Clock Configuration

The CLKCTRL register controls I2C timing. Pre-calculated values for 288 MHz system clock:

| Speed | CLKCTRL Value | Notes |
|-------|---------------|-------|
| 10 kHz | 0xB170FFFF | Standard Mode - slow |
| 50 kHz | 0xC0E06969 | Standard Mode |
| 100 kHz | 0x80504C4E | Standard Mode - typical |
| 200 kHz | 0x30F03C6B | Fast Mode |

### CLKCTRL Register Fields

```
CLKCTRL = (DIVL << 28) | (DIVH << 24) | (SCLD << 20) | (SDAD << 16) | (SCLH << 8) | SCLL

Where:
- SCLL[7:0]   : SCL low period (in I2C clock cycles)
- SCLH[7:0]   : SCL high period (in I2C clock cycles)
- SDAD[3:0]   : SDA hold time
- SCLD[3:0]   : SCL delay time
- DIVH[3:0]   : Clock high prescaler
- DIVL[3:0]   : Clock low prescaler
```

---

## Configuration Types

### Address Mode

```c
typedef enum {
  I2C_ADDRESS_MODE_7BIT  = 0x00,  /* 7-bit address mode */
  I2C_ADDRESS_MODE_10BIT = 0x01   /* 10-bit address mode */
} i2c_address_mode_type;
```

### Transfer Direction

```c
typedef enum {
  I2C_DIR_TRANSMIT = 0x00,  /* Master write / Slave receive */
  I2C_DIR_RECEIVE  = 0x01   /* Master read / Slave transmit */
} i2c_transfer_dir_type;
```

### Reload/Stop Mode

```c
typedef enum {
  I2C_AUTO_STOP_MODE = 0x02000000,  /* Auto generate STOP after CNT bytes */
  I2C_SOFT_STOP_MODE = 0x00000000,  /* Manual STOP generation */
  I2C_RELOAD_MODE    = 0x01000000   /* Reload CNT for transfers > 255 bytes */
} i2c_reload_stop_mode_type;
```

### Start Mode

```c
typedef enum {
  I2C_WITHOUT_START    = 0x00000000,  /* Continue without START */
  I2C_GEN_START_READ   = 0x00002400,  /* Generate START + Read */
  I2C_GEN_START_WRITE  = 0x00002000   /* Generate START + Write */
} i2c_start_mode_type;
```

### SMBus Mode

```c
typedef enum {
  I2C_SMBUS_MODE_DEVICE = 0x00,  /* SMBus device (slave) */
  I2C_SMBUS_MODE_HOST   = 0x01   /* SMBus host (master) */
} i2c_smbus_mode_type;
```

### Address 2 Mask

```c
typedef enum {
  I2C_ADDR2_NOMASK = 0x00,  /* Compare all bits [7:1] */
  I2C_ADDR2_MASK01 = 0x01,  /* Compare bits [7:2] only */
  I2C_ADDR2_MASK02 = 0x02,  /* Compare bits [7:3] only */
  I2C_ADDR2_MASK03 = 0x03,  /* Compare bits [7:4] only */
  I2C_ADDR2_MASK04 = 0x04,  /* Compare bits [7:5] only */
  I2C_ADDR2_MASK05 = 0x05,  /* Compare bits [7:6] only */
  I2C_ADDR2_MASK06 = 0x06,  /* Compare bit [7] only */
  I2C_ADDR2_MASK07 = 0x07   /* Respond to all addresses */
} i2c_addr2_mask_type;
```

---

## Flags

| Flag | Description |
|------|-------------|
| I2C_TDBE_FLAG | Transmit data buffer empty |
| I2C_TDIS_FLAG | Transmit interrupt status |
| I2C_RDBF_FLAG | Receive data buffer full |
| I2C_ADDRF_FLAG | Address match flag |
| I2C_ACKFAIL_FLAG | Acknowledge failure |
| I2C_STOPF_FLAG | Stop condition detected |
| I2C_TDC_FLAG | Transmit data complete |
| I2C_TCRLD_FLAG | Transfer complete, reload required |
| I2C_BUSERR_FLAG | Bus error |
| I2C_ARLOST_FLAG | Arbitration lost |
| I2C_OUF_FLAG | Overflow/underflow |
| I2C_PECERR_FLAG | PEC error |
| I2C_TMOUT_FLAG | Timeout |
| I2C_ALERTF_FLAG | SMBus alert |
| I2C_BUSYF_FLAG | Bus busy |
| I2C_SDIR_FLAG | Slave direction |

---

## Interrupts

| Interrupt | Description |
|-----------|-------------|
| I2C_TD_INT | Transmit data interrupt |
| I2C_RD_INT | Receive data interrupt |
| I2C_ADDR_INT | Address match interrupt |
| I2C_ACKFIAL_INT | ACK fail interrupt |
| I2C_STOP_INT | Stop detect interrupt |
| I2C_TDC_INT | Transmit data complete interrupt |
| I2C_ERR_INT | Bus error interrupt (BUSERR, ARLOST, OUF, PEC, TMOUT, ALERT) |

---

## DMA Requests

| Request | DMAMUX ID | Description |
|---------|-----------|-------------|
| I2C1_TX | DMAMUX_DMAREQ_ID_I2C1_TX | I2C1 transmit |
| I2C1_RX | DMAMUX_DMAREQ_ID_I2C1_RX | I2C1 receive |
| I2C2_TX | DMAMUX_DMAREQ_ID_I2C2_TX | I2C2 transmit |
| I2C2_RX | DMAMUX_DMAREQ_ID_I2C2_RX | I2C2 receive |
| I2C3_TX | DMAMUX_DMAREQ_ID_I2C3_TX | I2C3 transmit |
| I2C3_RX | DMAMUX_DMAREQ_ID_I2C3_RX | I2C3 receive |

---

## API Reference

### Initialization Functions

```c
/**
  * @brief  Reset I2C peripheral
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @retval none
  */
void i2c_reset(i2c_type *i2c_x);

/**
  * @brief  Initialize I2C with digital filter and clock control
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  dfilters: Digital filter cycles (0x00-0x0F)
  * @param  clk: Clock control register value
  * @retval none
  */
void i2c_init(i2c_type *i2c_x, uint8_t dfilters, uint32_t clk);

/**
  * @brief  Enable/disable I2C peripheral
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_enable(i2c_type *i2c_x, confirm_state new_state);
```

### Address Configuration

```c
/**
  * @brief  Configure own address 1 (7 or 10-bit)
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  mode: I2C_ADDRESS_MODE_7BIT or I2C_ADDRESS_MODE_10BIT
  * @param  address: Own address (e.g., 0xA0 for 7-bit)
  * @retval none
  */
void i2c_own_address1_set(i2c_type *i2c_x, i2c_address_mode_type mode, uint16_t address);

/**
  * @brief  Configure own address 2 with mask (7-bit only)
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  address: Second own address
  * @param  mask: Address comparison mask
  * @retval none
  */
void i2c_own_address2_set(i2c_type *i2c_x, uint8_t address, i2c_addr2_mask_type mask);

/**
  * @brief  Enable/disable own address 2
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_own_address2_enable(i2c_type *i2c_x, confirm_state new_state);
```

### Transfer Configuration

```c
/**
  * @brief  Configure transfer parameters and optionally generate START
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  address: Target slave address
  * @param  cnt: Byte count (0-255)
  * @param  rld_stop: I2C_AUTO_STOP_MODE, I2C_SOFT_STOP_MODE, or I2C_RELOAD_MODE
  * @param  start: I2C_WITHOUT_START, I2C_GEN_START_READ, or I2C_GEN_START_WRITE
  * @retval none
  */
void i2c_transmit_set(i2c_type *i2c_x, uint16_t address, uint8_t cnt, 
                      i2c_reload_stop_mode_type rld_stop, i2c_start_mode_type start);

/**
  * @brief  Set transfer byte count
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  cnt: Byte count (0-255)
  * @retval none
  */
void i2c_cnt_set(i2c_type *i2c_x, uint8_t cnt);

/**
  * @brief  Set transfer direction
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  i2c_direction: I2C_DIR_TRANSMIT or I2C_DIR_RECEIVE
  * @retval none
  */
void i2c_transfer_dir_set(i2c_type *i2c_x, i2c_transfer_dir_type i2c_direction);

/**
  * @brief  Set target slave address
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  address: Slave address
  * @retval none
  */
void i2c_transfer_addr_set(i2c_type *i2c_x, uint16_t address);
```

### Start/Stop Control

```c
/**
  * @brief  Generate START condition
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @retval none
  */
void i2c_start_generate(i2c_type *i2c_x);

/**
  * @brief  Generate STOP condition
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @retval none
  */
void i2c_stop_generate(i2c_type *i2c_x);

/**
  * @brief  Enable/disable auto STOP after transfer
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_auto_stop_enable(i2c_type *i2c_x, confirm_state new_state);

/**
  * @brief  Enable/disable reload mode for large transfers
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_reload_enable(i2c_type *i2c_x, confirm_state new_state);
```

### Data Transfer

```c
/**
  * @brief  Send one byte of data
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  data: Byte to transmit
  * @retval none
  */
void i2c_data_send(i2c_type *i2c_x, uint8_t data);

/**
  * @brief  Receive one byte of data
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @retval Received byte
  */
uint8_t i2c_data_receive(i2c_type *i2c_x);
```

### ACK Control

```c
/**
  * @brief  Enable/disable ACK generation
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  new_state: TRUE (ACK) or FALSE (NACK)
  * @retval none
  */
void i2c_ack_enable(i2c_type *i2c_x, confirm_state new_state);

/**
  * @brief  Enable/disable clock stretching
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_clock_stretch_enable(i2c_type *i2c_x, confirm_state new_state);
```

### Flag and Interrupt Functions

```c
/**
  * @brief  Get flag status
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  flag: Flag to check
  * @retval SET or RESET
  */
flag_status i2c_flag_get(i2c_type *i2c_x, uint32_t flag);

/**
  * @brief  Clear flag
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  flag: Flag(s) to clear
  * @retval none
  */
void i2c_flag_clear(i2c_type *i2c_x, uint32_t flag);

/**
  * @brief  Enable/disable interrupt source
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  source: Interrupt source(s)
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_interrupt_enable(i2c_type *i2c_x, uint32_t source, confirm_state new_state);

/**
  * @brief  Get interrupt flag status
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  flag: Flag to check
  * @retval SET or RESET
  */
flag_status i2c_interrupt_flag_get(i2c_type *i2c_x, uint32_t flag);
```

### DMA Functions

```c
/**
  * @brief  Enable/disable DMA request
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  dma_req: I2C_DMA_REQUEST_TX or I2C_DMA_REQUEST_RX
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_dma_enable(i2c_type *i2c_x, i2c_dma_request_type dma_req, confirm_state new_state);
```

### SMBus Functions

```c
/**
  * @brief  Enable/disable SMBus mode
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  mode: I2C_SMBUS_MODE_DEVICE or I2C_SMBUS_MODE_HOST
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_smbus_enable(i2c_type *i2c_x, i2c_smbus_mode_type mode, confirm_state new_state);

/**
  * @brief  Enable/disable PEC calculation
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_pec_calculate_enable(i2c_type *i2c_x, confirm_state new_state);

/**
  * @brief  Enable PEC transmission after data
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_pec_transmit_enable(i2c_type *i2c_x, confirm_state new_state);

/**
  * @brief  Get current PEC value
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @retval PEC value (0-255)
  */
uint8_t i2c_pec_value_get(i2c_type *i2c_x);

/**
  * @brief  Set SMBus alert pin level
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  level: I2C_SMBUS_ALERT_HIGH or I2C_SMBUS_ALERT_LOW
  * @retval none
  */
void i2c_smbus_alert_set(i2c_type *i2c_x, i2c_smbus_alert_set_type level);
```

### Timeout Functions

```c
/**
  * @brief  Set timeout value
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  timeout: Timeout value (0x0000-0x0FFF)
  * @retval none
  */
void i2c_timeout_set(i2c_type *i2c_x, uint16_t timeout);

/**
  * @brief  Enable/disable timeout detection
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_timeout_enable(i2c_type *i2c_x, confirm_state new_state);

/**
  * @brief  Set extended timeout
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  timeout: Extended timeout value (0x0000-0x0FFF)
  * @retval none
  */
void i2c_ext_timeout_set(i2c_type *i2c_x, uint16_t timeout);

/**
  * @brief  Enable/disable extended timeout
  * @param  i2c_x: I2C1, I2C2, or I2C3
  * @param  new_state: TRUE or FALSE
  * @retval none
  */
void i2c_ext_timeout_enable(i2c_type *i2c_x, confirm_state new_state);
```

---

## Code Examples

### Example 1: Basic Master Transmit (Polling)

```c
#include "at32f435_437.h"

#define I2C_TIMEOUT    0xFFFFF
#define SLAVE_ADDRESS  0xA0

/**
  * @brief  Initialize I2C1 as master
  */
void i2c_master_init(void)
{
    gpio_init_type gpio_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_I2C1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    
    /* Configure PB6 (SCL) and PB7 (SDA) as open-drain alternate function */
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_4);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE7, GPIO_MUX_4);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOB, &gpio_init_struct);
    
    /* Initialize I2C: 15 filter cycles, 100kHz */
    i2c_init(I2C1, 0x0F, 0x80504C4E);
    
    /* Set own address (for slave mode, optional for master) */
    i2c_own_address1_set(I2C1, I2C_ADDRESS_MODE_7BIT, 0xA0);
    
    /* Enable I2C */
    i2c_enable(I2C1, TRUE);
}

/**
  * @brief  Master transmit data to slave
  */
i2c_status_type i2c_master_write(uint8_t addr, uint8_t *data, uint16_t size)
{
    uint32_t timeout;
    
    /* Wait for bus not busy */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_BUSYF_FLAG) == SET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Configure transfer: address, count, auto-stop, start+write */
    i2c_transmit_set(I2C1, addr, size, I2C_AUTO_STOP_MODE, I2C_GEN_START_WRITE);
    
    /* Send data */
    while(size > 0)
    {
        /* Wait for TDIS (ready to send) */
        timeout = I2C_TIMEOUT;
        while(i2c_flag_get(I2C1, I2C_TDIS_FLAG) == RESET)
        {
            if(i2c_flag_get(I2C1, I2C_ACKFAIL_FLAG) == SET)
            {
                i2c_flag_clear(I2C1, I2C_ACKFAIL_FLAG);
                return I2C_ERR_ACKFAIL;
            }
            if(--timeout == 0) return I2C_ERR_TIMEOUT;
        }
        
        /* Send byte */
        i2c_data_send(I2C1, *data++);
        size--;
    }
    
    /* Wait for STOP */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_STOPF_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Clear STOP flag */
    i2c_flag_clear(I2C1, I2C_STOPF_FLAG);
    
    return I2C_OK;
}

int main(void)
{
    uint8_t tx_data[] = {0x01, 0x02, 0x03, 0x04};
    
    system_clock_config();
    i2c_master_init();
    
    if(i2c_master_write(SLAVE_ADDRESS, tx_data, 4) == I2C_OK)
    {
        /* Success */
    }
    
    while(1);
}
```

---

### Example 2: Master Receive (Polling)

```c
/**
  * @brief  Master receive data from slave
  */
i2c_status_type i2c_master_read(uint8_t addr, uint8_t *data, uint16_t size)
{
    uint32_t timeout;
    
    /* Wait for bus not busy */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_BUSYF_FLAG) == SET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Configure transfer: address, count, auto-stop, start+read */
    i2c_transmit_set(I2C1, addr, size, I2C_AUTO_STOP_MODE, I2C_GEN_START_READ);
    
    /* Receive data */
    while(size > 0)
    {
        /* Wait for RDBF (data available) */
        timeout = I2C_TIMEOUT;
        while(i2c_flag_get(I2C1, I2C_RDBF_FLAG) == RESET)
        {
            if(i2c_flag_get(I2C1, I2C_ACKFAIL_FLAG) == SET)
            {
                i2c_flag_clear(I2C1, I2C_ACKFAIL_FLAG);
                return I2C_ERR_ACKFAIL;
            }
            if(--timeout == 0) return I2C_ERR_TIMEOUT;
        }
        
        /* Read byte */
        *data++ = i2c_data_receive(I2C1);
        size--;
    }
    
    /* Wait for STOP */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_STOPF_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Clear STOP flag */
    i2c_flag_clear(I2C1, I2C_STOPF_FLAG);
    
    return I2C_OK;
}
```

---

### Example 3: EEPROM Memory Read/Write

```c
#define EEPROM_ADDRESS   0xA0
#define MEM_ADDR_SIZE    1  /* 1 byte for small EEPROMs, 2 for large */

/**
  * @brief  Write to EEPROM memory address
  */
i2c_status_type eeprom_write(uint16_t mem_addr, uint8_t *data, uint16_t size)
{
    uint32_t timeout;
    
    /* Wait for bus not busy */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_BUSYF_FLAG) == SET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Configure transfer: EEPROM addr, total bytes (mem_addr + data), auto-stop */
    i2c_transmit_set(I2C1, EEPROM_ADDRESS, MEM_ADDR_SIZE + size, 
                     I2C_AUTO_STOP_MODE, I2C_GEN_START_WRITE);
    
    /* Send memory address first */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_TDIS_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    i2c_data_send(I2C1, (uint8_t)(mem_addr & 0xFF));
    
    /* Send data bytes */
    while(size > 0)
    {
        timeout = I2C_TIMEOUT;
        while(i2c_flag_get(I2C1, I2C_TDIS_FLAG) == RESET)
        {
            if(i2c_flag_get(I2C1, I2C_ACKFAIL_FLAG) == SET)
            {
                i2c_flag_clear(I2C1, I2C_ACKFAIL_FLAG);
                return I2C_ERR_ACKFAIL;
            }
            if(--timeout == 0) return I2C_ERR_TIMEOUT;
        }
        i2c_data_send(I2C1, *data++);
        size--;
    }
    
    /* Wait for STOP */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_STOPF_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    i2c_flag_clear(I2C1, I2C_STOPF_FLAG);
    
    return I2C_OK;
}

/**
  * @brief  Read from EEPROM memory address
  */
i2c_status_type eeprom_read(uint16_t mem_addr, uint8_t *data, uint16_t size)
{
    uint32_t timeout;
    
    /* Wait for bus not busy */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_BUSYF_FLAG) == SET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Step 1: Write memory address (soft stop for repeated start) */
    i2c_transmit_set(I2C1, EEPROM_ADDRESS, MEM_ADDR_SIZE, 
                     I2C_SOFT_STOP_MODE, I2C_GEN_START_WRITE);
    
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_TDIS_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    i2c_data_send(I2C1, (uint8_t)(mem_addr & 0xFF));
    
    /* Wait for TDC (transfer complete) */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_TDC_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Step 2: Read data with repeated start */
    i2c_transmit_set(I2C1, EEPROM_ADDRESS, size, 
                     I2C_AUTO_STOP_MODE, I2C_GEN_START_READ);
    
    while(size > 0)
    {
        timeout = I2C_TIMEOUT;
        while(i2c_flag_get(I2C1, I2C_RDBF_FLAG) == RESET)
        {
            if(--timeout == 0) return I2C_ERR_TIMEOUT;
        }
        *data++ = i2c_data_receive(I2C1);
        size--;
    }
    
    /* Wait for STOP */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_STOPF_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    i2c_flag_clear(I2C1, I2C_STOPF_FLAG);
    
    return I2C_OK;
}
```

---

### Example 4: DMA Master Transmit

```c
#include "at32f435_437.h"

#define I2C_TIMEOUT    0xFFFFF
#define SLAVE_ADDRESS  0xA0
#define BUF_SIZE       8

uint8_t tx_buf[BUF_SIZE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

/**
  * @brief  Initialize I2C with DMA
  */
void i2c_dma_init(void)
{
    gpio_init_type gpio_init_struct;
    dma_init_type dma_init_struct;
    
    /* Enable clocks */
    crm_periph_clock_enable(CRM_I2C1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    
    /* Configure GPIO */
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_4);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE7, GPIO_MUX_4);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOB, &gpio_init_struct);
    
    /* Configure DMA1 Channel 1 for I2C1 TX */
    dma_reset(DMA1_CHANNEL1);
    dma_init_struct.peripheral_base_addr = (uint32_t)&I2C1->txdt;
    dma_init_struct.memory_base_addr = (uint32_t)tx_buf;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.buffer_size = BUF_SIZE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    
    /* Configure DMAMUX for I2C1 TX */
    dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_I2C1_TX);
    dmamux_enable(DMA1, TRUE);
    
    /* Enable DMA interrupt */
    nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0);
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
    
    /* Initialize I2C */
    i2c_init(I2C1, 0x0F, 0x80504C4E);
    i2c_own_address1_set(I2C1, I2C_ADDRESS_MODE_7BIT, 0xA0);
    i2c_enable(I2C1, TRUE);
}

/**
  * @brief  Master transmit using DMA
  */
i2c_status_type i2c_master_transmit_dma(uint8_t addr, uint8_t *data, uint16_t size)
{
    uint32_t timeout;
    
    /* Wait for bus not busy */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_BUSYF_FLAG) == SET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Update DMA memory address and size */
    DMA1_CHANNEL1->maddr = (uint32_t)data;
    DMA1_CHANNEL1->dtcnt = size;
    
    /* Enable DMA channel */
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    
    /* Enable I2C DMA TX request */
    i2c_dma_enable(I2C1, I2C_DMA_REQUEST_TX, TRUE);
    
    /* Configure and start transfer */
    i2c_transmit_set(I2C1, addr, size, I2C_AUTO_STOP_MODE, I2C_GEN_START_WRITE);
    
    /* Wait for DMA complete (handled in ISR) or timeout */
    timeout = I2C_TIMEOUT;
    while(dma_flag_get(DMA1_FDT1_FLAG) == RESET)
    {
        if(--timeout == 0)
        {
            i2c_dma_enable(I2C1, I2C_DMA_REQUEST_TX, FALSE);
            dma_channel_enable(DMA1_CHANNEL1, FALSE);
            return I2C_ERR_TIMEOUT;
        }
    }
    dma_flag_clear(DMA1_FDT1_FLAG);
    
    /* Wait for STOP */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_STOPF_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    i2c_flag_clear(I2C1, I2C_STOPF_FLAG);
    
    /* Disable DMA */
    i2c_dma_enable(I2C1, I2C_DMA_REQUEST_TX, FALSE);
    dma_channel_enable(DMA1_CHANNEL1, FALSE);
    
    return I2C_OK;
}
```

---

### Example 5: Slave Mode (Polling)

```c
#define SLAVE_ADDRESS  0xA0
#define BUF_SIZE       8

uint8_t rx_buf[BUF_SIZE];
uint8_t tx_buf[BUF_SIZE] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};

/**
  * @brief  Initialize I2C as slave
  */
void i2c_slave_init(void)
{
    /* ... GPIO and clock init same as master ... */
    
    /* Initialize I2C */
    i2c_init(I2C1, 0x0F, 0x80504C4E);
    
    /* Set slave address */
    i2c_own_address1_set(I2C1, I2C_ADDRESS_MODE_7BIT, SLAVE_ADDRESS);
    
    /* Enable clock stretching (important for slave) */
    i2c_clock_stretch_enable(I2C1, TRUE);
    
    /* Enable I2C */
    i2c_enable(I2C1, TRUE);
}

/**
  * @brief  Slave receive data
  */
i2c_status_type i2c_slave_receive(uint8_t *data, uint16_t size)
{
    uint32_t timeout;
    
    /* Wait for address match */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_ADDRF_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Check direction (master wants to write = slave receives) */
    if(i2c_transfer_dir_get(I2C1) == I2C_DIR_TRANSMIT)
    {
        /* Clear address flag */
        i2c_flag_clear(I2C1, I2C_ADDRF_FLAG);
        
        /* Receive data */
        while(size > 0)
        {
            timeout = I2C_TIMEOUT;
            while(i2c_flag_get(I2C1, I2C_RDBF_FLAG) == RESET)
            {
                if(i2c_flag_get(I2C1, I2C_STOPF_FLAG) == SET)
                {
                    i2c_flag_clear(I2C1, I2C_STOPF_FLAG);
                    return I2C_OK;  /* Transfer ended early */
                }
                if(--timeout == 0) return I2C_ERR_TIMEOUT;
            }
            *data++ = i2c_data_receive(I2C1);
            size--;
        }
        
        /* Wait for STOP */
        timeout = I2C_TIMEOUT;
        while(i2c_flag_get(I2C1, I2C_STOPF_FLAG) == RESET)
        {
            if(--timeout == 0) return I2C_ERR_TIMEOUT;
        }
        i2c_flag_clear(I2C1, I2C_STOPF_FLAG);
    }
    
    return I2C_OK;
}

/**
  * @brief  Slave transmit data
  */
i2c_status_type i2c_slave_transmit(uint8_t *data, uint16_t size)
{
    uint32_t timeout;
    
    /* Wait for address match */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_ADDRF_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Check direction (master wants to read = slave transmits) */
    if(i2c_transfer_dir_get(I2C1) == I2C_DIR_RECEIVE)
    {
        /* Clear address flag */
        i2c_flag_clear(I2C1, I2C_ADDRF_FLAG);
        
        /* Transmit data */
        while(size > 0)
        {
            timeout = I2C_TIMEOUT;
            while(i2c_flag_get(I2C1, I2C_TDIS_FLAG) == RESET)
            {
                if(i2c_flag_get(I2C1, I2C_ACKFAIL_FLAG) == SET)
                {
                    /* Master sent NACK - end of transfer */
                    i2c_flag_clear(I2C1, I2C_ACKFAIL_FLAG);
                    return I2C_OK;
                }
                if(--timeout == 0) return I2C_ERR_TIMEOUT;
            }
            i2c_data_send(I2C1, *data++);
            size--;
        }
        
        /* Wait for STOP or NACK */
        timeout = I2C_TIMEOUT;
        while(i2c_flag_get(I2C1, I2C_STOPF_FLAG) == RESET)
        {
            if(i2c_flag_get(I2C1, I2C_ACKFAIL_FLAG) == SET)
            {
                i2c_flag_clear(I2C1, I2C_ACKFAIL_FLAG);
                break;
            }
            if(--timeout == 0) return I2C_ERR_TIMEOUT;
        }
        i2c_flag_clear(I2C1, I2C_STOPF_FLAG);
    }
    
    return I2C_OK;
}
```

---

### Example 6: SMBus with PEC

```c
#define SMBUS_ADDRESS  0xA2
#define BUF_SIZE       9  /* 8 data + 1 PEC */

/**
  * @brief  Initialize I2C for SMBus with PEC
  */
void smbus_init(void)
{
    /* ... GPIO and clock init ... */
    
    /* Initialize I2C */
    i2c_init(I2C1, 0x0F, 0x80504C4E);
    i2c_own_address1_set(I2C1, I2C_ADDRESS_MODE_7BIT, SMBUS_ADDRESS);
    
    /* Enable PEC calculation */
    i2c_pec_calculate_enable(I2C1, TRUE);
    
    /* Enable I2C */
    i2c_enable(I2C1, TRUE);
}

/**
  * @brief  SMBus master transmit with PEC
  */
i2c_status_type smbus_master_transmit(uint8_t addr, uint8_t *data, uint16_t size)
{
    uint32_t timeout;
    
    /* Wait for bus not busy */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_BUSYF_FLAG) == SET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    
    /* Enable PEC transmission (PEC byte will be sent automatically) */
    i2c_pec_transmit_enable(I2C1, TRUE);
    
    /* Configure transfer: size+1 for PEC byte */
    i2c_transmit_set(I2C1, addr, size, I2C_AUTO_STOP_MODE, I2C_GEN_START_WRITE);
    
    /* Send data */
    while(size > 0)
    {
        timeout = I2C_TIMEOUT;
        while(i2c_flag_get(I2C1, I2C_TDIS_FLAG) == RESET)
        {
            if(--timeout == 0) return I2C_ERR_TIMEOUT;
        }
        i2c_data_send(I2C1, *data++);
        size--;
    }
    
    /* Wait for STOP (PEC is sent automatically) */
    timeout = I2C_TIMEOUT;
    while(i2c_flag_get(I2C1, I2C_STOPF_FLAG) == RESET)
    {
        if(--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    i2c_flag_clear(I2C1, I2C_STOPF_FLAG);
    
    /* Disable PEC transmission */
    i2c_pec_transmit_enable(I2C1, FALSE);
    
    return I2C_OK;
}
```

---

## Pin Configuration

### I2C1

| Pin | Function | GPIO MUX |
|-----|----------|----------|
| PB6 | I2C1_SCL | GPIO_MUX_4 |
| PB7 | I2C1_SDA | GPIO_MUX_4 |
| PB8 | I2C1_SCL | GPIO_MUX_4 |
| PB9 | I2C1_SDA | GPIO_MUX_4 |

### I2C2

| Pin | Function | GPIO MUX |
|-----|----------|----------|
| PB10 | I2C2_SCL | GPIO_MUX_4 |
| PB11 | I2C2_SDA | GPIO_MUX_4 |
| PF1 | I2C2_SCL | GPIO_MUX_4 |
| PF0 | I2C2_SDA | GPIO_MUX_4 |

### I2C3

| Pin | Function | GPIO MUX |
|-----|----------|----------|
| PA8 | I2C3_SCL | GPIO_MUX_4 |
| PC9 | I2C3_SDA | GPIO_MUX_4 |
| PH7 | I2C3_SCL | GPIO_MUX_4 |
| PH8 | I2C3_SDA | GPIO_MUX_4 |

---

## Configuration Checklist

### Master Mode
- [ ] Enable GPIO and I2C peripheral clocks
- [ ] Configure SCL/SDA pins as open-drain, alternate function (MUX 4)
- [ ] Call `i2c_init()` with digital filter and clock value
- [ ] Optionally set own address with `i2c_own_address1_set()`
- [ ] Enable I2C with `i2c_enable()`
- [ ] Use `i2c_transmit_set()` to configure and start transfers

### Slave Mode
- [ ] Enable GPIO and I2C peripheral clocks
- [ ] Configure SCL/SDA pins as open-drain, alternate function
- [ ] Call `i2c_init()` with filter and clock
- [ ] Set slave address with `i2c_own_address1_set()`
- [ ] Enable clock stretching with `i2c_clock_stretch_enable()`
- [ ] Enable I2C with `i2c_enable()`
- [ ] Poll `I2C_ADDRF_FLAG` or enable address match interrupt

### DMA Mode
- [ ] Complete master/slave initialization
- [ ] Enable DMA clock
- [ ] Configure DMA channel for TX (memory→peripheral) or RX (peripheral→memory)
- [ ] Configure DMAMUX for I2Cx_TX or I2Cx_RX
- [ ] Enable DMAMUX
- [ ] Enable I2C DMA request with `i2c_dma_enable()`
- [ ] Enable DMA channel before starting transfer

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Bus always busy | SDA stuck low | Check pull-up resistors, bus recovery |
| | Previous transfer not complete | Clear flags, reset I2C |
| No ACK from slave | Wrong address | Verify slave address (7-bit vs 8-bit format) |
| | Slave not ready | Check slave power and initialization |
| | No pull-ups | Add 4.7kΩ pull-ups on SCL/SDA |
| Arbitration lost | Multi-master collision | Retry transfer |
| NACK on data | Slave buffer full | Reduce transfer speed or add delays |
| Timeout | Clock stretching too long | Increase timeout value |
| | Slave not responding | Check connections and slave |
| PEC error | Data corruption | Check signal integrity, reduce speed |
| DMA not working | DMAMUX not enabled | Call `dmamux_enable()` |
| | Wrong DMA request ID | Verify DMAMUX channel configuration |

### Bus Recovery

If SDA is stuck low, perform bus recovery:

```c
void i2c_bus_recovery(void)
{
    gpio_init_type gpio_init_struct;
    
    /* Temporarily configure SCL as GPIO output */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_6;  /* SCL pin */
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);
    
    /* Generate 9 clock pulses to release bus */
    for(int i = 0; i < 9; i++)
    {
        gpio_bits_reset(GPIOB, GPIO_PINS_6);
        delay_us(5);
        gpio_bits_set(GPIOB, GPIO_PINS_6);
        delay_us(5);
    }
    
    /* Reconfigure as I2C */
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_4);
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init(GPIOB, &gpio_init_struct);
}
```

---

## Related Peripherals

| Peripheral | Relationship |
|------------|-------------|
| [GPIO](GPIO_General_Purpose_IO.md) | SCL/SDA pin configuration |
| [CRM](CRM_Clock_Reset_Management.md) | I2C and GPIO clock enable |
| [DMA](DMA_Direct_Memory_Access.md) | DMA transfers for TX/RX |
| [DMAMUX](DMA_Direct_Memory_Access.md) | DMA request routing |
| [NVIC](Cortex_M4_Core_Features.md) | Interrupt handling |

---

## References

- AT32F435/437 Reference Manual - Chapter: I2C
- AT32F435/437 Datasheet - I2C specifications and pin mapping
- Application Note AN0096 - I2C Application Guide
- I2C-Bus Specification (NXP UM10204)
- SMBus Specification (Version 3.0)

