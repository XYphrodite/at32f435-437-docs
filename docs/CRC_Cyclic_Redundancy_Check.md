---
title: CRC (Cyclic Redundancy Check)
mcu: AT32F435/437
peripheral: CRC
version: 2.0.9
---

# CRC Cyclic Redundancy Check

## Overview

The AT32F435/437 CRC calculation unit computes CRC checksums using configurable polynomial sizes and values. It provides hardware-accelerated CRC calculation for data integrity verification, significantly faster than software implementations.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           CRC Calculation Unit                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌────────────────┐                                                          │
│  │   AHB Bus      │                                                          │
│  │   Interface    │                                                          │
│  └───────┬────────┘                                                          │
│          │                                                                   │
│          ▼                                                                   │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │                     Input Data Processing                              │  │
│  │  ┌─────────────────┐    ┌─────────────────────────────────────────┐   │  │
│  │  │  Data Register  │───▶│          Input Reversal                 │   │  │
│  │  │     (DT)        │    │  ┌────────┐ ┌────────┐ ┌────────┐      │   │  │
│  │  │   32-bit        │    │  │ None   │ │ By     │ │ By     │      │   │  │
│  │  └─────────────────┘    │  │        │ │ Byte   │ │ H-Word │      │   │  │
│  │                         │  └────────┘ └────────┘ └────────┘      │   │  │
│  │                         │             ┌────────┐                  │   │  │
│  │                         │             │ By     │                  │   │  │
│  │                         │             │ Word   │                  │   │  │
│  │                         │             └────────┘                  │   │  │
│  │                         └─────────────────────────────────────────┘   │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                      │                                       │
│                                      ▼                                       │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │                      CRC Computation Engine                            │  │
│  │  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐   │  │
│  │  │  Initial Value  │    │   Polynomial    │    │ Polynomial Size │   │  │
│  │  │     (IDT)       │    │     (POLY)      │    │   7/8/16/32-bit │   │  │
│  │  │   32-bit        │    │    32-bit       │    │                 │   │  │
│  │  └─────────────────┘    └─────────────────┘    └─────────────────┘   │  │
│  │                                                                       │  │
│  │                    ┌─────────────────────┐                            │  │
│  │                    │   CRC Calculator    │                            │  │
│  │                    │    (Hardware)       │                            │  │
│  │                    └─────────────────────┘                            │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                      │                                       │
│                                      ▼                                       │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │                     Output Data Processing                             │  │
│  │  ┌─────────────────────────────────────────┐    ┌─────────────────┐   │  │
│  │  │          Output Reversal                │───▶│  Result (DT)    │   │  │
│  │  │  ┌────────────┐  ┌────────────┐        │    │   32-bit        │   │  │
│  │  │  │   None     │  │  Reversed  │        │    └─────────────────┘   │  │
│  │  │  │            │  │  (by word) │        │                          │  │
│  │  │  └────────────┘  └────────────┘        │                          │  │
│  │  └─────────────────────────────────────────┘                          │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  ┌─────────────────┐    ┌─────────────────┐                                 │
│  │ Common Data     │    │ Control         │                                 │
│  │ Register (CDT)  │    │ Register (CTRL) │                                 │
│  │ 8-bit user data │    │ Reset, Config   │                                 │
│  └─────────────────┘    └─────────────────┘                                 │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Key Features

| Feature | Specification |
|---------|---------------|
| Polynomial Sizes | 7-bit, 8-bit, 16-bit, 32-bit |
| Polynomial | Programmable |
| Initial Value | Programmable 32-bit |
| Input Reversal | None, by byte, by halfword, by word |
| Output Reversal | None, by word |
| Common Data Register | 8-bit user storage |
| Reset Function | Hardware reset to initial value |

---

## Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| `DT` | 0x00 | Data register (input/output) |
| `CDT` | 0x04 | Common data register (8-bit user data) |
| `CTRL` | 0x08 | Control register |
| `IDT` | 0x10 | Initial data register |
| `POLY` | 0x14 | Polynomial register |

### CTRL Register Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0 | RST | Reset CRC calculation (auto-clear) |
| 4:3 | POLY_SIZE | Polynomial size selection |
| 6:5 | REVID | Input data reversal |
| 7 | REVOD | Output data reversal |

---

## Configuration Options

### Polynomial Sizes

| Enum | Value | Description |
|------|-------|-------------|
| `CRC_POLY_SIZE_32B` | 0x00 | 32-bit polynomial |
| `CRC_POLY_SIZE_16B` | 0x01 | 16-bit polynomial |
| `CRC_POLY_SIZE_8B` | 0x02 | 8-bit polynomial |
| `CRC_POLY_SIZE_7B` | 0x03 | 7-bit polynomial |

### Input Data Reversal

| Enum | Value | Description |
|------|-------|-------------|
| `CRC_REVERSE_INPUT_NO_AFFECTE` | 0x00 | No reversal |
| `CRC_REVERSE_INPUT_BY_BYTE` | 0x01 | Reverse bits in each byte |
| `CRC_REVERSE_INPUT_BY_HALFWORD` | 0x02 | Reverse bits in each 16-bit halfword |
| `CRC_REVERSE_INPUT_BY_WORD` | 0x03 | Reverse all 32 bits |

### Output Data Reversal

| Enum | Value | Description |
|------|-------|-------------|
| `CRC_REVERSE_OUTPUT_NO_AFFECTE` | 0x00 | No reversal |
| `CRC_REVERSE_OUTPUT_DATA` | 0x01 | Reverse all output bits |

---

## Common CRC Standards

| Standard | Polynomial | Size | Init Value | Input Rev | Output Rev |
|----------|------------|------|------------|-----------|------------|
| CRC-32 (IEEE 802.3) | 0x04C11DB7 | 32-bit | 0xFFFFFFFF | By byte | Yes |
| CRC-32/MPEG-2 | 0x04C11DB7 | 32-bit | 0xFFFFFFFF | None | None |
| CRC-16/CCITT | 0x1021 | 16-bit | 0xFFFF | By byte | Yes |
| CRC-16/MODBUS | 0x8005 | 16-bit | 0xFFFF | By byte | Yes |
| CRC-8 | 0x07 | 8-bit | 0x00 | None | None |
| CRC-7 (MMC) | 0x09 | 7-bit | 0x00 | None | None |

### Default Configuration

The AT32 CRC unit defaults to:
- **Polynomial:** 0x04C11DB7 (CRC-32)
- **Initial Value:** 0xFFFFFFFF
- **Input Reversal:** None
- **Output Reversal:** None

---

## API Reference

### Core Functions

| Function | Description |
|----------|-------------|
| `crc_data_reset()` | Reset CRC to initial value |
| `crc_one_word_calculate(data)` | Calculate CRC for single 32-bit word |
| `crc_block_calculate(buffer, length)` | Calculate CRC for data block |
| `crc_data_get()` | Get current CRC value |

### Configuration Functions

| Function | Description |
|----------|-------------|
| `crc_init_data_set(value)` | Set initial CRC value |
| `crc_poly_value_set(value)` | Set polynomial value |
| `crc_poly_value_get()` | Get polynomial value |
| `crc_poly_size_set(size)` | Set polynomial size (7/8/16/32-bit) |
| `crc_poly_size_get()` | Get polynomial size |
| `crc_reverse_input_data_set(type)` | Set input data reversal mode |
| `crc_reverse_output_data_set(type)` | Set output data reversal mode |

### Common Data Register

| Function | Description |
|----------|-------------|
| `crc_common_data_set(value)` | Store 8-bit user data |
| `crc_common_data_get()` | Retrieve 8-bit user data |

---

## Complete Examples

### Example 1: Basic CRC-32 Calculation

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * CRC-32 Calculation Example
 * 
 * Demonstrates hardware CRC calculation on a data buffer.
 * Uses default CRC-32 polynomial (0x04C11DB7).
 * 
 * Expected Result: 0xE5DFCF6D for the test data
 ******************************************************************************/

#define BUFFER_SIZE  120

/* Test data buffer */
static const uint32_t data_buffer[BUFFER_SIZE] = {
  0xc33dd31c, 0xe37ff35e, 0x129022f3, 0x32d24235, 0x52146277, 0x7256b5ea,
  0x4a755a54, 0x6a377a16, 0x0af11ad0, 0x2ab33a92, 0xed0fdd6c, 0xcd4dbdaa,
  0xbb3bab1a, 0x6ca67c87, 0x5cc52c22, 0x3c030c60, 0x1c41edae, 0xfd8fcdec,
  0xad8b9de8, 0x8dc97c26, 0x5c644c45, 0x3ca22c83, 0x1ce00cc1, 0xef1fff3e,
  0x95a88589, 0xf56ee54f, 0xd52cc50d, 0x34e224c3, 0x04817466, 0x64475424,
  0x78066827, 0x18c008e1, 0x28a3cb7d, 0xdb5ceb3f, 0xfb1e8bf9, 0x9bd8abbb,
  0xdf7caf9b, 0xbfba8fd9, 0x9ff86e17, 0x7e364e55, 0x2e933eb2, 0x0ed11ef0,
  0xa35ad3bd, 0xc39cf3ff, 0xe3de2462, 0x34430420, 0x64e674c7, 0x44a45485,
  0xad2abd0b, 0x8d689d49, 0x7e976eb6, 0x5ed54ef4, 0x2e321e51, 0x0e70ff9f,
  0xefbedfdd, 0xcffcbf1b, 0x9f598f78, 0x918881a9, 0xb1caa1eb, 0xd10cc12d,
  0xe16f1080, 0x00a130c2, 0x20e35004, 0x40257046, 0x83b99398, 0xa3fbb3da,
  0x00001021, 0x20423063, 0x408450a5, 0x60c670e7, 0x9129a14a, 0xb16bc18c,
  0x569546b4, 0xb75ba77a, 0x97198738, 0xf7dfe7fe, 0xc7bc48c4, 0x58e56886,
  0x4405a7db, 0xb7fa8799, 0xe75ff77e, 0xc71dd73c, 0x26d336f2, 0x069116b0,
  0x76764615, 0x5634d94c, 0xc96df90e, 0xe92f99c8, 0xb98aa9ab, 0x58444865,
  0x78a70840, 0x18612802, 0xc9ccd9ed, 0xe98ef9af, 0x89489969, 0xa90ab92b,
  0xd1ade1ce, 0xf1ef1231, 0x32732252, 0x52b54294, 0x72f762d6, 0x93398318,
  0xa56ab54b, 0x85289509, 0xf5cfc5ac, 0xd58d3653, 0x26721611, 0x063076d7,
  0x8d689d49, 0xf7dfe7fe, 0xe98ef9af, 0x063076d7, 0x93398318, 0xb98aa9ab,
  0x4ad47ab7, 0x6a961a71, 0x0a503a33, 0x2a12dbfd, 0xfbbfeb9e, 0x9b798b58
};

__IO uint32_t crc_value = 0;

int main(void)
{
  system_clock_config();
  at32_board_init();

  /* Enable CRC peripheral clock */
  crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);

  /* Reset CRC calculation unit (loads initial value) */
  crc_data_reset();

  /* Calculate CRC for entire buffer */
  crc_value = crc_block_calculate((uint32_t *)data_buffer, BUFFER_SIZE);

  /* Verify result */
  if(crc_value == 0xE5DFCF6D)
  {
    at32_led_on(LED3);  /* Success - CRC matches expected */
  }
  else
  {
    at32_led_on(LED4);  /* Error - CRC mismatch */
  }

  while(1)
  {
  }
}
```

---

### Example 2: CRC-32 with IEEE 802.3 Configuration

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * CRC-32 IEEE 802.3 (Ethernet) Configuration
 * 
 * Standard Ethernet CRC-32 with:
 * - Polynomial: 0x04C11DB7
 * - Initial value: 0xFFFFFFFF
 * - Input reversal: By byte
 * - Output reversal: Yes
 * - Final XOR: 0xFFFFFFFF (applied manually)
 ******************************************************************************/

void crc32_ieee_init(void)
{
  /* Enable CRC clock */
  crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
  
  /* Set polynomial (default is already 0x04C11DB7) */
  crc_poly_value_set(0x04C11DB7);
  
  /* Set polynomial size to 32-bit */
  crc_poly_size_set(CRC_POLY_SIZE_32B);
  
  /* Set initial value */
  crc_init_data_set(0xFFFFFFFF);
  
  /* Enable input reversal by byte (required for IEEE 802.3) */
  crc_reverse_input_data_set(CRC_REVERSE_INPUT_BY_BYTE);
  
  /* Enable output reversal */
  crc_reverse_output_data_set(CRC_REVERSE_OUTPUT_DATA);
  
  /* Reset to apply initial value */
  crc_data_reset();
}

uint32_t crc32_ieee_calculate(uint8_t *data, uint32_t length)
{
  uint32_t i;
  uint32_t crc;
  uint32_t word;
  
  /* Reset CRC */
  crc_data_reset();
  
  /* Process 4 bytes at a time */
  for(i = 0; i < length / 4; i++)
  {
    word = (data[i*4] << 0) | (data[i*4+1] << 8) | 
           (data[i*4+2] << 16) | (data[i*4+3] << 24);
    crc = crc_one_word_calculate(word);
  }
  
  /* Handle remaining bytes (if any) */
  if(length % 4 != 0)
  {
    word = 0;
    for(i = 0; i < length % 4; i++)
    {
      word |= data[(length/4)*4 + i] << (i * 8);
    }
    crc = crc_one_word_calculate(word);
  }
  
  /* Get result and apply final XOR */
  crc = crc_data_get() ^ 0xFFFFFFFF;
  
  return crc;
}

int main(void)
{
  uint8_t test_data[] = "123456789";  /* Standard test vector */
  uint32_t crc;
  
  system_clock_config();
  at32_board_init();
  
  crc32_ieee_init();
  
  crc = crc32_ieee_calculate(test_data, 9);
  
  /* Expected CRC for "123456789": 0xCBF43926 */
  if(crc == 0xCBF43926)
  {
    at32_led_on(LED3);  /* Success */
  }
  else
  {
    at32_led_on(LED4);  /* Error */
  }
  
  while(1);
}
```

---

### Example 3: CRC-16 CCITT Configuration

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * CRC-16 CCITT Configuration
 * 
 * Common CRC-16 used in protocols like X.25, HDLC:
 * - Polynomial: 0x1021
 * - Initial value: 0xFFFF
 * - Input reversal: By byte
 * - Output reversal: Yes
 ******************************************************************************/

void crc16_ccitt_init(void)
{
  crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
  
  /* Set 16-bit polynomial */
  crc_poly_size_set(CRC_POLY_SIZE_16B);
  crc_poly_value_set(0x1021);
  
  /* Set initial value (upper 16 bits for 16-bit CRC) */
  crc_init_data_set(0xFFFF);
  
  /* Enable bit reversal for CCITT compatibility */
  crc_reverse_input_data_set(CRC_REVERSE_INPUT_BY_BYTE);
  crc_reverse_output_data_set(CRC_REVERSE_OUTPUT_DATA);
  
  crc_data_reset();
}

uint16_t crc16_ccitt_calculate(uint8_t *data, uint32_t length)
{
  uint32_t i;
  uint16_t crc;
  
  crc_data_reset();
  
  /* Feed data byte by byte */
  for(i = 0; i < length; i++)
  {
    /* Write single byte - only lower 8 bits used for 16-bit CRC */
    CRC->dt = data[i];
  }
  
  /* Get 16-bit result */
  crc = (uint16_t)crc_data_get();
  
  return crc;
}

int main(void)
{
  uint8_t test_data[] = "123456789";
  uint16_t crc;
  
  system_clock_config();
  at32_board_init();
  
  crc16_ccitt_init();
  crc = crc16_ccitt_calculate(test_data, 9);
  
  /* Verify against known value */
  if(crc == 0x29B1)  /* Expected for "123456789" with CCITT settings */
  {
    at32_led_on(LED3);
  }
  else
  {
    at32_led_on(LED4);
  }
  
  while(1);
}
```

---

### Example 4: CRC-8 Configuration

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * CRC-8 Configuration
 * 
 * Simple 8-bit CRC:
 * - Polynomial: 0x07 (x^8 + x^2 + x + 1)
 * - Initial value: 0x00
 * - No reversal
 ******************************************************************************/

void crc8_init(void)
{
  crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
  
  /* Set 8-bit polynomial */
  crc_poly_size_set(CRC_POLY_SIZE_8B);
  crc_poly_value_set(0x07);
  
  /* Initial value 0 */
  crc_init_data_set(0x00);
  
  /* No reversal */
  crc_reverse_input_data_set(CRC_REVERSE_INPUT_NO_AFFECTE);
  crc_reverse_output_data_set(CRC_REVERSE_OUTPUT_NO_AFFECTE);
  
  crc_data_reset();
}

uint8_t crc8_calculate(uint8_t *data, uint32_t length)
{
  uint32_t i;
  
  crc_data_reset();
  
  for(i = 0; i < length; i++)
  {
    CRC->dt = data[i];
  }
  
  return (uint8_t)crc_data_get();
}

int main(void)
{
  uint8_t test_data[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
  uint8_t crc;
  
  system_clock_config();
  at32_board_init();
  
  crc8_init();
  crc = crc8_calculate(test_data, 9);
  
  /* Expected: 0xF4 for "123456789" with polynomial 0x07 */
  if(crc == 0xF4)
  {
    at32_led_on(LED3);
  }
  else
  {
    at32_led_on(LED4);
  }
  
  while(1);
}
```

---

### Example 5: Using Common Data Register

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * Common Data Register Example
 * 
 * The CDT register provides 8 bits of user storage that persist across
 * CRC operations. Useful for storing flags, counters, or metadata.
 ******************************************************************************/

int main(void)
{
  uint32_t data[] = {0x12345678, 0x9ABCDEF0};
  uint32_t crc;
  uint8_t packet_count = 0;
  
  system_clock_config();
  at32_board_init();
  
  crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
  
  while(1)
  {
    /* Store packet counter in common data register */
    crc_common_data_set(packet_count);
    
    /* Calculate CRC */
    crc_data_reset();
    crc = crc_block_calculate(data, 2);
    
    /* Retrieve packet counter (persists through CRC calculation) */
    if(crc_common_data_get() == packet_count)
    {
      at32_led_toggle(LED2);  /* CDT preserved correctly */
    }
    
    packet_count++;
    delay_ms(500);
  }
}
```

---

## Incremental CRC Calculation

The CRC unit supports incremental calculation - you can feed data in chunks and get a running CRC:

```c
void crc_incremental_example(void)
{
  uint32_t chunk1[] = {0x12345678, 0x9ABCDEF0};
  uint32_t chunk2[] = {0x11111111, 0x22222222};
  uint32_t crc;
  
  /* Enable and reset */
  crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
  crc_data_reset();
  
  /* Feed first chunk */
  crc_block_calculate(chunk1, 2);
  
  /* Feed second chunk (CRC accumulates) */
  crc = crc_block_calculate(chunk2, 2);
  
  /* 'crc' now contains CRC of chunk1 + chunk2 combined */
  
  /* This is equivalent to: */
  uint32_t combined[] = {0x12345678, 0x9ABCDEF0, 0x11111111, 0x22222222};
  crc_data_reset();
  crc = crc_block_calculate(combined, 4);
}
```

---

## Performance

| Operation | Cycles | Notes |
|-----------|--------|-------|
| Single word CRC | ~4 | Excluding bus access |
| Block CRC (per word) | ~4 | Pipelined |
| Reset | ~1 | Immediate |

### Hardware vs Software Comparison

| Data Size | Hardware CRC | Software CRC | Speedup |
|-----------|--------------|--------------|---------|
| 64 bytes | ~64 cycles | ~2000 cycles | ~31x |
| 1 KB | ~1024 cycles | ~32000 cycles | ~31x |
| 64 KB | ~65536 cycles | ~2M cycles | ~31x |

---

## Implementation Checklist

### Basic Setup
- [ ] Enable CRC peripheral clock (`CRM_CRC_PERIPH_CLOCK`)
- [ ] Reset CRC unit before calculation (`crc_data_reset()`)
- [ ] Use appropriate calculation function (word vs block)

### Custom Polynomial Configuration
- [ ] Set polynomial size (`crc_poly_size_set()`)
- [ ] Set polynomial value (`crc_poly_value_set()`)
- [ ] Set initial value (`crc_init_data_set()`)
- [ ] Configure input reversal if needed
- [ ] Configure output reversal if needed
- [ ] Reset after configuration changes

### Data Handling
- [ ] Ensure 32-bit alignment for block operations
- [ ] Handle remaining bytes for non-aligned data
- [ ] Apply final XOR if required by standard

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| CRC doesn't match expected | Wrong polynomial/init | Verify standard configuration |
| CRC changes unexpectedly | Missing reset between calculations | Call `crc_data_reset()` |
| Wrong result with byte data | Incorrect reversal settings | Check input/output reversal |
| Block function crashes | Unaligned pointer | Ensure 4-byte alignment |
| Different results each run | Clock not enabled | Enable `CRM_CRC_PERIPH_CLOCK` |

### Standard CRC Test Vector

The string "123456789" (ASCII bytes 0x31-0x39) is a standard test vector:

| Standard | Expected CRC |
|----------|-------------|
| CRC-32 (IEEE 802.3) | 0xCBF43926 |
| CRC-32/MPEG-2 | 0x0376E6E7 |
| CRC-16/CCITT-FALSE | 0x29B1 |
| CRC-16/MODBUS | 0x4B37 |
| CRC-8 | 0xF4 |

---

## See Also

- [ADC Documentation](./ADC_Analog_to_Digital_Converter.md)
- [CAN Documentation](./CAN_Controller_Area_Network.md)
- [Cortex-M4 Documentation](./Cortex_M4_Core_Features.md)
- AT32F435_437 Reference Manual - CRC Chapter

