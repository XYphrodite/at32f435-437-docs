---
title: Cortex-M4 Core Features
mcu: AT32F435/437
peripheral: ARM Cortex-M4 Core
version: 2.0.9
---

# Cortex-M4 Core Features

## Overview

The AT32F435/437 is based on the **ARM Cortex-M4** processor core with FPU (Floating Point Unit), running at up to **288 MHz**. This document covers the key Cortex-M4 features available on the AT32F435/437, including SysTick timer, FPU, bit-banding, and CMSIS-DSP library integration.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ARM Cortex-M4 Core                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                    CPU Core (288 MHz max)                            │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │   │
│  │  │   3-stage   │  │  Hardware   │  │   Single    │  │   Branch    │ │   │
│  │  │  Pipeline   │  │  Multiply   │  │   Cycle     │  │ Prediction  │ │   │
│  │  │ (Fetch/     │  │  (1 cycle)  │  │    MAC      │  │             │ │   │
│  │  │  Decode/    │  │  32×32=32   │  │             │  │             │ │   │
│  │  │  Execute)   │  │  32×32=64   │  │             │  │             │ │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
│  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  │
│  │   FPU (FPv4-SP)     │  │    SysTick Timer    │  │       NVIC          │  │
│  │  ┌───────────────┐  │  │  ┌───────────────┐  │  │  ┌───────────────┐  │  │
│  │  │ Single-prec.  │  │  │  │ 24-bit down   │  │  │  │ 240 external  │  │  │
│  │  │ IEEE 754      │  │  │  │   counter     │  │  │  │  interrupts   │  │  │
│  │  │ Hardware      │  │  │  │ Auto-reload   │  │  │  │  256 levels   │  │  │
│  │  │ sqrt, div     │  │  │  │ Interrupt     │  │  │  │  of priority  │  │  │
│  │  └───────────────┘  │  │  └───────────────┘  │  │  └───────────────┘  │  │
│  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘  │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                      Memory System                                    │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │   │
│  │  │  Bit-Band   │  │   MPU       │  │   Debug     │  │   Trace     │  │   │
│  │  │  Regions    │  │ (8 regions) │  │   (SWD/     │  │   (ETM/     │  │   │
│  │  │  SRAM/      │  │             │  │   JTAG)     │  │   ITM)      │  │   │
│  │  │  Periph     │  │             │  │             │  │             │  │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘  │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Key Features

| Feature | Specification |
|---------|---------------|
| Core | ARM Cortex-M4 |
| Max Frequency | 288 MHz |
| FPU | FPv4-SP (Single Precision) |
| Pipeline | 3-stage (Fetch, Decode, Execute) |
| Multiply | 1-cycle 32×32 |
| MAC | Single-cycle multiply-accumulate |
| NVIC | 240 interrupts, 256 priority levels |
| SysTick | 24-bit down counter |
| Bit-Band | SRAM + Peripheral regions |
| MPU | 8 regions |
| Debug | SWD, JTAG |
| DSP | SIMD instructions, CMSIS-DSP compatible |

---

## 1. SysTick Timer

### Overview

The SysTick is a 24-bit system timer that counts down from a reload value to zero. It provides a simple, consistent time base for RTOS scheduling or periodic interrupts.

### SysTick Registers

| Register | Address Offset | Description |
|----------|----------------|-------------|
| `SysTick->CTRL` | 0xE000E010 | Control and status |
| `SysTick->LOAD` | 0xE000E014 | Reload value (24-bit) |
| `SysTick->VAL` | 0xE000E018 | Current value |
| `SysTick->CALIB` | 0xE000E01C | Calibration value |

### CTRL Register Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0 | ENABLE | Counter enable |
| 1 | TICKINT | Exception request enable |
| 2 | CLKSOURCE | 0=AHB/8, 1=AHB |
| 16 | COUNTFLAG | Count reached zero since last read |

### Timing Calculation

```
Reload Value = (Desired_Period × Clock_Frequency) - 1

Example: 1ms tick at 288 MHz
  Reload = (0.001 × 288,000,000) - 1 = 287,999
```

### SysTick Clock Source Options

```c
/* Clock source selection */
systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);  /* Full AHB clock */
systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_DIV8);   /* AHB / 8 */
```

### Example: SysTick Interrupt (1ms Tick)

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * SysTick Interrupt Example
 * 
 * Configures SysTick for 1ms interrupts and toggles LED every 200ms.
 * Demonstrates periodic timer functionality for RTOS-like scheduling.
 ******************************************************************************/

#define MS_TICK    (system_core_clock / 1000U)  /* Ticks for 1ms */
#define DELAY      200                           /* Toggle every 200ms */

/* SysTick interrupt handler (called every 1ms) */
void systick_handler(void)
{
  static uint32_t ticks = 0;
  
  ticks++;
  
  /* Toggle LED every DELAY milliseconds */
  if(ticks > DELAY)
  {
    at32_led_toggle(LED2);
    ticks = 0;
  }
}

/* Configure SysTick with interrupt */
static uint32_t systick_interrupt_config(uint32_t ticks)
{
  /* Validate reload value (24-bit max) */
  if((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return 1UL;  /* Error: reload value too large */
  }

  /* Set reload value */
  SysTick->LOAD = (uint32_t)(ticks - 1UL);
  
  /* Set lowest priority for SysTick interrupt */
  NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
  
  /* Reset counter */
  SysTick->VAL = 0UL;
  
  /* Enable SysTick with interrupt */
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk |    /* Enable interrupt */
                   SysTick_CTRL_ENABLE_Msk;       /* Enable counter */
  
  return 0UL;  /* Success */
}

int main(void)
{
  system_clock_config();
  
  /* Select SysTick clock source (full AHB clock) */
  systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);
  
  /* Configure 1ms SysTick interrupt */
  systick_interrupt_config(MS_TICK);
  
  /* Initialize LED */
  at32_led_init(LED2);
  
  /* Main loop - work done in interrupt */
  while(1)
  {
    /* Can add low-priority tasks here */
  }
}
```

---

## 2. Floating Point Unit (FPU)

### Overview

The Cortex-M4 FPU implements the **FPv4-SP** (single-precision) floating-point extension, providing hardware acceleration for IEEE 754 single-precision operations.

### FPU Features

| Feature | Support |
|---------|---------|
| Precision | Single (32-bit float) |
| Standard | IEEE 754 |
| Add/Sub | 1 cycle |
| Multiply | 1 cycle |
| Divide | 14 cycles |
| Square Root | 14 cycles |
| MAC | 3 cycles |
| Registers | 32 × 32-bit (S0-S31) or 16 × 64-bit (D0-D15) |

### Enabling the FPU

The FPU must be enabled before use. This is typically done in the startup code.

```c
/* Enable FPU access (Coprocessor Access Control Register) */
SCB->CPACR |= ((3UL << 10*2) |   /* CP10: Full access */
               (3UL << 11*2));    /* CP11: Full access */
__DSB();  /* Data Synchronization Barrier */
__ISB();  /* Instruction Synchronization Barrier */
```

### Compiler Settings

| Compiler | Flag | Description |
|----------|------|-------------|
| GCC | `-mfpu=fpv4-sp-d16` | FPU type |
| GCC | `-mfloat-abi=hard` | Use FPU registers for args |
| Keil | `--fpu=FPv4-SP` | Enable FPU |
| IAR | `--fpu=VFPv4_sp` | Enable FPU |

### Example: Julia Set Fractal with FPU

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * FPU Julia Set Example
 * 
 * Generates Julia set fractal using hardware FPU for floating-point math.
 * Demonstrates significant performance improvement vs software FP emulation.
 * 
 * With FPU: ~10x faster than software floating point
 ******************************************************************************/

#define SCREEN_X_SIZE    320
#define SCREEN_Y_SIZE    240
#define ZOOM             100
#define ITERATION        128

/* Julia set constants */
#define REAL_CONSTANT    -0.8f
#define IMG_CONSTANT     0.156f

uint8_t buffer[SCREEN_X_SIZE * SCREEN_Y_SIZE];

/**
 * @brief Generate Julia set fractal using FPU
 * @param size_x   Width in pixels
 * @param size_y   Height in pixels
 * @param offset_x X center offset
 * @param offset_y Y center offset
 * @param zoom     Zoom level
 * @param buffer   Output buffer for iteration counts
 */
void generate_julia_fpu(uint16_t size_x, uint16_t size_y, 
                        uint16_t offset_x, uint16_t offset_y, 
                        uint16_t zoom, uint8_t *buffer)
{
  float tmp1, tmp2;
  float num_real, num_img;
  float rayon;
  uint16_t i;
  uint16_t x, y;
  
  for(y = 0; y < size_y; y++)
  {
    for(x = 0; x < size_x; x++)
    {
      /* Map pixel coordinates to complex plane */
      num_real = (float)(y - offset_y) / zoom;
      num_img = (float)(x - offset_x) / zoom;
      
      i = 0;
      rayon = 0.0f;
      
      /* Iterate Julia function: z = z² + c */
      while((i < ITERATION - 1) && (rayon < 4.0f))
      {
        tmp1 = num_real * num_real;   /* FPU: 1 cycle */
        tmp2 = num_img * num_img;     /* FPU: 1 cycle */
        
        num_img = 2.0f * num_real * num_img + IMG_CONSTANT;
        num_real = tmp1 - tmp2 + REAL_CONSTANT;
        
        rayon = tmp1 + tmp2;
        i++;
      }
      
      /* Store iteration count (can be mapped to colors) */
      buffer[x + y * size_x] = i;
    }
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  
  while(1)
  {
    at32_led_toggle(LED4);
    
    /* Generate fractal (measures FPU performance) */
    generate_julia_fpu(SCREEN_X_SIZE, SCREEN_Y_SIZE, 
                       SCREEN_X_SIZE / 2, SCREEN_Y_SIZE / 2, 
                       ZOOM, buffer);
  }
}
```

### FPU Performance Comparison

| Operation | FPU Enabled | Software Emulation |
|-----------|-------------|-------------------|
| Julia Set (320×240) | ~50 ms | ~500 ms |
| FFT (1024 points) | ~2 ms | ~25 ms |
| Matrix 4×4 multiply | ~1 µs | ~15 µs |

---

## 3. Bit-Banding

### Overview

Bit-banding allows atomic read-modify-write operations on individual bits in SRAM and peripheral memory regions. Each bit in the bit-band region is aliased to a 32-bit word in the bit-band alias region.

### Memory Regions

| Region | Base Address | Alias Base | Size |
|--------|--------------|------------|------|
| SRAM | 0x20000000 | 0x22000000 | 1 MB bit-band, 32 MB alias |
| Peripheral | 0x40000000 | 0x42000000 | 1 MB bit-band, 32 MB alias |

### Bit-Band Address Formula

```
Alias_Address = Alias_Base + (Byte_Offset × 32) + (Bit_Number × 4)

Where:
  Byte_Offset = Target_Address - Region_Base
  Bit_Number = 0-31
```

### Bit-Band Macros

```c
/* SRAM Bit-Band Macros */
#define RAM_BASE              0x20000000
#define RAM_BITBAND_BASE      0x22000000

#define SRAM_BB_ADDR(addr, bit)  \
    (RAM_BITBAND_BASE + (((addr) - RAM_BASE) * 32) + ((bit) * 4))

#define SRAM_SET_BIT(addr, bit)    \
    (*(volatile uint32_t *)SRAM_BB_ADDR(addr, bit) = 1)

#define SRAM_RESET_BIT(addr, bit)  \
    (*(volatile uint32_t *)SRAM_BB_ADDR(addr, bit) = 0)

#define SRAM_GET_BIT(addr, bit)    \
    (*(volatile uint32_t *)SRAM_BB_ADDR(addr, bit))


/* Peripheral Bit-Band Macros */
#define PERIPH_BASE           0x40000000
#define PERIPH_BITBAND_BASE   0x42000000

#define PERIPH_BB_ADDR(addr, bit)  \
    (PERIPH_BITBAND_BASE + (((addr) - PERIPH_BASE) * 32) + ((bit) * 4))

#define PERIPH_SET_BIT(addr, bit)   \
    (*(volatile uint32_t *)PERIPH_BB_ADDR(addr, bit) = 1)

#define PERIPH_RESET_BIT(addr, bit) \
    (*(volatile uint32_t *)PERIPH_BB_ADDR(addr, bit) = 0)
```

### Example: Bit-Banding for Atomic Bit Operations

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * Bit-Band Example
 * 
 * Demonstrates atomic bit manipulation using Cortex-M4 bit-banding feature.
 * - SRAM bit-banding for variables
 * - Peripheral bit-banding for GPIO control
 * 
 * Benefits:
 * - Atomic (interrupt-safe) bit operations
 * - Single instruction set/clear
 * - No read-modify-write cycles needed
 ******************************************************************************/

/* SRAM Bit-Band Definitions */
#define RAM_BASE               0x20000000
#define RAM_BITBAND_BASE       0x22000000

#define VARIABLES_RESET_BIT(addr, bit)  \
    (*(uint32_t *)(RAM_BITBAND_BASE + ((addr - RAM_BASE) * 32) + ((bit) * 4)) = 0)

#define VARIABLES_SET_BIT(addr, bit)    \
    (*(uint32_t *)(RAM_BITBAND_BASE + ((addr - RAM_BASE) * 32) + ((bit) * 4)) = 1)

#define VARIABLES_GET_BIT(addr, bit)    \
    (*(uint32_t *)(RAM_BITBAND_BASE + ((addr - RAM_BASE) * 32) + ((bit) * 4)))

/* Peripheral Bit-Band Definitions */
#define PERIPHERAL_BASE        0x40000000
#define PERIPHERAL_BITBAND_BASE 0x42000000

#define PERIPHERAL_RESET_BIT(addr, bit)  \
    (*(uint32_t *)(PERIPHERAL_BITBAND_BASE + ((addr - PERIPHERAL_BASE) * 32) + ((bit) * 4)) = 0)

#define PERIPHERAL_SET_BIT(addr, bit)    \
    (*(uint32_t *)(PERIPHERAL_BITBAND_BASE + ((addr - PERIPHERAL_BASE) * 32) + ((bit) * 4)) = 1)

/* Test variable in SRAM */
__IO uint32_t variables, variables_addr = 0;

void result_error(void)
{
  while(1)
  {
    at32_led_toggle(LED4);
    delay_sec(1);
  }
}

int main(void)
{
  system_clock_config();
  at32_board_init();
  
  /* Initialize test variable */
  variables = 0xA5A5A5A5;
  variables_addr = (uint32_t)&variables;

  /***************************************************************************
   * Test SRAM Bit-Banding
   ***************************************************************************/
  
  /* Test bit 0 */
  VARIABLES_RESET_BIT(variables_addr, 0);
  if((variables != 0xA5A5A5A4) || (VARIABLES_GET_BIT(variables_addr, 0) != 0))
  {
    result_error();
  }
  
  VARIABLES_SET_BIT(variables_addr, 0);
  if((variables != 0xA5A5A5A5) || (VARIABLES_GET_BIT(variables_addr, 0) != 1))
  {
    result_error();
  }

  /* Test bit 16 */
  VARIABLES_RESET_BIT(variables_addr, 16);
  if((variables != 0xA5A4A5A5) || (VARIABLES_GET_BIT(variables_addr, 16) != 0))
  {
    result_error();
  }
  
  VARIABLES_SET_BIT(variables_addr, 16);
  if((variables != 0xA5A5A5A5) || (VARIABLES_GET_BIT(variables_addr, 16) != 1))
  {
    result_error();
  }

  /* Test bit 31 */
  VARIABLES_RESET_BIT(variables_addr, 31);
  if((variables != 0x25A5A5A5) || (VARIABLES_GET_BIT(variables_addr, 31) != 0))
  {
    result_error();
  }
  
  VARIABLES_SET_BIT(variables_addr, 31);
  if((variables != 0xA5A5A5A5) || (VARIABLES_GET_BIT(variables_addr, 31) != 1))
  {
    result_error();
  }

  /***************************************************************************
   * Demonstrate Peripheral Bit-Banding (GPIO Control)
   ***************************************************************************/
  
  while(1)
  {
    /* Toggle LED using bit-band peripheral access (pin 13 on LED2_GPIO->odt) */
    PERIPHERAL_RESET_BIT((uint32_t)&LED2_GPIO->odt, 13);  /* LED ON */
    delay_ms(500);
    
    PERIPHERAL_SET_BIT((uint32_t)&LED2_GPIO->odt, 13);    /* LED OFF */
    delay_ms(500);
  }
}
```

### Bit-Banding vs Traditional Access

| Method | Code | Instructions | Atomic |
|--------|------|--------------|--------|
| **Bit-Band** | `*(alias) = 1;` | 1 | ✅ Yes |
| **Read-Modify-Write** | `reg |= (1 << bit);` | 3 | ❌ No |

### Use Cases for Bit-Banding

| Use Case | Benefit |
|----------|---------|
| Flag variables | Atomic set/clear without disabling interrupts |
| GPIO pins | Fast single-bit toggle |
| Peripheral control bits | Safe configuration in ISRs |
| RTOS synchronization | Lock-free bit flags |

---

## 4. CMSIS-DSP Library

### Overview

The CMSIS-DSP library provides optimized signal processing functions for Cortex-M processors. It leverages DSP instructions (SIMD, MAC) and FPU for maximum performance.

### DSP Instruction Set Features

| Instruction | Description |
|-------------|-------------|
| SIMD | Single Instruction Multiple Data (4×8-bit, 2×16-bit) |
| MAC | Single-cycle multiply-accumulate |
| SSAT/USAT | Signed/unsigned saturation |
| CLZ | Count leading zeros |
| REV | Byte/halfword reverse |

### CMSIS-DSP Categories

| Category | Functions |
|----------|-----------|
| **Basic Math** | `arm_add_f32`, `arm_sub_f32`, `arm_mult_f32`, `arm_scale_f32` |
| **Statistics** | `arm_mean_f32`, `arm_std_f32`, `arm_var_f32`, `arm_max_f32`, `arm_min_f32` |
| **Matrix** | `arm_mat_mult_f32`, `arm_mat_add_f32`, `arm_mat_trans_f32`, `arm_mat_inverse_f32` |
| **Filtering** | `arm_fir_f32`, `arm_iir_f32`, `arm_biquad_cascade_f32` |
| **FFT** | `arm_cfft_f32`, `arm_rfft_f32`, `arm_cfft_radix4_f32` |
| **Complex** | `arm_cmplx_mult_f32`, `arm_cmplx_mag_f32` |
| **Controller** | `arm_pid_f32`, `arm_sin_cos_f32` |

### Setting Up CMSIS-DSP

1. **Include Header**
```c
#include "arm_math.h"
```

2. **Link Library**
```
# GCC
-larm_cortexM4lf_math

# Keil - Add to project:
ARM::CMSIS:DSP
```

3. **Define Processor**
```c
#define ARM_MATH_CM4
#define __FPU_PRESENT 1
```

### Example: Statistical Analysis with CMSIS-DSP

```c
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"
#include "arm_math.h"

/*******************************************************************************
 * CMSIS-DSP Example
 * 
 * Demonstrates CMSIS-DSP library usage for statistical analysis.
 * Calculates: max, min, mean, standard deviation, variance
 * Uses matrix multiplication to sum columns (total marks per student).
 * 
 * Data: 20 students × 4 subjects = 80 test scores
 ******************************************************************************/

#define NUMSTUDENTS    20
#define NUMSUBJECTS    4
#define TEST_LENGTH    (NUMSTUDENTS * NUMSUBJECTS)

/* Test marks for 20 students across 4 subjects */
const float32_t testMarks_f32[TEST_LENGTH] = {
  42.0f, 37.0f, 81.0f, 28.0f,   /* Student 1 */
  83.0f, 72.0f, 36.0f, 38.0f,   /* Student 2 */
  32.0f, 51.0f, 63.0f, 64.0f,   /* Student 3 */
  97.0f, 82.0f, 95.0f, 90.0f,   /* Student 4 */
  66.0f, 51.0f, 54.0f, 42.0f,   /* Student 5 */
  67.0f, 56.0f, 45.0f, 57.0f,   /* Student 6 */
  67.0f, 69.0f, 35.0f, 52.0f,   /* Student 7 */
  29.0f, 81.0f, 58.0f, 47.0f,   /* Student 8 */
  38.0f, 76.0f, 100.0f, 29.0f,  /* Student 9 */
  33.0f, 47.0f, 29.0f, 50.0f,   /* Student 10 */
  34.0f, 41.0f, 61.0f, 46.0f,   /* Student 11 */
  52.0f, 50.0f, 48.0f, 36.0f,   /* Student 12 */
  47.0f, 55.0f, 44.0f, 40.0f,   /* Student 13 */
  100.0f, 94.0f, 84.0f, 37.0f,  /* Student 14 */
  32.0f, 71.0f, 47.0f, 77.0f,   /* Student 15 */
  31.0f, 50.0f, 49.0f, 35.0f,   /* Student 16 */
  63.0f, 67.0f, 40.0f, 31.0f,   /* Student 17 */
  29.0f, 68.0f, 61.0f, 38.0f,   /* Student 18 */
  31.0f, 28.0f, 28.0f, 76.0f,   /* Student 19 */
  55.0f, 33.0f, 29.0f, 39.0f    /* Student 20 */
};

/* Unity vector for column sum (all 1s) */
const float32_t testUnity_f32[NUMSUBJECTS] = {
  1.0f, 1.0f, 1.0f, 1.0f
};

/* Output buffer for total marks per student */
static float32_t totalMarks[NUMSTUDENTS];

/* Results */
float32_t max_marks, min_marks, mean, std, var;
uint32_t student_num;

int32_t main(void)
{
  system_clock_config();
  uart_print_init(115200);
  
  /***************************************************************************
   * Matrix Setup
   * 
   * srcA: 20×4 matrix (students × subjects)
   * srcB: 4×1 vector (all 1s)
   * dstC: 20×1 result (total marks per student)
   ***************************************************************************/
  
  arm_matrix_instance_f32 srcA = {
    .numRows = NUMSTUDENTS,
    .numCols = NUMSUBJECTS,
    .pData = (float32_t *)testMarks_f32
  };
  
  arm_matrix_instance_f32 srcB = {
    .numRows = NUMSUBJECTS,
    .numCols = 1,
    .pData = (float32_t *)testUnity_f32
  };
  
  arm_matrix_instance_f32 dstC = {
    .numRows = NUMSTUDENTS,
    .numCols = 1,
    .pData = totalMarks
  };
  
  /***************************************************************************
   * Matrix Multiplication
   * dstC = srcA × srcB
   * Sums each student's marks across all subjects
   ***************************************************************************/
  arm_mat_mult_f32(&srcA, &srcB, &dstC);
  
  /***************************************************************************
   * Statistical Analysis
   ***************************************************************************/
  
  /* Find maximum total marks and which student */
  arm_max_f32(totalMarks, NUMSTUDENTS, &max_marks, &student_num);
  
  /* Find minimum total marks */
  arm_min_f32(totalMarks, NUMSTUDENTS, &min_marks, &student_num);
  
  /* Calculate mean (average) */
  arm_mean_f32(totalMarks, NUMSTUDENTS, &mean);
  
  /* Calculate standard deviation */
  arm_std_f32(totalMarks, NUMSTUDENTS, &std);
  
  /* Calculate variance */
  arm_var_f32(totalMarks, NUMSTUDENTS, &var);
  
  /***************************************************************************
   * Output Results
   ***************************************************************************/
  printf("=== Student Performance Analysis ===\n");
  printf("Max marks:  %.2f\n", max_marks);
  printf("Min marks:  %.2f\n", min_marks);
  printf("Mean:       %.2f\n", mean);
  printf("Std Dev:    %.2f\n", std);
  printf("Variance:   %.2f\n", var);
  
  /* Expected output:
   * Max marks:  364.00 (Student 4: 97+82+95+90)
   * Min marks:  163.00 (Student 19: 31+28+28+76)
   * Mean:       213.00
   * Std Dev:    ~50
   * Variance:   ~2500
   */
  
  while(1);
}
```

### Common CMSIS-DSP Function Examples

#### Vector Operations

```c
float32_t a[4] = {1.0f, 2.0f, 3.0f, 4.0f};
float32_t b[4] = {5.0f, 6.0f, 7.0f, 8.0f};
float32_t result[4];

/* Vector add: result = a + b */
arm_add_f32(a, b, result, 4);

/* Vector multiply: result = a * b (element-wise) */
arm_mult_f32(a, b, result, 4);

/* Dot product: scalar = a · b */
float32_t dotProduct;
arm_dot_prod_f32(a, b, 4, &dotProduct);

/* Scale: result = a * 2.0 */
arm_scale_f32(a, 2.0f, result, 4);
```

#### FFT Example

```c
#define FFT_SIZE 256

float32_t fftInput[FFT_SIZE * 2];   /* Complex: real, imag, real, imag... */
float32_t fftOutput[FFT_SIZE];

arm_cfft_instance_f32 fftInstance;

/* Initialize FFT */
arm_cfft_init_f32(&fftInstance, FFT_SIZE);

/* Perform FFT */
arm_cfft_f32(&fftInstance, fftInput, 0, 1);  /* ifftFlag=0, bitReverseFlag=1 */

/* Calculate magnitude */
arm_cmplx_mag_f32(fftInput, fftOutput, FFT_SIZE);
```

#### FIR Filter Example

```c
#define NUM_TAPS    32
#define BLOCK_SIZE  128

float32_t firCoeffs[NUM_TAPS] = { /* Filter coefficients */ };
float32_t firState[NUM_TAPS + BLOCK_SIZE - 1];
float32_t input[BLOCK_SIZE];
float32_t output[BLOCK_SIZE];

arm_fir_instance_f32 firInstance;

/* Initialize FIR filter */
arm_fir_init_f32(&firInstance, NUM_TAPS, firCoeffs, firState, BLOCK_SIZE);

/* Process data */
arm_fir_f32(&firInstance, input, output, BLOCK_SIZE);
```

---

## Implementation Checklist

### SysTick Setup
- [ ] Select clock source (AHB or AHB/8)
- [ ] Calculate reload value for desired period
- [ ] Verify reload value fits in 24 bits
- [ ] Set NVIC priority for SysTick interrupt
- [ ] Enable SysTick with `SysTick->CTRL`

### FPU Setup
- [ ] Enable FPU in `CPACR` (CP10, CP11 full access)
- [ ] Configure compiler for hardware FPU
- [ ] Use `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` (GCC)
- [ ] Include `__DSB()` and `__ISB()` after enabling

### Bit-Band Setup
- [ ] Verify address is in bit-band region
- [ ] Use correct base address (0x20000000 SRAM, 0x40000000 Periph)
- [ ] Calculate alias address correctly
- [ ] Use volatile for peripheral accesses

### CMSIS-DSP Setup
- [ ] Include `arm_math.h`
- [ ] Define `ARM_MATH_CM4` and `__FPU_PRESENT`
- [ ] Link appropriate library (`arm_cortexM4lf_math`)
- [ ] Initialize structures before use

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| SysTick not firing | Interrupt not enabled | Set `TICKINT` bit in `CTRL` |
| FPU usage fault | FPU not enabled | Enable CP10/CP11 in CPACR |
| Hard fault on FP | Wrong ABI | Use `-mfloat-abi=hard` |
| Bit-band wrong value | Address calculation | Verify formula and alignment |
| CMSIS-DSP link error | Missing library | Add `arm_cortexM4lf_math.lib` |
| Slow performance | Software FP | Verify FPU enabled and used |

---

## See Also

- [ADC Documentation](./ADC_Analog_to_Digital_Converter.md)
- [CAN Documentation](./CAN_Controller_Area_Network.md)
- [ACC Documentation](./ACC_Auto_Clock_Calibration.md)
- ARM Cortex-M4 Technical Reference Manual
- CMSIS-DSP Library Documentation

