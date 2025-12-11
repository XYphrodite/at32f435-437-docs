---
title: DVP (Digital Video Port)
mcu: AT32F435/437
peripheral: DVP
version: 2.0.9
---

# DVP Digital Video Port

## Overview

The AT32F435/437 DVP (Digital Video Port) is a camera interface peripheral designed to receive parallel video data from external image sensors such as OV2640, OV5640, and other CMOS cameras. It supports multiple data formats, synchronization modes, frame rate control, and hardware image processing features including cropping, scaling, and format conversion.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           DVP Architecture                                       │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │                        Camera Interface Pins                              │   │
│  │                                                                           │   │
│  │  ┌─────────┐  ┌─────────┐  ┌──────────────────────────────────────────┐ │   │
│  │  │  PCLK   │  │ HSYNC   │  │           D[0:13] Data Bus               │ │   │
│  │  │ (Clock) │  │ (Horz)  │  │    (8/10/12/14-bit parallel data)       │ │   │
│  │  └────┬────┘  └────┬────┘  └────────────────┬─────────────────────────┘ │   │
│  │       │            │                         │                          │   │
│  │       │       ┌────┴────┐                    │                          │   │
│  │       │       │  VSYNC  │                    │                          │   │
│  │       │       │ (Vert)  │                    │                          │   │
│  │       │       └────┬────┘                    │                          │   │
│  └───────┼────────────┼─────────────────────────┼──────────────────────────┘   │
│          │            │                         │                               │
│          ▼            ▼                         ▼                               │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │                     Synchronization Control                               │   │
│  │  ┌───────────────────────┐    ┌───────────────────────────────────────┐ │   │
│  │  │    Hardware Sync      │    │      Embedded Sync (SAV/EAV)          │ │   │
│  │  │  (VSYNC/HSYNC pins)   │ OR │   (Sync codes in data stream)        │ │   │
│  │  └───────────────────────┘    └───────────────────────────────────────┘ │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                         │                                        │
│                                         ▼                                        │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │                        Data Processing Pipeline                           │   │
│  │                                                                           │   │
│  │  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐  │   │
│  │  │   Window    │──▶│   Zoom Out  │──▶│   Format    │──▶│   Scaling   │  │   │
│  │  │   Crop      │   │   (Pixel/   │   │  Conversion │   │   Resize    │  │   │
│  │  │             │   │    Line)    │   │  YUV/RGB/Y8 │   │             │  │   │
│  │  └─────────────┘   └─────────────┘   └─────────────┘   └─────────────┘  │   │
│  │                                                                           │   │
│  │  ┌─────────────┐   ┌─────────────┐   ┌─────────────────────────────────┐│   │
│  │  │ Frame Rate  │   │ Binarization│   │        FIFO Buffer (32-bit)    ││   │
│  │  │  Control    │   │ (Mono B&W)  │   │    4-word depth, DMA ready     ││   │
│  │  └─────────────┘   └─────────────┘   └─────────────────────────────────┘│   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                         │                                        │
│                                         ▼                                        │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │                         DMA Interface                                     │   │
│  │  ┌─────────────────────────────────────────────────────────────────────┐│   │
│  │  │    DVP Data Register (DT) ──► EDMA/DMA ──► Memory / LCD / XMC      ││   │
│  │  │         @ DVP_BASE + 0x28                                           ││   │
│  │  └─────────────────────────────────────────────────────────────────────┘│   │
│  │                                                                           │   │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐        │   │
│  │  │  Single    │  │   Burst    │  │  Double    │  │  Linked    │        │   │
│  │  │  Transfer  │  │  Transfer  │  │  Buffer    │  │   List     │        │   │
│  │  └────────────┘  └────────────┘  └────────────┘  └────────────┘        │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                                                                  │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │                           Interrupts                                      │   │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐           │   │
│  │  │  CFD    │ │  OVR    │ │  ESE    │ │   VS    │ │   HS    │           │   │
│  │  │ Frame   │ │ Overrun │ │ Sync    │ │ V-Sync  │ │ H-Sync  │           │   │
│  │  │  Done   │ │  Error  │ │  Error  │ │  Event  │ │  Event  │           │   │
│  │  └─────────┘ └─────────┘ └─────────┘ └─────────┘ └─────────┘           │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Key Features

| Feature | Specification |
|---------|---------------|
| Data Width | 8, 10, 12, or 14 bits parallel |
| Synchronization | Hardware (VSYNC/HSYNC) or Embedded (SAV/EAV) |
| Capture Modes | Continuous or Single Frame |
| Frame Rate Control | All, 1/2, 1/4 (basic); N/M (enhanced) |
| Window Cropping | Programmable start position and size |
| Pixel Decimation | All, 1/2, 1/4, 2/4 |
| Line Decimation | All, 1/2 |
| Data Formats | Bypass, YUV422 (UYVY/YUYV), RGB565/555, Y8 |
| Image Scaling | Hardware resize (source to target) |
| Binarization | Threshold-based monochrome conversion |
| DMA Mode | Single or Burst transfers |
| FIFO Depth | 4 x 32-bit words |

---

## Hardware Resources

### DVP Pins

| Signal | Pin Options | Description |
|--------|-------------|-------------|
| DVP_PCLK | PA6 | Pixel Clock (from camera) |
| DVP_HSYNC | PA4 | Horizontal Sync |
| DVP_VSYNC | PB7 | Vertical Sync |
| DVP_D0 | PA9 | Data bit 0 |
| DVP_D1 | PA10 | Data bit 1 |
| DVP_D2 | PA11 | Data bit 2 |
| DVP_D3 | PA12 | Data bit 3 |
| DVP_D4 | PE4 | Data bit 4 |
| DVP_D5 | PB4 | Data bit 5 |
| DVP_D6 | PE5 | Data bit 6 |
| DVP_D7 | PB9 | Data bit 7 |
| DVP_D8-D13 | Various | Extended data bits (10-14 bit modes) |

> **Note:** All DVP pins use GPIO_MUX_13 alternate function.

### Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| `CTRL` | 0x00 | Control register |
| `STS` | 0x04 | Basic status register |
| `ESTS` | 0x08 | Event status register |
| `IER` | 0x0C | Interrupt enable register |
| `ISTS` | 0x10 | Interrupt status register |
| `ICLR` | 0x14 | Interrupt clear register |
| `SCR` | 0x18 | Sync code register |
| `SUR` | 0x1C | Sync unmask register |
| `CWST` | 0x20 | Crop window start register |
| `CWSZ` | 0x24 | Crop window size register |
| `DT` | 0x28 | Data register (DMA source) |
| `ACTRL` | 0x40 | Advanced control register |
| `HSCF` | 0x48 | Horizontal scaling config |
| `VSCF` | 0x4C | Vertical scaling config |
| `FRF` | 0x50 | Frame rate control |
| `BTH` | 0x54 | Binarization threshold |

### DMA Requests

| DMA Request | Description |
|-------------|-------------|
| `EDMAMUX_DMAREQ_ID_DVP` | DVP data ready (use with EDMA) |

---

## Configuration Options

### Capture Mode

| Enum | Value | Description |
|------|-------|-------------|
| `DVP_CAP_FUNC_MODE_CONTINUOUS` | 0x00 | Capture frames continuously |
| `DVP_CAP_FUNC_MODE_SINGLE` | 0x01 | Capture single frame then stop |

### Synchronization Mode

| Enum | Value | Description |
|------|-------|-------------|
| `DVP_SYNC_MODE_HARDWARE` | 0x00 | Use VSYNC/HSYNC pins |
| `DVP_SYNC_MODE_EMBEDDED` | 0x01 | Sync codes in data stream |

### Pixel Data Length

| Enum | Value | Description |
|------|-------|-------------|
| `DVP_PIXEL_DATA_LENGTH_8` | 0x00 | 8-bit data (D0-D7) |
| `DVP_PIXEL_DATA_LENGTH_10` | 0x01 | 10-bit data (D0-D9) |
| `DVP_PIXEL_DATA_LENGTH_12` | 0x02 | 12-bit data (D0-D11) |
| `DVP_PIXEL_DATA_LENGTH_14` | 0x03 | 14-bit data (D0-D13) |

### Clock/Sync Polarity

| Function | Options | Description |
|----------|---------|-------------|
| `dvp_pclk_polarity_set()` | `DVP_CLK_POLARITY_RISING`, `DVP_CLK_POLARITY_FALLING` | Sample on rising/falling PCLK |
| `dvp_hsync_polarity_set()` | `DVP_HSYNC_POLARITY_HIGH`, `DVP_HSYNC_POLARITY_LOW` | HSYNC active level |
| `dvp_vsync_polarity_set()` | `DVP_VSYNC_POLARITY_HIGH`, `DVP_VSYNC_POLARITY_LOW` | VSYNC active level |

### Pixel Capture/Drop Control (Zoom Out)

| Enum | Value | Description |
|------|-------|-------------|
| `DVP_PCDC_ALL` | 0x00 | Capture all pixels |
| `DVP_PCDC_ONE_IN_TWO` | 0x01 | Capture 1 in 2 (50%) |
| `DVP_PCDC_ONE_IN_FOUR` | 0x02 | Capture 1 in 4 (25%) |
| `DVP_PCDC_TWO_IN_FOUR` | 0x03 | Capture 2 in 4 (50%) |

### Line Capture/Drop Control

| Enum | Value | Description |
|------|-------|-------------|
| `DVP_LCDC_ALL` | 0x00 | Capture all lines |
| `DVP_LCDC_ONE_IN_TWO` | 0x01 | Capture 1 in 2 lines (50%) |

### Basic Frame Rate Control

| Enum | Value | Description |
|------|-------|-------------|
| `DVP_BFRC_ALL` | 0x00 | Capture all frames |
| `DVP_BFRC_HALF` | 0x01 | Capture every other frame |
| `DVP_BFRC_QUARTER` | 0x02 | Capture 1 in 4 frames |

### Enhanced Data Format

| Enum | Value | Description |
|------|-------|-------------|
| `DVP_EFDF_BYPASS` | 0x00 | Pass data unchanged |
| `DVP_EFDF_YUV422_UYVY` | 0x04 | YUV422 format (U-Y-V-Y order) |
| `DVP_EFDF_YUV422_YUYV` | 0x05 | YUV422 format (Y-U-Y-V order) |
| `DVP_EFDF_RGB565_555` | 0x06 | RGB565 or RGB555 format |
| `DVP_EFDF_Y8` | 0x07 | 8-bit grayscale (Y only) |

---

## Interrupts and Events

### Event Flags

| Flag | Description |
|------|-------------|
| `DVP_CFD_EVT_FLAG` | Capture Frame Done event |
| `DVP_OVR_EVT_FLAG` | Data FIFO Overrun event |
| `DVP_ESE_EVT_FLAG` | Embedded Sync Error event |
| `DVP_VS_EVT_FLAG` | Vertical Sync event |
| `DVP_HS_EVT_FLAG` | Horizontal Sync event |

### Interrupt Flags

| Flag | Description |
|------|-------------|
| `DVP_CFD_INT_FLAG` | Capture Frame Done interrupt |
| `DVP_OVR_INT_FLAG` | Data FIFO Overrun interrupt |
| `DVP_ESE_INT_FLAG` | Embedded Sync Error interrupt |
| `DVP_VS_INT_FLAG` | Vertical Sync interrupt |
| `DVP_HS_INT_FLAG` | Horizontal Sync interrupt |

### Interrupt Enables

| Interrupt | Description |
|-----------|-------------|
| `DVP_CFD_INT` | Frame capture complete |
| `DVP_OVR_INT` | FIFO overflow (data loss) |
| `DVP_ESE_INT` | Embedded sync code error |
| `DVP_VS_INT` | Vertical sync detected |
| `DVP_HS_INT` | Horizontal sync detected |

---

## API Reference

### Core Functions

| Function | Description |
|----------|-------------|
| `dvp_reset()` | Reset DVP peripheral to default state |
| `dvp_enable(state)` | Enable/disable DVP peripheral |
| `dvp_capture_enable(state)` | Start/stop image capture |
| `dvp_capture_mode_set(mode)` | Set continuous or single capture |

### Synchronization Functions

| Function | Description |
|----------|-------------|
| `dvp_sync_mode_set(mode)` | Set hardware or embedded sync |
| `dvp_sync_code_set(fmsc, fmec, lnsc, lnec)` | Set embedded sync codes |
| `dvp_sync_unmask_set(fmsu, fmeu, lnsu, lneu)` | Set sync code masks |
| `dvp_pclk_polarity_set(edge)` | Set pixel clock sampling edge |
| `dvp_hsync_polarity_set(pol)` | Set HSYNC active polarity |
| `dvp_vsync_polarity_set(pol)` | Set VSYNC active polarity |

### Data Configuration Functions

| Function | Description |
|----------|-------------|
| `dvp_pixel_data_length_set(length)` | Set data bus width (8/10/12/14) |
| `dvp_enhanced_data_format_set(format)` | Set YUV/RGB/Y8 conversion |
| `dvp_input_data_unused_set(idus, idun)` | Configure unused data bits |
| `dvp_dma_burst_set(mode)` | Set DMA transfer mode |

### Window and Cropping Functions

| Function | Description |
|----------|-------------|
| `dvp_window_crop_enable(state)` | Enable/disable window cropping |
| `dvp_window_crop_set(x, y, w, h, bytes)` | Set crop region |

### Zoom and Decimation Functions

| Function | Description |
|----------|-------------|
| `dvp_zoomout_set(pcdc, pcds, lcdc, lcds)` | Configure pixel/line decimation |
| `dvp_zoomout_select(pcdes)` | Extended pixel selection |
| `dvp_basic_frame_rate_control_set(rate)` | Set basic frame rate (all/half/quarter) |

### Enhanced Processing Functions

| Function | Description |
|----------|-------------|
| `dvp_enhanced_scaling_resize_enable(state)` | Enable image scaling |
| `dvp_enhanced_scaling_resize_set(src_w, des_w, src_h, des_h)` | Configure scaling parameters |
| `dvp_enhanced_framerate_set(efrcsf, efrctf, state)` | Fine frame rate control (N/M) |
| `dvp_monochrome_image_binarization_set(threshold, state)` | Enable B/W conversion |

### Status and Interrupt Functions

| Function | Description |
|----------|-------------|
| `dvp_basic_status_get(status)` | Get HSYN/VSYN/OFNE status |
| `dvp_interrupt_enable(int, state)` | Enable/disable interrupts |
| `dvp_flag_get(flag)` | Get event flag status |
| `dvp_interrupt_flag_get(flag)` | Get interrupt flag status |
| `dvp_flag_clear(flag)` | Clear event/interrupt flags |
| `dvp_sync_event_interrupt_set(hseid, vseid)` | Configure sync interrupt timing |

---

## Complete Examples

### Example 1: OV5640 Camera Capture to LCD

```c
#include "at32f435_437.h"
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/*******************************************************************************
 * OV5640 Camera Capture Example
 * 
 * Captures RGB565 video from OV5640 camera and displays on LCD via XMC.
 * Uses EDMA for high-speed data transfer.
 * 
 * Hardware:
 * - AT-START-F435 board
 * - OV5640 camera module
 * - LCD display (320x240)
 ******************************************************************************/

#define LCD_W  320
#define LCD_H  240

/*-----------------------------------------------------------------------------
 * DVP GPIO Initialization
 *---------------------------------------------------------------------------*/
void dvp_io_init(void)
{
  gpio_init_type gpio_init_struct;

  /* Enable peripheral clocks */
  crm_periph_clock_enable(CRM_DVP_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);

  /* Configure DVP pins for alternate function */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE4,  GPIO_MUX_13);  /* HSYNC */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE6,  GPIO_MUX_13);  /* PCLK */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9,  GPIO_MUX_13);  /* D0 */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_13);  /* D1 */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_13);  /* D2 */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_13);  /* D3 */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4,  GPIO_MUX_13);  /* D5 */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE7,  GPIO_MUX_13);  /* VSYNC */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9,  GPIO_MUX_13);  /* D7 */
  gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE4,  GPIO_MUX_13);  /* D4 */
  gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE5,  GPIO_MUX_13);  /* D6 */

  /* Configure GPIO: Alternate function, Push-Pull, Pull-Up */
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_6 | GPIO_PINS_9 | 
                                GPIO_PINS_10 | GPIO_PINS_11 | GPIO_PINS_12;
  gpio_init(GPIOA, &gpio_init_struct);

  gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_7 | GPIO_PINS_9;
  gpio_init(GPIOB, &gpio_init_struct);

  gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOE, &gpio_init_struct);
}

/*-----------------------------------------------------------------------------
 * DVP Configuration for RGB565 Capture
 *---------------------------------------------------------------------------*/
void dvp_config(void)
{
  /* Initialize DVP GPIO */
  dvp_io_init();

  /* Enable DVP interrupt */
  nvic_irq_enable(DVP_IRQn, 0, 0);

  /* Configure capture mode: continuous capture */
  dvp_capture_mode_set(DVP_CAP_FUNC_MODE_CONTINUOUS);

  /* Configure polarities for OV5640
   * - HSYNC: Active LOW
   * - VSYNC: Active LOW  
   * - PCLK: Sample on rising edge
   */
  dvp_hsync_polarity_set(DVP_HSYNC_POLARITY_LOW);
  dvp_vsync_polarity_set(DVP_VSYNC_POLARITY_LOW);
  dvp_pclk_polarity_set(DVP_CLK_POLARITY_RISING);

  /* No pixel/line decimation - capture all data */
  dvp_zoomout_set(DVP_PCDC_ALL, DVP_PCDS_CAP_FIRST, 
                  DVP_LCDC_ALL, DVP_LCDS_CAP_FIRST);
  dvp_zoomout_select(DVP_PCDES_CAP_FIRST);

  /* 8-bit data bus */
  dvp_pixel_data_length_set(DVP_PIXEL_DATA_LENGTH_8);

  /* Hardware synchronization mode */
  dvp_sync_mode_set(DVP_SYNC_MODE_HARDWARE);

  /* Enable all DVP interrupts */
  dvp_interrupt_enable(DVP_CFD_INT | DVP_OVR_INT | DVP_ESE_INT | 
                       DVP_VS_INT | DVP_HS_INT, TRUE);

  /* Enable DVP peripheral */
  dvp_enable(TRUE);
}

/*-----------------------------------------------------------------------------
 * EDMA Configuration for DVP to LCD Transfer
 *---------------------------------------------------------------------------*/
void dvp_dma_init(uint32_t mem_addr, uint16_t buffer_size)
{
  edma_init_type edma_init_struct;

  /* Enable EDMA clock */
  crm_periph_clock_enable(CRM_EDMA_PERIPH_CLOCK, TRUE);

  /* Configure EDMA Stream 4 for DVP */
  edma_init_struct.peripheral_base_addr = (uint32_t)&DVP->dt;
  edma_init_struct.memory0_base_addr = mem_addr;
  edma_init_struct.buffer_size = buffer_size;
  edma_init_struct.direction = EDMA_DIR_PERIPHERAL_TO_MEMORY;
  edma_init_struct.peripheral_inc_enable = FALSE;
  edma_init_struct.memory_inc_enable = FALSE;  /* LCD address fixed */
  edma_init_struct.peripheral_data_width = EDMA_PERIPHERAL_DATA_WIDTH_WORD;
  edma_init_struct.memory_data_width = EDMA_MEMORY_DATA_WIDTH_HALFWORD;
  edma_init_struct.loop_mode_enable = TRUE;
  edma_init_struct.priority = EDMA_PRIORITY_HIGH;
  edma_init_struct.fifo_mode_enable = TRUE;
  edma_init_struct.fifo_threshold = EDMA_FIFO_THRESHOLD_FULL;
  edma_init_struct.memory_burst_mode = EDMA_MEMORY_SINGLE;
  edma_init_struct.peripheral_burst_mode = EDMA_PERIPHERAL_SINGLE;

  edma_reset(EDMA_STREAM4);
  edma_init(EDMA_STREAM4, &edma_init_struct);

  /* Configure EDMAMUX for DVP request */
  edmamux_enable(TRUE);
  edmamux_init(EDMAMUX_CHANNEL4, EDMAMUX_DMAREQ_ID_DVP);

  /* Enable DMA transfer complete interrupt */
  edma_interrupt_enable(EDMA_STREAM4, EDMA_FDT_INT, TRUE);
  nvic_irq_enable(EDMA_Stream4_IRQn, 0, 0);
}

/*-----------------------------------------------------------------------------
 * Start DVP Capture
 *---------------------------------------------------------------------------*/
void dvp_start(void)
{
  /* Enable EDMA stream */
  edma_stream_enable(EDMA_STREAM4, TRUE);
  
  /* Start DVP capture */
  dvp_capture_enable(TRUE);
}

/*-----------------------------------------------------------------------------
 * Stop DVP Capture
 *---------------------------------------------------------------------------*/
void dvp_stop(void)
{
  /* Disable capture */
  dvp_capture_enable(FALSE);
  
  /* Wait for capture to stop */
  while(DVP->ctrl & 0x01);
  
  /* Disable EDMA stream */
  edma_stream_enable(EDMA_STREAM4, FALSE);
}

/*-----------------------------------------------------------------------------
 * DVP Interrupt Handler
 *---------------------------------------------------------------------------*/
uint32_t frame_count = 0;

void DVP_IRQHandler(void)
{
  /* Synchronization error */
  if(dvp_interrupt_flag_get(DVP_ESE_INT_FLAG) != RESET)
  {
    dvp_flag_clear(DVP_ESE_INT_FLAG);
    /* Handle sync error */
  }

  /* FIFO overrun - data lost */
  if(dvp_interrupt_flag_get(DVP_OVR_INT_FLAG) != RESET)
  {
    dvp_flag_clear(DVP_OVR_INT_FLAG);
    /* Handle overrun - may need to restart */
  }

  /* Horizontal sync (line complete) */
  if(dvp_interrupt_flag_get(DVP_HS_INT_FLAG) != RESET)
  {
    dvp_flag_clear(DVP_HS_INT_FLAG);
  }

  /* Vertical sync (frame boundary) */
  if(dvp_interrupt_flag_get(DVP_VS_INT_FLAG) != RESET)
  {
    dvp_flag_clear(DVP_VS_INT_FLAG);
  }

  /* Frame capture complete */
  if(dvp_interrupt_flag_get(DVP_CFD_INT_FLAG) != RESET)
  {
    dvp_flag_clear(DVP_CFD_INT_FLAG);
    frame_count++;
    /* Frame done - can process or display */
  }
}

/*-----------------------------------------------------------------------------
 * EDMA Interrupt Handler
 *---------------------------------------------------------------------------*/
void EDMA_Stream4_IRQHandler(void)
{
  if(edma_interrupt_flag_get(EDMA_FDT4_FLAG) != RESET)
  {
    edma_flag_clear(EDMA_FDT4_FLAG);
  }
}

/*-----------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------*/
int main(void)
{
  system_clock_config();
  at32_board_init();
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  /* Initialize camera (OV5640 specific - not shown) */
  /* ov5640_init(); */
  /* ov5640_rgb565_mode(); */
  /* ov5640_outsize_set(0, 0, LCD_W, LCD_H); */

  /* Configure DVP */
  dvp_config();

  /* Configure DMA - LCD data address, buffer for one frame */
  /* dvp_dma_init((uint32_t)XMC_LCD_DATA, LCD_W * LCD_H / 2); */

  /* Start capture */
  dvp_start();

  while(1)
  {
    /* Application code */
  }
}
```

---

### Example 2: Embedded Synchronization Mode

```c
/*******************************************************************************
 * DVP with Embedded Synchronization
 * 
 * Some cameras use embedded sync codes (SAV/EAV) instead of HSYNC/VSYNC pins.
 * This example shows how to configure DVP for embedded sync mode.
 ******************************************************************************/

void dvp_embedded_sync_config(void)
{
  dvp_io_init();

  /* Configure for embedded synchronization */
  dvp_sync_mode_set(DVP_SYNC_MODE_EMBEDDED);

  /*=========================================================================
   * Embedded Sync Code Configuration
   * 
   * Typical ITU-R BT.656 sync codes:
   * - Frame Start: 0xFF (or 0x00 depending on standard)
   * - Frame End:   0xFF
   * - Line Start:  0xAB (SAV - Start of Active Video)
   * - Line End:    0xB6 (EAV - End of Active Video)
   * 
   * Adjust these values based on your camera's output format!
   *=========================================================================*/
  dvp_sync_code_set(
    0xFF,   /* FMSC: Frame Start Code */
    0xFF,   /* FMEC: Frame End Code */
    0xC7,   /* LNSC: Line Start Code (example) */
    0xDA    /* LNEC: Line End Code (example) */
  );

  /* Sync code unmask - which bits to check (0xFF = check all bits) */
  dvp_sync_unmask_set(0xFF, 0xFF, 0xFF, 0xFF);

  /* Rest of configuration */
  dvp_capture_mode_set(DVP_CAP_FUNC_MODE_CONTINUOUS);
  dvp_pclk_polarity_set(DVP_CLK_POLARITY_RISING);
  dvp_pixel_data_length_set(DVP_PIXEL_DATA_LENGTH_8);
  
  dvp_interrupt_enable(DVP_CFD_INT | DVP_ESE_INT, TRUE);
  dvp_enable(TRUE);
}
```

---

### Example 3: Window Cropping

```c
/*******************************************************************************
 * DVP Window Cropping Example
 * 
 * Captures only a portion of the camera image.
 * Useful for extracting regions of interest.
 ******************************************************************************/

void dvp_crop_config(uint16_t start_x, uint16_t start_y, 
                     uint16_t width, uint16_t height)
{
  dvp_io_init();

  /* Basic DVP configuration */
  dvp_capture_mode_set(DVP_CAP_FUNC_MODE_CONTINUOUS);
  dvp_sync_mode_set(DVP_SYNC_MODE_HARDWARE);
  dvp_hsync_polarity_set(DVP_HSYNC_POLARITY_LOW);
  dvp_vsync_polarity_set(DVP_VSYNC_POLARITY_LOW);
  dvp_pclk_polarity_set(DVP_CLK_POLARITY_RISING);
  dvp_pixel_data_length_set(DVP_PIXEL_DATA_LENGTH_8);

  /*=========================================================================
   * Window Cropping Configuration
   * 
   * Parameters:
   * - start_x: Horizontal start position (pixels)
   * - start_y: Vertical start position (lines)
   * - width: Crop window width (pixels)
   * - height: Crop window height (lines)
   * - bytes: Bytes per pixel (1 for Y8, 2 for RGB565/YUV422)
   *=========================================================================*/
  dvp_window_crop_set(start_x, start_y, width, height, 2);  /* RGB565 = 2 bytes */
  dvp_window_crop_enable(TRUE);

  dvp_enable(TRUE);
}

/* Example: Crop 160x120 window from center of 320x240 image */
void crop_center_example(void)
{
  uint16_t full_width = 320;
  uint16_t full_height = 240;
  uint16_t crop_width = 160;
  uint16_t crop_height = 120;
  
  uint16_t start_x = (full_width - crop_width) / 2;    /* = 80 */
  uint16_t start_y = (full_height - crop_height) / 2;  /* = 60 */
  
  dvp_crop_config(start_x, start_y, crop_width, crop_height);
}
```

---

### Example 4: Image Zoom Out (Decimation)

```c
/*******************************************************************************
 * DVP Zoom Out (Decimation) Example
 * 
 * Reduces image resolution by dropping pixels and lines.
 * Useful for preview mode or reducing data bandwidth.
 ******************************************************************************/

void dvp_zoomout_config(void)
{
  dvp_io_init();

  /* Basic configuration */
  dvp_capture_mode_set(DVP_CAP_FUNC_MODE_CONTINUOUS);
  dvp_sync_mode_set(DVP_SYNC_MODE_HARDWARE);
  dvp_pixel_data_length_set(DVP_PIXEL_DATA_LENGTH_8);

  /*=========================================================================
   * Zoom Out Configuration
   * 
   * This example captures 1 in 2 pixels AND 1 in 2 lines
   * Result: 4x reduction (320x240 → 160x120)
   *=========================================================================*/
  dvp_zoomout_set(
    DVP_PCDC_ONE_IN_TWO,   /* Capture 1 pixel, drop 1 pixel */
    DVP_PCDS_CAP_FIRST,    /* Capture first pixel in pair */
    DVP_LCDC_ONE_IN_TWO,   /* Capture 1 line, drop 1 line */
    DVP_LCDS_CAP_FIRST     /* Capture first line in pair */
  );

  dvp_enable(TRUE);
}

/* Alternative: 2x horizontal, no vertical decimation */
void dvp_zoomout_2x_horizontal(void)
{
  dvp_zoomout_set(
    DVP_PCDC_ONE_IN_TWO,   /* 50% horizontal */
    DVP_PCDS_CAP_FIRST,
    DVP_LCDC_ALL,          /* 100% vertical (all lines) */
    DVP_LCDS_CAP_FIRST
  );
}

/* 4x horizontal decimation */
void dvp_zoomout_4x_horizontal(void)
{
  dvp_zoomout_set(
    DVP_PCDC_ONE_IN_FOUR,  /* 25% horizontal */
    DVP_PCDS_CAP_FIRST,
    DVP_LCDC_ALL,
    DVP_LCDS_CAP_FIRST
  );
}
```

---

### Example 5: Enhanced Image Scaling

```c
/*******************************************************************************
 * DVP Enhanced Image Scaling Example
 * 
 * Hardware image resizing from source to target resolution.
 * More flexible than simple decimation.
 ******************************************************************************/

void dvp_scaling_config(uint16_t src_width, uint16_t src_height,
                        uint16_t dst_width, uint16_t dst_height)
{
  dvp_io_init();

  /* Basic configuration */
  dvp_capture_mode_set(DVP_CAP_FUNC_MODE_CONTINUOUS);
  dvp_sync_mode_set(DVP_SYNC_MODE_HARDWARE);
  dvp_pixel_data_length_set(DVP_PIXEL_DATA_LENGTH_8);

  /* IMPORTANT: Disable basic decimation when using enhanced scaling */
  dvp_zoomout_set(DVP_PCDC_ALL, DVP_PCDS_CAP_FIRST, 
                  DVP_LCDC_ALL, DVP_LCDS_CAP_FIRST);

  /* Enable data format processing (required for scaling) */
  dvp_enhanced_data_format_set(DVP_EFDF_RGB565_555);

  /*=========================================================================
   * Enhanced Scaling Configuration
   * 
   * Parameters:
   * - src_width: Source image width (from camera)
   * - dst_width: Target image width (after scaling)
   * - src_height: Source image height
   * - dst_height: Target image height
   * 
   * Scaling ratio = dst / src
   * Example: 640x480 → 320x240 = 0.5x scaling (50%)
   *=========================================================================*/
  dvp_enhanced_scaling_resize_set(src_width, dst_width, src_height, dst_height);
  dvp_enhanced_scaling_resize_enable(TRUE);

  dvp_enable(TRUE);
}

/* Example: Scale VGA (640x480) to QVGA (320x240) */
void scale_vga_to_qvga(void)
{
  dvp_scaling_config(640, 480, 320, 240);
}

/* Example: Scale 800x600 to 400x300 */
void scale_svga_to_half(void)
{
  dvp_scaling_config(800, 600, 400, 300);
}
```

---

### Example 6: Frame Rate Control

```c
/*******************************************************************************
 * DVP Frame Rate Control Example
 * 
 * Reduces effective frame rate to reduce processing load or match display.
 ******************************************************************************/

void dvp_framerate_config(void)
{
  dvp_io_init();

  /* MUST use continuous mode for frame rate control */
  dvp_capture_mode_set(DVP_CAP_FUNC_MODE_CONTINUOUS);
  dvp_sync_mode_set(DVP_SYNC_MODE_HARDWARE);
  dvp_pixel_data_length_set(DVP_PIXEL_DATA_LENGTH_8);

  /*=========================================================================
   * Basic Frame Rate Control
   * 
   * Options:
   * - DVP_BFRC_ALL: Capture all frames (100%)
   * - DVP_BFRC_HALF: Capture every other frame (50%)
   * - DVP_BFRC_QUARTER: Capture 1 in 4 frames (25%)
   *=========================================================================*/
  dvp_basic_frame_rate_control_set(DVP_BFRC_HALF);

  dvp_enable(TRUE);
}

/*-----------------------------------------------------------------------------
 * Enhanced Frame Rate Control (N out of M frames)
 *---------------------------------------------------------------------------*/
void dvp_enhanced_framerate_config(uint8_t capture_frames, uint8_t total_frames)
{
  dvp_io_init();

  /* MUST use continuous mode and basic rate = ALL */
  dvp_capture_mode_set(DVP_CAP_FUNC_MODE_CONTINUOUS);
  dvp_basic_frame_rate_control_set(DVP_BFRC_ALL);

  /*=========================================================================
   * Enhanced Frame Rate: Capture N frames out of M
   * 
   * Parameters:
   * - efrcsf: Source frame count (total frames in cycle) - max 31
   * - efrctf: Target frame count (frames to capture) - max 31
   * 
   * Note: efrctf must be <= efrcsf
   * 
   * Examples:
   * - 15, 30: Capture 15 out of 30 frames (50%, same as BFRC_HALF)
   * - 1, 30: Capture 1 out of 30 frames (~3.3%)
   * - 10, 30: Capture 10 out of 30 frames (~33%)
   *=========================================================================*/
  dvp_enhanced_framerate_set(total_frames, capture_frames, TRUE);

  dvp_enable(TRUE);
}

/* Example: Capture 1 frame per second from 30fps camera */
void framerate_1fps_from_30fps(void)
{
  dvp_enhanced_framerate_config(1, 30);
}
```

---

### Example 7: Monochrome Binarization

```c
/*******************************************************************************
 * DVP Monochrome Binarization Example
 * 
 * Converts grayscale image to binary (black/white) using threshold.
 * Useful for document scanning, barcode reading, edge detection.
 ******************************************************************************/

void dvp_binarization_config(uint8_t threshold)
{
  dvp_io_init();

  dvp_capture_mode_set(DVP_CAP_FUNC_MODE_CONTINUOUS);
  dvp_sync_mode_set(DVP_SYNC_MODE_HARDWARE);
  dvp_pixel_data_length_set(DVP_PIXEL_DATA_LENGTH_8);

  /* Set Y8 (grayscale) data format */
  dvp_enhanced_data_format_set(DVP_EFDF_Y8);

  /*=========================================================================
   * Binarization Configuration
   * 
   * Threshold: 0-255
   * - Pixels with Y value >= threshold → WHITE (0xFF)
   * - Pixels with Y value < threshold → BLACK (0x00)
   * 
   * Typical values:
   * - 128: Mid-gray threshold (50%)
   * - 64: Dark threshold (captures more as white)
   * - 192: Light threshold (captures more as black)
   *=========================================================================*/
  dvp_monochrome_image_binarization_set(threshold, TRUE);

  dvp_enable(TRUE);
}

/* Example: Document scanning with mid-threshold */
void document_scan_config(void)
{
  dvp_binarization_config(128);
}

/* Example: Low-light barcode reading */
void barcode_config(void)
{
  dvp_binarization_config(80);
}
```

---

## Supported Camera Sensors

| Sensor | Resolution | Interface | Notes |
|--------|------------|-----------|-------|
| OV2640 | 2MP (1600x1200) | 8-bit, Hardware Sync | Common, low cost |
| OV5640 | 5MP (2592x1944) | 8-bit, Hardware/Embedded | Autofocus support |
| OV7670 | VGA (640x480) | 8-bit, Hardware Sync | Very low cost |
| OV7725 | VGA (640x480) | 8-bit, Hardware Sync | Low power |
| MT9V034 | WVGA (752x480) | 10-bit | Global shutter |

### Camera Connection Diagram

```
┌─────────────────┐                    ┌─────────────────────┐
│   Camera        │                    │   AT32F435/437      │
│   (OV5640)      │                    │                     │
├─────────────────┤                    ├─────────────────────┤
│ PCLK    ────────┼────────────────────┼──► DVP_PCLK (PA6)   │
│ HREF    ────────┼────────────────────┼──► DVP_HSYNC (PA4)  │
│ VSYNC   ────────┼────────────────────┼──► DVP_VSYNC (PB7)  │
│ D0      ────────┼────────────────────┼──► DVP_D0 (PA9)     │
│ D1      ────────┼────────────────────┼──► DVP_D1 (PA10)    │
│ D2      ────────┼────────────────────┼──► DVP_D2 (PA11)    │
│ D3      ────────┼────────────────────┼──► DVP_D3 (PA12)    │
│ D4      ────────┼────────────────────┼──► DVP_D4 (PE4)     │
│ D5      ────────┼────────────────────┼──► DVP_D5 (PB4)     │
│ D6      ────────┼────────────────────┼──► DVP_D6 (PE5)     │
│ D7      ────────┼────────────────────┼──► DVP_D7 (PB9)     │
│ SIOC    ────────┼────────────────────┼──► I2C_SCL (PH2)    │
│ SIOD    ────────┼────────────────────┼──► I2C_SDA (PH3)    │
│ PWDN    ────────┼────────────────────┼──► GPIO (PC1)       │
│ RESET   ────────┼────────────────────┼──► GPIO (PC3)       │
└─────────────────┘                    └─────────────────────┘
```

---

## Implementation Checklist

### Basic DVP Setup
- [ ] Enable DVP peripheral clock (`CRM_DVP_PERIPH_CLOCK`)
- [ ] Configure GPIO pins for DVP function (GPIO_MUX_13)
- [ ] Set synchronization mode (hardware/embedded)
- [ ] Configure polarities to match camera
- [ ] Set pixel data length
- [ ] Enable DVP interrupts as needed
- [ ] Enable DVP peripheral

### DMA Configuration
- [ ] Enable EDMA clock
- [ ] Configure EDMA stream for peripheral-to-memory
- [ ] Set up EDMAMUX with `EDMAMUX_DMAREQ_ID_DVP`
- [ ] Configure buffer size based on frame size
- [ ] Enable DMA interrupts if needed

### Advanced Features
- [ ] Configure window cropping if needed
- [ ] Set up pixel/line decimation for zoom out
- [ ] Configure frame rate control
- [ ] Set up enhanced scaling if resizing needed
- [ ] Configure data format conversion
- [ ] Set up binarization if needed

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| No image captured | DVP not enabled | Call `dvp_enable(TRUE)` |
| Corrupted image | Wrong polarity settings | Check HSYNC/VSYNC/PCLK polarity |
| Partial image | DMA buffer too small | Increase buffer size |
| FIFO overrun | DMA too slow | Use burst mode, higher priority |
| Image shifted | Wrong sync polarity | Invert HSYNC or VSYNC |
| Color distortion | Wrong data format | Match camera output format |
| Sync errors (ESE) | Embedded sync code mismatch | Verify sync codes |
| Black image | Camera not initialized | Initialize camera via I2C/SCCB |
| Flickering | Frame rate mismatch | Match camera and display rates |

### Debug Tips

```c
/* Check DVP status */
void dvp_status_check(void)
{
  /* Check if HSYNC is active */
  if(dvp_basic_status_get(DVP_STATUS_HSYN) == SET)
  {
    /* HSYNC detected */
  }

  /* Check if VSYNC is active */
  if(dvp_basic_status_get(DVP_STATUS_VSYN) == SET)
  {
    /* VSYNC detected - frame boundary */
  }

  /* Check if FIFO has data */
  if(dvp_basic_status_get(DVP_STATUS_OFNE) == SET)
  {
    /* FIFO not empty - data available */
  }
}
```

---

## Performance Considerations

| Resolution | Format | Data Rate @ 30fps | EDMA Bandwidth |
|------------|--------|-------------------|----------------|
| QVGA (320x240) | RGB565 | ~4.4 MB/s | Low |
| VGA (640x480) | RGB565 | ~17.6 MB/s | Medium |
| SVGA (800x600) | RGB565 | ~27.5 MB/s | High |
| XGA (1024x768) | RGB565 | ~45 MB/s | Very High |

> **Note:** Higher resolutions require careful DMA configuration and may need double buffering to avoid frame tearing.

---

## See Also

- [DMA Documentation](./DMA_Direct_Memory_Access.md) - DMA configuration for DVP
- [GPIO Documentation](./GPIO_General_Purpose_IO.md) - Pin configuration
- [XMC Documentation](./XMC_External_Memory_Controller.md) - LCD interface
- [I2C Documentation](./I2C_Inter_Integrated_Circuit.md) - Camera control
- Application Note AN0087 - DVP Camera Interface Guide
- AT32F435_437 Reference Manual - DVP Chapter

