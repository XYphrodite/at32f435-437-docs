# Frequently Asked Questions (FAQ)
## AT32F435/437 MCU Documentation

**For Context7:** Optimized for semantic search and quick retrieval

---

## 🔥 Top 10 Most Common Questions

### 1. What are the main differences between AT32F435 and AT32F437?

**Tags:** #device-comparison #memory #variants  
**Quick Answer:** Same peripherals, different memory sizes

**Key Differences:**
- **AT32F435:** 256KB to 4032KB Flash options, 192KB to 512KB SRAM
- **AT32F437:** Same as AT32F435, naming convention mostly interchangeable
- **Peripherals:** Identical peripheral set on both
- **Performance:** Both run at up to 288MHz

**Memory Variants:**
- **C models (CCT7, CMT7, RCT7, VCT7, ZCT7):** 256KB Flash, 192KB SRAM
- **M models (CMT7, CMU7, RMT7, VMT7, ZMT7):** 4032KB Flash, 512KB SRAM

**Reference:** See [Datasheet](DS_AT32F435_437_V2.20_EN.pdf) for complete specifications

---

### 2. How do I migrate from AT32F403A/407 to AT32F435/437?

**Tags:** #migration #upgrade #compatibility  
**Quick Answer:** Mostly straightforward, with some API updates and new features

**What's Compatible:**
- ✅ Core peripherals (GPIO, TMR, USART, SPI, I2C, CAN, ADC, DAC)
- ✅ ARM Cortex-M4F core (same instruction set)
- ✅ Basic API structure
- ✅ Development tools and IDEs

**What Changed:**
- ⚠️ Clock frequency up to 288MHz (was 240MHz)
- ⚠️ Some register names updated
- ⚠️ New peripherals: QSPI, DVP, EDMA, EMAC
- ⚠️ Memory addressing for larger Flash/RAM
- ⚠️ Different errata (ES0003 vs ES0002)

**Migration Steps:**
1. Read the [Migration Guide PDF](MG0018_Migrating_from_AT32F403A_407_to_AT32F435_437_EN_V2.0.3.pdf)
2. Update clock configuration for 288MHz
3. Update peripheral driver includes (at32f435_437_*.h)
4. Review new features (QSPI, DVP, EDMA, EMAC)
5. Check ES0003 errata for F435/437-specific issues
6. Test thoroughly

---

### 3. What advanced peripherals are new in F435/437?

**Tags:** #peripherals #features #advanced  
**Quick Answer:** QSPI, DVP, EDMA, EMAC - not in F403A/407

**New Peripherals:**

**QSPI (Quad-SPI):**
- High-speed serial flash interface
- Execute-in-place (XIP) support
- Up to 144MHz clock speed
- 1-1-1, 1-1-4, 1-4-4 modes

**DVP (Digital Video Port):**
- Camera interface for CMOS/CCD sensors
- 8-bit/10-bit/12-bit/14-bit data width
- Continuous/snapshot capture modes
- Hardware JPEG support

**EDMA (Enhanced DMA):**
- Advanced DMA controller
- 2D transfer support
- Enhanced addressing modes
- Better performance than standard DMA

**EMAC (Ethernet MAC):**
- 10/100 Mbps Ethernet
- IEEE 1588 PTP support
- LWIP stack integration included
- DMA descriptor chaining

**Reference:** See [Reference Manual](RM_AT32F435_437_V2.07_EN.pdf) for detailed documentation

---

### 4. How do I use QSPI with external flash?

**Tags:** #QSPI #flash #XIP #external-memory  
**Quick Answer:** Use QSPI peripheral with XIP for code execution

**Basic Setup:**
1. Configure QSPI peripheral
2. Initialize external QSPI flash
3. Enable XIP (execute-in-place) mode
4. Map flash to memory space

**Example Location:** `project/at_start_f437/examples/qspi/`

**QSPI Flash Algorithm:** Available in `utilities/at32f435_437_qspi_algorithm_demo/`

**Use Cases:**
- Expand code space beyond internal flash
- Store fonts, graphics, data tables
- Execute code from external flash (XIP)
- Fast data logging

**Supported Flash Types:**
- Winbond W25Q series
- Micron N25Q series
- Macronix MX25L series
- Other QSPI-compatible flash

**Reference:** [Reference Manual QSPI chapter](RM_AT32F435_437_V2.07_EN.pdf)

---

### 5. Can I use FreeRTOS with this MCU?

**Tags:** #FreeRTOS #RTOS #multitasking  
**Quick Answer:** YES, complete FreeRTOS port included

**What's Included:**
- ✅ FreeRTOS source code in `middlewares/freertos/`
- ✅ Complete demo project in `utilities/at32f435_437_freertos_demo/`
- ✅ ARM Cortex-M4F port with FPU support
- ✅ Preemptive scheduling with priority levels
- ✅ Tasks, queues, semaphores, mutexes, timers

**Getting Started:**
1. Review FreeRTOS demo: `utilities/at32f435_437_freertos_demo/`
2. Copy FreeRTOS source to your project
3. Configure FreeRTOSConfig.h for your needs
4. Create tasks and start scheduler

**Example Tasks:**
- LED blinking at different rates
- UART communication handler
- ADC sampling task
- CAN message processing

**Reference:** `middlewares/freertos/` and demo project

---

### 6. How do I implement Ethernet (EMAC)?

**Tags:** #EMAC #Ethernet #networking #LWIP  
**Quick Answer:** Use EMAC peripheral with LWIP TCP/IP stack

**What's Included:**
- ✅ LWIP 2.1.2 stack in `middlewares/lwip_2.1.2/`
- ✅ EMAC HAL driver in `libraries/drivers/`
- ✅ Example projects in `project/at_surf_f437/examples/emac/`
- ✅ IAP over Ethernet demo in `utilities/at32f437_emac_iap_demo/`

**Features:**
- 10/100 Mbps Ethernet MAC
- RMII interface (preferred) or MII
- TCP/IP stack with sockets API
- DHCP, DNS, HTTP server support
- RAW API and Netconn API

**Getting Started:**
1. Review LWIP configuration in examples
2. Configure PHY (typically LAN8720 or similar)
3. Initialize EMAC and LWIP stack
4. Implement application (TCP server, HTTP, etc.)

**Use Cases:**
- Web server for configuration
- MQTT for IoT applications
- Modbus TCP for industrial control
- Firmware update over network

**Reference:** EMAC examples in `project/at_surf_f437/examples/emac/`

---

### 7. What USB capabilities are supported?

**Tags:** #USB #device #host #CDC #MSC #HID  
**Quick Answer:** Full USB Device and Host support with multiple classes

**USB Device Classes:**
- ✅ **CDC (Communication Device Class):** Virtual COM port
- ✅ **MSC (Mass Storage Class):** USB flash drive emulation
- ✅ **HID (Human Interface Device):** Keyboard, mouse, custom HID
- ✅ **Audio:** USB audio device
- ✅ **Printer:** USB printer class
- ✅ **Composite:** Multiple classes (CDC+MSC, CDC+HID, etc.)
- ✅ **WinUSB:** Custom Windows driver-less communication

**USB Host Classes:**
- ✅ **CDC:** USB to serial adapter host
- ✅ **MSC:** USB flash drive host
- ✅ **HID:** Keyboard/mouse host

**Location:**
- Device drivers: `middlewares/usb_drivers/`
- Device classes: `middlewares/usbd_class/`
- Host classes: `middlewares/usbh_class/`
- Examples: `project/at_start_f437/examples/usb_device/` and `usb_host/`

**Popular Use Cases:**
- Virtual COM port for debugging
- USB flash drive for data logging
- USB keyboard/mouse HID interface
- Firmware update over USB (IAP)

---

### 8. How do I implement camera interface (DVP)?

**Tags:** #DVP #camera #image #video  
**Quick Answer:** Use DVP peripheral with compatible CMOS sensor

**DVP Features:**
- 8/10/12/14-bit parallel data interface
- Continuous or snapshot mode
- Hardware JPEG support
- DMA for efficient data transfer
- Synchronization signals (HSYNC, VSYNC)

**Typical Cameras:**
- OV7670 (VGA resolution)
- OV2640 (2MP with JPEG)
- OV5640 (5MP with JPEG)
- Other CMOS sensors with DVP interface

**Example Location:** `project/at_surf_f437/examples/dvp/` and applications

**Implementation Steps:**
1. Connect camera to DVP pins
2. Initialize I2C for camera configuration
3. Configure DVP peripheral
4. Setup DMA for frame capture
5. Process captured images

**Use Cases:**
- Image capture and processing
- QR code scanning
- Machine vision
- Video streaming with JPEG compression

**Reference:** [Reference Manual DVP chapter](RM_AT32F435_437_V2.07_EN.pdf)

---

### 9. What development boards are available?

**Tags:** #boards #hardware #development-kit  
**Quick Answer:** Three official Artery development boards

**Available Boards:**

**1. AT-START-F435:**
- Entry-level development board
- AT32F435CCT7 or AT32F435CMT7 MCU
- Basic peripherals (UART, USB, CAN)
- Affordable for learning

**2. AT-START-F437:**
- High-performance development board
- AT32F437RMT7 or AT32F437VMT7 MCU
- More GPIO and peripherals
- QSPI flash expansion

**3. AT-SURF-F437:**
- Advanced evaluation board
- AT32F437ZMT7 MCU (144-pin, max memory)
- DVP camera interface
- TFT LCD display
- Ethernet (EMAC) connector
- SD card slot
- Full peripheral breakout

**Example Code:**
- AT-START-F435: `project/at_start_f435/`
- AT-START-F437: `project/at_start_f437/`
- AT-SURF-F437: `project/at_surf_f437/`

**Purchasing:** Available from [Artery Technology](https://www.arterytek.com/) and distributors

---

### 10. What tools do I need for development?

**Tags:** #development #tools #IDE #compiler  
**Quick Answer:** ARM GCC + any IDE, or Keil/IAR

**Compilers:**
- ARM GCC (free, recommended)
- Keil MDK-ARM
- IAR EWARM

**IDEs:**
- VS Code + PlatformIO
- Eclipse + GNU ARM Plugin
- Keil μVision
- IAR Embedded Workbench
- Artery IDE (AT32 IDE)

**Debug Tools:**
- JTAG/SWD debugger (AT-Link, ST-Link, J-Link)
- OpenOCD (free, open source)
- GDB server
- Serial Wire Viewer (SWV) for printf debugging

**SDK & Drivers:**
- AT32 Firmware Library (in this repo)
- CMSIS support included
- Middleware (FreeRTOS, LWIP, LVGL, FatFS)

**Getting Started:**
1. Download this repository
2. Install toolchain (ARM GCC or Keil/IAR)
3. Get debugger hardware (AT-Link recommended)
4. Follow [Get Started Guide](AN0128_AT32F435_437_Get_started_guide_V2.0.5_EN.pdf)
5. Build and flash example project

---

## 📑 By Topic

### Peripherals

#### Advanced
- [What advanced peripherals are new?](#3-what-advanced-peripherals-are-new-in-f435437)
- [How to use QSPI?](#4-how-do-i-use-qspi-with-external-flash)
- [How to implement Ethernet (EMAC)?](#6-how-do-i-implement-ethernet-emac)
- [How to implement camera (DVP)?](#8-how-do-i-implement-camera-interface-dvp)

#### Communication
- [USB capabilities](#7-what-usb-capabilities-are-supported)
- CAN bus (see examples in `project/*/examples/can/`)
- I2C, SPI, USART (see examples in `project/*/examples/`)

### Middleware
- [FreeRTOS](#5-can-i-use-freertos-with-this-mcu)
- [LWIP TCP/IP](#6-how-do-i-implement-ethernet-emac)
- LVGL GUI (see `middlewares/lvgl/`)
- FatFS filesystem (see `middlewares/3rd_party/fatfs/`)

### General
- [F435 vs F437 differences](#1-what-are-the-main-differences-between-at32f435-and-at32f437)
- [Migration from F403A/407](#2-how-do-i-migrate-from-at32f403a407-to-at32f435437)
- [Development boards](#9-what-development-boards-are-available)
- [Development tools](#10-what-tools-do-i-need-for-development)

### Device Issues
- Errata: See [ES0003 Errata Sheet](ES0003_AT32F435_437_Errata_Sheet_V2.0.15_EN.pdf)
- Known limitations and workarounds documented in ES0003
- Flash, CAN, PWC considerations

---

## 💡 Quick Tips

### Memory Optimization
- Use QSPI XIP for code execution from external flash
- Store constant data (fonts, graphics) in Flash, not SRAM
- Enable compiler optimizations (-O2 or -Os)
- Use DMA for bulk data transfers

### Power Management
- Review PWC examples in `project/*/examples/pwc/`
- Use FreeRTOS tickless idle for RTOS power saving
- Configure unused pins to reduce leakage
- Use low-power modes (Sleep, Deepsleep) appropriately

### Debugging
- Use SWV (Serial Wire Viewer) for printf via SWD
- Enable HardFault handler for crash debugging
- Use USART for logging in production
- FreeRTOS has awareness plugins for RTOS debugging

### Performance
- Run at 288MHz for maximum performance
- Use EDMA for 2D graphics operations
- Enable instruction cache and data cache
- Use QSPI XIP for faster external flash access

---

## 🔗 Additional Resources

### In This Repository
- **[README](README.md):** Complete overview and documentation index
- **[Reference Manual](RM_AT32F435_437_V2.07_EN.pdf):** Detailed peripheral documentation
- **[Errata Sheet](ES0003_AT32F435_437_Errata_Sheet_V2.0.15_EN.pdf):** Known issues and workarounds
- **[Get Started Guide](AN0128_AT32F435_437_Get_started_guide_V2.0.5_EN.pdf):** Setup and first project
- **[Migration Guide](MG0018_Migrating_from_AT32F403A_407_to_AT32F435_437_EN_V2.0.3.pdf):** Upgrade from F403A/407

### External Resources
- **[Artery Official Website](https://www.arterytek.com/):** Latest downloads and support
- **[AT32 Product Page](https://www.arterytek.com/en/product/index.jsp):** Product information
- **Community Forums:** EEVBlog, embedded.com

### Related Projects
- **AT32F403A/407 Documentation:** Previous generation
- **Context7 Repository:** Main project
- **TafcoMcuCore:** MCU core implementation

---

## 🤝 Contributing to FAQ

**Have a question not answered here?**
- Open an issue with your question
- We'll add it to the FAQ if commonly asked

**Want to improve an answer?**
- Submit a pull request with improvements
- Add code examples or clarifications
- Fix errors or outdated information

---

**Status:** 10 common questions answered  
**For Device Errata:** Download ES0003 from [Artery Technology](https://www.arterytek.com/)  
**For Code Examples:** See `project/` directories  
**For Middleware:** See `middlewares/` directory  
**Last Updated:** November 2024

