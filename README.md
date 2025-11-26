---
title: "AT32F435/437 MCU Documentation Repository"
type: "mcu-documentation"
purpose: "context7-knowledge-source"
mcu_family: "AT32F435/437"
architecture: "ARM Cortex-M4F"
vendor: "Artery"
documentation_version: "2.07"
last_updated: "2024-11-26"
tags:
  - mcu
  - microcontroller
  - embedded-systems
  - arm-cortex-m4
  - errata
  - device-limitations
  - context7
  - artery-at32
peripherals:
  - CAN
  - Flash
  - PWC
  - ADC
  - I2C
  - I2S
  - TMR
  - USART
  - USB
  - GPIO
  - EMAC
  - QSPI
  - DVP
  - EDMA
  - SDIO
  - XMC
status: "production-ready"
---

# AT32F435/437 MCU Documentation Repository
## Reference Source for Context7

![Status](https://img.shields.io/badge/Documentation-Complete-brightgreen)
![Type](https://img.shields.io/badge/Type-MCU%20Documentation-blue)
![Firmware Library](https://img.shields.io/badge/Firmware-v2.2.2-green)
![Reference Manual](https://img.shields.io/badge/RM-v2.07-blue)

**Comprehensive MCU documentation repository** for the **Artery AT32F435/437** ARM Cortex-M4F microcontroller series. This repository serves as a primary documentation source for Context7, providing structured technical references and implementation guidelines.

**Includes:**
- **Complete firmware library v2.2.2** with peripheral drivers and CMSIS support
- **Extensive middleware support** (FreeRTOS, LWIP, LVGL, FatFS, USB stacks)
- **Working examples** for all peripherals across multiple development boards
- **Official PDF documentation** (Reference Manual, Datasheet, Errata Sheet, Application Notes)
- **Structured markdown documentation** for Context7 integration (peripheral guides with examples)
- **Migration guide** from AT32F403A/407 to AT32F435/437

**Note:** This repository contains the complete Artery firmware library and official documentation. For the latest updates, visit [Artery Technology's official website](https://www.arterytek.com/).

---

## 📖 Context7 Integration

This repository is designed as a **documentation source for Context7**, providing:

- **Structured Technical References:** Complete MCU specifications and peripheral documentation
- **Working Code Examples:** Comprehensive peripheral examples and middleware integrations
- **Advanced Features:** QSPI, DVP (camera), EMAC (Ethernet), EDMA support
- **Production Middleware:** FreeRTOS, LWIP TCP/IP stack, LVGL GUI, FatFS filesystem
- **Quick Access:** Search-friendly format for rapid information retrieval during development

**Use this repository as a reference when:**
- Developing firmware for AT32F435/437 microcontrollers
- Implementing Context7 MCU support and peripherals
- Migrating from AT32F403A/407 series
- Integrating middleware (RTOS, networking, GUI, filesystem)
- Training new developers on AT32 capabilities

---

## 🚀 Quick Start

**Jump directly to:**
- [📝 Markdown Documentation](#-markdown-documentation) - Structured peripheral guides
- [📦 Firmware Library (v2.2.2)](#-main-documentation)
- [📚 Official Documentation (PDFs)](#-official-pdf-documentation)
- [🔧 Middleware Stack](#-middleware-support)
- [🔄 Migration Guide](#-migration-from-at32f403a407)
- [🔗 Official Artery Website](https://www.arterytek.com/)

---

## 📚 Main Documentation

### **[Markdown Documentation](docs/)** 📝

**Structured peripheral documentation** optimized for Context7 integration:

| Peripheral | Documentation | Description |
|------------|---------------|-------------|
| **ACC** | [ACC Auto Clock Calibration](docs/ACC_Auto_Clock_Calibration.md) | USB clock calibration, HICK trimming, calibration modes |
| **ADC** | [ADC Analog to Digital Converter](docs/ADC_Analog_to_Digital_Converter.md) | Multi-channel ADC, DMA, trigger sources, oversampling |
| **CAN** | [CAN Controller Area Network](docs/CAN_Controller_Area_Network.md) | CAN bus communication, filtering, bit timing, loopback |
| **Cortex-M4** | [Cortex-M4 Core Features](docs/Cortex_M4_Core_Features.md) | SysTick, FPU, Bit-Banding, CMSIS-DSP |
| **CRC** | [CRC Cyclic Redundancy Check](docs/CRC_Cyclic_Redundancy_Check.md) | Hardware CRC calculation, polynomial configs, standards |
| **CRM** | [CRM Clock Reset Management](docs/CRM_Clock_Reset_Management.md) | Clock tree, PLL configuration, clock switching, CFD |

**Each documentation includes:**
- ✅ Architectural overview with block diagrams
- ✅ Complete API reference
- ✅ Working code examples
- ✅ Configuration tables and formulas
- ✅ Implementation checklists
- ✅ Troubleshooting guides

**Perfect for:**
- Context7 AI integration and knowledge base
- Quick reference during development
- Learning peripheral capabilities
- Copy-paste code examples

### **[Firmware Library v2.2.2](libraries/)** 🔥

**Complete firmware library** with drivers and examples:
- ✅ **CMSIS support** - ARM Cortex-M4F core definitions with DSP library
- ✅ **Peripheral drivers** - Complete API headers and implementations for all peripherals
- ✅ **Examples for all peripherals** - ADC, CAN, Flash, I2C, SPI, TMR, USART, USB, QSPI, DVP, EMAC, etc.
- ✅ **Multiple board support** - AT-START-F435, AT-START-F437, AT-SURF-F437
- ✅ **Development utilities** - IAP demos, QSPI flash algorithms, sLib demo

### **[Middleware Stack](middlewares/)** 🛠️

**Production-ready middleware integrations:**
- ✅ **FreeRTOS** - Real-time operating system with demo
- ✅ **LWIP 2.1.2** - Lightweight TCP/IP stack for EMAC
- ✅ **LVGL** - Light and Versatile Graphics Library for GUI
- ✅ **FatFS** - FAT filesystem for SD cards and USB mass storage
- ✅ **USB Stack** - Device and Host drivers with multiple device classes
  - USB Device: Audio, CDC, HID, MSC, Printer, Composite devices
  - USB Host: CDC, HID, MSC classes

### **[Example Projects](project/)** 📝

**Working examples organized by development board:**
- **AT-START-F435:** Examples for AT32F435 series (lower flash/RAM variants)
- **AT-START-F437:** Examples for AT32F437 series (higher performance)
- **AT-SURF-F437:** Advanced applications including camera (DVP) and display demos

**Example categories:**
- Basic peripherals (GPIO, TMR, USART, SPI, I2C)
- Advanced peripherals (QSPI, DVP, EMAC, SDIO, XMC)
- Communication (CAN, USB Device/Host, Ethernet)
- Storage (Flash, SD card with FatFS)
- Multimedia (DVP camera interface, display controllers)
- System (DMA, EDMA, power management, watchdog)

### **[Utilities](utilities/)** 🧰

**Development utilities and demos:**
- **FreeRTOS Demo** - Complete RTOS example with tasks and synchronization
- **Random Number Generator Demo** - Hardware RNG implementation
- **QSPI Flash Algorithm** - Flash programming algorithms for debuggers
- **sLib Demo** - Secure library implementation example
- **IAP Demos** - In-Application Programming via USART, USB, and Ethernet

---

## 📄 Official PDF Documentation

### Core Documentation

| Document | Version | Description |
|----------|---------|-------------|
| [Reference Manual](RM_AT32F435_437_V2.07_EN.pdf) | v2.07 | Complete peripheral and register documentation (19MB) |
| [Datasheet](DS_AT32F435_437_V2.20_EN.pdf) | v2.20 | Electrical specifications and pin definitions (3.6MB) |
| [Errata Sheet](ES0003_AT32F435_437_Errata_Sheet_V2.0.15_EN.pdf) | v2.0.15 | Known silicon issues and workarounds (1.1MB) |

### Application Notes

| Document | Version | Description |
|----------|---------|-------------|
| [Get Started Guide](AN0128_AT32F435_437_Get_started_guide_V2.0.5_EN.pdf) | v2.0.5 | Development setup and first project (2.5MB) |
| [Security Library](AN0081_AT32F435_437_Security_Library_Application_Note_V2.0.1_EN.pdf) | v2.0.1 | Code protection and sLib implementation (2.2MB) |
| [Migration Guide](MG0018_Migrating_from_AT32F403A_407_to_AT32F435_437_EN_V2.0.3.pdf) | v2.0.3 | Upgrading from F403A/407 series (879KB) |

### Library Documentation

Located in [document/](document/) directory:
- Firmware Library Release Notes (v2.2.2)
- Firmware Library Drivers Release Notes (v2.0.6)
- BSP & Pack User Guides (English and Chinese)

---

## 🎨 Key Features

### Advanced Peripherals

**AT32F435/437 includes advanced features not in F403A/407:**
- **QSPI (Quad-SPI):** High-speed serial flash interface with XIP support
- **DVP (Digital Video Port):** Camera interface for image capture
- **EDMA (Enhanced DMA):** Advanced DMA with 2D transfer support
- **EMAC (Ethernet MAC):** 10/100 Mbps Ethernet with LWIP stack
- **Higher Performance:** Up to 288MHz (vs 240MHz in F403A/407)
- **More Memory:** Up to 4032KB Flash, 512KB SRAM

### Middleware Ecosystem

**Production-ready software components:**
- **FreeRTOS:** Task scheduling, semaphores, queues, timers
- **LWIP:** TCP/IP stack with sockets API, DHCP, DNS, HTTP server
- **LVGL:** Modern GUI with widgets, animations, touch support
- **FatFS:** Full-featured FAT32 filesystem with long filename support
- **USB:** Comprehensive device and host class implementations

### Development Boards

**Three reference designs included:**
1. **AT-START-F435:** Entry-level development board
2. **AT-START-F437:** High-performance development board
3. **AT-SURF-F437:** Advanced board with camera, display, Ethernet

---

## 🔄 Migration from AT32F403A/407

### Why Upgrade?

**Performance improvements:**
- 🚀 **Faster:** 288MHz vs 240MHz (+20%)
- 💾 **More memory:** Up to 4032KB Flash vs 1024KB
- 📡 **Advanced peripherals:** QSPI, DVP, EDMA
- 🌐 **Networking:** EMAC Ethernet with LWIP
- 📸 **Multimedia:** DVP camera interface

### Migration Considerations

**Compatible:**
- ✅ Pin layout (mostly compatible for equivalent packages)
- ✅ ARM Cortex-M4F core
- ✅ Basic peripheral API (minimal changes)
- ✅ Development tools (same toolchain)

**Differences:**
- ⚠️ Some register names updated
- ⚠️ New peripheral drivers (QSPI, DVP, EDMA, EMAC)
- ⚠️ Clock configuration differences
- ⚠️ Different errata (see ES0003)

**Migration Guide:** See [MG0018 Migration Guide PDF](MG0018_Migrating_from_AT32F403A_407_to_AT32F435_437_EN_V2.0.3.pdf)

---

## 🛠️ Technical Details

### Repository Information

- **Firmware Library Version:** v2.2.2
- **Firmware Drivers Version:** v2.0.6
- **Reference Manual Version:** v2.07
- **Last Updated:** November 2024
- **Supported Devices:** AT32F435/437 series

### Supported Devices

**AT32F435 Series:**
- AT32F435CCT7, AT32F435CCU7
- AT32F435CMT7, AT32F435CMU7
- AT32F435RCT7
- AT32F435RMT7

**AT32F437 Series:**
- AT32F437RCT7, AT32F437RMT7
- AT32F437VCT7, AT32F437VMT7
- AT32F437ZCT7, AT32F437ZMT7

**Memory Variants:**
- **C models:** 256KB Flash, 192KB SRAM
- **M models:** 4032KB Flash, 512KB SRAM

**Core Features:**
- ARM Cortex-M4F with FPU and DSP
- Up to 288MHz operation
- Hardware floating-point unit
- DSP instruction set

---

## ⚠️ Device Limitations

The AT32F435/437 series has documented silicon errata that developers should be aware of. Please refer to the [ES0003 Errata Sheet](ES0003_AT32F435_437_Errata_Sheet_V2.0.15_EN.pdf) for:

- Complete list of known issues
- Affected peripherals and conditions
- Workarounds and code examples
- Hardware revision information

**Critical areas to review:**
- Flash memory operations
- Power management (Deepsleep modes)
- CAN bus communication
- Peripheral-specific limitations

**📥 Download Latest Errata Sheet:**  
Visit [Artery Technology](https://www.arterytek.com/) to download the latest version of document **ES0003** (AT32F435/437 Errata Sheet v2.0.15 or later).

---

## 📖 Usage Guide

### For New Developers

1. **Start with the Get Started Guide** ([AN0128](AN0128_AT32F435_437_Get_started_guide_V2.0.5_EN.pdf))
2. **Review the Reference Manual** ([RM v2.07](RM_AT32F435_437_V2.07_EN.pdf)) for peripheral details
3. **Check the Errata Sheet** ([ES0003](ES0003_AT32F435_437_Errata_Sheet_V2.0.15_EN.pdf)) for known issues
4. **Explore examples** in [project/](project/) directories
5. **Review middleware** if using RTOS, networking, or GUI

### For Experienced Developers

- **Browse examples** by peripheral in [project/at_start_f437/examples/](project/at_start_f437/examples/)
- **Copy driver code** from [libraries/drivers/](libraries/drivers/)
- **Integrate middleware** from [middlewares/](middlewares/)
- **Use utilities** for IAP, QSPI programming, etc.

### Example Workflow: QSPI Flash

1. Review QSPI peripheral in Reference Manual (Chapter on QSPI)
2. Check QSPI examples in `project/at_start_f437/examples/qspi/`
3. Use QSPI flash algorithm from `utilities/at32f435_437_qspi_algorithm_demo/`
4. Implement application code with XIP (execute-in-place) if needed

---

## 🔍 Search Tips

### By Peripheral
Search by name: `QSPI`, `DVP`, `EMAC`, `CAN`, `Flash`, `PWC`, `ADC`, `TMR`, `USART`, etc.

**Markdown Documentation:**
- Quick access: `docs/ACC_Auto_Clock_Calibration.md`, `docs/ADC_Analog_to_Digital_Converter.md`, etc.
- Search within docs: Use your editor's search to find specific APIs or configurations

### By Middleware
Search for specific components:
- "FreeRTOS task"
- "LWIP TCP"
- "LVGL widget"
- "FatFS SD card"
- "USB device CDC"

### By Example Type
Search for specific implementations:
- "DMA transfer"
- "interrupt handler"
- "PWM generation"
- "USB device"
- "Ethernet LWIP"
- "Camera DVP"

---

## 🔗 Related Resources

### Official Artery Resources
- **[Artery Official Website](https://www.arterytek.com/)** - Download latest documents
- **[AT32 MCU Series](https://www.arterytek.com/en/product/index.jsp)** - Product lineup
- **AT32F435/437 Product Page** - Specifications and tools

### Community Resources
- **GitHub:** Search "AT32F435" or "AT32F437" for code examples
- **Forums:** EEVBlog, embedded.com discussions
- **STM32 Code:** Often compatible due to ARM Cortex-M4F core

### Related Projects
- **AT32F403A/407 Documentation:** Previous generation MCU docs
- **Context7 Repository:** Primary project repository
- **TafcoMcuCore:** MCU core implementation and drivers

---

## 🤝 Contributing

This repository contains **official Artery Technology documentation and firmware**.

### How to Contribute

**Found an issue?**
- Open an issue with specific reference to file or document

**Have improvements?**
- Submit pull request with additional examples
- Add clarifying documentation
- Improve README or FAQ

**Want to add content?**
- Add practical application examples
- Create integration guides
- Translate documentation to other languages

### Guidelines

- Maintain technical accuracy
- Reference official documentation
- Include working code examples
- Test on real hardware
- Follow existing formatting style

---

## 📜 License & Attribution

### Firmware Library & Documentation

- **Copyright © Artery Technology Co., Ltd.**
- **Firmware Version:** v2.2.2
- **Documentation:** Reference Manual v2.07, Datasheet v2.20
- **Content:** Peripheral drivers, examples, middleware, CMSIS support

### Middleware Licenses

- **FreeRTOS:** MIT License
- **LWIP:** BSD License
- **LVGL:** MIT License
- **FatFS:** Custom open-source license
- **USB Stack:** Artery Technology

### This Repository

- **Purpose:** MCU firmware library and documentation for Context7 and embedded development
- **Content:** Official Artery firmware library v2.2.2 with complete middleware stack
- **Status:** Production-ready for Context7 integration
- **Target Audience:** Embedded systems engineers, MCU firmware developers, Context7 AI developers

**Important:** For the latest documentation and firmware updates, always check [Artery Technology's official website](https://www.arterytek.com/).

---

## 📈 Repository Statistics

![Firmware](https://img.shields.io/badge/Firmware-v2.2.2-green)
![Reference Manual](https://img.shields.io/badge/RM-v2.07-blue)
![Peripherals](https://img.shields.io/badge/Peripherals-20+-blue)
![Middleware](https://img.shields.io/badge/Middleware-5%20Stacks-yellow)
![Drivers](https://img.shields.io/badge/Drivers-Complete-brightgreen)

---

## 🎯 Repository Goals

### Included ✅

- ✅ **Complete firmware library v2.2.2** with all peripheral drivers
- ✅ **Structured markdown documentation** (ACC, ADC, CAN, Cortex-M4, CRC, CRM)
- ✅ **Official PDF documentation** (RM, DS, ES, ANs)
- ✅ **Production middleware** (FreeRTOS, LWIP, LVGL, FatFS, USB)
- ✅ **Working examples** for all peripherals and middleware
- ✅ **Development utilities** (IAP, QSPI algorithms, sLib demo)
- ✅ **Multiple board support** (AT-START-F435, F437, AT-SURF-F437)
- ✅ **CMSIS support** with ARM Cortex-M4F DSP library
- ✅ **Migration guide** from AT32F403A/407

### Future Enhancements

- 📌 Additional peripheral markdown documentation (DAC, DMA, GPIO, I2C, SPI, TMR, USART, USB, etc.)
- 📌 Context7 integration guide for AI developers
- 📌 Comprehensive FAQ for common development questions
- 📌 Advanced peripheral integration guides (QSPI XIP, DVP camera, EMAC networking)
- 📌 Performance optimization tips and techniques
- 📌 Low-power mode implementation guide
- 📌 Community-contributed examples
- 📌 Video tutorials (community-driven)

---

## 💬 Support

- **Issues:** Use GitHub Issues for questions or problems
- **Discussions:** Use GitHub Discussions for general topics
- **Official Support:** Contact Artery Technology directly
- **Community:** EEVBlog, embedded.com forums

---

## 🔗 Context7 Resources

**In This Repository:**
- **[Markdown Documentation](docs/):** Structured peripheral guides (ACC, ADC, CAN, Cortex-M4, CRC, CRM)
- **[Firmware Library](libraries/):** Complete drivers and CMSIS support (v2.2.2)
- **[Middleware](middlewares/):** FreeRTOS, LWIP, LVGL, FatFS, USB stacks
- **[Examples](project/):** Working code for all peripherals
- **[Utilities](utilities/):** Development tools and demos

**External Resources:**
- **[Official Documentation](https://www.arterytek.com/):** Download latest PDFs and tools
- **[Migration Guide](MG0018_Migrating_from_AT32F403A_407_to_AT32F435_437_EN_V2.0.3.pdf):** Upgrade from F403A/407

**Related Projects:**
- **Context7 Repository:** Primary project repository
- **TafcoMcuCore:** MCU core implementation and drivers
- **AT32F403A/407 Docs:** Previous generation documentation

---

**Last Updated:** November 2024  
**Firmware Version:** v2.2.2  
**Reference Manual:** v2.07  
**Repository Status:** ✅ Production Ready for Context7

**🎯 MCU Firmware Library:** Complete reference for AT32F435/437 development  
**📚 Context7 Source:** Primary documentation with firmware library v2.2.2  
**📝 Markdown Docs:** Structured peripheral guides optimized for Context7  
**🚀 Advanced Features:** QSPI, DVP, EMAC, EDMA support with middleware  
**⚡ High Performance:** 288MHz, 4032KB Flash, 512KB SRAM  

**⭐ Help Others:** Star this repo if it helped your project!  
**🔄 Share:** Help other developers working on AT32 projects!  
**🤝 Contribute:** Improve documentation for the embedded community!

