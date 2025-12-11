---
title: EMAC - Ethernet MAC Controller
description: Comprehensive documentation for the AT32F437 EMAC peripheral
peripheral: EMAC
category: Communication
target: AT32F437 (F437 Only)
---

# EMAC - Ethernet MAC Controller

## Overview

The AT32F437 features an integrated **Ethernet MAC (EMAC)** controller compliant with IEEE 802.3-2002 standards, supporting 10/100 Mbps operation. The EMAC provides a complete Ethernet interface with dedicated DMA for efficient data transfer, hardware checksum offloading, and IEEE 1588 Precision Time Protocol (PTP) support.

> **Note:** The EMAC peripheral is **only available on AT32F437** devices. The AT32F435 does not include Ethernet capability.

## Key Features

| Feature | Description |
|---------|-------------|
| **Standards** | IEEE 802.3-2002 (10BASE-T, 100BASE-TX) |
| **Speed** | 10 Mbps and 100 Mbps |
| **Duplex** | Half-duplex and Full-duplex |
| **Interface** | MII (Media Independent Interface), RMII (Reduced MII) |
| **DMA** | Dedicated DMA with ring buffer descriptors |
| **Checksum** | Hardware IPv4/TCP/UDP/ICMP checksum offload |
| **MAC Filtering** | Perfect/Hash unicast, multicast, broadcast filtering |
| **Flow Control** | IEEE 802.3x pause frame support |
| **VLAN** | IEEE 802.1Q VLAN tagging |
| **PTP** | IEEE 1588v2 Precision Time Protocol |
| **Power Management** | Magic Packet, Wake-on-LAN, Remote Wakeup |
| **Statistics** | MMC (MAC Management Counters) |
| **Max Frame** | 1518 bytes standard, 9000 bytes jumbo frames |

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           AT32F437 EMAC                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                 │
│  │   AHB Bus   │◄──►│  DMA Engine │◄──►│ TX/RX FIFO  │                 │
│  │  Interface  │    │             │    │  (4KB each) │                 │
│  └─────────────┘    └─────────────┘    └──────┬──────┘                 │
│                                               │                         │
│  ┌─────────────────────────────────────────────┴─────────────────────┐ │
│  │                         MAC Core                                   │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ │ │
│  │  │ TX Logic │ │ RX Logic │ │ Flow Ctrl│ │  Filter  │ │ Checksum │ │ │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └──────────┘ │ │
│  └─────────────────────────────────────────────────────────────────────┘ │
│                                               │                         │
│  ┌─────────────────────────────────────────────┴─────────────────────┐ │
│  │                      Station Management                           │ │
│  │            ┌──────────────────────────────────────┐              │ │
│  │            │      SMI/MDIO Interface              │              │ │
│  │            │    (PHY Register Access)             │              │ │
│  │            └──────────────────────────────────────┘              │ │
│  └─────────────────────────────────────────────────────────────────────┘ │
│                                               │                         │
│            MII/RMII Interface ────────────────┘                        │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                             ┌──────┴──────┐
                             │ External PHY │
                             │ (DM9162,     │
                             │  DP83848,    │
                             │  etc.)       │
                             └─────────────┘
```

## Pin Configuration

### RMII Mode (Recommended - Fewer Pins)

| Signal | Pin | Function |
|--------|-----|----------|
| RMII_REF_CLK | PA1 | 50 MHz Reference Clock |
| RMII_MDIO | PA2 | Management Data I/O |
| RMII_MDC | PC1 | Management Data Clock |
| RMII_CRS_DV | PA7 or PD8 | Carrier Sense/Data Valid |
| RMII_RXD0 | PC4 or PD9 | Receive Data 0 |
| RMII_RXD1 | PC5 or PD10 | Receive Data 1 |
| RMII_TX_EN | PB11 or PG11 | Transmit Enable |
| RMII_TXD0 | PB12 or PG13 | Transmit Data 0 |
| RMII_TXD1 | PB13 or PG14 | Transmit Data 1 |

### MII Mode (Full Interface)

| Signal | Pin | Function |
|--------|-----|----------|
| MII_TX_CLK | PC3 | Transmit Clock (25 MHz) |
| MII_RX_CLK | PA1 | Receive Clock (25 MHz) |
| MII_MDIO | PA2 | Management Data I/O |
| MII_MDC | PC1 | Management Data Clock |
| MII_CRS | PA0 or PH2 | Carrier Sense |
| MII_COL | PA3 or PH3 | Collision Detect |
| MII_RX_DV | PA7 or PD8 | Receive Data Valid |
| MII_RX_ER | PB10 | Receive Error |
| MII_RXD[0:3] | PC4, PC5, PB0, PB1 | Receive Data |
| MII_TX_EN | PB11 or PG11 | Transmit Enable |
| MII_TXD[0:3] | PB12, PB13, PC2, PB8 | Transmit Data |

## DMA Descriptor Structure

### Enhanced DMA Descriptor (Recommended)

```c
typedef struct {
    uint32_t status;           // OWN, status flags
    uint32_t controlsize;      // Control bits, buffer sizes
    uint32_t buf1addr;         // Buffer 1 address
    uint32_t buf2nextdescaddr; // Buffer 2 or next descriptor
    uint32_t extendedstatus;   // Extended status (PTP)
    uint32_t reserved1;
    uint32_t timestamp_l;      // PTP timestamp low
    uint32_t timestamp_h;      // PTP timestamp high
} emac_dma_desc_type;
```

### TX Descriptor Status Bits

| Bit | Name | Description |
|-----|------|-------------|
| 31 | OWN | Descriptor owned by DMA |
| 30 | IC | Interrupt on Completion |
| 29 | LS | Last Segment |
| 28 | FS | First Segment |
| 27 | DC | Disable CRC |
| 26 | DP | Disable Padding |
| 25 | TTSE | Transmit Timestamp Enable |
| 22:23 | CIC | Checksum Insertion Control |
| 21 | TER | Transmit End of Ring |
| 20 | TCH | Second Address Chained |

### RX Descriptor Status Bits

| Bit | Name | Description |
|-----|------|-------------|
| 31 | OWN | Descriptor owned by DMA |
| 30 | AFM | DA Filter Fail |
| 16:29 | FL | Frame Length |
| 15 | ES | Error Summary |
| 9 | FS | First Descriptor |
| 8 | LS | Last Descriptor |

## Supported PHY Chips

| PHY | Address | Notes |
|-----|---------|-------|
| DM9162 | 0x03 | Default on AT-START board |
| DP83848 | 0x01 | Alternative PHY |
| LAN8720 | 0x00/0x01 | Common RMII PHY |

## API Reference

### Initialization Functions

```c
// Reset EMAC peripheral
void emac_reset(void);

// Set SMI clock range based on HCLK
void emac_clock_range_set(void);

// Software reset DMA
void emac_dma_software_reset_set(void);
flag_status emac_dma_software_reset_get(void);

// Start/Stop EMAC
void emac_start(void);
void emac_stop(void);
```

### PHY Management Functions

```c
// Read PHY register via MDIO
error_status emac_phy_register_read(uint8_t address, uint8_t reg, uint16_t *data);

// Write PHY register via MDIO
error_status emac_phy_register_write(uint8_t address, uint8_t reg, uint16_t data);

// Check MII busy flag
flag_status emac_mii_busy_get(void);
```

### MAC Configuration Functions

```c
// Initialize control parameters to defaults
void emac_control_para_init(emac_control_config_type *control_para);

// Apply MAC configuration
void emac_control_config(emac_control_config_type *control_struct);

// Enable/disable receiver
void emac_receiver_enable(confirm_state new_state);

// Enable/disable transmitter
void emac_trasmitter_enable(confirm_state new_state);

// Set speed (10/100 Mbps)
void emac_fast_speed_set(emac_speed_type speed);

// Set duplex mode
void emac_duplex_mode_set(emac_duplex_type duplex_mode);

// Set interframe gap
void emac_interframe_gap_set(emac_intergrame_gap_type number);

// Enable checksum offload
void emac_ipv4_checksum_offload_set(confirm_state new_state);
```

### MAC Address and Filtering Functions

```c
// Set local MAC address
void emac_local_address_set(uint8_t *address);

// Configure address filtering
void emac_address_filter_set(emac_address_type mac, 
                             emac_address_filter_type filter, 
                             emac_address_mask_type mask_bit, 
                             confirm_state new_state);

// Set promiscuous mode
void emac_promiscuous_mode_set(confirm_state new_state);

// Enable hash filtering
void emac_hash_unicast_set(confirm_state new_state);
void emac_hash_multicast_set(confirm_state new_state);

// Set hash table
void emac_hash_table_high32bits_set(uint32_t high32bits);
void emac_hash_table_low32bits_set(uint32_t low32bits);

// Broadcast frame control
void emac_broadcast_frames_disable(confirm_state new_state);

// Receive all frames
void emac_receive_all_set(confirm_state new_state);
```

### DMA Configuration Functions

```c
// Initialize DMA parameters to defaults
void emac_dma_para_init(emac_dma_config_type *control_para);

// Apply DMA configuration
void emac_dma_config(emac_dma_config_type *control_para);

// Set descriptor list address
void emac_dma_descriptor_list_address_set(emac_dma_tx_rx_type transfer_type, 
                                          emac_dma_desc_type *dma_desc_tab, 
                                          uint8_t *buff, 
                                          uint32_t buffer_count);

// Set DMA thresholds
void emac_dma_receive_threshold_set(emac_dma_receive_threshold_type value);
void emac_dma_transmit_threshold_set(emac_dma_transmit_threshold_type value);

// DMA operations control
void emac_dma_operations_set(emac_dma_operations_type ops, confirm_state new_state);

// Poll demand for TX/RX
void emac_dma_poll_demand_set(emac_dma_tx_rx_type transfer_type, uint32_t value);
```

### DMA Interrupt Functions

```c
// Enable/disable DMA interrupts
void emac_dma_interrupt_enable(emac_dma_interrupt_type it, confirm_state new_state);

// Get DMA flag status
flag_status emac_dma_flag_get(uint32_t dma_flag);
flag_status emac_dma_interrupt_flag_get(uint32_t dma_flag);

// Clear DMA flag
void emac_dma_flag_clear(uint32_t dma_flag);
```

### Flow Control Functions

```c
// Enable flow control
void emac_transmit_flow_control_enable(confirm_state new_state);
void emac_receive_flow_control_enable(confirm_state new_state);

// Set pause time
void emac_pause_time_set(uint16_t pause_time);
void emac_pause_low_threshold_set(emac_pause_slot_threshold_type pasue_threshold);

// Back pressure activation
void emac_fcb_bpa_set(confirm_state new_state);
```

### Power Management Functions

```c
// Power down mode
void emac_power_down_set(confirm_state new_state);

// Magic packet detection
void emac_magic_packet_enable(confirm_state new_state);
flag_status emac_received_magic_packet_get(void);

// Wakeup frame detection
void emac_wakeup_frame_enable(confirm_state new_state);
void emac_wakeup_frame_set(uint32_t value);
flag_status emac_received_wakeup_frame_get(void);

// Global unicast
void emac_global_unicast_set(confirm_state new_state);
```

### VLAN Functions

```c
// Set VLAN tag identifier
void emac_vlan_tag_identifier_set(uint16_t identifier);

// Set VLAN comparison mode
void emac_vlan_tag_comparison_set(confirm_state new_state);
```

### MMC (Statistics) Functions

```c
// Reset/freeze counters
void emac_mmc_counter_reset(void);
void emac_mmc_counter_freeze(confirm_state new_state);
void emac_mmc_rollover_stop(confirm_state new_state);
void emac_mmc_reset_on_read_enable(confirm_state new_state);

// Get transmit statistics
uint32_t emac_mmc_transmit_good_frames_get(uint32_t flag);

// Get receive error statistics
uint32_t emac_mmc_received_error_frames_get(uint32_t flag);

// Get status flags
flag_status emac_mmc_received_status_get(uint32_t flag);
flag_status emac_mmc_transmit_status_get(uint32_t flag);
```

### PTP Functions

```c
// Enable timestamp
void emac_ptp_timestamp_enable(confirm_state new_state);

// Fine/coarse update mode
void emac_ptp_timestamp_fine_update_enable(confirm_state new_state);

// Initialize system time
void emac_ptp_timestamp_system_time_init(confirm_state new_state);
void emac_ptp_timestamp_system_time_update(confirm_state new_state);

// Get/set system time
uint32_t emac_ptp_system_second_get(void);
uint32_t emac_ptp_system_subsecond_get(void);
void emac_ptp_system_time_set(uint32_t sign, uint32_t second, uint32_t subsecond);

// Set addend for fine correction
void emac_ptp_timestamp_addend_set(uint32_t value);
void emac_ptp_addend_register_update(confirm_state new_state);

// PPS output
void emac_ptp_pps_frequency_set(emac_ptp_pps_control_type freq);
```

## Configuration Structures

### MAC Control Configuration

```c
typedef struct {
    emac_auto_negotiation_type auto_nego;       // Auto negotiation enable
    confirm_state              deferral_check;  // Deferral check enable
    emac_bol_type              back_off_limit;  // Back-off limit setting
    confirm_state              auto_pad_crc_strip; // Auto pad/CRC stripping
    confirm_state              retry_disable;   // Retry disable
    confirm_state              ipv4_checksum_offload; // IPv4 checksum offload
    emac_duplex_type           duplex_mode;     // Duplex mode
    confirm_state              loopback_mode;   // Loopback mode
    confirm_state              receive_own_disable;  // Receive own disable
    emac_speed_type            fast_ethernet_speed;  // Speed (10/100)
    confirm_state              carrier_sense_disable; // Carrier sense disable
    emac_intergrame_gap_type   interframe_gap;  // Interframe gap
    confirm_state              jabber_disable;  // Jabber disable
    confirm_state              watchdog_disable; // Watchdog disable
} emac_control_config_type;
```

### DMA Configuration

```c
typedef struct {
    confirm_state                    aab_enable;       // Address-aligned beats
    confirm_state                    usp_enable;       // Separate PBL
    emac_dma_pbl_type                rx_dma_pal;       // RX DMA PBL
    confirm_state                    fb_enable;        // Fixed burst
    emac_dma_pbl_type                tx_dma_pal;       // TX DMA PBL
    uint8_t                          desc_skip_length; // Descriptor skip length
    confirm_state                    da_enable;        // DMA arbitration
    emac_dma_rx_tx_ratio_type        priority_ratio;   // RX/TX priority ratio
    confirm_state                    dt_disable;       // Disable dropping TCP/IP errors
    confirm_state                    rsf_enable;       // RX store and forward
    confirm_state                    flush_rx_disable; // Disable RX flush
    confirm_state                    tsf_enable;       // TX store and forward
    emac_dma_transmit_threshold_type tx_threshold;     // TX threshold
    confirm_state                    fef_enable;       // Forward error frames
    confirm_state                    fugf_enable;      // Forward undersized good frames
    emac_dma_receive_threshold_type  rx_threshold;     // RX threshold
    confirm_state                    osf_enable;       // Operate on second frame
} emac_dma_config_type;
```

## Code Examples

### Basic EMAC Initialization with lwIP

```c
#include "at32f435_437.h"
#include "at32_emac.h"
#include "netconf.h"

// MAC address
uint8_t mac_address[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

// DMA descriptors and buffers
#define ETH_RXBUFNB 4
#define ETH_TXBUFNB 4
#define ETH_RX_BUF_SIZE 1524
#define ETH_TX_BUF_SIZE 1524

emac_dma_desc_type dma_rx_dscr_tab[ETH_RXBUFNB] __attribute__((aligned(4)));
emac_dma_desc_type dma_tx_dscr_tab[ETH_TXBUFNB] __attribute__((aligned(4)));
uint8_t rx_buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__((aligned(4)));
uint8_t tx_buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __attribute__((aligned(4)));

error_status emac_system_init(void)
{
    emac_control_config_type mac_control_para;
    emac_dma_config_type dma_control_para;
    
    // Enable clocks
    crm_periph_clock_enable(CRM_EMAC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_EMACTX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_EMACRX_PERIPH_CLOCK, TRUE);
    
    // Configure EMAC pins
    emac_pins_configuration();
    
    // Configure NVIC
    emac_nvic_configuration();
    
    // Select RMII mode
    scfg_emac_interface_set(SCFG_EMAC_SELECT_RMII);
    
    // Reset EMAC DMA
    emac_dma_software_reset_set();
    while(emac_dma_software_reset_get() == SET);
    
    // Set SMI clock range
    emac_clock_range_set();
    
    // Initialize PHY
    if(emac_phy_init(&mac_control_para) == ERROR)
    {
        return ERROR;
    }
    
    // Configure MAC
    emac_control_para_init(&mac_control_para);
    mac_control_para.auto_nego = EMAC_AUTO_NEGOTIATION_ON;
    mac_control_para.ipv4_checksum_offload = TRUE;
    emac_control_config(&mac_control_para);
    
    // Configure DMA
    emac_dma_para_init(&dma_control_para);
    dma_control_para.rsf_enable = TRUE;
    dma_control_para.tsf_enable = TRUE;
    dma_control_para.osf_enable = TRUE;
    emac_dma_config(&dma_control_para);
    
    // Set MAC address
    emac_local_address_set(mac_address);
    
    // Initialize DMA descriptors
    emac_dma_descriptor_list_address_set(EMAC_DMA_TRANSMIT, dma_tx_dscr_tab, 
                                         &tx_buff[0][0], ETH_TXBUFNB);
    emac_dma_descriptor_list_address_set(EMAC_DMA_RECEIVE, dma_rx_dscr_tab,
                                         &rx_buff[0][0], ETH_RXBUFNB);
    
    // Enable DMA interrupts
    emac_dma_interrupt_enable(EMAC_DMA_INTERRUPT_NORMAL_SUMMARY, TRUE);
    emac_dma_interrupt_enable(EMAC_DMA_INTERRUPT_RX, TRUE);
    
    // Start EMAC
    emac_start();
    
    return SUCCESS;
}

void emac_pins_configuration(void)
{
    gpio_init_type gpio_init_struct;
    
    // Enable GPIO clocks
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    
    // RMII_REF_CLK (PA1)
    gpio_init_struct.gpio_pins = GPIO_PINS_1;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE1, GPIO_MUX_11);
    
    // RMII_MDIO (PA2)
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_11);
    
    // RMII_MDC (PC1)
    gpio_init_struct.gpio_pins = GPIO_PINS_1;
    gpio_init(GPIOC, &gpio_init_struct);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE1, GPIO_MUX_11);
    
    // RMII_CRS_DV (PD8 with remap)
    gpio_init_struct.gpio_pins = GPIO_PINS_8;
    gpio_init(GPIOD, &gpio_init_struct);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE8, GPIO_MUX_11);
    
    // RMII_RXD0 (PD9 with remap)
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init(GPIOD, &gpio_init_struct);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE9, GPIO_MUX_11);
    
    // RMII_RXD1 (PD10 with remap)
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init(GPIOD, &gpio_init_struct);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE10, GPIO_MUX_11);
    
    // RMII_TX_EN (PB11)
    gpio_init_struct.gpio_pins = GPIO_PINS_11;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE11, GPIO_MUX_11);
    
    // RMII_TXD0 (PB12)
    gpio_init_struct.gpio_pins = GPIO_PINS_12;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE12, GPIO_MUX_11);
    
    // RMII_TXD1 (PB13)
    gpio_init_struct.gpio_pins = GPIO_PINS_13;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE13, GPIO_MUX_11);
}

void emac_nvic_configuration(void)
{
    nvic_irq_enable(EMAC_IRQn, 1, 0);
}
```

### TCP Client Example

```c
#include "at32f435_437.h"
#include "at32_emac.h"
#include "netconf.h"
#include "tcp_client.h"

volatile uint32_t local_time = 0;

int main(void)
{
    error_status status;
    
    system_clock_config();
    at32_board_init();
    uart_print_init(115200);
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    delay_init();
    
    // Initialize EMAC
    status = emac_system_init();
    while(status == ERROR);
    
    // Initialize TCP/IP stack (lwIP)
    tcpip_stack_init();
    
    // Initialize TCP client
    tcp_client_init(TCP_LOCAL_PORT, TCP_SERVER_PORT, TCP_SERVER_IP);
    
    while(1)
    {
        // lwIP receive handling
        lwip_rx_loop_handler();
        
        // Periodic timeout handling
        lwip_periodic_handle(local_time);
    }
}

// SysTick handler for lwIP timing
void SysTick_Handler(void)
{
    local_time += 10;
}

// EMAC interrupt handler
void EMAC_IRQHandler(void)
{
    if(emac_dma_flag_get(EMAC_DMA_RI_FLAG))
    {
        // Frame received - set flag for lwIP
        ethernetif_input_handler();
        emac_dma_flag_clear(EMAC_DMA_RI_FLAG);
    }
    
    emac_dma_flag_clear(EMAC_DMA_NIS_FLAG);
}
```

### HTTP Server Example

```c
#include "at32f435_437.h"
#include "at32_emac.h"
#include "netconf.h"
#include "httpd.h"

int main(void)
{
    system_clock_config();
    at32_board_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    delay_init();
    
    // Initialize EMAC
    if(emac_system_init() == ERROR)
    {
        while(1); // Failed to initialize
    }
    
    // Initialize TCP/IP stack
    tcpip_stack_init();
    
    // Start HTTP server
    httpd_init();
    
    printf("HTTP Server started!\r\n");
    printf("Connect to: http://%d.%d.%d.%d\r\n",
           IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    
    while(1)
    {
        lwip_rx_loop_handler();
        lwip_periodic_handle(local_time);
    }
}
```

### Wake-on-LAN Example

```c
#include "at32f435_437.h"
#include "at32_emac.h"

void configure_wake_on_lan(void)
{
    // Enable EMAC clocks (must stay enabled in sleep)
    crm_periph_clock_enable(CRM_EMAC_PERIPH_CLOCK, TRUE);
    
    // Enable Magic Packet detection
    emac_magic_packet_enable(TRUE);
    
    // Enable global unicast filter for WoL
    emac_global_unicast_set(TRUE);
    
    // Enable power down mode
    emac_power_down_set(TRUE);
    
    // Configure PMT interrupt
    emac_interrupt_mask_set(EMAC_INTERRUPT_PMT_MASK, FALSE);
}

void enter_low_power_mode(void)
{
    configure_wake_on_lan();
    
    // Enter stop mode - EMAC will wake MCU on Magic Packet
    pwc_deep_sleep_mode_enter(PWC_DEEP_SLEEP_ENTER_WFI);
    
    // MCU wakes here after Magic Packet received
    if(emac_received_magic_packet_get() == SET)
    {
        printf("Woken by Magic Packet!\r\n");
        
        // Disable power down
        emac_power_down_set(FALSE);
        emac_magic_packet_enable(FALSE);
    }
}
```

### PHY Link Status Detection

```c
#include "at32f435_437.h"
#include "at32_emac.h"

#define PHY_ADDRESS       0x03
#define PHY_STATUS_REG    0x01
#define PHY_LINKED_STATUS 0x0004

uint16_t check_link_status(void)
{
    uint16_t phy_status;
    
    if(emac_phy_register_read(PHY_ADDRESS, PHY_STATUS_REG, &phy_status) == SUCCESS)
    {
        if(phy_status & PHY_LINKED_STATUS)
        {
            return 1; // Link up
        }
    }
    return 0; // Link down
}

void link_status_polling(void)
{
    static uint16_t last_link_status = 0;
    uint16_t current_status;
    
    current_status = check_link_status();
    
    if(current_status != last_link_status)
    {
        last_link_status = current_status;
        
        if(current_status)
        {
            printf("Ethernet link UP\r\n");
            // Reconfigure speed/duplex based on negotiation
            ethernetif_update_config(netif);
        }
        else
        {
            printf("Ethernet link DOWN\r\n");
        }
    }
}
```

### MQTT Client Example

```c
#include "at32f435_437.h"
#include "at32_emac.h"
#include "mqtt_client.h"

#define MQTT_BROKER_IP     "192.168.1.100"
#define MQTT_BROKER_PORT   1883
#define MQTT_CLIENT_ID     "at32f437_client"
#define MQTT_TOPIC         "sensors/temperature"

mqtt_client_t mqtt_client;

void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    if(status == MQTT_CONNECT_ACCEPTED)
    {
        printf("MQTT Connected!\r\n");
        
        // Subscribe to topic
        mqtt_subscribe(client, "commands/#", 0, mqtt_incoming_cb, NULL);
    }
}

void mqtt_publish_data(float temperature)
{
    char payload[32];
    
    sprintf(payload, "{\"temp\":%.1f}", temperature);
    
    mqtt_publish(&mqtt_client, MQTT_TOPIC, payload, strlen(payload),
                 0, 0, mqtt_publish_cb, NULL);
}

int main(void)
{
    system_clock_config();
    at32_board_init();
    
    // Initialize EMAC
    emac_system_init();
    tcpip_stack_init();
    
    // Initialize MQTT client
    mqtt_client_init(&mqtt_client);
    
    // Connect to broker
    ip_addr_t broker_ip;
    IP4_ADDR(&broker_ip, 192, 168, 1, 100);
    mqtt_client_connect(&mqtt_client, &broker_ip, MQTT_BROKER_PORT,
                        mqtt_connection_cb, NULL, NULL);
    
    while(1)
    {
        lwip_rx_loop_handler();
        lwip_periodic_handle(local_time);
        
        // Publish data periodically
        if(publish_timer_expired())
        {
            mqtt_publish_data(read_temperature());
        }
    }
}
```

## DMA Interrupt Flags

| Flag | Description |
|------|-------------|
| `EMAC_DMA_TI_FLAG` | Transmit Interrupt |
| `EMAC_DMA_TPS_FLAG` | Transmit Process Stopped |
| `EMAC_DMA_TBU_FLAG` | Transmit Buffer Unavailable |
| `EMAC_DMA_TJT_FLAG` | Transmit Jabber Timeout |
| `EMAC_DMA_OVF_FLAG` | Receive Overflow |
| `EMAC_DMA_UNF_FLAG` | Transmit Underflow |
| `EMAC_DMA_RI_FLAG` | Receive Interrupt |
| `EMAC_DMA_RBU_FLAG` | Receive Buffer Unavailable |
| `EMAC_DMA_RPS_FLAG` | Receive Process Stopped |
| `EMAC_DMA_RWT_FLAG` | Receive Watchdog Timeout |
| `EMAC_DMA_ETI_FLAG` | Early Transmit Interrupt |
| `EMAC_DMA_FBEI_FLAG` | Fatal Bus Error Interrupt |
| `EMAC_DMA_ERI_FLAG` | Early Receive Interrupt |
| `EMAC_DMA_AIS_FLAG` | Abnormal Interrupt Summary |
| `EMAC_DMA_NIS_FLAG` | Normal Interrupt Summary |

## Configuration Checklist

1. **Clock Configuration**
   - [ ] Enable EMAC peripheral clock
   - [ ] Enable EMAC TX clock
   - [ ] Enable EMAC RX clock
   - [ ] Provide 50 MHz reference clock for RMII (or 25 MHz for MII)

2. **Pin Configuration**
   - [ ] Configure GPIO pins for selected interface (MII/RMII)
   - [ ] Set correct GPIO alternate function (GPIO_MUX_11)
   - [ ] Configure GPIO drive strength for high-speed signals

3. **PHY Configuration**
   - [ ] Select correct PHY address
   - [ ] Configure MII/RMII mode via SCFG
   - [ ] Reset PHY and wait for initialization
   - [ ] Enable auto-negotiation or set manual speed/duplex

4. **DMA Configuration**
   - [ ] Allocate aligned descriptor arrays
   - [ ] Allocate aligned buffer arrays
   - [ ] Initialize descriptor chains
   - [ ] Configure DMA thresholds

5. **Interrupt Configuration**
   - [ ] Enable EMAC interrupt in NVIC
   - [ ] Enable required DMA interrupts
   - [ ] Implement EMAC_IRQHandler

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| PHY not responding | Wrong PHY address | Check PHY datasheet for correct address |
| No link detected | PHY not initialized | Ensure PHY reset sequence is correct |
| TX not working | DMA not started | Call `emac_start()` after configuration |
| RX not working | Descriptor chain broken | Verify descriptor initialization |
| Slow performance | Threshold too high | Reduce RX/TX thresholds |
| Checksum errors | Offload not enabled | Enable `ipv4_checksum_offload` |
| Frame drops | Buffer too small | Increase buffer count or size |

## lwIP Integration Files

```
middlewares/lwip/
├── src/
│   ├── api/          # Sequential API
│   ├── core/         # Core stack
│   ├── netif/        # Network interface drivers
│   └── include/      # Headers
├── port/
│   ├── ethernetif.c  # EMAC driver for lwIP
│   ├── ethernetif.h
│   └── lwipopts.h    # lwIP configuration
```

## Related Documentation

- **AN0052** - TCP Client Application Note
- **AN0053** - HTTP Server Application Note
- **AN0054** - TCP Server Application Note
- **AN0055** - Wake-on-LAN Application Note
- **LwIP Documentation** - Lightweight TCP/IP Stack

## See Also

- [DMA - Direct Memory Access](DMA_Direct_Memory_Access.md)
- [GPIO - General Purpose I/O](GPIO_General_Purpose_IO.md)
- [CRM - Clock Reset Management](CRM_Clock_Reset_Management.md)
- [PWC - Power Controller](PWC_Power_Controller.md)

