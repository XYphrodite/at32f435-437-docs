---
title: ERTC - Enhanced Real-Time Clock
category: Timer/Counter
complexity: Advanced
mcu: AT32F435/437
peripheral: ERTC
keywords: [ertc, rtc, calendar, alarm, wakeup, timestamp, tamper, backup]
---

# ERTC - Enhanced Real-Time Clock

## Overview

The Enhanced Real-Time Clock (ERTC) is a sophisticated timekeeping peripheral that provides calendar functionality, programmable alarms, wakeup timer, timestamp recording, and tamper detection with backup register protection. It operates from a low-frequency clock source (LEXT 32.768 kHz or LICK) and maintains time even during low-power modes when powered by a backup battery.

### Key Features

| Feature | Specification |
|---------|---------------|
| Calendar | Year, Month, Day, Week, Hour, Min, Sec |
| Hour Format | 12-hour (AM/PM) or 24-hour |
| Alarms | 2 programmable alarms (A and B) |
| Wakeup Timer | 16-bit auto-reload counter |
| Timestamp | Records time on external event |
| Tamper Detection | 2 inputs with BPR erase |
| Backup Registers | 20 × 32-bit (80 bytes) |
| Sub-second | 15-bit sub-second counter |
| Clock Sources | LEXT (32.768 kHz), LICK (~40 kHz) |
| Calibration | Smooth and coarse calibration |
| Low Power | Maintains time in standby/VBAT modes |

## Architecture

```
                         ┌─────────────────────────────────────────────────┐
                         │                    ERTC                         │
                         │                                                 │
  LEXT (32.768 kHz) ─────┤──► ┌──────────┐    ┌───────────────────────┐   │
  LICK (~40 kHz) ────────┤──► │ Prescaler│───►│    Calendar Engine    │   │
                         │    │ DIV_A    │    │  ┌─────┬────┬───────┐ │   │
                         │    │ DIV_B    │    │  │Date │Time│Subsec │ │   │
                         │    └──────────┘    │  └─────┴────┴───────┘ │   │
                         │         │          └───────────────────────┘   │
                         │         │ 1Hz               │                  │
                         │         ▼                   ▼                  │
                         │    ┌──────────┐    ┌───────────────────────┐   │
                         │    │ Wakeup   │    │      Alarms A/B       │   │
                         │    │ Timer    │    │  ┌───────┬────────┐   │   │
                         │    └────┬─────┘    │  │Alarm A│Alarm B │   │   │
                         │         │          │  └───────┴────────┘   │   │
                         │         ▼          └───────────┬───────────┘   │
                         │    ERTC_WKUP_IRQ              │                │
                         │                               ▼                │
    PC13/PA0 ────────────┤──► ┌──────────┐    ┌───────────────────────┐   │
   (Timestamp/Tamper)    │    │Timestamp │    │   Interrupt Control   │   │
                         │    │ Capture  │    │  ┌─────────────────┐  │   │
                         │    └────┬─────┘    │  │ALA ALB WAT TS TP│  │   │
                         │         │          │  └─────────────────┘  │   │
    PC13/PA0 ────────────┤──► ┌────┴─────┐    └───────────┬───────────┘   │
   (Tamper 1/2)          │    │ Tamper   │               │                │
                         │    │Detection │    ┌──────────▼───────────┐   │
                         │    └────┬─────┘    │   Backup Registers   │───┼──► PC13 (Output)
                         │         │          │   DT1 - DT20 (80B)   │   │    Alarm/Wakeup/Cal
                         │         │          └──────────────────────┘   │
                         │         └───────────────────► BPR Erase       │
                         └─────────────────────────────────────────────────┘
```

## Clock Source Configuration

### Prescaler Calculation

The ERTC requires a 1 Hz clock for the calendar. The prescaler formula:

```
ERTC_CLK_1Hz = ERTC_CLK / ((DIV_A + 1) × (DIV_B + 1))
```

| Clock Source | DIV_A | DIV_B | Resulting Frequency |
|--------------|-------|-------|---------------------|
| LEXT 32.768 kHz | 127 | 255 | 1 Hz |
| LICK ~40 kHz | 127 | See calibration | ~1 Hz |

### Clock Source Selection

| Source | Macro | Characteristics |
|--------|-------|-----------------|
| LEXT | `CRM_ERTC_CLOCK_LEXT` | Most accurate, requires external crystal |
| LICK | `CRM_ERTC_CLOCK_LICK` | Internal, less accurate (~40 kHz ±5%) |
| HEXT/128 | `CRM_ERTC_CLOCK_HEXT_DIV_128` | External high-speed divided |

## Register Map

### Base Address: 0x40002800

| Offset | Register | Description |
|--------|----------|-------------|
| 0x00 | TIME | Time register (hour, minute, second) |
| 0x04 | DATE | Date register (year, month, day, week) |
| 0x08 | CTRL | Control register |
| 0x0C | STS | Status register |
| 0x10 | DIV | Prescaler divider |
| 0x14 | WAT | Wakeup timer register |
| 0x18 | CCAL | Coarse calibration register |
| 0x1C | ALA | Alarm A register |
| 0x20 | ALB | Alarm B register |
| 0x24 | WP | Write protection register |
| 0x28 | SBS | Sub-second register |
| 0x2C | TADJ | Time adjustment register |
| 0x30 | TSTM | Timestamp time register |
| 0x34 | TSDT | Timestamp date register |
| 0x38 | TSSBS | Timestamp sub-second register |
| 0x3C | SCAL | Smooth calibration register |
| 0x40 | TAMP | Tamper configuration register |
| 0x44 | ALASBS | Alarm A sub-second register |
| 0x48 | ALBSBS | Alarm B sub-second register |
| 0x50-0x9C | DT1-DT20 | Backup data registers |

## Data Types

### Time Structure

```c
typedef struct
{
  uint8_t year;                          /* Year (0-99) */
  uint8_t month;                         /* Month (1-12) */
  uint8_t day;                           /* Day (1-31) */
  uint8_t hour;                          /* Hour (0-23 or 0-12) */
  uint8_t min;                           /* Minute (0-59) */
  uint8_t sec;                           /* Second (0-59) */
  uint8_t week;                          /* Week day (1-7) */
  ertc_am_pm_type ampm;                  /* AM/PM indicator */
} ertc_time_type;
```

### Alarm Structure

```c
typedef struct
{
  uint8_t day;                           /* Date (1-31) */
  uint8_t hour;                          /* Hour */
  uint8_t min;                           /* Minute */
  uint8_t sec;                           /* Second */
  ertc_am_pm_type ampm;                  /* AM/PM indicator */
  uint32_t mask;                         /* Alarm mask */
  uint8_t week_date_sel;                 /* Week or date mode */
  uint8_t week;                          /* Week day (1-7) */
} ertc_alarm_value_type;
```

## Configuration Options

### Hour Mode

| Mode | Macro | Description |
|------|-------|-------------|
| 24-hour | `ERTC_HOUR_MODE_24` | Standard 24-hour format |
| 12-hour | `ERTC_HOUR_MODE_12` | AM/PM format |

### AM/PM Selection

| Value | Macro | Description |
|-------|-------|-------------|
| AM | `ERTC_AM` | Ante meridiem (12-hour mode) |
| PM | `ERTC_PM` | Post meridiem (12-hour mode) |
| 24H | `ERTC_24H` | 24-hour format |

### Alarm Mask Options

| Mask | Macro | Description |
|------|-------|-------------|
| None | `ERTC_ALARM_MASK_NONE` | Match all fields |
| Second | `ERTC_ALARM_MASK_SEC` | Don't match seconds |
| Minute | `ERTC_ALARM_MASK_MIN` | Don't match minute |
| Hour | `ERTC_ALARM_MASK_HOUR` | Don't match hour |
| Date/Week | `ERTC_ALARM_MASK_DATE_WEEK` | Don't match date/week |
| All | `ERTC_ALARM_MASK_ALL` | Trigger every second |

### Wakeup Timer Clock Source

| Source | Macro | Frequency |
|--------|-------|-----------|
| DIV/16 | `ERTC_WAT_CLK_ERTCCLK_DIV16` | ERTC_CLK / 16 |
| DIV/8 | `ERTC_WAT_CLK_ERTCCLK_DIV8` | ERTC_CLK / 8 |
| DIV/4 | `ERTC_WAT_CLK_ERTCCLK_DIV4` | ERTC_CLK / 4 |
| DIV/2 | `ERTC_WAT_CLK_ERTCCLK_DIV2` | ERTC_CLK / 2 |
| CK_B 16-bit | `ERTC_WAT_CLK_CK_B_16BITS` | 1 Hz, counter = WAT |
| CK_B 17-bit | `ERTC_WAT_CLK_CK_B_17BITS` | 1 Hz, counter = WAT + 65535 |

### Timestamp Edge

| Edge | Macro | Description |
|------|-------|-------------|
| Rising | `ERTC_TIMESTAMP_EDGE_RISING` | Rising edge trigger |
| Falling | `ERTC_TIMESTAMP_EDGE_FALLING` | Falling edge trigger |

### Tamper Configuration

| Parameter | Options | Description |
|-----------|---------|-------------|
| Edge | `ERTC_TAMPER_EDGE_RISING/FALLING` | Trigger edge |
| Filter | `ERTC_TAMPER_FILTER_DISABLE/2/4/8` | Sample filter |
| Precharge | `ERTC_TAMPER_PR_1/2/4/8_ERTCCLK` | Pin precharge time |
| Frequency | `ERTC_TAMPER_FREQ_DIV_256` to `_DIV_32768` | Detection frequency |

## Interrupts and Flags

### Interrupts

| Interrupt | Macro | EXINT Line | IRQ Handler |
|-----------|-------|------------|-------------|
| Alarm A | `ERTC_ALA_INT` | EXINT17 | `ERTCAlarm_IRQHandler` |
| Alarm B | `ERTC_ALB_INT` | EXINT17 | `ERTCAlarm_IRQHandler` |
| Wakeup | `ERTC_WAT_INT` | EXINT22 | `ERTC_WKUP_IRQHandler` |
| Timestamp | `ERTC_TS_INT` | EXINT21 | `TAMP_STAMP_IRQHandler` |
| Tamper | `ERTC_TP_INT` | EXINT21 | `TAMP_STAMP_IRQHandler` |

### Status Flags

| Flag | Macro | Description |
|------|-------|-------------|
| ALAWF | `ERTC_ALAWF_FLAG` | Alarm A write allowed |
| ALBWF | `ERTC_ALBWF_FLAG` | Alarm B write allowed |
| WATWF | `ERTC_WATWF_FLAG` | Wakeup timer write allowed |
| TADJF | `ERTC_TADJF_FLAG` | Time adjustment flag |
| INITF | `ERTC_INITF_FLAG` | Initialization flag |
| UPDF | `ERTC_UPDF_FLAG` | Calendar update flag |
| IMF | `ERTC_IMF_FLAG` | Init mode flag |
| ALAF | `ERTC_ALAF_FLAG` | Alarm A flag |
| ALBF | `ERTC_ALBF_FLAG` | Alarm B flag |
| WATF | `ERTC_WATF_FLAG` | Wakeup timer flag |
| TSF | `ERTC_TSF_FLAG` | Timestamp flag |
| TSOF | `ERTC_TSOF_FLAG` | Timestamp overflow flag |
| TP1F | `ERTC_TP1F_FLAG` | Tamper 1 flag |
| TP2F | `ERTC_TP2F_FLAG` | Tamper 2 flag |
| CALUPDF | `ERTC_CALUPDF_FLAG` | Calibration update flag |

## API Reference

### Initialization Functions

| Function | Description |
|----------|-------------|
| `ertc_reset()` | Reset ERTC to default state |
| `ertc_divider_set(div_a, div_b)` | Set prescaler dividers |
| `ertc_hour_mode_set(mode)` | Set 12/24 hour mode |
| `ertc_wait_update()` | Wait for register synchronization |

### Write Protection

| Function | Description |
|----------|-------------|
| `ertc_write_protect_enable()` | Enable write protection |
| `ertc_write_protect_disable()` | Disable write protection |

### Calendar Functions

| Function | Description |
|----------|-------------|
| `ertc_date_set(year, month, date, week)` | Set date |
| `ertc_time_set(hour, min, sec, ampm)` | Set time |
| `ertc_calendar_get(time)` | Get current calendar |
| `ertc_sub_second_get()` | Get sub-second value |

### Alarm Functions

| Function | Description |
|----------|-------------|
| `ertc_alarm_set(alarm_x, week_date, hour, min, sec, ampm)` | Set alarm time |
| `ertc_alarm_mask_set(alarm_x, mask)` | Set alarm mask |
| `ertc_alarm_week_date_select(alarm_x, wk)` | Select week/date mode |
| `ertc_alarm_sub_second_set(alarm_x, value, mask)` | Set sub-second alarm |
| `ertc_alarm_enable(alarm_x, state)` | Enable/disable alarm |
| `ertc_alarm_get(alarm_x, alarm)` | Get alarm settings |
| `ertc_alarm_sub_second_get(alarm_x)` | Get alarm sub-second |

### Wakeup Timer Functions

| Function | Description |
|----------|-------------|
| `ertc_wakeup_clock_set(clock)` | Set wakeup clock source |
| `ertc_wakeup_counter_set(counter)` | Set wakeup counter |
| `ertc_wakeup_counter_get()` | Get wakeup counter |
| `ertc_wakeup_enable(state)` | Enable/disable wakeup |

### Timestamp Functions

| Function | Description |
|----------|-------------|
| `ertc_timestamp_pin_select(pin)` | Select timestamp pin |
| `ertc_timestamp_valid_edge_set(edge)` | Set trigger edge |
| `ertc_timestamp_enable(state)` | Enable/disable timestamp |
| `ertc_timestamp_get(time)` | Get timestamp value |
| `ertc_timestamp_sub_second_get()` | Get timestamp sub-second |

### Tamper Functions

| Function | Description |
|----------|-------------|
| `ertc_tamper_1_pin_select(pin)` | Select tamper 1 pin |
| `ertc_tamper_valid_edge_set(tamper_x, edge)` | Set trigger edge |
| `ertc_tamper_filter_set(filter)` | Set detection filter |
| `ertc_tamper_detect_freq_set(freq)` | Set detection frequency |
| `ertc_tamper_precharge_set(precharge)` | Set precharge time |
| `ertc_tamper_pull_up_enable(state)` | Enable pin pull-up |
| `ertc_tamper_timestamp_enable(state)` | Enable timestamp on tamper |
| `ertc_tamper_enable(tamper_x, state)` | Enable/disable tamper |

### Calibration Functions

| Function | Description |
|----------|-------------|
| `ertc_smooth_calibration_config(period, clk_add, clk_dec)` | Configure smooth cal |
| `ertc_coarse_calibration_set(dir, value)` | Set coarse calibration |
| `ertc_coarse_calibration_enable(state)` | Enable coarse calibration |
| `ertc_cal_output_select(output)` | Select calibration output |
| `ertc_cal_output_enable(state)` | Enable calibration output |
| `ertc_time_adjust(add1s, decsbs)` | Fine time adjustment |

### Backup Register Functions

| Function | Description |
|----------|-------------|
| `ertc_bpr_data_write(dt, data)` | Write to backup register |
| `ertc_bpr_data_read(dt)` | Read from backup register |

### Daylight Saving Functions

| Function | Description |
|----------|-------------|
| `ertc_daylight_set(operation, save)` | Set DST adjustment |
| `ertc_daylight_bpr_get()` | Get DST flag |

### Output and Mode Functions

| Function | Description |
|----------|-------------|
| `ertc_output_set(source, polarity, type)` | Configure output pin |
| `ertc_direct_read_enable(state)` | Enable direct read mode |
| `ertc_refer_clock_detect_enable(state)` | Enable reference clock |

### Interrupt and Flag Functions

| Function | Description |
|----------|-------------|
| `ertc_interrupt_enable(source, state)` | Enable/disable interrupt |
| `ertc_interrupt_get(source)` | Get interrupt status |
| `ertc_flag_get(flag)` | Get flag status |
| `ertc_interrupt_flag_get(flag)` | Get interrupt flag |
| `ertc_flag_clear(flag)` | Clear flag |

### Utility Functions

| Function | Description |
|----------|-------------|
| `ertc_num_to_bcd(num)` | Convert decimal to BCD |
| `ertc_bcd_to_num(bcd)` | Convert BCD to decimal |
| `ertc_init_mode_enter()` | Enter initialization mode |
| `ertc_init_mode_exit()` | Exit initialization mode |
| `ertc_wait_flag(flag, status)` | Wait for flag status |

## Code Examples

### Example 1: Basic Calendar Setup

```c
/**
 * @brief  Configure ERTC with LEXT clock source
 */
void ertc_calendar_init(void)
{
  /* Enable PWC clock */
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  
  /* Allow access to ERTC */
  pwc_battery_powered_domain_access(TRUE);
  
  /* Reset battery powered domain */
  crm_battery_powered_domain_reset(TRUE);
  crm_battery_powered_domain_reset(FALSE);
  
  /* Enable LEXT oscillator */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_LEXT, TRUE);
  while(crm_flag_get(CRM_LEXT_STABLE_FLAG) == RESET);
  
  /* Select LEXT as ERTC clock */
  crm_ertc_clock_select(CRM_ERTC_CLOCK_LEXT);
  crm_ertc_clock_enable(TRUE);
  
  /* Reset and wait for synchronization */
  ertc_reset();
  ertc_wait_update();
  
  /* Configure prescaler: 32768 / (127+1) / (255+1) = 1 Hz */
  ertc_divider_set(127, 255);
  
  /* Set 24-hour mode */
  ertc_hour_mode_set(ERTC_HOUR_MODE_24);
  
  /* Set initial date: 2024-01-01, Monday */
  ertc_date_set(24, 1, 1, 1);
  
  /* Set initial time: 12:00:00 */
  ertc_time_set(12, 0, 0, ERTC_24H);
}

/**
 * @brief  Display current time
 */
void display_time(void)
{
  ertc_time_type time;
  
  ertc_calendar_get(&time);
  
  printf("20%02d-%02d-%02d %02d:%02d:%02d\r\n",
         time.year, time.month, time.day,
         time.hour, time.min, time.sec);
}
```

### Example 2: Alarm Configuration

```c
/**
 * @brief  Configure Alarm A to trigger at specific time
 */
void alarm_config(uint8_t hour, uint8_t min, uint8_t sec)
{
  exint_init_type exint_init_struct;
  
  /* Disable alarm to configure */
  ertc_alarm_enable(ERTC_ALA, FALSE);
  
  /* Set alarm mask - match hour, minute, second (ignore date) */
  ertc_alarm_mask_set(ERTC_ALA, ERTC_ALARM_MASK_DATE_WEEK);
  
  /* Select date mode (not week) */
  ertc_alarm_week_date_select(ERTC_ALA, ERTC_SLECT_DATE);
  
  /* Set alarm time */
  ertc_alarm_set(ERTC_ALA, 1, hour, min, sec, ERTC_24H);
  
  /* Enable alarm interrupt */
  ertc_interrupt_enable(ERTC_ALA_INT, TRUE);
  
  /* Enable alarm */
  ertc_alarm_enable(ERTC_ALA, TRUE);
  
  /* Clear alarm flag */
  ertc_flag_clear(ERTC_ALAF_FLAG);
  
  /* Configure EXINT line 17 for alarm */
  exint_default_para_init(&exint_init_struct);
  exint_init_struct.line_enable   = TRUE;
  exint_init_struct.line_mode     = EXINT_LINE_INTERRUPT;
  exint_init_struct.line_select   = EXINT_LINE_17;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);
  
  /* Enable NVIC */
  nvic_irq_enable(ERTCAlarm_IRQn, 0, 1);
}

/**
 * @brief  Alarm interrupt handler
 */
void ERTCAlarm_IRQHandler(void)
{
  if(ertc_flag_get(ERTC_ALAF_FLAG) != RESET)
  {
    /* Clear alarm flag */
    ertc_flag_clear(ERTC_ALAF_FLAG);
    
    /* Clear EXINT flag */
    exint_flag_clear(EXINT_LINE_17);
    
    /* User alarm handler */
    printf("Alarm A triggered!\r\n");
  }
}
```

### Example 3: Wakeup Timer

```c
/**
 * @brief  Configure wakeup timer for periodic wakeup
 * @param  seconds: Wakeup period in seconds (1-65535)
 */
void wakeup_timer_config(uint16_t seconds)
{
  exint_init_type exint_init_struct;
  
  /* Disable wakeup timer first */
  ertc_wakeup_enable(FALSE);
  
  /* Select CK_B as clock source (1 Hz) */
  ertc_wakeup_clock_set(ERTC_WAT_CLK_CK_B_16BITS);
  
  /* Set wakeup counter (period = counter + 1 seconds) */
  ertc_wakeup_counter_set(seconds - 1);
  
  /* Enable wakeup interrupt */
  ertc_interrupt_enable(ERTC_WAT_INT, TRUE);
  
  /* Enable wakeup timer */
  ertc_wakeup_enable(TRUE);
  
  /* Configure EXINT line 22 */
  exint_default_para_init(&exint_init_struct);
  exint_init_struct.line_enable   = TRUE;
  exint_init_struct.line_mode     = EXINT_LINE_INTERRUPT;
  exint_init_struct.line_select   = EXINT_LINE_22;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);
  
  /* Enable NVIC */
  nvic_irq_enable(ERTC_WKUP_IRQn, 0, 1);
}

/**
 * @brief  Wakeup timer interrupt handler
 */
void ERTC_WKUP_IRQHandler(void)
{
  if(ertc_flag_get(ERTC_WATF_FLAG) != RESET)
  {
    /* Clear wakeup flag */
    ertc_flag_clear(ERTC_WATF_FLAG);
    
    /* Clear EXINT flag */
    exint_flag_clear(EXINT_LINE_22);
    
    /* User periodic handler */
    printf("Wakeup timer triggered!\r\n");
  }
}
```

### Example 4: Timestamp Capture

```c
/**
 * @brief  Configure timestamp on falling edge of PC13
 */
void timestamp_config(void)
{
  exint_init_type exint_init_struct;
  
  /* Configure EXINT line 21 */
  exint_default_para_init(&exint_init_struct);
  exint_init_struct.line_enable   = TRUE;
  exint_init_struct.line_mode     = EXINT_LINE_INTERRUPT;
  exint_init_struct.line_select   = EXINT_LINE_21;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);
  
  /* Enable NVIC */
  nvic_irq_enable(TAMP_STAMP_IRQn, 0, 0);
  
  /* Select timestamp pin (PC13 default) */
  ertc_timestamp_pin_select(ERTC_PIN_PC13);
  
  /* Set falling edge trigger */
  ertc_timestamp_valid_edge_set(ERTC_TIMESTAMP_EDGE_FALLING);
  
  /* Enable timestamp */
  ertc_timestamp_enable(TRUE);
  
  /* Enable timestamp interrupt */
  ertc_interrupt_enable(ERTC_TS_INT, TRUE);
}

/**
 * @brief  Get and display timestamp
 */
void display_timestamp(void)
{
  ertc_time_type ts_time;
  uint32_t subsec;
  
  /* Get timestamp */
  ertc_timestamp_get(&ts_time);
  subsec = ertc_timestamp_sub_second_get();
  
  printf("Timestamp: %02d-%02d %02d:%02d:%02d.%05d\r\n",
         ts_time.month, ts_time.day,
         ts_time.hour, ts_time.min, ts_time.sec,
         subsec);
}

/**
 * @brief  Timestamp interrupt handler
 */
void TAMP_STAMP_IRQHandler(void)
{
  if(ertc_flag_get(ERTC_TSF_FLAG) != RESET)
  {
    display_timestamp();
    
    /* Check for overflow */
    if(ertc_flag_get(ERTC_TSOF_FLAG) != RESET)
    {
      printf("Timestamp overflow!\r\n");
      ertc_flag_clear(ERTC_TSOF_FLAG);
    }
    
    /* Clear timestamp flag */
    ertc_flag_clear(ERTC_TSF_FLAG);
    
    /* Clear EXINT flag */
    exint_flag_clear(EXINT_LINE_21);
  }
}
```

### Example 5: Tamper Detection

```c
#define ERTC_BPR_DT_NUMBER  20

ertc_dt_type bpr_addr_tab[ERTC_BPR_DT_NUMBER] = {
  ERTC_DT1,  ERTC_DT2,  ERTC_DT3,  ERTC_DT4,  ERTC_DT5,
  ERTC_DT6,  ERTC_DT7,  ERTC_DT8,  ERTC_DT9,  ERTC_DT10,
  ERTC_DT11, ERTC_DT12, ERTC_DT13, ERTC_DT14, ERTC_DT15,
  ERTC_DT16, ERTC_DT17, ERTC_DT18, ERTC_DT19, ERTC_DT20
};

/**
 * @brief  Configure tamper detection with BPR erase
 */
void tamper_config(void)
{
  exint_init_type exint_init_struct;
  
  /* Configure EXINT line 21 */
  exint_default_para_init(&exint_init_struct);
  exint_init_struct.line_enable   = TRUE;
  exint_init_struct.line_mode     = EXINT_LINE_INTERRUPT;
  exint_init_struct.line_select   = EXINT_LINE_21;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);
  
  /* Enable NVIC */
  nvic_irq_enable(TAMP_STAMP_IRQn, 0, 0);
  
  /* Disable tamper first */
  ertc_tamper_enable(ERTC_TAMPER_1, FALSE);
  
  /* Clear tamper flag */
  ertc_flag_clear(ERTC_TP1F_FLAG);
  
  /* Configure rising edge trigger */
  ertc_tamper_valid_edge_set(ERTC_TAMPER_1, ERTC_TAMPER_EDGE_RISING);
  
  /* Enable tamper interrupt */
  ertc_interrupt_enable(ERTC_TP_INT, TRUE);
  
  /* Enable tamper detection */
  ertc_tamper_enable(ERTC_TAMPER_1, TRUE);
}

/**
 * @brief  Write test data to backup registers
 */
void bpr_write_test_data(void)
{
  uint32_t i;
  
  for(i = 0; i < ERTC_BPR_DT_NUMBER; i++)
  {
    ertc_bpr_data_write(bpr_addr_tab[i], 0x5A00 + (i * 0x5A));
  }
}

/**
 * @brief  Check if backup registers are intact
 * @retval TRUE if data matches, FALSE if erased/corrupted
 */
confirm_state bpr_check_data(void)
{
  uint32_t i;
  
  for(i = 0; i < ERTC_BPR_DT_NUMBER; i++)
  {
    if(ertc_bpr_data_read(bpr_addr_tab[i]) != (0x5A00 + (i * 0x5A)))
    {
      return FALSE;
    }
  }
  return TRUE;
}

/**
 * @brief  Tamper interrupt handler
 */
void TAMP_STAMP_IRQHandler(void)
{
  if(ertc_flag_get(ERTC_TP1F_FLAG) != RESET)
  {
    printf("Tamper detected! BPR registers erased.\r\n");
    
    /* Clear tamper flag */
    ertc_flag_clear(ERTC_TP1F_FLAG);
    
    /* Clear EXINT flag */
    exint_flag_clear(EXINT_LINE_21);
  }
}
```

### Example 6: LICK Calibration

```c
__IO uint32_t capture_number = 0;
__IO uint32_t period_value = 0;
__IO uint32_t capture_prev = 0;

/**
 * @brief  Measure LICK frequency using TMR5
 * @retval LICK frequency in Hz
 */
uint32_t lick_frequency_get(void)
{
  tmr_input_config_type tmr_input_config;
  crm_clocks_freq_type crm_clocks;
  
  /* Enable TMR5 clock */
  crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);
  
  /* Connect TMR5_CH4 internally to LICK */
  tmr_iremap_config(TMR5, TMR2_PTP_TMR5_LICK);
  
  /* Configure TMR5 prescaler */
  tmr_div_value_set(TMR5, 0);
  tmr_event_sw_trigger(TMR5, TMR_OVERFLOW_SWTRIG);
  
  /* Configure input capture */
  tmr_input_config.input_channel_select = TMR_SELECT_CHANNEL_4;
  tmr_input_config.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
  tmr_input_config.input_polarity_select = TMR_INPUT_RISING_EDGE;
  tmr_input_config.input_filter_value = 0;
  tmr_input_channel_init(TMR5, &tmr_input_config, TMR_CHANNEL_INPUT_DIV_1);
  tmr_input_channel_divider_set(TMR5, TMR_SELECT_CHANNEL_4, TMR_CHANNEL_INPUT_DIV_8);
  
  /* Enable interrupt */
  nvic_irq_enable(TMR5_GLOBAL_IRQn, 0, 0);
  tmr_interrupt_enable(TMR5, TMR_C4_INT, TRUE);
  
  /* Enable counter */
  tmr_counter_enable(TMR5, TRUE);
  
  /* Wait for measurement */
  while(capture_number < 2);
  
  /* Cleanup */
  tmr_reset(TMR5);
  
  /* Calculate frequency */
  crm_clocks_freq_get(&crm_clocks);
  
  if(CRM->cfg_bit.apb1div > 0)
  {
    return (((2 * crm_clocks.apb1_freq) / period_value) * 8);
  }
  else
  {
    return ((crm_clocks.apb1_freq / period_value) * 8);
  }
}

/**
 * @brief  TMR5 interrupt handler for LICK measurement
 */
void TMR5_GLOBAL_IRQHandler(void)
{
  if(tmr_flag_get(TMR5, TMR_C4_FLAG) != RESET)
  {
    uint32_t capture = tmr_channel_value_get(TMR5, TMR_SELECT_CHANNEL_4);
    
    if(capture_number == 0)
    {
      capture_prev = capture;
      capture_number = 1;
    }
    else
    {
      period_value = capture - capture_prev;
      capture_number = 2;
    }
    
    tmr_flag_clear(TMR5, TMR_C4_FLAG);
  }
}

/**
 * @brief  Configure ERTC with calibrated LICK
 */
void ertc_lick_calibrated_init(void)
{
  uint32_t lick_freq;
  
  /* Standard ERTC init with LICK */
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  pwc_battery_powered_domain_access(TRUE);
  
  crm_battery_powered_domain_reset(TRUE);
  crm_battery_powered_domain_reset(FALSE);
  
  crm_clock_source_enable(CRM_CLOCK_SOURCE_LICK, TRUE);
  while(crm_flag_get(CRM_LICK_STABLE_FLAG) == RESET);
  
  crm_ertc_clock_select(CRM_ERTC_CLOCK_LICK);
  crm_ertc_clock_enable(TRUE);
  
  ertc_reset();
  ertc_wait_update();
  
  /* Measure actual LICK frequency */
  lick_freq = lick_frequency_get();
  
  /* Configure calibrated divider */
  /* 1 Hz = lick_freq / (127+1) / (div_b+1) */
  /* div_b = (lick_freq / 128) - 1 */
  ertc_divider_set(127, (lick_freq / 128) - 1);
  
  printf("LICK frequency: %d Hz\r\n", lick_freq);
  printf("DIV_B value: %d\r\n", (lick_freq / 128) - 1);
  
  ertc_hour_mode_set(ERTC_HOUR_MODE_24);
  ertc_date_set(24, 1, 1, 1);
  ertc_time_set(12, 0, 0, ERTC_24H);
}
```

### Example 7: Backup Domain Persistence

```c
#define ERTC_INIT_MAGIC  0x1234

/**
 * @brief  Initialize ERTC with persistence check
 */
void ertc_persistent_init(void)
{
  /* Enable PWC and allow access */
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  pwc_battery_powered_domain_access(TRUE);
  
  /* Check if ERTC was previously initialized */
  if(ertc_bpr_data_read(ERTC_DT1) != ERTC_INIT_MAGIC)
  {
    printf("ERTC: First initialization\r\n");
    
    /* Reset and configure */
    crm_battery_powered_domain_reset(TRUE);
    crm_battery_powered_domain_reset(FALSE);
    
    crm_clock_source_enable(CRM_CLOCK_SOURCE_LEXT, TRUE);
    while(crm_flag_get(CRM_LEXT_STABLE_FLAG) == RESET);
    
    crm_ertc_clock_select(CRM_ERTC_CLOCK_LEXT);
    crm_ertc_clock_enable(TRUE);
    
    ertc_reset();
    ertc_wait_update();
    ertc_divider_set(127, 255);
    ertc_hour_mode_set(ERTC_HOUR_MODE_24);
    
    /* Set default time */
    ertc_date_set(24, 1, 1, 1);
    ertc_time_set(12, 0, 0, ERTC_24H);
    
    /* Mark as initialized */
    ertc_bpr_data_write(ERTC_DT1, ERTC_INIT_MAGIC);
  }
  else
  {
    printf("ERTC: Already initialized, restoring...\r\n");
    
    /* Just wait for synchronization */
    ertc_wait_update();
    
    /* Clear any pending alarm flags */
    ertc_flag_clear(ERTC_ALAF_FLAG);
    ertc_flag_clear(ERTC_ALBF_FLAG);
    exint_flag_clear(EXINT_LINE_17);
  }
}

/**
 * @brief  Store user data in backup registers
 */
void store_application_data(uint32_t data, uint8_t reg_index)
{
  if(reg_index >= 2 && reg_index < 20)  /* Reserve DT1 for magic number */
  {
    ertc_bpr_data_write((ertc_dt_type)reg_index, data);
  }
}

/**
 * @brief  Read user data from backup registers
 */
uint32_t read_application_data(uint8_t reg_index)
{
  if(reg_index >= 2 && reg_index < 20)
  {
    return ertc_bpr_data_read((ertc_dt_type)reg_index);
  }
  return 0;
}
```

## Initialization Sequence

```
┌─────────────────────────────────────┐
│ 1. Enable PWC peripheral clock      │
│    crm_periph_clock_enable()        │
└─────────────────┬───────────────────┘
                  ▼
┌─────────────────────────────────────┐
│ 2. Enable battery domain access     │
│    pwc_battery_powered_domain_access│
└─────────────────┬───────────────────┘
                  ▼
┌─────────────────────────────────────┐
│ 3. Reset battery domain (optional)  │
│    crm_battery_powered_domain_reset │
└─────────────────┬───────────────────┘
                  ▼
┌─────────────────────────────────────┐
│ 4. Enable clock source (LEXT/LICK)  │
│    crm_clock_source_enable()        │
│    Wait for stable flag             │
└─────────────────┬───────────────────┘
                  ▼
┌─────────────────────────────────────┐
│ 5. Select ERTC clock source         │
│    crm_ertc_clock_select()          │
│    crm_ertc_clock_enable()          │
└─────────────────┬───────────────────┘
                  ▼
┌─────────────────────────────────────┐
│ 6. Reset ERTC and wait for update   │
│    ertc_reset()                     │
│    ertc_wait_update()               │
└─────────────────┬───────────────────┘
                  ▼
┌─────────────────────────────────────┐
│ 7. Configure prescaler              │
│    ertc_divider_set(div_a, div_b)   │
└─────────────────┬───────────────────┘
                  ▼
┌─────────────────────────────────────┐
│ 8. Set hour mode                    │
│    ertc_hour_mode_set()             │
└─────────────────┬───────────────────┘
                  ▼
┌─────────────────────────────────────┐
│ 9. Set initial date and time        │
│    ertc_date_set()                  │
│    ertc_time_set()                  │
└─────────────────┬───────────────────┘
                  ▼
┌─────────────────────────────────────┐
│ 10. Configure features as needed    │
│     Alarms, Wakeup, Timestamp, etc. │
└─────────────────────────────────────┘
```

## Hardware Connections

### PC13 Multi-function Pin

| Function | Configuration |
|----------|---------------|
| ERTC Output | Alarm, Wakeup, Calibration output |
| Tamper 1 | Tamper detection input |
| Timestamp | Timestamp trigger input |

### PA0 Alternative Pin

| Function | Configuration |
|----------|---------------|
| Tamper 1 Alt | Alternative tamper 1 input |
| Timestamp Alt | Alternative timestamp input |

### PI8 (Tamper 2)

| Function | Configuration |
|----------|---------------|
| Tamper 2 | Second tamper detection input |

## Calibration

### Smooth Calibration

The smooth calibration allows fine adjustment of the ERTC clock:

```c
/* Add 512 pulses in 32-second period */
ertc_smooth_calibration_config(ERTC_SMOOTH_CAL_PERIOD_32,
                               ERTC_SMOOTH_CAL_CLK_ADD_512,
                               0);

/* Remove 100 pulses in 32-second period */
ertc_smooth_calibration_config(ERTC_SMOOTH_CAL_PERIOD_32,
                               ERTC_SMOOTH_CAL_CLK_ADD_0,
                               100);
```

### Calibration Range

| Period | Add Pulses | Dec Pulses | PPM Range |
|--------|------------|------------|-----------|
| 32s | 0 or 512 | 0-511 | -487.1 to +488.5 |
| 16s | 0 or 512 | 0-511 | -974.2 to +977.0 |
| 8s | 0 or 512 | 0-511 | -1948.4 to +1954.0 |

## Low Power Considerations

### VBAT Mode

When VDD is removed and VBAT is present:
- ERTC continues operation
- Backup registers retain data
- PC13 tamper/timestamp can still trigger
- No interrupts to CPU

### Standby Mode Wake Sources

| Source | EXINT Line | Wake Capability |
|--------|------------|-----------------|
| Alarm A | EXINT17 | Yes |
| Alarm B | EXINT17 | Yes |
| Wakeup Timer | EXINT22 | Yes |
| Timestamp | EXINT21 | No |
| Tamper | EXINT21 | No |

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| ERTC not running | Clock source not enabled | Enable LEXT/LICK and wait for stable |
| Time drifts | Wrong prescaler values | Verify DIV_A and DIV_B calculation |
| BPR data lost | Battery domain reset | Check battery connection, avoid unnecessary resets |
| Alarm not firing | Interrupt not configured | Configure EXINT line and NVIC |
| Timestamp not working | Wrong edge configured | Verify edge polarity matches signal |
| Write fails | Write protection enabled | Call `ertc_write_protect_disable()` |

## Performance Specifications

| Parameter | Value |
|-----------|-------|
| Clock accuracy (LEXT) | ±20 ppm (typical crystal) |
| Clock accuracy (LICK) | ±5% (requires calibration) |
| Sub-second resolution | 1/32768 s (LEXT) |
| Backup current (VBAT) | ~1.4 µA typical |
| Wakeup timer range | 1s to 65536s (or more with 17-bit) |
| Alarm resolution | 1 second (or sub-second) |

## Related Peripherals

| Peripheral | Relationship |
|------------|--------------|
| PWC | Battery domain access control |
| CRM | Clock source selection and enable |
| EXINT | Interrupt line routing |
| TMR5 | LICK frequency measurement |
| GPIO | Tamper/Timestamp/Output pins |

## References

- AT32F435/437 Reference Manual - ERTC Chapter
- AT32F435/437 Datasheet - Electrical Characteristics
- Application Note AN0047 - ERTC Configuration

