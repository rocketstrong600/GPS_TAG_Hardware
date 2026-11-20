# Hardware Connections & Peripherals (Reverse-Engineered)

> **Target MCU:** `STM32F412RGTx` (`stm32f412rg`, LQFP-64)  
> **Cellular/GNSS Modem:** Quectel `EC21-AU`  
> **IMU:** Bosch `BMI160` (6-axis accelerometer + gyroscope)

---

## 1. Clocks & System Timers

| Resource | Configuration | Notes |
| --- | --- | --- |
| **HSE** | 16 MHz (`HseMode::Oscillator`) | `Sysclk::HSE` (`PH0` / `PH1`) |
| **LSE** | 32.768 kHz (`LseMode::Oscillator`, `LseDrive::MediumLow`) | `RtcClockSource::LSE` (`PC14` / `PC15`) |
| **Embassy Time Driver** | `TIM4` (`tick-hz-32_768`) | Configured in `Cargo.toml` |
| **Debug Interface** | SWD (`PA13` SWDIO, `PA14` SWCLK) | Used via `probe-rs` + `defmt-rtt` |

---

## 2. Modem — Quectel `EC21-AU`

### UART Interface (`USART6`)

| STM32 Pin | Peripheral / Signal | Notes |
| --- | --- | --- |
| `PC6` | `USART6_TX` | DMA: `DMA2_CH6` |
| `PC7` | `USART6_RX` | DMA: `DMA2_CH2` (ring-buffered) |

### Control & Power GPIOs

| STM32 Pin | Firmware Name | Initial / Active State | Purpose |
| --- | --- | --- | --- |
| `PB12` | `m_batt` | Init `LOW`, driven `HIGH` | Modem VBAT / power rail enable |
| `PC4` | `m_pwr_key` | Init `LOW`, pulsed `HIGH` for 750 ms | Modem `PWRKEY` |
| `PA7` | `m_reset` | Init `LOW` | Modem `RESET_N` |
| `PB0` | `m_ap_ready` | Init `LOW`, driven `HIGH` | `AP_READY` |
| `PB1` | `m_wake` | Init `LOW`, driven `HIGH` | Modem wake |
| `PB15` | `m_uart_enable` | Init `HIGH` | UART level-shifter / buffer enable |
| `PC8` | `m_uart_dtr` | Init `LOW` | UART `DTR` |

### Power-On Sequence (`src/main.rs`)

1. **Initial pin states:** `PB15` (`m_uart_enable`) = `HIGH`; `PB0` (`m_ap_ready`), `PB1` (`m_wake`), `PB12` (`m_batt`), `PC4` (`m_pwr_key`), `PA7` (`m_reset`), `PC8` (`m_uart_dtr`) = `LOW`.
2. Wait **400 ms** after `USART6` / ATAT setup.
3. **Enable power & wake lines:** Set `PB0` (`m_ap_ready`), `PB1` (`m_wake`), and `PB12` (`m_batt`) `HIGH`.
4. Wait **250 ms** for VBAT rail to stabilize.
5. **Pulse `PWRKEY`:** Set `PC4` (`m_pwr_key`) `HIGH` for **750 ms**, then set `PC4` `LOW`.
6. **Wait for boot:** Spawn `reader_task` on `USART6_RX` and wait for the `RDY` URC (`modem_commands::Urc::Ready`).
7. **Post-ready:** Play Nokia tune on `PB4` (`TIM3_CH1`), spawn `blink_task` on `PB5`, wait **2 s**, then poll GNSS (`GetGpsLocation`) every 30 s.

---

## 3. IMU — Bosch `BMI160`

- **Bus:** `I2C1` (external pull-ups detected on `PB8` and `PB9`)
- **I2C Address:** `0x68` (`SDO` low)
- **Chip ID (`0x00`):** `0xD1` (verified on hardware)

| STM32 Pin | Signal | Notes |
| --- | --- | --- |
| `PB8` | `I2C1_SCL` | Tested at 100 kHz |
| `PB9` | `I2C1_SDA` | Tested at 100 kHz |
| `PB7` | `INT1` | Verified by toggling `INT_OUT_CTRL` (`0x53`) |
| `PB6` | `INT2` | Verified by toggling `INT_OUT_CTRL` (`0x53`) |

---

## 4. Board Peripherals (UI / Audio)

| STM32 Pin | Peripheral / Signal | Notes |
| --- | --- | --- |
| `PB5` | `Output` (`led`) | Status LED, toggled every 1 s in `blink_task` |
| `PB4` | `TIM3_CH1` (`SimplePwm`, Push-Pull) | Speaker / buzzer PWM output |

---

## 5. Unidentified / Observed Signals

| STM32 Pin | Firmware / Probe Observation | Notes |
| --- | --- | --- |
| `PB14` | `_uknwn_vreg` | Suspected voltage regulator / lights enable |
| `PB13` | `_stm_out_2` | Output pin (unidentified function) |
| `PA12` | Input probe `0b1111` | Continuously reads `HIGH` against internal `Pull::Down` (external pull-up or driven high) |
| `PC0` | Input probe `0b1111` | Continuously reads `HIGH` against internal `Pull::Down` (external pull-up or driven high) |

---

## 6. Unconnected / Floating Candidates (Tested)

During the `I2C` and `BMI160` GPIO probes:
- **No I2C devices found** on `I2C1 (PB6/PB7)`, `I2C2 (PB10/PB3)`, or `I2C3 (PA8/PC9)`.
- **Read `LOW` (with internal `Pull::Down`):** `PA0..PA6`, `PA8..PA11`, `PA15`, `PB2`, `PB3`, `PB10`, `PC1..PC3`, `PC5`, `PC9..PC13`, `PD2`.
