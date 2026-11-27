# `gps-tag-drivers`

`no_std` Embassy hardware drivers and board support package for the **STM32F412RG** GPS Tag board.

## Modules

| Module | Hardware | Highlights |
| --- | --- | --- |
| `board` | `STM32F412RGTx` | 16 MHz HSE, 48 MHz PLL1_Q hardware `RNG`, 32.768 kHz LSE, `TIM4` time driver, `USART6` (4 KiB DMA RX), `I2C1` / `EXTI`, holds `MBN52832` in reset (`PC2` HIGH) |
| `modem` | Quectel `EC21-AU` | Cold-boot PMIC wait, volatile `460800 bps` UART upgrade, `AT+CMUX=0` multiplexing (`DLCI 1` AT/GPS, `DLCI 2` PPP) |
| `modem::cmux` | TS 27.010 Basic Option | Bidirectional `MSC` flow control (`FC=1`/`FC=0`), chunked RX, shared-`0xF9` flags, EC21 `PppDeframer` (~34 KB/s lossless at `460800 bps`) |
| `modem::gps` | EC21-AU GNSS | Multi-constellation (`gnssconfig=1`), Qualcomm DPO (`dpoenable=1`), `xtra3grc.bin` + NITZ UTC, periodic `fixrate`, clean `AT+QGPSEND` Hot Start |
| `modem::power` | EC21-AU Sleep / RF | DTR slow-clock sleep (`AT+QSCLK=1`, `PC8` HIGH=sleep / LOW=wake in ~85 ms keeping LTE + CMUX + PPP alive), standby, flight mode (`AT+CFUN`) |
| `modem::ppp` | `embassy-net-ppp` | IPv4 stack over `DLCI 2` concurrent with `DLCI 1` AT/GPS, automatic re-dial on drop (`set_auto_redial` / `request_redial`), + built-in `run_perf_test` benchmark |
| `modem::https` | `reqwless` + `embedded-tls` | TLS 1.3 HTTPS / HTTP client (`get`, `post_json`, `request`) over PPP with hardware TRNG seeding and optional DER CA / mTLS verification |
| `modem::https::sse` | WHATWG SSE over TLS 1.3 | Zero-allocation Server-Sent Events streaming client (`listen`, `listen_once`) with `Last-Event-ID`, `retry:`, `: keepalive`, and automatic cellular reconnect |
| `modem::sms` | EC21-AU SMS | Text-mode SMS init, send, read (`AT+CMGR`/`AT+CMGL`), `+CMTI` async receiver (`sms::subscribe`), and delete over `DLCI 1` |
| `modem::status` | EC21-AU Telemetry | Typed battery % / mV (`AT+CBC`), signal dBm (`AT+CSQ`), LTE registration (`AT+CEREG?`), operator (`AT+COPS?`), UTC time (`AT+QLTS=1`), ADC (`AT+QADC`), IMEI/ICCID, and combined `snapshot()` |
| `bmi160` | Bosch `BMI160` | `I2C1` (`PB8`/`PB9` @ `0x68`), 6-axis burst reads, temperature, hardware `AnyMotion`/`NoMotion`/`HighG`/`LowG` interrupts on `INT1` (`PB7`) / `INT2` (`PB6`) |
| `soft_i2c` | Bit-banged I2C | Open-drain ~100 kHz `embedded_hal::i2c::I2c` on `Flex` pins, clock stretching (5 ms), 9-clock bus recovery, runtime pin swap |
| `st25dv` | ST `ST25DV16K` | DS10925 Rev 12 compliant EEPROM (4 B page-aligned timeouts), `GPO -> SCL` (`PC10`/`EXTI10`) zero-polling RF-write wake & debounce, generic `watch_rf_write` / `protect_ndef_records_on_rf_write` / `protect_region_on_rf_write`, `RFA1SS` lock, `RF_MNGT_Dyn`, FTM mailbox |
| `st25dv::ndef` | NFC Type 5 NDEF | Zero-allocation record iterator (`iter_type5_records`), per-record decoders (`decode_uri_record`, `decode_text_record`, `decode_vcard_record`, `decode_record`), and generic encoder (`encode_type5_records` with URI, Text, `ContactCard` / raw vCard, and Raw records) |
| `buzzer` | Piezo (`TIM3_CH1`) | Async PWM `beep()` and BPM-timed `play()` melodies (`PB4`) |
| `i2c_scanner` | `I2C1`/`I2C2`/`I2C3` | Hardware I2C bus diagnostic scanner |

## Building API Documentation

Generate and open full `rustdoc` API documentation (from the workspace root inside `nix develop`):

```bash
cargo doc -p gps-tag-drivers --no-deps --open
```

## Running Examples on Hardware

All examples live in `drivers/examples/` and flash via `probe-rs` (`--probe 0483:3748` if multiple probes are connected):

| Example | Command | What it tests |
| --- | --- | --- |
| Buzzer & LED | `cargo run -p gps-tag-drivers --example buzzer_melody` | Built-in melodies (`CHIRP`, `NOKIA`) + single notes on `PB4` |
| BMI160 Motion Wake | `cargo run -p gps-tag-drivers --example imu_motion_wake` | IMU init, temp/accel read, `EXTI` wake on `INT1` (any-motion) & `INT2` (high-G) |
| ST25DV16K Read on IRQ | `cargo run -p gps-tag-drivers --example nfc_gpo_read` | Zero-polling `GPO -> SCL` (`EXTI10`) wake + generic per-record NDEF decoding on phone write |
| ST25DV16K Rewrite Protect | `cargo run -p gps-tag-drivers --example nfc_gpo_protect` | Zero-polling `GPO -> SCL` (`EXTI10`) leading-record rewrite protection via `protect_ndef_records_on_rf_write` (preserves extra records) |
| ST25DV16K vCard Contact | `cargo run -p gps-tag-drivers --example nfc_vcard` | Encodes & writes a vCard 3.0 `ContactCard` to EEPROM, verifies round-trip decode, and watches `EXTI10` |
| EC21-AU Low-Power GPS | `cargo run -p gps-tag-drivers --example modem_gps_low_power` | `AT+QSCLK=1` DTR sleep, DPO + 5 s `fixrate`, `poll_fix_and_sleep`, standby + Hot Start |
| EC21-AU PPP + GPS | `cargo run -p gps-tag-drivers --example modem_ppp_http` | `460800 bps` PPP DNS + TCP HTTP GET on `DLCI 2` concurrently with 1 Hz GPS on `DLCI 1` |
| EC21-AU HTTPS (TLS 1.3) | `cargo run -p gps-tag-drivers --example modem_https` | Hardware TRNG + TLS 1.3 HTTPS `GET` (`example.com`) and `POST` JSON telemetry (`httpbin.org`) over PPP |
| EC21-AU HTTPS SSE | `cargo run -p gps-tag-drivers --example modem_sse` | Persistent TLS 1.3 Server-Sent Events command stream (`ntfy.sh` SSE) with auto-reconnect and `Last-Event-ID` |
| SMS to NFC Tag | `cargo run -p gps-tag-drivers --example sms_to_nfc` | Logs `modem::status` telemetry (battery %/mV, signal, operator), receives SMS (`+CMTI`/`AT+CMGR`), and writes the text to ST25DV16K as an NDEF Text record |
