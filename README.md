# FakeFC

An emulated [Betaflight](https://betaflight.com/) flight controller running on an ESP32. FakeFC lets you use **Betaflight Configurator** and **ELRS Configurator** without a real flight controller — ideal for integration testing ExpressLRS (ELRS) receivers and flashing their firmware via the Betaflight Passthrough method.

## Features

- **MSP v1 / v2 protocol** — responds to Betaflight Configurator queries so it appears as a real Betaflight 4.3 FC
- **CRSF RC receiver** — reads 16 RC channels from an ExpressLRS receiver at 420000 baud
- **Serial passthrough** — bridges USB ↔ ELRS receiver UART so ELRS Configurator can flash receiver firmware through the standard Betaflight Passthrough workflow
- **DTR forwarding** *(ESP32-S2 only)* — forwards the USB DTR signal to the ELRS receiver's reset pin so `esptool` can trigger bootloader mode automatically
- **Web dashboard** — live HTTP page showing all 16 RC channel values, served from a built-in WiFi access point

## Hardware Requirements

| Component | Details |
|---|---|
| Microcontroller | ESP32-S2 Mini (LOLIN S2 Mini) **or** ESP32 WROOM 32 |
| ELRS receiver | Any ExpressLRS receiver with a UART CRSF output |

> **Recommended board:** ESP32-S2 Mini — it has native USB CDC, which enables automatic DTR forwarding for hands-free ELRS firmware flashing. The WROOM 32 works too but requires manually resetting the ELRS receiver before flashing.

## Wiring

| Signal | ESP32 GPIO | ELRS Receiver pin |
|---|---|---|
| CRSF RX (ESP32 receives) | GPIO 25 | TX |
| CRSF TX / Reset (ESP32 sends) | GPIO 26 | RX / RST |

The USB port of the ESP32 connects to your computer and is used for both Betaflight Configurator communication (MSP) and ELRS firmware flashing (passthrough).

> Pin numbers are defined in [`src/crsf.h`](src/crsf.h) (`CRSF_RX_PIN` / `CRSF_TX_PIN`) and can be changed to match your board layout.

## Getting Started

### Prerequisites

Install [PlatformIO](https://platformio.org/) (CLI or the VS Code extension).

### Build & Flash

```bash
# Clone the repository
git clone https://github.com/Akceptor/FakeFC.git
cd FakeFC

# Flash to ESP32-S2 Mini (recommended)
platformio run --environment lolin_s2_mini --target upload

# Flash to ESP32 WROOM 32
platformio run --environment esp32_wroom_32 --target upload
```

### Monitor Serial Output

```bash
platformio device monitor --speed 115200
```

## Usage

### Betaflight Configurator

1. Connect the ESP32 to your computer via USB.
2. Open [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator).
3. Select the correct COM/serial port and click **Connect**.
4. FakeFC identifies itself as Betaflight 4.3 and responds to all standard MSP queries (board info, RC channels, serial config, etc.).

The **Receiver** tab will show live RC channel values streamed from the ELRS receiver.

### ELRS Receiver Firmware Flashing (Betaflight Passthrough)

1. Wire the ELRS receiver to the ESP32 as described in the [Wiring](#wiring) section.
2. Open [ELRS Configurator](https://github.com/ExpressLRS/ExpressLRS-Configurator) and select the **Betaflight Passthrough** flashing method.
3. ELRS Configurator will:
   - Query FakeFC via MSP to discover UART1 (CRSF receiver port)
   - Enter CLI mode and issue `serialpassthrough 1 420000`
4. FakeFC bridges USB ↔ UART1, allowing `esptool` to flash the receiver.
5. On ESP32-S2, DTR changes are forwarded automatically to reset the receiver into bootloader mode. On WROOM 32, reset the receiver manually when prompted.
6. **Power-cycle the ESP32** after flashing to return to normal operation.

### Web Dashboard

After power-on, FakeFC creates a WiFi access point:

| Setting | Value |
|---|---|
| SSID | `FakeFC` |
| Password | *(none — open network)* |
| Dashboard URL | http://192.168.4.1 |

Connect to the `FakeFC` network and open the URL in a browser to see a live bar graph of all 16 RC channels, updated every 100 ms.

The endpoint `GET /channels` returns a JSON array of the 16 channel values (1000–2000 µs range) for programmatic access:

```json
[1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
```

## Project Structure

```
FakeFC/
├── platformio.ini          # PlatformIO build configuration
└── src/
    ├── main.cpp            # Setup & main loop
    ├── crsf.h / crsf.cpp   # CRSF protocol decoder (AlfredoCRSF wrapper)
    ├── msp.h / msp.cpp     # MSP v1 / v2 state machine and command handlers
    ├── passthrough.h / passthrough.cpp  # CLI parser and UART passthrough bridge
    └── web.h / web.cpp     # WiFi AP, HTTP server and RC channel dashboard
```

## Board Comparison

| Feature | ESP32-S2 Mini | ESP32 WROOM 32 |
|---|---|---|
| USB | Native USB CDC (TinyUSB) | UART-to-USB bridge (CP2102 / CH340) |
| Automatic DTR forwarding | ✅ Yes | ❌ No — manual receiver reset required |
| Build environment | `lolin_s2_mini` | `esp32_wroom_32` |

## License

This project does not currently include a license file. Please contact the repository owner for usage terms.
