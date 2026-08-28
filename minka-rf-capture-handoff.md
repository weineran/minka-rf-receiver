# MinkaAire RF Light Control - Claude Code Handoff

## Project Goal

Build ESP32 + CC1101 firmware that:
1. **Captures and decodes** the MinkaAire ceiling fan remote's light button RF signal
2. **Toggles an SPDT relay** when the light signal is received, with debounce
3. The relay is wired as one half of a **3-way switch pair** (the other half is a wall dimmer)

This is a DIY replacement for the MinkaAire WCS213-S wall controller, which has a known firmware debounce defect in its light button (toggle commands fire multiple times per press, causing the light to flash or not respond). The ESP32/CC1101 listens for the same RF signal and applies proper debounce before toggling the relay.

## Hardware

### Wiring: CC1101 (wireless receiver) to ESP32 (microcontroller)

| CC1101 pin | Label | ESP32 pin |
|---|---|---|
| 1 | GND | GND |
| 2 | VCC | 3V3 (NOT 5V) |
| 3 | GDO0 | GPIO 26 (D26) |
| 4 | CSN | GPIO 5 (D5) |
| 5 | SCK | GPIO 18 (D18) |
| 6 | MOSI | GPIO 23 (D23) |
| 7 | MISO/GDO1 | GPIO 19 (D19) |
| 8 | GDO2 | GPIO 27 (D27) |

### Relay (not yet wired, for later phase)

| ESP32 pin | Relay module pin |
|---|---|
| GPIO 4 (configurable) | Signal / IN |
| 5V (from HLK-PM01 power supply) | VCC |
| GND | GND |

Relay is SPDT (COM, NO, NC terminals). Supports 3.3V trigger (high/low level trigger with optocoupler).

### CC1101 module
- MELIFE CC1101 with SMA antenna, v2.0 8-pin module
- Module is labeled "433M" but the CC1101 chip supports 300-348 MHz natively
- Antenna is tuned for 433 MHz; sensitivity at 303 MHz is reduced but functional at short range (tested)

### ESP32 module
- ESP-WROOM-32 DevKitC with CP2102 USB-UART bridge
- Connects to PC via USB Micro-B (must use a DATA cable, not charge-only)
- Shows up as Silicon Labs CP210x on COM5 (Windows, after driver install)

### Remote control
- Model: DL-4111T-01 (MinkaAire / Summer Wind International)
- FCC ID: 2A767-DL-4111T
- Frequency: 303.875 MHz (confirmed via FCC filing)
- Single frequency for ALL functions (fan and light)
- Modulation: OOK/ASK
- Encoding: 256-bit pairing codes (proprietary, Rhine Electronics / Summer Wind)
- Two light buttons (top = main light on/off + hold to brighten, bottom = light on/off + hold to dim)
- Three fan speed buttons + fan off button
- Light buttons send a TOGGLE command (same signal for on and off)

## Development Environment

- **IDE:** VS Code + PlatformIO extension
- **Platform:** espressif32
- **Board:** esp32dev
- **Framework:** Arduino
- **Library:** SmartRC-CC1101-Driver-Lib@^2.5.7 (by LSatan)
- **Upload port:** COM5 (auto-detected)
- **Monitor speed:** 115200 baud
- **Owner's OS:** Windows 11, PowerShell 7

### platformio.ini
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    LSatan/SmartRC-CC1101-Driver-Lib@^2.5.7
```

## What's Been Done

### v1 firmware (signal capture, narrow settings)
- CC1101 configured: 303.875 MHz, OOK, data rate 2.048 kbps, RX bandwidth 135 kHz
- Successfully captured fan button signals (clean, consistent)
- Light button signals: almost never captured (1 out of ~10 presses)

### v1 findings: fan signal characteristics
- Two distinct pulse durations: ~315 us ("short") and ~710 us ("long")
- Roughly 1:2 ratio, consistent with PWM-style OOK encoding
- ~12000 us gaps between repeated frames
- Each button press sends the frame 3-8 times
- Pattern: short-HIGH + long-LOW = one bit value, long-HIGH + short-LOW = other bit value
- Each frame appears to have ~12 data bit-pairs before the ~12000 us gap

### The problem
The light buttons on the remote ARE transmitting at 303.875 MHz (confirmed by FCC filing, single-frequency device). But the v1 CC1101 settings are not capturing the light signal. The light signal likely has different pulse timing, data rate, or encoding characteristics than the fan signal. The CC1101's digital filter at 2.048 kbps / 135 kHz bandwidth was rejecting the light signal.

### v2 firmware (current, wider settings)
Changes made to catch the light signal:
- RX bandwidth: 650 kHz (was 135 kHz)
- Data rate: 1.0 kbps (was 2.048 kbps)
- AGC registers set to max sensitivity (AGCCTRL2=0x03, AGCCTRL1=0x00, AGCCTRL0=0x91)
- MIN_PULSES lowered to 8 (was 20)
- Added RSSI monitoring (prints signal strength every 500ms)
- Added bit stream decoder (converts pulse pairs to 0/1 based on short/long threshold)

v2 has NOT been tested yet. The owner is about to flash and test it.

## Next Steps

### Phase 1: Signal Capture (current)
1. Flash v2 firmware
2. Test with remote held close to CC1101 antenna
3. Monitor RSSI to confirm light buttons produce RF energy at 303.875 MHz
4. If RSSI spikes but no signal capture: adjust CC1101 demodulation settings (data rate, bandwidth, AGC)
5. If no RSSI spike: investigate antenna (may need 24.5 cm wire antenna for 303 MHz quarter-wave)
6. Capture clean signals from: top light button, bottom light button, fan speed 1, fan speed 2, fan speed 3, fan off
7. Decode the bit patterns for each command
8. Identify the unique bit pattern for the light toggle command(s)

### Phase 2: Relay Toggle Firmware
Once the light signal is decoded:
1. Program ESP32 to recognize the light toggle bit pattern
2. On valid light signal received: toggle relay (GPIO 4)
3. Apply 500ms debounce window (ignore duplicate signals within window)
4. Must distinguish light commands from fan commands (only react to light)
5. Must match on the room's specific 256-bit pairing code (ignore signals from other rooms' remotes)
6. The relay is SPDT wired as a 3-way switch companion to a wall dimmer

### Phase 3: Production Firmware
- Strip out serial debug output
- Add watchdog timer for reliability
- Consider OTA update capability (optional, WiFi not required for core function)
- Optimize power consumption (ESP32 will be always-on, powered by HLK-PM01 AC-DC converter)
- Solution will be deployed in 5-6 rooms, each with its own ESP32/CC1101/relay and unique pairing code

## Key Reference Projects

These projects have solved similar problems (303 MHz ceiling fan control with CC1101):

- [ESPHome CC1101 + MinkaAire at 303.875 MHz (HA Community)](https://community.home-assistant.io/t/esphome-cc1101-success-with-minkaaire-ceiling-fan-303-875-mhz/1012700)
- [CC1101 + MQTT fan controller for 303 MHz (GitHub)](https://github.com/spetryjohnson/cc1101-mqtt-fan-controller)
- [Hampton Bay fan MQTT bridge at 303 MHz (GitHub)](https://github.com/owenb321/hampton-bay-fan-mqtt)
- [General ESP32 + CC1101 RF capture/replay (GitHub)](https://github.com/sha1cybr/esp32-cc1101-rf-caprep)

## Design Constraints

- No soldering. All connections are Dupont jumper wires (female-to-female) or screw terminals.
- No WiFi or internet required for core operation.
- No Home Assistant required. Standalone firmware preferred.
- Owner is comfortable with hardware but prefers minimal complexity.
- Owner studied electrical and computer engineering, comfortable with microcontrollers and RF concepts.

## Full System Design Context

For the complete system design (wiring diagrams, component list, 3-way switch wiring, installation procedure), see the separate design document: `minka-aire-light-control-design.md`. This Claude Code session is focused on the firmware only.
