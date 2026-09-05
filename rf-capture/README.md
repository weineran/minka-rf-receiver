# rf-capture

Capture and fingerprint On-Off Keying (OOK/ASK) remote-control signals with an
ESP32 + CC1101.

> This tool began as a MinkaAire-specific capture sketch and is being generalized
> to work with any OOK remote.

## What is OOK?
- On-Off Keying: the radio carrier is switched fully on and off to send bits. It
  is the simplest form of ASK (Amplitude-Shift Keying), and it is what most cheap
  300-433 MHz remotes use.

## What it does
- Listens on a set frequency (default 303.875 MHz), OOK/ASK.
- RSSI noise squelch: ignores the ambient noise floor and captures only real
  bursts above a strength threshold (`CARRIER_THRESHOLD_DBM`).
- Records the pulse timing of each burst and prints it over serial (115200 baud).
- Prints peak RSSI per capture so you can calibrate the squelch.
- Serial toggle: press `q` to turn the RSSI/diagnostic output on or off.

Planned next:
- Segment the repeated frames, vote across the repeats, and print one stable
  per-button fingerprint (bits + hex) with a confidence check. Design in progress.

## Build and run
- Requires VS Code + PlatformIO (or the PlatformIO CLI).
- Build: `pio run`
- Flash and monitor: `pio run -t upload -t monitor`
- Serial monitor speed: 115200 baud (port auto-detected; COM5 on the owner's PC).

## How to use
- Power the board and open the serial monitor.
- While idle you see the RSSI floor print, and background noise gets squelched.
- Hold the remote close to the antenna and press a button.
- A real press captures and prints its pulse data and peak RSSI.
- To capture several buttons, press each in turn and label the output yourself.

## Prerequisites, manual steps, and constraints
The tool does not do these for you:

- **Hardware:** wire the ESP32 to the CC1101 per the project wiring table, use a
  USB data cable (not charge-only), and select the correct COM port.
- **Frequency:** it captures only the frequency the radio is set to. To capture a
  different frequency, edit `FREQ_MHZ` in `src/main.cpp` and reflash. Confirm your
  antenna suits that band.
- **Antenna and range:** the module antenna is tuned for 433 MHz. At 303 MHz
  sensitivity is reduced, so hold the remote close to the antenna.
- **Modulation:** OOK/ASK only. The radio is configured for OOK. FSK and other
  modulations are not received or decoded.
- **Encoding:** bit decoding assumes a 2-symbol short/long PWM scheme
  (PT2262 / EV1527 style, common in household remotes). Other encodings
  (Manchester, PPM, variable-length) still capture as raw pulses, but the 0/1
  interpretation may be wrong.
- **Repetition:** the confidence check needs the signal to repeat its frame (most
  remotes send several repeats per press). A single, non-repeating burst is
  captured as one frame with no repeat verification.
- **Squelch tuning:** the default `CARRIER_THRESHOLD_DBM` (-52 dBm) suits the
  tested setup. If real presses are missed, lower it; if noise still gets through,
  raise it. Use the printed peak RSSI values to choose.

## Wiring
- See the parent repo's [handoff doc](../minka-rf-capture-handoff.md) for the full
  CC1101-to-ESP32 pin table and hardware notes.
