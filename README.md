# MinkaAire RF Light Control

DIY ESP32 + CC1101 firmware that replaces a defective MinkaAire wall controller's
light button. It listens for the ceiling-fan remote's RF signal, debounces it, and
drives a relay.

## The problem
- The MinkaAire WCS213-S wall controller has a firmware debounce defect: the light
  button fires its toggle command several times per press, so the light flashes or
  does not respond.
- This project listens for the same 303.875 MHz remote signal, applies proper
  debounce, and toggles an SPDT relay wired as one half of a 3-way switch pair
  (the other half is a wall dimmer).

## Repo layout
This repo is a monorepo of separate PlatformIO projects:

- `rf-capture/`: generic tool to capture and fingerprint OOK remote signals. Used
  to reverse-engineer the remote's light and fan codes. See its own README.
- `minka-light-receiver/` (planned): the deployed firmware. Matches the light
  codes, ignores the fan codes, debounces, and toggles the relay.

## Hardware (summary)
- ESP32 (ESP-WROOM-32 DevKitC) + CC1101 radio module, wired over SPI.
- Frequency 303.875 MHz, OOK/ASK modulation.
- SPDT relay (later phase), wired as a 3-way companion to a wall dimmer.
- Full wiring, component list, and install notes: see
  [minka-rf-capture-handoff.md](minka-rf-capture-handoff.md).

## Status
- Phase 1 (signal capture & decode): in progress.
- Phase 2 (relay toggle firmware): planned.
- Phase 3 (production, deploy to 5-6 rooms): planned.

## Deployment note
- Each room's remote has its own pairing code, so each room gets its own
  ESP32 + CC1101 + relay running the same firmware with a different code to match.
