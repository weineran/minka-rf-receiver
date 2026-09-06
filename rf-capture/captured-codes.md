# Captured Button Codes (MinkaAire DL-4111T-01 remote)

Captured with `rf-capture` at 303.875 MHz, OOK/ASK, RX BW 650 kHz, AGC gain
`AGCCTRL2 = 0x13`. Each code is the ~12-bit PWM frame the remote repeats per
press (RC-Switch "protocol 6" style, ~350/700 us pulses).

| Button | Code (12-bit) | Hex | Command nibble |
|---|---|---|---|
| Fan 1 | `000000000011` | 0x003 | `0011` |
| Fan 2 | `000000000110` | 0x006 | `0110` |
| Fan 3 | `000000001100` | 0x00C | `1100` (inferred from pattern + noisy frames) |
| Fan off | `000000001111` | 0x00F | `1111` |
| Light (top) | `000000000111` | 0x007 | `0111` |
| Light (bottom) | `000000001101` | 0x00D | `1101` |

## Structure
- All codes share an 8-bit `00000000` prefix (common address/preamble); the last
  4 bits (the command nibble) identify the button.
- Fan speeds walk a "11" pattern left: `0011` / `0110` / `1100`; fan-off is `1111`.
- The two light buttons send DIFFERENT codes (`0x007` vs `0x00D`) even though both
  toggle the same light, so Phase 2 must match both.

## Cautions for Phase 2 (relay toggle)
- Some codes differ by a single bit: light-top `0111` vs fan-1 `0011`, and
  light-bottom `1101` vs fan-off `1111`. A noisy fan press could bit-flip into a
  light code, so detection must be robust (require the code across multiple
  frames / repeats, not a single capture).
- Single-press decoding is marginal: the pulse contrast drifts over each burst,
  so only ~2-4 of the ~6 repeats per press decode correctly. The correct code
  wins the plurality (mode) across a press, but individual captures occasionally
  read wrong. Reduced AGC gain (`0x13`) was needed to make the mode stable across
  presses; a proper 303 MHz antenna would likely firm this up further.
- These 12 bits distinguish buttons on THIS remote. Whether they also carry the
  room-specific pairing code (needed to ignore other rooms' remotes) is not yet
  confirmed; the all-zero prefix may be an address that differs between remotes.
