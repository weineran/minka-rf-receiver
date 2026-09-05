# Frame-Segmentation Decoder Implementation Plan

> **For agentic workers:** implement task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the unreliable whole-blob bit decoder in `rf-capture` with an adaptive frame-segmentation decoder that prints one stable, comparable per-button fingerprint (bits + hex) with a confidence check.

**Architecture:** On each accepted capture, adaptively find the inter-frame gap threshold from the pulse data (no hardcoded gap), split the burst into frames on those gaps, decode each fully-bracketed frame to bits using an adaptive short/long threshold, discard corrupt frames, then majority-vote across the survivors. Print a compact result by default; keep the raw pulse dump behind an `r` serial toggle.

**Tech Stack:** ESP32 (Arduino framework), CC1101 via SmartRC-CC1101-Driver-Lib, PlatformIO. Single file: `rf-capture/src/main.cpp`.

## Global Constraints

- Stay generic: no hardcoded device-specific constants (no fixed 11.9 ms gap, no fixed 500 us bit boundary). All thresholds derived from the capture or exposed as tunable `#define`s.
- Keep it simple (owner's rule: as simple as possible, but no simpler). No new libraries.
- Modulation stays OOK/ASK; frequency stays an editable `#define`, not a runtime command.
- Serial output must stay readable while capturing 6-7 buttons back to back (compact by default).
- Verification is compile (`pio run`) + hardware acceptance, not unit tests.
- Preserve the existing RSSI noise squelch (commit 8cadb67) unchanged.

---

## Signal facts (from real captures, for reference)

- Preamble: ~9 cycles of a ~500 us square wave, then the frames.
- Frames repeat ~15 times per press, separated by ~11.9 ms dead gaps.
- Bit cell = one ON pulse + one OFF pulse, fixed ~1 ms period, PWM-style:
  short-ON + long-OFF = `0`, long-ON + short-OFF = `1`. ~13 cells per frame.
- Glitches: occasional ~120-130 us slivers that split a real pulse and shift
  the ON/OFF pairing for the rest of that frame. These corrupt individual
  frames but fall at random spots, so voting across repeats recovers the truth.
- The current `HIGH`/`LOW` labels are guessed from edge parity, NOT measured, so
  the decoder must ignore them and work from durations plus gap-anchored ON phase
  (the first pulse after a gap is always an ON pulse).

---

## File structure

Single file, `rf-capture/src/main.cpp`. New tunables near the existing capture
settings. New helper functions above `loop()`. The capture-harvest block inside
`loop()` (currently the raw dump + blob decode) is replaced by: store raw ->
segment -> decode -> vote -> print compact. A global `lastCaptured[]` buffer
plus `r` toggle serve the on-demand raw dump.

---

## Task 1: Adaptive thresholds + frame segmentation helpers

**Files:**
- Modify: `rf-capture/src/main.cpp` (add `#define`s and helper functions)

**Interfaces produced (used by later tasks):**
- `unsigned long deriveGapThreshold(const unsigned long *p, int n)`
  - Returns the boundary above which a pulse is treated as an inter-frame gap.
- `unsigned long deriveBitThreshold(const unsigned long *p, int n, unsigned long gapThr)`
  - Returns the short/long divider for bit pulses (pulses below `gapThr`).

**Tunables to add (near other capture settings):**
```cpp
#define GAP_FACTOR      4      // gap = pulse longer than GAP_FACTOR x median bit pulse
#define GAP_MIN_US      2000   // floor so tiny-pulse captures do not misfire
#define MAX_FRAMES      32     // most frames we vote across
#define MAX_FRAME_BITS  40     // most bits we decode per frame
```

**Algorithm — gap threshold (adaptive, no hardcoded 11.9 ms):**
- Copy the pulses into a global scratch buffer (`static unsigned long sortBuf[MAX_PULSES]`,
  NOT a stack array — the loop already holds a 2400-byte `captured[]`, so avoid a
  second big stack allocation), sort ascending, take the median.
- `gapThr = max(GAP_MIN_US, GAP_FACTOR * median)`.
- Rationale: bit pulses dominate the count so the median sits among them; real
  inter-frame gaps are many times larger and land above `GAP_FACTOR * median`.

**Algorithm — bit threshold (adaptive short/long divider):**
- Average all pulses `< gapThr` (the bit pulses). With roughly equal numbers of
  short and long pulses, the mean falls between the two clusters, so it separates
  short from long. Return that mean.

- [ ] **Step 1: Add the tunable `#define`s** near `MIN_PULSES` / `CARRIER_THRESHOLD_DBM`.

- [ ] **Step 2: Implement `deriveGapThreshold`** (copy + insertion/`std::sort` on a local array, median, apply factor and floor).

- [ ] **Step 3: Implement `deriveBitThreshold`** (mean of pulses `< gapThr`; guard against zero bit pulses by returning `gapThr / 2`).

- [ ] **Step 4: Compile.** Run `pio run`. Expected: SUCCESS (functions unused so far, no behavior change).

---

## Task 2: Per-frame decode with corrupt-frame rejection

**Files:**
- Modify: `rf-capture/src/main.cpp`

**Interfaces produced:**
- `int decodeFrame(const unsigned long *p, int start, int end, unsigned long bitThr, char *outBits)`
  - Decodes pulses `[start, end)` (a frame, starting on an ON pulse) into a
    `'0'/'1'` string in `outBits` (NUL-terminated). Returns bit count on success,
    or `-1` if the frame is corrupt.

**Decode rules (durations only; ignore parity labels):**
- Frame length must be even; else return `-1`.
- Reject if it would exceed `MAX_FRAME_BITS`.
- For each cell `(on = p[i], off = p[i+1])`:
  - `on < bitThr && off > bitThr` -> append `'0'`
  - `on > bitThr && off < bitThr` -> append `'1'`
  - otherwise (both short or both long) -> corrupt, return `-1`.
- NUL-terminate and return the bit count.

- [ ] **Step 1: Implement `decodeFrame`** exactly as above.

- [ ] **Step 2: Compile.** Run `pio run`. Expected: SUCCESS.

---

## Task 3: Segment, vote, and print the compact fingerprint

**Files:**
- Modify: `rf-capture/src/main.cpp` (replace the capture-harvest dump/blob-decode block)

**Interfaces consumed:** `deriveGapThreshold`, `deriveBitThreshold`, `decodeFrame`.

**Interfaces produced:**
- `void bitsToHex(const char *bits, char *outHex)` — MSB-first, left-padded to nibbles.
- Global `unsigned long lastCaptured[MAX_PULSES]; int lastCount;` — snapshot of the
  most recent accepted capture, for the `r` raw dump (Task 4).

**Segmentation + voting flow (replaces the old raw table + blob `Bit stream` code, keeping the squelch and `Peak RSSI` line):**
- Save the accepted capture into `lastCaptured` / `lastCount`.
- `gapThr = deriveGapThreshold(...)`, `bitThr = deriveBitThreshold(...)`.
- Walk the pulses, recording gap indices (pulse `> gapThr`).
- For each pair of consecutive gaps, the pulses strictly between them are one
  gap-bracketed frame (guaranteed to start on an ON). Decode each with
  `decodeFrame`. Collect clean bit-strings (cap at `MAX_FRAMES`); count dropped.
- Tally identical clean strings (O(n^2) over <=32 frames). The most common string
  is the fingerprint; `agree = topCount`, `clean = survivors`.
- Print compact:
```
===== BUTTON SIGNAL =====
Peak RSSI : -31 dBm
Frames    : 14 seen, 11 clean, 3 dropped (glitch)
Fingerprint: 0100000001010  (13 bits)  hex 0x080A
Agreement : 11/11 clean frames identical  [OK]
(press 'r' to dump raw pulses for this capture)
=========================
```
- Edge cases (print a clear one-liner, no crash): fewer than 2 gaps (no bracketed
  frame) -> "no complete frame (need a repeating signal)"; zero clean frames ->
  "all frames corrupt (check encoding assumption)"; agreement `< clean` ->
  still print the winner but flag `[weak: X/Y]` so a rolling code or wrong
  encoding is visible.

- [ ] **Step 1: Add globals** `lastCaptured`, `lastCount`, and implement `bitsToHex`.

- [ ] **Step 2: Replace the harvest print block** (the raw `Index | Duration` table, the `shortSum/longSum` blob decode, and the histogram) with the segment/vote/compact-print flow above. Keep the squelch check and `Peak RSSI`.

- [ ] **Step 3: Compile.** Run `pio run`. Expected: SUCCESS.

---

## Task 4: `r` toggle for on-demand raw dump

**Files:**
- Modify: `rf-capture/src/main.cpp` (serial input handler in `loop()`)

**Interfaces consumed:** globals `lastCaptured`, `lastCount`, `deriveGapThreshold`, `deriveBitThreshold`, `decodeFrame`.

**Behavior:** extend the existing `Serial.read()` handler (currently only `q`):
- `r` / `R`: dump the last capture's raw pulses (`Index | Duration | inferred state`)
  and the per-frame decode (each frame's bit-string or `CORRUPT`). If `lastCount == 0`,
  print `no capture yet`.
- Keep `q` toggling `rssiMode` as-is.
- Update the banner line in `setup()` to mention `r` (raw dump) alongside `q`.

- [ ] **Step 1: Add the `r` branch** to the serial handler; implement the raw dump using the stored `lastCaptured`.

- [ ] **Step 2: Update the `setup()` banner** text to document `q` and `r`.

- [ ] **Step 3: Compile.** Run `pio run`. Expected: SUCCESS.

- [ ] **Step 4: Commit.**
```bash
git add rf-capture/src/main.cpp
git commit -m "Add adaptive frame-segmentation decoder to rf-capture"
```

---

## Task 5: Hardware acceptance (owner-run)

Not a code task. The real test, run by the owner on the bench:

- [ ] Flash: `pio run -t upload -t monitor` in `rf-capture/`.
- [ ] **Repeatability:** press the top light button 3 times. Expect the SAME
      fingerprint each time, with `Agreement` all-clean-identical. This is the
      correctness signal.
- [ ] **Capture table:** record the fingerprint for top-light, bottom-light,
      fan-1, fan-2, fan-3, fan-off.
- [ ] **Separation check:** the two light buttons vs the fan buttons must be
      clearly distinguishable (this is what Phase 2 will match/ignore on).
- [ ] If a real press shows `[weak]` or high drop count, use `r` to inspect raw
      frames and tune `GAP_FACTOR` / thresholds.

---

## Self-review notes

- Spec coverage: adaptive gap (Task 1), adaptive bit threshold (Task 1), decode +
  corrupt rejection (Task 2), discard-and-vote + compact output (Task 3), `r` raw
  toggle (Task 4), repeatability/confidence acceptance (Task 5). All design points
  covered.
- No device-specific constants: gap and bit thresholds are derived; `GAP_FACTOR`,
  `GAP_MIN_US`, `FREQ_MHZ`, `CARRIER_THRESHOLD_DBM` are tunable `#define`s.
- Type consistency: `deriveGapThreshold`/`deriveBitThreshold`/`decodeFrame`/
  `bitsToHex` signatures and the `lastCaptured`/`lastCount` globals are used
  consistently across Tasks 2-4.
