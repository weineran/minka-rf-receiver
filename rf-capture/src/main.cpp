/*
 * rf-capture: OOK signal capture and fingerprinting - v2.2
 *
 * Captures On-Off Keying (OOK/ASK) remote-control signals with an ESP32 + CC1101,
 * squelches ambient noise, and prints a stable per-button "fingerprint".
 *
 * Changes from v1:
 *   - Much wider RX bandwidth (650 kHz vs 135 kHz)
 *   - Lower data rate (1.0 kbps) for broader pulse capture
 *   - RSSI monitoring: prints signal strength every 500ms
 *   - Lower MIN_PULSES threshold (8 vs 20) to catch shorter frames
 *
 * Changes in v2.1:
 *   - Added an RSSI noise squelch. The wide-band, max-gain OOK front end slices
 *     ambient RF noise into edges whenever no real carrier is present, which
 *     flooded the log with fake "captures". We track the peak signal strength
 *     during each burst and discard any burst that never rises above
 *     CARRIER_THRESHOLD_DBM. Peak RSSI is printed per capture for calibration.
 *
 * Changes in v2.2:
 *   - Replaced the whole-blob bit decoder with an adaptive frame-segmentation
 *     decoder: it splits each burst into frames on adaptively-detected
 *     inter-frame gaps (no hardcoded gap length), decodes each gap-anchored
 *     frame to bits, discards glitch-corrupted frames, and majority-votes across
 *     the repeats to print one stable fingerprint (bits + hex) with a confidence
 *     line. The raw pulse dump moved behind the 'r' serial toggle.
 */

#include <Arduino.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <stdlib.h>   // qsort
#include <string.h>   // memcpy, strcmp, strcpy, strlen

// ---- Pin assignments ----
#define PIN_GDO0  26
#define PIN_GDO2  27
#define PIN_CSN    5
#define PIN_SCK   18
#define PIN_MOSI  23
#define PIN_MISO  19

// ---- Capture settings ----
#define FREQ_MHZ       303.875
#define MAX_PULSES     600
#define GAP_TIMEOUT_US 15000   // No edges for this long => the burst is over
#define MIN_PULSES     8       // Lowered from 20 to catch shorter frames

// Noise squelch: reject any burst whose peak signal strength never rises above
// this level (dBm). The OOK slicer chops ambient noise into edges when there is
// no real carrier; a genuine remote press reads well above the noise floor.
// TUNE THIS: compare the "[noise ignored]" peak values (the floor) against the
// "Peak RSSI" of a real button press, then set this between the two.
#define CARRIER_THRESHOLD_DBM -52

// ---- Frame-segmentation decoder settings ----
// The frame gap is derived from the signal, not hardcoded: any pulse longer than
// GAP_FACTOR x the median pulse (floored at GAP_MIN_US) is treated as the dead
// gap between repeated frames.
#define GAP_FACTOR      4
#define GAP_MIN_US      2000
#define MAX_FRAMES      32     // most repeated frames we vote across
#define MAX_FRAME_BITS  40     // most bit-cells we decode per frame
#define MIN_FRAME_BITS  8      // ignore shorter decodes (noise fragments, not real frames)

// ---- RSSI monitoring ----
#define RSSI_INTERVAL_MS 500   // Print RSSI every 500ms
unsigned long lastRssiPrint = 0;
bool rssiMode = false;         // Diagnostics off by default; 'q' toggles the RSSI floor + [noise ignored] lines
int  captureRssiPeak = -200;   // Peak RSSI (dBm) seen during the current burst

// ---- Pulse buffer ----
volatile unsigned long pulseTimes[MAX_PULSES];
volatile int           pulseIndex = 0;
volatile unsigned long lastEdgeUs = 0;
volatile bool          capturing  = false;

// ---- Last accepted capture (snapshot for the 'r' raw dump) ----
unsigned long lastCaptured[MAX_PULSES];
int           lastCount = 0;

// Scratch buffer for the median sort. Kept off the stack on purpose: loop()
// already holds a MAX_PULSES stack array, so a second one there risks overflow.
static unsigned long sortBuf[MAX_PULSES];

// ---- Live-tunable front end (adjust over serial: + / - / g) ----
// RX bandwidth steps (kHz), matching the CC1101's discrete filter settings.
const float RXBW_STEPS[] = {58, 68, 81, 102, 116, 135, 162, 203,
                            232, 270, 325, 406, 464, 541, 650, 812};
const int   RXBW_COUNT   = sizeof(RXBW_STEPS) / sizeof(RXBW_STEPS[0]);
int rxbwIndex = 14;   // start at 650 kHz (the v2 wide setting)

// AGCCTRL2 values from most to least LNA gain. Less gain = less noise slicing.
const uint8_t AGC_STEPS[] = {0x03, 0x0B, 0x13, 0x1B};
const int     AGC_COUNT   = sizeof(AGC_STEPS) / sizeof(AGC_STEPS[0]);
int agcIndex = 2;     // start at reduced LNA gain (0x13): steadier OOK slicing, less AGC pumping

void IRAM_ATTR onEdge() {
    unsigned long now = micros();
    if (lastEdgeUs > 0 && pulseIndex < MAX_PULSES) {
        pulseTimes[pulseIndex++] = now - lastEdgeUs;
    }
    lastEdgeUs = now;
    capturing  = true;
}

int readRSSI() {
    int rssi = ELECHOUSE_cc1101.getRssi();
    return rssi;
}

// Apply the current RX bandwidth and AGC gain, then re-enter RX. Called at boot
// and whenever the user steps the front end over serial.
void applyFrontEnd() {
    ELECHOUSE_cc1101.setRxBW(RXBW_STEPS[rxbwIndex]);
    ELECHOUSE_cc1101.SpiWriteReg(0x1B, AGC_STEPS[agcIndex]);  // AGCCTRL2 (LNA gain)
    ELECHOUSE_cc1101.SetRx(FREQ_MHZ);
    Serial.printf("[front-end] RX BW = %.0f kHz, gain(AGCCTRL2) = 0x%02X\n",
                  RXBW_STEPS[rxbwIndex], AGC_STEPS[agcIndex]);
}

// Ascending compare for qsort over unsigned long.
static int cmpUL(const void *a, const void *b) {
    unsigned long x = *(const unsigned long *)a;
    unsigned long y = *(const unsigned long *)b;
    if (x < y) { return -1; }
    if (x > y) { return 1; }
    return 0;
}

// Derive the inter-frame gap threshold from the capture itself, so we never
// hardcode a device-specific gap. Bit pulses dominate the count, so the median
// sits among them; real inter-frame gaps are many times larger.
unsigned long deriveGapThreshold(const unsigned long *p, int n) {
    if (n <= 0) { return GAP_MIN_US; }
    memcpy(sortBuf, p, n * sizeof(unsigned long));
    qsort(sortBuf, n, sizeof(unsigned long), cmpUL);
    unsigned long median = sortBuf[n / 2];
    unsigned long thr = GAP_FACTOR * median;
    return (thr < GAP_MIN_US) ? GAP_MIN_US : thr;
}

// Derive the short/long divider for bit pulses (those below the gap threshold).
// With roughly equal numbers of short and long pulses, their mean falls between
// the two clusters and separates them.
unsigned long deriveBitThreshold(const unsigned long *p, int n, unsigned long gapThr) {
    unsigned long sum = 0;
    int cnt = 0;
    for (int i = 0; i < n; i++) {
        if (p[i] < gapThr) {
            sum += p[i];
            cnt++;
        }
    }
    if (cnt == 0) { return gapThr / 2; }
    return sum / cnt;
}

// Decode one gap-anchored frame (pulses [start, end), starting on an ON pulse)
// into a '0'/'1' string. One bit per (ON, OFF) cell, decided by relative
// duration: long-ON/short-OFF -> '1', short-ON/long-OFF -> '0'. Relative compare
// means no cell is ever "ambiguous", so a low-contrast or glitchy frame still
// decodes (its bits just may be wrong, which the per-frame listing reveals).
//
// A frame is inherently an ODD pulse count: the last bit's OFF is the long
// inter-frame gap (stripped during segmentation), leaving a trailing lone ON.
// We decode only whole cells and ignore that trailing pulse.
// Durations only; the HIGH/LOW parity labels are ignored (unreliable once a
// glitch has shifted the edge parity). Returns the bit count.
int decodeFrame(const unsigned long *p, int start, int end, char *outBits) {
    int b = 0;
    for (int i = start; i + 1 < end && b < MAX_FRAME_BITS; i += 2) {
        unsigned long on  = p[i];
        unsigned long off = p[i + 1];
        outBits[b++] = (on > off) ? '1' : '0';
    }
    outBits[b] = '\0';
    return b;
}

// Convert an MSB-first bit string to hex (left-padded to whole nibbles).
void bitsToHex(const char *bits, char *outHex) {
    int n = (int)strlen(bits);
    int pad = (4 - (n % 4)) % 4;              // virtual leading zeros
    int total = n + pad;
    int oi = 0;
    const char *digits = "0123456789ABCDEF";
    for (int nib = 0; nib < total; nib += 4) {
        int val = 0;
        for (int k = 0; k < 4; k++) {
            int idx = nib + k - pad;
            int bit = (idx < 0) ? 0 : (bits[idx] - '0');
            val = (val << 1) | bit;
        }
        outHex[oi++] = digits[val];
    }
    outHex[oi] = '\0';
}

// Dump the most recent accepted capture: raw pulse table plus per-frame decode,
// for debugging when a fingerprint looks weak. Triggered by 'r' over serial.
void dumpLastRaw() {
    if (lastCount == 0) {
        Serial.println("[raw] no capture yet");
        return;
    }
    unsigned long gapThr = deriveGapThreshold(lastCaptured, lastCount);
    unsigned long bitThr = deriveBitThreshold(lastCaptured, lastCount, gapThr);

    Serial.println("----- RAW DUMP (last capture) -----");
    Serial.printf("Pulses: %d   gap > %lu us   short/long @ %lu us\n",
                  lastCount, gapThr, bitThr);
    Serial.println("Index | Duration (us) | note");
    Serial.println("------|---------------|-----");
    for (int i = 0; i < lastCount; i++) {
        const char *note = (lastCaptured[i] > gapThr) ? "GAP" : "";
        Serial.printf("%5d | %13lu | %s\n", i, lastCaptured[i], note);
    }

    Serial.println("Frames (between gaps):");
    int prevGap = -1;
    int frameNo = 0;
    bool anyFrame = false;
    for (int i = 0; i < lastCount; i++) {
        if (lastCaptured[i] > gapThr) {
            if (prevGap >= 0 && i > prevGap + 1) {
                char bits[MAX_FRAME_BITS + 1];
                int nbits = decodeFrame(lastCaptured, prevGap + 1, i, bits);
                Serial.printf("  frame %d: %s (%d bits, %d pulses)\n",
                              frameNo++, bits, nbits, i - (prevGap + 1));
                anyFrame = true;
            }
            prevGap = i;
        }
    }
    if (!anyFrame) {
        Serial.println("  (no bracketed frames -> need a repeating signal)");
    }
    Serial.println("-----------------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("===========================================");
    Serial.println("  rf-capture: OOK signal capture");
    Serial.println("  (wide bandwidth, high sensitivity)");
    Serial.println("===========================================");
    Serial.printf("  Frequency : %.3f MHz\n", FREQ_MHZ);
    Serial.println("  Modulation: OOK / ASK");
    Serial.println("  Data rate : 1.0 kbps");
    Serial.println("  Data pin  : GDO0 -> D26");
    Serial.println("===========================================");
    Serial.println();
    Serial.printf("Noise squelch: bursts peaking below %d dBm are ignored.\n", CARRIER_THRESHOLD_DBM);
    Serial.println("Diagnostics (RSSI floor + [noise ignored]) OFF by default; 'q' toggles them.");
    Serial.println("Serial keys: q=diagnostics  r=raw dump  +/-=RX bandwidth  g=cycle gain");
    Serial.println();

    ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
    ELECHOUSE_cc1101.Init();

    ELECHOUSE_cc1101.setCCMode(0);       // Raw mode
    ELECHOUSE_cc1101.setModulation(2);   // ASK/OOK
    ELECHOUSE_cc1101.setMHZ(FREQ_MHZ);
    ELECHOUSE_cc1101.setDRate(1.0);      // Lower data rate for wider pulse acceptance
    ELECHOUSE_cc1101.setSyncMode(0);     // No sync word
    ELECHOUSE_cc1101.setPktFormat(3);    // Async serial on GDO0

    // AGCCTRL1 (carrier sense) and AGCCTRL0 (hysteresis). The LNA gain (AGCCTRL2)
    // and the RX bandwidth are applied by applyFrontEnd() so they can be tuned
    // live over serial.
    ELECHOUSE_cc1101.SpiWriteReg(0x1C, 0x00);  // AGCCTRL1
    ELECHOUSE_cc1101.SpiWriteReg(0x1D, 0x91);  // AGCCTRL0

    applyFrontEnd();   // sets RX bandwidth + gain, then enters RX

    pinMode(PIN_GDO0, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GDO0), onEdge, CHANGE);

    Serial.println("Listening... press a remote button now.");
    Serial.println();
}

void loop() {
    // ---- Track peak signal strength during an active burst ----
    // Sample RSSI continuously while a burst is being captured so we know how
    // strong it actually got. This peak is what the noise squelch checks below.
    if (capturing) {
        int rssi = readRSSI();
        if (rssi > captureRssiPeak) {
            captureRssiPeak = rssi;
        }
    }

    // ---- RSSI floor monitoring (idle only) ----
    if (rssiMode && !capturing && (millis() - lastRssiPrint >= RSSI_INTERVAL_MS)) {
        Serial.printf("[RSSI] %d dBm\n", readRSSI());
        lastRssiPrint = millis();
    }

    // ---- Serial commands ----
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'q' || c == 'Q') {
            rssiMode = !rssiMode;
            Serial.printf("[RSSI monitoring %s]\n", rssiMode ? "ON" : "OFF");
        } else if (c == 'r' || c == 'R') {
            dumpLastRaw();
        } else if (c == '+') {
            if (rxbwIndex < RXBW_COUNT - 1) { rxbwIndex++; }
            applyFrontEnd();
        } else if (c == '-') {
            if (rxbwIndex > 0) { rxbwIndex--; }
            applyFrontEnd();
        } else if (c == 'g' || c == 'G') {
            agcIndex = (agcIndex + 1) % AGC_COUNT;
            applyFrontEnd();
        }
    }

    // ---- Signal capture ----
    if (!capturing) return;

    unsigned long elapsed = micros() - lastEdgeUs;
    if (elapsed < GAP_TIMEOUT_US) return;

    // Harvest buffer
    noInterrupts();
    int count = pulseIndex;
    unsigned long captured[MAX_PULSES];
    memcpy(captured, (const void *)pulseTimes, count * sizeof(unsigned long));
    pulseIndex  = 0;
    lastEdgeUs  = 0;
    capturing   = false;
    interrupts();

    if (count < MIN_PULSES) {
        captureRssiPeak = -200;   // reset for the next burst
        return;
    }

    // ---- Noise squelch ----
    // A burst that never rose above the carrier threshold is ambient noise the
    // OOK demod sliced into edges, not a real remote frame. Discard it. We print
    // a compact one-liner (when diagnostics are on) so the noise floor stays
    // visible for calibrating CARRIER_THRESHOLD_DBM.
    int peakRssi = captureRssiPeak;
    captureRssiPeak = -200;       // reset for the next burst
    if (peakRssi < CARRIER_THRESHOLD_DBM) {
        if (rssiMode) {
            Serial.printf("[noise ignored] %d pulses, peak RSSI %d dBm (below %d threshold)\n",
                          count, peakRssi, CARRIER_THRESHOLD_DBM);
        }
        return;
    }

    // ---- Decode: segment into frames, vote across repeats ----
    // Snapshot this capture for the on-demand raw dump ('r').
    memcpy(lastCaptured, captured, count * sizeof(unsigned long));
    lastCount = count;

    unsigned long gapThr = deriveGapThreshold(captured, count);

    // Find gap indices. Frames are the pulses strictly between two consecutive
    // gaps: those are guaranteed to start on an ON pulse (the first edge after a
    // dead gap), which fixes the bit alignment.
    int gapIdx[MAX_FRAMES + 2];
    int nGaps = 0;
    for (int i = 0; i < count && nGaps < MAX_FRAMES + 2; i++) {
        if (captured[i] > gapThr) {
            gapIdx[nGaps++] = i;
        }
    }

    Serial.println("===== BUTTON SIGNAL =====");
    Serial.printf("Peak RSSI : %d dBm\n", peakRssi);

    if (nGaps < 2) {
        Serial.println("No complete frame (need a repeating signal). Press 'r' for raw pulses.");
        Serial.println("=========================");
        Serial.println();
        Serial.println("Listening... press another button.");
        Serial.println();
        return;
    }

    // Decode each gap-bracketed frame, keeping only those long enough to be a
    // real frame (short decodes are noise fragments). The most common surviving
    // frame (mode) is the fingerprint: the true code wins the plurality even when
    // the drifting tail corrupts individual repeats.
    char frames[MAX_FRAMES][MAX_FRAME_BITS + 1];
    int  frameCount = 0;
    int  dropped = 0;
    for (int g = 0; g + 1 < nGaps && frameCount < MAX_FRAMES; g++) {
        int start = gapIdx[g] + 1;
        int end   = gapIdx[g + 1];
        if (end <= start) { continue; }
        char bits[MAX_FRAME_BITS + 1];
        int nbits = decodeFrame(captured, start, end, bits);
        if (nbits < MIN_FRAME_BITS) { dropped++; continue; }
        strcpy(frames[frameCount++], bits);
    }

    if (frameCount == 0) {
        Serial.printf("Frames    : 0 usable (%d noise fragments dropped). Press 'r' for raw.\n", dropped);
        Serial.println("=========================");
        Serial.println();
        Serial.println("Listening... press another button.");
        Serial.println();
        return;
    }

    Serial.printf("Frames    : %d usable", frameCount);
    if (dropped > 0) { Serial.printf(" (+%d noise dropped)", dropped); }
    Serial.println();
    for (int i = 0; i < frameCount; i++) {
        Serial.printf("  %s\n", frames[i]);
    }

    // Most common exact frame (mode) wins.
    int bestIdx = 0, bestVotes = 0;
    for (int i = 0; i < frameCount; i++) {
        int votes = 0;
        for (int j = 0; j < frameCount; j++) {
            if (strcmp(frames[i], frames[j]) == 0) { votes++; }
        }
        if (votes > bestVotes) { bestVotes = votes; bestIdx = i; }
    }
    char hex[MAX_FRAME_BITS / 4 + 2];
    bitsToHex(frames[bestIdx], hex);
    Serial.printf("Best      : %s  hex 0x%s  (%d/%d match)\n",
                  frames[bestIdx], hex, bestVotes, frameCount);
    Serial.println("(keys: r=raw dump  +/-=bandwidth  g=gain)");
    Serial.println("=========================");
    Serial.println();
    Serial.println("Listening... press another button.");
    Serial.println();
}
