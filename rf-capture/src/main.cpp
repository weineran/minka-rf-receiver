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

// ---- RSSI monitoring ----
#define RSSI_INTERVAL_MS 500   // Print RSSI every 500ms
unsigned long lastRssiPrint = 0;
bool rssiMode = true;          // Set false to suppress RSSI spam
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
// into a '0'/'1' string. Returns the bit count, or -1 if the frame is corrupt.
// Durations only; the HIGH/LOW parity labels are ignored (they become unreliable
// once a glitch has shifted the edge parity).
int decodeFrame(const unsigned long *p, int start, int end,
                unsigned long bitThr, char *outBits) {
    int len = end - start;
    if (len <= 0 || (len % 2) != 0) { return -1; }   // must be whole (ON,OFF) cells
    int bits = len / 2;
    if (bits > MAX_FRAME_BITS) { return -1; }

    int b = 0;
    for (int i = start; i + 1 < end; i += 2) {
        unsigned long on  = p[i];
        unsigned long off = p[i + 1];
        if (on < bitThr && off > bitThr) {
            outBits[b++] = '0';               // short-ON, long-OFF
        } else if (on > bitThr && off < bitThr) {
            outBits[b++] = '1';               // long-ON, short-OFF
        } else {
            return -1;                        // both short or both long -> corrupt
        }
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
                int nbits = decodeFrame(lastCaptured, prevGap + 1, i, bitThr, bits);
                if (nbits < 0) {
                    Serial.printf("  frame %d: CORRUPT (%d pulses)\n", frameNo++, i - (prevGap + 1));
                } else {
                    Serial.printf("  frame %d: %s (%d bits)\n", frameNo++, bits, nbits);
                }
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
    Serial.println("  RX BW     : 650 kHz (wide)");
    Serial.println("  Data rate : 1.0 kbps");
    Serial.println("  Data pin  : GDO0 -> D26");
    Serial.println("===========================================");
    Serial.println();
    Serial.println("RSSI floor prints every 500ms while idle.");
    Serial.printf("Noise squelch: bursts peaking below %d dBm are ignored.\n", CARRIER_THRESHOLD_DBM);
    Serial.println("A real button press should read well above the noise floor.");
    Serial.println("Serial: 'q' toggles diagnostics, 'r' dumps raw pulses of the last capture.");
    Serial.println();

    ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
    ELECHOUSE_cc1101.Init();

    ELECHOUSE_cc1101.setCCMode(0);       // Raw mode
    ELECHOUSE_cc1101.setModulation(2);   // ASK/OOK
    ELECHOUSE_cc1101.setMHZ(FREQ_MHZ);
    ELECHOUSE_cc1101.setDRate(1.0);      // Lower data rate for wider pulse acceptance
    ELECHOUSE_cc1101.setRxBW(650.0);     // Much wider bandwidth
    ELECHOUSE_cc1101.setSyncMode(0);     // No sync word
    ELECHOUSE_cc1101.setPktFormat(3);    // Async serial on GDO0

    // Increase sensitivity via AGC settings
    // AGCCTRL2: max LNA gain, target amplitude 33dB
    ELECHOUSE_cc1101.SpiWriteReg(0x1B, 0x03);  // AGCCTRL2: max gain
    // AGCCTRL1: relative carrier sense disabled
    ELECHOUSE_cc1101.SpiWriteReg(0x1C, 0x00);  // AGCCTRL1
    // AGCCTRL0: medium hysteresis, 16 samples
    ELECHOUSE_cc1101.SpiWriteReg(0x1D, 0x91);  // AGCCTRL0

    ELECHOUSE_cc1101.SetRx(FREQ_MHZ);

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

    // ---- Serial commands: 'q' toggle diagnostics, 'r' raw dump ----
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'q' || c == 'Q') {
            rssiMode = !rssiMode;
            Serial.printf("[RSSI monitoring %s]\n", rssiMode ? "ON" : "OFF");
        } else if (c == 'r' || c == 'R') {
            dumpLastRaw();
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
    unsigned long bitThr = deriveBitThreshold(captured, count, gapThr);

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

    // Decode each gap-bracketed frame; collect the clean bit-strings.
    char frames[MAX_FRAMES][MAX_FRAME_BITS + 1];
    int  frameCount = 0;
    int  seen = 0, dropped = 0;
    for (int g = 0; g + 1 < nGaps; g++) {
        int start = gapIdx[g] + 1;
        int end   = gapIdx[g + 1];
        if (end <= start) { continue; }
        seen++;
        char bits[MAX_FRAME_BITS + 1];
        int nbits = decodeFrame(captured, start, end, bitThr, bits);
        if (nbits < 0) {
            dropped++;
            continue;
        }
        if (frameCount < MAX_FRAMES) {
            strcpy(frames[frameCount++], bits);
        }
    }

    if (frameCount == 0) {
        Serial.printf("Frames    : %d seen, 0 clean, %d dropped (glitch)\n", seen, dropped);
        Serial.println("All frames corrupt (check the encoding assumption). Press 'r' for raw pulses.");
        Serial.println("=========================");
        Serial.println();
        Serial.println("Listening... press another button.");
        Serial.println();
        return;
    }

    // Majority vote: the most common identical bit-string wins.
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

    Serial.printf("Frames    : %d seen, %d clean, %d dropped (glitch)\n",
                  seen, frameCount, dropped);
    Serial.printf("Fingerprint: %s  (%d bits)  hex 0x%s\n",
                  frames[bestIdx], (int)strlen(frames[bestIdx]), hex);
    if (bestVotes == frameCount) {
        Serial.printf("Agreement : %d/%d clean frames identical  [OK]\n", bestVotes, frameCount);
    } else {
        Serial.printf("Agreement : %d/%d clean frames agree  [weak: rolling code or wrong encoding?]\n",
                      bestVotes, frameCount);
    }
    Serial.println("(press 'r' to dump raw pulses for this capture)");
    Serial.println("=========================");
    Serial.println();
    Serial.println("Listening... press another button.");
    Serial.println();
}
