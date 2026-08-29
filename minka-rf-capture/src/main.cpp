/*
 * MinkaAire RF Signal Capture - v2.1 (wider sensitivity + noise squelch)
 *
 * Changes from v1:
 *   - Much wider RX bandwidth (650 kHz vs 135 kHz)
 *   - Lower data rate (1.0 kbps) for broader pulse capture
 *   - RSSI monitoring: prints signal strength every 500ms so you can see
 *     if the light buttons produce ANY RF energy at 303.875 MHz
 *   - Lower MIN_PULSES threshold (8 vs 20) to catch shorter frames
 *   - Added raw bit stream output for easier protocol decoding
 *
 * Changes in v2.1:
 *   - Added an RSSI noise squelch. The wide-band, max-gain OOK front end
 *     slices ambient RF noise into edges whenever no real carrier is present,
 *     which flooded the log with fake "captures". We now track the peak signal
 *     strength during each burst and discard any burst that never rises above
 *     CARRIER_THRESHOLD_DBM. Peak RSSI is printed per capture so the threshold
 *     can be calibrated against a real remote press.
 */

#include <Arduino.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

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
#define GAP_TIMEOUT_US 15000
#define MIN_PULSES     8       // Lowered from 20 to catch shorter frames

// Noise squelch: reject any burst whose peak signal strength never rises above
// this level (dBm). The OOK slicer chops ambient noise into edges when there is
// no real carrier; a genuine remote press reads well above the noise floor.
// TUNE THIS: compare the "[noise ignored]" peak values (the floor) against the
// "Peak RSSI" of a real button press, then set this between the two.
#define CARRIER_THRESHOLD_DBM -52

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

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("===========================================");
    Serial.println("  MinkaAire RF Signal Capture v2");
    Serial.println("  (wider bandwidth, higher sensitivity)");
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
    Serial.println("Type 'q' in serial to toggle RSSI/diagnostic output.");
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

    // ---- Toggle RSSI output if user sends 'q' ----
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'q' || c == 'Q') {
            rssiMode = !rssiMode;
            Serial.printf("[RSSI monitoring %s]\n", rssiMode ? "ON" : "OFF");
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

    // ---- Print capture ----
    Serial.println("===== SIGNAL CAPTURED =====");
    Serial.printf("Pulses: %d\n", count);
    Serial.printf("Peak RSSI: %d dBm\n", peakRssi);
    Serial.println();

    // Raw pulse list
    Serial.println("Index | Duration (us) | State");
    Serial.println("------|---------------|------");
    for (int i = 0; i < count; i++) {
        Serial.printf("%5d | %13lu | %s\n",
                      i, captured[i],
                      (i % 2 == 0) ? "HIGH" : "LOW ");
    }

    // Bit stream interpretation
    // Determine threshold between "short" and "long" pulses
    // by finding the midpoint of the two clusters
    unsigned long shortSum = 0, longSum = 0;
    int shortCount = 0, longCount = 0;
    for (int i = 0; i < count; i++) {
        if (captured[i] < 500) {
            shortSum += captured[i];
            shortCount++;
        } else if (captured[i] < 1000) {
            longSum += captured[i];
            longCount++;
        }
    }

    if (shortCount > 0 && longCount > 0) {
        unsigned long shortAvg = shortSum / shortCount;
        unsigned long longAvg = longSum / longCount;
        unsigned long threshold = (shortAvg + longAvg) / 2;

        Serial.println();
        Serial.printf("Short pulse avg: %lu us (%d pulses)\n", shortAvg, shortCount);
        Serial.printf("Long pulse avg:  %lu us (%d pulses)\n", longAvg, longCount);
        Serial.printf("Threshold:       %lu us\n", threshold);
        Serial.println();

        // Decode: look at pairs of (HIGH, LOW) durations
        // Short-Long = "0", Long-Short = "1" (common OOK convention)
        Serial.print("Bit stream: ");
        for (int i = 0; i < count - 1; i += 2) {
            unsigned long high = captured[i];
            unsigned long low  = captured[i + 1];

            // Skip gaps (>5000 us = frame separator)
            if (high > 5000 || low > 5000) {
                Serial.print(" | ");
                continue;
            }

            if (high < threshold && low > threshold) {
                Serial.print("0");
            } else if (high > threshold && low < threshold) {
                Serial.print("1");
            } else if (high < threshold && low < threshold) {
                Serial.print("s");  // both short (unusual)
            } else {
                Serial.print("L");  // both long (unusual)
            }
        }
        Serial.println();
    }

    // Histogram
    Serial.println();
    Serial.println("Pulse duration histogram:");
    int buckets[10] = {0};
    unsigned long limits[] = {100, 200, 400, 600, 800, 1200, 2000, 4000, 8000};
    for (int i = 0; i < count; i++) {
        int b = 9;
        for (int j = 0; j < 9; j++) {
            if (captured[i] < limits[j]) { b = j; break; }
        }
        buckets[b]++;
    }
    const char* labels[] = {
        "   0-100 us", " 100-200 us", " 200-400 us", " 400-600 us",
        " 600-800 us", " 800-1200us", "1200-2000us", "2000-4000us",
        "4000-8000us", "   8000+ us"
    };
    for (int i = 0; i < 10; i++) {
        if (buckets[i] > 0) {
            Serial.printf("  %s : %d pulses\n", labels[i], buckets[i]);
        }
    }

    Serial.println("===== END =====");
    Serial.println();
    Serial.println("Listening... press another button.");
    Serial.println();
}
