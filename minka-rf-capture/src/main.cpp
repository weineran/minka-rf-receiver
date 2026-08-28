/*
 * MinkaAire RF Signal Capture
 * 
 * Listens at 303.875 MHz using a CC1101 module in OOK/ASK async mode.
 * Records the timing of signal transitions (pulse durations) and dumps
 * them to the serial monitor. Use this to decode the MinkaAire remote's
 * light toggle command.
 *
 * Hardware:
 *   CC1101 pin 1 (GND)  -> ESP32 GND
 *   CC1101 pin 2 (VCC)  -> ESP32 3V3 (NOT 5V!)
 *   CC1101 pin 3 (GDO0) -> ESP32 D26
 *   CC1101 pin 4 (CSN)  -> ESP32 D5
 *   CC1101 pin 5 (SCK)  -> ESP32 D18
 *   CC1101 pin 6 (MOSI) -> ESP32 D23
 *   CC1101 pin 7 (MISO) -> ESP32 D19
 *   CC1101 pin 8 (GDO2) -> ESP32 D27
 *
 * Usage:
 *   1. Flash this sketch and open the serial monitor at 115200 baud
 *   2. Press the MinkaAire remote's LIGHT button
 *   3. You should see captured pulse timings appear
 *   4. Press the button several times to confirm the pattern repeats
 *   5. Also try pressing FAN buttons to see how they differ
 */

#include <Arduino.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// ---- Pin assignments (match your wiring) ----
#define PIN_GDO0  26   // CC1101 pin 3 -> demodulated data output
#define PIN_GDO2  27   // CC1101 pin 8 -> carrier sense / status
#define PIN_CSN    5   // CC1101 pin 4 -> SPI chip select
#define PIN_SCK   18   // CC1101 pin 5 -> SPI clock
#define PIN_MOSI  23   // CC1101 pin 6 -> SPI data to CC1101
#define PIN_MISO  19   // CC1101 pin 7 -> SPI data from CC1101

// ---- Capture settings ----
#define FREQ_MHZ       303.875  // MinkaAire operating frequency
#define MAX_PULSES     600      // Max transitions to record per burst
#define GAP_TIMEOUT_US 15000    // Microseconds of silence = end of signal
#define MIN_PULSES     20       // Ignore bursts shorter than this (noise)

// ---- Pulse buffer (filled by interrupt) ----
volatile unsigned long pulseTimes[MAX_PULSES];
volatile int           pulseIndex = 0;
volatile unsigned long lastEdgeUs = 0;
volatile bool          capturing  = false;

// Interrupt fires on every rising AND falling edge of the demodulated signal
void IRAM_ATTR onEdge() {
    unsigned long now = micros();

    if (lastEdgeUs > 0 && pulseIndex < MAX_PULSES) {
        pulseTimes[pulseIndex++] = now - lastEdgeUs;
    }

    lastEdgeUs = now;
    capturing  = true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);  // Let serial settle

    Serial.println();
    Serial.println("===========================================");
    Serial.println("  MinkaAire RF Signal Capture");
    Serial.println("===========================================");
    Serial.printf("  Frequency : %.3f MHz\n", FREQ_MHZ);
    Serial.println("  Modulation: OOK / ASK");
    Serial.println("  Data pin  : GDO0 -> D26");
    Serial.println("===========================================");

    // Tell the library which SPI pins to use
    ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);

    // Initialize the CC1101
    ELECHOUSE_cc1101.Init();

    // Configure for raw / async OOK reception
    ELECHOUSE_cc1101.setCCMode(0);       // 0 = raw (no packet handling)
    ELECHOUSE_cc1101.setModulation(2);   // 2 = ASK/OOK
    ELECHOUSE_cc1101.setMHZ(FREQ_MHZ);
    ELECHOUSE_cc1101.setDRate(2.048);    // Low data rate for wide capture
    ELECHOUSE_cc1101.setRxBW(135.0);     // Receiver bandwidth in kHz
    ELECHOUSE_cc1101.setSyncMode(0);     // No sync word (capture everything)
    ELECHOUSE_cc1101.setPktFormat(3);    // Async serial mode on GDO0

    // Enter receive mode
    ELECHOUSE_cc1101.SetRx(FREQ_MHZ);

    // Attach interrupt on GDO0 for both rising and falling edges
    pinMode(PIN_GDO0, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GDO0), onEdge, CHANGE);

    Serial.println();
    Serial.println("Listening... press a remote button now.");
    Serial.println();
}

void loop() {
    // Nothing to do until we are capturing
    if (!capturing) return;

    // Wait for the signal to end (gap with no transitions)
    unsigned long elapsed = micros() - lastEdgeUs;
    if (elapsed < GAP_TIMEOUT_US) return;

    // ---- Signal complete, harvest the buffer ----
    noInterrupts();
    int count = pulseIndex;
    unsigned long captured[MAX_PULSES];
    memcpy(captured, (const void *)pulseTimes, count * sizeof(unsigned long));
    pulseIndex  = 0;
    lastEdgeUs  = 0;
    capturing   = false;
    interrupts();

    // Ignore short bursts (noise / glitches)
    if (count < MIN_PULSES) return;

    // ---- Print the capture ----
    Serial.println("===== SIGNAL CAPTURED =====");
    Serial.printf("Pulses: %d\n", count);
    Serial.println();

    // Raw pulse list (index, duration, state)
    Serial.println("Index | Duration (us) | State");
    Serial.println("------|---------------|------");
    for (int i = 0; i < count; i++) {
        Serial.printf("%5d | %13lu | %s\n",
                      i, captured[i],
                      (i % 2 == 0) ? "HIGH" : "LOW ");
    }

    // Summary: group similar pulse durations to help identify bit encoding
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
