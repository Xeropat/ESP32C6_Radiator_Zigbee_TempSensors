/**
 * ESP32-C6 battery-powered hydronic radiator temperature sensor (v2)
 * ------------------------------------------------------------------
 * Reads two NTC thermistors (radiator inlet/flow and outlet/return)
 * and reports both as Zigbee Temperature Measurement endpoints.
 * Deep-sleeps between readings to save battery.
 *
 * REQUIREMENTS (Arduino IDE):
 *  - Board package: "esp32" by Espressif Systems, v3.x or newer
 *  - Board:            "ESP32C6 Dev Module" (or your specific C6 board)
 *  - Tools -> Zigbee mode:        "Zigbee ED (end device)"
 *  - Tools -> Partition Scheme:   any scheme with "Zigbee" in the name
 *  - Erase flash fully the first time you flash, then turn it back off.
 *
 * WIRING (per thermistor, repeat for both):
 *   DIVIDER_POWER_PIN ---[NTC]---+---[FIXED_RESISTOR]--- GND
 *                                |
 *                           ADC input pin
 *
 *   The dividers are fed from a GPIO so they only draw current while
 *   measuring. A C6 GPIO can source ~20 mA; two dividers of ~10k draw
 *   well under 1 mA, so this is fine.
 *
 *   Pins (change to match your board; all must be ADC-capable):
 *     GPIO2 -> DIVIDER_POWER_PIN
 *     GPIO0 -> INLET_ADC_PIN
 *     GPIO1 -> OUTLET_ADC_PIN
 *     GPIO3 -> BATTERY_ADC_PIN (optional, set to -1 to disable)
 *
 *   OPTIONAL battery monitor: battery+ --[1M]--+--[1M]-- GND, midpoint to
 *   BATTERY_ADC_PIN, plus a 100 nF capacitor from that pin to GND (the
 *   source impedance is high). Costs ~1.5 uA continuously.
 *
 * CALIBRATION:
 *   1. With DIVIDER_POWER_PIN HIGH, measure the real voltage at the top of
 *      the dividers with a multimeter and enter it as RAIL_MV.
 *   2. Put both probes in a glass of ice water (0 C) and in a warm water
 *      bath next to a trusted thermometer, then adjust INLET_OFFSET_C and
 *      OUTLET_OFFSET_C. The delta-T is what you care about, so matching
 *      the two sensors to each other matters more than absolute accuracy.
 */

#include <Arduino.h>

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"

// ---------------------------------------------------------------------
// USER CONFIG
// ---------------------------------------------------------------------

// Pins
#define DIVIDER_POWER_PIN  2
#define INLET_ADC_PIN      0
#define OUTLET_ADC_PIN     1
#define BATTERY_ADC_PIN    -1        // e.g. 3 to enable, -1 to disable
#define BUTTON_PIN         BOOT_PIN  // factory reset only

// Thermistor / divider parameters
#define NTC_NOMINAL_R      10000.0f  // Ohms at 25 C (R0)
#define NTC_NOMINAL_T      25.0f     // Celsius
#define NTC_BETA           3950.0f
// 4.7k centres the divider for ~20-90 C with a 10k NTC. With 10k the node
// voltage gets close to the top of the ADC range when the pipe is hot.
#define FIXED_RESISTOR     4700.0f

// Measured voltage at the top of the dividers while DIVIDER_POWER_PIN is
// HIGH, in millivolts. Measure it rather than assuming 3300.
#define RAIL_MV            3300.0f

// Per-sensor calibration offsets in Celsius (see CALIBRATION above)
#define INLET_OFFSET_C     0.0f
#define OUTLET_OFFSET_C    0.0f

// Readings outside these node voltages are treated as a fault
// (open/short thermistor, or ADC saturation at 11 dB).
#define NODE_MIN_MV        50.0f
#define NODE_MAX_MV        3000.0f

// Battery voltage range used for the percentage estimate (single Li-ion
// cell by default; for 2xAA alkaline use roughly 2000 / 3000).
#define BATT_EMPTY_MV      3300
#define BATT_FULL_MV       4150

// Sleep / reporting
#define uS_TO_S_FACTOR     1000000ULL
#define TIME_TO_SLEEP      60        // seconds between readings
#define JOIN_TIMEOUT_MS    30000     // give up joining/rejoining after this
#define REPORT_SETTLE_MS   1500      // time to let reports go out before sleeping
#define FACTORY_RESET_MS   5000      // hold BOOT this long to factory reset

// Zigbee endpoints - must be unique numbers 1-254
#define INLET_ENDPOINT     10
#define OUTLET_ENDPOINT    11

// ---------------------------------------------------------------------

ZigbeeTempSensor zbInlet(INLET_ENDPOINT);
ZigbeeTempSensor zbOutlet(OUTLET_ENDPOINT);

// ---------------------------------------------------------------------
// Go to deep sleep (timer wake is already configured in setup)
// ---------------------------------------------------------------------
void sleepNow() {
  Serial.println("Going to sleep.");
  Serial.flush();
  esp_deep_sleep_start();
}

// ---------------------------------------------------------------------
// Convert one ADC pin into a temperature (C) via the Beta equation.
// Returns NAN if the reading is implausible.
// Divider: RAIL -- NTC -- [node/ADC] -- FIXED_RESISTOR -- GND
// ---------------------------------------------------------------------
float readThermistorC(uint8_t pin, float offsetC) {
  const int SAMPLES = 16;
  uint32_t total = 0;
  for (int i = 0; i < SAMPLES; i++) {
    total += analogReadMilliVolts(pin);  // factory-calibrated by the core
    delayMicroseconds(200);
  }
  float nodeMv = (float)total / SAMPLES;

  if (nodeMv < NODE_MIN_MV || nodeMv > NODE_MAX_MV) return NAN;

  // Vnode/Vrail = FIXED / (Rntc + FIXED)  =>  Rntc = FIXED * (Vrail/Vnode - 1)
  float rNtc = FIXED_RESISTOR * ((RAIL_MV / nodeMv) - 1.0f);
  if (rNtc <= 0.0f) return NAN;

  float t0Kelvin = NTC_NOMINAL_T + 273.15f;
  float steinhart = logf(rNtc / NTC_NOMINAL_R) / NTC_BETA + 1.0f / t0Kelvin;
  float tempC = 1.0f / steinhart - 273.15f + offsetC;

  if (isnan(tempC) || tempC < -40.0f || tempC > 150.0f) return NAN;
  return tempC;
}

// ---------------------------------------------------------------------
// Battery: returns millivolts, or 0 if disabled
// ---------------------------------------------------------------------
uint32_t readBatteryMv() {
  if (BATTERY_ADC_PIN < 0) return 0;
  const int SAMPLES = 16;
  uint32_t total = 0;
  for (int i = 0; i < SAMPLES; i++) {
    total += analogReadMilliVolts(BATTERY_ADC_PIN);
    delay(1);
  }
  return (total / SAMPLES) * 2;  // undo the 1M/1M divider
}

// ---------------------------------------------------------------------
// Measure, report over Zigbee, then deep sleep
// ---------------------------------------------------------------------
void measureReportAndSleep() {
  digitalWrite(DIVIDER_POWER_PIN, HIGH);
  delay(10);  // let the dividers settle

  float inletC = readThermistorC(INLET_ADC_PIN, INLET_OFFSET_C);
  float outletC = readThermistorC(OUTLET_ADC_PIN, OUTLET_OFFSET_C);

  digitalWrite(DIVIDER_POWER_PIN, LOW);  // dividers off before we use the radio

  bool inletOk = !isnan(inletC);
  bool outletOk = !isnan(outletC);

  if (inletOk && outletOk) {
    Serial.printf("Inlet: %.2f C   Outlet: %.2f C   Delta-T: %.2f C\r\n", inletC, outletC, inletC - outletC);
  } else {
    Serial.printf("Sensor fault - inlet %s, outlet %s\r\n", inletOk ? "ok" : "BAD", outletOk ? "ok" : "BAD");
  }

  // Battery (on the inlet endpoint only)
  uint32_t battMv = readBatteryMv();
  if (battMv > 0) {
    int pct = map((long)battMv, BATT_EMPTY_MV, BATT_FULL_MV, 0, 100);
    pct = constrain(pct, 0, 100);
    // Zigbee battery percentage is in 0.5% steps (0-200); voltage in 100 mV units.
    // Check your core version's ZigbeeEP docs if these units differ.
    zbInlet.setBatteryPercentage((uint8_t)pct);
    zbInlet.setBatteryVoltage((uint8_t)(battMv / 100));
    Serial.printf("Battery: %u mV (%d%%)\r\n", (unsigned)battMv, pct);
  }

  // Only report valid readings - never send NaN to the coordinator
  if (inletOk) {
    zbInlet.setTemperature(inletC);
    zbInlet.report();
  }
  if (outletOk) {
    zbOutlet.setTemperature(outletC);
    zbOutlet.report();
  }
  if (battMv > 0) {
    zbInlet.reportBatteryPercentage();
  }

  // Attribute reports don't reliably produce an application-level ack, so
  // just give the stack a short, fixed window to transmit, then sleep.
  delay(REPORT_SETTLE_MS);

  sleepNow();
}

// ---------------------------------------------------------------------
// Hold BOOT for FACTORY_RESET_MS while the device is awake to factory
// reset. Call only after Zigbee.begin() succeeded.
// NOTE: don't hold BOOT while pressing RESET/power-cycling - that enters
// the ROM bootloader. Press and hold it while the device is waking.
// ---------------------------------------------------------------------
void checkFactoryReset() {
  if (digitalRead(BUTTON_PIN) != LOW) return;

  Serial.println("BOOT held - keep holding to factory reset...");
  unsigned long start = millis();
  while (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - start >= FACTORY_RESET_MS) {
      Serial.println("Factory reset...");
      delay(200);
      Zigbee.factoryReset();  // clears Zigbee NVS and restarts
      return;
    }
    delay(50);
  }
  Serial.println("Released early, reset cancelled.");
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(DIVIDER_POWER_PIN, OUTPUT);
  digitalWrite(DIVIDER_POWER_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(INLET_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(OUTLET_ADC_PIN, ADC_11db);
  if (BATTERY_ADC_PIN >= 0) analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // --- Inlet endpoint ---
  zbInlet.setManufacturerAndModel("DIY", "RadiatorInletTemp");
  zbInlet.setMinMaxValue(-10, 120);
  zbInlet.setDefaultValue(20.0);
  zbInlet.setTolerance(0.5);
  zbInlet.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, 37);  // placeholder until first real reading

  // --- Outlet endpoint ---
  zbOutlet.setManufacturerAndModel("DIY", "RadiatorOutletTemp");
  zbOutlet.setMinMaxValue(-10, 120);
  zbOutlet.setDefaultValue(20.0);
  zbOutlet.setTolerance(0.5);

  Zigbee.addEndpoint(&zbInlet);
  Zigbee.addEndpoint(&zbOutlet);

  // Battery-friendly end device config. Verify that zed_cfg.keep_alive
  // exists in your core version (field names have shifted between 3.x).
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;
  Zigbee.setTimeout(10000);

  Serial.println("Starting Zigbee...");
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    Serial.println("Zigbee failed to start! Sleeping and retrying next cycle.");
    sleepNow();
  }

  checkFactoryReset();

  Serial.println("Connecting to network");
  unsigned long joinStart = millis();
  while (!Zigbee.connected()) {
    if (millis() - joinStart > JOIN_TIMEOUT_MS) {
      Serial.println("\nJoin timed out - sleeping and retrying next cycle.");
      sleepNow();
    }
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected.");

  measureReportAndSleep();  // never returns
}

void loop() {
  // Never reached in normal operation: setup() always ends in deep sleep.
}
