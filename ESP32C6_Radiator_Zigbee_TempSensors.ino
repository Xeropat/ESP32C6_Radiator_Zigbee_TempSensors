/**
 * ESP32-C6 battery-powered hydronic radiator temperature sensor
 * -------------------------------------------------------------
 * Reads two NTC thermistors (radiator inlet/flow and outlet/return)
 * and reports both as Zigbee Temperature Measurement endpoints.
 * Deep-sleeps between readings to save battery.
 *
 * REQUIREMENTS (Arduino IDE):
 *  - Board package: "esp32" by Espressif Systems, v3.x or newer
 *    (Boards Manager -> search "esp32" -> install)
 *  - Board:            "ESP32C6 Dev Module" (or your specific C6 board)
 *  - Tools -> Zigbee mode:        "Zigbee ED (end device)"
 *  - Tools -> Partition Scheme:   any scheme with "Zigbee" in the name
 *                                 (e.g. "Zigbee 4MB with spiffs")
 *  - Erase flash fully the first time you flash (Tools -> Erase All
 *    Flash Before Sketch Upload -> Enabled), then you can turn that back off.
 *
 * WIRING (per thermistor, repeat for both):
 *   3.3V ---[NTC thermistor]---+---[10k fixed resistor]--- GND
 *                               |
 *                          ADC input pin
 *
 *   The "3.3V" side of each divider is NOT the board's always-on 3V3 pin,
 *   it is fed from DIVIDER_POWER_PIN below, so the dividers only draw
 *   current for the ~1 second it takes to measure, not 24/7.
 *
 *   Pins used here (change to match your board):
 *     GPIO2 -> DIVIDER_POWER_PIN (drives both dividers' high side)
 *     GPIO0 -> INLET_ADC_PIN     (flow / supply temperature)
 *     GPIO1 -> OUTLET_ADC_PIN    (return temperature)
 *   All three must be ADC1-capable pins on your specific C6 board -
 *   check your board's pinout diagram if you move them.
 *
 * Created for: hydronic radiator delta-T monitoring
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
#define DIVIDER_POWER_PIN  2   // GPIO that powers both thermistor dividers
#define INLET_ADC_PIN      0   // Radiator inlet / flow thermistor
#define OUTLET_ADC_PIN     1   // Radiator outlet / return thermistor
#define BUTTON_PIN         BOOT_PIN  // Used for factory reset only

// Thermistor / divider parameters - EDIT to match your actual parts
#define NTC_NOMINAL_R      10000.0f   // Ohms at 25C (R0), from datasheet
#define NTC_NOMINAL_T      25.0f      // Reference temp for R0, in Celsius
#define NTC_BETA           3950.0f    // Beta coefficient, from datasheet
#define FIXED_RESISTOR     10000.0f   // The fixed resistor value in the divider

// Sleep / reporting
#define uS_TO_S_FACTOR     1000000ULL
#define TIME_TO_SLEEP      60         // seconds between readings
#define REPORT_TIMEOUT_MS  2000       // how long to wait for Zigbee ack

// Zigbee endpoints - must be unique numbers 1-254
#define INLET_ENDPOINT     10
#define OUTLET_ENDPOINT    11

// ---------------------------------------------------------------------

ZigbeeTempSensor zbInlet(INLET_ENDPOINT);
ZigbeeTempSensor zbOutlet(OUTLET_ENDPOINT);

// Counts down as each endpoint's report is acknowledged, so we know
// when it's safe to go back to sleep.
volatile uint8_t pendingReports = 0;
volatile bool resend = false;

// ---------------------------------------------------------------------
// Zigbee report acknowledgement callback (fires for either endpoint)
// ---------------------------------------------------------------------
void onZbResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
  if (command != ZB_CMD_REPORT_ATTRIBUTE) return;
  if (endpoint != INLET_ENDPOINT && endpoint != OUTLET_ENDPOINT) return;

  if (status == ESP_ZB_ZCL_STATUS_SUCCESS) {
    if (pendingReports > 0) pendingReports--;
  } else {
    resend = true;
  }
}

// ---------------------------------------------------------------------
// Convert one ADC pin reading into a temperature via the Beta formula
// ---------------------------------------------------------------------
float readThermistorC(uint8_t pin) {
  const int SAMPLES = 16;
  uint32_t total = 0;
  for (int i = 0; i < SAMPLES; i++) {
    total += analogRead(pin);
    delayMicroseconds(200);
  }
  float adcAvg = (float)total / SAMPLES;
  float adcMax = 4095.0f; // 12-bit ADC

  // Divider: 3.3V -- NTC -- [node/ADC] -- FIXED_RESISTOR -- GND
  // Vnode/Vcc = FIXED / (Rntc + FIXED)  =>  Rntc = FIXED * (Vcc/Vnode - 1)
  // If you wire it the other way around (fixed resistor on top, NTC to
  // GND), use instead: Rntc = FIXED / (Vcc/Vnode - 1)
  float rNtc = FIXED_RESISTOR * ((adcMax / adcAvg) - 1.0f);

  // Beta parameter equation
  float t0Kelvin = NTC_NOMINAL_T + 273.15f;
  float steinhart = log(rNtc / NTC_NOMINAL_R) / NTC_BETA;
  steinhart += 1.0f / t0Kelvin;
  float tempKelvin = 1.0f / steinhart;
  return tempKelvin - 273.15f;
}

// ---------------------------------------------------------------------
// Measure both thermistors, report over Zigbee, then deep sleep
// ---------------------------------------------------------------------
void measureReportAndSleep() {
  // Power up the dividers and let them settle
  digitalWrite(DIVIDER_POWER_PIN, HIGH);
  delay(10);

  float inletC = readThermistorC(INLET_ADC_PIN);
  float outletC = readThermistorC(OUTLET_ADC_PIN);

  // Cut divider power immediately - no need to keep it on while we
  // talk over Zigbee.
  digitalWrite(DIVIDER_POWER_PIN, LOW);

  Serial.printf("Inlet: %.2f C   Outlet: %.2f C   Delta-T: %.2f C\r\n", inletC, outletC, inletC - outletC);

  zbInlet.setTemperature(inletC);
  zbOutlet.setTemperature(outletC);

  pendingReports = 2;
  zbInlet.report();
  zbOutlet.report();

  unsigned long start = millis();
  int tries = 0;
  const int maxTries = 3;
  while (pendingReports != 0 && tries < maxTries) {
    if (resend) {
      resend = false;
      pendingReports = 2;
      zbInlet.report();
      zbOutlet.report();
    }
    if (millis() - start >= REPORT_TIMEOUT_MS) {
      pendingReports = 2;
      zbInlet.report();
      zbOutlet.report();
      start = millis();
      tries++;
    }
    delay(50);
  }

  Serial.println("Going to sleep.");
  Serial.flush();
  esp_deep_sleep_start();
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(DIVIDER_POWER_PIN, OUTPUT);
  digitalWrite(DIVIDER_POWER_PIN, LOW); // keep dividers off by default

  analogReadResolution(12);
  analogSetPinAttenuation(INLET_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(OUTLET_ADC_PIN, ADC_11db);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // --- Inlet endpoint setup ---
  zbInlet.setManufacturerAndModel("DIY", "RadiatorInletTemp");
  zbInlet.setMinMaxValue(-10, 120);   // sane range for a radiator pipe
  zbInlet.setDefaultValue(20.0);
  zbInlet.setTolerance(0.5);
  zbInlet.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, 30); // placeholder 100%, 3.0V

  // --- Outlet endpoint setup ---
  zbOutlet.setManufacturerAndModel("DIY", "RadiatorOutletTemp");
  zbOutlet.setMinMaxValue(-10, 120);
  zbOutlet.setDefaultValue(20.0);
  zbOutlet.setTolerance(0.5);
  zbOutlet.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, 30);

  Zigbee.onGlobalDefaultResponse(onZbResponse);

  Zigbee.addEndpoint(&zbInlet);
  Zigbee.addEndpoint(&zbOutlet);

  // Battery-friendly end device config: shorter keep-alive, shorter
  // join timeout so a failed join doesn't drain the battery scanning
  // channels for a long time.
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;
  Zigbee.setTimeout(10000);

  Serial.println("Starting Zigbee...");
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    Serial.println("Zigbee failed to start! Rebooting...");
    ESP.restart();
  }

  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected.");

  measureReportAndSleep(); // never returns - device deep sleeps here
}

void loop() {
  // Only reached if something above didn't sleep (e.g. during first
  // pairing troubleshooting). Holding the boot button 5s factory-resets.
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(100);
    unsigned long start = millis();
    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(50);
      if (millis() - start > 5000) {
        Serial.println("Factory reset...");
        delay(500);
        Zigbee.factoryReset();
      }
    }
  }
  delay(100);
}
