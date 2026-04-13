#include "BAT_Driver.h"
#include <math.h>

// Charging detection with hysteresis prevents icon/state flapping near 4.0 V.
// Tuned for this board's ADC path which often reports near ~4.0V at full.
#define CHARGING_VOLTAGE_ON 4.00f
#define CHARGING_VOLTAGE_OFF 3.95f

// Board-specific voltage span used for mapping to UI percent.
#define MAX_BAT_VOLTAGE 4.05f
#define MIN_BAT_VOLTAGE 3.0f

// Post-unplug settle window: voltage rebounds after charger removal and under
// burst load (screen on/WiFi). Use slower filtering during this period.
#define UNPLUG_SETTLE_MS 180000UL  // 3 minutes

// Voltage filter tuning (sample period is 5s)
#define VOLTAGE_FILTER_ALPHA_CHARGING   0.25f
#define VOLTAGE_FILTER_ALPHA_DISCHARGE  0.12f
#define VOLTAGE_FILTER_ALPHA_UNPLUG     0.05f

// Percent filter tuning (separate from voltage filter for stable UI)
#define PERCENT_FILTER_ALPHA_CHARGING   0.20f
#define PERCENT_FILTER_ALPHA_DISCHARGE  0.15f
#define PERCENT_FILTER_ALPHA_UNPLUG     0.06f

// ADC sampling: trimmed mean removes outliers from wake/noise spikes.
#define ADC_NUM_SAMPLES 16
#define ADC_TRIM_COUNT 2

typedef struct {
  float voltage;
  int percent;
} battery_curve_point_t;

static const battery_curve_point_t kBatteryCurve[] = {
    {4.05f, 100},
    {4.00f, 98},
    {3.95f, 95},
    {3.90f, 90},
    {3.85f, 82},
    {3.80f, 70},
    {3.75f, 62},
    {3.70f, 54},
    {3.65f, 45},
    {3.60f, 35},
    {3.55f, 27},
    {3.50f, 20},
    {3.45f, 14},
    {3.40f, 10},
    {3.35f, 7},
    {3.30f, 5},
    {3.20f, 2},
    {3.10f, 1},
    {3.00f, 0},
};

static int map_voltage_to_percent(float v) {
  const int points = sizeof(kBatteryCurve) / sizeof(kBatteryCurve[0]);

  if (v >= kBatteryCurve[0].voltage) return 100;
  if (v <= kBatteryCurve[points - 1].voltage) return 0;

  for (int i = 0; i < points - 1; i++) {
    const float vHigh = kBatteryCurve[i].voltage;
    const float vLow = kBatteryCurve[i + 1].voltage;
    if (v <= vHigh && v >= vLow) {
      const float span = vHigh - vLow;
      if (span <= 0.0f) return kBatteryCurve[i].percent;
      const float t = (v - vLow) / span;
      const float p = kBatteryCurve[i + 1].percent +
                      t * (kBatteryCurve[i].percent - kBatteryCurve[i + 1].percent);
      int percent = (int)lroundf(p);
      if (percent < 0) percent = 0;
      if (percent > 100) percent = 100;
      return percent;
    }
  }

  return 0;
}

float voltage = 0.0;
int chargePercentage;
char chargePercentageStr[5] = { 0 };
bool isCharging = false;

void BAT_Init(void) {
  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);
}

void BAT_Get_Volts(void) {
  static long lastRead = 0;
  static float filteredVoltage = 0.0f;
  static float filteredPercent = 0.0f;
  static bool hasFilterState = false;
  static bool hasPercentState = false;
  static unsigned long unplugTimestampMs = 0;
  static bool wasCharging = false;
  long now = millis();

  if (lastRead + 5000 < now) {
    lastRead = now;

    // Trimmed mean to reject outliers from ADC jitter and wake transitions.
    int samples[ADC_NUM_SAMPLES];
    for (int i = 0; i < ADC_NUM_SAMPLES; i++) {
      samples[i] = analogReadMilliVolts(BAT_ADC_PIN);
    }

    // Insertion sort (small fixed N, no heap use)
    for (int i = 1; i < ADC_NUM_SAMPLES; i++) {
      int key = samples[i];
      int j = i - 1;
      while (j >= 0 && samples[j] > key) {
        samples[j + 1] = samples[j];
        j--;
      }
      samples[j + 1] = key;
    }

    long sumMv = 0;
    int used = 0;
    for (int i = ADC_TRIM_COUNT; i < ADC_NUM_SAMPLES - ADC_TRIM_COUNT; i++) {
      sumMv += samples[i];
      used++;
    }
    if (used <= 0) return;

    float measuredVoltage = (float)(sumMv / used) * 3.0f / 1000.0f / Measurement_offset;
    voltage = measuredVoltage;

    // Charging-state hysteresis removes threshold chatter near 4.0 V.
    if (isCharging) {
      if (measuredVoltage < CHARGING_VOLTAGE_OFF) isCharging = false;
    } else {
      if (measuredVoltage > CHARGING_VOLTAGE_ON) isCharging = true;
    }

    if (wasCharging && !isCharging) {
      unplugTimestampMs = now;
    }
    wasCharging = isCharging;

    bool inUnplugSettle = (!isCharging &&
                           unplugTimestampMs > 0 &&
                           (unsigned long)(now - unplugTimestampMs) < UNPLUG_SETTLE_MS);

    if (!hasFilterState) {
      filteredVoltage = measuredVoltage;
      hasFilterState = true;
    } else {
      float alpha = VOLTAGE_FILTER_ALPHA_DISCHARGE;
      if (isCharging) alpha = VOLTAGE_FILTER_ALPHA_CHARGING;
      else if (inUnplugSettle) alpha = VOLTAGE_FILTER_ALPHA_UNPLUG;

      filteredVoltage = (alpha * measuredVoltage) +
                        ((1.0f - alpha) * filteredVoltage);
    }

    // Map filtered voltage to percentage.
    float displayVoltage = filteredVoltage;
    if (displayVoltage > MAX_BAT_VOLTAGE) displayVoltage = MAX_BAT_VOLTAGE;
    if (displayVoltage < MIN_BAT_VOLTAGE) displayVoltage = MIN_BAT_VOLTAGE;

    int rawPercent = map_voltage_to_percent(displayVoltage);

    // Additional smoothing directly in percent-space for a stable UI readout.
    if (!hasPercentState) {
      filteredPercent = (float)rawPercent;
      hasPercentState = true;
    } else {
      float pctAlpha = PERCENT_FILTER_ALPHA_DISCHARGE;
      if (isCharging) pctAlpha = PERCENT_FILTER_ALPHA_CHARGING;
      else if (inUnplugSettle) pctAlpha = PERCENT_FILTER_ALPHA_UNPLUG;

      filteredPercent = (pctAlpha * (float)rawPercent) +
                        ((1.0f - pctAlpha) * filteredPercent);

      // While charging, avoid fast downward jumps from transient load/ADC noise.
      // At 5 s sampling this caps drop to ~0.6%/minute.
      if (isCharging && filteredPercent < (float)chargePercentage - 0.05f) {
        filteredPercent = (float)chargePercentage - 0.05f;
      }

      // On battery, cap rebound speed so brief load relief doesn't jump 4-8% instantly.
      if (!isCharging && filteredPercent > (float)chargePercentage + 1.0f) {
        filteredPercent = (float)chargePercentage + 1.0f;
      }
    }

    chargePercentage = (int)lroundf(filteredPercent);
    if (chargePercentage < 0) chargePercentage = 0;
    if (chargePercentage > 100) chargePercentage = 100;
    sprintf(chargePercentageStr, "%d%%", (int)chargePercentage);
  }
}

bool BAT_Is_Charging(void) {
  return isCharging;
}

const char *BAT_Get_Charge_Percentage(void) {
  return chargePercentageStr;
}

const char *BAT_Get_Charge_Percentage_Str(void) {
  return chargePercentageStr;
}
