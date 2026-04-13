#include "magic8ball.h"
#include "Display_SPD2010.h"
#include "Touch_SPD2010.h"
#include "RTC_PCF85063.h"
#include "LVGL_Driver.h"
#include "SD_Card.h"
#include "BAT_Driver.h"
#include "PWR_Key.h"
#include "Gyro_QMI8658.h"
#include "lvgl.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "soc/rtc.h"

static constexpr unsigned long kBatteryLogIntervalMs = 60000UL;

void Driver_Loop(void *parameter) {
  unsigned long lastImuRead = 0;
  unsigned long lastSlowRead = 0;
  unsigned long lastBatteryLog = 0;

  while (1) {
    if (PWR_IsShutdownInProgress()) {
      vTaskSuspend(NULL);
    }

    // PWR_Loop always runs at 100ms — it drives the display wake/sleep state machine.
    PWR_Loop();

    bool awake = PWR_IsDisplayAwake();
    unsigned long now = millis();

    // Higher IMU cadence helps catch shorter wrist-motion peaks used by the
    // software step detector, especially during light walking.
    unsigned long imuInterval = awake ? 50UL : 75UL;
    if (now - lastImuRead >= imuInterval) {
      lastImuRead = now;
      QMI8658_Loop();
    }

    // RTC/battery/time refresh does not need 10 Hz; reduce background wakeups.
    unsigned long slowInterval = awake ? 500UL : 2000UL;
    if (now - lastSlowRead >= slowInterval) {
      lastSlowRead = now;
      PCF85063_Loop();
      BAT_Get_Volts();
    }

    if (lastBatteryLog == 0 || now - lastBatteryLog >= kBatteryLogIntervalMs) {
      const char *batteryPercent = BAT_Get_Charge_Percentage_Str();
      if (batteryPercent != nullptr && batteryPercent[0] != '\0') {
        Serial.printf("[Battery] remaining=%s state=%s\n",
                      batteryPercent,
                      BAT_Is_Charging() ? "charging" : "discharging");
        lastBatteryLog = now;
      }
    }

    // 100 ms when awake for responsive UI; slower cadence when asleep reduces
    // background wakeups and gives the automatic PM/tickless-idle path longer
    // uninterrupted windows to enter light sleep safely.
    if (awake) {
      vTaskDelay(pdMS_TO_TICKS(100));
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void Driver_Init() {
  Flash_test();
  PWR_Init();
  BAT_Init();
  I2C_Init();
  TCA9554PWR_Init(0x00);
  Backlight_Init();
  Set_Backlight(60);
  PCF85063_Init();
  QMI8658_Init();
}

TaskHandle_t driverTaskHandle = NULL;
TaskHandle_t uiLoopTaskHandle = NULL;

// UI loop task — replaces default loop() with 16KB stack (internal RAM)
void UI_Loop_Task(void *parameter) {
  while (1) {
    if (PWR_IsShutdownInProgress()) {
      vTaskSuspend(NULL);
    }

    bool awake = PWR_IsDisplayAwake();

    if (awake) {
      if (PWR_ConsumeWakeRefreshRequest()) {
        Magic8Ball_Reset();
        lv_obj_invalidate(lv_scr_act());
      }

      Magic8Ball_Init();
    }

    if (awake) {
      Lvgl_Loop();  // LVGL rendering + input processing
    } else {
      if (Touch_HasPendingInterrupt()) {
        if (PWR_GetDisplaySleepMs() > 3000) {
          PWR_UpdateActivity();
        }
      }
    }

    if (awake) {
        // When awake, run the UI loop at a responsive cadence (10 Hz) to keep touch and display updates smooth.
        // This is the main loop for processing user interactions and rendering the UI.
    }
    vTaskDelay(pdMS_TO_TICKS(awake ? 10 : 300));
  }
}


void setup() {
  // Calibrate the internal fast RC oscillator for better sleep timing
  rtc_clk_fast_freq_set(RTC_FAST_FREQ_8M);

  Driver_Init();
  setCpuFrequencyMhz(240);

  Serial.begin(115200);
  Serial.setTxBufferSize(2048);
  delay(200);

  {
    int _mv = analogReadMilliVolts(BAT_ADC_PIN);
    bool bootCharging = ((float)(_mv * 3.0f / 1000.0f) / Measurement_offset) > 4.0f;
    if (bootCharging) {
      unsigned long serialWait = millis();
      while (!Serial && millis() - serialWait < 3000) { delay(10); }  // key for native USB CDC
    }
  }

  SD_Init();
  LCD_Init();
  I2C_Init();
  QMI8658_Init();
  Lvgl_Init();

  xTaskCreatePinnedToCoreWithCaps(
    Driver_Loop,
    "Driver Task",
    6144,
    NULL,
    3,
    &driverTaskHandle,
    0,
    MALLOC_CAP_SPIRAM);

  xTaskCreatePinnedToCoreWithCaps(
    UI_Loop_Task,
    "UI Loop",
    16384,
    NULL,
    1,
    &uiLoopTaskHandle,
    1,
    MALLOC_CAP_SPIRAM);
  
  int _pmMv = analogReadMilliVolts(BAT_ADC_PIN);
  bool usbConnected = ((float)(_pmMv * 3.0f / 1000.0f) / Measurement_offset) > 4.0f;
  bool allow_light_sleep = !usbConnected;
  esp_pm_config_t pm_config = {
    .max_freq_mhz       = 240,
    .min_freq_mhz       = 40,
    .light_sleep_enable = allow_light_sleep,
  };
  esp_err_t pm_err = esp_pm_configure(&pm_config);
  if (pm_err == ESP_ERR_NOT_SUPPORTED) {
    pm_config.light_sleep_enable = false;
    pm_err = esp_pm_configure(&pm_config);
    if (pm_err == ESP_OK) {
      Serial.println(">> Power management: DFS enabled (40-240MHz), light sleep not supported");
    }
  }
  if (pm_err != ESP_OK) {
    Serial.printf(">> Power management setup failed: %s\n", esp_err_to_name(pm_err));
  } else if (pm_config.light_sleep_enable) {
    Serial.println(">> Power management: DFS + automatic light sleep enabled");
  } else if (allow_light_sleep == false) {
    Serial.println(">> Power management: DFS enabled, light sleep disabled (USB connected)");
  }
}


void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
