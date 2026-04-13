#include "PWR_Key.h"
#include "BAT_Driver.h"
#include "LVGL_Driver.h"
#include "Touch_SPD2010.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_pm.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "esp_system.h"

static uint8_t BAT_State = 0;
static uint8_t Device_State = 0;
static uint16_t Long_Press = 0;
static bool lastButtonPressed = false;

// Display idle-blanking state
static volatile unsigned long lastActivityTime = 0;  // updated from ISR and main loop
static volatile bool wakeRequestFromIsr = false;
static bool displayAwake = true;
static unsigned long displaySleepStartMs = 0;
static volatile bool wakeRefreshPending = false;
static esp_pm_lock_handle_t cpuMaxLock = nullptr;
static esp_pm_lock_handle_t noLightSleepLock = nullptr;
static volatile bool shutdownInProgress = false;

static bool PWR_HasExternalPower(void) {
  int mv = analogReadMilliVolts(BAT_ADC_PIN);
  float sensedVoltage = ((float)(mv * 3.0f / 1000.0f) / Measurement_offset);
  return sensedVoltage > 4.0f;
}

static void AcquireAwakeCpuLock() {
  if (!cpuMaxLock) return;
  esp_err_t err = esp_pm_lock_acquire(cpuMaxLock);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf(">> Failed to acquire awake CPU PM lock: %s\n", esp_err_to_name(err));
  }
}

static void ReleaseAwakeCpuLock() {
  if (!cpuMaxLock) return;
  esp_err_t err = esp_pm_lock_release(cpuMaxLock);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf(">> Failed to release awake CPU PM lock: %s\n", esp_err_to_name(err));
  }
}

// Called from touch ISR and anywhere else user activity is detected.
// Only updates a timestamp — safe to call from an interrupt context.
void PWR_UpdateActivity(void) {
  lastActivityTime = millis();
}

void PWR_RequestWakeFromISR(void) {
  wakeRequestFromIsr = true;
}

// ISR for the power button (FALLING edge = button press).
// With Driver_Loop polling at 500 ms, a brief press could be entirely missed
// between two polls.  This ISR stamps lastActivityTime the instant the button
// is pressed so the next PWR_Loop wake check always sees it.
static void IRAM_ATTR PWR_Key_ISR(void) {
  lastActivityTime = millis();
}

bool PWR_IsDisplayAwake(void) {
  return displayAwake;
}

bool PWR_IsShutdownInProgress(void) {
  return shutdownInProgress;
}

unsigned long PWR_GetDisplaySleepMs(void) {
  if (displayAwake) return 0;
  return millis() - displaySleepStartMs;
}

bool PWR_ConsumeWakeRefreshRequest(void) {
  bool pending = wakeRefreshPending;
  wakeRefreshPending = false;
  return pending;
}

void Fall_Asleep(void) {
  displayAwake = false;
  displaySleepStartMs = millis();
  wakeRequestFromIsr = false;
  ReleaseAwakeCpuLock();
  // Force panel brightness off without modifying LCD_Backlight.
  // This preserves the user's configured brightness for wake restore.
  // Detach the LEDC backlight channel entirely — setting duty to 0 still
  // keeps the LEDC peripheral clocked, which can hold an APB_FREQ_MAX PM
  // lock and prevent DFS from downclocking the CPU.
  ledcDetach(LCD_Backlight_PIN);
  // After detach the pin floats — drive it LOW to ensure backlight is off.
  pinMode(LCD_Backlight_PIN, OUTPUT);
  digitalWrite(LCD_Backlight_PIN, LOW);
  LCD_Sleep(true);
  // Clear any pending touch interrupt — the SPD2010 fires phantom interrupts
  // shortly after LCD_Sleep(true).  The 3-second guard in UI_Loop_Task
  // provides additional protection, but clearing here avoids an immediate
  // false wake from a stale flag.
  Touch_ClearPendingInterrupt();
  // Stop the 20 ms LVGL tick timer — it wakes the CPU from light sleep
  // every 20 ms even though there is nothing to render while display is off.
  Lvgl_PauseTick();
}

void Restart(void) {

}

void Shutdown(void) {
  if (shutdownInProgress) {
    while (true) {
      esp_rom_delay_us(50000);
    }
  }

  shutdownInProgress = true;
  displayAwake = false;
  displaySleepStartMs = 0;
  wakeRequestFromIsr = false;
  wakeRefreshPending = false;

  if (noLightSleepLock) {
    esp_err_t err = esp_pm_lock_acquire(noLightSleepLock);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      Serial.printf(">> Failed to acquire shutdown no-light-sleep PM lock: %s\n", esp_err_to_name(err));
    }
  }
  ReleaseAwakeCpuLock();
  detachInterrupt(digitalPinToInterrupt(PWR_KEY_Input_PIN));
  Touch_ClearPendingInterrupt();
  Lvgl_PauseTick();

  // Quiesce display-related loads before dropping the power latch.
  ledcDetach(LCD_Backlight_PIN);
  pinMode(LCD_Backlight_PIN, OUTPUT);
  digitalWrite(LCD_Backlight_PIN, LOW);
  LCD_Sleep(true);

  bool hasExternalPower = PWR_HasExternalPower();
  if (!hasExternalPower) {
    pinMode(PWR_Control_PIN, OUTPUT);
    digitalWrite(PWR_Control_PIN, LOW);

    // On battery power, dropping the latch should remove power completely.
    // If it does not, fall back to deep sleep rather than spinning forever.
    esp_rom_delay_us(200000);
  } else {
    Serial.println("[Power] external power present, entering deep sleep instead of unlatching");
  }

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PWR_KEY_Input_PIN, 0);
  esp_deep_sleep_start();

  while (true) {
    vTaskDelay(portMAX_DELAY);
  }
}

void PWR_Loop(void) {
  unsigned long now = millis();
  static unsigned long lastPowerLog = 0;
  static unsigned long lastLoopTs = 0;
  static unsigned long awakeAccumMs = 0;
  static unsigned long asleepAccumMs = 0;

  wakeRequestFromIsr = false;

  if (lastLoopTs != 0) {
    unsigned long dt = now - lastLoopTs;
    if (displayAwake) awakeAccumMs += dt;
    else asleepAccumMs += dt;
  }
  lastLoopTs = now;

  // --- Button long-press handling ---
  // IMPORTANT: Button handling MUST run before the display wake check below.
  // PWR_UpdateActivity() sets lastActivityTime, and the wake check tests
  // (now - lastActivityTime < 1000).  If button handling ran after the wake
  // check, the activity timestamp wouldn't be seen until the NEXT PWR_Loop
  // iteration — with a 500 ms Driver_Loop delay the wake window would expire.
  if (BAT_State) {
    bool buttonPressed = !digitalRead(PWR_KEY_Input_PIN);
    if (buttonPressed) {
      // Treat button as activity only on the press edge, and only once the
      // boot-time key release guard has finished (BAT_State == 2).
      // This prevents a noisy/stuck-low key pin from continuously resetting
      // screen idle timeout and causing high battery drain.
      if (BAT_State == 2 && !lastButtonPressed) {
        PWR_UpdateActivity();
      }
      if (BAT_State == 2) {
        Long_Press++;
        if (Long_Press >= Device_Sleep_Time) {
          if (Long_Press >= Device_Sleep_Time && Long_Press < Device_Restart_Time)
            Device_State = 1;
          else if (Long_Press >= Device_Restart_Time && Long_Press < Device_Shutdown_Time)
            Device_State = 2;
          else if (Long_Press >= Device_Shutdown_Time)
            Shutdown();
        }
      }
      lastButtonPressed = true;
    } else {
      if (BAT_State == 1)
        BAT_State = 2;
      Long_Press = 0;
      lastButtonPressed = false;
    }
  }

  // --- Display idle timeout ---
  // Re-read now after button handling so the wake check uses a fresh timestamp.
  now = millis();
  if (displayAwake) {
    if (now - lastActivityTime > SCREEN_TIMEOUT_MS) {
      Serial.println("[Power] inactivity timeout reached, shutting down");
      Shutdown();
    }
  } else {
    // Wake display when fresh activity arrives. The off-state Driver_Loop can
    // sleep for up to 1000 ms, so keep a wider window here; otherwise a valid
    // button IRQ can age out before the next poll and the display appears to
    // ignore the press.
    if (now - lastActivityTime < 2000) {
      unsigned long sleptForMs = PWR_GetDisplaySleepMs();
      displayAwake = true;
      displaySleepStartMs = 0;
      wakeRefreshPending = true;
      AcquireAwakeCpuLock();
      Serial.printf("[Power] waking display after %lums asleep\n",
                    sleptForMs);
      Lvgl_ResumeTick();
      LCD_Sleep(false);
      // Re-attach LEDC before setting brightness (was detached in Fall_Asleep).
      ledcAttach(LCD_Backlight_PIN, Frequency, Resolution);
      Set_Backlight(LCD_Backlight);
    }
  }

  if (now - lastPowerLog >= 60000) {
    lastPowerLog = now;
    unsigned long total = awakeAccumMs + asleepAccumMs;
    unsigned long awakePct = (total > 0) ? ((awakeAccumMs * 100UL) / total) : 0;
    Serial.printf("[Power] display=%s idle_ms=%lu backlight=%u key=%d awake_pct=%lu%%\n",
                  displayAwake ? "on" : "off",
                  (unsigned long)(now - lastActivityTime),
                  (unsigned)LCD_Backlight,
                  !digitalRead(PWR_KEY_Input_PIN),
                  awakePct);
    awakeAccumMs = 0;
    asleepAccumMs = 0;
  }
}

void PWR_Init(void) {
  // Active-low key input. Internal pull-up prevents floating/false-low reads
  // that can continuously reset activity and block display sleep.
  pinMode(PWR_KEY_Input_PIN, INPUT_PULLUP);
  // FALLING edge ISR: ensures even a brief button press (< 500 ms) sets
  // lastActivityTime immediately, so the next PWR_Loop wake check sees it.
  attachInterrupt(digitalPinToInterrupt(PWR_KEY_Input_PIN), PWR_Key_ISR, FALLING);
  pinMode(PWR_Control_PIN, OUTPUT);
  // Always latch power so the device stays on when USB is removed.
  // Shutdown() drives this LOW explicitly when a power-off is intended.
  digitalWrite(PWR_Control_PIN, HIGH);
  if (!digitalRead(PWR_KEY_Input_PIN)) {  // Button is active-low: pressed = LOW
    BAT_State = 1;  // Wait for button release before counting long-press
  } else {
    BAT_State = 2;  // USB/no-button boot: immediately ready for button events
  }
  lastActivityTime = millis();  // Start the idle timer from boot

  esp_err_t err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX,
                                     0,
                                     "display_awake_cpu",
                                     &cpuMaxLock);
  if (err != ESP_OK) {
    Serial.printf(">> Failed to create awake CPU PM lock: %s\n", esp_err_to_name(err));
    cpuMaxLock = nullptr;
  } else if (displayAwake) {
    AcquireAwakeCpuLock();
  }

  err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP,
                           0,
                           "shutdown_no_light_sleep",
                           &noLightSleepLock);
  if (err != ESP_OK) {
    Serial.printf(">> Failed to create shutdown PM lock: %s\n", esp_err_to_name(err));
    noLightSleepLock = nullptr;
  }
}
