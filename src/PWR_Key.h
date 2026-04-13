#pragma once
#include "Arduino.h"
#include "Display_SPD2010.h"

#define PWR_KEY_Input_PIN   6
#define PWR_Control_PIN     7

#define Device_Sleep_Time    10
#define Device_Restart_Time  15
#define Device_Shutdown_Time 20

#define SCREEN_TIMEOUT_MS    15000   // Fully power off after 15 s of inactivity

void Fall_Asleep(void);
void Shutdown(void);
void Restart(void);

void PWR_Init(void);
void PWR_Loop(void);
void PWR_UpdateActivity(void);  // Call on any user interaction (touch, button)
void PWR_RequestWakeFromISR(void);  // ISR-safe wake request
bool PWR_IsDisplayAwake(void);
bool PWR_IsShutdownInProgress(void);
unsigned long PWR_GetDisplaySleepMs(void);  // 0 when awake, else elapsed sleep time
bool PWR_ConsumeWakeRefreshRequest(void);  // true once after each display wake
