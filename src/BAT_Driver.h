#pragma once
#include <Arduino.h> 

#define BAT_ADC_PIN   8
#define Measurement_offset 0.990476   

void BAT_Init(void);
void BAT_Get_Volts(void);
bool BAT_Is_Charging(void);
const char *BAT_Get_Charge_Percentage(void);
const char *BAT_Get_Charge_Percentage_Str(void);