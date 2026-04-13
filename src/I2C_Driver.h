#pragma once
#include <Wire.h> 
#include <stdint.h>
#include <esp_err.h>

#define I2C_MASTER_FREQ_HZ              (400000)                     /*!< I2C master clock frequency */
#define I2C_SCL_PIN       10
#define I2C_SDA_PIN       11


void I2C_Init(void);
bool I2C_Lock(uint32_t timeoutMs = 200);
void I2C_Unlock(void);

esp_err_t I2C_Read(uint8_t Driver_addr, uint8_t Reg_addr, uint8_t *Reg_data, uint32_t Length);
esp_err_t I2C_Write(uint8_t Driver_addr, uint8_t Reg_addr, const uint8_t *Reg_data, uint32_t Length);
