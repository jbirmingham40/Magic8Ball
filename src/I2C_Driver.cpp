#include "I2C_Driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t g_i2c_mutex = nullptr;

void I2C_Init(void) {
  if (!g_i2c_mutex) {
    g_i2c_mutex = xSemaphoreCreateMutex();
  }
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
}

bool I2C_Lock(uint32_t timeoutMs) {
  if (!g_i2c_mutex) {
    return false;
  }
  return xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void I2C_Unlock(void) {
  if (g_i2c_mutex) {
    xSemaphoreGive(g_i2c_mutex);
  }
}

esp_err_t I2C_Read(uint8_t Driver_addr, uint8_t Reg_addr, uint8_t *Reg_data, uint32_t Length) {
  if (!Reg_data || Length == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!I2C_Lock()) {
    printf("I2C lock timeout - I2C Read\r\n");
    return ESP_ERR_TIMEOUT;
  }

  Wire.beginTransmission(Driver_addr);
  Wire.write(Reg_addr);
  if (Wire.endTransmission(true)) {
    printf("The I2C transmission fails. - I2C Read\r\n");
    I2C_Unlock();
    return ESP_FAIL;
  }
  int received = Wire.requestFrom(Driver_addr, Length);
  if (received != (int)Length) {
    printf("The I2C read length is invalid. expected=%u got=%d\r\n", (unsigned)Length, received);
    I2C_Unlock();
    return ESP_FAIL;
  }
  for (uint32_t i = 0; i < Length && Wire.available(); i++) {
    *Reg_data++ = Wire.read();
  }
  I2C_Unlock();
  return ESP_OK;
}
esp_err_t I2C_Write(uint8_t Driver_addr, uint8_t Reg_addr, const uint8_t *Reg_data, uint32_t Length) {
  if ((!Reg_data && Length > 0) || !I2C_Lock()) {
    if (Length > 0 && !Reg_data) {
      return ESP_ERR_INVALID_ARG;
    }
    printf("I2C lock timeout - I2C Write\r\n");
    return ESP_ERR_TIMEOUT;
  }

  Wire.beginTransmission(Driver_addr);
  Wire.write(Reg_addr);
  for (uint32_t i = 0; i < Length; i++) {
    Wire.write(*Reg_data++);
  }
  if (Wire.endTransmission(true)) {
    printf("The I2C transmission fails. - I2C Write\r\n");
    I2C_Unlock();
    return ESP_FAIL;
  }
  I2C_Unlock();
  return ESP_OK;
}
