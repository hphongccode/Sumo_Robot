#ifndef MAX17043_H
#define MAX17043_H

#include "esp_err.h"

/** @brief Initialize I2C communication with the MAX17043 sensor
     * @param sda_io GPIO pin for SDA
     * @param scl_io GPIO pin for SCL
     * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t max17043_init(int sda_io, int scl_io);
/**
 * @brief Read State of Charge (SOC)
 * * @return Value in percentage (0.0 - 100.0), or -1.0 on error
 */
float max17043_get_soc();
/**
 * @brief Read Cell Voltage (VCELL)
 * * @return Voltage in volts, or -1.0 on error
 */
float max17043_get_vcell();
/**
 * @brief Send Quick-Start command to reset ModelGauge algorithm
 */
void max17043_quick_start();

#endif