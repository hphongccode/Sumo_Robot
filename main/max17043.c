#include "max17043.h"   
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "MAX17043";

#define MAX17043_ADDR         0x36
#define MAX17043_VCELL_REG    0x02
#define MAX17043_SOC_REG      0x04
#define MAX17043_MODE_REG     0x06
#define MAX17043_COMMAND_REG  0xFE

esp_err_t max17043_init(int sda_pin, int scl_pin) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,              
        .sda_pullup_en = GPIO_PULLUP_ENABLE, 
        .scl_io_num = scl_pin,              
        .scl_pullup_en = GPIO_PULLUP_ENABLE, 
        .master.clk_speed = 100000,
    };
    
    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) return err;
    
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

float max17043_get_vcell() {
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX17043_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MAX17043_VCELL_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX17043_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return -1.0;

    int res = (data[0] << 4) | (data[1] >> 4);
    return (float)res * 0.00125f;
}

float max17043_get_soc() {
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX17043_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MAX17043_SOC_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX17043_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return -1.0;
    return (float)(data[0] + (data[1] / 256.0));
}

void max17043_quick_start() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX17043_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MAX17043_MODE_REG, true);
    i2c_master_write_byte(cmd, 0x40, true); 
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Quick-Start executed successfully");
    } else {
        ESP_LOGE(TAG, "Quick-Start failed");
    }
}