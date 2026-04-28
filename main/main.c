#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_https_ota.h"
#include "mdns.h"
#include "driver/i2c_master.h"
#include "led_strip.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// python -m http.server 8070

static const char *TAG = "ROBOT_SUMO";


#define WIFI_SSID           "B17.17"  
#define WIFI_PASS           "12345678"   
// #define BLYNK_TOKEN         ""

#define LED_GPIO_PIN        48
#define LED_NUMBERS         1


#define I2C_SDA_PIN         4
#define I2C_SCL_PIN         5
#define MAX17043_ADDR       0x36
#define MAX17043_VCELL      0x02
#define MAX17043_SOC        0x04


#define UART_PORT_NUM       UART_NUM_1
#define UART_BAUD_RATE      115200
#define UART_TX_PIN         17   
#define UART_RX_PIN         18   
#define UART_BUF_SIZE       256

// Biến toàn cục
esp_mqtt_client_handle_t client;
volatile bool is_wifi_connected = false;
i2c_master_dev_handle_t max17043_handle;
led_strip_handle_t led_strip;

void uart_stm32_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // UART config
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "Đã khởi tạo UART1 (TX:%d, RX:%d) giao tiếp STM32.", UART_TX_PIN, UART_RX_PIN);
}

void uart_send_to_stm32(const char* cmd, int value) {
    char send_buf[32];

    int len = snprintf(send_buf, sizeof(send_buf), "%s:%d\n", cmd, value);

    uart_write_bytes(UART_PORT_NUM, send_buf, len);
    
    ESP_LOGI(TAG, ">> Send UART -> STM32: %s", send_buf);
}


void led_init(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO_PIN,
        .max_leds = LED_NUMBERS,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, 
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

void max17043_init() {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX17043_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &max17043_handle));
}

esp_err_t max17043_read_reg(uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    esp_err_t err = i2c_master_transmit_receive(max17043_handle, &reg, 1, data, 2, -1);
    if (err == ESP_OK) {
        *value = (data[0] << 8) | data[1];
    }
    return err;
}


void ota_task(void *pvParameter) {
    ESP_LOGI(TAG, "Bắt đầu OTA...");
    set_led_color(0, 0, 255); 

    esp_ip4_addr_t addr;
    addr.addr = 0;
    
    ESP_LOGI(TAG, "Get IP of qbs-laptop...");
    esp_err_t err = mdns_query_a("qbs-laptop", 3000, &addr);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cant find qbs-laptop. Canceling OTA.");
        set_led_color(255, 0, 0); 
        vTaskDelete(NULL);
        return;
    }

    char ip_str[16];
    esp_ip4addr_ntoa(&addr, ip_str, sizeof(ip_str));
    char ota_url[128];
    snprintf(ota_url, sizeof(ota_url), "http://%s:8070/test_iot.bin", ip_str);
    
    esp_http_client_config_t ota_client_config = {
        .url = ota_url,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_config = {
        .http_config = &ota_client_config,
    };

    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA successful! Rebooting...");
        set_led_color(0, 255, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(ret));
        set_led_color(255, 0, 0);
        vTaskDelete(NULL);
    }
}


static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Blynk MQTT connected!");
            set_led_color(0, 255, 0); 
            
            esp_mqtt_client_subscribe(client, "downlink/ds/POWER", 0);
            esp_mqtt_client_subscribe(client, "downlink/ds/MODECTRL", 0);
            esp_mqtt_client_subscribe(client, "downlink/ds/IntegerX", 0);
            esp_mqtt_client_subscribe(client, "downlink/ds/IntegerY", 0);
            esp_mqtt_client_subscribe(client, "downlink/ds/FW", 0);
            esp_mqtt_client_subscribe(client, "downlink/ds/BW", 0);
            esp_mqtt_client_subscribe(client, "downlink/ds/RL", 0);
            esp_mqtt_client_subscribe(client, "downlink/ds/RR", 0);
            esp_mqtt_client_subscribe(client, "downlink/ds/UPDATE_FW", 0);
            break;

        case MQTT_EVENT_DATA:
            {
                char topic_buf[64], data_buf[16];
                snprintf(topic_buf, sizeof(topic_buf), "%.*s", event->topic_len, event->topic);
                snprintf(data_buf, sizeof(data_buf), "%.*s", event->data_len, event->data);
                int val = atoi(data_buf);

                if (strcmp(topic_buf, "downlink/ds/UPDATE_FW") == 0) {
                    if (val == 1) {
                        ESP_LOGW(TAG, "Kích hoạt OTA...");
                        xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
                    }
                } 
                else if (strcmp(topic_buf, "downlink/ds/POWER") == 0)    uart_send_to_stm32("PWR", val);
                else if (strcmp(topic_buf, "downlink/ds/MODECTRL") == 0) uart_send_to_stm32("MODE", val);
                else if (strcmp(topic_buf, "downlink/ds/IntegerX") == 0) uart_send_to_stm32("JX", val);
                else if (strcmp(topic_buf, "downlink/ds/IntegerY") == 0) uart_send_to_stm32("JY", val);
                else if (strcmp(topic_buf, "downlink/ds/FW") == 0)       uart_send_to_stm32("FW", val);
                else if (strcmp(topic_buf, "downlink/ds/BW") == 0)       uart_send_to_stm32("BW", val);
                else if (strcmp(topic_buf, "downlink/ds/RL") == 0)       uart_send_to_stm32("RL", val);
                else if (strcmp(topic_buf, "downlink/ds/RR") == 0)       uart_send_to_stm32("RR", val);
            }
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            set_led_color(255, 100, 0); 
            break;
        default: break;
    }
}

void telemetry_task(void *pvParameters) {
    int current_mode = 0; 
    // uint16_t raw_soc;

    while (1) {
        current_mode = !current_mode; 
        char mode_data[8];
        snprintf(mode_data, sizeof(mode_data), "%d", current_mode);
        esp_mqtt_client_publish(client, "ds/MODE", mode_data, strlen(mode_data), 0, 0);
        
        vTaskDelay(pdMS_TO_TICKS(5000)); 
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        is_wifi_connected = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        is_wifi_connected = true; 
    }
}

void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    wifi_config_t wifi_config = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS, .threshold.authmode = WIFI_AUTH_WPA2_PSK },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://blynk.cloud",
        .broker.address.port = 1883,
        .credentials.username = "device",                  
        .credentials.authentication.password = CONFIG_BLYNK_TOKEN, 
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}


void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }

    led_init();
    set_led_color(255, 0, 0); 
    uart_stm32_init(); 
    // max17043_init();

    ESP_LOGI(TAG, "Initializing Wi-Fi...");
    wifi_init_sta();
    
    while (!is_wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
    
    mdns_init();
    mdns_hostname_set("robot-sumo");
    mqtt_app_start();
    
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
}