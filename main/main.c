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
#include "led_strip.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// python -m http.server 8070

static const char *TAG = "ROBOT_SUMO";


#define WIFI_SSID           ""  
#define WIFI_PASS           ""
// #define BLYNK_TOKEN         ""

#define LED_GPIO_PIN        48
#define LED_NUMBERS         1


#define UART_PORT_NUM       UART_NUM_1
#define UART_BAUD_RATE      115200
#define UART_TX_PIN         17   
#define UART_RX_PIN         18   
#define UART_BUF_SIZE       256

// Biến toàn cục
esp_mqtt_client_handle_t client;
volatile bool is_wifi_connected = false;
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
            esp_mqtt_client_subscribe(client, "downlink/ds/SPEED", 0);
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
                else if (strcmp(topic_buf, "downlink/ds/SPEED") == 0)    uart_send_to_stm32("SPEED", val);
            }
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            set_led_color(255, 100, 0); 
            break;
        default: break;
    }
}

void uart_rx_task(void *arg) {
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            
            // Xóa ký tự xuống dòng nếu có
            char *newline = strchr((char *)data, '\n');
            if (newline) *newline = '\0';
            newline = strchr((char *)data, '\r');
            if (newline) *newline = '\0';
            
            ESP_LOGI(TAG, "<< Receive UART <- STM32: %s", data);
            
            if (client != NULL) {
                // Kiểm tra xem có chuỗi RSTATE không (ví dụ: RSTATE:0,70,70)
                char *rstate_ptr = strstr((char *)data, "RSTATE:");
                if (rstate_ptr != NULL) {
                    int state = 0, ls = 0, rs = 0;
                    if (sscanf(rstate_ptr, "RSTATE:%d,%d,%d", &state, &ls, &rs) == 3) {
                        char buf[16];
                        
                        const char *state_str = "UNKNOWN";
                        switch (state) {
                            case 0: state_str = "NORMAL"; break;
                            case 1: state_str = "ESCAPE REVERSE"; break;
                            case 2: state_str = "ESCAPE TURN"; break;
                            case 3: state_str = "SEARCH"; break;
                            case 4: state_str = "IDLE"; break;
                            default: state_str = "UNKNOWN"; break;
                        }
                        esp_mqtt_client_publish(client, "ds/STATE_RX", state_str, 0, 1, 0);
                        
                        snprintf(buf, sizeof(buf), "%d", ls);
                        esp_mqtt_client_publish(client, "ds/LS_RX", buf, 0, 1, 0);
                        
                        snprintf(buf, sizeof(buf), "%d", rs);
                        esp_mqtt_client_publish(client, "ds/RS_RX", buf, 0, 1, 0);
                    }
                } 
                else {
                    // Fallback logic cũ
                    char *speed_ptr = strstr((char *)data, "SPEED:");
                    if (speed_ptr != NULL) {
                        esp_mqtt_client_publish(client, "ds/SPEED_RX", speed_ptr + 6, 0, 1, 0);
                    } else if (strlen((char *)data) > 0) {
                        // Nếu STM32 chỉ gửi một số
                        esp_mqtt_client_publish(client, "ds/SPEED_RX", (char *)data, 0, 1, 0);
                    }
                }
            }
        }
    }
    free(data);
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

    ESP_LOGI(TAG, "Initializing Wi-Fi...");
    wifi_init_sta();
    
    while (!is_wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
    
    mdns_init();
    mdns_hostname_set("robot-sumo");
    mqtt_app_start();
    
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);
}
