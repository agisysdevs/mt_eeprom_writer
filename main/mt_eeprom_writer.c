#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define UART_NUM UART_NUM_0
#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 21
#define EEPROM_ADDR 0x70
#define EEPROM_START_ADDR 0x200
#define MAX_TEXT_LENGTH 256
#define MIN_TEXT_LENGTH 100

// static const char *TAG = "EEPROM_UART";

void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);
}

void init_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t write_to_eeprom(const char *text) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EEPROM_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (EEPROM_START_ADDR >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, EEPROM_START_ADDR & 0xFF, true);

    size_t data_len = strlen(text);
    if (data_len < MIN_TEXT_LENGTH || data_len > MAX_TEXT_LENGTH) {
        // ESP_LOGE(TAG, "Error: Text string length out of range");
        i2c_cmd_link_delete(cmd);
        return ESP_FAIL;
    }

    i2c_master_write(cmd, (uint8_t *)text, data_len, true);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "Error: Failed to write to EEPROM");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t read_from_eeprom(char *buffer) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EEPROM_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (EEPROM_START_ADDR >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, EEPROM_START_ADDR & 0xFF, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EEPROM_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, (uint8_t *)buffer, MAX_TEXT_LENGTH, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "Error: Failed to read from EEPROM");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void app_main() {
    char buffer[MAX_TEXT_LENGTH + 1]; // +1 for null-terminator
    init_uart();
    init_i2c();

    while (1) {
        int len = uart_read_bytes(UART_NUM, (uint8_t *)buffer, sizeof(buffer), 1000 / portTICK_PERIOD_MS);
        if (len > 0) {
            buffer[len] = '\0'; // Null-terminate the received string
            // ESP_LOGI(TAG, "Received text: %s", buffer);

            esp_err_t write_result = write_to_eeprom(buffer);
            if (write_result == ESP_OK) {
                memset(buffer, 0, sizeof(buffer)); // Clear buffer before reading
                esp_err_t read_result = read_from_eeprom(buffer);
                if (read_result == ESP_OK && strcmp(buffer, "buffer") == 0) {
                    // ESP_LOGI(TAG, "Memory written and read successfully!");
                    uart_write_bytes(UART_NUM, "OK", 2);
                } else {
                    // ESP_LOGE(TAG, "Error: Memory read failed or data corrupted");
                    uart_write_bytes(UART_NUM, "error (memory read failed or data corrupted)", 40);
                }
            } else {
                // ESP_LOGE(TAG, "Error: Memory write failed");
                uart_write_bytes(UART_NUM, "error (memory write failed)", 27);
            }
        }
    }
}
