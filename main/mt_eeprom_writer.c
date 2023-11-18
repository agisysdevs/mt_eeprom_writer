#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

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
#define MAIN_PROCESSOR_RESET_PIN (GPIO_NUM_13)
#define STATUS_LED (GPIO_NUM_16)

// static const char *TAG = "EEPROM_UART";

void init_uart()
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);
}

void init_i2c()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000};
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t write_to_eeprom(const char *text)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EEPROM_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (EEPROM_START_ADDR >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, EEPROM_START_ADDR & 0xFF, true);

    size_t data_len = strlen(text);
    if (data_len < MIN_TEXT_LENGTH || data_len > MAX_TEXT_LENGTH)
    {
        // ESP_LOGE(TAG, "Error: Text string length out of range");
        i2c_cmd_link_delete(cmd);
        return ESP_FAIL;
    }

    i2c_master_write(cmd, (uint8_t *)text, data_len, true);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t read_from_eeprom(char *buffer)
{
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

    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void uart_task(void *pvParameters)
{
    char buffer[MAX_TEXT_LENGTH + 1]; // +1 for null-terminator
    char aux[MAX_TEXT_LENGTH + 1];    // to compare read/write results
    char ans[MAX_TEXT_LENGTH + 10];   // to send back the written result
    int index = 0;

    while (1)
    {
        uint8_t data[1];
        int len = uart_read_bytes(UART_NUM, data, 1, 10 / portTICK_PERIOD_MS);

        if (len > 0)
        {
            if (data[0] == '\r')
            {
                // CR received, process the received data
                buffer[index] = '\0';                // Null-terminate the string
                memcpy(aux, buffer, sizeof(buffer)); // clone the buffer to compare later

                if (index >= MIN_TEXT_LENGTH)
                {
                    esp_err_t write_result = write_to_eeprom(buffer);

                    if (write_result == ESP_OK)
                    {
                        if (strcmp(buffer, aux) == 0)
                        {
                            sprintf(ans, "OK: %s\n", buffer); // OK
                            uart_write_bytes(UART_NUM, ans, index + 5);
                        }
                        else
                        {
                            sprintf(ans, "E3: %s\n", buffer); // E3: written / read values do not match
                            uart_write_bytes(UART_NUM, ans, index + 5);
                        }
                    }
                    else
                    {
                        uart_write_bytes(UART_NUM, "E2\n", 3); // E2: read memory error
                    }
                }
                else
                {
                    sprintf(ans, "E1: %s\n", buffer); // E1: buffer too short
                    uart_write_bytes(UART_NUM, ans, index + 5);
                }

                // Reset buffer index
                index = 0;
            }
            else
            {
                // Add the received character to the buffer
                if (index < MAX_TEXT_LENGTH)
                {
                    buffer[index++] = data[0];
                }
                else
                {
                    sprintf(ans, "E0: %s\n", buffer); // E0: buffer overflow
                    uart_write_bytes(UART_NUM, ans, index + 5);
                    index = 0;
                }
            }
        }
    }
}

void app_main()
{
    init_uart();
    init_i2c();

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
}