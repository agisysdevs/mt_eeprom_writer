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
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define EEPROM_ADDR 0x52       // 0x50
#define EEPROM_START_ADDR 0x00 // 0x200
#define EEPROM_PAGE_SIZE 16
#define MAX_TEXT_LENGTH 256
#define MIN_TEXT_LENGTH 10 // 100
#define MAIN_PROCESSOR_RESET_PIN (GPIO_NUM_13)
#define STATUS_LED (GPIO_NUM_16)

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
        .scl_io_num = I2C_SCL_PIN,
        .master.clk_speed = 100000};
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t eeprom_write(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *data, size_t size)
{
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | I2C_MASTER_WRITE, 1);
    // i2c_master_write_byte(cmd, eeaddress >> 8, 1);
    i2c_master_write_byte(cmd, eeaddress & 0xFF, 1);

    int bytes_remaining = size;
    int current_address = eeaddress;
    int first_write_size = ((EEPROM_PAGE_SIZE - 1) - eeaddress % (EEPROM_PAGE_SIZE - 1)) + 1;
    if (eeaddress % (EEPROM_PAGE_SIZE - 1) == 0 && eeaddress != 0)
        first_write_size = 1;
    if (bytes_remaining <= first_write_size)
    {
        i2c_master_write(cmd, data, bytes_remaining, 1);
    }
    else
    {
        i2c_master_write(cmd, data, first_write_size, 1);
        bytes_remaining -= first_write_size;
        current_address += first_write_size;
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK)
            return ret;
        while (bytes_remaining > 0)
        {
            cmd = i2c_cmd_link_create();

            // 2ms delay period to allow EEPROM to write the page
            // buffer to memory.
            vTaskDelay(20 / portTICK_PERIOD_MS);

            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (deviceaddress << 1) | I2C_MASTER_WRITE, 1);
            // i2c_master_write_byte(cmd, current_address >> 8, 1);
            i2c_master_write_byte(cmd, current_address & 0xFF, 1);
            if (bytes_remaining <= EEPROM_PAGE_SIZE)
            {
                i2c_master_write(cmd, data + (size - bytes_remaining), bytes_remaining, 1);
                bytes_remaining = 0;
            }
            else
            {
                i2c_master_write(cmd, data + (size - bytes_remaining), EEPROM_PAGE_SIZE, 1);
                bytes_remaining -= EEPROM_PAGE_SIZE;
                current_address += EEPROM_PAGE_SIZE;
            }
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            if (ret != ESP_OK)
                return ret;
        }
    }

    return ret;
}

esp_err_t eeprom_read(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | I2C_MASTER_WRITE, 1);
    // i2c_master_write_byte(cmd, eeaddress << 8, 1);
    i2c_master_write_byte(cmd, eeaddress & 0xFF, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | I2C_MASTER_READ, 1);

    if (size > 1)
    {
        i2c_master_read(cmd, data, size - 1, 0);
    }
    i2c_master_read_byte(cmd, data + size - 1, 1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void fill_mem(uint8_t value)
{
    uint8_t buffer[MAX_TEXT_LENGTH];
    memset(buffer, value, MAX_TEXT_LENGTH);
    // write_to_eeprom(buffer, 0x00, MAX_TEXT_LENGTH);
    eeprom_write(EEPROM_ADDR, 0x00, buffer, MAX_TEXT_LENGTH);
}

void read_mem(void)
{
    char memdump[0x100], mdp[7];
    // read_from_eeprom((uint8_t *)memdump, 0x00, 0xFF);
    eeprom_read(EEPROM_ADDR, 0x00, (uint8_t *)memdump, MAX_TEXT_LENGTH);
    for (uint16_t i = 0; i < 0x100; i++)
    {
        if (i % 16 == 0)
            uart_write_bytes(UART_NUM, "\n", 1);
        sprintf(mdp, "%02x: %c ", i, memdump[i]);
        uart_write_bytes(UART_NUM, mdp, 7);
    }
    uart_write_bytes(UART_NUM, "\n", 1);
}

void uart_task(void *pvParameters)
{
    char buffer[MAX_TEXT_LENGTH + 1]; // +1 for null-terminator
    char empty[MAX_TEXT_LENGTH + 1];  // to check if memory is empty
    char aux0[MAX_TEXT_LENGTH + 1];   // to check if memory is empty
    char aux1[MAX_TEXT_LENGTH + 1];   // to compare read/write results
    char ans[MAX_TEXT_LENGTH + 10];   // to send back the written result
    uint16_t index = 0;
    bool memfill = false;

    memset(empty, 0, MAX_TEXT_LENGTH + 1);

    // while (1)
    // {
    //     esp_err_t res;
    //     printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    //     printf("00:         ");
    //     for (uint8_t i = 3; i < 0x78; i++)
    //     {
    //         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //         i2c_master_start(cmd);
    //         i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
    //         i2c_master_stop(cmd);

    //         res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
    //         if (i % 16 == 0)
    //             printf("\n%.2x:", i);
    //         if (res == 0)
    //             printf(" %.2x", i);
    //         else
    //             printf(" --");
    //         i2c_cmd_link_delete(cmd);
    //     }
    //     printf("\n");
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    // write_to_eeprom((uint8_t *)"zzzzzzzzzzzzzzzz", 0x20, 16);

    // read_mem();

    while (1)
    {
        uint8_t data[1];
        int len = uart_read_bytes(UART_NUM, data, 1, 10 / portTICK_PERIOD_MS);

        if (len > 0)
        {
            if (data[0] == '\r')
            {
                if (memcmp(buffer, "mf", 2) == 0)
                {
                    memfill = true;
                }
                else if (memfill == true)
                {
                    if (index == 1)
                    {
                        fill_mem(buffer[0]);
                    }
                    memfill = false;
                }
                else if (memcmp(buffer, "mp", 2) == 0)
                {
                    read_mem();
                }
                else
                {
                    // CR received, process the received data
                    buffer[index] = '\0';            // Null-terminate the string
                    memcpy(aux1, buffer, index + 1); // clone the buffer to compare later

                    if (index >= MIN_TEXT_LENGTH)
                    {
                        esp_err_t read_result = eeprom_read(EEPROM_ADDR, EEPROM_START_ADDR, (uint8_t *)aux0, index);
                        if (read_result == ESP_OK)
                        {
                            // if (strcmp(empty, aux0) == 0)
                            // {
                            if (strcmp(empty, aux0) != 0)
                            {
                                aux0[index] = '\0';             // terminate aux0 no matter what just in case i am writing a string smaller than the one recorded
                                sprintf(ans, "E3: %s\n", aux0); // E3: memory not empty
                                uart_write_bytes(UART_NUM, ans, strlen(aux0) + 5);
                            }

                            // sprintf(ans, "O3: %s\n", buffer); // output 0
                            // uart_write_bytes(UART_NUM, ans, index + 5);
                            esp_err_t write_result = eeprom_write(EEPROM_ADDR, EEPROM_START_ADDR, (uint8_t *)buffer, index);

                            if (write_result == ESP_OK)
                            {
                                read_result = eeprom_read(EEPROM_ADDR, EEPROM_START_ADDR, (uint8_t *)buffer, index);
                                buffer[index] = '\0';
                                if (read_result == ESP_OK)
                                {
                                    if (strcmp(buffer, aux1) == 0)
                                    {
                                        sprintf(ans, "OK: %s\n", buffer); // OK
                                        uart_write_bytes(UART_NUM, ans, index + 5);
                                    }
                                    else
                                    {
                                        sprintf(ans, "E6: %s\n", buffer); // E6: written / read values do not match
                                        uart_write_bytes(UART_NUM, ans, index + 5);
                                    }
                                }
                                else
                                {
                                    sprintf(ans, "E5: %02d\n", read_result); // E5: re-read memory error
                                    uart_write_bytes(UART_NUM, ans, 8);
                                }
                            }
                            else
                            {
                                uart_write_bytes(UART_NUM, "E4\n", 3); // E4: write memory error
                            }
                            // }
                            // else
                            // {
                            //     sprintf(ans, "E3: %s\n", aux0); // E3: memory not empty
                            //     uart_write_bytes(UART_NUM, ans, index + 5);
                            // }
                        }
                        else
                        {
                            sprintf(ans, "E2: %02d\n", read_result); // E2: read memory error
                            uart_write_bytes(UART_NUM, ans, 8);
                        }
                    }
                    else
                    {
                        sprintf(ans, "E1: %s\n", buffer); // E1: buffer too short
                        uart_write_bytes(UART_NUM, ans, index + 5);
                    }
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