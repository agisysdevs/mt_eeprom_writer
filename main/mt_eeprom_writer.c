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
#define EEPROM_ADDR 0x50
#define EEPROM_START_ADDR 0x200
#define EEPROM_PAGE_SIZE 16
#define EEPROM_SIZE 0x4000
#define MAX_TEXT_LENGTH 256
#define MIN_TEXT_LENGTH 100
#define MAIN_PROCESSOR_RESET_PIN (GPIO_NUM_13)
#define STATUS_LED (GPIO_NUM_16)
#define POWER_MOSFET (GPIO_NUM_17)
#define LED_RED (GPIO_NUM_26)
#define LED_GREEN (GPIO_NUM_33)
#define LED_BLUE (GPIO_NUM_25)

#define BLINK_FAST 250  // 250 mseg
#define BLINK_SLOW 1000 // 1 seg
#define COLOR_RED BIT0
#define COLOR_GREEN BIT1
#define COLOR_BLUE BIT2
#define COLOR_YELLOW COLOR_GREEN | COLOR_RED
#define COLOR_MAGENTA COLOR_RED | COLOR_BLUE
#define COLOR_CYAN COLOR_GREEN | COLOR_BLUE
#define COLOR_WHITE COLOR_RED | COLOR_GREEN | COLOR_BLUE

#define SOFTWARE_VERSION "0.2"
#define SOFTWARE_DATE "20231127"

uint8_t color = 0;
bool blink = false;
bool blink_slow = false;
bool board_on = false;
bool err_state = false;

void led_set_in_progress(void);
void led_set_safe_to_remove(void);
void led_set_power_on(void);
void led_set_off(void);

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

void reset_target(void)
{
    gpio_set_direction(MAIN_PROCESSOR_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MAIN_PROCESSOR_RESET_PIN, 0);
}

void release_target(void)
{
    gpio_set_level(MAIN_PROCESSOR_RESET_PIN, 1);
    gpio_set_direction(MAIN_PROCESSOR_RESET_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(MAIN_PROCESSOR_RESET_PIN, GPIO_PULLUP_ONLY);
}

esp_err_t eeprom_write(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *data, uint16_t size)
{
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // char debug[200];
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | I2C_MASTER_WRITE, 1);
    if (EEPROM_SIZE > 0x100)
        i2c_master_write_byte(cmd, (eeaddress >> 8) & 0xFF, 1);
    i2c_master_write_byte(cmd, eeaddress & 0xFF, 1);

    uint16_t bytes_remaining = size;
    uint16_t current_address = eeaddress;
    uint16_t aux_size_0 = eeaddress % EEPROM_PAGE_SIZE;
    uint16_t first_write_size = EEPROM_PAGE_SIZE - aux_size_0;

    // sprintf(debug, "bytes_rem: %d, curr_add: %.4x, first write size: %d\n", bytes_remaining, current_address, first_write_size);
    // uart_write_bytes(UART_NUM, debug, strlen(debug));

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
            // sprintf(debug, "bytes_rem: %d, curr_add: %.4x\n", bytes_remaining, current_address);
            // uart_write_bytes(UART_NUM, debug, strlen(debug));
            cmd = i2c_cmd_link_create();

            // 5ms delay period to allow EEPROM to write the page
            // buffer to memory.
            vTaskDelay(50 / portTICK_PERIOD_MS);

            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (deviceaddress << 1) | I2C_MASTER_WRITE, 1);
            if (EEPROM_SIZE > 0x100)
                i2c_master_write_byte(cmd, (current_address >> 8) & 0xFF, 1);
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
    if (EEPROM_SIZE > 0x100)
        i2c_master_write_byte(cmd, (eeaddress >> 8) & 0xFF, 1);
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
    uint8_t buffer[EEPROM_SIZE];
    memset(buffer, value, EEPROM_SIZE);
    reset_target();
    gpio_set_level(STATUS_LED, 1);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    eeprom_write(EEPROM_ADDR, 0x00, buffer, EEPROM_SIZE);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    release_target();
    gpio_set_level(STATUS_LED, 0);
}

void read_mem(void)
{
    char memdump[EEPROM_SIZE], mdp[30];
    reset_target();
    gpio_set_level(STATUS_LED, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    eeprom_read(EEPROM_ADDR, 0x00, (uint8_t *)memdump, EEPROM_SIZE);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    release_target();
    gpio_set_level(STATUS_LED, 0);
    for (uint16_t i = 0; i < EEPROM_SIZE; i++)
    {
        if (i % 16 == 0)
            uart_write_bytes(UART_NUM, "\n", 1);
        sprintf(mdp, "%04x: %c ", i, memdump[i]);
        uart_write_bytes(UART_NUM, mdp, 9);
    }
    uart_write_bytes(UART_NUM, "\n", 1);
}

void detect_devices(void)
{
    esp_err_t res;
    char aux[20];
    uart_write_bytes(UART_NUM, "       0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n", 54);
    uart_write_bytes(UART_NUM, "0000:         ", 14);
    reset_target();
    gpio_set_level(STATUS_LED, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    for (uint8_t i = 3; i < 0x78; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
        if (i % 16 == 0)
        {
            sprintf(aux, "\n%.4x:", i);
            uart_write_bytes(UART_NUM, aux, strlen(aux));
        }
        if (res == 0)
        {
            sprintf(aux, " %.2x", i);
            uart_write_bytes(UART_NUM, aux, strlen(aux));
        }
        else
            uart_write_bytes(UART_NUM, " --", 3);
        i2c_cmd_link_delete(cmd);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    release_target();
    gpio_set_level(STATUS_LED, 0);
    uart_write_bytes(UART_NUM, "\n", 1);
}

void uart_task(void *pvParameters)
{
    char buffer[MAX_TEXT_LENGTH + 1]; // +1 for null-terminator
    char empty[MAX_TEXT_LENGTH + 1];  // to check if memory is empty
    char aux0[MAX_TEXT_LENGTH + 1];   // to check if memory is empty
    char aux1[MAX_TEXT_LENGTH + 1];   // to compare read/write results
    char aux2[MAX_TEXT_LENGTH + 1];   // to compare read/write results
    char ans[MAX_TEXT_LENGTH + 20];   // to send back the written result
    uint16_t index = 0;
    bool memfill = false;

    while (1)
    {
        uint8_t data[1];
        int len = uart_read_bytes(UART_NUM, data, 1, 10 / portTICK_PERIOD_MS);

        if (len > 0)
        {
            if (data[0] == '\r' || data[0] == '\n')
            {
                if (index == 0)
                {
                    // do nothing
                }
                else if (memcmp(buffer, "vv", 2) == 0)
                {
                    sprintf(ans, "V: %s D: %s\r\n", SOFTWARE_VERSION, SOFTWARE_DATE);
                    uart_write_bytes(UART_NUM, ans, strlen(ans)); // print version and date
                }
                else if (memcmp(buffer, "p1", 2) == 0)
                {
                    led_set_power_on();
                    gpio_set_level(POWER_MOSFET, 0);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    sprintf(ans, "P: ON\r\n");
                    uart_write_bytes(UART_NUM, ans, strlen(ans)); // print version and date
                    board_on = true;
                }
                else if (memcmp(buffer, "p0", 2) == 0)
                {
                    led_set_safe_to_remove();
                    gpio_set_level(POWER_MOSFET, 1);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    sprintf(ans, "P: OFF\r\n");
                    uart_write_bytes(UART_NUM, ans, strlen(ans)); // print version and date
                    board_on = false;
                }
                else
                {
                    if (board_on)
                    {
                        led_set_in_progress();
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
                        else if (memcmp(buffer, "ff", 2) == 0)
                        {
                            fill_mem(0xFF);
                            uart_write_bytes(UART_NUM, "ME OK\r\n", 7); // Memory erase OK
                        }
                        else if (memcmp(buffer, "mp", 2) == 0)
                        {
                            read_mem();
                        }
                        else if (memcmp(buffer, "dp", 2) == 0)
                        {
                            detect_devices();
                        }
                        else
                        {
                            // CR received, process the received data
                            buffer[index] = '\0';            // Null-terminate the string
                            memcpy(aux1, buffer, index + 1); // clone the buffer to compare later

                            reset_target();
                            gpio_set_level(STATUS_LED, 1);
                            vTaskDelay(250 / portTICK_PERIOD_MS);

                            if (index >= MIN_TEXT_LENGTH)
                            {
                                esp_err_t read_result = eeprom_read(EEPROM_ADDR, EEPROM_START_ADDR, (uint8_t *)aux0, index + 1);
                                if (read_result == ESP_OK)
                                {
                                    memset(empty, 0xFF, index);
                                    empty[index] = '\0';
                                    if (memcmp(empty, aux0, index) == 0)
                                    {
                                        esp_err_t write_result = eeprom_write(EEPROM_ADDR, EEPROM_START_ADDR, (uint8_t *)buffer, index + 1);
                                        if (write_result == ESP_OK)
                                        {
                                            vTaskDelay(100 / portTICK_PERIOD_MS);
                                            esp_err_t second_read_result = eeprom_read(EEPROM_ADDR, EEPROM_START_ADDR, (uint8_t *)aux2, index + 1);
                                            // aux2[index] = '\0';
                                            if (second_read_result == ESP_OK)
                                            {
                                                if (strcmp(aux2, aux1) == 0)
                                                {
                                                    sprintf(ans, "OK: %s\r\n", aux2); // OK
                                                    uart_write_bytes(UART_NUM, ans, strlen(ans));
                                                }
                                                else
                                                {
                                                    sprintf(ans, "E6: %s\r\n", aux2); // E6: written / read values do not match
                                                    uart_write_bytes(UART_NUM, ans, strlen(ans));
                                                }
                                            }
                                            else
                                            {
                                                sprintf(ans, "E5: %02d - %s\r\n", second_read_result, aux2); // E5: re-read memory error
                                                uart_write_bytes(UART_NUM, ans, strlen(ans));
                                            }
                                        }
                                        else
                                        {
                                            sprintf(ans, "E4: %02d\r\n", write_result); // E4: write memory error
                                            uart_write_bytes(UART_NUM, ans, strlen(ans));
                                        }
                                    }
                                    else
                                    {
                                        aux0[index] = '\0';               // terminate aux0 no matter what just in case i am writing a string smaller than the one recorded
                                        sprintf(ans, "E3: %s\r\n", aux0); // E3: memory not empty
                                        uart_write_bytes(UART_NUM, ans, strlen(ans));
                                    }
                                }
                                else
                                {
                                    sprintf(ans, "E2: %02d\r\n", read_result); // E2: read memory error
                                    uart_write_bytes(UART_NUM, ans, strlen(ans));
                                }
                            }
                            else
                            {
                                sprintf(ans, "E1: %s\r\n", buffer); // E1: buffer too short
                                uart_write_bytes(UART_NUM, ans, strlen(ans));
                            }

                            vTaskDelay(250 / portTICK_PERIOD_MS);
                            release_target();
                            gpio_set_level(STATUS_LED, 0);
                        }
                        led_set_power_on();
                    }
                    else
                    {
                        sprintf(ans, "E7\r\n"); // E7: board is off
                        uart_write_bytes(UART_NUM, ans, strlen(ans));
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
                    sprintf(ans, "E0: %s\r\n", buffer); // E0: buffer overflow
                    uart_write_bytes(UART_NUM, ans, strlen(ans));
                    index = 0;
                }
            }
        }
    }
}

void led_task(void *pvParameters)
{
    gpio_set_direction(STATUS_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED, 0);
    gpio_set_level(LED_RED, 0);
    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(LED_BLUE, 0);

    while (1)
    {
        if (color & COLOR_RED)
            gpio_set_level(LED_RED, 1);
        else
            gpio_set_level(LED_RED, 0);
        if (color & COLOR_GREEN)
            gpio_set_level(LED_GREEN, 1);
        else
            gpio_set_level(LED_GREEN, 0);
        if (color & COLOR_BLUE)
            gpio_set_level(LED_BLUE, 1);
        else
            gpio_set_level(LED_BLUE, 0);
        if (blink)
        {
            vTaskDelay((blink_slow ? BLINK_SLOW : BLINK_FAST) / portTICK_PERIOD_MS);
            gpio_set_level(LED_RED, 0);
            gpio_set_level(LED_GREEN, 0);
            gpio_set_level(LED_BLUE, 0);
            vTaskDelay((blink_slow ? BLINK_SLOW : BLINK_FAST) / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}

void led_set_power_on(void)
{
    color = COLOR_RED;
    blink = false;
}
void led_set_safe_to_remove(void)
{
    color = COLOR_GREEN;
    blink = false;
}
void led_set_in_progress(void)
{
    color = COLOR_YELLOW;
    blink = true;
}
void led_set_off(void)
{
    color = 0;
    blink = false;
}

void app_main()
{
    init_uart();
    init_i2c();

    gpio_set_direction(POWER_MOSFET, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(POWER_MOSFET, GPIO_PULLUP_ONLY);
    gpio_set_level(POWER_MOSFET, 1);

    release_target();
    led_set_safe_to_remove();

    xTaskCreate(led_task, "led_task", 4096, NULL, 2, NULL);
    xTaskCreate(uart_task, "uart_task", 20480, NULL, 1, NULL);
}

// test pattern
// PN 1 "00205675" CR 2 "A" PY 1 "08" PW 1 "25" SN 2 "0000000000620089" MI 1 "S" RI 1 "R" MAC 1 "00:10:52:D2:43:FB"
// PN 1 "00205675" CR 2 "A" PY 1 "08" PW 1 "25" SN 2 "0000000000620456" MI 1 "S" RI 1 "R" MAC 1 "00:10:52:D2:44:FF"
// 0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ9876543210zyxwvutsrqponmlkjihgfedcbaZYXWVUTSRQPONMLKJIHGFEDCBA
