/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file test_tsl2561_smbus.c
 * @brief Test application for the TSL2561 Light-to-Digital Converter.
 * This program performs a series of I2C and SMBus transactions on a connected
 * TSL2561 device, and reports back the results for manual verification.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "smbus.h"

#define TAG "test"

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

// Register addresses
#define REG_CONTROL         0x00
#define REG_TIMING          0x01
#define REG_THRESHLOWLOW    0x02
#define REG_THRESHLOWHIGH   0x03
#define REG_THRESHHIGHLOW   0x04
#define REG_THRESHHIGHHIGH  0x05
#define REG_INTERRUPT       0x06
#define REG_ID              0x0A
#define REG_DATA0LOW        0x0C
#define REG_DATA0HIGH       0x0D
#define REG_DATA1LOW        0x0E
#define REG_DATA1HIGH       0x0F

// The following values are bitwise ORed with register addresses to create a command value
#define SMB_BLOCK           0x10  // Transaction to use Block Write/Read protocol
#define SMB_WORD            0x20  // Transaction to use Word Write/Read protocol
#define SMB_CLEAR           0x40  // Clear any pending interrupt (self-clearing)
#define SMB_COMMAND         0x80  // Select command register

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

void test_smbus_task(void * pvParameter)
{
    i2c_master_init();

    i2c_port_t i2c_num = I2C_MASTER_NUM;
    i2c_address_t address = CONFIG_TSL2561_I2C_ADDRESS;

    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_num, address);
    smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);

    // SMBus Quick Commands:
//    ESP_LOGI(TAG, "smbus_quick:");
//    ESP_ERROR_CHECK(smbus_quick(smbus_info, true));
//    ESP_ERROR_CHECK(smbus_quick(smbus_info, false));

    // Send Byte
    ESP_LOGI(TAG, "smbus_send_byte:");
    ESP_ERROR_CHECK(smbus_send_byte(smbus_info, SMB_COMMAND));
    ESP_ERROR_CHECK(smbus_send_byte(smbus_info, 0x03));  // power up

    // Receive Byte
    ESP_LOGI(TAG, "smbus_receive_byte:");
    uint8_t status = 0;
    ESP_ERROR_CHECK(smbus_send_byte(smbus_info, SMB_COMMAND));
    ESP_ERROR_CHECK(smbus_receive_byte(smbus_info, &status));
    ESP_LOGI(TAG, "status 0x%02x (expect 0x03)", status);

    // Write Byte
    ESP_LOGI(TAG, "smbus_write_byte:");
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONTROL | SMB_COMMAND, 0x00));  // power down
    ESP_ERROR_CHECK(smbus_receive_byte(smbus_info, &status));
    ESP_LOGI(TAG, "status 0x%02x (expect 0x00)", status);
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONTROL | SMB_COMMAND, 0x03));  // power up

    // Read Byte
    ESP_LOGI(TAG, "smbus_read_byte:");
    uint8_t timing = 0;
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_TIMING | SMB_COMMAND, 0x0d));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_TIMING | SMB_COMMAND, &timing));
    ESP_LOGI(TAG, "timing 0x%02x (expect 0x0d)", timing);

    // Write Word
    ESP_LOGI(TAG, "smbus_write_word:");
    ESP_ERROR_CHECK(smbus_write_word(smbus_info, REG_CONTROL | SMB_COMMAND | SMB_WORD, 0x1102));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_CONTROL | SMB_COMMAND, &status));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_TIMING | SMB_COMMAND, &timing));
    ESP_LOGI(TAG, "status 0x%02x (expect 0x02)", status);
    ESP_LOGI(TAG, "timing 0x%02x (expect 0x11)", timing);

    // Read Word
    ESP_LOGI(TAG, "smbus_read_word:");
    ESP_ERROR_CHECK(smbus_write_word(smbus_info, REG_CONTROL | SMB_COMMAND | SMB_WORD, 0x0103));
    uint16_t word = 0;
    ESP_ERROR_CHECK(smbus_read_word(smbus_info, REG_CONTROL | SMB_COMMAND | SMB_WORD, &word));
    ESP_LOGI(TAG, "word[0] 0x%02x (expect 0x03)", word & 0xff);
    ESP_LOGI(TAG, "word[1] 0x%02x (expect 0x01)", (word >> 8) & 0xff);

    // Block Write
    ESP_LOGI(TAG, "smbus_write_block:");
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONTROL | SMB_COMMAND, 0x00));  // power down
    uint8_t block_write_data[6] = { 0x03, 0x02, 0xAA, 0x55, 0x00, 0x0F };
    ESP_ERROR_CHECK(smbus_write_block(smbus_info, REG_CONTROL | SMB_COMMAND | SMB_BLOCK, block_write_data, 6));
    uint8_t block_write_actual_data[6] = { 0 };
    for (size_t i = 0; i < 6; ++i)
    {
        ESP_ERROR_CHECK(smbus_read_byte(smbus_info, i | SMB_COMMAND, &block_write_actual_data[i]));
        ESP_LOGI(TAG, "actual[%d] 0x%02x (expect 0x%02x)", i, block_write_actual_data[i], block_write_data[i]);
    }

    // Block Read - see loop below

    // I2C Block Write (no SMB_BLOCK)
    // On the TSL2561 this does not auto-increment the register address, so all writes go to the same register
    ESP_LOGI(TAG, "smbus_i2c_write_block:");
    uint8_t i2c_block_write_data[4] = { 0x45, 0xA2, 0x6E, 0x09 };
    ESP_ERROR_CHECK(smbus_i2c_write_block(smbus_info, REG_THRESHLOWLOW | SMB_COMMAND, i2c_block_write_data, 4));
    for (size_t i = 0; i < 4; ++i)
    {
        uint8_t val = 0;
        ESP_ERROR_CHECK(smbus_read_byte(smbus_info, (REG_THRESHLOWLOW + i) | SMB_COMMAND, &val));
        // first byte is the last byte written, other bytes are from previous Block Write
        ESP_LOGI(TAG, "actual[%d] 0x%02x (expect 0x%02x)", i, val, i == 0 ? i2c_block_write_data[3] : block_write_data[2 + i]);
    }

    // I2C Block Read (Combined Format) (no SMB_BLOCK)
    // On the TSL2561 this auto-increments the register address, so reads from all four Threshold registers in sequence.
    ESP_LOGI(TAG, "smbus_i2c_read_block:");
    uint8_t i2c_block_read_data[4] = { 0 };
    ESP_ERROR_CHECK(smbus_i2c_read_block(smbus_info, REG_THRESHLOWLOW | SMB_COMMAND, i2c_block_read_data, 4));
    for (size_t i = 0; i < 4; ++i)
    {
        // first byte is the last byte written, other bytes are from previous Block Write
        ESP_LOGI(TAG, "actual[%d] 0x%02x (expect 0x%02x)", i, i2c_block_read_data[i], i ==0 ? i2c_block_write_data[3] : block_write_data[2 + i]);
    }


    // Test sensor reading
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONTROL | SMB_COMMAND, 0x03));  // power up
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_TIMING | SMB_COMMAND, 0x02));   // 402ms integration time
    vTaskDelay(500 / portTICK_RATE_MS);

    // Block Read - only supported by the ADC registers with special command 0x9B
    ESP_LOGI(TAG, "smbus_read_block:");
    uint8_t block_read_actual_data[4] = { 0 };
    uint8_t len = 4;
    ESP_ERROR_CHECK(smbus_read_block(smbus_info, 0x9B, block_read_actual_data, &len));

    // compare against Read Word:
    uint16_t ch0 = 0;
    uint16_t ch1 = 0;
    ESP_ERROR_CHECK(smbus_read_word(smbus_info, REG_DATA0LOW | SMB_COMMAND | SMB_WORD, &ch0));
    ESP_ERROR_CHECK(smbus_read_word(smbus_info, REG_DATA1LOW | SMB_COMMAND | SMB_WORD, &ch1));

    ESP_LOGI(TAG, "DATA0LOW  block 0x%02x, word 0x%02x", block_read_actual_data[0], ch0 & 0xff);
    ESP_LOGI(TAG, "DATA0HIGH block 0x%02x, word 0x%02x", block_read_actual_data[1], (ch0 >> 8) & 0xff);
    ESP_LOGI(TAG, "DATA1LOW  block 0x%02x, word 0x%02x", block_read_actual_data[2], ch1 & 0xff);
    ESP_LOGI(TAG, "DATA1HIGH block 0x%02x, word 0x%02x", block_read_actual_data[3], (ch1 >> 8) & 0xff);

    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONTROL | SMB_COMMAND, 0x00));  // power down

    ESP_LOGW(TAG, "Test complete.");

    while(1)
        ;
}
