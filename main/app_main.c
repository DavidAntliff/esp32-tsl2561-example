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
 * @file app_main.c
 * @brief Example application for the TSL2561 Light-to-Digital Converter.
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
#include "tsl2561.h"

#define TAG "app"

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

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

void tsl2561_task(void * pvParameter)
{
    // Set up I2C
    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = CONFIG_TSL2561_I2C_ADDRESS;

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_num, address);
    smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);

    // Set up the TSL2561 device
    tsl2561_info_t * tsl2561_info = tsl2561_malloc();
    tsl2561_init(tsl2561_info, smbus_info);

    // Set sensor integration time and gain
    tsl2561_set_integration_time_and_gain(tsl2561_info, TSL2561_INTEGRATION_TIME_402MS, TSL2561_GAIN_1X);
    //tsl2561_set_integration_time_and_gain(tsl2561_info, TSL2561_INTEGRATION_TIME_402MS, TSL2561_GAIN_16X);

    while (1)
    {
        tsl2561_visible_t visible = 0;
        tsl2561_infrared_t infrared = 0;
        tsl2561_read(tsl2561_info, &visible, &infrared);

        printf("\nFull spectrum: %d\n", visible + infrared);
        printf("Infrared:      %d\n", infrared);
        printf("Visible:       %d\n", visible);
        printf("Lux:           %d\n", tsl2561_compute_lux(tsl2561_info, visible, infrared));

        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void app_main()
{
    xTaskCreate(&tsl2561_task, "tsl2561_task", 2048, NULL, 5, NULL);

    // I2C/SMBus Test application
    //extern void test_smbus_task(void * pvParameter);
    //xTaskCreate(&test_smbus_task, "test_smbus_task", 2048, NULL, 5, NULL);
}

