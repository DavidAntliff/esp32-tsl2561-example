#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "smbus.h"

#define TAG "tsl2561"

#define BLUE_LED_GPIO (GPIO_NUM_2)
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN 0   // disabled
#define I2C_MASTER_RX_BUF_LEN 0   // disabled
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL
#define WRITE_BIT  I2C_MASTER_WRITE
#define READ_BIT   I2C_MASTER_READ
#define ACK_CHECK_EN   true     // I2C master will check ack from slave
#define ACK_CHECK_DIS  false    // I2C master will not check ack from slave
#define ACK_VAL        0x0      // I2C ack value
#define NACK_VAL       0x1      // I2C nack value

#define REG_COMMAND   0x80
#define REG_CONTROL   0x00
#define REG_TIMING    0x01
#define REG_ID        0x0A
#define REG_DATA0LOW  0x0C
#define REG_DATA0HIGH 0x0D
#define REG_DATA1LOW  0x0E
#define REG_DATA1HIGH 0x0F

#define SMB_BLOCK 0x10
#define SMB_WORD 0x20

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

static esp_err_t write_byte_data(i2c_port_t i2c_num, uint8_t address, uint8_t command, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | WRITE_BIT, true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "I2C write_byte_data error");
        return ret;
    }
    return ESP_OK;
}

static int read_block_data(i2c_port_t i2c_num, uint8_t address, uint8_t command, uint8_t * data, uint8_t len)
{
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, address << 1 | WRITE_BIT, true);
//    i2c_master_write_byte(cmd, command, true);
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, address << 1 | READ_BIT, true);
//    for (size_t i = 0; i < len; ++i)
//    {
//        i2c_master_read_byte(cmd, &data[i], i < len - 1 ? ACK_VAL : NACK_VAL);
//    }
//    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    if (ret == ESP_FAIL) {
//        ESP_LOGE(TAG, "I2C read_block_data error");
//        return -1;
//    }
//
//    ESP_LOGD(TAG, "slave has %d bytes for reading", slave_len);
//
//    if (slave_len > 32)
//    {
//        ESP_LOGW(TAG, "slave length %d exceeds 32 - abort read", slave_len);
//        slave_len = 0;
//    }
//
//    cmd = i2c_cmd_link_create();
//    for (uint8_t i = 0; i < slave_len; ++i)
//    {
//        i2c_master_read_byte(cmd, &data[i], i < slave_len - 1 ? ACK_VAL : NACK_VAL);
//    }
//    i2c_master_stop(cmd);
//    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    if (ret == ESP_FAIL) {
//        ESP_LOGE(TAG, "I2C read_block_data (2) error");
//        return -1;
//    }
//    return slave_len;
    return -1;
}

static esp_err_t read_word(i2c_port_t i2c_num, uint8_t address, uint8_t command, uint8_t * data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | WRITE_BIT, true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | READ_BIT, true);
    i2c_master_read_byte(cmd, &data[0], ACK_VAL);   // data byte low
    i2c_master_read_byte(cmd, &data[1], NACK_VAL);  // data byte high
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "I2C read_word error");
        return ret;
    }
    return ESP_OK;
}

static esp_err_t read_byte(i2c_port_t i2c_num, uint8_t address, uint8_t command, uint8_t * data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | WRITE_BIT, true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | READ_BIT, true);
    i2c_master_read_byte(cmd, data, NACK_VAL);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "I2C read_byte error");
        return ret;
    }
    return ESP_OK;
}

static esp_err_t receive_byte(i2c_port_t i2c_num, uint8_t address, uint8_t * data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | READ_BIT, true);
    i2c_master_read_byte(cmd, data, NACK_VAL);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "I2C receive_byte error");
        return ret;
    }
    return ESP_OK;
}

void tsl2561_task(void * pvParameter)
{
    i2c_master_init();

    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = CONFIG_TSL2561_I2C_ADDRESS;

    uint8_t id = 0;
    read_byte(i2c_num, address, REG_ID | REG_COMMAND, &id);
    printf("ID: 0x%02x\n", id);

    uint8_t powered = 0;
    //read_byte(i2c_num, address, REG_CONTROL | REG_COMMAND, &powered);
    //printf("Powered: 0x%02x\n", powered);

    // Power on sensor by writing 0x03 to Control register (via Command register)
    write_byte_data(i2c_num, address, REG_CONTROL | REG_COMMAND, 0x03);

    //read_byte(i2c_num, address, REG_CONTROL | REG_COMMAND, &powered);
    receive_byte(i2c_num, address, &powered);
    printf("Powered: 0x%02x\n", powered & 0x03);

    // Set integration time to 402ms (0x02) via timing register
    write_byte_data(i2c_num, address, REG_TIMING | REG_COMMAND, 0x02);

    // Wait at least 0.5 seconds
    vTaskDelay(500 / portTICK_RATE_MS);

    while (1)
    {
        // Read DATA0LOW and DATA0HIGH together
        //uint8_t data0[2] = {0};
        //read_block_data(i2c_num, address, REG_DATA0LOW | REG_COMMAND, data0, sizeof(data0));
        uint8_t data0[2] = {0};
        read_word(i2c_num, address, REG_DATA0LOW | REG_COMMAND | SMB_WORD, data0);
        //read_word(i2c_num, address, REG_DATA0LOW | REG_COMMAND, data0);
        //read_byte(i2c_num, address, REG_DATA0LOW | REG_COMMAND, &data0[0]);
        //read_byte(i2c_num, address, REG_DATA0HIGH | REG_COMMAND, &data0[1]);

        uint8_t data1[2] = {0};
        read_word(i2c_num, address, REG_DATA1LOW | REG_COMMAND | SMB_WORD, data1);
        //read_word(i2c_num, address, REG_DATA1LOW | REG_COMMAND, data1);
        //read_byte(i2c_num, address, REG_DATA1LOW | REG_COMMAND, &data1[0]);
        //read_byte(i2c_num, address, REG_DATA1HIGH | REG_COMMAND, &data1[1]);

        uint16_t ch0 = (data0[1] << 8) + data0[0];
        uint16_t ch1 = (data1[1] << 8) + data1[0];

        printf("Full spectrum: %d\n", ch0);
        printf("Infrared:      %d\n", ch1);
        printf("Visible:       %d\n\n", ch0 - ch1);

        gpio_set_level(BLUE_LED_GPIO, 0);
        vTaskDelay(1000 / portTICK_RATE_MS);
        gpio_set_level(BLUE_LED_GPIO, 1);
        vTaskDelay(1000 / portTICK_RATE_MS);
	}
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
//    ESP_ERROR_CHECK(smbus_quick(smbus_info, true));
//    ESP_ERROR_CHECK(smbus_quick(smbus_info, false));

    // Send Byte
    ESP_ERROR_CHECK(smbus_send_byte(smbus_info, REG_COMMAND));
    ESP_ERROR_CHECK(smbus_send_byte(smbus_info, 0x03));  // power up

    // Receive Byte
    uint8_t status = 0;
    ESP_ERROR_CHECK(smbus_send_byte(smbus_info, REG_COMMAND));
    ESP_ERROR_CHECK(smbus_receive_byte(smbus_info, &status));
    ESP_LOGI(TAG, "status 0x%02x (expect 0x03)", status);

    // Write Byte
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONTROL | REG_COMMAND, 0x00));  // power down
    ESP_ERROR_CHECK(smbus_receive_byte(smbus_info, &status));
    ESP_LOGI(TAG, "status 0x%02x (expect 0x00)", status);
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONTROL | REG_COMMAND, 0x03));  // power up

    // Read Byte
    uint8_t timing = 0;
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_TIMING | REG_COMMAND, 0x0d));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_TIMING | REG_COMMAND, &timing));
    ESP_LOGI(TAG, "timing 0x%02x (expect 0xd0)", timing);

    // Write Word
    ESP_ERROR_CHECK(smbus_write_word(smbus_info, REG_CONTROL | REG_COMMAND | SMB_WORD, 0x1102));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_CONTROL | REG_COMMAND, &status));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_TIMING | REG_COMMAND, &timing));
    ESP_LOGI(TAG, "status 0x%02x (expect 0x02)", status);
    ESP_LOGI(TAG, "timing 0x%02x (expect 0x11)", timing);

    // Read Word
    ESP_ERROR_CHECK(smbus_write_word(smbus_info, REG_CONTROL | REG_COMMAND | SMB_WORD, 0x0103));
    uint16_t word = 0;
    ESP_ERROR_CHECK(smbus_read_word(smbus_info, REG_CONTROL | REG_COMMAND | SMB_WORD, &word));
    ESP_LOGI(TAG, "word[0] 0x%02x (expect 0x03)", word & 0xff);
    ESP_LOGI(TAG, "word[1] 0x%02x (expect 0x01)", (word >> 8) & 0xff);

    while (1)
    {
        ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONTROL | REG_COMMAND, 0x03));  // power up
        ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_TIMING | REG_COMMAND, 0x02));   // 402ms integration time

        vTaskDelay(500 / portTICK_RATE_MS);

        uint16_t ch0 = 0;
        uint16_t ch1 = 0;

        ESP_ERROR_CHECK(smbus_read_word(smbus_info, REG_DATA0LOW | REG_COMMAND | SMB_WORD, &ch0));
        ESP_ERROR_CHECK(smbus_read_word(smbus_info, REG_DATA1LOW | REG_COMMAND | SMB_WORD, &ch1));

        printf("Full spectrum: %d\n", ch0);
        printf("Infrared:      %d\n", ch1);
        printf("Visible:       %d\n\n", ch0 - ch1);

        ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONTROL | REG_COMMAND, 0x00));  // power down
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void app_main()
{
    gpio_pad_select_gpio(BLUE_LED_GPIO);
    gpio_set_direction(BLUE_LED_GPIO, GPIO_MODE_OUTPUT);

//	xTaskCreate(&tsl2561_task, "tsl2561_task", 2048, NULL, 5, NULL);
    xTaskCreate(&test_smbus_task, "test_smbus_task", 2048, NULL, 5, NULL);

}

