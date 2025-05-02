#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include "nau7802.h"

#define I2C_MASTER_SCL_IO    9        /*!< GPIO number for I2C SCL */
#define I2C_MASTER_SDA_IO    8        /*!< GPIO number for I2C SDA */
#define I2C_MASTER_FREQ_HZ   100000   /*!< I2C master clock frequency */
#define I2C_MASTER_PORT      I2C_NUM_0 /*!< I2C port number */
#define I2C_TIMEOUT_MS       1000     /*!< I2C transaction timeout in ms */

static const char *TAG = "NAU7802_TEST";

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C master initialized on SDA: GPIO%d, SCL: GPIO%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t i2c_probe_device(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
    } else {
        ESP_LOGD(TAG, "No device at address 0x%02X: %s", addr, esp_err_to_name(ret));
    }
    return ret;
}

static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus for NAU7802 at 0x%02X...", NAU7802_DEFAULT_ADDR);
    esp_err_t ret = i2c_probe_device(NAU7802_DEFAULT_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NAU7802 not detected at 0x%02X. Check wiring and pull-ups.", NAU7802_DEFAULT_ADDR);
    }
    ESP_LOGI(TAG, "Performing full I2C bus scan...");
    for (uint8_t addr = 0x08; addr <= 0x7F; addr++) {
        if (addr == NAU7802_DEFAULT_ADDR) continue;
        i2c_probe_device(addr);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void nau7802_test_task(void *pvParameters)
{
    nau7802_dev_t nau7802 = {0};
    esp_err_t ret;

    i2c_scan();

    nau7802.i2c_port = I2C_MASTER_PORT;
    nau7802.i2c_addr = NAU7802_DEFAULT_ADDR;

    ESP_LOGI(TAG, "Attempting NAU7802 initialization at address 0x%02X", NAU7802_DEFAULT_ADDR);
    ret = nau7802_init(&nau7802, I2C_MASTER_PORT, NAU7802_DEFAULT_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NAU7802 initialization failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
    }

    // ESP_LOGI(TAG, "Attempting Channel 1 AFE calibration");
    // ret = nau7802_calibrate_afe(&nau7802);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Channel 1 AFE calibration failed: %s", esp_err_to_name(ret));
    //     ESP_LOGW(TAG, "Continuing without calibration for testing");
    // } else {
    //     ESP_LOGI(TAG, "Channel 1 AFE calibration completed");
    // }

    while (1) {
        int32_t adc_value;
        bool ready;

        // ret = nau7802_set_channel(&nau7802, NAU7802_CHANNEL_1);
        // if (ret != ESP_OK) {
        //     ESP_LOGE(TAG, "Failed to select Channel 1: %s", esp_err_to_name(ret));
        //     vTaskDelay(pdMS_TO_TICKS(1000));
        //     continue;
        // }

        for (int i = 0; i < 20; i++) {
            ret = nau7802_is_data_ready(&nau7802, &ready);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to check data ready: %s", esp_err_to_name(ret));
                break;
            }
            if (ready) break;
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (ready) {
            ret = nau7802_read_adc(&nau7802, &adc_value);
            if (ret == ESP_OK) {
                printf("Channel 1 ADC Value: %ld\n", adc_value);
            } else {
                ESP_LOGE(TAG, "Failed to read Channel 1 ADC: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "Channel 1 data not ready after retries");
            uint8_t reg_val = 0;
            ret = read_register(&nau7802, NAU7802_REG_R0_STATUS, &reg_val);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "R0_STATUS: 0x%02X (CR bit: %d)", reg_val, (reg_val & 0x40) ? 1 : 0);
            } else {
                ESP_LOGE(TAG, "Failed to read R0_STATUS for debug: %s", esp_err_to_name(ret));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return;
    }

    xTaskCreate(nau7802_test_task, "nau7802_test_task", 4096, NULL, 5, NULL);
}