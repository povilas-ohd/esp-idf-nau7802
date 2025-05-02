/**
 * @file nau7802.c
 * @brief Implementation of NAU7802 24-bit ADC driver for ESP-IDF using legacy I2C API.
 *
 * Controls the NAU7802 ADC, mirroring the sequence of a working driver to ensure
 * reliable initialization, calibration, and data reading.
 */

 #include "nau7802.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 
 static const char *TAG = "NAU7802";
 
 #define CHECK(x) do { esp_err_t ret = (x); if (ret != ESP_OK) { return ret; } } while(0)
 #define I2C_TIMEOUT_MS 1000
 #define ACK_CHECK_EN 0x1
 #define ACK_VAL 0x0
 #define NACK_VAL 0x1
 
 static esp_err_t nau7802_write(nau7802_dev_t *dev, uint8_t *data, size_t size)
 {
     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
     i2c_master_start(cmd);
     i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
     i2c_master_write(cmd, data, size, ACK_CHECK_EN);
     i2c_master_stop(cmd);
     esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
     i2c_cmd_link_delete(cmd);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
     }
     return ret;
 }
 
 static esp_err_t nau7802_read(nau7802_dev_t *dev, uint8_t *data, size_t size)
 {
     if (size == 0) return ESP_OK;
     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
     i2c_master_start(cmd);
     i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
     if (size > 1) {
         i2c_master_read(cmd, data, size - 1, ACK_VAL);
     }
     i2c_master_read_byte(cmd, data + size - 1, NACK_VAL);
     i2c_master_stop(cmd);
     esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
     i2c_cmd_link_delete(cmd);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
     }
     return ret;
 }
 
 static esp_err_t write_register(nau7802_dev_t *dev, uint8_t reg, uint8_t value)
 {
     uint8_t data[2] = {reg, value};
     esp_err_t ret = nau7802_write(dev, data, 2);
     if (ret == ESP_OK) {
         ESP_LOGD(TAG, "Wrote 0x%02X to reg 0x%02X", value, reg);
     }
     return ret;
 }
 
 esp_err_t read_register(nau7802_dev_t *dev, uint8_t reg, uint8_t *value)
 {
     esp_err_t ret = nau7802_write(dev, &reg, 1);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to write reg address 0x%02X: %s", reg, esp_err_to_name(ret));
         return ret;
     }
     ret = nau7802_read(dev, value, 1);
     if (ret == ESP_OK) {
         ESP_LOGD(TAG, "Read 0x%02X from reg 0x%02X", *value, reg);
     }
     return ret;
 }
 
 static esp_err_t read_registers(nau7802_dev_t *dev, uint8_t reg, uint8_t *data, size_t len)
 {
     esp_err_t ret = nau7802_write(dev, &reg, 1);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to write reg address 0x%02X: %s", reg, esp_err_to_name(ret));
         return ret;
     }
     ret = nau7802_read(dev, data, len);
     if (ret == ESP_OK) {
         ESP_LOGD(TAG, "Read %d bytes from reg 0x%02X", len, reg);
     }
     return ret;
 }
 
 static esp_err_t set_bit(nau7802_dev_t *dev, uint8_t bit, uint8_t reg)
 {
     uint8_t value;
     CHECK(read_register(dev, reg, &value));
     value |= (1 << bit);
     return write_register(dev, reg, value);
 }
 
 static esp_err_t clear_bit(nau7802_dev_t *dev, uint8_t bit, uint8_t reg)
 {
     uint8_t value;
     CHECK(read_register(dev, reg, &value));
     value &= ~(1 << bit);
     return write_register(dev, reg, value);
 }
 
 static bool get_bit(nau7802_dev_t *dev, uint8_t bit, uint8_t reg)
 {
     uint8_t value;
     esp_err_t ret = read_register(dev, reg, &value);
     if (ret != ESP_OK) return false;
     return (value & (1 << bit)) != 0;
 }
 
 static bool is_connected(nau7802_dev_t *dev)
 {
     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
     i2c_master_start(cmd);
     i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
     i2c_master_stop(cmd);
     esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
     i2c_cmd_link_delete(cmd);
     return ret == ESP_OK;
 }
 
 esp_err_t nau7802_reset(nau7802_dev_t *dev)
 {
     CHECK(set_bit(dev, 0, NAU7802_REG_R0_STATUS)); // RR bit
     vTaskDelay(pdMS_TO_TICKS(1));
     CHECK(clear_bit(dev, 0, NAU7802_REG_R0_STATUS));
     ESP_LOGI(TAG, "NAU7802 reset complete");
     return ESP_OK;
 }
 
 esp_err_t nau7802_power_up(nau7802_dev_t *dev)
 {
     CHECK(set_bit(dev, NAU7802_PU_CTRL_PUD, NAU7802_REG_R0_STATUS)); // PUD
     vTaskDelay(pdMS_TO_TICKS(2));
     CHECK(set_bit(dev, NAU7802_PU_CTRL_PUA, NAU7802_REG_R0_STATUS)); // PUA
     vTaskDelay(pdMS_TO_TICKS(200));
     for (int i = 0; i < 100; i++) {
         if (get_bit(dev, NAU7802_PU_CTRL_PUR, NAU7802_REG_R0_STATUS)) { // PUR
             ESP_LOGI(TAG, "NAU7802 powered up");
             return ESP_OK;
         }
         vTaskDelay(pdMS_TO_TICKS(2));
     }
     ESP_LOGE(TAG, "NAU7802 power-up failed");
     return ESP_FAIL;
 }
 
 esp_err_t nau7802_init(nau7802_dev_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr)
 {
     if (!dev) return ESP_ERR_INVALID_ARG;
     dev->i2c_port = i2c_port;
     dev->i2c_addr = i2c_addr;
 
     ESP_LOGI(TAG, "Initializing NAU7802 at address 0x%02X", i2c_addr);
 
     if (!is_connected(dev)) {
        if (!is_connected(dev)) {
            ESP_LOGE(TAG, "NAU7802 not detected at 0x%02X", i2c_addr);
            return ESP_ERR_NOT_FOUND;
        }
     }
 
     uint8_t reg_val = 0;
     esp_err_t ret = 0;
     CHECK(nau7802_reset(dev));
     CHECK(nau7802_power_up(dev));
 
     uint8_t rev_id = 0;
     CHECK(nau7802_get_revision_id(dev, &rev_id));
     ESP_LOGI(TAG, "NAU7802 Revision ID: 0x%02X", rev_id);
     if (rev_id != 0x0F) {
         ESP_LOGW(TAG, "Unexpected revision ID: 0x%02X (expected 0x0F)", rev_id);
     }
 
     ESP_LOGI(TAG, "Configuring NAU7802");
     CHECK(nau7802_set_ldo(dev, NAU7802_LDO_3V0));
     vTaskDelay(pdMS_TO_TICKS(10));
     CHECK(set_bit(dev, NAU7802_PU_CTRL_AVDDS, NAU7802_REG_R0_STATUS)); // AVDDS
     CHECK(nau7802_set_gain(dev, NAU7802_GAIN_128));
     CHECK(nau7802_set_rate(dev, NAU7802_RATE_40SPS));
     CHECK(write_register(dev, NAU7802_REG_ADC, 0x30)); // ADC config
     CHECK(set_bit(dev, 7, NAU7802_REG_PWR_CTRL)); // PGA_CAP_EN 
     CHECK(nau7802_set_channel(dev, NAU7802_CHANNEL_1));
    //  CHECK(set_bit(dev, NAU7802_PU_CTRL_CS, NAU7802_REG_R0_STATUS)); 
     CHECK(nau7802_calibrate_afe(dev));

     vTaskDelay(pdMS_TO_TICKS(100));
     CHECK(set_bit(dev, NAU7802_PU_CTRL_CS, NAU7802_REG_R0_STATUS)); 
     // Dummy read to stabilize ADC
     int32_t dummy;
     for (int i = 0; i < 3; i++) {
         if (nau7802_read_adc(dev, &dummy) == ESP_OK) {
             ESP_LOGI(TAG, "Dummy ADC read: %ld", dummy);
         }
         vTaskDelay(pdMS_TO_TICKS(50));
     }
 
    //  read_register(dev, NAU7802_REG_R0_STATUS, &reg_val);
    //  ESP_LOGI(TAG, "R0_STATUS after init: 0x%02X", reg_val);
    //  read_register(dev, NAU7802_REG_CTRL1, &reg_val);
    //  ESP_LOGI(TAG, "CTRL1 after init: 0x%02X", reg_val);
    //  read_register(dev, NAU7802_REG_CTRL2, &reg_val);
    //  ESP_LOGI(TAG, "CTRL2 after init: 0x%02X", reg_val);
    //  read_register(dev, NAU7802_REG_ADC, &reg_val);
    //  ESP_LOGI(TAG, "ADC after init: 0x%02X", reg_val);
    //  read_register(dev, NAU7802_REG_PWR_CTRL, &reg_val);
    //  ESP_LOGI(TAG, "PWR_CTRL after init: 0x%02X", reg_val);
 
     ESP_LOGI(TAG, "NAU7802 initialized");
     return ESP_OK;
 }
 
 esp_err_t nau7802_get_revision_id(nau7802_dev_t *dev, uint8_t *rev_id)
 {
     return read_register(dev, NAU7802_REG_REV_ID, rev_id);
 }
 
 esp_err_t nau7802_set_gain(nau7802_dev_t *dev, nau7802_gain_t gain)
 {
     uint8_t value;
     CHECK(read_register(dev, NAU7802_REG_CTRL1, &value));
     value = (value & ~0x07) | (gain & 0x07);
     CHECK(write_register(dev, NAU7802_REG_CTRL1, value));
     ESP_LOGI(TAG, "Set gain to %d", 1 << gain);
     return ESP_OK;
 }
 
 esp_err_t nau7802_set_rate(nau7802_dev_t *dev, nau7802_rate_t rate)
 {
     uint8_t value;
     CHECK(read_register(dev, NAU7802_REG_CTRL2, &value));
     value = (value & ~0x70) | ((rate & 0x07) << 4);
     CHECK(write_register(dev, NAU7802_REG_CTRL2, value));
     ESP_LOGI(TAG, "Set sample rate to %d SPS", rate == NAU7802_RATE_320SPS ? 320 : 10 << rate);
     return ESP_OK;
 }
 
 esp_err_t nau7802_set_ldo(nau7802_dev_t *dev, nau7802_ldo_t ldo)
 {
     uint8_t value;
     CHECK(read_register(dev, NAU7802_REG_CTRL1, &value));
     value = (value & ~0x38) | ((ldo & 0x07) << 3);
     CHECK(write_register(dev, NAU7802_REG_CTRL1, value));
     ESP_LOGI(TAG, "Set LDO to %.1fV", 4.5 - (ldo * 0.3));
     return ESP_OK;
 }
 
 esp_err_t nau7802_set_channel(nau7802_dev_t *dev, nau7802_channel_t channel)
 {
     uint8_t value;
     CHECK(read_register(dev, NAU7802_REG_CTRL2, &value));
     if (channel == NAU7802_CHANNEL_1) {
         value &= ~(1 << 7);
     } else {
         value |= (1 << 7);
     }
     CHECK(write_register(dev, NAU7802_REG_CTRL2, value));
     ESP_LOGI(TAG, "Selected channel %d", channel + 1);
     return ESP_OK;
 }
 
 static nau7802_cal_status_t cal_status(nau7802_dev_t *dev)
 {
     if (get_bit(dev, 2, NAU7802_REG_CTRL2)) {
         return NAU7802_CAL_IN_PROGRESS;
     }
     if (get_bit(dev, 3, NAU7802_REG_CTRL2)) {
         return NAU7802_CAL_FAILURE;
     }
     return NAU7802_CAL_SUCCESS;
 }
 
 esp_err_t nau7802_calibrate_afe(nau7802_dev_t *dev)
 {
     CHECK(set_bit(dev, NAU7802_CTRL2_CALS, NAU7802_REG_CTRL2)); // CALS
     TickType_t start = xTaskGetTickCount();
     while (cal_status(dev) == NAU7802_CAL_IN_PROGRESS) {
         vTaskDelay(pdMS_TO_TICKS(100));
         if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(1000)) {
             uint8_t ctrl2;
             read_register(dev, NAU7802_REG_CTRL2, &ctrl2);
             ESP_LOGE(TAG, "AFE calibration timeout, CTRL2: 0x%02X", ctrl2);
             return ESP_ERR_TIMEOUT;
         }
     }
     nau7802_cal_status_t status = cal_status(dev);
     if (status == NAU7802_CAL_SUCCESS) {
         ESP_LOGI(TAG, "AFE calibration completed");
         return ESP_OK;
     }
     uint8_t ctrl2;
     read_register(dev, NAU7802_REG_CTRL2, &ctrl2);
     ESP_LOGE(TAG, "AFE calibration failed, CTRL2: 0x%02X", ctrl2);
     return ESP_FAIL;
 }
 
 esp_err_t nau7802_is_data_ready(nau7802_dev_t *dev, bool *ready)
 {
     uint8_t value;


    //  esp_err_t ret = read_register(dev, NAU7802_REG_R0_STATUS, &value);
    //  if (ret != ESP_OK) return false;
    //  return (value & (1 << NAU7802_PU_CTRL_CR)) != 0;
     
     esp_err_t ret = read_register(dev, NAU7802_REG_R0_STATUS, &value);
     if (ret == ESP_OK) {

         *ready = (value & (1 << NAU7802_PU_CTRL_CR)) != 0;
        //  ESP_LOGD(TAG, "R0_STATUS: 0x%02X, CR bit: %d", value, *ready ? 1 : 0);
     }
     return ret;
 }
 
 esp_err_t nau7802_read_adc(nau7802_dev_t *dev, int32_t *value)
 {
     uint8_t data[3];
     CHECK(read_registers(dev, NAU7802_REG_ADCO_B2, data, 3));

     *value = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
     if (*value & 0x800000) {
         *value |= 0xFF000000; // Sign extend
     }
     
     ESP_LOGD(TAG, "ADC value: %ld", *value);
     return ESP_OK;
 }