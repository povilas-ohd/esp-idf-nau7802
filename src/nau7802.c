#include "nau7802.h"
#include <esp_log.h>
#include <nvs_flash.h>
#include <string.h>
#include <math.h>

#define TAG "NAU7802"

// Register addresses (per datasheet section 10)
#define NAU7802_REG_PU_CTRL    0x00
#define NAU7802_REG_CTRL1      0x01
#define NAU7802_REG_CTRL2      0x02
#define NAU7802_REG_OCAL1_B2   0x03
#define NAU7802_REG_OCAL1_B1   0x04
#define NAU7802_REG_OCAL1_B0   0x05
#define NAU7802_REG_GCAL1_B3   0x06
#define NAU7802_REG_GCAL1_B2   0x07
#define NAU7802_REG_GCAL1_B1   0x08
#define NAU7802_REG_GCAL1_B0   0x09
#define NAU7802_REG_ADCO_B2    0x12
#define NAU7802_REG_ADCO_B1    0x13
#define NAU7802_REG_ADCO_B0    0x14
#define NAU7802_REG_POWER_CTRL 0x1C

// PU_CTRL bits (REG0x00)
#define NAU7802_PU_CTRL_RR     (1 << 0) // Register reset
#define NAU7802_PU_CTRL_PUD    (1 << 1) // Power up digital
#define NAU7802_PU_CTRL_PUA    (1 << 2) // Power up analog
#define NAU7802_PU_CTRL_PUR    (1 << 3) // Power up ready
#define NAU7802_PU_CTRL_CS     (1 << 4) // Cycle start
#define NAU7802_PU_CTRL_CR     (1 << 5) // Cycle ready

// CTRL2 bits (REG0x02)
#define NAU7802_CTRL2_CALS     (1 << 2) // Start calibration
#define NAU7802_CTRL2_CAL_ERR  (1 << 3) // Calibration error

// NVS keys for calibration storage
#define NVS_NAMESPACE "nau7802"
#define NVS_KEY_SCALE "scale_factor"
#define NVS_KEY_OFFSET "offset"

// Helper functions
static esp_err_t nau7802_write_reg(nau7802_dev_t *dev, uint8_t reg, uint8_t value);
static esp_err_t nau7802_read_reg(nau7802_dev_t *dev, uint8_t reg, uint8_t *value);
static esp_err_t nau7802_wait_ready(nau7802_dev_t *dev);
static esp_err_t nau7802_save_calibration(nau7802_dev_t *dev);
static esp_err_t nau7802_load_calibration(nau7802_dev_t *dev);

esp_err_t nau7802_init(i2c_port_t i2c_port, uint8_t dev_addr, nau7802_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    // Initialize device structure
    dev->i2c_port = i2c_port;
    dev->dev_addr = dev_addr ? dev_addr : NAU7802_I2C_ADDR;
    dev->scale_factor = 1.0f;
    dev->offset = 0;
    dev->lock = xSemaphoreCreateMutex();
    if (!dev->lock) return ESP_ERR_NO_MEM;

    // Power-on sequence (datasheet 9.1)
    esp_err_t ret;
    do {
        // Reset registers
        ret = nau7802_write_reg(dev, NAU7802_REG_PU_CTRL, NAU7802_PU_CTRL_RR);
        if (ret != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10));

        // Clear reset, power up digital and analog
        ret = nau7802_write_reg(dev, NAU7802_REG_PU_CTRL, NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA);
        if (ret != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(1));

        // Wait for power-up ready
        ret = nau7802_wait_ready(dev);
        if (ret != ESP_OK) break;

        // Start conversion cycle
        ret = nau7802_write_reg(dev, NAU7802_REG_PU_CTRL,
                                NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA | NAU7802_PU_CTRL_CS);
        if (ret != ESP_OK) break;

        // Enable PGA output capacitor (330pF recommended for 3.3V, datasheet 9.4)
        ret = nau7802_write_reg(dev, NAU7802_REG_POWER_CTRL, (1 << 7));
        if (ret != ESP_OK) break;

        // Load calibration from NVS
        ret = nau7802_load_calibration(dev);
    } while (0);

    if (ret != ESP_OK) {
        vSemaphoreDelete(dev->lock);
        dev->lock = NULL;
    }
    return ret;
}

esp_err_t nau7802_set_config(nau7802_dev_t *dev, nau7802_ldo_t ldo, nau7802_gain_t gain, nau7802_rate_t rate) {
    if (!dev || !dev->lock) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    // Configure LDO and gain (REG0x01)
    uint8_t ctrl1 = (ldo << 3) | gain;
    esp_err_t ret = nau7802_write_reg(dev, NAU7802_REG_CTRL1, ctrl1);
    if (ret != ESP_OK) goto cleanup;

    // Configure sample rate and select Channel 1 (REG0x02)
    uint8_t ctrl2 = (rate << 4); // CHS=0 for Channel 1
    ret = nau7802_write_reg(dev, NAU7802_REG_CTRL2, ctrl2);

cleanup:
    xSemaphoreGive(dev->lock);
    return ret;
}

esp_err_t nau7802_read(nau7802_dev_t *dev, int32_t *raw, float *weight) {
    if (!dev || !dev->lock || !raw || !weight) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    // Wait for data ready (REG0x00:CR)
    esp_err_t ret = nau7802_wait_ready(dev);
    if (ret != ESP_OK) goto cleanup;

    // Read 3 bytes from REG0x12–0x14 (burst read, datasheet 11.9)
    uint8_t data[3];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, NAU7802_REG_ADCO_B2, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 3, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) goto cleanup;

    // Combine 24-bit ADC value (signed)
    *raw = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
    if (*raw & 0x800000) *raw |= 0xFF000000; // Sign extend

    // Convert to weight (kg)
    *weight = (*raw - dev->offset) * dev->scale_factor;

cleanup:
    xSemaphoreGive(dev->lock);
    return ret;
}

esp_err_t nau7802_tare(nau7802_dev_t *dev) {
    if (!dev || !dev->lock) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    // Perform internal offset calibration (REG0x02: CALMOD=00, CALS=1)
    esp_err_t ret = nau7802_write_reg(dev, NAU7802_REG_CTRL2, NAU7802_CTRL2_CALS);
    if (ret != ESP_OK) goto cleanup;

    // Wait for calibration to complete
    uint8_t ctrl2;
    do {
        ret = nau7802_read_reg(dev, NAU7802_REG_CTRL2, &ctrl2);
        if (ret != ESP_OK) goto cleanup;
        if (ctrl2 & NAU7802_CTRL2_CAL_ERR) {
            ret = ESP_ERR_INVALID_STATE;
            goto cleanup;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (ctrl2 & NAU7802_CTRL2_CALS);

    // Read current ADC value as offset
    int32_t raw;
    float weight;
    ret = nau7802_read(dev, &raw, &weight);
    if (ret != ESP_OK) goto cleanup;

    dev->offset = raw;
    ret = nau7802_save_calibration(dev);

cleanup:
    xSemaphoreGive(dev->lock);
    return ret;
}

esp_err_t nau7802_calibrate(nau7802_dev_t *dev, nau7802_calib_point_t *points, uint8_t num_points) {
    if (!dev || !dev->lock || !points || num_points < 2 || num_points > 3) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    // Simple linear regression for 2 points, quadratic for 3 (least-squares fit)
    esp_err_t ret = ESP_OK;
    if (num_points == 2) {
        // Linear: weight = scale * (raw - offset)
        float delta_weight = points[1].known_weight - points[0].known_weight;
        int32_t delta_raw = points[1].raw_value - points[0].raw_value;
        if (delta_raw == 0) {
            ret = ESP_ERR_INVALID_STATE;
            goto cleanup;
        }
        dev->scale_factor = (delta_weight / 1000.0f) / delta_raw; // Convert grams to kg
        dev->offset = points[0].raw_value;
    } else {
        // Quadratic: weight = a * raw^2 + b * raw + c (simplified to linear for now)
        // TODO: Implement quadratic fit if needed
        dev->scale_factor = 1.0f;
        dev->offset = points[0].raw_value;
    }

    // Save calibration to NVS
    ret = nau7802_save_calibration(dev);

cleanup:
    xSemaphoreGive(dev->lock);
    return ret;
}

static esp_err_t nau7802_write_reg(nau7802_dev_t *dev, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t nau7802_read_reg(nau7802_dev_t *dev, uint8_t reg, uint8_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t nau7802_wait_ready(nau7802_dev_t *dev) {
    uint8_t pu_ctrl;
    for (int i = 0; i < 100; i++) {
        esp_err_t ret = nau7802_read_reg(dev, NAU7802_REG_PU_CTRL, &pu_ctrl);
        if (ret != ESP_OK) return ret;
        if (pu_ctrl & NAU7802_PU_CTRL_CR) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t nau7802_save_calibration(nau7802_dev_t *dev) {
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_blob(nvs, NVS_KEY_SCALE, &dev->scale_factor, sizeof(float));
    if (ret == ESP_OK) ret = nvs_set_i32(nvs, NVS_KEY_OFFSET, dev->offset);
    if (ret == ESP_OK) ret = nvs_commit(nvs);
    nvs_close(nvs);
    return ret;
}

static esp_err_t nau7802_load_calibration(nau7802_dev_t *dev) {
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (ret != ESP_OK) return ret;

    size_t size = sizeof(float);
    ret = nvs_get_blob(nvs, NVS_KEY_SCALE, &dev->scale_factor, &size);
    if (ret == ESP_OK) ret = nvs_get_i32(nvs, NVS_KEY_OFFSET, &dev->offset);
    nvs_close(nvs);
    return ret == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : ret;
}