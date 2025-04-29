// bento_firmware/components/nau7802/src/nau7802.c
#include "nau7802.h"
#include <esp_log.h>
#include <string.h>

#define TAG "NAU7802"

// Register addresses
#define NAU7802_REG_PU_CTRL    0x00
#define NAU7802_REG_CTRL1      0x01
#define NAU7802_REG_CTRL2      0x02
#define NAU7802_REG_ADCO_B2    0x12
#define NAU7802_REG_ADCO_B1    0x13
#define NAU7802_REG_ADCO_B0    0x14
#define NAU7802_REG_ADC        0x15
#define NAU7802_REG_PGA        0x1B
#define NAU7802_REG_PGA_PWR    0x1C

// PU_CTRL bits
#define NAU7802_PU_CTRL_RR     (1 << 0)
#define NAU7802_PU_CTRL_PUD    (1 << 1)
#define NAU7802_PU_CTRL_PUA    (1 << 2)
#define NAU7802_PU_CTRL_PUR    (1 << 3)
#define NAU7802_PU_CTRL_CS     (1 << 4)
#define NAU7802_PU_CTRL_CR     (1 << 5)
#define NAU7802_PU_CTRL_OSCS   (1 << 6)
#define NAU7802_PU_CTRL_AVDDS  (1 << 7)

// CTRL2 bits
#define NAU7802_CTRL2_CALS     (1 << 2)
#define NAU7802_CTRL2_CAL_ERR  (1 << 3)
#define NAU7802_CTRL2_CRS_POS  4
#define NAU7802_CTRL2_CHS      (1 << 7)

// PGA bits
#define NAU7802_PGA_LDOMODE    (1 << 6)

// PGA_PWR bits
#define NAU7802_PGA_PWR_CAP_EN (1 << 7)

// Helper function prototypes
static esp_err_t nau7802_write_reg(nau7802_dev_t *dev, uint8_t reg, uint8_t value);
static esp_err_t nau7802_read_reg(nau7802_dev_t *dev, uint8_t reg, uint8_t *value);
static esp_err_t nau7802_wait_ready(nau7802_dev_t *dev);
static esp_err_t nau7802_calibrate_afe(nau7802_dev_t *dev);

esp_err_t nau7802_init(i2c_port_t i2c_port, uint8_t dev_addr, nau7802_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    // Initialize device structure
    dev->i2c_port = i2c_port;
    dev->dev_addr = dev_addr ? dev_addr : NAU7802_I2C_ADDR;
    dev->scale_factor = 1.0f;
    dev->offset = 0;
    dev->lock = xSemaphoreCreateMutex();
    if (!dev->lock) return ESP_ERR_NO_MEM;

    esp_err_t ret;

    // Reset chip
    ESP_LOGD(TAG, "Resetting NAU7802");
    ret = nau7802_write_reg(dev, NAU7802_REG_PU_CTRL, NAU7802_PU_CTRL_RR);
    if (ret != ESP_OK) goto cleanup;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Clear reset bit
    ret = nau7802_write_reg(dev, NAU7802_REG_PU_CTRL, 0);
    if (ret != ESP_OK) goto cleanup;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Power up digital and analog
    ESP_LOGD(TAG, "Powering up NAU7802");
    ret = nau7802_write_reg(dev, NAU7802_REG_PU_CTRL, NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA);
    if (ret != ESP_OK) goto cleanup;

    // Wait for power-up ready
    ret = nau7802_wait_ready(dev);
    if (ret != ESP_OK) goto cleanup;

    // Configure ADC (disable CLK_CHP, enable 330pF capacitor)
    ESP_LOGD(TAG, "Configuring ADC");
    ret = nau7802_write_reg(dev, NAU7802_REG_ADC, 0x30);
    if (ret != ESP_OK) goto cleanup;
    ret = nau7802_write_reg(dev, NAU7802_REG_PGA_PWR, NAU7802_PGA_PWR_CAP_EN);
    if (ret != ESP_OK) goto cleanup;

    // Clear LDOMODE
    uint8_t pga_reg;
    ret = nau7802_read_reg(dev, NAU7802_REG_PGA, &pga_reg);
    if (ret != ESP_OK) goto cleanup;
    pga_reg &= ~NAU7802_PGA_LDOMODE;
    ret = nau7802_write_reg(dev, NAU7802_REG_PGA, pga_reg);
    if (ret != ESP_OK) goto cleanup;

    return ESP_OK;

cleanup:
    ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(ret));
    vSemaphoreDelete(dev->lock);
    dev->lock = NULL;
    return ret;
}

esp_err_t nau7802_set_config(nau7802_dev_t *dev, nau7802_ldo_t ldo, nau7802_gain_t gain, nau7802_sps_t rate) {
    if (!dev || !dev->lock) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    esp_err_t ret;

    // Configure LDO and gain
    uint8_t ctrl1 = (ldo << 3) | gain;
    ESP_LOGD(TAG, "Writing CTRL1: 0x%02X (LDO=%d, Gain=%d)", ctrl1, ldo, gain);
    ret = nau7802_write_reg(dev, NAU7802_REG_CTRL1, ctrl1);
    if (ret != ESP_OK) goto cleanup;

    // Configure sample rate
    uint8_t ctrl2 = (rate << NAU7802_CTRL2_CRS_POS);
    ESP_LOGD(TAG, "Writing CTRL2: 0x%02X (Rate=%d)", ctrl2, rate);
    ret = nau7802_write_reg(dev, NAU7802_REG_CTRL2, ctrl2);
    if (ret != ESP_OK) goto cleanup;

    // Enable conversions
    uint8_t pu_ctrl;
    ret = nau7802_read_reg(dev, NAU7802_REG_PU_CTRL, &pu_ctrl);
    if (ret != ESP_OK) goto cleanup;
    pu_ctrl |= NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA | NAU7802_PU_CTRL_CS;
    ESP_LOGD(TAG, "Writing PU_CTRL: 0x%02X", pu_ctrl);
    ret = nau7802_write_reg(dev, NAU7802_REG_PU_CTRL, pu_ctrl);
    if (ret != ESP_OK) goto cleanup;

    // Calibrate AFE
    ret = nau7802_calibrate_afe(dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AFE calibration failed: %s", esp_err_to_name(ret));
        // Proceed anyway for debugging
    }

cleanup:
    xSemaphoreGive(dev->lock);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set config failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t nau7802_read(nau7802_dev_t *dev, int32_t *raw, float *weight) {
    if (!dev || !dev->lock || !raw || !weight) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    // Wait for data ready
    esp_err_t ret = nau7802_wait_ready(dev);
    if (ret != ESP_OK) goto cleanup;

    // Read 3 bytes from ADCO_B2–B0
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

    ESP_LOGD(TAG, "ADC Bytes: B2=0x%02X, B1=0x%02X, B0=0x%02X", data[0], data[1], data[2]);

    // Combine 24-bit ADC value
    int32_t adc_value = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
    if (adc_value & 0x800000) adc_value |= 0xFF000000; // Sign extend
    *raw = adc_value;
    *weight = (*raw - dev->offset) * dev->scale_factor;

cleanup:
    xSemaphoreGive(dev->lock);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t nau7802_tare(nau7802_dev_t *dev) {
    if (!dev || !dev->lock) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    // Take multiple readings and average
    int32_t raw_sum = 0;
    float weight;
    int readings = 10;
    esp_err_t ret;
    for (int i = 0; i < readings; i++) {
        int32_t raw;
        ret = nau7802_read(dev, &raw, &weight);
        if (ret != ESP_OK) goto cleanup;
        raw_sum += raw;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    dev->offset = raw_sum / readings;

cleanup:
    xSemaphoreGive(dev->lock);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Tare failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t nau7802_calibrate(nau7802_dev_t *dev, nau7802_calib_point_t *points, uint8_t num_points) {
    if (!dev || !dev->lock || !points || num_points < 2) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    esp_err_t ret = ESP_OK;
    float delta_weight = points[1].known_weight - points[0].known_weight;
    int32_t delta_raw = points[1].raw_value - points[0].raw_value;
    if (delta_raw == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto cleanup;
    }
    dev->scale_factor = (delta_weight / 1000.0f) / delta_raw;
    dev->offset = points[0].raw_value;

cleanup:
    xSemaphoreGive(dev->lock);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Calibrate failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t nau7802_calibrate_afe(nau7802_dev_t *dev) {
    // Start calibration
    esp_err_t ret = nau7802_write_reg(dev, NAU7802_REG_CTRL2, NAU7802_CTRL2_CALS);
    if (ret != ESP_OK) return ret;

    // Wait for calibration to complete (up to 5 seconds)
    uint8_t ctrl2;
    for (int i = 0; i < 500; i++) {
        ret = nau7802_read_reg(dev, NAU7802_REG_CTRL2, &ctrl2);
        if (ret != ESP_OK) return ret;
        ESP_LOGD(TAG, "CTRL2 during calibration: 0x%02X", ctrl2);
        if (!(ctrl2 & NAU7802_CTRL2_CALS)) {
            if (ctrl2 & NAU7802_CTRL2_CAL_ERR) {
                ESP_LOGE(TAG, "Calibration error detected");
                return ESP_ERR_INVALID_STATE;
            }
            ESP_LOGI(TAG, "AFE calibration completed in %d ms", i * 10);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "AFE calibration timeout");
    return ESP_ERR_TIMEOUT;
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
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
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
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Read reg 0x%02X: 0x%02X", reg, *value);
    }
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
    ESP_LOGE(TAG, "Timeout waiting for NAU7802 ready");
    return ESP_ERR_TIMEOUT;
}