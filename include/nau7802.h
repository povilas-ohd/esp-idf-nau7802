// bento_firmware/components/nau7802/include/nau7802.h
#ifndef NAU7802_H
#define NAU7802_H

#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#define NAU7802_I2C_ADDR 0x2A

typedef struct {
    i2c_port_t i2c_port;
    uint8_t dev_addr;
    float scale_factor;
    int32_t offset;
    SemaphoreHandle_t lock;
} nau7802_dev_t;

typedef enum {
    NAU7802_LDO_2V4 = 7,
    NAU7802_LDO_2V7 = 6,
    NAU7802_LDO_3V0 = 5,
    NAU7802_LDO_3V3 = 4,
    NAU7802_LDO_3V6 = 3,
    NAU7802_LDO_3V9 = 2,
    NAU7802_LDO_4V2 = 1,
    NAU7802_LDO_4V5 = 0,
} nau7802_ldo_t;

typedef enum {
    NAU7802_GAIN_1 = 0,
    NAU7802_GAIN_2 = 1,
    NAU7802_GAIN_4 = 2,
    NAU7802_GAIN_8 = 3,
    NAU7802_GAIN_16 = 4,
    NAU7802_GAIN_32 = 5,
    NAU7802_GAIN_64 = 6,
    NAU7802_GAIN_128 = 7,
} nau7802_gain_t;

typedef enum {
    NAU7802_SPS_10 = 0,
    NAU7802_SPS_20 = 1,
    NAU7802_SPS_40 = 2,
    NAU7802_SPS_80 = 3,
    NAU7802_SPS_320 = 7,
} nau7802_sps_t;

typedef struct {
    float known_weight;
    int32_t raw_value;
} nau7802_calib_point_t;

esp_err_t nau7802_init(i2c_port_t i2c_port, uint8_t dev_addr, nau7802_dev_t *dev);
esp_err_t nau7802_set_config(nau7802_dev_t *dev, nau7802_ldo_t ldo, nau7802_gain_t gain, nau7802_sps_t rate);
esp_err_t nau7802_read(nau7802_dev_t *dev, int32_t *raw, float *weight);
esp_err_t nau7802_tare(nau7802_dev_t *dev);
esp_err_t nau7802_calibrate(nau7802_dev_t *dev, nau7802_calib_point_t *points, uint8_t num_points);

#endif