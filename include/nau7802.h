#ifndef NAU7802_H
#define NAU7802_H

#include <esp_err.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default I2C address for NAU7802
#define NAU7802_I2C_ADDR 0x2A

// Conversion rates (samples per second, REG0x02:CRS)
typedef enum {
    NAU7802_RATE_10SPS = 0x00,
    NAU7802_RATE_20SPS = 0x01,
    NAU7802_RATE_40SPS = 0x02,
    NAU7802_RATE_80SPS = 0x03,
    NAU7802_RATE_320SPS = 0x07
} nau7802_rate_t;

// Gain settings (REG0x01:GAINS)
typedef enum {
    NAU7802_GAIN_1 = 0x00,
    NAU7802_GAIN_2 = 0x01,
    NAU7802_GAIN_4 = 0x02,
    NAU7802_GAIN_8 = 0x03,
    NAU7802_GAIN_16 = 0x04,
    NAU7802_GAIN_32 = 0x05,
    NAU7802_GAIN_64 = 0x06,
    NAU7802_GAIN_128 = 0x07
} nau7802_gain_t;

// LDO voltage settings (REG0x01:VLDO)
typedef enum {
    NAU7802_LDO_2V4 = 0x07,
    NAU7802_LDO_2V7 = 0x06,
    NAU7802_LDO_3V0 = 0x05,
    NAU7802_LDO_3V3 = 0x04,
    NAU7802_LDO_3V6 = 0x03,
    NAU7802_LDO_3V9 = 0x02,
    NAU7802_LDO_4V2 = 0x01,
    NAU7802_LDO_4V5 = 0x00
} nau7802_ldo_t;

// Calibration modes (REG0x02:CALMOD)
typedef enum {
    NAU7802_CALMOD_OFFSET_INTERNAL = 0x00,
    NAU7802_CALMOD_OFFSET_SYSTEM = 0x02,
    NAU7802_CALMOD_GAIN_SYSTEM = 0x03
} nau7802_calmod_t;

// Calibration point for multipoint calibration
typedef struct {
    float known_weight; // Known weight in grams
    int32_t raw_value;  // Corresponding raw ADC value
} nau7802_calib_point_t;

// NAU7802 device handle
typedef struct {
    i2c_port_t i2c_port;        // I2C port number
    uint8_t dev_addr;           // I2C device address
    float scale_factor;         // Calibration scale factor (kg per ADC count)
    int32_t offset;             // Calibration offset (ADC counts)
    SemaphoreHandle_t lock;     // Mutex for thread safety
} nau7802_dev_t;

// Initialize NAU7802 device
esp_err_t nau7802_init(i2c_port_t i2c_port, uint8_t dev_addr, nau7802_dev_t *dev);

// Configure NAU7802 (LDO voltage, gain, sample rate)
esp_err_t nau7802_set_config(nau7802_dev_t *dev, nau7802_ldo_t ldo, nau7802_gain_t gain, nau7802_rate_t rate);

// Read raw ADC value and calibrated weight
esp_err_t nau7802_read(nau7802_dev_t *dev, int32_t *raw, float *weight);

// Perform taring (zero the load cell)
esp_err_t nau7802_tare(nau7802_dev_t *dev);

// Perform multipoint calibration
esp_err_t nau7802_calibrate(nau7802_dev_t *dev, nau7802_calib_point_t *points, uint8_t num_points);

#ifdef __cplusplus
}
#endif

#endif // NAU7802_H