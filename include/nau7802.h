/**
 * @file nau7802.h
 * @brief Header file for NAU7802 24-bit ADC driver for ESP-IDF.
 *
 * Provides functions to interface with the NAU7802 ADC over I2C, supporting
 * load cell measurements, calibration, and configuration. Uses legacy I2C API
 * and mirrors working library's sequence.
 */

 #ifndef NAU7802_H
 #define NAU7802_H
 
 #include <esp_err.h>
 #include <driver/i2c.h>
 #include <stdint.h>
 #include <stdbool.h>
 
 /**
  * @defgroup NAU7802_Registers NAU7802 Register Definitions
  * @{
  */
 #define NAU7802_REG_R0_STATUS   0x00 ///< Status and control register
 #define NAU7802_REG_CTRL1       0x01 ///< Control register 1 (gain, LDO)
 #define NAU7802_REG_CTRL2       0x02 ///< Control register 2 (rate, calibration)
 #define NAU7802_REG_OCAL1_B2    0x03 ///< Offset calibration byte 2
 #define NAU7802_REG_OCAL1_B1    0x04 ///< Offset calibration byte 1
 #define NAU7802_REG_OCAL1_B0    0x05 ///< Offset calibration byte 0
 #define NAU7802_REG_GCAL1_B3    0x06 ///< Gain calibration byte 3
 #define NAU7802_REG_GCAL1_B2    0x07 ///< Gain calibration byte 2
 #define NAU7802_REG_GCAL1_B1    0x08 ///< Gain calibration byte 1
 #define NAU7802_REG_GCAL1_B0    0x09 ///< Gain calibration byte 0
 #define NAU7802_REG_OCAL2_B2    0x0A ///< Offset calibration channel 2 byte 2
 #define NAU7802_REG_OCAL2_B1    0x0B ///< Offset calibration channel 2 byte 1
 #define NAU7802_REG_OCAL2_B0    0x0C ///< Offset calibration channel 2 byte 0
 #define NAU7802_REG_GCAL2_B3    0x0D ///< Gain calibration channel 2 byte 3
 #define NAU7802_REG_GCAL2_B2    0x0E ///< Gain calibration channel 2 byte 2
 #define NAU7802_REG_GCAL2_B1    0x0F ///< Gain calibration channel 2 byte 1
 #define NAU7802_REG_GCAL2_B0    0x10 ///< Gain calibration channel 2 byte 0
 #define NAU7802_REG_I2C_CTRL    0x11 ///< I2C control register
 #define NAU7802_REG_ADCO_B2     0x12 ///< ADC output byte 2 (MSB)
 #define NAU7802_REG_ADCO_B1     0x13 ///< ADC output byte 1
 #define NAU7802_REG_ADCO_B0     0x14 ///< ADC output byte 0 (LSB)

 #define NAU7802_REG_ADC         0x15 ///< ADC control register
 #define NAU7802_REG_PGA         0x1B ///< PGA control register
 #define NAU7802_REG_PWR_CTRL    0x1C ///< Power control register
 #define NAU7802_REG_REV_ID      0x1F ///< Device revision ID

 /** @} */
 
 /**
  * @brief NAU7802 default I2C address.
  */
 #define NAU7802_DEFAULT_ADDR 0x2A
 
 /**
  * @brief NAU7802 gain settings.
  */
 typedef enum {
     NAU7802_GAIN_1   = 0x0, ///< Gain x1
     NAU7802_GAIN_2   = 0x1, ///< Gain x2
     NAU7802_GAIN_4   = 0x2, ///< Gain x4
     NAU7802_GAIN_8   = 0x3, ///< Gain x8
     NAU7802_GAIN_16  = 0x4, ///< Gain x16
     NAU7802_GAIN_32  = 0x5, ///< Gain x32
     NAU7802_GAIN_64  = 0x6, ///< Gain x64
     NAU7802_GAIN_128 = 0x7  ///< Gain x128
 } nau7802_gain_t;
 
 /**
  * @brief NAU7802 sample rate settings (samples per second).
  */
 typedef enum {
     NAU7802_RATE_10SPS  = 0x0, ///< 10 SPS
     NAU7802_RATE_20SPS  = 0x1, ///< 20 SPS
     NAU7802_RATE_40SPS  = 0x2, ///< 40 SPS
     NAU7802_RATE_80SPS  = 0x3, ///< 80 SPS
     NAU7802_RATE_320SPS = 0x7  ///< 320 SPS
 } nau7802_rate_t;
 
 /**
  * @brief NAU7802 LDO voltage settings.
  */
 typedef enum {
     NAU7802_LDO_2V4 = 0x7, ///< 2.4V
     NAU7802_LDO_2V7 = 0x6, ///< 2.7V
     NAU7802_LDO_3V0 = 0x5, ///< 3.0V
     NAU7802_LDO_3V3 = 0x4, ///< 3.3V
     NAU7802_LDO_3V6 = 0x3, ///< 3.6V
     NAU7802_LDO_3V9 = 0x2, ///< 3.9V
     NAU7802_LDO_4V2 = 0x1, ///< 4.2V
     NAU7802_LDO_4V5 = 0x0  ///< 4.5V
 } nau7802_ldo_t;
 
//Bits within the PU_CTRL register
typedef enum
{
  NAU7802_PU_CTRL_RR = 0,
  NAU7802_PU_CTRL_PUD,
  NAU7802_PU_CTRL_PUA,
  NAU7802_PU_CTRL_PUR,
  NAU7802_PU_CTRL_CS,
  NAU7802_PU_CTRL_CR,
  NAU7802_PU_CTRL_OSCS,
  NAU7802_PU_CTRL_AVDDS,
} PU_CTRL_Bits;

typedef enum
{
  NAU7802_CTRL2_CALMOD = 0,
  NAU7802_CTRL2_CALS = 2,
  NAU7802_CTRL2_CAL_ERROR = 3,
  NAU7802_CTRL2_CRS = 4,
  NAU7802_CTRL2_CHS = 7,
} CTRL2_Bits;

 /**
  * @brief NAU7802 channel selection.
  */
 typedef enum {
     NAU7802_CHANNEL_1 = 0x0, ///< Channel 1 (AIN1)
     NAU7802_CHANNEL_2 = 0x1  ///< Channel 2 (AIN2)
 } nau7802_channel_t;
 
 /**
  * @brief NAU7802 calibration status.
  */
 typedef enum {
     NAU7802_CAL_SUCCESS = 0,
     NAU7802_CAL_IN_PROGRESS = 1,
     NAU7802_CAL_FAILURE = 2
 } nau7802_cal_status_t;
 
 /**
  * @brief NAU7802 device handle.
  */
 typedef struct {
     i2c_port_t i2c_port; ///< I2C port number
     uint8_t i2c_addr;    ///< I2C device address (typically 0x2A)
 } nau7802_dev_t;

 /**
  * @brief NAU7802 initialization configuration.
  *
  * Passed to nau7802_init() to configure the device.
  * All fields have sensible defaults via NAU7802_CONFIG_DEFAULT().
  */
 typedef struct {
     nau7802_ldo_t ldo;           ///< LDO voltage
     nau7802_gain_t gain;         ///< PGA gain
     nau7802_rate_t rate;         ///< Sample rate
     nau7802_channel_t channel;   ///< Input channel
 } nau7802_config_t;

 /**
  * @brief Default configuration: 3.0V LDO, 128x gain, 10 SPS, channel 1.
  */
 #define NAU7802_CONFIG_DEFAULT() { \
     .ldo = NAU7802_LDO_3V0, \
     .gain = NAU7802_GAIN_128, \
     .rate = NAU7802_RATE_10SPS, \
     .channel = NAU7802_CHANNEL_1, \
 }

 /**
  * @brief Initialize the NAU7802 device with the given configuration.
  */
 esp_err_t nau7802_init(nau7802_dev_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr,
                        const nau7802_config_t *config);
 
 /**
  * @brief Reset the NAU7802 device.
  */
 esp_err_t nau7802_reset(nau7802_dev_t *dev);
 
 /**
  * @brief Power up the NAU7802 device.
  */
 esp_err_t nau7802_power_up(nau7802_dev_t *dev);
 
 /**
  * @brief Read the NAU7802 revision ID.
  */
 esp_err_t nau7802_get_revision_id(nau7802_dev_t *dev, uint8_t *rev_id);
 
 /**
  * @brief Set the gain for the PGA.
  */
 esp_err_t nau7802_set_gain(nau7802_dev_t *dev, nau7802_gain_t gain);
 
 /**
  * @brief Set the sample rate.
  */
 esp_err_t nau7802_set_rate(nau7802_dev_t *dev, nau7802_rate_t rate);
 
 /**
  * @brief Set the LDO voltage.
  */
 esp_err_t nau7802_set_ldo(nau7802_dev_t *dev, nau7802_ldo_t ldo);
 
 /**
  * @brief Select the input channel.
  */
 esp_err_t nau7802_set_channel(nau7802_dev_t *dev, nau7802_channel_t channel);
 
 /**
  * @brief Perform AFE calibration.
  */
 esp_err_t nau7802_calibrate_afe(nau7802_dev_t *dev);
 
 /**
  * @brief Check if conversion data is ready.
  */
 esp_err_t nau7802_is_data_ready(nau7802_dev_t *dev, bool *ready);
 
 /**
  * @brief Read the 24-bit ADC conversion result.
  */
 esp_err_t nau7802_read_adc(nau7802_dev_t *dev, int32_t *value);
 
 /**
  * @brief Read a single byte from an NAU7802 register.
  */
 esp_err_t read_register(nau7802_dev_t *dev, uint8_t reg, uint8_t *value);
 
 #endif // NAU7802_H