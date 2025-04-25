#include "nau7802.h"

void app_main() {
    // Initialize I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    // Initialize NAU7802
    nau7802_dev_t dev;
    ESP_ERROR_CHECK(nau7802_init(I2C_NUM_0, NAU7802_I2C_ADDR, &dev));

    // Configure (3.0V LDO, 128x gain, 80 SPS)
    ESP_ERROR_CHECK(nau7802_set_config(&dev, NAU7802_LDO_3V0, NAU7802_GAIN_128, NAU7802_RATE_80SPS));

    // Read weight
    int32_t raw;
    float weight;
    ESP_ERROR_CHECK(nau7802_read(&dev, &raw, &weight));
    printf("Raw: %ld, Weight: %.3f kg\n", raw, weight);

    // Tare
    ESP_ERROR_CHECK(nau7802_tare(&dev));

    // Calibrate (e.g., 0g, 50g)
    nau7802_calib_point_t points[2] = {
        {0.0f, 0}, // Set raw_value later
        {50.0f, 0}
    };
    ESP_ERROR_CHECK(nau7802_read(&dev, &points[0].raw_value, &weight)); // 0g
    // Place 50g weight
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_ERROR_CHECK(nau7802_read(&dev, &points[1].raw_value, &weight)); // 50g
    ESP_ERROR_CHECK(nau7802_calibrate(&dev, points, 2));
}