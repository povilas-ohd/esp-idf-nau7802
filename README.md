# esp-idf-nau7802

ESP-IDF component for the Nuvoton NAU7802 24-bit ADC, designed for load cell applications. This library supports initialization, configuration, reading, taring, and multipoint calibration, compatible with ESP32 I2C interfaces.

## Features
- I2C communication (address 0x2A)
- Configurable LDO (2.4V–4.5V), gain (1x–128x), and sample rate (10–320 SPS)
- Read raw ADC values and calibrated weights (kg)
- Taring (zeroing) with internal offset calibration
- Multipoint calibration (2–3 points, linear fit)
- Calibration persistence using NVS
- Thread-safe with FreeRTOS mutex

## Installation
1. Add as a Git submodule in your ESP-IDF project:
   ```bash
   git submodule add https://github.com/your-repo/esp-idf-nau7802 components/nau7802