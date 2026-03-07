# BSP Dust Sensor API Documentation

## Sharp GP2Y1010AU0F Driver

### Overview

BSP (Board Support Package) driver for Sharp GP2Y1010AU0F dust sensor providing standardized API for air quality monitoring.

### Features

- ✅ Automatic dust density measurement (µg/m³)
- ✅ Running average calculation for stable readings
- ✅ Air Quality Index (AQI) classification
- ✅ Configurable baseline and calibration
- ✅ Callback-based asynchronous updates
- ✅ Full C++ wrapper around Sharp GP2Y library

### Hardware Connection

```
GP2Y1010AU0F Pin → ESP32-S3
----------------------------
LED (Pin 3)      → GPIO15 (configurable)
VO (Pin 5)       → GPIO4  (ADC, configurable)
VCC (Pin 1)      → 5V
GND (Pin 2,4,6)  → GND
```

**Important:** Add 150µF capacitor between VCC and GND near sensor.

### Quick Start

#### 1. Basic Usage (Default Configuration)

```cpp
#include "bsp_dust_sensor.h"

bsp_dust_sensor_t dust_sensor;

void dust_callback(bsp_dust_sensor_data_t *data) {
    Serial.printf("Dust: %d µg/m³\\n", data->dust_density);
}

void setup() {
    // Initialize with defaults
    bsp_dust_sensor_init(&dust_sensor);
    bsp_dust_sensor_register_callback(&dust_sensor, dust_callback);
    bsp_dust_sensor_begin(&dust_sensor);
}

void loop() {
    bsp_dust_sensor_update(&dust_sensor);  // Call every loop
    delay(100);
}
```

#### 2. Custom Configuration

```cpp
bsp_dust_sensor_config_t config = {
    .led_pin           = 15,
    .analog_pin        = 4,
    .sample_count      = 20,    // Samples per reading
    .avg_buffer_size   = 60,    // 60 samples for running average
    .baseline_voltage  = 0.4f,  // No-dust voltage
    .calibration_factor = 1.0f, // Calibration multiplier
    .update_rate_ms    = 1000,  // Update every 1 second
};

bsp_dust_sensor_init_with_config(&dust_sensor, &config);
```

### API Reference

#### Initialization

| Function | Description |
|----------|-------------|
| `bsp_dust_sensor_init()` | Initialize with default config |
| `bsp_dust_sensor_init_with_config()` | Initialize with custom config |
| `bsp_dust_sensor_begin()` | Start sensor operation |
| `bsp_dust_sensor_deinit()` | Cleanup and free resources |

#### Reading Data

| Function | Description |
|----------|-------------|
| `bsp_dust_sensor_read()` | Read new dust measurement |
| `bsp_dust_sensor_get_data()` | Get complete data structure |
| `bsp_dust_sensor_get_density()` | Get current density (µg/m³) |
| `bsp_dust_sensor_get_running_average()` | Get averaged density |
| `bsp_dust_sensor_get_aqi_level()` | Get AQI classification |
| `bsp_dust_sensor_update()` | Auto-update (use in loop) |

#### Configuration

| Function | Description |
|----------|-------------|
| `bsp_dust_sensor_set_baseline()` | Set zero-dust voltage baseline |
| `bsp_dust_sensor_get_baseline()` | Get current baseline |
| `bsp_dust_sensor_set_calibration()` | Set calibration factor |
| `bsp_dust_sensor_set_sensitivity()` | Set sensor sensitivity |
| `bsp_dust_sensor_get_sensitivity()` | Get sensitivity value |
| `bsp_dust_sensor_register_callback()` | Register update callback |

### Data Structures

#### `bsp_dust_sensor_data_t`

```cpp
typedef struct {
    uint16_t dust_density;     // Current µg/m³
    uint16_t running_average;  // Averaged µg/m³
    float    baseline_voltage; // Baseline V
    float    sensitivity;      // Sensitivity factor
    uint32_t timestamp_ms;     // Reading timestamp
    bool     is_valid;         // Data validity flag
} bsp_dust_sensor_data_t;
```

#### Air Quality Index (AQI) Levels

| Level | Range (µg/m³) | Description |
|-------|---------------|-------------|
| `BSP_DUST_AQI_EXCELLENT` | 0-12 | Clean air |
| `BSP_DUST_AQI_GOOD` | 13-35 | Good air quality |
| `BSP_DUST_AQI_MODERATE` | 36-55 | Acceptable |
| `BSP_DUST_AQI_POOR` | 56-150 | Begin health effects |
| `BSP_DUST_AQI_UNHEALTHY` | 151-250 | Unhealthy |
| `BSP_DUST_AQI_HAZARDOUS` | >250 | Hazardous |

### Calibration Guide

#### 1. Baseline Calibration

Run sensor in clean environment for 10 minutes:

```cpp
float baseline = 0.0f;
bsp_dust_sensor_get_baseline(&dust_sensor, &baseline);
Serial.printf("Measured baseline: %.3fV\\n", baseline);
// Use this value in config
```

#### 2. Sensitivity Calibration

Compare against reference device:

```cpp
// If your sensor reads 80 but reference shows 100:
float factor = 100.0 / 80.0;  // = 1.25
bsp_dust_sensor_set_calibration(&dust_sensor, factor);
```

### Pin Configuration

Edit `lib/common/device_pin_conf.h`:

```cpp
#define DUST_SENSOR_LED_PIN (15)  // LED control pin
#define DUST_SENSOR_VO_PIN  (4)   // Analog input pin
```

### Example Output

```
[DUST_SENSOR] Initialized with LED pin=15, Analog pin=4
[DUST_SENSOR] Started - Baseline=0.40V, Calibration=1.00
[DUST] Density: 28 ug/m3 | Avg: 25 ug/m3 | AQI: GOOD | Baseline: 0.40V
[DUST] Density: 32 ug/m3 | Avg: 27 ug/m3 | AQI: GOOD | Baseline: 0.40V
```

### Integration with Project

Add to your main code:

```cpp
#include "bsp_dust_sensor.h"

bsp_dust_sensor_t g_dust_sensor;

void setup() {
    bsp_dust_sensor_init(&g_dust_sensor);
    bsp_dust_sensor_begin(&g_dust_sensor);
}

void loop() {
    bsp_dust_sensor_update(&g_dust_sensor);
    
    // Get current reading
    uint16_t density = 0;
    bsp_dust_sensor_get_density(&g_dust_sensor, &density);
    
    // Update UI or send to server
    // ...
}
```

### Troubleshooting

| Issue | Solution |
|-------|----------|
| Always reads 0 | Check wiring, verify 5V power |
| Unstable readings | Add 150µF capacitor, check baseline |
| Very high values | Recalibrate baseline in clean air |
| No response | Verify analog pin supports ADC |

### Technical Specifications

- Supply Voltage: 5V
- Current: ~11mA (LED on), ~2mA (LED off)
- Detection Range: 0-500 µg/m³
- Sensitivity: 0.5V per 100µg/m³
- Response Time: <1 second
- Operating Temp: -10°C to +65°C

### License

Copyright (C) 2025 ITRVN - Fiot License
