#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"


/*----------------------------------------------------------
 * Device Configuration
 *---------------------------------------------------------*/

#define ADXL345_I2C_ADDR         0x53
#define ADXL345_DEVICE_ID        0xE5


/*----------------------------------------------------------
 * Register Addresses
 *---------------------------------------------------------*/

#define REG_DEVID                0x00

#define REG_BW_RATE              0x2C
#define REG_POWER_CTL            0x2D

#define REG_INT_ENABLE           0x2E
#define REG_INT_MAP              0x2F
#define REG_INT_SOURCE           0x30

#define REG_DATA_FORMAT          0x31

#define REG_DATAX0               0x32
#define REG_DATAX1               0x33

#define REG_DATAY0               0x34
#define REG_DATAY1               0x35

#define REG_DATAZ0               0x36
#define REG_DATAZ1               0x37


/*----------------------------------------------------------
 * Configuration Values
 *---------------------------------------------------------*/

/*
 * POWER_CTL register
 */
#define ADXL345_MEASURE_MODE     0x08


/*
 * DATA_FORMAT register
 */
#define ADXL345_FULL_RES         0x08

#define ADXL345_RANGE_2G         0x00
#define ADXL345_RANGE_4G         0x01
#define ADXL345_RANGE_8G         0x02
#define ADXL345_RANGE_16G        0x03


/*
 * BW_RATE register
 */
#define ADXL345_RATE_100HZ       0x0A
#define ADXL345_RATE_200HZ       0x0B
#define ADXL345_RATE_400HZ       0x0C
#define ADXL345_RATE_800HZ       0x0D


/*
 * Interrupt bits
 *
 * DATA_READY = bit 7
 */
#define ADXL345_INT_DATA_READY   0x80


/*
 * ADXL345 INT1 is connected to ESP32 GPIO34.
 *
 * GPIO34 is input-only, which is appropriate because
 * the ADXL345 drives the interrupt signal into the ESP32.
 *
 * Physical board labels may show this as:
 *
 *     D34
 *     34
 *     GPIO34
 *
 * They refer to the same ESP32 GPIO number.
 */
#define ADXL345_INT1_GPIO        GPIO_NUM_34


/*
 * Full-resolution scale factor.
 *
 * Approximately 3.9 mg/LSB.
 */
#define ADXL345_SCALE_FACTOR     0.0039f


/*----------------------------------------------------------
 * Public API
 *---------------------------------------------------------*/

/*
 * Configure the ADXL345.
 *
 * This configures:
 *  - Device identification
 *  - Measurement range
 *  - Output data rate
 *  - DATA_READY interrupt source
 *
 * Measurement mode is intentionally enabled separately
 * using adxl345_enable_measurement().
 */
esp_err_t adxl345_init(void);


/*
 * Read device ID.
 */
esp_err_t adxl345_get_device_id(
    uint8_t *id
);


/*
 * Enable measurement mode.
 */
esp_err_t adxl345_enable_measurement(void);


/*
 * Configure measurement range / resolution.
 *
 * Example:
 *
 *     ADXL345_FULL_RES |
 *     ADXL345_RANGE_2G
 */
esp_err_t adxl345_set_range(
    uint8_t range
);


/*
 * Configure output data rate.
 */
esp_err_t adxl345_set_data_rate(
    uint8_t rate
);


/*
 * Configure and enable DATA_READY.
 *
 * DATA_READY is explicitly routed to INT1.
 */
esp_err_t adxl345_enable_data_ready(void);


/*
 * Configure the ESP32 GPIO interrupt connected to
 * the ADXL345 INT1 output.
 *
 * This function records the current task as the task
 * that will receive DATA_READY notifications.
 */
esp_err_t adxl345_configure_data_ready_interrupt(
    gpio_num_t gpio_num
);


/*
 * Wait for one or more DATA_READY interrupt events.
 *
 * timeout_ms:
 *     Maximum time to wait.
 *
 * event_count:
 *     Number of DATA_READY events accumulated since
 *     the previous wait.
 *
 * If event_count > 1, the acquisition task did not
 * service each DATA_READY event individually.
 */
esp_err_t adxl345_wait_for_data_ready(
    uint32_t timeout_ms,
    uint32_t *event_count
);


/*
 * Read the ADXL345 DATA_READY status directly.
 *
 * This is retained for diagnostics and polling-based
 * testing. The main interrupt-driven acquisition path
 * does not depend on this function.
 */
esp_err_t adxl345_is_data_ready(
    bool *ready
);


/*
 * Read raw signed XYZ acceleration values.
 */
esp_err_t adxl345_read_xyz(
    int16_t *x,
    int16_t *y,
    int16_t *z
);


/*
 * Read XYZ acceleration converted to g.
 */
esp_err_t adxl345_read_acceleration(
    float *x_g,
    float *y_g,
    float *z_g
);


#endif /* ADXL345_H */