#include "ds18b20.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"


static const char *TAG = "DS18B20";

static int s_gpio_num = DS18B20_DEFAULT_GPIO;


/*
 * 1-Wire timing is sensitive to interrupts.
 * Keep each timing slot deterministic.
 */
static portMUX_TYPE s_onewire_mux =
    portMUX_INITIALIZER_UNLOCKED;


/*----------------------------------------------------------
 * Configure GPIO
 *---------------------------------------------------------*/

static void onewire_release(void)
{
    gpio_set_level(
        s_gpio_num,
        1
    );
}


static void onewire_pull_low(void)
{
    gpio_set_level(
        s_gpio_num,
        0
    );
}


/*----------------------------------------------------------
 * Reset and presence detection
 *---------------------------------------------------------*/

static bool onewire_reset(void)
{
    bool presence;

    portENTER_CRITICAL(
        &s_onewire_mux
    );

    /*
     * Master reset pulse.
     */
    onewire_pull_low();

    esp_rom_delay_us(480);


    /*
     * Release the bus.
     */
    onewire_release();

    esp_rom_delay_us(70);


    /*
     * DS18B20 pulls the bus LOW
     * during the presence pulse.
     */
    presence =
        (
            gpio_get_level(
                s_gpio_num
            ) == 0
        );


    /*
     * Complete reset timing.
     */
    esp_rom_delay_us(410);


    portEXIT_CRITICAL(
        &s_onewire_mux
    );


    return presence;
}


/*----------------------------------------------------------
 * Write one bit
 *---------------------------------------------------------*/

static void onewire_write_bit(
    uint8_t bit
)
{
    portENTER_CRITICAL(
        &s_onewire_mux
    );


    onewire_pull_low();


    if (bit)
    {
        /*
         * Write '1'
         */
        esp_rom_delay_us(6);

        onewire_release();

        esp_rom_delay_us(64);
    }
    else
    {
        /*
         * Write '0'
         */
        esp_rom_delay_us(60);

        onewire_release();

        esp_rom_delay_us(10);
    }


    portEXIT_CRITICAL(
        &s_onewire_mux
    );
}


/*----------------------------------------------------------
 * Read one bit
 *---------------------------------------------------------*/

static uint8_t onewire_read_bit(void)
{
    uint8_t bit;


    portENTER_CRITICAL(
        &s_onewire_mux
    );


    /*
     * Start read slot.
     */
    onewire_pull_low();

    esp_rom_delay_us(3);


    /*
     * Release bus.
     */
    onewire_release();


    /*
     * Allow DS18B20 to place its bit
     * onto the 1-Wire bus.
     */
    esp_rom_delay_us(10);


    bit =
        (uint8_t)gpio_get_level(
            s_gpio_num
        );


    /*
     * Complete the time slot.
     */
    esp_rom_delay_us(53);


    portEXIT_CRITICAL(
        &s_onewire_mux
    );


    return bit;
}


/*----------------------------------------------------------
 * Write one byte
 *---------------------------------------------------------*/

static void onewire_write_byte(
    uint8_t value
)
{
    for (int i = 0; i < 8; i++)
    {
        onewire_write_bit(
            value & 0x01
        );

        value >>= 1;
    }
}


/*----------------------------------------------------------
 * Read one byte
 *---------------------------------------------------------*/

static uint8_t onewire_read_byte(void)
{
    uint8_t value = 0;


    for (int i = 0; i < 8; i++)
    {
        uint8_t bit =
            onewire_read_bit();


        value |=
            (
                bit << i
            );
    }


    return value;
}


/*----------------------------------------------------------
 * Dallas / Maxim CRC8
 *---------------------------------------------------------*/

static uint8_t ds18b20_crc8(
    const uint8_t *data,
    size_t length
)
{
    uint8_t crc = 0;


    for (size_t i = 0; i < length; i++)
    {
        uint8_t inbyte =
            data[i];


        for (int j = 0; j < 8; j++)
        {
            uint8_t mix =
                (
                    crc ^ inbyte
                )
                &
                0x01;


            crc >>= 1;


            if (mix)
            {
                crc ^= 0x8C;
            }


            inbyte >>= 1;
        }
    }


    return crc;
}


/*----------------------------------------------------------
 * Initialization
 *---------------------------------------------------------*/

esp_err_t ds18b20_init(
    int gpio_num
)
{
    if (gpio_num < 0)
    {
        return ESP_ERR_INVALID_ARG;
    }


    s_gpio_num =
        gpio_num;


    /*
     * Open-drain GPIO.
     *
     * The DS18B20 module should provide
     * the 1-Wire pull-up.
     */
    gpio_config_t config =
    {
        .pin_bit_mask =
            (
                1ULL << s_gpio_num
            ),

        .mode =
            GPIO_MODE_INPUT_OUTPUT_OD,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    esp_err_t err =
        gpio_config(
            &config
        );


    if (err != ESP_OK)
    {
        return err;
    }


    /*
     * Release 1-Wire bus.
     */
    onewire_release();


    /*
     * Check for DS18B20 presence.
     */
    if (!onewire_reset())
    {
        ESP_LOGE(
            TAG,
            "DS18B20 not detected on GPIO %d",
            s_gpio_num
        );

        return ESP_ERR_NOT_FOUND;
    }


    ESP_LOGI(
        TAG,
        "DS18B20 detected on GPIO %d",
        s_gpio_num
    );


    return ESP_OK;
}


/*----------------------------------------------------------
 * Read temperature
 *---------------------------------------------------------*/

esp_err_t ds18b20_read_temperature(
    float *temperature_c
)
{
    if (temperature_c == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    /*------------------------------------------------------
     * Start temperature conversion
     *-----------------------------------------------------*/

    if (!onewire_reset())
    {
        return ESP_ERR_NOT_FOUND;
    }


    /*
     * Skip ROM.
     *
     * We currently have one DS18B20
     * on this bus.
     */
    onewire_write_byte(
        0xCC
    );


    /*
     * Convert temperature.
     */
    onewire_write_byte(
        0x44
    );


    /*
     * 12-bit conversion:
     * up to approximately 750 ms.
     */
    vTaskDelay(
        pdMS_TO_TICKS(750)
    );


    /*------------------------------------------------------
     * Read scratchpad
     *-----------------------------------------------------*/

    if (!onewire_reset())
    {
        return ESP_ERR_NOT_FOUND;
    }


    /*
     * Skip ROM.
     */
    onewire_write_byte(
        0xCC
    );


    /*
     * Read Scratchpad.
     */
    onewire_write_byte(
        0xBE
    );


    uint8_t scratchpad[9];


    memset(
        scratchpad,
        0,
        sizeof(scratchpad)
    );


    for (int i = 0; i < 9; i++)
    {
        scratchpad[i] =
            onewire_read_byte();
    }


    /*------------------------------------------------------
     * Verify CRC
     *-----------------------------------------------------*/

    uint8_t calculated_crc =
        ds18b20_crc8(
            scratchpad,
            8
        );


    if (calculated_crc !=
        scratchpad[8])
    {
        ESP_LOGE(
            TAG,
            "CRC error: calculated=0x%02X received=0x%02X",
            calculated_crc,
            scratchpad[8]
        );

        return ESP_ERR_INVALID_CRC;
    }


    /*------------------------------------------------------
     * Convert raw temperature
     *-----------------------------------------------------*/

    int16_t raw_temperature =
        (int16_t)(
            (
                (
                    (uint16_t)scratchpad[1]
                    << 8
                )
                |
                scratchpad[0]
            )
        );


    *temperature_c =
        (
            (float)raw_temperature
            /
            16.0f
        );


    /*------------------------------------------------------
     * Basic sanity check
     *-----------------------------------------------------*/

    if (
        *temperature_c < -55.0f ||
        *temperature_c > 125.0f
    )
    {
        ESP_LOGE(
            TAG,
            "Invalid temperature: %.2f C",
            *temperature_c
        );

        return ESP_ERR_INVALID_RESPONSE;
    }


    return ESP_OK;
}