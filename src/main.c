#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "string.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"


#define USE_TOUCH TOUCH_TYPE_NONE
#define PARALLEL_LINES 16

#define PIN_NUM_LED 10  // LED
#define PIN_NUM_MISO -1 // SPI MISO
#define PIN_NUM_MOSI 15 // SPI MOSI
#define PIN_NUM_CLK 13  // SPI CLOCK pin
#define PIN_NUM_CS 5    // Display CS pin
#define PIN_NUM_DC 23   // Display command/data pin
#define PIN_NUM_TCS 0   // Touch screen CS pin (NOT used if USE_TOUCH=0)

#define PIN_NUM_RST 18 // GPIO used for RESET control (#16)
#define PIN_NUM_BCKL 5 // GPIO used for backlight control
#define PIN_BCKL_ON 1  // GPIO value for backlight ON
#define PIN_BCKL_OFF 0 // GPIO value for backlight OFF

typedef struct
{
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;


DRAM_ATTR static const lcd_init_cmd_t init_cmds[] = {
    {0x01, {0}, 0x80},
    {0x11, {0}, 0x80},
    {0xB1, {0x01, 0x2C, 0x2D}, 3},
    {0xB2, {0x01, 0x2C, 0x2D}, 3},
    {0xB3, {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D}, 6},
    {0xB4, {0x07}, 1},
    {0xC0, {0xA2, 0x02, 0x84}, 3},
    {0xC1, {0xC5}, 1},
    {0xC2, {0x0A, 0x00}, 2},
    {0xC3, {0x8A, 0x2A}, 2},
    {0xC4, {0x8A, 0xEE}, 2},
    {0xC5, {0x0E}, 1},
    {0x21, {0}, 0},
    {0x36, {0xC8}, 1},
    {0x3A, {0x06}, 0x81},

    {0x2A, {0x00, 0x02, 0x00, 0x7F + 0x02}, 4},
    {0x2B, {0x00, 0x01, 0x00, 0x9F + 0x01}, 4},

    {0xE0, {0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10}, 16},
    {0xE1, {0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10}, 16},
    {0x13, {0}, 0x80},
    {0x29, {0}, 0x80},

    {0, {0}, 0xFF},
};

void init_tft_pins()
{
    gpio_pad_select_gpio(PIN_NUM_DC);
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(PIN_NUM_RST);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(PIN_NUM_BCKL);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
}

void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                   //Zero out the transaction
    t.length = 8;                               //Command is 8 bits
    t.tx_buffer = &cmd;                         //The data is the cmd itself
    t.user = (void *)0;                         //D/C needs to be set to 0
    ret = spi_device_polling_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);                      //Should have had no issues.
}

void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0)
        return;                                 //no need to send anything
    memset(&t, 0, sizeof(t));                   //Zero out the transaction
    t.length = len * 8;                         //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;                         //Data
    t.user = (void *)1;                         //D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);                      //Should have had no issues.
}

void lcd_init(spi_device_handle_t spi)
{
    uint8_t cmd = 0;
    init_tft_pins();
    while (init_cmds[cmd].databytes != 0xff)
    {
        lcd_cmd(spi, init_cmds[cmd].cmd);
        lcd_data(spi, init_cmds[cmd].data, init_cmds[cmd].databytes & 0x1F);
        if (init_cmds[cmd].databytes & 0x80)
        {
            printf("delaying for command: %#x\n", init_cmds[cmd].cmd);
            vTaskDelay(150 / portTICK_RATE_MS);
        }
        cmd++;
    }
    printf("finished init");
    fflush(stdout);
}


void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}


void app_main()
{
    spi_device_handle_t device_handle;
    esp_err_t ret;

    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t dev_config = {

        .clock_speed_hz = 25000000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .pre_cb=lcd_spi_pre_transfer_callback,
        .queue_size = 8
        };

    ret = spi_bus_initialize(SPI2_HOST, &bus_config, 2);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI2_HOST, &dev_config, &device_handle);
    ESP_ERROR_CHECK(ret);
    lcd_init(device_handle);
}
