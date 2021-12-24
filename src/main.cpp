#include <Arduino.h>
#include "led_strip.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <U8g2lib.h>
#include <Wire.h>

#define COLOR_ORDER GRB
#define NUM_LEDS 9

#define LED_PIN GPIO_NUM_7
#define SDA 5
#define SCL 6

#define BUZ 8
#define RMT_TX_CHANNEL RMT_CHANNEL_0

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i)
    {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

uint32_t red = 0;
uint32_t green = 0;
uint32_t blue = 0;
uint16_t hue = 0;
uint16_t start_rgb = 0;
led_strip_t *strip;
rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_PIN, RMT_TX_CHANNEL);

void esp32_beep(unsigned int freq, unsigned int dur_ms)
{
    ledc_timer_config_t ledc_timer;
    ledc_channel_config_t ledc_channel;

    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT; // resolution of PWM duty
    ledc_timer.freq_hz = freq;                      // frequency of PWM signal
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;    // timer mode
    ledc_timer.timer_num = LEDC_TIMER_0;            // timer index
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;             // timer index

    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = BUZ;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;

    ledc_channel_config(&ledc_channel);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4095));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    vTaskDelay(pdMS_TO_TICKS(dur_ms));
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

void setup()
{
    delay(3000); // 3 second delay for recovery

    u8g2.begin();
    u8g2.clearBuffer();                         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);         // choose a suitable font
    u8g2.drawStr(2, 10, "Amplience Hackathon"); // write something to the internal memory
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(40, 30, "2021");    // write something to the internal memory
    u8g2.drawStr(25, 55, "Winner!"); // write something to the internal memory
    u8g2.sendBuffer();

    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(NUM_LEDS, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);

    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    int melody[] = {
        830, 830, 830, 830, 660, 740, 830, 740, 830};

    // note durations: 4 = quarter note, 8 = eighth note, etc.:
    int noteDurations[] = {
        8, 8, 8, 4, 3, 3, 5, 8, 5};

    for (int thisNote = 0; thisNote < 9; thisNote++)
    {
        int noteDuration = 1000 / noteDurations[thisNote];
        esp32_beep(melody[thisNote], noteDuration);
        int pauseBetweenNotes = noteDuration * 1.30;
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

void loop()
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = i; j < NUM_LEDS; j += 3)
        {
            hue = j * 360 / NUM_LEDS + start_rgb;
            led_strip_hsv2rgb(hue, 100, 50, &red, &green, &blue);
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
        }
        ESP_ERROR_CHECK(strip->refresh(strip, 50));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    start_rgb += 5;
}
