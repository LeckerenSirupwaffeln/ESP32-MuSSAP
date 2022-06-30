#include "driver_LEDs.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"

#define DRIVER_LEDS_TAG "driver_LEDs"

#define RED_LED_GPIO 8
#define GREEN_LED_GPIO 7
#define BLUE_LED_GPIO 3

#define RED_LED_CHANNEL LEDC_CHANNEL_0
#define GREEN_LED_CHANNEL LEDC_CHANNEL_1
#define BLUE_LED_CHANNEL LEDC_CHANNEL_2

#define LEDS_TIMER_DUTY_RESOLUTION 7
#define LEDS_MAX_DUTY 128 // = 2 ** LEDS_TIMER_DUTY_RESOLUTION, LEDs fully OFF
#define LEDS_MIN_DUTY 96 // LEDS_MAX_DUTY - LEDS_MIN_DUTY = usable duty cycle. Decrease this value to increase max LED brightness. At 0 LEDs have maximum brightness.

void init_LEDs_timer(void)
{
	ledc_timer_config_t LEDs_timer;
	LEDs_timer.speed_mode      = LEDC_LOW_SPEED_MODE;
	LEDs_timer.timer_num       = LEDC_TIMER_0;
	LEDs_timer.duty_resolution = LEDS_TIMER_DUTY_RESOLUTION;
	LEDs_timer.freq_hz         = 2000;
	LEDs_timer.clk_cfg = LEDC_AUTO_CLK;

	ledc_timer_config(&LEDs_timer);
}

void init_LEDs_channels(void)
{
	ledc_channel_config_t ledc_channel;

	//RED LED LEDC CHANNEL CONFIG
	ledc_channel.channel    = RED_LED_CHANNEL;
	ledc_channel.duty       = LEDS_MAX_DUTY;
	ledc_channel.gpio_num   = RED_LED_GPIO;
	ledc_channel.hpoint = 0;
	ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_channel.timer_sel  = LEDC_TIMER_0;
	ledc_channel.intr_type = GPIO_INTR_DISABLE;
	ledc_channel_config(&ledc_channel);

	//GREEN LED LEDC CHANNEL CONFIG
	ledc_channel.channel    = GREEN_LED_CHANNEL;
	ledc_channel.duty       = LEDS_MAX_DUTY;
	ledc_channel.gpio_num   = GREEN_LED_GPIO;
	ledc_channel.hpoint = 0;
	ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_channel.timer_sel  = LEDC_TIMER_0;
	ledc_channel.intr_type = GPIO_INTR_DISABLE;
	ledc_channel_config(&ledc_channel);

	//BLUE LED LEDC CHANNEL CONFIG
	ledc_channel.channel    = BLUE_LED_CHANNEL;
	ledc_channel.duty       = LEDS_MAX_DUTY;
	ledc_channel.gpio_num   = BLUE_LED_GPIO;
	ledc_channel.hpoint = 0;
	ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_channel.timer_sel  = LEDC_TIMER_0;
	ledc_channel.intr_type = GPIO_INTR_DISABLE;
	ledc_channel_config(&ledc_channel);
}

void init_LEDs(void)
{
	init_LEDs_timer();
	init_LEDs_channels();
}

void ledc_change_duty(ledc_channel_t channel, uint32_t duty)
{
	ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void set_red_LED_intensity(uint8_t duty)
{
		ledc_change_duty(RED_LED_CHANNEL, duty);
}

void set_green_LED_intensity(uint8_t duty)
{
		ledc_change_duty(GREEN_LED_CHANNEL, duty);
}

void set_blue_LED_intensity(uint8_t duty)
{
		ledc_change_duty(BLUE_LED_CHANNEL, duty);
}

