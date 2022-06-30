#include "driver_vibration_motor.h"

#include <stdint.h>

#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"

#define DRIVER_VIBRATION_MOTOR_TAG "driver_vibration_motor"

#define VIBRATION_MOTOR_GPIO 1

#define VIBRATION_MOTOR_CHANNEL LEDC_CHANNEL_5

#define VIBRATION_MOTOR_DUTY_RESOLUTION 7
#define VIBRATION_MOTOR_MAX_DUTY 128 // = 2 ** VIBRATION_MOTOR_DUTY_RESOLUTION, fully ON

void init_vibration_motor_timer(void)
{
	ledc_timer_config_t vibration_motor_timer;
	vibration_motor_timer.speed_mode      = LEDC_LOW_SPEED_MODE;
	vibration_motor_timer.timer_num       = LEDC_TIMER_2;
	vibration_motor_timer.duty_resolution = VIBRATION_MOTOR_DUTY_RESOLUTION;
	vibration_motor_timer.freq_hz         = 200;
	vibration_motor_timer.clk_cfg = LEDC_AUTO_CLK;

	ledc_timer_config(&vibration_motor_timer);
}

void init_vibration_motor_channel(void)
{
	ledc_channel_config_t ledc_channel;
		ledc_channel.channel    = VIBRATION_MOTOR_CHANNEL;
		ledc_channel.duty       = 0;
		ledc_channel.gpio_num   = VIBRATION_MOTOR_GPIO;
		ledc_channel.hpoint = 0;
		ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
		ledc_channel.timer_sel  = LEDC_TIMER_2;
		ledc_channel.intr_type = GPIO_INTR_DISABLE;
		ledc_channel_config(&ledc_channel);
}

void init_vibration_motor(void)
{
	init_vibration_motor_timer();
	init_vibration_motor_channel();
}

void inline ledc_change_duty(ledc_channel_t channel, uint32_t duty)
{
	ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void set_vibration_intensity(uint8_t duty)
{
		ledc_change_duty(VIBRATION_MOTOR_CHANNEL, duty);
}




