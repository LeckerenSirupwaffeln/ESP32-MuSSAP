#include <stdint.h>

#ifndef MAIN_DRIVER_LEDS_H_
#define MAIN_DRIVER_LEDS_H_

	void init_LEDs(void);
	void set_red_LED_intensity(uint8_t duty);
	void set_green_LED_intensity(uint8_t duty);
	void set_blue_LED_intensity(uint8_t duty);

#endif
