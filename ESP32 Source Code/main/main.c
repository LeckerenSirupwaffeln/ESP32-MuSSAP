#include "driver_LEDs.h"
#include "driver_vibration_motor.h"
#include "driver_SSM2518.h"
#include "server_gatts.h"

void app_main(void)
{
	init_LEDs();
	init_vibration_motor();
	init_SSM2518();
	init_server_gatts();
}

