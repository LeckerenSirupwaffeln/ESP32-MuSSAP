#include <stddef.h>
#include <stdint.h>

#ifndef MAIN_DRIVER_SSM2518_H_
#define MAIN_DRIVER_SSM2518_H_

void init_SSM2518();
void i2s_write_SSM2518(const void *src, size_t size, size_t *bytes_written);
void set_volume_SSM2518(uint8_t volume);

#endif
