#include "driver_SSM2518.h"

#include "driver/i2s.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_err.h"

#define DRIVER_SSM2518_TAG "driver_SSM2518"

#define SSM2518_ENABLE_GPIO 10

#define SSM2518_I2C_SDA_GPIO 19
#define SSM2518_I2C_SCL_GPIO 18
#define SSM2518_I2C_MASTER_PORT 0
#define SSM2518_I2C_ADDRESS 0x34 //Depends on ADDR pin of SSM2518. 0x34 when GROUNDED, 0x35 when PULLED UP
#define SSM2518_RIGHT_VOLUME_REGISTER 0x06

#define SSM2518_I2S_MASTER_PORT 0
#define SSM2518_MCLK_GPIO 0
#define SSM2518_BCLK_GPIO 6
#define SSM2518_LRCLK_GPIO 5
#define SSM2518_SDATA_GPIO 4


#define VOLUME_MAX 128 //Up to unsigned 8 bit max: 255, however above 128 sound quality suffers.

void write_i2c_SSM2518(uint8_t ssm2518_subaddress, uint8_t data)
{
	uint8_t i2c_address = 0x34;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, ssm2518_subaddress, 1);
	i2c_master_write_byte(cmd, data, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(SSM2518_I2C_MASTER_PORT, cmd, 1000);
	i2c_cmd_link_delete(cmd);
}

void read_i2c_SSM2518(uint8_t ssm2518_subaddress, uint8_t* src_data)
{
	uint8_t i2c_address = 0x34;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, ssm2518_subaddress, 1);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_READ, 1);
	i2c_master_read_byte(cmd, src_data , 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(SSM2518_I2C_MASTER_PORT, cmd, 1000);
	i2c_cmd_link_delete(cmd);
}

void set_volume_SSM2518(uint8_t volume)
{
		write_i2c_SSM2518(SSM2518_RIGHT_VOLUME_REGISTER, volume);
}

void enable_SSM2518()
{
	gpio_config_t io_config = {
		.pin_bit_mask = 1UL << SSM2518_ENABLE_GPIO,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&io_config);

	gpio_set_level(SSM2518_ENABLE_GPIO, 1);
}

void disable_SSM2518()
{
	gpio_set_level(SSM2518_ENABLE_GPIO, 0);
}

void init_i2c()
{
	i2c_config_t i2c_conf;

	i2c_conf.mode = I2C_MODE_MASTER;
	i2c_conf.sda_io_num = SSM2518_I2C_SDA_GPIO;
	i2c_conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	i2c_conf.scl_io_num = SSM2518_I2C_SCL_GPIO;
	i2c_conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	i2c_conf.master.clk_speed = 100000;  // 400k max for SSM2518
	// i2c_conf.clk_flags = 0;          //!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here.
	if(i2c_param_config(SSM2518_I2C_MASTER_PORT, &i2c_conf) == ESP_OK && i2c_driver_install(SSM2518_I2C_MASTER_PORT, i2c_conf.mode, 0, 0, 0) == ESP_OK)
	{
		ESP_LOGI(DRIVER_SSM2518_TAG, "SSM2518 I2C connection initialized");
		return;
	}
	ESP_LOGE(DRIVER_SSM2518_TAG, "SSM2518 I2C connection initialization failed");
}

void init_i2s()
{
	const i2s_config_t i2s_config =
	{
		.mode = I2S_MODE_MASTER | I2S_MODE_TX,
		.sample_rate = 16000,
		.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
		.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
		.communication_format = I2S_COMM_FORMAT_STAND_I2S,
		.tx_desc_auto_clear = true,
		.dma_buf_count = 80,
		.dma_buf_len = 200, //dma_buf_len * dma_buf_count = 16000 samples, enough for 1 second buffering. = 32000 bytes
		.use_apll = false,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1  // Interrupt level 1, default 0
	};

	const i2s_pin_config_t i2s_pin_config =
	{
		.mck_io_num = 0,
		.bck_io_num = 6,
		.ws_io_num = 5,
		.data_out_num = 4,
		.data_in_num = -1,
	};


	if(i2s_driver_install(SSM2518_I2S_MASTER_PORT, &i2s_config, 0, NULL) == ESP_OK)
	{
		ESP_LOGI(DRIVER_SSM2518_TAG, "SSM2518 I2S driver installed");

	}
	else
	{
		ESP_LOGE(DRIVER_SSM2518_TAG, "SSM2518 I2S installing driver failed");
		return;
	}


	if(i2s_set_pin(SSM2518_I2S_MASTER_PORT, &i2s_pin_config) == ESP_OK)
	{
		ESP_LOGI(DRIVER_SSM2518_TAG, "SSM2518 I2S pins are set");
	}
	else
	{
		ESP_LOGE(DRIVER_SSM2518_TAG, "SSM2518 I2S setting pins failed");
		return;
	}

}

void i2s_write_SSM2518(const void *src, size_t size, size_t *bytes_written)
{
	i2s_write(SSM2518_I2S_MASTER_PORT, src, size, bytes_written, 100);
}

void test_i2s()
{
	//This will pause all of the program just to test if the SSM2518 works properly. When running, you should hear an audible high-frequency noise.
	size_t i2s_bytes_written = 0;
	uint16_t samples_data[64];

	for(uint16_t i=0; i<64; i++)
	{
		samples_data[i] = i * 1023;
	}

	while(1)
	{

		for(int i=0; i<64; i++)
		{
			uint16_t sample_data = samples_data[i];
			i2s_write_SSM2518(&sample_data, 1, &i2s_bytes_written);
		}

		for(int i=63; i>=0; i--)
		{
			uint16_t sample_data = samples_data[i];
			i2s_write_SSM2518(&sample_data, 1, &i2s_bytes_written);
		}
	}
}

void init_SSM2518()
{
	enable_SSM2518();
	init_i2c();
	//Later on the sub-register numbers should be more documented, but for now...
	write_i2c_SSM2518(0x09, 0b10011011);
	write_i2c_SSM2518(0x07, 0b00000000);
	write_i2c_SSM2518(0x03, 0b00000000);
	write_i2c_SSM2518(0x02, 0b00000010);
	write_i2c_SSM2518(0x01, 0b00000000);
	write_i2c_SSM2518(0x00, 0b00000100);
	ESP_LOGI(DRIVER_SSM2518_TAG, "SSM2518 configuration via I2C finished");

	init_i2s();
	//test_i2s(); //If uncommented, will infinitely test the SSM2518 making the rest of the device unfunctional.
}
