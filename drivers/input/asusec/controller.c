/*
 * ASUS EC: Controller
 *
 * Written by: Michal Miroslaw <mirq-linux@rere.qmqm.pl>
 *
 * Copyright (C) 2017 Michal Miroslaw
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/string.h>

#include <asm/gpio.h>

#include "asusec.h"

#define RSP_BUFFER_SIZE         8

int asus_ec_signal_request(struct i2c_client *client, int ecreq_gpio)
{
	dev_dbg(&client->dev, "EC request\n");

	gpio_set_value(ecreq_gpio, 0);
	msleep(50);
	gpio_set_value(ecreq_gpio, 1);
	msleep(200);
	
	return 0;
}

int asus_ec_read(struct i2c_client *client, char *buf)
{
	int ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, buf);
	dev_dbg(&client->dev, "EC read: %*ph, ret = %d\n", 8, buf, ret);
	return ret;
}

int asus_ec_write(struct i2c_client *client, u16 data)
{
	int ret = i2c_smbus_write_word_data(client, 0x64, data);
	dev_dbg(&client->dev, "EC write: %04x, ret = %d\n", data, ret);
	return ret;
}

void asus_ec_clear_buffer(struct i2c_client *client, char *buf)
{
	int retry = RSP_BUFFER_SIZE;

	while (retry--) {
		if (asus_ec_read(client, buf) < 0)
			continue;

		break;
	}
}

int asus_ec_reset(struct i2c_client *client)
{
	int retry, ret;

	for (retry = 0; retry < 3; ++retry) {
		ret = asus_ec_write(client, 0x0000);
		if (!ret)
			return 0;

		msleep(300);
	}

	return ret;
}
