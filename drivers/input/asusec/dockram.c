/*
 * ASUS EC: DockRAM
 *
 * Written by: Michal Miroslaw <mirq-linux@rere.qmqm.pl>
 *
 * Copyright (C) 2017 Michal Miroslaw
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/asusec.h>
#include <linux/i2c.h>
#include <linux/string.h>

int asus_dockram_init(struct i2c_client *dockram_client, struct i2c_client *client, unsigned short addr)
{
	dockram_client->adapter = client->adapter;
	dockram_client->addr = addr;
	dockram_client->detected = client->detected;
	dockram_client->dev = client->dev;
	dockram_client->driver = client->driver;
	dockram_client->flags = client->flags;
	strcpy(dockram_client->name, client->name);

	return 0;
}

int asus_dockram_read(struct i2c_client *dockram_client, int reg, char *buf)
{
	int rc;

	memset(buf, 0, DOCKRAM_ENTRY_BUFSIZE);
	rc = i2c_smbus_read_i2c_block_data(dockram_client, reg, DOCKRAM_ENTRY_BUFSIZE, buf);

	if (buf[0] > DOCKRAM_ENTRY_SIZE) {
		dev_err(&dockram_client->dev, "bad data len; buffer: %*ph; rc: %d\n",
			DOCKRAM_ENTRY_BUFSIZE, buf, rc);
		return -EPROTO;
	}

	dev_dbg(&dockram_client->dev, "got data; buffer: %*ph; rc: %d\n",
		DOCKRAM_ENTRY_BUFSIZE, buf, rc);

	return rc;
}

int asus_dockram_write(struct i2c_client *dockram_client, int reg, const char *buf)
{
	if (buf[0] > DOCKRAM_ENTRY_SIZE)
		return -EINVAL;

	dev_dbg(&dockram_client->dev, "sending data; buffer: %*ph\n", 9, buf);

	return i2c_smbus_write_i2c_block_data(dockram_client, reg, 9, buf);
}
