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

#include <linux/asusec.h>
#include <linux/delay.h>
#include <linux/gfp.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/string.h>

#define RSP_BUFFER_SIZE         8

struct asus_ec_initdata
{
	const char *model;
	const char *name;
	unsigned components;
	unsigned flags;
};

static const struct asus_ec_initdata asus_ec_model_info[] = {
	{	/* Asus Transformer Pad */
		.model		= "ASUS-TF201-PAD",
		.name		= "pad",
//		.components	= EC_PART_BATTERY|EC_PART_CHARGE_LED,
//		.flags		= EC_FLAG_SET_MODE,
	},
	{	/* Asus Mobile Dock */
		.model		= "ASUS-TF201-DOCK",
		.name		= "dock",
//		.components	= EC_PART_BATTERY|EC_PART_CHARGE_LED|
//				  EC_PART_I8042,
	},
};

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

static int asus_ec_log_info(struct i2c_client *dockram_client, unsigned reg,
			    const char *name, char **out)
{
	char buf[32];
	int ret;

	ret = asus_dockram_read(dockram_client, reg, buf);
	if (ret < 0)
		return ret;

	dev_info(&dockram_client->dev, "%-14s: %.*s\n", name, buf[0], buf + 1);

	if (out)
		*out = kstrndup(buf + 1, buf[0], GFP_KERNEL);

	return 0;
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

int asus_ec_irq_request(void *dev, int gpio, irq_handler_t handler,
			unsigned long flags, const char *label)
{
	int rc = 0;
	unsigned irq = gpio_to_irq(gpio);

	rc = gpio_request_one(gpio, GPIOF_DIR_IN, label);
	if (rc)
		pr_err("%s: gpio_request failed for input %d\n", __func__, gpio);

	rc = request_irq(irq, handler, flags, label, dev);
	if (rc < 0) {
		pr_err("%s: could not register for %s interrupt, irq = %d, rc = %d\n", __func__, label, irq, rc);
		rc = -EIO;
	}

	if (flags == IRQF_TRIGGER_LOW)
		enable_irq_wake(irq);

	if (rc)
		gpio_free(gpio);

	return rc;
}

static const struct asus_ec_initdata *asus_ec_match(const char *model)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(asus_ec_model_info); ++i) {
		if (!strcmp(model, asus_ec_model_info[i].model))
			return &asus_ec_model_info[i];
	}

	return NULL;
}

int asus_ec_detect(struct i2c_client *dockram_client, struct i2c_client *client, char *buf)
{
	const struct asus_ec_initdata *info;
	char *model = NULL;
	int ret;

	ret = asus_ec_reset(client);
	if (ret)
		goto err_exit;

	asus_ec_clear_buffer(client, buf);

	ret = asus_ec_log_info(dockram_client, 0x01, "Model", &model);
	if (ret)
		goto err_exit;

	ret = asus_ec_log_info(dockram_client, 0x02, "FW version", NULL);
	if (ret)
		goto err_exit;

	ret = asus_ec_log_info(dockram_client, 0x03, "Config format", NULL);
	if (ret)
		goto err_exit;

	ret = asus_ec_log_info(dockram_client, 0x04, "HW version", NULL);
	if (ret)
		goto err_exit;

	info = asus_ec_match(model);
	if (!info) {
		dev_err(&client->dev, "EC model not recognized\n");
		ret = -ENODEV;
		goto out_free;
	}

err_exit:
	if (ret)
		dev_err(&client->dev, "failed to access EC: %d\n", ret);
out_free:
	kfree(model);
	return ret;
}

int asus_ec_input_switch(struct i2c_client *client, u16 state)
{
	int retry, definer;
	u8 i2c_data[32];

	asus_ec_clear_buffer(client, i2c_data);
	asus_ec_write(client, state);

	for (retry = 0; retry < ASUSEC_RETRY_COUNT; retry++) {
		asus_ec_read(client, i2c_data);

		if ((state == ASUSDEC_KB_ENABLE) || (state == ASUSDEC_KB_DISABLE))
			definer = !(i2c_data[1] & ASUSEC_AUX_MASK);
		else
			definer = (i2c_data[1] & ASUSEC_AUX_MASK);

		if ((i2c_data[1] & ASUSEC_OBF_MASK) && definer) {
			if (i2c_data[2] == ASUSDEC_PS2_ACK)
				goto exit;
		}
		msleep(DELAY_TIME_MS);
	}

	dev_err(&client->dev, "failed to write 0x%x into EC\n", state);
	return -1;

exit:
	return 0;
}
