/*
 * Elantech Touchpad driver (v2)
 *
 * Copyright (c) 2007-2009 Arjan Opmeer <arjan@opmeer.net>
 * Copyright (c) 2012, ASUSTek Corporation.
 * Copyright (c) 2019, Svyatoslav Ryhel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/asusec.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>

#include "elantech.h"

/*
 * This driver is simplified version of elantech touchpad driver.
 * Unfortunatelly generic driver can't be used because of asus-ec
 * dependency. As a elantech_ps2_command this driver uses asus-ec-rw
 * functions which causes bit mask shift and usage of different registers.
 * Touchpad used in transformers has hw revision 3 of elantech labeling.
 *
 * Laptop-model:           fw_version:     hw_version:     buttons:
 * Asus TF                 0x450f02        3               2 hw buttons
 */

/*
 * Non-Elantech generic function, but labeled as such for simpler work
 */
static int elantech_ps2_command(struct i2c_client *client, unsigned char *param, int command)
{
	u16 asus_ec_cmd;
	int ret = 0;
	int retry = ELAN_RETRY_COUNT;
	int i, retry_data_count;
	u8 i2c_data[16];

	asus_ec_cmd = (((command & 0x00ff) << 8) | 0xD4);
	ret = asus_ec_write(client, asus_ec_cmd);
	if (ret < 0) {
		pr_err("elantech: %s: write to device fails status %x\n", __func__, ret);
		return ret;
	}

	msleep(DELAY_TIME_MS);

	while(retry-- > 0){
		ret = asus_ec_read(client, i2c_data);
		if (ret < 0) {
			pr_err("elantech: %s: fail to read data, status %d\n", __func__, ret);
			return ret;
		}

		if ((i2c_data[1] & ASUSEC_OBF_MASK) &&
			(i2c_data[1] & ASUSEC_AUX_MASK)) {
			if (i2c_data[2] == PSMOUSE_RET_ACK)
				break;
			else if (i2c_data[2] == PSMOUSE_RET_NAK)
				goto fail_elan_touchpad_i2c;
		}
		msleep(DELAY_TIME_MS/5);
	}

	retry_data_count = (command & 0x0f00) >> 8;
	for (i = 1; i <= retry_data_count; i++)
		param[i-1] = i2c_data[i+2];

	return 0;

fail_elan_touchpad_i2c:
	pr_err("elantech: %s: fail to get touchpad response", __func__);
	return -1;
}

static void elantech_set_slot(struct input_dev *dev, int slot, bool active,
			      unsigned int x, unsigned int y)
{
	input_mt_slot(dev, slot);
	input_mt_report_slot_state(dev, MT_TOOL_FINGER, active);
	if (active) {
		input_report_abs(dev, ABS_MT_POSITION_X, x);
		input_report_abs(dev, ABS_MT_POSITION_Y, y);
	}
}

/* x1 < x2 and y1 < y2 when two fingers, x = y = 0 when not pressed */
static void elantech_report_semi_mt_data(struct input_dev *dev,
					 unsigned int num_fingers,
					 unsigned int x1, unsigned int y1,
					 unsigned int x2, unsigned int y2)
{
	elantech_set_slot(dev, 0, num_fingers != 0, x1, y1);
	elantech_set_slot(dev, 1, num_fingers >= 2, x2, y2);
}

/*
 * Interpret complete data packets and report absolute mode input events for
 * hardware version 3. (12 byte packets for two fingers)
 */
static void elantech_report_absolute(struct asusdec_chip *ec_chip,
					int packet_type)
{
	struct elantech_data *etd = (struct elantech_data *) ec_chip->private;
	struct input_dev *dev = etd->input_dev;
	unsigned char *packet = ec_chip->ec_data;
	unsigned int fingers, x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	unsigned int width = 0, pres = 0;

	/* byte 0: n1  n0   .   .   .   .   R   L */
	fingers = (packet[0] & 0xc0) >> 6;

	switch (fingers) {
	case 3:
	case 1:
		/*
		 * byte 1:  .   .   .   .  x11 x10 x9  x8
		 * byte 2: x7  x6  x5  x4  x4  x2  x1  x0
		 */
		x1 = ((packet[1] & 0x0f) << 8) | packet[2];
		/*
		 * byte 4:  .   .   .   .  y11 y10 y9  y8
		 * byte 5: y7  y6  y5  y4  y3  y2  y1  y0
		 */
		y1 = etd->y_max - (((packet[4] & 0x0f) << 8) | packet[5]);
		break;

	case 2:
		if (packet_type == PACKET_V3_HEAD) {
			/*
			 * byte 1:   .    .    .    .  ax11 ax10 ax9  ax8
			 * byte 2: ax7  ax6  ax5  ax4  ax3  ax2  ax1  ax0
			 */
			etd->mt[0].x = ((packet[1] & 0x0f) << 8) | packet[2];
			/*
			 * byte 4:   .    .    .    .  ay11 ay10 ay9  ay8
			 * byte 5: ay7  ay6  ay5  ay4  ay3  ay2  ay1  ay0
			 */
			etd->mt[0].y = etd->y_max -
				(((packet[4] & 0x0f) << 8) | packet[5]);
			/*
			 * wait for next packet
			 */
			return;
		}

		/* packet_type == PACKET_V3_TAIL */
		x1 = etd->mt[0].x;
		y1 = etd->mt[0].y;
		x2 = ((packet[1] & 0x0f) << 8) | packet[2];
		y2 = etd->y_max - (((packet[4] & 0x0f) << 8) | packet[5]);
		break;
	}

	pres = (packet[1] & 0xf0) | ((packet[4] & 0xf0) >> 4);
	width = ((packet[0] & 0x30) >> 2) | ((packet[3] & 0x30) >> 4);

	input_report_key(dev, BTN_TOUCH, fingers != 0);

	if (fingers != 0) {
		input_report_abs(dev, ABS_X, x1);
		input_report_abs(dev, ABS_Y, y1);
	}

	elantech_report_semi_mt_data(dev, fingers, x1, y1, x2, y2);

	input_report_key(dev, BTN_TOOL_FINGER, fingers == 1);
	input_report_key(dev, BTN_TOOL_DOUBLETAP, fingers == 2);
	input_report_key(dev, BTN_TOOL_TRIPLETAP, fingers == 3);

	input_report_key(dev, BTN_LEFT, packet[0] & 0x01);
	input_report_key(dev, BTN_RIGHT, packet[0] & 0x02);

	input_report_abs(dev, ABS_PRESSURE, pres);
	input_report_abs(dev, ABS_TOOL_WIDTH, width);

	input_sync(dev);
}

/*
 * We check the constant bits to determine what packet type we get,
 * so packet checking is mandatory for v3 and later hardware.
 */
static int elantech_packet_check(struct asusdec_chip *ec_chip)
{
	const u8 debounce_packet[] = { 0xc4, 0xff, 0xff, 0x02, 0xff, 0xff };
	unsigned char *packet = ec_chip->ec_data;

	/*
	 * check debounce first, it has the same signature in byte 0
	 * and byte 3 as PACKET_V3_HEAD.
	 */
	if (!memcmp(packet, debounce_packet, sizeof(debounce_packet)))
		return PACKET_DEBOUNCE;

	if ((packet[0] & 0x0c) == 0x04 && (packet[3] & 0xcf) == 0x02)
		return PACKET_V3_HEAD;

	if ((packet[0] & 0x0c) == 0x0c && (packet[3] & 0xce) == 0x0c)
		return PACKET_V3_TAIL;

	return PACKET_UNKNOWN;
}

/*
 * Process byte stream from mouse and handle complete packets
 */
static void elantech_process_byte(struct asusdec_chip *ec_chip)
{
	int packet_type;

	if ((ec_chip->tp_enable) && (!ec_chip->tp_model)){

		packet_type = elantech_packet_check(ec_chip);

		/* ignore debounce */
		if (packet_type == PACKET_DEBOUNCE)
			return; 	// PSMOUSE_FULL_PACKET

		if (packet_type == PACKET_UNKNOWN)
			return; 	// PSMOUSE_BAD_DATA

		elantech_report_absolute(ec_chip, packet_type);

	} else if (ec_chip->tp_model){
		ec_chip->susb_on = 1;
		ec_chip->init_success = -1;
		asus_ec_signal_request(ec_chip->client, DOCK_ECREQ_GPIO);
		ec_chip->dock_init = 0;
	}
}

void asusdec_touchpad_processing(struct asusdec_chip *ec_chip)
{
	int i;
	int length = 0;
	int tp_start = 0;

	length = ec_chip->i2c_data[0];

	if (ec_chip->tp_wait_ack) {
		ec_chip->tp_wait_ack = 0;
		tp_start = 1;
		ec_chip->d_index = 0;
	} else
		tp_start = 0;

	for (i = tp_start; i < length - 1 ; i++) {
		ec_chip->ec_data[ec_chip->d_index] = ec_chip->i2c_data[i+2];
		ec_chip->d_index++;
		if (ec_chip->d_index == 6) {
			elantech_process_byte(ec_chip);
			ec_chip->d_index = 0;
		}
	}

	if (ec_chip->d_index)
		mod_timer(&ec_chip->asusdec_timer, jiffies + (HZ * 1/20));
}

/*
 * V3 and later support this fast command
 */
static int elantech_send_cmd(struct i2c_client *client, unsigned char c,
				unsigned char *param)
{
	if (elantech_ps2_command(client, NULL, ETP_PS2_CUSTOM_COMMAND) ||
	    elantech_ps2_command(client, NULL, c) ||
	    elantech_ps2_command(client, param, PSMOUSE_CMD_GETINFO)) {
		dev_err(&client->dev, "%s query 0x%02x failed.\n", __func__, c);
		return -1;
	}

	return 0;
}

/*
 * Send an Elantech style special command to write a register with a value
 */
static int elantech_write_reg(struct i2c_client *client, unsigned char reg,
				unsigned char val)
{
	int rc = 0;

	if (elantech_ps2_command(client, NULL, ETP_PS2_CUSTOM_COMMAND) ||
	    elantech_ps2_command(client, NULL, ETP_REGISTER_READWRITE) ||
	    elantech_ps2_command(client, NULL, ETP_PS2_CUSTOM_COMMAND) ||
	    elantech_ps2_command(client, NULL, reg) ||
	    elantech_ps2_command(client, NULL, ETP_PS2_CUSTOM_COMMAND) ||
	    elantech_ps2_command(client, NULL, val) ||
	    elantech_ps2_command(client, NULL, PSMOUSE_CMD_SETSCALE11)) {
		rc = -1;
	}

	if (rc)
		dev_err(&client->dev, "failed to write register 0x%02x with value 0x%02x.\n",
			    reg, val);

	return rc;
}

/*
 * Put the touchpad into absolute mode
 */
static int elantech_set_absolute_mode(struct i2c_client *client)
{
	int rc = 0;

	/* Since hw_version = 3 set proper reg and val */
	unsigned char reg = 0x0010; // orig
	unsigned char val = 0x03;   // orig

	rc = elantech_write_reg(client, reg, val);
	if (rc)
		dev_err(&client->dev, "failed to initialise registers.\n");

	return rc;
}

static int elantech_set_range(struct i2c_client *client,
			      unsigned int *x_min, unsigned int *y_min,
			      unsigned int *x_max, unsigned int *y_max,
			      unsigned int *width)
{
	unsigned char param[3];

	if (elantech_send_cmd(client, ETP_FW_ID_QUERY, param))
		return -1;

	*x_max = (0x0f & param[0]) << 8 | param[1];
	*y_max = (0xf0 & param[0]) << 4 | param[2];

	return 0;
}

/*
 * Set the appropriate event bits for the input subsystem
 */
static int elantech_set_input_params(struct asusdec_chip *ec_chip)
{
	struct elantech_data *etd = ec_chip->private;
	struct i2c_client *client = ec_chip->client;

	unsigned int x_min = 0, y_min = 0, x_max = 0, y_max = 0, width = 0;
	int ret;

	if (elantech_set_range(client, &x_min, &y_min, &x_max, &y_max, &width))
		return -1;

	if (etd->input_dev)
		return 0;

	etd->input_dev = input_allocate_device();
	if (!etd->input_dev) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	etd->input_dev->name = "elantech_touchscreen";

	__set_bit(INPUT_PROP_POINTER, etd->input_dev->propbit);
	__set_bit(EV_KEY, etd->input_dev->evbit);
	__set_bit(EV_ABS, etd->input_dev->evbit);
	__clear_bit(EV_REL, etd->input_dev->evbit);

	__set_bit(BTN_LEFT, etd->input_dev->keybit);
	__set_bit(BTN_RIGHT, etd->input_dev->keybit);

	__set_bit(BTN_TOUCH, etd->input_dev->keybit);
	__set_bit(BTN_TOOL_FINGER, etd->input_dev->keybit);
	__set_bit(BTN_TOOL_DOUBLETAP, etd->input_dev->keybit);
	__set_bit(BTN_TOOL_TRIPLETAP, etd->input_dev->keybit);

	input_set_abs_params(etd->input_dev, ABS_X, x_min, x_max, 0, 0);
	input_set_abs_params(etd->input_dev, ABS_Y, y_min, y_max, 0, 0);

	input_set_abs_params(etd->input_dev, ABS_PRESSURE, ETP_PMIN_V2, ETP_PMAX_V2, 0, 0);
	input_set_abs_params(etd->input_dev, ABS_TOOL_WIDTH, ETP_WMIN_V2, ETP_WMAX_V2, 0, 0);

	input_mt_init_slots(etd->input_dev, 2);
	input_set_abs_params(etd->input_dev, ABS_MT_POSITION_X, x_min, x_max, 0, 0);
	input_set_abs_params(etd->input_dev, ABS_MT_POSITION_Y, y_min, y_max, 0, 0);

	etd->y_max = y_max;
	etd->width = width;

	ret = input_register_device(etd->input_dev);
	if (ret)
		pr_err("elantech: %s: Unable to register %s input device\n", __func__, etd->input_dev->name);

	return 0;
}

/*
 * Initialize the touchpad and create sysfs entries
 */
int elantech_init(struct asusdec_chip *ec_chip)
{
	struct i2c_client *client = ec_chip->client;
	int rc = 0;

	rc = elantech_set_absolute_mode(client);
	if (rc){
		pr_err("elantech: %s: failed to put touchpad into absolute mode.\n", __func__);
		return rc;
	}

	rc = elantech_set_input_params(ec_chip);
	if (rc){
		pr_err("elantech: %s: failed to set input rel params.\n", __func__);
		return rc;
	}

	pr_info("elantech: 2.6.2X-Elan-touchpad-2010-11-27\n");
	pr_info("elantech: Elan et1059 elantech_init\n");

	return 0;
}

MODULE_DESCRIPTION("Elan Touchpad Driver");
MODULE_LICENSE("GPL");
