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

#ifndef _ELAN_I2C_ASUS_H
#define _ELAN_I2C_ASUS_H

#define ELAN_RETRY_COUNT		3

#define PSMOUSE_CMD_SETSCALE11	0x00e6
#define PSMOUSE_CMD_GETINFO	0x03e9

#define PSMOUSE_RET_ACK		0xfa
#define PSMOUSE_RET_NAK		0xfe

/*
 * Command values for Synaptics style queries
 */
#define ETP_FW_ID_QUERY			0x00

/*
 * Command values for register reading or writing
 */
#define ETP_REGISTER_READ		0x10
#define ETP_REGISTER_WRITE		0x11
#define ETP_REGISTER_READWRITE		0x00

/*
 * Hardware version 2 custom PS/2 command value
 */
#define ETP_PS2_CUSTOM_COMMAND		0x00f8

/*
 * Hardware version 3 provides command to detect resolution
 */
#define ETP_PMIN_V2			0
#define ETP_PMAX_V2			255
#define ETP_WMIN_V2			0
#define ETP_WMAX_V2			15

/*
 * v3 hardware has 2 kinds of packet types.
 */
#define PACKET_UNKNOWN			0x01
#define PACKET_DEBOUNCE			0x02
#define PACKET_V3_HEAD			0x03
#define PACKET_V3_TAIL			0x04

/*
 * track up to 5 fingers for v4 hardware
 */
#define ETP_MAX_FINGERS			5

/*
 * The base position for one finger, v4 hardware
 */
struct finger_pos {
	unsigned int x;
	unsigned int y;
};

struct elantech_data {
	struct input_dev *input_dev;

	unsigned int y_max;
	unsigned int width;

	struct finger_pos mt[ETP_MAX_FINGERS];

	bool reports_pressure;
};

int elantech_init(struct asusdec_chip *ec_chip);
void asusdec_touchpad_processing(struct asusdec_chip *ec_chip);
#endif
