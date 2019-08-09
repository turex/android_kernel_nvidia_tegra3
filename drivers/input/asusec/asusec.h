/*
 * ASUS EC driver
 *
 * Copyright (c) 2012, ASUSTek Corporation.
 * Copyright (c) 2018, Svyatoslav Ryhel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ASUSEC_H
#define _ASUSEC_H

#include <linux/interrupt.h>
#include <linux/switch.h>
#include <linux/wakelock.h>

/*
 * compiler option
 */
#define FACTORY_MODE			0
#define TOUCHPAD_MODE			1	// 0: relative mode, 1: absolute mode
#define DOCK_USB				1	// 0: not ready, 1: ready
#define BATTERY_DRIVER			1	// 0: not ready, 1: ready

//-----------------------------------------

struct i2c_client;

/* dockram comm */

int asus_dockram_init(struct i2c_client *dockram_client, struct i2c_client *client, unsigned short addr);
int asus_dockram_read(struct i2c_client *dockram_client, int reg, char *buf);
int asus_dockram_write(struct i2c_client *dockram_client, int reg, const char *buf);

/* ec comm */

int asus_ec_detect(struct i2c_client *dockram_client, struct i2c_client *client, char *buf);
int asus_ec_read(struct i2c_client *client, char *buf);
int asus_ec_write(struct i2c_client *client, u16 data);
int asus_ec_reset(struct i2c_client *client);
int asus_ec_signal_request(struct i2c_client *client, int ecreq_gpio);
int asus_ec_irq_request(struct i2c_client *client, int gpio, irq_handler_t handler,
			unsigned long flags, const char *label);
void asus_ec_clear_buffer(struct i2c_client *client, char *buf);

#define DOCKRAM_ENTRIES                 0x100
#define DOCKRAM_ENTRY_SIZE              32
#define DOCKRAM_ENTRY_BUFSIZE           (DOCKRAM_ENTRY_SIZE + 1)

#define ASUSPEC_DOCKRAM_ADDR            0x17
#define ASUSDEC_DOCKRAM_ADDR            0x1b

#define DELAY_TIME_MS                   50

#define ASUSEC_RETRY_COUNT              3

#define ASUSEC_OBF_MASK                 0x01
#define ASUSEC_KEY_MASK                 0x04
#define ASUSEC_KBC_MASK                 0x08
#define ASUSEC_AUX_MASK                 0x20
#define ASUSEC_SCI_MASK                 0x40
#define ASUSEC_SMI_MASK                 0x80

#define ASUSDEC_PS2_ACK                 0xFA
#define ASUSDEC_TP_ENABLE               0xF4D4
#define ASUSDEC_TP_DISABLE              0xF5D4
#define ASUSDEC_KP_ENABLE               0xF400
#define ASUSDEC_KP_DISABLE              0xF500

/*            - IRQ labels -              */
#define ASUSPEC_REQUEST                 "asuspec_request"
#define ASUSPEC_APWAKE                  "asuspec_apwake"
#define ASUSDEC_REQUEST                 "asusdec_request"
#define ASUSDEC_INPUT                   "asusdec_input"
#define ASUSDEC_DOCKIN                  "asusdec_dock_in"

/*         - Function keys row -          */
#define ASUSDEC_KEYPAD_ESC              0x76
#define ASUSDEC_KEYPAD_LOCK             0xE071
#define ASUSDEC_KEYPAD_KEY_BREAK        0xF0
#define ASUSDEC_KEYPAD_KEY_EXTEND       0xE0
#define ASUSDEC_KEY_TP_SWITCH           0x3C
#define ASUSDEC_KEY_TOUCHPAD            KEY_F2
#define ASUSDEC_KEY_AUTOBRIGHT          KEY_F3
#define ASUSDEC_KEY_SETTING             KEY_F4

/*           - First key row -            */
#define ASUSDEC_KEYPAD_KEY_WAVE         0x0E
#define ASUSDEC_KEYPAD_KEY_1            0x16
#define ASUSDEC_KEYPAD_KEY_2            0X1E
#define ASUSDEC_KEYPAD_KEY_3            0x26
#define ASUSDEC_KEYPAD_KEY_4            0x25
#define ASUSDEC_KEYPAD_KEY_5            0x2E
#define ASUSDEC_KEYPAD_KEY_6            0x36
#define ASUSDEC_KEYPAD_KEY_7            0x3D
#define ASUSDEC_KEYPAD_KEY_8            0x3E
#define ASUSDEC_KEYPAD_KEY_9            0x46
#define ASUSDEC_KEYPAD_KEY_0            0x45
#define ASUSDEC_KEYPAD_KEY_MINUS        0x4E
#define ASUSDEC_KEYPAD_KEY_EQUAL        0x55
#define ASUSDEC_KEYPAD_KEY_BACKSPACE    0x66

/*           - Second key row -           */
#define ASUSDEC_KEYPAD_KEY_TAB          0x0D
#define ASUSDEC_KEYPAD_KEY_Q            0x15
#define ASUSDEC_KEYPAD_KEY_W            0x1D
#define ASUSDEC_KEYPAD_KEY_E            0x24
#define ASUSDEC_KEYPAD_KEY_R            0x2D
#define ASUSDEC_KEYPAD_KEY_T            0x2C
#define ASUSDEC_KEYPAD_KEY_Y            0x35
#define ASUSDEC_KEYPAD_KEY_U            0x3C
#define ASUSDEC_KEYPAD_KEY_I            0x43
#define ASUSDEC_KEYPAD_KEY_O            0x44
#define ASUSDEC_KEYPAD_KEY_P            0x4D
#define ASUSDEC_KEYPAD_KEY_LEFTBRACE    0x54
#define ASUSDEC_KEYPAD_KEY_RIGHTBRACE   0x5B
#define ASUSDEC_KEYPAD_KEY_BACKSLASH    0x5D

/*            - Third key row -           */
#define ASUSDEC_KEYPAD_KEY_CAPSLOCK     0x58
#define ASUSDEC_KEYPAD_KEY_A            0x1C
#define ASUSDEC_KEYPAD_KEY_S            0x1B
#define ASUSDEC_KEYPAD_KEY_D            0x23
#define ASUSDEC_KEYPAD_KEY_F            0x2B
#define ASUSDEC_KEYPAD_KEY_G            0x34
#define ASUSDEC_KEYPAD_KEY_H            0x33
#define ASUSDEC_KEYPAD_KEY_J            0x3B
#define ASUSDEC_KEYPAD_KEY_K            0x42
#define ASUSDEC_KEYPAD_KEY_L            0x4B
#define ASUSDEC_KEYPAD_KEY_SEMICOLON    0x4C
#define ASUSDEC_KEYPAD_KEY_APOSTROPHE   0x52
#define ASUSDEC_KEYPAD_KEY_ENTER        0x5A

/*           - Fourth key row -           */
#define ASUSDEC_KEYPAD_KEY_LEFTSHIFT    0x12
#define ASUSDEC_KEYPAD_KEY_Z            0x1A
#define ASUSDEC_KEYPAD_KEY_X            0x22
#define ASUSDEC_KEYPAD_KEY_C            0x21
#define ASUSDEC_KEYPAD_KEY_V            0x2A
#define ASUSDEC_KEYPAD_KEY_B            0x32
#define ASUSDEC_KEYPAD_KEY_N            0x31
#define ASUSDEC_KEYPAD_KEY_M            0x3A
#define ASUSDEC_KEYPAD_KEY_COMMA        0x41
#define ASUSDEC_KEYPAD_KEY_DOT          0x49
#define ASUSDEC_KEYPAD_KEY_SLASH        0x4A
#define ASUSDEC_KEYPAD_KEY_RIGHTSHIFT   0x59
#define ASUSDEC_KEYPAD_KEY_UP           0xE075
#define ASUSDEC_KEYPAD_PAGEUP           0xE07D
#define ASUSDEC_KEYPAD_RIGHTWIN         0xE027

/*            - Fifth key row -           */
#define ASUSDEC_KEYPAD_LEFTCTRL         0x14
#define ASUSDEC_KEYPAD_LEFTWIN          0xE01F
#define ASUSDEC_KEYPAD_LEFTALT          0x11
#define ASUSDEC_KEYPAD_KEY_SPACE        0x29
#define ASUSDEC_KEYPAD_RIGHTALT         0xE011
#define ASUSDEC_KEYPAD_WINAPP           0xE02F
#define ASUSDEC_KEYPAD_RIGHTCTRL        0xE014
#define ASUSDEC_KEYPAD_KEY_LEFT         0xE06B
#define ASUSDEC_KEYPAD_KEY_DOWN         0xE072
#define ASUSDEC_KEYPAD_KEY_RIGHT        0xE074
#define ASUSDEC_KEYPAD_HOME             0xE06C
#define ASUSDEC_KEYPAD_PAGEDOWN         0xE07A
#define ASUSDEC_KEYPAD_END              0xE069

/*                JP keys                 */
#define ASUSDEC_HANKAKU_ZENKAKU         0x5F
#define ASUSDEC_YEN                     0x6A
#define ASUSDEC_MUHENKAN                0x67
#define ASUSDEC_HENKAN                  0x64
#define ASUSDEC_HIRAGANA_KATAKANA       0x13
#define ASUSDEC_RO                      0x51

/*                UK keys                 */
#define ASUSDEC_EUROPE_2                0x61

/*               SMI event                */
#define ASUSEC_SMI_HANDSHAKING          0x50
#define ASUSEC_SMI_WAKE                 0x53
#define ASUSEC_SMI_RESET                0x5F
#define ASUSEC_SMI_ADAPTER_EVENT        0x60
#define ASUSEC_SMI_BACKLIGHT_ON         0x63

/*
 * data struct
 */

struct asusdec_keypad {
	int value;
	int input_keycode;
	int extend;
};

struct asusdec_touchpad_relative {
	int y_overflow;
	int x_overflow;
	int y_sign;
	int x_sign;
	int left_btn;
	int right_btn;
	int delta_x;
	int delta_y;
};

struct asusdec_touchpad_absolute {
	int w_val;
	int x_pos;
	int y_pos;
	int z_val;
	int left;
	int right;
	int x_prev;
	int y_prev;
	int z_prev;
	int x2_pos;
	int y2_pos;
	int z2_val;
};

struct asusdec_chip {
	struct input_dev	*indev;
	struct i2c_client	*client;
	struct switch_dev 	dock_sdev;
	struct mutex		input_lock;
	struct mutex		dock_init_lock;
	struct wake_lock 	wake_lock;
	struct wake_lock 	wake_lock_init;
	struct delayed_work asusdec_work;
	struct delayed_work asusdec_dock_init_work;
	struct delayed_work asusdec_led_on_work;
	struct delayed_work asusdec_led_off_work;
	struct delayed_work asusdec_pad_battery_report_work;
	struct asusdec_keypad keypad_data;
	struct elantech_data *private;
	struct timer_list asusdec_timer;

#if TOUCHPAD_MODE
	struct asusdec_touchpad_absolute t_abs;
#else
	struct asusdec_touchpad_relative touchpad_data;
#endif

	int bc;			// byte counter
	int status;
	int dock_in;	// 0: without dock, 1: with dock
	int kbc_value;	// capslock_led 0: led off, 1: led on
	int dock_init;	// 0: dock not init, 1: dock init successfully
	int d_index;	// touchpad byte counter
	int init_success; // 0: ps/2 not ready. 1: init OK, -1: tp not ready
	int tp_model;	// 0: touchpad init ok, -1: touchpad init fail
	int tp_wait_ack;	// 0 : normal mode, 1: waiting for an ACK
	int tp_enable;		// 0 : touchpad has not enabled, 1: touchpad has enabled
	int ec_in_s3;		// 0: normal mode, 1: ec in deep sleep mode
	int susb_on;	// 0: susb off, 1: susb on

	u8 ec_data[32];
	u8 i2c_data[32];
	u8 i2c_dm_data[32];
};

struct asuspec_chip {
	struct i2c_client	*client;
	struct mutex		state_change_lock;

	struct delayed_work asuspec_work;
	struct delayed_work asuspec_init_work;
	struct delayed_work asuspec_enter_s3_work;

	struct wake_lock 	wake_lock;
	struct timer_list	asuspec_timer;

	int status;
	int ec_in_s3;         // 0: normal mode, 1: ec in deep sleep mode

	u8 i2c_data[32];
	u8 i2c_dm_data[32];
	u8 i2c_dm_battery[32];
};

#endif
