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

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/switch.h>
#include <linux/wakelock.h>

/*
 * compiler option
 */
#define FACTORY_MODE                   0
#define DOCK_USB                       1	// 0: not ready, 1: ready
#define BATTERY_DRIVER                 1	// 0: not ready, 1: ready
#define BATTERY_CALLBACK_ENABLED       1
#define DOCK_EC_ENABLED                1
#define GET_USB_CABLE_STATUS_ENABLED   1

//-----------------------------------------

struct i2c_client;
struct tegra_udc;
struct tegra_usb_phy;
struct usb_hcd;

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
int asus_ec_irq_request(void *dev, int gpio, irq_handler_t handler,
			unsigned long flags, const char *label);
void asus_ec_clear_buffer(struct i2c_client *client, char *buf);

/* USB lines */
void transformer_link_udc(struct tegra_udc *udc);
void transformer_cable_detect(struct tegra_udc *udc);
void transformer_usb_definer(struct usb_hcd *hcd, struct tegra_usb_phy *phy);

void fsl_dock_ec_callback(void);
void utmi_xcvr_setup_corrector(struct tegra_usb_phy *phy);
void tegra_usb3_smi_backlight_on_callback(void);
unsigned int tegra_get_usb_cable_status(void);
void tegra_detect_charging_type_is_cdp_or_dcp(struct tegra_udc *udc); /* export from tegra_udc */

void cable_status_setup(struct tegra_udc *udc);
void cable_status_reset(void);

int __init transformer_udc_init(void);
void __exit transformer_udc_exit(void);

/* Battery monitor */
int pad_battery_monitor(int offs);
int dock_battery_monitor(int offs);
int dock_ac_callback(void);

void battery_callback(unsigned usb_cable_state);
int docking_callback(int docking_in);
void register_usb_cable_status_cb(unsigned (*fn) (void));

extern int usb_suspend_tag;

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
#define ASUSDEC_KB_ENABLE               0xF400
#define ASUSDEC_KB_DISABLE              0xF500

/*            - IRQ labels -              */
#define ASUSPEC_REQUEST                 "pad_request"
#define ASUSPEC_APWAKE                  "pad_apwake"
#define ASUSDEC_REQUEST                 "dock_request"
#define ASUSDEC_INPUT                   "dock_input"
#define ASUSDEC_DOCKIN                  "dock_in"

#define KEY_FLAGS_FN_LOCK               1	/* Fn keys without modifier */
#define KEY_FLAGS_BACK_AS_ESC           2
#define KEY_FLAGS_HOME_AS_LEFTMETA      8	/* aka Super_L or "Windows key" */

#define ASUSDEC_KEYPAD_KEY_BREAK        0xF0
#define ASUSDEC_KEYPAD_KEY_EXTEND       0xE0

static const unsigned short asus_dock_ext_keys[] = {
	[0x01] = KEY_SLEEP,
	[0x02] = KEY_WLAN,
	[0x03] = KEY_BLUETOOTH,
	[0x04] = KEY_TOUCHPAD_TOGGLE,
	[0x05] = KEY_BRIGHTNESSDOWN,
	[0x06] = KEY_BRIGHTNESSUP,
	[0x07] = KEY_BRIGHTNESS_AUTO,
	[0x08] = KEY_CAMERA,
	[0x10] = KEY_WWW,
	[0x11] = KEY_CONFIG,
	[0x12] = KEY_PREVIOUSSONG,
	[0x13] = KEY_PLAYPAUSE,
	[0x14] = KEY_NEXTSONG,
	[0x15] = KEY_MUTE,
	[0x16] = KEY_VOLUMEDOWN,
	[0x17] = KEY_VOLUMEUP,
};

static const unsigned short fn_dock_ext_keys[] = {
	[0x01] = KEY_DELETE,
	[0x02] = KEY_F1,
	[0x03] = KEY_F2,
	[0x04] = KEY_F3,
	[0x05] = KEY_F4,
	[0x06] = KEY_F5,
	[0x07] = KEY_F6,
	[0x08] = KEY_F7,
	[0x10] = KEY_F8,
	[0x11] = KEY_F9,
	[0x12] = KEY_F10,
	[0x13] = KEY_F11,
	[0x14] = KEY_F12,
	[0x15] = KEY_INSERT,
	[0x16] = KEY_VOLUMEDOWN,
	[0x17] = KEY_VOLUMEUP,
};

static const unsigned short asus_dock_keys[] = {
	[0x76] = KEY_BACK,              /* ASUSDEC_KEYPAD_ESC */

	/* First key row */
	[0x0E] = KEY_GRAVE,             /* ASUSDEC_KEYPAD_KEY_WAVE */
	[0x16] = KEY_1,                 /* ASUSDEC_KEYPAD_KEY_1 */
	[0X1E] = KEY_2,                 /* ASUSDEC_KEYPAD_KEY_2 */
	[0x26] = KEY_3,                 /* ASUSDEC_KEYPAD_KEY_3 */
	[0x25] = KEY_4,                 /* ASUSDEC_KEYPAD_KEY_4 */
	[0x2E] = KEY_5,                 /* ASUSDEC_KEYPAD_KEY_5 */
	[0x36] = KEY_6,                 /* ASUSDEC_KEYPAD_KEY_6 */
	[0x3D] = KEY_7,                 /* ASUSDEC_KEYPAD_KEY_7 */
	[0x3E] = KEY_8,                 /* ASUSDEC_KEYPAD_KEY_8 */
	[0x46] = KEY_9,                 /* ASUSDEC_KEYPAD_KEY_9 */
	[0x45] = KEY_0,                 /* ASUSDEC_KEYPAD_KEY_0 */
	[0x4E] = KEY_MINUS,             /* ASUSDEC_KEYPAD_KEY_MINUS */
	[0x55] = KEY_EQUAL,             /* ASUSDEC_KEYPAD_KEY_EQUAL */
	[0x66] = KEY_BACKSPACE,         /* ASUSDEC_KEYPAD_KEY_BACKSPACE */

	/* Second key row */
	[0x0D] = KEY_TAB,               /* ASUSDEC_KEYPAD_KEY_TAB */
	[0x15] = KEY_Q,                 /* ASUSDEC_KEYPAD_KEY_Q */
	[0x1D] = KEY_W,                 /* ASUSDEC_KEYPAD_KEY_W */
	[0x24] = KEY_E,                 /* ASUSDEC_KEYPAD_KEY_E */
	[0x2D] = KEY_R,                 /* ASUSDEC_KEYPAD_KEY_R */
	[0x2C] = KEY_T,                 /* ASUSDEC_KEYPAD_KEY_T */
	[0x35] = KEY_Y,                 /* ASUSDEC_KEYPAD_KEY_Y */
	[0x3C] = KEY_U,                 /* ASUSDEC_KEYPAD_KEY_U */
	[0x43] = KEY_I,                 /* ASUSDEC_KEYPAD_KEY_I */
	[0x44] = KEY_O,                 /* ASUSDEC_KEYPAD_KEY_O */
	[0x4D] = KEY_P,                 /* ASUSDEC_KEYPAD_KEY_P */
	[0x54] = KEY_LEFTBRACE,         /* ASUSDEC_KEYPAD_KEY_LEFTBRACE */
	[0x5B] = KEY_RIGHTBRACE,        /* ASUSDEC_KEYPAD_KEY_RIGHTBRACE */
	[0x5D] = KEY_BACKSLASH,         /* ASUSDEC_KEYPAD_KEY_BACKSLASH */

	/* Third key row */
	[0x58] = KEY_CAPSLOCK,          /* ASUSDEC_KEYPAD_KEY_CAPSLOCK */
	[0x1C] = KEY_A,                 /* ASUSDEC_KEYPAD_KEY_A */
	[0x1B] = KEY_S,                 /* ASUSDEC_KEYPAD_KEY_S */
	[0x23] = KEY_D,                 /* ASUSDEC_KEYPAD_KEY_D */
	[0x2B] = KEY_F,                 /* ASUSDEC_KEYPAD_KEY_F */
	[0x34] = KEY_G,                 /* ASUSDEC_KEYPAD_KEY_G */
	[0x33] = KEY_H,                 /* ASUSDEC_KEYPAD_KEY_H */
	[0x3B] = KEY_J,                 /* ASUSDEC_KEYPAD_KEY_J */
	[0x42] = KEY_K,                 /* ASUSDEC_KEYPAD_KEY_K */
	[0x4B] = KEY_L,                 /* ASUSDEC_KEYPAD_KEY_L */
	[0x4C] = KEY_SEMICOLON,         /* ASUSDEC_KEYPAD_KEY_SEMICOLON */
	[0x52] = KEY_APOSTROPHE,        /* ASUSDEC_KEYPAD_KEY_APOSTROPHE */
	[0x5A] = KEY_ENTER,             /* ASUSDEC_KEYPAD_KEY_ENTER */

	/* Fourth key row */
	[0x12] = KEY_LEFTSHIFT,         /* ASUSDEC_KEYPAD_KEY_LEFTSHIFT */
	[0x1A] = KEY_Z,                 /* ASUSDEC_KEYPAD_KEY_Z */
	[0x22] = KEY_X,                 /* ASUSDEC_KEYPAD_KEY_X */
	[0x21] = KEY_C,                 /* ASUSDEC_KEYPAD_KEY_C */
	[0x2A] = KEY_V,                 /* ASUSDEC_KEYPAD_KEY_V */
	[0x32] = KEY_B,                 /* ASUSDEC_KEYPAD_KEY_B */
	[0x31] = KEY_N,                 /* ASUSDEC_KEYPAD_KEY_N */
	[0x3A] = KEY_M,                 /* ASUSDEC_KEYPAD_KEY_M */
	[0x41] = KEY_COMMA,             /* ASUSDEC_KEYPAD_KEY_COMMA */
	[0x49] = KEY_DOT,               /* ASUSDEC_KEYPAD_KEY_DOT */
	[0x4A] = KEY_SLASH,             /* ASUSDEC_KEYPAD_KEY_SLASH */
	[0x59] = KEY_RIGHTSHIFT,        /* ASUSDEC_KEYPAD_KEY_RIGHTSHIFT */
	[0xE075] = KEY_UP,              /* ASUSDEC_KEYPAD_KEY_UP */
	[0xE07D] = KEY_PAGEUP,          /* ASUSDEC_KEYPAD_PAGEUP */
	[0xE027] = KEY_SEARCH,          /* ASUSDEC_KEYPAD_RIGHTWIN */

	/* Fifth key row */
	[0x14] = KEY_LEFTCTRL,          /* ASUSDEC_KEYPAD_LEFTCTRL */
	[0xE01F] = KEY_HOMEPAGE,        /* ASUSDEC_KEYPAD_LEFTWIN */
	[0x11] = KEY_LEFTALT,           /* ASUSDEC_KEYPAD_LEFTALT */
	[0x29] = KEY_SPACE,             /* ASUSDEC_KEYPAD_KEY_SPACE */
	[0xE011] = KEY_RIGHTALT,        /* ASUSDEC_KEYPAD_RIGHTALT */
	[0xE02F] = KEY_MENU,            /* ASUSDEC_KEYPAD_WINAPP */
	[0xE014] = KEY_RIGHTCTRL,       /* ASUSDEC_KEYPAD_RIGHTCTRL */
	[0xE06B] = KEY_LEFT,            /* ASUSDEC_KEYPAD_KEY_LEFT */
	[0xE072] = KEY_DOWN,            /* ASUSDEC_KEYPAD_KEY_DOWN */
	[0xE074] = KEY_RIGHT,           /* ASUSDEC_KEYPAD_KEY_RIGHT */
	[0xE06C] = KEY_HOME,            /* ASUSDEC_KEYPAD_HOME */
	[0xE07A] = KEY_PAGEDOWN,        /* ASUSDEC_KEYPAD_PAGEDOWN */
	[0xE069] = KEY_END,             /* ASUSDEC_KEYPAD_END */

	/* JP keys */
	[0x5F] = KEY_ZENKAKUHANKAKU,    /* ASUSDEC_HANKAKU_ZENKAKU */
	[0x6A] = KEY_YEN,               /* ASUSDEC_YEN */
	[0x67] = KEY_MUHENKAN,          /* ASUSDEC_MUHENKAN */
	[0x64] = KEY_HENKAN,            /* ASUSDEC_HENKAN */
	[0x13] = KEY_KATAKANAHIRAGANA,  /* ASUSDEC_HIRAGANA_KATAKANA */
	[0x51] = KEY_RO,                /* ASUSDEC_RO */

	/* UK keys */
	[0x61] = KEY_102ND,             /* ASUSDEC_EUROPE_2 */
};

static const unsigned short linux_dock_keys[] = {
	[0x76] = KEY_ESC,               /* ASUSDEC_KEYPAD_ESC */
	[0xE01F] = KEY_LEFTMETA,        /* ASUSDEC_KEYPAD_LEFTWIN */
};

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
};

#endif
