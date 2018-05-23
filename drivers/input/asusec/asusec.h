#ifndef _ASUSEC_H
#define _ASUSEC_H

#include <linux/switch.h>
#include <linux/wakelock.h>

/*
 * compiler option
 */
#define FACTORY_MODE			0
#define TOUCHPAD_MODE			1	// 0: relative mode, 1: absolute mode
#define DOCK_USB				1	// 0: not ready, 1: ready
#define BATTERY_DRIVER			1	// 0: not ready, 1: ready

/*
 * Debug Utility
 */

#define ASUSEC_INFO(format, arg...)	\
	pr_info("asusec: [%s] " format , __FUNCTION__ , ## arg)

#define ASUSEC_NOTICE(format, arg...)	\
	pr_notice("asusdec: [%s] " format , __FUNCTION__ , ## arg)

#define ASUSEC_ERR(format, arg...)	\
	pr_err("asusdec: [%s] " format , __FUNCTION__ , ## arg)

//-----------------------------------------

#define ASUSDEC_DRIVER_DESC		"ASUS Dock EC Driver"
#define ASUSPEC_DRIVER_DESC		"ASUS PAD EC driver"
#define DOCK_SDEV_NAME			"dock"
#define PAD_SDEV_NAME			"pad"
#define APOWER_SDEV_NAME		"apower"

#define DELAY_TIME_MS		50

#define ASUSDEC_I2C_ERR_TOLERANCE	8
#define ASUSPEC_I2C_ERR_TOLERANCE	32
#define ASUSEC_RETRY_COUNT		3
#define ASUSDEC_POLLING_RATE		80

#define ASUSEC_OBF_MASK			0x1
#define ASUSEC_KEY_MASK			0x4
#define ASUSEC_KBC_MASK			0x8
#define ASUSEC_AUX_MASK			0x20
#define ASUSEC_SCI_MASK			0x40
#define ASUSEC_SMI_MASK			0x80

#define ASUSDEC_RELATIVE_MODE		0
#define ASUSDEC_ABSOLUTE_MODE		1

#define NUM_RELATIVE_MODE		3	// The number of bytes per packet in relative mode
#define NUM_ABSOLUTE_MODE		6	// The number of bytes per packet in absolute mode

/* relative mode packet formate */
#define Y_OVERFLOW_MASK			0x80
#define X_OVERFLOW_MASK			0x40
#define Y_SIGN_MASK			0x20
#define X_SIGN_MASK			0x10
#define RIGHT_BTN_MASK			0x2
#define LEFT_BTN_MASK			0x1

/* absolute mode packet formate */
#define TOUCHPAD_SAMPLE_RATE		20
#define ABSOLUTE_BIT_MASK		0x80
#define Z_SNESITIVITY			30
#define X_LIMIT				5600
#define Y_LIMIT				1300
#define X_MAX				5855
#define X_MIN				1198
#define Y_MAX				4942
#define Y_MIN				946

#define ASUSDEC_PS2_ACK			0xFA

//-----------------------------------------
#define ASUSDEC_KEY_TOUCHPAD	KEY_F2
#define ASUSDEC_KEY_AUTOBRIGHT	KEY_F3
#define ASUSDEC_KEY_SETTING		KEY_F4

/*************scan 2 make mapping***************/
#define ASUSDEC_KEYPAD_ESC		0x76
#define ASUSDEC_KEYPAD_KEY_WAVE		0x0E
#define ASUSDEC_KEYPAD_KEY_1		0x16
#define ASUSDEC_KEYPAD_KEY_2		0X1E
#define ASUSDEC_KEYPAD_KEY_3		0x26
#define ASUSDEC_KEYPAD_KEY_4		0x25
#define ASUSDEC_KEYPAD_KEY_5		0x2E
#define ASUSDEC_KEYPAD_KEY_6        	0x36
#define ASUSDEC_KEYPAD_KEY_7        	0x3D
#define ASUSDEC_KEYPAD_KEY_8        	0x3E
#define ASUSDEC_KEYPAD_KEY_9        	0x46
#define ASUSDEC_KEYPAD_KEY_0        	0x45
#define ASUSDEC_KEYPAD_KEY_MINUS    	0x4E
#define ASUSDEC_KEYPAD_KEY_EQUAL		0x55
#define ASUSDEC_KEYPAD_KEY_BACKSPACE	0x66
#define ASUSDEC_KEYPAD_KEY_TAB      	0x0D
#define ASUSDEC_KEYPAD_KEY_Q        	0x15
#define ASUSDEC_KEYPAD_KEY_W        	0x1D
#define ASUSDEC_KEYPAD_KEY_E        	0x24
#define ASUSDEC_KEYPAD_KEY_R        	0x2D
#define ASUSDEC_KEYPAD_KEY_T        	0x2C
#define ASUSDEC_KEYPAD_KEY_Y        	0x35
#define ASUSDEC_KEYPAD_KEY_U        	0x3C
#define ASUSDEC_KEYPAD_KEY_I        	0x43
#define ASUSDEC_KEYPAD_KEY_O        	0x44
#define ASUSDEC_KEYPAD_KEY_P        	0x4D
#define ASUSDEC_KEYPAD_KEY_LEFTBRACE	0x54
#define ASUSDEC_KEYPAD_KEY_RIGHTBRACE 	0x5B
#define ASUSDEC_KEYPAD_KEY_BACKSLASH	0x5D
#define ASUSDEC_KEYPAD_KEY_CAPSLOCK 	0x58
#define ASUSDEC_KEYPAD_KEY_A        	0x1C
#define ASUSDEC_KEYPAD_KEY_S        	0x1B
#define ASUSDEC_KEYPAD_KEY_D        	0x23
#define ASUSDEC_KEYPAD_KEY_F        	0x2B
#define ASUSDEC_KEYPAD_KEY_G        	0x34
#define ASUSDEC_KEYPAD_KEY_H        	0x33
#define ASUSDEC_KEYPAD_KEY_J        	0x3B
#define ASUSDEC_KEYPAD_KEY_K        	0x42
#define ASUSDEC_KEYPAD_KEY_L        	0x4B
#define ASUSDEC_KEYPAD_KEY_SEMICOLON	0x4C
#define ASUSDEC_KEYPAD_KEY_APOSTROPHE	0x52
#define ASUSDEC_KEYPAD_KEY_ENTER    	0x5A
#define ASUSDEC_KEYPAD_KEY_LEFTSHIFT 	0x12
#define ASUSDEC_KEYPAD_KEY_Z        	0x1A
#define ASUSDEC_KEYPAD_KEY_X        	0x22
#define ASUSDEC_KEYPAD_KEY_C        	0x21
#define ASUSDEC_KEYPAD_KEY_V        	0x2A
#define ASUSDEC_KEYPAD_KEY_B        	0x32
#define ASUSDEC_KEYPAD_KEY_N        	0x31
#define ASUSDEC_KEYPAD_KEY_M        	0x3A
#define ASUSDEC_KEYPAD_KEY_COMMA    	0x41
#define ASUSDEC_KEYPAD_KEY_DOT   	0x49
#define ASUSDEC_KEYPAD_KEY_SLASH    	0x4A
#define ASUSDEC_KEYPAD_KEY_RIGHTSHIFT   	0x59

#define ASUSDEC_KEYPAD_KEY_LEFT   	0xE06B
#define ASUSDEC_KEYPAD_KEY_RIGHT   	0xE074
#define ASUSDEC_KEYPAD_KEY_UP		0xE075
#define ASUSDEC_KEYPAD_KEY_DOWN		0xE072

#define ASUSDEC_KEYPAD_RIGHTWIN		0xE027
#define ASUSDEC_KEYPAD_LEFTCTRL		0x14
#define ASUSDEC_KEYPAD_LEFTWIN		0xE01F
#define ASUSDEC_KEYPAD_LEFTALT		0x11
#define ASUSDEC_KEYPAD_KEY_SPACE		0x29
#define ASUSDEC_KEYPAD_RIGHTALT		0xE011
#define ASUSDEC_KEYPAD_WINAPP		0xE02F
#define ASUSDEC_KEYPAD_RIGHTCTRL		0xE014
#define ASUSDEC_KEYPAD_HOME			0xE06C
#define ASUSDEC_KEYPAD_PAGEUP		0xE07D
#define ASUSDEC_KEYPAD_PAGEDOWN		0xE07A
#define ASUSDEC_KEYPAD_END			0xE069
/************  JP keys *************/
#define ASUSDEC_HANKAKU_ZENKAKU		0x5F
#define ASUSDEC_YEN					0x6A
#define ASUSDEC_MUHENKAN				0x67
#define ASUSDEC_HENKAN				0x64
#define ASUSDEC_HIRAGANA_KATAKANA	0x13
#define ASUSDEC_RO					0x51
/********************************/
/************  UK keys *************/
#define ASUSDEC_EUROPE_2				0x61
/********************************/

#define ASUSDEC_KEYPAD_LOCK			0xE071
#define ASUSDEC_KEYPAD_KEY_BREAK   	0xF0
#define ASUSDEC_KEYPAD_KEY_EXTEND   	0xE0


/************* SMI event ********************/
#define ASUSEC_SMI_HANDSHAKING		0x50
#define ASUSEC_SMI_WAKE			0x53
#define ASUSEC_SMI_RESET			0x5F
#define ASUSEC_SMI_ADAPTER_EVENT	0x60
#define ASUSEC_SMI_BACKLIGHT_ON	0x63
#define ASUSEC_SMI_AUDIO_DOCK_IN	0x70

/*************APOWER ************************/
#define APOWER_IDLE			0
#define APOWER_RESUME			1
#define APOWER_SUSPEND			2
#define APOWER_POWEROFF			3
#define APOWER_NOTIFY_SHUTDOWN		4
#define APOWER_SMI_S3				0x83
#define APOWER_SMI_S5				0x85
#define APOWER_SMI_NOTIFY_SHUTDOWN		0x90
#define APOWER_SMI_RESUME			0x91

/*************IO control setting***************/
#define ASUSEC_IOCTL_HEAVY	2
#define ASUSEC_IOCTL_NORMAL	1
#define ASUSEC_IOCTL_END	0
#define ASUSEC_IOC_MAGIC	0xf4
#define ASUSDEC_IOC_MAXNR	7
#define ASUSPEC_IOC_MAXNR	11
#define ASUSEC_POLLING_DATA 	_IOR(ASUSEC_IOC_MAGIC,	1,	int)
#define ASUSEC_FW_UPDATE 		_IOR(ASUSEC_IOC_MAGIC,	2,	int)
#define ASUSEC_INIT 			_IOR(ASUSEC_IOC_MAGIC,	3,	int)
#define ASUSDEC_INIT			_IOR(ASUSEC_IOC_MAGIC,	4,	int)
#define ASUSDEC_TP_CONTROL		_IOR(ASUSEC_IOC_MAGIC,	5,	int)
#define ASUSDEC_EC_WAKEUP		_IOR(ASUSEC_IOC_MAGIC,	6,	int)
#define ASUSEC_FW_DUMMY			_IOR(ASUSEC_IOC_MAGIC,	7,	int)
#define ASUSEC_SWITCH_HDMI		_IOR(ASUSEC_IOC_MAGIC,	10,	int)
#define ASUSEC_WIN_SHUTDOWN		_IOR(ASUSEC_IOC_MAGIC,	11,	int)

#define ASUSDEC_CPAS_LED_ON	1
#define ASUSDEC_CPAS_LED_OFF	0
#define ASUSDEC_TP_ON	1
#define ASUSDEC_TP_OFF	0
#define ASUSDEC_EC_ON	1
#define ASUSDEC_EC_OFF	0

/************* Dock Defifition ***********/
#define DOCK_UNKNOWN		0
#define MOBILE_DOCK		1
//#define AUDIO_DOCK		2
//#define AUDIO_STAND		3

/************* Dock State ***********/
#define DOCK_OUT		0
#define DOCK_IN		1

/************* Cable Type ***********/
#define BAT_CABLE_OUT		0
#define BAT_CABLE_USB		1
#define BAT_CABLE_AC		3
#define BAT_CABLE_UNKNOWN		-1
#define CABLE_0V		0x00
#define CABLE_5V		0x05
#define CABLE_12V		0x12
#define CABLE_15V		0x15

/************* EC FW update ***********/
#define EC_BUFF_LEN  256
/********************** ***********/

/*****************************************/
#define ASUSPEC_MAGIC_NUM	0x19850604

/********************** ***********/
#define MB 1024*1024

/*
 * The x/y limits are taken from the Synaptics TouchPad interfacing Guide,
 * section 2.3.2, which says that they should be valid regardless of the
 * actual size of the sensor.
 */
#define XMIN_NOMINAL 0
#define XMAX_NOMINAL 1279
#define YMIN_NOMINAL 0
#define YMAX_NOMINAL 799

/*
 * data struct
 */

struct asusdec_keypad{
	int value;
	int input_keycode;
	int extend;
};

struct asusdec_touchpad_relative{
	int y_overflow;
	int x_overflow;
	int y_sign;
	int x_sign;
	int left_btn;
	int right_btn;
	int delta_x;
	int delta_y;
};

struct asusdec_touchpad_absolute{
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
	struct input_dev	*lid_indev;
	struct switch_dev 	dock_sdev;
	struct i2c_client	*client;
	struct mutex		lock;
	struct mutex		kbc_lock;
	struct mutex		input_lock;
	struct mutex		dock_init_lock;
	struct wake_lock 	wake_lock;
	struct wake_lock 	wake_lock_init;
	struct delayed_work asusdec_work;
	struct delayed_work asusdec_dock_init_work;
	struct delayed_work asusdec_fw_update_work;
	struct delayed_work asusdec_led_on_work;
	struct delayed_work asusdec_led_off_work;
	struct delayed_work asusdec_hall_sensor_work;
	struct delayed_work asusdec_audio_work;
	struct delayed_work audio_in_out_work;
	struct delayed_work asusdec_pad_battery_report_work;
	struct asusdec_keypad keypad_data;
	struct elantech_data *private;
	struct timer_list asusdec_timer;
#if TOUCHPAD_MODE
	struct asusdec_touchpad_absolute t_abs;
#else
	struct asusdec_touchpad_relative touchpad_data;
#endif
	int ret_val;
	u8 ec_data[32];
	u8 i2c_data[32];
	u8 i2c_dm_data[32];
	u8 mcu_fw_version[5];
	u8 mcu_type;
	int bc;			// byte counter
	int index;		// for message
	int status;
	int touchpad_member;
	char ec_model_name[32];
	char ec_version[32];
	char dock_pid[32];
	int polling_rate;
	int dock_in;	// 0: without dock, 1: with dock
	int op_mode;	// 0: normal mode, 1: fw update mode
	int kbc_value;	// capslock_led 0: led off, 1: led on
	int dock_det;	// dock-in interrupt count
	int dock_init;	// 0: dock not init, 1: dock init successfully
	int d_index;	// touchpad byte counter
	int suspend_state; // 0: normal, 1: suspend
	int init_success; // 0: ps/2 not ready. 1: init OK, -1: tp not ready
	int wakeup_lcd;		// 0 : keep lcd state 1: make lcd on
	int tp_wait_ack;	// 0 : normal mode, 1: waiting for an ACK
	int tp_enable;		// 0 : touchpad has not enabled, 1: touchpad has enabled
	int re_init;		// 0 : first time init, not re-init, 1: in re-init procedure
	int ec_wakeup;		// 0 : ec shutdown when PAD in LP0, 1 : keep ec active when PAD in LP0,
	int ap_wake_wakeup;	// 0 : no ap_wake wakeup signal, 1: get ap_wake wakeup signal
	int tf_dock;		// 0 : not tf dock, 1: tf dock
	int dock_behavior;	// 0: susb_on follows wakeup event, 1: susb_on follows ec_req
	int ec_in_s3;		// 0: normal mode, 1: ec in deep sleep mode
	int susb_on;	// 0: susb off, 1: susb on
	int dock_type; //0: unknown, 1: mobile_dock, 2: audio_dock, 3: audio_stand
};

struct asuspec_chip {
	struct input_dev	*indev;
	struct switch_dev 	pad_sdev;
	struct switch_dev 	apower_sdev;
	struct i2c_client	*client;
	struct mutex		lock;
	struct mutex		irq_lock;
	struct mutex		state_change_lock;
	struct delayed_work asuspec_fw_update_work;
	struct delayed_work asuspec_init_work;
	struct delayed_work asuspec_work;
	struct delayed_work asuspec_enter_s3_work;
	struct wake_lock 	wake_lock;
	struct timer_list	asuspec_timer;
	int polling_rate;
	int status;
	int ret_val;
	u8 ec_data[32];
	u8 i2c_data[32];
	u8 i2c_dm_data[32];
	u8 i2c_dm_battery[32];
	u8 i2c_dm_storage[32];
	char ec_model_name[32];
	char ec_version[32];
	char ec_pcba[32];
	int op_mode;	// 0: normal mode, 1: fw update mode
	int ec_ram_init;	// 0: not init, MAGIC_NUM: init successfully
	int ec_in_s3;	// 0: normal mode, 1: ec in deep sleep mode
	int i2c_err_count;
	int apwake_disabled;	// 0: normal mode, 1: apwake gets disabled
	unsigned long storage_total;
	unsigned long storage_avail;
	unsigned int pad_pid;
	int apower_state;
};

#endif
