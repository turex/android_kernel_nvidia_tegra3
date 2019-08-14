/*
 * ASUS Dock EC driver.
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

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/asusec.h>

#include <asm/gpio.h>
#include <asm/uaccess.h>

#include <mach/board-transformer-misc.h>

#include "elantech.h"

/*
 * global variable
 */
int switch_value[] = {0, 10}; /* 0: no dock, 1: mobile dock */

static struct i2c_client dock_client;
static struct class *asusdec_class;
static struct device *asusdec_device;
static struct asusdec_chip *ec_chip;

static dev_t asusdec_dev;
static int asusdec_major = 0;
static int asusdec_minor = 0;

static struct workqueue_struct *asusdec_wq;

int key_flags = 0;
module_param(key_flags, int, 0644);

int key_autorepeat = 0;
static int key_autorepeat_set(const char *arg, const struct kernel_param *kp)
{
	int ret = param_set_int(arg, kp);

	if (!ret) {
		if (ec_chip && ec_chip->indev) {
			if (key_autorepeat)
				set_bit(EV_REP, ec_chip->indev->evbit);
			else
				clear_bit(EV_REP, ec_chip->indev->evbit);
		}
	}

	return ret;
}

static struct kernel_param_ops key_autorepeat_ops = {
	.set = key_autorepeat_set,
	.get = param_get_int,
};
module_param_cb(key_autorepeat, &key_autorepeat_ops, &key_autorepeat, 0644);

/*
 * functions definition
 */
int dock_battery_monitor(int offs)
{
	int ret = 0;
	u8 batt_data[32];

	if (!offs) {
		if (!ec_chip->dock_in)
			return 0;
		else
			return 1;
	}

	if (!ec_chip->dock_in)
		return -1;

	if (ec_chip->ec_in_s3 && ec_chip->status)
		msleep(200);

	if (offs == 1) {
		ret = asus_dockram_read(&dock_client, 0x0A, batt_data);
		if (ret < 0){
			pr_err("asusdec: fail to access battery info\n");
			return -1;
		}

		return batt_data[1];
	}

	ret = asus_dockram_read(&dock_client, 0x14, batt_data);
	if (ret < 0){
		pr_err("asusdec: fail to access battery info\n");
		return -1;
	}

	return batt_data[offs + 1] << 8 | batt_data[offs];
}

static int asus_input_switch(struct i2c_client *client, u16 state)
{
	int retry, definer;

	asus_ec_clear_buffer(client, ec_chip->i2c_data);
	asus_ec_write(client, state);

	for (retry = 0; retry < ASUSEC_RETRY_COUNT; retry++) {
		asus_ec_read(client, ec_chip->i2c_data);
		
		if ((state == ASUSDEC_KB_ENABLE) || (state == ASUSDEC_KB_DISABLE))
			definer = !(ec_chip->i2c_data[1] & ASUSEC_AUX_MASK);
		else
			definer = (ec_chip->i2c_data[1] & ASUSEC_AUX_MASK);

		if ((ec_chip->i2c_data[1] & ASUSEC_OBF_MASK) && definer) {
			if (ec_chip->i2c_data[2] == ASUSDEC_PS2_ACK)
				goto exit;
		}
		msleep(DELAY_TIME_MS);
	}

	dev_err(&client->dev, "failed to write 0x%x into EC\n", state);
	return -1;

exit:
	return 0;
}

static void asusdec_keypad_led_on(struct work_struct *dat)
{
	ec_chip->kbc_value = 1;
	pr_info("asusdec: set led on\n");
	msleep(250);
	asus_ec_write(ec_chip->client, 0xED00);
}

static void asusdec_keypad_led_off(struct work_struct *dat)
{
	ec_chip->kbc_value = 0;
	pr_info("asusdec: set led off\n");
	msleep(250);
	asus_ec_write(ec_chip->client, 0xED00);
}

static void asusdec_tp_control(void)
{
	if (ec_chip->tp_enable) {
		pr_info("asusdec: switching touchpad off\n");
		ec_chip->tp_enable = 0;
		ec_chip->tp_wait_ack = 1;
		asus_input_switch(ec_chip->client, ASUSDEC_TP_DISABLE);
		ec_chip->d_index = 0;
	} else if (!ec_chip->tp_enable) {
		pr_info("asusdec: switching touchpad on\n");
		ec_chip->tp_enable = 1;
		ec_chip->tp_wait_ack = 1;
		asus_input_switch(ec_chip->client, ASUSDEC_TP_ENABLE);
		ec_chip->d_index = 0;

		if (ec_chip->tp_model) {
			ec_chip->susb_on = 1;
			ec_chip->init_success = -1;
			asus_ec_signal_request(ec_chip->client, DOCK_ECREQ_GPIO);
			ec_chip->dock_init = 0;
		}
	}
}

static void asusdec_dock_status_report(void)
{
	pr_info("asusdec: dock_in = %d\n", ec_chip->dock_in);
	switch_set_state(&ec_chip->dock_sdev, switch_value[ec_chip->dock_in]);
#if BATTERY_DRIVER
	queue_delayed_work(asusdec_wq, &ec_chip->asusdec_pad_battery_report_work, 0);
#endif
}

static int asusdec_event(struct input_dev *dev, unsigned int type,
				unsigned int code, int led)
{
	if (!ec_chip->dock_in)
		return -ENOTTY;

	if ((type != EV_LED) || (code != LED_CAPSL))
		return -ENOTTY;

	if (!led)
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_led_off_work, 0);
	else
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_led_on_work, 0);

	return 0;
}

static int asusdec_input_device_create(struct i2c_client *client)
{
	int err = 0;
	int i = 0;

	if (ec_chip->indev)
		return 0;

	ec_chip->indev = input_allocate_device();
	if (!ec_chip->indev) {
		dev_err(&client->dev, "input device allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->indev->name = "asusdec";
	ec_chip->indev->phys = "/dev/input/asusdec";
	ec_chip->indev->dev.parent = &client->dev;
	ec_chip->indev->event = asusdec_event;

	/* Set keypad input parameters */
	set_bit(EV_KEY, ec_chip->indev->evbit);

	for (i = 0; i < 246; i++)
		set_bit(i, ec_chip->indev->keybit);

	input_set_capability(ec_chip->indev, EV_LED, LED_CAPSL);

	/* Register input device */
	err = input_register_device(ec_chip->indev);
	if (err) {
		dev_err(&client->dev, "input device registration fails\n");
		goto exit_input_free;
	}

	return 0;

exit_input_free:
	input_free_device(ec_chip->indev);
	ec_chip->indev = NULL;
exit:
	return err;
}

static int asusdec_chip_init(struct i2c_client *client)
{
	int err = 0;

	mutex_lock(&ec_chip->dock_init_lock);
	err = ec_chip->dock_init;
	ec_chip->dock_init = 1;
	mutex_unlock(&ec_chip->dock_init_lock);

	if (err)
		return 0;

	wake_lock(&ec_chip->wake_lock);
	disable_irq_nosync(gpio_to_irq(DOCK_APWAKE_GPIO));

	err = asus_ec_detect(&dock_client, client, ec_chip->i2c_data);
	if (err < 0)
		goto fail_to_access_ec;

	if (asusdec_input_device_create(client))
		goto fail_to_access_ec;

	if (!ec_chip->init_success)
		msleep(750);

	asus_input_switch(client, ASUSDEC_TP_DISABLE);
	asus_input_switch(client, ASUSDEC_KB_DISABLE);

	ec_chip->tp_model = elantech_init(ec_chip);
	ec_chip->d_index = 0;

	asus_input_switch(client, ASUSDEC_KB_ENABLE);

	if(ec_chip->tp_enable){
		if(ec_chip->tp_model){
			ec_chip->susb_on = 1;
			ec_chip->init_success = -1;
			asus_ec_signal_request(client, DOCK_ECREQ_GPIO);
			ec_chip->dock_init = 0;
		}
		ec_chip->tp_wait_ack = 1;
		asus_input_switch(client, ASUSDEC_TP_ENABLE);
	}

	enable_irq(gpio_to_irq(DOCK_APWAKE_GPIO));

	pr_info("asusdec: touchpad and keyboard init\n");

	ec_chip->init_success = 1;
	ec_chip->status = 1;
	asusdec_dock_status_report();
	wake_unlock(&ec_chip->wake_lock);

	return 0;

fail_to_access_ec:
	if (asus_dockram_read(&dock_client, 0x00, ec_chip->i2c_dm_data) < 0){
		pr_info("asusdec: No EC detected\n");
		ec_chip->dock_in = 0;
	} else
		pr_info("asusdec: need DOCK FW update\n");

	enable_irq(gpio_to_irq(DOCK_APWAKE_GPIO));
	wake_unlock(&ec_chip->wake_lock);

	return -1;
}

static irqreturn_t asusdec_interrupt_handler(int irq, void *dev_id)
{
	if (irq == gpio_to_irq(DOCK_APWAKE_GPIO)){
		disable_irq_nosync(irq);
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_work, 0);
	}

	if (irq == gpio_to_irq(DOCK_IN_GPIO)){
		ec_chip->dock_in = 0;
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
	}

	return IRQ_HANDLED;
}

static void asusdec_reset_counter(unsigned long data){
	ec_chip->d_index = 0;
}

static void asusdec_dock_init_work_function(struct work_struct *dat)
{
	wake_lock(&ec_chip->wake_lock_init);
	mutex_lock(&ec_chip->input_lock);

	if (gpio_get_value(DOCK_IN_GPIO)){
		pr_info("asusdec: No dock detected\n");

		ec_chip->dock_in = 0;
		ec_chip->init_success = 0;
		ec_chip->tp_enable = 1;
		ec_chip->tp_model = -1;

		if (ec_chip->indev){
			input_unregister_device(ec_chip->indev);
			ec_chip->indev = NULL;
		}

		if (ec_chip->private->input_dev){
			input_unregister_device(ec_chip->private->input_dev);
			ec_chip->private->input_dev = NULL;
		}

		asusdec_dock_status_report();
	} else {
		pr_info("asusdec: Dock-in detected\n");

		if (gpio_get_value(HALL_SENSOR_GPIO) || (!ec_chip->status)){
			if (!ec_chip->init_success){
				ec_chip->susb_on = 1;
				msleep(200);
				asus_ec_signal_request(ec_chip->client, DOCK_ECREQ_GPIO);
				ec_chip->dock_init = 0;
			}
		} else {
			pr_info("asusdec: Keyboard is closed\n");
		}
	}
	mutex_unlock(&ec_chip->input_lock);
	wake_unlock(&ec_chip->wake_lock_init);
}

static void asusdec_kp_wake(void)
{
	if (asusdec_input_device_create(ec_chip->client))
		return;

	input_report_key(ec_chip->indev, KEY_MENU, 1);
	input_sync(ec_chip->indev);
	input_report_key(ec_chip->indev, KEY_MENU, 0);
	input_sync(ec_chip->indev);
}

static void asusdec_kp_smi(void)
{
	switch (ec_chip->i2c_data[2]) {
		case ASUSEC_SMI_HANDSHAKING:
			dev_info(&ec_chip->client->dev, "ASUSDEC_SMI_HANDSHAKING\n");
			ec_chip->ec_in_s3 = 0;
			if (ec_chip->susb_on)
				asusdec_chip_init(ec_chip->client);
			break;
		case ASUSEC_SMI_RESET:
			dev_info(&ec_chip->client->dev, "ASUSDEC_SMI_RESET\n");
			ec_chip->init_success = 0;
			asusdec_dock_init_work_function(NULL);
			break;
		case ASUSEC_SMI_WAKE:
			dev_info(&ec_chip->client->dev, "ASUSDEC_SMI_WAKE\n");
			asusdec_kp_wake();
			break;
		case ASUSEC_SMI_ADAPTER_EVENT:
			dev_info(&ec_chip->client->dev, "ASUSDEC_SMI_ADAPTER_EVENT\n");
#if DOCK_USB
			fsl_dock_ec_callback();
#endif
			break;
		case ASUSEC_SMI_BACKLIGHT_ON:
			dev_info(&ec_chip->client->dev, "ASUSDEC_SMI_BACKLIGHT_ON\n");
			ec_chip->susb_on = 1;
			asus_ec_signal_request(ec_chip->client, DOCK_ECREQ_GPIO);
			ec_chip->dock_init = 0;
#if DOCK_USB
			tegra_usb3_smi_backlight_on_callback();
#endif
			/* Fall through since default just brakes switch */
		default:
			break;
	}
}

static void asusdec_kp_kbc(void)
{
	if (ec_chip->i2c_data[2] == ASUSDEC_PS2_ACK){
		if (ec_chip->kbc_value == 0){
			pr_info("asusdec: send led cmd 0x0000\n");
			asus_ec_write(ec_chip->client, 0x0000);
		} else {
			pr_info("asusdec: send led cmd 0x0400\n");
			asus_ec_write(ec_chip->client, 0x0400);
		}
	}
}

static int fn_keys_active(struct input_dev *dev)
{
	return test_bit(KEY_RIGHTALT, dev->key) ^ !!(key_flags & KEY_FLAGS_FN_LOCK);
}

static void asusdec_kp_sci(void)
{
	int ec_signal = ec_chip->i2c_data[2];

	if (fn_keys_active(ec_chip->indev))
		ec_chip->keypad_data.input_keycode = fn_dock_ext_keys[ec_signal];
	else
		ec_chip->keypad_data.input_keycode = asus_dock_ext_keys[ec_signal];

	if (ec_signal == 0x04)
		asusdec_tp_control();

	if (ec_chip->keypad_data.input_keycode > 0) {
//		pr_info("asusdec: ext_input_keycode = 0x%x, ec_signal = 0x%x\n",
//					ec_chip->keypad_data.input_keycode, ec_signal);

		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 1);
		input_sync(ec_chip->indev);
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
		input_sync(ec_chip->indev);
	} else {
		pr_info("asusdec: unknown ec_signal = 0x%x\n", ec_signal);
	}
}

static void asusdec_kp_key(void)
{
	int scancode = 0;

	if (ec_chip->i2c_data[2] == ASUSDEC_KEYPAD_KEY_EXTEND){
		ec_chip->keypad_data.extend = 1;
		ec_chip->bc = 3;
	} else {
		ec_chip->keypad_data.extend = 0;
		ec_chip->bc = 2;
	}

	if (ec_chip->i2c_data[ec_chip->bc] == ASUSDEC_KEYPAD_KEY_BREAK){
		ec_chip->keypad_data.value = 0;
		ec_chip->bc++;
	} else {
		ec_chip->keypad_data.value = 1;
	}

	if (ec_chip->keypad_data.extend == 1){
		scancode = ((ASUSDEC_KEYPAD_KEY_EXTEND << 8) | ec_chip->i2c_data[ec_chip->bc]);
	} else {
		scancode = ec_chip->i2c_data[ec_chip->bc];
	}

	if (ec_chip->i2c_data[0] == 6){
		if ((ec_chip->i2c_data[2] == 0xE0) &&
			(ec_chip->i2c_data[3] == 0xF0) &&
			(ec_chip->i2c_data[4] == 0x12)){
			scancode = ec_chip->i2c_data[5] << 8 | ec_chip->i2c_data[6];
			ec_chip->keypad_data.value = 1;
		} else if ((ec_chip->i2c_data[2] == 0xE0) &&
			(ec_chip->i2c_data[3] == 0xF0) &&
			(ec_chip->i2c_data[4] == 0x59)){
			scancode = ec_chip->i2c_data[5] << 8 | ec_chip->i2c_data[6];
			ec_chip->keypad_data.value = 1;
		}
	}

	if (scancode == 0x76 || scancode == 0xE01F) {
		if ((key_flags & KEY_FLAGS_BACK_AS_ESC) ||
		    (key_flags & KEY_FLAGS_HOME_AS_LEFTMETA))
				ec_chip->keypad_data.input_keycode = linux_dock_keys[scancode];
	} else
		ec_chip->keypad_data.input_keycode = asus_dock_keys[scancode];

	if (ec_chip->keypad_data.input_keycode > 0) {
		input_report_key(ec_chip->indev,
			ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		input_sync(ec_chip->indev);
	} else
		pr_info("asusdec: unknown scancode = 0x%x\n", scancode);
}

static void asusdec_keypad_processing(void)
{
	if (ec_chip->i2c_data[1] & ASUSEC_KBC_MASK)
		asusdec_kp_kbc();
	else if (ec_chip->i2c_data[1] & ASUSEC_SCI_MASK)
		asusdec_kp_sci();
	else
		asusdec_kp_key();
}

#if BATTERY_DRIVER
static void asusdec_pad_battery_report_function(struct work_struct *dat)
{
	int val = 0;
	int dock_in = ec_chip->dock_in;

	val = docking_callback(dock_in);
	if (val < 0)
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_pad_battery_report_work, 2 * HZ);
}
#endif

static void asusdec_work_function(struct work_struct *dat)
{
	int irq = gpio_to_irq(DOCK_APWAKE_GPIO);
	int err = 0;

	ec_chip->dock_in = gpio_get_value(DOCK_IN_GPIO) ? 0 : 1;

	err = asus_ec_read(ec_chip->client, ec_chip->i2c_data);
	enable_irq(irq);

	if (err < 0)
		return;

	if (ec_chip->i2c_data[1] & ASUSEC_OBF_MASK){
		if (ec_chip->i2c_data[1] & ASUSEC_SMI_MASK){
			asusdec_kp_smi();
			return;
		}
	}

	mutex_lock(&ec_chip->input_lock);
	if (!ec_chip->indev){
		mutex_unlock(&ec_chip->input_lock);
		return;
	}

	if (ec_chip->i2c_data[1] & ASUSEC_OBF_MASK){
		if (ec_chip->i2c_data[1] & ASUSEC_AUX_MASK){
			if (ec_chip->private->input_dev)
				asusdec_touchpad_processing(ec_chip);
		} else {
			asusdec_keypad_processing();
		}
	}
	mutex_unlock(&ec_chip->input_lock);
}

static ssize_t asusdec_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", switch_value[ec_chip->dock_in]);
}

static int __devinit asusdec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	ec_chip = kzalloc(sizeof (struct asusdec_chip), GFP_KERNEL);
	if (!ec_chip) {
		pr_err("asusdec: memory allocation (asusdec_chip) fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->private = kzalloc(sizeof(struct elantech_data), GFP_KERNEL);
	if (!ec_chip->private) {
		pr_err("asusdec: memory allocation (elantech_data) fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->client = client;
	i2c_set_clientdata(client, ec_chip);

	mutex_init(&ec_chip->input_lock);
	mutex_init(&ec_chip->dock_init_lock);

	init_timer(&ec_chip->asusdec_timer);
	ec_chip->asusdec_timer.function = asusdec_reset_counter;

	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "asusdec_wake");
	wake_lock_init(&ec_chip->wake_lock_init, WAKE_LOCK_SUSPEND, "asusdec_wake_init");

	ec_chip->status = 0;
	ec_chip->dock_in = 0;
	ec_chip->dock_init = 0;
	ec_chip->d_index = 0;
	ec_chip->init_success = 0;
	ec_chip->tp_wait_ack = 0;
	ec_chip->tp_enable = 1;
	ec_chip->ec_in_s3 = 1;
	ec_chip->susb_on = 1;
	ec_chip->indev = NULL;
	ec_chip->private->input_dev = NULL;

	asus_dockram_init(&dock_client, client, ASUSDEC_DOCKRAM_ADDR);

	ec_chip->dock_sdev.name = "dock";
	ec_chip->dock_sdev.print_state = asusdec_switch_state;
	if(switch_dev_register(&ec_chip->dock_sdev) < 0) {
		pr_err("asusdec: switch_dev_register for dock failed!\n");
		goto exit;
	}
	switch_set_state(&ec_chip->dock_sdev, 0);

	asusdec_wq = create_singlethread_workqueue("asusdec_wq");

#if BATTERY_DRIVER
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_pad_battery_report_work, asusdec_pad_battery_report_function);
#endif
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_work, asusdec_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_dock_init_work, asusdec_dock_init_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_led_on_work, asusdec_keypad_led_on);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_led_off_work, asusdec_keypad_led_off);

	asus_ec_irq_request(client, DOCK_IN_GPIO, asusdec_interrupt_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, DOCK_IN);
	asus_ec_irq_request(client, DOCK_ECREQ_GPIO, NULL, 0, DOCK_REQUEST);
	asus_ec_irq_request(client, DOCK_APWAKE_GPIO, asusdec_interrupt_handler,
			IRQF_TRIGGER_LOW, DOCK_INPUT);

	queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);

	pr_info("asusdec: probed\n");
	return 0;

exit:
	return err;
}

static int __devexit asusdec_remove(struct i2c_client *client)
{
	struct asusdec_chip *chip = i2c_get_clientdata(client);

	input_unregister_device(chip->indev);
	kfree(chip);
	return 0;
}

static int asusdec_suspend(struct device *dev)
{
	int err;

	ec_chip->susb_on = 0;
	flush_workqueue(asusdec_wq);

	if (ec_chip->dock_in && !ec_chip->ec_in_s3){
		err = asus_ec_reset(ec_chip->client);
		if (err < 0)
			goto fail_to_access_ec;

		asus_dockram_read(&dock_client, 0x0A, ec_chip->i2c_dm_data);

		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xDF;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x22;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0x7F;

		asus_dockram_write(&dock_client, 0x0A, ec_chip->i2c_dm_data);
	}

fail_to_access_ec:
	ec_chip->init_success = 0;
	ec_chip->ec_in_s3 = 1;
	ec_chip->tp_model = -1;

	pr_info("asusdec: suspended\n");
	return 0;
}

static int asusdec_resume(struct device *dev)
{
	if (!gpio_get_value(DOCK_IN_GPIO) &&
	     gpio_get_value(DOCK_APWAKE_GPIO)) {
		asus_ec_signal_request(ec_chip->client, DOCK_ECREQ_GPIO);
		ec_chip->dock_init = 0;
	}

	wake_lock(&ec_chip->wake_lock_init);
	ec_chip->init_success = 0;
	queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);

	pr_info("asusdec: resumed\n");
	return 0;
}

int dock_ac_callback(void)
{
	int err;
	u8 ac_data[32];

	if (!ec_chip || !ec_chip->dock_in || !ec_chip->client)
		goto not_ready;

	err = asus_ec_reset(ec_chip->client);
	if (err < 0)
		goto not_ready;

	err = asus_dockram_read(&dock_client, 0x0A, ac_data);
	if (err < 0)
		goto not_ready;

	return ac_data[1] & 0x20;

not_ready:
	pr_info("asusdec: dock isn't ready\n");
	return -1;
}

static SIMPLE_DEV_PM_OPS(asusdec_dev_pm_ops, asusdec_suspend, asusdec_resume);

static const struct i2c_device_id asusdec_id[] = {
	{"asusdec", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, asusdec_id);

static struct i2c_driver asusdec_driver = {
	.class	= I2C_CLASS_HWMON,
	.probe	 = asusdec_probe,
	.remove	 = __devexit_p(asusdec_remove),
	.id_table = asusdec_id,
	.driver	 = {
		.name = "asusdec",
		.owner = THIS_MODULE,
		.pm = &asusdec_dev_pm_ops,
	},
};

static int __init asusdec_init(void)
{
	int err = 0;

	if (asusdec_major) {
		asusdec_dev = MKDEV(asusdec_major, asusdec_minor);
		err = register_chrdev_region(asusdec_dev, 1, "asusdec");
	} else {
		err = alloc_chrdev_region(&asusdec_dev, asusdec_minor, 1,"asusdec");
		asusdec_major = MAJOR(asusdec_dev);
	}

	err = i2c_add_driver(&asusdec_driver);
	if (err) {
		pr_err("asusdec: i2c_add_driver fail\n");
		goto i2c_add_driver_fail;
	}

	asusdec_class = class_create(THIS_MODULE, "asusdec");
	if (asusdec_class <= 0) {
		pr_err("asusdec: asusdec_class create fail\n");
		err = -1;
		goto class_create_fail;
	}

	asusdec_device = device_create(asusdec_class, NULL, MKDEV(asusdec_major, asusdec_minor), NULL, "asusdec");
	if (asusdec_device <= 0) {
		pr_err("asusdec: asusdec_device create fail\n");
		err = -1;
		goto device_create_fail;
	}

	pr_info("asusdec: initiated\n");
	return 0;

device_create_fail:
	class_destroy(asusdec_class);
class_create_fail:
	i2c_del_driver(&asusdec_driver);
i2c_add_driver_fail:
	return err;
}
module_init(asusdec_init);

static void __exit asusdec_exit(void)
{
	device_destroy(asusdec_class, MKDEV(asusdec_major, asusdec_minor));
	class_destroy(asusdec_class);
	i2c_del_driver(&asusdec_driver);
	unregister_chrdev_region(asusdec_dev, 1);
	switch_dev_unregister(&ec_chip->dock_sdev);
}
module_exit(asusdec_exit);

MODULE_DESCRIPTION("ASUS Dock EC Driver");
MODULE_LICENSE("GPL");
