/*
 * ASUS PAD driver
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
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/power/pad_battery.h>
#include <linux/statfs.h>

#include <asm/gpio.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>

#include <../gpio-names.h>

#include "asusec.h"

/*
 * global variable
 */
static unsigned int asuspec_apwake_gpio = TEGRA_GPIO_PS2;
static unsigned int asuspec_ecreq_gpio = TEGRA_GPIO_PQ1;

static struct i2c_client pad_client;
static struct class *asuspec_class;
static struct device *asuspec_device;
static struct asuspec_chip *ec_chip;

static dev_t asuspec_dev;
static int asuspec_major = 0;
static int asuspec_minor = 0;

static struct workqueue_struct *asuspec_wq;

/*
 * functions definition
 */

int asuspec_battery_monitor(char *cmd)
{
	int ret = 0;

	if (ec_chip->ec_in_s3)
		asus_ec_signal_request(ec_chip->client, asuspec_ecreq_gpio);

	ret = asus_dockram_read(&pad_client, 0x14, ec_chip->i2c_dm_battery);
	if (ret < 0){
		pr_err("asuspec: fail to access battery info\n");
		return -1;
	}

	if (!strcmp(cmd, "status"))
		ret = (ec_chip->i2c_dm_battery[2] << 8 ) | ec_chip->i2c_dm_battery[1];
	else if (!strcmp(cmd, "temperature"))
		ret = (ec_chip->i2c_dm_battery[8] << 8 ) | ec_chip->i2c_dm_battery[7];
	else if (!strcmp(cmd, "voltage"))
		ret = (ec_chip->i2c_dm_battery[10] << 8 ) | ec_chip->i2c_dm_battery[9];
	else if (!strcmp(cmd, "current"))
		ret = (ec_chip->i2c_dm_battery[12] << 8 ) | ec_chip->i2c_dm_battery[11];
	else if (!strcmp(cmd, "capacity"))
		ret = (ec_chip->i2c_dm_battery[14] << 8 ) | ec_chip->i2c_dm_battery[13];
	else if (!strcmp(cmd, "remaining_capacity"))
		ret = (ec_chip->i2c_dm_battery[16] << 8 ) | ec_chip->i2c_dm_battery[15];
	else if (!strcmp(cmd, "avg_time_to_empty"))
		ret = (ec_chip->i2c_dm_battery[18] << 8 ) | ec_chip->i2c_dm_battery[17];
	else if (!strcmp(cmd, "avg_time_to_full"))
		ret = (ec_chip->i2c_dm_battery[20] << 8 ) | ec_chip->i2c_dm_battery[19];
	else {
		pr_err("asuspec: unknown command\n");
		ret = -1;
	}
	return ret;
}

static void asuspec_enter_normal_mode(struct i2c_client *client)
{
	int err = 0;

	err = asus_dockram_read(&pad_client, 0x0A, ec_chip->i2c_dm_data);
	if (err < 0)
		goto err_exit;

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xBF;

	err = asus_dockram_write(&pad_client, 0x0A, ec_chip->i2c_dm_data);
	if (err < 0)
		goto err_exit;

	pr_info("asuspec: entering normal mode\n");

err_exit:
	if (err < 0)
		pr_err("asuspec: entering normal mode failed\n");
}

static int asuspec_chip_init(struct i2c_client *client)
{
	int err = 0;

	err = asus_ec_detect(&pad_client, client, ec_chip->i2c_data);
	if (err < 0)
		goto fail_to_access_ec;

	asuspec_enter_normal_mode(client);

	ec_chip->status = 1;

fail_to_access_ec:
	return 0;
}

static irqreturn_t asuspec_interrupt_handler(int irq, void *dev_id)
{
	if (irq == gpio_to_irq(asuspec_apwake_gpio)){
		disable_irq_nosync(irq);
		queue_delayed_work(asuspec_wq, &ec_chip->asuspec_work, 0);
	}

	return IRQ_HANDLED;
}

static int asuspec_irq_ec_request(struct i2c_client *client)
{
	int err = 0;
	unsigned gpio = asuspec_ecreq_gpio;
	const char* label = "asuspec_request";

	err = gpio_request(gpio, label);
	if (err) {
		pr_err("asuspec: gpio_request failed for input %d\n", gpio);
		goto err_exit;
	}

	err = gpio_direction_output(gpio, 1);
	if (err) {
		pr_err("asuspec: gpio_direction_output failed for input %d\n", gpio);
		goto err_exit;
	}

	return 0;

err_exit:
	return err;
}

static int asuspec_irq_ec_apwake(struct i2c_client *client)
{
	int err = 0;
	unsigned gpio = asuspec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_apwake";

	err = gpio_request(gpio, label);
	if (err) {
		pr_err("asuspec: gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	err = gpio_direction_input(gpio);
	if (err) {
		pr_err("asuspec: gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}

	err = request_irq(irq, asuspec_interrupt_handler, IRQF_TRIGGER_LOW, label, client);
	if (err < 0) {
		pr_err("asuspec: can't register for %s interrupt, irq = %d, err = %d\n", label, irq, err);
		err = -EIO;
		goto err_gpio_request_irq_fail;
	}

	enable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));

	return 0;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed:
	return err;
}

static void asuspec_enter_s3_timer(unsigned long data)
{
	queue_delayed_work(asuspec_wq, &ec_chip->asuspec_enter_s3_work, 0);
}

static void asuspec_smi(void)
{
	if (ec_chip->i2c_data[2] == ASUSEC_SMI_HANDSHAKING){
		pr_info("asuspec: ASUSPEC_SMI_HANDSHAKING\n");
		if (!ec_chip->status)
			asuspec_chip_init(ec_chip->client);
		ec_chip->ec_in_s3 = 0;
	} else if (ec_chip->i2c_data[2] == ASUSEC_SMI_RESET){
		pr_info("asuspec: ASUSPEC_SMI_RESET\n");
		queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
	} else if (ec_chip->i2c_data[2] == ASUSEC_SMI_WAKE){
		pr_info("asuspec: ASUSPEC_SMI_WAKE\n");
	}
}

static void asuspec_enter_s3_work_function(struct work_struct *dat)
{
	int err = 0;

	mutex_lock(&ec_chip->state_change_lock);

	ec_chip->ec_in_s3 = 1;

	err = asus_dockram_read(&pad_client, 0x0A, ec_chip->i2c_dm_data);
	if (err < 0)
		goto err_exit;

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x02;

	err = asus_dockram_write(&pad_client, 0x0A, ec_chip->i2c_dm_data);
	if (err < 0)
		goto err_exit;

	pr_info("asuspec: EC in S3 mode\n");

err_exit:
	if (err < 0)
		pr_err("asuspec: send s3 command fail\n");
	mutex_unlock(&ec_chip->state_change_lock);
}

static void asuspec_init_work_function(struct work_struct *dat)
{
	asus_ec_signal_request(ec_chip->client, asuspec_ecreq_gpio);
	asuspec_chip_init(ec_chip->client);
}

static void asuspec_work_function(struct work_struct *dat)
{
	int irq = gpio_to_irq(asuspec_apwake_gpio);
	int err = 0;

	err = asus_ec_read(ec_chip->client, ec_chip->i2c_data);
	enable_irq(irq);

	pr_info("asuspec: 0x%x 0x%x 0x%x 0x%x\n", ec_chip->i2c_data[0],
		ec_chip->i2c_data[1], ec_chip->i2c_data[2], ec_chip->i2c_data[3]);

	if (err < 0)
		return;

	if (ec_chip->i2c_data[1] & ASUSEC_OBF_MASK){
		if (ec_chip->i2c_data[1] & ASUSEC_SMI_MASK){
			asuspec_smi();
			return;
		}
	}
}

static int __devinit asuspec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	ec_chip = kzalloc(sizeof (struct asuspec_chip), GFP_KERNEL);
	if (!ec_chip) {
		pr_err("asuspec: memory allocation failed\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->client = client;
	i2c_set_clientdata(client, ec_chip);

	init_timer(&ec_chip->asuspec_timer);
	ec_chip->asuspec_timer.function = asuspec_enter_s3_timer;

	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "asuspec_wake");

	mutex_init(&ec_chip->state_change_lock);

	ec_chip->status = 0;
	ec_chip->ec_in_s3 = 0;

	asus_dockram_init(&pad_client, client, ASUSPEC_DOCKRAM_ADDR);

	asuspec_wq = create_singlethread_workqueue("asuspec_wq");

	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_work, asuspec_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_init_work, asuspec_init_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_enter_s3_work, asuspec_enter_s3_work_function);

	asuspec_irq_ec_request(client);
	asuspec_irq_ec_apwake(client);

	queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);

	pr_info("asuspec: probed\n");

	return 0;

exit:
	return err;
}

static int __devexit asuspec_remove(struct i2c_client *client)
{
	struct asuspec_chip *chip = i2c_get_clientdata(client);

	kfree(chip);
	return 0;
}

static int asuspec_suspend(struct device *dev)
{
	del_timer_sync(&ec_chip->asuspec_timer);
	ec_chip->ec_in_s3 = 1;
	pr_info("asuspec: suspended\n");
	return 0;
}

static int asuspec_resume(struct device *dev)
{
	pr_info("asuspec: resumed\n");
	return 0;
}

static SIMPLE_DEV_PM_OPS(asuspec_dev_pm_ops, asuspec_suspend, asuspec_resume);

static const struct i2c_device_id asuspec_id[] = {
	{"asuspec", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, asuspec_id);

static struct i2c_driver asuspec_driver = {
	.class	= I2C_CLASS_HWMON,
	.probe	 = asuspec_probe,
	.remove	 = __devexit_p(asuspec_remove),
	.id_table = asuspec_id,
	.driver	 = {
		.name = "asuspec",
		.owner = THIS_MODULE,
		.pm = &asuspec_dev_pm_ops,
	},
};

static int __init asuspec_init(void)
{
	int err = 0;

	if (asuspec_major) {
		asuspec_dev = MKDEV(asuspec_major, asuspec_minor);
		err = register_chrdev_region(asuspec_dev, 1, "asuspec");
	} else {
		err = alloc_chrdev_region(&asuspec_dev, asuspec_minor, 1, "asuspec");
		asuspec_major = MAJOR(asuspec_dev);
	}

	err = i2c_add_driver(&asuspec_driver);
	if (err) {
		pr_err("asuspec: i2c_add_driver failed\n");
		goto i2c_add_driver_fail;
	}

	asuspec_class = class_create(THIS_MODULE, "asuspec");
	if (asuspec_class <= 0) {
		pr_err("asuspec: asuspec_class create failed\n");
		err = -1;
		goto class_create_fail;
	}

	asuspec_device = device_create(asuspec_class, NULL,
				MKDEV(asuspec_major, asuspec_minor), NULL, "asuspec");
	if (asuspec_device <= 0) {
		pr_err("asuspec: asuspec_device create failed\n");
		err = -1;
		goto device_create_fail;
	}

	return 0;

device_create_fail:
	class_destroy(asuspec_class);
class_create_fail:
	i2c_del_driver(&asuspec_driver);
i2c_add_driver_fail:
	return err;
}
module_init(asuspec_init);

static void __exit asuspec_exit(void)
{
	device_destroy(asuspec_class, MKDEV(asuspec_major, asuspec_minor));
	class_destroy(asuspec_class);
	i2c_del_driver(&asuspec_driver);
	unregister_chrdev_region(asuspec_dev, 1);
}
module_exit(asuspec_exit);

MODULE_DESCRIPTION("ASUS PAD EC Driver");
MODULE_LICENSE("GPL");
