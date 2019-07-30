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

struct i2c_client dockram_client;
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
static void asuspec_dockram_init(struct i2c_client *client)
{
	dockram_client.adapter = client->adapter;
	dockram_client.addr = 0x17;
	dockram_client.detected = client->detected;
	dockram_client.dev = client->dev;
	dockram_client.driver = client->driver;
	dockram_client.flags = client->flags;
	strcpy(dockram_client.name, client->name);
	ec_chip->ec_ram_init = ASUSPEC_MAGIC_NUM;
}

static int asuspec_dockram_write_data(int cmd, int length)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		pr_err("asuspec: DockRam is not ready\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE)
		return -3;

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_data);
	if (ret < 0)
		pr_err("asuspec: fail to write dockram data, status %d\n", ret);
	else
		ec_chip->i2c_err_count = 0;

	return ret;
}

static int asuspec_dockram_read_data(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		pr_err("asuspec: DockRam is not ready\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE)
		return -3;

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);
	if (ret < 0)
		pr_err("asuspec: fail to read dockram data, status %d\n", ret);
	else
		ec_chip->i2c_err_count = 0;

	return ret;
}

static int asuspec_dockram_read_battery(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		pr_err("asuspec: DockRam is not ready\n");
		return -1;
	}

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_battery);
	if (ret < 0) {
		pr_err("asuspec: fail to read dockram battery, status %d\n", ret);
		ret = -1;
	} else {
		if (ec_chip->apwake_disabled){
			mutex_lock(&ec_chip->irq_lock);
			if (ec_chip->apwake_disabled){
				enable_irq(gpio_to_irq(asuspec_apwake_gpio));
				enable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));
				ec_chip->apwake_disabled = 0;
				pr_err("asuspec: enable pad apwake\n");
			}
			mutex_unlock(&ec_chip->irq_lock);
		}
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static void asuspec_send_ec_req(void)
{
	pr_info("asuspec: send EC request\n");
	gpio_set_value(asuspec_ecreq_gpio, 0);
	msleep(DELAY_TIME_MS);
	gpio_set_value(asuspec_ecreq_gpio, 1);
}

int asuspec_battery_monitor(char *cmd)
{
	int ret = 0;

	if (ec_chip->ec_in_s3){
		asuspec_send_ec_req();
		msleep(200);
	}

	ret = asuspec_dockram_read_battery(0x14);
	if (ret < 0){
		pr_err("asuspec: fail to access battery info\n");
		return -1;
	} else {
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
			ret = -2;
		}
		return ret;
	}
}

static int asuspec_i2c_write_data(struct i2c_client *client, u16 data)
{
	int ret = 0;

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE)
		return -3;

	ret = i2c_smbus_write_word_data(client, 0x64, data);
	if (ret < 0)
		pr_err("asuspec: Fail to write data, status %d\n", ret);
	else
		ec_chip->i2c_err_count = 0;

	return ret;
}

static int asuspec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		mutex_lock(&ec_chip->irq_lock);
		if(!ec_chip->apwake_disabled){
			disable_irq_nosync(gpio_to_irq(asuspec_apwake_gpio));
			disable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));
			ec_chip->apwake_disabled = 1;
			pr_err("asuspec: disable pad apwake\n");
		}
		mutex_unlock(&ec_chip->irq_lock);
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);
	if (ret < 0) {
		pr_err("asuspec: fail to read data, status %d\n", ret);
		ec_chip->i2c_err_count++;
	} else {
		ec_chip->i2c_err_count = 0;
	}

	return ret;
}

static int asuspec_i2c_test(struct i2c_client *client)
{
	return asuspec_i2c_write_data(client, 0x0000);
}

static void asuspec_enter_normal_mode(void)
{
	int err = 0;
	int i = 0;

	for (i = 0; i < 3; i++){
		err = asuspec_dockram_read_data(0x0A);
		if (err < 0){
			pr_err("asuspec: fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xBF;

	for (i = 0; i < 3; i++){
		err = asuspec_dockram_write_data(0x0A,9);
		if (err < 0){
			pr_err("asuspec: entering normal mode failed\n");
			msleep(100);
		} else {
			pr_info("asuspec: entering normal mode\n");
			break;
		}
	}
}

static int asuspec_chip_init(struct i2c_client *client)
{
	int err = 0;
	int i = 0;

	for (i = 0; i < 10; i++){
		err = asuspec_i2c_test(client);
		if (err < 0)
			msleep(300);
		else
			break;
	}

	if (err < 0)
		goto fail_to_access_ec;

	for (i = 0; i < 8; i++)
		asuspec_i2c_read_data(client);

	if (asuspec_dockram_read_data(0x01) < 0)
		goto fail_to_access_ec;
	strcpy(ec_chip->ec_model_name, &ec_chip->i2c_dm_data[1]);
	pr_info("PAD Model Name: %s\n", ec_chip->ec_model_name);

	if (asuspec_dockram_read_data(0x02) < 0)
		goto fail_to_access_ec;
	strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
	pr_info("PAD EC-FW Version: %s\n", ec_chip->ec_version);

	if (asuspec_dockram_read_data(0x03) < 0)
		goto fail_to_access_ec;
	pr_info("PAD EC-Config Format: %s\n", &ec_chip->i2c_dm_data[1]);

	if (asuspec_dockram_read_data(0x04) < 0)
		goto fail_to_access_ec;
	strcpy(ec_chip->ec_pcba, &ec_chip->i2c_dm_data[1]);
	pr_info("PAD PCBA Version: %s\n", ec_chip->ec_pcba);

	asuspec_enter_normal_mode();

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
	int i = 0;

	mutex_lock(&ec_chip->state_change_lock);

	ec_chip->ec_in_s3 = 1;
	for (i = 0; i < 3; i++) {
		err = asuspec_dockram_read_data(0x0A);
		if (err < 0){
			pr_err("asuspec: fail to get control flag\n");
			msleep(100);
		} else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x02;

	for (i = 0; i < 3; i++) {
		err = asuspec_dockram_write_data(0x0A,9);
		if (err < 0){
			pr_err("asuspec: Send s3 command fail\n");
			msleep(100);
		} else {
			pr_info("asuspec: EC in S3\n");
			break;
		}
	}
	mutex_unlock(&ec_chip->state_change_lock);
}

static void asuspec_init_work_function(struct work_struct *dat)
{
	asuspec_send_ec_req();
	msleep(200);
	asuspec_chip_init(ec_chip->client);
}

static void asuspec_work_function(struct work_struct *dat)
{
	int gpio = asuspec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	int err = 0;

	err = asuspec_i2c_read_data(ec_chip->client);
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

	mutex_init(&ec_chip->irq_lock);
	mutex_init(&ec_chip->state_change_lock);

	ec_chip->ec_ram_init = 0;
	ec_chip->status = 0;
	ec_chip->ec_in_s3 = 0;
	ec_chip->apwake_disabled = 0;

	asuspec_dockram_init(client);

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

	input_unregister_device(chip->indev);
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
	ec_chip->i2c_err_count = 0;
	pr_info("asuspec: resumeed\n");
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

	asuspec_device = device_create(asuspec_class, NULL, MKDEV(asuspec_major, asuspec_minor), NULL, "asuspec");
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
