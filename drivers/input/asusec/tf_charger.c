/*
 * Transformer Cable detect driver.
 *
 * Copyright (c) 2019, Svyatoslav Ryhel
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/asusec.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/gadget.h>

#include <mach/board-transformer-misc.h>

#include <../tegra_usb_phy.h>
#include <../drivers/usb/gadget/tegra_udc.h>

/*
 * previous_cable_status :
 *     CONNECT_TYPE_NONE = 0
 *     CONNECT_TYPE_SDP = 1
 *     CONNECT_TYPE_DCP = 2
 *     CONNECT_TYPE_CDP = 3
 *     CONNECT_TYPE_NON_STANDARD_CHARGER = 4
 */
static unsigned int previous_cable_status;
static int usb3_init = 0;

int usb_suspend_tag;

static struct usb_hcd *usb3_ehci_handle;
static struct delayed_work usb3_ehci_dock_in_work;
static struct tegra_udc *the_udc;

static struct gpio tf_charger_gpios[] = {
	{ DOCK_IN_GPIO,    GPIOF_IN,           DOCK_IN    },
	{ ADAPTER_IN_GPIO, GPIOF_IN,           ADAPTER_IN },
	{ LIMIT_SET0_GPIO, GPIOF_OUT_INIT_LOW, LIMIT_SET0 },
};

struct cable_info {

	/*
	 * The cable status:
	 * 0000: no cable
	 * 0001: USB cable
	 * 0011: AC apdater
	 */

	unsigned int cable_status;
	bool ac_15v_connected;

	struct delayed_work usb_cable_detect;
	struct mutex cable_info_mutex;
};

static struct cable_info s_cable_info;

void transformer_link_udc(struct tegra_udc *udc)
{
	the_udc = udc;
}

/*
 * Add for GPIO LIMIT_SET0 set
 * USB Cable  -> LIMIT_SET0 = 0
 * AC adaptor -> LIMIT_SET0 = 1
 */
void transformer_cable_detect(struct tegra_udc *udc)
{
	int dock_in    = !gpio_get_value(DOCK_IN_GPIO);
	int adapter_in = gpio_get_value(ADAPTER_IN_GPIO);
	int dock_ac    = 0;
	int ask_ec_num = 0;

	mutex_lock(&s_cable_info.cable_info_mutex);
	s_cable_info.cable_status = 0x0; //0000

	switch (udc->connect_type) {
		case CONNECT_TYPE_NONE:
			pr_info("tf_charger: USB/AC cable is disconnected\n");
			s_cable_info.cable_status = 0x0; //0000
			s_cable_info.ac_15v_connected = false;
			gpio_direction_output(LIMIT_SET0_GPIO, 0);
			break;

		case CONNECT_TYPE_SDP:
			pr_info("tf_charger: detected SDP port\n");
			if (!adapter_in) {
				pr_info("tf_charger: USB cable is connected (0.5A)\n");
				s_cable_info.cable_status = 0x1; //0001
				s_cable_info.ac_15v_connected = false;
				gpio_direction_output(LIMIT_SET0_GPIO, 0);
			} else if (adapter_in) {
				pr_info("tf_charger: USB cable + AC adapter 15V is connected (1A)\n");
				s_cable_info.cable_status = 0x3; //0011
				s_cable_info.ac_15v_connected = true;
				gpio_direction_output(LIMIT_SET0_GPIO, 1);
			}
			break;

		case CONNECT_TYPE_DCP:
			pr_info("tf_charger: detected DCP port (wall charger)\n");
			if (!dock_in) {
				if (adapter_in) {
					pr_info("tf_charger: AC adapter 15V connected (1A)\n");
					s_cable_info.cable_status = 0x3; //0011
					s_cable_info.ac_15v_connected = true;
				} else if (!adapter_in) {
					pr_info("tf_charger: AC adapter 5V connected (1A)\n");
					s_cable_info.cable_status = 0x1; //0001
					s_cable_info.ac_15v_connected = false;
				} else {
					pr_err("tf_charger: undefined adapter status\n");
					s_cable_info.cable_status = 0x1; //0001
				}
			} else if (dock_in) {
				if (usb_suspend_tag) {
					mutex_unlock(&s_cable_info.cable_info_mutex);
					return;
				}
				while (ask_ec_num < 3) {
					ask_ec_num ++;
#if DOCK_EC_ENABLED
					dock_ac = dock_ac_callback();
#endif
					pr_info("tf_charger: limt_set1 = %d dock_ac = %#X\n", adapter_in, dock_ac);
					s_cable_info.cable_status = 0x1; //0001
					s_cable_info.ac_15v_connected = false;

					if (dock_ac == 0x20) {
						pr_info("tf_charger: AC adapter + Docking 15V connected (1A)\n");
						s_cable_info.cable_status = 0x3; //0011
						s_cable_info.ac_15v_connected = true;
						ask_ec_num = 0;
						break;
					} else if (dock_ac == 0) {
						pr_info("tf_charger: AC adapter + Docking 5V connected (1A)\n");
						s_cable_info.cable_status = 0x1; //0001
						s_cable_info.ac_15v_connected = false;
						ask_ec_num = 0;
						break;
					} else {
						msleep(500);
						continue;
					}
				}
			} else {
				pr_err("tf_charger: undefined USB status\n");
			}
			gpio_direction_output(LIMIT_SET0_GPIO, 1);
			break;

		case CONNECT_TYPE_CDP:
			pr_info("tf_charger: detected CDP port (1A USB port)\n");
			s_cable_info.cable_status = 0x1; //0001
			s_cable_info.ac_15v_connected = false;
			gpio_direction_output(LIMIT_SET0_GPIO, 1);	//(5V/1.0A)
			break;

		case CONNECT_TYPE_NON_STANDARD_CHARGER:
		default:
			pr_info("tf_charger: detected non-standard charging port\n");
			s_cable_info.cable_status = 0x1; //0001
			s_cable_info.ac_15v_connected = false;
			gpio_direction_output(LIMIT_SET0_GPIO, 0);	//(5V/0.5A)
			break;
	}

	mutex_unlock(&s_cable_info.cable_info_mutex);
}

static void usb_cable_detection(struct work_struct *w)
{
	tegra_detect_charging_type_is_cdp_or_dcp(the_udc);
	transformer_cable_detect(the_udc);
#if BATTERY_CALLBACK_ENABLED
	battery_callback(s_cable_info.cable_status);
#endif
}

/* For the issue of USB AC adaptor inserted half on PAD + Docking */
void fsl_dock_ec_callback(void)
{
	int dock_in = !gpio_get_value(DOCK_IN_GPIO);

	pr_info("tf_charger: cable status %d\n", s_cable_info.cable_status);

	if (dock_in && (s_cable_info.cable_status != 0) &&
	   (the_udc->connect_type == CONNECT_TYPE_NON_STANDARD_CHARGER))
	    schedule_delayed_work(&s_cable_info.usb_cable_detect, 0*HZ);
}

/* For the issue of USB AC adaptor inserted half on PAD */
static irqreturn_t charger_interrupt_handler(int irq, void *dev_id)
{
	int adapter_in = gpio_get_value(ADAPTER_IN_GPIO);
	int dock_in = !gpio_get_value(DOCK_IN_GPIO);

	if (irq == gpio_to_irq(ADAPTER_IN_GPIO)) {
		if (!dock_in && (adapter_in != s_cable_info.ac_15v_connected) &&
		   (the_udc->connect_type == CONNECT_TYPE_NON_STANDARD_CHARGER))
			schedule_delayed_work(&s_cable_info.usb_cable_detect, 0.2*HZ);
	}

	if (irq == gpio_to_irq(DOCK_IN_GPIO)) {
		if (usb3_ehci_handle && dock_in)
			schedule_delayed_work(&usb3_ehci_dock_in_work, 0.5*HZ);
	}

	return IRQ_HANDLED;
}

void utmi_xcvr_setup_corrector(struct tegra_usb_phy *phy)
{
	if (phy->inst == 0 || phy->inst == 2) {
		if (tegra3_get_project_id() == TEGRA3_PROJECT_TF201) {
			if (phy->utmi_xcvr_setup >= 48)
				phy->utmi_xcvr_setup -= 48;
			else
				phy->utmi_xcvr_setup = 0;
		} else {
			phy->utmi_xcvr_setup += 8;
			if (phy->utmi_xcvr_setup > 63)
				phy->utmi_xcvr_setup = 63;
		}
		pr_info("phy->inst = %d, phy->utmi_xcvr_setup = %d\n", phy->inst, phy->utmi_xcvr_setup);
	}
}

void tegra_usb3_smi_backlight_on_callback(void)
{
	if (usb3_init) {
		if(!gpio_get_value(DOCK_IN_GPIO))
			schedule_delayed_work(&usb3_ehci_dock_in_work, 0.5*HZ);
	}
}

static void usb3_ehci_dock_in_work_handler(struct work_struct *w)
{
	usb_hcd_resume_root_hub(usb3_ehci_handle);
	msleep(100);
}

static void usb3_dockin_irq(struct usb_hcd *hcd)
{
	asus_ec_irq_request(hcd, DOCK_IN_GPIO, charger_interrupt_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, DOCK_IN);

	INIT_DELAYED_WORK(&usb3_ehci_dock_in_work, usb3_ehci_dock_in_work_handler);
}

void transformer_usb_definer(struct usb_hcd *hcd, struct tegra_usb_phy *phy)
{
	if (phy->inst == 1) {
		if (!usb3_init) {
			usb3_ehci_handle = hcd;
			usb3_init = 1;
			usb3_dockin_irq(hcd);
		} else if (usb3_init) {
			free_irq(gpio_to_irq(DOCK_IN_GPIO), hcd);
			usb3_ehci_handle = NULL;
			usb3_init = 0;
		}
	}
}

void cable_status_setup(struct tegra_udc *udc)
{
#if BATTERY_CALLBACK_ENABLED
	if (previous_cable_status != udc->connect_type)
		battery_callback(s_cable_info.cable_status);
#endif
	previous_cable_status = udc->connect_type;
}

void cable_status_reset(void)
{
#if BATTERY_CALLBACK_ENABLED
		battery_callback(0x0);
		previous_cable_status = 0;
#endif
}

int __init transformer_udc_init(void)
{
	int ret = 0;

	/* Cable status init */
	usb_suspend_tag = 0;
	previous_cable_status = 0;

	mutex_init(&s_cable_info.cable_info_mutex);

	s_cable_info.cable_status = 0x0;
	s_cable_info.ac_15v_connected = false;

	INIT_DELAYED_WORK(&s_cable_info.usb_cable_detect, usb_cable_detection);

	/* Charging gpios init */
	ret = gpio_request_array(tf_charger_gpios, ARRAY_SIZE(tf_charger_gpios));
	if (ret < 0)
		pr_err("tf_charger: failed to request tf_charger_gpios array: %d\n", ret);

	ret = request_irq(gpio_to_irq(ADAPTER_IN_GPIO), charger_interrupt_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, ADAPTER_IN, NULL);
	if (ret < 0) {
		pr_err("tf_charger: could not register for %s interrupt, irq = %d, rc = %d\n",
				ADAPTER_IN, gpio_to_irq(ADAPTER_IN_GPIO), ret);
		ret = -EIO;
	}

	return ret;
}

void __exit transformer_udc_exit(void)
{
	/* Charging gpios free */
	gpio_free_array(tf_charger_gpios, ARRAY_SIZE(tf_charger_gpios));
	free_irq(gpio_to_irq(ADAPTER_IN_GPIO), NULL);
	mutex_destroy(&s_cable_info.cable_info_mutex);
}
