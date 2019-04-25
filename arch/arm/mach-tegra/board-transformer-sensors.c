/*
 * arch/arm/mach-tegra/board-transformer-sensors.c
 *
 * Copyright (c) 2010-2012, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/nct1008.h>
#include <linux/mpu_inv.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#ifdef CONFIG_INPUT_LID
#include <linux/input/lid.h>
#endif

#include <mach/edp.h>
#include <mach/board-transformer-misc.h>

#include "board-transformer.h"
#include "cpu-tegra.h"

static struct throttle_table tj_throttle_table[] = {
	      /* CPU_THROT_LOW cannot be used by other than CPU */
	      /* NO_CAP cannot be used by CPU */
	      /*      CPU,    CBUS,    SCLK,     EMC */
	      { { 1000000,  NO_CAP,  NO_CAP,  NO_CAP } },
	      { {  760000,  NO_CAP,  NO_CAP,  NO_CAP } },
	      { {  760000,  NO_CAP,  NO_CAP,  NO_CAP } },
	      { {  620000,  NO_CAP,  NO_CAP,  NO_CAP } },
	      { {  620000,  NO_CAP,  NO_CAP,  NO_CAP } },
	      { {  620000,  437000,  NO_CAP,  NO_CAP } },
	      { {  620000,  352000,  NO_CAP,  NO_CAP } },
	      { {  475000,  352000,  NO_CAP,  NO_CAP } },
	      { {  475000,  352000,  NO_CAP,  NO_CAP } },
	      { {  475000,  352000,  250000,  375000 } },
	      { {  475000,  352000,  250000,  375000 } },
	      { {  475000,  247000,  204000,  375000 } },
	      { {  475000,  247000,  204000,  204000 } },
	      { {  475000,  247000,  204000,  204000 } },
	{ { CPU_THROT_LOW,  247000,  204000,  102000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init cardhu_throttle_init(void)
{
	balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(cardhu_throttle_init);

static struct nct1008_platform_data cardhu_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
	.shutdown_ext_limit = 90,
	.shutdown_local_limit = 90,

	.passive_delay = 2000,

	.num_trips = 1,
	.trips = {
		/* Thermal Throttling */
		[0] = {
			.cdev_type = "tegra-balanced",
			.trip_temp = 80000,
			.trip_type = THERMAL_TRIP_PASSIVE,
			.upper = THERMAL_NO_LIMIT,
			.lower = THERMAL_NO_LIMIT,
			.hysteresis = 0,
		},
	},
};

static struct i2c_board_info cardhu_i2c4_pad_bat_board_info[] = {
	{
		I2C_BOARD_INFO("pad-battery", 0xb),
	},
};

static struct i2c_board_info cardhu_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &cardhu_nct1008_pdata,
		.irq = -1,
	}
};

static int cardhu_nct1008_init(void)
{
	int ret = 0;

	cardhu_i2c4_nct1008_board_info[0].irq =
		gpio_to_irq(TEGRA_GPIO_PCC2);

	ret = gpio_request_one(TEGRA_GPIO_PCC2, GPIOF_DIR_IN, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed\n", __func__);
		return ret;
	}

	tegra_platform_edp_init(cardhu_nct1008_pdata.trips,
				&cardhu_nct1008_pdata.num_trips,
				0); /* edp temperature margin */
	return ret;
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct thermal_trip_info skin_trips[] = {
	{
		.cdev_type = "skin-balanced",
		.trip_temp = 45000,
		.trip_type = THERMAL_TRIP_PASSIVE,
		.upper = THERMAL_NO_LIMIT,
		.lower = THERMAL_NO_LIMIT,
		.hysteresis = 0,
	},
};

static struct therm_est_subdevice skin_devs[] = {
	{
		.dev_data = "nct_ext",
		.coeffs = {
			2, 1, 1, 1,
			1, 1, 1, 1,
			1, 1, 1, 0,
			1, 1, 0, 0,
			0, 0, -1, -7
		},
	},
	{
		.dev_data = "nct_int",
		.coeffs = {
			-11, -7, -5, -3,
			-3, -2, -1, 0,
			0, 0, 1, 1,
			1, 2, 2, 3,
			4, 6, 11, 18
		},
	},
};

static struct therm_est_data skin_data = {
	.num_trips = ARRAY_SIZE(skin_trips),
	.trips = skin_trips,
	.toffset = 9793,
	.polling_period = 1100,
	.passive_delay = 30000,
	.tc1 = 5,
	.tc2 = 1,
	.ndevs = ARRAY_SIZE(skin_devs),
	.devs = skin_devs,
};

static struct throttle_table skin_throttle_table[] = {
		/* CPU_THROT_LOW cannot be used by other than CPU */
		/* NO_CAP cannot be used by CPU */
		/*      CPU,    CBUS,    SCLK,     EMC */
		{ { 1000000,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  760000,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  760000,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  620000,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  620000,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  620000,  437000,  NO_CAP,  NO_CAP } },
		{ {  620000,  352000,  NO_CAP,  NO_CAP } },
		{ {  475000,  352000,  NO_CAP,  NO_CAP } },
		{ {  475000,  352000,  NO_CAP,  NO_CAP } },
		{ {  475000,  352000,  250000,  375000 } },
		{ {  475000,  352000,  250000,  375000 } },
		{ {  475000,  247000,  204000,  375000 } },
		{ {  475000,  247000,  204000,  204000 } },
		{ {  475000,  247000,  204000,  204000 } },
	  { { CPU_THROT_LOW,  247000,  204000,  102000 } },
};

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = ARRAY_SIZE(skin_throttle_table),
	.throt_tab = skin_throttle_table,
};

static int __init cardhu_skin_init(void)
{
	balanced_throttle_register(&skin_throttle, "skin-balanced");
	tegra_skin_therm_est_device.dev.platform_data = &skin_data;
	platform_device_register(&tegra_skin_therm_est_device);

	return 0;
}
late_initcall(cardhu_skin_init);
#endif

static struct mpu_platform_data mpu3050_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	.orientation	= MPU_GYRO_ORIENTATION,	/* Located in board_[platformname].h */
};

static struct ext_slave_platform_data mpu3050_accel_data = {
	.address	= MPU_ACCEL_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_ACCEL_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_SECONDARY,
	.orientation	= MPU_ACCEL_ORIENTATION,	/* Located in board_[platformname].h */
};

static struct ext_slave_platform_data mpu3050_compass_data = {
	.address	= MPU_COMPASS_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_COMPASS_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_PRIMARY,
	.orientation	= MPU_COMPASS_ORIENTATION,	/* Located in board_[platformname].h */
};

static struct i2c_board_info __initdata inv_mpu_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu3050_gyro_data,
	},
	{
		I2C_BOARD_INFO(MPU_ACCEL_NAME, MPU_ACCEL_ADDR),
		.platform_data = &mpu3050_accel_data,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
		.platform_data = &mpu3050_compass_data,
	},
};

/* Sensors orientation definition */
struct mpu_orientation_def{
	__s8 gyro_orientation[9];
	__s8 accel_orientation[9];
	__s8 compass_orientation[9];
};

static void mpuirq_init(void)
{
	int ret = 0;
	int i = 0;

	pr_info("*** MPU START *** mpuirq_init...\n");

	if (tegra3_get_project_id() == TEGRA3_PROJECT_TF300T)
	{
		/* Use "TF300T" to check the project name */
		struct mpu_orientation_def TF300T = {
			TF300T_GYRO_ORIENTATION,
			TF300T_ACCEL_ORIENTATION,
			TF300T_COMPASS_ORIENTATION,
		};

		pr_info("initial mpu with TF300T config...\n");
		memcpy(mpu3050_gyro_data.orientation, TF300T.gyro_orientation, sizeof(mpu3050_gyro_data.orientation));
		memcpy(mpu3050_accel_data.orientation, TF300T.accel_orientation, sizeof(mpu3050_accel_data.orientation));
		memcpy(mpu3050_compass_data.orientation, TF300T.compass_orientation, sizeof(mpu3050_compass_data.orientation));
	}
	else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF300TG)
	{
		/* Use "TF300TG" to check the project name */
		struct mpu_orientation_def TF300TG = {
			TF300TG_GYRO_ORIENTATION,
			TF300TG_ACCEL_ORIENTATION,
			TF300TG_COMPASS_ORIENTATION,
		};

		pr_info("initial mpu with TF300TG config...\n");
		memcpy(mpu3050_gyro_data.orientation, TF300TG.gyro_orientation, sizeof(mpu3050_gyro_data.orientation));
		memcpy(mpu3050_accel_data.orientation, TF300TG.accel_orientation, sizeof(mpu3050_accel_data.orientation));
		memcpy(mpu3050_compass_data.orientation, TF300TG.compass_orientation, sizeof(mpu3050_compass_data.orientation));
	}
	else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
	{
		/* Use "TF700T" to check the project name */
		struct mpu_orientation_def TF700T = {
			TF700T_GYRO_ORIENTATION,
			TF700T_ACCEL_ORIENTATION,
			TF700T_COMPASS_ORIENTATION,
		};

		pr_info("initial mpu with TF700T config...\n");
		memcpy(mpu3050_gyro_data.orientation, TF700T.gyro_orientation, sizeof(mpu3050_gyro_data.orientation));
		memcpy(mpu3050_accel_data.orientation, TF700T.accel_orientation, sizeof(mpu3050_accel_data.orientation));
		memcpy(mpu3050_compass_data.orientation, TF700T.compass_orientation, sizeof(mpu3050_compass_data.orientation));
	}
	else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF300TL)
	{
		/* Use "TF300TL" to check the project name */
		struct mpu_orientation_def TF300TL = {
			TF300TL_GYRO_ORIENTATION,
			TF300TL_ACCEL_ORIENTATION,
			TF300TL_COMPASS_ORIENTATION,
		};

		pr_info("initial mpu with TF300TL config...\n");
		memcpy(mpu3050_gyro_data.orientation, TF300TL.gyro_orientation, sizeof(mpu3050_gyro_data.orientation));
		memcpy(mpu3050_accel_data.orientation, TF300TL.accel_orientation, sizeof(mpu3050_accel_data.orientation));
		memcpy(mpu3050_compass_data.orientation, TF300TL.compass_orientation, sizeof(mpu3050_compass_data.orientation));
	}
	else pr_info("initial mpu with TF201 config...\n");

#if	MPU_ACCEL_IRQ_GPIO
	/* ACCEL-IRQ assignment */
	ret = gpio_request_one(MPU_ACCEL_IRQ_GPIO, GPIOF_DIR_IN, MPU_ACCEL_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}
#endif

	/* MPU-IRQ assignment */
	ret = gpio_request_one(MPU_GYRO_IRQ_GPIO, GPIOF_DIR_IN, MPU_GYRO_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	pr_info("*** MPU END *** mpuirq_init...\n");

	inv_mpu_i2c2_board_info[i++].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
#if MPU_ACCEL_IRQ_GPIO
	inv_mpu_i2c2_board_info[i].irq = gpio_to_irq(MPU_ACCEL_IRQ_GPIO);
#endif
	i++;
#if MPU_COMPASS_IRQ_GPIO
	inv_mpu_i2c2_board_info[i++].irq = gpio_to_irq(MPU_COMPASS_IRQ_GPIO);
#endif
	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c2_board_info,
		ARRAY_SIZE(inv_mpu_i2c2_board_info));
}

static struct i2c_board_info cardhu_i2c1_board_info_al3010[] = {
	{
		I2C_BOARD_INFO("al3010",0x1C),
	},
};

#ifdef CONFIG_INPUT_LID
static struct lid_sensor_platform_data cardhu_lid_pdata = {
	.irq_gpio = TEGRA_GPIO_PS6,
};

static struct platform_device lid_device = {
	.name	= "lid-sensor",
	.id		= -1,
};

static void lid_init(void)
{
	pr_info("%s\n", __func__);
	lid_device.dev.platform_data = &cardhu_lid_pdata;
	platform_device_register(&lid_device);
}
#endif

int __init cardhu_sensors_init(void)
{
	int err;

	err = cardhu_nct1008_init();
	if (err)
		pr_err("%s: nct1008 init failed\n", __func__);
	else
		i2c_register_board_info(4, cardhu_i2c4_nct1008_board_info,
			ARRAY_SIZE(cardhu_i2c4_nct1008_board_info));

	cardhu_i2c1_board_info_al3010[0].irq = gpio_to_irq(TEGRA_GPIO_PZ2);
	i2c_register_board_info(2, cardhu_i2c1_board_info_al3010,
		ARRAY_SIZE(cardhu_i2c1_board_info_al3010));

	i2c_register_board_info(4, cardhu_i2c4_pad_bat_board_info,
			ARRAY_SIZE(cardhu_i2c4_pad_bat_board_info));

#ifdef CONFIG_INPUT_LID
	lid_init();
#endif

	mpuirq_init();

	return 0;
}
