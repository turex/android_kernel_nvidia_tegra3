/*
 * arch/arm/mach-tegra/board-transformer-camera.c
 *
 * Copyright (c) 2010-2012, NVIDIA CORPORATION, All rights reserved.
 * Copyright (C) 2019, Unlegacy Android Project
 * Copyright (c) 2019, Svyatoslav Ryhel
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

/*
 * This board file is based on original Asus board-cardhu-sensors.c file
 * from 3.1.10 kernel for Transformer Series. It's purpose is to contain
 * camera related code and prevent additional changes to sensors.
 *
 * All Tegra 3 Transformers have different camera sensors. Asus Transformer
 * Prime TF201 and Asus Transformer Infinity TF700T have Fujitsu M6MO ISP
 * sensor, while Asus Transformer Pad TF300T/TF300TG/TF300TL has iCatch 7002A
 * ISP sensor. This makes work with them a bit difficult. Altough there is
 * also a good news. They all use same front camera module Aptina mi1040
 * camera sensor same as grouper uses. Since Transformer ODM blobs are broken
 * anyway at least we can try to launch front camera using grouper blobs
 * and kernel driver.
 *
 * Code below contains initiation of all camera related gpios, for both front
 * and rear cameras, but only mi1040 sensor is registered.
 */

#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#ifdef CONFIG_VIDEO_MI1040
#include <media/yuv_sensor.h>
#endif

#include <mach/board-transformer-misc.h>

#include "board-transformer.h"

/* Camera related GPIOs on TF201 */
#define ISP_POWER_1V2_EN_GPIO         TEGRA_GPIO_PS3   //ISP_1V2_EN VDD_ISP_1V2
#define ISP_POWER_RESET_GPIO          TEGRA_GPIO_PBB0  //CAM_RST_5M, RSTX
#define CAM3_POWER_DWN_GPIO           TEGRA_GPIO_PBB7
#define FRONT_YUV_SENSOR_RST_GPIO     TEGRA_GPIO_PO0   //1.2M CAM_RST

/* Camera related GPIOs on TF700T */
#define TF700T_ISP_POWER_1V2_EN_GPIO  TEGRA_GPIO_PR7   //ISP_1V2_EN VDD_ISP_1V2
#define TF700T_ISP_POWER_1V8_EN_GPIO  TEGRA_GPIO_PBB7  //ISP_1V8_EN VDD_ISP_1V8

/* Camera related GPIOs on TF300T */
#define ICATCH7002A_VDDIO_EN_GPIO     TEGRA_GPIO_PBB4
#define ICATCH7002A_VDDC_EN_GPIO      TEGRA_GPIO_PBB7
#define ICATCH7002A_PWR_DN_GPIO       TEGRA_GPIO_PBB5
#define ICATCH7002A_RST_GPIO          TEGRA_GPIO_PBB0

#ifdef CONFIG_VIDEO_MI1040
static struct regulator *reg_transformer_1v8_cam;    /* VDDIO_CAM 1.8V PBB4 */
static struct regulator *reg_transformer_2v85_cam;   /* Front Camera 2.85V power */

static int transformer_camera_gpio_init(void)
{
	int ret;

	pr_info("%s", __func__);

	if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T){
		ret = gpio_request(TF700T_ISP_POWER_1V2_EN_GPIO, "isp_power_1v2_en");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "TF700T_ISP_POWER_1V2_EN_GPIO");

		ret = gpio_request(TF700T_ISP_POWER_1V8_EN_GPIO, "isp_power_1v8_en");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "TF700T_ISP_POWER_1V8_EN_GPIO");

		ret = gpio_request(ISP_POWER_RESET_GPIO, "isp_power_rstx");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "ISP_POWER_RESET_GPIO");
	} else if(tegra3_get_project_id() == TEGRA3_PROJECT_TF201){
		ret = gpio_request(ISP_POWER_1V2_EN_GPIO, "isp_power_1v2_en");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "ISP_POWER_1V2_EN_GPIO");

		ret = gpio_request(ISP_POWER_RESET_GPIO, "isp_power_rstx");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "ISP_POWER_RESET_GPIO");

		ret = gpio_request(CAM3_POWER_DWN_GPIO, "cam3_power_dwn");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "CAM3_POWER_DWN_GPIO");

		ret = gpio_request(FRONT_YUV_SENSOR_RST_GPIO, "yuv_sensor_rst_lo");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "FRONT_YUV_SENSOR_RST_GPIO");
	} else {
		ret = gpio_request(ICATCH7002A_VDDIO_EN_GPIO, "cam_vddio_ldo_en");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "ICATCH7002A_VDDIO_EN_GPIO");

		ret = gpio_request(ICATCH7002A_VDDC_EN_GPIO, "cam_vddc_ldo_en");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "ICATCH7002A_VDDC_EN_GPIO");

		ret = gpio_request(ICATCH7002A_PWR_DN_GPIO, "cam_power_dwn");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "ICATCH7002A_PWR_DN_GPIO");

		ret = gpio_request(ICATCH7002A_RST_GPIO, "cam_sensor_rst_lo");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "ICATCH7002A_RST_GPIO");
	}

	return 0;
}

static int transformer_mi1040_power_on(void)
{
	/* 1.8V VDDIO_CAM controlled by EN_1V8_CAM(GPIO_PBB4) */
	if (!reg_transformer_1v8_cam) {
		reg_transformer_1v8_cam = regulator_get(NULL, "vdd_1v8_cam1"); /* cam2/3? */
		if (IS_ERR_OR_NULL(reg_transformer_1v8_cam)) {
			reg_transformer_1v8_cam = NULL;
			pr_err("%s: couldn't get regulator vdd_1v8_cam1: %d\n",
				__func__, (int)PTR_ERR(reg_transformer_1v8_cam));
			goto fail_to_get_reg;
		}
		regulator_set_voltage(reg_transformer_1v8_cam, 1800000, 1800000);
		regulator_enable(reg_transformer_1v8_cam);
	}

	/* 2.85V VDD_CAM2 controlled by CAM2/3_LDO_EN(GPIO_PS0) */
	if (!reg_transformer_2v85_cam) {
		reg_transformer_2v85_cam = regulator_get(NULL, "vdd_cam3");
		if (IS_ERR_OR_NULL(reg_transformer_2v85_cam)) {
			reg_transformer_2v85_cam = NULL;
			pr_err("%s: couldn't get regulator vdd_cam3: %d\n",
				__func__, (int)PTR_ERR(reg_transformer_2v85_cam));
			goto fail_to_get_reg;
		}
		regulator_set_voltage(reg_transformer_2v85_cam, 2850000, 2850000);
		regulator_enable(reg_transformer_2v85_cam);
	}

	/* cam_power_en, powdn */
	gpio_set_value(CAM3_POWER_DWN_GPIO, 0);
	gpio_direction_output(CAM3_POWER_DWN_GPIO, 0);

	/* yuv_sensor_rst_lo */
	gpio_set_value(FRONT_YUV_SENSOR_RST_GPIO, 1);
	gpio_direction_output(FRONT_YUV_SENSOR_RST_GPIO, 1);

	return 0;

fail_to_get_reg:
	if (reg_transformer_2v85_cam) {
		regulator_put(reg_transformer_2v85_cam);
		reg_transformer_2v85_cam = NULL;
	}

	if (reg_transformer_1v8_cam) {
		regulator_put(reg_transformer_1v8_cam);
		reg_transformer_1v8_cam = NULL;
	}

	return -ENODEV;
}

static int transformer_mi1040_power_off(void)
{
	gpio_set_value(FRONT_YUV_SENSOR_RST_GPIO, 0);
	gpio_direction_output(FRONT_YUV_SENSOR_RST_GPIO, 0);

	gpio_set_value(CAM3_POWER_DWN_GPIO, 1);
	gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);

	if (reg_transformer_2v85_cam) {
		regulator_disable(reg_transformer_2v85_cam);
		regulator_put(reg_transformer_2v85_cam);
		reg_transformer_2v85_cam = NULL;
	}

	if (reg_transformer_1v8_cam) {
		regulator_disable(reg_transformer_1v8_cam);
		regulator_put(reg_transformer_1v8_cam);
		reg_transformer_1v8_cam = NULL;
	}

	return 0;
}

struct yuv_sensor_platform_data transformer_mi1040_data = {
	.power_on = transformer_mi1040_power_on,
	.power_off = transformer_mi1040_power_off,
};

static struct i2c_board_info transformer_i2c2_mi1040_board_info[] = {
	{
		I2C_BOARD_INFO("mi1040", 0x48),
		.platform_data = &transformer_mi1040_data,
	},
};

int __init cardhu_camera_init(void)
{
	transformer_camera_gpio_init();

	/* mi1040 front camera */
	pr_info("%s: mi1040 i2c_register_board_info", __func__);
	i2c_register_board_info(2, transformer_i2c2_mi1040_board_info,
		ARRAY_SIZE(transformer_i2c2_mi1040_board_info));

	return 0;
}
#else
int __init cardhu_camera_init(void) { return 0; }
#endif /* CONFIG_VIDEO_MI1040 */
