/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file mx6_bcm_ap6212a.c
 *
 * @brief This driver is implement a rfkill control interface of bluetooth
 * chip on i.MX serial boards. Register the power regulator function and
 * reset function in platform data.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>

struct mxc_bcm_ap6212a_data {
	int bcm_power_gpio;
};

#if defined(CONFIG_OF)
static const struct of_device_id mxc_bcm_ap6212a_dt_ids[] = {
	{ .compatible = "fsl,bcm_ap6212a", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxc_bcm_ap6212a_dt_ids);

#endif
static int mxc_bcm_ap6212a_probe(struct platform_device *pdev)
{
	int rc, ret;
	struct mxc_bcm_ap6212a_data *data;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;

	data = devm_kzalloc(dev, sizeof(struct mxc_bcm_ap6212a_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->bcm_power_gpio = of_get_named_gpio(np, "bcm-power-gpios", 0);
	if (gpio_is_valid(data->bcm_power_gpio)) {
		printk(KERN_INFO "bcm power gpio is:%d\n", data->bcm_power_gpio);
		rc = devm_gpio_request_one(&pdev->dev,
								data->bcm_power_gpio,
								GPIOF_OUT_INIT_LOW,
								"BCM power enable");
		if (rc) {
			dev_err(&pdev->dev, "unable to get bcm-power-gpios\n");
			goto error_request_gpio;
		}
	}

	if (gpio_is_valid(data->bcm_power_gpio)) {
		gpio_set_value(data->bcm_power_gpio, 1);
		mdelay(500);
	}

	platform_set_drvdata(pdev,  data);
	printk(KERN_INFO "mxc_bcm_ap6212a driver success loaded\n");
	return 0;

error_request_gpio:
	kfree(data);
	return rc;
}

static int  mxc_bcm_ap6212a_remove(struct platform_device *pdev)
{
	struct mxc_bcm_ap6212a_data *data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	kfree(data);
	return 0;
}

static struct platform_driver mxc_bcm_ap6212a_driver = {
	.probe	= mxc_bcm_ap6212a_probe,
	.remove	= mxc_bcm_ap6212a_remove,
	.driver = {
		.name	= "mxc_bcm_ap6212a",
		.owner	= THIS_MODULE,
		.of_match_table = mxc_bcm_ap6212a_dt_ids,
	},
};

module_platform_driver(mxc_bcm_ap6212a_driver)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("bcm ap6212a on MX6 Platform");
