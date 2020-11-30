/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx6_bt_rfkill.c
 *
 * @brief This driver is implement a rfkill control interface of bluetooth
 * chip on i.MX serial boards. Register the power regulator function and
 * reset function in platform data.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/rfkill.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

static int system_in_suspend;

struct mxc_bt_rfkill_data {
	int bt_reset_gpio;
	int bt_ext_wake_gpio;
	struct rfkill *rfk;
};

#define  GPIO_BCM_WL_POWER_ON	((3-1)*32+29)
static int mxc_bt_set_block(void *rfk_data, bool blocked)
{
	struct mxc_bt_rfkill_data *data = rfk_data;
	int ret;

	/* Bluetooth stack will reset the bluetooth chip during
	 * resume, since we keep bluetooth's power during suspend,
	 * don't let rfkill to actually reset the chip. */
	if (system_in_suspend)
		return 0;

	if (!blocked)
	{
		printk("mxc_bt_power... on\n");
		gpio_set_value(GPIO_BCM_WL_POWER_ON, 1);
		gpio_set_value(data->bt_reset_gpio, 0);
		msleep(200);
		gpio_set_value(data->bt_reset_gpio, 1);
		msleep(20);
	}
	else
	{
		printk("mxc_bt_power...  off\n");
	#if 1
//		gpio_set_value(data->bt_ext_wake_gpio, 1);
		gpio_set_value(data->bt_reset_gpio, 0);
		msleep(20);
//		gpio_set_value(GPIO_BCM_WL_POWER_ON, 0);
//		msleep(20);
	#endif
	}
	return ret;
}

static const struct rfkill_ops mxc_bt_rfkill_ops = {
	.set_block = mxc_bt_set_block,
};

static int mxc_bt_power_event(struct notifier_block *this,
			      unsigned long event, void *dummy)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		system_in_suspend = 1;
		/* going to suspend, don't reset chip */
		break;
	case PM_POST_SUSPEND:
		system_in_suspend = 0;
		/* System is resume, can reset chip */
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block mxc_bt_power_notifier = {
	.notifier_call = mxc_bt_power_event,
};

static int mxc_bt_rfkill_probe(struct platform_device *pdev)
{
	int rc;
	
	struct mxc_bt_rfkill_data *data;
	struct device_node *np = pdev->dev.of_node;

	printk("mxc_bt_rfkill_probe\n");
	data = devm_kzalloc(&pdev->dev, sizeof(struct mxc_bt_rfkill_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	rc = register_pm_notifier(&mxc_bt_power_notifier);
	if (rc)
		goto error_check_func;

	data->rfk = rfkill_alloc("ratta-bluetooth", &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			   &mxc_bt_rfkill_ops, data);

	if (!data->rfk) {
		rc = -ENOMEM;
		goto error_rfk_alloc;
	}

	rc = rfkill_register(data->rfk);
	if (rc)
		goto error_rfkill;

	data->bt_reset_gpio = of_get_named_gpio(np, "bt-reset-gpios", 0);
	if (gpio_is_valid(data->bt_reset_gpio)) {
		printk(KERN_INFO "bt reset gpio is:%d\n", data->bt_reset_gpio);
		rc = devm_gpio_request_one(&pdev->dev,
								data->bt_reset_gpio,
								GPIOF_OUT_INIT_LOW,
								"BT reset enable");
		if (rc) {
			dev_err(&pdev->dev, "unable to get bt-reset-gpios\n");
			goto error_rfkill;
		}
	}
	gpio_export(data->bt_reset_gpio, false); 

	data->bt_ext_wake_gpio = of_get_named_gpio(np, "bt-ext-wake-gpios", 0);
	if (gpio_is_valid(data->bt_ext_wake_gpio)) {
		printk(KERN_INFO "bt ext wake gpio is:%d\n", data->bt_ext_wake_gpio);
		rc = devm_gpio_request_one(&pdev->dev,
								data->bt_ext_wake_gpio,
								GPIOF_OUT_INIT_LOW,
								"BT reset enable");
		if (rc) {
			dev_err(&pdev->dev, "unable to get bt-reset-gpios\n");
			goto error_rfkill;
		}
	}

	platform_set_drvdata(pdev, data);
	printk(KERN_INFO "mxc_bt_rfkill driver success loaded\n");
	return 0;

error_rfkill:
	rfkill_destroy(data->rfk);
error_rfk_alloc:
error_check_func:
	return rc;
}

static int  mxc_bt_rfkill_remove(struct platform_device *pdev)
{

	struct mxc_bt_rfkill_data *data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (data->rfk) {
		rfkill_unregister(data->rfk);
		rfkill_destroy(data->rfk);
	}

	kfree(data);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id mxc_bt_rfkill_dt_ids[] = {
	{ .compatible = "fsl,mxc_bt_rfkill", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mxc_bt_rfkill_dt_ids);
#endif


static struct platform_driver mxc_bt_rfkill_driver = {
	.driver = {
		.name = "mxc_bt_rfkill",
		.owner	= THIS_MODULE,
		.of_match_table = mxc_bt_rfkill_dt_ids,			
	},
	.probe	= mxc_bt_rfkill_probe,
	.remove = mxc_bt_rfkill_remove,
};

static int __init mxc_bt_rfkill_init(void)
{
	return platform_driver_register(&mxc_bt_rfkill_driver);
}

module_init(mxc_bt_rfkill_init);

static void __exit mxc_bt_rfkill_exit(void)
{
	platform_driver_unregister(&mxc_bt_rfkill_driver);
}

module_exit(mxc_bt_rfkill_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("RFKill control interface of BT on MX6 Platform");
