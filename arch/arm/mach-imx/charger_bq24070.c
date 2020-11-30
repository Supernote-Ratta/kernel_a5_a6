/*
 * Copyright (C) 2018 Ratta, Inc. All Rights Reserved.
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

struct mxc_charger_data {
	int charge_en ; 
	int charge_ok ;	
	int charge_acok ;	
};

#if defined(CONFIG_OF)
static const struct of_device_id mxc_charger_dt_ids[] = {
	{ .compatible = "ti,bq24070-charger", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxc_charger_dt_ids);

#endif
static int mxc_charger_probe(struct platform_device *pdev)
{
	int rc, ret;
	struct mxc_charger_data *data;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;

	data = devm_kzalloc(dev, sizeof(struct mxc_charger_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->charge_en = of_get_named_gpio(np, "charge_en", 0);
	if (gpio_is_valid(data->charge_en)) {
		dev_dbg(&pdev->dev, "charge_en is:%d\n", data->charge_en);
		rc = devm_gpio_request_one(&pdev->dev,
								data->charge_en,
								GPIOF_OUT_INIT_HIGH,
								"Bcharger enable");
		if (rc) {
		//	dev_err(&pdev->dev, "unable to get bcm-power-gpios\n");
			goto error_request_gpio;
		}
	}

	if (gpio_is_valid(data->charge_en)) {
		gpio_set_value(data->charge_en, 0);
		gpio_export(data->charge_en,false); 	
		mdelay(500);
	}

	data->charge_ok = of_get_named_gpio(np, "charge_ok", 0);
	if (gpio_is_valid(data->charge_ok)) {
		dev_dbg(&pdev->dev, "charge_ok is:%d\n", data->charge_ok);
		rc = devm_gpio_request_one(&pdev->dev,
								data->charge_ok,
								GPIOF_IN,
								"charge ok state");
		if (rc) {
			goto error_request_gpio;
		}
	}
	if (gpio_is_valid(data->charge_ok)) {
		gpio_export(data->charge_ok,false); 
	}

	data->charge_acok = of_get_named_gpio(np, "charge_acok", 0);
	if (gpio_is_valid(data->charge_acok)) {
		dev_dbg(&pdev->dev, "charge_acok is:%d\n", data->charge_acok);
		rc = devm_gpio_request_one(&pdev->dev,
								data->charge_acok,
								GPIOF_IN,
								"charge acok state");
		if (rc) {
			goto error_request_gpio;
		}
	}
	if (gpio_is_valid(data->charge_acok)) {
		gpio_export(data->charge_acok,false); 
	}
		
	platform_set_drvdata(pdev,  data);
	dev_dbg(&pdev->dev, "mxc_charger success loaded\n");	
	return 0;

error_request_gpio:
	kfree(data);
	return rc;
}

static int  mxc_charger_remove(struct platform_device *pdev)
{
	struct mxc_charger_data *data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	kfree(data);
	return 0;
}

static struct platform_driver mxc_charger_driver = {
	.probe	= mxc_charger_probe,
	.remove	= mxc_charger_remove,
	.driver = {
		.name	= "mxc_charger_bq24070",
		.owner	= THIS_MODULE,
		.of_match_table = mxc_charger_dt_ids,
	},
};

module_platform_driver(mxc_charger_driver)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RATTA");
MODULE_DESCRIPTION("RATTA CHARGER BQ24070");
