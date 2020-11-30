/*
 *hall sensor
 *
 * Copyright (c) 2017 Ratta.
 * <
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */
#define HALL_DEBUG

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#define DEVICE_NAME             "hall_sensor"
#define CONVERSION_TIME_MS		50

struct hall_sensor  {
	struct input_dev *input;
	int irq;
	int gpio_hall_sensor;
};

#ifdef HALL_DEBUG
#  define dprintk(fmt, args...) printk(KERN_DEBUG DEVICE_NAME ": " fmt , ## args)
#else
#  define dprintk(fmt, args...)
#endif
extern char powerdown_flag; 
char hall_suspend_flag=0;
static irqreturn_t hall_sensor_irq(int irq, void *dev_id)
{
	struct hall_sensor *hall_data = dev_id;
	int value = 0;
	printk("wym hall_sensor_irq\n");
	if(hall_suspend_flag==0x55)
		powerdown_flag=0xbb;
	msleep(CONVERSION_TIME_MS);
	value = gpio_get_value(hall_data->gpio_hall_sensor) ? 1 : 0;
	input_report_switch(hall_data->input, SW_LID, !value);
	input_sync(hall_data->input);
	dprintk("hall state: %d\n", value);
	printk("wym hall_sensor_irq 1111\n");
	
	return IRQ_HANDLED;
}

static int hall_sensor_open(struct input_dev *dev)
{
	//struct hall_sensor *hall_sensor = input_get_drvdata(dev);
	//printk("hall_sensor_open...........................\n");

	//enable_irq(hall_sensor->irq);
	
	return 0;
}

static void hall_sensor_close(struct input_dev *dev)
{
	//struct hall_sensor *hall_sensor = input_get_drvdata(dev);
	//printk("hall_sensor_close...........................\n");

	//disable_irq(hall_sensor->irq);
}

static int hall_sensor_probe(struct platform_device *pdev)
{
	struct hall_sensor *hall_sensor;
	struct input_dev *input;
	int error = 0;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;

	int gpio_hall_sensor = 0;	
	
	dprintk("%s go1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", __FUNCTION__);
			
	if (!np)
		return -ENODEV;

	gpio_hall_sensor = of_get_named_gpio(np, "gpio_hall_sensor", 0);							
	dprintk("gpio_hall_sensor is %x\n", gpio_hall_sensor);
			
	if (!gpio_is_valid(gpio_hall_sensor))
		return -ENODEV;

				
	dprintk("%s go2!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", __FUNCTION__);
	
	error = devm_gpio_request_one(dev, gpio_hall_sensor,
			GPIOF_IN, "gpio_hall_sensor");
	if (error < 0) {
		dev_err(dev,
			"request gpio failed: %d\n", error);
		return error;
	}
					
	/*	Allocate memory	*/
	hall_sensor = kzalloc(sizeof(struct hall_sensor), GFP_KERNEL);
	input = input_allocate_device();
	if (!hall_sensor || !input) {
		dev_err(dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}


	/*	Save the stuctures to be used in the driver	*/
	hall_sensor->input = input;
	hall_sensor->gpio_hall_sensor = gpio_hall_sensor;

	input->name = "Hall Sensor";
	input->id.bustype = BUS_HOST;
	input->dev.parent = dev;
	input->open = hall_sensor_open;
	input->close = hall_sensor_close;

#if 0
	set_bit(EV_SW, input->evbit);
	set_bit(SW_LID, input->swbit);
#else
	input_set_capability(input, EV_SW, SW_LID);
#endif

	/*	Request an interrup on the RDY line	*/
	hall_sensor->irq = gpio_to_irq(gpio_hall_sensor);
	dprintk("to_irq %d\n", hall_sensor->irq);
	
	/* Set driver data */
	input_set_drvdata(input,  hall_sensor);

	/*
	 *	Request the interrupt on a falling trigger
	 *	and only one trigger per falling edge
	 */
	error = request_threaded_irq(hall_sensor->irq, NULL, hall_sensor_irq,
				     IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
				     "hall_sensor", hall_sensor);
	if (error) {
		dev_err(dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_mem;
	}
	
	error = input_register_device(hall_sensor->input);
	if (error) {
		dev_err(dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_irq;
	}

	platform_set_drvdata(pdev,  hall_sensor);
	device_init_wakeup(&pdev->dev, 1);
	enable_irq_wake(hall_sensor->irq);
	printk("wym hall irq:%d\n",hall_sensor->irq);
	return 0;
	
err_free_irq:
	free_irq(hall_sensor->irq, hall_sensor);
err_free_mem:
	input_free_device(input);
	kfree(hall_sensor);

	return error;
}

static int hall_sensor_remove(struct platform_device *pdev)
{
	struct hall_sensor *hall_sensor = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);
	platform_set_drvdata(pdev, NULL);

	free_irq(hall_sensor->irq, hall_sensor);
	input_unregister_device(hall_sensor->input);
	kfree(hall_sensor);	
	return 0;
}

#define  GPIO4BASE   0X20A8000
#define GPIO4_ICR1   0x0c
#define GPIO4_ICR2   0x10
#define  GPIO4_EDGE_SEL  0x1c
#define  IOMUXC_SW_PAD_CTL_PAD_FEC_TX_CLK  0x20E0434
#define GPIO4MASK   0X14    //gpio4 intr mask,wym added 20180806
int gpio4mask=0;
int   gpio4_icr1;
int   gpio4_icr2;
int   gpio4_edgesel;
int  close_gpio_intr(void)
{
	
	static void *p_gpj0_base;
	int datalong;
	int dataa;
	datalong=8;
	
	p_gpj0_base = ioremap(GPIO4BASE, datalong*4);
	if (!p_gpj0_base)
		return -ENOMEM;
	gpio4mask=readl(p_gpj0_base+GPIO4MASK);
	printk("WYM GPIO4MASK111-%x\n",gpio4mask);
	writel(0x00a00000,p_gpj0_base+GPIO4MASK);	 //low level interrupt
	iounmap(p_gpj0_base);

	
//	gpio4_icr1=readl(p_gpj0_base+GPIO4_ICR1);
//	printk("WYM GPIO4MASK111-%x\n",gpio4_icr1);
//	writel(0,p_gpj0_base+GPIO4_ICR1);	 //low level interrupt

/*	
	gpio4_icr2=readl(p_gpj0_base+GPIO4_ICR2);
	printk("WYM GPIO4MASK111-%x\n",gpio4_icr2);
	writel(gpio4_icr2&0xffff3fff,p_gpj0_base+GPIO4_ICR2);	 //low level interrupt

	gpio4_edgesel=readl(p_gpj0_base+GPIO4_EDGE_SEL);
	printk("WYM GPIO4MASK111-%x\n",gpio4_edgesel);
	writel(gpio4_edgesel&0xff7fffff,p_gpj0_base+GPIO4_EDGE_SEL);	 //low level interrupt

	p_gpj0_base = ioremap(IOMUXC_SW_PAD_CTL_PAD_FEC_TX_CLK, 4);
	dataa=readl(p_gpj0_base);
	printk("WYM IOMUXC_SW_PAD_CTL_PAD_FEC_TX_CLK-%x\n",dataa);
	*/
}

int restor_gpio_intr(void)
{
	
	static void *p_gpj0_base;
	int datalong;
	datalong=8;
	
	p_gpj0_base = ioremap(GPIO4BASE, datalong*4);
	if (!p_gpj0_base)
		return -ENOMEM;
//	gpio4mask=readl(p_gpj0_base);
//		  printk("WYM GPIO4MASK-%x\n",gpio4mask);
	writel(gpio4mask,p_gpj0_base+GPIO4MASK);	 //low level interrupt
//	writel(gpio4_icr1,p_gpj0_base+GPIO4_ICR1);	 //low level interrupt
//	writel(gpio4_icr2,p_gpj0_base+GPIO4_ICR2);	 //low level interrupt
//	writel(gpio4_edgesel,p_gpj0_base+GPIO4_EDGE_SEL);	 //low level interrupt
	iounmap(p_gpj0_base);
}

static int __maybe_unused hall_sensor_suspend(struct device *dev)
{
//	struct platform_device *platform_dev = to_platform_device(dev);

//	struct hall_sensor *hall_sensor = platform_get_drvdata(platform_dev);

	//disable_irq(hall_sensor->irq);
	close_gpio_intr();
	hall_suspend_flag=0x55;

	return 0;
}

static int __maybe_unused hall_sensor_resume(struct device *dev)
{
//	struct platform_device *platform_dev = to_platform_device(dev);

//	struct hall_sensor *hall_sensor = platform_get_drvdata(platform_dev);

	//enable_irq(hall_sensor->irq);
	restor_gpio_intr();
	hall_suspend_flag=0;

	return 0;
}

static SIMPLE_DEV_PM_OPS(hall_sensor_pm, hall_sensor_suspend, hall_sensor_resume);

static const struct of_device_id hall_dt_ids[] = {
	{
		.compatible = "sn100,hall_sensor",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, hall_dt_ids);

static struct platform_driver hall_sensor_driver = {
	.probe	= hall_sensor_probe,
	.remove	= hall_sensor_remove,
	.driver = {
		.name	= "hall_sensor",
		.owner	= THIS_MODULE,		
		.pm	= &hall_sensor_pm,
		.of_match_table = hall_dt_ids,
	},
};

module_platform_driver(hall_sensor_driver);

MODULE_AUTHOR("Ratta");
MODULE_DESCRIPTION("Hall Sensor Driver");
MODULE_LICENSE("GPL");
