/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011 - 2013 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */
//#define DEBUG           /* Enable initcall_debug */ 


#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_SPI_RATTA_EPA
#include <linux/spiepa.h>
#endif
#include <linux/mxc_epdc_helper.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif //CONFIG_HAS_EARLYSUSPEND
#define WACOM_CMD_QUERY0	0x04
#define WACOM_CMD_QUERY1	0x00
#define WACOM_CMD_QUERY2	0x33
#define WACOM_CMD_QUERY3	0x02
#define WACOM_CMD_THROW0	0x05
#define WACOM_CMD_THROW1	0x00
#define WACOM_QUERY_SIZE	21
#define WACOM_REGULAR_INPUT     10
#define AA_OFFSET               0

#define HID_DESC_REG            0x01

#define KERNEL_OLDER33          0

#define RESET 1

#define WACOM_EMR_103		0x59
#define WACOM_EMR_103_FW96	0x48
#define WACOM_EMR_078		0x4A
#define WACOM_EMR_078_RATTA	0x9A
#define RATTA_PROC_SUPERNOTE   "supernote"
#define WACOM_PEN_G12           12
#define WACOM_PEN_G14           14

struct feature_support {
	bool height;
	bool tilt;
};

typedef struct hid_descriptor {
	u16 wHIDDescLength;
	u16 bcdVersion;
	u16 wReportDescLength;
	u16 wReportDescRegister;
	u16 wInputRegister;
	u16 wMaxInputLength;
	u16 wOutputRegister;
	u16 wMaxOutputLength;
	u16 wCommandRegister;
	u16 wDataRegister;
	u16 wVendorID;
	u16 wProductID;
	u16 wVersion;
	u16 RESERVED_HIGH;
	u16 RESERVED_LOW;
} HID_DESC;

struct wacom_features {
	struct feature_support support;
	int x_max;
	int y_max;
	int pressure_max;
	int height_max;
	int tilt_x_max;
	int tilt_y_max;
	unsigned int fw_version;
	HID_DESC hid_desc;
};

struct wacom_i2c {
	struct i2c_client *client;
	struct input_dev *input;
	struct wacom_features features;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	u8 data[WACOM_QUERY_SIZE];
	u8 pre_data[WACOM_QUERY_SIZE];
	bool prox;
	int tool;	
	int gpio_wacom_rst;
	int gpio_wacom_pwr_en;
#ifdef	CONFIG_RATTA_GET_WACOM_PEN_TYPE
	int emr_pen_type;
	struct mutex mlock;
#endif
};

static int wacom_query_device(struct i2c_client *client,
			      struct wacom_features *features)
{
	int ret;
	u8 cmd_hid_desc[] = {HID_DESC_REG, 0x00};
	u8 cmd1[] = { WACOM_CMD_QUERY0, WACOM_CMD_QUERY1,
			WACOM_CMD_QUERY2, WACOM_CMD_QUERY3 };
	u8 cmd2[] = { WACOM_CMD_THROW0, WACOM_CMD_THROW1 };
	u8 data[WACOM_QUERY_SIZE];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd1),
			.buf = cmd1,
		},
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd2),
			.buf = cmd2,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(data),
			.buf = data,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	features->x_max = get_unaligned_le16(&data[3]);
	features->y_max = get_unaligned_le16(&data[5]);
	features->pressure_max = get_unaligned_le16(&data[11]);
	features->fw_version = get_unaligned_le16(&data[13]);
	features->height_max = data[15];
	features->tilt_x_max = get_unaligned_le16(&data[17]);
	features->tilt_y_max =  get_unaligned_le16(&data[19]);

	if (features->height_max)
		features->support.height = true;
	else
		features->support.height = false;

	if (features->tilt_x_max && features->tilt_y_max)
		features->support.tilt = true;
	else
		features->support.tilt = false;

	ret = i2c_master_send(client, cmd_hid_desc, sizeof(cmd_hid_desc));
	if (ret < 0) {
		dev_err(&client->dev, "cannot send register info\n");
		return ret;
	}

	ret = i2c_master_recv(client, (char *)&features->hid_desc, sizeof(HID_DESC));
	if (ret < 0) {
		dev_err(&client->dev, "cannot receive register info\n");
		return ret;
	}

	dev_dbg(&client->dev, "input report length %d product_id:0x%x\n",
		features->hid_desc.wMaxInputLength,features->hid_desc.wProductID);
	dev_dbg(&client->dev, "x_max:%d, y_max:%d, pressure:%d, fw:%d\n",
		features->x_max, features->y_max,
		features->pressure_max, features->fw_version);
	dev_dbg(&client->dev, "height:%d, tilt_x:%d, tilt_y:%d\n",
		features->height_max, features->tilt_x_max,
		features->tilt_y_max);

	return 0;
}

/*Eliminating the digitizer AA offset; this makes the coordination exactly fit to the LCD size*/
static void set_offset(int *x, int *y, int x_max, int y_max)
{
	int temp_coord = *x - AA_OFFSET;
	
	if (temp_coord < 0)
		*x = 0;
	else if (temp_coord > x_max)
		*x = x_max;
	else
		*x = temp_coord;
	
	temp_coord = *y - AA_OFFSET;
	
	if (temp_coord < 0)
		*y = 0;
	else if (temp_coord > y_max)
		*y = y_max;
	else
		*y = temp_coord;
}

static irqreturn_t wacom_i2c_irq_sn078(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	u8 *data = wac_i2c->data;
	unsigned int x, y, pressure;
	int tilt_x, tilt_y;
	unsigned char tsw, f1, f2, ers;
	int data_len = wac_i2c->features.hid_desc.wMaxInputLength;
	int error, ret;
	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, data_len);
	if (error < 0)
		goto out;
	tsw = data[3] & 0x01;  /* tip switch */
	ers = data[3] & 0x04;  /* eraser */
	f1 = data[3] & 0x02;   /* side switch */
	f2 = data[3] & 0x10;   /* 2nd side switch */
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);
	if ( pressure < 110 )
		pressure = 0;

	set_offset(&x, &y, wac_i2c->features.x_max, wac_i2c->features.y_max);

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & 0x20;

#ifdef CONFIG_RATTA_GET_WACOM_PEN_TYPE
	/*
	 *  g14 emr Supported pen series
	 *     1.    CP Pen           //data[2] == 2
	 *     2.    Shinonome Refill //data[2] == 26 
	 */
	if(unlikely( data[2] == 2)) { //G12 pen
		if (wac_i2c->emr_pen_type != WACOM_PEN_G12) {
			mutex_lock(&wac_i2c->mlock);
			wac_i2c->emr_pen_type = WACOM_PEN_G12;
			mutex_unlock(&wac_i2c->mlock);
		}
		input_report_key(input, KEY_F14 , 0 );
	} else {
		if (wac_i2c->emr_pen_type != WACOM_PEN_G14) {
			mutex_lock(&wac_i2c->mlock);
			wac_i2c->emr_pen_type = WACOM_PEN_G14;
			mutex_unlock(&wac_i2c->mlock);
		}
		input_report_key(input, KEY_F14 , 1 );
	}
#endif
#ifdef CONFIG_SPI_RATTA_EPA
	spiepa_event_enqueue(x, y, pressure,
			     (wac_i2c->tool == BTN_TOOL_PEN) ?
			     SPIEPA_TOOL_PEN :
			     SPIEPA_TOOL_RUBBER,
			     data[2]);
#endif
	epdc_helper_add_point(x, y, pressure,
			      wac_i2c->tool, data[2], !!f1, !!f2);

	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_PRESSURE, pressure);

	if (data_len > WACOM_REGULAR_INPUT) {
		if (wac_i2c->features.support.height)
			input_report_abs(input, ABS_DISTANCE, data[10]);

		if (wac_i2c->features.support.tilt) {
			tilt_x = (int)le16_to_cpup((__le16 *)&data[11]);
			tilt_y = (int)le16_to_cpup((__le16 *)&data[13]);
			input_report_abs(input, ABS_TILT_X, tilt_x);
			input_report_abs(input, ABS_TILT_Y, tilt_y);
		}
	}

	input_sync(input);

out:
	return IRQ_HANDLED;
}

static irqreturn_t wacom_i2c_irq_sn100(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	u8 *data = wac_i2c->data;
	unsigned int x, y, pressure;
	int tilt_x, tilt_y;
	unsigned char tsw, f1, f2, ers;
	int data_len = wac_i2c->features.hid_desc.wMaxInputLength;
	int error, ret;
	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, data_len);
	if (error < 0)
		goto out;
	tsw = data[3] & 0x01;  /* tip switch */
	ers = data[3] & 0x04;  /* eraser */
	f1 = data[3] & 0x02;   /* side switch */
	f2 = data[3] & 0x10;   /* 2nd side switch */
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);

	// x y cursor rotation 180 to app
	x =  wac_i2c->features.x_max - x;
	y =  wac_i2c->features.y_max -y;
	pressure = le16_to_cpup((__le16 *)&data[8]);
	if ( pressure < 110 )
		pressure = 0;

	set_offset(&x, &y, wac_i2c->features.x_max, wac_i2c->features.y_max);

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & 0x20;

#ifdef CONFIG_RATTA_GET_WACOM_PEN_TYPE
	/*
	 *  g14 emr Supported pen series
	 *     1.    CP Pen           //data[2] == 2
	 *     2.    Shinonome Refill //data[2] == 26 
	 */
	if(unlikely( data[2] == 2)) { //G12 pen
		if (wac_i2c->emr_pen_type != WACOM_PEN_G12) {
			mutex_lock(&wac_i2c->mlock);
			wac_i2c->emr_pen_type = WACOM_PEN_G12;
			mutex_unlock(&wac_i2c->mlock);
		}
		input_report_key(input, KEY_F14 , 0 );
	} else {
		if (wac_i2c->emr_pen_type != WACOM_PEN_G14) {
			mutex_lock(&wac_i2c->mlock);
			wac_i2c->emr_pen_type = WACOM_PEN_G14;
			mutex_unlock(&wac_i2c->mlock);
		}
		input_report_key(input, KEY_F14 , 1 );
	}
#endif
#ifdef CONFIG_SPI_RATTA_EPA
	spiepa_event_enqueue(x, y, pressure,
			     (wac_i2c->tool == BTN_TOOL_PEN) ?
			     SPIEPA_TOOL_PEN :
			     SPIEPA_TOOL_RUBBER,
			     data[2]);
#endif
	epdc_helper_add_point(x, y, pressure,
			      wac_i2c->tool, data[2], !!f1, !!f2);

	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_PRESSURE, pressure);

	if (data_len > WACOM_REGULAR_INPUT) {
		if (wac_i2c->features.support.height)
			input_report_abs(input, ABS_DISTANCE, data[10]);

		if (wac_i2c->features.support.tilt) {
			tilt_x = (int)le16_to_cpup((__le16 *)&data[11]);
			tilt_y = (int)le16_to_cpup((__le16 *)&data[13]);
			input_report_abs(input, ABS_TILT_X, tilt_x);
			input_report_abs(input, ABS_TILT_Y, tilt_y);
		}
	}

	input_sync(input);

out:
	return IRQ_HANDLED;
}

static int wacom_i2c_open(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	enable_irq(client->irq);

	return 0;
}

static void wacom_i2c_close(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	disable_irq(client->irq);
}

static unsigned char *g_ratta_mtype = "ratta-other\n";
int get_hardware_from_emr(void)
{
	if (!strncmp(g_ratta_mtype,
		     "ratta-sn100",
		     strlen("ratta-sn100")))
		return 100;
	else if (!strncmp(g_ratta_mtype,
			  "ratta-sn078",
			  strlen("ratta-sn078")))
		return 78;

	return -1;
}
EXPORT_SYMBOL(get_hardware_from_emr);

static supernote_mtype_read (struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	return simple_read_from_buffer(buf, count, ppos,g_ratta_mtype,
				       strlen(g_ratta_mtype)); 
}

static const struct file_operations sn_mtype_file_ops = {      
	    .owner = THIS_MODULE,
	    .read  = supernote_mtype_read,
};

static ssize_t wacom_get_version_pid_show(struct device *kdev, struct device_attribute *attr, char *buf)  
{
	struct i2c_client *client = to_i2c_client(kdev);
        struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	return sprintf(buf,"%04X.%04X\n",wac_i2c->input->id.product,wac_i2c->features.fw_version);
}
static DEVICE_ATTR(version, S_IRUGO, wacom_get_version_pid_show, NULL);

#ifdef CONFIG_RATTA_GET_WACOM_PEN_TYPE
static ssize_t wacom_get_emr_pen_type_show(struct device *kdev, struct device_attribute *attr, char *buf)  
{
	struct i2c_client *client = to_i2c_client(kdev);
        struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	int cur_pen;

	mutex_lock(&wac_i2c->mlock);
	cur_pen = (wac_i2c->emr_pen_type == WACOM_PEN_G12) ?
		       WACOM_PEN_G12 : WACOM_PEN_G14;
	mutex_unlock(&wac_i2c->mlock);
	return sprintf(buf,"G%02d\n", cur_pen);
}
static DEVICE_ATTR(emr_pen_type, S_IRUGO, wacom_get_emr_pen_type_show, NULL);
#endif

static struct attribute *wacom_attrs[] = {
	&dev_attr_version.attr,
#ifdef CONFIG_RATTA_GET_WACOM_PEN_TYPE
	&dev_attr_emr_pen_type.attr,
#endif
	NULL
};

#ifdef CONFIG_SPI_RATTA_EPA
static __u32 emr_width = 0;
static struct property emr_width_prop = {
	.name = SPIEPA_EMR_WIDTH_PROPERTY,
	.length = sizeof(emr_width),
	.value = &emr_width,
};

static __u32 emr_height = 0;
static struct property emr_height_prop = {
	.name = SPIEPA_EMR_HEIGHT_PROPERTY,
	.length = sizeof(emr_height),
	.value = &emr_height,
};
#endif

static int wacom_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct wacom_i2c *wac_i2c;
	struct input_dev *input;
	struct wacom_features features = { 0 };
	int error = 0;

	struct device_node *np = client->dev.of_node;
#ifdef CONFIG_SPI_RATTA_EPA
	struct device_node *epa_np;
#endif
	int gpio_wacom_rst;
	int gpio_wacom_pwr_en;
	irq_handler_t  wacom_i2c_irq = wacom_i2c_irq_sn078;
	dev_dbg(&client->dev,
		"Wacom I2C probe called for i2c 0x%02x\n", client->addr);
#if 1
	if (!np)
		return -ENODEV;

#ifdef CONFIG_SPI_RATTA_EPA
	epa_np = of_parse_phandle(np, "ratta-epa", 0);
	if (!epa_np) {
		return -EPROBE_DEFER;
	}
#endif

	gpio_wacom_rst = of_get_named_gpio(np, "gpio_wacom_rst", 0);
	dev_dbg(&client->dev,
		"Wacom I2C probe gpio_wacom_rst is %x\n", gpio_wacom_rst);

	if (!gpio_is_valid(gpio_wacom_rst))
		return -ENODEV;

	gpio_wacom_pwr_en = of_get_named_gpio(np, "gpio_wacom_pwr_en", 0);
	dev_dbg(&client->dev,
		"Wacom I2C probe gpio_wacom_pwr_en is %x\n", gpio_wacom_pwr_en);

	if (!gpio_is_valid(gpio_wacom_pwr_en))
		return -ENODEV;

	error = devm_gpio_request_one(&client->dev, gpio_wacom_rst,
			GPIOF_OUT_INIT_HIGH, "gpio_wacom_rst");
	if (error < 0) {
		dev_err(&client->dev,
			"request gpio failed: %d\n", error);
		return error;
	}

	error = devm_gpio_request_one(&client->dev, gpio_wacom_pwr_en,
				GPIOF_OUT_INIT_HIGH, "gpio_wacom_pwr_en");
	if (error < 0) {
		dev_err(&client->dev,
			"request gpio failed: %d\n", error);
		return error;
	}
	msleep(100);

	gpio_set_value(gpio_wacom_rst, 0);
	msleep(150);
	gpio_set_value(gpio_wacom_rst, 1);
	msleep(50);

#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	error = wacom_query_device(client, &features);
	if (error)
	{
		dev_dbg(&client->dev,
			"%s error: %x!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", __FUNCTION__, error);
		return error;
	}
	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	input = input_allocate_device();
	if (!wac_i2c || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	wac_i2c->client = client;
	wac_i2c->input = input;
	wac_i2c->gpio_wacom_rst = gpio_wacom_rst;
	wac_i2c->gpio_wacom_pwr_en = gpio_wacom_pwr_en;

	input->name = "Wacom I2C Digitizer";
	input->id.bustype = BUS_I2C;
	input->id.vendor = features.hid_desc.wVendorID;
	input->id.product = features.hid_desc.wProductID;
	input->id.version = features.hid_desc.wVersion;
	input->dev.parent = &client->dev;
	input->open = wacom_i2c_open;
	input->close = wacom_i2c_close;
	input->hint_events_per_packet=64;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);

#ifdef CONFIG_RATTA_GET_WACOM_PEN_TYPE
	__set_bit(KEY_F14, input->keybit);
#endif
	/*Setting maximum coordinate values  */
	/*eliminating 1mm offset on each side*/
	features.x_max -= (AA_OFFSET * 2);
	features.y_max -= (AA_OFFSET * 2);
	wac_i2c->features = features;
	dev_dbg(&client->dev,"feature_xmax: %d feature_ymax: %d \n", wac_i2c->features.x_max, wac_i2c->features.y_max);

	input_set_abs_params(input, ABS_X, 0, features.x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, features.y_max, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE,
			     0, features.pressure_max, 0, 0);
	epdc_helper_setup_emr(features.x_max, features.y_max);
#ifdef CONFIG_SPI_RATTA_EPA
	emr_width = cpu_to_be32(features.x_max);
	emr_height = cpu_to_be32(features.y_max);
	of_add_property(epa_np, &emr_width_prop);
	of_add_property(epa_np, &emr_height_prop);
#endif

	if (features.support.height)
		input_set_abs_params(input, ABS_DISTANCE, 0, features.height_max, 0, 0);
	if (features.support.tilt) {
		input_set_abs_params(input, ABS_TILT_X, 
				     -features.tilt_x_max, features.tilt_x_max, 0, 0);
		input_set_abs_params(input, ABS_TILT_Y, 
				     -features.tilt_y_max, features.tilt_y_max, 0, 0);
	}
#ifdef CONFIG_RATTA_GET_WACOM_PEN_TYPE
	mutex_init(&wac_i2c->mlock);
#endif
	input_set_drvdata(input, wac_i2c);

	struct proc_dir_entry *dir_sn, *dir_mtype;
	dir_sn = proc_mkdir(RATTA_PROC_SUPERNOTE, NULL);
	if (!dir_sn) {
		dev_err(&client->dev," creat supernote proc error \n");
	}
	else	{
	dir_mtype= proc_create_data("machine_tpye", 0444,dir_sn,&sn_mtype_file_ops,NULL);
        if (!dir_mtype) {
		dev_err(&client->dev," creat supernote proc error \n");
		proc_remove(dir_sn);
	}
	}

	error = sysfs_create_files(&client->dev.kobj,wacom_attrs);
	if (error < 0) {
		dev_err(&client->dev, "failed to creat version file %d\n", error);
	}

	struct proc_dir_entry *ver;
	ver = proc_symlink("emr_fw_ver", NULL, "/sys/bus/i2c/devices/1-0009/version");
	if ( !ver )
		dev_err(&client->dev," creat emr_fw_ver error \n");

#ifdef CONFIG_RATTA_GET_WACOM_PEN_TYPE
	ver = proc_symlink("emr_pen_type", NULL,
			   "/sys/bus/i2c/devices/1-0009/emr_pen_type");
	if ( !ver )
		dev_err(&client->dev," creat emr_pen_type error \n");
#endif	
	/* check emr 10.3 or 7.8*/
	if( input->id.product == WACOM_EMR_103 || input->id.product == WACOM_EMR_103_FW96)	{
		dev_dbg(&client->dev,"current emr is EMR_103 \n");
		wacom_i2c_irq = wacom_i2c_irq_sn100;
		g_ratta_mtype="ratta-sn100\n";
	}
	else if( input->id.product == WACOM_EMR_078 || input->id.product == WACOM_EMR_078_RATTA )	{
		dev_dbg(&client->dev,"current emr is EMR_078 \n");
		wacom_i2c_irq = wacom_i2c_irq_sn078; 
		g_ratta_mtype="ratta-sn078\n";
	}
	else {
		//default sn078
		g_ratta_mtype="ratta-other\n";
		dev_info(&client->dev,"%d(0x%x) check emr error use"
			" default emr:EMR_078"
			 "\n",input->id.product,input->id.product);
	}

	error = request_threaded_irq(client->irq, NULL, wacom_i2c_irq,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     "wacom_i2c", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_mem;
	}

	/* Disable the IRQ, we'll enable it in wac_i2c_open() */
	disable_irq(client->irq);

	error = input_register_device(wac_i2c->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_irq;
	}

	i2c_set_clientdata(client, wac_i2c);
	return 0;

err_free_irq:
	free_irq(client->irq, wac_i2c);
err_free_mem:
	input_free_device(input);
	kfree(wac_i2c);

	return error;
}

#define GPIO4_DR   0x20A8000

void  clr_gpio_wacom(void)
{
	static void *p_gpj0_base;
	int datalong;
	int i;
	int data;

	datalong=1;
	p_gpj0_base = ioremap(GPIO4_DR, datalong*4);
	if (!p_gpj0_base)
		return;

	data=readl(p_gpj0_base);
	printk("WYM clr_gpio_wacom GPIO4_DR-%x\n",data);
	data=data&0xfffeffcb;  //gpio  4  ,io18,19,25 
	writel(data,p_gpj0_base);	  //low level interrupt
	iounmap(p_gpj0_base);
	p_gpj0_base = NULL;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
//	pinctrl_pm_select_sleep_state(client->dev);
//	gpio_set_value(wac_i2c->gpio_wacom_rst, 0);
//	gpio_set_value(wac_i2c->gpio_wacom_pwr_en, 0);	  //added by wym 2018-04-17
	remove_proc_entry(RATTA_PROC_SUPERNOTE,NULL);
	sysfs_remove_files(&client->dev.kobj,wacom_attrs);
	printk("wym wacom_i2c_remove \n");
	free_irq(client->irq, wac_i2c);
	input_unregister_device(wac_i2c->input);
	kfree(wac_i2c);
	clr_gpio_wacom();
	printk("wym wacom_i2c_remove end \n");

	return 0;
}

static int __maybe_unused wacom_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	
	disable_irq(client->irq);
	gpio_set_value(wac_i2c->gpio_wacom_pwr_en, 0);
	
	return 0;
}

static int __maybe_unused wacom_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	gpio_set_value(wac_i2c->gpio_wacom_pwr_en, 1);
//	gpio_set_value(wac_i2c->gpio_wacom_rst, 0);
	msleep(150);
//	gpio_set_value(wac_i2c->gpio_wacom_rst, 1);
//	msleep(50);

//#if 1
//	gpio_set_value(wac_i2c->gpio_wacom_pwr_en, 1);
//	msleep(100);
//#endif

	gpio_set_value(wac_i2c->gpio_wacom_rst, 0);
	msleep(150);
	gpio_set_value(wac_i2c->gpio_wacom_rst, 1);
	msleep(50);

	enable_irq(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);

static const struct i2c_device_id wacom_i2c_id[] = {
	{"wac_i2c_emr", 0},
	{}
};

static const struct of_device_id wacom_dt_ids[] = {
	{
		.compatible = "wacom,wac_i2c_emr",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, wacom_dt_ids);

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom_i2c",
		.owner	= THIS_MODULE,
		.pm	= &wacom_i2c_pm,
		.of_match_table = wacom_dt_ids,		
	},

	.probe		= wacom_i2c_probe,
	.remove		= wacom_i2c_remove,
	.id_table	= wacom_i2c_id,
};
module_i2c_driver(wacom_i2c_driver);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM EMR I2C Driver");
MODULE_LICENSE("GPL");
