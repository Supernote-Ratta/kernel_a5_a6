/*
 * drivers/video/fbdev/mxc/mxc_epdc_cfg.c
 *
 * Copyright (C) 2020 Ratta
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

static ssize_t pen_type_show(struct class *cls,
			     struct class_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", epdc_helper->pen_type);
}

static ssize_t pen_type_store(struct class *cls,
			      struct class_attribute *attr,
			      const char *buf, size_t size)
{
	char *pstr;
	int value = 0;

	pstr = strchr(buf, '\r');
	if (pstr)
		*pstr = 0;

	pstr = strchr(buf, '\n');
	if (pstr)
		*pstr = 0;

	if (kstrtoint(buf, 10, &value))
		return size;

	lock_helper();
	if ((value > 0) && (value != epdc_helper->pen_type)) {
		epdc_helper->pen_type = value;
		epdc_helper->pen_type_copy = value;
		sysfs_notify(cls->dev_kobj, NULL, "pen_type");
	}
	unlock_helper();

        return size;
}

static ssize_t pen_width_show(struct class *cls,
			      struct class_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "%d\n", epdc_helper->pen_width);
}

static ssize_t pen_width_store(struct class *cls,
			       struct class_attribute *attr,
			       const char *buf, size_t size)
{
	char *pstr;
	int width = 0;

	pstr = strchr(buf, '\r');
	if (pstr)
		*pstr = 0;

	pstr = strchr(buf, '\n');
	if (pstr)
		*pstr = 0;

	if (kstrtoint(buf, 10, &width))
		return size;

	lock_helper();
	if ((width > 0) && (width != epdc_helper->pen_width)) {
		epdc_helper->pen_width = width;
		epdc_helper->pen_width_copy = width;
		sysfs_notify(cls->dev_kobj, NULL, "pen_width");
	}
	unlock_helper();

	return size;
}

static ssize_t pen_color_show(struct class *cls,
			      struct class_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "%d\n", epdc_helper->pen_color);
}

static ssize_t pen_color_store(struct class *cls,
			       struct class_attribute *attr,
			       const char *buf, size_t size)
{
	char *pstr;
	int value = 0;

	pstr = strchr(buf, '\r');
	if (pstr)
		*pstr = 0;

	pstr = strchr(buf, '\n');
	if (pstr)
		*pstr = 0;

	if (kstrtoint(buf, 10, &value))
		return size;

	lock_helper();
	if ((value > 0) && (value != epdc_helper->pen_color)) {
		epdc_helper->pen_color = value;
		epdc_helper->pen_color_copy = value;
		sysfs_notify(cls->dev_kobj, NULL, "pen_color");
	}
	unlock_helper();

        return size;
}

static struct class_attribute epdc_cfg_attrs[] = {
	__ATTR_RW(pen_type),
	__ATTR_RW(pen_width),
	__ATTR_RW(pen_color),
	__ATTR_NULL,
};

/** Device model classes */
struct class epdc_cfg_class = {
	.name        = "epdc_config",
	.class_attrs = epdc_cfg_attrs,
};
