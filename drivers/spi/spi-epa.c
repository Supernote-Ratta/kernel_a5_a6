/*
 * Simple synchronous interface to Ratta Epa SPI device
 *
 * Copyright (C) 2019 Ratta
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/semaphore.h>
#include <linux/fb.h>
#include <linux/file.h>
#include <linux/poll.h>
#include <linux/syscalls.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/of_gpio.h>

#include <linux/spi/spi.h>
#include <linux/spiepa.h>

#include <linux/uaccess.h>

#define SPIEPA_DEBUG
#define SPIEPA_BITMAP
//#define SPIEPA_DEBUG_SEND_TIME
//#define SPIEPA_DEBUG_TX
//#define SPIEPA_DEBUG_RX
//#define SPIEPA_DEBUG_OVERFLOW
#define SPIEPA_SCHED_RR

#define SPIEPA_MAJOR	0	/* dynamically assign */

#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
				| SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)
#define MAX_BUFFER_SZ		(60 * 1024)
#define SPIEPA_RING_SIZE	(4 * 1024)
#define SPIEPA_QUEUE_SIZE	(100 * 1024)
#define SPIEPA_TEMP_ARRAY_SIZE	(12 * 1024)
#define SPIEPA_PEN_WIDTH	2

#define SPIEPA_STATE_NONE	0
#define SPIEPA_STATE_WORKING	1
#define SPIEPA_STATE_DISABLED	2

#define SPIEPA_FLAG_EMR		0x1
#define SPIEPA_FLAG_EMR_REPEAT	0x2
#define SPIEPA_FLAG_TAIL	0xFE

#define SPIEPA_COLOR_WHITE	((u8)(0x80 + 0x14))
#define SPIEPA_COLOR_BLACK	((u8)0x14)

#define SPIEPA_PEN_G12		2
#define SPIEPA_PEN_G14		26

static int spiepa_major;
static struct mutex g_lock;
static struct completion reader_done;

static struct spiepa_data *spidata = NULL;

struct my_point {
	s16 x;
	s16 y;
};

#define SPIEPA_PRESSURE_SIZE	256
struct spiepa_pressure {
	int min, max;
};

static struct my_point circle0[] = {
	{0, 0},
};

static struct my_point circle1[] = {
	{0, 0},
	{-1, 0}, {0, -1}, {0, 1}, {1, 0},
};

static struct my_point circle2[] = {
	{0, 0},
	{-2, 0}, {-1, -1}, {-1, 0}, {-1, 1}, {0, -2}, {0, -1},
	{0, 1}, {0, 2}, {1, -1}, {1, 0}, {1, 1}, {2, 0},
};

static struct my_point circle3[] = {
	{0, 0},
	{-3, 0}, {-2, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-2, 2}, {-1, -2},
	{-1, -1}, {-1, 0}, {-1, 1}, {-1, 2}, {0, -3}, {0, -2}, {0, -1},
	{0, 1}, {0, 2}, {0, 3}, {1, -2}, {1, -1}, {1, 0}, {1, 1},
	{1, 2}, {2, -2}, {2, -1}, {2, 0}, {2, 1}, {2, 2}, {3, 0},
};

static struct my_point circle4[] = {
	{0, 0},
	{-4, 0}, {-3, -2}, {-3, -1}, {-3, 0}, {-3, 1}, {-3, 2}, {-2, -3},
	{-2, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-2, 2}, {-2, 3}, {-1, -3},
	{-1, -2}, {-1, -1}, {-1, 0}, {-1, 1}, {-1, 2}, {-1, 3}, {0, -4},
	{0, -3}, {0, -2}, {0, -1},
	{0, 1}, {0, 2}, {0, 3},
	{0, 4}, {1, -3}, {1, -2}, {1, -1}, {1, 0}, {1, 1}, {1, 2},
	{1, 3}, {2, -3}, {2, -2}, {2, -1}, {2, 0}, {2, 1}, {2, 2},
	{2, 3}, {3, -2}, {3, -1}, {3, 0}, {3, 1}, {3, 2}, {4, 0},
};

static struct my_point circle5[] = {
	{0, 0},
	{-5, 0}, {-4, -3}, {-4, -2}, {-4, -1}, {-4, 0}, {-4, 1}, {-4, 2},
	{-4, 3}, {-3, -4}, {-3, -3}, {-3, -2}, {-3, -1}, {-3, 0}, {-3, 1},
	{-3, 2}, {-3, 3}, {-3, 4}, {-2, -4}, {-2, -3}, {-2, -2}, {-2, -1},
	{-2, 0}, {-2, 1}, {-2, 2}, {-2, 3}, {-2, 4}, {-1, -4}, {-1, -3},
	{-1, -2}, {-1, -1}, {-1, 0}, {-1, 1}, {-1, 2}, {-1, 3}, {-1, 4},
	{0, -5}, {0, -4}, {0, -3}, {0, -2}, {0, -1},
	{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5},
	{1, -4}, {1, -3}, {1, -2}, {1, -1}, {1, 0}, {1, 1}, {1, 2},
	{1, 3}, {1, 4}, {2, -4}, {2, -3}, {2, -2}, {2, -1}, {2, 0},
	{2, 1}, {2, 2}, {2, 3}, {2, 4}, {3, -4}, {3, -3}, {3, -2},
	{3, -1}, {3, 0}, {3, 1}, {3, 2}, {3, 3}, {3, 4}, {4, -3},
	{4, -2}, {4, -1}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {5, 0},
};

static struct my_point circle6[] = {
	{0, 0},
	{-6, 0}, {-5, -3}, {-5, -2}, {-5, -1}, {-5, 0}, {-5, 1}, {-5, 2},
	{-5, 3}, {-4, -4}, {-4, -3}, {-4, -2}, {-4, -1}, {-4, 0}, {-4, 1},
	{-4, 2}, {-4, 3}, {-4, 4}, {-3, -5}, {-3, -4}, {-3, -3}, {-3, -2},
	{-3, -1}, {-3, 0}, {-3, 1}, {-3, 2}, {-3, 3}, {-3, 4}, {-3, 5},
	{-2, -5}, {-2, -4}, {-2, -3}, {-2, -2}, {-2, -1}, {-2, 0}, {-2, 1},
	{-2, 2}, {-2, 3}, {-2, 4}, {-2, 5}, {-1, -5}, {-1, -4}, {-1, -3},
	{-1, -2}, {-1, -1}, {-1, 0}, {-1, 1}, {-1, 2}, {-1, 3}, {-1, 4},
	{-1, 5}, {0, -6}, {0, -5}, {0, -4}, {0, -3}, {0, -2}, {0, -1},
	{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {1, -5},
	{1, -4}, {1, -3}, {1, -2}, {1, -1}, {1, 0}, {1, 1}, {1, 2},
	{1, 3}, {1, 4}, {1, 5}, {2, -5}, {2, -4}, {2, -3}, {2, -2},
	{2, -1}, {2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4}, {2, 5},
	{3, -5}, {3, -4}, {3, -3}, {3, -2}, {3, -1}, {3, 0}, {3, 1},
	{3, 2}, {3, 3}, {3, 4}, {3, 5}, {4, -4}, {4, -3}, {4, -2},
	{4, -1}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}, {5, -3},
	{5, -2}, {5, -1}, {5, 0}, {5, 1}, {5, 2}, {5, 3}, {6, 0},
};

static struct my_point circle7[] = {
	{0, 0},
	{-7, 0}, {-6, -3}, {-6, -2}, {-6, -1}, {-6, 0}, {-6, 1}, {-6, 2},
	{-6, 3}, {-5, -4}, {-5, -3}, {-5, -2}, {-5, -1}, {-5, 0}, {-5, 1},
	{-5, 2}, {-5, 3}, {-5, 4}, {-4, -5}, {-4, -4}, {-4, -3}, {-4, -2},
	{-4, -1}, {-4, 0}, {-4, 1}, {-4, 2}, {-4, 3}, {-4, 4}, {-4, 5},
	{-3, -6}, {-3, -5}, {-3, -4}, {-3, -3}, {-3, -2}, {-3, -1}, {-3, 0},
	{-3, 1}, {-3, 2}, {-3, 3}, {-3, 4}, {-3, 5}, {-3, 6}, {-2, -6},
	{-2, -5}, {-2, -4}, {-2, -3}, {-2, -2}, {-2, -1}, {-2, 0}, {-2, 1},
	{-2, 2}, {-2, 3}, {-2, 4}, {-2, 5}, {-2, 6}, {-1, -6}, {-1, -5},
	{-1, -4}, {-1, -3}, {-1, -2}, {-1, -1}, {-1, 0}, {-1, 1}, {-1, 2},
	{-1, 3}, {-1, 4}, {-1, 5}, {-1, 6}, {0, -7}, {0, -6}, {0, -5},
	{0, -4}, {0, -3}, {0, -2}, {0, -1},
	{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {0, 7},
	{1, -6}, {1, -5}, {1, -4}, {1, -3}, {1, -2}, {1, -1}, {1, 0},
	{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {2, -6},
	{2, -5}, {2, -4}, {2, -3}, {2, -2}, {2, -1}, {2, 0}, {2, 1},
	{2, 2}, {2, 3}, {2, 4}, {2, 5}, {2, 6}, {3, -6}, {3, -5},
	{3, -4}, {3, -3}, {3, -2}, {3, -1}, {3, 0}, {3, 1}, {3, 2},
	{3, 3}, {3, 4}, {3, 5}, {3, 6}, {4, -5}, {4, -4}, {4, -3},
	{4, -2}, {4, -1}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4},
	{4, 5}, {5, -4}, {5, -3}, {5, -2}, {5, -1}, {5, 0}, {5, 1},
	{5, 2}, {5, 3}, {5, 4}, {6, -3}, {6, -2}, {6, -1}, {6, 0},
	{6, 1}, {6, 2}, {6, 3}, {7, 0},
};

static struct my_point circle8[] = {
	{0, 0},
	{-8, 0}, {-7, -3}, {-7, -2}, {-7, -1}, {-7, 0}, {-7, 1}, {-7, 2},
	{-7, 3}, {-6, -5}, {-6, -4}, {-6, -3}, {-6, -2}, {-6, -1}, {-6, 0},
	{-6, 1}, {-6, 2}, {-6, 3}, {-6, 4}, {-6, 5}, {-5, -6}, {-5, -5},
	{-5, -4}, {-5, -3}, {-5, -2}, {-5, -1}, {-5, 0}, {-5, 1}, {-5, 2},
	{-5, 3}, {-5, 4}, {-5, 5}, {-5, 6}, {-4, -6}, {-4, -5}, {-4, -4},
	{-4, -3}, {-4, -2}, {-4, -1}, {-4, 0}, {-4, 1}, {-4, 2}, {-4, 3},
	{-4, 4}, {-4, 5}, {-4, 6}, {-3, -7}, {-3, -6}, {-3, -5}, {-3, -4},
	{-3, -3}, {-3, -2}, {-3, -1}, {-3, 0}, {-3, 1}, {-3, 2}, {-3, 3},
	{-3, 4}, {-3, 5}, {-3, 6}, {-3, 7}, {-2, -7}, {-2, -6}, {-2, -5},
	{-2, -4}, {-2, -3}, {-2, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-2, 2},
	{-2, 3}, {-2, 4}, {-2, 5}, {-2, 6}, {-2, 7}, {-1, -7}, {-1, -6},
	{-1, -5}, {-1, -4}, {-1, -3}, {-1, -2}, {-1, -1}, {-1, 0}, {-1, 1},
	{-1, 2}, {-1, 3}, {-1, 4}, {-1, 5}, {-1, 6}, {-1, 7}, {0, -8},
	{0, -7}, {0, -6}, {0, -5}, {0, -4}, {0, -3}, {0, -2}, {0, -1},
	{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {0, 7},
	{0, 8}, {1, -7}, {1, -6}, {1, -5}, {1, -4}, {1, -3}, {1, -2},
	{1, -1}, {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5},
	{1, 6}, {1, 7}, {2, -7}, {2, -6}, {2, -5}, {2, -4}, {2, -3},
	{2, -2}, {2, -1}, {2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4},
	{2, 5}, {2, 6}, {2, 7}, {3, -7}, {3, -6}, {3, -5}, {3, -4},
	{3, -3}, {3, -2}, {3, -1}, {3, 0}, {3, 1}, {3, 2}, {3, 3},
	{3, 4}, {3, 5}, {3, 6}, {3, 7}, {4, -6}, {4, -5}, {4, -4},
	{4, -3}, {4, -2}, {4, -1}, {4, 0}, {4, 1}, {4, 2}, {4, 3},
	{4, 4}, {4, 5}, {4, 6}, {5, -6}, {5, -5}, {5, -4}, {5, -3},
	{5, -2}, {5, -1}, {5, 0}, {5, 1}, {5, 2}, {5, 3}, {5, 4},
	{5, 5}, {5, 6}, {6, -5}, {6, -4}, {6, -3}, {6, -2}, {6, -1},
	{6, 0}, {6, 1}, {6, 2}, {6, 3}, {6, 4}, {6, 5}, {7, -3},
	{7, -2}, {7, -1}, {7, 0}, {7, 1}, {7, 2}, {7, 3}, {8, 0},
};

static struct my_point *circles[9] = {
	circle0,
	circle1,
	circle2,
	circle3,
	circle4,
	circle5,
	circle6,
	circle7,
	circle8,
};

static int circle_lens[9] = {
	sizeof(circle0) / sizeof(circle0[0]),
	sizeof(circle1) / sizeof(circle1[0]),
	sizeof(circle2) / sizeof(circle2[0]),
	sizeof(circle3) / sizeof(circle3[0]),
	sizeof(circle4) / sizeof(circle4[0]),
	sizeof(circle5) / sizeof(circle5[0]),
	sizeof(circle6) / sizeof(circle6[0]),
	sizeof(circle7) / sizeof(circle7[0]),
	sizeof(circle8) / sizeof(circle8[0]),
};

struct spiepa_node {
	u16 flag;
	s16 x;
	s16 y;
	s16 p;
	u8 gray;
};

struct spiepa_point {
	int x;
	int y;
	/* pressure */
	int p;
	/* pen type */
	u8 pen;
	/* tool type */
	u8 tool:2;
};

struct graphic_info {
	struct mutex lock;
	/* pen type */
	int pen_type;
	/* pen width */
	int pen_width;
	/* pixel gray */
	int gray_scale;
	/* maximum graphic width */
	int max_width;
	/* maximum graphic height */
	int max_height;
	int max_emr_x;
	int max_emr_y;
	/* maximum pen width */
	int max_pen_width;
	/* compluted from pressure */
	int cur_penwidth;
	/* delta change */
	int delta_change;
	/* x correct offset */
	int delta_x;
	/* y correct offset */
	int delta_y;
	/* temp disabled regions */
	struct spiepa_regions *pregions_tmp;
	/* last disabled regions */
	struct spiepa_regions *pregions;
	/* fast lookup */
	int *row_start_table;
};

struct spiepa_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	spinlock_t		compute_lock;
	struct spi_device	*spi;

	int			sw_gpio;
	int			test_gpio;
	int			sw_active_low;

	/* struct spiepa_node */
	struct spiepa_node	*to_epa;
	struct spiepa_node	*to_user_array;
	int			to_user_head;
	int			to_user_tail;
	struct spiepa_node	*tmp_node_array;
	struct spiepa_node	*emr_node_array;
	int			emr_cnt;
	struct mutex		to_user_lock;
	unsigned int		users;
	u32			speed_hz;

	int			state;
	struct mutex		buf_lock;
	wait_queue_head_t	wait;
	struct task_struct	*kreader;
	struct spiepa_point	*event_array;
	int			event_head;
	int			event_tail;
	struct semaphore	event_sema;
	struct mutex		event_lock;
	struct graphic_info	gi;
#ifdef SPIEPA_BITMAP
	unsigned long		*map;
#endif
	int			map_size;
#ifdef SPIEPA_BITMAP
	unsigned long		*region_map;
#endif
	int			region_size;
#ifdef SPIEPA_BITMAP
	unsigned long		*emr_map;
#endif
	struct spiepa_pressure	pressures[SPIEPA_PRESSURE_SIZE];
	int			emr_map_size;
	struct mutex		region_lock;
	int			pressure;
	unsigned long		tool_jiffs;
	__s16			x, y, last_x, last_y;
	bool			active;
	bool			status;
	u8			*tx_buffer, *rx_buffer;
	u8			tool;
#ifndef SPIEPA_BITMAP
	u8			*map;
	u8			*region_map;
	u8			*emr_map;
#endif
};

static int spiepa_take_update_right(void);
static int spiepa_give_update_right(void);
static int spiepa_write_direct(int);

static void spiepa_notify_badepa(void)
{
	char data[20] = {0}, *envp[] = {data, NULL};

	snprintf(data, sizeof(data), "RATTABADEPA");

	kobject_uevent_env(&spidata->spi->dev.kobj, KOBJ_CHANGE, envp);
}

static int spiepa_init_graphic(void)
{
	struct graphic_info *pgi;

	if (!spidata)
		return -ENODEV;

	pgi = &spidata->gi;
	mutex_init(&pgi->lock);
	pgi->pen_type = 0;
	pgi->pen_width = SPIEPA_PEN_WIDTH;
	pgi->gray_scale = 20;
	pgi->max_pen_width = 20;

	pgi->pregions = (typeof(pgi->pregions))
		kzalloc(sizeof(*(pgi->pregions)), GFP_KERNEL);
	if (!pgi->pregions) {
		return -ENOMEM;
	}

	pgi->pregions_tmp = (typeof(pgi->pregions_tmp))
		kzalloc(sizeof(*(pgi->pregions_tmp)), GFP_KERNEL);
	if (!pgi->pregions_tmp) {
		kfree(pgi->pregions);
		pgi->pregions = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void spiepa_deinit_graphic(void)
{
	struct graphic_info *pgi;

	pgi = &spidata->gi;

	if (pgi->pregions_tmp) {
		kfree(pgi->pregions_tmp);
		pgi->pregions_tmp = NULL;
	}

	if (pgi->pregions) {
		kfree(pgi->pregions);
		pgi->pregions = NULL;
	}
}

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spiepa_complete(void *arg)
{
	complete(arg);
}

static ssize_t spiepa_sync(struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spiepa_complete;
	message->context = &done;

	spin_lock_irq(&spidata->spi_lock);
	if (spidata->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spidata->spi, message);
	spin_unlock_irq(&spidata->spi_lock);


	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}

	return status;
}

static ssize_t spiepa_send(char *tx, char *rx, size_t len)
{
	int status;

	struct spi_transfer t = {
			.tx_buf = tx,
			.rx_buf = rx,
			.len = len,
			.speed_hz = spidata->speed_hz,
		};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	status = spiepa_sync(&m);

	return status;
}

static u8 spiepa_make_sum(char *buf, int len)
{
	int i;
	u8 sum;

	sum = buf[0];
	for (i = 1; i < len; i++) {
		sum ^= buf[i];
	}

	return sum;
}

static int spiepa_make_request(u8 cmd,
				u16 slen,
				u8 *data,
				u16 rlen)
{
	u8 *pb = spidata->tx_buffer, sum;

	if ((slen + 6) > MAX_BUFFER_SZ)
		return -EINVAL;

	*pb++ = cmd;

	*pb++ = (slen >> 8) & 0xff;
	*pb++ = slen & 0xff;

	if (data && slen) {
		memcpy(pb, data, slen);
		pb += slen;
	}

	*pb++ = (rlen >> 8) & 0xff;
	*pb++ = rlen & 0xff;

	sum = spiepa_make_sum(spidata->tx_buffer, (char *)pb -
			      (char *)(spidata->tx_buffer));
	*pb++ = sum;
	*pb++ = 0;

	return slen + 6;
}

static bool spiepa_check_recv(char *buf, int len)
{
	u8 sum;
	int i;

	if (buf[0] != 0)
		return false;

	sum = buf[0];

	for (i = 1; i < len; i++) {
		sum ^= buf[i];
	}

	if (sum != 0)
		return false;

	return true;
}

/* 00 00 00 00 09 09 ff - ff ff ff ff ff ff ff ff ff ff ff ff ff */
static int spiepa_ask_version(char *buf, int len)
{
	int ret;

	mutex_lock(&spidata->buf_lock);

	memset(spidata->tx_buffer, 0xff, MAX_BUFFER_SZ);
	memset(spidata->rx_buffer, 0xff, MAX_BUFFER_SZ);

	ret = spiepa_make_request(0, 0, NULL, 9);
	if (ret < 0) {
		mutex_unlock(&spidata->buf_lock);
		return ret;
	}

	ret = spiepa_send(spidata->tx_buffer, spidata->rx_buffer,
			  6 + 1 + 9 + 4);

	if (!spiepa_check_recv(spidata->rx_buffer + 7, 20 - 7)) {
		spiepa_send(spidata->tx_buffer, spidata->rx_buffer,
			    6 + 1 + 9 + 4);
		if (!spiepa_check_recv(spidata->rx_buffer + 7, 20 - 7)) {
			spidata->status = false;
			spiepa_take_update_right();
			mutex_unlock(&spidata->buf_lock);
			spiepa_notify_badepa();
			return -EFAULT;
		}
	}

	if (ret == 20) {
		buf[len - 1] = 0;
		/* mm.ss.YYYYMMDDHHMMSS */
		snprintf(buf, len - 1, "%02u.%02u.%02x%02x%02x%02x%02x%02x%02x",
			 (unsigned int)(spidata->rx_buffer[10]),
			 (unsigned int)(spidata->rx_buffer[11]),
			 (unsigned int)(spidata->rx_buffer[12]),
			 (unsigned int)(spidata->rx_buffer[13]),
			 (unsigned int)(spidata->rx_buffer[14]),
			 (unsigned int)(spidata->rx_buffer[15]),
			 (unsigned int)(spidata->rx_buffer[16]),
			 (unsigned int)(spidata->rx_buffer[17]),
			 (unsigned int)(spidata->rx_buffer[18]));
	}

	mutex_unlock(&spidata->buf_lock);

	return (ret == 20) ? ret : -EFAULT;
}

static bool spiepa_validate_point(int x, int y)
{
	struct graphic_info *pgi = &spidata->gi;
	int nr = x + pgi->row_start_table[y];

	if (x < 0 || x >= pgi->max_width ||
	    y < 0 || y >= pgi->max_height) {
		return false;
	}

#ifdef SPIEPA_BITMAP
	if (test_bit(nr, spidata->region_map)) {
		return false;
	}

	if (test_bit(nr, spidata->map)) {
		return false;
	}

	set_bit(nr, spidata->map);
#else
	if (spidata->map[nr]) {
		return false;
	}

	if (spidata->region_map[nr]) {
		return false;
	}

	spidata->map[nr] = 1;
#endif

	return true;
}

static int spiepa_draw_circle(int x, int y, int pressure,
			      int r, u8 color, int start)
{
	int i, cnt = 0;
	struct spiepa_node *node;

	for (i = 0; i < circle_lens[r]; i++) {
		if (spiepa_validate_point(circles[r][i].x + x,
					  circles[r][i].y + y)) {
			if (cnt >= (SPIEPA_TEMP_ARRAY_SIZE - 2)) {
				dev_err(&spidata->spi->dev,
					"No more space in temp array,cnt=%d\n",
					cnt);
				break;
			}

			node = &spidata->tmp_node_array[(cnt++) + start];
			node->flag = 0;
			node->x =
				(typeof(node->x))
				(circles[r][i].x) +
				x;
			node->y = (typeof(node->y))
				(circles[r][i].y) +
				y;
			node->p = (typeof(node->p))pressure;
			node->gray =
				(typeof(node->gray))
				color;
		}
	}

	return cnt;
}

static int spiepa_draw_dot(int x, int y, int pressure,
			   int width, u8 color, int start)
{
	int i, j, cnt = 0, yoff;
	struct spiepa_node *node;
	struct graphic_info *pgi = &spidata->gi;

	yoff = width;
	if (!pgi->pen_type)
		yoff = 3;

	for (i = y; i < y + yoff; i++) {
		for (j = x; j < x + width; j++) {
			if (spiepa_validate_point(j, i)) {
				if (cnt >= (SPIEPA_TEMP_ARRAY_SIZE - 2)) {
					dev_err(&spidata->spi->dev,
						"No more space in temp array,cnt=%d\n",
						cnt);
					break;
				}

				node = &spidata->tmp_node_array[(cnt++) +
					start];
				node->flag = 0;
				node->x = (typeof(node->x))j;
				node->y = (typeof(node->y))i;
				node->p = (typeof(node->p))pressure;
				node->gray =
					(typeof(node->gray))
					color;
			}
		}
	}

	return cnt;
}

static int spiepa_get_line_width(bool first, int pressure, u8 pen)
{
	struct graphic_info *pgi = &spidata->gi;
	int delta_mom = spidata->pressures[pen].max -
		spidata->pressures[pen].min;
	int delta_son = pressure - spidata->pressures[pen].min;
	int tmp, tmp_penwidth, tmp_delta;

	if (delta_son < 0)
		delta_son = 0;
	else if (delta_son > spidata->pressures[pen].max)
		delta_son = spidata->pressures[pen].max;

	if (first) {
		tmp_penwidth = pgi->pen_width * delta_son * 10000 /
			delta_mom;
		pgi->cur_penwidth = tmp_penwidth / 10000 + 1;
		if ((tmp_penwidth % 10000) >= 5000)
			pgi->cur_penwidth += 1;
	} else {
		tmp = pgi->pen_width * delta_son * 10000 / delta_mom;
		tmp_penwidth = tmp / 10000 + 1;
		if ((tmp % 10000) >= 5000)
			tmp_penwidth += 1;
		tmp_delta = tmp_penwidth - pgi->cur_penwidth;
		if (pgi->delta_change < 1) {
			pgi->delta_change++;
		} else if (tmp_delta != 0) {
			pgi->cur_penwidth = tmp_delta > 0 ?
				pgi->cur_penwidth + 1 :
				pgi->cur_penwidth - 1;
			pgi->delta_change = 0;
		}
	}

	if (pgi->cur_penwidth < 1)
		pgi->cur_penwidth = 1;
	else if (pgi->cur_penwidth > pgi->pen_width)
		pgi->cur_penwidth = pgi->pen_width;

	return pgi->cur_penwidth;
}

static void spiepa_draw_45rect(int x, int y, int pressure, int width)
{
	int i, j, cir_num = 11, val_tmp = 0, dif1 = 0, dif2 = 0, cnt = 0,
	    counts = 0;

	for (i = y; i < y + width; i++) {
		val_tmp = i + 1;
		dif1 = val_tmp % cir_num;
		dif2 = cir_num - dif1;

		for (j = x; j < x + width; j++) {
			if ((j % cir_num == dif1) ||
			    (j % cir_num == dif1 + 1) ||
			    (j % cir_num == dif2) ||
			    (j % cir_num == dif2 + 1)) {
				/* black */
				cnt = spiepa_draw_dot(j, i, pressure,
						      1, 0x14, counts);
				counts += cnt;
			} else {
				/* white */
				cnt = spiepa_draw_dot(j, i, pressure,
						      1, 0x80 + 0x14, counts);
				counts += cnt;
			}
		}
	}

	if (counts > 0) {
		spiepa_write_direct(counts);
	}
}

static void spiepa_draw_line(struct my_point *begin,
			     struct my_point *end,
			     int pressure,
			     int width, u8 color)
{
	int x = begin->x, y = begin->y;
	int dx = abs(end->x - begin->x), dy = abs(end->y - begin->y);
	int s1 = end->x > begin->x ? 1 : -1, s2 = end->y > begin->y ? 1 : -1;
	int interchange = 0, p, i, cnt = 0, counts = 0;
	struct graphic_info *pgi = &spidata->gi;

	/* if slop rate > 1, dx, dy exchange */
	if (dy > dx) {
		int temp = dx;
		dx = dy;
		dy = temp;
		interchange = 1;
	}

	p = 2 * dy - dx;
	for (i = 0; i <= dx; i++) {
		if ((pgi->pen_type < 3) &&
		    (pgi->pen_width < 9)) {
			cnt = spiepa_draw_circle(x, y, pressure,
						 width, color,
						 counts);
			counts += cnt;
		} else if (pgi->pen_type < 3) {
			cnt = spiepa_draw_dot(x, y, pressure,
					      width, color,
					      counts);
			counts += cnt;
		} else {
			spiepa_draw_45rect(x, y, pressure,
					   width);
		}

		if (p >= 0) {
			if (!interchange)
				y += s2;
			else
				x += s1;
			p -= 2 * dx;
		}

		if (!interchange)
			x += s1;
		else
			y += s2;

		p += 2 * dy;
	}

	if (counts > 0) {
		spiepa_write_direct(counts);
	}
}

static int spiepa_take_update_right(void)
{
	gpio_set_value(spidata->sw_gpio,
		       spidata->sw_active_low ? 0 : 1);
	dev_info(&spidata->spi->dev,
		 "Set gpio%d %s\n",
		 spidata->sw_gpio,
		 spidata->sw_active_low ? "low" : "high");

	return 0;
}

static int spiepa_give_update_right(void)
{
	gpio_set_value(spidata->sw_gpio,
		       spidata->sw_active_low ? 1 : 0);
	dev_info(&spidata->spi->dev,
		 "Set gpio%d %s\n",
		 spidata->sw_gpio,
		 spidata->sw_active_low ? "high" : "low");

	return 0;
}

static void spiepa_send_points(int counts)
{
	struct spiepa_node *p;
	u8 *data;
	int cnt = 0, ret, i;
#ifdef SPIEPA_DEBUG_SEND_TIME
	unsigned long pre_jiffs, end_jiffs;
#endif

	mutex_lock(&spidata->buf_lock);

	spidata->tx_buffer[0] = 0x90;

	data = (u8 *)(&spidata->tx_buffer[3]);
	for (i = 0; i < counts; i++)
	{
		p = &spidata->tmp_node_array[i];

		if ((data + cnt + 5 - spidata->tx_buffer) >=
		    (MAX_BUFFER_SZ - 40)) {
			dev_err(&spidata->spi->dev,
				"Too many points.\n");
			break;
		}
		data[cnt] = 0xff;
		/* gray scale */
		data[cnt + 1] = p->gray & 0xff;
		/* x */
		data[cnt + 2] = (((__u16)(p->x)) >> 8) & 0xff;
		data[cnt + 3] = p->x & 0xff;
		/* y */
		data[cnt + 4] = (((__u16)(p->y)) >> 8) & 0xff;
		data[cnt + 5] = p->y & 0xff;
		cnt += 6;
	}
	spidata->tx_buffer[1] = (cnt >> 8) & 0xff;
	spidata->tx_buffer[2] = cnt & 0xff;

	/* expect return length */
	data[cnt] = 0;
	data[cnt + 1] = 0;
	/* check sum */
	data[cnt + 2] = spiepa_make_sum(spidata->tx_buffer,
				    5 + cnt);
	/* reserved */
	data[cnt + 3] = 0;

	data[cnt + 4] = 0xff;
	data[cnt + 5] = 0xff;
	data[cnt + 6] = 0xff;
	data[cnt + 7] = 0xff;

#ifdef SPIEPA_DEBUG_OVERFLOW
	if ((data + cnt + 5 - spidata->tx_buffer) >=
	    (MAX_BUFFER_SZ - 40)) {
		dev_info(&spidata->spi->dev, "TX:\n");
		for (i = 3; i < cnt + 3 + 4 + 4; i += 6) {
			printk("%02x%02x %02x%02x\n",
			       spidata->tx_buffer[i+2],
			       spidata->tx_buffer[i+3],
			       spidata->tx_buffer[i+4],
			       spidata->tx_buffer[i+5]
			       );
		}
		printk("\n");
	}
#endif

#ifdef SPIEPA_DEBUG_TX
	dev_info(&spidata->spi->dev, "TX:\n");
	for (i = 0; i < cnt + 3 + 4 + 4; i++) {
		printk("%02x ", spidata->tx_buffer[i]);
	}
	printk("\n");
#endif

	spidata->rx_buffer[cnt + 3 + 4] = 0xff;
#ifdef SPIEPA_DEBUG_SEND_TIME
	pre_jiffs = jiffies;
#endif
	ret = spiepa_send(spidata->tx_buffer, spidata->rx_buffer, cnt +
			  3 + 4 + 4);
#ifdef SPIEPA_DEBUG_SEND_TIME
	end_jiffs = jiffies;
	if ((cnt + 3 + 4 + 4 > 50000)) {
		printk("%d bytes cost %ums\n",
		       cnt + 3 + 4 + 4,
		       jiffies_to_msecs(end_jiffs - pre_jiffs));
	}
#endif
#ifdef SPIEPA_DEBUG_RX
	dev_info(&spidata->spi->dev, "RX:\n");
	for (i = 0; i < cnt + 3 + 4 + 4; i++) {
		printk("%02x ", spidata->rx_buffer[i]);
	}
	printk("\n");
#endif
#if 0
	if (!spiepa_check_recv(spidata->rx_buffer +
			       3 + cnt + 4,
			       4)) {
		ret = spiepa_send(spidata->tx_buffer, spidata->rx_buffer, cnt +
				  3 + 4 + 4);
		if (!spiepa_check_recv(spidata->rx_buffer +
				       3 + cnt + 4,
				       4)) {
			spidata->status = false;
			spiepa_take_update_right();
			spiepa_notify_badepa();
		}
	}
#endif

	mutex_unlock(&spidata->buf_lock);
}

static int spiepa_write_direct(int cnt)
{
	struct spiepa_node *dst, *src;
	struct graphic_info *pgi = &spidata->gi;
	int i = 0, idx;

	spiepa_send_points(cnt);

	if (cnt > 0) {
		src = &spidata->tmp_node_array[cnt];
		src->flag = 0xFF;
		cnt++;
	}

	mutex_lock(&spidata->to_user_lock);
	while (((spidata->to_user_tail + 1) % SPIEPA_QUEUE_SIZE) !=
	       spidata->to_user_head) {
		if (cnt == 0)
			break;
		idx = spidata->to_user_tail;
		src = &spidata->tmp_node_array[i];
		dst = &spidata->to_user_array[idx];
#ifdef SPIEPA_BITMAP
		if (!test_bit(src->x +
			      pgi->row_start_table[src->y],
			      spidata->emr_map))
#else
		if (!spidata->emr_map[src->x + pgi->row_start_table[src->y]])
#endif
		{
			memcpy(dst, src, sizeof(*dst));
			spidata->to_user_tail = (idx + 1) % SPIEPA_QUEUE_SIZE;
		}
		cnt--;
		i++;
	}

	cnt = 0;
	for (i = 0; i < spidata->emr_cnt; i++) {
		if (((spidata->to_user_tail + 1) % SPIEPA_QUEUE_SIZE) ==
		    spidata->to_user_head) {
			break;
		}
		idx = spidata->to_user_tail;
		src = &spidata->emr_node_array[i];
		dst = &spidata->to_user_array[idx];

		memcpy(dst, src, sizeof(*dst));
		spidata->to_user_tail = (idx + 1) % SPIEPA_QUEUE_SIZE;
		cnt++;
	}
	spidata->emr_cnt -= cnt;
	mutex_unlock(&spidata->to_user_lock);

	return 0;
}

#if 0
static void spiepa_test_points(int start_x)
{
	int ret, i, j, cnt = 0;
	unsigned long pre_jiffs, end_jiffs;
	u8 *data;

	mutex_lock(&spidata->buf_lock);

	spidata->tx_buffer[0] = 0x90;

	data = (u8 *)(&spidata->tx_buffer[3]);
	for (i = start_x; i < start_x + 100; i++) {
		for (j = 0; j < 100; j++) {
			if ((data + cnt + 5 - spidata->tx_buffer) >=
			    (MAX_BUFFER_SZ - 40)) {
				dev_err(&spidata->spi->dev,
					"Too many points.\n");
				break;
			}
			data[cnt] = 0xff;
			/* gray scale */
			data[cnt + 1] = 20;
			/* x */
			data[cnt + 2] = (((__u16)(i)) >> 8) & 0xff;
			data[cnt + 3] = i & 0xff;
			/* y */
			data[cnt + 4] = (((__u16)(j)) >> 8) & 0xff;
			data[cnt + 5] = j & 0xff;
			cnt += 6;
		}
	}
	spidata->tx_buffer[1] = (cnt >> 8) & 0xff;
	spidata->tx_buffer[2] = cnt & 0xff;

	/* expect return length */
	data[cnt] = 0;
	data[cnt + 1] = 0;
	/* check sum */
	data[cnt + 2] = spiepa_make_sum(spidata->tx_buffer,
				    5 + cnt);
	/* reserved */
	data[cnt + 3] = 0;

	data[cnt + 4] = 0xff;
	data[cnt + 5] = 0xff;
	data[cnt + 6] = 0xff;
	data[cnt + 7] = 0xff;

	spidata->rx_buffer[cnt + 3 + 4] = 0xff;
	pre_jiffs = jiffies;
	ret = spiepa_send(spidata->tx_buffer, spidata->rx_buffer, cnt +
			  3 + 4 + 4);
	end_jiffs = jiffies;
	dev_info(&spidata->spi->dev,
		 "%d bytes cost %ums\n",
		 cnt + 3 + 4 + 4,
		 jiffies_to_msecs(end_jiffs - pre_jiffs));

	mutex_unlock(&spidata->buf_lock);
}
#endif

static void spiepa_parse_event(struct spiepa_point *sp,
				bool *first)
{
	int width;
	struct spiepa_node *sn;
	struct my_point begin, end;
	struct graphic_info *pgi = &spidata->gi;
	u8 color;

	if ((spidata->pressure <= 0) &&
	    (sp->p > 0)) {
		if (spidata->tool != sp->tool) {
			if (jiffies_to_msecs(jiffies - spidata->tool_jiffs) >
			    10) {
				spidata->tool = sp->tool;
				spidata->tool_jiffs = jiffies;
			}
		}

		spidata->last_x = sp->x;
		spidata->last_y = sp->y;
		*first = true;

		memset(spidata->map, 0, spidata->map_size);
		memset(spidata->emr_map, 0, spidata->emr_map_size);
		spidata->emr_cnt = 0;
	}

#ifdef SPIEPA_BITMAP
	if (!test_bit(sp->x + spidata->gi.row_start_table[sp->y],
		      spidata->emr_map))
#else
	if (!spidata->emr_map[sp->x + spidata->gi.row_start_table[sp->y]])
#endif
	{
		sn = &spidata->emr_node_array[spidata->emr_cnt++];
		sn->flag = 1;
		sn->x = sp->x;
		sn->y = sp->y;
		sn->p = sp->p;
		sn->gray = (pgi->gray_scale & 0xff);
		sn = NULL;
#ifdef SPIEPA_BITMAP
		set_bit(sp->x + spidata->gi.row_start_table[sp->y],
			spidata->emr_map);
#else
		spidata->emr_map[sp->x + spidata->gi.row_start_table[sp->y]] =
			1;
#endif
	}

	if (sp->p > 0) {
		if ((spidata->last_x >= 0) &&
		    (spidata->last_y >= 0) &&
#if 1
		    ((abs(spidata->last_x - sp->x) > 3) ||
		     (abs(spidata->last_y - sp->y) > 3) || *first) &&
#endif
		    (spidata->state == SPIEPA_STATE_WORKING)) {
			begin.x = spidata->last_x;
			begin.y = spidata->last_y;
			end.x = sp->x;
			end.y = sp->y;

			color = (u8)(pgi->gray_scale);
			if (spidata->tool == SPIEPA_TOOL_RUBBER) {
				color = SPIEPA_COLOR_WHITE;
			}
			width = pgi->pen_width;
			if (pgi->pen_type == 1 ||
			    pgi->pen_type == 2) {
				width = spiepa_get_line_width(*first,
							      sp->p,
							      sp->pen);
			}
			spiepa_draw_line(&begin,
					 &end,
					 sp->p,
					 width,
					 color);
			spidata->last_x = sp->x;
			spidata->last_y = sp->y;
			*first = false;
		}
	} else if (sp->p <= 0) {
		if (spidata->pressure > 0) {
			mutex_lock(&spidata->to_user_lock);
			if ((spidata->to_user_tail + 1) % SPIEPA_QUEUE_SIZE !=
			    spidata->to_user_head) {
				sn = &spidata->to_user_array[spidata->
					to_user_tail];
				sn->flag = 0xFE;
				spidata->to_user_tail =
					(spidata->to_user_tail + 1) %
					SPIEPA_QUEUE_SIZE;
			} else {
				dev_err(&spidata->spi->dev,
					"No more space to add tail.\n");
			}
			mutex_unlock(&spidata->to_user_lock);

			*first = true;
			spidata->x = spidata->y = -1;
			spidata->last_x = spidata->last_y = -1;
			spidata->tool = SPIEPA_TOOL_PEN;
			spidata->tool_jiffs = jiffies;
			memset(spidata->map, 0, spidata->map_size);

			/* x and y will be -1 */
			goto done;
		}
	}

	spidata->x = sp->x;
	spidata->y = sp->y;
done:
	spidata->pressure = sp->p;
}

void spiepa_event_enqueue(int x, int y, int p, u8 tool, u8 pen)
{
	struct spiepa_point *sp;
	struct graphic_info *pgi;
	int x0, y0;

	if (!spidata || !(pgi = &spidata->gi) ||
	    (spidata->state != SPIEPA_STATE_WORKING) ||
	    !pgi->max_width ||
	    !pgi->max_height ||
	    !pgi->max_emr_x ||
	    !pgi->max_emr_y) {
		return;
	}

	x0 = (x * pgi->max_width) / pgi->max_emr_x;
	y0 = (y * pgi->max_height) / pgi->max_emr_y;
	x0 += pgi->delta_x;
	y0 += pgi->delta_y;

	if ((x0 < 0) ||
	    (x0 >= pgi->max_width) ||
	    (y0 < 0) ||
	    (y0 >= pgi->max_height)) {
		return;
	}

	if ((x == spidata->x) &&
	    (y == spidata->y) &&
	    (!!p == !!spidata->pressure) &&
	    (tool == spidata->tool))
		return;

	mutex_lock(&spidata->event_lock);
	if (((spidata->event_tail + 1) % SPIEPA_QUEUE_SIZE) !=
	    (spidata->event_head)) {
		sp = &spidata->event_array[spidata->event_tail];
		sp->x = x0;
		sp->y = y0;
		sp->p = p;
		sp->pen = pen;
		sp->tool = tool;
		spidata->event_tail = (spidata->event_tail + 1) %
			SPIEPA_QUEUE_SIZE;
		mutex_unlock(&spidata->event_lock);

		up(&spidata->event_sema);

		return;
	} else {
		dev_err(&spidata->spi->dev,
			"No more space in emr queue\n");
	}
	mutex_unlock(&spidata->event_lock);
}
EXPORT_SYMBOL_GPL(spiepa_event_enqueue);

static int spiepa_event_dequeue(struct spiepa_point *sp)
{
	mutex_lock(&spidata->event_lock);

	if (spidata->event_head ==
	    spidata->event_tail) {
		mutex_unlock(&spidata->event_lock);
		return -EAGAIN;
	}

	*sp = spidata->event_array[spidata->event_head];
	spidata->event_head = (spidata->event_head + 1) %
		SPIEPA_QUEUE_SIZE;

	mutex_unlock(&spidata->event_lock);

	return 0;
}


static int spiepa_reader(void *data)
{
	struct spiepa_point st;
	struct spiepa_point *sp = NULL;
	bool first = true;

	while (spidata->active &&
	       (down_interruptible(&spidata->event_sema) == 0)) {
		if (!spidata->active)
			break;
		if (!spiepa_event_dequeue(&st)) {
			sp = &st;
		} else {
			dev_info(&spidata->spi->dev,
				 "Got empty spiepa event.\n");
			if (spidata->active)
				continue;
			break;
		}

		spiepa_parse_event(sp, &first);
	}

	complete(&reader_done);

	return 0;
}

static int spiepa_alloc_map(void)
{
	int ret, i, size;
	struct graphic_info *pgi = &spidata->gi;

	if (spidata->region_map)
		return 0;

	if ((pgi->max_width <= 0) ||
	    (pgi->max_height <= 0)) {
		return -EFAULT;
	}

	if (!pgi->row_start_table) {
		pgi->row_start_table = (typeof(pgi->row_start_table))
			kzalloc(sizeof(*(pgi->row_start_table)) *
				pgi->max_height,
				GFP_KERNEL);
	}
	if (!pgi->row_start_table) {
		pgi->max_width = pgi->max_height = 0;
		return -ENOMEM;
	}

	for (i = 0; i < pgi->max_height; i++) {
		pgi->row_start_table[i] = i * pgi->max_width;
	}

#ifdef SPIEPA_BITMAP
	size = BITS_TO_LONGS(pgi->max_width * pgi->max_height) * sizeof(long);
#else
	size = pgi->max_width * pgi->max_height;
#endif
	if (!spidata->map) {
		spidata->map = (typeof(spidata->map))kzalloc(size, GFP_KERNEL);
	}
	if (!spidata->map) {
		ret = -ENOMEM;
		goto err_map;
	}
	spidata->map_size = size;

	if (!spidata->emr_map) {
		spidata->emr_map = (typeof(spidata->emr_map))
			kzalloc(size, GFP_KERNEL);
	}
	if (!spidata->emr_map) {
		ret = -ENOMEM;
		goto err_emr;
	}
	spidata->emr_map_size = size;

	if (!spidata->region_map) {
		spidata->region_map = (typeof(spidata->region_map))
			kmalloc(size, GFP_KERNEL);
	}
	if (!spidata->region_map) {
		ret = -ENOMEM;
		goto err_region;
	}
	spidata->region_size = size;

	memset(spidata->region_map, 0xff, spidata->region_size);
	mutex_init(&spidata->region_lock);

	return 0;

err_region:
	kfree(spidata->emr_map);
	spidata->emr_map = NULL;
	spidata->emr_map_size = 0;
err_emr:
	kfree(spidata->map);
	spidata->map = NULL;
	spidata->map_size = 0;
err_map:
	kfree(pgi->row_start_table);
	pgi->row_start_table = NULL;

	return ret;
}

static void spiepa_free_map(void)
{
	if (spidata->region_map) {
		kfree(spidata->region_map);
		spidata->region_map = NULL;
		spidata->region_size = 0;
	}

	if (spidata->emr_map) {
		kfree(spidata->emr_map);
		spidata->emr_map = NULL;
		spidata->emr_map_size = 0;
	}

	if (spidata->map) {
		kfree(spidata->map);
		spidata->map = NULL;
		spidata->map_size = 0;
	}

	if (spidata->gi.row_start_table) {
		kfree(spidata->gi.row_start_table);
		spidata->gi.row_start_table = NULL;
	}
}

static int spiepa_alloc_queue(void)
{
	int ret;

	if (spidata->tmp_node_array) {
		return 0;
	}

	if (!spidata->event_array) {
		spidata->event_array = (typeof(spidata->event_array))
			kmalloc(sizeof(*(spidata->event_array)) *
				SPIEPA_QUEUE_SIZE, GFP_KERNEL);
		if (!spidata->event_array)
			return -ENOMEM;
	}

	if (!spidata->to_user_array) {
		spidata->to_user_array = (typeof(spidata->to_user_array))
			kmalloc(sizeof(*(spidata->to_user_array)) *
				SPIEPA_QUEUE_SIZE, GFP_KERNEL);
		if (!spidata->to_user_array) {
			ret = -ENOMEM;
			goto err_user_array;
		}
	}

	if (!spidata->emr_node_array) {
		spidata->emr_node_array = (typeof(spidata->emr_node_array))
			kmalloc(sizeof(*(spidata->emr_node_array)) *
				SPIEPA_TEMP_ARRAY_SIZE, GFP_KERNEL);
		if (!spidata->emr_node_array) {
			ret = -ENOMEM;
			goto err_emr_array;
		}
	}

	if (!spidata->tmp_node_array) {
		spidata->tmp_node_array = (typeof(spidata->tmp_node_array))
			kmalloc(sizeof(*(spidata->tmp_node_array)) *
				SPIEPA_TEMP_ARRAY_SIZE, GFP_KERNEL);
		if (!spidata->tmp_node_array) {
			ret = -ENOMEM;
			goto err_tmp_array;
		}
	}

	return 0;

err_tmp_array:
	kfree(spidata->emr_node_array);
	spidata->emr_node_array = NULL;
err_emr_array:
	kfree(spidata->to_user_array);
	spidata->to_user_array = NULL;
err_user_array:
	kfree(spidata->event_array);
	spidata->event_array = NULL;

	return ret;
}

static void spiepa_free_queue(void)
{
	if (spidata->tmp_node_array) {
		kfree(spidata->tmp_node_array);
		spidata->tmp_node_array = NULL;
	}

	if (spidata->emr_node_array) {
		kfree(spidata->emr_node_array);
		spidata->emr_node_array = NULL;
	}

	if (spidata->to_user_array) {
		kfree(spidata->to_user_array);
		spidata->to_user_array = NULL;
	}

	if (spidata->event_array) {
		kfree(spidata->event_array);
		spidata->event_array = NULL;
	}
}

static int spiepa_open(struct inode *inode, struct file *filp)
{
	if (!spidata)
		return -ENODEV;

	mutex_lock(&g_lock);

	spidata->users++;
	nonseekable_open(inode, filp);

	dev_info(&spidata->spi->dev,
		 "users count %d\n", spidata->users);

	mutex_unlock(&g_lock);

	return 0;
}

static int spiepa_release(struct inode *inode, struct file *filp)
{
	struct graphic_info *pgi;

	if (!spidata)
		return -ENODEV;

	mutex_lock(&g_lock);
	spidata->users--;
	if (!spidata->kreader ||
	    (spidata->users > 0)) {
		dev_info(&spidata->spi->dev,
			 "users count %d\n", spidata->users);
		mutex_unlock(&g_lock);
		return 0;
	}

	pgi = &spidata->gi;

	mutex_lock(&pgi->lock);
	if (pgi->pregions) {
		memset(pgi->pregions, 0, sizeof(*(pgi->pregions)));
	}
	mutex_unlock(&pgi->lock);

	spiepa_take_update_right();

	spidata->active = false;
	spidata->state = SPIEPA_STATE_NONE;

	if (spidata->kreader) {
		up(&spidata->event_sema);
		wait_for_completion(&reader_done);
		spidata->kreader = NULL;
	}

	if (spidata->spi)
		spidata->speed_hz = spidata->spi->max_speed_hz;

	mutex_unlock(&g_lock);

	return 0;
}

static int spiepa_set_region(struct spiepa_region *pr)
{
	struct graphic_info *pgi = &spidata->gi;
	int i, j;

	if (((pr->x < 0) || (pr->x >= pgi->max_width)) ||
	    ((pr->y < 0) || (pr->y >= pgi->max_height)) ||
	    ((pr->x + pr->w) > pgi->max_width) ||
	    ((pr->y + pr->h) > pgi->max_height)) {
		dev_err(&spidata->spi->dev,
			"Invalid region (%d,%d,%d,%d)\n",
			pr->x, pr->y, pr->w, pr->h);
		return -EINVAL;
	}

	if (pr->flag == 0) {
		/* disabled region */
		for (i = pr->y; i < (pr->y + pr->h); i++) {
			for (j = pr->x; j < (pr->x + pr->w); j++) {
#ifdef SPIEPA_BITMAP
				set_bit(j + pgi->row_start_table[i],
					spidata->region_map);
#else
				spidata->region_map[j +
					pgi->row_start_table[i]] = 1;
#endif
			}
		}
	} else if (pr->flag == 1) {
		/* enabled region */
		for (i = pr->y; i < (pr->y + pr->h); i++) {
			for (j = pr->x; j < (pr->x + pr->w); j++) {
#ifdef SPIEPA_BITMAP
				clear_bit(j + pgi->row_start_table[i],
					  spidata->region_map);
#else
				spidata->region_map[j +
					pgi->row_start_table[i]] = 0;
#endif
			}
		}
	}

	return 0;
}

static int spiepa_get_properties(void)
{
	struct device_node *node = NULL;
	struct graphic_info *pgi = &spidata->gi;
	int ret;
	__u32 value;

	if ((pgi->max_width > 0) &&
	    (pgi->max_height > 0) &&
	    (pgi->max_emr_x > 0) &&
	    (pgi->max_emr_y > 0)) {
		goto done;
	}

	node = spidata->spi->dev.of_node;

	ret = of_property_read_u32(node,
				     SPIEPA_EMR_WIDTH_PROPERTY,
				     &value);
	if (ret < 0) {
		dev_err(&spidata->spi->dev,
			"Get emr width failed,%d\n", ret);
		return ret;
	}
	pgi->max_emr_x = value;

	ret = of_property_read_u32(node,
				     SPIEPA_EMR_HEIGHT_PROPERTY,
				     &value);
	if (ret < 0) {
		dev_err(&spidata->spi->dev,
			"Get emr height failed,%d\n", ret);
		return ret;
	}
	pgi->max_emr_y = value;

	ret = of_property_read_u32(node,
				     SPIEPA_EPD_WIDTH_PROPERTY,
				     &value);
	if (ret < 0) {
		dev_err(&spidata->spi->dev,
			"Get epd width failed,%d\n", ret);
		return ret;
	}
	pgi->max_width = value;

	ret = of_property_read_u32(node,
				     SPIEPA_EPD_HEIGHT_PROPERTY,
				     &value);
	if (ret < 0) {
		dev_err(&spidata->spi->dev,
			"Get epd height failed,%d\n", ret);
		return ret;
	}
	pgi->max_height = value;

	dev_info(&spidata->spi->dev,
		 "emr(%d,%d) epd(%d,%d)\n",
		 pgi->max_emr_x, pgi->max_emr_y,
		 pgi->max_width, pgi->max_height);

done:
	return 0;
}

static long spiepa_ioctl(struct file *filp,
			 unsigned int cmd,
			 unsigned long value)
{
	struct task_struct *task;
	u32 save, tmp;
	int ret, i, xy[2], len;
	char epa_version[64];
	struct graphic_info *pgi;
#ifdef SPIEPA_SCHED_RR
	struct sched_param param;
#endif

	pgi = &spidata->gi;

	switch (cmd) {
	case SPIEPA_IOC_PREPARE:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_PREPARE\n");
#endif
		mutex_lock(&g_lock);
		ret = spiepa_get_properties();
		if (ret < 0) {
			spidata->state = SPIEPA_STATE_DISABLED;
			mutex_unlock(&g_lock);

			return ret;
		}

		ret = spiepa_alloc_map();
		if (ret < 0) {
			spidata->state = SPIEPA_STATE_DISABLED;
			mutex_unlock(&g_lock);

			return ret;
		}

		ret = spiepa_alloc_queue();
		if (ret < 0) {
			spidata->state = SPIEPA_STATE_DISABLED;
			mutex_unlock(&g_lock);

			return ret;
		}
		mutex_unlock(&g_lock);

		return ret;
	case SPIEPA_IOC_START:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_START\n");
#endif
		if (!spidata->status)
			return -ENODEV;

		mutex_lock(&g_lock);

		if (spidata->kreader) {
			mutex_unlock(&g_lock);
			return -EBUSY;
		}

		if (!spidata->tx_buffer) {
			spidata->tx_buffer = kmalloc(MAX_BUFFER_SZ, GFP_KERNEL);
			if (!spidata->tx_buffer) {
				mutex_unlock(&g_lock);
				return -ENOMEM;
			}
		}

		if (!spidata->rx_buffer) {
			spidata->rx_buffer = kmalloc(MAX_BUFFER_SZ, GFP_KERNEL);
			if (!spidata->rx_buffer) {
				kfree(spidata->tx_buffer);
				spidata->tx_buffer = NULL;
				mutex_unlock(&g_lock);
				return -ENOMEM;
			}
		}

		spidata->pressure = 0;
		spidata->x = spidata->y = -1;

		save = spidata->spi->mode;
		tmp = save & ~SPI_MODE_MASK;
		spidata->spi->mode = (u16)tmp;
		ret = spi_setup(spidata->spi);
		if (ret < 0) {
			spidata->spi->mode = save;
			mutex_unlock(&g_lock);
			return ret;
		}

		sema_init(&spidata->event_sema, 0);
		spidata->event_head = 0;
		spidata->event_tail = 0;
		spidata->to_user_head = 0;
		spidata->to_user_tail = 0;
		spidata->active = true;

		init_completion(&reader_done);
		task = kthread_run(spiepa_reader,
				   NULL,
				   "spiepa-rd");
		if (IS_ERR(task)) {
			spidata->spi->mode = save;
			spidata->active = false;
			spidata->state = SPIEPA_STATE_NONE;
			mutex_unlock(&g_lock);
			return PTR_ERR(task);
		}
		spidata->kreader = task;

#ifdef SPIEPA_SCHED_RR
		memset(&param, 0, sizeof(param));
		param.sched_priority = MAX_RT_PRIO - 2;
		ret = sched_setscheduler(task, SCHED_RR, &param);
		if (ret < 0) {
			dev_info(&spidata->spi->dev,
				 "set scheduler for writer failed,%d\n", ret);
			ret = 0;
		}

#endif
		spidata->state = SPIEPA_STATE_WORKING;

		mutex_unlock(&g_lock);
		break;
	case SPIEPA_IOC_STOP:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_STOP\n");
#endif
		spiepa_take_update_right();
		mutex_lock(&g_lock);

		spidata->active = false;
		spidata->state = SPIEPA_STATE_NONE;

		mutex_unlock(&g_lock);

		mutex_lock(&pgi->lock);
		if (pgi->pregions) {
			memset(pgi->pregions, 0, sizeof(*(pgi->pregions)));
		}
		mutex_unlock(&pgi->lock);

		mutex_lock(&g_lock);

		if (spidata->kreader) {
			up(&spidata->event_sema);
			wait_for_completion(&reader_done);
			spidata->kreader = NULL;
		}

		mutex_unlock(&g_lock);

		break;
	case SPIEPA_IOC_SPENTYPE:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_SPENTYPE: %d\n", (int)value);
#endif
		mutex_lock(&pgi->lock);
		pgi->pen_type = value;
		mutex_unlock(&pgi->lock);
		break;
	case SPIEPA_IOC_GPENTYPE:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_GPENTYPE\n");
#endif
		if ((void *)value == NULL)
			return -EINVAL;
		ret = copy_to_user((void *)value,
				   &pgi->pen_type,
				   sizeof(pgi->pen_type));
		if (ret < 0) {
			dev_err(&spidata->spi->dev,
				"copy pen type failed,%d\n", ret);
			return ret;
		}
		break;
	case SPIEPA_IOC_SPENWIDTH:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_SPENWIDTH: %d\n", (int)value);
#endif
		if ((value <= 0) ||
		    (value > pgi->max_pen_width))
			return -EINVAL;

		mutex_lock(&pgi->lock);
		pgi->pen_width = value;
		mutex_unlock(&pgi->lock);
		break;
	case SPIEPA_IOC_GPENWIDTH:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_GPENWIDTH\n");
#endif
		if ((void *)value == NULL)
			return -EINVAL;
		ret = copy_to_user((void *)value,
				   &pgi->pen_width,
				   sizeof(pgi->pen_width));
		if (ret < 0) {
			dev_err(&spidata->spi->dev,
				"copy pen width failed,%d\n", ret);
			return ret;
		}
		break;
	case SPIEPA_IOC_SGRAY:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_SGRAY: %d\n", (int)value);
#endif
		mutex_lock(&pgi->lock);
		pgi->gray_scale = value;
		mutex_unlock(&pgi->lock);
		break;
	case SPIEPA_IOC_GGRAY:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_GGRAY\n");
#endif
		if ((void *)value == NULL)
			return -EINVAL;
		ret = copy_to_user((void *)value,
				   &pgi->gray_scale,
				   sizeof(pgi->gray_scale));
		if (ret < 0) {
			dev_err(&spidata->spi->dev,
				"copy gray failed,%d\n", ret);
			return ret;
		}
		break;
	case SPIEPA_IOC_SREGION:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_SREGION\n");
#endif
		len = 0;
		ret = copy_from_user(&len, (void *)value, sizeof(len));
		if (ret < 0) {
			dev_err(&spidata->spi->dev,
				"Copy regions len from user failed,%d\n", ret);
			return ret;
		}

		if ((len <= 0) ||
		    (len > SPIEPA_MAX_REGION)) {
			return -EINVAL;
		}

		mutex_lock(&g_lock);

		ret = copy_from_user(pgi->pregions_tmp, (void *)value,
				     sizeof(len) +
				     len * sizeof(struct spiepa_region));
		if (ret < 0) {
			dev_err(&spidata->spi->dev,
				"Copy regions from user failed,%d\n", ret);
			mutex_unlock(&g_lock);
			return ret;
		}

		if ((len == 1) &&
		    (pgi->pregions_tmp->regions[0].flag == (__u16)0xff)) {
#ifdef SPIEPA_DEBUG
			dev_info(&spidata->spi->dev,
				 "taking update right,0xff\n");
#endif
			spidata->state = SPIEPA_STATE_DISABLED;

#ifdef SPIEPA_DEBUG
			dev_info(&spidata->spi->dev,
				 "state=%d\n", spidata->state);
#endif

			spiepa_take_update_right();

			mutex_unlock(&g_lock);
			return 0;
		} else if ((len == 1) &&
			   (pgi->pregions_tmp->regions[0].flag ==
			    (__u16)0x55)) {
#ifdef SPIEPA_DEBUG
			dev_info(&spidata->spi->dev,
				 "giving update right,0x55\n");
#endif
			spidata->state = SPIEPA_STATE_WORKING;
#ifdef SPIEPA_DEBUG
			dev_info(&spidata->spi->dev,
				 "state=%d\n", spidata->state);
#endif

			spiepa_give_update_right();

			mutex_unlock(&g_lock);
			return 0;
		}

		if (!(pgi->pregions->len)) {
			memset(spidata->region_map, 0,
			       spidata->region_size);
			for (i = 0; i < pgi->pregions_tmp->len; i++) {
#ifdef SPIEPA_DEBUG
				dev_info(&spidata->spi->dev,
					 "region (%d,%d,%d,%d,%d)\n",
					 (int)(pgi->pregions_tmp->regions[i].flag),
					 (int)(pgi->pregions_tmp->regions[i].x),
					 (int)(pgi->pregions_tmp->regions[i].y),
					 (int)(pgi->pregions_tmp->regions[i].w),
					 (int)(pgi->pregions_tmp->regions[i].h));
#endif
				spiepa_set_region(&pgi->pregions_tmp->
						  regions[i]);
			}
			memcpy(pgi->pregions, pgi->pregions_tmp,
			       sizeof(len) +
			       len * sizeof(struct spiepa_region));
		} else if (memcmp(pgi->pregions,
				  pgi->pregions_tmp,
				  sizeof(len) +
				  len * sizeof(struct spiepa_region))) {
			memset(spidata->region_map, 0,
			       spidata->region_size);

			for (i = 0; i < pgi->pregions_tmp->len; i++) {
#ifdef SPIEPA_DEBUG
				dev_info(&spidata->spi->dev,
					 "region (%d,%d,%d,%d,%d)\n",
					 (int)(pgi->pregions_tmp->regions[i].flag),
					 (int)(pgi->pregions_tmp->regions[i].x),
					 (int)(pgi->pregions_tmp->regions[i].y),
					 (int)(pgi->pregions_tmp->regions[i].w),
					 (int)(pgi->pregions_tmp->regions[i].h));
#endif
					spiepa_set_region(&pgi->pregions_tmp->
							  regions[i]);
			}

			memcpy(pgi->pregions, pgi->pregions_tmp,
			       sizeof(len) +
			       len * sizeof(struct spiepa_region));
		}

		mutex_unlock(&g_lock);
		break;
	case SPIEPA_IOC_GVERSION:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_GVERSION\n");
#endif
		if (!spidata->status)
			return -ENODEV;

		memset(epa_version, 0, sizeof(epa_version));
		ret = spiepa_ask_version(epa_version, sizeof(epa_version));
		if (ret > 0) {
			ret = copy_to_user((void *)value,
				     epa_version,
				     strlen(epa_version));
			if (ret < 0)
				return ret;
		} else {
			return -EFAULT;
		}
		break;
	case SPIEPA_IOC_EPDRIGHT:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_EPDRIGHT\n");
#endif
		if (value == 0) {
			/* give */
			spiepa_give_update_right();
		} else if (value == 1) {
			/* take */
			spiepa_take_update_right();
		}
		break;
	case SPIEPA_IOC_CORRECT:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_CORRECT\n");
#endif
		ret = copy_from_user(xy, (void *)value, sizeof(xy));
		if (ret < 0) {
			dev_err(&spidata->spi->dev,
				"Copy xy from user failed,%d\n", ret);
			return ret;
		}
		pgi->delta_x = xy[0];
		pgi->delta_y = xy[1];
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "deltaX=%d,deltaY=%d\n",
			 pgi->delta_x, pgi->delta_y);
#endif
		break;
	case SPIEPA_IOC_CLEAR:
#ifdef SPIEPA_DEBUG
		dev_info(&spidata->spi->dev,
			 "SPIEPA_IOC_CLEAR\n");
#endif
		break;
	default:
		break;
	}

	return 0;
}

static ssize_t spiepa_read(struct file *file, char __user *buf,
			size_t count, loff_t *off)
{
	struct spiepa_usr_data *data = NULL;
	struct spiepa_node *sn;
	int len = 0, ret, idx;

	if (count < sizeof(*data))
		return 0;

	if (spidata->to_user_head ==
	    spidata->to_user_tail) {
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			ret = wait_event_interruptible(spidata->wait,
						       spidata->to_user_head !=
						       spidata->to_user_tail);
			if (ret)
				return ret;
		}
	}

	len = count / sizeof(*data);
	data = (typeof(data))buf;
	mutex_lock(&spidata->to_user_lock);
	while (spidata->to_user_head != spidata->to_user_tail) {
		if (len == 0)
			break;
		idx = spidata->to_user_head;
		sn = &spidata->to_user_array[idx];
		data->flag = sn->flag;
		data->x = sn->x;
		data->y = sn->y;
		data->p = sn->p;
		data->gray = sn->gray;
		data++;
		len--;
		spidata->to_user_head = (idx + 1) % SPIEPA_QUEUE_SIZE;
	}
	mutex_unlock(&spidata->to_user_lock);

	return (count / sizeof(*data) - len) * sizeof(*data);
}

static unsigned int spiepa_poll(struct file *file,
				poll_table *wait)
{
	unsigned int mask = POLLWRNORM;

	poll_wait(file, &spidata->wait, wait);

	if (spidata->to_user_head !=
	    spidata->to_user_tail)
		mask |= POLLIN;

	return mask;
}

static const struct file_operations spi_epa_fops = {
	.owner =	THIS_MODULE,
	.open =		spiepa_open,
	.read =		spiepa_read,
	.poll =		spiepa_poll,
	.unlocked_ioctl = spiepa_ioctl,
	.release =	spiepa_release,
	.llseek =	no_llseek,
};

static int spiepa_parse_dt(struct spiepa_data *data)
{
	struct device_node *node = data->spi->dev.of_node;
	enum of_gpio_flags flags;
	int ret;

	spidata->sw_gpio = of_get_gpio_flags(node, 0, &flags);
	if (spidata->sw_gpio < 0) {
		dev_err(&data->spi->dev,
			"Get switch gpio failed,%d\n",
			spidata->sw_gpio);
		return spidata->sw_gpio;
	}
	spidata->sw_active_low = flags & OF_GPIO_ACTIVE_LOW;

	ret = devm_gpio_request(&spidata->spi->dev,
				spidata->sw_gpio,
				"spiepa-switch");
	if (ret < 0) {
		dev_err(&spidata->spi->dev,
			"Request spiepa switch gpio failed,%d\n",
			ret);
		return ret;
	}

	gpio_direction_output(spidata->sw_gpio,
			      spidata->sw_active_low ? 0 : 1);
	gpio_export(spidata->sw_gpio, false);

	spidata->test_gpio = of_get_gpio(node, 1);
	if (spidata->test_gpio < 0) {
		dev_err(&data->spi->dev,
			"Get test gpio failed,%d\n",
			spidata->test_gpio);
		return spidata->test_gpio;
	}

	ret = devm_gpio_request(&spidata->spi->dev,
				spidata->test_gpio,
				"spiepa-test");
	if (ret < 0) {
		dev_err(&spidata->spi->dev,
			"Request spiepa test gpio failed,%d\n",
			ret);
		return ret;
	}

	gpio_direction_output(spidata->test_gpio, 1);
	gpio_export(spidata->test_gpio, false);

	return 0;
}

static struct class *spiepa_class;

static const struct of_device_id spi_epa_dt_ids[] = {
	{ .compatible = "ratta,epa" },
	{},
};
MODULE_DEVICE_TABLE(of, spi_epa_dt_ids);

static int spi_epa_probe(struct spi_device *spi)
{
	int status = 0, i;
	struct device *dev;

	if (spidata) {
		dev_err(&spi->dev, "Too many spi epa devices.\n");
		return -EBUSY;
	}

	/*
	 * spidev should never be referenced in DT without a specific
	 * compatbile string, it is a Linux implementation thing
	 * rather than a description of the hardware.
	 */
	if (spi->dev.of_node && !of_match_device(spi_epa_dt_ids, &spi->dev)) {
		dev_err(&spi->dev, "buggy DT: spiepa listed directly in DT\n");
		WARN_ON(spi->dev.of_node &&
			!of_match_device(spi_epa_dt_ids, &spi->dev));
	}

	/* Allocate driver data */
	spidata = kzalloc(sizeof(*spidata), GFP_KERNEL);
	if (!spidata)
		return -ENOMEM;

	spidata->status = true;
	spidata->gi.pregions = NULL;
	spidata->gi.pregions_tmp = NULL;
	sema_init(&spidata->event_sema, 0);
	mutex_init(&spidata->to_user_lock);
	mutex_init(&spidata->event_lock);
	mutex_init(&spidata->buf_lock);
	init_waitqueue_head(&spidata->wait);
	mutex_init(&g_lock);

	/* Initialize the driver data */
	spidata->spi = spi;
	spidata->speed_hz = spi->max_speed_hz;
	spin_lock_init(&spidata->spi_lock);
	spin_lock_init(&spidata->compute_lock);

	status = spiepa_parse_dt(spidata);
	if (status < 0) {
		goto err_dt;
	}

	status = spiepa_init_graphic();
	if (status < 0) {
		goto err_dt;
	}

	for (i = 0; i < SPIEPA_PRESSURE_SIZE; i++) {
		spidata->pressures[i].min = 200;
		spidata->pressures[i].max = 1800;
	}
	spidata->pressures[SPIEPA_PEN_G12].min = 200;
	spidata->pressures[SPIEPA_PEN_G12].max = 3400;

	spidata->devt = MKDEV(spiepa_major, 0);
	dev = device_create(spiepa_class, &spi->dev, spidata->devt,
			    spidata, "spiepa");
	status = PTR_ERR_OR_ZERO(dev);
	if (status != 0) {
		goto err_device;
	}

	spi_set_drvdata(spi, spidata);

	return status;

err_device:
	spiepa_deinit_graphic();
err_dt:
	kfree(spidata);
	spidata = NULL;

	return status;
}

static int spi_epa_remove(struct spi_device *spi)
{
	mutex_lock(&g_lock);
	if (spidata->users > 0) {
		mutex_unlock(&g_lock);
		return -EBUSY;
	}

	device_destroy(spiepa_class, spidata->devt);

	if (spidata->tx_buffer) {
		kfree(spidata->tx_buffer);
		spidata->tx_buffer = NULL;
	}

	if (spidata->rx_buffer) {
		kfree(spidata->rx_buffer);
		spidata->rx_buffer = NULL;
	}

	spiepa_free_map();
	spiepa_free_queue();
	spiepa_deinit_graphic();

	kfree(spidata);
	spidata = NULL;

	mutex_unlock(&g_lock);

	return 0;
}

static int spiepa_of_device_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/* must call put_device() when done with returned spi_device device */
struct spi_device *spiepa_find_device_by_node(struct device_node *node)
{
	struct device *dev = bus_find_device(&spi_bus_type, NULL, node,
						spiepa_of_device_match);
	return dev ? to_spi_device(dev) : NULL;
}
EXPORT_SYMBOL_GPL(spiepa_find_device_by_node);

static struct spi_driver spi_epa_driver = {
	.driver = {
		.name =		"spiepa",
		.owner =	THIS_MODULE,
		.of_match_table = spi_epa_dt_ids,
	},
	.probe =	spi_epa_probe,
	.remove =	spi_epa_remove,
};

static int __init spi_epa_init(void)
{
	int status;

	spiepa_major = register_chrdev(SPIEPA_MAJOR, "spiepa", &spi_epa_fops);
	if (spiepa_major < 0)
		return spiepa_major;

	spiepa_class = class_create(THIS_MODULE, "spiepa");
	if (IS_ERR(spiepa_class)) {
		unregister_chrdev(spiepa_major, spi_epa_driver.driver.name);
		return PTR_ERR(spiepa_class);
	}

	status = spi_register_driver(&spi_epa_driver);
	if (status < 0) {
		unregister_chrdev(spiepa_major, spi_epa_driver.driver.name);
	}

	return status;
}
module_init(spi_epa_init);

static void __exit spi_epa_exit(void)
{
	spi_unregister_driver(&spi_epa_driver);
	class_destroy(spiepa_class);
	unregister_chrdev(spiepa_major, spi_epa_driver.driver.name);
	spiepa_major = 0;
}
module_exit(spi_epa_exit);

MODULE_DESCRIPTION("Ratta Epa SPI device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spiepa");
