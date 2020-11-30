#ifndef __UAPI_MXC_EPDC_HELPER_H__
#define __UAPI_MXC_EPDC_HELPER_H__
#include <linux/types.h>

struct write_disables;
#define EPDC_HELPER_MAGIC 'E'

#define EPDC_HELPER_IOC_START		_IO(EPDC_HELPER_MAGIC,	1)
#define EPDC_HELPER_IOC_STOP		_IO(EPDC_HELPER_MAGIC,	2)
#define EPDC_HELPER_IOC_SREGION		_IOW(EPDC_HELPER_MAGIC, 3, struct write_disables)
#define EPDC_HELPER_IOC_SPENTYPE	_IOW(EPDC_HELPER_MAGIC, 4, int)
#define EPDC_HELPER_IOC_GPENTYPE	_IOR(EPDC_HELPER_MAGIC, 5, int)
#define EPDC_HELPER_IOC_SPENWIDTH	_IOW(EPDC_HELPER_MAGIC, 6, int)
#define EPDC_HELPER_IOC_GPENWIDTH	_IOR(EPDC_HELPER_MAGIC, 7, int)
#define EPDC_HELPER_IOC_SPENCOLOR	_IOW(EPDC_HELPER_MAGIC, 8, int)
#define EPDC_HELPER_IOC_GPENCOLOR	_IOR(EPDC_HELPER_MAGIC, 9, int)
#define EPDC_HELPER_IOC_LOCK		_IO(EPDC_HELPER_MAGIC, 10)
#define EPDC_HELPER_IOC_UNLOCK		_IO(EPDC_HELPER_MAGIC, 11)
#define EPDC_HELPER_IOC_SCORRECT	_IOW(EPDC_HELPER_MAGIC, 12, int[2])
#define EPDC_HELPER_IOC_GCORRECT	_IOR(EPDC_HELPER_MAGIC, 13, int[2])

struct epdc_helper_data {
	int	x;
	int	y;
	int	p;
	__u8	pen;
	__u8	color;
};

struct write_disable {
	int	x;
	int	y;
	int	w;
	int	h;
};

#define MAX_DISABLE_COUNT	100
struct write_disables {
	int			count;
	struct write_disable	disables[MAX_DISABLE_COUNT];
};

#endif
