#ifndef __SPIEPA_UAPI_H__
#define __SPIEPA_UAPI_H__

#include <linux/types.h>

#define SPIEPA_MAX_REGION 100
struct spiepa_regions;

#define SPIEPA_IOC_MAGIC 'F'

#define SPIEPA_IOC_PREPARE	_IO(SPIEPA_IOC_MAGIC, 1)
#define SPIEPA_IOC_START	_IO(SPIEPA_IOC_MAGIC, 2)
#define SPIEPA_IOC_STOP		_IO(SPIEPA_IOC_MAGIC, 3)
#define SPIEPA_IOC_SPENTYPE	_IOW(SPIEPA_IOC_MAGIC, 4, __u32)
#define SPIEPA_IOC_GPENTYPE	_IOR(SPIEPA_IOC_MAGIC, 5, __u32)
#define SPIEPA_IOC_SPENWIDTH	_IOW(SPIEPA_IOC_MAGIC, 6, __u32)
#define SPIEPA_IOC_GPENWIDTH	_IOR(SPIEPA_IOC_MAGIC, 7, __u32)
#define SPIEPA_IOC_SGRAY	_IOW(SPIEPA_IOC_MAGIC, 8, __u32)
#define SPIEPA_IOC_GGRAY	_IOR(SPIEPA_IOC_MAGIC, 9, __u32)
#define SPIEPA_IOC_SREGION	_IOW(SPIEPA_IOC_MAGIC, 10, struct spiepa_regions)
#define SPIEPA_IOC_CREGION	_IO(SPIEPA_IOC_MAGIC, 11)
#define SPIEPA_IOC_GVERSION	_IOW(SPIEPA_IOC_MAGIC, 12, char[64])
#define SPIEPA_IOC_EPDRIGHT	_IOW(SPIEPA_IOC_MAGIC, 13, __u32)
#define SPIEPA_IOC_CORRECT	_IOW(SPIEPA_IOC_MAGIC, 14, int[2])
#define SPIEPA_IOC_CLEAR	_IO(SPIEPA_IOC_MAGIC, 15)
#define SPIEPA_IOC_DUMMY	_IOW(SPIEPA_IOC_MAGIC, 15, __u32)

struct spiepa_region {
	__u16 flag;
	__s16 x;
	__s16 y;
	__s16 w;
	__s16 h;
};

struct spiepa_regions {
	int len;
	struct spiepa_region regions[100];
};

struct spiepa_usr_data {
	__u8 flag;
	__u8 gray;
	__s16 x;
	__s16 y;
	__s16 p;
};

#endif
