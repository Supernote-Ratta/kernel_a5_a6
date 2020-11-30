
#include <osl.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_linux.h>
#define  GPIO_BCM_WL_POWER_ON 	((3-1)*32+29) 

#define GPIO_BCM_WL_REG_ON ((4-1)*32+26)
#define GPIO_BCM_WL_HOST_WAKE ((3-1)*32+24)


#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 9, 0)
#include <linux/of_gpio.h>
#endif
#include <linux/gpio.h>

#include <linux/fs.h>
#include <linux/string.h>
#include <linux/err.h>

#ifdef CONFIG_MACH_ODROID_4210
#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/sdhci.h>
#include <plat/devs.h>
#define	sdmmc_channel	s3c_device_hsmmc0
#endif


struct wifi_platform_data dhd_wlan_control = {0};

#ifdef CUSTOMER_OOB
uint bcm_wlan_get_oob_irq(void)
{
	uint host_oob_irq = 0;

#ifdef CONFIG_MACH_ODROID_4210
	printf("GPIO(WL_HOST_WAKE) = EXYNOS4_GPX0(7) = %d\n", EXYNOS4_GPX0(7));
	host_oob_irq = gpio_to_irq(EXYNOS4_GPX0(7));
	gpio_direction_input(EXYNOS4_GPX0(7));
#else
	printf("GPIO(WL_HOST_WAKE)  = %d\n", GPIO_BCM_WL_HOST_WAKE);
	host_oob_irq = gpio_to_irq(GPIO_BCM_WL_HOST_WAKE);
	gpio_direction_input(GPIO_BCM_WL_HOST_WAKE);
#endif

	printf("host_oob_irq: %d\n", host_oob_irq);

	return host_oob_irq;
}

uint bcm_wlan_get_oob_irq_flags(void)
{
	uint host_oob_irq_flags = 0;

#ifdef CONFIG_MACH_ODROID_4210
#ifdef HW_OOB
	host_oob_irq_flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE;
#else
	host_oob_irq_flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_SHAREABLE;
#endif
#else
#ifdef HW_OOB
	host_oob_irq_flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE;
#else
	host_oob_irq_flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_SHAREABLE;
#endif

#endif


	printf("host_oob_irq_flags=0x%X\n", host_oob_irq_flags);

	return host_oob_irq_flags;
}
#endif

#define GPIO3_DR   0x20A4000
#define GPIO5_DR   0x20AC000

void  clr_gpio_bt(void)
{
	static void *p_gpj0_base;
	int datalong;
	int i;
	int data;

	datalong=1;
	p_gpj0_base = ioremap(GPIO3_DR, datalong*4);
	if (!p_gpj0_base)
		return;
	data = readl(p_gpj0_base);
	printk("WYM GPIO3_DR-%x\n",data);
	data = data&0xebbfffff;  //
	writel(data,p_gpj0_base);	  //low level interrupt
	iounmap(p_gpj0_base);
	p_gpj0_base = NULL;

	p_gpj0_base = ioremap(GPIO5_DR, datalong*4);
	if (!p_gpj0_base)
		return;
	data = readl(p_gpj0_base);
	printk("WYM GPIO3_DR-%x\n",data);
	data=data&0xffffe97f;  //
	writel(data,p_gpj0_base);	  //low level interrupt
	iounmap(p_gpj0_base);
	p_gpj0_base = NULL;
}

int bcm_wlan_set_power(bool on)
{
	int err = 0;

	if (on) {
		printf("======== PULL WL_REG_ON HIGH! ========\n");
#ifdef CONFIG_MACH_ODROID_4210
		err = gpio_set_value(EXYNOS4_GPK1(0), 1);
#else
		gpio_set_value(GPIO_BCM_WL_REG_ON, 0);
		msleep(10);
		gpio_set_value(GPIO_BCM_WL_POWER_ON, 1);
		msleep(200);

		gpio_set_value(GPIO_BCM_WL_REG_ON, 1);
#endif

		/* Lets customer power to get stable */
		mdelay(200);
	} else {
		printf("======== PULL WL_REG_ON LOW! ========\n");
#ifdef CONFIG_MACH_ODROID_4210
		err = gpio_set_value(EXYNOS4_GPK1(0), 0);
#else
		gpio_set_value(GPIO_BCM_WL_REG_ON, 0);

		gpio_set_value(GPIO_BCM_WL_POWER_ON, 0);
		mdelay(200);
		msleep(500);
#endif

	}

	return err;
}

extern void wifi_card_detect(bool);
int bcm_wlan_set_carddetect(bool present)
{
	int err = 0;

	if (present) {
		printf("======== Card detection to detect SDIO card! ========\n");
#ifdef CONFIG_MACH_ODROID_4210
		err = sdhci_s3c_force_presence_change(&sdmmc_channel, 1);
#else
		wifi_card_detect(present);
#endif

	} else {
		printf("======== Card detection to remove SDIO card! ========\n");
#ifdef CONFIG_MACH_ODROID_4210
		err = sdhci_s3c_force_presence_change(&sdmmc_channel, 0);
#else
		wifi_card_detect(present);
#endif

	}

	return err;
}

#ifndef GET_MAC_FROM_UBOOT
#define CUSTOMER_MAC_FILE "/productinfo/wifimac.txt"
#endif

#ifdef GET_MAC_FROM_UBOOT
extern int platform_get_mac_addr(unsigned char *buf);
static unsigned char brcm_mac_addr[IFHWADDRLEN] = { 0, 0x90, 0x4c, 0, 0, 0 };

int bcm_wlan_get_flash_mac(unsigned char *buf)
{
	return platform_get_mac_addr(buf);
}
#endif

int bcm_wlan_get_mac_address(unsigned char *buf)
{
	int err = 0;

	printf("======== %s ========\n", __FUNCTION__);
#ifdef EXAMPLE_GET_MAC
	/* EXAMPLE code */
	{
		struct ether_addr ea_example = {{0x00, 0x11, 0x22, 0x33, 0x44, 0xFF}};
		bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
	}
#endif /* EXAMPLE_GET_MAC */

	if (!buf)
		return -EFAULT;

#ifdef GET_MAC_FROM_UBOOT
	uint rand_mac;
	err = bcm_wlan_get_flash_mac(brcm_mac_addr);
	if(err < 0)
		return -EFAULT;
	if ((brcm_mac_addr[4] == 0) && (brcm_mac_addr[5] == 0)) {
		prandom_seed((uint)jiffies);
		rand_mac = prandom_u32();
		brcm_mac_addr[3] = (unsigned char)rand_mac;
		brcm_mac_addr[4] = (unsigned char)(rand_mac >> 8);
		brcm_mac_addr[5] = (unsigned char)(rand_mac >> 16);
	}
	memcpy(buf, brcm_mac_addr, IFHWADDRLEN);
#else
	struct file *fp      = NULL;
    	char macbuffer[18] = {0};
    	mm_segment_t oldfs = {0};
	char buffer[18] = {0};
	char randommac[3] = {0};
	unsigned char octet[ETHER_ADDR_LEN] = {0}; 
	char* filepath = CUSTOMER_MAC_FILE;
	char* nvpath = "/system/etc/wifi/nvram.txt";
	char dummy_mac[ETHER_ADDR_LEN]		= { 0x98, 0xFF, 0xD0, 0xc5, 0x12, 0x38 };
	int ret =0;
	fp = filp_open(filepath, O_RDONLY, 0); //try to open wifi_macaddr
	if (IS_ERR(fp)) {
		fp = filp_open(filepath, O_RDWR | O_CREAT, 0666);
		if (IS_ERR(fp)){
			printk("[WIFI] %s: File open error\n", filepath);
			return -1;
		}
		oldfs = get_fs();
		set_fs(get_ds());
		get_random_bytes(randommac, 3);
		//srandom32((uint)jiffies);	
		//randommac[0] = (random32()%(0x68-0x5b+1))+0x5b;	
		//randommac[0] = (randommac[0]%(0x68-0x5b+1))+0x5b;
		sprintf(macbuffer,"%02X:%02X:%02X:%02X:%02X:%02X",0x98,0xFF,0xD0,randommac[0],randommac[1],randommac[2]);
		printk("[WIFI] The Random Generated MAC ID : %s\n", macbuffer);
		if(fp->f_mode & FMODE_WRITE){
			ret = fp->f_op->write(fp, (const char *)macbuffer, sizeof(macbuffer), &fp->f_pos);
			if(ret < 0)
				printk("[WIFI] Mac address [%s] Failed to write into File: %s\n", macbuffer, filepath);
			else
				printk("[WIFI] Mac address [%s] written into File: %s successfully\n", macbuffer, filepath);
		}
		set_fs(oldfs);
		ret = kernel_read(fp, 0, buffer, 18);
	} else {
		ret = kernel_read(fp, 0, buffer, 18);
		buffer[17] ='\0';	 // to prevent abnormal string display when mac address is displayed on the screen. 
		printk("Read MAC : [%s] [%d] \r\n" , buffer, strncmp(buffer , "00:00:00:00:00:00" , 17));
		if(strncmp(buffer, "00:00:00:00:00:00" , 17) == 0) {
			sprintf(macbuffer,"%02X:%02X:%02X:%02X:%02X:%02X",
                                       dummy_mac[0],dummy_mac[1],dummy_mac[2],
                                       dummy_mac[3],dummy_mac[4],dummy_mac[5]);
			if(fp->f_mode & FMODE_WRITE){
				ret = fp->f_op->write(fp, (const char *)macbuffer, sizeof(macbuffer), &fp->f_pos);
			} else {
				filp_close(fp, NULL);
				return -EINVAL;//use default mac in nvram.txt
			}
		}
	}
	if(ret){
		sscanf(buffer,"%02X:%02X:%02X:%02X:%02X:%02X",
			   &octet[0], &octet[1], &octet[2], 
			   &octet[3], &octet[4], &octet[5]);
		printk("%s:@@@ MAC read value is %02x:%02x:%02X:%02x:%02x:%02x @@@\n",__func__,
                          octet[0],octet[1],octet[2],
                          octet[3],octet[4],octet[5]);
		memcpy(buf, (unsigned char *)octet,ETHER_ADDR_LEN);
		printk("%s:@@@ MAC dump %02x:%02x:%02X:%02x:%02x:%02x addr=%p @@@\n",__func__,
                          buf[0],buf[1],buf[2],
                          buf[3],buf[4],buf[5],buf);
	} else {
		printk("%s: Reading from the '%s' returns 0 bytes\n", __func__, filepath);
		if (fp){
		   filp_close(fp, NULL);
		}
		return -EINVAL;//use default mac in nvram.txt
	}
	if (fp)
		filp_close(fp, NULL);
#endif
	return err;
}

/*Please cut the below to platform arch/arcm/xxx/board_xxx.c */
static unsigned char wifi_mac_addr[IFHWADDRLEN] = { 0, 0x90, 0x4c, 0, 0, 0 };
int platform_get_mac_addr(unsigned char *buf)
{
	if(!buf)
	    return -1;
	memcpy(buf, wifi_mac_addr, IFHWADDRLEN);	
	return 0;
}

static int __init wifi_mac_addr_setup(char *str)
{
	char macstr[IFHWADDRLEN*3];
	char *macptr = macstr;
	char *token;
	int i = 0;

	if (!str)
		return 0;
	printk("wlan MAC = %s\n", str);
	if (strlen(str) >= sizeof(macstr))
		return 0;
	strlcpy(macstr, str, sizeof(macstr));

	while ((token = strsep(&macptr, ":")) != NULL) {
		unsigned long val;
		int res;

		if (i >= IFHWADDRLEN)
			break;
		res = kstrtoul(token, 0x10, &val);
		if (res < 0)
			break;
		wifi_mac_addr[i++] = (u8)val;
	}

	if (i < IFHWADDRLEN && strlen(macstr)==IFHWADDRLEN*2) {
		/* try again with wrong format (sans colons) */
		u64 mac;
		if (kstrtoull(macstr, 0x10, &mac) < 0)
			return 0;
		for (i=0; i<IFHWADDRLEN; i++)
			wifi_mac_addr[IFHWADDRLEN-1-i] = (u8)((0xFF)&(mac>>(i*8)));
	}

	return i==IFHWADDRLEN ? 1:0;
}
__setup("wifimacaddr=", wifi_mac_addr_setup);
/*Please cut the above to platform arch/arcm/xxx/board_xxx.c */

#ifdef CONFIG_DHD_USE_STATIC_BUF
extern void *bcmdhd_mem_prealloc(int section, unsigned long size);
void* bcm_wlan_prealloc(int section, unsigned long size)
{
	void *alloc_ptr = NULL;
	alloc_ptr = bcmdhd_mem_prealloc(section, size);
	if (alloc_ptr) {
		printf("success alloc section %d, size %ld\n", section, size);
		if (size != 0L)
			bzero(alloc_ptr, size);
		return alloc_ptr;
	}
	printf("can't alloc section %d\n", section);
	return NULL;
}
#endif

#if !defined(WL_WIRELESS_EXT)
struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];	/* ISO 3166-1 country abbreviation */
	char custom_locale[WLC_CNTRY_BUF_SZ];	/* Custom firmware locale */
	int32 custom_locale_rev;		/* Custom local revisin default -1 */
};
#endif

static struct cntry_locales_custom brcm_wlan_translate_custom_table[] = {
	/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XT", 49},  /* Universal if Country code is unknown or empty */
	{"US", "US", 0},
};

static void *bcm_wlan_get_country_code(char *ccode)
{
	struct cntry_locales_custom *locales;
	int size;
	int i;

	if (!ccode)
		return NULL;

	locales = brcm_wlan_translate_custom_table;
	size = ARRAY_SIZE(brcm_wlan_translate_custom_table);

	for (i = 0; i < size; i++)
		if (strcmp(ccode, locales[i].iso_abbrev) == 0)
			return &locales[i];
	return NULL;
}

int bcm_wlan_set_plat_data(void) {
	int ret;
	printk("======== %s ========\n", __FUNCTION__);

#if 1
	ret = gpio_request(GPIO_BCM_WL_POWER_ON, "bcm power on");
	if (ret < 0)
	{
		printk("gpio_request(GPIO_BCM_WL_POWER_ON) error!\n");
	}
	gpio_direction_output(GPIO_BCM_WL_POWER_ON, 1);
	gpio_set_value(GPIO_BCM_WL_POWER_ON, 1);
#endif
	
	ret = gpio_request(GPIO_BCM_WL_REG_ON, "bcm wl reg on");
	if (ret < 0)
	{
		printk("gpio_request(GPIO_BCM_WL_REG_ON) error!\n");
	}
	gpio_direction_output(GPIO_BCM_WL_REG_ON, 1);
	gpio_set_value(GPIO_BCM_WL_REG_ON, 1);


	dhd_wlan_control.set_power = bcm_wlan_set_power;
	dhd_wlan_control.set_carddetect = bcm_wlan_set_carddetect;
	dhd_wlan_control.get_mac_addr = bcm_wlan_get_mac_address;
#ifdef CONFIG_DHD_USE_STATIC_BUF
	dhd_wlan_control.mem_prealloc = bcm_wlan_prealloc;
#endif
	dhd_wlan_control.get_country_code = bcm_wlan_get_country_code;
	return 0;
}

