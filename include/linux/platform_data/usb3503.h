#ifndef USB3503_H
#define USB3503_H

#define USB3503_I2C_NAME "usb3503"
#define HUB_TAG "usb3503: "

#define CFG1_REG		0x06
#define CFG1_SELF_BUS_PWR	(0x1 << 7)

#define SP_ILOCK_REG		0xE7
#define SPILOCK_CONNECT_N	(0x1 << 1)
#define SPILOCK_CONFIG_N	(0x1 << 0)

#define CFGP_REG		0xEE
#define CFGP_CLKSUSP		(0x1 << 7)

#define PDS_REG			0x0A
#define PDS_PORT1		(0x1 << 1)
#define PDS_PORT2		(0x1 << 2)
#define PDS_PORT3		(0x1 << 3)


enum usb3503_mode {
	USB3503_MODE_UNKNOWN,
	USB3503_MODE_HUB,
	USB3503_MODE_STANDBY,
};

struct usb3503_platform_data {
	char initial_mode;
	int (*reset_n)(int);
	int (*register_hub_handler)(void (*)(void), void *);
	int (*port_enable)(int, int);
};

struct usb3503_hubctl {
	int mode;
	int (*reset_n)(int);
	int (*port_enable)(int, int);
	struct i2c_client *i2c_dev;
};
#endif
