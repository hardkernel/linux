//[*]--------------------------------------------------------------------------------------------------[*]
//
//
// 
//  ODROID IOBOARD Board : IOBOARD SST25WF020 SPI Flash driver (charles.park)
//  2013.08.28
// 
//
//[*]--------------------------------------------------------------------------------------------------[*]
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <mach/regs-pmu.h>
#include <plat/gpio-cfg.h>
#include <linux/spi/spi.h>

#if defined(CONFIG_OF)
    #include <linux/of.h>
    #include <linux/of_gpio.h>
#endif

#include <linux/platform_data/spi-s3c64xx.h>
#include "ioboard-spi.h"

//[*]--------------------------------------------------------------------------------------------------[*]
#define SST25WF020_NAME 	    "ioboard-spi"

#define SPI_BUS_NUM     1
#define SPI_CS_NUM      0
#define SPI_CS_GPIO     190         /* GPA2.5(#190) */
#define SPI_WP_GPIO     21          /* GPX1.5(#21) */
#define SPI_BUS_SPEED   10000000

static  struct  ioboard_spi     *ioboard_spi;

//[*]--------------------------------------------------------------------------------------------------[*]
#define SPI_MAX_BUFFER_SIZE     32

#define SPI_MAX_RETRY_CNT       100

// Read memory
#define CMD_READ                0x03
#define CMD_HIGH_SPEED_READ     0x0B
#define CMD_READ_STATUS_REG     0x05

#define SPI_READ_HIGHSPEED

// Erase memory
#define CMD_ERASE_4KB           0x20
#define CMD_ERASE_32KB          0x52
#define CMD_ERASE_64KB          0xD8
//#define CMD_ERASE_ALL           0x60
#define CMD_ERASE_ALL           0xC7

// BYTE write command
#define CMD_BYTE_WRITE          0x02
// Auto address increment programming (AAI word programming)
#define CMD_WRITE_WORD          0xAD

#define CMD_WRITE_ENABLE        0x06
#define CMD_WRITE_DISABLE       0x04

#define CMD_WRITE_STATUS_REG    0x01
    // BUSY
    //  1   : internal write operation is in progress
    //  0   : no internal write operation is in progress
    #define STATUS_REG_BUSY     0x01    // (Read)
    // WEL
    //  1   : Device is memory write enabled
    //  0   : Device is not memory write enabled
    #define STATUS_REG_WEL      0x02    // (Read)
    // BP1 BP0
    //  0   0   : none protected memory
    //  0   1   : 030000H-03FFFFH protected
    //  1   0   : 020000H-03FFFFH protected
    //  1   1   : 000000H-03FFFFH protected (Power-up default value)
    #define STATUS_REG_BP0      0x04    // (R/W)
    #define STATUS_REG_BP1      0x08    // (R/W)
    // BPL
    //  1   : BP1 and BP0 are read-only bits
    //  0   : BP1 and BP0 are read/writable (Power-up default value)
    #define STATUS_REG_BPL      0x80    // (R/W)

//[*]--------------------------------------------------------------------------------------------------[*]
int wait_until_ready         (struct spi_device *spi, unsigned int check_usec)
{
    unsigned char   tx, status, retry_cnt = 0;

    do  {
        tx = CMD_READ_STATUS_REG; // ReaD Status Register(RDSR)
        
        spi_write_then_read(spi, &tx, sizeof(tx), &status, sizeof(status));
        udelay(check_usec);
        
        if(retry_cnt++ >= SPI_MAX_RETRY_CNT)    {
            pr_err("%s : timeout error!!\n", __func__); return  -1;
        }
    }   while(status & STATUS_REG_BUSY);
    
    return  0;
}

//[*]--------------------------------------------------------------------------------------------------[*]
int is_ready    (struct spi_device *spi)
{
    unsigned char   tx, status;

    tx = CMD_READ_STATUS_REG; // ReaD Status Register(RDSR)
        
    spi_write_then_read(spi, &tx, sizeof(tx), &status, sizeof(status));

    // 0 : not ready, 1 : ready
    return  (status & STATUS_REG_BUSY) ? 0 : 1;
}

//[*]--------------------------------------------------------------------------------------------------[*]
static int ioboard_spi_write_enable     (struct spi_device *spi, unsigned char enable)
{
    unsigned char   tx;
    
    tx = enable ? CMD_WRITE_ENABLE : CMD_WRITE_DISABLE;

    spi_write_then_read(spi, &tx, sizeof(tx), NULL, 0);

    if(wait_until_ready(spi, 100))  {
        pr_err("%s : error!!\n", __func__);     return  -1;
    }   
    return  0;
}

//[*]--------------------------------------------------------------------------------------------------[*]
int ioboard_spi_erase        (struct spi_device *spi, unsigned int addr, unsigned char mode)
{
    unsigned char   tx[4], tx_size;

    switch(mode) {
        case CMD_ERASE_4KB :    case CMD_ERASE_32KB:    case CMD_ERASE_64KB:
            tx[0] = mode;  tx_size = 4;
            tx[1] = (addr >> 16) & 0xFF;   
            tx[2] = (addr >>  8) & 0xFF;
            tx[3] = (addr      ) & 0xFF;
            break;
        case CMD_ERASE_ALL :
            tx[0] = mode;  tx_size = 1;
            break;
        default :    
            pr_err("%s : Unknown command 0x%02X\n", __func__, mode);
            return  -1;
    }
    // write enable
    if(ioboard_spi_write_enable(spi, 1) != 0)   return  -1;     // write enable

    spi_write_then_read(spi, &tx[0], tx_size, NULL, 0);     

    if(mode == CMD_ERASE_ALL)   mdelay(200);
    else                        mdelay(100);

    if(wait_until_ready(spi, 100))              return  -1;

    // write disable
    if(ioboard_spi_write_enable(spi, 0) != 0)   return  -1;     // write disable

    return  0;
}

//[*]--------------------------------------------------------------------------------------------------[*]
static int ioboard_spi_byte_write   (struct spi_device *spi, unsigned int addr, unsigned char wdata)
{
    unsigned char   tx[5];

    // write enable
    if(ioboard_spi_write_enable(spi, 1) != 0)   return  -1;     // write enable

    tx[0] = CMD_BYTE_WRITE;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >>  8) & 0xFF;
    tx[3] = (addr      ) & 0xFF;
    tx[4] = wdata;    
    
    spi_write_then_read(spi, &tx[0], 5, NULL, 0);

    if(wait_until_ready(spi, 100))     {
        pr_err("%s : error!!\n", __func__);     return  -1;
    }

    if(ioboard_spi_write_enable(spi, 0) != 0)   return  -1;     // write disable

    return  0;
}
#ifdef SPI_READ_HIGHSPEED
//[*]--------------------------------------------------------------------------------------------------[*]
static int ioboard_spi_read_memory_highspeed    (struct spi_device *spi, unsigned int addr, unsigned char *rdata, unsigned int size)
{
    unsigned char   tx[5]; // read memory highspeed
    
    tx[0] = CMD_HIGH_SPEED_READ;    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >>  8) & 0xFF;    tx[3] = (addr      ) & 0xFF;
    tx[4] = 0x00; // Dummy cycle

    spi_write_then_read(spi, &tx[0], sizeof(tx), &rdata[0], size);

    if(wait_until_ready(spi, 100))     {
        pr_err("%s : error!!\n", __func__);     return  -1;
    }

    if(size == 0)   {
        pr_err("%s : read size is 0. error!!\n", __func__); return -1;
    }

    return  0;
}
#else
//[*]--------------------------------------------------------------------------------------------------[*]
static int ioboard_spi_read_memory              (struct spi_device *spi, unsigned int addr, unsigned char *rdata, unsigned int size)
{
    unsigned char   tx[4]; // read memory
    
    tx[0] = CMD_READ;               tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >>  8) & 0xFF;    tx[3] = (addr      ) & 0xFF;

    spi_write_then_read(spi, &tx[0], sizeof(tx), &rdata[0], size);

    if(wait_until_ready(spi, 100))     {
        pr_err("%s : error!!\n", __func__);     return  -1;
    }

    if(size == 0)   {
        pr_err("%s : read size is 0. error!!\n", __func__); return -1;
    }

    return  0;
}
#endif

//[*]--------------------------------------------------------------------------------------------------[*]
int ioboard_spi_read    (struct spi_device *spi, unsigned int addr, unsigned char *rdata, unsigned int size)
{
    unsigned int    offset = 0, mok, na;
    
    mok     = size / SPI_MAX_BUFFER_SIZE;
    na      = size % SPI_MAX_BUFFER_SIZE;
    
    while(mok)  {
#ifdef SPI_READ_HIGHSPEED
        ioboard_spi_read_memory_highspeed(spi, addr + offset, &rdata[offset], SPI_MAX_BUFFER_SIZE);
#else
        ioboard_spi_read_memory(spi, addr + offset, &rdata[offset], SPI_MAX_BUFFER_SIZE);
#endif
        offset += SPI_MAX_BUFFER_SIZE;      mok = mok - 1;
    }
    
    if(na)  {
#ifdef SPI_READ_HIGHSPEED
        ioboard_spi_read_memory_highspeed(spi, addr + offset, &rdata[offset], na);
#else
        ioboard_spi_read_memory(spi, addr + offset, &rdata[offset], na);
#endif
    }
    
    return  0;
}

//[*]--------------------------------------------------------------------------------------------------[*]
int ioboard_spi_write   (struct spi_device *spi, unsigned int addr, unsigned char *wdata, unsigned int size)
{
    unsigned int    i;

    if(size == 0)   {
        pr_err("%s : write size is 0. error!!\n", __func__); return -1;
    }
    
    for(i = 0; i < size; i++)   ioboard_spi_byte_write(spi, addr + i, wdata[i]);

    return  0;
}

//[*]--------------------------------------------------------------------------------------------------[*]
static int ioboard_spi_id_read      (struct spi_device *spi)
{
    unsigned char   tx[] = { 0xAB, 0x00, 0x00, 0x00 };
    unsigned char   rx[2];
    
    spi_write_then_read(spi, &tx, sizeof(tx), &rx, sizeof(rx));
    
    if(wait_until_ready(spi, 100) != 0)     {
        pr_err("%s : ready error!\n", __func__);    return  -1;
    }
    
    pr_info("%s : id read ! [0x%02X][0x%02X]\n", __func__, rx[0], rx[1]);

    return  0;
}

//[*]--------------------------------------------------------------------------------------------------[*]
static int ioboard_spi_write_status_reg (struct spi_device *spi, unsigned char status)
{
    unsigned char   tx[2];
    
    // write enable
    if(ioboard_spi_write_enable(spi, 1) != 0)   return  -1;     // write enable
        
    tx[0] = CMD_WRITE_STATUS_REG;
    tx[1] = status;
    spi_write_then_read(spi, &tx, sizeof(tx), NULL, 0);
    
    if(ioboard_spi_write_enable(spi, 0) != 0)   return  -1;     // write disable
        
    return  0;
}

//[*]--------------------------------------------------------------------------------------
static void ioboard_spi_wp_disable   (unsigned char disable)
{
    // write protection port init
    if(gpio_request(ioboard_spi->wp_gpio, "ioboard-spi-wp"))  {
        pr_err("%s : %s gpio request error!(GPIO[%d])\n", __func__, "ioboard-spi-wp", ioboard_spi->wp_gpio);
        ioboard_spi->wp_gpio = -1;
    }
    else    {
        s3c_gpio_setpull(ioboard_spi->wp_gpio, S3C_GPIO_PULL_NONE);

        // write-protection disable 
        if(disable) gpio_direction_output(ioboard_spi->wp_gpio, 1);   // write enable
        else        gpio_direction_output(ioboard_spi->wp_gpio, 0);   // write disable
    }
}

//[*]--------------------------------------------------------------------------------------------------[*]
#ifdef CONFIG_ODROID_EXYNOS5_IOBOARD_DEBUG
static void ioboard_spi_test        (struct spi_device *spi)
{
    unsigned char   wdata[64], rdata[64];
    unsigned int    addr = 0, i;
    
    for(i = 0; i < sizeof(wdata); i++)  wdata[i] = i;
    
    // read before erase
    memset(rdata, 0x00, sizeof(rdata));
    ioboard_spi_read (spi, addr, &rdata[0], sizeof(rdata));

    pr_info("\nread before erase : addr = 0x%04X", addr);
    for(i = 0; i < sizeof(rdata); i++)  {
        if(!(i % 16))   pr_info("\n");
        pr_info("[0x%02X] ", rdata[i]);
    }
    
    ioboard_spi_erase(spi, addr, CMD_ERASE_ALL);
    
    // read after erase
    memset(rdata, 0x00, sizeof(rdata));
    ioboard_spi_read (spi, addr, &rdata[0], sizeof(rdata));

    pr_info("\nread after erase : addr = 0x%04X", addr);
    for(i = 0; i < sizeof(rdata); i++)  {
        if(!(i % 16))   pr_info("\n");
        pr_info("[0x%02X] ", rdata[i]);
    }
    
    ioboard_spi_write(spi, addr, &wdata[0], sizeof(wdata));

    // read after erase
    memset(rdata, 0x00, sizeof(rdata));
    ioboard_spi_read (spi, 0, &rdata[0], sizeof(rdata));

    pr_info("\nread after write : addr = 0x%04X", addr);
    for(i = 0; i < sizeof(rdata); i++)  {
        if(!(i % 16))   pr_info("\n");
        pr_info("[0x%02X] ", rdata[i]);
    }
}
#endif

//[*]--------------------------------------------------------------------------------------------------[*]
static int ioboard_spi_probe        (struct spi_device *spi)
{
	int     ret = 0;

    // spi write protection disable
    ioboard_spi_wp_disable(1);

    if(ioboard_spi->wp_gpio == -1)  {
        ret = -1;
        goto    err;
    }

	dev_set_drvdata(&spi->dev, ioboard_spi);

    // read chip id
    if((ret = ioboard_spi_id_read(spi)) < 0)
        goto    err;

    // software protection disable
    if((ret = ioboard_spi_write_status_reg(spi, 0))< 0)
        goto    err;

    if((ret = ioboard_spi_misc_probe(spi)) < 0) {
        pr_err("%s : misc driver added fail!\n", __func__);
        goto    err;
    }

#if defined(CONFIG_ODROID_EXYNOS5_IOBOARD_DEBUG)
    ioboard_spi_test(spi);
#endif

    pr_info("\n=================== %s ===================\n\n", __func__);

    return  0;

err:
    pr_err("\n=================== %s FAIL! ===================\n\n", __func__);

    return  ret;
}

//[*]--------------------------------------------------------------------------------------------------[*]
static int ioboard_spi_remove   (struct spi_device *spi)
{
    if(ioboard_spi->wp_gpio != -1)
        gpio_free(ioboard_spi->wp_gpio);

    ioboard_spi_misc_remove(&ioboard_spi->spi->dev);

    pr_info("\n=================== %s ===================\n\n", __func__);

	return 0;
}

//[*]--------------------------------------------------------------------------------------------------[*]
static struct spi_driver ioboard_spi_driver = {
	.driver = {
		.name	= SST25WF020_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= ioboard_spi_probe,
	.remove	= ioboard_spi_remove,
};

//[*]--------------------------------------------------------------------------------------------------[*]
/*
 * Module init and exit
 */
//[*]--------------------------------------------------------------------------------------------------[*]
static int __init ioboard_spi_init(void)
{
    struct  spi_master  *spi_master;
    struct  device      *pdev;
    char    buff[64];
    int     status = 0;

    if(!(spi_master = spi_busnum_to_master(SPI_BUS_NUM))) {
        pr_err("%s : spi_busnum_to_master(%d) returned NULL!\n", __func__, SPI_BUS_NUM);
        return  -1;
    }

	if(!(ioboard_spi = kzalloc(sizeof(struct ioboard_spi), GFP_KERNEL)))	{
		pr_err("%s : ioboard-spi struct malloc error!\n", __func__);
		return	-ENOMEM;
	}

    if(!(ioboard_spi->spi = spi_alloc_device(spi_master))) {
        put_device(&spi_master->dev);
        pr_err("%s : spi_alloc_device() failed!\n", __func__);
        return  -1;
    }

    ioboard_spi->spi->chip_select = SPI_CS_NUM;
    ioboard_spi->wp_gpio = SPI_WP_GPIO;

    /* Check whether this SPI bus.cs is already claimed */
    snprintf(buff, sizeof(buff), "%s.%u",
        dev_name(&ioboard_spi->spi->master->dev), ioboard_spi->spi->chip_select);

    if ((pdev = bus_find_device_by_name(ioboard_spi->spi->dev.bus, NULL, buff))) {
        /* We are not going to use this spi_device, so free it */
        device_del(pdev);

        /*
         * There is already a device configured for this bus.cs
         * It is okay if it us, otherwise complain and fail.
        */
        if (pdev->driver && pdev->driver->name &&
            strcmp("ioboard-spi", pdev->driver->name)) {
            pr_err("%s : Driver [%s] already registered for %s\n",
                __func__, pdev->driver->name, buff);
            status = -1;
        }
    }

    if(status == 0) {
        ioboard_spi->spi->max_speed_hz = SPI_BUS_SPEED;
        ioboard_spi->spi->mode = SPI_MODE_0;
        ioboard_spi->spi->bits_per_word = 8;
        ioboard_spi->spi->irq = -1;
        ioboard_spi->spi->controller_state = NULL;

        ioboard_spi->csinfo =
            kzalloc(sizeof(struct s3c64xx_spi_csinfo), GFP_KERNEL);

        ioboard_spi->csinfo->line = SPI_CS_GPIO;

        ioboard_spi->spi->controller_data = (void *)ioboard_spi->csinfo;
        strlcpy(ioboard_spi->spi->modalias, "ioboard-spi", SPI_NAME_SIZE);
        status = spi_add_device(ioboard_spi->spi);
    }

    if (status < 0) {
        spi_dev_put(ioboard_spi->spi);
        pr_err("%s : spi_add_device() failed: %d\n", __func__, status);
        return  status;
    }

    put_device(&spi_master->dev);

	return spi_register_driver(&ioboard_spi_driver);
}
module_init(ioboard_spi_init);

//[*]--------------------------------------------------------------------------------------------------[*]
static void __exit ioboard_spi_exit(void)
{
    if(ioboard_spi->csinfo)  {
        if(ioboard_spi->csinfo->line)
            gpio_free(ioboard_spi->csinfo->line);

        kfree(ioboard_spi->csinfo);
    }

	if (ioboard_spi->spi) {
		device_del(&ioboard_spi->spi->dev);
		kfree(ioboard_spi->spi);
	}
	kfree(ioboard_spi);

	spi_unregister_driver(&ioboard_spi_driver);
}
module_exit(ioboard_spi_exit);

//[*]--------------------------------------------------------------------------------------------------[*]
MODULE_DESCRIPTION("IOBOARD driver for ODROIDXU-Dev board");
MODULE_AUTHOR("Hard-Kernel");
MODULE_LICENSE("GPL");

//[*]--------------------------------------------------------------------------------------------------[*]
//[*]--------------------------------------------------------------------------------------------------[*]
