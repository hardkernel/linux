/*
 * drivers/amlogic/cec/hdmi_ao_cec.c
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
*/
#include <linux/version.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/timer.h>

#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/amlogic/tvin/tvin.h>
#include <linux/wakelock_android.h>

#include <linux/amlogic/hdmi_tx/hdmi_tx_cec_20.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_irq.h>
#include "hdmi_ao_cec.h"
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/amlogic/pm.h>
#include <linux/of_address.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend aocec_suspend_handler;
#endif


#define CEC_FRAME_DELAY     msecs_to_jiffies(400)
#define CEC_DEV_NAME        "cec"

#define DEV_TYPE_TX     4
#define DEV_TYPE_RX     0

#define CEC_EARLY_SUSPEND   (1 << 0)
#define CEC_DEEP_SUSPEND    (1 << 1)

/* global struct for tx and rx */
struct ao_cec_dev {
    unsigned long dev_type;
    unsigned int port_num;
    unsigned int arc_port;
    unsigned int hal_flag;
    unsigned int phy_addr;
    unsigned int port_seq;
    unsigned long irq_cec;
    void __iomem *exit_reg;
    void __iomem *cec_reg;
    void __iomem *hdmi_rxreg;
    struct hdmitx_dev *tx_dev;
    struct workqueue_struct *cec_thread;
    struct device *dbg_dev;
    struct delayed_work cec_work;
    struct completion rx_ok;
    struct completion tx_ok;
    spinlock_t cec_reg_lock;
    struct mutex cec_mutex;
#ifdef CONFIG_PM
    int cec_suspend;
#endif
    struct vendor_info_data v_data;
    struct cec_global_info_t cec_info;
};

static int phy_addr_test;

/* from android cec hal */
enum {
    HDMI_OPTION_WAKEUP = 1,
    HDMI_OPTION_ENABLE_CEC = 2,
    HDMI_OPTION_SYSTEM_CEC_CONTROL = 3,
    HDMI_OPTION_SET_LANG = 5,
};

static struct ao_cec_dev *cec_dev;
static int cec_tx_result;

static unsigned char rx_msg[MAX_MSG];
static unsigned char rx_len;
static unsigned int  new_msg;
bool cec_msg_dbg_en = 0;

#define CEC_ERR(format, args...)                \
    {if (cec_dev->dbg_dev)                  \
        dev_err(cec_dev->dbg_dev, "%s(): " format, __func__, ##args);  \
    }

#define CEC_INFO(format, args...)               \
    {if (cec_msg_dbg_en && cec_dev->dbg_dev)        \
        dev_info(cec_dev->dbg_dev, "%s(): " format, __func__, ##args); \
    }

static unsigned char msg_log_buf[128] = { 0 };

#define waiting_aocec_free() \
    do {\
        unsigned long cnt = 0;\
        while (readl(cec_dev->cec_reg + AO_CEC_RW_REG) & (1<<23)) {\
            if (3500 == cnt++) { \
                pr_info("waiting aocec free time out.\n");\
                break;\
            } \
        } \
    } while (0)

#define HR_DELAY(n)     (ktime_set(0, n * 1000 * 1000))
__u16 cec_key_map[160] = {
    KEY_ENTER, KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, 0 , 0 , 0 ,//0x00
    0 , KEY_HOMEPAGE , KEY_MENU, 0, 0, KEY_BACK, 0, 0,
    0 , 0, 0, 0, 0, 0, 0, 0,//0x10
    0 , 0, 0, 0, 0, 0, 0, 0,
    KEY_0 , KEY_1, KEY_2, KEY_3,KEY_4, KEY_5, KEY_6, KEY_7,//0x20
    KEY_8 , KEY_9, KEY_DOT, 0, 0, 0, 0, 0,
    KEY_CHANNELUP , KEY_CHANNELDOWN, KEY_CHANNEL, 0, 0, 0, 0, 0,//0x30
    0 , 0, 0, 0, 0, 0, 0, 0,
    
    KEY_POWER , KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_MUTE, KEY_PLAYPAUSE, KEY_STOP, KEY_PLAYPAUSE, KEY_RECORD,//0x40
    KEY_REWIND, KEY_FASTFORWARD, KEY_EJECTCD, KEY_NEXTSONG, KEY_PREVIOUSSONG, 0, 0, 0,
    0 , 0, 0, KEY_PROGRAM, 0, 0, 0, 0,//0x50
    0 , 0, 0, 0, 0, 0, 0, 0,
    KEY_PLAYCD, KEY_PLAYPAUSE, KEY_RECORD, KEY_PAUSECD, KEY_STOPCD, KEY_MUTE, 0, KEY_TUNER,//0x60
    0 , KEY_MEDIA, 0, 0, KEY_POWER, 0, 0, 0,
    0 , KEY_BLUE, KEY_RED, KEY_GREEN, KEY_YELLOW, 0, 0, 0,//0x70
    0 , 0, 0, 0, 0, 0, 0, 0x2fd,
    0 , 0, 0, 0, 0, 0, 0, 0,//0x80
    0 , 0, 0, 0, 0, 0, 0, 0,
    0 , KEY_EXIT, 0, 0, 0, 0, KEY_PVR, 0,//0x90  //samsung vendor buttons return and channel_list
    0 , 0, 0, 0, 0, 0, 0, 0,
};

struct hrtimer cec_key_timer;
static int last_key_irq = -1;
static int key_value = 1;
enum hrtimer_restart cec_key_up(struct hrtimer *timer)
{
    if (key_value == 1){
        input_event(cec_dev->cec_info.remote_cec_dev,
            EV_KEY, cec_key_map[last_key_irq], 0);
    }
    input_sync(cec_dev->cec_info.remote_cec_dev);
    CEC_INFO("last:%d up\n", cec_key_map[last_key_irq]);
    key_value = 2;

    return HRTIMER_NORESTART;
}

void cec_user_control_pressed_irq(unsigned char message_irq)
{
    if (message_irq < 160) {
        CEC_INFO("Key pressed: %d\n", message_irq);
        input_event(cec_dev->cec_info.remote_cec_dev, EV_KEY,
                cec_key_map[message_irq], key_value);
        input_sync(cec_dev->cec_info.remote_cec_dev);
        last_key_irq = message_irq;
        hrtimer_start(&cec_key_timer, HR_DELAY(200), HRTIMER_MODE_REL);
        CEC_INFO(":key map:%d\n", cec_key_map[message_irq]);
    }
}

void cec_user_control_released_irq(void)
{
    /*
     * key must be valid
     */
    if (last_key_irq != -1) {
        CEC_INFO("Key released: %d\n",last_key_irq);
        hrtimer_cancel(&cec_key_timer);
        input_event(cec_dev->cec_info.remote_cec_dev,
            EV_KEY, cec_key_map[last_key_irq], 0);
        input_sync(cec_dev->cec_info.remote_cec_dev);
        key_value = 1;
    }
}

void cec_set_reg_bits(unsigned int addr, unsigned int value,
    unsigned int offset, unsigned int len)
{
    unsigned int data32 = 0;

    data32 = readl(cec_dev->cec_reg + addr);
    data32 &= ~(((1 << len) - 1) << offset);
    data32 |= (value & ((1 << len) - 1)) << offset;
    writel(data32, cec_dev->cec_reg + addr);
}

unsigned int aocec_rd_reg(unsigned long addr)
{
    unsigned int data32;
    unsigned long flags;
    waiting_aocec_free();
    spin_lock_irqsave(&cec_dev->cec_reg_lock, flags);
    data32 = 0;
    data32 |= 0 << 16; /* [16]   cec_reg_wr */
    data32 |= 0 << 8; /* [15:8]   cec_reg_wrdata */
    data32 |= addr << 0; /* [7:0]   cec_reg_addr */
    writel(data32, cec_dev->cec_reg + AO_CEC_RW_REG);

    waiting_aocec_free();
    data32 = ((readl(cec_dev->cec_reg + AO_CEC_RW_REG)) >> 24) & 0xff;
    spin_unlock_irqrestore(&cec_dev->cec_reg_lock, flags);
    return data32;
} /* aocec_rd_reg */

void aocec_wr_reg(unsigned long addr, unsigned long data)
{
    unsigned long data32;
    unsigned long flags;
    waiting_aocec_free();
    spin_lock_irqsave(&cec_dev->cec_reg_lock, flags);
    data32 = 0;
    data32 |= 1 << 16; /* [16]   cec_reg_wr */
    data32 |= data << 8; /* [15:8]   cec_reg_wrdata */
    data32 |= addr << 0; /* [7:0]   cec_reg_addr */
    writel(data32, cec_dev->cec_reg + AO_CEC_RW_REG);
    spin_unlock_irqrestore(&cec_dev->cec_reg_lock, flags);
} /* aocec_wr_only_reg */

static void cec_enable_irq(void)
{
    cec_set_reg_bits(AO_CEC_INTR_MASKN, 0x6, 0, 3);
    CEC_INFO("enable:int mask:0x%x\n",
         readl(cec_dev->cec_reg + AO_CEC_INTR_MASKN));
}

static void cec_hw_buf_clear(void)
{
    aocec_wr_reg(CEC_RX_MSG_CMD, RX_DISABLE);
    aocec_wr_reg(CEC_TX_MSG_CMD, TX_ABORT);
    aocec_wr_reg(CEC_RX_CLEAR_BUF, 1);
    aocec_wr_reg(CEC_TX_CLEAR_BUF, 1);
    udelay(100);
    aocec_wr_reg(CEC_RX_CLEAR_BUF, 0);
    aocec_wr_reg(CEC_TX_CLEAR_BUF, 0);
    udelay(100);
    aocec_wr_reg(CEC_RX_MSG_CMD, RX_NO_OP);
    aocec_wr_reg(CEC_TX_MSG_CMD, TX_NO_OP);
}

void cec_logicaddr_clear(void)
{
    int i;
    for (i = 0; i < 5; i++) {
        aocec_wr_reg((CEC_LOGICAL_ADDR0 + i), 0);
        cec_dev->cec_info.log_addr[i] = 0;
        udelay(100);
    }
    cec_hw_buf_clear();
}

void cec_logicaddr_setByMask(unsigned int mask)
{
    int i, j;
    int reg = 0;
    int primary = -1;
    // ignore reserved device type
    const int device_types[5] = {CEC_RECORDING_DEVICE, 
                                 CEC_PLAYBACK_DEVICE, 
                                 CEC_TUNER_DEVICE,
                                 CEC_AUDIO_SYSTEM_DEVICE,
                                 CEC_DISPLAY_DEVICE|CEC_UNREGISTERED 
                                 };
    mask &= 0xffff;

    if (mask == 0) {
        cec_logicaddr_clear();
        return;
    }

    for (i = CEC_TV_ADDR; i <= CEC_UNREGISTERED_ADDR; i++) {
        if (reg > 4) {
            break;
        }
        if ((mask & 1<<i) == 1<<i) {
            for (j = 0; j <= sizeof(device_types); j++) {
                // Max. one of each type
                if (1<<i & device_types[j]) {
                    CEC_INFO("ADDING LA:0x%d reg:0x%d\n", i, 
                            (CEC_LOGICAL_ADDR0 + reg));
                    mask &= ~(mask & device_types[j]);
                    cec_dev->cec_info.log_addr[reg] = i;
                    cec_logicaddr_set(i, (CEC_LOGICAL_ADDR0 + reg));
                    if (primary == -1);
                        primary = i;
                        cec_logicaddr_config(primary, 1);
                    reg++;
                    break;
                }
            }
        }
    }
}

void cec_logicaddr_set(int logicaddr, int logreg)
{
    aocec_wr_reg(logreg, 0);
    cec_hw_buf_clear();
    aocec_wr_reg(logreg, (logicaddr & 0xf));
    udelay(100);
    aocec_wr_reg(logreg, (0x1 << 4) | (logicaddr & 0xf));
    if (cec_msg_dbg_en)
        CEC_INFO("set logical addr:0x%x\n",
            aocec_rd_reg(logreg));
}

static void cec_hw_reset(void)
{
    writel(0x1, cec_dev->cec_reg + AO_CEC_GEN_CNTL);
    /* Enable gated clock (Normal mode). */
    cec_set_reg_bits(AO_CEC_GEN_CNTL, 1, 1, 1);
    /* Release SW reset */
    udelay(100);
    cec_set_reg_bits(AO_CEC_GEN_CNTL, 0, 0, 1);

    /* Enable all AO_CEC interrupt sources */
    cec_set_reg_bits(AO_CEC_INTR_MASKN, 0x6, 0, 3);

    cec_logicaddr_set(cec_dev->cec_info.log_addr[0], CEC_LOGICAL_ADDR0);

    /* Cec arbitration 3/5/7 bit time set. */
    cec_arbit_bit_time_set(3, 0x118, 0);
    cec_arbit_bit_time_set(5, 0x000, 0);
    cec_arbit_bit_time_set(7, 0x2aa, 0);

    CEC_INFO("hw reset :logical addr:0x%x\n",
        aocec_rd_reg(CEC_LOGICAL_ADDR0));
}

void cec_rx_buf_clear(void)
{
    aocec_wr_reg(CEC_RX_CLEAR_BUF, 0x1);
    aocec_wr_reg(CEC_RX_CLEAR_BUF, 0x0);
    CEC_INFO("rx buf clean\n");
}

int cec_rx_buf_check(void)
{
    unsigned int rx_num_msg = aocec_rd_reg(CEC_RX_NUM_MSG);
    if (rx_num_msg)
        CEC_INFO("rx msg num:0x%02x\n", rx_num_msg);

    return rx_num_msg;
}

int cec_ll_rx(unsigned char *msg, unsigned char *len)
{
    int i;
    int ret = -1;
    int pos;
    int rx_stat;

    rx_stat = aocec_rd_reg(CEC_RX_MSG_STATUS);
    if ((RX_DONE != rx_stat) || (1 != aocec_rd_reg(CEC_RX_NUM_MSG))) {
        CEC_INFO("rx status:%x\n", rx_stat);
        writel((1 << 2), cec_dev->cec_reg + AO_CEC_INTR_CLR);
        aocec_wr_reg(CEC_RX_MSG_CMD,  RX_ACK_CURRENT);
        aocec_wr_reg(CEC_RX_MSG_CMD,  RX_NO_OP);
        return ret;
    }

    *len = aocec_rd_reg(CEC_RX_MSG_LENGTH) + 1;

    for (i = 0; i < (*len) && i < MAX_MSG; i++)
        msg[i] = aocec_rd_reg(CEC_RX_MSG_0_HEADER + i);

    ret = rx_stat;

    /* ignore ping message */
    if (cec_msg_dbg_en  == 1 && *len > 1) {
        pos = 0;
        pos += sprintf(msg_log_buf + pos,
            "CEC: rx msg len: %d   dat: ", *len);
        for (i = 0; i < (*len); i++)
            pos += sprintf(msg_log_buf + pos, "%02x ", msg[i]);
        pos += sprintf(msg_log_buf + pos, "\n");
        msg_log_buf[pos] = '\0';
        CEC_INFO("%s", msg_log_buf);
    }
    writel((1 << 2), cec_dev->cec_reg + AO_CEC_INTR_CLR);
    aocec_wr_reg(CEC_RX_MSG_CMD, RX_ACK_NEXT);
    aocec_wr_reg(CEC_RX_MSG_CMD, RX_NO_OP);
    return ret;
}

void cec_polling_online_dev(int log_addr, int *bool)
{
    unsigned int r;
    unsigned char msg[1];
    int retry = 5;

    msg[0] = (log_addr<<4) | log_addr;
    /* set broadcast address first */
    cec_logicaddr_set(0xf, CEC_LOGICAL_ADDR0);
    if (cec_msg_dbg_en == 1)
        CEC_INFO("CEC_LOGICAL_ADDR0:0x%i\n",
               aocec_rd_reg(CEC_LOGICAL_ADDR0));
    while (retry) {
        r = cec_ll_tx(msg, 1);
        if (r == CEC_FAIL_BUSY) {
            retry--;
            CEC_INFO("try log addr %x busy, retry:%d\n",
                   log_addr, retry);
            /*
             * try to reset CEC if tx busy is found
             */
            cec_hw_reset();
        } else
            break;
    }

    if (r == CEC_FAIL_NACK) {
        *bool = 0;
    } else if (r == CEC_FAIL_NONE) {
        *bool = 1;
    }
    CEC_INFO("CEC: poll online logic device: 0x%x BOOL: %d\n",
        log_addr, *bool);
}

/************************ cec arbitration cts code **************************/
/* using the cec pin as fiq gpi to assist the bus arbitration */

/* return value: 1: successful    0: error */
static int cec_ll_trigle_tx(const unsigned char *msg, int len)
{
    int i;
    unsigned int n;
    int pos;
    int reg;
    unsigned int j = 20;
    unsigned tx_stat;
    static int cec_timeout_cnt = 1;
	int flag = 0;

    while (1) {
        tx_stat = aocec_rd_reg(CEC_TX_MSG_STATUS);
        if (tx_stat != TX_BUSY)
            break;

		if (!flag && tx_stat == TX_BUSY) {
            CEC_INFO("TX is busy. Sending TX_ABORT\n");
			aocec_wr_reg(CEC_TX_MSG_CMD, TX_ABORT);
			flag = 1;
		}

        if (!(j--)) {
            CEC_INFO("TX is still busy. Sending TX_NO_OP\n");
            aocec_wr_reg(CEC_TX_MSG_CMD, TX_NO_OP);
            cec_timeout_cnt++;
            if (cec_timeout_cnt > 0x08) {
                cec_hw_reset();
                break;
            }
        }
        msleep(20);
    }

    reg = aocec_rd_reg(CEC_TX_MSG_STATUS);
    if (reg == TX_IDLE || reg == TX_DONE) {
        for (i = 0; i < len; i++)
            aocec_wr_reg(CEC_TX_MSG_0_HEADER + i, msg[i]);

        aocec_wr_reg(CEC_TX_MSG_LENGTH, len-1);
        aocec_wr_reg(CEC_TX_MSG_CMD, TX_REQ_CURRENT);

        if (cec_msg_dbg_en  == 1) {
            pos = 0;
            pos += sprintf(msg_log_buf + pos,
                       "CEC: tx msg len: %d   dat: ", len);
            for (n = 0; n < len; n++) {
                pos += sprintf(msg_log_buf + pos,
                           "%02x ", msg[n]);
            }

            pos += sprintf(msg_log_buf + pos, "\n");

            msg_log_buf[pos] = '\0';
            pr_info("%s", msg_log_buf);
        }
        cec_timeout_cnt = 0;
        return 0;
    }
    return -1;
}

void tx_irq_handle(void)
{
    unsigned tx_status = aocec_rd_reg(CEC_TX_MSG_STATUS);
    switch (tx_status) {
    case TX_DONE:
        aocec_wr_reg(CEC_TX_MSG_CMD, TX_NO_OP);
        cec_tx_result = 0;
        complete(&cec_dev->tx_ok);
        break;

    case TX_BUSY:
        CEC_ERR("TX_BUSY\n");
        break;

    case TX_ERROR:
        if (cec_msg_dbg_en  == 1)
            CEC_ERR("TX ERROR!!!\n");
        if (RX_ERROR == aocec_rd_reg(CEC_RX_MSG_STATUS)) {
            cec_hw_reset();
        } else {
            aocec_wr_reg(CEC_TX_MSG_CMD, TX_NO_OP);
        }
        cec_tx_result = 1;
        complete(&cec_dev->tx_ok);
        break;

    case TX_IDLE:
        break;
    default:
        break;
    }
    writel((1 << 1), cec_dev->cec_reg + AO_CEC_INTR_CLR);
}

/* Return value: < 0: fail, > 0: success */
int cec_ll_tx(const unsigned char *msg, unsigned char len)
{
    int ret = 0;
    int timeout = msecs_to_jiffies(1000);

    if (len == 0)
        return CEC_FAIL_NONE;

    mutex_lock(&cec_dev->cec_mutex);
    /*
     * do not send messanges if tv is not support CEC
     */
    ret = cec_ll_trigle_tx(msg, len);
    if (ret < 0) {
        /* we should increase send idx if busy */
        CEC_INFO("tx busy\n");
        mutex_unlock(&cec_dev->cec_mutex);
        return CEC_FAIL_BUSY;
    }
    cec_tx_result = 0;
    ret = wait_for_completion_timeout(&cec_dev->tx_ok, timeout);
    if (ret <= 0) {
        /* timeout of interrupt */
        ret = CEC_FAIL_OTHER;
        CEC_INFO("tx timeout\n");
    } else {
        if (cec_tx_result)
            ret = CEC_FAIL_NACK;
        else
            ret = CEC_FAIL_NONE;
    }
    mutex_unlock(&cec_dev->cec_mutex);

    return ret;
}

/* -------------------------------------------------------------------------- */
/* AO CEC0 config */
/* -------------------------------------------------------------------------- */
void ao_cec_init(void)
{
    unsigned long data32;
    unsigned int reg;
    /* Assert SW reset AO_CEC */
    reg = readl(cec_dev->cec_reg + AO_CRT_CLK_CNTL1);
    /* 24MHz/ (731 + 1) = 32786.885Hz */
    reg &= ~(0x7ff << 16);
    reg |= (731 << 16); /* divider from 24MHz */
    reg |= (0x1 << 26);
    reg &= ~(0x800 << 16);  /* select divider */
    writel(reg, cec_dev->cec_reg + AO_CRT_CLK_CNTL1);
    data32  = 0;
    data32 |= 0 << 1;   /* [2:1]    cntl_clk: */
                /* 0=Disable clk (Power-off mode); */
                /* 1=Enable gated clock (Normal mode); */
                /* 2=Enable free-run clk (Debug mode). */
    data32 |= 1 << 0;   /* [0]    sw_reset: 1=Reset */
    writel(data32, cec_dev->cec_reg + AO_CEC_GEN_CNTL);
    /* Enable gated clock (Normal mode). */
    cec_set_reg_bits(AO_CEC_GEN_CNTL, 1, 1, 1);
    /* Release SW reset */
    cec_set_reg_bits(AO_CEC_GEN_CNTL, 0, 0, 1);

    /* Enable all AO_CEC interrupt sources */
    cec_enable_irq();
}

void cec_arbit_bit_time_set(unsigned bit_set, unsigned time_set, unsigned flag)
{   /* 11bit:bit[10:0] */
    if (flag) {
        CEC_INFO("bit_set:0x%x;time_set:0x%x\n",
            bit_set, time_set);
    }

    switch (bit_set) {
    case 3:
        /* 3 bit */
        if (flag) {
            CEC_INFO("read 3 bit:0x%x%x\n",
                aocec_rd_reg(AO_CEC_TXTIME_4BIT_BIT10_8),
                aocec_rd_reg(AO_CEC_TXTIME_4BIT_BIT7_0));
        }
        aocec_wr_reg(AO_CEC_TXTIME_4BIT_BIT7_0, time_set & 0xff);
        aocec_wr_reg(AO_CEC_TXTIME_4BIT_BIT10_8, (time_set >> 8) & 0x7);
        if (flag) {
            CEC_INFO("write 3 bit:0x%x%x\n",
                aocec_rd_reg(AO_CEC_TXTIME_4BIT_BIT10_8),
                aocec_rd_reg(AO_CEC_TXTIME_4BIT_BIT7_0));
        }
        break;
        /* 5 bit */
    case 5:
        if (flag) {
            CEC_INFO("read 5 bit:0x%x%x\n",
                aocec_rd_reg(AO_CEC_TXTIME_2BIT_BIT10_8),
                aocec_rd_reg(AO_CEC_TXTIME_2BIT_BIT7_0));
        }
        aocec_wr_reg(AO_CEC_TXTIME_2BIT_BIT7_0, time_set & 0xff);
        aocec_wr_reg(AO_CEC_TXTIME_2BIT_BIT10_8, (time_set >> 8) & 0x7);
        if (flag) {
            CEC_INFO("write 5 bit:0x%x%x\n",
                aocec_rd_reg(AO_CEC_TXTIME_2BIT_BIT10_8),
                aocec_rd_reg(AO_CEC_TXTIME_2BIT_BIT7_0));
        }
        break;
        /* 7 bit */
    case 7:
        if (flag) {
            CEC_INFO("read 7 bit:0x%x%x\n",
                aocec_rd_reg(AO_CEC_TXTIME_17MS_BIT10_8),
                aocec_rd_reg(AO_CEC_TXTIME_17MS_BIT7_0));
        }
        aocec_wr_reg(AO_CEC_TXTIME_17MS_BIT7_0, time_set & 0xff);
        aocec_wr_reg(AO_CEC_TXTIME_17MS_BIT10_8, (time_set >> 8) & 0x7);
        if (flag) {
            CEC_INFO("write 7 bit:0x%x%x\n",
                aocec_rd_reg(AO_CEC_TXTIME_17MS_BIT10_8),
                aocec_rd_reg(AO_CEC_TXTIME_17MS_BIT7_0));
        }
        break;
    default:
        break;
    }
}

static unsigned int ao_cec_intr_stat(void)
{
    return readl(cec_dev->cec_reg + AO_CEC_INTR_STAT);
}

unsigned int cec_intr_stat(void)
{
    return ao_cec_intr_stat();
}

/*
 *wr_flag: 1 write; value valid
 *       0 read;  value invalid
 */
unsigned int cec_config(unsigned int value, bool wr_flag)
{
    if (wr_flag)
        cec_set_reg_bits(AO_DEBUG_REG0, value, 0, 8);

    return readl(cec_dev->cec_reg + AO_DEBUG_REG0);
}

/*
 *wr_flag:1 write; value valid
 *      0 read;  value invalid
 */
unsigned int cec_phyaddr_config(unsigned int value, bool wr_flag)
{
    if (wr_flag)
        cec_set_reg_bits(AO_DEBUG_REG1, value, 0, 16);

    return readl(cec_dev->cec_reg + AO_DEBUG_REG1);
}

/*
 *wr_flag:1 write; value valid
 *      0 read;  value invalid
 */
unsigned int cec_logicaddr_config(unsigned int value, bool wr_flag)
{
    if (wr_flag)
        cec_set_reg_bits(AO_DEBUG_REG3, value, 0, 8);

    return readl(cec_dev->cec_reg + AO_DEBUG_REG3);
}

void cec_keep_reset(void)
{
    writel(0x1, cec_dev->cec_reg + AO_CEC_GEN_CNTL);
}
/*
 * cec hw module init befor allocate logical address
 */
static void cec_pre_init(void)
{
    ao_cec_init();

    cec_arbit_bit_time_set(3, 0x118, 0);
    cec_arbit_bit_time_set(5, 0x000, 0);
    cec_arbit_bit_time_set(7, 0x2aa, 0);
}

static int cec_late_check_rx_buffer(void)
{
    int ret;
    struct delayed_work *dwork = &cec_dev->cec_work;

    ret = cec_rx_buf_check();
    if (!ret)
        return 0;
    /*
     * start another check if rx buffer is full
     */
    if ((-1) == cec_ll_rx(rx_msg, &rx_len)) {
        CEC_INFO("buffer got unrecorgnized msg\n");
        cec_rx_buf_clear();
        return 0;
    } else {
        mod_delayed_work(cec_dev->cec_thread, dwork, 0);
        return 1;
    }
}

void cec_key_report(int suspend)
{
    input_event(cec_dev->cec_info.remote_cec_dev, EV_KEY, KEY_POWER, 1);
    input_sync(cec_dev->cec_info.remote_cec_dev);
    input_event(cec_dev->cec_info.remote_cec_dev, EV_KEY, KEY_POWER, 0);
    input_sync(cec_dev->cec_info.remote_cec_dev);
    if (!suspend)
        CEC_INFO("== WAKE UP BY CEC ==\n")
    else
        CEC_INFO("== SLEEP by CEC==\n")
}

void cec_give_version(unsigned int dest)
{
    unsigned char index = cec_dev->cec_info.log_addr[0];
    unsigned char msg[3];

    if (0xf != dest) {
        msg[0] = ((index & 0xf) << 4) | dest;
        msg[1] = CEC_OC_CEC_VERSION;
        msg[2] = CEC_VERSION_14A;
        cec_ll_tx(msg, 3);
    }
}

void cec_report_physical_address_smp(void)
{
    unsigned char msg[5];
    unsigned char index = cec_dev->cec_info.log_addr[0];
    unsigned char phy_addr_ab, phy_addr_cd;

    phy_addr_ab = (cec_dev->phy_addr >> 8) & 0xff;
    phy_addr_cd = (cec_dev->phy_addr >> 0) & 0xff;
    msg[0] = ((index & 0xf) << 4) | CEC_BROADCAST_ADDR;
    msg[1] = CEC_OC_REPORT_PHYSICAL_ADDRESS;
    msg[2] = phy_addr_ab;
    msg[3] = phy_addr_cd;
    msg[4] = cec_dev->dev_type;

    cec_ll_tx(msg, 5);
}

void cec_device_vendor_id(void)
{
    unsigned char index = cec_dev->cec_info.log_addr[0];
    unsigned char msg[5];
    unsigned int vendor_id;

    vendor_id = cec_dev->v_data.vendor_id;
    msg[0] = ((index & 0xf) << 4) | CEC_BROADCAST_ADDR;
    msg[1] = CEC_OC_DEVICE_VENDOR_ID;
    msg[2] = (vendor_id >> 16) & 0xff;
    msg[3] = (vendor_id >> 8) & 0xff;
    msg[4] = (vendor_id >> 0) & 0xff;

    cec_ll_tx(msg, 5);
}

void cec_give_deck_status(unsigned int dest)
{
    unsigned char index = cec_dev->cec_info.log_addr[0];
    unsigned char msg[3];

    msg[0] = ((index & 0xf) << 4) | dest;
    msg[1] = CEC_OC_DECK_STATUS;
    msg[2] = 0x1a;
    cec_ll_tx(msg, 3);
}

void cec_menu_status_smp(int dest, int status)
{
    unsigned char msg[3];
    unsigned char index = cec_dev->cec_info.log_addr[0];

    msg[0] = ((index & 0xf) << 4) | dest;
    msg[1] = CEC_OC_MENU_STATUS;
    if (status == DEVICE_MENU_ACTIVE)
        msg[2] = DEVICE_MENU_ACTIVE;
    else
        msg[2] = DEVICE_MENU_INACTIVE;
    cec_ll_tx(msg, 3);
}

void cec_imageview_on_smp(void)
{
    unsigned char msg[2];
    unsigned char index = cec_dev->cec_info.log_addr[0];

    msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
    msg[1] = CEC_OC_IMAGE_VIEW_ON;
    cec_ll_tx(msg, 2);
}

void cec_get_menu_language_smp(void)
{
    unsigned char msg[2];
    unsigned char index = cec_dev->cec_info.log_addr[0];

    msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
    msg[1] = CEC_OC_GET_MENU_LANGUAGE;

    cec_ll_tx(msg, 2);
}


void cec_inactive_source(int dest)
{
    unsigned char index = cec_dev->cec_info.log_addr[0];
    unsigned char msg[4];
    unsigned char phy_addr_ab, phy_addr_cd;

    phy_addr_ab = (cec_dev->phy_addr >> 8) & 0xff;
    phy_addr_cd = (cec_dev->phy_addr >> 0) & 0xff;
    msg[0] = ((index & 0xf) << 4) | dest;
    msg[1] = CEC_OC_INACTIVE_SOURCE;
    msg[2] = phy_addr_ab;
    msg[3] = phy_addr_cd;

    cec_ll_tx(msg, 4);
}

void cec_set_osd_name(int dest)
{
    unsigned char index = cec_dev->cec_info.log_addr[0];
    unsigned char osd_len = strlen(cec_dev->cec_info.osd_name);
    unsigned char msg[16];

    if (0xf != dest) {
        msg[0] = ((index & 0xf) << 4) | dest;
        msg[1] = CEC_OC_SET_OSD_NAME;
        memcpy(&msg[2], cec_dev->cec_info.osd_name, osd_len);

        cec_ll_tx(msg, 2 + osd_len);
    }
}

void cec_active_source_smp(void)
{
    unsigned char msg[4];
    unsigned char index = cec_dev->cec_info.log_addr[0];
    unsigned char phy_addr_ab;
    unsigned char phy_addr_cd;

    phy_addr_ab = (cec_dev->phy_addr >> 8) & 0xff;
    phy_addr_cd = (cec_dev->phy_addr >> 0) & 0xff;
    msg[0] = ((index & 0xf) << 4) | CEC_BROADCAST_ADDR;
    msg[1] = CEC_OC_ACTIVE_SOURCE;
    msg[2] = phy_addr_ab;
    msg[3] = phy_addr_cd;
    cec_ll_tx(msg, 4);
}

void cec_set_stream_path(unsigned char *msg)
{
    unsigned int phy_addr_active;

    phy_addr_active = (unsigned int)(msg[2] << 8 | msg[3]);
    if (phy_addr_active == cec_dev->phy_addr) {
        cec_active_source_smp();
        /*
         * some types of TV such as panasonic need to send menu status,
         * otherwise it will not send remote key event to control
         * device's menu
         */
        cec_menu_status_smp(msg[0] >> 4, DEVICE_MENU_ACTIVE);
    }
}

void cec_report_power_status(int dest, int status)
{
    unsigned char index = cec_dev->cec_info.log_addr[0];
    unsigned char msg[3];

    msg[0] = ((index & 0xf) << 4) | dest;
    msg[1] = CEC_OC_REPORT_POWER_STATUS;
    msg[2] = status;
    cec_ll_tx(msg, 3);
}

void cec_send_simplink_alive(void)
{
    unsigned char index = cec_dev->cec_info.log_addr[0];
    unsigned char msg[4];

    msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
    msg[1] = CEC_OC_VENDOR_COMMAND;
    msg[2] = 0x2;
    msg[3] = 0x5;

    cec_ll_tx(msg, 4);
}

void cec_send_simplink_ack(void)
{
    unsigned char index = cec_dev->cec_info.log_addr[0];
    unsigned char msg[4];

    msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
    msg[1] = CEC_OC_VENDOR_COMMAND;
    msg[2] = 0x5;
    msg[3] = 0x1;

    cec_ll_tx(msg, 4);
}

int cec_node_init(struct hdmitx_dev *hdmitx_device)
{
    unsigned char a, b, c, d;

    int i, bool = 0;
    int phy_addr_ok = 1;
    const enum _cec_log_dev_addr_e player_dev[3] = {
        CEC_RECORDING_DEVICE_1_ADDR,
        CEC_RECORDING_DEVICE_2_ADDR,
        CEC_RECORDING_DEVICE_3_ADDR,
    };

    unsigned long cec_phy_addr;

    /* If no connect, return directly */
    if ((hdmitx_device->cec_init_ready == 0) ||
        (hdmitx_device->hpd_state == 0)) {
        return -1;
    }
    a = hdmitx_device->hdmi_info.vsdb_phy_addr.a;
    b = hdmitx_device->hdmi_info.vsdb_phy_addr.b;
    c = hdmitx_device->hdmi_info.vsdb_phy_addr.c;
    d = hdmitx_device->hdmi_info.vsdb_phy_addr.d;

    /* Don't init if switched to libcec mode*/
    if ((cec_dev->hal_flag & (1 << HDMI_OPTION_SYSTEM_CEC_CONTROL)))
        return -1;

    CEC_INFO("cec_node_init started\n");

    cec_phy_addr = ((a << 12) | (b << 8) | (c << 4) | (d << 0));

    for (i = 0; i < 3; i++) {
        CEC_INFO("CEC: start poll dev\n");
        cec_polling_online_dev(player_dev[i], &bool);
        CEC_INFO("player_dev[%d]:0x%x\n", i, player_dev[i]);
        if (bool == 0) {   /* 0 means that no any respond */
    /* If VSDB is not valid, use last or default physical address. */
            cec_logicaddr_set(player_dev[i], CEC_LOGICAL_ADDR0);
            if (hdmitx_device->hdmi_info.vsdb_phy_addr.valid == 0) {
                phy_addr_ok = 0;
                CEC_INFO("invalid cec PhyAddr\n");
                if (cec_phyaddr_config(0, 0)) {
                    CEC_INFO("use last physical address\n");
                } else {
                    cec_phyaddr_config(0x2000, 1);
                    CEC_INFO("use Def Phy address\n");
                }
            } else
                cec_phyaddr_config(cec_phy_addr, 1);

                CEC_INFO("physical address:0x%x\n",
                        cec_phyaddr_config(0, 0));

                cec_dev->cec_info.power_status = TRANS_STANDBY_TO_ON;
                cec_logicaddr_config(player_dev[i], 1);
                cec_dev->cec_info.log_addr[0] = player_dev[i];
                /* Set Physical address */
                cec_dev->phy_addr = cec_phy_addr;

                cec_dev->cec_info.cec_version = CEC_VERSION_14A;
                cec_dev->cec_info.vendor_id = cec_dev->v_data.vendor_id;
                strcpy(cec_dev->cec_info.osd_name,
                       cec_dev->v_data.cec_osd_string);
                cec_logicaddr_set(player_dev[i], CEC_LOGICAL_ADDR0);

                CEC_INFO("Set logical address: %d\n",
                    player_dev[i]);

                cec_dev->cec_info.power_status = POWER_ON;
                if (cec_dev->cec_info.menu_status == DEVICE_MENU_INACTIVE)
                    break;
                msleep(100);
                if (phy_addr_ok) {
                    cec_report_physical_address_smp();
                    msleep(150);
                }
                cec_device_vendor_id();
                cec_set_osd_name(0);

                /* Disable switch TV on automatically */
                if (!(hdmitx_device->cec_func_config &
                     (1 << AUTO_POWER_ON_MASK))) {
                    CEC_INFO("Auto TV switch on disabled\n");
                    break;
                }

                cec_active_source_smp();
                cec_imageview_on_smp();

                cec_menu_status_smp(CEC_TV_ADDR, DEVICE_MENU_ACTIVE);

                msleep(100);
                cec_get_menu_language_smp();
                cec_dev->cec_info.menu_status = DEVICE_MENU_ACTIVE;
            break;
        }
    }
    if (bool == 1) {
        CEC_INFO("Can't get a valid logical address\n");
        return -1;
    } else {
        CEC_INFO("cec node init: cec features ok !\n");
        return 0;
    }
}

static void cec_rx_process(void)
{
    int len = rx_len;
    int initiator, follower;
    int opcode;
    unsigned char msg[MAX_MSG] = {};

    if (len < 2 || !new_msg)        /* ignore ping message */
        return;

    memcpy(msg, rx_msg, len);
    initiator = ((msg[0] >> 4) & 0xf);
    follower  = msg[0] & 0xf;
    if (follower != 0xf && follower != cec_dev->cec_info.log_addr[0]) {
        CEC_ERR("wrong rx message of bad follower:%x", follower);
        return;
    }
    opcode = msg[1];
    switch (opcode) {
    case CEC_OC_GET_CEC_VERSION:
        cec_give_version(initiator);
        break;

    case CEC_OC_GIVE_DECK_STATUS:
        cec_give_deck_status(initiator);
        break;

    case CEC_OC_GIVE_PHYSICAL_ADDRESS:
        cec_report_physical_address_smp();
        break;

    case CEC_OC_GIVE_DEVICE_VENDOR_ID:
        cec_device_vendor_id();
        break;

    case CEC_OC_GIVE_OSD_NAME:
        cec_set_osd_name(initiator);
        break;

    case CEC_OC_STANDBY:
        cec_inactive_source(initiator);
        cec_menu_status_smp(initiator, DEVICE_MENU_INACTIVE);
        break;

    case CEC_OC_SET_STREAM_PATH:
        cec_set_stream_path(msg);
        /* wake up if in early suspend */
        if (cec_dev->cec_suspend == CEC_EARLY_SUSPEND)
            cec_key_report(0);
        break;

    case CEC_OC_REQUEST_ACTIVE_SOURCE:
        if (!cec_dev->cec_suspend)
            cec_active_source_smp();
        break;

    case CEC_OC_GIVE_DEVICE_POWER_STATUS:
        if (cec_dev->cec_suspend)
            cec_report_power_status(initiator, POWER_STANDBY);
        else
            cec_report_power_status(initiator, POWER_ON);
        break;


    case CEC_OC_USER_CONTROL_RELEASED:
        cec_user_control_released_irq();
        break;

    case CEC_OC_USER_CONTROL_PRESSED:
        if (len < 3)
            break;
        cec_user_control_pressed_irq(msg[2]);
        /* wake up by key function */
        if (cec_dev->cec_suspend == CEC_EARLY_SUSPEND) {
            if (msg[2] == 0x40 || msg[2] == 0x6d)
                cec_key_report(0);
        }
        break;

    case CEC_OC_PLAY:
        if (len < 3)
            break;
        switch (msg[2]) {
        case 0x24:
            input_event(cec_dev->cec_info.remote_cec_dev,
                EV_KEY, KEY_PLAYPAUSE, 1);
            input_sync(cec_dev->cec_info.remote_cec_dev);
            input_event(cec_dev->cec_info.remote_cec_dev,
                EV_KEY, KEY_PLAYPAUSE, 0);
            input_sync(cec_dev->cec_info.remote_cec_dev);
            break;
        case 0x25:
            input_event(cec_dev->cec_info.remote_cec_dev,
                EV_KEY, KEY_PLAYPAUSE, 1);
            input_sync(cec_dev->cec_info.remote_cec_dev);
            input_event(cec_dev->cec_info.remote_cec_dev,
                EV_KEY, KEY_PLAYPAUSE, 0);
            input_sync(cec_dev->cec_info.remote_cec_dev);
            break;
        default:
            break;
        }
        break;

    case CEC_OC_DECK_CONTROL:
        if (len < 3)
            break;
        switch (msg[2]) {
        case 0x3:
            input_event(cec_dev->cec_info.remote_cec_dev,
                EV_KEY, KEY_STOP, 1);
            input_sync(cec_dev->cec_info.remote_cec_dev);
            input_event(cec_dev->cec_info.remote_cec_dev,
                EV_KEY, KEY_STOP, 0);
            input_sync(cec_dev->cec_info.remote_cec_dev);
            break;
        default:
            break;
        }
        break;

    case CEC_OC_VENDOR_REMOTE_BUTTON_DOWN:
        if (len < 3)
            break;
        switch(msg[2]){
            //samsung vendor keys
            case 0x91:
                input_event(cec_dev->cec_info.remote_cec_dev, EV_KEY, KEY_EXIT, 1);
                input_sync(cec_dev->cec_info.remote_cec_dev);
                input_event(cec_dev->cec_info.remote_cec_dev, EV_KEY, KEY_EXIT, 0);
                input_sync(cec_dev->cec_info.remote_cec_dev);
                break;
            case 0x96:
                input_event(cec_dev->cec_info.remote_cec_dev, EV_KEY, KEY_LIST, 1);
                input_sync(cec_dev->cec_info.remote_cec_dev);
                input_event(cec_dev->cec_info.remote_cec_dev, EV_KEY, KEY_LIST, 0);
                input_sync(cec_dev->cec_info.remote_cec_dev);
                break;
            default:
                break;
        }
        break;
    case CEC_OC_MENU_REQUEST:
        if (cec_dev->cec_suspend)
            cec_menu_status_smp(initiator, DEVICE_MENU_INACTIVE);
        else
            cec_menu_status_smp(initiator, DEVICE_MENU_ACTIVE);
        break;
    case CEC_OC_VENDOR_COMMAND:
        if (len < 3)
            break;
        if (msg[2] == 0x1) {
            cec_report_power_status(initiator, cec_dev->cec_info.power_status);
            cec_send_simplink_alive();
        } else if (msg[2] == 0x4) {
            cec_send_simplink_ack();
        }
        break;
	case CEC_OC_DEVICE_VENDOR_ID:
		break;

    default:
        break;
    }
    new_msg = 0;
}

static void cec_task(struct work_struct *work)
{
    struct delayed_work *dwork;
    int ret;

    dwork = &cec_dev->cec_work;
    if (cec_dev &&
       !(cec_dev->hal_flag & (1 << HDMI_OPTION_SYSTEM_CEC_CONTROL))) {
        if (1 << cec_dev->cec_info.log_addr[0] & (1 << 0x0 | 1 << 0xF)) {
            ret = cec_node_init(cec_dev->tx_dev);
            if (ret < 0) {
                return;
            }
        }
        cec_rx_process();
    }
    if (!cec_late_check_rx_buffer())
        queue_delayed_work(cec_dev->cec_thread, dwork, CEC_FRAME_DELAY);
}

irqreturn_t cec_isr_handler(int irq, void *dev_instance)
{
    unsigned int intr_stat = 0;
    struct delayed_work *dwork;

    dwork = &cec_dev->cec_work;
    intr_stat = cec_intr_stat();
    if (intr_stat & (1<<1)) {   /* aocec tx intr */
        tx_irq_handle();
        return IRQ_HANDLED;
    }
    if ((-1) == cec_ll_rx(rx_msg, &rx_len))
        return IRQ_HANDLED;

    complete(&cec_dev->rx_ok);
    /* check rx buffer is full */
    new_msg = 1;
    mod_delayed_work(cec_dev->cec_thread, dwork, 0);
    return IRQ_HANDLED;
}

/******************** cec class interface *************************/
static ssize_t device_type_show(struct class *cla,
    struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%ld\n", cec_dev->dev_type);
}

static ssize_t device_type_store(struct class *cla,
    struct class_attribute *attr, const char *buf, size_t count)
{
    unsigned long type;

    if (sscanf(buf, "%ld", &type) != 1)
        return -EINVAL;

    cec_dev->dev_type = type;
    CEC_ERR("set dev_type to %ld\n", type);
    return count;
}

static ssize_t menu_language_show(struct class *cla,
    struct class_attribute *attr, char *buf)
{
    char a, b, c;
    a = ((cec_dev->cec_info.menu_lang >> 16) & 0xff);
    b = ((cec_dev->cec_info.menu_lang >>  8) & 0xff);
    c = ((cec_dev->cec_info.menu_lang >>  0) & 0xff);
    return sprintf(buf, "%c%c%c\n", a, b, c);
}

static ssize_t menu_language_store(struct class *cla,
    struct class_attribute *attr, const char *buf, size_t count)
{
    char a, b, c;

    if (sscanf(buf, "%c%c%c", &a, &b, &c) != 3)
        return -EINVAL;

    cec_dev->cec_info.menu_lang = (a << 16) | (b << 8) | c;
    CEC_ERR("set menu_language to %s\n", buf);
    return count;
}

static ssize_t vendor_id_show(struct class *cla,
    struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%x\n", cec_dev->cec_info.vendor_id);
}

static ssize_t vendor_id_store(struct class *cla, struct class_attribute *attr,
    const char *buf, size_t count)
{
    int id;

    if (sscanf(buf, "%x", &id) != 1)
        return -EINVAL;
    cec_dev->cec_info.vendor_id = id;
    return count;
}

static ssize_t port_num_show(struct class *cla,
    struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", cec_dev->port_num);
}

static ssize_t arc_port_show(struct class *cla,
    struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%x\n", cec_dev->arc_port);
}

static ssize_t osd_name_show(struct class *cla,
    struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", cec_dev->cec_info.osd_name);
}

static ssize_t port_seq_show(struct class *cla,
    struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%x\n", cec_dev->port_seq);
}

static ssize_t port_status_show(struct class *cla,
    struct class_attribute *attr, char *buf)
{
    unsigned int tmp = 0;

    if (cec_dev->dev_type == DEV_TYPE_TX) {
        tmp = cec_dev->tx_dev->hpd_state;
        return sprintf(buf, "%x\n", tmp);
    }
     else {
#ifdef CONFIG_TVIN_HDMI
        tmp = hdmirx_rd_top(TOP_HPD_PWR5V);
        CEC_INFO("TOP_HPD_PWR5V:%x\n", tmp);
        tmp >>= 20;
        tmp &= 0xf;
#endif
        return sprintf(buf, "%x\n", tmp);
    }
}

static ssize_t physical_addr_show(struct class *cla,
    struct class_attribute *attr, char *buf)
{
    unsigned int tmp = cec_dev->phy_addr;

    return sprintf(buf, "%04x\n", tmp);
}

static ssize_t physical_addr_store(struct class *cla,
    struct class_attribute *attr,
    const char *buf, size_t count)
{
    int addr;

    if (sscanf(buf, "%x", &addr) != 1)
        return -EINVAL;

    if (addr > 0xffff || addr < 0) {
        CEC_ERR("invalid input:%s\n", buf);
        phy_addr_test = 0;
        return -EINVAL;
    }
    cec_dev->phy_addr = addr;
    phy_addr_test = 1;
    return count;
}

static ssize_t dbg_en_show(struct class *cla,
    struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%x\n", cec_msg_dbg_en);
}

static ssize_t dbg_en_store(struct class *cla, struct class_attribute *attr,
    const char *buf, size_t count)
{
    int en;

    if (sscanf(buf, "%d", &en) != 1)
        return -EINVAL;

    cec_msg_dbg_en = en ? 1 : 0;
    return count;
}

static struct class_attribute aocec_class_attr[] = {
    __ATTR_RO(port_num),
    __ATTR_RO(port_seq),
    __ATTR_RO(osd_name),
    __ATTR_RO(port_status),
    __ATTR_RO(arc_port),
    __ATTR(physical_addr, 0664, physical_addr_show, physical_addr_store),
    __ATTR(vendor_id, 0664, vendor_id_show, vendor_id_store),
    __ATTR(menu_language, 0664, menu_language_show, menu_language_store),
    __ATTR(device_type, 0664, device_type_show, device_type_store),
    __ATTR(dbg_en, 0664, dbg_en_show, dbg_en_store),
    __ATTR_NULL
};

/******************** cec hal interface ***************************/
static int hdmitx_cec_open(struct inode *inode, struct file *file)
{
    cec_dev->cec_info.open_count++;
    if (cec_dev->cec_info.open_count) {
        cec_dev->cec_info.hal_ctl = 1;
        /* enable all cec features */
        cec_config(0x2f, 1);
    }
    return 0;
}

static int hdmitx_cec_release(struct inode *inode, struct file *file)
{
    cec_dev->cec_info.open_count--;
    if (!cec_dev->cec_info.open_count) {
        cec_dev->cec_info.hal_ctl = 0;
        /* disable all cec features */
        cec_config(0x0, 1);
    }
    return 0;
}

static ssize_t hdmitx_cec_read(struct file *f, char __user *buf,
               size_t size, loff_t *p)
{
    int ret;

    if ((cec_dev->hal_flag & (1 << HDMI_OPTION_SYSTEM_CEC_CONTROL)))
        rx_len = 0;
    ret = wait_for_completion_timeout(&cec_dev->rx_ok, CEC_FRAME_DELAY);
    if (ret <= 0)
        return ret;
    if (rx_len == 0)
        return 0;

    if (copy_to_user(buf, rx_msg, rx_len))
        return -EINVAL;
    CEC_INFO("RX len: %d\n", rx_len);
    return rx_len;
}

static ssize_t hdmitx_cec_write(struct file *f, const char __user *buf,
                size_t size, loff_t *p)
{
    unsigned char tempbuf[16] = {};
    int ret;

    if (size > 16)
        size = 16;
    if (size <= 0)
        return 0;

    if (copy_from_user(tempbuf, buf, size))
        return -EINVAL;

    ret = cec_ll_tx(tempbuf, size);
    /* delay a byte for continue message send */
    msleep(25);
    if (ret == CEC_FAIL_NACK) {
        return -1;
    }
    else {
        return size;
    }
}

static void init_cec_port_info(struct hdmi_port_info *port,
                   struct ao_cec_dev *cec_dev)
{
    unsigned int a, b, c;

    b = cec_dev->port_num;
    for (a = 0; a < b; a++) {
        port[a].type = HDMI_INPUT;
        port[a].port_id = a + 1;
        port[a].cec_supported = 1;
        /* set ARC feature according mask */
        if (cec_dev->arc_port & (1 << a))
            port[a].arc_supported = 1;
        else
            port[a].arc_supported = 0;

        /* set port physical address according port sequence */
        if (cec_dev->port_seq) {
            c = (cec_dev->port_seq >> (4 * a)) & 0xf;
            port[a].physical_address = (c + 1) * 0x1000;
        } else {
            /* asending order if port_seq is not set */
            port[a].physical_address = (a + 1) * 0x1000;
        }
    }
}

static long hdmitx_cec_ioctl(struct file *f,
                 unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    unsigned long tmp;
    struct hdmi_port_info *port;
    int a, b, c, d;

    switch (cmd) {
    case CEC_IOC_GET_PHYSICAL_ADDR:
        if (cec_dev->dev_type ==  DEV_TYPE_TX && !phy_addr_test) {
            /* physical address for mbox */
            if (cec_dev->tx_dev->hdmi_info.vsdb_phy_addr.valid == 0)
                return -EINVAL;
            a = cec_dev->tx_dev->hdmi_info.vsdb_phy_addr.a;
            b = cec_dev->tx_dev->hdmi_info.vsdb_phy_addr.b;
            c = cec_dev->tx_dev->hdmi_info.vsdb_phy_addr.c;
            d = cec_dev->tx_dev->hdmi_info.vsdb_phy_addr.d;
            tmp = ((a << 12) | (b << 8) | (c << 4) | (d << 0));
        } else {
            /* physical address for TV */
            tmp = 0;
        }
        if (!phy_addr_test) {
            cec_dev->phy_addr = tmp;
            cec_phyaddr_config(tmp, 1);
        } else
            tmp = cec_dev->phy_addr;

        if (copy_to_user(argp, &tmp, _IOC_SIZE(cmd)))
            return -EINVAL;
        break;

    case CEC_IOC_GET_VERSION:
        tmp = CEC_VERSION_14A;
        if (copy_to_user(argp, &tmp, _IOC_SIZE(cmd)))
            return -EINVAL;
        break;

    case CEC_IOC_GET_VENDOR_ID:
        tmp = cec_dev->v_data.vendor_id;
        if (copy_to_user(argp, &tmp, _IOC_SIZE(cmd)))
            return -EINVAL;
        break;

    case CEC_IOC_GET_PORT_NUM:
        tmp = cec_dev->port_num;
        if (copy_to_user(argp, &tmp, _IOC_SIZE(cmd)))
            return -EINVAL;
        break;

    case CEC_IOC_GET_PORT_INFO:
        port = kzalloc(sizeof(*port) * cec_dev->port_num, GFP_KERNEL);
        if (!port) {
            CEC_ERR("no memory\n");
            return -EINVAL;
        }
        if (cec_dev->dev_type == DEV_TYPE_TX) {
            /* for tx only 1 port */
            a = cec_dev->tx_dev->hdmi_info.vsdb_phy_addr.a;
            b = cec_dev->tx_dev->hdmi_info.vsdb_phy_addr.b;
            c = cec_dev->tx_dev->hdmi_info.vsdb_phy_addr.c;
            d = cec_dev->tx_dev->hdmi_info.vsdb_phy_addr.d;
            tmp = ((a << 12) | (b << 8) | (c << 4) | (d << 0));
            if (cec_dev->tx_dev->hdmi_info.vsdb_phy_addr.valid == 0)
                tmp = 0xffff;
            port->type = HDMI_OUTPUT;
            port->port_id = 1;
            port->cec_supported = 1;
            /* not support arc for tx */
            port->arc_supported = 0;
            port->physical_address = tmp & 0xffff;
            if (copy_to_user(argp, port, sizeof(*port))) {
                kfree(port);
                return -EINVAL;
            }
        } else {
            b = cec_dev->port_num;
            init_cec_port_info(port, cec_dev);
            if (copy_to_user(argp, port, sizeof(*port) * b)) {
                kfree(port);
                return -EINVAL;
            }
        }
        kfree(port);
        break;

    case CEC_IOC_SET_OPTION_WAKEUP:
        /* TODO: */
        break;

    case CEC_IOC_SET_OPTION_ENALBE_CEC:
        tmp = (1 << HDMI_OPTION_ENABLE_CEC);
        if (arg) {
            cec_dev->hal_flag |= tmp;
            cec_pre_init();
        } else {
            cec_dev->hal_flag &= ~(tmp);
            CEC_INFO("disable CEC\n");
            cec_keep_reset();
        }
        break;

    case CEC_IOC_SET_OPTION_SYS_CTRL:
        tmp = (1 << HDMI_OPTION_SYSTEM_CEC_CONTROL);
        if (arg)
            cec_dev->hal_flag |= tmp;
        else
            cec_dev->hal_flag &= ~(tmp);
        break;

    case CEC_IOC_SET_OPTION_SET_LANG:
        cec_dev->cec_info.menu_lang = arg;
        break;

    case CEC_IOC_GET_CONNECT_STATUS:
        if (cec_dev->dev_type == DEV_TYPE_TX)
            tmp = cec_dev->tx_dev->hpd_state;
    #ifdef CONFIG_TVIN_HDMI
        else {
            if (copy_from_user(&a, argp, _IOC_SIZE(cmd)))
                return -EINVAL;
            tmp = (hdmirx_rd_top(TOP_HPD_PWR5V) >> 20);
            if (tmp & (1 << (a - 1)))
                tmp = 1;
            else
                tmp = 0;
        }
    #endif
        if (copy_to_user(argp, &tmp, _IOC_SIZE(cmd)))
            return -EINVAL;
        break;

    case CEC_IOC_ADD_LOGICAL_ADDR:
        tmp = arg & 0xf;
        CEC_INFO("CEC LA ARG:%ld", arg);
        cec_logicaddr_set(tmp, CEC_LOGICAL_ADDR0);
        cec_dev->cec_info.log_addr[0] = tmp;
        /* TODO set multiple LA's. Does not work yet
         * cec_logicaddr_setByMask(tmp);
        */
        /* add by hal, to init some data structure */
        cec_dev->cec_info.power_status = POWER_ON;
        cec_logicaddr_config(tmp, 1);

        cec_dev->cec_info.cec_version = CEC_VERSION_14A;
        cec_dev->cec_info.vendor_id = cec_dev->v_data.vendor_id;
        strcpy(cec_dev->cec_info.osd_name,
               cec_dev->v_data.cec_osd_string);

        if (cec_dev->dev_type == DEV_TYPE_TX)
            cec_dev->cec_info.menu_status = DEVICE_MENU_ACTIVE;
        break;

    case CEC_IOC_CLR_LOGICAL_ADDR:
        cec_logicaddr_clear();
        break;

    case CEC_IOC_SET_DEV_TYPE:
        if (arg != DEV_TYPE_TX && arg != DEV_TYPE_RX)
            return -EINVAL;
        cec_dev->dev_type = arg;
        break;

    case CEC_IOC_SET_ARC_ENABLE:
    #ifdef CONFIG_TVIN_HDMI
        /* select arc according arg */
        if (arg)
            hdmirx_wr_top(TOP_ARCTX_CNTL, 0x03);
        else
            hdmirx_wr_top(TOP_ARCTX_CNTL, 0x00);
        CEC_INFO("set arc en:%ld, reg:%lx\n",
             arg, hdmirx_rd_top(TOP_ARCTX_CNTL));
    #endif
        break;

    default:
        break;
    }
    return 0;
}

#ifdef CONFIG_COMPAT
static long hdmitx_cec_compat_ioctl(struct file *f,
                    unsigned int cmd, unsigned long arg)
{
    arg = (unsigned long)compat_ptr(arg);
    return hdmitx_cec_ioctl(f, cmd, arg);
}
#endif

/* for improve rw permission */
static char *aml_cec_class_devnode(struct device *dev, umode_t *mode)
{
    if (mode)
        *mode = 0666;
    CEC_INFO("mode is %x\n", *mode);
    return NULL;
}

static struct class aocec_class = {
    .name = CEC_DEV_NAME,
    .class_attrs = aocec_class_attr,
    .devnode = aml_cec_class_devnode,
};


static const struct file_operations hdmitx_cec_fops = {
    .owner          = THIS_MODULE,
    .open           = hdmitx_cec_open,
    .read           = hdmitx_cec_read,
    .write          = hdmitx_cec_write,
    .release        = hdmitx_cec_release,
    .unlocked_ioctl = hdmitx_cec_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = hdmitx_cec_compat_ioctl,
#endif
};

/************************ cec high level code *****************************/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void aocec_early_suspend(struct early_suspend *h)
{
    cec_dev->cec_suspend = CEC_EARLY_SUSPEND;
    CEC_INFO("%s, suspend:%d\n", __func__, cec_dev->cec_suspend);
}

static void aocec_late_resume(struct early_suspend *h)
{
    cec_dev->cec_suspend = 0;
    CEC_INFO("%s, suspend:%d\n", __func__, cec_dev->cec_suspend);

}
#endif

static int aml_cec_probe(struct platform_device *pdev)
{
    int i;
    struct device *cdev;
#ifdef CONFIG_OF
    struct device_node *node = pdev->dev.of_node;
    int irq_idx = 0, r;
    const char *pin_name = NULL, *irq_name = NULL;
    struct pinctrl *p;
    struct vendor_info_data *vend;
    struct resource *res;
    resource_size_t *base;
#endif

    cec_dev = kzalloc(sizeof(struct ao_cec_dev), GFP_KERNEL);
    if (!cec_dev) {
        CEC_ERR("alloc memory failed\n");
        return -ENOMEM;
    }
    cec_dev->dev_type = DEV_TYPE_TX;
    cec_dev->dbg_dev  = &pdev->dev;
    cec_dev->tx_dev   = get_hdmitx_device();
    phy_addr_test = 0;

    /* cdev registe */
    r = class_register(&aocec_class);
    if (r) {
        CEC_ERR("regist class failed\n");
        return -EINVAL;
    }
    pdev->dev.class = &aocec_class;
    cec_dev->cec_info.dev_no = register_chrdev(0, CEC_DEV_NAME,
                      &hdmitx_cec_fops);
    if (cec_dev->cec_info.dev_no < 0) {
        CEC_ERR("alloc chrdev failed\n");
        return -EINVAL;
    }
    CEC_INFO("alloc chrdev %x\n", cec_dev->cec_info.dev_no);
    cdev = device_create(&aocec_class, &pdev->dev,
                 MKDEV(cec_dev->cec_info.dev_no, 0),
                 NULL, CEC_DEV_NAME);
    if (IS_ERR(cdev)) {
        CEC_ERR("create chrdev failed, dev:%p\n", cdev);
        unregister_chrdev(cec_dev->cec_info.dev_no,
                  CEC_DEV_NAME);
        return -EINVAL;
    }

    init_completion(&cec_dev->rx_ok);
    init_completion(&cec_dev->tx_ok);
    mutex_init(&cec_dev->cec_mutex);
    spin_lock_init(&cec_dev->cec_reg_lock);
    cec_dev->cec_thread = create_workqueue("cec_work");
    if (cec_dev->cec_thread == NULL) {
        CEC_INFO("create work queue failed\n");
        return -EFAULT;
    }
    INIT_DELAYED_WORK(&cec_dev->cec_work, cec_task);
    hrtimer_init(&cec_key_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    cec_key_timer.function = cec_key_up;
    cec_dev->cec_info.remote_cec_dev = input_allocate_device();
    if (!cec_dev->cec_info.remote_cec_dev)
        CEC_INFO("No enough memory\n");

    cec_dev->cec_info.remote_cec_dev->name = "cec_input";

    cec_dev->cec_info.remote_cec_dev->evbit[0] = BIT_MASK(EV_KEY);
    cec_dev->cec_info.remote_cec_dev->keybit[BIT_WORD(BTN_0)] =
        BIT_MASK(BTN_0);
    cec_dev->cec_info.remote_cec_dev->id.bustype = BUS_ISA;
    cec_dev->cec_info.remote_cec_dev->id.vendor = 0x1b8e;
    cec_dev->cec_info.remote_cec_dev->id.product = 0x0cec;
    cec_dev->cec_info.remote_cec_dev->id.version = 0x0001;

    for (i = 0; i < 160; i++)
        set_bit(cec_key_map[i], cec_dev->cec_info.remote_cec_dev->keybit);
    if (input_register_device(cec_dev->cec_info.remote_cec_dev)) {
        CEC_INFO("Failed to register device\n");
        input_free_device(cec_dev->cec_info.remote_cec_dev);
    }

#ifdef CONFIG_OF
    /* pinmux set */
    if (of_get_property(node, "pinctrl-names", NULL)) {
        r = of_property_read_string(node,
                        "pinctrl-names",
                        &pin_name);
        if (!r)
            p = devm_pinctrl_get_select(&pdev->dev, pin_name);
    }

    /* irq set */
    irq_idx = of_irq_get(node, 0);
    cec_dev->irq_cec = irq_idx;
    if (of_get_property(node, "interrupt-names", NULL)) {
        r = of_property_read_string(node, "interrupt-names", &irq_name);
        if (!r) {
            r = request_irq(irq_idx, &cec_isr_handler, IRQF_SHARED,
                    irq_name, (void *)cec_dev);
        }
    }
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res) {
        base = ioremap(res->start, res->end - res->start);
        cec_dev->exit_reg = (void *)base;
    } else {
        CEC_INFO("no memory resource\n");
        cec_dev->exit_reg = NULL;
    }
    res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if (res) {
        base = ioremap(res->start, res->end - res->start);
        cec_dev->cec_reg = (void *)base;
    } else {
        CEC_ERR("no CEC reg resource\n");
        cec_dev->cec_reg = NULL;
    }
    res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
    if (res) {
        base = ioremap(res->start, res->end - res->start);
        cec_dev->hdmi_rxreg = (void *)base;
    } else {
        CEC_ERR("no hdmirx reg resource\n");
        cec_dev->hdmi_rxreg = NULL;
    }

    r = of_property_read_u32(node, "port_num", &(cec_dev->port_num));
    if (r) {
        CEC_ERR("not find 'port_num'\n");
        cec_dev->port_num = 1;
    }
    r = of_property_read_u32(node, "arc_port_mask", &(cec_dev->arc_port));
    if (r) {
        CEC_ERR("not find 'arc_port_mask'\n");
        cec_dev->arc_port = 0;
    }

    vend = &cec_dev->v_data;
    r = of_property_read_string(node, "vendor_name",
        (const char **)&(vend->vendor_name));
    if (r)
        CEC_INFO("not find vendor name\n");

    r = of_property_read_u32(node, "vendor_id", &(vend->vendor_id));
    if (r)
        CEC_INFO("not find vendor id\n");

    r = of_property_read_string(node, "product_desc",
        (const char **)&(vend->product_desc));
    if (r)
        CEC_INFO("not find product desc\n");

    r = of_property_read_string(node, "cec_osd_string",
        (const char **)&(vend->cec_osd_string));
    if (r) {
        CEC_INFO("not find cec osd string\n");
        strcpy(vend->cec_osd_string, "AML TV/BOX");
    }

    /* get port sequence */
    node = of_find_node_by_path("/hdmirx");
    if (node == NULL) {
        CEC_ERR("can't find hdmirx\n");
        cec_dev->port_seq = 0;
    } else {
        r = of_property_read_u32(node, "rx_port_maps",
                     &(cec_dev->port_seq));
        if (r)
            CEC_INFO("not find rx_port_maps\n");
    }
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
    aocec_suspend_handler.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 20;
    aocec_suspend_handler.suspend = aocec_early_suspend;
    aocec_suspend_handler.resume  = aocec_late_resume;
    aocec_suspend_handler.param   = cec_dev;
    register_early_suspend(&aocec_suspend_handler);
#endif
    /* for init */
    cec_pre_init();
    queue_delayed_work(cec_dev->cec_thread, &cec_dev->cec_work, 0);
    cec_dev->tx_dev->cec_init_ready = 1;
    return 0;
}

static int aml_cec_remove(struct platform_device *pdev)
{
    CEC_INFO("cec uninit!\n");
    free_irq(cec_dev->irq_cec, (void *)cec_dev);
    cec_dev->tx_dev->cec_init_ready = 0;

    if (cec_dev->cec_thread) {
        cancel_delayed_work_sync(&cec_dev->cec_work);
        destroy_workqueue(cec_dev->cec_thread);
    }
    input_unregister_device(cec_dev->cec_info.remote_cec_dev);
    unregister_chrdev(cec_dev->cec_info.dev_no, CEC_DEV_NAME);
    class_unregister(&aocec_class);
    kfree(cec_dev);
    return 0;
}

#ifdef CONFIG_PM
static int aml_cec_pm_prepare(struct device *dev)
{
    cec_dev->cec_suspend = CEC_DEEP_SUSPEND;
    CEC_INFO("%s, cec_suspend:%d\n", __func__, cec_dev->cec_suspend);
    return 0;
}

static void aml_cec_pm_complete(struct device *dev)
{
    int exit = 0;

    if (cec_dev->exit_reg) {
        exit = readl(cec_dev->exit_reg);
        CEC_INFO("wake up flag:%x\n", exit);
    }
    if (((exit >> 28) & 0xf) == CEC_WAKEUP) {
        cec_key_report(0);
    }
}

static int aml_cec_resume_noirq(struct device *dev)
{
    CEC_INFO("cec resume noirq!\n");
    cec_dev->cec_info.power_status = TRANS_STANDBY_TO_ON;
    return 0;
}

static const struct dev_pm_ops aml_cec_pm = {
    .prepare  = aml_cec_pm_prepare,
    .complete = aml_cec_pm_complete,
    .resume_noirq = aml_cec_resume_noirq,
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id aml_cec_dt_match[] = {
    {
        .compatible = "amlogic, amlogic-aocec",
    },
    {},
};
#endif

static struct platform_driver aml_cec_driver = {
    .driver = {
        .name  = "cectx",
        .owner = THIS_MODULE,
    #ifdef CONFIG_PM
        .pm     = &aml_cec_pm,
    #endif
    #ifdef CONFIG_OF
        .of_match_table = aml_cec_dt_match,
    #endif
    },
    .probe  = aml_cec_probe,
    .remove = aml_cec_remove,
};

static int __init cec_init(void)
{
    int ret;
    ret = platform_driver_register(&aml_cec_driver);
    return ret;
}

static void __exit cec_uninit(void)
{
    platform_driver_unregister(&aml_cec_driver);
}


module_init(cec_init);
module_exit(cec_uninit);
MODULE_DESCRIPTION("AMLOGIC HDMI TX CEC driver");
MODULE_LICENSE("GPL");
