/*
 * Synopsys DesignWare Multimedia Card Interface driver
 *  (Based on NXP driver for lpc 31xx)
 *
 * Copyright (C) 2009 NXP Semiconductors
 * Copyright (C) 2009, 2010 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/blkdev.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/dw_mmc.h>
#include <linux/mmc/sd.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <mach/smc.h>
#include <plat/map-s5p.h>

#include "dw_mmc.h"

#ifdef CONFIG_MMC_DW_FMP_DM_CRYPT
#include "../card/queue.h"
#endif

/* Common flag combinations */
#define DW_MCI_DATA_ERROR_FLAGS	(SDMMC_INT_DTO | SDMMC_INT_DCRC | \
				 SDMMC_INT_HTO | SDMMC_INT_SBE  | \
				 SDMMC_INT_EBE)
#define DW_MCI_CMD_ERROR_FLAGS	(SDMMC_INT_RTO | SDMMC_INT_RCRC | \
				 SDMMC_INT_RESP_ERR)
#define DW_MCI_ERROR_FLAGS	(DW_MCI_DATA_ERROR_FLAGS | \
				 DW_MCI_CMD_ERROR_FLAGS  | SDMMC_INT_HLE)
#define DW_MCI_SEND_STATUS	1
#define DW_MCI_RECV_STATUS	2
#define DW_MCI_DMA_THRESHOLD	4
#define MMC_CHECK_CMDQ_MODE(host)				\
	(host->cur_slot && host->cur_slot->mmc &&		\
	 host->cur_slot->mmc->card &&				\
	 host->cur_slot->mmc->card->ext_csd.cmdq_mode_en)

#ifdef CONFIG_MMC_DW_IDMAC
struct idmac_desc {
	u32		des0;	/* Control Descriptor */
#define IDMAC_DES0_DIC	BIT(1)
#define IDMAC_DES0_LD	BIT(2)
#define IDMAC_DES0_FD	BIT(3)
#define IDMAC_DES0_CH	BIT(4)
#define IDMAC_DES0_ER	BIT(5)
#define IDMAC_DES0_CES	BIT(30)
#define IDMAC_DES0_OWN	BIT(31)

	u32		des1;	/* Buffer sizes */
#define IDMAC_SET_BUFFER1_SIZE(d, s) \
	((d)->des1 = ((d)->des1 & 0x03ffe000) | ((s) & 0x1fff))

	u32		des2;	/* buffer 1 physical address */

	u32		des3;	/* buffer 2 physical address */
	u32		des4;	/* Sector Key */
	u32		des5;	/* Application Key 0 */
	u32		des6;	/* Application Key 1 */
	u32		des7;	/* Application Key 2 */
};
#endif /* CONFIG_MMC_DW_IDMAC */

#define DATA_RETRY	1
#define DRTO		200
#define DRTO_MON_PERIOD	50

int dw_mci_ciu_clk_en(struct dw_mci *host, bool force_gating)
{
	int ret = 1;
	struct clk *gate_clk = (!IS_ERR(host->gate_clk)) ? host->gate_clk :
		((!IS_ERR(host->ciu_clk)) ? host->ciu_clk : NULL);

	if (!host->pdata->use_gate_clock && !force_gating)
		return 0;

	if (!gate_clk) {
		dev_err(host->dev, "no available CIU gating clock\n");
		return 1;
	}

	if (!atomic_cmpxchg(&host->ciu_clk_cnt, 0, 1)) {
		ret = clk_prepare_enable(gate_clk);
		if (ret)
			dev_err(host->dev, "failed to enable ciu clock\n");
	}

	return ret;
}
EXPORT_SYMBOL(dw_mci_ciu_clk_en);

void dw_mci_ciu_clk_dis(struct dw_mci *host)
{
	struct clk *gate_clk = (!IS_ERR(host->gate_clk)) ? host->gate_clk :
		((!IS_ERR(host->ciu_clk)) ? host->ciu_clk : NULL);

	BUG_ON(!gate_clk);

	if (!host->pdata->use_gate_clock)
		return;

	if (atomic_read(&host->ciu_en_win)) {
		dev_err(host->dev, "Not available CIU off: %d\n",
				atomic_read(&host->ciu_en_win));
		return;
	}

	if (atomic_cmpxchg(&host->ciu_clk_cnt, 1, 0))
		clk_disable_unprepare(gate_clk);
}
EXPORT_SYMBOL(dw_mci_ciu_clk_dis);

static int dw_mci_biu_clk_en(struct dw_mci *host)
{
	int ret = 1;

	if (!atomic_read(&host->biu_clk_cnt)) {
		ret = clk_prepare_enable(host->biu_clk);
		atomic_inc_return(&host->biu_clk_cnt);
		if (ret)
			dev_err(host->dev, "failed to enable biu clock\n");
	}

	return ret;
}

static void dw_mci_biu_clk_dis(struct dw_mci *host)
{
	if (atomic_read(&host->biu_clk_cnt)) {
		clk_disable_unprepare(host->biu_clk);
		atomic_dec_return(&host->biu_clk_cnt);
	}
}

#if defined(CONFIG_DEBUG_FS)
static int dw_mci_req_show(struct seq_file *s, void *v)
{
	struct dw_mci_slot *slot = s->private;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_command *stop;
	struct mmc_data	*data;

	/* Make sure we get a consistent snapshot */
	spin_lock_bh(&slot->host->lock);
	mrq = slot->mrq;

	if (mrq) {
		cmd = mrq->cmd;
		data = mrq->data;
		stop = mrq->stop;

		if (cmd)
			seq_printf(s,
				   "CMD%u(0x%x) flg %x rsp %x %x %x %x err %d\n",
				   cmd->opcode, cmd->arg, cmd->flags,
				   cmd->resp[0], cmd->resp[1], cmd->resp[2],
				   cmd->resp[2], cmd->error);
		if (data)
			seq_printf(s, "DATA %u / %u * %u flg %x err %d\n",
				   data->bytes_xfered, data->blocks,
				   data->blksz, data->flags, data->error);
		if (stop)
			seq_printf(s,
				   "CMD%u(0x%x) flg %x rsp %x %x %x %x err %d\n",
				   stop->opcode, stop->arg, stop->flags,
				   stop->resp[0], stop->resp[1], stop->resp[2],
				   stop->resp[2], stop->error);
	}

	spin_unlock_bh(&slot->host->lock);

	return 0;
}

static int dw_mci_req_open(struct inode *inode, struct file *file)
{
	return single_open(file, dw_mci_req_show, inode->i_private);
}

static const struct file_operations dw_mci_req_fops = {
	.owner		= THIS_MODULE,
	.open		= dw_mci_req_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dw_mci_regs_show(struct seq_file *s, void *v)
{
	seq_printf(s, "STATUS:\t0x%08x\n", SDMMC_STATUS);
	seq_printf(s, "RINTSTS:\t0x%08x\n", SDMMC_RINTSTS);
	seq_printf(s, "CMD:\t0x%08x\n", SDMMC_CMD);
	seq_printf(s, "CTRL:\t0x%08x\n", SDMMC_CTRL);
	seq_printf(s, "INTMASK:\t0x%08x\n", SDMMC_INTMASK);
	seq_printf(s, "CLKENA:\t0x%08x\n", SDMMC_CLKENA);

	return 0;
}

static int dw_mci_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dw_mci_regs_show, inode->i_private);
}

static const struct file_operations dw_mci_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= dw_mci_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void dw_mci_init_debugfs(struct dw_mci_slot *slot)
{
	struct mmc_host	*mmc = slot->mmc;
	struct dw_mci *host = slot->host;
	struct dentry *root;
	struct dentry *node;

	root = mmc->debugfs_root;
	if (!root)
		return;

	node = debugfs_create_file("regs", S_IRUSR, root, host,
				   &dw_mci_regs_fops);
	if (!node)
		goto err;

	node = debugfs_create_file("req", S_IRUSR, root, slot,
				   &dw_mci_req_fops);
	if (!node)
		goto err;

	node = debugfs_create_u32("state", S_IRUSR, root,
			(u32 *)&host->state_cmd);
	if (!node)
		goto err;

	node = debugfs_create_x32("pending_events", S_IRUSR, root,
				  (u32 *)&host->pending_events);
	if (!node)
		goto err;

	node = debugfs_create_x32("completed_events", S_IRUSR, root,
				  (u32 *)&host->completed_events);
	if (!node)
		goto err;

	return;

err:
	dev_err(&mmc->class_dev, "failed to initialize debugfs for slot\n");
}
#endif /* defined(CONFIG_DEBUG_FS) */

void dw_mci_cmd_reg_summary(struct dw_mci *host)
{
	u32 reg;
	reg = mci_readl(host, CMD);

	dev_err(host->dev, ": ================= CMD REG =================\n");
	dev_err(host->dev, ": read/write        : %s\n",
					(reg & (0x1 << 10)) ? "write" : "read");
	dev_err(host->dev, ": data expected     : %d\n", (reg >> 9) & 0x1);
	dev_err(host->dev, ": cmd index         : %d\n", (reg >> 0) & 0x3f);
}

void dw_mci_status_reg_summary(struct dw_mci *host)
{
	u32 reg;
	reg = mci_readl(host, STATUS);

	dev_err(host->dev, ": ================ STATUS REG ===============\n");
	dev_err(host->dev, ": fifocount         : %d\n", (reg >> 17) & 0x1fff);
	dev_err(host->dev, ": response index    : %d\n", (reg >> 11) & 0x3f);
	dev_err(host->dev, ": data state mc busy: %d\n", (reg >> 10) & 0x1);
	dev_err(host->dev, ": data busy         : %d\n", (reg >> 9) & 0x1);
	dev_err(host->dev, ": data 3 state      : %d\n", (reg >> 8) & 0x1);
	dev_err(host->dev, ": command fsm state : %d\n", (reg >> 4) & 0xf);
	dev_err(host->dev, ": fifo full         : %d\n", (reg >> 3) & 0x1);
	dev_err(host->dev, ": fifo empty        : %d\n", (reg >> 2) & 0x1);
	dev_err(host->dev, ": fifo tx watermark : %d\n", (reg >> 1) & 0x1);
	dev_err(host->dev, ": fifo rx watermark : %d\n", (reg >> 0) & 0x1);
}

void dw_mci_reg_dump(struct dw_mci *host)
{
	const struct dw_mci_drv_data *drv_data = host->drv_data;

	dev_err(host->dev, ": ============== REGISTER DUMP ==============\n");
	dev_err(host->dev, ": CTRL:	0x%08x\n", mci_readl(host, CTRL));
	dev_err(host->dev, ": PWREN:	0x%08x\n", mci_readl(host, PWREN));
	dev_err(host->dev, ": CLKDIV:	0x%08x\n", mci_readl(host, CLKDIV));
	dev_err(host->dev, ": CLKSRC:	0x%08x\n", mci_readl(host, CLKSRC));
	dev_err(host->dev, ": CLKENA:	0x%08x\n", mci_readl(host, CLKENA));
	dev_err(host->dev, ": TMOUT:	0x%08x\n", mci_readl(host, TMOUT));
	dev_err(host->dev, ": CTYPE:	0x%08x\n", mci_readl(host, CTYPE));
	dev_err(host->dev, ": BLKSIZ:	0x%08x\n", mci_readl(host, BLKSIZ));
	dev_err(host->dev, ": BYTCNT:	0x%08x\n", mci_readl(host, BYTCNT));
	dev_err(host->dev, ": INTMSK:	0x%08x\n", mci_readl(host, INTMASK));
	dev_err(host->dev, ": CMDARG:	0x%08x\n", mci_readl(host, CMDARG));
	dev_err(host->dev, ": CMD:	0x%08x\n", mci_readl(host, CMD));
	dev_err(host->dev, ": RESP0:	0x%08x\n", mci_readl(host, RESP0));
	dev_err(host->dev, ": RESP1:	0x%08x\n", mci_readl(host, RESP1));
	dev_err(host->dev, ": RESP2:	0x%08x\n", mci_readl(host, RESP2));
	dev_err(host->dev, ": RESP3:	0x%08x\n", mci_readl(host, RESP3));
	dev_err(host->dev, ": MINTSTS:	0x%08x\n", mci_readl(host, MINTSTS));
	dev_err(host->dev, ": RINTSTS:	0x%08x\n", mci_readl(host, RINTSTS));
	dev_err(host->dev, ": STATUS:	0x%08x\n", mci_readl(host, STATUS));
	dev_err(host->dev, ": FIFOTH:	0x%08x\n", mci_readl(host, FIFOTH));
	dev_err(host->dev, ": CDETECT:	0x%08x\n", mci_readl(host, CDETECT));
	dev_err(host->dev, ": WRTPRT:	0x%08x\n", mci_readl(host, WRTPRT));
	dev_err(host->dev, ": GPIO:	0x%08x\n", mci_readl(host, GPIO));
	dev_err(host->dev, ": TCBCNT:	0x%08x\n", mci_readl(host, TCBCNT));
	dev_err(host->dev, ": TBBCNT:	0x%08x\n", mci_readl(host, TBBCNT));
	dev_err(host->dev, ": DEBNCE:	0x%08x\n", mci_readl(host, DEBNCE));
	dev_err(host->dev, ": USRID:	0x%08x\n", mci_readl(host, USRID));
	dev_err(host->dev, ": VERID:	0x%08x\n", mci_readl(host, VERID));
	dev_err(host->dev, ": HCON:	0x%08x\n", mci_readl(host, HCON));
	dev_err(host->dev, ": UHS_REG:	0x%08x\n", mci_readl(host, UHS_REG));
	dev_err(host->dev, ": BMOD:	0x%08x\n", mci_readl(host, BMOD));
	dev_err(host->dev, ": PLDMND:	0x%08x\n", mci_readl(host, PLDMND));
	dev_err(host->dev, ": DBADDR:	0x%08x\n", mci_readl(host, DBADDR));
	dev_err(host->dev, ": IDSTS:	0x%08x\n", mci_readl(host, IDSTS));
	dev_err(host->dev, ": IDINTEN:	0x%08x\n", mci_readl(host, IDINTEN));
	dev_err(host->dev, ": DSCADDR:	0x%08x\n", mci_readl(host, DSCADDR));
	dev_err(host->dev, ": BUFADDR:	0x%08x\n", mci_readl(host, BUFADDR));
	if (drv_data && drv_data->register_dump)
		drv_data->register_dump(host);
	dw_mci_cmd_reg_summary(host);
	dw_mci_status_reg_summary(host);
	dev_err(host->dev, ": ============== STATUS DUMP ================\n");
	dev_err(host->dev, ": cmd_status:      0x%08x\n", host->cmd_status);
	dev_err(host->dev, ": data_status:     0x%08x\n", host->data_status);
	dev_err(host->dev, ": pending_events:  0x%08lx\n", host->pending_events);
	dev_err(host->dev, ": completed_events:0x%08lx\n", host->completed_events);
	dev_err(host->dev, ": state:           %d\n", host->state_cmd);
	dev_err(host->dev, ": gate-clk:            %s\n",
			      atomic_read(&host->ciu_clk_cnt) ?
			      "enable" : "disable");
	dev_err(host->dev, ": ciu_en_win:           %d\n",
			atomic_read(&host->ciu_en_win));
	dev_err(host->dev, ": ===========================================\n");
}

static inline bool dw_mci_stop_abort_cmd(struct mmc_command *cmd)
{
	u32 op = cmd->opcode;

	if ((op == MMC_STOP_TRANSMISSION) ||
	    (op == MMC_GO_IDLE_STATE) ||
	    (op == MMC_GO_INACTIVE_STATE) ||
	    ((op == SD_IO_RW_DIRECT) && (cmd->arg & 0x80000000) &&
	     ((cmd->arg >> 9) & 0x1FFFF) == SDIO_CCCR_ABORT))
		return true;
	return false;
}

static u32 dw_mci_prepare_command(struct mmc_host *mmc, struct mmc_command *cmd)
{
	struct mmc_data	*data;
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	const struct dw_mci_drv_data *drv_data = slot->host->drv_data;
	u32 cmdr, argr;
	cmd->error = -EINPROGRESS;

	cmdr = cmd->opcode;
	argr = ((cmd->arg >> 9) & 0x1FFFF);

	if (cmdr == MMC_STOP_TRANSMISSION)
		cmdr |= SDMMC_CMD_STOP;
	else if (cmdr != MMC_SEND_STATUS &&
		 cmdr != MMC_SET_QUEUE_CONTEXT &&
		 cmdr != MMC_QUEUE_READ_ADDRESS)
		cmdr |= SDMMC_CMD_PRV_DAT_WAIT;

	if ((cmd->opcode == SD_IO_RW_DIRECT) &&
			(argr == SDIO_CCCR_ABORT)) {
		cmdr &= ~SDMMC_CMD_PRV_DAT_WAIT;
		cmdr |= SDMMC_CMD_STOP;
	}

	if (cmd->flags & MMC_RSP_PRESENT) {
		/* We expect a response, so set this bit */
		cmdr |= SDMMC_CMD_RESP_EXP;
		if (cmd->flags & MMC_RSP_136)
			cmdr |= SDMMC_CMD_RESP_LONG;
	}

	if (cmd->flags & MMC_RSP_CRC)
		cmdr |= SDMMC_CMD_RESP_CRC;

	if (host->quirks & DW_MMC_QUIRK_SW_DATA_TIMEOUT)
		cmdr |= SDMMC_CMD_CEATA_RD;

	data = cmd->data;
	if (data) {
		cmdr |= SDMMC_CMD_DAT_EXP;
		if (data->flags & MMC_DATA_STREAM)
			cmdr |= SDMMC_CMD_STRM_MODE;
		if (data->flags & MMC_DATA_WRITE)
			cmdr |= SDMMC_CMD_DAT_WR;
	}

	if (drv_data && drv_data->prepare_command)
		drv_data->prepare_command(slot->host, &cmdr);

	return cmdr;
}

static u32 dw_mci_prep_stop(struct dw_mci *host, struct mmc_command *cmd)
{
	struct mmc_command *stop = &host->stop;
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	u32 cmdr = cmd->opcode;

	memset(stop, 0, sizeof(struct mmc_command));

	if (cmdr == MMC_READ_SINGLE_BLOCK ||
			cmdr == MMC_READ_MULTIPLE_BLOCK ||
			cmdr == MMC_WRITE_BLOCK ||
			cmdr == MMC_WRITE_MULTIPLE_BLOCK) {
		stop->opcode = MMC_STOP_TRANSMISSION;
		stop->arg = 0;
		stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	} else if (cmdr == SD_IO_RW_EXTENDED) {
		stop->opcode = SD_IO_RW_DIRECT;
		stop->arg = 0x80000000;
		/* stop->arg &= ~(1 << 28); */
		stop->arg |= (cmd->arg >> 28) & 0x7;
		stop->arg |= SDIO_CCCR_ABORT << 9;
		stop->flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_AC;
	} else
		return 0;

	cmdr = stop->opcode | SDMMC_CMD_STOP |
		SDMMC_CMD_RESP_CRC | SDMMC_CMD_RESP_EXP;

	/* Use hold bit register */
	if (drv_data && drv_data->prepare_command)
		drv_data->prepare_command(host, &cmdr);

	return cmdr;
}

static void dw_mci_start_command(struct dw_mci *host,
				 struct mmc_command *cmd, u32 cmd_flags)
{
	struct mmc_data *data;
	u32 mask;

	host->cmd = cmd;
	data = cmd->data;
	mask = mci_readl(host, INTMASK);

	dev_vdbg(host->dev,
		 "start command: ARGR=0x%08x CMDR=0x%08x\n",
		 cmd->arg, cmd_flags);

	if ((host->quirks & DW_MCI_QUIRK_NO_DETECT_EBIT) &&
			data && (data->flags & MMC_DATA_READ)) {
		mask &= ~SDMMC_INT_EBE;
	} else {
		mask |= SDMMC_INT_EBE;
		mci_writel(host, RINTSTS, SDMMC_INT_EBE);
	}

	mci_writel(host, INTMASK, mask);

	if (MMC_CHECK_CMDQ_MODE(host)) {
		if (mci_readl(host, SHA_CMD_IS) & QRDY_INT)
			mci_writel(host, SHA_CMD_IS, QRDY_INT);
		else {
			mask = mci_readl(host, SHA_CMD_IE);
			if (!(mask & QRDY_INT_EN)) {
				mask |= QRDY_INT_EN;
				mci_writel(host, SHA_CMD_IE, mask);
			}
		}
	}

	mci_writel(host, CMDARG, cmd->arg);
	wmb();

	mci_writel(host, CMD, cmd_flags | SDMMC_CMD_START);
}

static void send_stop_cmd(struct dw_mci *host, struct mmc_data *data)
{
	dw_mci_start_command(host, data->stop, host->stop_cmdr);
}

/* DMA interface functions */
static void dw_mci_stop_dma(struct dw_mci *host)
{
	if (host->using_dma) {
		host->dma_ops->stop(host);
		host->dma_ops->cleanup(host);
		host->dma_ops->reset(host);
	} else {
		/* Data transfer was stopped by the interrupt handler */
		set_bit(EVENT_XFER_COMPLETE, &host->pending_events);
	}
}

static bool dw_mci_wait_reset(struct device *dev, struct dw_mci *host,
		unsigned int reset_val)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned int ctrl;

	ctrl = mci_readl(host, CTRL);
	ctrl |= reset_val;
	mci_writel(host, CTRL, ctrl);

	/* wait till resets clear */
	do {
		if (!(mci_readl(host, CTRL) & reset_val))
			return true;
	} while (time_before(jiffies, timeout));

	dev_err(dev, "Timeout resetting block (ctrl %#x)\n", ctrl);

	return false;
}

static void mci_send_cmd(struct dw_mci_slot *slot, u32 cmd, u32 arg)
{
	struct dw_mci *host = slot->host;
	unsigned long timeout = jiffies + msecs_to_jiffies(10);
	unsigned int cmd_status = 0;
	int try = 50;

	mci_writel(host, CMDARG, arg);
	wmb();
	mci_writel(host, CMD, SDMMC_CMD_START | cmd);

	do {
		while (time_before(jiffies, timeout)) {
			cmd_status = mci_readl(host, CMD);
			if (!(cmd_status & SDMMC_CMD_START))
				return;
		}

		dw_mci_wait_reset(host->dev, host, SDMMC_CTRL_RESET);
		mci_writel(host, CMD, SDMMC_CMD_START | cmd);
		timeout = jiffies + msecs_to_jiffies(10);
	} while (--try);

	dev_err(&slot->mmc->class_dev,
		"Timeout sending command (cmd %#x arg %#x status %#x)\n",
		cmd, arg, cmd_status);
}

static void dw_mci_ciu_reset(struct device *dev, struct dw_mci *host)
{
	struct dw_mci_slot *slot = host->cur_slot;
	unsigned long timeout = jiffies + msecs_to_jiffies(10);
	int retry = 10;
	u32 status;

	if (slot) {
		dw_mci_wait_reset(dev, host, SDMMC_CTRL_RESET);
		/* Check For DATA busy */
		do {

			while (time_before(jiffies, timeout)) {
				status = mci_readl(host, STATUS);
				if (!(status & SDMMC_DATA_BUSY))
					goto out;
			}

			dw_mci_wait_reset(host->dev, host, SDMMC_CTRL_RESET);
			timeout = jiffies + msecs_to_jiffies(10);
		} while (--retry);

out:
		mci_send_cmd(slot, SDMMC_CMD_UPD_CLK |
			SDMMC_CMD_PRV_DAT_WAIT, 0);
	}
}

static bool dw_mci_fifo_reset(struct device *dev, struct dw_mci *host)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	unsigned int ctrl;
	bool result;

	do {
		result = dw_mci_wait_reset(host->dev, host, SDMMC_CTRL_FIFO_RESET);

		if (!result)
			break;

		ctrl = mci_readl(host, STATUS);
		if (!(ctrl & SDMMC_STATUS_DMA_REQ)) {
			result = dw_mci_wait_reset(host->dev, host,
					SDMMC_CTRL_FIFO_RESET);
			if (result) {
				/* clear exception raw interrupts can not be handled
				   ex) fifo full => RXDR interrupt rising */
				ctrl = mci_readl(host, RINTSTS);
				ctrl = ctrl & ~(mci_readl(host, MINTSTS));
				if (ctrl)
					mci_writel(host, RINTSTS, ctrl);

				return true;
			}
		}
	} while (time_before(jiffies, timeout));

	dev_err(dev, "%s: Timeout while resetting host controller after err\n",
		__func__);

	return false;
}

static int dw_mci_get_dma_dir(struct mmc_data *data)
{
	if (data->flags & MMC_DATA_WRITE)
		return DMA_TO_DEVICE;
	else
		return DMA_FROM_DEVICE;
}

#ifdef CONFIG_MMC_DW_IDMAC
static void dw_mci_dma_cleanup(struct dw_mci *host)
{
	struct mmc_data *data = host->data;

	if (data)
		if (!data->host_cookie)
			dma_unmap_sg(host->dev,
				     data->sg,
				     data->sg_len,
				     dw_mci_get_dma_dir(data));
}

static void dw_mci_idmac_stop_dma(struct dw_mci *host)
{
	u32 temp;

	/* Disable and reset the IDMAC interface */
	temp = mci_readl(host, CTRL);
	temp &= ~SDMMC_CTRL_USE_IDMAC;
	mci_writel(host, CTRL, temp);

	/* reset the IDMAC interface */
	dw_mci_wait_reset(host->dev, host, SDMMC_CTRL_DMA_RESET);

	/* Stop the IDMAC running */
	temp = mci_readl(host, BMOD);
	temp &= ~(SDMMC_IDMAC_ENABLE | SDMMC_IDMAC_FB);
	mci_writel(host, BMOD, temp);
}

static void dw_mci_idma_reset_dma(struct dw_mci *host)
{
	u32 temp;

	temp = mci_readl(host, BMOD);
	/* Software reset of DMA */
	temp |= SDMMC_IDMAC_SWRESET;
	mci_writel(host, BMOD, temp);
}

static void dw_mci_idmac_complete_dma(struct dw_mci *host)
{
	struct mmc_data *data = host->data;

	dev_vdbg(host->dev, "DMA complete\n");

	host->dma_ops->cleanup(host);

	/*
	 * If the card was removed, data will be NULL. No point in trying to
	 * send the stop command or waiting for NBUSY in this case.
	 */
	if (data) {
		set_bit(EVENT_XFER_COMPLETE, &host->pending_events);
		tasklet_schedule(&host->tasklet);
	}
}

static void dw_mci_translate_sglist(struct dw_mci *host, struct mmc_data *data,
				    unsigned int sg_len)
{
	int i, j;
	int desc_cnt = 0;
	struct idmac_desc *desc = host->sg_cpu;
	unsigned int rw_size = DW_MMC_MAX_TRANSFER_SIZE;
#ifdef CONFIG_MMC_DW_FMP_DM_CRYPT
	unsigned int sector = 0;
	unsigned int sector_key = DW_MMC_BYPASS_SECTOR;
	struct mmc_blk_request *brq = NULL;
	struct mmc_queue_req *mq_rq = NULL;

	if (data->mrq->host) {
		/* it means this request comes from block i/o */
		brq = container_of(data, struct mmc_blk_request, data);
		if (brq) {
			mq_rq = container_of(brq, struct mmc_queue_req, brq);
			sector = mq_rq->req->bio->bi_sector;
			rw_size = (mq_rq->req->bio->bi_sensitive_data == 1) ?
				DW_MMC_SECTOR_SIZE : DW_MMC_MAX_TRANSFER_SIZE;
			sector_key = (mq_rq->req->bio->bi_sensitive_data == 1) ?
				DW_MMC_ENCRYPTION_SECTOR : DW_MMC_BYPASS_SECTOR;
			mq_rq->req->bio->bi_sensitive_data = 0;
		}
	}
#endif
	for (i = 0; i < sg_len; i++) {
		unsigned int length = sg_dma_len(&data->sg[i]);
		unsigned int sz_per_desc;
		unsigned int left = length;
		u32 mem_addr = sg_dma_address(&data->sg[i]);

		for (j = 0; j < (length + rw_size - 1) / rw_size; j++) {
			/*
			 * Set the OWN bit
			 * and disable interrupts for this descriptor
			 */
			desc->des0 = IDMAC_DES0_OWN | IDMAC_DES0_DIC |
					IDMAC_DES0_CH;

			/* Buffer length */
			sz_per_desc = min(left, rw_size);
			desc->des1 = length;
			IDMAC_SET_BUFFER1_SIZE(desc, sz_per_desc);

			/* Physical address to DMA to/from */
			desc->des2 = mem_addr;
#ifdef CONFIG_MMC_DW_FMP_DM_CRYPT
			if (sector_key == DW_MMC_ENCRYPTION_SECTOR) {
				desc->des4 = sector;
				desc->des5 = 0;
				desc->des6 = 0;
				desc->des7 = 0;
			} else
				desc->des4 = DW_MMC_BYPASS_SECTOR;
			sector += rw_size / DW_MMC_SECTOR_SIZE;
#else
			desc->des4 = DW_MMC_BYPASS_SECTOR;
#endif
			desc++;
			desc_cnt++;
			mem_addr += sz_per_desc;
			left -= sz_per_desc;
		}

	}

	/* Set first descriptor */
	desc = host->sg_cpu;
	desc->des0 |= IDMAC_DES0_FD;

	/* Set last descriptor */
	desc = host->sg_cpu + (desc_cnt - 1) * sizeof(struct idmac_desc);
	desc->des0 &= ~(IDMAC_DES0_CH | IDMAC_DES0_DIC);
	desc->des0 |= IDMAC_DES0_LD;

	wmb();
}

static void dw_mci_idmac_start_dma(struct dw_mci *host, unsigned int sg_len)
{
	u32 temp;

	dw_mci_translate_sglist(host, host->data, sg_len);

	/* Select IDMAC interface */
	temp = mci_readl(host, CTRL);
	temp |= SDMMC_CTRL_USE_IDMAC;
	mci_writel(host, CTRL, temp);

	wmb();

	/* Enable the IDMAC */
	temp = mci_readl(host, BMOD);
	temp |= SDMMC_IDMAC_ENABLE | SDMMC_IDMAC_FB;
	mci_writel(host, BMOD, temp);

	/* Start it running */
	mci_writel(host, PLDMND, 1);
}

static int dw_mci_idmac_init(struct dw_mci *host)
{
	struct idmac_desc *p;
	int i;

	/* Number of descriptors in the ring buffer */
	host->ring_size = host->desc_sz * PAGE_SIZE / sizeof(struct idmac_desc);

	/* Forward link the descriptor list */
	for (i = 0, p = host->sg_cpu; i < host->ring_size *
		MMC_DW_IDMAC_MULTIPLIER - 1; i++, p++)
		p->des3 = host->sg_dma + (sizeof(struct idmac_desc) * (i + 1));

	/* Set the last descriptor as the end-of-ring descriptor */
	p->des3 = host->sg_dma;
	p->des0 = IDMAC_DES0_ER;

	mci_writel(host, BMOD, SDMMC_IDMAC_SWRESET);

	/* Mask out interrupts - get Tx & Rx complete only */
	mci_writel(host, IDINTEN, SDMMC_IDMAC_INT_NI | SDMMC_IDMAC_INT_RI |
		   SDMMC_IDMAC_INT_TI);

	/* Set the descriptor base address */
	mci_writel(host, DBADDR, host->sg_dma);

	host->align_size = (host->data_shift == 3) ? 8 : 4;

	return 0;
}

static const struct dw_mci_dma_ops dw_mci_idmac_ops = {
	.init = dw_mci_idmac_init,
	.start = dw_mci_idmac_start_dma,
	.stop = dw_mci_idmac_stop_dma,
	.reset = dw_mci_idma_reset_dma,
	.complete = dw_mci_idmac_complete_dma,
	.cleanup = dw_mci_dma_cleanup,
};
#endif /* CONFIG_MMC_DW_IDMAC */

static int dw_mci_pre_dma_transfer(struct dw_mci *host,
				   struct mmc_data *data,
				   bool next)
{
	struct scatterlist *sg;
	unsigned int i, sg_len;
	unsigned int align_mask = host->align_size - 1;

	if (!next && data->host_cookie)
		return data->host_cookie;

	/*
	 * We don't do DMA on "complex" transfers, i.e. with
	 * non-word-aligned buffers or lengths. Also, we don't bother
	 * with all the DMA setup overhead for short transfers.
	 */
	if (data->blocks * data->blksz < DW_MCI_DMA_THRESHOLD)
		return -EINVAL;

	if (data->blksz & align_mask)
		return -EINVAL;

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & align_mask || sg->length & align_mask)
			return -EINVAL;
	}

	sg_len = dma_map_sg(host->dev,
			    data->sg,
			    data->sg_len,
			    dw_mci_get_dma_dir(data));
	if (sg_len == 0)
		return -EINVAL;

	if (next)
		data->host_cookie = sg_len;

	return sg_len;
}

static void dw_mci_pre_req(struct mmc_host *mmc,
			   struct mmc_request *mrq,
			   bool is_first_req)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!slot->host->use_dma || !data)
		return;

	if (data->host_cookie) {
		data->host_cookie = 0;
		return;
	}

	if (dw_mci_pre_dma_transfer(slot->host, mrq->data, 1) < 0)
		data->host_cookie = 0;
}

static void dw_mci_post_req(struct mmc_host *mmc,
			    struct mmc_request *mrq,
			    int err)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!slot->host->use_dma || !data)
		return;

	if (data->host_cookie)
		dma_unmap_sg(slot->host->dev,
			     data->sg,
			     data->sg_len,
			     dw_mci_get_dma_dir(data));
	data->host_cookie = 0;
}

static int dw_mci_submit_data_dma(struct dw_mci *host, struct mmc_data *data)
{
	int sg_len;
	u32 temp;

	host->using_dma = 0;

	/* If we don't have a channel, we can't do DMA */
	if (!host->use_dma)
		return -ENODEV;

	if (host->use_dma && host->dma_ops->init)
		host->dma_ops->init(host);

	sg_len = dw_mci_pre_dma_transfer(host, data, 0);
	if (sg_len < 0) {
		host->dma_ops->stop(host);
		return sg_len;
	}

	host->using_dma = 1;

	dev_vdbg(host->dev,
		 "sd sg_cpu: %#lx sg_dma: %#lx sg_len: %d\n",
		 (unsigned long)host->sg_cpu, (unsigned long)host->sg_dma,
		 sg_len);

	/* Enable the DMA interface */
	temp = mci_readl(host, CTRL);
	temp |= SDMMC_CTRL_DMA_ENABLE;
	mci_writel(host, CTRL, temp);

	/* Disable RX/TX IRQs, let DMA handle it */
	mci_writel(host, RINTSTS, SDMMC_INT_TXDR | SDMMC_INT_RXDR);
	temp = mci_readl(host, INTMASK);
	temp  &= ~(SDMMC_INT_RXDR | SDMMC_INT_TXDR);
	mci_writel(host, INTMASK, temp);

	host->dma_ops->start(host, sg_len);

	return 0;
}

static void dw_mci_submit_data(struct dw_mci *host, struct mmc_data *data)
{
	u32 temp;
	struct dw_mci_slot *slot = host->cur_slot;
	struct mmc_card *card = slot->mmc->card;

	data->error = -EINPROGRESS;

	WARN_ON(host->data);
	host->sg = NULL;
	host->data = data;

	if (card && mmc_card_sdio(card)) {
		unsigned int rxwmark_val, msize_val, i;
		unsigned int msize[8] = {1, 4, 8, 16, 32, 64, 128, 256};

		for (i = 1; i < sizeof(msize) / sizeof(unsigned int); i++) {
			if (data->blksz != 0 &&
				(data->blksz / (1 << host->data_shift)) % msize[i] == 0)
				continue;
			else
				break;
		}
		if (data->blksz < host->fifo_depth / 2) {
			if (i > 1) {
				msize_val = i - 1;
				rxwmark_val = msize[i-1] - 1;
			} else {
				msize_val = 0;
				rxwmark_val = 1;
			}
		} else {
			if (i > 5) {
				msize_val = i - 5;
				rxwmark_val = msize[i-5] - 1;
			} else {
				msize_val = 0;
				rxwmark_val = 1;
			}
		}
		dev_dbg(&slot->mmc->class_dev,
			"msize_val : %d, rxwmark_val : %d\n",
			msize_val, rxwmark_val);

		host->fifoth_val = ((msize_val << 28) | (rxwmark_val << 16) |
				   ((host->fifo_depth/2) << 0));

		mci_writel(host, FIFOTH, host->fifoth_val);

		if (mmc_card_uhs(card)
				&& card->host->caps & MMC_CAP_UHS_SDR104
				&& data->flags & MMC_DATA_READ)
			mci_writel(host, CDTHRCTL, data->blksz << 16 | 1);
	}

	if (data->flags & MMC_DATA_READ)
		host->dir_status = DW_MCI_RECV_STATUS;
	else
		host->dir_status = DW_MCI_SEND_STATUS;

	if (dw_mci_submit_data_dma(host, data)) {
		int flags = SG_MITER_ATOMIC;

		if (SDMMC_GET_FCNT(mci_readl(host, STATUS)))
			dw_mci_wait_reset(host->dev, host, SDMMC_CTRL_FIFO_RESET);

		if (host->data->flags & MMC_DATA_READ)
			flags |= SG_MITER_TO_SG;
		else
			flags |= SG_MITER_FROM_SG;

		sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
		host->sg = data->sg;
		host->part_buf_start = 0;
		host->part_buf_count = 0;

		mci_writel(host, RINTSTS, SDMMC_INT_TXDR | SDMMC_INT_RXDR);
		temp = mci_readl(host, INTMASK);
		temp |= SDMMC_INT_TXDR | SDMMC_INT_RXDR;
		mci_writel(host, INTMASK, temp);

		temp = mci_readl(host, CTRL);
		temp &= ~SDMMC_CTRL_DMA_ENABLE;
		mci_writel(host, CTRL, temp);
	}
}

static bool dw_mci_wait_data_busy(struct dw_mci *host, struct mmc_request *mrq)
{
	u32 status;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	struct dw_mci_slot *slot = host->cur_slot;
	int try = 6;
	u32 clkena;
	bool ret = false;

	do {
		do {
			status = mci_readl(host, STATUS);
			if (!(status & SDMMC_DATA_BUSY)) {
				ret = true;
				goto out;
			}

			usleep_range(10, 20);
		} while (time_before(jiffies, timeout));

		/* card is checked every 1s by CMD13 at least */
		if (mrq->cmd->opcode == MMC_SEND_STATUS)
			return true;

		dw_mci_wait_reset(host->dev, host, SDMMC_CTRL_RESET);
		/* After CTRL Reset, Should be needed clk val to CIU */
		if (host->cur_slot) {
			/* Disable low power mode */
			clkena = mci_readl(host, CLKENA);
			clkena &= ~((SDMMC_CLKEN_LOW_PWR) << slot->id);
			mci_writel(host, CLKENA, clkena);

			mci_send_cmd(host->cur_slot,
				SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);
		}
		timeout = jiffies + msecs_to_jiffies(500);
	} while (--try);
out:
	if (host->cur_slot) {
		if (ret == false)
			dev_err(host->dev, "Data[0]: data is busy\n");

		/* enable clock */
		mci_writel(host, CLKENA, ((SDMMC_CLKEN_ENABLE |
			   SDMMC_CLKEN_LOW_PWR) << slot->id));

		/* inform CIU */
		mci_send_cmd(slot,
			     SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);
	}

	return ret;
}

static void dw_mci_setup_bus(struct dw_mci_slot *slot, bool force_clkinit)
{
	struct dw_mci *host = slot->host;
	u32 div, actual_speed;
	bool reset_div = false;
	u32 clk_en_a;

	if (slot->clock && ((slot->clock != host->current_speed) || force_clkinit)) {
		do {
			div = host->bus_hz / slot->clock;
			if ((host->bus_hz % slot->clock) &&
				(host->bus_hz > slot->clock))
				/*
				 * move the + 1 after the divide to prevent
				 * over-clocking the card.
				 */
				div++;

			div = (host->bus_hz != slot->clock) ?
				DIV_ROUND_UP(div, 2) : 0;

			/* CLKDIV limitation is 0xFF */
			if (div > 0xFF)
				div = 0xFF;

			actual_speed = div ?
				(host->bus_hz / div) >> 1 : host->bus_hz;

			/* Change SCLK_MMC */
			if (actual_speed > slot->clock &&
				host->bus_hz != 0 && !reset_div) {
				dev_err(host->dev,
					"Actual clock is high than a reqeust clock."
					"Source clock is needed to change\n");
				reset_div = true;
				slot->mmc->ios.timing = MMC_TIMING_LEGACY;
				host->drv_data->set_ios(host, 0, &slot->mmc->ios);
			} else
				reset_div = false;
		} while (reset_div);

		dev_info(&slot->mmc->class_dev,
			 "Bus speed (slot %d) = %dHz (slot req %dHz, actual %dHZ"
			 " div = %d)\n", slot->id, host->bus_hz, slot->clock,
			 div ? ((host->bus_hz / div) >> 1) : host->bus_hz, div);

		/* disable clock */
		mci_writel(host, CLKENA, 0);
		mci_writel(host, CLKSRC, 0);

		/* inform CIU */
		mci_send_cmd(slot,
			     SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

		/* set clock to desired speed */
		mci_writel(host, CLKDIV, div);

		/* inform CIU */
		mci_send_cmd(slot,
			     SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

		/* enable clock; only low power if no SDIO */
		clk_en_a = SDMMC_CLKEN_ENABLE << slot->id;
		if (!(mci_readl(host, INTMASK) & SDMMC_INT_SDIO(slot->id)))
			clk_en_a |= SDMMC_CLKEN_LOW_PWR << slot->id;
		mci_writel(host, CLKENA, clk_en_a);

		/* inform CIU */
		mci_send_cmd(slot,
			     SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

		host->current_speed = slot->clock;
	}

	/* Set the current slot bus width */
	mci_writel(host, CTYPE, (slot->ctype << slot->id));
}

static void __dw_mci_start_request(struct dw_mci *host,
				   struct dw_mci_slot *slot,
				   struct mmc_command *cmd)
{
	struct mmc_request *mrq;
	struct mmc_data	*data;
	u32 cmdflags;

	mrq = slot->mrq;
	if (host->pdata->select_slot)
		host->pdata->select_slot(slot->id);

	if (host->pdata->sw_timeout)
		mod_timer(&host->timer,
			jiffies + msecs_to_jiffies(host->pdata->sw_timeout));
	else
		mod_timer(&host->timer, jiffies + msecs_to_jiffies(10000));

	host->cur_slot = slot;
	host->mrq_cmd = mrq;

	host->cmd_status = 0;
	host->completed_events = 0;

	if (cmd->data) {
		host->mrq_dat = mrq;

		host->stop_cmdr = 0;
		host->stop_snd = false;

		host->data_status = 0;
		host->dir_status = 0;
		host->pending_events = 0;
	} else {
		if (MMC_CHECK_CMDQ_MODE(host))
			clear_bit(EVENT_CMD_COMPLETE, &host->pending_events);
		else
			host->pending_events = 0;
	}

	if (host->pdata->tp_mon_tbl)
		host->cmd_cnt++;

	data = cmd->data;
	if (data) {
		dw_mci_set_timeout(host);
		mci_writel(host, BYTCNT, data->blksz*data->blocks);
		mci_writel(host, BLKSIZ, data->blksz);
		if (host->pdata->tp_mon_tbl)
			host->transferred_cnt += data->blksz * data->blocks;
	}

	cmdflags = dw_mci_prepare_command(slot->mmc, cmd);

	/* this is the first command, send the initialization clock */
	if (test_and_clear_bit(DW_MMC_CARD_NEED_INIT, &slot->flags))
		cmdflags |= SDMMC_CMD_INIT;

	if (data) {
		dw_mci_submit_data(host, data);
		wmb();
	}

	dw_mci_start_command(host, cmd, cmdflags);

	if (mrq->stop)
		host->stop_cmdr = dw_mci_prepare_command(slot->mmc, mrq->stop);
	else {
		if (data)
			host->stop_cmdr = dw_mci_prep_stop(host, cmd);
	}
}

static void dw_mci_start_request(struct dw_mci *host,
				 struct dw_mci_slot *slot)
{
	struct mmc_request *mrq = slot->mrq;
	struct mmc_command *cmd;

	host->req_state = DW_MMC_REQ_BUSY;

	if (mrq->cmd->data &&
	    (mrq->cmd->error || mrq->cmd->data->error))
		cmd = mrq->stop;
	else
		cmd = mrq->sbc ? mrq->sbc : mrq->cmd;
	__dw_mci_start_request(host, slot, cmd);
}

/* must be called with host->lock held */
static void dw_mci_queue_request(struct dw_mci *host, struct dw_mci_slot *slot,
				 struct mmc_request *mrq)
{
	dev_vdbg(&slot->mmc->class_dev, "queue request: state=%d\n",
		 host->state_cmd);

	if (host->state_cmd == STATE_IDLE && host->tasklet_state == 0) {
		slot->mrq = mrq;
		host->state_cmd = STATE_SENDING_CMD;
		dw_mci_start_request(host, slot);
	} else {
		list_add_tail(&mrq->hlist, &slot->mrq_list);
		list_add_tail(&slot->queue_node, &host->queue);
	}
}

static void dw_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;

	if (!test_bit(DW_MMC_CARD_PRESENT, &slot->flags)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	if (!MMC_CHECK_CMDQ_MODE(host)) {
		if (!dw_mci_stop_abort_cmd(mrq->cmd)) {
			if (!dw_mci_wait_data_busy(host, mrq)) {
				mrq->cmd->error = -ENOTRECOVERABLE;
				mmc_request_done(mmc, mrq);
				return;
			}
		}
	}

	spin_lock_bh(&host->lock);

	dw_mci_queue_request(host, slot, mrq);

	spin_unlock_bh(&host->lock);
}

static void dw_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	const struct dw_mci_drv_data *drv_data = slot->host->drv_data;
	u32 regs;
	bool cclk_request_turn_off = 0;

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_4:
		slot->ctype = SDMMC_CTYPE_4BIT;
		break;
	case MMC_BUS_WIDTH_8:
		slot->ctype = SDMMC_CTYPE_8BIT;
		break;
	default:
		/* set default 1 bit mode */
		slot->ctype = SDMMC_CTYPE_1BIT;
	}

	regs = mci_readl(slot->host, UHS_REG);

	if (ios->timing == MMC_TIMING_UHS_DDR50 ||
	    ios->timing == MMC_TIMING_MMC_HS200_DDR) {
		if (!mmc->tuning_progress)
			regs |= ((SDMMC_UHS_DDR_MODE << slot->id) << 16);
	} else
		regs &= ~((SDMMC_UHS_DDR_MODE << slot->id) << 16);

	if (slot->host->pdata->caps &
				(MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
				 MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR104 |
				 MMC_CAP_UHS_DDR50))
		regs |= (0x1 << slot->id);

	mci_writel(slot->host, UHS_REG, regs);

	if (ios->timing == MMC_TIMING_MMC_HS200_DDR)
		if (!mmc->tuning_progress)
			mci_writel(slot->host, CDTHRCTL, 512 << 16 | 1);

	if (ios->clock) {
		/*
		 * Use mirror of ios->clock to prevent race with mmc
		 * core ios update when finding the minimum.
		 */
		slot->clock = ios->clock;
		pm_qos_update_request(&slot->host->pm_qos_int,
				slot->host->pdata->qos_int_level);
	} else {
		pm_qos_update_request(&slot->host->pm_qos_int, 0);
		cclk_request_turn_off = 1;
	}

	if (drv_data && drv_data->set_ios) {
		drv_data->set_ios(slot->host, mmc->tuning_progress, ios);

		/* Reset the min/max in case the set_ios() changed bus_hz */
		mmc->f_min = DIV_ROUND_UP(slot->host->bus_hz, 510);
		mmc->f_max = slot->host->bus_hz;
	}

	/*
	 * CIU clock should be enabled because dw_mci_setup_bus is called
	 * unconditionally in this function
	 */
	atomic_inc_return(&slot->host->ciu_en_win);
	dw_mci_ciu_clk_en(slot->host, false);

	/* Slot specific timing and width adjustment */
	dw_mci_setup_bus(slot, false);
	atomic_dec_return(&slot->host->ciu_en_win);

	switch (ios->power_mode) {
	case MMC_POWER_UP:
		set_bit(DW_MMC_CARD_NEED_INIT, &slot->flags);
		if (slot->host->pdata->tp_mon_tbl)
			schedule_delayed_work(&slot->host->tp_mon, HZ);
		break;
	case MMC_POWER_OFF:
		cclk_request_turn_off = 1;

		if (slot->host->pdata->tp_mon_tbl) {
			cancel_delayed_work_sync(&slot->host->tp_mon);
			pm_qos_update_request(&slot->host->pm_qos_mif, 0);
			pm_qos_update_request(&slot->host->pm_qos_cpu, 0);
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
			pm_qos_update_request(&slot->host->pm_qos_kfc, 0);
#endif
		}
		break;
	default:
		break;
	}

	if (cclk_request_turn_off)
		dw_mci_ciu_clk_dis(slot->host);
}

static int dw_mci_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	const struct dw_mci_drv_data *drv_data = slot->host->drv_data;
	int ret = 0;

	if (drv_data && drv_data->execute_tuning)
		ret = drv_data->execute_tuning(slot->host, opcode);

	return ret;
}

static int dw_mci_get_ro(struct mmc_host *mmc)
{
	int read_only;
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci_board *brd = slot->host->pdata;

	/* Use platform get_ro function, else try on board write protect */
	if (slot->quirks & DW_MCI_SLOT_QUIRK_NO_WRITE_PROTECT)
		read_only = 0;
	else if (brd->get_ro)
		read_only = brd->get_ro(slot->id);
	else if (gpio_is_valid(slot->wp_gpio))
		read_only = gpio_get_value(slot->wp_gpio);
	else
		read_only =
			mci_readl(slot->host, WRTPRT) & (1 << slot->id) ? 1 : 0;

	dev_dbg(&mmc->class_dev, "card is %s\n",
		read_only ? "read-only" : "read-write");

	return read_only;
}

static int dw_mci_get_cd(struct mmc_host *mmc)
{
	int present;
	int temp;
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	struct dw_mci_board *brd = host->pdata;
	const struct dw_mci_drv_data *drv_data = host->drv_data;

	/* Use platform get_cd function, else try onboard card detect */
	if (brd->quirks & DW_MCI_QUIRK_BROKEN_CARD_DETECTION)
		present = 1;
	else if (brd->get_cd)
		present = !brd->get_cd(slot->id);
	else
		present = (mci_readl(host, CDETECT) & (1 << slot->id))
			== 0 ? 1 : 0;

	if (drv_data && drv_data->misc_control) {
		temp = drv_data->misc_control(host,
				CTRL_CHECK_CD_GPIO, NULL);
		if (temp != -1)
			present = temp;
	}

	if (present)
		dev_dbg(&mmc->class_dev, "card is present\n");
	else
		dev_dbg(&mmc->class_dev, "card is not present\n");

	return present;
}

/*
 * Disable lower power mode.
 *
 * Low power mode will stop the card clock when idle.  According to the
 * description of the CLKENA register we should disable low power mode
 * for SDIO cards if we need SDIO interrupts to work.
 *
 * This function is fast if low power mode is already disabled.
 */
static void dw_mci_disable_low_power(struct dw_mci_slot *slot)
{
	struct dw_mci *host = slot->host;
	u32 clk_en_a;
	const u32 clken_low_pwr = SDMMC_CLKEN_LOW_PWR << slot->id;

	clk_en_a = mci_readl(host, CLKENA);

	if (clk_en_a & clken_low_pwr) {
		mci_writel(host, CLKENA, clk_en_a & ~clken_low_pwr);
		mci_send_cmd(slot, SDMMC_CMD_UPD_CLK |
			     SDMMC_CMD_PRV_DAT_WAIT, 0);
	}
}

static void dw_mci_enable_sdio_irq(struct mmc_host *mmc, int enb)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	u32 int_mask;

	/* Enable/disable Slot Specific SDIO interrupt */
	int_mask = mci_readl(host, INTMASK);
	if (enb) {
		/*
		 * Turn off low power mode if it was enabled.  This is a bit of
		 * a heavy operation and we disable / enable IRQs a lot, so
		 * we'll leave low power mode disabled and it will get
		 * re-enabled again in dw_mci_setup_bus().
		 */
		dw_mci_disable_low_power(slot);

		mci_writel(host, INTMASK,
			   (int_mask | SDMMC_INT_SDIO(slot->id)));
	} else {
		mci_writel(host, INTMASK,
			   (int_mask & ~SDMMC_INT_SDIO(slot->id)));
	}
}

static int dw_mci_3_3v_signal_voltage_switch(struct dw_mci_slot *slot)
{
	struct dw_mci *host = slot->host;
	u32 reg;
	int ret = 0;

	if (host->vqmmc) {
		ret = regulator_set_voltage(host->vqmmc, 3300000, 3300000);
		if (ret) {
			dev_warn(host->dev, "Switching to 3.3V signalling "
					"voltage failed\n");
			return -EIO;
		}
	} else {
		reg = mci_readl(slot->host, UHS_REG);
		reg &= ~(0x1 << slot->id);
		mci_writel(slot->host, UHS_REG, reg);
	}

	/* Wait for 5ms */
	usleep_range(5000, 5500);

	return ret;
}

static int dw_mci_1_8v_signal_voltage_switch(struct dw_mci_slot *slot)
{
	struct dw_mci *host = slot->host;
	unsigned long timeout = jiffies + msecs_to_jiffies(10);
	u32 reg;
	int ret = 0, retry = 10;
	u32 status;

	dw_mci_wait_reset(host->dev, host, SDMMC_CTRL_RESET);

	/* Check For DATA busy */
	do {

		while (time_before(jiffies, timeout)) {
			status = mci_readl(host, STATUS);
			if (!(status & SDMMC_DATA_BUSY))
				goto out;
		}

		dw_mci_wait_reset(host->dev, host, SDMMC_CTRL_RESET);
		timeout = jiffies + msecs_to_jiffies(10);
	} while (--retry);

out:
	atomic_inc_return(&host->ciu_en_win);
	dw_mci_ciu_clk_en(host, false);
	reg = mci_readl(host, CLKENA);
	reg &= ~((SDMMC_CLKEN_LOW_PWR | SDMMC_CLKEN_ENABLE) << slot->id);
	mci_writel(host, CLKENA, reg);
	mci_send_cmd(slot, SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

	if (host->vqmmc) {
		ret = regulator_set_voltage(host->vqmmc, 2000000, 2000000);
		if (ret) {
			dev_warn(host->dev, "Switching to 1.8V signalling "
					"voltage failed\n");
			return -EIO;
		}
	} else {
		reg = mci_readl(slot->host, UHS_REG);
		reg |= (0x1 << slot->id);
		mci_writel(slot->host, UHS_REG, reg);
	}

	/* Wait for 5ms */
	usleep_range(5000, 5500);

	dw_mci_ciu_clk_en(host, false);
	reg = mci_readl(host, CLKENA);
	reg |= SDMMC_CLKEN_ENABLE << slot->id;
	mci_writel(host, CLKENA, reg);
	mci_send_cmd(slot, SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);
	atomic_dec_return(&host->ciu_en_win);

	return ret;
}

static int dw_mci_start_signal_voltage_switch(struct mmc_host *mmc,
		struct mmc_ios *ios)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		return dw_mci_3_3v_signal_voltage_switch(slot);
	else if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180)
		return dw_mci_1_8v_signal_voltage_switch(slot);
	else
		return 0;
}

static void dw_mci_hw_reset(struct mmc_host *host)
{
	struct dw_mci_slot *slot = mmc_priv(host);
	struct dw_mci_board *brd = slot->host->pdata;

	dev_dbg(&host->class_dev, "card is going to h/w reset\n");

	/* Use platform hw_reset function */
	if (brd->hw_reset)
		brd->hw_reset(slot->id);
}

static int dw_mci_card_busy(struct mmc_host *host)
{
	struct dw_mci_slot *slot = mmc_priv(host);
	u32 status, ret = -1;

	status = mci_readl(slot->host, STATUS);
	ret = (status & SDMMC_DATA_BUSY);

	return ret;
}

static const struct mmc_host_ops dw_mci_ops = {
	.request		= dw_mci_request,
	.pre_req		= dw_mci_pre_req,
	.post_req		= dw_mci_post_req,
	.set_ios		= dw_mci_set_ios,
	.get_ro			= dw_mci_get_ro,
	.get_cd			= dw_mci_get_cd,
	.enable_sdio_irq	= dw_mci_enable_sdio_irq,
	.execute_tuning		= dw_mci_execute_tuning,
	.start_signal_voltage_switch	= dw_mci_start_signal_voltage_switch,
	.hw_reset		= dw_mci_hw_reset,
	.card_busy		= dw_mci_card_busy,
};

static void dw_mci_request_end(struct dw_mci *host, struct mmc_request *mrq,
				enum dw_mci_state *state)
	__releases(&host->lock)
	__acquires(&host->lock)
{
	struct mmc_host	*prev_mmc = host->cur_slot->mmc;

	del_timer(&host->timer);

	host->req_state = DW_MMC_REQ_IDLE;

	(*state) = STATE_IDLE;

	spin_unlock(&host->lock);
	mmc_request_done(prev_mmc, mrq);
	spin_lock(&host->lock);
}

static void dw_mci_command_complete(struct dw_mci *host, struct mmc_command *cmd)
{
	u32 status = host->cmd_status;

	host->cmd_status = 0;

	/* Read the response from the card (up to 16 bytes) */
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[3] = mci_readl(host, RESP0);
			cmd->resp[2] = mci_readl(host, RESP1);
			cmd->resp[1] = mci_readl(host, RESP2);
			cmd->resp[0] = mci_readl(host, RESP3);
		} else {
			cmd->resp[0] = mci_readl(host, RESP0);
			cmd->resp[1] = 0;
			cmd->resp[2] = 0;
			cmd->resp[3] = 0;
		}
	}

	if (status & SDMMC_INT_RTO)
		cmd->error = -ETIMEDOUT;
	else if ((cmd->flags & MMC_RSP_CRC) && (status & SDMMC_INT_RCRC))
		cmd->error = -EILSEQ;
	else if (status & SDMMC_INT_RESP_ERR)
		cmd->error = -EIO;
	else
		cmd->error = 0;

	if (cmd->error) {
		/* newer ip versions need a delay between retries */
		if (host->quirks & DW_MCI_QUIRK_RETRY_DELAY)
			mdelay(20);

		if (cmd->data) {
			dw_mci_stop_dma(host);
			host->data = NULL;
		}
	}
}

static void dw_mci_dto_timer(unsigned long data)
{
	struct dw_mci *host = (struct dw_mci *)data;
	u32 fifo_cnt = 0, done = false;

	if (!(host->quirks & DW_MMC_QUIRK_SW_DATA_TIMEOUT))
		return;

	/* Check Data trasnfer Done */
	if (host->pending_events & EVENT_DATA_COMPLETE ||
			host->completed_events & EVENT_DATA_COMPLETE)
		done = true;

	/* Check Data Transfer start */
	fifo_cnt = mci_readl(host, STATUS);
	fifo_cnt = (fifo_cnt >> 17) & 0x1FFF;
	if (fifo_cnt > 0)
		done = true;

	if (done == true) {
		dev_info(host->dev,
		"Done, S/W timer for data timeout %d ms fifo count %d\n",
		 host->dto_cnt, fifo_cnt);
		return;
	}

	if (host->dto_cnt < (DRTO / DRTO_MON_PERIOD)) {
		/* monitoring */
		host->dto_cnt++;
		mod_timer(&host->dto_timer, jiffies + msecs_to_jiffies(DRTO_MON_PERIOD));
	} else {
		/* data timeout */
		host->data_status |= SDMMC_INT_DTO;
		set_bit(EVENT_DATA_ERROR, &host->pending_events);
		tasklet_schedule(&host->tasklet);
	}
}

static int dw_mci_tasklet_cmd(struct dw_mci *host)
{
	struct mmc_data	*data;
	struct mmc_command *cmd;
	enum dw_mci_state state;
	enum dw_mci_state prev_state;
	u32 done = 0;

	state = host->state_cmd;
	data = host->data;

	do {
		prev_state = state;

		switch (state) {
		case STATE_IDLE:
			break;

		case STATE_SENDING_CMD:
			if (!test_and_clear_bit(EVENT_CMD_COMPLETE,
						&host->pending_events))
				break;

			cmd = host->cmd;
			set_bit(EVENT_CMD_COMPLETE, &host->completed_events);
			dw_mci_command_complete(host, cmd);
			if (cmd && cmd == host->mrq_cmd->sbc && !cmd->error) {
				prev_state = state = STATE_SENDING_CMD;
				__dw_mci_start_request(host, host->cur_slot,
						       host->mrq_cmd->cmd);
				goto exit_cmd;
			}

			if (cmd->data && cmd->error &&
					cmd != host->mrq_cmd->data->stop) {
				/* To avoid fifo full condition */
				dw_mci_fifo_reset(host->dev, host);
				dw_mci_ciu_reset(host->dev, host);

				if (MMC_CHECK_CMDQ_MODE(host)) {
					list_add_tail(&host->mrq_cmd->hlist,
						&host->cur_slot->mrq_list);
					del_timer(&host->timer);
					dw_mci_stop_dma(host);
					sg_miter_stop(&host->sg_miter);
					host->sg = NULL;
					dw_mci_fifo_reset(host->dev, host);
					state = STATE_IDLE;
				} else {
					if (host->mrq_cmd->data->stop)
						send_stop_cmd(host,
							host->mrq_cmd->data);
					else {
						dw_mci_start_command(host,
							&host->stop,
							host->stop_cmdr);
						host->stop_snd = true;
					}
					state = STATE_SENDING_STOP;
				}
				break;
			}

			if (!host->mrq_cmd->data || cmd->error) {
				done = 1;
				goto exit_cmd;
			}

			if (cmd->data && cmd->error &&
					cmd == host->mrq_cmd->data->stop) {
				done = 1;
				goto exit_cmd;
			}

			prev_state = state = STATE_SENDING_DATA;
			if (host->quirks & DW_MMC_QUIRK_SW_DATA_TIMEOUT) {
				if (cmd->data &&
					(cmd->data->flags & MMC_DATA_READ)) {
					host->dto_cnt = 0;
					mod_timer(&host->dto_timer,
						jiffies + msecs_to_jiffies(DRTO_MON_PERIOD));
				}
			}

			break;

		case STATE_SENDING_STOP:
			if (!test_and_clear_bit(EVENT_CMD_COMPLETE,
						&host->pending_events))
				break;

			if (host->mrq_cmd->cmd->error &&
					host->mrq_cmd->data) {
				dw_mci_stop_dma(host);
				sg_miter_stop(&host->sg_miter);
				host->sg = NULL;
				dw_mci_fifo_reset(host->dev, host);
			}

			host->cmd = NULL;
			host->data = NULL;

			if (host->mrq_cmd->stop)
				dw_mci_command_complete(host,
						host->mrq_cmd->stop);
			else
				host->cmd_status = 0;

			done = 1;
			goto exit_cmd;

		default:
			break;
		}
	} while (state != prev_state);

	host->state_cmd = state;
exit_cmd:

	return done;
}

static int dw_mci_tasklet_dat(struct dw_mci *host)
{
	struct mmc_data *data;
	enum dw_mci_state state;
	enum dw_mci_state prev_state;
	u32 status, done = 0;

	state = host->state_dat;
	data = host->data;

	do {
		prev_state = state;

		switch (state) {
		case STATE_IDLE:
			break;

		case STATE_SENDING_DATA:
			if (test_and_clear_bit(EVENT_DATA_ERROR,
					       &host->pending_events)) {
				set_bit(EVENT_XFER_COMPLETE,
						&host->pending_events);

				/* To avoid fifo full condition */
				dw_mci_fifo_reset(host->dev, host);

				if (MMC_CHECK_CMDQ_MODE(host)) {
					list_add_tail(&host->mrq_dat->hlist,
						&host->cur_slot->mrq_list);
					del_timer(&host->timer);
					dw_mci_stop_dma(host);
					sg_miter_stop(&host->sg_miter);
					host->sg = NULL;
					dw_mci_fifo_reset(host->dev, host);
					state = STATE_IDLE;
				} else {
					if (data->stop)
						send_stop_cmd(host, data);
					else {
						dw_mci_start_command(host,
							&host->stop,
							host->stop_cmdr);
						host->stop_snd = true;
					}
					state = STATE_DATA_ERROR;
				}
				break;
			}

			if (!test_and_clear_bit(EVENT_XFER_COMPLETE,
						&host->pending_events))
				break;

			set_bit(EVENT_XFER_COMPLETE, &host->completed_events);
			prev_state = state = STATE_DATA_BUSY;
			/* fall through */

		case STATE_DATA_BUSY:
			if (!test_and_clear_bit(EVENT_DATA_COMPLETE,
						&host->pending_events))
				break;

			set_bit(EVENT_DATA_COMPLETE, &host->completed_events);
			status = host->data_status;

			if (status & DW_MCI_DATA_ERROR_FLAGS) {
				if (status & SDMMC_INT_DTO) {
					dev_err(host->dev,
						"data timeout error\n");
					data->error = -ETIMEDOUT;
					host->mrq_dat->cmd->error = -ETIMEDOUT;
				} else if (status & SDMMC_INT_DCRC) {
					dev_err(host->dev,
						"data CRC error\n");
					data->error = -EILSEQ;
				} else if (status & SDMMC_INT_EBE) {
					if (host->dir_status ==
									DW_MCI_SEND_STATUS) {
						/*
						 * No data CRC status was returned.
						 * The number of bytes transferred will
						 * be exaggerated in PIO mode.
						 */
						data->bytes_xfered = 0;
						data->error = -ETIMEDOUT;
						dev_err(host->dev,
							"Write no CRC\n");
					} else {
						data->error = -EIO;
						dev_err(host->dev,
							"End bit error\n");
					}

				} else if (status & SDMMC_INT_SBE) {
					dev_err(host->dev,
						"Start bit error "
						"(status=%08x)\n",
						status);
					data->error = -EIO;
				} else {
					dev_err(host->dev,
						"data FIFO error "
						"(status=%08x)\n",
						status);
					data->error = -EIO;
				}
				/*
				 * After an error, there may be data lingering
				 * in the FIFO, so reset it - doing so
				 * generates a block interrupt, hence setting
				 * the scatter-gather pointer to NULL.
				 */
				sg_miter_stop(&host->sg_miter);
				host->sg = NULL;
				dw_mci_fifo_reset(host->dev, host);
				dw_mci_ciu_reset(host->dev, host);

			} else {
				data->bytes_xfered = data->blocks * data->blksz;
				data->error = 0;
			}

			if (host->quirks & DW_MMC_QUIRK_SW_DATA_TIMEOUT &&
					(data->flags & MMC_DATA_READ))
				del_timer(&host->dto_timer);

			host->data = NULL;

			if (!data->stop && !host->stop_snd) {
				done = 1;
				goto exit_dat;
			}

			if (host->mrq_dat->sbc && !data->error) {
				if (data->stop)
					data->stop->error = 0;
				done = 1;
				goto exit_dat;
			}

			if (MMC_CHECK_CMDQ_MODE(host) && !data->error) {
				done = 1;
				goto exit_dat;
			}

			if (MMC_CHECK_CMDQ_MODE(host)) {
				list_add_tail(&host->mrq_dat->hlist,
					&host->cur_slot->mrq_list);
				del_timer(&host->timer);
				dw_mci_stop_dma(host);
				sg_miter_stop(&host->sg_miter);
				host->sg = NULL;
				dw_mci_fifo_reset(host->dev, host);
				state = STATE_IDLE;
				break;
			}

			prev_state = state = STATE_SENDING_STOP;
			if (!data->error) {
				if (data->stop) {
					BUG_ON(!data->stop);
					send_stop_cmd(host, data);
				}
				else {
					dw_mci_start_command(host,
					&host->stop,
					host->stop_cmdr);
					host->stop_snd = true;
				}
			}
			if (test_and_clear_bit(EVENT_DATA_ERROR,
						&host->pending_events)) {
				if (MMC_CHECK_CMDQ_MODE(host)) {
					list_add_tail(&host->mrq_dat->hlist,
						&host->cur_slot->mrq_list);
					del_timer(&host->timer);
					dw_mci_stop_dma(host);
					sg_miter_stop(&host->sg_miter);
					host->sg = NULL;
					dw_mci_fifo_reset(host->dev, host);
					state = STATE_IDLE;
					break;
				} else {
					if (data->stop)
						send_stop_cmd(host, data);
					else {
						dw_mci_start_command(host,
							&host->stop,
							host->stop_cmdr);
						host->stop_snd = true;
					}
				}
			}
			/* fall through */

		case STATE_SENDING_STOP:
			if (!test_and_clear_bit(EVENT_CMD_COMPLETE,
						&host->pending_events))
				break;

			if (host->mrq_dat->cmd->error &&
					host->mrq_dat->data) {
				dw_mci_stop_dma(host);
				sg_miter_stop(&host->sg_miter);
				host->sg = NULL;
				dw_mci_fifo_reset(host->dev, host);
			}

			host->cmd = NULL;
			host->data = NULL;

			if (host->mrq_dat->stop)
				dw_mci_command_complete(host,
						host->mrq_dat->stop);
			else
				host->cmd_status = 0;

			done = 1;
			goto exit_dat;

		case STATE_DATA_ERROR:
			if (!test_and_clear_bit(EVENT_XFER_COMPLETE,
						&host->pending_events))
				break;

			dw_mci_stop_dma(host);
			set_bit(EVENT_XFER_COMPLETE, &host->completed_events);
			set_bit(EVENT_DATA_COMPLETE, &host->pending_events);

			state = STATE_DATA_BUSY;
			break;

		default:
			break;
		}
	} while (state != prev_state);

	host->state_dat = state;
exit_dat:

	return done;
}

static void dw_mci_tasklet_func(unsigned long priv)
{
	struct dw_mci *host = (struct dw_mci *)priv;
	int done_cmd, done_dat;
	struct mmc_request *mrq_cmd, *mrq_dat;

	spin_lock(&host->lock);
	host->tasklet_state = 1;

	if (host->cmd_status & SDMMC_INT_HLE) {
		dw_mci_reg_dump(host);
		clear_bit(EVENT_CMD_COMPLETE, &host->pending_events);
		dev_err(host->dev, "hardware locked write error\n");
		goto unlock;
	}

	/* command state */
	done_cmd = dw_mci_tasklet_cmd(host);
	mrq_cmd = host->mrq_cmd;

	if (done_cmd)
		host->state_cmd = STATE_IDLE;

	if (host->state_cmd == STATE_SENDING_DATA ||
	    host->state_cmd == STATE_DATA_BUSY ||
	    host->state_cmd == STATE_DATA_ERROR ||
	    host->state_cmd == STATE_SENDING_STOP) {
		host->state_dat = host->state_cmd;
		host->mrq_dat = host->mrq_cmd;
		host->state_cmd = STATE_IDLE;
		host->mrq_cmd = NULL;
	}

	/* data state */
	done_dat = dw_mci_tasklet_dat(host);
	mrq_dat = host->mrq_dat;

	if (done_dat)
		host->state_dat = STATE_IDLE;

	if (host->state_cmd == STATE_IDLE) {
		if (!list_empty(&host->cur_slot->mrq_list)) {
			host->cur_slot->mrq = list_first_entry(
				&host->cur_slot->mrq_list,
				struct mmc_request, hlist);
			list_del_init(&host->cur_slot->mrq->hlist);
			host->state_cmd = STATE_SENDING_CMD;
			dw_mci_start_request(host, host->cur_slot);
		} else {
			host->cur_slot->mrq = NULL;
			if (!list_empty(&host->queue)) {
				struct dw_mci_slot *slot;
				slot = list_entry(host->queue.next,
					  struct dw_mci_slot, queue_node);
				list_del_init(&slot->queue_node);
				if (!list_empty(&slot->mrq_list)) {
					slot->mrq = list_first_entry(
						&slot->mrq_list,
						struct mmc_request, hlist);
					list_del_init(&slot->mrq->hlist);
					host->state_cmd = STATE_SENDING_CMD;
					dw_mci_start_request(host, slot);
				} else
					slot->mrq = NULL;
			}
		}
	}
	host->tasklet_state = 0;

	if (done_cmd)
		dw_mci_request_end(host, mrq_cmd, &host->state_cmd);

	if (done_dat)
		dw_mci_request_end(host, mrq_dat, &host->state_dat);
unlock:

	host->tasklet_state = 0;
	spin_unlock(&host->lock);

	if (test_and_clear_bit(EVENT_QUEUE_READY, &host->pending_events))
		mmc_handle_queued_request(host->cur_slot->mmc);
}

/* push final bytes to part_buf, only use during push */
static void dw_mci_set_part_bytes(struct dw_mci *host, void *buf, int cnt)
{
	memcpy((void *)&host->part_buf, buf, cnt);
	host->part_buf_count = cnt;
}

/* append bytes to part_buf, only use during push */
static int dw_mci_push_part_bytes(struct dw_mci *host, void *buf, int cnt)
{
	cnt = min(cnt, (1 << host->data_shift) - host->part_buf_count);
	memcpy((void *)&host->part_buf + host->part_buf_count, buf, cnt);
	host->part_buf_count += cnt;
	return cnt;
}

/* pull first bytes from part_buf, only use during pull */
static int dw_mci_pull_part_bytes(struct dw_mci *host, void *buf, int cnt)
{
	cnt = min(cnt, (int)host->part_buf_count);
	if (cnt) {
		memcpy(buf, (void *)&host->part_buf + host->part_buf_start,
		       cnt);
		host->part_buf_count -= cnt;
		host->part_buf_start += cnt;
	}
	return cnt;
}

/* pull final bytes from the part_buf, assuming it's just been filled */
static void dw_mci_pull_final_bytes(struct dw_mci *host, void *buf, int cnt)
{
	memcpy(buf, &host->part_buf, cnt);
	host->part_buf_start = cnt;
	host->part_buf_count = (1 << host->data_shift) - cnt;
}

static void dw_mci_push_data16(struct dw_mci *host, void *buf, int cnt)
{
	struct mmc_data *data = host->data;
	int init_cnt = cnt;

	/* try and push anything in the part_buf */
	if (unlikely(host->part_buf_count)) {
		int len = dw_mci_push_part_bytes(host, buf, cnt);
		buf += len;
		cnt -= len;
		if (host->part_buf_count == 2) {
			mci_writew(host, DATA(host->data_offset),
					host->part_buf16);
			host->part_buf_count = 0;
		}
	}
#ifndef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
	if (unlikely((unsigned long)buf & 0x1)) {
		while (cnt >= 2) {
			u16 aligned_buf[64];
			int len = min(cnt & -2, (int)sizeof(aligned_buf));
			int items = len >> 1;
			int i;
			/* memcpy from input buffer into aligned buffer */
			memcpy(aligned_buf, buf, len);
			buf += len;
			cnt -= len;
			/* push data from aligned buffer into fifo */
			for (i = 0; i < items; ++i)
				mci_writew(host, DATA(host->data_offset),
						aligned_buf[i]);
		}
	} else
#endif
	{
		u16 *pdata = buf;
		for (; cnt >= 2; cnt -= 2)
			mci_writew(host, DATA(host->data_offset), *pdata++);
		buf = pdata;
	}
	/* put anything remaining in the part_buf */
	if (cnt) {
		dw_mci_set_part_bytes(host, buf, cnt);
		 /* Push data if we have reached the expected data length */
		if ((data->bytes_xfered + init_cnt) ==
		    (data->blksz * data->blocks))
			mci_writew(host, DATA(host->data_offset),
				   host->part_buf16);
	}
}

static void dw_mci_pull_data16(struct dw_mci *host, void *buf, int cnt)
{
#ifndef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
	if (unlikely((unsigned long)buf & 0x1)) {
		while (cnt >= 2) {
			/* pull data from fifo into aligned buffer */
			u16 aligned_buf[64];
			int len = min(cnt & -2, (int)sizeof(aligned_buf));
			int items = len >> 1;
			int i;
			for (i = 0; i < items; ++i)
				aligned_buf[i] = mci_readw(host,
						DATA(host->data_offset));
			/* memcpy from aligned buffer into output buffer */
			memcpy(buf, aligned_buf, len);
			buf += len;
			cnt -= len;
		}
	} else
#endif
	{
		u16 *pdata = buf;
		for (; cnt >= 2; cnt -= 2)
			*pdata++ = mci_readw(host, DATA(host->data_offset));
		buf = pdata;
	}
	if (cnt) {
		host->part_buf16 = mci_readw(host, DATA(host->data_offset));
		dw_mci_pull_final_bytes(host, buf, cnt);
	}
}

static void dw_mci_push_data32(struct dw_mci *host, void *buf, int cnt)
{
	struct mmc_data *data = host->data;
	int init_cnt = cnt;

	/* try and push anything in the part_buf */
	if (unlikely(host->part_buf_count)) {
		int len = dw_mci_push_part_bytes(host, buf, cnt);
		buf += len;
		cnt -= len;
		if (host->part_buf_count == 4) {
			mci_writel(host, DATA(host->data_offset),
					host->part_buf32);
			host->part_buf_count = 0;
		}
	}
#ifndef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
	if (unlikely((unsigned long)buf & 0x3)) {
		while (cnt >= 4) {
			u32 aligned_buf[32];
			int len = min(cnt & -4, (int)sizeof(aligned_buf));
			int items = len >> 2;
			int i;
			/* memcpy from input buffer into aligned buffer */
			memcpy(aligned_buf, buf, len);
			buf += len;
			cnt -= len;
			/* push data from aligned buffer into fifo */
			for (i = 0; i < items; ++i)
				mci_writel(host, DATA(host->data_offset),
						aligned_buf[i]);
		}
	} else
#endif
	{
		u32 *pdata = buf;
		for (; cnt >= 4; cnt -= 4)
			mci_writel(host, DATA(host->data_offset), *pdata++);
		buf = pdata;
	}
	/* put anything remaining in the part_buf */
	if (cnt) {
		dw_mci_set_part_bytes(host, buf, cnt);
		 /* Push data if we have reached the expected data length */
		if ((data->bytes_xfered + init_cnt) ==
		    (data->blksz * data->blocks))
			mci_writel(host, DATA(host->data_offset),
				   host->part_buf32);
	}
}

static void dw_mci_pull_data32(struct dw_mci *host, void *buf, int cnt)
{
#ifndef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
	if (unlikely((unsigned long)buf & 0x3)) {
		while (cnt >= 4) {
			/* pull data from fifo into aligned buffer */
			u32 aligned_buf[32];
			int len = min(cnt & -4, (int)sizeof(aligned_buf));
			int items = len >> 2;
			int i;
			for (i = 0; i < items; ++i)
				aligned_buf[i] = mci_readl(host,
						DATA(host->data_offset));
			/* memcpy from aligned buffer into output buffer */
			memcpy(buf, aligned_buf, len);
			buf += len;
			cnt -= len;
		}
	} else
#endif
	{
		u32 *pdata = buf;
		for (; cnt >= 4; cnt -= 4)
			*pdata++ = mci_readl(host, DATA(host->data_offset));
		buf = pdata;
	}
	if (cnt) {
		host->part_buf32 = mci_readl(host, DATA(host->data_offset));
		dw_mci_pull_final_bytes(host, buf, cnt);
	}
}

static void dw_mci_push_data64(struct dw_mci *host, void *buf, int cnt)
{
	struct mmc_data *data = host->data;
	int init_cnt = cnt;

	/* try and push anything in the part_buf */
	if (unlikely(host->part_buf_count)) {
		int len = dw_mci_push_part_bytes(host, buf, cnt);
		buf += len;
		cnt -= len;

		if (host->part_buf_count == 8) {
			mci_writeq(host, DATA(host->data_offset),
					host->part_buf);
			host->part_buf_count = 0;
		}
	}
#ifndef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
	if (unlikely((unsigned long)buf & 0x7)) {
		while (cnt >= 8) {
			u64 aligned_buf[16];
			int len = min(cnt & -8, (int)sizeof(aligned_buf));
			int items = len >> 3;
			int i;
			/* memcpy from input buffer into aligned buffer */
			memcpy(aligned_buf, buf, len);
			buf += len;
			cnt -= len;
			/* push data from aligned buffer into fifo */
			for (i = 0; i < items; ++i)
				mci_writeq(host, DATA(host->data_offset),
						aligned_buf[i]);
		}
	} else
#endif
	{
		u64 *pdata = buf;
		for (; cnt >= 8; cnt -= 8)
			mci_writeq(host, DATA(host->data_offset), *pdata++);
		buf = pdata;
	}
	/* put anything remaining in the part_buf */
	if (cnt) {
		dw_mci_set_part_bytes(host, buf, cnt);
		/* Push data if we have reached the expected data length */
		if ((data->bytes_xfered + init_cnt) ==
		    (data->blksz * data->blocks))
			mci_writeq(host, DATA(host->data_offset),
				   host->part_buf);
	}
}

static void dw_mci_pull_data64(struct dw_mci *host, void *buf, int cnt)
{
#ifndef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
	if (unlikely((unsigned long)buf & 0x7)) {
		while (cnt >= 8) {
			/* pull data from fifo into aligned buffer */
			u64 aligned_buf[16];
			int len = min(cnt & -8, (int)sizeof(aligned_buf));
			int items = len >> 3;
			int i;
			for (i = 0; i < items; ++i)
				aligned_buf[i] = mci_readq(host,
						DATA(host->data_offset));
			/* memcpy from aligned buffer into output buffer */
			memcpy(buf, aligned_buf, len);
			buf += len;
			cnt -= len;
		}
	} else
#endif
	{
		u64 *pdata = buf;
		for (; cnt >= 8; cnt -= 8)
			*pdata++ = mci_readq(host, DATA(host->data_offset));
		buf = pdata;
	}
	if (cnt) {
		host->part_buf = mci_readq(host, DATA(host->data_offset));
		dw_mci_pull_final_bytes(host, buf, cnt);
	}
}

static void dw_mci_pull_data(struct dw_mci *host, void *buf, int cnt)
{
	int len;

	/* get remaining partial bytes */
	len = dw_mci_pull_part_bytes(host, buf, cnt);
	if (unlikely(len == cnt))
		return;
	buf += len;
	cnt -= len;

	/* get the rest of the data */
	host->pull_data(host, buf, cnt);
}

static void dw_mci_read_data_pio(struct dw_mci *host, bool dto)
{
	struct sg_mapping_iter *sg_miter = &host->sg_miter;
	void *buf;
	unsigned int offset;
	struct mmc_data	*data = host->data;
	int shift = host->data_shift;
	u32 status;
	unsigned int len;
	unsigned int remain, fcnt;
	u32 temp;

	do {
		if (!sg_miter_next(sg_miter))
			goto done;

		host->sg = sg_miter->piter.sg;
		buf = sg_miter->addr;
		remain = sg_miter->length;
		offset = 0;

		do {
			fcnt = (SDMMC_GET_FCNT(mci_readl(host, STATUS))
					<< shift) + host->part_buf_count;
			len = min(remain, fcnt);
			if (!len)
				break;
			dw_mci_pull_data(host, (void *)(buf + offset), len);
			data->bytes_xfered += len;
			offset += len;
			remain -= len;
		} while (remain);

		sg_miter->consumed = offset;
		status = mci_readl(host, MINTSTS);
		mci_writel(host, RINTSTS, SDMMC_INT_RXDR);
	/* if the RXDR is ready read again */
	} while ((status & SDMMC_INT_RXDR) ||
		 (dto && SDMMC_GET_FCNT(mci_readl(host, STATUS))));

	if (!remain) {
		if (!sg_miter_next(sg_miter))
			goto done;
		sg_miter->consumed = 0;
	}
	sg_miter_stop(sg_miter);
	return;

done:

	/* Disable RX/TX IRQs */
	mci_writel(host, RINTSTS, SDMMC_INT_TXDR | SDMMC_INT_RXDR);
	temp = mci_readl(host, INTMASK);
	temp  &= ~(SDMMC_INT_RXDR | SDMMC_INT_TXDR);
	mci_writel(host, INTMASK, temp);

	sg_miter_stop(sg_miter);
	host->sg = NULL;
	smp_wmb();
	set_bit(EVENT_XFER_COMPLETE, &host->pending_events);
}

static void dw_mci_write_data_pio(struct dw_mci *host)
{
	struct sg_mapping_iter *sg_miter = &host->sg_miter;
	void *buf;
	unsigned int offset;
	struct mmc_data	*data = host->data;
	int shift = host->data_shift;
	u32 status;
	unsigned int len;
	unsigned int fifo_depth = host->fifo_depth;
	unsigned int remain, fcnt;
	u32 temp;

	do {
		if (!sg_miter_next(sg_miter))
			goto done;

		host->sg = sg_miter->piter.sg;
		buf = sg_miter->addr;
		remain = sg_miter->length;
		offset = 0;

		do {
			fcnt = ((fifo_depth -
				 SDMMC_GET_FCNT(mci_readl(host, STATUS)))
					<< shift) - host->part_buf_count;
			len = min(remain, fcnt);
			if (!len)
				break;
			host->push_data(host, (void *)(buf + offset), len);
			data->bytes_xfered += len;
			offset += len;
			remain -= len;
		} while (remain);

		sg_miter->consumed = offset;
		status = mci_readl(host, MINTSTS);
		mci_writel(host, RINTSTS, SDMMC_INT_TXDR);
	} while (status & SDMMC_INT_TXDR); /* if TXDR write again */

	if (!remain) {
		if (!sg_miter_next(sg_miter))
			goto done;
		sg_miter->consumed = 0;
	}
	sg_miter_stop(sg_miter);
	return;

done:

	/* Disable RX/TX IRQs */
	mci_writel(host, RINTSTS, SDMMC_INT_TXDR | SDMMC_INT_RXDR);
	temp = mci_readl(host, INTMASK);
	temp  &= ~(SDMMC_INT_RXDR | SDMMC_INT_TXDR);
	mci_writel(host, INTMASK, temp);

	sg_miter_stop(sg_miter);
	host->sg = NULL;
	smp_wmb();
	set_bit(EVENT_XFER_COMPLETE, &host->pending_events);
}

static void dw_mci_cmd_interrupt(struct dw_mci *host, u32 status)
{
	if (!host->cmd_status)
		host->cmd_status = status;

	smp_wmb();

	set_bit(EVENT_CMD_COMPLETE, &host->pending_events);
	tasklet_schedule(&host->tasklet);
}

static irqreturn_t dw_mci_interrupt(int irq, void *dev_id)
{
	struct dw_mci *host = dev_id;
	u32 status, pending;
	int i;
	int ret = IRQ_NONE;

	status = mci_readl(host, RINTSTS);
	pending = mci_readl(host, MINTSTS); /* read-only mask reg */

	if (pending) {

		/*
		 * DTO fix - version 2.10a and below, and only if internal DMA
		 * is configured.
		 */
		if (host->quirks & DW_MCI_QUIRK_IDMAC_DTO) {
			if (!pending &&
			    ((mci_readl(host, STATUS) >> 17) & 0x1fff))
				pending |= SDMMC_INT_DATA_OVER;
		}

		if (host->quirks & DW_MCI_QUIRK_NO_DETECT_EBIT &&
				host->dir_status == DW_MCI_RECV_STATUS) {
			if (status & SDMMC_INT_EBE)
				mci_writel(host, RINTSTS, SDMMC_INT_EBE);
		}

		if (pending & SDMMC_INT_HLE) {
			mci_writel(host, RINTSTS, SDMMC_INT_HLE);
			host->cmd_status = pending;
			tasklet_schedule(&host->tasklet);
			ret = IRQ_HANDLED;
		}

		if (pending & DW_MCI_CMD_ERROR_FLAGS) {
			mci_writel(host, RINTSTS, DW_MCI_CMD_ERROR_FLAGS);
			host->cmd_status = pending;
			ret = IRQ_HANDLED;
		}

		if (pending & SDMMC_INT_VOLT_SW) {
			u32 cmd = mci_readl(host, CMD);
			u32 cmd_up_clk = cmd;
			cmd = cmd & 0x3f;
			if ((cmd == SD_SWITCH_VOLTAGE) ||
				(cmd_up_clk & SDMMC_CMD_UPD_CLK)) {
				mci_writel(host, RINTSTS, SDMMC_INT_VOLT_SW);
				pending &= ~(SDMMC_INT_VOLT_SW);
				dw_mci_cmd_interrupt(host, pending);
				ret = IRQ_HANDLED;
			}
		}

		if (pending & DW_MCI_DATA_ERROR_FLAGS) {
			if (mci_readl(host, RINTSTS) & SDMMC_INT_HTO)
				dw_mci_reg_dump(host);

			/* if there is an error report DATA_ERROR */
			mci_writel(host, RINTSTS, DW_MCI_DATA_ERROR_FLAGS);
			host->data_status = pending;
			smp_wmb();
			set_bit(EVENT_DATA_ERROR, &host->pending_events);
			if (pending & SDMMC_INT_SBE)
				set_bit(EVENT_DATA_COMPLETE,
					&host->pending_events);
			tasklet_schedule(&host->tasklet);
			ret = IRQ_HANDLED;
		}

		if (pending & SDMMC_INT_DATA_OVER) {
			mci_writel(host, RINTSTS, SDMMC_INT_DATA_OVER);
			if (!host->data_status)
				host->data_status = pending;
			smp_wmb();
			if (host->dir_status == DW_MCI_RECV_STATUS) {
				if (host->sg != NULL)
					dw_mci_read_data_pio(host, true);
			}
			set_bit(EVENT_DATA_COMPLETE, &host->pending_events);
			tasklet_schedule(&host->tasklet);
			ret = IRQ_HANDLED;
		}

		if (pending & SDMMC_INT_RXDR) {
			mci_writel(host, RINTSTS, SDMMC_INT_RXDR);
			if (host->dir_status == DW_MCI_RECV_STATUS && host->sg)
				dw_mci_read_data_pio(host, false);
			ret = IRQ_HANDLED;
		}

		if (pending & SDMMC_INT_TXDR) {
			mci_writel(host, RINTSTS, SDMMC_INT_TXDR);
			if (host->dir_status == DW_MCI_SEND_STATUS && host->sg)
				dw_mci_write_data_pio(host);
			ret = IRQ_HANDLED;
		}

		if (pending & SDMMC_INT_CMD_DONE) {
			mci_writel(host, RINTSTS, SDMMC_INT_CMD_DONE);
			dw_mci_cmd_interrupt(host, pending);
			ret = IRQ_HANDLED;
		}

		if (pending & SDMMC_INT_CD) {
			mci_writel(host, RINTSTS, SDMMC_INT_CD);
			queue_work(host->card_workqueue, &host->card_work);
			ret = IRQ_HANDLED;
		}

		/* Handle SDIO Interrupts */
		for (i = 0; i < host->num_slots; i++) {
			struct dw_mci_slot *slot = host->slot[i];
			if (pending & SDMMC_INT_SDIO(i)) {
				mci_writel(host, RINTSTS, SDMMC_INT_SDIO(i));
				mmc_signal_sdio_irq(slot->mmc);
				ret = IRQ_HANDLED;
			}
		}

	}

#ifdef CONFIG_MMC_DW_IDMAC
	/* Handle DMA interrupts */
	pending = mci_readl(host, IDSTS);
	if (pending & (SDMMC_IDMAC_INT_TI | SDMMC_IDMAC_INT_RI)) {
		mci_writel(host, IDSTS, SDMMC_IDMAC_INT_TI | SDMMC_IDMAC_INT_RI);
		mci_writel(host, IDSTS, SDMMC_IDMAC_INT_NI);
		host->dma_ops->complete(host);
		ret = IRQ_HANDLED;
	}
#endif

	/* handle queue ready interrupt */
	pending = mci_readl(host, SHA_CMD_IS);
	if (pending & QRDY_INT) {
		u32 qrdy = mci_readl(host, SHA_CMD_IE);
		if (qrdy & QRDY_INT_EN) {
			set_bit(EVENT_QUEUE_READY, &host->pending_events);
			qrdy &= ~QRDY_INT_EN;
			mci_writel(host, SHA_CMD_IE, qrdy);
		}

		/* clear queue ready interrupt */
		mci_writel(host, SHA_CMD_IS, QRDY_INT);

		if (test_bit(EVENT_QUEUE_READY, &host->pending_events))
			tasklet_schedule(&host->tasklet);

		ret = IRQ_HANDLED;
	}

	if (ret == IRQ_NONE)
		pr_warn_ratelimited("%s: no interrupts handled, pending %08x %08x\n",
				dev_name(host->dev),
				mci_readl(host, MINTSTS),
				mci_readl(host, IDSTS));

	return ret;
}

static void dw_mci_timeout_timer(unsigned long data)
{
	struct dw_mci *host = (struct dw_mci *)data;
	struct mmc_request *mrq;

	if (host && host->mrq_dat) {
		mrq = host->mrq_dat;

		dev_err(host->dev,
			"Timeout waiting for hardware interrupt."
			" state = %d\n", host->state_dat);
		dw_mci_reg_dump(host);

		spin_lock(&host->lock);

		host->sg = NULL;
		host->data = NULL;
		host->cmd = NULL;

		switch (host->state_dat) {
		case STATE_IDLE:
			break;
		case STATE_SENDING_CMD:
			mrq->cmd->error = -ENOMEDIUM;
			if (!mrq->data)
				break;
			/* fall through */
		case STATE_SENDING_DATA:
			mrq->data->error = -ENOMEDIUM;
			dw_mci_stop_dma(host);
			break;
		case STATE_DATA_BUSY:
		case STATE_DATA_ERROR:
			if (mrq->data->error == -EINPROGRESS)
				mrq->data->error = -ENOMEDIUM;
			/* fall through */
		case STATE_SENDING_STOP:
			if (mrq->stop)
				mrq->stop->error = -ENOMEDIUM;
			break;
		}

		spin_unlock(&host->lock);
		dw_mci_fifo_reset(host->dev, host);
		dw_mci_ciu_reset(host->dev, host);
		spin_lock(&host->lock);

		dw_mci_request_end(host, mrq, &host->state_dat);
		spin_unlock(&host->lock);
	}
}

static void dw_mci_tp_mon(struct work_struct *work)
{
	struct dw_mci *host = container_of(work, struct dw_mci, tp_mon.work);
	struct dw_mci_mon_table *tp_tbl = host->pdata->tp_mon_tbl;
	s32 mif_lock_value = 0;
	s32 cpu_lock_value = 0;
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	s32 kfc_lock_value = 0;
#endif

	while (tp_tbl->range) {
		if (host->transferred_cnt > tp_tbl->range)
			break;
		tp_tbl++;
	}

	mif_lock_value = tp_tbl->mif_lock_value;
	cpu_lock_value = tp_tbl->cpu_lock_value;
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	kfc_lock_value = tp_tbl->kfc_lock_value;
#endif

#ifndef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	dev_dbg(host->dev, "%d byte/s cnt=%d mif=%d cpu=%d\n",
						host->transferred_cnt,
						host->cmd_cnt,
						mif_lock_value,
						cpu_lock_value);
#else
	dev_dbg(host->dev, "%d byte/s cnt=%d mif=%d cpu=%d kfc=%d\n",
						host->transferred_cnt,
						host->cmd_cnt,
						mif_lock_value,
						cpu_lock_value,
						kfc_lock_value);
#endif

	pm_qos_update_request_timeout(&host->pm_qos_int,
					host->pdata->qos_int_level,
					200000);
	pm_qos_update_request_timeout(&host->pm_qos_mif,
					mif_lock_value, 2000000);
	pm_qos_update_request_timeout(&host->pm_qos_cpu,
					cpu_lock_value, 2000000);
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	pm_qos_update_request_timeout(&host->pm_qos_kfc,
					kfc_lock_value, 2000000);
#endif

	host->transferred_cnt = 0;
	host->cmd_cnt = 0;
	schedule_delayed_work(&host->tp_mon, HZ);
}

static void dw_mci_work_routine_card(struct work_struct *work)
{
	struct dw_mci *host = container_of(work, struct dw_mci, card_work);
	int i;

	for (i = 0; i < host->num_slots; i++) {
		struct dw_mci_slot *slot = host->slot[i];
		struct mmc_host *mmc = slot->mmc;
		struct mmc_request *mrq;
		int present;

		present = dw_mci_get_cd(mmc);
		while (present != slot->last_detect_state) {
			dev_dbg(&slot->mmc->class_dev, "card %s\n",
				present ? "inserted" : "removed");

			/* Power up slot (before spin_lock, may sleep) */
			if (present != 0 && host->pdata->setpower)
				host->pdata->setpower(slot->id, mmc->ocr_avail);

			spin_lock_bh(&host->lock);

			/* Card change detected */
			slot->last_detect_state = present;

			/* Mark card as present if applicable */
			if (present != 0)
				set_bit(DW_MMC_CARD_PRESENT, &slot->flags);

			/* Clean up queue if present */
			mrq = slot->mrq;
			if (mrq) {
				enum dw_mci_state *state = NULL;
				if (mrq == host->mrq_cmd)
					state = &host->state_cmd;
				else if (mrq == host->mrq_dat)
					state = &host->state_dat;
				if (state) {
					host->data = NULL;
					host->cmd = NULL;

					switch (*state) {
					case STATE_IDLE:
						break;
					case STATE_SENDING_CMD:
						mrq->cmd->error = -ENOMEDIUM;
						if (!mrq->data)
							break;
						/* fall through */
					case STATE_SENDING_DATA:
						mrq->data->error = -ENOMEDIUM;
						dw_mci_stop_dma(host);
						break;
					case STATE_DATA_BUSY:
					case STATE_DATA_ERROR:
						if (mrq->data->error == -EINPROGRESS)
							mrq->data->error = -ENOMEDIUM;
						/* fall through */
					case STATE_SENDING_STOP:
						if (mrq->stop)
							mrq->stop->error = -ENOMEDIUM;
						break;
					}

					dw_mci_request_end(host, mrq, state);
					slot->mrq = NULL;
				} else {
					list_del(&slot->queue_node);
					mrq->cmd->error = -ENOMEDIUM;
					if (mrq->data)
						mrq->data->error = -ENOMEDIUM;
					if (mrq->stop)
						mrq->stop->error = -ENOMEDIUM;

					del_timer(&host->timer);
					spin_unlock(&host->lock);
					mmc_request_done(slot->mmc, mrq);
					spin_lock(&host->lock);
				}
			}

			/* Power down slot */
			atomic_inc_return(&host->ciu_en_win);
			dw_mci_ciu_clk_en(host, false);
			if (present == 0) {
				clear_bit(DW_MMC_CARD_PRESENT, &slot->flags);

				/*
				 * Clear down the FIFO - doing so generates a
				 * block interrupt, hence setting the
				 * scatter-gather pointer to NULL.
				 */
				sg_miter_stop(&host->sg_miter);
				host->sg = NULL;
				dw_mci_fifo_reset(host->dev, host);
#ifdef CONFIG_MMC_DW_IDMAC
				dw_mci_idma_reset_dma(host);
#endif
			} else if (host->cur_slot) {
				dw_mci_ciu_reset(host->dev, host);
				mci_writel(host, RINTSTS, 0xFFFFFFFF);
			}
			atomic_dec_return(&host->ciu_en_win);

			spin_unlock_bh(&host->lock);

			/* Power down slot (after spin_unlock, may sleep) */
			if (present == 0 && host->pdata->setpower)
				host->pdata->setpower(slot->id, 0);

			present = dw_mci_get_cd(mmc);
		}

		mmc_detect_change(slot->mmc,
			msecs_to_jiffies(host->pdata->detect_delay_ms));
	}
}

static void dw_mci_notify_change(struct platform_device *dev, int state)
{
	struct dw_mci *host = platform_get_drvdata(dev);
	unsigned long flags;

	if (host) {
		spin_lock_irqsave(&host->lock, flags);
		if (state) {
			dev_dbg(&dev->dev, "card inserted.\n");
			host->pdata->quirks |= DW_MCI_QUIRK_BROKEN_CARD_DETECTION;
		} else {
			dev_dbg(&dev->dev, "card removed.\n");
			host->pdata->quirks &= ~DW_MCI_QUIRK_BROKEN_CARD_DETECTION;
		}
		queue_work(host->card_workqueue, &host->card_work);
		spin_unlock_irqrestore(&host->lock, flags);
	}
}

#ifdef CONFIG_OF
/* given a slot id, find out the device node representing that slot */
static struct device_node *dw_mci_of_find_slot_node(struct device *dev, u8 slot)
{
	struct device_node *np;
	const __be32 *addr;
	int len;

	if (!dev || !dev->of_node)
		return NULL;

	for_each_child_of_node(dev->of_node, np) {
		addr = of_get_property(np, "reg", &len);
		if (!addr || (len < sizeof(int)))
			continue;
		if (be32_to_cpup(addr) == slot)
			return np;
	}
	return NULL;
}

static struct dw_mci_of_slot_quirks {
	char *quirk;
	int id;
} of_slot_quirks[] = {
	{
		.quirk	= "disable-wp",
		.id	= DW_MCI_SLOT_QUIRK_NO_WRITE_PROTECT,
	},
};

static int dw_mci_of_get_slot_quirks(struct device *dev, u8 slot)
{
	struct device_node *np = dw_mci_of_find_slot_node(dev, slot);
	int quirks = 0;
	int idx;

	/* get quirks */
	for (idx = 0; idx < ARRAY_SIZE(of_slot_quirks); idx++)
		if (of_get_property(np, of_slot_quirks[idx].quirk, NULL))
			quirks |= of_slot_quirks[idx].id;

	return quirks;
}

/* find out bus-width for a given slot */
static u32 dw_mci_of_get_bus_wd(struct device *dev, u8 slot)
{
	struct device_node *np = dw_mci_of_find_slot_node(dev, slot);
	u32 bus_wd = 1;

	if (!np)
		return 1;

	if (of_property_read_u32(np, "bus-width", &bus_wd))
		dev_err(dev, "bus-width property not found, assuming width"
			       " as 1\n");
	return bus_wd;
}

/* find the write protect gpio for a given slot; or -1 if none specified */
static int dw_mci_of_get_wp_gpio(struct device *dev, u8 slot)
{
	struct device_node *np = dw_mci_of_find_slot_node(dev, slot);
	int gpio;

	if (!np)
		return -EINVAL;

	if (of_get_property(np, "wp-gpios", NULL))
		gpio = of_get_named_gpio(np, "wp-gpios", 0);
	else
		gpio = -1;

	/* Having a missing entry is valid; return silently */
	if (!gpio_is_valid(gpio))
		return -EINVAL;

	if (devm_gpio_request(dev, gpio, "dw-mci-wp")) {
		dev_warn(dev, "gpio [%d] request failed\n", gpio);
		return -EINVAL;
	}

	return gpio;
}
#else /* CONFIG_OF */
static int dw_mci_of_get_slot_quirks(struct device *dev, u8 slot)
{
	return 0;
}
static u32 dw_mci_of_get_bus_wd(struct device *dev, u8 slot)
{
	return 1;
}
static struct device_node *dw_mci_of_find_slot_node(struct device *dev, u8 slot)
{
	return NULL;
}
static int dw_mci_of_get_wp_gpio(struct device *dev, u8 slot)
{
	return -EINVAL;
}
#endif /* CONFIG_OF */

static irqreturn_t dw_mci_detect_interrupt(int irq, void *dev_id)
{
	struct dw_mci *host = dev_id;

	queue_work(host->card_workqueue, &host->card_work);

	return IRQ_HANDLED;
}

static int dw_mci_init_slot(struct dw_mci *host, unsigned int id)
{
	struct mmc_host *mmc;
	struct dw_mci_slot *slot;
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	int ctrl_id, ret;
	u8 bus_width;

	mmc = mmc_alloc_host(sizeof(struct dw_mci_slot), host->dev);
	if (!mmc)
		return -ENOMEM;

	slot = mmc_priv(mmc);
	slot->id = id;
#ifdef CONFIG_MMC_CLKGATE
	mmc->clkgate_delay = 10;
#endif
	slot->mmc = mmc;
	slot->host = host;
	host->slot[id] = slot;

	slot->quirks = dw_mci_of_get_slot_quirks(host->dev, slot->id);

	mmc->ops = &dw_mci_ops;
	mmc->f_min = DIV_ROUND_UP(host->bus_hz, 510);
	mmc->f_max = host->bus_hz;

	if (host->pdata->get_ocr)
		mmc->ocr_avail = host->pdata->get_ocr(id);
	else
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	/*
	 * Start with slot power disabled, it will be enabled when a card
	 * is detected.
	 */
	if (host->pdata->setpower)
		host->pdata->setpower(id, 0);

	if (host->pdata->caps)
		mmc->caps = host->pdata->caps;

	if (host->pdata->pm_caps)
		mmc->pm_caps = host->pdata->pm_caps;

	if (host->dev->of_node) {
		ctrl_id = of_alias_get_id(host->dev->of_node, "mshc");
		if (ctrl_id < 0)
			ctrl_id = 0;
	} else {
		ctrl_id = to_platform_device(host->dev)->id;
	}
	if (drv_data && drv_data->caps)
		mmc->caps |= drv_data->caps[ctrl_id];

	if (host->pdata->caps2)
		mmc->caps2 = host->pdata->caps2;

	if (host->pdata->pm_caps) {
		mmc->pm_caps |= host->pdata->pm_caps;
		mmc->pm_flags = mmc->pm_caps;
	}

	if (host->pdata->get_bus_wd)
		bus_width = host->pdata->get_bus_wd(slot->id);
	else if (host->dev->of_node)
		bus_width = dw_mci_of_get_bus_wd(host->dev, slot->id);
	else
		bus_width = 1;

	switch (bus_width) {
	case 8:
		mmc->caps |= MMC_CAP_8_BIT_DATA;
	case 4:
		mmc->caps |= MMC_CAP_4_BIT_DATA;
	}

	if (host->pdata->quirks & DW_MCI_QUIRK_BYPASS_SMU)
		drv_data->cfg_smu(host);

	if (host->pdata->quirks & DW_MCI_QUIRK_HIGHSPEED)
		mmc->caps |= MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED;

	if (host->pdata->blk_settings) {
		mmc->max_segs = host->pdata->blk_settings->max_segs;
		mmc->max_blk_size = host->pdata->blk_settings->max_blk_size;
		mmc->max_blk_count = host->pdata->blk_settings->max_blk_count;
		mmc->max_req_size = host->pdata->blk_settings->max_req_size;
		mmc->max_seg_size = host->pdata->blk_settings->max_seg_size;
	} else {
		/* Useful defaults if platform data is unset. */
#ifdef CONFIG_MMC_DW_IDMAC
		mmc->max_segs = host->ring_size;
		mmc->max_blk_size = 65536;
		mmc->max_seg_size = 0x1000;
		mmc->max_req_size = mmc->max_seg_size * host->ring_size;
		mmc->max_blk_count = mmc->max_req_size / 512;
#else
		mmc->max_segs = 64;
		mmc->max_blk_size = 65536; /* BLKSIZ is 16 bits */
		mmc->max_blk_count = 512;
		mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
		mmc->max_seg_size = mmc->max_req_size;
#endif /* CONFIG_MMC_DW_IDMAC */
	}

	if (host->align_size)
		mmc->align_size = host->align_size;

	if (!(host->quirks & DW_MMC_QUIRK_FIXED_VOLTAGE)) {
		host->vmmc = devm_regulator_get(mmc_dev(mmc), "vmmc");
		if (IS_ERR(host->vmmc)) {
			pr_info("%s: no vmmc regulator found\n", mmc_hostname(mmc));
			host->vmmc = NULL;
		} else {
			pr_info("%s: vmmc regulator found\n", mmc_hostname(mmc));
			ret = regulator_enable(host->vmmc);
			if (ret) {
				dev_err(host->dev,
					"failed to enable vmmc regulator: %d\n", ret);
				goto err_setup_bus;
			}
		}

		host->vqmmc = devm_regulator_get(mmc_dev(mmc), "vqmmc");
		if (IS_ERR(host->vqmmc)) {
			pr_info("%s: no vqmmc regulator found\n", mmc_hostname(mmc));
			host->vqmmc = NULL;
		} else {
			pr_info("%s: vqmmc regulator found\n", mmc_hostname(mmc));
			ret = regulator_enable(host->vqmmc);
			if (ret) {
				dev_err(host->dev,
					"failed to enable vqmmc regulator: %d\n", ret);
				goto err_setup_bus;
			}
		}
	}

	if (host->pdata->init)
		host->pdata->init(id, dw_mci_detect_interrupt, host);

	if (dw_mci_get_cd(mmc))
		set_bit(DW_MMC_CARD_PRESENT, &slot->flags);
	else
		clear_bit(DW_MMC_CARD_PRESENT, &slot->flags);

	slot->wp_gpio = dw_mci_of_get_wp_gpio(host->dev, slot->id);

	ret = mmc_add_host(mmc);
	if (ret)
		goto err_setup_bus;

	INIT_LIST_HEAD(&slot->mrq_list);

#if defined(CONFIG_DEBUG_FS)
	dw_mci_init_debugfs(slot);
#endif

	/* Card initially undetected */
	slot->last_detect_state = 0;

	if (host->pdata->cd_type == DW_MCI_CD_EXTERNAL)
		host->pdata->ext_cd_init(&dw_mci_notify_change);

	/*
	 * Card may have been plugged in prior to boot so we
	 * need to run the detect tasklet
	 */
	queue_work(host->card_workqueue, &host->card_work);

	return 0;

err_setup_bus:
	mmc_free_host(mmc);
	return -EINVAL;
}

static void dw_mci_cleanup_slot(struct dw_mci_slot *slot, unsigned int id)
{
	/* Shutdown detect IRQ */
	if (slot->host->pdata->exit)
		slot->host->pdata->exit(id);

	/* Debugfs stuff is cleaned up by mmc core */
	mmc_remove_host(slot->mmc);
	slot->host->slot[id] = NULL;
	mmc_free_host(slot->mmc);
}

static void dw_mci_init_dma(struct dw_mci *host)
{
	if (host->pdata->desc_sz)
		host->desc_sz = host->pdata->desc_sz;
	else
		host->desc_sz = 1;

	/* Alloc memory for sg translation */
	host->sg_cpu = dmam_alloc_coherent(host->dev,
			host->desc_sz * PAGE_SIZE * MMC_DW_IDMAC_MULTIPLIER,
			&host->sg_dma, GFP_KERNEL);
	if (!host->sg_cpu) {
		dev_err(host->dev, "%s: could not alloc DMA memory\n",
			__func__);
		goto no_dma;
	}

	/* Determine which DMA interface to use */
#ifdef CONFIG_MMC_DW_IDMAC
	host->dma_ops = &dw_mci_idmac_ops;
	dev_info(host->dev, "Using internal DMA controller.\n");
#endif

	if (!host->dma_ops)
		goto no_dma;

	if (host->dma_ops->init && host->dma_ops->start &&
	    host->dma_ops->stop && host->dma_ops->cleanup) {
		if (host->dma_ops->init(host)) {
			dev_err(host->dev, "%s: Unable to initialize "
				"DMA Controller.\n", __func__);
			goto no_dma;
		}
	} else {
		dev_err(host->dev, "DMA initialization not found.\n");
		goto no_dma;
	}

	host->use_dma = 1;
	return;

no_dma:
	dev_info(host->dev, "Using PIO mode.\n");
	host->use_dma = 0;
	return;
}

static bool mci_wait_reset(struct device *dev, struct dw_mci *host)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned int ctrl;

	mci_writel(host, CTRL, (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET |
				SDMMC_CTRL_DMA_RESET));

	/* wait till resets clear */
	do {
		ctrl = mci_readl(host, CTRL);
		if (!(ctrl & (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET |
			      SDMMC_CTRL_DMA_RESET)))
			return true;
	} while (time_before(jiffies, timeout));

	dev_err(dev, "Timeout resetting block (ctrl %#x)\n", ctrl);

	return false;
}

#define REGISTER_NOTI 0
#define UNREGISTER_NOTI 1

static void dw_mci_register_notifier(struct dw_mci *host, u32 reg_noti)
{
	const struct dw_mci_drv_data *drv_data = host->drv_data;

	if (reg_noti == REGISTER_NOTI) {
		if (drv_data && drv_data->register_notifier)
			drv_data->register_notifier(host);
	} else if (reg_noti == UNREGISTER_NOTI) {
		if (drv_data && drv_data->unregister_notifier)
			drv_data->unregister_notifier(host);
	}
}

#ifdef CONFIG_OF
static struct dw_mci_of_quirks {
	char *quirk;
	int id;
} of_quirks[] = {
	{
		.quirk	= "supports-highspeed",
		.id	= DW_MCI_QUIRK_HIGHSPEED,
	}, {
		.quirk	= "broken-cd",
		.id	= DW_MCI_QUIRK_BROKEN_CARD_DETECTION,
	}, {
		.quirk	= "bypass-smu",
		.id	= DW_MCI_QUIRK_BYPASS_SMU,
	}, {
		.quirk	= "fixed_volt",
		.id	= DW_MMC_QUIRK_FIXED_VOLTAGE,
	}, {
		.quirk	= "sw_data_timeout",
		.id	= DW_MMC_QUIRK_SW_DATA_TIMEOUT,
	},
};

//-------------------------------------------------------------
//
// eMMC H/W Reset GPIO Control
//
//-------------------------------------------------------------
#if defined(CONFIG_MACH_ODROIDXU3)
    int  eMMC_HW_RESET_GPIO = 0;
    
    // Use arch/arm/mach-exynos/common.c
    // eMMC H/W Reset Control
    EXPORT_SYMBOL(eMMC_HW_RESET_GPIO);
#endif    

//-------------------------------------------------------------

static struct dw_mci_board *dw_mci_parse_dt(struct dw_mci *host)
{
	struct dw_mci_board *pdata;
	struct device *dev = host->dev;
	struct device_node *np = dev->of_node;
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	int idx, ret;
	u32 clock_frequency;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for pdata\n");
		return ERR_PTR(-ENOMEM);
	}

	/* find out number of slots supported */
	if (of_property_read_u32(dev->of_node, "num-slots",
				&pdata->num_slots)) {
		dev_info(dev, "num-slots property not found, "
				"assuming 1 slot is available\n");
		pdata->num_slots = 1;
	}

	/* get quirks */
	for (idx = 0; idx < ARRAY_SIZE(of_quirks); idx++)
		if (of_get_property(np, of_quirks[idx].quirk, NULL))
			pdata->quirks |= of_quirks[idx].id;

	if (of_property_read_u32(np, "fifo-depth", &pdata->fifo_depth))
		dev_info(dev, "fifo-depth property not found, using "
				"value of FIFOTH register as default\n");

	of_property_read_u32(np, "card-detect-delay", &pdata->detect_delay_ms);
	of_property_read_u32(np, "qos_int_level", &pdata->qos_int_level);

	if (!of_property_read_u32(np, "clock-frequency", &clock_frequency))
		pdata->bus_hz = clock_frequency;

	if (drv_data && drv_data->parse_dt) {
		ret = drv_data->parse_dt(host);
		if (ret)
			return ERR_PTR(ret);
	}

	/* caps */
	if (of_find_property(np, "caps-control", NULL)) {
		if (of_find_property(np, "support-ddr50", NULL))
			pdata->caps |= MMC_CAP_UHS_DDR50;

		if (of_find_property(np, "support-1-8v-ddr", NULL))
			pdata->caps |= MMC_CAP_1_8V_DDR;

		if (of_find_property(np, "support-8-bit", NULL))
			pdata->caps |= MMC_CAP_8_BIT_DATA;

		if (of_find_property(np, "support-cmd23", NULL))
			pdata->caps |= MMC_CAP_CMD23;

		if (of_find_property(np, "support-sdr104", NULL))
			pdata->caps |= MMC_CAP_UHS_SDR104;
	} else if (drv_data && drv_data->misc_control)
		pdata->caps = drv_data->misc_control(host,
				CTRL_SET_DEF_CAPS, NULL);

	/* caps2 */
	if (of_find_property(np, "extra_tuning", NULL))
		pdata->extra_tuning = true;

	if (of_find_property(np, "only_once_tune", NULL))
		pdata->only_once_tune = true;

	if (of_find_property(np, "keep-power-in-suspend", NULL))
		pdata->pm_caps |= MMC_PM_KEEP_POWER;

	if (of_find_property(np, "enable-sdio-wakeup", NULL))
		pdata->pm_caps |= MMC_PM_WAKE_SDIO_IRQ;

	if (of_find_property(np, "enable-cache-control", NULL))
		pdata->caps2 |= MMC_CAP2_CACHE_CTRL;

	if (of_find_property(np, "supports-poweroff-notification", NULL))
		pdata->caps2 |= MMC_CAP2_POWEROFF_NOTIFY;

	if (of_find_property(np, "enable-no-sleep-cmd", NULL))
		pdata->caps2 |= MMC_CAP2_NO_SLEEP_CMD;

	if (of_find_property(np, "supports-hs200-1-8v-mode", NULL))
		pdata->caps2 |= MMC_CAP2_HS200_1_8V_SDR;

	if (of_find_property(np, "supports-hs200-1-2v-mode", NULL))
		pdata->caps2 |= MMC_CAP2_HS200_1_2V_SDR;

	if (of_find_property(np, "supports-hs200-mode", NULL))
		pdata->caps2 |= MMC_CAP2_HS200;

	if (of_find_property(np, "supports-ddr200-1-8v-mode", NULL))
		pdata->caps2 |= MMC_CAP2_HS200_1_8V_DDR;

	if (of_find_property(np, "supports-ddr200-1-2v-mode", NULL))
		pdata->caps2 |= MMC_CAP2_HS200_1_2V_DDR;

	if (of_find_property(np, "supports-ddr200-mode", NULL))
		pdata->caps2 |= MMC_CAP2_HS200_DDR;

	if (of_find_property(np, "use-broken-voltage", NULL))
		pdata->caps2 |= MMC_CAP2_BROKEN_VOLTAGE;

	if (of_find_property(np, "enable-packed-rd", NULL))
		pdata->caps2 |= MMC_CAP2_PACKED_RD;

	if (of_find_property(np, "enable-packed-wr", NULL))
		pdata->caps2 |= MMC_CAP2_PACKED_WR;

	if (of_find_property(np, "enable-packed-CMD", NULL))
		pdata->caps2 |= MMC_CAP2_PACKED_CMD;

	if (of_find_property(np, "enable-cmdq", NULL))
		pdata->caps2 |= MMC_CAP2_CMDQ;

	if (of_find_property(np, "device-driver", NULL))
		pdata->caps2 |= MMC_CAP2_DEVICE_DRIVER;

	if (of_find_property(np, "clock-gate", NULL))
		pdata->use_gate_clock = true;

	if (of_property_read_u32(dev->of_node, "cd-type",
				&pdata->cd_type))
		pdata->cd_type = DW_MCI_CD_PERMANENT;


	if (of_find_property(np, "supports-sdr104-mode", NULL))
		pdata->caps |= MMC_CAP_UHS_SDR104;

#if defined(CONFIG_MACH_ODROIDXU3)
    //-------------------------------------------------------------
    //
    // eMMC Hardware Reset Control GPIO (Global)
    //
    //-------------------------------------------------------------
    {
        int np_gpio = of_get_named_gpio(np, "dw-mshc-hw-reset", 0);

    	if (gpio_is_valid(np_gpio)) {
    		if(devm_gpio_request_one(host->dev, np_gpio, GPIOF_OUT_INIT_HIGH,
    					    "emmc_reset"))  {
    			dev_err(host->dev, "%s : emmc h/w reset gpio[%d] request fail!!!\n", __func__, np_gpio);
		    }
    		else {
        		dev_info(host->dev, "%s : emmc h/w reset gpio[%d] notifier registered.\n", __func__, np_gpio);
        		// Export symbol : use arch/arm/mach-exynos/common.c
        	    eMMC_HW_RESET_GPIO = np_gpio;
    		}
    	}
    }
    //-------------------------------------------------------------
#endif    
	return pdata;
}

#else /* CONFIG_OF */
static struct dw_mci_board *dw_mci_parse_dt(struct dw_mci *host)
{
	return ERR_PTR(-EINVAL);
}
#endif /* CONFIG_OF */

int dw_mci_probe(struct dw_mci *host)
{
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	int width, i, ret = 0;
	u32 fifo_size, qrdy_int, msize, tx_wmark, rx_wmark;
	int init_slots = 0;

	if (drv_data && drv_data->misc_control) {
		ret = drv_data->misc_control(host, CTRL_TURN_ON_2_8V, NULL);
		if (ret)
			return ret;
	}

	if (!host->pdata) {
		host->pdata = dw_mci_parse_dt(host);
		if (IS_ERR(host->pdata)) {
			dev_err(host->dev, "platform data not available\n");
			return -EINVAL;
		}
	}

	if (drv_data && drv_data->misc_control)
		drv_data->misc_control(host, CTRL_REQUEST_EXT_IRQ,
				dw_mci_detect_interrupt);

	if (!host->pdata->select_slot && host->pdata->num_slots > 1) {
		dev_err(host->dev,
			"Platform data must supply select_slot function\n");
		return -ENODEV;
	}

	/*
	 * Get clock sources
	 */
	host->biu_clk = devm_clk_get(host->dev, "biu");
	if (IS_ERR(host->biu_clk)) {
		dev_dbg(host->dev, "biu clock not available\n");
	}
	host->ciu_clk = devm_clk_get(host->dev, "gate_ciu");
	if (IS_ERR(host->ciu_clk)) {
		dev_dbg(host->dev, "ciu clock not available\n");
		host->bus_hz = host->pdata->bus_hz;
	}
	host->gate_clk = devm_clk_get(host->dev, "gate_mmc");
	if (IS_ERR(host->gate_clk))
		dev_dbg(host->dev, "clock for gating not available\n");

	/*
	 * BIU clock enable
	 */
	ret = dw_mci_biu_clk_en(host);
	if (ret) {
		dev_err(host->dev, "failed to enable biu clock\n");
		goto err_clk_biu;
	}

	/*
	 * CIU clock enable
	 */
	ret = dw_mci_ciu_clk_en(host, true);
	if (ret) {
		goto err_clk_ciu;
	} else {
		if (host->pdata->bus_hz) {
			ret = clk_set_rate(host->ciu_clk,
					host->pdata->bus_hz);
			if (ret)
				dev_warn(host->dev,
					 "Unable to set bus rate to %ul\n",
					 host->pdata->bus_hz);
		}
		host->bus_hz = clk_get_rate(host->ciu_clk);
	}

	if (drv_data && drv_data->setup_clock) {
		ret = drv_data->setup_clock(host);
		if (ret) {
			dev_err(host->dev,
				"implementation specific clock setup failed\n");
			goto err_clk_ciu;
		}
	}

	if (!host->bus_hz) {
		dev_err(host->dev,
			"Platform data must supply bus speed\n");
		ret = -ENODEV;
		goto err_clk_ciu;
	}

	host->quirks = host->pdata->quirks;

	spin_lock_init(&host->lock);
	INIT_LIST_HEAD(&host->queue);

	/*
	 * Get the host data width - this assumes that HCON has been set with
	 * the correct values.
	 */
	i = (mci_readl(host, HCON) >> 7) & 0x7;
	if (!i) {
		host->push_data = dw_mci_push_data16;
		host->pull_data = dw_mci_pull_data16;
		width = 16;
		host->data_shift = 1;
	} else if (i == 2) {
		host->push_data = dw_mci_push_data64;
		host->pull_data = dw_mci_pull_data64;
		width = 64;
		host->data_shift = 3;
	} else {
		/* Check for a reserved value, and warn if it is */
		WARN((i != 1),
		     "HCON reports a reserved host data width!\n"
		     "Defaulting to 32-bit access.\n");
		host->push_data = dw_mci_push_data32;
		host->pull_data = dw_mci_pull_data32;
		width = 32;
		host->data_shift = 2;
	}

	/* Reset all blocks */
	if (!mci_wait_reset(host->dev, host))
		return -ENODEV;

	host->dma_ops = host->pdata->dma_ops;
	dw_mci_init_dma(host);

	/* Clear the interrupts for the host controller */
	mci_writel(host, RINTSTS, 0xFFFFFFFF);
	mci_writel(host, INTMASK, 0); /* disable all mmc interrupt first */

	/* Put in max timeout */
	mci_writel(host, TMOUT, 0xFFFFFFFF);

	if (!host->pdata->fifo_depth) {
		/*
		 * Power-on value of RX_WMark is FIFO_DEPTH-1, but this may
		 * have been overwritten by the bootloader, just like we're
		 * about to do, so if you know the value for your hardware, you
		 * should put it in the platform data.
		 */
		fifo_size = mci_readl(host, FIFOTH);
		fifo_size = 1 + ((fifo_size >> SDMMC_FIFOTH_RX_WMARK) & 0xfff);
	} else {
		fifo_size = host->pdata->fifo_depth;
	}

	host->fifo_depth = fifo_size;

	WARN_ON(fifo_size < 8);

	/*
	 *	HCON[9:7] -> H_DATA_WIDTH
	 *	000 16 bits
	 *	001 32 bits
	 *	010 64 bits
	 *
	 *	FIFOTH[30:28] -> DW_DMA_Mutiple_Transaction_Size
	 *	msize:
	 *	000  1 transfers
	 *	001  4
	 *	010  8
	 *	011  16
	 *	100  32
	 *	101  64
	 *	110  128
	 *	111  256
	 *
	 *	AHB Master can support 1/4/8/16 burst in DMA.
	 *	So, Max support burst spec is 16 burst.
	 *
	 *	msize <= 011(16 burst)
	 *	Transaction_Size = msize * H_DATA_WIDTH;
	 *	rx_wmark = Transaction_Size - 1;
	 *	tx_wmark = fifo_size - Transaction_Size;
	 */
	msize = host->data_shift;
	msize &= 7;
	rx_wmark = ((1 << (msize + 1)) - 1) & 0xfff;
	tx_wmark = (fifo_size - (1 << (msize + 1))) & 0xfff;

	host->fifoth_val = msize << SDMMC_FIFOTH_DMA_MULTI_TRANS_SIZE;
	host->fifoth_val |= (rx_wmark << SDMMC_FIFOTH_RX_WMARK) | tx_wmark;

	mci_writel(host, FIFOTH, host->fifoth_val);

	dev_info(host->dev, "FIFOTH: 0x %08x", mci_readl(host, FIFOTH));

	/* disable clock to CIU */
	mci_writel(host, CLKENA, 0);
	mci_writel(host, CLKSRC, 0);

	/*
	 * In 2.40a spec, Data offset is changed.
	 * Need to check the version-id and set data-offset for DATA register.
	 */
	host->verid = SDMMC_GET_VERID(mci_readl(host, VERID));
	dev_info(host->dev, "Version ID is %04x\n", host->verid);

	if (host->verid < DW_MMC_240A)
		host->data_offset = DATA_OFFSET;
	else
		host->data_offset = DATA_240A_OFFSET;

	tasklet_init(&host->tasklet, dw_mci_tasklet_func, (unsigned long)host);
	host->tasklet_state = 0;
	host->card_workqueue = alloc_workqueue("dw-mci-card",
			WQ_MEM_RECLAIM | WQ_NON_REENTRANT, 1);
	if (!host->card_workqueue)
		goto err_dmaunmap;
	INIT_WORK(&host->card_work, dw_mci_work_routine_card);

	pm_qos_add_request(&host->pm_qos_int, PM_QOS_DEVICE_THROUGHPUT, 0);
	if (host->pdata->tp_mon_tbl) {
		INIT_DELAYED_WORK(&host->tp_mon, dw_mci_tp_mon);
		pm_qos_add_request(&host->pm_qos_mif,
					PM_QOS_BUS_THROUGHPUT, 0);
		pm_qos_add_request(&host->pm_qos_cpu,
					PM_QOS_CPU_FREQ_MIN, 0);
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		pm_qos_add_request(&host->pm_qos_kfc,
					PM_QOS_KFC_FREQ_MIN, 0);
#endif
	}

	ret = devm_request_irq(host->dev, host->irq, dw_mci_interrupt,
			       host->irq_flags, "dw-mci", host);

	setup_timer(&host->timer, dw_mci_timeout_timer, (unsigned long)host);
	setup_timer(&host->dto_timer, dw_mci_dto_timer, (unsigned long)host);

	if (ret)
		goto err_workqueue;

	if (host->pdata->num_slots)
		host->num_slots = host->pdata->num_slots;
	else
		host->num_slots = ((mci_readl(host, HCON) >> 1) & 0x1F) + 1;

	/*
	 * Enable interrupts for command done, data over, data empty, card det,
	 * receive ready and error such as transmit, receive timeout, crc error
	 */
	mci_writel(host, RINTSTS, 0xFFFFFFFF);
	if (host->pdata->cd_type == DW_MCI_CD_INTERNAL)
		mci_writel(host, INTMASK, SDMMC_INT_CMD_DONE |
		   SDMMC_INT_DATA_OVER | SDMMC_INT_TXDR | SDMMC_INT_RXDR |
		   DW_MCI_ERROR_FLAGS | SDMMC_INT_CD);
	else
		mci_writel(host, INTMASK, SDMMC_INT_CMD_DONE |
		   SDMMC_INT_DATA_OVER | SDMMC_INT_TXDR | SDMMC_INT_RXDR |
		   DW_MCI_ERROR_FLAGS);
	mci_writel(host, CTRL, SDMMC_CTRL_INT_ENABLE); /* Enable mci interrupt */

	/* disable queue ready interrupt */
	qrdy_int = mci_readl(host, SHA_CMD_IE);
	qrdy_int &= ~QRDY_INT_EN;
	mci_writel(host, SHA_CMD_IE, qrdy_int);

	dev_info(host->dev, "DW MMC controller at irq %d, "
		 "%d bit host data width, "
		 "%u deep fifo\n",
		 host->irq, width, fifo_size);

	/* We need at least one slot to succeed */
	for (i = 0; i < host->num_slots; i++) {
		ret = dw_mci_init_slot(host, i);
		if (ret)
			dev_dbg(host->dev, "slot %d init failed\n", i);
		else
			init_slots++;
	}

	if (init_slots) {
		dev_info(host->dev, "%d slots initialized\n", init_slots);
	} else {
		dev_dbg(host->dev, "attempted to initialize %d slots, "
					"but failed on all\n", host->num_slots);
		goto err_workqueue;
	}

	if (host->quirks & DW_MCI_QUIRK_IDMAC_DTO)
		dev_info(host->dev, "Internal DMAC interrupt fix enabled.\n");

	if (drv_data && drv_data->register_notifier)
		dw_mci_register_notifier(host, REGISTER_NOTI);


	return 0;

err_workqueue:
	destroy_workqueue(host->card_workqueue);
	pm_qos_remove_request(&host->pm_qos_int);
	if (host->pdata->tp_mon_tbl) {
		pm_qos_remove_request(&host->pm_qos_mif);
		pm_qos_remove_request(&host->pm_qos_cpu);
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		pm_qos_remove_request(&host->pm_qos_kfc);
#endif
	}

err_dmaunmap:
	if (host->use_dma && host->dma_ops->exit)
		host->dma_ops->exit(host);

	if (host->vmmc)
		regulator_disable(host->vmmc);

err_clk_ciu:
	if (!IS_ERR(host->ciu_clk) || !IS_ERR(host->gate_clk))
		dw_mci_ciu_clk_dis(host);
err_clk_biu:
	if (!IS_ERR(host->biu_clk))
		dw_mci_biu_clk_dis(host);

	return ret;
}
EXPORT_SYMBOL(dw_mci_probe);

void dw_mci_remove(struct dw_mci *host)
{
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	int i;

	mci_writel(host, RINTSTS, 0xFFFFFFFF);
	mci_writel(host, INTMASK, 0); /* disable all mmc interrupt first */

	if (host->pdata->cd_type == DW_MCI_CD_EXTERNAL)
		host->pdata->ext_cd_cleanup(&dw_mci_notify_change);

	for (i = 0; i < host->num_slots; i++) {
		dev_dbg(host->dev, "remove slot %d\n", i);
		if (host->slot[i])
			dw_mci_cleanup_slot(host->slot[i], i);
	}

	/* disable clock to CIU */
	mci_writel(host, CLKENA, 0);
	mci_writel(host, CLKSRC, 0);

	del_timer_sync(&host->timer);
	del_timer_sync(&host->dto_timer);
	destroy_workqueue(host->card_workqueue);
	if (host->pdata->tp_mon_tbl) {
		cancel_delayed_work_sync(&host->tp_mon);
		pm_qos_remove_request(&host->pm_qos_mif);
		pm_qos_remove_request(&host->pm_qos_cpu);
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		pm_qos_remove_request(&host->pm_qos_kfc);
#endif
	}

	if (drv_data && drv_data->register_notifier)
		dw_mci_register_notifier(host, UNREGISTER_NOTI);

	pm_qos_remove_request(&host->pm_qos_int);

	if (host->use_dma && host->dma_ops->exit)
		host->dma_ops->exit(host);

	if (host->vmmc)
		regulator_disable(host->vmmc);

	if (!IS_ERR(host->gate_clk))
		dw_mci_ciu_clk_dis(host);
}
EXPORT_SYMBOL(dw_mci_remove);



#ifdef CONFIG_PM_SLEEP
/*
 * TODO: we should probably disable the clock to the card in the suspend path.
 */
int dw_mci_suspend(struct dw_mci *host)
{
	int i, ret = 0;
	u32 clkena;

	for (i = 0; i < host->num_slots; i++) {
		struct dw_mci_slot *slot = host->slot[i];
		if (!slot)
			continue;
		if (slot->mmc) {
			atomic_inc_return(&host->ciu_en_win);
			dw_mci_ciu_clk_en(host, false);
			clkena = mci_readl(host, CLKENA);
			clkena &= ~((SDMMC_CLKEN_LOW_PWR) << slot->id);
			mci_writel(host, CLKENA, clkena);
			mci_send_cmd(slot,
				SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);
			atomic_dec_return(&host->ciu_en_win);

			slot->mmc->pm_flags |= slot->mmc->pm_caps;
			ret = mmc_suspend_host(slot->mmc);
			if (ret < 0) {
				while (--i >= 0) {
					slot = host->slot[i];
					if (slot)
						mmc_resume_host(host->slot[i]->mmc);
				}
				return ret;
			}
		}
	}

	if (host->pdata->tp_mon_tbl &&
		(host->pdata->pm_caps & MMC_PM_KEEP_POWER)) {
		cancel_delayed_work_sync(&host->tp_mon);
		pm_qos_update_request(&host->pm_qos_mif, 0);
		pm_qos_update_request(&host->pm_qos_cpu, 0);
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		pm_qos_update_request(&host->pm_qos_kfc, 0);
#endif
		host->transferred_cnt = 0;
		host->cmd_cnt = 0;
	}
	if (host->vmmc)
		regulator_disable(host->vmmc);

	return 0;
}
EXPORT_SYMBOL(dw_mci_suspend);

int dw_mci_resume(struct dw_mci *host)
{
	const struct dw_mci_drv_data *drv_data = host->drv_data;

	int i, ret;

	host->current_speed = 0;

	if (host->vmmc) {
		ret = regulator_enable(host->vmmc);
		if (ret) {
			dev_err(host->dev,
				"failed to enable regulator: %d\n", ret);
			return ret;
		}
	}

	atomic_inc_return(&host->ciu_en_win);
	dw_mci_ciu_clk_en(host, false);

	if (!mci_wait_reset(host->dev, host)) {
		dw_mci_ciu_clk_dis(host);
		atomic_dec_return(&host->ciu_en_win);
		ret = -ENODEV;
		return ret;
	}
	atomic_dec_return(&host->ciu_en_win);

	if (host->use_dma && host->dma_ops->init)
		host->dma_ops->init(host);

	if (host->pdata->quirks & DW_MCI_QUIRK_BYPASS_SMU)
		drv_data->cfg_smu(host);

#ifdef CONFIG_MMC_DW_FMP_DM_CRYPT
	ret = exynos_smc(SMC_CMD_FMP, FMP_MMC_RESUME, 0, 0);
#endif

	/* Restore the old value at FIFOTH register */
	mci_writel(host, FIFOTH, host->fifoth_val);

	mci_writel(host, RINTSTS, 0xFFFFFFFF);
	if (host->pdata->cd_type == DW_MCI_CD_INTERNAL)
		mci_writel(host, INTMASK, SDMMC_INT_CMD_DONE |
		   SDMMC_INT_DATA_OVER | SDMMC_INT_TXDR | SDMMC_INT_RXDR |
		   DW_MCI_ERROR_FLAGS | SDMMC_INT_CD);
	else
		mci_writel(host, INTMASK, SDMMC_INT_CMD_DONE |
		   SDMMC_INT_DATA_OVER | SDMMC_INT_TXDR | SDMMC_INT_RXDR |
		   DW_MCI_ERROR_FLAGS);
	mci_writel(host, CTRL, SDMMC_CTRL_INT_ENABLE);

	for (i = 0; i < host->num_slots; i++) {
		struct dw_mci_slot *slot = host->slot[i];
		struct mmc_ios ios;
		if (!slot)
			continue;
		if (slot->mmc->pm_flags & MMC_PM_KEEP_POWER &&
					dw_mci_get_cd(slot->mmc)) {
			memcpy(&ios, &slot->mmc->ios,
						sizeof(struct mmc_ios));
			ios.timing = MMC_TIMING_LEGACY;
			dw_mci_set_ios(slot->mmc, &ios);
			dw_mci_set_ios(slot->mmc, &slot->mmc->ios);
			atomic_inc_return(&host->ciu_en_win);
			dw_mci_ciu_clk_en(host, false);
			dw_mci_setup_bus(slot, true);
			atomic_dec_return(&host->ciu_en_win);
			if (host->pdata->tuned) {
				if (drv_data && drv_data->misc_control)
					drv_data->misc_control(host,
						CTRL_RESTORE_CLKSEL, NULL);
				mci_writel(host, CDTHRCTL,
						host->cd_rd_thr << 16 | 1);
			}
		}

		if (dw_mci_get_cd(slot->mmc))
			set_bit(DW_MMC_CARD_PRESENT, &slot->flags);
		else
			clear_bit(DW_MMC_CARD_PRESENT, &slot->flags);

		ret = mmc_resume_host(host->slot[i]->mmc);
		if (ret < 0)
			return ret;
	}

	if (host->pdata->tp_mon_tbl &&
		(host->pdata->pm_caps & MMC_PM_KEEP_POWER)) {
		host->transferred_cnt = 0;
		host->cmd_cnt = 0;
		schedule_delayed_work(&host->tp_mon, HZ);
	}

	return 0;
}
EXPORT_SYMBOL(dw_mci_resume);
#endif /* CONFIG_PM_SLEEP */

static int __init dw_mci_init(void)
{
	pr_info("Synopsys Designware Multimedia Card Interface Driver\n");
	return 0;
}

static void __exit dw_mci_exit(void)
{
}

module_init(dw_mci_init);
module_exit(dw_mci_exit);

MODULE_DESCRIPTION("DW Multimedia Card Interface driver");
MODULE_AUTHOR("NXP Semiconductor VietNam");
MODULE_AUTHOR("Imagination Technologies Ltd");
MODULE_LICENSE("GPL v2");
