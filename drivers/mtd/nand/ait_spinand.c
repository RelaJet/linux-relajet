/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <asm/sizes.h>
#include <mach/ait_spinand.h>

//#define SPINAND_DEBUG
#ifdef SPINAND_DEBUG
#define sn_dbg(format, arg...) printk( format, ##arg)
#else
#define sn_dbg(format, arg...)
#endif

//Definition
#define	DRIVER_NAME	"ait_spinand"
#define MAX_PAGE_SIZE	0x2000
#define MAX_OOB_SIZE	0x200

//variable
static struct nand_ecclayout nand_oob_64 = {
	.eccbytes = 14*4,
	.eccpos = {3, 4, 5, 6, 7, 8, 9, 10, 11,
		},
};

//SPINAND PARTITION
static struct mtd_partition ait_nand_partition[] = {
	{
		.name	= "boot",
		.size	= 1 * SZ_128K,
		.offset = 0x0,
        },{
                .name   = "u-boot",
                .size   = 5 * SZ_128K,
                .offset = MTDPART_OFS_APPEND,
	},{
		.name	= "kernel",
		.size	= 5 * SZ_1M,
		.offset	= MTDPART_OFS_APPEND,
	},{
		.name	= "rootfs",
		.size	= 100 * SZ_1M,
		.offset	= MTDPART_OFS_APPEND,
	},{
		.name	= "usrdata",
		.size	= MTDPART_SIZ_FULL,
		.offset	= MTDPART_OFS_APPEND,
	},
};

static void ait_spinand_read_id(struct mtd_info *mtd);
static int spinand_write_read(struct spi_device *spi, const void *txbuf, unsigned n_tx, void *rxbuf, unsigned n_rx)
{
	int status;
	struct spi_message message;
	struct spi_transfer x[2];
	sn_dbg("%s n_tx = 0x%x, n_rx = 0x%x\n", __func__, n_tx,  n_rx);

	//setup spi message
	spi_message_init(&message);
	memset(x, 0, sizeof x);
	if(n_tx)
	{
		x[0].len = n_tx;
		x[0].tx_buf = txbuf;
		spi_message_add_tail(&x[0], &message);
	}
	if(n_rx)
	{
		x[1].len = n_rx;
		x[1].rx_buf = rxbuf;
		spi_message_add_tail(&x[1], &message);
	}

	//do spi transfer
	status = spi_sync(spi, &message);
	if(status != 0)
	{
		printk("ERROR: %s status = 0x%x\n", __func__, status);
		return -EIO;
	}

	return 0;
}

static int spinand_write_data(struct spi_device *spi, const void *cmd_buf, unsigned n_cmd, void *txbuf, unsigned n_tx)
{
	int status;
	struct spi_message message;
	struct spi_transfer x[2];
	sn_dbg("%s n_cmd = 0x%x, n_tx = 0x%x\n", __func__, n_cmd,  n_tx);

	//setup spi message
	spi_message_init(&message);
	memset(x, 0, sizeof x);
	if(n_cmd)
	{
		x[0].len = n_cmd;
		x[0].tx_buf = cmd_buf;
		spi_message_add_tail(&x[0], &message);
	}
	if(n_tx)
	{
		x[1].len = n_tx;
		x[1].tx_buf = txbuf;
		spi_message_add_tail(&x[1], &message);
	}

	//do spi transfer
	status = spi_sync(spi, &message);
	if(status != 0)
	{
		printk("ERROR: %s status = 0x%x\n", __func__, status);
		return -EIO;
	}

	return 0;
}

static void ait_spinand_write_enable(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	unsigned char cmd = SPINAND_WRITE_ENABLE;
	int ret;

	//send write enable
	ret = spinand_write_read(spi, &cmd, 1, 0, 0);
	if(ret)
	{
		printk("ERROR: SPINAND send write enable fail\n");
	}

	sn_dbg("Enable SPINAND write\n");
}

static void ait_spinand_write_disable(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	unsigned char cmd = SPINAND_WRITE_DISABLE;
	int ret;

	//send write disable
	ret = spinand_write_read(spi, &cmd, 1, 0, 0);
	if(ret)
	{
		printk("ERROR: SPINAND send write disable fail\n");
	}

	sn_dbg("Disable SPINAND write\n");
}

static unsigned char ait_spinand_get_status(struct mtd_info *mtd, unsigned char reg)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	unsigned char status_cmd[2] = {SPINAND_GET_FEATURE, reg}; //third byts is dummy
	unsigned char status_buf[2] = {0}; //status_buf[1] -> dummy
	int ret;

	//get status
	ret = spinand_write_read(spi, status_cmd, sizeof(status_cmd), status_buf, 1);
	if(ret)
	{
		printk("ERROR: %s SPINAND get status cmd = 0x%x, reg =0x%x fail\n", __func__, SPINAND_GET_FEATURE, reg);
	}

	sn_dbg("%s register = 0x%x, status = 0x%x\n", __func__, reg, status_buf[0]);

	return status_buf[0];
}

static void ait_spinand_set_status(struct mtd_info *mtd, unsigned char reg, unsigned char data)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	unsigned char status_cmd[3] = {SPINAND_SET_FEATURE, reg, data};
	int ret;

	//enable write
	ait_spinand_write_enable(mtd);

	//set status
	ret = spinand_write_read(spi, status_cmd, sizeof(status_cmd), 0, 0);
	if(ret)
	{
		printk("ERROR: %s SPINAND set status cmd = 0x%x, reg = 0x%x, data = 0x%x fail\n", __func__, SPINAND_SET_FEATURE, reg, data);
	}

	sn_dbg("%s register = 0x%x, data = 0x%x\n", __func__, reg, data);
}

static void ait_spinand_enable_ecc(struct mtd_info *mtd)
{
	unsigned char status_b0;

	//get ecc enable register
	status_b0 = ait_spinand_get_status(mtd, 0xB0);

	//check ecc enable bit if we need to enable
	if((status_b0 & GIGA_STATUS_B0_ECC_EN) != GIGA_STATUS_B0_ECC_EN)
	{
		ait_spinand_set_status(mtd, 0xB0, status_b0 | GIGA_STATUS_B0_ECC_EN);
	}

	sn_dbg("%s enable ecc\n", __func__);
}

static void ait_spinand_disable_ecc(struct mtd_info *mtd)
{
	unsigned char status_b0;

	//get ecc enable register
	status_b0 = ait_spinand_get_status(mtd, 0xB0);

	//check ecc enable bit if we need to disable
	if((status_b0 & GIGA_STATUS_B0_ECC_EN) == GIGA_STATUS_B0_ECC_EN)
	{
		ait_spinand_set_status(mtd, 0xB0, (status_b0 & ~GIGA_STATUS_B0_ECC_EN));
	}

	sn_dbg("%s disable ecc\n", __func__);
}

/**
 * @param mtd   MTD device structure
 * @return
 *      1 - busy
 *      0 - ready
 */
static int ait_spinand_wait_dev_ready(struct mtd_info *mtd)
{
	int i = 0, status_c0;

	//read the status and check if device is busy
	do{
		status_c0 = ait_spinand_get_status(mtd, 0xC0);
		if((status_c0 & GIGA_STATUS_C0_OIP) == GIGA_STATUS_C0_OIP)
		{
			udelay(1000);
			i++;
			if(i == 10)
			{
				printk("ERROR: %s timeout\n", __func__);
				return 1;
			}
		}else{
			break;
		}
	}while(i < 10);

	return 0;
}

static void ait_spinand_read_status(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;
	unsigned char status_c0, status_a0, *rev_buf = chip->buffers->databuf;

	//get status c0
	status_c0 = ait_spinand_get_status(mtd, 0xC0);

	//get status a0
	status_a0 = ait_spinand_get_status(mtd, 0xA0);

	//transfer spinand status to normal nand status
	//reset status byte
	*rev_buf = 0;

	//check write protect
	if(((status_a0 & GIGA_STATUS_A0_BP0) != GIGA_STATUS_A0_BP0) &&
	   ((status_a0 & GIGA_STATUS_A0_BP1) != GIGA_STATUS_A0_BP1) &&
	   ((status_a0 & GIGA_STATUS_A0_BP1) != GIGA_STATUS_A0_BP1))
	{
		*rev_buf = *rev_buf | NAND_STATUS_WP;
	}

	//check erase and program status
	if(((status_c0 & GIGA_STATUS_C0_E_F) == GIGA_STATUS_C0_E_F) ||
	   ((status_c0 & GIGA_STATUS_C0_P_F) == GIGA_STATUS_C0_P_F))
	{
		*rev_buf = *rev_buf | NAND_STATUS_FAIL;
	}

	//check busy status
	if((status_c0 & GIGA_STATUS_C0_OIP) != GIGA_STATUS_C0_OIP)
	{
		*rev_buf = *rev_buf | NAND_STATUS_TRUE_READY | NAND_STATUS_READY;
	}

	//reset r_index
	info->r_index = 0;

	sn_dbg("%s status_c0 = 0x%x, status_a0 = 0x%x, report status = 0x%x\n", __func__, status_c0, status_a0, *rev_buf);
}

static void ait_spinand_erase(struct mtd_info *mtd, int page)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	unsigned char cmd[4] = {SPINAND_BLOCK_ERASE, ((page >> 16) & 0xFF), ((page >> 8) & 0xFF), (page & 0xFF)};
	int ret;

	//enable write
	ait_spinand_write_enable(mtd);

	//send erase cmd (SPI CMD = 0xD8)
	ret = spinand_write_read(spi, cmd, sizeof(cmd), 0, 0);
	if(ret)
	{
		printk("ERROR: SPINAND send erase cmd(0x%x) fail\n", SPINAND_BLOCK_ERASE);
	}

	//wait device ready. It could delete.
	ret = ait_spinand_wait_dev_ready(mtd);
	if(ret)
	{
		printk("ERROR: %s SPINAND wiat busy timeout\n", __func__);
	}

	sn_dbg("%s page = 0x%x\n", __func__, page);
}

static void ait_spinand_reset(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	unsigned char reset_cmd = SPINAND_RESET;
	int ret;

	//reset spinand
	ret = spinand_write_read(spi, &reset_cmd, 1, 0, 0);
	if(ret)
	{
		 printk("ERROR: SPINAND send reset cmd(0x%x) fail\n", SPINAND_RESET);
	}

	//reset r_index
	info->r_index = 0;

	sn_dbg("reset spi nand\n");
}

static void ait_spinand_read_id(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	unsigned char read_id_cmd[3] = {SPINAND_READ_ID, 0, 0}, *rev_buf = chip->buffers->databuf, temp_byte;
	int ret;

	//read id
	ret = spinand_write_read(spi, read_id_cmd, sizeof(read_id_cmd), rev_buf, 3);
	if(ret)
	{
		printk("ERROR: SPINAND send read id cmd fail\n");
	}

	//need swap MID and DID
	temp_byte = rev_buf[0];
	rev_buf[0] = rev_buf[1];
	rev_buf[1] = temp_byte;

	if((rev_buf[0] == 0xC8) && (rev_buf[1] == 0x48))
	{
		temp_byte = rev_buf[1];
		rev_buf[1] = rev_buf[2];
		rev_buf[2] = temp_byte;
	}

	sn_dbg("SPINAND ID = 0x%x 0x%x 0x%x\n", rev_buf[0], rev_buf[1], rev_buf[2]);

	//reset r_index
	info->r_index = 0;
}

static uint8_t ait_spinand_r_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;
	unsigned char *rev_buf = chip->buffers->databuf, temp_byte;

	temp_byte = *(rev_buf + info->r_index);
	info->r_index = info->r_index + 1;

	sn_dbg("%s bytes = %x, index = %d\n", __func__, temp_byte, info->r_index - 1);

	return temp_byte;
}

static void ait_spinand_r_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;

	memcpy(buf, chip->buffers->databuf, len);
	info->r_index = info->r_index + len;

	sn_dbg("%s buf = 0x%x len = 0x%x", __func__, (unsigned int)buf, len);
}

static void ait_spinand_w_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;

	memcpy(chip->buffers->databuf, buf, len);

	sn_dbg("%s buf = 0x%x len = 0x%x", __func__, (unsigned int)buf, len);
}

//dummy function
static void ait_spinand_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
	sn_dbg("%s dat = 0x%x, ctrl = 0x%x\n", __func__, dat, ctrl);
}

static void ait_spinand_cmdfunc(struct mtd_info *mtd, unsigned command, int column,int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;

	//change NAND CMD to SPINAND CMD
	switch(command)
	{
	case NAND_CMD_READID:
		sn_dbg("NAND READ ID (0x%x)\n", command);
		ait_spinand_read_id(mtd);
		break;
	case NAND_CMD_READ0:
		sn_dbg("NAND READ DATA (0x%x) column = 0x%x, page_addr = 0x%x, block at 0x%x\n", \
			command, column, page_addr, page_addr/(mtd->erasesize/mtd->writesize));
		break;
	case NAND_CMD_RESET:
		sn_dbg("NAND RESET (0x%x)\n", command);
		ait_spinand_reset(mtd);
		break;
	case NAND_CMD_STATUS:
		sn_dbg("NAND CHECK STATUS (0x%x)\n", command);
		ait_spinand_read_status(mtd);
		break;
	case NAND_CMD_ERASE1:
		sn_dbg("NAND ERASE CMD 1(0x%x) column = 0x%x, page_addr = 0x%x, block at 0x%x\n", \
			command, column, page_addr, page_addr/(mtd->erasesize/mtd->writesize));
		ait_spinand_erase(mtd, page_addr);
		break;
	case NAND_CMD_ERASE2:
		sn_dbg("NAND ERASE CMD 2(0x%x)\n", command);
		break;
	case NAND_CMD_SEQIN:
		sn_dbg("NAND PROGRAM CMD 1(0x%x) column = 0x%x, page_addr = 0x%x, block at 0x%x\n", \
			command, column, page_addr, page_addr/(mtd->erasesize/mtd->writesize));
		info->w_page_addr = page_addr;
		break;
	case NAND_CMD_PAGEPROG:
		sn_dbg("NAND PROGRAM CMD 2(0x%x)\n", command);
		info->w_page_addr = INVAL_PAGE_ADDR;
		break;
	default:
		sn_dbg("ERROR: Command not support (0x%x)\n", command);
	}
}

//dummy for ait_spinand
static void ait_spinand_select_chip(struct mtd_info *mtd, int chip)
{
	sn_dbg("%s chip = %d\n", __func__, chip);
}

static int ait_spinand_init_size(struct mtd_info *mtd, struct nand_chip *this, u8 *id_data)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_spinand_info *info = chip->priv;
	int status_a0, status_b0, status_c0;

	switch(id_data[1])
	{
	case 0xF1:
		//setup mtd information
		printk("SPINAND: GigaDevice, 128K erase size, 2048 page size, 64 oob size\n");
		mtd->erasesize = 0x20000;
		mtd->writesize = 0x800;
		mtd->oobsize = 64;
		info->fast_read = 0;
		info->write_dummy = 0;

		//get a0 status for checking protect
		status_a0 = ait_spinand_get_status(mtd, 0xA0);

		//check lock. If lock all block,
		if((status_a0 & (GIGA_STATUS_A0_BP0 | GIGA_STATUS_A0_BP1 | GIGA_STATUS_A0_BP2)) == (GIGA_STATUS_A0_BP0 | GIGA_STATUS_A0_BP1 | GIGA_STATUS_A0_BP2))
		{
			ait_spinand_set_status(mtd, 0xA0, 0x00);
			sn_dbg("Turn off block protect\n");
		}

		//get all status
		status_a0 = ait_spinand_get_status(mtd, 0xA0);
		status_b0 = ait_spinand_get_status(mtd, 0xB0);
		status_c0 = ait_spinand_get_status(mtd, 0xC0);

		printk("SPINAND: GigaDevice, status_a0 = 0x%x, status_b0 = 0x%x, status_c0 = 0x%x\n", status_a0, status_b0, status_c0);

		break;
	case 0xD1:
		//setup mtd information
		printk("SPINAND: GigaDevice, 128K erase size, 2048 page size, 128 oob size(normal read)\n");
		mtd->erasesize = 0x20000;
		mtd->writesize = 0x800;
		mtd->oobsize = 128;
		info->fast_read = 0;
		info->write_dummy = 0;

		//get a0 status for checking protect
		status_a0 = ait_spinand_get_status(mtd, 0xA0);

		//check lock. If lock all block,
		if((status_a0 & (GIGA_STATUS_A0_BP0 | GIGA_STATUS_A0_BP1 | GIGA_STATUS_A0_BP2)) == (GIGA_STATUS_A0_BP0 | GIGA_STATUS_A0_BP1 | GIGA_STATUS_A0_BP2))
		{
			ait_spinand_set_status(mtd, 0xA0, 0x00);
			sn_dbg("Turn off block protect\n");
		}

		//get all status
		status_a0 = ait_spinand_get_status(mtd, 0xA0);
		status_b0 = ait_spinand_get_status(mtd, 0xB0);
		status_c0 = ait_spinand_get_status(mtd, 0xC0);

		printk("SPINAND: GigaDevice, status_a0 = 0x%x, status_b0 = 0x%x, status_c0 = 0x%x\n", status_a0, status_b0, status_c0);

		break;
	case 0xB1:
		//setup mtd information
		printk("SPINAND: GigaDevice, 128K erase size, 2048 page size, 128 oob size(fast read)\n");
		mtd->erasesize = 0x20000;
		mtd->writesize = 0x800;
		mtd->oobsize = 128;
		info->fast_read = 1;
		info->write_dummy = 0;

		//get a0 status for checking protect
		status_a0 = ait_spinand_get_status(mtd, 0xA0);

		//check lock. If lock all block,
		if((status_a0 & (GIGA_STATUS_A0_BP0 | GIGA_STATUS_A0_BP1 | GIGA_STATUS_A0_BP2)) == (GIGA_STATUS_A0_BP0 | GIGA_STATUS_A0_BP1 | GIGA_STATUS_A0_BP2))
		{
			ait_spinand_set_status(mtd, 0xA0, 0x00);
			sn_dbg("Turn off block protect\n");
		}

		//get all status
		status_a0 = ait_spinand_get_status(mtd, 0xA0);
		status_b0 = ait_spinand_get_status(mtd, 0xB0);
		status_c0 = ait_spinand_get_status(mtd, 0xC0);

		printk("SPINAND: GigaDevice, status_a0 = 0x%x, status_b0 = 0x%x, status_c0 = 0x%x\n", status_a0, status_b0, status_c0);

		break;
	case 0x12:
		//setup mtd information
		printk("SPINAND: ATO, 128K erase size, 2048 page size, 64 oob size\n");
		mtd->erasesize = 0x20000;
		mtd->writesize = 0x800;
		mtd->oobsize = 128;
		info->fast_read = 0;
		info->write_dummy = 0;

		//get a0 status for checking protect
		status_a0 = ait_spinand_get_status(mtd, 0xA0);

		//check lock. If lock all block,
		if((status_a0 & (GIGA_STATUS_A0_BP0 | GIGA_STATUS_A0_BP1 | GIGA_STATUS_A0_BP2)) == (GIGA_STATUS_A0_BP0 | GIGA_STATUS_A0_BP1 | GIGA_STATUS_A0_BP2))
		{
			ait_spinand_set_status(mtd, 0xA0, 0x00);
			sn_dbg("Turn off block protect\n");
		}

		//get all status
		status_a0 = ait_spinand_get_status(mtd, 0xA0);
		status_b0 = ait_spinand_get_status(mtd, 0xB0);
		status_c0 = ait_spinand_get_status(mtd, 0xC0);

		printk("SPINAND: ATO, status_a0 = 0x%x, status_b0 = 0x%x, status_c0 = 0x%x\n", status_a0, status_b0, status_c0);

		break;
	default:
		//setup mtd information
		printk("WARNING: Unknown Device ID\n");
		mtd->erasesize = 0x20000;
		mtd->writesize = 0x800;
		mtd->oobsize =64;
		break;
	}

	return 0;  //0->Bus width 8
}

/*
 *
 * returns 0 : busy. 1 : ready
 */
static int ait_spinand_dev_ready(struct mtd_info *mtd)
{
	int status_c0;

	status_c0 = ait_spinand_get_status(mtd, 0xC0);
	if((status_c0 & GIGA_STATUS_C0_OIP) == GIGA_STATUS_C0_OIP)
	{
		return 0;
	}else{
		return 1;
	}
}

static int ait_spinand_r_page(struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buf, int page)
{
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	unsigned char r13_cmd[4] = {SPINAND_PAGE_READ, ((page >> 16) & 0xFF), ((page >> 8) & 0xFF), (page & 0xFF)};
	unsigned char r0b_cmd[4] = {SPINAND_FAST_READ_CACHE, 0, 0, 0};
	unsigned char status_c0;
	uint8_t *read_buf;
	int ret;
	sn_dbg("%s buf = 0x%x, page = 0x%x\n", __func__, (unsigned int)buf, page);

	//enable ECC for safe
	//ait_spinand_enable_ecc(mtd);

	//change the read cmd base on device
	if(info->fast_read == 0)
	{
		r0b_cmd[0] = SPINAND_READ_CACHE;
	}

	//send first cmd for reading page (SPI CMD = 0x13)
	ret = spinand_write_read(spi, r13_cmd, sizeof(r13_cmd), 0, 0);
	if(ret)
	{
		printk("ERROR: %s SPINAND send read cmd(0x%x) fail\n", __func__, SPINAND_PAGE_READ);
		return -EIO;
	}

	//wait device ready
	ret = ait_spinand_wait_dev_ready(mtd);
	if(ret)
	{
		printk("ERROR: %s SPINAND wiat busy timeout\n", __func__);
		return -EBUSY;
	}

	//get status for ecc check
	status_c0 = ait_spinand_get_status(mtd, 0xC0);

	//check ecc status
	if((status_c0 & (GIGA_STATUS_C0_ECCS0 | GIGA_STATUS_C0_ECCS1)) == GIGA_STATUS_C0_ECCS1)
	{
		printk("ERROR: %s SPINAND ECC is error. status_c0 = 0x%x\n", __func__, status_c0);
		return -EIO;
	}

	//check if buf address is valid
	if(virt_addr_valid(buf) && virt_addr_valid(buf + mtd->writesize))
	{
		read_buf = buf;
	}else{
		read_buf = info->dma_page_buffer;
	}

	//send second cmd for reading page (SPI CMD = 0x0B)
	ret = spinand_write_read(spi, r0b_cmd, sizeof(r0b_cmd), read_buf, mtd->writesize);
	if(ret)
	{
		printk("ERROR: %s SPINAND send read cmd(0x%x) fail(data area)\n", __func__, SPINAND_FAST_READ_CACHE);
		return -EIO;
	}

	//If we use dme_page_buffer, we need to copy data
	if(!(virt_addr_valid(buf) && virt_addr_valid(buf + mtd->writesize)))
	{
		memcpy(buf, read_buf, mtd->writesize);
	}

	return 0;
}

static void ait_spinand_w_page(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buf)
{
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	int page = info->w_page_addr, ret;
	unsigned char w02_cmd[4] = {SPINAND_PROGRAM_LOAD, 0, 0, 0};
	unsigned char w10_cmd[4] = {SPINAND_PROGRAM_EXEC, ((page >> 16) & 0xFF), ((page >> 8) & 0xFF), (page & 0xFF)};
	uint8_t *write_buf;
	sn_dbg("%s buf = 0x%x, page_addr = 0x%x\n", __func__, (unsigned int)buf, page);

	//enable ECC for safe
	//ait_spinand_enable_ecc(mtd);
	//enable write
	ait_spinand_write_enable(mtd);

	//check if buf address is valid
	if(virt_addr_valid(buf) && virt_addr_valid(buf + mtd->writesize))
	{
		write_buf = (uint8_t *)buf;
	}else{
		memcpy(info->dma_page_buffer, buf, mtd->writesize);
		write_buf = info->dma_page_buffer;
	}

	//send first cmd for writing page (SPI CMD = 0x02)
	if(info->write_dummy == 1)  //need to check with vendor
	{
		ret = spinand_write_data(spi, w02_cmd, sizeof(w02_cmd), (void *)write_buf, mtd->writesize);
	}else{
		ret = spinand_write_data(spi, w02_cmd, sizeof(w02_cmd) - 1, (void *)write_buf, mtd->writesize);
	}
	if(ret)
	{
		printk("ERROR: %s SPINAND send write cmd(0x%x) fail\n", __func__, SPINAND_PROGRAM_LOAD);
	}

	//send second cmd for writing page (SPI CMD = 0x10)
	ret = spinand_write_read(spi, w10_cmd, sizeof(w10_cmd), 0, 0);
	if(ret)
	{
		printk("ERROR: %s SPINAND send write cmd(0x%x) fail\n", __func__, SPINAND_PROGRAM_EXEC);
	}

	//wait device ready
	ret = ait_spinand_wait_dev_ready(mtd);
	if(ret)
	{
		printk("ERROR: %s SPINAND wiat busy timeout\n", __func__);
	}
}

static int ait_spinand_r_page_raw(struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buf, int page)
{
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	int ret;
	unsigned char r13_cmd[4] = {SPINAND_PAGE_READ, ((page >> 16) & 0xFF), ((page >> 8) & 0xFF), (page & 0xFF)};
	unsigned char r0b_cmd[4] = {SPINAND_FAST_READ_CACHE, 0, 0, 0};
	sn_dbg("%s buf = 0x%x, page = 0x%x\n", __func__, (unsigned int)buf, page);

	//disable ecc
	ait_spinand_disable_ecc(mtd);

	//change the read cmd base on device
	if(info->fast_read == 0)
	{
		r0b_cmd[0] = SPINAND_READ_CACHE;
	}

	//send first cmd for reading page (SPI CMD = 0x13)
	ret = spinand_write_read(spi, r13_cmd, sizeof(r13_cmd), 0, 0);	
	if(ret)
	{
		printk("ERROR: %s SPINAND send read cmd(0x%x) fail\n", __func__, SPINAND_PAGE_READ);
		return -EIO;
	}

	//wait device ready
	ret = ait_spinand_wait_dev_ready(mtd);
	if(ret)
	{
		printk("ERROR: %s SPINAND wiat busy timeout\n", __func__);
		return -EBUSY;
	}

	//send second cmd for reading page (SPI CMD = 0x0B)
	ret = spinand_write_read(spi, r0b_cmd, sizeof(r0b_cmd), buf, mtd->writesize);
	if(ret)
	{
		printk("ERROR: %s SPINAND send read cmd(0x%x) fail\n", __func__, SPINAND_FAST_READ_CACHE);
		return -EIO;
	}

	//copy oob data into temp buffer, because the framework of nand will copy data from temp buffer.
	memcpy(chip->oob_poi, buf + mtd->writesize, mtd->oobsize);

	sn_dbg("%s status_a0 = 0x%x, status_b0 = 0x%x, status_c0 = 0x%x\n", __func__, \
		ait_spinand_get_status(mtd, 0xA0), ait_spinand_get_status(mtd, 0xB0), ait_spinand_get_status(mtd, 0xC0));

	return 0;
}

static void ait_spinand_w_page_raw(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buf)
{
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	int page = info->w_page_addr, ret;
	unsigned char w02_cmd[4] = {SPINAND_PROGRAM_LOAD, 0, 0, 0};
	unsigned char w10_cmd[4] = {SPINAND_PROGRAM_EXEC, ((page >> 16) & 0xFF), ((page >> 8) & 0xFF), (page & 0xFF)};
	uint8_t *write_buf;
	sn_dbg("%s buf = 0x%x, page_addr = 0x%x\n", __func__, (unsigned int)buf, page);

	//disable ecc
	ait_spinand_disable_ecc(mtd);

	//enable write
	ait_spinand_write_enable(mtd);

	//check if buf address is valid
	if(virt_addr_valid(buf) && virt_addr_valid(buf + mtd->writesize + mtd->oobsize))
	{
		write_buf = (uint8_t *)buf;
	}else{
		memcpy(info->dma_page_buffer, buf, mtd->writesize + mtd->oobsize);
		write_buf = info->dma_page_buffer;
	}

	//send first cmd for writing page (SPI CMD = 0x02)
	if(info->write_dummy == 1)  //need to check with vendor
	{
		ret = spinand_write_data(spi, w02_cmd, sizeof(w02_cmd), (void *)write_buf, mtd->writesize + mtd->oobsize);
	}else{
		ret = spinand_write_data(spi, w02_cmd, sizeof(w02_cmd) - 1, (void *)write_buf, mtd->writesize + mtd->oobsize);
	}
	if(ret)
	{
		printk("ERROR: %s SPINAND send write cmd(0x%x) fail\n", __func__, SPINAND_PROGRAM_LOAD);
	}

	//send second cmd for writing page (SPI CMD = 0x10)
	ret = spinand_write_read(spi, w10_cmd, sizeof(w10_cmd), 0, 0);
	if(ret)
	{
		printk("ERROR: %s SPINAND send write cmd(0x%x) fail\n", __func__, SPINAND_PROGRAM_EXEC);
	}

	//wait device ready
	ret = ait_spinand_wait_dev_ready(mtd);
	if(ret)
	{
		printk("ERROR: %s SPINAND wiat busy timeout\n", __func__);
	}

	//enable ECC for safe
	ait_spinand_enable_ecc(mtd);
}

static int ait_spinand_r_oob(struct mtd_info *mtd, struct nand_chip *chip, int page, int sndcmd)
{
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	int ret;
	unsigned char r13_cmd[4] = {SPINAND_PAGE_READ, ((page >> 16) & 0xFF), ((page >> 8) & 0xFF), (page & 0xFF)};
	unsigned char r0b_cmd[4] = {SPINAND_FAST_READ_CACHE, 0, 0, 0};
	sn_dbg("%s page = 0x%x, sndcmd = 0x%x\n", __func__, page, sndcmd);

	//change the read cmd base on device
	if(info->fast_read == 0)
	{
		r0b_cmd[0] = SPINAND_READ_CACHE;
	}

	//send first cmd for reading page (SPI CMD = 0x13)
	ret = spinand_write_read(spi, r13_cmd, sizeof(r13_cmd), 0, 0);
	if(ret)
	{
		printk("ERROR: %s SPINAND send read cmd(0x%x) fail\n", __func__, SPINAND_PAGE_READ);
		return -EIO;
	}

	//wait device ready
	ret = ait_spinand_wait_dev_ready(mtd);
	if(ret)
	{
		printk("ERROR: %s SPINAND wiat busy timeout\n", __func__);
		return -EBUSY;
	}

	//send second cmd for reading data area + spare area(SPI CMD = 0x0B)
	ret = spinand_write_read(spi, r0b_cmd, sizeof(r0b_cmd), chip->buffers->databuf, mtd->writesize + mtd->oobsize);
	if(ret)
	{
		printk("ERROR: %s SPINAND send read cmd(0x%x) fail(data area)\n", __func__, SPINAND_FAST_READ_CACHE);
		return -EIO;
	}

	return 0;
}

static int ait_spinand_w_oob(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	struct ait_spinand_info *info = chip->priv;
	struct spi_device *spi = info->spi;
	int ret;
	unsigned char w02_cmd[4] = {SPINAND_PROGRAM_LOAD, 0, 0, 0};
	unsigned char w10_cmd[4] = {SPINAND_PROGRAM_EXEC, ((page >> 16) & 0xFF), ((page >> 8) & 0xFF), (page & 0xFF)};
	uint8_t *write_buf = chip->buffers->databuf;
	sn_dbg("%s page = 0x%x\n", __func__, page);

	//enable write
	ait_spinand_write_enable(mtd);

	//send first cmd for writing page (SPI CMD = 0x02)
	if(info->write_dummy == 1)  //need to check with vendor
	{
		ret = spinand_write_data(spi, w02_cmd, sizeof(w02_cmd), (void *)write_buf, mtd->writesize + mtd->oobsize);
	}else{
		ret = spinand_write_data(spi, w02_cmd, sizeof(w02_cmd) - 1, (void *)write_buf, mtd->writesize + mtd->oobsize);
	}
	if(ret)
	{
		printk("ERROR: %s SPINAND send write cmd(0x%x) fail\n", __func__, SPINAND_PROGRAM_LOAD);
	}

	//send second cmd for writing page (SPI CMD = 0x10)
	ret = spinand_write_read(spi, w10_cmd, sizeof(w10_cmd), 0, 0);
	if(ret)
	{
		printk("ERROR: %s SPINAND send write cmd(0x%x) fail\n", __func__, SPINAND_PROGRAM_EXEC);
	}

	//wait device ready
	ret = ait_spinand_wait_dev_ready(mtd);
	if(ret)
	{
		printk("ERROR: %s SPINAND wiat busy timeout\n", __func__);
	}

	return 0;
}

static int __devinit ait_spinand_probe(struct spi_device *spi)
{
	unsigned char id[5] = {0};
	unsigned char reset_cmd = SPINAND_RESET;
	unsigned char read_id_cmd[3] = {SPINAND_READ_ID, 0, 0};
	struct ait_spinand_info *info;
	printk("%s cs = 0x%x, SPEED = %d, mode = 0x%x\n", __func__, spi->chip_select, spi->max_speed_hz, spi->mode);

	//reset spinand
	spinand_write_read(spi, &reset_cmd, 1, 0, 0);

	//read id
	spinand_write_read(spi, read_id_cmd, sizeof(read_id_cmd), id, 3);
	sn_dbg("%s id[0] = 0x%x id[1] = 0x%x id[2] = 0x%x\n", __func__, id[0], id[1], id[2]);

	//alloc device struct memory
	info = kzalloc(sizeof(struct ait_spinand_info), GFP_KERNEL);
	if(!info)
	{
		return -ENOMEM;
	}

	//setup spi info
	info->spi = spi;

	//set device data
	dev_set_drvdata(&spi->dev, info);

	//setup mtd info
	info->mtd.priv = &info->nand;
	info->mtd.name = DRIVER_NAME;
	info->mtd.owner = THIS_MODULE;

	//setup nand interface functions and settings
	info->nand.priv = info;
	info->nand.read_byte = ait_spinand_r_byte;
	info->nand.read_buf = ait_spinand_r_buf;
	info->nand.write_buf = ait_spinand_w_buf;
	info->nand.cmd_ctrl = ait_spinand_cmd_ctrl;
	info->nand.cmdfunc = ait_spinand_cmdfunc;
	info->nand.select_chip = ait_spinand_select_chip;
	info->nand.init_size = ait_spinand_init_size;
	info->nand.dev_ready = ait_spinand_dev_ready;
	info->nand.buffers = kmalloc(sizeof(struct nand_buffers), GFP_KERNEL);
	if(!info->nand.buffers)
	{
		kfree(info);
		return -ENOMEM;
	}
	info->dma_page_buffer = kmalloc(MAX_PAGE_SIZE + MAX_OOB_SIZE, GFP_KERNEL);	//This buffer is only for vmalloc memory 
	if(!info->dma_page_buffer)
	{
		kfree(info);
		return -ENOMEM;
	}
	info->nand.options |= NAND_NO_SUBPAGE_WRITE | NAND_OWN_BUFFERS;

	//setup nand ecc function and settings
	info->nand.ecc.mode = NAND_ECC_HW;
	info->nand.ecc.layout = &nand_oob_64;
	info->nand.ecc.steps = 4;
	info->nand.ecc.size = 128;
	info->nand.ecc.bytes = 16;
	info->nand.ecc.read_page = ait_spinand_r_page;
	info->nand.ecc.write_page = ait_spinand_w_page;
	info->nand.ecc.read_page_raw = ait_spinand_r_page_raw;
	info->nand.ecc.write_page_raw = ait_spinand_w_page_raw;
	info->nand.ecc.read_oob = ait_spinand_r_oob;
	info->nand.ecc.write_oob = ait_spinand_w_oob;

	//scan nand 
	if(nand_scan(&info->mtd, 1))
	{
		kfree(info);
		kfree(info->nand.buffers);
		return -ENXIO;
	}

	mtd_device_parse_register(&info->mtd, NULL, NULL, ait_nand_partition, ARRAY_SIZE(ait_nand_partition));

	return 0;
}

static int ait_spinand_remove(struct spi_device *spi)
{
	struct ait_spinand_info *info = dev_get_drvdata(&spi->dev);

	//check if it need to release resource
	if(info != NULL)
	{
		//free nand buffer
		if(info->nand.buffers != NULL)
		{
			kfree(info->nand.buffers);
		}

		//free dma buffer
		if(info->dma_page_buffer != NULL)
		{
			kfree(info->dma_page_buffer);
		}

		//release nand
		nand_release(&info->mtd);

		//free info memory
		kfree(info);
	}

	return 0;
}

static struct spi_driver ait_spinand_driver = {
	.probe		= ait_spinand_probe,
	.remove		= ait_spinand_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ait_spinand_init(void)
{
	//register spi driver
	return spi_register_driver(&ait_spinand_driver);
}

static void __exit ait_spinand_exit(void)
{

}

//late_initcall(ait_spinand_init);
module_init(ait_spinand_init);
module_exit(ait_spinand_exit);
