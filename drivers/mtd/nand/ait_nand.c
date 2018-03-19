/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <asm/sizes.h>
#include <mach/ait_nand.h>

//#define NAND_DEBUG
#ifdef NAND_DEBUG
#define n_dbg(format, arg...) printk( format, ##arg)
#else
#define n_dbg(format, arg...)
#endif
#define MAX_PAGE_SIZE	0x2000
#define MAX_OOB_SIZE	0x200
#define NAND_DMA_READ	0
#define NAND_DMA_WRITE	1
static struct nand_ecclayout nand_oob_64 = {
        .eccbytes = 6*4,
        .eccpos = {0x8, 0x9, 0xa, 0xd, 0xe, 0xf, 0x18, 0x19, 0x1a, 0x1d, 0x1e, 0x1f, \
			0x28, 0x29, 0x2a, 0x2d, 0x2e, 0x2f, 0x38, 0x39, 0x3a, 0x3d, 0x3e, 0x3f,
                },
};
static struct mtd_partition ait_nand_partition[] = {
	{
		.name   = "bootlooder",
		.size   = 1 * SZ_128K,
		.offset = 0x0,
	},{
		.name	= "u-boot",
		.size	= 5 * SZ_128K, 
		.offset	= MTDPART_OFS_APPEND,
	},{
		.name   = "kernel",
		.size   = 5 * SZ_1M,
		.offset = SZ_1M,
	},{
		.name   = "rootfs",
		.size   = 100 * SZ_1M,
		.offset = MTDPART_OFS_APPEND,
	},{
		.name   = "usrdata",
		.size   = MTDPART_SIZ_FULL,
		.offset = MTDPART_OFS_APPEND,
	},
};


static void nand_cmd(struct mtd_info *mtd, unsigned char cmd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;

	pNAD->NAD_CTL |= NAD_CLE;
	pNAD->NAD_CAD = cmd;
	pNAD->NAD_CTL &= ~NAD_CLE;
}

static void nand_w_addr(struct mtd_info *mtd, unsigned char *addr, int num)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;
	int i;

	pNAD->NAD_CTL |= NAD_ALE;
	for(i=0;i<num;i++)
	{
		pNAD->NAD_CAD = *(addr +i);
	}
	pNAD->NAD_CTL &= ~NAD_ALE;
}

static void nand_w_data(struct mtd_info *mtd, unsigned char data)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;

	pNAD->NAD_CAD = data;
}

static unsigned char nand_r_data(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;

	return pNAD->NAD_CAD;
}

static unsigned int nand_map_memory(struct mtd_info *mtd, int rd_wr, void *buf, unsigned int len)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	struct platform_device *pdev = info->pdev;
	dma_addr_t dma_addr = 0;

	if(rd_wr == NAND_DMA_READ)
	{
		dma_addr = dma_map_single(&pdev->dev, buf, len, DMA_FROM_DEVICE);
		if(dma_mapping_error(&pdev->dev, dma_addr))
		{
			return -ENOMEM;
		}
	}else if(rd_wr == NAND_DMA_WRITE){
		dma_addr = dma_map_single(&pdev->dev, buf, len, DMA_TO_DEVICE);
		if(dma_mapping_error(&pdev->dev, dma_addr))
		{
			return -ENOMEM;
		}
	}else{
		printk("ERROR: %s map type(0x%x) is unkown\n", __func__, rd_wr);
		return -ENOMEM;
	}

	n_dbg("%s buf = 0x%x len = 0x%x rd_wr = 0x%x dma_addr = 0x%x\n", __func__, (unsigned int)buf, len, rd_wr, dma_addr);

	return dma_addr;
}

static unsigned int nand_unmap_memory(struct mtd_info *mtd, int rd_wr, dma_addr_t dma_addr, unsigned int len)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	struct platform_device *pdev = info->pdev;
	
	if(rd_wr == NAND_DMA_READ)
	{
		dma_unmap_single(&pdev->dev, dma_addr, len, DMA_FROM_DEVICE);
	}else if(rd_wr == NAND_DMA_WRITE){
		dma_unmap_single(&pdev->dev, dma_addr, len, DMA_TO_DEVICE);
	}else{
		printk("ERROR: %s map type(0x%x) is unkown\n", __func__, rd_wr);
		return -EIO;
	}

	n_dbg("%s len = 0x%x rd_wr = 0x%x dma_addr = 0x%x\n", __func__, len, rd_wr, dma_addr);

	return 0;
}

static int nand_wait_dma(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;
	int i = 0;

	pNAD->NAD_DMA_CTL = NAD_DMA_EN;

	do{
		if(i > 1000)
		{
			printk("ERROR: wait dma timeout NAD_CPU_INT_SR = 0x%x\n", pNAD->NAD_CPU_INT_SR);
			printk("Nand can't read/write before reboot\n");
			pNAD->NAD_DMA_CTL = 0;
			return -EBUSY;
		}else{
			i++;
			udelay(100);
		}
	}while(!(pNAD->NAD_CPU_INT_SR & NAD_DMA_DONE));

	return 0;
}

static int nand_wait_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;

	while(!(pNAD->NAD_CTL & NAD_RB));

	return 0;
}

static uint8_t ait_nand_r_byte(struct mtd_info *mtd)
{
	unsigned char temp_byte;

	temp_byte = nand_r_data(mtd);
	n_dbg("%s bytes = %x\n", __func__, temp_byte);

	return temp_byte;
}

static void ait_nand_r_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i=0;

	for(i=0;i<len;i++)
	{
		*(buf+i) = nand_r_data(mtd);
	}

	n_dbg("%s buf = 0x%x len = 0x%x", __func__, (unsigned int)buf, len);
}

static void ait_nand_w_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i=0;

	for(i=0;i<len;i++)
	{
		nand_w_data(mtd, *(buf+i));
	}

	n_dbg("%s buf = 0x%x len = 0x%x", __func__, (unsigned int)buf, len);
}

//linux kernel control nand pin function.
static void ait_nand_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
	n_dbg("%s dat = 0x%x, ctrl = 0x%x\n", __func__, dat, ctrl);
}

static void ait_nand_cmdfunc(struct mtd_info *mtd, unsigned command, int column,int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	unsigned char addr[3]={0};
	AITPS_NAND  pNAD = info->nand_reg;

	switch(command)
	{
	case NAND_CMD_READID:
		n_dbg("NAND READ ID (0x%x)\n", command);
		nand_cmd(mtd, command);
		nand_w_addr(mtd, addr, 1);
		break;
	case NAND_CMD_READ0:
		n_dbg("NAND READ DATA (0x%x) column = 0x%x, page_addr = 0x%x, block at 0x%x\n", \
			command, column, page_addr, page_addr/(mtd->erasesize/mtd->writesize));
		break;
	case NAND_CMD_RESET:
		n_dbg("NAND RESET (0x%x)\n", command);
		nand_cmd(mtd, command);
		do{
			udelay(10);
		}while(!(pNAD->NAD_CTL & NAD_RB));
		break;
	case NAND_CMD_STATUS:
		n_dbg("NAND CHECK STATUS (0x%x)\n", command);
		nand_cmd(mtd, command);
		break;
	case NAND_CMD_ERASE1:
		n_dbg("NAND ERASE CMD 1(0x%x) column = 0x%x, page_addr = 0x%x, block at 0x%x\n", \
			command, column, page_addr, page_addr/(mtd->erasesize/mtd->writesize));
		nand_cmd(mtd, command);
		addr[0] = (page_addr & 0xFF);
		addr[1] = ((page_addr >> 8) & 0xFF);
		addr[2] = ((page_addr >> 16) & 0xFF);
		if(chip->chipsize > SZ_128M)
		{
			nand_w_addr(mtd, addr, 3);
		}else{
			nand_w_addr(mtd, addr, 2);
		}
		break;
	case NAND_CMD_ERASE2:
		n_dbg("NAND ERASE CMD 2(0x%x)\n", command);
		nand_cmd(mtd, command);
		break;
	case NAND_CMD_SEQIN:
		n_dbg("NAND PROGRAM CMD 1(0x%x) column = 0x%x, page_addr = 0x%x, block at 0x%x\n", \
			command, column, page_addr, page_addr/(mtd->erasesize/mtd->writesize));
		info->w_page_addr = page_addr;
		break;
	case NAND_CMD_PAGEPROG:
		n_dbg("NAND PROGRAM CMD 2(0x%x)\n", command);
		info->w_page_addr = INVAL_PAGE_ADDR;
		break;
	default:
		n_dbg("ERROR: Command not support (0x%x)\n", command);
	}
}

static void ait_nand_select_chip(struct mtd_info *mtd, int chip_nr)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;

	if(chip_nr == 0)
	{
		pNAD->NAD_CTL = NAD_CE2 | NAD_EN | NAD_WPN;
	}else if(chip_nr < 0){
		pNAD->NAD_CTL = NAD_CE1 | NAD_CE2;
	}else{
		printk("ERROR: not support chip select = 0x%x\n", chip_nr);
	}
	n_dbg("%s chip = %d\n", __func__, chip_nr);
}

/*
 * returns 0 : busy. 1 : ready
 */
static int ait_nand_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;

	if(pNAD->NAD_CTL & NAD_RB)
	{
		return 1;
	}else{
		return 0;
	}
}

static int ait_nand_r_page(struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buf, int page)
{
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;
	unsigned char addr[5] = {0, 0, (page & 0xFF), ((page >> 8) & 0xFF), ((page >> 16) & 0xFF)}, oob_buf[mtd->oobsize];
	unsigned int dma_addr, dma_addr_oob;
	uint8_t *read_buf;
	int i, status;

	if(virt_addr_valid(buf) && virt_addr_valid(buf + mtd->writesize))
	{
		read_buf = buf;
	}else{
		read_buf = info->dma_page_buffer;
	}
	dma_addr = nand_map_memory(mtd, NAND_DMA_READ, (void *)read_buf, mtd->writesize);
	dma_addr_oob = nand_map_memory(mtd, NAND_DMA_READ, (void *)chip->oob_poi, mtd->oobsize);

	nand_cmd(mtd, NAND_CMD_READ0);
	if(chip->chipsize > SZ_128M)
	{
		nand_w_addr(mtd, addr, 5);
	}else{
		nand_w_addr(mtd, addr, 4);
	}
	nand_cmd(mtd,NAND_CMD_READSTART);
	nand_wait_dev_ready(mtd);
	for(i=0;i<mtd->writesize/512;i++)
	{
		pNAD->NAD_DMA_ADDR = (unsigned int)(dma_addr+i*512);
		pNAD->NAD_DMA_LEN = 511;	//512-1=511
		pNAD->NAD_CTL |= NAD_READ;
		pNAD->NAD_RDN_CTL = NAD_RS_EN;
		nand_wait_dma(mtd);
		*(oob_buf+(i<<4)+13) = pNAD->NAD_ECC0;
		*(oob_buf+(i<<4)+14) = pNAD->NAD_ECC1;
		*(oob_buf+(i<<4)+15) = pNAD->NAD_ECC2;
		*(oob_buf+(i<<4)+8) = pNAD->NAD_ECC3;
		*(oob_buf+(i<<4)+9) = pNAD->NAD_ECC4;
		*(oob_buf+(i<<4)+10) = pNAD->NAD_ECC5;
		n_dbg("Sector=%d ECC = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", i, *(oob_buf+(i<<4)+13), \
			*(oob_buf+(i<<4)+14), *(oob_buf+(i<<4)+15), *(oob_buf+(i<<4)+8), *(oob_buf+(i<<4)+9), *(oob_buf+(i<<4)+10));
		pNAD->NAD_RDN_CTL = 0;
	}

	pNAD->NAD_DMA_ADDR = dma_addr_oob;
	pNAD->NAD_DMA_LEN = mtd->oobsize - 1;
	pNAD->NAD_CTL |= NAD_READ;
	pNAD->NAD_RDN_CTL = 0;
	nand_wait_dma(mtd);
	pNAD->NAD_CTL &= (~NAD_READ);
	nand_unmap_memory(mtd, NAND_DMA_READ, dma_addr, mtd->writesize);
	nand_unmap_memory(mtd, NAND_DMA_READ, dma_addr_oob, mtd->oobsize);

	if(!(virt_addr_valid(buf) && virt_addr_valid(buf + mtd->writesize)))
	{
		memcpy(buf, read_buf, mtd->writesize);
	}

	for(i=0;i<mtd->writesize/512;i++)
	{
		if((*(oob_buf+(i<<4)+13) != *(chip->oob_poi+(i<<4)+13)) || \
			(*(oob_buf+(i<<4)+14) != *(chip->oob_poi+(i<<4)+14))|| \
			(*(oob_buf+(i<<4)+15) != *(chip->oob_poi+(i<<4)+15)))
		{
			printk("ECC ERROR in Page 0x%x 1A: C:0x%x 0x%x 0x%x, ", page, *(oob_buf+(i<<4)+13) , *(oob_buf+(i<<4)+14), *(oob_buf+(i<<4)+15));
			printk("S: 0x%x 0x%x 0x%x\n", *(chip->oob_poi+(i<<4)+13), *(chip->oob_poi+(i<<4)+14), *(chip->oob_poi+(i<<4)+15) );
			status = MMPF_NAND_ECCDecode((MMP_UBYTE*)(buf+i*512), (chip->oob_poi+(i<<4)+13), (oob_buf+(i<<4)+13));
			if(status == MMP_NAND_ERR_ECC)
			{
				printk("ECC ERROR over 1 bit in page 1A!!!! = 0x%x\n", page);
				return -EIO;
			}
		}
		if((*(oob_buf+(i<<4)+8) != *(chip->oob_poi+(i<<4)+8)) || \
			(*(oob_buf+(i<<4)+9) != *(chip->oob_poi+(i<<4)+9)) || \
			(*(oob_buf+(i<<4)+10) != *(chip->oob_poi+(i<<4)+10)))
		{
			printk("ECC ERROR in Page 0x%x 2A: C:0x%x 0x%x 0x%x, ", page, *(oob_buf+(i<<4)+8) , *(oob_buf+(i<<4)+9), *(oob_buf+(i<<4)+10));
			printk("S: 0x%x 0x%x 0x%x\n", *(chip->oob_poi+(i<<4)+8), *(chip->oob_poi+(i<<4)+9), *(chip->oob_poi+(i<<4)+10));
			status = MMPF_NAND_ECCDecode((MMP_UBYTE*)(buf+i*512+256), (chip->oob_poi+(i<<4)+8), (oob_buf+(i<<4)+8));
			if(status == MMP_NAND_ERR_ECC)
			{
				printk("ECC ERROR over 1 bit in page 2A!!!! = 0x%x\n", page);
				return -EIO;
			}
		}
	}

	n_dbg("%s buf = 0x%x, page = 0x%x 0x%x 0x%x\n", __func__, (unsigned int)buf, page, *(buf), *(buf+1));

	return 0;
}

static void ait_nand_w_page(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buf)
{
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;
	int page = info->w_page_addr, i;
	unsigned char addr[5] = {0, 0, (page & 0xFF), ((page >> 8) & 0xFF), ((page >> 16) & 0xFF)}, *oob_buf = info->w_buffer;
	uint8_t *write_buf;
	unsigned int dma_addr, dma_addr_oob;

	for(i=0;i<mtd->oobsize/4;i++)
	{
		*(((unsigned int *)oob_buf)+i) = 0xFFFFFFFF;
	}

	if(virt_addr_valid(buf) && virt_addr_valid(buf + mtd->writesize))
	{
		write_buf = (uint8_t *)buf;
	}else{
		memcpy(info->dma_page_buffer, buf, mtd->writesize);
		write_buf = info->dma_page_buffer;
	}
	dma_addr = nand_map_memory(mtd, NAND_DMA_WRITE, (void *)write_buf, mtd->writesize);

	nand_cmd(mtd, NAND_CMD_SEQIN);
	if(chip->chipsize > SZ_128M)
	{
		nand_w_addr(mtd, addr, 5);
	}else{
		nand_w_addr(mtd, addr, 4);
	}
	udelay(10);
	for(i=0;i<mtd->writesize/512;i++)
	{
		pNAD->NAD_DMA_ADDR = (unsigned int)(dma_addr+i*512);
		pNAD->NAD_DMA_LEN = 511;	//512-1=511
		pNAD->NAD_CTL &= (~NAD_READ);
		pNAD->NAD_RDN_CTL = NAD_RS_EN;	//enable ecc
		nand_wait_dma(mtd);
		*(oob_buf+(i<<4)+13) = pNAD->NAD_ECC0;
		*(oob_buf+(i<<4)+14) = pNAD->NAD_ECC1;
		*(oob_buf+(i<<4)+15) = pNAD->NAD_ECC2;
		*(oob_buf+(i<<4)+8) = pNAD->NAD_ECC3;
		*(oob_buf+(i<<4)+9) = pNAD->NAD_ECC4;
		*(oob_buf+(i<<4)+10) = pNAD->NAD_ECC5;
		n_dbg("Sector=%d ECC = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", i, *(oob_buf+(i<<4)+13), \
			*(oob_buf+(i<<4)+14), *(oob_buf+(i<<4)+15), *(oob_buf+(i<<4)+8), *(oob_buf+(i<<4)+9), *(oob_buf+(i<<4)+10));
		pNAD->NAD_RDN_CTL = 0;
	}

	dma_addr_oob = nand_map_memory(mtd, NAND_DMA_WRITE, (void *)oob_buf, mtd->oobsize);
	pNAD->NAD_DMA_ADDR = dma_addr_oob;
	pNAD->NAD_DMA_LEN = mtd->oobsize - 1;
	pNAD->NAD_CTL &= (~NAD_READ);
	pNAD->NAD_RDN_CTL = 0;	//close ecc
	nand_wait_dma(mtd);
	nand_cmd(mtd, NAND_CMD_PAGEPROG);
	nand_unmap_memory(mtd, NAND_DMA_WRITE, dma_addr, mtd->writesize);
	nand_unmap_memory(mtd, NAND_DMA_WRITE, dma_addr_oob, mtd->oobsize);
	
	n_dbg("%s buf = 0x%x, page_addr = 0x%x\n", __func__, (unsigned int)buf, page);
}

static int ait_nand_r_page_raw(struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buf, int page)
{
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;
	unsigned char addr[5] = {0, 0, (page & 0xFF), ((page >> 8) & 0xFF), ((page >> 16) & 0xFF)};
	unsigned int dma_addr;
	uint8_t *read_buf;

	if(virt_addr_valid(buf) && virt_addr_valid(buf + mtd->writesize + mtd->oobsize))
	{
		read_buf = buf;
	}else{
		read_buf = info->dma_page_buffer;
	}
	dma_addr = nand_map_memory(mtd, NAND_DMA_READ, (void *)read_buf, mtd->writesize + mtd->oobsize);

	nand_cmd(mtd, NAND_CMD_READ0);
	if(chip->chipsize > SZ_128M)
	{
		nand_w_addr(mtd, addr, 5);
	}else{
		nand_w_addr(mtd, addr, 4);
	}
	nand_cmd(mtd,NAND_CMD_READSTART);
	nand_wait_dev_ready(mtd);
	pNAD->NAD_DMA_ADDR = (unsigned int)(dma_addr);
	pNAD->NAD_DMA_LEN = (mtd->writesize + mtd->oobsize - 1);
	pNAD->NAD_CTL |= NAD_READ;
	pNAD->NAD_RDN_CTL = 0;
	nand_wait_dma(mtd);
	nand_unmap_memory(mtd, NAND_DMA_READ, dma_addr, mtd->writesize + mtd->oobsize);

	if(!(virt_addr_valid(buf) && virt_addr_valid(buf + mtd->writesize + mtd->oobsize)))
	{
		memcpy(buf, read_buf, mtd->writesize + mtd->oobsize);
	}

	n_dbg("%s buf = 0x%x, page = 0x%x\n", __func__, (unsigned int)buf, page);

	return 0;
}

static void ait_nand_w_page_raw(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buf)
{
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;
	int page = info->w_page_addr;
	unsigned char addr[5] = {0, 0, (page & 0xFF), ((page >> 8) & 0xFF), ((page >> 16) & 0xFF)};
	unsigned int dma_addr;
	uint8_t *write_buf;

	if(virt_addr_valid(buf) && virt_addr_valid(buf + mtd->writesize + mtd->oobsize))
	{
		write_buf = (uint8_t *)buf;
	}else{
		memcpy(info->dma_page_buffer, buf, mtd->writesize + mtd->oobsize);
		write_buf = info->dma_page_buffer;
	}
	dma_addr = nand_map_memory(mtd, NAND_DMA_WRITE, (void *)write_buf, mtd->writesize + mtd->oobsize);

	nand_cmd(mtd, NAND_CMD_SEQIN);
	if(chip->chipsize > SZ_128M)
	{
		nand_w_addr(mtd, addr, 5);
	}else{
		nand_w_addr(mtd, addr, 4);
	}
	udelay(10);
	pNAD->NAD_DMA_ADDR = (unsigned int)(dma_addr);
	pNAD->NAD_DMA_LEN = (mtd->writesize + mtd->oobsize - 1);
	pNAD->NAD_CTL &= (~NAD_READ);
	pNAD->NAD_RDN_CTL = 0;
	nand_wait_dma(mtd);
	nand_cmd(mtd, NAND_CMD_PAGEPROG);
	nand_unmap_memory(mtd, NAND_DMA_WRITE, dma_addr, mtd->writesize + mtd->oobsize);

	n_dbg("%s buf = 0x%x, page_addr = 0x%x\n", __func__, (unsigned int)buf, page);
}

static int ait_nand_r_oob(struct mtd_info *mtd, struct nand_chip *chip, int page, int sndcmd)
{
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;
	unsigned char addr[5] = {(mtd->writesize & 0xFF), ((mtd->writesize >> 8) & 0xFF), (page & 0xFF), ((page >> 8) & 0xFF), ((page >> 16) & 0xFF)};
	unsigned int dma_addr;

	dma_addr = nand_map_memory(mtd, NAND_DMA_READ, (void *)chip->oob_poi, mtd->oobsize);
	nand_cmd(mtd, NAND_CMD_READ0);
	if(chip->chipsize > SZ_128M)
	{
		nand_w_addr(mtd, addr, 5);
	}else{
		nand_w_addr(mtd, addr, 4);
	}
	nand_cmd(mtd,NAND_CMD_READSTART);
	nand_wait_dev_ready(mtd);
	pNAD->NAD_DMA_ADDR = dma_addr;
	pNAD->NAD_DMA_LEN = (mtd->oobsize - 1);
	pNAD->NAD_CTL |= NAD_READ;
	pNAD->NAD_RDN_CTL = 0;
	nand_wait_dma(mtd);
	pNAD->NAD_CTL &= (~NAD_READ);
	nand_unmap_memory(mtd, NAND_DMA_READ, dma_addr, mtd->oobsize);

	n_dbg("%s page = 0x%x, sndcmd = 0x%x oob = 0x%x 0x%x\n", __func__, page, sndcmd, *(chip->oob_poi), *(chip->oob_poi+1));
	return 0;
}

static int ait_nand_w_oob(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	struct ait_nand_info *info = chip->priv;
	AITPS_NAND  pNAD = info->nand_reg;
	unsigned char addr[5] = {(mtd->writesize & 0xFF), ((mtd->writesize >> 8) & 0xFF), (page & 0xFF), ((page >> 8) & 0xFF), ((page >> 16) & 0xFF)};
	unsigned int dma_addr, status = 0; //Set to be 0 for busy status.

	dma_addr = nand_map_memory(mtd, NAND_DMA_WRITE, (void *)chip->oob_poi, mtd->oobsize);
	nand_cmd(mtd, NAND_CMD_SEQIN);
	if(chip->chipsize > SZ_128M)
	{
		nand_w_addr(mtd, addr, 5);
	}else{
		nand_w_addr(mtd, addr, 4);
	}
	udelay(10);
	pNAD->NAD_DMA_ADDR = dma_addr;
	pNAD->NAD_DMA_LEN = (mtd->oobsize - 1);
	pNAD->NAD_CTL &= (~NAD_READ);
	pNAD->NAD_RDN_CTL = 0;
	nand_wait_dma(mtd);
	nand_cmd(mtd, NAND_CMD_PAGEPROG);
	nand_unmap_memory(mtd, NAND_DMA_WRITE, dma_addr, mtd->oobsize);

	while((status & NAND_STATUS_READY) != NAND_STATUS_READY)
	{
		nand_cmd(mtd, NAND_CMD_STATUS);
		status = ait_nand_r_byte(mtd); 
	}
	if((status & NAND_STATUS_FAIL) == NAND_STATUS_FAIL)
	{
		printk("%s ERROR status = 0x%x\n", __func__, status);
		return -EIO;
	}

	n_dbg("%s page = 0x%x oob = 0x%x 0x%x\n", __func__, page, *(chip->oob_poi), *(chip->oob_poi+1));

	return 0;
}

static void nand_pin_time_config(struct ait_nand_info *info)
{
	AITPS_GBL pGBL = AITC_BASE_GBL;
	AITPS_PAD pPAD = AITC_BASE_PAD;
	AITPS_NAND pNAD = info->nand_reg;

	pGBL->GBL_SW_RST_EN[1] |= GBL_RST_SM;
        mdelay(1);
        pGBL->GBL_SW_RST_DIS[1] |= GBL_RST_SM;
	pNAD->NAD_TIMING = (NAD_RCV_CYC(0)) | (NAD_CMD_CYC(3));
	pGBL->GBL_GPIO_CFG[0] &= 0x0000FFFF ;
	pGBL->GBL_SD_PAD_CFG &= ~(GBL_SD1_PAD_MASK);
	pGBL->GBL_MISC_IO_CFG |= GBL_NAND_PAD_EN;
	pGBL->GBL_BOOT_STRAP_CTL &= ~(JTAG_CHAIN_MODE_DIS);
	pPAD->PAD_IO_CFG_PBGPIO[6] = 0x64;
	pPAD->PAD_IO_CFG_PBGPIO[7] = 0x64;
        pPAD->PAD_IO_CFG_PBGPIO[10] = 0x60;
        pPAD->PAD_IO_CFG_PBGPIO[11] = 0x60;
        pPAD->PAD_IO_CFG_PBGPIO[12] = 0x64;
        pPAD->PAD_IO_CFG_PBGPIO[13] = 0x64;
        pPAD->PAD_IO_CFG_PBGPIO[14] = 0x64;
        pPAD->PAD_IO_CFG_PBGPIO[15] = 0x64;
        pPAD->PAD_IO_CFG_PBGPIO2[0] = 0x64;
        pPAD->PAD_IO_CFG_PBGPIO2[1] = 0x64;
        pPAD->PAD_IO_CFG_PBGPIO2[2] = 0x64;
        pPAD->PAD_IO_CFG_PBGPIO2[3] = 0x64;
        pPAD->PAD_IO_CFG_PBGPIO2[4] = 0x64;
        pPAD->PAD_IO_CFG_PBGPIO2[5] = 0x64;

	n_dbg("pGBL->GBL_GPIO_CFG[0] = 0x%x\n", pGBL->GBL_GPIO_CFG[0]);
	n_dbg("pGBL->GBL_PWM_IO_CFG = 0x%x\n", pGBL->GBL_PWM_IO_CFG);
	n_dbg("pGBL->GBL_SPI_PAD_CFG = 0x%x\n", pGBL->GBL_SPI_PAD_CFG);
	n_dbg("pGBL->GBL_UART_PAD_CFG = 0x%x\n", pGBL->GBL_UART_PAD_CFG);
	n_dbg("pGBL->GBL_VIF_IGBT_CFG = 0x%x\n", pGBL->GBL_VIF_IGBT_CFG);
	n_dbg("pGBL->GBL_I2S_DMIC_CFG = 0x%x\n", pGBL->GBL_I2S_DMIC_CFG);
	n_dbg("pGBL->GBL_SD_PAD_CFG = 0x%x\n", pGBL->GBL_SD_PAD_CFG);
	n_dbg("pGBL->GBL_I2CM_PAD_CFG = 0x%x\n", pGBL->GBL_I2CM_PAD_CFG);
	n_dbg("pGBL->GBL_MISC_IO_CFG = 0x%x\n", pGBL->GBL_MISC_IO_CFG);
	n_dbg("pGBL->GBL_JTAG_CFG = 0x%x\n", pGBL->GBL_JTAG_CFG);
	n_dbg("pGBL->GBL_BOOT_STRAP_CTL = 0x%x\n", pGBL->GBL_BOOT_STRAP_CTL);
}

static int __devinit ait_nand_probe(struct platform_device *pdev)
{
	struct ait_nand_info *info;
	struct resource *r;
	void __iomem *nand_reg_map;
	int irq;
	n_dbg("%s +++\n", __func__);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(r == NULL) 
	{
		printk("ERROR: Can't get nand IO mem resource\n");
		return -ENODEV;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if(r == NULL)
	{
		printk("ERROR: Can't request CPU mem resource\n");
		return -EBUSY;
	}

	nand_reg_map = ioremap(r->start, resource_size(r));
	if(nand_reg_map == NULL)
	{
		printk("ERROR: Can't remap io resource\n");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if(irq < 0)
	{
		printk("ERROR: Can'get nand irq\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct ait_nand_info), GFP_KERNEL);
	if(info == NULL)
	{
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, info);
	info->pdev = pdev;
	info->nand_reg = (AITPS_NAND)(nand_reg_map);
	info->mtd.priv = &info->nand;
	info->mtd.name = DEVICE_NAME_NAND;
	info->mtd.owner = THIS_MODULE;
	info->nand.priv = info;
	info->nand.read_byte = ait_nand_r_byte;
	info->nand.read_buf = ait_nand_r_buf;
	info->nand.write_buf = ait_nand_w_buf;
	info->nand.cmd_ctrl = ait_nand_cmd_ctrl;
	info->nand.cmdfunc = ait_nand_cmdfunc;
	info->nand.select_chip = ait_nand_select_chip;
	info->nand.dev_ready = ait_nand_dev_ready;
	info->dma_page_buffer = (unsigned char *)((unsigned int)kmalloc(MAX_PAGE_SIZE + MAX_OOB_SIZE + 0x100, GFP_KERNEL) & ~0x3F);
	if(!info->dma_page_buffer)
	{
		kfree(info);
		return -ENOMEM;
	}
	info->w_buffer = (unsigned char *)((unsigned int)kmalloc(MAX_OOB_SIZE + 0x100, GFP_KERNEL) & ~0x3F);
	if(!info->w_buffer)
	{
		kfree(info->dma_page_buffer);
		kfree(info);
		return -ENOMEM;
	}
	info->nand.options |= NAND_NO_SUBPAGE_WRITE;
        info->nand.ecc.mode = NAND_ECC_HW;
        info->nand.ecc.layout = &nand_oob_64;
        info->nand.ecc.size = 512;
        info->nand.ecc.bytes = 16;
        info->nand.ecc.read_page = ait_nand_r_page;
        info->nand.ecc.write_page = ait_nand_w_page;
        info->nand.ecc.read_page_raw = ait_nand_r_page_raw;
        info->nand.ecc.write_page_raw = ait_nand_w_page_raw;
        info->nand.ecc.read_oob = ait_nand_r_oob;
        info->nand.ecc.write_oob = ait_nand_w_oob;

	info->clk = clk_get(&(info->pdev->dev), "sm_clk");
	if(IS_ERR(info->clk)) 
	{
		printk("ERROR: Can't get nand clock\n");
		kfree(info->w_buffer);
		kfree(info->dma_page_buffer);
		kfree(info);
		return -ENODEV;
	}else{
		clk_enable(info->clk);
	}
	nand_pin_time_config(info);

	if(nand_scan(&info->mtd, 1))
	{
		kfree(info->w_buffer);
		kfree(info->dma_page_buffer);
		kfree(info);
		return -ENOMEM;
	}

	mtd_device_parse_register(&info->mtd, NULL, NULL, ait_nand_partition, ARRAY_SIZE(ait_nand_partition));

	n_dbg("%s ---\n", __func__);

	return 0;
}

static int ait_nand_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ait_nand_driver = {
	.probe	= ait_nand_probe,
	.remove	= ait_nand_remove,
	.driver	={
		.name	= DEVICE_NAME_NAND,
		.owner	= THIS_MODULE,
	},
};

static int __init ait_nand_init(void)
{
	//register nand driver
	return platform_driver_register(&ait_nand_driver);
}

static void __exit ait_nand_exit(void)
{
	platform_driver_unregister(&ait_nand_driver);
}

module_init(ait_nand_init);
module_exit(ait_nand_exit);

