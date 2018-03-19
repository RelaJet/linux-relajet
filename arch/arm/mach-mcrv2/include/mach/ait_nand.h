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
#ifndef __AIT_NAND_H__
#define __AIT_NAND_H__

#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <mach/mmp_reg_nand.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_pad.h>
#include <mach/mmpf_system.h>

#define DEVICE_NAME_NAND		"ait_nand"
#define INVAL_PAGE_ADDR			0xffffffff

struct ait_nand_info {
        struct mtd_info         mtd;
        struct nand_chip        nand;
	AITPS_NAND 		nand_reg;
	struct platform_device	*pdev;
	struct clk		*clk;
        unsigned int            w_page_addr;
        uint8_t                 *dma_page_buffer;
	uint8_t 		*w_buffer;
};

int MMPF_NAND_ECCDecode(unsigned char *p_buf, unsigned char *p_ecc, unsigned char *ecc_data);
int MMPF_NAND_ECCCorrect(unsigned char *p_data, unsigned char *ecc1, unsigned char *ecc2, unsigned char *p_offset, unsigned char *p_corrected);

#endif

