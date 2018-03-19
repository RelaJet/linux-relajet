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
#ifndef __AIT_SPINAND_H__
#define __AIT_SPINAND_H__

/* SPI NAND commands */
#define SPINAND_WRITE_ENABLE            0x06
#define SPINAND_WRITE_DISABLE           0x04
#define SPINAND_GET_FEATURE             0x0f
#define SPINAND_SET_FEATURE             0x1f
#define SPINAND_PAGE_READ               0x13
#define SPINAND_READ_CACHE              0x03
#define SPINAND_FAST_READ_CACHE         0x0b
#define SPINAND_READ_CACHE_X2           0x3b
#define SPINAND_READ_CACHE_X4           0x6b
#define SPINAND_READ_CACHE_DUAL_IO      0xbb
#define SPINAND_READ_CACHE_QUAD_IO      0xeb
#define SPINAND_READ_ID                 0x9f
#define SPINAND_PROGRAM_LOAD            0x02
#define SPINAND_PROGRAM_LOAD4           0x32
#define SPINAND_PROGRAM_EXEC            0x10
#define SPINAND_PROGRAM_LOAD_RANDOM     0x84
#define SPINAND_PROGRAM_LOAD_RANDOM4    0xc4
#define SPINAND_BLOCK_ERASE             0xd8
#define SPINAND_RESET                   0xff
#define INVAL_PAGE_ADDR                 0xffffffff

/* GIGA DEVICE STATUS DEFINE */
#define GIGA_STATUS_A0_BP0              (1 << 3)
#define GIGA_STATUS_A0_BP1              (1 << 4)
#define GIGA_STATUS_A0_BP2              (1 << 5)
#define GIGA_STATUS_A0_BRWD             (1 << 7)
#define GIGA_STATUS_B0_ECC_EN           (1 << 4)
#define GIGA_STATUS_C0_OIP              (1 << 0)
#define GIGA_STATUS_C0_WEL              (1 << 1)
#define GIGA_STATUS_C0_E_F              (1 << 2)
#define GIGA_STATUS_C0_P_F              (1 << 3)
#define GIGA_STATUS_C0_ECCS0            (1 << 4)
#define GIGA_STATUS_C0_ECCS1            (1 << 5)

struct ait_spinand_info {
	struct mtd_info         mtd;
	struct nand_chip        nand;
	struct spi_device       *spi;
	unsigned int            r_index;
	unsigned int            w_page_addr;
	uint8_t                 *dma_page_buffer;
	unsigned int            fast_read;
	unsigned int            write_dummy;
};

#endif
