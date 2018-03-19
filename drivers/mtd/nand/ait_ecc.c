/*
 * (C) Copyright AIT
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <asm/sizes.h>
#include <mach/ait_nand.h>

int MMPF_NAND_ECCDecode(unsigned char *p_buf, unsigned char *p_ecc, unsigned char *ecc_data)
{
	MMP_UBYTE offset, corrected;
	MMP_ERR status = 0;

	if((p_ecc[0] != ecc_data[0]) || (p_ecc[1] != ecc_data[1]) || (p_ecc[2] != ecc_data[2]))
	{
		status = MMPF_NAND_ECCCorrect(p_buf, p_ecc, ecc_data, &offset, &corrected);
		if(status == MMP_NAND_ERR_ECC_CORRECTABLE)
		{
			*((MMP_UBYTE*)p_buf + offset) = corrected;
			printk("ECC Correct: offset = 0x%x, corrected = 0x%x\n", offset, corrected);
			return MMP_NAND_ERR_ECC_CORRECTABLE;
		}else if(status == MMP_NAND_ERR_ECC){
			return MMP_NAND_ERR_ECC;
		}
	}

	return MMP_ERR_NONE;
}

int MMPF_NAND_ECCCorrect(unsigned char *p_data, unsigned char *ecc1, unsigned char *ecc2, unsigned char *p_offset, unsigned char *p_corrected)
{
	int i;
	MMP_UBYTE tmp0_bit[8],tmp1_bit[8],tmp2_bit[8], tmp0, tmp1, tmp2;
	MMP_UBYTE comp0_bit[8],comp1_bit[8],comp2_bit[8];
	MMP_UBYTE  ecc_bit[22];
	MMP_UBYTE ecc_gen[3];
	MMP_UBYTE ecc_sum=0;
	MMP_UBYTE ecc_value,find_byte,find_bit;

	tmp0 = ~ecc1[0];
	tmp1 = ~ecc1[1];
	tmp2 = ~ecc1[2];

	for(i = 0; i <= 2; ++i)
	{
		ecc_gen[i] = ~ecc2[i];
	}

	for(i = 0; i < 7; i++)
	{
		tmp0_bit[i]= tmp0 & 0x01;
		tmp0 >>= 1;
	}

	tmp0_bit[7] = tmp0 & 0x01;

	for(i = 0; i < 7; i++)
	{
		tmp1_bit[i]= tmp1 & 0x01;
		tmp1 >>= 1;
	}

	tmp1_bit[7] = tmp1 & 0x01;

	for(i = 0; i < 7; i++)
	{
		tmp2_bit[i]= tmp2 & 0x01;
		tmp2 >>= 1;
	}
	tmp2_bit[7]= tmp2 & 0x01;

	for(i = 0; i < 7; i++)
	{
		comp0_bit[i]= ecc_gen[0] & 0x01;
		ecc_gen[0] >>= 1;
	}
	comp0_bit[7]= ecc_gen[0] & 0x01;

	for(i = 0; i < 7; i++)
	{
		comp1_bit[i]= ecc_gen[1] & 0x01;
		ecc_gen[1] >>= 1;
	}
	comp1_bit[7]= ecc_gen[1] & 0x01;

	for(i = 0; i < 7; i++)
	{
		comp2_bit[i]= ecc_gen[2] & 0x01;
		ecc_gen[2] >>= 1;
	}
	comp2_bit[7]= ecc_gen[2] & 0x01;

	for(i = 0; i <= 5; ++i)
	{
		ecc_bit[i] = tmp2_bit[i + 2] ^ comp2_bit[i + 2];
	}

	for (i = 0; i <= 7; ++i)
	{
		ecc_bit[i + 6] = tmp0_bit[i] ^ comp0_bit[i];
	}

	for(i = 0; i <= 7; ++i)
	{
		ecc_bit[i + 14] = tmp1_bit[i] ^ comp1_bit[i];
	}

	for(i = 0; i <= 21; ++i)
	{
		ecc_sum += ecc_bit[i];
	}

	if(ecc_sum == 11)
	{
		find_byte = (ecc_bit[21] << 7) + (ecc_bit[19] << 6) + (ecc_bit[17] << 5) + (ecc_bit[15] << 4) + \
			(ecc_bit[13] << 3) + (ecc_bit[11] << 2) + (ecc_bit[9] << 1) + ecc_bit[7];
		find_bit = (ecc_bit[5] << 2) + (ecc_bit[3] << 1) + ecc_bit[1];
		ecc_value = (p_data[find_byte] >> find_bit) & 0x01;
		if(ecc_value == 0)
		{
			ecc_value = 1;
		}else{
			ecc_value = 0;
		}

		*p_offset = find_byte;
		*p_corrected = p_data[find_byte];
		*p_corrected = p_data[find_byte];

		for(i = 0; i < 7; i++)
		{
			tmp0_bit[i] = *p_corrected & 0x01;
			*p_corrected >>= 1;
		}
		tmp0_bit[7] = *p_corrected & 0x01;

		tmp0_bit[find_bit] = ecc_value;

		*p_corrected = (tmp0_bit[7] << 7) + (tmp0_bit[6] << 6) + (tmp0_bit[5] << 5) + (tmp0_bit[4] << 4) + \
				(tmp0_bit[3] << 3) + (tmp0_bit[2] << 2) + (tmp0_bit[1] << 1) + tmp0_bit[0];

		return MMP_NAND_ERR_ECC_CORRECTABLE;
	}else if((ecc_sum == 0) || (ecc_sum == 1)){
		return MMP_ERR_NONE;
	}else{
		return MMP_NAND_ERR_ECC;
	}

	return MMP_ERR_NONE;
}



