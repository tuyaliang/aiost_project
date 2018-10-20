/*
 * arch/arm/mach-milbeaut/snapshot.c
 *
 *  Copyright (C) 2016-2017  Lineo Solutions, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/warp_param.h>
#include <asm/pgtable.h>
#include <asm/sections.h>
#include <asm/smp_twd.h>

#include <linux/syscalls.h> 

#define GPIO_NUM        10
#define UART_NUM        10
#define GPIO_BLACK_LIST_SIZE	(13)
#define RTOS_NF_USE		(0xBEEFCAFE)
#define RTOS_NF_NOT_USE		(0xFEEDBACF)
#define RTOS_NF_MAGIC_OFFSET	(0x40)
//#define CO_TEST
//#define CO_TEST_PRINT
#define CONVERT_INT_BIG_LITTLE(a)	(((a & 0xff)<<24) | ((a & 0xff00)<<8) | ((a & 0xff0000)>>8) |(a >>24))
#define CA7_OFFSET_BLACK_LIST	(3)
#define CLKSEL_NUM	(12)
#define CLK_NUM_AT_SEL	(13)
#define CLKTOP_NUM	(14)
#define CLKTOP_BIT_NUM	(16)
#define PLL_NUM	(9)
#define PLL_BIT_NUM_AT	(19)
#define CLKTOP_OFFSET	(0x54)
#define CLKSEL_OFFSET	(0x00)
#define CLKPLL_OFFSET	(0x30)
#define GPIO_PDR_OFFSET		(0x00c)
#define GPIO_DDR_OFFSET		(0x10c)
#define GPIO_EPCR_OFFSET	(0x20c)
#define GPIO_PUDEDR_OFFSET	(0x30c)
#define GPIO_PUDCR_OFFSET	(0x40c)
#define CLK_CRSWR_OFFSET	(0x8c)
#define CLK_CRRRS_OFFSET	(0x90)
#define CLK_CRRSM_OFFSET	(0x94)

#define KERNEL_UNBOOT_FLAG	0x12345678

extern u32 trampoline_phys;

static unsigned int black_list[GPIO_BLACK_LIST_SIZE];/*Pinctrl's Black-list*/
struct clk_bit {
    unsigned int mask;
    unsigned int wbit;
    unsigned int reg_ofs0;
    unsigned int reg_mask0;
    unsigned int reg_ofs1;
    unsigned int reg_mask1;
    int read_only;
    char dtb_name[100];
};
struct clock_map_bit {
    unsigned int mask;
    int read_only;
    char dtb_name[100];
};
static struct clock_map_bit clk_top_bit[CLKTOP_NUM][CLKTOP_BIT_NUM] = {
    {                           /* CLKSTOP1 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DSPCK */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DSPAX */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SENCK */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SENAX */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SENAH */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SENAP */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* GPIOAP */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* AU0CK */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* AU2CK */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* AU3CK */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* AU4CK */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* AU5CK */
        {
			.mask = 3 <<  24,
#ifdef CO_TEST
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/netauck/read-only",
#else
			.dtb_name ="",
#endif
			.read_only = 1,
		},                         /* NETAUCK */
        {
			.mask = 3 <<  28,
			.dtb_name ="",
			.read_only = 1,
		},                         /* TEMPCK */
        {
			.mask = 0,
#ifdef CO_TEST
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/tempck/read-only",
#else
			.dtb_name ="",
#endif
			.read_only = 1,
		},                         /* Stpper */
	},
    {                           /* CLKSTOP2 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* RCK */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UHS1CK0 */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UHS1CK1 */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UHS1CK2 */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UHS2CK */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* NFCK */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* EMMCCK */
        {
			.mask = 3 <<  14,
#ifdef CO_TEST
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/netsecck/read-only",
#else
			.dtb_name ="",
#endif
			.read_only = 1,
		},                         /* NETSECCK */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* NETRCK */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* EXSAX */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SPICK */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SLIMB00CK */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SLIMB01CK */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SLIMB10CK */
        {
			.mask = 3 <<  28,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SLIMB11CK */
        {
			.mask = 3 <<  30,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PCISUPPCK */
	},
    {                           /* CLKSTOP3 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IIPCK */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IIPAP */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IIPAH */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IIPAX */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LCDCK */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* HIFCK */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MIFCK */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DISPAP */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DISPAH */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DISPAX */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* JPGCK */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* JPGAX */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* JPGAH */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* JPGAP */
        {
			.mask = 3 <<  28,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PDM0CK */
        {
			.mask = 3 <<  30,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PDM1CK */
	},
    {                           /* CLKSTOP4 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* GPUCK */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* GPUAP */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* GPUAH */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* GPUAX */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* FPT0CK */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* FPT0AX */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* FPT0AH */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* FPT0AP */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* FPT1CK */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* FPT1AP */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* FPT1AH */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* FPT1AX */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* APCK0 */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* APCK1 */
        {
			.mask = 3 <<  28,
			.dtb_name ="",
			.read_only = 1,
		},                         /* APCK2 */
        {
			.mask = 3 <<  30,
			.dtb_name ="",
			.read_only = 1,
		},                         /* APCK3 */
	},
    {                           /* CLKSTOP4 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAX0 */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAX1 */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAX2 */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAX3 */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAX4 */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAX5 */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAX6 */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAP0 */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAP1 */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAP2 */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAP3 */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAP4 */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAP5 */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAP6 */
        {
			.mask = 0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* Stopper */
	},
    {                           /* CLKSTOP6 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAH1 */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAH2 */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MICAH3 */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IMGAX */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IMGAH0 */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IMGAH1 */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IMGAH3 */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IMGAP3 */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* REGAP */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* XCHAX */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* XCHAP */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* ELACK */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* ELACX */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* ELACP */
        {
			.mask = 0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* Stpper */
	},
    {                           /* CLKSTOP7 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IPUFDCK */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IPUVISCK */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IPUAX */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IIPAX */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IPUAH */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* RAWCK */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* RAWCX */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* RAWCP */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SHDRCK */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SHDRAX */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SHDRAH */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SHDRAP */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* M0CK */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MECK */
        {
			.mask = 3 <<  28,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MEAX */
        {
			.mask = 3 <<  30,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MEAP */
	},
    {                           /* CLKSTOP8 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* RIBCK */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* RIBAH */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* HEVDFCK */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* HEPXFCK */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* HEIPPCK */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0HEVCMX2 */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0HEVCMX4 */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0RBRMX4 */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1HEVCMX2 */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1HEVCMX4 */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1RBRMX4 */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0CMNAX */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0MX1AX */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0MX2AX */
        {
			.mask = 3 <<  28,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0MX3AX */
        {
			.mask = 3 <<  30,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0MX4AX */
	},
    {                           /* CLKSTOP9 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0MX5AX */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1CMNAX */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1MX1AX */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1MX2AX */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1MX3AX */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1MX4AX */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1MX5AX */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0MX0AX3 */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0MX6AX4 */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0HEVCAX4 */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1MX0AX3 */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1MX6AX4 */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1HEVCAX4 */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC0AP */
        {
			.mask = 3 <<  28,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMC1AP */
        {
			.mask = 0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* Stopper */
	},
    {                           /* CLKSTOP10 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO1CK */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO1CK_2 */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO1AX */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO1AH */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO1AP */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2B1CK */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2B1AX */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2B1AH */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2B1AP */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LTMRBK1CK */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2R1CK */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2R1AX */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2R1AH */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2R1AP */
        {
			.mask = 0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* STOPPER */
	},
    {                           /* CLKSTOP11 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LTM1CK */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LTM1AX */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LTM1AH */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LTM1AP */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* R2Y1CK */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* R2Y1AX */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* R2Y1AH */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* R2Y1AP */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* CNR1CK */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* CNR1AX */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* CNR1AP */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* APAH */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DBGAP */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* NFBCHCK */
        {
			.mask = 0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* STOPPER */
	},
    {                           /* CLKSTOP12 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO2CK */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO2CK_2 */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO2AX */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO2AH */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO2AP */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2B2CK */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2B2AX */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2B2AH */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2B2AP */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LTMRBK2CK */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2R2CK */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2R2AX */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2R2AH */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* B2R2AP */
        {
			.mask = 0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* STOPPER */
	},
    {                           /* CLKSTOP13 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LTM2CK */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LTM2AX */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LTM2AH */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* LTM2AP */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* R2Y2CK */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* R2Y2AX */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* R2Y2AH */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* R2Y2AP */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* CNR2CK */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* CNR2AX */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* CNR2AP */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMCVDFMX4 */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMCPXFMX4 */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMCVDFMX2 */
        {
			.mask = 3 <<  28,
			.dtb_name ="",
			.read_only = 1,
		},                         /* UMCPXFMX2 */
        {
			.mask = 3 <<  30,
			.dtb_name ="",
			.read_only = 1,
		},                         /* BMH1CK */
	},
    {                           /* CLKSTOP14 */
        {
			.mask = 3 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* STATAX */
        {
			.mask = 3 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* STATAH */
        {
			.mask = 3 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* STATAP */
        {
			.mask = 3 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PERIAH */
        {
			.mask = 3 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PERIAP */
        {
			.mask = 3 <<  10,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SENMSKCK */
        {
			.mask = 3 <<  12,
			.dtb_name ="",
			.read_only = 1,
		},                         /* GYROCK */
        {
			.mask = 3 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* EXSAH */
        {
			.mask = 3 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* EXSAP */
        {
			.mask = 3 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PASCK */
        {
			.mask = 3 <<  20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* BMH0CK */
        {
			.mask = 3 <<  22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* BMH0AX */
        {
			.mask = 3 <<  24,
			.dtb_name ="",
			.read_only = 1,
		},                         /* RDMAAX */
        {
			.mask = 3 <<  26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* RDMAAP */
        {
			.mask = 3 <<  28,
			.dtb_name ="",
			.read_only = 1,
		},                         /* BMH1AX */
        {
			.mask = 0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* Stopper */
	},
};
static struct clock_map_bit pll_bit[PLL_NUM][PLL_BIT_NUM_AT] = {
    {                           /* PLLCNT1 */
        {
			.mask = 1 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL00ST */

        {
			.mask = 1 <<  1,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pll1/read-only",
			.read_only = 1,
		},                         /* PL01ST */
        {
			.mask = 1 <<  2,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pll2/read-only",
			.read_only = 1,
		},                         /* PL02ST */
        {
			.mask = 1 <<  3,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL03ST */
        {
			.mask = 1 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL04ST */
        {
			.mask = 1 <<  5,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL05ST */
        {
			.mask = 1 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL05AST */
        {
			.mask = 1 <<  7,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pll6/read-only",
			.read_only = 1,
		},                         /* PL06ST */
        {
			.mask = 1 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL07ST */
        {
			.mask = 1 <<  9,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL08ST */
        {
			.mask = 1 <<  10,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pll10/read-only",
			.read_only = 1,
		},                         /* PL10ST */
        {
			.mask = 1 <<  11,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL10AST */
        {
			.mask = 1 <<  12,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pll11/read-only",
			.read_only = 1,
		},                         /* PL10AST */
        {
			.mask = 1 <<  13,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DPL00ST */
        {
			.mask = 1 <<  14,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DPL01ST */
        {
			.mask = 1 <<  15,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DPL02ST */
        {
			.mask = 1 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DPL10ST */
        {
			.mask = 1 <<  17,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DPL11ST */
        {
			.mask = 1 <<  18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DPL12ST */
    },
    {                           /* PLLCNT2 */
        {
			.mask = 1 <<  0,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL00SEL */

        {
			.mask = 1 <<  1,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pll1/read-only",
			.read_only = 1,
		},                         /* PL01SEL */
        {
			.mask = 1 <<  2,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pll2/read-only",
			.read_only = 1,
		},                         /* PL02SEL */
        {
			.mask = 1 <<  3,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL03SEL */
        {
			.mask = 1 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL04SEL */
        {
			.mask = 1 <<  5,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL05SEL */
        {
			.mask = 1 <<  6,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL05ASEL */
        {
			.mask = 1 <<  7,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pll6/read-only",
			.read_only = 1,
		},                         /* PL06SEL */
        {
			.mask = 1 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL07SEL */
        {
			.mask = 1 <<  9,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL08SEL */
        {
			.mask = 1 <<  10,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pll10/read-only",
			.read_only = 1,
		},                         /* PL10SEL */
        {
			.mask = 1 <<  11,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL10ASEL */
        {
			.mask = 1 <<  12,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pll11/read-only",
			.read_only = 1,
		},                         /* PL10ASEL */
        {
			.mask = 1 <<  16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL01SSEN */
        {
			.mask = 1 <<  17,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PL02SSEN */
        {
			.mask = 0,
		},                         /* Nothing */

	},
	{	/* PLLCNT3 */
        {
			.mask = 0xff ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P00POSTDIV */
        {
			.mask = 3 << 8 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P00POSTDIV */
        {
			.mask = 0xff << 16 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P00PLLDIV */
        {
			.mask = 0,
		},                         /* Nothing */
	},
	{	/* PLLCNT4 */
        {
			.mask = 0xffffffff ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P00FNUM */
        {
			.mask = 0,
		},                         /* Nothing */
	},
	{	/* PLLCNT5 */
        {
			.mask = 0xffffffff ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P00FDEN */
        {
			.mask = 0,
		},                         /* Nothing */
	},
	{	/* PLLCNT6 */
        {
			.mask = 0x3ff ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P01RATE */
        {
			.mask = 3 << 10 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P01FREQ */
        {
			.mask = 1 << 12 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P01MODE */
        {
			.mask = 0x3ff << 16 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P02RATE */
        {
			.mask = 3 << 26 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P02FREQ */
        {
			.mask = 1 << 28 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P02MODE */
        {
			.mask = 0,
		},                         /* Nothing */
	},
	{	/* PLLCNT7 */
        {
			.mask = 0x0f ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P03POSTDIV0 */
        {
			.mask = 0xf0 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P03POSTDIV1 */
        {
			.mask = 7 << 8 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P03PREDIV */
        {
			.mask = 0xff << 16 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P03PLLDIV */
        {
			.mask = 1 << 24 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P03OC0 */
        {
			.mask = 1 << 25 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P03OC1 */
        {
			.mask = 1 << 26 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* D0XOC */
        {
			.mask = 1 << 27 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* D1XOC */
        {
			.mask = 1 << 28 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DPLX8 */
        {
			.mask = 1 << 29 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DSEL */
        {
			.mask = 0,
		},                         /* Nothing */
	},
	{	/* PLLCNT8 */
        {
			.mask = 0xff ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P04POSTDIV */
        {
			.mask = 7 << 8 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P04PREDIV */
        {
			.mask = 0xff << 16 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P04PLLDIV */
        {
			.mask = 0,
		},                         /* Nothing */
	},
	{	/* PLLCNT9 */
        {
			.mask = 3 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P10APLLDIV */
        {
			.mask = 0x7f << 8 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P08PLLDIV */
        {
			.mask = 1 << 16 ,
			.dtb_name ="",
			.read_only = 1,
		},                         /* P05CHG */
        {
			.mask = 0,
		},                         /* Nothing */
	},
};

static struct clk_bit clk_bit[CLKSEL_NUM][CLK_NUM_AT_SEL] = {
    {                           /* CLKSEL1 */
        {
			.mask = 0x003 <<  0,
			.wbit = 1 <<  2,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/rclk/read-only",
			.read_only = 1,
		},                         /* RCLK */
        {
			.mask = 0x00f <<  3,
			.wbit = 1 <<  7,
			.reg_ofs0 = 0x58,
			.reg_mask0 = 3 << 2,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/uhs1clk0/read-only",
			.read_only = 1,
		},           /* UHS1CLK0 */
        {
			.mask = 0x00f <<  8, 
			.wbit = 1 << 12,
			.reg_ofs0 = 0x58,
			.reg_mask0 = 3 << 4,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/uhs1clk1/read-only",
			.read_only = 1,
		},           /* UHS1CLK1 */
        {
			.mask = 0x00f << 13,
			.wbit = 1 << 17,
			.reg_ofs0 = 0x58,
			.reg_mask0 = 3 << 6,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/uhs1clk2/read-only",
			.read_only = 1,
		},           /* UHS1CLK2 */
        {
			.mask = 0x007 << 18,
			.wbit = 1 << 21,
			.reg_ofs0 = 0x58,
			.reg_mask0 = 3 << 8,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/uhs2clk/read-only",
			.read_only = 1,
		},           /* UHS2CLK */
        {
			.mask = 0x01f << 22,
			.wbit = 1 << 27,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/nfclk/read-only",
			.read_only = 1,
		},                         /* NFCLK */
        {
			.mask = 0x003 << 28,
			.wbit = 1 << 30,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/emmcclk/read-only",
			.read_only = 1,
		},                         /* EMMCCLK */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL2 */
        {
			.mask = 0x007 <<  0,
			.wbit = 1 <<  3,
			.dtb_name ="",
			.read_only = 1,
		},                         /* ELACLK */
        {
			.mask = 0x007 <<  4,
			.wbit = 1 <<  7,
			.dtb_name ="",
			.read_only = 1,
		},                         /* JPGCLK */
        {
			.mask = 0x007 <<  8,
			.wbit = 1 << 11,
			.dtb_name ="",
			.read_only = 1,
		},                         /* GPUCLK */
        {
			.mask = 0x007 << 12,
			.wbit = 1 << 15,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IPUTMECLK */
        {
			.mask = 0x007 << 16,
			.wbit = 1 << 19,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IPUCLK */
        {
			.mask = 0x007 << 20,
			.wbit = 1 << 23,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MIFCLK */
        {
			.mask = 0x007 << 24,
			.wbit = 1 << 27,
			.dtb_name ="",
			.read_only = 1,
		},                         /* HIFCLK */
        {
			.mask = 0x003 << 28,
			.wbit = 1 << 30,
			.dtb_name ="",
			.read_only = 1,
		},                         /* RAWCLK */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL3 */
        {
			.mask = 0x007 <<  0,
			.wbit = 1 <<  3, 
			.reg_ofs0 = 0x70,
			.reg_mask0 = 3 << 4,
			.dtb_name ="",
			.read_only = 1,
		},           /* VDFCLK */
        {
			.mask = 0x007 <<  4,
			.wbit = 1 <<  7,
			.reg_ofs0 = 0x70,
			.reg_mask0 = 3 << 6,
			.dtb_name ="",
			.read_only = 1,
		},           /* PXFCLK */
        {
			.mask = 0x00f <<  8,
			.wbit = 1 << 12,
			.reg_ofs0 = 0x70,
			.reg_mask0 = 3 << 8,
			.dtb_name ="",
			.read_only = 1,
		},           /* IPPCLK */
        {
			.mask = 0x007 << 13,
			.wbit = 1 << 16,
			.dtb_name ="",
			.read_only = 1,
		},                         /* PASCLK */
        {
			.mask = 0x007 << 17,
			.wbit = 1 << 20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IIPCLK */
        {
			.mask = 0x001 << 21,
			.wbit = 1 << 22,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SENMSKCLK */
        {
			.mask = 0x007 << 23,
			.wbit = 1 << 26,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SENCLK */
        {
			.mask = 0x001 << 27,
			.wbit = 1 << 28,
			.dtb_name ="",
			.read_only = 1,
		},                         /* IPPESEL */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL4 */
        {
			.mask = 0x03f <<  0,
			.wbit = 1 <<  6,                          /* CNR1CLK */
            .reg_ofs0 = 0x7c,
            .reg_mask0 = (3 << 8) | (3 << 16),
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x03f <<  7,
			.wbit = 1 << 13,                          /* B2R1CLK */
            .reg_ofs0 = 0x78,
            .reg_mask0 = (3 << 0) | (3 << 2) | (3 << 10) | (3 << 20),
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x03f << 14,
			.wbit = 1 << 20,                          /* LTM1CLK */
            .reg_ofs0 = 0x78, 
            .reg_mask0 = (3 << 0) | (3 << 2) | (3 << 10) | (3 << 20),
            .reg_ofs1 = 0x7c,
            .reg_mask1 = 3 << 0,
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x03f << 21,
			.wbit = 1 << 27,
			.dtb_name ="",
			.read_only = 1,
		},                         /* R2Y1CLK */
        {
			.mask = 0x007 << 28,
			.wbit = 1 << 31,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SRO1CLK_2 */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL5 */
        {
			.mask = 0x01f <<  0,
			.wbit = 1 <<  5,                          /* LTMRBK1CLK */
            .reg_ofs0 = 0x78,
            .reg_mask0 = (3 << 0) | (3 << 2),
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x01f <<  6,
			.wbit = 1 << 11,                          /* B2B1CLK */
            .reg_ofs0 = 0x78,
            .reg_mask0 = (3 << 0) | (3 << 2) | (3 << 10),
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x00f << 12,
			.wbit = 1 << 16,                          /* SRO1CLK */
            .reg_ofs0 = CLKTOP_OFFSET,
            .reg_mask0 = 3 << 4,
            .reg_ofs1 = 0x78,
            .reg_mask1 = 3 << 0,
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask=0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL6 */
        {
			.mask = 0x03f <<  0,
			.wbit = 1 <<  6,                          /* CNR2CLK */
            .reg_ofs0 = 0x84,
            .reg_mask0 = (3 << 8) | (3 << 16),
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x03f <<  7,
			.wbit = 1 << 13,                          /* B2R2CLK */
            .reg_ofs0 = 0x80,
            .reg_mask0 = (3 << 0) | (3 << 2) | (3 << 10) | (3 << 20),
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x03f << 14,
			.wbit = 1 << 20,                          /* LTM2CLK */
            .reg_ofs0 = 0x80, 
            .reg_mask0 = (3 << 0) | (3 << 2) | (3 << 10) | (3 << 20),
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x03f << 21,
			.wbit = 1 << 27,
			.reg_ofs0 =  0x84,
			.reg_mask0 = 3 << 8,
			.dtb_name ="",
			.read_only = 1,
		},           /* R2Y2CLK */
        {
			.mask = 0x007 << 28,
			.wbit = 1 << 31,
			.reg_ofs0 = 0x80,
			.reg_mask0 = 3 << 2,
			.dtb_name ="",
			.read_only = 1,
		},           /* SRO2CLK_2 */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL7 */
        {
			.mask = 0x01f <<  0,
			.wbit = 1 <<  5,                          /* LTMRBK2CLK */
            .reg_ofs0 = 0x80,
            .reg_mask0 = (3 << 0) | (3 << 2),
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x01f <<  6,
			.wbit = 1 << 11,                          /* B2B2CLK */
            .reg_ofs0 = 0x80,
            .reg_mask0 = (3 << 0) | (3 << 2) | (3 << 10),
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x00f << 12,
			.wbit = 1 << 16,
			.reg_ofs0 = 0x80,
			.reg_mask0 = 3 << 0,
			.dtb_name ="",
			.read_only = 1,
		},           /* SRO2CLK */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL8 */
        {
			.mask = 0x003 <<  0,
			.wbit = 1 <<  2,
			.dtb_name ="",
			.read_only = 1,
		},                         /* DSPCLK */
        {
			.mask = 0x003 <<  3,
			.wbit = 1 <<  5,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/spiclk/read-only",
			.read_only = 1,
		},                         /* SPICLK */
        {
			.mask = 0x003 <<  6,
			.wbit = 1 <<  8,
			.dtb_name ="",
			.read_only = 1,
		},                         /* AUCLK */
        {
			.mask = 0x001 <<  9,
			.wbit = 1 << 10,
			.reg_ofs0 = CLKTOP_OFFSET,
			.reg_mask0 = 3 << 14,
			.dtb_name ="",
			.read_only = 1,
		},          /* AU0CLK */
        {
			.mask = 0x001 << 11,
			.wbit = 1 << 12,
			.reg_ofs0 = CLKTOP_OFFSET,
			.reg_mask0 = 3 << 16,
			.dtb_name ="",
			.read_only = 1,
		},          /* AU2CLK */
        {
			.mask = 0x001 << 13,
			.wbit = 1 << 14,
			.reg_ofs0 = CLKTOP_OFFSET,
			.reg_mask0 = 3 << 18,
			.dtb_name ="",
			.read_only = 1,
		},          /* AU3CLK */
        {
			.mask = 0x001 << 15,
			.wbit = 1 << 16,
			.reg_ofs0 = CLKTOP_OFFSET,
			.reg_mask0 = 3 << 24,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/clk5/read-only",
			.read_only = 1,
		},          /* NETAUSEL */
        {
			.mask = 0x01f << 17,
			.wbit = 1 << 22,                          /* APCLK */
            .reg_ofs0 = 0x60,
            .reg_mask0 = (3 << 24) | (3 << 26) | (3 << 28) | (3 << 30),
			.dtb_name ="",
			.read_only = 1,
        },
        {
			.mask = 0x001 << 23,
			.wbit = 1 << 24,
			.reg_ofs0 = 0x60,
			.reg_mask0 =  3 << 24,
			.dtb_name ="",
			.read_only = 1,
		},          /* AP0SEL */
        {
			.mask = 0x001 << 25,
			.wbit = 1 << 26,
			.reg_ofs0 =  0x60,
			.reg_mask0 = 3 << 26,
			.dtb_name ="",
			.read_only = 1,
		},          /* AP1SEL */
        {
			.mask = 0x001 << 27,
			.wbit = 1 << 28,
			.reg_ofs0 = 0x60,
			.reg_mask0 = 3 << 28,
			.dtb_name ="",
			.read_only = 1,
		},          /* AP2SEL */
        {
			.mask = 0x001 << 29,
			.wbit = 1 << 30,
			.reg_ofs0 = 0x60,
			.reg_mask0 = 3 << 30,
			.dtb_name ="",
			.read_only = 1,
		},          /* AP3SEL */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL9 */
        {
			.mask = 0x03f <<  0,
			.wbit = 1 <<  6,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/pclk/read-only",
			.read_only = 1,
		},                         /* PCLK */
        {
			.mask = 0x00f <<  7,
			.wbit = 1 << 11,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/hclk/read-only",
			.read_only = 1,
		},                         /* HCLK */
        {
			.mask = 0x007 << 12,
			.wbit = 1 << 15,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/hclkbmh/read-only",
			.read_only = 1,
		},                         /* HCLKBMH */
        {
			.mask = 0x007 << 16,
			.wbit = 1 << 19,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/aclkexs/read-only",
			.read_only = 1,
		},                         /* ACLKEXS */
        {
			.mask = 0x007 << 20,
			.wbit = 1 << 23,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/aclk/read-only",
			.read_only = 1,
		},                         /* ACLK */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL10 */
        {
			.mask = 0x003 <<  0,
			.wbit = 1 <<  2,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/aclk400/read-only",
			.read_only = 1,
		},                         /* ACLK400 */
        {
			.mask = 0x007 <<  3,
			.wbit = 1 <<  6,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/mclk200/read-only",
			.read_only = 1,
		},                         /* MCLK200 */
        {
			.mask = 0x003 <<  7,
			.wbit = 1 <<  9,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/mclk400/read-only",
			.read_only = 1,
		},                         /* MCLK400 */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL11 */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
    {                           /* CLKSEL12 */
        {
			.mask = 0x001 <<  0,
			.wbit = 1 <<  1,
			.dtb_name ="/proc/device-tree/mlb01-clk-tree@/clocks/aclk300/read-only",
			.read_only = 1,
		},                         /* ACLK300 */
        {
			.mask = 0x003 <<  2,
			.wbit = 1 <<  4,
			.dtb_name ="",
			.read_only = 1,
		},                         /* GYROCLK */
        {
			.mask = 0x007 <<  8,
			.wbit = 1 << 11,
			.dtb_name ="",
			.read_only = 1,
		},                         /* FPT0CLK */
        {
			.mask = 0x007 << 12,
			.wbit = 1 << 15,
			.dtb_name ="",
			.read_only = 1,
		},                         /* FPT1CLK */
        {
			.mask = 0x003 << 16,
			.wbit = 1 << 18,
			.dtb_name ="",
			.read_only = 1,
		},                         /* MECLK */
        {
			.mask = 0x001 << 19,
			.wbit = 1 << 20,
			.dtb_name ="",
			.read_only = 1,
		},                         /* NFBCHCLK */
        {
			.mask = 0x00f << 21,
			.wbit = 1 << 25,
			.dtb_name ="",
			.read_only = 1,
		},                         /* RIBCLK */
        {
			.mask = 0x007 << 26,
			.wbit = 1 << 29,
			.dtb_name ="",
			.read_only = 1,
		},                         /* SHDRCLK */
        {
			.mask = 0,
			.wbit = 0,
			.dtb_name ="",
			.read_only = 1,
		},
    },
};

static unsigned char ControlMagicNumNF_from_dev[16];
static unsigned int ControlMagicNumAddr;
static void __iomem *ControlMagicNumNF;

#ifdef CONFIG_PM_WARP_DEBUG

#define UART_BASE              0x1e700010       /* UART1 */

static void *uart_base;

static void warp_putchar (char c)
{
    while (!(*((volatile unsigned char *)(uart_base + 0x03)) & 0x01))
        ;

    *((volatile unsigned short *)(uart_base + 0x04)) = c;
}

#endif

#ifndef WARP_HIBDRV_FIXED

extern struct warp_drv_info warp_drv_info[];

static char *pmd_sav;

int set_mem_exec(void)
{
    int sec;
    pmd_t *pmd;
    unsigned long addr = round_up((unsigned long)__init_end, SECTION_SIZE);

    if (warp_drv_info[0].mode != WARP_DRV_FLOATING)
        return 0;

    pmd_sav = kzalloc((1 << (32 - SECTION_SHIFT)), GFP_KERNEL);
    if (pmd_sav == "")
        return -ENOMEM;

    pmd = pmd_offset(pud_offset(pgd_offset(current->mm, 0), 0), 0);
    for (sec = addr >> SECTION_SHIFT;
         sec <= 0x100000000UL >> SECTION_SHIFT; sec++) {
        if ((pmd_val(pmd[sec]) & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
            if (pmd_val(pmd[sec]) & PMD_SECT_XN) {
                pmd_val(pmd[sec]) &= ~PMD_SECT_XN;
                pmd_sav[sec] = 1;
                flush_pmd_entry(&pmd[sec]);
            }
        }
    }
    flush_tlb_kernel_range(addr, 0xffffffff);

    return 0;
}

void restore_mem_exec(void)
{
    int sec;
    pmd_t *pmd;
    unsigned long addr = round_up((unsigned long)__init_end, SECTION_SIZE);

    if (warp_drv_info[0].mode != WARP_DRV_FLOATING)
        return;

    pmd = pmd_offset(pud_offset(pgd_offset(current->mm, 0), 0), 0);
    for (sec = addr >> SECTION_SHIFT;
         sec <= 0x100000000UL >> SECTION_SHIFT; sec++) {
        if ((pmd_val(pmd[sec]) & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
            if (pmd_sav[sec]) {
                pmd_val(pmd[sec]) |= PMD_SECT_XN;
                flush_pmd_entry(&pmd[sec]);
            }
        }
    }
    flush_tlb_kernel_range(addr, 0xffffffff);
    kfree(pmd_sav);
}

#endif  /* WARP_HIBDRV_FIXED */
/*
 * Check the file whether it exist
*/
static int file_exist(const char* file_name)
{
	int ret = 0;
	int fd;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(file_name, O_RDONLY, 0);
	if (fd >= 0) {
		sys_close(fd);
		/* For test*/
#ifdef CO_TEST
		printk("%s exist\n",file_name);
#endif
		ret = 1;
	}
	else {
		/* For test*/
#ifdef CO_TEST
		printk("%s not exist\n",file_name);
#endif
	}
	set_fs(old_fs);	
	return ret;
}
/*
 * Read the file to buffer
*/
static int get_data_from_file(const char* file_name, 
							char* buf, 
							int buf_size)
{
	int ret = 0;
	int fd;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(file_name, O_RDONLY, 0);
	if (fd >= 0) {
		sys_read(fd, buf, buf_size);
		sys_close(fd);
	}
	else {
		ret =-1;
	}
	set_fs(old_fs);	
	return ret;
}

static int warp_drv_init(void)
{
	int i,j;
	void *buff_addr;
#ifdef CONFIG_PM_WARP_DEBUG
    if ((uart_base = ioremap(UART_BASE, 0x10)) == "")
        return -ENOMEM;
#endif
	/* Get IPCU NF use address */
    get_data_from_file("/proc/device-tree/snrtos0@0/reg", (char*)ControlMagicNumNF_from_dev,sizeof(ControlMagicNumNF_from_dev));

	// make control magic num addr
    ControlMagicNumAddr = (ControlMagicNumNF_from_dev[8] << 24) + (ControlMagicNumNF_from_dev[9] << 16) + (ControlMagicNumNF_from_dev[10] << 8) + (ControlMagicNumNF_from_dev[11]);

    buff_addr = ioremap_nocache(ControlMagicNumAddr, 0x4);

	/* NF can be used by RTOS*/
    ControlMagicNumNF = ioremap_nocache(ioread32(buff_addr) + RTOS_NF_MAGIC_OFFSET, 0x1000);

	/* Get GPIO's black-list */
    get_data_from_file("/proc/device-tree/pinctrl@1d022000/blacklist", (char*)black_list,sizeof(black_list));
    for(i = 0; i < GPIO_BLACK_LIST_SIZE; i++) {
		black_list[i] = CONVERT_INT_BIG_LITTLE(black_list[i]);
		/* For test*/
#ifdef CO_TEST
		printk("black_list[%d]:%08X\n",i,black_list[i]);
#endif
	}
	/* Get CLKSEL's reaonly property from DTB*/
    for (i = 0; i < CLKSEL_NUM; i++) {
        for (j = 0; j < CLK_NUM_AT_SEL; j++) {
			if( clk_bit[i][j].mask == 0 ) {
				break;
			}
			if(clk_bit[i][j].dtb_name[0] != 0) {
				clk_bit[i][j].read_only = file_exist(clk_bit[i][j].dtb_name);
			}
		}
	}
	/* Get PLL's reaonly property from DTB*/
    for (i = 0; i < PLL_NUM; i++) {
        for (j = 0; j < PLL_BIT_NUM_AT; j++) {
			if( pll_bit[i][j].mask == 0 ) {
				break;
			}
			if(pll_bit[i][j].dtb_name[0] != 0) {
				pll_bit[i][j].read_only = file_exist(pll_bit[i][j].dtb_name);
			}
		}
	}
	/* Get CLKTOP's reaonly property from DTB*/
    for (i = 0; i < CLKTOP_NUM; i++) {
        for (j = 0; j < CLKTOP_BIT_NUM; j++) {
			if( clk_top_bit[i][j].mask == 0 ) {
				break;
			}
			if(clk_top_bit[i][j].dtb_name[0] != 0) {
				clk_top_bit[i][j].read_only = file_exist(clk_top_bit[i][j].dtb_name);
			}
		}
	}
    return 0;
}

static void warp_drv_uninit(void)
{
#ifdef CONFIG_PM_WARP_DEBUG
    iounmap(uart_base);
#endif
}
#ifdef CO_TEST /* for test */
/* Test PLLCNT */
static void test_pllcnt(int i,
						unsigned int clk_pllcntl[PLL_NUM],
						void __iomem *clk_base
						)
{
	unsigned int change = clk_pllcntl[i]; 
	printk("clk_pllcntl[%d]: %08X\n",i,clk_pllcntl[i]);
	switch( i ) {
		case 0:/* test PLLCNT1 DPL10ST read-only*/
			change &= 0xFFFEFFFF;
			change |= 0x00010000;
			writel(change, clk_base + CLKPLL_OFFSET + i * 4);
			printk(" Change[%d]: %08X\n",i,readl(clk_base + CLKPLL_OFFSET + i * 4));

			break;
		case 1:/* test PLLCNT2 PL10SEL,PL11SEL no read-only*/
			change &= 0xFFFFEBFF;
			writel(change, clk_base + CLKPLL_OFFSET + i * 4);
			printk(" Change[%d]: %08X\n",i,readl(clk_base + CLKPLL_OFFSET + i * 4));
			break;
	}
}
/* Test CLKSEL */
static void test_clksel(int i,
						unsigned int clk_sel[CLKSEL_NUM],
						void __iomem *clk_base
						)
{
	unsigned int change = clk_sel[i]; 
	printk("clk_sel[%d]: %08X\n",i,clk_sel[i]);
	switch( i ) {
		case 0:
			/* Test EMMCCLK Readonly */
			change &= 0x0FFFFFFF;
			change |= 0x50000000;
			writel(change, clk_base + CLKSEL_OFFSET + i * 4);
			printk(" Change[%d]: %08X\n",i,readl(clk_base + CLKSEL_OFFSET + i * 4));
			break;
		case 3:
		{
			/* Test SRO1CLK_2 No exist at DTS*/
			change &= 0x0FFFFFFF;
			change |= 0xA0000000;
			writel(change, clk_base + CLKSEL_OFFSET + i * 4);
			printk(" Change[%d]: %08X\n",i,readl(clk_base + CLKSEL_OFFSET + i * 4));
			break;
		}
		case 7:
		{
			/* Test NETAUSEL: Not ReadOnly */
			change &= 0xFFFE7FFF;
			change |= 0x00018000;
			writel(change, clk_base + CLKSEL_OFFSET + i * 4);
			printk(" Change[%d]: %08X\n",i,readl(clk_base + CLKSEL_OFFSET + i * 4));
			break;
		}
	}
}
/* Test GPIO*/
static void test_gpio( int i, 
					unsigned int gpio_pdr[GPIO_NUM], 
					unsigned int gpio_ddr[GPIO_NUM], 
					unsigned int gpio_epcr[GPIO_NUM],
					unsigned int gpio_pudedr[GPIO_NUM], 
					unsigned int gpio_pudcr[GPIO_NUM], 
					void __iomem  *gpio_base)
{
	unsigned int changed = gpio_pdr[i];
	switch(i) {
		case 1: /*Test P86 registered at blacklist*/
			/* Test PDR */
			changed &= 0xFFFFFFBF;
			changed |= 0x00400000;
			printk("gpio_pdr[%d]:%08X, changed:%08X\n",i,gpio_pdr[i],changed);
		    writel(changed, gpio_base + GPIO_PDR_OFFSET + i * 4);
			printk("changed:%08X, Read:%08X\n",changed,readl(gpio_base + GPIO_PDR_OFFSET + i * 4));
			/* Test EPCR */
			changed = gpio_epcr[i];
			changed &= 0xFFFFFFBF;
			changed |= (~0xFFFFFFBF);
			printk("gpio_epcr[%d]:%08X, changed:%08X\n",i,gpio_epcr[i],changed);
		    writel(changed, gpio_base + GPIO_EPCR_OFFSET + i * 4);
			printk("changed:%08X, Read:%08X\n",changed,readl(gpio_base + GPIO_EPCR_OFFSET + i * 4));
			/* Test DDR */
			changed = gpio_ddr[i];
			changed &= 0xFFFFFFBF;
			printk("gpio_ddr[%d]:%08X, changed:%08X\n",i,gpio_ddr[i],changed);
		    writel(changed, gpio_base + GPIO_DDR_OFFSET + i * 4);
			printk("changed:%08X, Read:%08X\n",changed,readl(gpio_base + GPIO_DDR_OFFSET + i * 4));
			/* Test PUDEDR */
			changed = gpio_pudedr[i];
			changed &= 0xFFFFFFBF;
			changed |= (~0xFFFFFFBF);
			printk("gpio_pudedr[%d]:%08X, changed:%08X\n",i,gpio_pudedr[i],changed);
		    writel(changed, gpio_base + GPIO_PUDEDR_OFFSET + i * 4);
			printk("changed:%08X, Read:%08X\n",changed,readl(gpio_base + GPIO_PUDEDR_OFFSET + i * 4));
			/* Test PUDEDR */
			changed = gpio_pudcr[i];
			changed &= 0xFFFFFFBF;
			changed |= (~0xFFFFFFBF);
			printk("gpio_pudcr[%d]:%08X, changed:%08X\n",i,gpio_pudcr[i],changed);
		    writel(changed, gpio_base + GPIO_PUDCR_OFFSET + i * 4);
			printk("changed:%08X, Read:%08X\n",changed,readl(gpio_base + GPIO_PUDCR_OFFSET + i * 4));

			break;
		case 4: /*Test PF3,PF4 don't registered at blacklist*/
			/* Test PDR */
			changed &=0xFFFFE7FF;
			changed |=0x18001800;
			printk("gpio_pdr[%d]:%08X, changed:%08X\n",i,gpio_pdr[i],changed);
		    writel(changed, gpio_base + GPIO_PDR_OFFSET + i * 4);
			printk("changed:%08X, Read:%08X\n",changed,readl(gpio_base + GPIO_PDR_OFFSET + i * 4));
			/* Test EPCR */
			changed = gpio_epcr[i];
			changed &= 0xFFFFE7FF;
			changed |= (~0xFFFFE7FF);
			printk("gpio_epcr[%d]:%08X, changed:%08X\n",i,gpio_epcr[i],changed);
		    writel(changed, gpio_base + GPIO_EPCR_OFFSET + i * 4);
			printk("changed:%08X, Read:%08X\n",changed,readl(gpio_base + GPIO_EPCR_OFFSET + i * 4));
			/* Test DDR */
			changed = gpio_ddr[i];
			changed &= 0xFFFFE7FF;
			printk("gpio_ddr[%d]:%08X, changed:%08X\n",i,gpio_ddr[i],changed);
		    writel(changed, gpio_base + GPIO_DDR_OFFSET + i * 4);
			printk("changed:%08X, Read:%08X\n",changed,readl(gpio_base + GPIO_DDR_OFFSET + i * 4));

			/* Test PUDEDR */
			changed = gpio_pudedr[i];
			changed &= 0xFFFFE7FF;
			printk("gpio_pudedr[%d]:%08X, changed:%08X\n",i,gpio_pudedr[i],changed);
		    writel(changed, gpio_base + GPIO_PUDEDR_OFFSET + i * 4);
			printk("changed:%08X, Read:%08X\n",changed,readl(gpio_base + GPIO_PUDEDR_OFFSET + i * 4));
			/* Test PUDCR */
			changed = gpio_pudcr[i];
			changed |= 0x00001800;
			printk("gpio_pudcr[%d]:%08X, changed:%08X\n",i,gpio_pudcr[i],changed);
		    writel(changed, gpio_base + GPIO_PUDCR_OFFSET + i * 4);
			printk("changed:%08X, Read:%08X\n",changed,readl(gpio_base + GPIO_PUDCR_OFFSET + i * 4));

			break;
	}
}
static void test_clktop(int i,
						unsigned int clk_stop[CLKTOP_NUM],
						void __iomem *clk_base
						)
{
	unsigned int change = clk_stop[i]; 
	printk("clk_stop[%d]: %08X\n",i,clk_stop[i]);
	switch( i ) {
		case 0:
			/* Test TEMPCK Readonly */
			change &= 0xCFFFFFFF;
			change |= (~0xCFFFFFFF);
			writel(change, clk_base + CLKTOP_OFFSET + i * 4);
			printk(" Change[%d]: %08X\n",i,readl(clk_base + CLKTOP_OFFSET + i * 4));
			break;
		case 1:
		{
			/* Test NETSECCK: Not ReadOnly */
			change &=   0xFFFF3FFF;
			change |= (~0xFFFF3FFF);
			writel(change, clk_base + CLKTOP_OFFSET + i * 4);
			printk(" Change[%d]: %08X\n",i,readl(clk_base + CLKTOP_OFFSET + i * 4));
			break;
		}
		case 2:
		{
			/* Test HIFCK No exist at DTS*/
			change &= 0x0FFFF3FF;
			change |= (~0x0FFFF3FF);
			writel(change, clk_base + CLKTOP_OFFSET + i * 4);
			printk(" Change[%d]: %08X\n",i,readl(clk_base + CLKTOP_OFFSET + i * 4));
			break;
		}
	}
}
#endif 
#ifdef CO_TEST_PRINT
static void	print_gpio( void __iomem *gpio_address,
					unsigned int gpio_pdr,
					unsigned int gpio_ddr,
					unsigned int gpio_epcr,
					unsigned int gpio_pudedr,
					unsigned int gpio_pudcr,
					const char *outchar,
					int index)
{
	printk("%s Resume[%d]: %08X,savedPDR:%08X\n",
			outchar,
			index,
			readl(gpio_address + GPIO_PDR_OFFSET),
			gpio_pdr);
	printk("%s Resume[%d]: %08X,savedDDR:%08X\n",
			outchar,
			index,
			readl(gpio_address + GPIO_DDR_OFFSET),
			gpio_ddr);
	printk("%s Resume[%d]: %08X,savedEPCR:%08X\n",
			outchar,
			index,
			readl(gpio_address + GPIO_EPCR_OFFSET),
			gpio_epcr);
	printk("%s Resume[%d]: %08X,savedPUDEDR:%08X\n",
			outchar,
			index,
			readl(gpio_address + GPIO_PUDEDR_OFFSET),
			gpio_pudedr);
	printk("%s Resume[%d]: %08X,savedPUDCR:%08X\n",
			outchar,
			index,
			readl(gpio_address + GPIO_PUDCR_OFFSET),
			gpio_pudcr);
}
#endif
static unsigned int get_write_part(unsigned int save_val,
									unsigned int val,
									unsigned int mask,
									int read_only)
{
/* Set the data changed only */
	if(read_only == 0) {
	/* Resume Linux caring clock only*/
        val = (val & (~mask)) | /* Set all other bits*/
        (save_val & mask);  /* Set Saved setting bits only */
	}
	return val;
}
static void resume_gpio(int i, void __iomem *gpio_address, unsigned int save_val)
{
	unsigned int read_val;
	read_val = readl(gpio_address + i * 4);
	/* clear the bit no register at black-list*/
	read_val &= black_list[i + CA7_OFFSET_BLACK_LIST]; 
	/* Resume the bit no register at black-list*/
    read_val |= save_val;
    writel(read_val  , 
			gpio_address + i * 4);
}

static int warp_snapshot(void)
{
    int ret = 0;
    int i, j;
    unsigned int val, ex;
    unsigned int clk_sel[CLKSEL_NUM], clk_pllcntl[PLL_NUM], clk_stop[CLKTOP_NUM];
    unsigned int clk_crswr, clk_crrrs, clk_crrsm;
    unsigned int gpio_pdr[GPIO_NUM], gpio_ddr[GPIO_NUM], gpio_epcr[GPIO_NUM];
    unsigned int gpio_pudedr[GPIO_NUM], gpio_pudcr[GPIO_NUM];
    void __iomem *clk_base, *gpio_base;
    warp_param.private[0] = ((1 << 0) | /* Number of NAND physical chips */
                             (1 << 8)); /* Number of NAND logical chips */
    warp_param.private[1] = trampoline_phys;
    warp_param.private[2] = KERNEL_UNBOOT_FLAG;
    warp_param.private[3] = trampoline_phys + 4 * 4;

    /* GPIO save */
    gpio_base = ioremap_nocache(0x1d022000, 0x1000);
    for (i = 0; i < GPIO_NUM; i++) {
        gpio_pdr[i] = readl(gpio_base + GPIO_PDR_OFFSET + i * 4);
        gpio_ddr[i] = readl(gpio_base + GPIO_DDR_OFFSET + i * 4);
        gpio_epcr[i] = readl(gpio_base + GPIO_EPCR_OFFSET + i * 4);
        gpio_pudedr[i] = readl(gpio_base + GPIO_PUDEDR_OFFSET + i * 4);
        gpio_pudcr[i] = readl(gpio_base + GPIO_PUDCR_OFFSET + i * 4);
#ifdef CO_TEST /* for test */
		test_gpio( i,
				gpio_pdr, 
				gpio_ddr, 
				gpio_epcr,
				gpio_pudedr, 
				gpio_pudcr, 
				gpio_base);
#endif
		/* Get the data which no register on the black-list */
        gpio_pdr[i] |= ((~black_list[i + CA7_OFFSET_BLACK_LIST])<<16);
    }

    /* CLK save */
    clk_base = ioremap_nocache(0x1d021000, 0x1000);/* From CLKSEL1 */
    for (i = 0; i < CLKSEL_NUM; i++) {
        clk_sel[i] = readl(clk_base + CLKSEL_OFFSET + i * 4);
		/* For test*/
#ifdef CO_TEST
		test_clksel(i,clk_sel,clk_base);
#endif
	}
    for (i = 0; i < PLL_NUM; i++) {
        if (i != 6) {
            clk_pllcntl[i] = readl(clk_base + CLKPLL_OFFSET + i * 4);
#ifdef CO_TEST /* for test */
			test_pllcnt(i, clk_pllcntl, clk_base);
#endif
					}
				}
    for (i = 0; i < CLKTOP_NUM; i++) {
        clk_stop[i] = readl(clk_base + CLKTOP_OFFSET + i * 4);
#ifdef CO_TEST /* for test */
			test_clktop(i,
						clk_stop,
						clk_base
						);
#endif
	}


    clk_crswr = readl(clk_base + CLK_CRSWR_OFFSET);
    clk_crrrs = readl(clk_base + CLK_CRRRS_OFFSET);
    clk_crrsm = readl(clk_base + CLK_CRRSM_OFFSET);
#ifndef WARP_HIBDRV_FIXED
    set_mem_exec();
#endif


    /* call hibernation driver */
    ret = hibdrv_snapshot();

#ifndef WARP_HIBDRV_FIXED
    restore_mem_exec();
#endif

    /* Set NF can be used by RTOS */
    writel(RTOS_NF_USE, ControlMagicNumNF);
    iounmap(ControlMagicNumNF);

    /* CLK restore */
    for (i = 0; i < PLL_NUM; i++) {
        if (i != 6) {
	        val = readl(clk_base + CLKPLL_OFFSET + i * 4);
	        ex = val ^ clk_pllcntl[i];
	        for (j = 0; j < PLL_BIT_NUM_AT; j++) {
		        if (pll_bit[i][j].mask == 0)
		            break;
	            if (ex & pll_bit[i][j].mask) {
		            val = get_write_part(clk_pllcntl[i], val,
								pll_bit[i][j].mask, pll_bit[i][j].read_only);
				}
			}
            writel(val, clk_base + CLKPLL_OFFSET + i * 4);
#ifdef CO_TEST_PRINT
		printk("Write  val: %08X,clk_pllcntl[%d]: %08X\n",val,i,clk_pllcntl[i]);
		printk("Resume[%d]: %08X\n",i,readl(clk_base + CLKPLL_OFFSET + i * 4));
#endif
		}
	}

    for (i = 0; i < CLKSEL_NUM; i++) {
        val = readl(clk_base + CLKSEL_OFFSET + i * 4);
        ex = val ^ clk_sel[i];
        for (j = 0; j < CLK_NUM_AT_SEL; j++) {
            if (clk_bit[i][j].mask == 0)
                break;
            if (ex & clk_bit[i][j].mask) {
				/* Set the data changed only */
				if(clk_bit[i][j].read_only == 0) {
					/* Resume Linux caring clock only*/
		            if (clk_bit[i][j].reg_ofs0) {
		                writel(clk_bit[i][j].reg_mask0,
		                       clk_base + clk_bit[i][j].reg_ofs0);
#ifdef CO_TEST
		printk("i:%d,j:%d\n",i,j);
#endif
					}
		            if (clk_bit[i][j].reg_ofs1) {
		                writel(clk_bit[i][j].reg_mask1,
		                       clk_base + clk_bit[i][j].reg_ofs1);
#ifdef CO_TEST
		printk("i:%d,j:%d\n",i,j);
#endif
					}
		            val = (val & ~clk_bit[i][j].mask) | /* Set all other bits*/
		                (clk_sel[i] & clk_bit[i][j].mask) | /* Set Saved setting bits only */
							clk_bit[i][j].wbit; /* Set write bit */
				}
            }
        }
        writel(val, clk_base + CLKSEL_OFFSET + i * 4);
#ifdef CO_TEST_PRINT
		printk("Write  val: %08X,clk_sel[%d]: %08X\n",val,i,clk_sel[i]);
		printk("Resume[%d]: %08X\n",i,readl(clk_base + CLKSEL_OFFSET + i * 4));
#endif
    }
    for (i = 0; i < CLKTOP_NUM; i++) {
        val = readl(clk_base + CLKTOP_OFFSET + i * 4);
        ex = val ^ clk_stop[i];
#ifdef CO_TEST_PRINT
		printk("Read  val[%d]: %08X,ex=%08x\n",i,val,ex);
#endif
        for (j = 0; j < CLKTOP_BIT_NUM; j++) {
	        if (clk_top_bit[i][j].mask == 0)
	            break;
            if (ex & clk_top_bit[i][j].mask) {
				if(clk_top_bit[i][j].read_only == 0) {
#ifdef CO_TEST_PRINT
		printk("clk_top[%d][%d]\n",i,j);
#endif
			/* Resume Linux caring clock only*/
					val = (val & (~clk_top_bit[i][j].mask)) | /* Set all other bits*/
						(clk_stop[i] & clk_top_bit[i][j].mask) |  /* Set Saved setting bits only */
						(clk_top_bit[i][j].mask & (clk_top_bit[i][j].mask<<1)); /* Set write bit */
				}
			}
		}
        writel(val, clk_base + CLKTOP_OFFSET + i * 4);
#ifdef CO_TEST_PRINT
		printk("Write  val: %08X,clk_stop[%d]: %08X\n",val,i,clk_stop[i]);
		printk("Resume[%d]: %08X\n",i,readl(clk_base + CLKTOP_OFFSET + i * 4));
#endif
    }
    writel(clk_crswr, clk_base + CLK_CRSWR_OFFSET);
    writel(clk_crrrs, clk_base + CLK_CRRRS_OFFSET);
    writel(clk_crrsm, clk_base + CLK_CRRSM_OFFSET);
    iounmap(clk_base);

    /* GPIO restore */
    for (i = 0; i < GPIO_NUM; i++) {
#ifdef CO_TEST_PRINT /* for test */
		print_gpio( gpio_base + i * 4,
					gpio_pdr[i],
					gpio_ddr[i],
					gpio_epcr[i],
					gpio_pudedr[i],
					gpio_pudcr[i],
					"Before",
					i);
#endif
		resume_gpio(i, gpio_base + GPIO_EPCR_OFFSET, gpio_epcr[i]);
		resume_gpio(i, gpio_base + GPIO_PUDEDR_OFFSET, gpio_pudedr[i]);
		resume_gpio(i, gpio_base + GPIO_PUDCR_OFFSET, gpio_pudcr[i]);
		resume_gpio(i, gpio_base + GPIO_DDR_OFFSET, gpio_ddr[i]);
		resume_gpio(i, gpio_base + GPIO_PDR_OFFSET, gpio_pdr[i]);

#ifdef CO_TEST_PRINT /* for test */
		print_gpio( gpio_base + i * 4,
					gpio_pdr[i],
					gpio_ddr[i],
					gpio_epcr[i],
					gpio_pudedr[i],
					gpio_pudcr[i],
					"After",
					i);
#endif
    }
    iounmap(gpio_base);
    return ret;
}

static struct warp_ops warp_machine_ops = {
    .snapshot = warp_snapshot,
    .drv_init = warp_drv_init,
    .drv_uninit = warp_drv_uninit,
#ifdef CONFIG_PM_WARP_DEBUG
    .putc = warp_putchar,
#endif
};

static int __init warp_machine_init(void)
{
    return warp_register_machine(&warp_machine_ops);
}

static void __exit warp_machine_exit(void)
{
    warp_unregister_machine(&warp_machine_ops);
}

module_init(warp_machine_init);
module_exit(warp_machine_exit);
