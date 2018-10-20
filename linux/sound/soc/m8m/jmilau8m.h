/*
 * linux/sound/soc/m8m/jmilau8m.h
 *
 * Copyright (C) 2015 Linaro, Ltd
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _F_SAIF_H
#define _F_SAIF_H

/* this is the only per-IP register */
#define JMILAU_REG_AURES		0

/* these are all offset by 0x100 per channel */
#define JMILAU_CHANREG_AUIDLR		0	/* audio in data */
#define JMILAU_CHANREG_AUODLR		4	/* audio out data */
#define JMILAU_CHANREG_AUCR		8
	/* output (8 + 16 bits): 0 = repeat MSB, 1 = pad with 0 */
  #define JMILAU_AUCR__AUORF		BIT(31)
	/* output format */
  #define JMILAU_AUCR__AUOF_SHIFT	24
  #define JMILAU_AUCR__AUOF_MASK	0x3f
	/* input: 0 = repeat MSB, 1 = pad with 0 */
  #define JMILAU_AUCR__AUIRF		BIT(23)
	/* input format */
  #define JMILAU_AUCR__AUIF_SHIFT	16
  #define JMILAU_AUCR__AUIF_MASK	0xf
	/* If the output register is empty: This is a bit to enable
	 * DMA start request to (AUCR INTO = 1).
	 * When "1", the output register is when empty, and outputs
	 * a DMA start request.
	 * When "0", the output register does not perform the DMA
	 * start request operation by empty.
	 * ※ If you do the DMA transfer to the output operation,
	 * DMOE or AUST: either EDMOE should be set to "1".
	 * ※ When DMA transfer should be set the transfer mode to demand transfer.
	 */
  #define JMILAU_AUCR__DMOE		BIT(15)
	/* as DMOE but for input / AUCT: INTI / DMIE / EDMIE */
  #define JMILAU_AUCR__DMIE		BIT(14)
	/* Enable OUTPUT operation */
  #define JMILAU_AUCR__AUOE		BIT(13)
	/* Enable INPUT operation */
  #define JMILAU_AUCR__AUIE		BIT(12)
	/* Output register empty Interrupt Enable */
  #define JMILAU_AUCR__INTOE		BIT(11)
	/* Input register full Interrupt Enable */
  #define JMILAU_AUCR__INTIE		BIT(10)
	/* Output register Empty IRQ Status (cleared by write to out reg) */
  #define JMILAU_AUCR__INTO		BIT(9)
	/* Input register Full IRQ Status (cleared by read from reg) */
  #define JMILAU_AUCR__INTI		BIT(8)
	/* set 0 if MONO output, 1 = 24-bit or normal */
  #define JMILAU_AUCR__AUODSEL		BIT(1)
	/* set to 0 if MONO input, 1 = 24-bit / normal */
  #define JMILAU_AUCR__AUIDSEL		BIT(0)
#define JMILAU_CHANREG_AUMD		0xc
	/* loopback */
  #define JMILAU_AUMD__LB		BIT(31)
	/* 0 = MONO, 1 = Stereo */
  #define JMILAU_AUMD__STEREO		BIT(27)
	/* Output: 0 = 1 stage FIFO, 1 = 16-stage FIFO */
  #define JMILAU_AUMD__OSTG16		BIT(25)
	/* Input: 0 = 1 stage FIFO, 1 = 16-stage FIFO */
  #define JMILAU_AUMD__ISTG16		BIT(24)
#define JMILAU_CHANREG_AUST		0x10
	/* FIFO interrupt level 00 = if not empty/full, 01=12, 10=8, 11=4 */
  #define JMILAU_AUST__ESTG_SHIFT	30
  #define JMILAU_AUST__ESTG_MASK	3
	/* Output DMA request enable when FIFO trigger hit */
  #define JMILAU_AUST__EDMOE		BIT(29)
	/* Input DMA request enable when FIFO trigger hit */
  #define JMILAU_AUST__EDMIE		BIT(28)
	/* Output Interrupt Enable */
  #define JMILAU_AUST__EINTOE		BIT(27)
	/* Input Interrupt Enable */
  #define JMILAU_AUST__EINTIE		BIT(26)
	/* Output FIFO reached trigger limit */
  #define JMILAU_AUST__EINTO		BIT(25)
	/* Input FIFO reached trigger limit */
  #define JMILAU_AUST__EINTI		BIT(24)
	/* Output Underflow Error Interrupt Enable */
  #define JMILAU_AUST__UFIE		BIT(19)
	/* Input Overflow Error Interrupt Enable */
  #define JMILAU_AUST__OFIE		BIT(18)
	/* Output Underflow Interrupt Status */
  #define JMILAU_AUST__ORUF		BIT(17)
	/* Input Overflow Interrupt Status */
  #define JMILAU_AUST__IROF		BIT(16)
#define JMILAU_CHANREG_AUCC		0x14
	/* Audio clock output enable (0 = clock is input, 1 = clock is output)*/
  #define JMILAU_AUCC__AUCKOE		BIT(31)
	/* Enable the clock dividers below */
  #define JMILAU_AUCC__DIVE		BIT(30)
	/* CK clocks per MCLK (0 = 1:1, 1 = 2:1, then all odd to 0x1f = 32:1) */
  #define JMILAU_AUCC__DIVMCK_SHIFT	24
  #define JMILAU_AUCC__DIVMCK_MASK	0x1f
	/* input clock division -> CK (0 = 4, 1 = 8 ... 7 = 32) */
  #define JMILAU_AUCC__DIVCK_SHIFT	19
  #define JMILAU_AUCC__DIVCK_MASK	7
	/* how many AUCLKs per word (0 =16, 1 = 32, 2 = 48, 3 = 64 */
  #define JMILAU_AUCC__DIVLR_SHIFT	16
  #define JMILAU_AUCC__DIVLR_MASK	3
#define JMILAU_CHANREG_AUDP		0x18
	/* Extra gain on INPUT (00 = none, 01 = 6dB, 10 = 12dB, 11 = 18dB) */
  #define JMILAU_AUDP__AUIDS_SHIFT	28
  #define JMILAU_AUDP__AUIDS_MASK	3
	/* Extra gain on OUTPUT (00 = none, 01 = 6dB, 10 = 12dB, 11 = 18dB) */
  #define JMILAU_AUDP__AUODS_SHIFT	24
  #define JMILAU_AUDP__AUODS_MASK	3
	/* select OUTPUT AHB data format (0 = LE, 1 = BE) */
  #define JMILAU_AUDP__AUODF		BIT(17)
	/* select OUTPUT AHB data format (0 = LE, 1 = BE) */
  #define JMILAU_AUDP__AUIDF		BIT(16)
	/* 0 = normal 1 = copy left to right */
  #define JMILAU_AUDP__AULRCP		BIT(8)
	/* 0 = normal 1 = mix left and right */
  #define JMILAU_AUDP__MIXPLAY		BIT(0)
#define JMILAU_CHANREG_AUIFST		0x1c
#define JMILAU_CHANREG_AUOFST		0x20
#define JMILAU_CHANREG_AUIDL		0x28
#define JMILAU_CHANREG_AUIDR		0x2c
#define JMILAU_CHANREG_AUODL		0x30
#define JMILAU_CHANREG_AUODR		0x34
#define JMILAU_CHANREG_AUIDDMAPT	0x38
#define JMILAU_CHANREG_AUODDMAPT	0x3c
#define JMILAU_CHANREG_AUDMA2CTL	0x40
	/* enable dma samples to output register */
  #define JMILAU_AUDMA2CTL__DMO2CHEN	BIT(8)
	/* enable dma samples from input register */
  #define JMILAU_AUDMA2CTL__DMI2CHEN	BIT(0)
/* count of INPUT samples to DMA */
#define JMILAU_CHANREG_AUDMISAMPLE	0x44
/* count of OUTPUT samples to DMA */
#define JMILAU_CHANREG_AUDMOSAMPLE	0x48

/* output format word structure */

#define JMILAU_OFORMAT_LEFT_JUSTIFIED	0
#define JMILAU_OFORMAT_RIGHT_JUSTIFIED	BIT(1)
#define JMILAU_OFORMAT_I2S		BIT(6)
#define JMILAU_OFORMAT_TYPE_MASK	(BIT(6) | BIT(1))

#define JMILAU_OFORMAT_BITS_8		0
#define JMILAU_OFORMAT_BITS_16		BIT(2)
#define JMILAU_OFORMAT_BITS_24		BIT(3)
#define JMILAU_OFORMAT_BITS_MASK	(BIT(3) | BIT(2))

#define JMILAU_OFORMAT_RATIO_32		0
#define JMILAU_OFORMAT_RATIO_48		BIT(4)
#define JMILAU_OFORMAT_RATIO_64		BIT(0)
#define JMILAU_OFORMAT_RATIO_128	BIT(5)
#define JMILAU_OFORMAT_RATIO_MASK	(BIT(5) | BIT(4) | BIT(0))

/* input format word structure */

#define JMILAU_IFORMAT_LEFT_JUSTIFIED	0
#define JMILAU_IFORMAT_RIGHT_JUSTIFIED	BIT(0)
#define JMILAU_IFORMAT_I2S		BIT(3)
#define JMILAU_IFORMAT_TYPE_MASK	(BIT(3) | BIT(0))

#define JMILAU_IFORMAT_BITS_8		0
#define JMILAU_IFORMAT_BITS_16		BIT(1)
#define JMILAU_IFORMAT_BITS_24		BIT(2)
#define JMILAU_IFORMAT_BITS_MASK	(BIT(2) | BIT(1))

#endif/* _F_SAIF_H */

