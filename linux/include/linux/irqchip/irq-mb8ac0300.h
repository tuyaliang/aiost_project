/*
 * linux/arch/arm/mach-mb8ac0300/include/mach/exiu.h
 *
 * Copyright (C) 2011-2015 SOCIONEXT
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

#ifndef __MACH_EXIU_H
#define __MACH_EXIU_H

extern int exiu_irq_set_type(unsigned long irq_num, unsigned long type);

#define EXIU_REG_EIMASK			0x00
#define EXIU_REG_EISRCSEL		0x04
#define EXIU_REG_EIREQSTA		0x08
#define EXIU_REG_EIRAWREQSTA		0x0c
#define EXIU_REG_EIREQCLR		0x10
#define EXIU_REG_EILVL			0x14
#define EXIU_REG_EIEDG			0x18
#define EXIU_REG_EISIR			0x1c

#define EXIU_EIMASK_ALL			0xffffffff

#define EXIU_EISRCSEL_EXINT		0x0
#define EXIU_EISRCSEL_SOFT		0x1
#define EXIU_EISRCSEL_ALL_EXINT		0x00000000

#define EXIU_EIREQCLR_ALL		0xffffffff

#define EXIU_EILVL_LOW			0x0
#define EXIU_EILVL_HIGH			0x1
#define EXIU_EILVL_MASK			0x1
#define EXIU_EILVL_ALL_HIGH		0xffffffff

#define EXIU_EIEDG_LEVEL		0x0
#define EXIU_EIEDG_EDGE			0x1
#define EXIU_EIEDG_MASK			0x1
#define EXIU_EIEDG_ALL_EDGE		0xffffffff

#define EXTINT16_OFFSET			16

#endif /* __MACH_EXIU_H */
