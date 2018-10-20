/**
 * ogma_basic_access.h
 *
 *  Copyright (c) 2015 SOCIONEXT INCORPORATED.
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *   
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *   
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 *
 */

#ifndef OGMA_BASIC_ACCESS_H
#define OGMA_BASIC_ACCESS_H

#include "ogma_internal.h"

static __inline void ogma_write_reg(ogma_ctrl_t * ctrl_p,
				    ogma_uint32 reg_addr, ogma_uint32 value)
{
	pfdep_iomem_write((void *)((pfdep_cpu_addr_t) ctrl_p->base_addr +
				   (reg_addr << 2)), value);
}

static __inline ogma_uint32 ogma_read_reg(ogma_ctrl_t * ctrl_p,
					  ogma_uint32 reg_addr)
{

	return (ogma_uint32)
	    pfdep_iomem_read((void *)((pfdep_cpu_addr_t) ctrl_p->base_addr +
				      (reg_addr << 2)));
}

void ogma_set_mac_reg(ogma_ctrl_t * ctrl_p,
		      ogma_uint32 addr, ogma_uint32 value);

ogma_uint32 ogma_get_mac_reg(ogma_ctrl_t * ctrl_p, ogma_uint32 addr);

#endif				/* OGMA_BASIC_ACCESS_H */
