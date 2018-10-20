/**
 * ogma_misc_internal.h
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
#ifndef OGMA_MISC_INTERNAL_H
#define OGMA_MISC_INTERNAL_H

#include "ogma_basic_type.h"
#include "ogma_api.h"

#define OGMA_CLK_EN_REG_DOM_ALL 0x3f

typedef struct ogma_global_s ogma_global_t;

struct ogma_global_s {
	ogma_uint valid_flag:1;

	ogma_uint8 list_entry_num;

	ogma_ctrl_t *list_head_p;
};

#endif				/* OGMA_MISC_INTERNAL_H */
