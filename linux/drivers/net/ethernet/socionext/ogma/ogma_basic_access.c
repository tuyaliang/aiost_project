/**
 * ogma_basic_access.c
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
 * Functions to provide basic access to OGMA resources:
 * GMAC and
 * general purpose registers.*
 *
 */
#include "ogma_internal.h"
#include "ogma_basic_access.h"

/**********************************************************************
 * Function definitions
 **********************************************************************/

void ogma_set_mac_reg(ogma_ctrl_t * ctrl_p, ogma_uint32 addr, ogma_uint32 value)
{

	ogma_uint32 cmd;

	ogma_check_clk_supply(ctrl_p, OGMA_CLK_EN_REG_DOM_G);

	/*
	 * Argument check is omitted because this function is
	 * of private use only.
	 */

	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_MAC_DATA, value);

	cmd = addr | OGMA_GMAC_CMD_ST_WRITE;

	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_MAC_CMD, cmd);

	/*
	 * Waits until BUSY bit is cleared.
	 */
	while ((ogma_read_reg(ctrl_p, OGMA_REG_ADDR_MAC_CMD)
		& OGMA_GMAC_CMD_ST_BUSY)
	       != 0) {
		;
	}
}

ogma_uint32 ogma_get_mac_reg(ogma_ctrl_t * ctrl_p, ogma_uint32 addr)
{
	ogma_uint32 cmd;

	ogma_check_clk_supply(ctrl_p, OGMA_CLK_EN_REG_DOM_G);

	/*
	 * Argument check is omitted because this function is
	 * of private use only.
	 */

	cmd = addr | OGMA_GMAC_CMD_ST_READ;

	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_MAC_CMD, cmd);

	/*
	 * Waits until BUSY bit is cleared.
	 */
	while ((ogma_read_reg(ctrl_p, OGMA_REG_ADDR_MAC_CMD)
		& OGMA_GMAC_CMD_ST_BUSY)
	       != 0) {
		;
	}
	return ogma_read_reg(ctrl_p, OGMA_REG_ADDR_MAC_DATA);
}
