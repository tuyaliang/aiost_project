/**
 * ogma_config.h
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
 */

#ifndef OGMA_CONFIG_H
#define OGMA_CONFIG_H

/* ---------------------------------------------------------------------------
    constant macro
 --------------------------------------------------------------------------- */

#define OGMA_CONFIG_USE_PKT_DESC_RING
#define OGMA_CONFIG_USE_GMAC

#define OGMA_CONFIG_CLK_HZ      250000000UL
#define OGMA_CONFIG_GMAC_CLK_HZ ( OGMA_CONFIG_CLK_HZ >> 1)

#define OGMA_CONFIG_F_NETSEC_VER       OGMA_F_NETSEC_VER_F_TAIKI

#endif				/* OGMA_CONFIG_H */
