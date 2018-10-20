/**
 * ogma_basic_type.h
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
 * Declarations of public functions and constants for OGMA primitive.
 *
 */

#ifndef OGMA_BASIC_TYPE_H
#define OGMA_BASIC_TYPE_H
#include "pfdep.h"

/**
 *
 */
#define OGMA_TRUE  PFDEP_TRUE
#define OGMA_FALSE PFDEP_FALSE

/**
 * OGMA SDK BASIC DATA TYPE
 */
typedef pfdep_int8 ogma_int8;
typedef pfdep_uint8 ogma_uint8;
typedef pfdep_int16 ogma_int16;
typedef pfdep_uint16 ogma_uint16;
typedef pfdep_int32 ogma_int32;
typedef pfdep_uint32 ogma_uint32;
typedef int ogma_int;
typedef unsigned int ogma_uint;
typedef pfdep_bool ogma_bool;
typedef pfdep_char ogma_char;

#ifdef PFDEP_INT64_AVAILABLE
typedef signed long long ogma_int64;
typedef unsigned long long ogma_uint64;
#endif				/* PFDEP_INT64_AVAILABLE */

#endif				/* OGMA_BASIC_TYPE_H */
