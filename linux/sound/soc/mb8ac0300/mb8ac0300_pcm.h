/*
 * linux/sound/soc/mb8ac0300/mb8_pcm.h
 *
 * Copyright (C) 2011-2012 SOCIONEXT
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

#ifndef _mb8_pcm_H
#define _mb8_pcm_H

#define MB8_PCM_ST_RUNNING 1

/* runtime data of pcm driver */
struct mb8_pcm_runtime {
	spinlock_t lock;			/* lock */
	int state;				/* state of pcm */
	unsigned int dma_period;		/* period bytes */
	u32 dma_start;			/* start pos of dma buffer */
	u32 dma_pos;			/* current pos of dma buffer */
	u32 dma_end;			/* end position of dma buffer */
	u32 dma_addr;	/* address of I2S TX/RX register */
	struct dma_chan *dchan;
	struct dma_async_tx_descriptor *desc[2];
	int idx;
};

#endif/*_mb8_pcm_H*/
