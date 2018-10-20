/**
 * ogma_misc.c
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

#define DEBUG

#include "ogma_config.h"
#include "ogma_internal.h"
#include "ogma_core.h"
#include "ogma_basic_access.h"
#include "ogma_misc_internal.h"

#define OGMA_CONFIG_DISABLE_CLK_CTRL

#undef STATIC
#ifdef MODULE_TEST
#define STATIC
#else
#define STATIC static
#endif

ogma_global_t ogma_global = {
	OGMA_FALSE,
	0,
	NULL
};

static const ogma_uint32 hw_ver_reg_addr = OGMA_REG_ADDR_F_TAIKI_VER;
static const ogma_uint32 mc_ver_reg_addr = OGMA_REG_ADDR_F_TAIKI_MC_VER;

static const ogma_uint32 desc_ring_irq_status_reg_addr[OGMA_DESC_RING_ID_MAX +
						       1] = {
	OGMA_REG_ADDR_NRM_TX_STATUS,
	OGMA_REG_ADDR_NRM_RX_STATUS,
	0,
	0
};

static const ogma_uint32 desc_ring_config_reg_addr[OGMA_DESC_RING_ID_MAX + 1] = {
	OGMA_REG_ADDR_NRM_TX_CONFIG,
	OGMA_REG_ADDR_NRM_RX_CONFIG,
	0,
	0
};

/* Internal function definition*/
#ifndef OGMA_CONFIG_DISABLE_CLK_CTRL

STATIC void ogma_set_clk_en_reg(ogma_ctrl_t * ctrl_p);
#endif				/* OGMA_CONFIG_DISABLE_CLK_CTRL */
STATIC void ogma_global_init(void);

STATIC ogma_err_t ogma_probe_hardware(void *base_addr);

STATIC void ogma_reset_hardware(ogma_ctrl_t * ctrl_p);

STATIC void ogma_internal_terminate(ogma_ctrl_t * ctrl_p);

#ifdef OGMA_CONFIG_DISABLE_CLK_CTRL

#define ogma_set_clk_en_reg( ctrl_p)

#else				/* OGMA_CONFIG_DISABLE_CLK_CTRL */
STATIC void ogma_set_clk_en_reg(ogma_ctrl_t * ctrl_p)
{
	ogma_uint32 value = 0;

	if (ctrl_p->clk_ctrl.dmac_req_num != 0) {
		value |= OGMA_CLK_EN_REG_DOM_D;
	}

	if (ctrl_p->clk_ctrl.core_req_num != 0) {
		value |= OGMA_CLK_EN_REG_DOM_C;
	}

	if (ctrl_p->clk_ctrl.mac_req_num != 0) {
		value |= OGMA_CLK_EN_REG_DOM_G;
	}

	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_CLK_EN, value);
}
#endif				/* OGMA_CONFIG_DISABLE_CLK_CTRL */

void ogma_push_clk_req(ogma_ctrl_t * ctrl_p, ogma_uint32 domain)
{
	pfdep_hard_lock_ctx_t clk_ctrl_hard_lock_ctx;

	pfdep_acquire_hard_lock(&ctrl_p->clk_ctrl_hard_lock,
				&clk_ctrl_hard_lock_ctx);

	if ((domain & OGMA_CLK_EN_REG_DOM_D) != 0) {
		++ctrl_p->clk_ctrl.dmac_req_num;
	}

	if ((domain & OGMA_CLK_EN_REG_DOM_C) != 0) {
		++ctrl_p->clk_ctrl.core_req_num;
	}

	if ((domain & OGMA_CLK_EN_REG_DOM_G) != 0) {
		++ctrl_p->clk_ctrl.mac_req_num;
	}

	ogma_set_clk_en_reg(ctrl_p);

	pfdep_release_hard_lock(&ctrl_p->clk_ctrl_hard_lock,
				&clk_ctrl_hard_lock_ctx);
}

void ogma_pop_clk_req(ogma_ctrl_t * ctrl_p, ogma_uint32 domain)
{
	pfdep_hard_lock_ctx_t clk_ctrl_hard_lock_ctx;

	pfdep_acquire_hard_lock(&ctrl_p->clk_ctrl_hard_lock,
				&clk_ctrl_hard_lock_ctx);

	if ((domain & OGMA_CLK_EN_REG_DOM_D) != 0) {
		--ctrl_p->clk_ctrl.dmac_req_num;
	}

	if ((domain & OGMA_CLK_EN_REG_DOM_C) != 0) {
		--ctrl_p->clk_ctrl.core_req_num;
	}

	if ((domain & OGMA_CLK_EN_REG_DOM_G) != 0) {
		--ctrl_p->clk_ctrl.mac_req_num;
	}

	ogma_set_clk_en_reg(ctrl_p);

	pfdep_release_hard_lock(&ctrl_p->clk_ctrl_hard_lock,
				&clk_ctrl_hard_lock_ctx);
}

/* Internal function */
STATIC void ogma_global_init(void)
{
	ogma_global.valid_flag = OGMA_TRUE;
}

STATIC ogma_err_t ogma_probe_hardware(void *base_addr)
{
	ogma_uint32 value;
	void *addr;

	addr = (void *)((pfdep_cpu_addr_t) base_addr +
			(OGMA_REG_ADDR_CLK_EN << 2));
	pfdep_iomem_write(addr, OGMA_CLK_EN_REG_DOM_ALL);
	/* Read CLK_EN Register */
	value = pfdep_iomem_read(addr);

	if (value != OGMA_CLK_EN_REG_DOM_ALL) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "Unexpected initial clk_en value - (0x%p) = 0x%08x\n",
			    addr, value);
		//      return OGMA_ERR_NOTAVAIL;
	}

	return OGMA_ERR_OK;
}

STATIC void ogma_reset_hardware(ogma_ctrl_t * ctrl_p)
{
	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_CLK_EN, OGMA_CLK_EN_REG_DOM_ALL);

	if (ctrl_p->core_enabled_flag) {

		ogma_write_reg(ctrl_p,
			       OGMA_REG_ADDR_DMA_HM_CTRL,
			       OGMA_DMA_CTRL_REG_STOP);

		ogma_write_reg(ctrl_p,
			       OGMA_REG_ADDR_DMA_MH_CTRL,
			       OGMA_DMA_CTRL_REG_STOP);

		while ((ogma_read_reg(ctrl_p, OGMA_REG_ADDR_DMA_HM_CTRL)
			& OGMA_DMA_CTRL_REG_STOP) != 0) {
			;
		}

		while ((ogma_read_reg(ctrl_p, OGMA_REG_ADDR_DMA_MH_CTRL)
			& OGMA_DMA_CTRL_REG_STOP) != 0) {
			;
		}
	}

	if (ctrl_p->param.use_gmac_flag) {

		/* Reset F_GMAC4MT */
		ogma_set_mac_reg(ctrl_p,
				 OGMA_GMAC_REG_ADDR_BMR,
				 OGMA_GMAC_BMR_REG_RESET);

	}

	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_SOFT_RST, OGMA_SOFT_RST_REG_RESET);

	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_SOFT_RST, OGMA_SOFT_RST_REG_RUN);

	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_COM_INIT, OGMA_COM_INIT_REG_ALL);

	while ((ogma_read_reg(ctrl_p, OGMA_REG_ADDR_COM_INIT)
		& OGMA_COM_INIT_REG_CORE) != 0) {
		;
	}

	if (ctrl_p->param.use_gmac_flag) {

		/* MAC desc init */
		ogma_write_reg(ctrl_p,
			       OGMA_REG_ADDR_MAC_DESC_INIT,
			       OGMA_MAC_DESC_INIT_REG_INIT);

		/* Wait MAC desc init done */
		while ((ogma_read_reg(ctrl_p, OGMA_REG_ADDR_MAC_DESC_INIT)
			& OGMA_MAC_DESC_INIT_REG_INIT) != 0) {
			;
		}

		/* set MAC_INTF_SEL */
		ogma_write_reg(ctrl_p,
			       OGMA_REG_ADDR_MAC_INTF_SEL,
			       ctrl_p->param.gmac_config.phy_interface);

	}

}

ogma_err_t ogma_init(void *base_addr,
		     pfdep_dev_handle_t dev_handle,
		     const ogma_param_t * param_p,
		     void *netsec_handle, ogma_handle_t * ogma_handle_p)
{
	ogma_int i;
	ogma_uint32 domain = 0, hw_ver, mc_ver, value;
	ogma_err_t ogma_err;
	ogma_ctrl_t *ctrl_p = NULL;

	ogma_bool inten_reg_hard_lock_init_flag = OGMA_FALSE;

	ogma_bool all_lock_init_done_flag = OGMA_FALSE;

	pfdep_err_t pfdep_err;

	if ((param_p == NULL) || (ogma_handle_p == NULL)) {
		return OGMA_ERR_PARAM;
	}

	if (ogma_global.list_entry_num + 1 > OGMA_INSTANCE_NUM_MAX) {
		return OGMA_ERR_INVALID;
	}

	if ((!param_p->desc_ring_param[OGMA_DESC_RING_ID_NRM_TX].valid_flag) ||
	    (!param_p->desc_ring_param[OGMA_DESC_RING_ID_NRM_RX].valid_flag) ||
	    (param_p->desc_ring_param[OGMA_DESC_RING_ID_RESERVED_RX].valid_flag)
	    || (param_p->
		desc_ring_param[OGMA_DESC_RING_ID_RESERVED_TX].valid_flag)) {
		pr_err("An error occurred at ogma_init.\n"
		       "Please set invalid packet desc_ring_param valid_flag.\n");
		return OGMA_ERR_DATA;
	}

	if (param_p->use_gmac_flag) {
		if ((param_p->gmac_config.phy_interface !=
		     OGMA_PHY_INTERFACE_GMII) &&
		    (param_p->gmac_config.phy_interface !=
		     OGMA_PHY_INTERFACE_RGMII) &&
		    (param_p->gmac_config.phy_interface !=
		     OGMA_PHY_INTERFACE_RMII)) {
			pr_err("An error occurred at ogma_init.\n"
			       "Please set phy_interface to valid value.\n");
			return OGMA_ERR_DATA;
		}
	} else {
		pr_err("An error occurred at ogma_init.\n"
		       "Please set use_gmac_flag OGMA_TRUE.\n");
		return OGMA_ERR_DATA;
	}

	ogma_err = ogma_probe_hardware(base_addr);

	if (ogma_err != OGMA_ERR_OK) {
		return ogma_err;
	}

	if (!ogma_global.valid_flag) {
		ogma_global_init();
	}

	if ((ctrl_p = pfdep_malloc(sizeof(ogma_ctrl_t))) == NULL) {
		pr_err("An error occurred at ogma_init.\n"
		       "Failed to ogma_handle memory allocation.\n");
		return OGMA_ERR_ALLOC;
	}

	pfdep_memset(ctrl_p, 0, sizeof(ogma_ctrl_t));

	ctrl_p->base_addr = base_addr;

	ctrl_p->dev_handle = dev_handle;

	ctrl_p->netsec_handle = netsec_handle;

	pfdep_memcpy((void *)&ctrl_p->param,
		     (void *)param_p, sizeof(ogma_param_t));

	/* Initialize hardware lock */
	pfdep_err = pfdep_init_hard_lock(&ctrl_p->inten_reg_hard_lock);
	if (pfdep_err != PFDEP_ERR_OK) {
		pr_err("An error occurred at ogma_init.\n"
		       "Failed to inten_reg_hard_lock's initialization.\n");
		ogma_err = OGMA_ERR_ALLOC;
		goto err;
	}
	inten_reg_hard_lock_init_flag = OGMA_TRUE;

	pfdep_err = pfdep_init_hard_lock(&ctrl_p->clk_ctrl_hard_lock);
	if (pfdep_err != PFDEP_ERR_OK) {
		pr_err("An error occurred at ogma_init.\n"
		       "Failed to clk_ctrl_hard_lock's initialization.\n");
		ogma_err = OGMA_ERR_ALLOC;
		goto err;
	}

	all_lock_init_done_flag = OGMA_TRUE;

	pfdep_err = pfdep_dma_malloc(dev_handle,
				     OGMA_DUMMY_DESC_ENTRY_LEN,
				     &ctrl_p->dummy_desc_entry_addr,
				     &ctrl_p->dummy_desc_entry_phys_addr);

	if (pfdep_err != PFDEP_ERR_OK) {
		ogma_err = OGMA_ERR_ALLOC;
		pr_err("An error occurred at ogma_init.\n"
		       "Failed to dummy_desc_entry's memory allocation.\n");
		goto err;
	}

	/* clear dummy desc entry */
	pfdep_memset(ctrl_p->dummy_desc_entry_addr,
		     0, OGMA_DUMMY_DESC_ENTRY_LEN);

	hw_ver = ogma_read_reg(ctrl_p, hw_ver_reg_addr);

	if (OGMA_F_NETSEC_VER_MAJOR_NUM(hw_ver) !=
	    OGMA_F_NETSEC_VER_MAJOR_NUM(OGMA_REG_ADDR_OGMA_VER_F_TAIKI)) {
		ogma_err = OGMA_ERR_NOTAVAIL;
		goto err;
	}

	domain = OGMA_CLK_EN_REG_DOM_D | OGMA_CLK_EN_REG_DOM_C;

	if (param_p->use_gmac_flag) {
		domain |= OGMA_CLK_EN_REG_DOM_G;
	}

	ogma_push_clk_req(ctrl_p, domain);

	ctrl_p->rx_pkt_buf_len = OGMA_RX_PKT_BUF_LEN;

	/* alloc desc_ring */
	for (i = 0; i <= OGMA_DESC_RING_ID_MAX; i++) {
		ogma_err =
		    ogma_alloc_desc_ring(ctrl_p, (ogma_desc_ring_id_t) i);
		if (ogma_err != OGMA_ERR_OK) {
			pr_err("An error occurred at ogma_init.\n"
			       "Failed to ring id NO.%d memory allocation.\n",
			       i);
			goto err;
		}
	}

	if (param_p->desc_ring_param[OGMA_DESC_RING_ID_NRM_RX].valid_flag) {
		if ((ogma_err = ogma_setup_rx_desc_ring(ctrl_p,
							&ctrl_p->desc_ring
							[OGMA_DESC_RING_ID_NRM_RX]))
		    != OGMA_ERR_OK) {
			pr_err("An error occurred at ogma_init.\n"
			       "Failed to NRM_RX packet memory allocation.\n");
			goto err;
		}
	}

	/* microengines need domain G */
	ogma_push_clk_req(ctrl_p, OGMA_CLK_EN_REG_DOM_G);

	/* Clock Enable */
	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_CLK_EN, 0x20);

	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_MAC_DESC_INIT, 1);

	do {
		udelay(1);
		value = ogma_read_reg(ctrl_p, OGMA_REG_ADDR_MAC_DESC_INIT);
	} while ((value & 1) != 0);

	/* Select RGMII */
	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_MAC_INTF_SEL, 0x00000001);

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_BMR,
			 OGMA_GMAC_BMR_REG_RESET);

	do {
		udelay(1);
		value = ogma_get_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_BMR);
	} while ((value & 1) != 0);

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_BMR,
			 OGMA_GMAC_BMR_REG_COMMON);

	/* MMC Receive Interrupt All Masked */
	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MMC_INTR_MASK_RX,
			 0x00FFFFFF);
	/* MMC Transmit Interrupt All Masked */
	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MMC_INTR_MASK_TX,
			 0x01FFFFFF);
	/* MMC Receive Checksum Offload Interrupt All Masked */
	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MMC_IPC_INTR_MASK_RX,
			 0x3FFF3FFF);
	/* Disable MAC LPI/Time Stamp/PMT Interrupt */
	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_IMR, 0x00000608);

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_RDLAR,
			 OGMA_GMAC_RDLAR_REG_COMMON);

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_TDLAR,
			 OGMA_GMAC_TDLAR_REG_COMMON);

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MFFR, 0x80000001UL);

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_OMR,
			 (OGMA_GMAC_OMR_REG_ST | OGMA_GMAC_OMR_REG_SR));

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MCR, 0x0200280c);	/* 1Gbps, full */

	ogma_pop_clk_req(ctrl_p, OGMA_CLK_EN_REG_DOM_G);

	ctrl_p->core_enabled_flag = OGMA_TRUE;

	ctrl_p->next_p = ogma_global.list_head_p;

	ogma_global.list_head_p = ctrl_p;

	++ogma_global.list_entry_num;

	*ogma_handle_p = ctrl_p;

	dev_info(dev_handle, "hardware version: %08x\n", hw_ver);

	mc_ver = ogma_read_reg(ctrl_p, mc_ver_reg_addr);
	dev_info(dev_handle, "microcode version: %08x\n", mc_ver);

	ogma_pop_clk_req(ctrl_p,
			 (OGMA_CLK_EN_REG_DOM_C | OGMA_CLK_EN_REG_DOM_D));

	return OGMA_ERR_OK;

 err:
	if (!all_lock_init_done_flag) {

		if (inten_reg_hard_lock_init_flag) {
			pfdep_uninit_hard_lock(&ctrl_p->inten_reg_hard_lock);
		}

	} else {

		ogma_internal_terminate(ctrl_p);

	}

	pfdep_free(ctrl_p);

	return ogma_err;
}

STATIC void ogma_internal_terminate(ogma_ctrl_t * ctrl_p)
{
	ogma_int i;

	ogma_reset_hardware(ctrl_p);

	/* free desc_ring */
	for (i = 0; i <= OGMA_DESC_RING_ID_MAX; i++) {
		ogma_free_desc_ring(ctrl_p, &ctrl_p->desc_ring[i]);
	}

	if (ctrl_p->dummy_desc_entry_addr != NULL) {
		pfdep_dma_free(ctrl_p->dev_handle,
			       OGMA_DUMMY_DESC_ENTRY_LEN,
			       ctrl_p->dummy_desc_entry_addr,
			       ctrl_p->dummy_desc_entry_phys_addr);
	}

	pfdep_uninit_hard_lock(&ctrl_p->inten_reg_hard_lock);

	pfdep_uninit_hard_lock(&ctrl_p->clk_ctrl_hard_lock);

}

ogma_err_t ogma_terminate(ogma_handle_t ogma_handle)
{

	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_ctrl_t *tmp_ctrl_p = NULL, *old_tmp_ctrl_p = NULL;

	if ((ctrl_p == NULL) || (ogma_global.list_entry_num == 0)) {
		return OGMA_ERR_PARAM;
	}

	pfdep_assert(ogma_global.list_head_p != NULL);

	tmp_ctrl_p = ogma_global.list_head_p;

	while (1) {
		if (tmp_ctrl_p == NULL) {
			/* Could not found ctrl_p specified from the list */
			return OGMA_ERR_NOTAVAIL;
		}
		if (ctrl_p == tmp_ctrl_p) {
			if (old_tmp_ctrl_p != NULL) {
				old_tmp_ctrl_p->next_p = ctrl_p->next_p;
			} else {
				ogma_global.list_head_p = ctrl_p->next_p;
			}
			break;
		}
		old_tmp_ctrl_p = tmp_ctrl_p;
		tmp_ctrl_p = tmp_ctrl_p->next_p;
	}

	ogma_internal_terminate(ctrl_p);

	--ogma_global.list_entry_num;

	pfdep_free(ctrl_p);
	return OGMA_ERR_OK;
}

void ogma_cnt_minus(void)
{
	--ogma_global.list_entry_num;
}

ogma_err_t ogma_enable_top_irq(ogma_handle_t ogma_handle,
			       ogma_uint32 irq_factor)
{
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;

	if (ctrl_p == NULL) {
		return OGMA_ERR_PARAM;
	}

	/* push clock */
	ogma_push_clk_req(ctrl_p, domain);

	/* set irq_factor */
	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_TOP_INTEN_SET, irq_factor);

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return OGMA_ERR_OK;
}

ogma_err_t ogma_disable_top_irq(ogma_handle_t ogma_handle,
				ogma_uint32 irq_factor)
{
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;

	if (ctrl_p == NULL) {
		return OGMA_ERR_PARAM;
	}

	/* push clock */
	ogma_push_clk_req(ctrl_p, domain);

	/* clear irq_factor */
	ogma_write_reg(ctrl_p, OGMA_REG_ADDR_TOP_INTEN_CLR, irq_factor);

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return OGMA_ERR_OK;
}

ogma_err_t ogma_enable_desc_ring_irq(ogma_handle_t ogma_handle,
				     ogma_desc_ring_id_t ring_id,
				     ogma_uint32 irq_factor)
{
	ogma_err_t ogma_err = OGMA_ERR_OK;
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_desc_ring_t *desc_ring_p;

	pfdep_err_t pfdep_err;
	pfdep_soft_lock_ctx_t soft_lock_ctx;

	if (ctrl_p == NULL) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_enable_desc_ring_irq.\n"
			    "Please set valid ogma_handle.\n");
		return OGMA_ERR_PARAM;
	}

	if (ring_id > OGMA_DESC_RING_ID_MAX) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_enable_desc_ring_irq.\n"
			    "Please select ring id number between 0 and %d.\n",
			    OGMA_DESC_RING_ID_MAX);
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_enable_desc_ring_irq.\n"
			    "Please select valid desc ring.\n");
		return OGMA_ERR_NOTAVAIL;
	}

	desc_ring_p = &ctrl_p->desc_ring[ring_id];

	/* get soft lock */
	pfdep_err = pfdep_acquire_soft_lock(&desc_ring_p->soft_lock,
					    &soft_lock_ctx);

	if (pfdep_err != PFDEP_ERR_OK) {
		return OGMA_ERR_INTERRUPT;
	}

	if (!desc_ring_p->running_flag) {
		pfdep_release_soft_lock(&desc_ring_p->soft_lock,
					&soft_lock_ctx);
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_enable_desc_ring_irq.\n"
			    "Please select running desc ring.\n");
		return OGMA_ERR_NOTAVAIL;
	}

	/* push clock */
	ogma_push_clk_req(ctrl_p, domain);

	/* set irq_factor */
	ogma_write_reg(ctrl_p,
		       desc_ring_irq_inten_set_reg_addr[ring_id], irq_factor);

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	/* free soft_lock */
	pfdep_release_soft_lock(&desc_ring_p->soft_lock, &soft_lock_ctx);
	return ogma_err;
}

ogma_err_t ogma_disable_desc_ring_irq(ogma_handle_t ogma_handle,
				      ogma_desc_ring_id_t ring_id,
				      ogma_uint32 irq_factor)
{
	ogma_err_t ogma_err = OGMA_ERR_OK;
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;

	if (ctrl_p == NULL) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_disable_desc_ring_irq.\n"
			    "Please set valid ogma_handle.\n");
		return OGMA_ERR_PARAM;
	}

	if (ring_id > OGMA_DESC_RING_ID_MAX) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_disable_desc_ring_irq.\n"
			    "Please select ring id number between 0 and %d.\n",
			    OGMA_DESC_RING_ID_MAX);
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_disable_desc_ring_irq.\n"
			    "Please select valid desc ring.\n");
		return OGMA_ERR_NOTAVAIL;
	}

	/* push clock */
	ogma_push_clk_req(ctrl_p, domain);

	/* Clear irq factor */
	ogma_write_reg(ctrl_p,
		       desc_ring_irq_inten_clr_reg_addr[ring_id], irq_factor);

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return ogma_err;
}

ogma_uint32 ogma_get_top_irq_enable(ogma_handle_t ogma_handle)
{
	ogma_uint32 value;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;

	if (ctrl_p == NULL) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_top_irq_enable.\n"
			    "Please set valid ogma_handle.\n");
		return 0;
	}

	/*push clock */
	ogma_push_clk_req(ctrl_p, domain);

	value = ogma_read_reg(ctrl_p, OGMA_REG_ADDR_TOP_INTEN);

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return value;

}

ogma_uint32 ogma_get_top_irq_status_non_clear(ogma_handle_t ogma_handle,
					      ogma_bool mask_flag)
{
	ogma_uint32 value;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;

	if (ctrl_p == NULL) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_top_irq_status_non_clear.\n"
			    "Please set valid ogma_handle.\n");
		return 0;
	}

	/*push domain d clock */
	ogma_push_clk_req(ctrl_p, domain);

	value = ogma_read_reg(ctrl_p, OGMA_REG_ADDR_TOP_STATUS);

	if (mask_flag) {
		value &= ogma_read_reg(ctrl_p, OGMA_REG_ADDR_TOP_INTEN);
	}

	/* pop domain d clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return value;
}

ogma_err_t ogma_clear_top_irq_status(ogma_handle_t ogma_handle,
				     ogma_uint32 value)
{
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;

	if (ctrl_p == NULL) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_clear_top_irq_status.\n"
			    "Please set valid ogma_handle.\n");
		return OGMA_ERR_PARAM;
	}

	/*push domain d clock */
	ogma_push_clk_req(ctrl_p, domain);

	/* Write clear irq top  status */
	ogma_write_reg(ctrl_p,
		       OGMA_REG_ADDR_TOP_STATUS,
		       (value & OGMA_TOP_IRQ_REG_CODE_LOAD_END));

	/* pop domain d clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return OGMA_ERR_OK;
}

ogma_uint32 ogma_get_desc_ring_irq_enable(ogma_handle_t ogma_handle,
					  ogma_desc_ring_id_t ring_id)
{
	ogma_uint32 value;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;

	if (ctrl_p == NULL) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_desc_ring_irq_enable.\n"
			    "Please set valid ogma_handle.\n");
		return 0;
	}

	if (ring_id > OGMA_DESC_RING_ID_MAX) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_desc_ring_irq_enable.\n"
			    "Please select ring id number between 0 and %d.\n",
			    OGMA_DESC_RING_ID_MAX);
		return 0;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_desc_ring_irq_enable.\n"
			    "Please select valid desc ring.\n");
		return 0;
	}

	/*push domain d clock */
	ogma_push_clk_req(ctrl_p, domain);

	value = ogma_read_reg(ctrl_p, desc_ring_irq_inten_reg_addr[ring_id]);

	/* pop domain d clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return value;
}

ogma_uint32 ogma_get_desc_ring_irq_status_non_clear(ogma_handle_t ogma_handle,
						    ogma_desc_ring_id_t ring_id,
						    ogma_bool mask_flag)
{
	ogma_uint32 value;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;

	if (ctrl_p == NULL) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_desc_ring_irq_status_non_clear.\n"
			    "Please set valid ogma_handle.\n");
		return 0;
	}

	if (ring_id > OGMA_DESC_RING_ID_MAX) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_desc_ring_irq_status_non_clear.\n"
			    "Please select ring id number between 0 and %d.\n",
			    OGMA_DESC_RING_ID_MAX);
		return 0;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_desc_ring_irq_status_non_clear.\n"
			    "Please select valid desc ring.\n");
		return 0;
	}

	/* push clock */
	ogma_push_clk_req(ctrl_p, domain);

	value = ogma_read_reg(ctrl_p, desc_ring_irq_status_reg_addr[ring_id]);

	if (mask_flag) {
		value &= ogma_read_reg(ctrl_p,
				       desc_ring_irq_inten_reg_addr[ring_id]);
	}

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return value;
}

ogma_err_t ogma_clear_desc_ring_irq_status(ogma_handle_t ogma_handle,
					   ogma_desc_ring_id_t ring_id,
					   ogma_uint32 value)
{

	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;

	if (ctrl_p == NULL) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_clear_desc_ring_irq_status.\n"
			    "Please set valid ogma_handle.\n");
		return OGMA_ERR_PARAM;
	}

	if (ring_id > OGMA_DESC_RING_ID_MAX) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_clear_desc_ring_irq_status.\n"
			    "Please select ring id number between 0 and %d.\n",
			    OGMA_DESC_RING_ID_MAX);
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_clear_desc_ring_irq_status.\n"
			    "Please select valid desc ring.\n");
		return OGMA_ERR_NOTAVAIL;
	}

	/* push clock */
	ogma_push_clk_req(ctrl_p, domain);

	/* Write clear descring irq status */
	ogma_write_reg(ctrl_p,
		       desc_ring_irq_status_reg_addr[ring_id],
		       (value & (OGMA_CH_IRQ_REG_EMPTY | OGMA_CH_IRQ_REG_ERR)));

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return OGMA_ERR_OK;

}

ogma_uint32 ogma_get_hw_ver(ogma_handle_t ogma_handle)
{
	ogma_uint32 value;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;

	if (ctrl_p == NULL) {
		return 0;
	}

	/*push clock */
	ogma_push_clk_req(ctrl_p, domain);

	value = ogma_read_reg(ctrl_p, hw_ver_reg_addr);

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return value;
}

ogma_uint32 ogma_get_mcr_ver(ogma_handle_t ogma_handle)
{
	ogma_uint32 value, domain = OGMA_CLK_EN_REG_DOM_D;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;

	if (ctrl_p == NULL) {
		return 0;
	}

	/* push clock */
	ogma_push_clk_req(ctrl_p, domain);

	value = ogma_read_reg(ctrl_p, mc_ver_reg_addr);

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return value;
}
