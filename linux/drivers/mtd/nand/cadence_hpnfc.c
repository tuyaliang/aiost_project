/*
 * NAND Flash Controller Device Driver
 * Copyright © 2015, Cadence and its suppliers.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/module.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/reset.h>

#include "cadence_hpnfc.h"


MODULE_LICENSE("GPL");
#define CADENCE_NAND_NAME    "cdns-hpnfc"
/* this macro allows us to convert from an MTD structure to our own
 * device context (hpnfc) structure. */
#define mtd_to_hpnfc(m) container_of(m, struct hpnfc_state_t, mtd)

#define HPNFC_MINIMUM_SPARE_SIZE        4
#define HPNFC_MAX_SPARE_SIZE_PER_SECTOR 32

static int limited_blocks = 0;
static int init_erase = 0;
static int maxchips = 8;
module_param(limited_blocks, int, S_IRUGO);
module_param(init_erase, int, S_IRUGO);
module_param(maxchips, int, S_IRUGO);

/* PHY configurations for nf_clk = 100MHz
 * phy ctrl, phy tsel, DQ timing, DQS timing, LPBK, dll master, dll slave*/
static uint32_t phy_timings_ddr[] = {
    0x0000,  0x00, 0x02, 0x00000004, 0x00200002, 0x01140004, 0x1f1f};
static uint32_t phy_timings_ddr2[] = {
    0x4000,  0x00, 0x02, 0x00000005, 0x00380000, 0x01140004, 0x1f1f};
static uint32_t phy_timings_toggle[] = {
    0x4000,  0x00, 0x02, 0x00000004, 0x00280001, 0x01140004, 0x1f1f};
static uint32_t phy_timings_async[] = {
    0x4040, 0x00, 0x02, 0x00100004, 0x1b << 19, 0x00800000, 0x0000};

static int wait_for_thread(hpnfc_state_t *hpnfc, int8_t thread);
static int hpnfc_wait_for_idle(hpnfc_state_t *hpnfc);

/* send set features command to nand flash memory device */
static int nf_mem_set_features(hpnfc_state_t *hpnfc, uint8_t feat_addr,
                               uint8_t feat_val, uint8_t mem, uint8_t thread,
                               uint8_t vol_id)
{
    uint32_t reg;
    int status;

    /* wait for thread ready */
    status = wait_for_thread(hpnfc, thread);
    if (status)
        return status;

    reg = 0;
    WRITE_FIELD(reg, HPNFC_CMD_REG1_FADDR, feat_addr);
    WRITE_FIELD(reg, HPNFC_CMD_REG1_BANK, mem);
    /* set feature address and bank number */
    IOWR_32(hpnfc->reg + HPNFC_CMD_REG1, reg);
    /* set feature - value*/
    IOWR_32(hpnfc->reg + HPNFC_CMD_REG2, feat_val);

    reg = 0;
    /* select PIO mode */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_CT, HPNFC_CMD_REG0_CT_PIO);
    /* thread number */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_TN, thread);
    /* volume ID */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_VOL_ID, vol_id);
    /* disabled interrupt */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_INT, 0);
    /* select set feature command */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_PIO_CC, HPNFC_CMD_REG0_PIO_CC_SF);
    /* execute command */
    IOWR_32(hpnfc->reg + HPNFC_CMD_REG0, reg);

    return 0;
}


/* send reset command to nand flash memory device */
static int nf_mem_reset(hpnfc_state_t *hpnfc, uint8_t mem, uint8_t thread,
                 uint8_t vol_id)
{
    uint32_t reg = 0;
    int status;

    /* wait for thread ready */
    status = wait_for_thread(hpnfc, thread);
    if (status)
        return status;

    WRITE_FIELD(reg, HPNFC_CMD_REG1_BANK, mem);
    IOWR_32(hpnfc->reg + HPNFC_CMD_REG1, reg);

    reg = 0;

    /* select PIO mode */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_CT, HPNFC_CMD_REG0_CT_PIO);
    /* thread number */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_TN, thread);
    /* volume ID */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_VOL_ID, vol_id);
    /* disabled interrupt */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_INT, 0);
    /* select reset command */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_PIO_CC, HPNFC_CMD_REG0_PIO_CC_RST);
    /* execute command */
    IOWR_32(hpnfc->reg + HPNFC_CMD_REG0, reg);

    return 0;
}

/* function returns thread status */
static uint32_t hpnfc_get_thrd_status(hpnfc_state_t *hpnfc, uint8_t thread)
{
    uint32_t  reg;

    IOWR_32(hpnfc->reg + HPNFC_CMD_STATUS_PTR, thread);
    reg = IORD_32(hpnfc->reg + HPNFC_CMD_STATUS);
    return reg;
}


/* Wait until operation is finished  */
static int hpnfc_pio_check_finished(hpnfc_state_t *hpnfc, uint8_t thread)
{
    uint32_t  thrd_status;
    unsigned long timeout = jiffies + msecs_to_jiffies(1000);

    /* wait for fail or complete status */
    do {
        thrd_status = hpnfc_get_thrd_status(hpnfc, thread);
        thrd_status &= (HPNFC_CDMA_CS_COMP_MASK | HPNFC_CDMA_CS_FAIL_MASK);
    }while ((thrd_status == 0) && time_before(jiffies, timeout));

    if (time_after_eq(jiffies, timeout)) {
        dev_err(hpnfc->dev,"Timeout while waiting for PIO command finished\n");
        return -ETIMEDOUT;
    }

    if (thrd_status & HPNFC_CDMA_CS_FAIL_MASK)
        return -EIO;
    if (thrd_status & HPNFC_CDMA_CS_COMP_MASK)
        return 0;

    return -EIO;
}


/* checks what is the best work mode */
static void hpnfc_check_the_best_mode(hpnfc_state_t *hpnfc,
                                      uint8_t *work_mode,
                                      uint8_t *nf_timing_mode)
{
    uint32_t reg;
    *work_mode = HPNFC_WORK_MODE_ASYNC;
    *nf_timing_mode = 0;

    if (hpnfc->dev_type != HPNFC_DEV_PARAMS_0_DEV_TYPE_ONFI){
        return;
    }

    /* check if DDR is supported */
    reg = IORD_32(hpnfc->reg + HPNFC_ONFI_TIME_MOD_0);
    reg = READ_FIELD(reg, HPNFC_ONFI_TIME_MOD_0_DDR);
    if (reg)
        *work_mode = HPNFC_WORK_MODE_NV_DDR;

    /* check if DDR2 is supported */
    reg = IORD_32(hpnfc->reg + HPNFC_ONFI_TIME_MOD_1);
    reg = READ_FIELD(reg, HPNFC_ONFI_TIME_MOD_1_DDR2);
    if (reg)
        *work_mode = HPNFC_WORK_MODE_NV_DDR2;

    /* check if DDR is supported */
    reg = READ_FIELD(reg, HPNFC_ONFI_TIME_MOD_1_DDR3);
    if (reg)
        *work_mode = HPNFC_WORK_MODE_NV_DDR3;


    switch (*work_mode) {
    case HPNFC_WORK_MODE_NV_DDR:
        reg = IORD_32(hpnfc->reg + HPNFC_ONFI_TIME_MOD_0);
        reg = READ_FIELD(reg, HPNFC_ONFI_TIME_MOD_0_DDR);
        break;
    case HPNFC_WORK_MODE_NV_DDR2:
    case HPNFC_WORK_MODE_TOGG:
        reg = IORD_32(hpnfc->reg + HPNFC_ONFI_TIME_MOD_1);
        reg = READ_FIELD(reg, HPNFC_ONFI_TIME_MOD_1_DDR2);
        break;
    case HPNFC_WORK_MODE_NV_DDR3:
        reg = IORD_32(hpnfc->reg + HPNFC_ONFI_TIME_MOD_1);
        reg = READ_FIELD(reg, HPNFC_ONFI_TIME_MOD_1_DDR3);
        break;
    case HPNFC_WORK_MODE_ASYNC:
    default:
        reg = IORD_32(hpnfc->reg + HPNFC_ONFI_TIME_MOD_0);
        reg = READ_FIELD(reg, HPNFC_ONFI_TIME_MOD_0_SDR);
    }

    /* calculate from timing mode 1 */
    reg >>= 1;
    while (reg != 0) {
        reg >>= 1;
        *nf_timing_mode  += 1;
    }
}

/* set NAND flash memory device work mode */
static int nf_mem_set_work_mode(hpnfc_state_t *hpnfc, uint8_t work_mode,
                                uint8_t timing_mode)
{
    uint8_t flash_work_mode = timing_mode;
    int i, status;

    switch (work_mode) {
    case HPNFC_WORK_MODE_NV_DDR:
        flash_work_mode |= (1 << 4);
        break;
    case HPNFC_WORK_MODE_NV_DDR2:
    case HPNFC_WORK_MODE_TOGG:
        flash_work_mode |= (2 << 4);
        break;
    case HPNFC_WORK_MODE_NV_DDR3:
        flash_work_mode |= (3 << 4);
        break;
    case HPNFC_WORK_MODE_ASYNC:
    default:
        break;
    }

    /* send SET FEATURES command */
    for (i=0; i < hpnfc->devnum; i++)
        nf_mem_set_features(hpnfc, 0x01, flash_work_mode, i, i, 0);
    for (i=0; i < hpnfc->devnum; i++) {
        status = hpnfc_pio_check_finished(hpnfc, i);
        if (status)
            return status;
    }

    /* wait for controller IDLE */
    status = hpnfc_wait_for_idle(hpnfc);
    if (status)
        return status;

    return 0;
}

static void hpnfc_apply_phy_settings(hpnfc_state_t *hpnfc, uint32_t settings[])
{
    IOWR_32(hpnfc->reg + HPNFC_PHY_CTRL_REG, settings[0]);
    IOWR_32(hpnfc->reg + HPNFC_PHY_TSEL_REG, settings[1]);
    IOWR_32(hpnfc->reg + HPNFC_PHY_DQ_TIMING_REG, settings[2]);
    IOWR_32(hpnfc->reg + HPNFC_PHY_DQS_TIMING_REG, settings[3]);
    IOWR_32(hpnfc->reg + HPNFC_PHY_GATE_LPBK_CTRL_REG, settings[4]);
    IOWR_32(hpnfc->reg + HPNFC_PHY_DLL_MASTER_CTRL_REG, settings[5]);
    IOWR_32(hpnfc->reg + HPNFC_PHY_DLL_SLAVE_CTRL_REG, settings[6]);
}

/* Sets desired work mode to HPNFC controller and all NAND flash devices
 *  Each memory is processed in separate thread, starting from thread 0 */
static int hpnfc_set_work_mode(hpnfc_state_t *hpnfc, uint8_t work_mode,
                               uint8_t timing_mode)
{
    uint32_t reg;
    uint32_t dll_phy_ctrl;
    int i, status;

    reg = (IORD_32(hpnfc->reg + HPNFC_DEV_PARAMS_1)) & 0xFF;
    reg = READ_FIELD(reg, HPNFC_DEV_PARAMS_1_READID_5);
    if (reg == 0x01)
        /* exit if memory works in NV-DDR3 mode */
        return -EINVAL;


    /* set NF memory in known work mode by sending the reset command */
    reg = 0;
    /* select SDR mode in the controller */
    WRITE_FIELD(reg, HPNFC_COMMON_SETT_OPR_MODE,
                HPNFC_COMMON_SETT_OPR_MODE_SDR);
    IOWR_32(hpnfc->reg + HPNFC_COMMON_SETT, reg);

    /* select default timings */
    hpnfc_apply_phy_settings(hpnfc, phy_timings_async);

    /* send reset command to all nand flash memory devices*/
    for (i = 0; i < hpnfc->devnum; i++){
        status = nf_mem_reset(hpnfc, i, i, 0);
        if (status)
            return status;
    }
    /* wait until reset commands is finished*/
    for (i = 0; i < hpnfc->devnum; i++) {
        status = hpnfc_pio_check_finished(hpnfc, i);
        if (status)
            return status;
    }

    /* set NAND flash memory work mode */
    status = nf_mem_set_work_mode(hpnfc, work_mode, timing_mode);
    if (status)
        return status;

    /* set dll_rst_n in dll_phy_ctrl to 0 */
    dll_phy_ctrl = IORD_32(hpnfc->reg + HPNFC_DLL_PHY_CTRL);
    dll_phy_ctrl &= ~HPNFC_DLL_PHY_CTRL_DLL_RST_N_MASK;
    IOWR_32(hpnfc->reg + HPNFC_DLL_PHY_CTRL, dll_phy_ctrl );

    /* set value of other PHY registers according to PHY user guide
     * currently all values for nf_clk = 100 MHz
     * */
    switch (work_mode) {
    case HPNFC_WORK_MODE_NV_DDR:
        dev_info(hpnfc->dev, "Switch to NV_DDR mode %d \n", timing_mode);
        hpnfc_apply_phy_settings(hpnfc, phy_timings_ddr);
        break;

    case HPNFC_WORK_MODE_NV_DDR2:
        dev_info(hpnfc->dev, "Switch to NV_DDR2 mode %d \n", timing_mode);
        hpnfc_apply_phy_settings(hpnfc, phy_timings_ddr2);
        dll_phy_ctrl &= ~HPNFC_DLL_PHY_CTRL_EXTENDED_RD_MODE_MASK;
        break;

    case HPNFC_WORK_MODE_TOGG:
        dev_info(hpnfc->dev, "Switch to toggle DDR mode\n");
        hpnfc_apply_phy_settings(hpnfc, phy_timings_toggle);
        break;

    case HPNFC_WORK_MODE_ASYNC:
    default:
        dev_info(hpnfc->dev, "Switch to SDR mode %d \n", timing_mode);
        hpnfc_apply_phy_settings(hpnfc, phy_timings_async);

        reg = 0;
        WRITE_FIELD(reg, HPNFC_ASYNC_TOGGLE_TIMINGS_TRH, 2);
        WRITE_FIELD(reg, HPNFC_ASYNC_TOGGLE_TIMINGS_TRP, 4);
        WRITE_FIELD(reg, HPNFC_ASYNC_TOGGLE_TIMINGS_TWH, 2);
        WRITE_FIELD(reg, HPNFC_ASYNC_TOGGLE_TIMINGS_TWP, 4);
        IOWR_32(hpnfc->reg + HPNFC_ASYNC_TOGGLE_TIMINGS, reg);

        dll_phy_ctrl |= HPNFC_DLL_PHY_CTRL_EXTENDED_RD_MODE_MASK;
        dll_phy_ctrl |= HPNFC_DLL_PHY_CTRL_EXTENDED_WR_MODE_MASK;
    }

    /* set HPNFC controller work mode */
    reg = IORD_32(hpnfc->reg + HPNFC_COMMON_SETT);
    switch (work_mode) {
    case HPNFC_WORK_MODE_NV_DDR:
        WRITE_FIELD(reg, HPNFC_COMMON_SETT_OPR_MODE,
                    HPNFC_COMMON_SETT_OPR_MODE_NV_DDR);
        break;
    case HPNFC_WORK_MODE_TOGG:
    case HPNFC_WORK_MODE_NV_DDR2:
    case HPNFC_WORK_MODE_NV_DDR3:
        WRITE_FIELD(reg, HPNFC_COMMON_SETT_OPR_MODE,
                    HPNFC_COMMON_SETT_OPR_MODE_TOGGLE);
        break;

    case HPNFC_WORK_MODE_ASYNC:
    default:
        WRITE_FIELD(reg, HPNFC_COMMON_SETT_OPR_MODE,
                    HPNFC_COMMON_SETT_OPR_MODE_SDR);
    }
    IOWR_32(hpnfc->reg + HPNFC_COMMON_SETT, reg);

    /* set dll_rst_n in dll_phy_ctrl to 1 */
    dll_phy_ctrl |= HPNFC_DLL_PHY_CTRL_DLL_RST_N_MASK;
    IOWR_32(hpnfc->reg + HPNFC_DLL_PHY_CTRL, dll_phy_ctrl );

    /* wait for controller IDLE */
    return hpnfc_wait_for_idle(hpnfc);
}

static int hpnfc_ecc_enable(hpnfc_state_t *hpnfc, bool enable)
{
    uint32_t reg;

    reg = IORD_32(hpnfc->reg + HPNFC_ECC_CONFIG_0);

    if(enable){
        switch(hpnfc->corr_cap){
        case HPNFC_ECC_CORR_CAP_2:
            WRITE_FIELD(reg, HPNFC_ECC_CONFIG_0_CORR_STR, 0);
            break;
        case HPNFC_ECC_CORR_CAP_4:
            WRITE_FIELD(reg, HPNFC_ECC_CONFIG_0_CORR_STR, 1);
            break;
        case HPNFC_ECC_CORR_CAP_8:
            WRITE_FIELD(reg, HPNFC_ECC_CONFIG_0_CORR_STR, 2);
            break;
        case HPNFC_ECC_CORR_CAP_12:
            WRITE_FIELD(reg, HPNFC_ECC_CONFIG_0_CORR_STR, 3);
            break;
        case HPNFC_ECC_CORR_CAP_16:
            WRITE_FIELD(reg, HPNFC_ECC_CONFIG_0_CORR_STR, 4);
            break;
        case HPNFC_ECC_CORR_CAP_24:
            WRITE_FIELD(reg, HPNFC_ECC_CONFIG_0_CORR_STR, 5);
            break;
        case HPNFC_ECC_CORR_CAP_40:
            WRITE_FIELD(reg, HPNFC_ECC_CONFIG_0_CORR_STR, 6);
            break;
        }
    }

    if(enable){
        reg |= HPNFC_ECC_CONFIG_0_ECC_EN_MASK;
        reg &= ~HPNFC_ECC_CONFIG_0_ERASE_DET_EN_MASK;
    }
    else {
        reg &= ~HPNFC_ECC_CONFIG_0_ECC_EN_MASK;
        reg &= ~HPNFC_ECC_CONFIG_0_ERASE_DET_EN_MASK;
    }

    IOWR_32(hpnfc->reg + HPNFC_ECC_CONFIG_0, reg);
    IOWR_32(hpnfc->reg + HPNFC_ECC_CONFIG_1, 0);

    return 0;
}

static void hpnfc_clear_interrupt(hpnfc_state_t *hpnfc,
                                  hpnfc_irq_status_t *irq_status)
{
    IOWR_32(hpnfc->reg + HPNFC_INTR_STATUS, irq_status->status);
    IOWR_32(hpnfc->reg + HPNFC_TRD_COMP_INT_STATUS, irq_status->trd_status);
    IOWR_32(hpnfc->reg + HPNFC_TRD_ERR_INT_STATUS, irq_status->trd_error);
}

static void hpnfc_read_int_status(hpnfc_state_t *hpnfc,
                                  hpnfc_irq_status_t *irq_status)
{
    irq_status->status = IORD_32(hpnfc->reg + HPNFC_INTR_STATUS);
    irq_status->trd_status = IORD_32(hpnfc->reg + HPNFC_TRD_COMP_INT_STATUS);
    irq_status->trd_error = IORD_32(hpnfc->reg + HPNFC_TRD_ERR_INT_STATUS);
}

static inline uint32_t irq_detected(hpnfc_state_t *hpnfc,
                                    hpnfc_irq_status_t *irq_status)
{
    hpnfc_read_int_status(hpnfc, irq_status);

    return irq_status->status || irq_status->trd_status
        || irq_status->trd_error;
}

/*
 * This is the interrupt service routine. It handles all interrupts
 * sent to this device.
 */
static irqreturn_t hpnfc_isr(int irq, void *dev_id)
{
    struct hpnfc_state_t *hpnfc = dev_id;
    hpnfc_irq_status_t irq_status;
    irqreturn_t result = IRQ_NONE;

    spin_lock(&hpnfc->irq_lock);

    if (irq_detected(hpnfc, &irq_status)) {
        /* handle interrupt */
        /* first acknowledge it */
        hpnfc_clear_interrupt(hpnfc, &irq_status);
        /* store the status in the device context for someone to read */
        hpnfc->irq_status.status |= irq_status.status;
        hpnfc->irq_status.trd_status |= irq_status.trd_status;
        hpnfc->irq_status.trd_error |= irq_status.trd_error;
        /* notify anyone who cares that it happened */
        complete(&hpnfc->complete);
        /* tell the OS that we've handled this */
        result = IRQ_HANDLED;
    }
    spin_unlock(&hpnfc->irq_lock);
    return result;
}

static void wait_for_irq(struct hpnfc_state_t *hpnfc,
                         hpnfc_irq_status_t *irq_mask,
                         hpnfc_irq_status_t *irq_status)
{
    unsigned long comp_res;
    unsigned long timeout = msecs_to_jiffies(10000);

    do {
        comp_res =
            wait_for_completion_timeout(&hpnfc->complete, timeout);
        spin_lock_irq(&hpnfc->irq_lock);
        *irq_status = hpnfc->irq_status;

        if ((irq_status->status & irq_mask->status)
            || (irq_status->trd_status & irq_mask->trd_status)
            || (irq_status->trd_error & irq_mask->trd_error)
           ) {
            hpnfc->irq_status.status &= ~irq_mask->status;
            hpnfc->irq_status.trd_status &= ~irq_mask->trd_status;
            hpnfc->irq_status.trd_error &= ~irq_mask->trd_error;
            spin_unlock_irq(&hpnfc->irq_lock);
            /* our interrupt was detected */
            break;
        }

        /*
         * these are not the interrupts you are looking for; need to wait again
         */
        spin_unlock_irq(&hpnfc->irq_lock);
    } while (comp_res != 0);

    if (comp_res == 0) {
        /* timeout */
        dev_err(hpnfc->dev, "timeout occurred:"
                "\t status = 0x%x, mask = 0x%x\n"
                "\t trd_status = 0x%x, trd_status mask= 0x%x\n"
                "\t trd_error = 0x%x, trd_error mask = 0x%x\n",
                irq_status->status, irq_mask->status,
                irq_status->trd_status, irq_mask->trd_status,
                irq_status->trd_error, irq_mask->trd_error
               );

        memset(&irq_status, 0, sizeof(irq_status));
    }
}

static void hpnfc_irq_cleanup(int irqnum, struct hpnfc_state_t *hpnfc)
{
    /* disable interrupts */
    IOWR_32(hpnfc->reg + HPNFC_INTR_ENABLE, HPNFC_INTR_ENABLE_INTR_EN_MASK);
    free_irq(irqnum, hpnfc);
}

/*
 * We need to buffer some data for some of the NAND core routines.
 * The operations manage buffering that data.
 */
static void reset_buf(struct hpnfc_state_t *hpnfc)
{
    hpnfc->buf.head = hpnfc->buf.tail = 0;
    memset(&hpnfc->buf.buf[0], 0, 20);
}

static void write_byte_to_buf(struct hpnfc_state_t *hpnfc, uint8_t byte)
{
    hpnfc->buf.buf[hpnfc->buf.tail++] = byte;
}

static void write_dword_to_buf(struct hpnfc_state_t *hpnfc, uint32_t dword)
{
    memcpy(&hpnfc->buf.buf[hpnfc->buf.tail], &dword, 4);
    hpnfc->buf.tail += 4;
}

/* wait until NAND flash device is ready */
int wait_for_rb_ready(hpnfc_state_t *hpnfc)
{
    uint32_t reg;
    unsigned long timeout = jiffies + msecs_to_jiffies(1000);

    do  {
        reg = IORD_32(hpnfc->reg + HPNFC_RBN_SETTINGS);
        reg = (reg >> hpnfc->chip_nr) & 0x01;
    } while ((reg == 0) && time_before(jiffies, timeout));

    if (time_after_eq(jiffies, timeout)) {
        dev_err(hpnfc->dev,
                "Timeout while waiting for flash device %d ready\n",
                hpnfc->chip_nr);
        return -ETIMEDOUT;
    }
    return 0;
}

static int wait_for_thread(hpnfc_state_t *hpnfc, int8_t thread)
{
    uint32_t reg;
    unsigned long timeout = jiffies + msecs_to_jiffies(1000);

    do  {
        /* get busy status of all threads */
        reg = IORD_32(hpnfc->reg + HPNFC_TRD_STATUS);
        /* mask all threads but selected */
        reg &=  (1 << thread);
    } while (reg && time_before(jiffies, timeout));

    if (time_after_eq(jiffies, timeout)) {
        dev_err(hpnfc->dev, "Timeout while waiting for thread  %d\n", thread);
        return -ETIMEDOUT;
    }

    return 0;
}

static int hpnfc_wait_for_idle(hpnfc_state_t *hpnfc)
{
    uint32_t reg;
    unsigned long timeout = jiffies + msecs_to_jiffies(1000);

    do  {
        reg = IORD_32(hpnfc->reg + HPNFC_CTRL_STATUS);
    } while ((reg & HPNFC_CTRL_STATUS_CTRL_BUSY_MASK)
             && time_before(jiffies, timeout));

    if (time_after_eq(jiffies, timeout)) {
        dev_err(hpnfc->dev, "Timeout while waiting for controller idle\n");
        return -ETIMEDOUT;
    }

    return 0;
}

/*  This function waits for device initialization */
static int wait_for_init_complete(hpnfc_state_t *hpnfc)
{
    uint32_t reg;
    unsigned long timeout = jiffies + msecs_to_jiffies(10000);

    do  {/* get ctrl status register */
        reg = IORD_32(hpnfc->reg + HPNFC_CTRL_STATUS);
    } while (((reg & HPNFC_CTRL_STATUS_INIT_COMP_MASK) == 0)
             && time_before(jiffies, timeout));

    if (time_after_eq(jiffies, timeout)) {
        dev_err(hpnfc->dev,
                "Timeout while waiting for controller init complete\n");
        return -ETIMEDOUT;
    }

    return 0;
}

/* execute generic command on HPNFC controller */
static int hpnfc_generic_cmd_send(hpnfc_state_t *hpnfc, uint64_t mini_ctrl_cmd,
                                  uint8_t use_intr)
{
    uint32_t reg = 0;
    uint8_t status, thread_nr = hpnfc->chip_nr;

    uint32_t mini_ctrl_cmd_l = mini_ctrl_cmd & 0xFFFFFFFF;
    uint32_t mini_ctrl_cmd_h = mini_ctrl_cmd >> 32;

    status = wait_for_thread(hpnfc, thread_nr);
    if (status)
        return status;

    IOWR_32(hpnfc->reg + HPNFC_CMD_REG2, mini_ctrl_cmd_l);
    IOWR_32(hpnfc->reg + HPNFC_CMD_REG3, mini_ctrl_cmd_h);

    /* select generic command */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_CT, HPNFC_CMD_REG0_CT_GEN);
    /* thread number */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_TN, thread_nr);
    if (use_intr)
        reg |= HPNFC_CMD_REG0_INT_MASK;

    /* issue command */
    IOWR_32(hpnfc->reg + HPNFC_CMD_REG0, reg);

    return 0;
}

/* preapre generic command on  HPNFC controller  */
static int hpnfc_generic_cmd_command(hpnfc_state_t *hpnfc, uint32_t command,
                                     uint64_t addr, uint8_t use_intr)
{
    uint64_t mini_ctrl_cmd = 0;

    switch(command){
    case HPNFC_GCMD_LAY_INSTR_RDPP:
        mini_ctrl_cmd |= HPNFC_GCMD_LAY_TWB_MASK;
        break;
    case HPNFC_GCMD_LAY_INSTR_RDID:
        mini_ctrl_cmd |= HPNFC_GCMD_LAY_TWB_MASK;
        break;
    default:
        break;
    }

    WRITE_FIELD64(mini_ctrl_cmd, HPNFC_GCMD_LAY_INSTR, command);

    WRITE_FIELD64(mini_ctrl_cmd, HPNFC_GCMD_LAY_CS, hpnfc->chip_nr);
    WRITE_FIELD64(mini_ctrl_cmd, HPNFC_GCMD_LAY_INPUT_ADDR0, addr);

    /* send command */
    return hpnfc_generic_cmd_send(hpnfc, mini_ctrl_cmd, use_intr);
}



/* preapre generic command used to data transfer on  HPNFC controller  */
static int hpnfc_generic_cmd_data(hpnfc_state_t *hpnfc,
                                  generic_data_t *generic_data)
{
    uint64_t mini_ctrl_cmd = 0;

    WRITE_FIELD64(mini_ctrl_cmd, HPNFC_GCMD_LAY_CS, hpnfc->chip_nr);
    WRITE_FIELD64(mini_ctrl_cmd, HPNFC_GCMD_LAY_INSTR,
                  HPNFC_GCMD_LAY_INSTR_DATA);
    WRITE_FIELD64(mini_ctrl_cmd, HPNFC_GCMD_DIR, generic_data->direction);

    if (generic_data->ecc_en)
        mini_ctrl_cmd |= HPNFC_GCMD_ECC_EN_MASK;

    if (generic_data->scr_en)
        mini_ctrl_cmd |= HPNFC_GCMD_SCR_EN_MASK;

    if (generic_data->erpg_en)
        mini_ctrl_cmd |= HPNFC_GCMD_ERPG_EN_MASK;

    WRITE_FIELD64(mini_ctrl_cmd, HPNFC_GCMD_SECT_SIZE,
                  (uint64_t)generic_data->sec_size);
    WRITE_FIELD64(mini_ctrl_cmd, HPNFC_GCMD_SECT_CNT,
                  (uint64_t)generic_data->sec_cnt);
    WRITE_FIELD64(mini_ctrl_cmd, HPNFC_GCMD_LAST_SIZE,
                  (uint64_t)generic_data->last_sec_size);
    WRITE_FIELD64(mini_ctrl_cmd, HPNFC_GCMD_CORR_CAP,
                  (uint64_t)generic_data->corr_cap);

    return hpnfc_generic_cmd_send(hpnfc, mini_ctrl_cmd,
                                  generic_data->use_intr);
}

/* wait for data on slave dma interface */
static int hpnfc_wait_on_sdma_trigg(hpnfc_state_t *hpnfc,
                                    uint8_t* out_sdma_trd,
                                    uint32_t* out_sdma_size)
{
    hpnfc_irq_status_t irq_mask, irq_status;

    irq_mask.trd_status = 0;
    irq_mask.trd_error = 0;
    irq_mask.status = HPNFC_INTR_STATUS_SDMA_TRIGG_MASK
        | HPNFC_INTR_STATUS_SDMA_ERR_MASK
        | HPNFC_INTR_STATUS_UNSUPP_CMD_MASK;

    wait_for_irq(hpnfc, &irq_mask, &irq_status);
    if (irq_status.status == 0){
        dev_err(hpnfc->dev, "Timeout while waiting for SDMA\n");
        return -ETIMEDOUT;
    }

    if (irq_status.status & HPNFC_INTR_STATUS_SDMA_TRIGG_MASK){
        *out_sdma_size = IORD_32(hpnfc->reg + HPNFC_SDMA_SIZE);
        *out_sdma_trd  = IORD_32(hpnfc->reg + HPNFC_SDMA_TRD_NUM);
        *out_sdma_trd = READ_FIELD(*out_sdma_trd, HPNFC_SDMA_TRD_NUM_SDMA_TRD);
    }
    else {
        dev_err(hpnfc->dev, "SDMA error - irq_status %x\n", irq_status.status);
        return -EIO;
    }

    return 0;
}

/* read data from slave DMA interface */
static int dma_read_data(hpnfc_state_t *hpnfc, uint32_t *buf, uint32_t size)
{
    int i;

    if (size & 3){
        return -1;
    }
    for(i = 0; i < size / 4; i++){
        *buf = IORD_32(hpnfc->slave_dma);
        buf++;
    }

    return 0;
}

static void hpnfc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
    int status;
    uint8_t sdmatrd_num;
    uint32_t sdma_size;
    generic_data_t generic_data;
    uint32_t tmp[2];
    uint8_t sub_size = 4;
    int i = 0;
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);

    memset(&generic_data, 0, sizeof(generic_data_t));
    generic_data.sec_cnt = 1;
    generic_data.last_sec_size = sub_size;
    generic_data.direction = HPNFC_GCMD_DIR_READ;

    /* read is made by 4 bytes because of platform limitation */
    while (len){
        /* send change column command */
        status = hpnfc_generic_cmd_command(hpnfc,
                                           HPNFC_GCMD_LAY_INSTR_CHRC,
                                           hpnfc->offset,
                                           0);
        if (status)
            return;

        /* wait for finishing operation */
        status = wait_for_rb_ready(hpnfc);
        if (status)
            return;

        /* prepare controller to read data in generic mode */
        status = hpnfc_generic_cmd_data(hpnfc, &generic_data);
        if (status)
            return;

        /* wait until data is read on slave DMA */
        status = hpnfc_wait_on_sdma_trigg(hpnfc, &sdmatrd_num, &sdma_size);
        if (status)
            return;

        /* read data  */
        status = dma_read_data(hpnfc, tmp, sdma_size);
        if (status)
            return;

        if (len < sub_size)
            sub_size = len;

        memcpy(buf + i, &tmp[0], sub_size);

        len -= sub_size;
        hpnfc->offset += sub_size;
        i += sub_size;
    }
}

/* read parameter page */
static int read_parameter_page(hpnfc_state_t *hpnfc, uint32_t size)
{
    int status;
    uint8_t sdmatrd_num;
    uint32_t sdma_size;
    generic_data_t generic_data;
    uint32_t tmp[2];
    const uint8_t sub_size = 4;
    uint32_t offset = 0;

    memset(&generic_data, 0, sizeof(generic_data_t));
    generic_data.sec_cnt = 1;
    generic_data.last_sec_size = sub_size;
    generic_data.direction = HPNFC_GCMD_DIR_READ;

    /* execute read parameter page instruction */
    status = hpnfc_generic_cmd_command(hpnfc, HPNFC_GCMD_LAY_INSTR_RDPP, 0, 0);
    if (status)
        return status;

    /* wait for finishing operation */
    status = wait_for_rb_ready(hpnfc);
    if (status)
        return status;

    /* read is made by 4 bytes because of platform limitation */
    while (size){
        /* send change column command */
        status = hpnfc_generic_cmd_command(hpnfc,
                                           HPNFC_GCMD_LAY_INSTR_CHRC, offset,
                                           0);
        if (status)
            return status;

        /* wait for finishing operation */
        status = wait_for_rb_ready(hpnfc);
        if (status)
            return status;

        /* prepare controller to read data in generic mode */
        status = hpnfc_generic_cmd_data(hpnfc, &generic_data);
        if (status)
            return status;

        /* wait until data is read on slave DMA */
        status = hpnfc_wait_on_sdma_trigg(hpnfc, &sdmatrd_num, &sdma_size);
        if (status)
            return status;

        /* read data (part of parameter page) */
        status = dma_read_data(hpnfc, tmp, sdma_size);
        if (status)
            return status;

        write_dword_to_buf(hpnfc, tmp[0]);

        size -= sub_size;
        offset += sub_size;
    }

    return 0;
}

/* read id of memory */
static int nf_mem_read_id(hpnfc_state_t *hpnfc, uint8_t address, uint32_t size)
{
    int status;
    uint8_t sdmatrd_num;
    uint32_t sdma_size;
    generic_data_t generic_data;
    uint32_t tmp[2];

    memset(&generic_data, 0, sizeof(generic_data_t));
    generic_data.sec_cnt = 1;
    generic_data.last_sec_size = size;
    generic_data.direction = HPNFC_GCMD_DIR_READ;

    /* execute read ID instruction */
    status = hpnfc_generic_cmd_command(hpnfc, HPNFC_GCMD_LAY_INSTR_RDID,
                                       address, 0);
    if (status)
        return status;

    /* wait for finishing operation */
    status = wait_for_rb_ready(hpnfc);
    if (status)
        return status;

    /* prepare controller to read data in generic mode */
    status = hpnfc_generic_cmd_data(hpnfc, &generic_data);
    if (status)
        return status;

    /* wait until data is read on slave DMA */
    status = hpnfc_wait_on_sdma_trigg(hpnfc, &sdmatrd_num, &sdma_size);
    if (status)
        return status;

    /* read data (flash id) */
    status = dma_read_data(hpnfc, tmp, sdma_size);
    if (status)
        return status;

    write_dword_to_buf(hpnfc, tmp[0]);

    return 0;
}

static void nand_par_set(hpnfc_state_t *hpnfc) 
{

    /* nand par setting */
    /* SYPRZESS */
    printk("hpnfc_dev_info() mid=0x%x, did=0x%x \n",hpnfc->mid, hpnfc->did);
    if (hpnfc->mid == 0x01) {	/* MID check */
        /* S34MS01G2_04G2 */
        if (hpnfc->did == 0xac) {	/* DID check */

            IOWR_32(hpnfc->reg + HPNFC_COMMON_SETT, (uint32_t)0x00000000);
            IOWR_32(hpnfc->reg + HPNFC_TOGGLE_TIMINGS_0, (uint32_t)0x01020a03);
            IOWR_32(hpnfc->reg + HPNFC_TOGGLE_TIMINGS_1, (uint32_t)0x0b020203);
            IOWR_32(hpnfc->reg + HPNFC_ASYNC_TOGGLE_TIMINGS, (uint32_t)0x02040203);
            IOWR_32(hpnfc->reg + HPNFC_SYNC_TIMINGS, (uint32_t)0x00040205);
            IOWR_32(hpnfc->reg + HPNFC_TIMINGS0, (uint32_t)0x0a32060a);
            IOWR_32(hpnfc->reg + HPNFC_TIMINGS1, (uint32_t)0x0a0c0505);
            IOWR_32(hpnfc->reg + HPNFC_TIMINGS2, (uint32_t)0x00640104);
            IOWR_32(hpnfc->reg + HPNFC_DLL_PHY_CTRL, (uint32_t)0x01030300);
            IOWR_32(hpnfc->reg + HPNFC_PHY_DQ_TIMING_REG, (uint32_t)0x00000002);
            IOWR_32(hpnfc->reg + HPNFC_PHY_DQS_TIMING_REG, (uint32_t)0x00100004);
            IOWR_32(hpnfc->reg + HPNFC_PHY_GATE_LPBK_CTRL_REG, (uint32_t)0x00380001);
            IOWR_32(hpnfc->reg + HPNFC_PHY_DLL_MASTER_CTRL_REG, (uint32_t)0x00800000);
            IOWR_32(hpnfc->reg + HPNFC_PHY_DLL_SLAVE_CTRL_REG, (uint32_t)0x00000000);
            IOWR_32(hpnfc->reg + HPNFC_PHY_CTRL_REG, (uint32_t)0x00000040);
            IOWR_32(hpnfc->reg + HPNFC_PHY_TSEL_REG, (uint32_t)0x00000000);

            IOWR_32(hpnfc->reg + HPNFC_DLL_UPDATE_CNT, (uint32_t)HPNFC_DLL_PHY_UPDATE_CNT_VAL);
        }
    }
    return;
}

/* obtain information about connected  */
static int hpnfc_dev_info(hpnfc_state_t *hpnfc)
{
    uint32_t  reg;

    reg = IORD_32(hpnfc->reg + HPNFC_DEV_PARAMS_0);
    hpnfc->dev_type = READ_FIELD(reg, HPNFC_DEV_PARAMS_0_DEV_TYPE);

    switch (hpnfc->dev_type) {
    case HPNFC_DEV_PARAMS_0_DEV_TYPE_ONFI:
        dev_info(hpnfc->dev, "Detected ONFI device:\n");
        break;
    case HPNFC_DEV_PARAMS_0_DEV_TYPE_JEDEC:
        dev_info(hpnfc->dev, "Detected JEDEC device:\n");
        break;
    default:
        dev_warn(hpnfc->dev, "Error: Device was not detected correctly.\n");
        return -1;
    }

    reg = IORD_32(hpnfc->reg + HPNFC_MANUFACTURER_ID);

    dev_info(hpnfc->dev, "-- Manufacturer ID: 0x%lx\n",
             READ_FIELD(reg, HPNFC_MANUFACTURER_ID_MID));

    dev_info(hpnfc->dev, "-- Device ID: 0x%lx\n",
             READ_FIELD(reg, HPNFC_MANUFACTURER_ID_DID));

    hpnfc->mid = READ_FIELD(reg, HPNFC_MANUFACTURER_ID_MID);
    hpnfc->did = READ_FIELD(reg, HPNFC_MANUFACTURER_ID_DID);

    reg =  IORD_32(hpnfc->reg + HPNFC_DEV_REVISION);
    dev_info(hpnfc->dev, "-- Device revisions codeword: 0x%x\r\n",
             reg & 0xffff );

    reg =  IORD_32(hpnfc->reg + HPNFC_NF_DEV_AREAS);
    hpnfc->mtd.oobsize = hpnfc->spare_size =
        READ_FIELD(reg, HPNFC_NF_DEV_AREAS_SPARE_SIZE);
    hpnfc->main_size = hpnfc->mtd.writesize =
        READ_FIELD(reg, HPNFC_NF_DEV_AREAS_MAIN_SIZE);
    dev_info(hpnfc->dev, "-- Page main area size: %d\n", hpnfc->main_size);
    dev_info(hpnfc->dev, "-- Page spare area size: %d\n", hpnfc->spare_size);

    reg =  IORD_32(hpnfc->reg + HPNFC_NF_DEV_LAYOUT);
    hpnfc->mtd.erasesize = READ_FIELD(reg, HPNFC_NF_DEV_LAYOUT_PPB)
        * hpnfc->main_size;

    reg =  IORD_32(hpnfc->reg + HPNFC_NF_DEV_LAYOUT);

    hpnfc->lun_count = READ_FIELD(reg, HPNFC_NF_DEV_LAYOUT_LN);

    reg =  IORD_32(hpnfc->reg + HPNFC_DEV_BLOCKS_PER_LUN);
    hpnfc->blocks_per_lun = reg;


    if (hpnfc->nand.numchips > 0) {
        hpnfc->devnum = hpnfc->nand.numchips;
    }
    hpnfc->chip_nr = 0;

    if (limited_blocks){
        hpnfc->lun_count = 1;
        hpnfc->blocks_per_lun = limited_blocks;
    }

    hpnfc->nand.chipsize = hpnfc->mtd.erasesize * hpnfc->blocks_per_lun
        * hpnfc->lun_count;


    hpnfc->mtd.size = hpnfc->nand.chipsize * hpnfc->devnum;

    hpnfc->nand.page_shift = ffs((unsigned)hpnfc->mtd.writesize) - 1;
    hpnfc->nand.pagemask = (hpnfc->nand.chipsize >>
                            hpnfc->nand.page_shift) - 1;
    hpnfc->nand.bbt_erase_shift = ffs((unsigned)hpnfc->mtd.erasesize) - 1;
    hpnfc->nand.phys_erase_shift = ffs((unsigned)hpnfc->mtd.erasesize) - 1;
    hpnfc->nand.chip_shift = ffs((unsigned)hpnfc->nand.chipsize) - 1;

    return 0;
}

/* preapre CDMA descriptor */
static void hpnfc_cdma_desc_prepare(hpnfc_cdma_desc_t* cdma_desc, char nf_mem,
                                    uint32_t flash_ptr, char* mem_ptr,
                                    uint16_t ctype, uint32_t cmd_cnt)
{
    memset(cdma_desc, 0, sizeof(hpnfc_cdma_desc_t));

    /* set fields for one descriptor */
    cdma_desc->flash_pointer = (nf_mem << HPNFC_CDMA_CFPTR_MEM_SHIFT)
        + flash_ptr;
    cdma_desc->command_flags |= HPNFC_CDMA_CF_DMA_MASTER;
    cdma_desc->command_flags  |= HPNFC_CDMA_CF_INT;

    cdma_desc->memory_pointer= (uintptr_t)mem_ptr;
    cdma_desc->status = 0;
    cdma_desc->sync_flag_pointer = 0;
    cdma_desc->sync_arguments = 0;

    cdma_desc->command_type = ctype;

    switch(ctype){
    case HPNFC_CDMA_CT_ERASE:
    case HPNFC_CDMA_CT_CPYB:
    case HPNFC_CDMA_CT_WR:
    case HPNFC_CDMA_CT_RD:
        cdma_desc->command_type |= (cmd_cnt - 1) & 0xff;
        break;
    }
}

static uint8_t hpnfc_check_desc_error(uint32_t desc_status)
{
    if (desc_status & HPNFC_CDMA_CS_ERP_MASK)
        return HPNFC_STAT_ERASED;

    if (desc_status & HPNFC_CDMA_CS_UNCE_MASK)
        return HPNFC_STAT_ECC_UNCORR;


    if (desc_status & HPNFC_CDMA_CS_ERR_MASK) {
        pr_err(CADENCE_NAND_NAME ":CDMA descriptor error flag detected.\n");
        return HPNFC_STAT_FAIL;
    }

    if (READ_FIELD(desc_status, HPNFC_CDMA_CS_MAXERR)){
        return HPNFC_STAT_ECC_CORR;
    }

    return HPNFC_STAT_FAIL;
}


static int hpnfc_wait_cdma_finish(hpnfc_cdma_desc_t* cdma_desc)
{
    hpnfc_cdma_desc_t *desc_ptr;
    uint8_t status = HPNFC_STAT_BUSY;

    desc_ptr = cdma_desc;

    do {
        if (desc_ptr->status & HPNFC_CDMA_CS_FAIL_MASK) {
            status = hpnfc_check_desc_error(desc_ptr->status);
            //pr_info(CADENCE_NAND_NAME ":CDMA error %x\n", desc_ptr->status);
            break;
        }
        if (desc_ptr->status & HPNFC_CDMA_CS_COMP_MASK) {
            /* descriptor finished with no errors */
            if (desc_ptr->command_flags & HPNFC_CDMA_CF_CONT) {
                /* not last descriptor */
                desc_ptr =
                    (hpnfc_cdma_desc_t*)(uintptr_t)desc_ptr->next_pointer;
            } else {
                /* last descriptor  */
                status = HPNFC_STAT_OK;
            }
        }
    } while (status == HPNFC_STAT_BUSY);

    return status;
}

static int hpnfc_cdma_send(hpnfc_state_t *hpnfc, uint8_t thread)
{
    uint32_t reg = 0;
    int status;

    /* wait for thread ready*/
    status = wait_for_thread(hpnfc, thread);
    if (status)
        return status;

    IOWR_32(hpnfc->reg + HPNFC_CMD_REG2, (uint32_t )hpnfc->dma_cdma_desc);
    IOWR_32(hpnfc->reg + HPNFC_CMD_REG3, 0 );

    /* select CDMA mode */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_CT, HPNFC_CMD_REG0_CT_CDMA);
    /* thread number */
    WRITE_FIELD(reg, HPNFC_CMD_REG0_TN, thread);
    /* issue command */
    IOWR_32(hpnfc->reg + HPNFC_CMD_REG0, reg);

    return 0;
}

/* send SDMA command and wait for finish */
static uint32_t hpnfc_cdma_send_and_wait(hpnfc_state_t *hpnfc, uint8_t thread)
{
    int status;
    hpnfc_irq_status_t irq_mask, irq_status;

    status = hpnfc_cdma_send(hpnfc, thread);
    if (status)
        return status;

    irq_mask.trd_status = 1 << thread;
    irq_mask.trd_error = 1 << thread;
    irq_mask.status = HPNFC_INTR_STATUS_CDMA_TERR_MASK;
    wait_for_irq(hpnfc, &irq_mask, &irq_status);

    if ((irq_status.status == 0) && (irq_status.trd_status == 0)
        && (irq_status.trd_error == 0)){
        dev_err(hpnfc->dev, "CDMA command timeout\n");
        return -ETIMEDOUT;
    }
    if (irq_status.status & irq_mask.status){
        dev_err(hpnfc->dev, "CDMA command failed\n");
        return -EIO;
    }

    return 0;
}

/* HPNFC hardware initialization */
static int hpnfc_hw_init(hpnfc_state_t *hpnfc)
{
    uint32_t tmp;
    uint32_t int_settings;
    int status;

    status = wait_for_init_complete(hpnfc);

    if (status) {
        return status;
    }

    tmp = HPNFC_DMA_SETTINGS_OTE_MASK;
    WRITE_FIELD(tmp, HPNFC_DMA_SETTINGS_BURST_SEL, 31);

    IOWR_32(hpnfc->reg + HPNFC_DMA_SETTINGS, tmp);

    /* disable cache and multiplane */

    IOWR_32(hpnfc->reg + HPNFC_MULTIPLANE_CFG, 0);

    IOWR_32(hpnfc->reg + HPNFC_CACHE_CFG, 0);

    /* enable interrupts */
    int_settings = HPNFC_INTR_ENABLE_INTR_EN_MASK
        | HPNFC_INTR_ENABLE_CDMA_TERR_EN_MASK
        | HPNFC_INTR_ENABLE_DDMA_TERR_EN_MASK
        | HPNFC_INTR_ENABLE_UNSUPP_CMD_EN_MASK
        | HPNFC_INTR_ENABLE_SDMA_TRIGG_EN_MASK
        | HPNFC_INTR_ENABLE_SDMA_ERR_EN_MASK;


    IOWR_32(hpnfc->reg + HPNFC_INTR_ENABLE, int_settings);
    /* enable signaling thread error interrupts for all threads  */


    IOWR_32(hpnfc->reg + HPNFC_TRD_ERR_INT_STATUS_EN, 0x0F);

    return 0;
}


/* skipbyte count and offset */ 
static void hpnfc_set_skipbyte_count_offset(struct hpnfc_state_t *hpnfc, int page)
{
    uint32_t reg;
    uint32_t skipbyte_offset, skipbyte_count;
    int32_t pages_paer_block;

    pages_paer_block = 1 << (hpnfc->nand.phys_erase_shift - hpnfc->nand.page_shift);

    if (page < pages_paer_block) {
        skipbyte_count = 0;
        skipbyte_offset = 0;
   }
    else {
        skipbyte_count = HPNFC_SKIP_BYTE_COUNT_VAL;
        skipbyte_offset = hpnfc->main_size;
    }

/* Skipbyte count setting */
    reg = IORD_32(hpnfc->reg + HPNFC_SKIP_BYTE_COUNT);
    reg &= 0xffffff00;
    reg |= skipbyte_count;
    reg |= 0xffff0000;

    IOWR_32(hpnfc->reg + HPNFC_SKIP_BYTE_COUNT, reg);

 /* Skipbyte offset setting */
   IOWR_32(hpnfc->reg + HPNFC_SKIP_BYTE_OFFSET, skipbyte_offset);
}

/* calculate size of check bit size per one sector */
int bch_calculate_ecc_size(struct hpnfc_state_t *hpnfc,
                           uint32_t *check_bit_size)
{
    uint8_t mult;
    uint32_t corr_cap, tmp;

    *check_bit_size = 0;

    switch(hpnfc->sector_size){
    case HPNFC_ECC_SEC_SIZE_1024:
        mult = 14;
        break;
    default:
        dev_err(hpnfc->dev,
                "Wrong ECC configuration, ECC sector size:%d"
                "is not supported\n", hpnfc->sector_size);
        return -1;
    }

    switch(hpnfc->corr_cap){
    case HPNFC_ECC_CORR_CAP_2:
        corr_cap = 2;
        break;
    case HPNFC_ECC_CORR_CAP_4:
        corr_cap = 4;
        break;
    case HPNFC_ECC_CORR_CAP_8:
        corr_cap = 8;
        break;
    case HPNFC_ECC_CORR_CAP_12:
        corr_cap = 12;
        break;
    case HPNFC_ECC_CORR_CAP_16:
        corr_cap = 16;
        break;
    case HPNFC_ECC_CORR_CAP_24:
        corr_cap = 24;
        break;
    case HPNFC_ECC_CORR_CAP_40:
        corr_cap = 40;
        break;

    default:
        dev_err(hpnfc->dev,
                "Wrong ECC configuration, correction capability:%d"
                "is not supported\n", hpnfc->corr_cap);
        return -1;
    }

    tmp = (mult * corr_cap) / 16;
    /* round up */
    if ((tmp * 16)< (mult * corr_cap)){
        tmp++;
    }

    /* check bit size per one sector */
    *check_bit_size = 2 * tmp;

    return 0;
}

#define TT_SPARE_AREA           1
#define TT_MAIN_SPARE_AREAS     2
#define TT_RAW_SPARE_AREA       3

/* preapre size of data to transfer */
static int hpnfc_prepare_data_size(hpnfc_state_t *hpnfc, int transfer_type)
{
    uint32_t reg = 0;
    uint32_t sec_size = 0, last_sec_size, offset, sec_cnt;
    uint32_t ecc_size = hpnfc->nand.ecc.bytes;

    if (hpnfc->curr_trans_type == transfer_type){
        return 0;
    }

    switch (transfer_type) {
    case TT_SPARE_AREA:
        offset = hpnfc->main_size - hpnfc->sector_size;
        ecc_size = ecc_size * (offset / hpnfc->sector_size);
        offset = offset + ecc_size;
        sec_cnt = 1;
        last_sec_size = hpnfc->sector_size + hpnfc->usnused_spare_size;
        break;
    case TT_MAIN_SPARE_AREAS:
        offset = 0;
        sec_cnt = hpnfc->sector_count;
        last_sec_size = hpnfc->sector_size;
        sec_size = hpnfc->sector_size;
        break;
    case TT_RAW_SPARE_AREA:
        offset = hpnfc->main_size;
        sec_cnt = 1;
        last_sec_size = hpnfc->usnused_spare_size;
        break;
    default:
        dev_err(hpnfc->dev, "Data size preparation failed \n");
        return -EINVAL;
    }

    reg = 0;
    WRITE_FIELD(reg, HPNFC_TRAN_CFG_0_OFFSET, offset);
    WRITE_FIELD(reg, HPNFC_TRAN_CFG_0_SEC_CNT, sec_cnt);
    IOWR_32(hpnfc->reg + HPNFC_TRAN_CFG_0, reg);

    reg = 0;
    WRITE_FIELD(reg, HPNFC_TRAN_CFG_1_LAST_SEC_SIZE, last_sec_size);
    WRITE_FIELD(reg, HPNFC_TRAN_CFG_1_SECTOR_SIZE, sec_size);
    IOWR_32(hpnfc->reg + HPNFC_TRAN_CFG_1, reg);

   return 0;
}

/* write data from flash memory using CDMA command */
static int cdma_write_data(struct mtd_info *mtd, int page,  bool with_ecc)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);
    dma_addr_t dma_buf = hpnfc->buf.dma_buf;
    hpnfc_cdma_desc_t *cdma_desc = hpnfc->cdma_desc;
    uint8_t thread_nr = hpnfc->chip_nr;
    int status = 0;

    hpnfc_ecc_enable(hpnfc, with_ecc);

    dma_sync_single_for_device(hpnfc->dev, dma_buf,
                               hpnfc->main_size + mtd->oobsize, DMA_TO_DEVICE);

    hpnfc_cdma_desc_prepare(cdma_desc, hpnfc->chip_nr, page, (void*)dma_buf,
                            HPNFC_CDMA_CT_WR, 1);

    status = hpnfc_cdma_send_and_wait(hpnfc, thread_nr);

    dma_sync_single_for_cpu(hpnfc->dev, dma_buf,
                            hpnfc->main_size + mtd->oobsize, DMA_TO_DEVICE);
    if (status)
        return status;

    status = hpnfc_wait_cdma_finish(hpnfc->cdma_desc);
    if (status == HPNFC_STAT_ECC_CORR){
        dev_err(hpnfc->dev, "CDMA write operation failed\n");
        return -EIO;
    }

    return 0;
}

/* get corrected ECC errors of last read operation */
static uint32_t get_ecc_count(hpnfc_state_t *hpnfc)
{
    return READ_FIELD(hpnfc->cdma_desc->status, HPNFC_CDMA_CS_MAXERR);
}

/* read data from flash memory using CDMA command */
static int cdma_read_data(struct mtd_info *mtd, int page, bool with_ecc,
                          uint32_t *ecc_err_count)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);
    dma_addr_t dma_buf = hpnfc->buf.dma_buf;
    hpnfc_cdma_desc_t *cdma_desc = hpnfc->cdma_desc;
    uint8_t thread_nr = hpnfc->chip_nr;
    int status;

    hpnfc_ecc_enable(hpnfc, with_ecc);

    dma_sync_single_for_device(hpnfc->dev, dma_buf,
                               hpnfc->main_size + mtd->oobsize,
                               DMA_FROM_DEVICE);

    hpnfc_cdma_desc_prepare(cdma_desc, hpnfc->chip_nr, page, (void*)dma_buf,
                            HPNFC_CDMA_CT_RD, 1);

    status = hpnfc_cdma_send_and_wait(hpnfc, thread_nr);

    dma_sync_single_for_cpu(hpnfc->dev, dma_buf,
                            hpnfc->main_size + mtd->oobsize, DMA_FROM_DEVICE);

    /* recover ecc settings */
    hpnfc_ecc_enable(hpnfc, hpnfc->ecc_enabled);

    status = hpnfc_wait_cdma_finish(hpnfc->cdma_desc);
    if (status == HPNFC_STAT_ECC_CORR){
        *ecc_err_count = get_ecc_count(hpnfc);
    }

    return status;
}

/* writes OOB data to the device */
static int write_oob_data(struct mtd_info *mtd, uint8_t *buf, int page)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);
    int status = 0;

    hpnfc->page = page;

    hpnfc_set_skipbyte_count_offset(hpnfc, page);

    /* to protect spare data by ECC
     * we send also one ECC sector set to 0xFF */
    memcpy(hpnfc->buf.buf, buf, mtd->oobsize);


    status = hpnfc_prepare_data_size(hpnfc, TT_RAW_SPARE_AREA);
    if(status){
        dev_err(hpnfc->dev, "write oob failed\n");
        return status;
    }

    return cdma_write_data(mtd, page, 0);
}

/* reads OOB data from the device */
static int read_oob_data(struct mtd_info *mtd, uint8_t *buf, int page)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);
    int status = 0;

    struct nand_chip *chip = hpnfc->mtd.priv;
    int i;
    int pages_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);

    hpnfc_set_skipbyte_count_offset(hpnfc, page);

    hpnfc->page = page;

    if ((hpnfc->page >= 0) && (hpnfc->page < pages_per_block)) {
            for (i = 0; i < mtd->oobsize; i++) {
            *buf++ = 0xff;
        }
        return 0;
    }


    status = hpnfc_prepare_data_size(hpnfc, TT_SPARE_AREA);
    if(status){
        return -EIO;
    }
    status = cdma_read_data(mtd, page, 1, NULL);

    switch(status){
    case HPNFC_STAT_ERASED:
    case HPNFC_STAT_ECC_UNCORR:
        status = hpnfc_prepare_data_size(hpnfc, TT_RAW_SPARE_AREA);
        if(status){
            return -EIO;
        }
        status = cdma_read_data(mtd, page, 0, NULL);
        if (status){
            dev_err(hpnfc->dev, "read oob failed\n");
        }
        memcpy(buf, hpnfc->buf.buf, mtd->oobsize);
        break;
    case HPNFC_STAT_OK:
    case HPNFC_STAT_ECC_CORR:
        memcpy(buf, hpnfc->buf.buf + hpnfc->sector_size, mtd->oobsize);
        break;
    default:
        dev_err(hpnfc->dev, "read oob failed\n");
        return -EIO;
    }

    return 0;
}

/*
 * this function examines buffers to see if they contain data that
 * indicate that the buffer is part of an erased region of flash.
 */
static bool is_erased(uint8_t *buf, int len)
{
    int i;

    for (i = 0; i < len; i++)
        if (buf[i] != 0xFF){
            return false;
        }
    return true;
}


/*
 * writes a page. user specifies type, and this function handles the
 * configuration details.
 */
static int write_page(struct mtd_info *mtd, struct nand_chip *chip,
                      const uint8_t *buf, bool raw_xfer)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);
    int status = 0;
    uint32_t page = hpnfc->page;

    hpnfc_set_skipbyte_count_offset(hpnfc, page);

    memcpy(hpnfc->buf.buf, buf, mtd->writesize);

    if (raw_xfer) {
        /* transfer the data to the spare area */
        memcpy(hpnfc->buf.buf + mtd->writesize, chip->oob_poi, mtd->oobsize);
    }
    else {
        /* just set spare data to 0xFF */
        memset(hpnfc->buf.buf + mtd->writesize, 0xFF, mtd->oobsize);
    }
    status = hpnfc_prepare_data_size(hpnfc, TT_MAIN_SPARE_AREAS);


    if(status){
        dev_err(hpnfc->dev, "write page failed\n");
        return -EIO;
    }

    return cdma_write_data(mtd, page,1);
}

/* NAND core entry points */

/*
 * this is the callback that the NAND core calls to write a page. Since
 * writing a page with ECC or without is similar, all the work is done
 * by write_page above.
 */
static int hpnfc_write_page(struct mtd_info *mtd, struct nand_chip *chip,
                            const uint8_t *buf, int oob_required, int page)
{
    /*
     * for regular page writes, we let HW handle all the ECC
     * data written to the device.
     */

    return write_page(mtd, chip, buf, oob_required ? true : false);
}

/*
 * This is the callback that the NAND core calls to write a page without ECC.
 * raw access is similar to ECC page writes, so all the work is done in the
 * write_page() function above.
 */
static int hpnfc_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
                                const uint8_t *buf, int oob_required, int page)
{
    /*
     * for raw page writes, we want to disable ECC and simply write
     * whatever data is in the buffer.
     */

    return write_page(mtd, chip, buf, true);
}

static int hpnfc_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
                           int page)
{

    return write_oob_data(mtd, chip->oob_poi, page);
}

static int hpnfc_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
                          int page)
{

    return read_oob_data(mtd, chip->oob_poi, page);
}

static int hpnfc_read_page(struct mtd_info *mtd, struct nand_chip *chip,
                           uint8_t *buf, int oob_required, int page)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);
    int status = 0;
    uint32_t ecc_err_count = 0;

    hpnfc_set_skipbyte_count_offset(hpnfc, page);

    status = hpnfc_prepare_data_size(hpnfc, TT_MAIN_SPARE_AREAS);
    if(status){
        return -EIO;
    }

    status = cdma_read_data(mtd, page, 1, &ecc_err_count);
    switch(status){
    case HPNFC_STAT_ERASED:
    case HPNFC_STAT_ECC_UNCORR:
        status = cdma_read_data(mtd, page, 0, NULL);
        if (status){
            dev_err(hpnfc->dev, "read page failed\n");
        }
        memcpy(buf, hpnfc->buf.buf, mtd->writesize);
        if (oob_required)
            memcpy(chip->oob_poi, hpnfc->buf.buf + mtd->writesize,
                   mtd->oobsize);
//        if (!is_erased(hpnfc->buf.buf, hpnfc->mtd.writesize + mtd->oobsize)){
        if (!is_erased(hpnfc->buf.buf, hpnfc->mtd.writesize)){
            hpnfc->mtd.ecc_stats.failed++;
            ecc_err_count++;
        }
        break;
    case HPNFC_STAT_ECC_CORR:
        if (ecc_err_count){
            hpnfc->mtd.ecc_stats.corrected += ecc_err_count;
        }
    case HPNFC_STAT_OK:
        memcpy(buf, hpnfc->buf.buf, mtd->writesize);
        if (oob_required)
            memcpy(chip->oob_poi, hpnfc->buf.buf + mtd->writesize,
                   mtd->oobsize);
        break;
    default:
        dev_err(hpnfc->dev, "read page failed\n");
        return -EIO;
    }

    return ecc_err_count;
}

static int hpnfc_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
                               uint8_t *buf, int oob_required, int page)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);
    int status = 0;

    hpnfc_set_skipbyte_count_offset(hpnfc, page);

    status = hpnfc_prepare_data_size(hpnfc, TT_MAIN_SPARE_AREAS);
    if(status){
        return -EIO;
    }

    status = cdma_read_data(mtd, page, 1, NULL);
    switch(status){
    case HPNFC_STAT_ERASED:
    case HPNFC_STAT_ECC_UNCORR:
        status = cdma_read_data(mtd, page, 0, NULL);
        memcpy(buf, hpnfc->buf.buf, mtd->writesize);
//        memcpy(buf, hpnfc->buf.buf + mtd->writesize, mtd->oobsize);

//        if (!is_erased(buf, hpnfc->mtd.writesize + mtd->oobsize)){
        if (!is_erased(buf, hpnfc->mtd.writesize)){
            return -EIO;
        }
    case HPNFC_STAT_ECC_CORR:
    case HPNFC_STAT_OK:
        memcpy(buf, hpnfc->buf.buf, mtd->writesize);
        memcpy(buf, hpnfc->buf.buf + mtd->writesize, mtd->oobsize);
        break;
    default:
        dev_err(hpnfc->dev, "read raw page failed\n");
        return -EIO;
    }

    return 0;
}


static uint8_t hpnfc_read_byte(struct mtd_info *mtd)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);
    uint8_t result = 0xff;

    if (hpnfc->buf.head < hpnfc->buf.tail){
        result = hpnfc->buf.buf[hpnfc->buf.head++];
    }

    return result;
}

static void hpnfc_select_chip(struct mtd_info *mtd, int chip)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);
    hpnfc->chip_nr = chip;
}

static int hpnfc_waitfunc(struct mtd_info *mtd, struct nand_chip *chip)
{
    return 0;
}

static int hpnfc_erase(struct mtd_info *mtd, int page)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);
    int status;
    uint8_t thread_nr = hpnfc->chip_nr;

    hpnfc_cdma_desc_prepare(hpnfc->cdma_desc, hpnfc->chip_nr, page, NULL,
                            HPNFC_CDMA_CT_ERASE, 1);
    status = hpnfc_cdma_send_and_wait(hpnfc, thread_nr);
    if(status){
        dev_err(hpnfc->dev, "erase operation failed\n");
        return -EIO;
    }

    status = hpnfc_wait_cdma_finish(hpnfc->cdma_desc);
    if (status)
        return status;

    return 0;
}

static void hpnfc_cmdfunc(struct mtd_info *mtd, unsigned int cmd, int col,
                          int page)
{
    struct hpnfc_state_t *hpnfc = mtd_to_hpnfc(mtd);

    struct nand_chip *chip = hpnfc->mtd.priv;

    //pr_info("cmd %x, col %d, page %d\n", cmd, col, page);

    hpnfc->offset = 0;
    switch (cmd) {
    case NAND_CMD_PAGEPROG:
        break;
    case NAND_CMD_STATUS:
        reset_buf(hpnfc);
        write_byte_to_buf(hpnfc, 0xE0);
        break;
    case NAND_CMD_READID:
        reset_buf(hpnfc);
        nf_mem_read_id(hpnfc, col, 4);
        break;
    case NAND_CMD_PARAM:
        reset_buf(hpnfc);
        read_parameter_page(hpnfc, 4096);
        break;
    case NAND_CMD_READ0:
    case NAND_CMD_SEQIN:
        hpnfc->page = page;
        break;
    case NAND_CMD_RESET:
        /* resets a specific device connected to the core */
        break;
    case NAND_CMD_READOOB:
        reset_buf(hpnfc);
        hpnfc_read_oob(mtd, chip, page);
        write_byte_to_buf(hpnfc, chip->oob_poi[col]);
        break;
    case NAND_CMD_RNDOUT:
        hpnfc->offset = col;
        break;
    default:
        dev_warn(hpnfc->dev, "unsupported command received 0x%x\n", cmd);
        break;
    }
}
/* end NAND core entry points */



static struct nand_ecclayout nand_ecc_oob = {
};


static uint8_t bbt_pattern[] = {'B', 'b', 't', ' ' };
static uint8_t mirror_pattern[] = {'1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descr = {
    .options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
        | NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
    .offs = 8,
    .len = 4,
    .veroffs = 12,
    .maxblocks = 4,
    .pattern = bbt_pattern,
};

static struct nand_bbt_descr bbt_mirror_descr = {
    .options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
        | NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
    .offs = 8,
    .len = 4,
    .veroffs = 12,
    .maxblocks = 4,
    .pattern = mirror_pattern,
};

int cadence_hpnfc_init(struct hpnfc_state_t *hpnfc)
{
    int ret, status;
    uint32_t ecc_per_sec_size;
    uint8_t timing_mode;
    uint8_t work_mode;

	struct mtd_part_parser_data     ppdata = {};

    hpnfc->buf.buf = devm_kzalloc(hpnfc->dev, 16 * 1024, GFP_DMA | GFP_KERNEL);
    if (!hpnfc->buf.buf) {
        return -ENOMEM;
    }

    hpnfc->cdma_desc = dma_alloc_coherent(hpnfc->dev,
                                        sizeof(hpnfc_cdma_desc_t),
                                        &hpnfc->dma_cdma_desc,
                                        GFP_KERNEL | GFP_DMA);

    if (!hpnfc->dma_cdma_desc) {
        return -ENOMEM;
    }

    hpnfc->mtd.dev.parent = hpnfc->dev;

    if (request_irq(hpnfc->irq, hpnfc_isr, IRQF_SHARED,
                    CADENCE_NAND_NAME, hpnfc)) {
        dev_err(hpnfc->dev, "Unable to allocate IRQ\n");
        ret = -ENODEV;

        goto failed_req_irq;
    }

    /* now that our ISR is registered, we can enable interrupts */
    hpnfc->mtd.name = CADENCE_NAND_NAME;
    hpnfc->mtd.owner = THIS_MODULE;
    hpnfc->mtd.priv = &hpnfc->nand;

    /* register the driver with the NAND core subsystem */
    hpnfc->nand.select_chip = hpnfc_select_chip;
    hpnfc->nand.cmdfunc = hpnfc_cmdfunc;
    hpnfc->nand.read_byte = hpnfc_read_byte;
    hpnfc->nand.waitfunc = hpnfc_waitfunc;

    if (!maxchips) {

        maxchips = 8;
    }


    hpnfc->max_banks = maxchips;
    /* temporary set number of devices to maximum suspected devices */
    hpnfc->devnum = maxchips;

    spin_lock_init(&hpnfc->irq_lock);

    init_completion(&hpnfc->complete);

    ret = hpnfc_hw_init(hpnfc);
    if (ret) {

        goto failed_req_irq;
    }
    hpnfc->nand.read_buf = hpnfc_read_buf;

    hpnfc_set_work_mode(hpnfc, HPNFC_WORK_MODE_ASYNC, 0);

    /*
     * scan for NAND devices attached to the controller
     * this is the first stage in a two step process to register
     * with the nand subsystem
     */

    if (nand_scan_ident(&hpnfc->mtd, hpnfc->max_banks, NULL)) {
        dev_warn(hpnfc->dev, "nand_scan_ident failed\n");
    }

    /* Get info about memory parameters  */
    if (hpnfc_dev_info(hpnfc)){
        dev_err(hpnfc->dev, "HW controller dev info failed\n");
        ret = -ENXIO;

       goto failed_req_irq;
    }

    /* allocate the right size buffer now */
    devm_kfree(hpnfc->dev, hpnfc->buf.buf);

    hpnfc->buf.buf = devm_kzalloc(hpnfc->dev,
                                  hpnfc->mtd.writesize + hpnfc->mtd.oobsize,
                                  GFP_DMA | GFP_KERNEL);

    if (!hpnfc->buf.buf) {
        ret = -ENOMEM;
        goto failed_req_irq;
    }

    /* Is 32-bit DMA supported? */
    ret = dma_set_mask(hpnfc->dev, DMA_BIT_MASK(32));
    if (ret) {
        dev_err(hpnfc->dev, "no usable DMA configuration\n");
        goto failed_req_irq;
    }

    hpnfc->buf.dma_buf =
        dma_map_single(hpnfc->dev, hpnfc->buf.buf,
                       hpnfc->mtd.writesize + hpnfc->mtd.oobsize,
                       DMA_BIDIRECTIONAL);
    if (dma_mapping_error(hpnfc->dev, hpnfc->buf.dma_buf)) {
        dev_err(hpnfc->dev, "Failed to map DMA buffer\n");
        ret = -EIO;
        goto failed_req_irq;
    }

    hpnfc_set_skipbyte_count_offset(hpnfc, hpnfc->main_size);

    /* Bad block management */
    hpnfc->nand.bbt_td = &bbt_main_descr;
    hpnfc->nand.bbt_md = &bbt_mirror_descr;

    hpnfc->nand.bbt_options |= NAND_BBT_USE_FLASH;
    hpnfc->nand.options |= NAND_SKIP_BBTSCAN | NAND_NO_SUBPAGE_WRITE;

    /* Error correction */
    hpnfc->nand.ecc.mode = NAND_ECC_HW;
    hpnfc->sector_count = hpnfc->main_size / hpnfc->sector_size;

    status = bch_calculate_ecc_size(hpnfc, &ecc_per_sec_size);
    if (status){
        hpnfc->ecc_enabled = 0;
        hpnfc->corr_cap = 0;
        hpnfc->nand.ecc.mode = NAND_ECC_NONE;
        hpnfc->sector_count = hpnfc->main_size;
        hpnfc->sector_size = hpnfc->main_size;
    }
    else {
        dev_info(hpnfc->dev,
                 "ECC enabled, correction capability: %d, sector size %d\n",
                 hpnfc->corr_cap, hpnfc->sector_size);
        hpnfc->ecc_enabled = 1;
    }
    hpnfc_ecc_enable(hpnfc, hpnfc->ecc_enabled);

    if ((hpnfc->sector_count * ecc_per_sec_size) >=
        (hpnfc->spare_size - HPNFC_MINIMUM_SPARE_SIZE)){
        /* to small spare area to hanlde such big ECC */
        ret = -EIO;
        goto failed_req_irq;
    }

    hpnfc->usnused_spare_size = hpnfc->spare_size
        - hpnfc->sector_count * ecc_per_sec_size;

    if (hpnfc->usnused_spare_size > HPNFC_MAX_SPARE_SIZE_PER_SECTOR)
        hpnfc->usnused_spare_size = HPNFC_MAX_SPARE_SIZE_PER_SECTOR;

    /* real spare size is hidden for MTD layer
     * because hardware handle all ECC stuff */
    hpnfc->mtd.oobsize = hpnfc->usnused_spare_size;

    hpnfc->nand.ecc.strength = hpnfc->corr_cap;
    hpnfc->nand.ecc.layout = &nand_ecc_oob;
    hpnfc->nand.ecc.bytes = ecc_per_sec_size;
    hpnfc->nand.ecc.layout->eccbytes = ecc_per_sec_size;
    hpnfc->nand.ecc.layout->oobfree[0].offset = 0;
    hpnfc->nand.ecc.layout->oobfree[0].length = hpnfc->usnused_spare_size;

    /* override the default read operations */
    hpnfc->nand.ecc.size = hpnfc->sector_size;
    hpnfc->nand.ecc.read_page = hpnfc_read_page;
    hpnfc->nand.ecc.read_page_raw = hpnfc_read_page_raw;
    hpnfc->nand.ecc.write_page = hpnfc_write_page;
    hpnfc->nand.ecc.write_page_raw = hpnfc_write_page_raw;
    hpnfc->nand.ecc.read_oob = hpnfc_read_oob;
    hpnfc->nand.ecc.write_oob = hpnfc_write_oob;
    hpnfc->nand.erase = hpnfc_erase;

    dev_info(hpnfc->dev, "hpnfc->mtd.writesize %d, hpnfc->mtd.oobsize %d\n",
             hpnfc->mtd.writesize, hpnfc->mtd.oobsize);
    dev_info(hpnfc->dev, "hpnfc->mtd.erasesize 0x%x, hpnfc->mtd.size 0x%llx\n",
             hpnfc->mtd.erasesize, hpnfc->mtd.size);

    if (init_erase){
        int status, i;
        uint8_t thread_nr = 0;
        dev_info(hpnfc->dev, "erase all blocks\n");

        for (i = 0; i < hpnfc->devnum; i++){
            hpnfc->chip_nr = i;
            hpnfc_cdma_desc_prepare(hpnfc->cdma_desc, hpnfc->chip_nr, 0, NULL,
                                    HPNFC_CDMA_CT_ERASE,
                                    hpnfc->blocks_per_lun * hpnfc->lun_count);
            status = hpnfc_cdma_send_and_wait(hpnfc, thread_nr);
            if (status){
                ret = -EIO;
                goto failed_req_irq;
            }
        }

        status = hpnfc_wait_cdma_finish(hpnfc->cdma_desc);
        if (status){
            ret = -EIO;
            goto failed_req_irq;
        }
    }

    hpnfc_check_the_best_mode(hpnfc, &work_mode, &timing_mode);

    if (hpnfc->mid == 0x01) {	/* MID check */
        /* S34MS01G2_04G2 */
        if (hpnfc->did == 0xac) {	/* DID check */
            timing_mode = 5;
        }
    }

    status = hpnfc_set_work_mode(hpnfc, work_mode, timing_mode);
    if (status){
        ret = -EIO;
        goto failed_req_irq;
    }

    nand_par_set(hpnfc);

    if (nand_scan_tail(&hpnfc->mtd)) {
        ret = -ENXIO;
        goto failed_req_irq;
    }

	ppdata.of_node = hpnfc->dev->of_node;
	ret = mtd_device_parse_register(&hpnfc->mtd, NULL, &ppdata, NULL, 0);
//    ret = mtd_device_register(&hpnfc->mtd, NULL, 0);
    if (ret) {
        dev_err(hpnfc->dev, "Failed to register MTD: %d\n",
                ret);
        goto failed_req_irq;
    }
    return 0;

failed_req_irq:
    hpnfc_irq_cleanup(hpnfc->irq, hpnfc);

    dma_free_coherent(hpnfc->dev, sizeof(hpnfc_cdma_desc_t), hpnfc->cdma_desc,
                      hpnfc->dma_cdma_desc);

    return ret;

}
EXPORT_SYMBOL(cadence_hpnfc_init);

/* driver exit point */
void cadence_hpnfc_remove(struct hpnfc_state_t *hpnfc)
{
    hpnfc_irq_cleanup(hpnfc->irq, hpnfc);
    dma_unmap_single(hpnfc->dev, hpnfc->buf.dma_buf,
                     hpnfc->mtd.writesize + hpnfc->mtd.oobsize,
                     DMA_BIDIRECTIONAL);
    dma_free_coherent(hpnfc->dev, sizeof(hpnfc_cdma_desc_t), hpnfc->cdma_desc,
                      hpnfc->dma_cdma_desc);

}
EXPORT_SYMBOL(cadence_hpnfc_remove);

void cadence_hpnfc_suspend(struct hpnfc_state_t *hpnfc)
{
	disable_irq(hpnfc->irq);
}

void cadence_hpnfc_resume(struct hpnfc_state_t *hpnfc)
{

    /* cadece ip relese of softreset */
    reset_control_deassert(hpnfc->rst); 

    /* WP enable setteing */
    IOWR_32(hpnfc->exstop_nfwpx_reg, (uint32_t )0x00000001);

    cadence_hpnfc_init(hpnfc);

}


