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

#ifndef CADENCE_HPNFC_H
#define CADENCE_HPNFC_H

#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>

/***************************************************/
/*  Register definition */
/***************************************************/

/* Command register 0.
 * Writing data to this register will initiate a new transaction
 * of the NF controller. */
#define HPNFC_CMD_REG0                              0x0000
/* command type field shift */
#define     HPNFC_CMD_REG0_CT_SHIFT                 30
/* command type field mask */
#define     HPNFC_CMD_REG0_CT_MASK                  (3uL << 30)
/* command type CDMA */
#define     HPNFC_CMD_REG0_CT_CDMA                  (0uL)
/* command type PIO */
#define     HPNFC_CMD_REG0_CT_PIO                   (1uL)
/* command type reset */
#define     HPNFC_CMD_REG0_CT_RST                   (2uL)
/* command type generic */
#define     HPNFC_CMD_REG0_CT_GEN                   (3uL)
/* command thread number field shift */
#define     HPNFC_CMD_REG0_TN_SHIFT                 24
/* command thread number field mask */
#define     HPNFC_CMD_REG0_TN_MASK                  (3uL << 24)
/* command code field shift */
#define     HPNFC_CMD_REG0_PIO_CC_SHIFT             0
/* command code field mask */
#define     HPNFC_CMD_REG0_PIO_CC_MASK              (0xFFFFuL << 0)
/* command code - read page */
#define     HPNFC_CMD_REG0_PIO_CC_RD                (0x2200uL)
/* command code - write page */
#define     HPNFC_CMD_REG0_PIO_CC_WR                (0x2100uL)
/* command code - copy back */
#define     HPNFC_CMD_REG0_PIO_CC_CPB               (0x1200uL)
/* command code - reset */
#define     HPNFC_CMD_REG0_PIO_CC_RST               (0x1100uL)
/* command code - set feature */
#define     HPNFC_CMD_REG0_PIO_CC_SF                (0x0100uL)
/* command interrupt shift */
#define     HPNFC_CMD_REG0_INT_SHIFT                20
/* command interrupt mask */
#define     HPNFC_CMD_REG0_INT_MASK                 (1uL << 20)

/* PIO command - volume ID - shift */
#define     HPNFC_CMD_REG0_VOL_ID_SHIFT             16
/* PIO command - volume ID - mask */
#define     HPNFC_CMD_REG0_VOL_ID_MASK              (0xFuL << 16)

/* Command register 1. */
#define HPNFC_CMD_REG1                              0x0004
    /* PIO command - bank number - shift */
#   define HPNFC_CMD_REG1_BANK_SHIFT                24
    /* PIO command - bank number - mask */
#   define HPNFC_CMD_REG1_BANK_MASK                 (0x3uL << 24)
    /* PIO command - set feature - feature address - shift */
#   define HPNFC_CMD_REG1_FADDR_SHIFT               0
    /* PIO command - set feature - feature address - mask */
#   define HPNFC_CMD_REG1_FADDR_MASK                (0xFFuL << 0)

/* Command register 2 */
#define HPNFC_CMD_REG2                              0x0008
/* Command register 3 */
#define HPNFC_CMD_REG3                              0x000C
/* Pointer register to select which thread status will be selected. */
#define HPNFC_CMD_STATUS_PTR                        0x0010
/* Command status register for selected thread */
#define HPNFC_CMD_STATUS                            0x0014


/* interrupt status register */
#define HPNFC_INTR_STATUS                           0x0110
#define     HPNFC_INTR_STATUS_SDMA_ERR_MASK         (1uL << 22)
#define     HPNFC_INTR_STATUS_SDMA_TRIGG_MASK       (1uL << 21)
#define     HPNFC_INTR_STATUS_UNSUPP_CMD_MASK       (1uL << 19)
#define     HPNFC_INTR_STATUS_DDMA_TERR_MASK        (1uL << 18)
#define     HPNFC_INTR_STATUS_CDMA_TERR_MASK        (1uL << 17)
#define     HPNFC_INTR_STATUS_CDMA_IDL_MASK         (1uL << 16)

/* interrupt enable register */
#define HPNFC_INTR_ENABLE                           0x0114
#define     HPNFC_INTR_ENABLE_INTR_EN_MASK          (1uL << 31)
#define     HPNFC_INTR_ENABLE_SDMA_ERR_EN_MASK      (1uL << 22)
#define     HPNFC_INTR_ENABLE_SDMA_TRIGG_EN_MASK    (1uL << 21)
#define     HPNFC_INTR_ENABLE_UNSUPP_CMD_EN_MASK    (1uL << 19)
#define     HPNFC_INTR_ENABLE_DDMA_TERR_EN_MASK     (1uL << 18)
#define     HPNFC_INTR_ENABLE_CDMA_TERR_EN_MASK     (1uL << 17)
#define     HPNFC_INTR_ENABLE_CDMA_IDLE_EN_MASK     (1uL << 16)

/* Controller internal state */
#define HPNFC_CTRL_STATUS                           0x0118
#define     HPNFC_CTRL_STATUS_INIT_COMP_MASK        (1uL << 9)
#define     HPNFC_CTRL_STATUS_CTRL_BUSY_MASK        (1uL << 8)

/* Command Engine threads state */
#define HPNFC_TRD_STATUS                            0x0120

/*  Command Engine interrupt thread error status */
#define HPNFC_TRD_ERR_INT_STATUS                    0x0128
/*  Command Engine interrupt thread error enable */
#define HPNFC_TRD_ERR_INT_STATUS_EN                 0x0130
/*  Command Engine interrupt thread complete status*/
#define HPNFC_TRD_COMP_INT_STATUS                   0x0138

/* Transfer config 0 register.
 * Configures data transfer parameters. */
#define HPNFC_TRAN_CFG_0                            0x0400
/* Offset value from the beginning of the page - shift */
#define     HPNFC_TRAN_CFG_0_OFFSET_SHIFT           16
/* Offset value from the beginning of the page - mask */
#define     HPNFC_TRAN_CFG_0_OFFSET_MASK            (0xFFFFuL << 16)
/* Numbers of sectors to transfer within single NF device's page. - shift */
#define     HPNFC_TRAN_CFG_0_SEC_CNT_SHIFT          0
/* Numbers of sectors to transfer within single NF device's page. - mask */
#define     HPNFC_TRAN_CFG_0_SEC_CNT_MASK           (0xFFuL << 0)

/* Transfer config 1 register.
 * Configures data transfer parameters. */
#define HPNFC_TRAN_CFG_1                            0x0404
/* Size of last data sector. - shift */
#define     HPNFC_TRAN_CFG_1_LAST_SEC_SIZE_SHIFT    16
/* Size of last data sector. - mask */
#define     HPNFC_TRAN_CFG_1_LAST_SEC_SIZE_MASK     (0xFFFFuL << 16)
/*  Size of not-last data sector. - shift*/
#define     HPNFC_TRAN_CFG_1_SECTOR_SIZE_SHIFT      0
/*  Size of not-last data sector. - last*/
#define     HPNFC_TRAN_CFG_1_SECTOR_SIZE_MASK       (0xFFFFuL << 0)

/* NF device layout. */
#define HPNFC_NF_DEV_LAYOUT                         0x0424
/* Bit in ROW address used for selecting of the LUN - shift */
#define     HPNFC_NF_DEV_LAYOUT_ROWAC_SHIFT         24
/* Bit in ROW address used for selecting of the LUN - mask */
#define     HPNFC_NF_DEV_LAYOUT_ROWAC_MASK          (0xFuL << 24)
/* The number of LUN presents in the device. - shift */
#define     HPNFC_NF_DEV_LAYOUT_LN_SHIFT            20
/* The number of LUN presents in the device. - mask */
#define     HPNFC_NF_DEV_LAYOUT_LN_MASK             (0xF << 20)
/* Enables Multi LUN operations - mask */
#define     HPNFC_NF_DEV_LAYOUT_LUN_EN_MASK         (1uL << 16)
/* Pages Per Block - number of pages in a block - shift */
#define     HPNFC_NF_DEV_LAYOUT_PPB_SHIFT           0
/* Pages Per Block - number of pages in a block - mask */
#define     HPNFC_NF_DEV_LAYOUT_PPB_MASK            (0xFFFFuL << 0)

/* Device areas settings. */
#define HPNFC_NF_DEV_AREAS                          0x0428
/* Spare area size in bytes for the NF device page - shift */
#define     HPNFC_NF_DEV_AREAS_SPARE_SIZE_SHIFT     16
/* Spare area size in bytes for the NF device page - mask */
#define     HPNFC_NF_DEV_AREAS_SPARE_SIZE_MASK      (0xFFFFuL << 16)
/* Main area size in bytes for the NF device page - shift*/
#define     HPNFC_NF_DEV_AREAS_MAIN_SIZE_SHIFT      0
/* Main area size in bytes for the NF device page - mask*/
#define     HPNFC_NF_DEV_AREAS_MAIN_SIZE_MASK       (0xFFFFuL << 0)

/* ECC engine configuration register 0. */
#define HPNFC_ECC_CONFIG_0                          0x042C
/* Correction strength - shift */
#define     HPNFC_ECC_CONFIG_0_CORR_STR_SHIFT       8
/* Correction strength - mask */
#define     HPNFC_ECC_CONFIG_0_CORR_STR_MASK        (3uL << 8)
/* Enables scrambler logic in the controller - mask */
#define     HPNFC_ECC_CONFIG_0_SCRAMBLER_EN_MASK    (1uL << 2)
/*  Enable erased pages detection mechanism - mask */
#define     HPNFC_ECC_CONFIG_0_ERASE_DET_EN_MASK    (1uL << 1)
/*  Enable controller ECC check bits generation and correction - mask */
#define     HPNFC_ECC_CONFIG_0_ECC_EN_MASK          (1uL << 0)
/* ECC engine configuration register 1. */
#define HPNFC_ECC_CONFIG_1                          0x0430

/* Multiplane settings register */
#define HPNFC_MULTIPLANE_CFG                        0x0438
/* Cache operation settings. */
#define HPNFC_CACHE_CFG                             0x043c

/* DMA settings register */
#define HPNFC_DMA_SETTINGS                          0x0440
/* Enable SDMA error report on access unprepared slave DMA interface. - mask */
#define     HPNFC_DMA_SETTINGS_SDMA_ERR_RSP_MASK    (1uL << 17)
/* Outstanding transaction enable - mask */
#define     HPNFC_DMA_SETTINGS_OTE_MASK             (1uL << 16)
/* DMA burst selection - shift */
#define     HPNFC_DMA_SETTINGS_BURST_SEL_SHIFT      0
/* DMA burst selection - mask */
#define     HPNFC_DMA_SETTINGS_BURST_SEL_MASK       (0xFFuL << 0)

/* Transferred data block size for the slave DMA module */
#define HPNFC_SDMA_SIZE                             0x0444

/* Thread number associated with transferred data block
 * for the slave DMA module */
#define HPNFC_SDMA_TRD_NUM                          0x0448
/* Thread number mask */
#define     HPNFC_SDMA_TRD_NUM_SDMA_TRD_MASK        (0x3 << 0)
/* Thread number shift */
#define     HPNFC_SDMA_TRD_NUM_SDMA_TRD_SHIFT       (0)

/* NAND Flash memory device ID informations */
#define HPNFC_MANUFACTURER_ID                       0x0808
/* Device ID - mask */
#define     HPNFC_MANUFACTURER_ID_DID_MASK              (0xFFuL << 16)
/* Device ID - shift */
#define     HPNFC_MANUFACTURER_ID_DID_SHIFT             16
/* Manufacturer ID - mask */
#define     HPNFC_MANUFACTURER_ID_MID_MASK              (0xFFuL << 0)
/* Manufacturer ID - shift */
#define     HPNFC_MANUFACTURER_ID_MID_SHIFT             0

/* device parameters 1 register contains device signature */
#define HPNFC_DEV_PARAMS_1                          0x0810
#define     HPNFC_DEV_PARAMS_1_READID_6_SHIFT       24
#define     HPNFC_DEV_PARAMS_1_READID_6_MASK        (0xFFuL << 24)
#define     HPNFC_DEV_PARAMS_1_READID_5_SHIFT       16
#define     HPNFC_DEV_PARAMS_1_READID_5_MASK        (0xFFuL << 16)
#define     HPNFC_DEV_PARAMS_1_READID_4_SHIFT       8
#define     HPNFC_DEV_PARAMS_1_READID_4_MASK        (0xFFuL << 8)
#define     HPNFC_DEV_PARAMS_1_READID_3_SHIFT       0
#define     HPNFC_DEV_PARAMS_1_READID_3_MASK        (0xFFuL << 0)

/* device parameters 0 register */
#define HPNFC_DEV_PARAMS_0                          0x080c
/* device type shift */
#define     HPNFC_DEV_PARAMS_0_DEV_TYPE_SHIFT       30
/* device type mask */
#define     HPNFC_DEV_PARAMS_0_DEV_TYPE_MASK        (3uL << 30)
/* device type - ONFI */
#define     HPNFC_DEV_PARAMS_0_DEV_TYPE_ONFI        1
/* device type - JEDEC */
#define     HPNFC_DEV_PARAMS_0_DEV_TYPE_JEDEC       2
/* device type - unknown */
#define     HPNFC_DEV_PARAMS_0_DEV_TYPE_UNKNOWN     3
/* Number of bits used to addressing planes  - shift*/
#define     HPNFC_DEV_PARAMS_0_PLANE_ADDR_SHIFT      8
/* Number of bits used to addressing planes  - mask*/
#define     HPNFC_DEV_PARAMS_0_PLANE_ADDR_MASK      (0xFFuL << 8)
/* Indicates the number of LUNS present - mask*/
#define     HPNFC_DEV_PARAMS_0_NO_OF_LUNS_MASK      (0xFFuL << 0)

/* Features and optional commands supported
 * by the connected device */
#define HPNFC_DEV_FEATURES                          0x0814

/* Number of blocks per LUN present in the NF device. */
#define HPNFC_DEV_BLOCKS_PER_LUN                    0x0818

/* Device revision version */
#define HPNFC_DEV_REVISION                          0x081c

/*  Device Timing modes 0*/
#define HPNFC_ONFI_TIME_MOD_0                       0x0820
/* SDR timing modes support - shift */
#define     HPNFC_ONFI_TIME_MOD_0_SDR_SHIFT         0
/* SDR timing modes support - mask */
#define     HPNFC_ONFI_TIME_MOD_0_SDR_MASK          (0xFFFFuL << 0)
/* DDR timing modes support - shift */
#define     HPNFC_ONFI_TIME_MOD_0_DDR_SHIFT         16
/* DDR timing modes support - mask */
#define     HPNFC_ONFI_TIME_MOD_0_DDR_MASK          (0xFFFFuL << 16)

/*  Device Timing modes 1*/
#define HPNFC_ONFI_TIME_MOD_1                       0x0824
/* DDR2 timing modes support - shift */
#define     HPNFC_ONFI_TIME_MOD_1_DDR2_SHIFT        0
/* DDR2 timing modes support - mask */
#define     HPNFC_ONFI_TIME_MOD_1_DDR2_MASK         (0xFFFFuL << 0)
/* DDR3 timing modes support - shift */
#define     HPNFC_ONFI_TIME_MOD_1_DDR3_SHIFT        16
/* DDR3 timing modes support - mask */
#define     HPNFC_ONFI_TIME_MOD_1_DDR3_MASK         (0xFFFFuL << 16)

/* Ready/Busy# line status */
#define HPNFC_RBN_SETTINGS                          0x1004

/*  Common settings */
#define HPNFC_COMMON_SETT                           0x1008
/* write warm-up cycles. shift */
#define     HPNFC_COMMON_SETT_WR_WARMUP_SHIFT       20
/* read warm-up cycles. shift */
#define     HPNFC_COMMON_SETT_RD_WARMUP_SHIFT       16
/* 16 bit device connected to the NAND Flash interface - shift */
#define     HPNFC_COMMON_SETT_DEVICE_16BIT_SHIFT    8
/* Operation work mode - mask */
#define     HPNFC_COMMON_SETT_OPR_MODE_MASK         0x3
/* Operation work mode - shift */
#define     HPNFC_COMMON_SETT_OPR_MODE_SHIFT        0
/* Operation work mode - SDR mode */
#define     HPNFC_COMMON_SETT_OPR_MODE_SDR          0
/* Operation work mode - NV_DDR mode */
#define     HPNFC_COMMON_SETT_OPR_MODE_NV_DDR       1
/* Operation work mode - ToggleMode/NV-DDR2/NV-DDR3 mode */
#define     HPNFC_COMMON_SETT_OPR_MODE_TOGGLE       2

/*  Skip Bytes count*/
#define HPNFC_SKIP_BYTE_COUNT                       0x100c
/* Operation Skipbytes count */
#define     HPNFC_SKIP_BYTE_COUNT_VAL               2
/*  Skip Bytes offset*/
#define HPNFC_SKIP_BYTE_OFFSET                      0x1010

/* Toggle Timings 0 */
#define HPNFC_TOGGLE_TIMINGS_0                      0x1014
/* Toggle Timings 1 */
#define HPNFC_TOGGLE_TIMINGS_1                      0x1018

/* ToggleMode/NV-DDR2/NV-DDR3 and SDR timings configuration. */
#define HPNFC_ASYNC_TOGGLE_TIMINGS                  0x101c
/*  The number of clock cycles (nf_clk) the Minicontroller
 *  needs to de-assert RE# to meet the tREH  - shift*/
#define     HPNFC_ASYNC_TOGGLE_TIMINGS_TRH_SHIFT    24
/*  The number of clock cycles (nf_clk) the Minicontroller
 *  needs to de-assert RE# to meet the tREH  - mask*/
#define     HPNFC_ASYNC_TOGGLE_TIMINGS_TRH_MASK     (0x1FuL << 24)
/* The number of clock cycles (nf_clk) the Minicontroller
 * needs to assert RE# to meet the tRP - shift*/
#define     HPNFC_ASYNC_TOGGLE_TIMINGS_TRP_SHIFT    16
/* The number of clock cycles (nf_clk) the Minicontroller
 * needs to assert RE# to meet the tRP - mask*/
#define     HPNFC_ASYNC_TOGGLE_TIMINGS_TRP_MASK     (0x1FuL << 16)
/* The number of clock cycles (nf_clk) the Minicontroller
 * needs to assert WE# to meet the tWH - shift*/
#define     HPNFC_ASYNC_TOGGLE_TIMINGS_TWH_SHIFT    8
/* The number of clock cycles (nf_clk) the Minicontroller
 * needs to assert WE# to meet the tWH - mask*/
#define     HPNFC_ASYNC_TOGGLE_TIMINGS_TWH_MASK     (0x1FuL << 8)
/* The number of clock cycles (nf_clk) the Minicontroller
 * needs to assert WE# to meet the tWP - shift */
#define     HPNFC_ASYNC_TOGGLE_TIMINGS_TWP_SHIFT    0
/* The number of clock cycles (nf_clk) the Minicontroller
 * needs to assert WE# to meet the tWP - mask */
#define     HPNFC_ASYNC_TOGGLE_TIMINGS_TWP_MASK     (0x1FuL << 0)

/* Sync Timings */
#define HPNFC_SYNC_TIMINGS                          0x1020
/* Timings0 */
#define HPNFC_TIMINGS0                              0x1024
/* Timings1 */
#define HPNFC_TIMINGS1                              0x1028
/* Timings2 */
#define HPNFC_TIMINGS2                              0x102c
/* DLL PHY Update count */
#define HPNFC_DLL_UPDATE_CNT                        0x1030

/* Configuration of the resynchronization of slave DLL of PHY */
#define HPNFC_DLL_PHY_CTRL                              0x1034
/* Acknowledge signal to resynchronize the DLLs and read and
 * write FIFO pointers - mask */
#define     HPNFC_DLL_PHY_CTRL_DFI_CTRLUPD_ACK_MASK     (1uL << 26)
/* Signal to resynchronize the DLLs and read and write FIFO pointers - mask */
#define     HPNFC_DLL_PHY_CTRL_DFI_CTRLUPD_REQ_MASK     (1uL << 25)
/*  Signal to reset the DLLs of the PHY and start
 *  searching for lock again - mask */
#define     HPNFC_DLL_PHY_CTRL_DLL_RST_N_MASK           (1uL << 24)
/* Information to controller and PHY that the WE# is in extended mode - mask*/
#define     HPNFC_DLL_PHY_CTRL_EXTENDED_WR_MODE_MASK    (1uL << 17)
/* Information to controller and PHY that the RE# is in extended mode - mask*/
#define     HPNFC_DLL_PHY_CTRL_EXTENDED_RD_MODE_MASK    (1uL << 16)
/*  This field defines the number of Minicontroller clock cycles
 *  (nf_clk) for which the DLL update request (dfi_ctrlupd_req)
 *  has to be asserted - mask */
#define     HPNFC_DLL_PHY_CTRL_RS_HIGH_WAIT_CNT_MASK    (0xFuL << 8)
/* This field defines the wait time(in terms of Minicontroller
 * clock cycles (nf_clk)) between the deassertion of the DLL
 * update request (dfi_ctrlupd_req) and resuming traffic to the PHY - mask */
#define     HPNFC_DLL_PHY_CTRL_RS_IDLE_CNT_MASK         (0xFFuL << 0)

/* register controlling DQ related timing  */
#define HPNFC_PHY_DQ_TIMING_REG                     0x2000
/* register controlling DSQ related timing  */
#define HPNFC_PHY_DQS_TIMING_REG                    0x2004
/* register controlling the gate and loopback control related timing. */
#define HPNFC_PHY_GATE_LPBK_CTRL_REG                0x2008
/* register holds the control for the master DLL logic */
#define HPNFC_PHY_DLL_MASTER_CTRL_REG               0x200C
/* register holds the control for the slave DLL logic */
#define HPNFC_PHY_DLL_SLAVE_CTRL_REG                0x2010

/*   This register handles the global control settings for the PHY */
#define HPNFC_PHY_CTRL_REG                          0x2080
/*The value that should be driven on the DQS pin while SDR
 * operations are in progress. - shift*/
#define     HPNFC_PHY_CTRL_REG_SDR_DQS_SHIFT        14
/*The value that should be driven on the DQS pin while SDR
 * operations are in progress. - mask*/
#define     HPNFC_PHY_CTRL_REG_SDR_DQS_MASK         (1uL << 14)
/* The timing of assertion of phony DQS to the data slices - shift */
#define     HPNFC_PHY_CTRL_REG_PHONY_DQS_SHIFT      4
/* The timing of assertion of phony DQS to the data slices - mask */
#define     HPNFC_PHY_CTRL_REG_PHONY_DQS_MASK       4
/* This register handles the global control settings
 * for the termination selects for reads */
#define HPNFC_PHY_TSEL_REG                          0x2084
/***************************************************/


/***************************************************/
/*  Register field accessing macros */
/***************************************************/
/* write field of 32 bit register */
#define WRITE_FIELD(reg, field_name, field_val)\
    reg = (reg & ~field_name ## _MASK) \
        | ((uint32_t)field_val) << field_name ## _SHIFT

/* write field of 64 bit register */
#define WRITE_FIELD64(reg, field_name, field_val)\
    reg = (reg & ~field_name ## _MASK) \
        | ((uint64_t)field_val) << field_name ## _SHIFT

/* read field of 32 bit register */
#define READ_FIELD(reg, field_name)\
    ((reg & field_name ## _MASK) >> field_name ## _SHIFT)
 
/***************************************************/


/***************************************************/
/*  Register write/read macros */
/***************************************************/
#define IOWR_32(reg, val)   iowrite32((val), (reg));
#define IORD_32(reg)        ioread32((reg))
/***************************************************/


/***************************************************/
/*  Generic command definition */
/***************************************************/
typedef struct generic_data_t {
    /* use interrupts   */
    uint8_t use_intr:1;
    /* transfer direction */
    uint8_t direction:1;
    /* enable ECC  */
    uint8_t ecc_en:1;
    /* enable scrambler (if ECC is enabled) */
    uint8_t scr_en:1;
    /* enable erase page detection */
    uint8_t erpg_en:1;
    /* number of bytes to transfer of the n-1 sectors
     * except the last onesector */
    uint16_t sec_size;
    /* defines the number of sectors to transfer within a single
     * sequence */
    uint8_t sec_cnt;
    /* number of bytes to transfer in the last sector */
    uint16_t last_sec_size;
    /* ECC correction capability (if ECC is enabled) */
    uint8_t corr_cap;
} generic_data_t;


/* generic command layout*/
/* chip select - shift */
#define HPNFC_GCMD_LAY_CS_SHIFT         8
/* chip select - mask */
#define HPNFC_GCMD_LAY_CS_MASK          (0xF << 8)
/* commands complaint with Jedec spec*/
#define HPNFC_GCMD_LAY_JEDEC_MASK       (1 << 7)
/* This bit informs the minicotroller if it has to wait for tWB
 * after sending the last CMD/ADDR/DATA in the sequence. */
#define HPNFC_GCMD_LAY_TWB_MASK         (1 << 6)
/*  type of instruction - shift */
#define HPNFC_GCMD_LAY_INSTR_SHIFT      0
/*  type of instruction - mask */
#define HPNFC_GCMD_LAY_INSTR_MASK       (0x3F << 0)
/*  type of instruction - data transfer */
#define     HPNFC_GCMD_LAY_INSTR_DATA   2
/*  type of instruction - read parameter page (0xEF) */
#define     HPNFC_GCMD_LAY_INSTR_RDPP   28
/* type of instruction - read memory ID (0x90) */
#define     HPNFC_GCMD_LAY_INSTR_RDID   27
/* type of instruction - reset command (0xFF) */
#define     HPNFC_GCMD_LAY_INSTR_RDST   7
/* type of instruction - change read column command */
#define     HPNFC_GCMD_LAY_INSTR_CHRC   12

/* input part of generic command type of input is address 0 - shift */
#define HPNFC_GCMD_LAY_INPUT_ADDR0_SHIFT    16
/* input part of generic command type of input is address 0 - mask */
#define HPNFC_GCMD_LAY_INPUT_ADDR0_MASK     (0xFFFFFFFFFFuLL << 16)

/* generic command data sequence - transfer direction - shift  */
#define HPNFC_GCMD_DIR_SHIFT        11
/* generic command data sequence - transfer direction - mask  */
#define HPNFC_GCMD_DIR_MASK         (1uL << 11)
/* generic command data sequence - transfer direction - read  */
#define     HPNFC_GCMD_DIR_READ     0
/* generic command data sequence - transfer direction - write  */
#define     HPNFC_GCMD_DIR_WRITE    1

/* generic command data sequence - ecc enabled - mask  */
#define HPNFC_GCMD_ECC_EN_MASK      (1uLL << 12)
/* generic command data sequence - scrambler enabled - mask  */
#define HPNFC_GCMD_SCR_EN_MASK      (1uLL << 13)
/* generic command data sequence - erase page detection enabled - mask  */
#define HPNFC_GCMD_ERPG_EN_MASK     (1uLL << 14)
/* generic command data sequence - sctor size - shift  */
#define HPNFC_GCMD_SECT_SIZE_SHIFT  16
/* generic command data sequence - sector size - mask  */
#define HPNFC_GCMD_SECT_SIZE_MASK   (0xFFFFuL << 16)
/* generic command data sequence - sector count - shift  */
#define HPNFC_GCMD_SECT_CNT_SHIFT   32
/* generic command data sequence - sector count - mask  */
#define HPNFC_GCMD_SECT_CNT_MASK    (0xFFuLL << 32)
/* generic command data sequence - last sector size - shift  */
#define HPNFC_GCMD_LAST_SIZE_SHIFT  40
/* generic command data sequence - last sector size - mask  */
#define HPNFC_GCMD_LAST_SIZE_MASK   (0xFFFFuLL << 40)
/* generic command data sequence - correction capability - shift  */
#define HPNFC_GCMD_CORR_CAP_SHIFT   56
/* generic command data sequence - correction capability - mask  */
#define HPNFC_GCMD_CORR_CAP_MASK    (3uLL << 56)



/***************************************************/
/*  CDMA descriptor fields */
/***************************************************/

/** command DMA descriptor type - erase command */
#define HPNFC_CDMA_CT_ERASE         0x1000
/** command DMA descriptor type - reset command */
#define HPNFC_CDMA_CT_RST           0x1100
/** command DMA descriptor type - copy back command */
#define HPNFC_CDMA_CT_CPYB          0x1200
/** command DMA descriptor type - write page command */
#define HPNFC_CDMA_CT_WR            0x2100
/** command DMA descriptor type - read page command */
#define HPNFC_CDMA_CT_RD            0x2200
/** command DMA descriptor type - nop command */
#define HPNFC_CDMA_CT_NOP           0xFFFF

/** flash pointer memory - shift */
#define HPNFC_CDMA_CFPTR_MEM_SHIFT  24
/** flash pointer memory - mask */
#define HPNFC_CDMA_CFPTR_MEM_MASK   (7uL << 24)

/** command DMA descriptor flags - issue interrupt after
 * the completion of descriptor processing */
#define HPNFC_CDMA_CF_INT           (1uL << 8)
/** command DMA descriptor flags - the next descriptor
 * address field is valid and descriptor processing should continue */
#define HPNFC_CDMA_CF_CONT          (1uL << 9)
/* command DMA descriptor flags - selects DMA slave */
#define HPNFC_CDMA_CF_DMA_SLAVE     (0uL << 10)
/* command DMA descriptor flags - selects DMA master */
#define HPNFC_CDMA_CF_DMA_MASTER    (1uL << 10)


/* command descriptor status  - the operation number
 * where the first error was detected - shift*/
#define HPNFC_CDMA_CS_ERR_IDX_SHIFT 24
/* command descriptor status  - the operation number
 * where the first error was detected - mask*/
#define HPNFC_CDMA_CS_ERR_IDX_MASK  (0xFFuL << 24)
/* command descriptor status  - operation complete - mask*/
#define HPNFC_CDMA_CS_COMP_MASK     (1uL << 15)
/* command descriptor status  - operation fail - mask*/
#define HPNFC_CDMA_CS_FAIL_MASK     (1uL << 14)
/* command descriptor status  - page erased - mask*/
#define HPNFC_CDMA_CS_ERP_MASK      (1uL << 11)
/* command descriptor status  - timeout occured - mask*/
#define HPNFC_CDMA_CS_TOUT_MASK     (1uL << 10)
/* command descriptor status - maximum amount of correction
 * applied to one ECC sector - shift */
#define HPNFC_CDMA_CS_MAXERR_SHIFT  2
/* command descriptor status - maximum amount of correction
 * applied to one ECC sector - mask */
#define HPNFC_CDMA_CS_MAXERR_MASK   (0xFFuL << 2)
/* command descriptor status - uncorrectable ECC error - mask*/
#define HPNFC_CDMA_CS_UNCE_MASK     (1uL << 1)
/* command descriptor status - descriptor error - mask*/
#define HPNFC_CDMA_CS_ERR_MASK      (1uL << 0)

/***************************************************/


/***************************************************/
/*  internal used status*/
/***************************************************/
/* status of operation - OK */
#define HPNFC_STAT_OK               0
/* status of operation - FAIL */
#define HPNFC_STAT_FAIL             2
/* status of operation - uncorrectable ECC error */
#define HPNFC_STAT_ECC_UNCORR       3
/* status of operation - page erased */
#define HPNFC_STAT_ERASED           5
/* status of operation - correctable ECC error */
#define HPNFC_STAT_ECC_CORR         6
/* status of operation - operation is not completed yet */
#define HPNFC_STAT_BUSY             0xFF
/***************************************************/

/***************************************************/
/* work modes  */
/***************************************************/
#define HPNFC_WORK_MODE_ASYNC       0x00
#define HPNFC_WORK_MODE_NV_DDR      0x10
#define HPNFC_WORK_MODE_NV_DDR2     0x20
#define HPNFC_WORK_MODE_NV_DDR3     0x30
#define HPNFC_WORK_MODE_TOGG        0x40
/***************************************************/

/***************************************************/
/* ECC correction capabilities  */
/***************************************************/
#define HPNFC_ECC_CORR_CAP_2    2
#define HPNFC_ECC_CORR_CAP_4    4
#define HPNFC_ECC_CORR_CAP_8    8
#define HPNFC_ECC_CORR_CAP_12   12
#define HPNFC_ECC_CORR_CAP_16   16
#define HPNFC_ECC_CORR_CAP_24   24
#define HPNFC_ECC_CORR_CAP_40   40
/***************************************************/

/***************************************************/
/* ECC sector sizes  */
/***************************************************/
#define HPNFC_ECC_SEC_SIZE_1024 1024

/***************************************************/
/* DLL PHY Update Count  */
/***************************************************/
#define HPNFC_DLL_PHY_UPDATE_CNT_VAL 0x0000400


/***************************************************/

struct nand_buf {
    uint8_t *buf;
    int tail;
    int head;
    dma_addr_t dma_buf;
};

/* Command DMA descriptor */
typedef struct hpnfc_cdma_desc_t {
    /* next descriptor address */
    uint64_t next_pointer;
    /* glash address is a 32-bit address comprising of BANK and ROW ADDR. */
    uint32_t flash_pointer;
    uint32_t rsvd0;
    /* operation the controller needs to perform */
    uint16_t command_type;
    uint16_t rsvd1;
    /* flags for operation of this command */
    uint16_t command_flags;
    uint16_t rsvd2;
    /* system/host memory address required for data DMA commands. */
    uint64_t memory_pointer;
    /* status of operation */
    uint32_t status;
    uint32_t rsvd3;
    /* address pointer to sync buffer location */
    uint64_t sync_flag_pointer;
    /* Controls the buffer sync mechanism. */
    uint32_t sync_arguments;
    uint32_t rsvd4;
} hpnfc_cdma_desc_t;

/* interrupt status */
typedef struct hpnfc_irq_status_t {
    /*  Thread operation complete status */
    uint32_t trd_status;
    /*  Thread operation error */
    uint32_t trd_error;
    /* Controller status  */
    uint32_t status;
} hpnfc_irq_status_t;

typedef struct hpnfc_state_t {
    hpnfc_cdma_desc_t *cdma_desc;
    dma_addr_t dma_cdma_desc;
    struct nand_buf buf;
    /* actual selected chip */
    uint8_t chip_nr;
    int page;
    int offset;

    struct mtd_info mtd;
    struct nand_chip nand;
    void __iomem *reg;
    void __iomem *slave_dma;
    void __iomem *exstop_nfwpx_reg;
    void __iomem *exsintram;

    struct device *dev;
    struct reset_control *rst;

    int irq;
    hpnfc_irq_status_t irq_status;
    hpnfc_irq_status_t irq_mask;
    struct completion complete;
    spinlock_t irq_lock;

    /* part of spare area of NANF flash memory page.
     * This part is available for user to read or write. Rest of spare area
     * is used by controller for ECC correction */
    uint32_t usnused_spare_size;
    /* spare area size of NANF flash memory page */
    uint32_t spare_size;
    /* main area size of NANF flash memory page */
    uint32_t main_size;
    /* sector size few sectors are located on main area of NF memory page */
    uint32_t sector_size;
    /* last sector size (containing spare area) */
    uint32_t last_sector_size;
    uint32_t sector_count;
    uint32_t curr_trans_type;
    uint8_t corr_cap;

    uint32_t lun_count;

    uint32_t blocks_per_lun;

    uint32_t devnum;
    uint32_t bbtskipbytes;
    uint8_t max_banks;
    uint8_t dev_type;
    uint8_t ecc_enabled;

    uint32_t mid;
    uint32_t did;

} hpnfc_state_t;


int cadence_hpnfc_init(struct hpnfc_state_t *hpnfc);
void cadence_hpnfc_remove(struct hpnfc_state_t *hpnfc);
void cadence_hpnfc_suspend(struct hpnfc_state_t *hpnfc);
void cadence_hpnfc_resume(struct hpnfc_state_t *hpnfc);

#endif

