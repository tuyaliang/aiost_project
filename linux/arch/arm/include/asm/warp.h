/*
 * include/asm/warp.h
 *
 */

#ifndef _ASM_WARP_H
#define _ASM_WARP_H

#ifdef CONFIG_ARCH_MILBEAUT

#define WARP_HIBDRV_FIXED

#define WARP_NAND_DEVICE         WARP_DEV(NAND, 0, 0)
#define WARP_NAND_DEV_NAME       "warp"
#define WARP_NAND_HIBDRV_OFFS_0  (CONFIG_WARP_NAND_HIBDRV_OFFS_0)
#define WARP_NAND_HIBDRV_OFFS_1  (CONFIG_WARP_NAND_HIBDRV_OFFS_1)
#define WARP_NAND_HIBDRV_SIZE    0x00040000
#define WARP_NAND_USERAPI_OFFS_0 (WARP_NAND_HIBDRV_OFFS_0+WARP_NAND_HIBDRV_SIZE)
#define WARP_NAND_USERAPI_OFFS_1 (WARP_NAND_HIBDRV_OFFS_1+WARP_NAND_HIBDRV_SIZE)
#define WARP_NAND_USERAPI_SIZE   0x00040000
#define WARP_NAND_SW_BF_OFFS_0   (WARP_NAND_USERAPI_OFFS_0+WARP_NAND_USERAPI_SIZE)
#define WARP_NAND_SW_BF_OFFS_1   (WARP_NAND_USERAPI_OFFS_1+WARP_NAND_USERAPI_SIZE)
#define WARP_NAND_BF_OFFS_0      (CONFIG_WARP_NAND_BF_OFFS_0)
#define WARP_NAND_BF_OFFS_1      (CONFIG_WARP_NAND_BF_OFFS_1)
#define WARP_NAND_BF_SIZE        0x00040000
#define WARP_NAND_SS_OFFS_0      (WARP_NAND_BF_OFFS_0+WARP_NAND_BF_SIZE)
#define WARP_NAND_SS_OFFS_1      (WARP_NAND_BF_OFFS_1+WARP_NAND_BF_SIZE)
#define WARP_NAND_SS_SIZE        (CONFIG_WARP_NAND_SS_SIZE_BASE - WARP_NAND_HIBDRV_SIZE - \
                                  WARP_NAND_USERAPI_SIZE - WARP_NAND_BF_SIZE)

#define WARP_EMMC_DEVICE         WARP_DEV(SD, 0, 0)
#define WARP_EMMC_DEV_NAME       "mmcblk0"
#define WARP_EMMC_HIBDRV_OFFS_0  (CONFIG_WARP_EMMC_HIBDRV_OFFS_0)
#define WARP_EMMC_HIBDRV_OFFS_1  (CONFIG_WARP_EMMC_HIBDRV_OFFS_1)
#define WARP_EMMC_HIBDRV_SIZE    0x00000200
#define WARP_EMMC_USERAPI_OFFS_0 (WARP_EMMC_HIBDRV_OFFS_0+WARP_EMMC_HIBDRV_SIZE)
#define WARP_EMMC_USERAPI_OFFS_1 (WARP_EMMC_HIBDRV_OFFS_1+WARP_EMMC_HIBDRV_SIZE)
#define WARP_EMMC_USERAPI_SIZE   0x00000200
#define WARP_EMMC_BF_OFFS_0      (WARP_EMMC_USERAPI_OFFS_0+WARP_EMMC_USERAPI_SIZE)
#define WARP_EMMC_BF_OFFS_1      (WARP_EMMC_USERAPI_OFFS_1+WARP_EMMC_USERAPI_SIZE)
#define WARP_EMMC_BF_SIZE        0x00000002
#define WARP_EMMC_SS_OFFS_0      (WARP_EMMC_BF_OFFS_0+WARP_EMMC_BF_SIZE)
#define WARP_EMMC_SS_OFFS_1      (WARP_EMMC_BF_OFFS_1+WARP_EMMC_BF_SIZE)
#define WARP_EMMC_SS_SIZE        (CONFIG_WARP_EMMC_SS_SIZE_BASE - WARP_EMMC_HIBDRV_SIZE - \
                                  WARP_EMMC_USERAPI_SIZE - WARP_EMMC_BF_SIZE)

#ifdef WARP_HIBDRV_FIXED

#define WARP_HIBDRV_PHYS        (CONFIG_WARP_HIBDRV_PHYS)
#define WARP_HIBDRV_VIRT        0xff800000
#define WARP_HIBDRV_SIZE        0x000c0000
#define WARP_USERAPI_PHYS       (WARP_HIBDRV_PHYS + WARP_HIBDRV_SIZE)
#define WARP_USERAPI_VIRT       (WARP_HIBDRV_VIRT + WARP_HIBDRV_SIZE)

#define WARP_DRV_INFO                                                   \
{                                                                       \
    /* Hibernation driver */                                            \
    .mode                       = WARP_DRV_FIXED,                       \
    .drv.fixed.phys             = WARP_HIBDRV_PHYS,                     \
    .drv.fixed.virt             = WARP_HIBDRV_VIRT,                     \
    .drv.fixed.size             = WARP_HIBDRV_SIZE,                     \
}, {                                                                    \
    /* UserAPI driver */                                                \
    .mode                       = WARP_DRV_FIXED,                       \
    .drv.fixed.phys             = WARP_USERAPI_PHYS,                    \
    .drv.fixed.virt             = WARP_USERAPI_VIRT,                    \
},

#else

#ifdef WARP_HIBDRV_NAND

#define WARP_DRV_INFO                                                   \
{                                                                       \
    /* Hibernation driver area0 */                                      \
    .mode                       = WARP_DRV_FLOATING,                    \
    .drv.floating.load          = WARP_LOAD_MTD_NAME,                   \
    .drv.floating.dev.name      = WARP_NAND_DEV_NAME,                   \
    .drv.floating.offs          = WARP_NAND_HIBDRV_OFFS_0,              \
    .drv.floating.size          = WARP_NAND_HIBDRV_SIZE,                \
}, {                                                                    \
    /* UserAPI driver area0 */                                          \
    .mode                       = WARP_DRV_FLOATING,                    \
    .drv.floating.load          = WARP_LOAD_MTD_NAME,                   \
    .drv.floating.dev.name      = WARP_NAND_DEV_NAME,                   \
    .drv.floating.offs          = WARP_NAND_USERAPI_OFFS_0,             \
    .drv.floating.size          = WARP_NAND_USERAPI_SIZE,               \
}, {                                                                    \
    /* Hibernation driver area1 */                                      \
    .mode                       = WARP_DRV_FLOATING,                    \
    .drv.floating.load          = WARP_LOAD_MTD_NAME,                   \
    .drv.floating.dev.name      = WARP_NAND_DEV_NAME,                   \
    .drv.floating.offs          = WARP_NAND_HIBDRV_OFFS_1,              \
    .drv.floating.size          = WARP_NAND_HIBDRV_SIZE,                \
}, {                                                                    \
    /* UserAPI driver area1 */                                          \
    .mode                       = WARP_DRV_FLOATING,                    \
    .drv.floating.load          = WARP_LOAD_MTD_NAME,                   \
    .drv.floating.dev.name      = WARP_NAND_DEV_NAME,                   \
    .drv.floating.offs          = WARP_NAND_USERAPI_OFFS_1,             \
    .drv.floating.size          = WARP_NAND_USERAPI_SIZE,               \
},

#else

#define WARP_DRV_INFO                                                   \
{                                                                       \
    /* Hibernation driver area0 */                                      \
    .mode                       = WARP_DRV_FLOATING,                    \
    .drv.floating.load          = WARP_LOAD_DEV,                        \
    .drv.floating.dev.name      = WARP_EMMC_DEV_NAME,                   \
    .drv.floating.offs          = WARP_EMMC_HIBDRV_OFFS_0,              \
    .drv.floating.size          = WARP_EMMC_HIBDRV_SIZE,                \
}, {                                                                    \
    /* UserAPI driver area0 */                                          \
    .mode                       = WARP_DRV_FLOATING,                    \
    .drv.floating.load          = WARP_LOAD_DEV,                        \
    .drv.floating.dev.name      = WARP_EMMC_DEV_NAME,                   \
    .drv.floating.offs          = WARP_EMMC_USERAPI_OFFS_0,             \
    .drv.floating.size          = WARP_EMMC_USERAPI_SIZE,               \
}, {                                                                    \
    /* Hibernation driver area1 */                                      \
    .mode                       = WARP_DRV_FLOATING,                    \
    .drv.floating.load          = WARP_LOAD_DEV,                        \
    .drv.floating.dev.name      = WARP_EMMC_DEV_NAME,                   \
    .drv.floating.offs          = WARP_EMMC_HIBDRV_OFFS_1,              \
    .drv.floating.size          = WARP_EMMC_HIBDRV_SIZE,                \
}, {                                                                    \
    /* UserAPI driver area1 */                                          \
    .mode                       = WARP_DRV_FLOATING,                    \
    .drv.floating.load          = WARP_LOAD_DEV,                        \
    .drv.floating.dev.name      = WARP_EMMC_DEV_NAME,                   \
    .drv.floating.offs          = WARP_EMMC_USERAPI_OFFS_1,             \
    .drv.floating.size          = WARP_EMMC_USERAPI_SIZE,               \
},

#endif

#endif

#ifdef CONFIG_WARP_SS_FIX_0_PARTITION

#define WARP_SAVEAREA                                                   \
{                                                                       \
    .sw_bootflag.load           = WARP_LOAD_MTD_NAME,                   \
    .sw_bootflag.dev.name       = WARP_NAND_DEV_NAME,                   \
    .sw_bootflag.offs           = WARP_NAND_SW_BF_OFFS_0,               \
    .bootflag.dev               = WARP_NAND_DEVICE,                     \
    .bootflag.offs              = WARP_NAND_BF_OFFS_0,                  \
    .bootflag.size              = WARP_NAND_BF_SIZE,                    \
    .snapshot[0].dev            = WARP_NAND_DEVICE,                     \
    .snapshot[0].offs           = WARP_NAND_SS_OFFS_0,                  \
    .snapshot[0].size           = WARP_NAND_SS_SIZE,                    \
}, {                                                                    \
    .sw_bootflag.load           = WARP_LOAD_DEV,                        \
    .sw_bootflag.dev.name       = WARP_EMMC_DEV_NAME,                   \
    .sw_bootflag.offs           = WARP_EMMC_BF_OFFS_0,                  \
    .bootflag.dev               = WARP_EMMC_DEVICE,                     \
    .bootflag.offs              = WARP_EMMC_BF_OFFS_0,                  \
    .bootflag.size              = WARP_EMMC_BF_SIZE,                    \
    .snapshot[0].dev            = WARP_EMMC_DEVICE,                     \
    .snapshot[0].offs           = WARP_EMMC_SS_OFFS_0,                  \
    .snapshot[0].size           = WARP_EMMC_SS_SIZE,                    \
}, {                                                                    \
    .sw_bootflag.load           = WARP_LOAD_MTD_NAME,                   \
    .sw_bootflag.dev.name       = WARP_NAND_DEV_NAME,                   \
    .sw_bootflag.offs           = WARP_NAND_SW_BF_OFFS_0,               \
    .bootflag.dev               = WARP_NAND_DEVICE,                     \
    .bootflag.offs              = WARP_NAND_BF_OFFS_0,                  \
    .bootflag.size              = WARP_NAND_BF_SIZE,                    \
    .snapshot[0].dev            = WARP_NAND_DEVICE,                     \
    .snapshot[0].offs           = WARP_NAND_SS_OFFS_0,                  \
    .snapshot[0].size           = WARP_NAND_SS_SIZE,                    \
}, {                                                                    \
    .sw_bootflag.load           = WARP_LOAD_DEV,                        \
    .sw_bootflag.dev.name       = WARP_EMMC_DEV_NAME,                   \
    .sw_bootflag.offs           = WARP_EMMC_BF_OFFS_0,                  \
    .bootflag.dev               = WARP_EMMC_DEVICE,                     \
    .bootflag.offs              = WARP_EMMC_BF_OFFS_0,                  \
    .bootflag.size              = WARP_EMMC_BF_SIZE,                    \
    .snapshot[0].dev            = WARP_EMMC_DEVICE,                     \
    .snapshot[0].offs           = WARP_EMMC_SS_OFFS_0,                  \
    .snapshot[0].size           = WARP_EMMC_SS_SIZE,                    \
},

#else

#define WARP_SAVEAREA                                                   \
{                                                                       \
    .sw_bootflag.load           = WARP_LOAD_MTD_NAME,                   \
    .sw_bootflag.dev.name       = WARP_NAND_DEV_NAME,                   \
    .sw_bootflag.offs           = WARP_NAND_SW_BF_OFFS_0,               \
    .bootflag.dev               = WARP_NAND_DEVICE,                     \
    .bootflag.offs              = WARP_NAND_BF_OFFS_0,                  \
    .bootflag.size              = WARP_NAND_BF_SIZE,                    \
    .snapshot[0].dev            = WARP_NAND_DEVICE,                     \
    .snapshot[0].offs           = WARP_NAND_SS_OFFS_0,                  \
    .snapshot[0].size           = WARP_NAND_SS_SIZE,                    \
}, {                                                                    \
    .sw_bootflag.load           = WARP_LOAD_DEV,                        \
    .sw_bootflag.dev.name       = WARP_EMMC_DEV_NAME,                   \
    .sw_bootflag.offs           = WARP_EMMC_BF_OFFS_0,                  \
    .bootflag.dev               = WARP_EMMC_DEVICE,                     \
    .bootflag.offs              = WARP_EMMC_BF_OFFS_0,                  \
    .bootflag.size              = WARP_EMMC_BF_SIZE,                    \
    .snapshot[0].dev            = WARP_EMMC_DEVICE,                     \
    .snapshot[0].offs           = WARP_EMMC_SS_OFFS_0,                  \
    .snapshot[0].size           = WARP_EMMC_SS_SIZE,                    \
}, {                                                                    \
    .sw_bootflag.load           = WARP_LOAD_MTD_NAME,                   \
    .sw_bootflag.dev.name       = WARP_NAND_DEV_NAME,                   \
    .sw_bootflag.offs           = WARP_NAND_SW_BF_OFFS_1,               \
    .bootflag.dev               = WARP_NAND_DEVICE,                     \
    .bootflag.offs              = WARP_NAND_BF_OFFS_1,                  \
    .bootflag.size              = WARP_NAND_BF_SIZE,                    \
    .snapshot[0].dev            = WARP_NAND_DEVICE,                     \
    .snapshot[0].offs           = WARP_NAND_SS_OFFS_1,                  \
    .snapshot[0].size           = WARP_NAND_SS_SIZE,                    \
}, {                                                                    \
    .sw_bootflag.load           = WARP_LOAD_DEV,                        \
    .sw_bootflag.dev.name       = WARP_EMMC_DEV_NAME,                   \
    .sw_bootflag.offs           = WARP_EMMC_BF_OFFS_1,                  \
    .bootflag.dev               = WARP_EMMC_DEVICE,                     \
    .bootflag.offs              = WARP_EMMC_BF_OFFS_1,                  \
    .bootflag.size              = WARP_EMMC_BF_SIZE,                    \
    .snapshot[0].dev            = WARP_EMMC_DEVICE,                     \
    .snapshot[0].offs           = WARP_EMMC_SS_OFFS_1,                  \
    .snapshot[0].size           = WARP_EMMC_SS_SIZE,                    \
},

#endif

#define WARP_CPU_INFO                                                   \
{                                                                       \
    .cpu                        = 3,                                    \
    .save_ratio                 = 100,                                  \
}, {                                                                    \
    .cpu                        = 2,                                    \
    .save_ratio                 = 0,                                    \
}, {                                                                    \
    .cpu                        = 1,                                    \
    .save_ratio                 = 0,                                    \
}, {                                                                    \
    .cpu                        = 0,                                    \
    .save_ratio                 = 0,                                    \
},

#define WARP_CONSOLE            1
#define WARP_BPS                115200

#define WARP_USE_FREE_MEM

#endif

#endif
