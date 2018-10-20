
#include <linux/of.h>
#include <linux/platform_device.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#ifdef CONFIG_PM_WARP
#include <linux/warp_param.h>
#endif

static struct map_desc milbeaut_io_desc[] __initdata = {
#if defined(CONFIG_PM_WARP) && defined(WARP_HIBDRV_FIXED)
	{
		.virtual = WARP_HIBDRV_VIRT,
		.pfn = __phys_to_pfn(WARP_HIBDRV_PHYS),
		.length = 0x00100000,
		.type = MT_MEMORY_RWX,
	},
#endif
};

void __init milbeaut_map_io(void)
{
	debug_ll_io_init();
	iotable_init(milbeaut_io_desc, ARRAY_SIZE(milbeaut_io_desc));
}

static const char * const milbeaut_compat[] = {
	"socionext,milbeaut-evb",
	NULL,
};

DT_MACHINE_START(M8M_REB, "Socionext Milbeaut")
	.dt_compat	= milbeaut_compat,
	.l2c_aux_mask	= 0xffffffff,
	.map_io		= milbeaut_map_io,
MACHINE_END
