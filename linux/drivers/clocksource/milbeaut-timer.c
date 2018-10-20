
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

#define FSL_TMR_TMCSR_OFS	0x0
#define FSL_TMR_TMR_OFS		0x4
#define FSL_TMR_TMRLR1_OFS	0x8
#define FSL_TMR_TMRLR2_OFS	0xc
#define FSL_RMT_REGSZPCH	0x10

#define FSL_TMR_TMCSR_OUTL		(1 << 5)
#define FSL_TMR_TMCSR_RELD		(1 << 4)
#define FSL_TMR_TMCSR_INTE		(1 << 3)
#define FSL_TMR_TMCSR_UF		(1 << 2)
#define FSL_TMR_TMCSR_CNTE		(1 << 1)
#define FSL_TMR_TMCSR_TRG		(1 << 0)

#define FSL_TMR_TMCSR_CSL_DIV2	0
#define FSL_TMR_TMCSR_CSL_SFT	10

struct milbeaut_clock_event_device {
	void __iomem *evt_base;
	void __iomem *src_base;
	u32 ticks_per_jiffy;
	struct clock_event_device dev;
#ifdef CONFIG_PM_WARP
	struct {
		u32 evt_csr, src_csr;
	} pm_regs;
#endif
};

static inline struct milbeaut_clock_event_device *to_milbeaut_clk(
				struct clock_event_device *c)
{
	return container_of(c, struct milbeaut_clock_event_device, dev);
}

static irqreturn_t milbeaut_timer_interrupt(int irq, void *dev_id)
{
	struct milbeaut_clock_event_device *evt = dev_id;
	u32 val;

	val = readl_relaxed(evt->evt_base + FSL_TMR_TMCSR_OFS);
	val &= ~FSL_TMR_TMCSR_UF;
	writel_relaxed(val, evt->evt_base + FSL_TMR_TMCSR_OFS);

	evt->dev.event_handler(&evt->dev);

	return IRQ_HANDLED;
}

static int milbeaut_set_state_periodic(struct clock_event_device *clk)
{
	struct milbeaut_clock_event_device *evt = to_milbeaut_clk(clk);
	u32 val = (FSL_TMR_TMCSR_CSL_DIV2 << FSL_TMR_TMCSR_CSL_SFT);

	writel_relaxed(val, evt->evt_base + FSL_TMR_TMCSR_OFS);

	writel_relaxed(evt->ticks_per_jiffy, evt->evt_base +
				FSL_TMR_TMRLR1_OFS);
	val |= FSL_TMR_TMCSR_RELD | FSL_TMR_TMCSR_CNTE |
		FSL_TMR_TMCSR_TRG | FSL_TMR_TMCSR_INTE;
	writel_relaxed(val, evt->evt_base + FSL_TMR_TMCSR_OFS);
	return 0;
}

/* Should we have to do something on this ? */
static int milbeaut_set_state_oneshot(struct clock_event_device *clk)
{
	struct milbeaut_clock_event_device *evt = to_milbeaut_clk(clk);
	u32 val = (FSL_TMR_TMCSR_CSL_DIV2 << FSL_TMR_TMCSR_CSL_SFT);

	writel_relaxed(val, evt->evt_base + FSL_TMR_TMCSR_OFS);
	return 0;
}

static int milbeaut_clkevt_next_event(unsigned long event,
				   struct clock_event_device *clk)
{
	struct milbeaut_clock_event_device *evt = to_milbeaut_clk(clk);

	writel_relaxed(event, evt->evt_base + FSL_TMR_TMRLR1_OFS);
	writel_relaxed((FSL_TMR_TMCSR_CSL_DIV2 << FSL_TMR_TMCSR_CSL_SFT) |
			FSL_TMR_TMCSR_CNTE | FSL_TMR_TMCSR_INTE |
			FSL_TMR_TMCSR_TRG, evt->evt_base +
			FSL_TMR_TMCSR_OFS);
	return 0;
}

#ifdef CONFIG_PM_WARP

static void milbeaut_timer_suspend(struct clock_event_device *clk)
{
	struct milbeaut_clock_event_device *evt = to_milbeaut_clk(clk);

	evt->pm_regs.evt_csr =
		readl_relaxed(evt->evt_base + FSL_TMR_TMCSR_OFS);
	evt->pm_regs.src_csr =
		readl_relaxed(evt->src_base + FSL_TMR_TMCSR_OFS);
}

static void milbeaut_timer_resume(struct clock_event_device *clk)
{
	struct milbeaut_clock_event_device *evt = to_milbeaut_clk(clk);

	writel_relaxed(0x0, evt->src_base + FSL_TMR_TMCSR_OFS);
	writel_relaxed(~0, evt->src_base + FSL_TMR_TMR_OFS);
	writel_relaxed(~0, evt->src_base + FSL_TMR_TMRLR1_OFS);
	writel_relaxed(~0, evt->src_base + FSL_TMR_TMRLR2_OFS);
	writel_relaxed(evt->pm_regs.src_csr,
		       evt->src_base + FSL_TMR_TMCSR_OFS);
	writel_relaxed(evt->pm_regs.evt_csr,
		       evt->evt_base + FSL_TMR_TMCSR_OFS);
}

#endif

static void __init milbeaut_timer_init(struct device_node *node)
{
	struct milbeaut_clock_event_device *evt;
	unsigned long rate = 0;
	struct clk *clk;

	evt = kzalloc(sizeof(*evt), GFP_KERNEL);
	if (!evt) {
		pr_warn("Can't allocate Milbeaut clock event driver struct");
		return;
	}

	evt->dev.name = "milbeaut_tick";
	evt->dev.rating = 500;
//	evt->dev.shift = 32; // jassi ?
	evt->dev.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	evt->dev.set_state_periodic = milbeaut_set_state_periodic;
	evt->dev.set_state_oneshot = milbeaut_set_state_oneshot;
	evt->dev.set_next_event = milbeaut_clkevt_next_event;
#ifdef CONFIG_PM_WARP
	evt->dev.suspend = milbeaut_timer_suspend;
	evt->dev.resume = milbeaut_timer_resume;
#endif
	evt->dev.cpumask = cpu_possible_mask;

	evt->evt_base = of_iomap(node, 0);
	if (IS_ERR(evt->evt_base)) {
		pr_warn("Can't get resource\n");
		return;
	}

	evt->src_base = of_iomap(node, 1);
	if (IS_ERR(evt->src_base)) {
		pr_warn("Can't get resource\n");
		return;
	}

	evt->dev.irq = irq_of_parse_and_map(node, 0);
	if (evt->dev.irq <= 0) {
		pr_warn("Can't parse IRQ");
		goto err_mem;
	}

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk)) {
		pr_warn("Can't get timer clock");
		goto err_irq;
	}

	if (clk_prepare_enable(clk)) {
		pr_warn("Can't prepare clock");
		goto err_clk_put;
	}
	/* CSL = 0 : rate = clk/2 */
	rate = clk_get_rate(clk) / 2;

	if (request_irq(evt->dev.irq, milbeaut_timer_interrupt,
			IRQF_TIMER | IRQF_IRQPOLL, "milbeaut_timer", evt)) {
		pr_warn("failed to setup irq %d\n", evt->dev.irq);
		goto err_clk_disable;
	}

	evt->ticks_per_jiffy = DIV_ROUND_UP(rate, HZ);

	/* Configure clock source */
	writel_relaxed(0x0, evt->src_base + FSL_TMR_TMCSR_OFS);
	writel_relaxed(~0, evt->src_base + FSL_TMR_TMR_OFS);
	writel_relaxed(~0, evt->src_base + FSL_TMR_TMRLR1_OFS);
	writel_relaxed(~0, evt->src_base + FSL_TMR_TMRLR2_OFS);
	writel_relaxed((1<<4) | (1<<1) | (1<<0), evt->src_base + FSL_TMR_TMCSR_OFS);

	clocksource_mmio_init(evt->src_base + FSL_TMR_TMR_OFS,
			node->name, rate, 300, 32, clocksource_mmio_readl_down);

	/* Configure clock event */
	writel_relaxed(0x0, evt->evt_base + FSL_TMR_TMCSR_OFS);
	clockevents_config_and_register(&evt->dev, rate, 0xf,
					0xffffffff);
	return;

err_clk_disable:
	clk_disable_unprepare(clk);
err_clk_put:
	clk_put(clk);
err_irq:
	irq_dispose_mapping(evt->dev.irq);
err_mem:
	iounmap(evt->evt_base);
	iounmap(evt->src_base);
}
CLOCKSOURCE_OF_DECLARE(milbeaut_peritimer, "socionext,milbeaut-timer", milbeaut_timer_init);
