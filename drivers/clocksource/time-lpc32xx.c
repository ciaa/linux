/*
 * Clocksource driver for NXP LPC32xx/18xx/43xx timer
 *
 * Copyright (C) 2014 Joachim Eastwood <manabian@gmail.com>
 *
 * Based on:
 * time-efm32 Copyright (C) 2013 Pengutronix
 * mach-lpc32xx/timer.c Copyright (C) 2009 - 2010 NXP Semiconductors
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

#define LPC32XX_TIMER_IR		0x000
#define  LPC32XX_TIMER_IR_MR0INT	BIT(0)
#define LPC32XX_TIMER_TCR		0x004
#define  LPC32XX_TIMER_TCR_CEN		BIT(0) /* Enable TC and PC counters */
#define  LPC32XX_TIMER_TCR_CRST		BIT(1) /* Reset counters on next clk */
#define LPC32XX_TIMER_TC		0x008
#define LPC32XX_TIMER_PR		0x00c
#define LPC32XX_TIMER_PC		0x010
#define LPC32XX_TIMER_MCR		0x014
#define  LPC32XX_TIMER_MCR_MR0I		BIT(0) /* Interrupt on MR0 */
#define  LPC32XX_TIMER_MCR_MR0R		BIT(1) /* Counter reset on MR0 */
#define  LPC32XX_TIMER_MCR_MR0S		BIT(2) /* Stop on MR0 */
#define LPC32XX_TIMER_MR0		0x018

struct lpc32xx_clock_event_ddata {
	struct clock_event_device evtdev;
	void __iomem *base;
};

static void __iomem *clocksource_timer_base;
static u32 ticks_per_jiffy;

static int lpc32xx_clkevt_next_event(unsigned long delta,
				     struct clock_event_device *evtdev)
{
	struct lpc32xx_clock_event_ddata *d =
		container_of(evtdev, struct lpc32xx_clock_event_ddata, evtdev);

	/* Reset timer counter and prescaler counter */
	writel_relaxed(LPC32XX_TIMER_TCR_CRST, d->base + LPC32XX_TIMER_TCR);

	/* Set prescaler maximum value */
	writel_relaxed(delta, d->base + LPC32XX_TIMER_PR);

	/* Enable the counters */
	writel_relaxed(LPC32XX_TIMER_TCR_CEN, d->base + LPC32XX_TIMER_TCR);

	return 0;
}

static void lpc32xx_clkevt_mode(enum clock_event_mode mode,
				struct clock_event_device *evtdev)
{
	struct lpc32xx_clock_event_ddata *d =
		container_of(evtdev, struct lpc32xx_clock_event_ddata, evtdev);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:

		/*
		 * Set match value. An interrupt will be generated each time
		 * the timer counter matches MR0.
		 */
		writel_relaxed(ticks_per_jiffy,
			       d->base + LPC32XX_TIMER_MR0);

		/* Interrupt and reset on MR0 */
		writel_relaxed(LPC32XX_TIMER_MCR_MR0I | LPC32XX_TIMER_MCR_MR0R,
			       d->base + LPC32XX_TIMER_MCR);

		/* Enable the counters */
		writel_relaxed(LPC32XX_TIMER_TCR_CEN,
			       d->base + LPC32XX_TIMER_TCR);

		break;

	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_SHUTDOWN:

		/* Set a minimum value for MR0 */
		writel_relaxed(1, d->base + LPC32XX_TIMER_MR0);

		/* Interrupt, reset and stop on MR0 */
		writel_relaxed(LPC32XX_TIMER_MCR_MR0I | LPC32XX_TIMER_MCR_MR0R |
		       LPC32XX_TIMER_MCR_MR0S, d->base + LPC32XX_TIMER_MCR);

		/*
		 * Disable the timer. When using oneshot, we must also
		 * disable the timer to wait for the first call to
		 * set_next_event().
		 */
		writel_relaxed(0, d->base + LPC32XX_TIMER_TCR);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static irqreturn_t lpc32xx_clock_event_handler(int irq, void *dev_id)
{
	struct lpc32xx_clock_event_ddata *d= dev_id;

	/* Clear match */
	writel_relaxed(LPC32XX_TIMER_IR_MR0INT, d->base + LPC32XX_TIMER_IR);

	d->evtdev.event_handler(&d->evtdev);

	return IRQ_HANDLED;
}

static struct lpc32xx_clock_event_ddata lpc32xx_clk_event_ddata = {
	.evtdev = {
		.name           = "lpc3250 clockevent",
		.features       = CLOCK_EVT_FEAT_PERIODIC |
				  CLOCK_EVT_FEAT_ONESHOT,
		.rating         = 300,
		.set_next_event = lpc32xx_clkevt_next_event,
		.set_mode       = lpc32xx_clkevt_mode,
	},
};

static struct irqaction lpc32xx_clock_event_irq = {
	.name		= "lpc3250 clockevent",
	.flags		= IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= lpc32xx_clock_event_handler,
	.dev_id		= &lpc32xx_clk_event_ddata,
};

static u64 notrace lpc32xx_read_sched_clock(void)
{
	return readl(clocksource_timer_base + LPC32XX_TIMER_TC);
}

static int __init lpc32xx_clocksource_init(struct device_node *np)
{
	void __iomem *base;
	unsigned long rate;
	struct clk *clk;
	int ret;

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		pr_err("clock get failed (%lu)\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("clock enable failed (%d)\n", ret);
		goto err_clk_enable;
	}

	rate = clk_get_rate(clk);

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("cannot map registers\n");
		ret = -EADDRNOTAVAIL;
		goto err_iomap;
	}

	writel_relaxed(LPC32XX_TIMER_TCR_CRST, base + LPC32XX_TIMER_TCR);
	writel_relaxed(0, base + LPC32XX_TIMER_PR);
	writel_relaxed(0, base + LPC32XX_TIMER_MCR);
	writel_relaxed(LPC32XX_TIMER_TCR_CEN, base + LPC32XX_TIMER_TCR);

	sched_clock_register(lpc32xx_read_sched_clock, 32, rate);

	ret = clocksource_mmio_init(base + LPC32XX_TIMER_TC, "lpc3250 timer",
				    rate, 300, 32, clocksource_mmio_readl_up);
	if (ret) {
		pr_err("failed to init clocksource (%d)\n", ret);
		goto err_clocksource_init;
	}

	clocksource_timer_base = base;
	return 0;

err_clocksource_init:
	iounmap(base);
err_iomap:
	clk_disable_unprepare(clk);
err_clk_enable:
	clk_put(clk);
	return ret;
}

static int __init lpc32xx_clockevent_init(struct device_node *np)
{
	void __iomem *base;
	unsigned long rate;
	struct clk *clk;
	int ret, irq;

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		pr_err("clock get failed (%lu)\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("clock enable failed (%d)\n", ret);
		goto err_clk_enable;
	}

	rate = clk_get_rate(clk);

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("unable to map registers\n");
		ret = -EADDRNOTAVAIL;
		goto err_iomap;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("cannot get irq\n");
		ret = -ENOENT;
		goto err_get_irq;
	}

	writel_relaxed(0, base + LPC32XX_TIMER_TCR);
	writel_relaxed(LPC32XX_TIMER_IR_MR0INT, base + LPC32XX_TIMER_IR);
	lpc32xx_clk_event_ddata.base = base;

	setup_irq(irq, &lpc32xx_clock_event_irq);

	ticks_per_jiffy = (rate + HZ/2) / HZ;
	clockevents_config_and_register(&lpc32xx_clk_event_ddata.evtdev,
					rate, 1, -1);

	return 0;

err_get_irq:
	iounmap(base);
err_iomap:
	clk_disable_unprepare(clk);
err_clk_enable:
	clk_put(clk);
	return ret;
}

/*
 * This function asserts that we have exactly one clocksource and one
 * clock_event_device in the end, no matter how many timers are available.
 */
static void __init lpc32xx_timer_init(struct device_node *np)
{
	static int has_clocksource, has_clockevent;
	int ret;

	if (!has_clocksource) {
		ret = lpc32xx_clocksource_init(np);
		if (!ret) {
			has_clocksource = 1;
			return;
		}
	}

	if (!has_clockevent) {
		ret = lpc32xx_clockevent_init(np);
		if (!ret) {
			has_clockevent = 1;
			return;
		}
	}
}
CLOCKSOURCE_OF_DECLARE(lpc32xx_timer, "nxp,lpc3250-timer", lpc32xx_timer_init);
