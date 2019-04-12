#include <linux/init.h>
#include <asm/time.h>
#include <loongson.h>
#include <asm/mipsregs.h>

#define ls_stable_freq cpu_clock_freq


static void stable_counter_suspend(struct clocksource *cs)
{
}

static void stable_counter_resume(struct clocksource *cs)
{
}

static cycle_t stable_counter_csr_read(struct clocksource *cs)
{
	return (cycle_t)drdtime();
}

static struct clocksource csrc_stable_counter = {
	.name = "stable",
	/* mips clocksource rating is less than 300, so stable_counter is better. */
	.rating = 380,
	.read = stable_counter_csr_read,
	.mask = CLOCKSOURCE_MASK(64),
	/* oneshot mode work normal with this flag */
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend = stable_counter_suspend,
	.resume = stable_counter_resume,
	.mult = 0,
	.shift = 10,
};

static int stable_enable(void)
{
    /*check stable clock feature*/
    if (!(read_cfg(LOONGSON_CPUCFG_CONFIG_FIELD2) & MIPS_LSE_LLFTP)) {
        printk(KERN_INFO "Don't support STABLE counter\n");
        return -ENXIO;
    }

    /*check stable clock enable*/
    if (!(dread_csr(LOONGSON_OTHER_FUNC_OFFSET) & LOONGSON_STABLE_COUNT_EN)) {
        printk(KERN_INFO "stable counter not enable \n");
        return -EBUSY;
    }

    return 0;
}

int __init init_stable_clocksource(void)
{
	struct cpuinfo_mips *c = &current_cpu_data;
	int res;

	if (!ls_stable_freq)
        return -ENXIO;

	switch (c->cputype) {
	case CPU_LOONGSON3:
		/* Loongson3A2000 and Loongson3A3000 has no stable counter */
        return -ENXIO;
	case CPU_LOONGSON3_COMP:
        if (stable_enable() == 0)
            csrc_stable_counter.read = stable_counter_csr_read;
        break;
    default:
        return -ENXIO;
	}

	csrc_stable_counter.mult =
        clocksource_hz2mult(ls_stable_freq, csrc_stable_counter.shift);
	res = clocksource_register_hz(&csrc_stable_counter, ls_stable_freq);

	printk(KERN_INFO "stable counter clock source device register\n");

    return res;
}
arch_initcall(init_stable_clocksource);
