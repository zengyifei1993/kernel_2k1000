#include <loongson.h>
#include <irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <asm/irq_cpu.h>
#include <asm/i8259.h>
#include <asm/mipsregs.h>

#include <loongson-pch.h>
#include <linux/reboot.h>

extern struct platform_controller_hub ls2h_pch;
extern struct platform_controller_hub ls7a_pch;
extern struct platform_controller_hub rs780_pch;

extern unsigned long long smp_group[4];

int ls3a_msi_enabled = 0;
EXPORT_SYMBOL(ls3a_msi_enabled);
extern unsigned char ls7a_ipi_irq2pos[];
extern unsigned int ls2h_irq2pos[];
extern void loongson3_ipi_interrupt(struct pt_regs *regs);

unsigned int ext_ini_en[MAX_32ARRAY_SIZE];

static DEFINE_SPINLOCK(affinity_lock);

void any_send(unsigned int off, unsigned int data, unsigned int cpu)
{
	unsigned long data64 = 0;
	unsigned int blk_bit_set = (unsigned int)(1 << LS_ANYSEND_BLOCK_SHIFT);

	data64 = blk_bit_set | off;
	data64 |= (cpu << LS_ANYSEND_CPU_SHIFT);
	data64 |= ((unsigned long)data << LS_ANYSEND_DATA_SHIFT);
	dwrite_csr(LOONGSON_ANY_SEND_OFFSET, data64);
}

static void set_irq_route(int pos, const struct cpumask *affinity)
{
	int k, pos_off;
	unsigned int nodemap[LS_ANYSEND_HANDLE_IRQS];
	unsigned char coremap[LS_ANYSEND_HANDLE_IRQS][MAX_NUMNODES];
	unsigned long data = 0;
	int package = -1;
	int cpu;

	pos_off = pos & (~3); /* get pos of aligned 4 irqs */
	memset(nodemap, 0, sizeof(unsigned int) * LS_ANYSEND_HANDLE_IRQS);
	memset(coremap, 0, sizeof(unsigned char) * LS_ANYSEND_HANDLE_IRQS * MAX_NUMNODES);

	/* calculate nodemap and coremap of 4 irqs including target irq */
	for (k = 0; k < LS_ANYSEND_HANDLE_IRQS; k++) {
		const struct cpumask *affinity_tmp;
		if (pos_off + k == pos) {
			affinity_tmp = affinity;
		} else {
			struct irq_desc *desc = irq_to_desc(pos_off + k);
			affinity_tmp = desc->irq_data.affinity;
		}
		
		for_each_cpu(cpu, affinity_tmp) {
			nodemap[k] |= (1 << (__cpu_logical_map[cpu] / cores_per_node));
			coremap[k][__cpu_logical_map[cpu] / cores_per_node] |= (1 << (__cpu_logical_map[cpu] % cores_per_node));
		}
	}

	/* csr write access to force current interrupt route completed on every core */
	for_each_cpu(cpu, cpu_online_mask)
		any_send(LOONGSON_EXT_IOI_BOUNCE64_OFFSET, LS_ANYSEND_IOI_EN32_DATA, __cpu_logical_map[cpu]);

	for_each_cpu(cpu, cpu_online_mask) {
		if (package != cpu_data[cpu].package) {

			package = cpu_data[cpu].package;
			data = 0;

			/* !!!! Only support 32bit data write, need to update 4 irqs route reg one time */
			for (k = 0; k < LS_ANYSEND_HANDLE_IRQS; k++) {
				if ((nodemap[k] & (1 << package)) && (__cpu_logical_map[cpu] != loongson_boot_cpu_id)) {
					/* target node only set itself to avoid unreasonable transfer */
					data |= ((unsigned long)(coremap[k][package] | ((1 << package) << LS_IOI_CPUNODE_SHIFT_IN_ROUTE)) << (LS_ANYSEND_ROUTE_DATA_POS(k)));
				} else {
					data |= ((unsigned long)(coremap[k][package] | (nodemap[k] << LS_IOI_CPUNODE_SHIFT_IN_ROUTE)) << (LS_ANYSEND_ROUTE_DATA_POS(k)));
				}
			}
			any_send(LOONGSON_EXT_IOI_ROUTE_OFFSET + pos_off, data, __cpu_logical_map[cpu]);
		}
	}

}


int ext_set_irq_affinity(struct irq_data *d, const struct cpumask *affinity,
			  bool force)
{
	unsigned long flags;
	unsigned short pos_off;

	if (!config_enabled(CONFIG_SMP))
		return -EPERM;

	spin_lock_irqsave(&affinity_lock, flags);

	if (cpumask_empty(affinity)) {
		spin_unlock_irqrestore(&affinity_lock, flags);
		return -EINVAL;
	}

	if (!cpumask_subset(affinity, cpu_online_mask)) {
		spin_unlock_irqrestore(&affinity_lock, flags);
		return -EINVAL;
	}

	/*
	 * control interrupt enable or disalbe through boot cpu
	 * which is reponsible for dispatching interrupts.
	 * */
	pos_off = d->irq >> 5;

	any_send(LOONGSON_EXT_IOI_EN64_OFFSET + (pos_off << 2), ext_ini_en[pos_off] & (~((1 << (d->irq & 0x1F)))), loongson_boot_cpu_id);
	set_irq_route(d->irq, affinity);
	any_send(LOONGSON_EXT_IOI_EN64_OFFSET + (pos_off << 2), ext_ini_en[pos_off], loongson_boot_cpu_id);

	cpumask_copy(d->affinity, affinity);
	spin_unlock_irqrestore(&affinity_lock, flags);

	return IRQ_SET_MASK_OK_NOCOPY;

}

int plat_set_irq_affinity(struct irq_data *d, const struct cpumask *affinity,
			  bool force)
{
	if ((loongson_pch == &ls7a_pch && ls7a_ipi_irq2pos[d->irq] == 255) || (loongson_pch == &ls2h_pch && ls2h_irq2pos[d->irq - LS2H_PCH_IRQ_BASE] == 0))
		return -EINVAL;

	if (!config_enabled(CONFIG_SMP))
		return -EPERM;

	if (cpumask_empty(affinity)) {
		return -EINVAL;
	}

	if (!cpumask_subset(affinity, cpu_online_mask)) {
		return -EINVAL;
	}

	cpumask_copy(d->affinity, affinity);

	return IRQ_SET_MASK_OK_NOCOPY;
}

#define UNUSED_IPS_GUEST (CAUSEF_IP1 | CAUSEF_IP0)
#define UNUSED_IPS (CAUSEF_IP5 | CAUSEF_IP4 | CAUSEF_IP1 | CAUSEF_IP0)

void mach_irq_dispatch(unsigned int pending)
{
	if (pending & CAUSEF_IP7)
		do_IRQ(LOONGSON_TIMER_IRQ);
#if defined(CONFIG_SMP)
	if (pending & CAUSEF_IP6)
		loongson3_ipi_interrupt(NULL);
#endif
	if(!cpu_has_vz) {
		pending = read_c0_cause() & read_c0_status() & ST0_IM;
		if (pending & CAUSEF_IP5)
			loongson_nodecounter_adjust();
		pending = read_c0_cause() & read_c0_status() & ST0_IM;
		if (pending & CAUSEF_IP4) {
			lsvirt_button_poweroff();
		}
 	}
	if (pending & CAUSEF_IP3)
		loongson_pch->irq_dispatch();
	if (pending & CAUSEF_IP2)
	{
		if(cpu_has_vz) {
			int cpu = cpu_logical_map(smp_processor_id());
			int irqs, irq, irqs_pci, irq_lpc;

			if(cpu == loongson_boot_cpu_id)
			{
				int core_index = loongson_boot_cpu_id % cores_per_node;
				irqs_pci = LOONGSON_INT_ROUTER_ISR(core_index) & 0xf0;
				irq_lpc = LOONGSON_INT_ROUTER_ISR(core_index) & 0x400;
				if(irqs_pci)
				{
					while ((irq = ffs(irqs_pci)) != 0) {
						do_IRQ(irq - 1 + SYS_IRQ_BASE);
						irqs_pci &= ~(1 << (irq-1));
					}
				}
				else if(irq_lpc)
				{
					if(ls_lpc_reg_base == LS3_LPC_REG_BASE)
					{
						irqs = ls2h_readl(LS_LPC_INT_ENA) & ls2h_readl(LS_LPC_INT_STS) & 0xfeff;
						if (irqs) {
							while ((irq = ffs(irqs)) != 0) {
								do_IRQ(irq - 1);
								irqs &= ~(1 << (irq-1));
							}
						}
					}

					do_IRQ(LOONGSON_UART_IRQ);
				}
			}
			else
				do_IRQ(LOONGSON_UART_IRQ);
		} else
			do_IRQ(LOONGSON_UART_IRQ);
	}
	if(cpu_has_vz) {	
		if (pending & UNUSED_IPS) {
			printk(KERN_ERR "%s : spurious interrupt\n", __func__);
			spurious_interrupt();
		}
	} else {
		if (pending & UNUSED_IPS_GUEST) {
			printk(KERN_ERR "%s : spurious interrupt\n", __func__);
			spurious_interrupt();
		}
 	}
}

static struct irqaction cascade_irqaction = {
	.handler = no_action,
	.flags = IRQF_NO_SUSPEND,
	.name = "cascade",
};

static inline void mask_loongson_irq(struct irq_data *d)
{
	struct irq_desc *desc = irq_to_desc(d->irq);
	if(!desc->action)
		return;

	/* Workaround: UART IRQ may deliver to any core */
	if (d->irq == LOONGSON_UART_IRQ) {
		int cpu = smp_processor_id();
		int node_id = cpu_logical_map(cpu) / cores_per_node;
		u64 intenclr_addr = smp_group[node_id] |
			(u64)(&LOONGSON_INT_ROUTER_INTENCLR);

		*(volatile u32 *)intenclr_addr = 1 << 10;
	}
}

static inline void unmask_loongson_irq(struct irq_data *d)
{
	/* Workaround: UART IRQ may deliver to any core */
	if (d->irq == LOONGSON_UART_IRQ) {
		int cpu = smp_processor_id();
		int node_id = cpu_logical_map(cpu) / cores_per_node;
		u64 intenset_addr = smp_group[node_id] |
			(u64)(&LOONGSON_INT_ROUTER_INTENSET);

		*(volatile u32 *)intenset_addr = 1 << 10;
	}
}

static inline unsigned int startup_loongson_irq(struct irq_data *d)
{
	return 0;
}

static inline void shutdown_loongson_irq(struct irq_data *d)
{

}

 /* For MIPS IRQs which shared by all cores */
static struct irq_chip loongson_irq_chip = {
	.name		= "Loongson",
	.irq_ack	= mask_loongson_irq,
	.irq_mask	= mask_loongson_irq,
	.irq_mask_ack	= mask_loongson_irq,
	.irq_unmask	= unmask_loongson_irq,
	.irq_eoi	= unmask_loongson_irq,
	.irq_startup	= startup_loongson_irq,
	.irq_shutdown	= shutdown_loongson_irq,
};

void __init mach_init_irq(void)
{
	int i;
	u64 intenset_addr;
	u64 introuter_lpc_addr;

	clear_c0_status(ST0_IM | ST0_BEV);

	mips_cpu_irq_init();
	if (loongson_pch)
		loongson_pch->init_irq();

	/* setup CASCADE irq */
	setup_irq(LOONGSON_BRIDGE_IRQ, &cascade_irqaction);

	irq_set_chip_and_handler(LOONGSON_UART_IRQ,
			&loongson_irq_chip, handle_level_irq);

	for (i = 0; i < nr_nodes_loongson; i++) {
		intenset_addr = smp_group[i] | (u64)(&LOONGSON_INT_ROUTER_INTENSET);
		introuter_lpc_addr = smp_group[i] | (u64)(&LOONGSON_INT_ROUTER_LPC);
		if (i == 0) {
			*(volatile u8 *)introuter_lpc_addr = LOONGSON_INT_COREx_INTy(loongson_boot_cpu_id, 0);
                } else {
			*(volatile u8 *)introuter_lpc_addr = LOONGSON_INT_COREx_INTy(0, 0);
		}
		*(volatile u32 *)intenset_addr = 1 << 10;
	}

	if(!cpu_has_vz)
		set_c0_status(STATUSF_IP2  | STATUSF_IP4 | STATUSF_IP5 | STATUSF_IP6);
	else
		set_c0_status(STATUSF_IP2 | STATUSF_IP6);
}

#ifdef CONFIG_HOTPLUG_CPU

void fixup_irqs(void)
{
	irq_cpu_offline();
	clear_c0_status(ST0_IM);
}

#endif
