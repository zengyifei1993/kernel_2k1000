#include <loongson.h>
#include <irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <asm/irq_cpu.h>
#include <asm/i8259.h>
#include <asm/mipsregs.h>

extern int ls3a_msi_enabled;
extern unsigned long long smp_group[4];
extern void loongson3_send_irq_by_ipi(int cpu, int irqs);
extern void loongson3_ipi_interrupt(struct pt_regs *regs);

unsigned int irq_cpu[16] = {[0 ... 15] = -1};
unsigned int ht_irq[] = {0, 1, 3, 4, 5, 6, 7, 8, 12, 14, 15};
unsigned int local_irq = 1<<0 | 1<<1 | 1<<2 | 1<<7 | 1<<8 | 1<<12;

static int bootcore_int_mask;
static int bootcore_int_mask2;
extern unsigned int rs780e_irq2pos[];
extern unsigned int rs780e_pos2irq[];
static unsigned int irq_msi[LS3A_NUM_MSI_IRQS] = {[0 ... LS3A_NUM_MSI_IRQS-1] = -1};

void dispatch_msi_irq(int cpu, int htid, int mask)
{
	/*dispatch ht irqs for smi*/
#define HT_irq_vector_regs(n)	*(volatile unsigned int *)(ht_control_base + 0x80 +(n)*4)
	int i, irqs, irq, irq0, irq1;
	int start, end;
	int cpumask;
	struct irq_data *irqd;

#ifdef CONFIG_PCI_MSI_IRQ_BALANCE
		start = 1;
		end = 7;
                cpumask = 0xff;
#else
		start = (htid == 0)? 1:htid*2;
		end = (htid == 0)?7:htid*2+1;
                cpumask=(htid == 0)?bootcore_int_mask2:(3<<htid*2);
#endif


	for(i=start;i<=end && cpumask;i++)
	{
                if(!(cpumask&(1<<i))) continue;
		cpumask &= ~(1<<i);
		irqs = 0;
		irqs |= HT_irq_vector_regs(i) & mask;
		irqs |= HT_irq_vector_regs(i) & mask;
		irqs |= HT_irq_vector_regs(i) & mask;
		if(!irqs) continue;
		HT_irq_vector_regs(i) = irqs;
		HT_irq_vector_regs(i) = irqs;
		HT_irq_vector_regs(i) = irqs;
		__sync();
//		irqs &= *(volatile unsigned int *)(HT_control_regs_base + 0xA0 + i*4);
		irq0 = 0;
		while((irq = ffs(irqs)) != 0){
			/*maybe reenter*/
			irq = irq - 1;
			irqs &= ~(1<<irq);

			irq1 = i*32+irq;

			irqd = irq_get_irq_data(IRQ_LS3A_MSI_0 + irq1);

			irq_msi[irq1] = cpumask_next(irq_msi[irq1], irqd->affinity);

			if (irq_msi[irq1] >= nr_cpu_ids)
				irq_msi[irq1] = cpumask_first(irqd->affinity);

			if (irq_msi[irq1] == cpu || !rs780e_irq2pos[irq1] || !cpu_online(irq_msi[irq1])) {
				irq0 |= 1<<irq;
			}
			else
				loongson3_send_irq_by_ipi(irq_msi[irq1], (0x1 << (rs780e_irq2pos[irq1]-1+16)));
		}

		while((irq = ffs(irq0)) != 0){
			irq = irq - 1;
			/*maybe reenter*/
			do_IRQ(IRQ_LS3A_MSI_0+i*32+irq);
			irq0 &= ~(1<<irq);
		}
	}
}

void ht0_irq_dispatch(int mask)
{
	unsigned int i, irq;
	struct irq_data *irqd;
	struct cpumask affinity;

	irq = LOONGSON_HT1_INT_VECTOR(0) & mask;
	LOONGSON_HT1_INT_VECTOR(0) = irq;

	for (i = 0; i < (sizeof(ht_irq) / sizeof(*ht_irq)); i++) {
		if (!(irq & (0x1 << ht_irq[i])))
			continue;

		/* handled by local core */
		if (local_irq & (0x1 << ht_irq[i])) {
			do_IRQ(ht_irq[i]);
			continue;
		}

		irqd = irq_get_irq_data(ht_irq[i]);
		cpumask_and(&affinity, irqd->affinity, cpu_active_mask);
		if (cpumask_empty(&affinity)) {
			do_IRQ(ht_irq[i]);
			continue;
		}

		irq_cpu[ht_irq[i]] = cpumask_next(irq_cpu[ht_irq[i]], &affinity);
		if (irq_cpu[ht_irq[i]] >= nr_cpu_ids)
			irq_cpu[ht_irq[i]] = cpumask_first(&affinity);

		if (irq_cpu[ht_irq[i]] == 0) {
			do_IRQ(ht_irq[i]);
			continue;
		}

		/* balanced by other cores */
		loongson3_send_irq_by_ipi(irq_cpu[ht_irq[i]], (0x1 << ht_irq[i]));
	}

}

void rs780_irq_dispatch(void)
{
	int cpu = smp_processor_id();
	int rawcpu = cpu_logical_map(cpu);
	int htid =(rawcpu == loongson_boot_cpu_id)?0:(rawcpu == 0)?loongson_boot_cpu_id:rawcpu;
	int mask;

#ifdef CONFIG_PCI_MSI_IRQ_BALANCE
		mask = 0x11111111<<htid;
	        if(rawcpu == loongson_boot_cpu_id)
                mask |= bootcore_int_mask;
		ht0_irq_dispatch(mask);
		dispatch_msi_irq(cpu, htid, mask);
#else

		mask = 0xffffffff;
		if(htid == 0)
			ht0_irq_dispatch(mask);
		dispatch_msi_irq(cpu, htid, mask);
#endif
}

void rs780_irq_router_init(void)
{
	int i, rawcpu;

#ifdef CONFIG_PCI_MSI_IRQ_BALANCE
	*(volatile int *)(LOONGSON_HT1_CFG_BASE+0x58) = (*(volatile int *)(LOONGSON_HT1_CFG_BASE+0x58) & ~0x700)|0x200;
#else
	*(volatile int *)(LOONGSON_HT1_CFG_BASE+0x58) &= ~0x700;
#endif
	/* route LPC int to cpu core0 int 0 */
	LOONGSON_INT_ROUTER_LPC = LOONGSON_INT_COREx_INTy(loongson_boot_cpu_id, 0);
	/* route HT1 int0 ~ int7 to cpu core0 INT1*/
	LOONGSON_INT_ROUTER_HT1(0) = LOONGSON_INT_COREx_INTy(loongson_boot_cpu_id, 1);
	for (i = 1; i <= 3; i++)
	{
		rawcpu = (i == loongson_boot_cpu_id)?0:i; 

		if(cpu_number_map(rawcpu)<setup_max_cpus && cpu_number_map(rawcpu)<nr_cpu_ids)
		LOONGSON_INT_ROUTER_HT1(i) = LOONGSON_INT_COREx_INTy(rawcpu, 1);
		else
		{
		LOONGSON_INT_ROUTER_HT1(i) = LOONGSON_INT_COREx_INTy(loongson_boot_cpu_id, 1);
                bootcore_int_mask |= 0x11111111<<i;
                bootcore_int_mask2 |= 0x3<<(2*i);
		}
	}

	bootcore_int_mask2 |= 2;
	for (i = 4; i < 8; i++)
		LOONGSON_INT_ROUTER_HT1(i) = LOONGSON_INT_COREx_INTy(loongson_boot_cpu_id, 1);
	/* enable HT1 interrupt */
	LOONGSON_HT1_INTN_EN(0) = 0x0000ffff;
	/* enable router interrupt intenset */
	LOONGSON_INT_ROUTER_INTENSET = LOONGSON_INT_ROUTER_INTEN | (0xffff << 16) | 0x1 << 10;
}

void __init rs780_init_irq(void)
{
	if(((read_c0_prid() & 0xff00) == PRID_IMP_LOONGSON2) && ((read_c0_prid() & 0xff) > PRID_REV_LOONGSON3A_R1) &&
		((read_c0_prid() & 0xff) < PRID_REV_LOONGSON3A_R3_1)) {
		pr_info("Do not supports HT MSI interrupt, disabling RS780E MSI Interrupt.\n");
		ls3a_msi_enabled = 0;
	} else {
		pr_info("Supports HT MSI interrupt, enabling RS780E MSI Interrupt.\n");
		ls3a_msi_enabled = 1;
	}
	rs780_irq_router_init();
	init_i8259_irqs();
}
