/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2019 Loongson Corporation.
 *   written by Xuefeng Li <lixuefeng@loongson.cn>
 *
 * based on arch/mips/loongson/loongson-3/cop2-ex.c
 * Copyright (C) 2014 Lemote Corporation,
 *   written by Huacai Chen <chenhc@lemote.com>
 */
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/notifier.h>

#include <asm/fpu.h>
#include <asm/cop2.h>
#include <asm/inst.h>
#include <asm/branch.h>
#include <asm/current.h>
#include <asm/uaccess.h>
#include <asm/mipsregs.h>
#include <ls2k.h>

#define OPCODE 0xfc000000
#define FUNC   0x0000003f
#define RD     0x0000f800
#define RS     0x03e00000
#define LWC2   0xc8000000
#define CPUCFG 0x00000018

struct cpucfg_info cpucfg_regs[NR_CPUS];

static int loongson_cu2_call(struct notifier_block *nfb, unsigned long action,
	void *data)
{
	struct pt_regs *regs = (struct pt_regs *)data;
	unsigned int __user *pc = (unsigned int __user *)exception_epc(regs);
	unsigned int opcode = 0;

	switch (action) {
	case CU2_EXCEPTION:
		if (unlikely(compute_return_epc(regs) < 0))
			goto fault;
		if (unlikely(get_user(opcode, pc) < 0))
			goto fault;
		if ((opcode & OPCODE) == LWC2 && (opcode & FUNC) == CPUCFG) {
			unsigned int cpu = smp_processor_id();
			int rd = (opcode & RD) >> 11;
			int rs = (opcode & RS) >> 21;
			unsigned long val = regs->regs[rs];
			switch (val) {
			case 0x0:
			case 0x1:
			case 0x2:
			case 0x3:
			case 0x4:
			case 0x5:
			case 0x6:
			case 0x7:
			case 0x8:
				regs->regs[rd] = cpucfg_regs[cpu].reg[val];
				return NOTIFY_STOP;	/* Don't call default notifier */
			default:
				regs->regs[rd] = 0;
				return NOTIFY_STOP;	/* Don't call default notifier */
			}
		}
	default:
		goto fault;
	}
fault:
	die_if_kernel("Reserved instruction in kernel code", regs);
	force_sig(SIGILL, current);

	return NOTIFY_STOP;	/* Don't call default notifier */
}

static int __init loongson_cu2_setup(void)
{
	return cu2_notifier(loongson_cu2_call, 0);
}
early_initcall(loongson_cu2_setup);

