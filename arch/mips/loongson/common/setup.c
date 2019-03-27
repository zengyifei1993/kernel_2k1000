/*
 * Copyright (C) 2007 Lemote Inc. & Insititute of Computing Technology
 * Author: Fuxin Zhang, zhangfx@lemote.com
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <asm/wbflush.h>
#include <linux/of_fdt.h>
#include <asm/prom.h>
#include <linux/bootmem.h>

#include <loongson.h>

#ifdef CONFIG_VT
#include <linux/console.h>
#include <linux/screen_info.h>
#endif

#include <linux/libfdt.h>

static void wbflush_loongson(void)
{
	asm(".set\tpush\n\t"
	    ".set\tnoreorder\n\t"
	    ".set mips3\n\t"
	    "sync\n\t"
	    "nop\n\t"
	    ".set\tpop\n\t"
	    ".set mips0\n\t");
}

void (*__wbflush)(void) = wbflush_loongson;
EXPORT_SYMBOL(__wbflush);

void __init plat_mem_setup(void)
{
#ifdef CONFIG_VT
#if defined(CONFIG_VGA_CONSOLE)
	conswitchp = &vga_con;

	screen_info = (struct screen_info) {
		.orig_x			= 0,
		.orig_y			= 25,
		.orig_video_cols	= 80,
		.orig_video_lines	= 25,
		.orig_video_isVGA	= VIDEO_TYPE_VGAC,
		.orig_video_points	= 16,
	};
#elif defined(CONFIG_DUMMY_CONSOLE)
	conswitchp = &dummy_con;
#endif
#endif
	if (loongson_fdt_blob)
		__dt_setup_arch(loongson_fdt_blob);
}

#define NR_CELLS 6

void __init device_tree_init(void)
{
	unsigned long base, size;
    void *dt;

	if (!initial_boot_params) {
		pr_warn("No valid device tree found, continuing without\n");
		return;
	}
	base = virt_to_phys((void *)initial_boot_params);
	size = be32_to_cpu(initial_boot_params->totalsize);

	/* Before we do anything, lets reserve the dt blob */
    dt = memblock_virt_alloc(size,roundup_pow_of_two(FDT_V17_SIZE));
    if (dt) {
         memcpy(dt, initial_boot_params, size);
         initial_boot_params = dt;
    }

	unflatten_device_tree();

}

void encode_cpucfg_info(struct cpuinfo_mips *c,int cpu)
{
	extern struct cpucfg_info cpucfg_regs[];

	cpucfg_regs[cpu].reg[0] = c->processor_id;
	cpucfg_regs[cpu].reg[1] = MIPS_LSE_REG1_BASE | MIPS_LSE_FPREV(c->fpu_id);
	cpucfg_regs[cpu].reg[2] = MIPS_LSE_REG2_BASE;
	cpucfg_regs[cpu].reg[3] = MIPS_LSE_REG3_BASE;
	cpucfg_regs[cpu].reg[4] = 0;
	cpucfg_regs[cpu].reg[5] = 0;
	cpucfg_regs[cpu].reg[6] = 0;
	cpucfg_regs[cpu].reg[7] = 0;
	cpucfg_regs[cpu].reg[8] = 0;

	switch (c->processor_id & 0xff0000) {
	case PRID_COMP_LEGACY:
		switch (c->processor_id & 0xff00) {
		case PRID_IMP_LOONGSON2:  /* Loongson-3 */
			switch (c->processor_id & PRID_REV_MASK) {
			case PRID_REV_LOONGSON3B_R2:
				cpucfg_regs[cpu].reg[2] |= MIPS_LSE_LGFTP;
			case PRID_REV_LOONGSON3A_R1:
			case PRID_REV_LOONGSON3B_R1:
			default:
				break;
			}
		default:
			break;
		}
		break;
	case PRID_COMP_LOONGSON:
		switch (c->processor_id & 0xff00) {
		case PRID_IMP_LOONGSON2:  /* Loongson-2/3 */
			switch (c->processor_id & PRID_REV_MASK) {
			case PRID_REV_LOONGSON3A_R3_1:
				cpucfg_regs[cpu].reg[1] |= MIPS_LSE_CMDAP;
			case PRID_REV_LOONGSON3A_R2:
			case PRID_REV_LOONGSON3A_R3_0:
				cpucfg_regs[cpu].reg[1] |= MIPS_LSE_CNT64 | MIPS_LSE_LSPREF | MIPS_LSE_LSPREFX |
							   MIPS_LSE_LLEXC | MIPS_LSE_SCRAND | MIPS_LSE_SFBP;
				cpucfg_regs[cpu].reg[2] |= MIPS_LSE_LEXT2 | MIPS_LSE_LSPW | MIPS_LSE_LBT2 |
							   MIPS_LSE_LBTMMU | MIPS_LSE_LVZP | MIPS_LSE_LGFTP |
							   MIPS_LSE_LLFTP | MIPS_LSE_LPMREV(LS_LPMREV_3AR2) |
							   MIPS_LSE_LVZREV(LS_LVZREV_3AR2);
			default:
				break;
			}
		default:
			break;
		}
		break;
	default:
		cpucfg_regs[cpu].reg[1] = 0;
		cpucfg_regs[cpu].reg[2] = 0;
		cpucfg_regs[cpu].reg[3] = 0;
		break;
	}
}
