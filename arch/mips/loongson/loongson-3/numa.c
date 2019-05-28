/*
 * Copyright (C) 2010 Loongson Inc. & Insititute of Computing Technology
 * Author:  Gao Xiang, gaoxiang@ict.ac.cn
 *          Meng Xiaofu, Shuangshuang Zhang
 *          Chen Huacai, chenhc@lemote.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/module.h>
#include <linux/nodemask.h>
#include <linux/swap.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>
#include <linux/pfn.h>
#include <linux/highmem.h>
#include <asm/page.h>
#include <asm/pgalloc.h>
#include <asm/sections.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <asm/bootinfo.h>
#include <asm/mc146818-time.h>
#include <asm/time.h>
#include <asm/wbflush.h>
#include <boot_param.h>

unsigned char __node_distances[MAX_NUMNODES][MAX_NUMNODES];
EXPORT_SYMBOL(__node_distances);
struct node_data *__node_data[MAX_NUMNODES];
EXPORT_SYMBOL(__node_data);
extern u64 vuma_vram_addr;
extern u64 vuma_vram_size;

static inline struct node_data * __init alloc_node_data(int nid)
{
	void * nd;
	unsigned long nd_pa;
	size_t nd_sz = roundup(sizeof(struct node_data), PAGE_SIZE);

	nd_pa = memblock_alloc_nid(nd_sz, SMP_CACHE_BYTES, nid);
	if (!nd_pa) {
		nd_pa = __memblock_alloc_base(nd_sz, SMP_CACHE_BYTES,
				MEMBLOCK_ALLOC_ACCESSIBLE);
		if (!nd_pa) {
			pr_err("Cannot find %zu Byte for node_data %d \n",
					nd_sz, nid);
			return NULL;
		}
	}

	nd = __va(nd_pa);
	pr_info("hp: nd_pa:%lx, nd:%p\n", nd_pa, nd);
	memset(nd, 0, sizeof(struct node_data));

	return (struct node_data *)nd;
}

static void enable_lpa(void)
{
	unsigned long value;

	value = __read_32bit_c0_register($16, 3);
	value |= 0x00000080;
	__write_32bit_c0_register($16, 3, value);
	value = __read_32bit_c0_register($16, 3);
	printk("CP0_Config3: CP0 16.3 (0x%lx)\n", value);

	value = __read_32bit_c0_register($5, 1);
	value |= 0x20000000;
	__write_32bit_c0_register($5, 1, value);
	value = __read_32bit_c0_register($5, 1);
	printk("CP0_PageGrain: CP0 5.1 (0x%lx)\n", value);
}

static void cpu_node_probe(void)
{
	int i;

	nodes_clear(node_possible_map);
	nodes_clear(node_online_map);
	for (i = 0; i < nr_nodes_loongson; i++) {
		node_set_state(num_online_nodes(), N_POSSIBLE);
		node_set_online(num_online_nodes());
	}

	printk("NUMA: Discovered %d cpus on %d nodes\n", nr_cpus_loongson, num_online_nodes());
}

/* TODO: We need a more reasonalble method */
static int __init compute_node_distance(int row, int col)
{
	int package_row = row * cores_per_node / cores_per_package;
	int package_col = col * cores_per_node / cores_per_package;

	if (col == row)
		return 0;
	else if (package_row == package_col)
		return 40;
	else
		return 200;
}

static void __init init_topology_matrix(void)
{
	int row, col;

	for (row = 0; row < MAX_NUMNODES; row++)
		for (col = 0; col < MAX_NUMNODES; col++)
			__node_distances[row][col] = -1;

	for_each_online_node(row) {
		for_each_online_node(col) {
			__node_distances[row][col] =
				compute_node_distance(row, col);
		}
	}
}

extern unsigned int has_systab;
extern unsigned long systab_addr;

static int phy_index = 0;
static int dma_index = 0;
struct dma_mem_map ls_phy_map[LOONGSON3_BOOT_MEM_MAP], ls_dma_map[LOONGSON3_BOOT_MEM_MAP];
static void __init add_memory_region_dma(struct dma_mem_map *loongson_map, phys_t start, phys_t size, long type, int *index)
{
	loongson_map[*index].mem_start = start;
	loongson_map[*index].mem_size = size;
	loongson_map[*index].mem_type = type;
	*index += 1;
}

static void __init szmem(unsigned int node)
{
	u32 i, mem_type;
	u64 node_id, node_psize, start_pfn, end_pfn, mem_size;

	/* Parse memory information and activate */
	for (i = 0; i < emap->nr_map; i++) {
		node_id = emap->map[i].node_id;
		mem_type = emap->map[i].mem_type;
		mem_size = emap->map[i].mem_size;

		if (node_id == node) {
			switch (mem_type) {
			case SYSTEM_RAM_LOW:
				if (node_id == 0)
					low_physmem_start = emap->map[i].mem_start;
				start_pfn = ((node_id << 44) + emap->map[i].mem_start) >> PAGE_SHIFT;
				node_psize = (mem_size << 20) >> PAGE_SHIFT;
				end_pfn  = start_pfn + node_psize;
				num_physpages += node_psize;
				printk("Debug: node_id:%d, mem_type:%d, mem_start:0x%llx, mem_size:0x%llx MB\n",
					(u32)node_id, mem_type, emap->map[i].mem_start, mem_size);
				printk("       start_pfn:0x%llx, end_pfn:0x%llx, num_physpages:0x%lx\n",
					start_pfn, end_pfn, num_physpages);
				add_memory_region((node_id << 44) + emap->map[i].mem_start,
					(u64)emap->map[i].mem_size << 20, BOOT_MEM_RAM);
				add_memory_region_dma(ls_phy_map, (node_id << 44) + emap->map[i].mem_start,
					(u64)emap->map[i].mem_size << 20, SYSTEM_RAM_LOW, &phy_index);
				memblock_add_node(PFN_PHYS(start_pfn), PFN_PHYS(end_pfn - start_pfn), node);
				break;
			case SYSTEM_RAM_HIGH:
				if (node_id == 0)
					high_physmem_start = emap->map[i].mem_start;
				start_pfn = ((node_id << 44) + emap->map[i].mem_start) >> PAGE_SHIFT;
				node_psize = (mem_size << 20) >> PAGE_SHIFT;
				end_pfn  = start_pfn + node_psize;
				num_physpages += node_psize;
				printk("Debug: node_id:%d, mem_type:%d, mem_start:0x%llx, mem_size:0x%llx MB\n",
					(u32)node_id, mem_type, emap->map[i].mem_start, mem_size);
				printk("       start_pfn:0x%llx, end_pfn:0x%llx, num_physpages:0x%lx\n",
					start_pfn, end_pfn, num_physpages);
				add_memory_region((node_id << 44) + emap->map[i].mem_start,
					(u64)emap->map[i].mem_size << 20, BOOT_MEM_RAM);
				add_memory_region_dma(ls_phy_map, (node_id << 44) + emap->map[i].mem_start,
					(u64)emap->map[i].mem_size << 20, SYSTEM_RAM_HIGH, &phy_index);
				memblock_add_node(PFN_PHYS(start_pfn), PFN_PHYS(end_pfn - start_pfn), node);
				break;
			case MEM_RESERVED:
				printk("Debug: node_id:%d, mem_type:%d, mem_start:0x%llx, mem_size:0x%llx MB\n",
					(u32)node_id, mem_type, emap->map[i].mem_start, mem_size);
				add_memory_region((node_id << 44) + emap->map[i].mem_start,
					(u64)emap->map[i].mem_size << 20, BOOT_MEM_RESERVED);
				memblock_reserve(((node_id << 44) | emap->map[i].mem_start), mem_size << 20);
				break;
			case SMBIOS_TABLE:
				has_systab = 1;
				systab_addr = emap->map[i].mem_start;
				add_memory_region((node_id << 44) + emap->map[i].mem_start,
					0x2000, BOOT_MEM_RESERVED);
				memblock_reserve(((node_id << 44) | emap->map[i].mem_start), 0x2000);
				break;
			case UMA_VIDEO_RAM:
				{
					unsigned long start_pfn, end_pfn, ts, te;
					get_pfn_range_for_nid(node, &start_pfn, &end_pfn);
					ts = PFN_DOWN(emap->map[i].mem_start);
					te = PFN_UP(emap->map[i].mem_start + (emap->map[i].mem_size << 20));
					
					/*
 					* case0: Don't add VRAM into reserved memblock when it's OUT OF the memory region,
 					* so that bootmem can't fetch VRAM from reserved memblock to reserve it.(Reserve a region
 					* beyond memory will lead to BUG())
 					*
 					* case1: Add VRAM into reserved memblock when it's INSIDE the memory region, so that bootmem
 					* will reserve the VRAM and don't use it. 
 					*
 					* case2: Add VRAM into reserved memblock when it ACROSSES the memory region, so that the
 					* error case can be checked out.(The kernel will BUG() when bootmem reserve the VRAM 
 					* memblock because of part of it beyond memory.)
 					*
 					* */
					if ((te >= start_pfn) && (ts < end_pfn)) {
						vram_type = VRAM_TYPE_UMA;
						add_memory_region((node_id << 44) + emap->map[i].mem_start,
						(u64)emap->map[i].mem_size << 20, BOOT_MEM_RESERVED);
						memblock_reserve(((node_id << 44) | emap->map[i].mem_start), mem_size << 20);	
					}
				}
				uma_vram_addr = emap->map[i].mem_start;
				uma_vram_size = emap->map[i].mem_size << 20;
				break;
			case VUMA_VIDEO_RAM:
				vuma_vram_addr = emap->map[i].mem_start;
				vuma_vram_size = emap->map[i].mem_size << 20;
				break;
			case SYSTEM_RAM_LOW_DMA:
				printk("Debug: node_id:%d, mem_type:%d, mem_start:0x%llx, mem_size:0x%llx MB\n",
					(u32)node_id, mem_type, emap->map[i].mem_start, mem_size);
				add_memory_region_dma(ls_dma_map, emap->map[i].mem_start,
					(u64)emap->map[i].mem_size << 20, SYSTEM_RAM_LOW_DMA, &dma_index);
				break;
			case SYSTEM_RAM_HIGH_DMA:
				printk("Debug: node_id:%d, mem_type:%d, mem_start:0x%llx, mem_size:0x%llx MB\n",
					(u32)node_id, mem_type, emap->map[i].mem_start, mem_size);
				add_memory_region_dma(ls_dma_map, emap->map[i].mem_start,
					(u64)emap->map[i].mem_size << 20, SYSTEM_RAM_HIGH_DMA, &dma_index);
			}
		}
	}
}

static void __init node_mem_init(unsigned int node)
{
	unsigned long node_addrspace_offset;
	unsigned long start_pfn, end_pfn;
	struct node_data *nd;

	node_addrspace_offset = nid_to_addroffset(node);
	pr_info("Node%d's addrspace_offset is 0x%lx\n",
			node, node_addrspace_offset);

	get_pfn_range_for_nid(node, &start_pfn, &end_pfn);
	pr_info("Node%d: start_pfn=0x%lx, end_pfn=0x%lx\n",
		node, start_pfn, end_pfn);

	free_bootmem_with_active_regions(node, end_pfn);
	nd = alloc_node_data(node);
	__node_data[node] = nd;
	NODE_DATA(node)->node_start_pfn = start_pfn;
	NODE_DATA(node)->node_spanned_pages = end_pfn - start_pfn;


	if (node == 0) {
		/* kernel end address */
		unsigned long kernel_end_pfn = PFN_UP(__pa_symbol(&_end));

		/* used by finalize_initrd() */
		max_low_pfn = end_pfn;

		/* Reserve the kernel text/data/bss */
		memblock_reserve(start_pfn << PAGE_SHIFT,
				((kernel_end_pfn - start_pfn) << PAGE_SHIFT));
		/* Reserve 0xfe000000~0xffffffff for RS780E integrated GPU */
		if (node_end_pfn(0) >= (0xffffffff >> PAGE_SHIFT))
			memblock_reserve((node_addrspace_offset | 0xfe000000),
					 32 << 20);
	}


	sparse_memory_present_with_active_regions(node);
}

static __init void prom_meminit(void)
{
	unsigned int node, cpu, active_cpu = 0;

	cpu_node_probe();
	init_topology_matrix();
	num_physpages = 0;

	for (node = 0; node < nr_nodes_loongson; node++) {
		if (node_online(node)) {
			szmem(node);
			node_mem_init(node);
			cpus_clear(__node_data[(node)]->cpumask);
		}
	}
	max_low_pfn = PHYS_PFN(memblock_end_of_DRAM());

	for (cpu = 0; cpu < nr_cpus_loongson; cpu++) {
		node = cpu / cores_per_node;
		if (node >= num_online_nodes())
			node = 0;

		if (loongson_reserved_cpus_mask & (1<<cpu))
			continue;

		cpu_set(active_cpu, __node_data[(node)]->cpumask);
		printk("NUMA: set cpumask cpu %d on node %d\n", active_cpu, node);

		active_cpu++;
	}
}

void __init paging_init(void)
{
	unsigned long zones_size[MAX_NR_ZONES] = {0, };

	pagetable_init();

#ifdef CONFIG_ZONE_DMA32
	zones_size[ZONE_DMA32] = MAX_DMA32_PFN;
#endif
	zones_size[ZONE_NORMAL] = max_low_pfn;
	free_area_init_nodes(zones_size);
}

extern unsigned long setup_zero_pages(void);

void __init mem_init(void)
{
	high_memory = (void *) __va(get_num_physpages() << PAGE_SHIFT);
	totalram_pages += free_all_bootmem();
	setup_zero_pages();	/* This comes from node 0 */
	mem_init_print_info(NULL);
}

/* All PCI device belongs to logical Node-0 */
int pcibus_to_node(struct pci_bus *bus)
{
        return 0;
}
EXPORT_SYMBOL(pcibus_to_node);

void __init prom_init_numa_memory(void)
{
	enable_lpa();
	prom_meminit();
}
EXPORT_SYMBOL(prom_init_numa_memory);
