#ifndef _ASM_X86_EFI_H
#define _ASM_X86_EFI_H

#include <asm/spec_ctrl.h>

/*
 * We map the EFI regions needed for runtime services non-contiguously,
 * with preserved alignment on virtual addresses starting from -4G down
 * for a total max space of 64G. This way, we provide for stable runtime
 * services addresses across kernels so that a kexec'd kernel can still
 * use them.
 *
 * This is the main reason why we're doing stable VA mappings for RT
 * services.
 *
 * This flag is used in conjuction with a chicken bit called
 * "efi=old_map" which can be used as a fallback to the old runtime
 * services mapping method in case there's some b0rkage with a
 * particular EFI implementation (haha, it is hard to hold up the
 * sarcasm here...).
 */
#define EFI_OLD_MEMMAP		EFI_ARCH_1

#define EFI32_LOADER_SIGNATURE	"EL32"
#define EFI64_LOADER_SIGNATURE	"EL64"

#ifdef CONFIG_X86_32


extern unsigned long asmlinkage efi_call_phys(void *, ...);

/*
 * Wrap all the virtual calls in a way that forces the parameters on the stack.
 */

/* Use this macro if your virtual returns a non-void value */
#define efi_call_virt(f, args...) \
	((efi_##f##_t __attribute__((regparm(0)))*)efi.systab->runtime->f)(args)

/* Use this macro if your virtual call does not return any value */
#define __efi_call_virt(f, args...) efi_call_virt(f, args)

#define efi_ioremap(addr, size, type, attr)	ioremap_cache(addr, size)

#else /* !CONFIG_X86_32 */

#define EFI_LOADER_SIGNATURE	"EL64"

extern u64 asmlinkage efi_call(void *fp, ...);

#define efi_call_phys(f, args...)		efi_call((f), args)

#define efi_call_virt(f, ...)						\
({									\
	efi_status_t __s;						\
	bool ibrs_on;							\
									\
	efi_sync_low_kernel_mappings();					\
	preempt_disable();						\
	ibrs_on = unprotected_firmware_begin();				\
	__s = efi_call((void *)efi.systab->runtime->f, __VA_ARGS__);	\
	unprotected_firmware_end(ibrs_on);				\
	preempt_enable();						\
	__s;								\
})

/*
 * All X86_64 virt calls return non-void values. Thus, use non-void call for
 * virt calls that would be void on X86_32.
 */
#define __efi_call_virt(f, args...) efi_call_virt(f, args)

extern void __iomem *efi_ioremap(unsigned long addr, unsigned long size,
				 u32 type, u64 attribute);

#endif /* CONFIG_X86_32 */

extern int add_efi_memmap;
extern unsigned long x86_efi_facility;
extern struct efi_scratch efi_scratch;
extern void efi_set_executable(efi_memory_desc_t *md, bool executable);
extern int efi_memblock_x86_reserve_range(void);
extern void efi_call_phys_prelog(void);
extern void efi_call_phys_epilog(void);
extern void efi_unmap_memmap(void);
extern void efi_memory_uc(u64 addr, unsigned long size);
extern void __init efi_map_region(efi_memory_desc_t *md);
extern void __init efi_map_region_fixed(efi_memory_desc_t *md);
extern void efi_sync_low_kernel_mappings(void);
extern int __init efi_alloc_page_tables(void);
extern int efi_setup_page_tables(unsigned long pa_memmap, unsigned num_pages);
extern void efi_cleanup_page_tables(unsigned long pa_memmap, unsigned num_pages);
extern void __init old_map_region(efi_memory_desc_t *md);
extern void __init efi_dump_pagetable(void);
extern void __init efi_apply_memmap_quirks(void);

struct efi_setup_data {
	u64 fw_vendor;
	u64 runtime;
	u64 tables;
	u64 smbios;
	u64 reserved[8];
};

extern u64 efi_setup;

#ifdef CONFIG_EFI

static inline bool efi_is_native(void)
{
	return IS_ENABLED(CONFIG_X86_64) == efi_enabled(EFI_64BIT);
}

static inline bool efi_runtime_supported(void)
{
	if (efi_is_native())
		return true;

	if (IS_ENABLED(CONFIG_EFI_MIXED) && !efi_enabled(EFI_OLD_MEMMAP))
		return true;

	return false;
}

extern struct console early_efi_console;

extern void parse_efi_setup(u64 phys_addr, u32 data_len);

#ifdef CONFIG_EFI_MIXED
extern void efi_thunk_runtime_setup(void);
extern efi_status_t efi_thunk_set_virtual_address_map(
	void *phys_set_virtual_address_map,
	unsigned long memory_map_size,
	unsigned long descriptor_size,
	u32 descriptor_version,
	efi_memory_desc_t *virtual_map);
#else
static inline void efi_thunk_runtime_setup(void) {}
static inline efi_status_t efi_thunk_set_virtual_address_map(
	void *phys_set_virtual_address_map,
	unsigned long memory_map_size,
	unsigned long descriptor_size,
	u32 descriptor_version,
	efi_memory_desc_t *virtual_map)
{
	return EFI_SUCCESS;
}
#endif /* CONFIG_EFI_MIXED */
#else
static inline void parse_efi_setup(u64 phys_addr, u32 data_len) {}
#endif /* CONFIG_EFI */

#endif /* _ASM_X86_EFI_H */
