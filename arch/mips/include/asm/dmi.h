#ifndef _ASM_DMI_H__
#define _ASM_DMI_H__

#include <linux/efi.h>
#include <linux/slab.h>

#define dmi_early_remap     early_ioremap
#define dmi_early_unmap     early_unmap

#define dmi_remap     dmi_ioremap
#define dmi_unmap     dmi_iounmap

#define dmi_alloc(l)    kzalloc(l, GFP_KERNEL)


void __init __iomem *early_ioremap(u64 phys_addr, unsigned long size)
{
    if (phys_addr == 0xF0000)
        phys_addr = 0xfffe000;

    return ((void *)TO_CAC(phys_addr));
}

void __init __iomem *dmi_ioremap(u64 phys_addr, unsigned long size)
{
    return ((void *)TO_CAC(phys_addr));
}

void early_unmap(void __iomem *addr, unsigned long size)
{

}

void dmi_iounmap(volatile void __iomem *addr)
{

}

#endif /* _ASM_DMI_H */
