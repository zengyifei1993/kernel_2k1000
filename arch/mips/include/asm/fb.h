#ifndef _ASM_FB_H_
#define _ASM_FB_H_

#include <linux/fb.h>
#include <linux/fs.h>
#include <asm/page.h>
#include <asm/cpu-type.h>
#ifndef CONFIG_CPU_LOONGSON2K
#include <loongson.h>
#endif

static inline void fb_pgprotect(struct file *file, struct vm_area_struct *vma,
				unsigned long off)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
}

static inline int fb_is_primary_device(struct fb_info *info)
{
#ifndef CONFIG_CPU_LOONGSON2K
	if(cpu_guestmode)
		return 1;
	else
#endif
		return 0;
}

#endif /* _ASM_FB_H_ */
