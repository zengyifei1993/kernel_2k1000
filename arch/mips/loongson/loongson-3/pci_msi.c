#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/msi.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <loongson-pch.h>
#include <asm/mach-loongson/loongson.h>

static bool nomsix=1;
core_param(nomsix, nomsix, bool, 0664);
static bool nomsi=0;
core_param(nomsi, nomsi, bool, 0664);

int arch_setup_msi_irqs(struct pci_dev *dev, int nvec, int type)
{
	struct msi_desc *entry;
	int ret;

	if ((type == PCI_CAP_ID_MSIX && nomsix) || (type == PCI_CAP_ID_MSI && nomsi))
		return -ENOSPC;
	/*
	 * If an architecture wants to support multiple MSI, it needs to
	 * override arch_setup_msi_irqs()
	 */
	if (type == PCI_CAP_ID_MSI && nvec > 1)
		return 1;

	list_for_each_entry(entry, &dev->msi_list, list) {
		ret = loongson_pch->pch_setup_msi_irq(dev, entry);
		if (ret < 0)
			return ret;
		if (ret > 0)
			return -ENOSPC;
	}

	return 0;
}

int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	if(nomsi)
	 return -ENOSPC;
        return loongson_pch->pch_setup_msi_irq(pdev, desc);
}

void arch_teardown_msi_irq(unsigned int irq)
{
	loongson_pch->pch_teardown_msi_irq(irq);
}

static int __init lspci_msi_init(void)
{
	if (cpu_guestmode) {
		nomsix = 0;
		nomsi = 0;
	}

	return 0;
}

arch_initcall(lspci_msi_init);
