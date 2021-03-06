/****************************************************************************
*
*    Copyright (C) 2005 - 2013 by Vivante Corp.
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*****************************************************************************/

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include "platform_driver.h"
#ifdef CONFIG_CPU_LOONGSON2K
#include "ls2k.h"
#else
#include <loongson-pch.h>
#endif


#define DEVICE_NAME "galcore"

int lsgpu_hw_coherent = 0;	/* set loongsong gpu use or not use hw coherent */
EXPORT_SYMBOL_GPL(lsgpu_hw_coherent);

#ifdef CONFIG_OF
static struct of_device_id ls_gpu_id_table[] = {
	{ .compatible = "loongson,galcore", },
	{},
};
#endif


static struct platform_driver gpu_driver = {
	.probe		= loongson_gpu_probe,
	.remove		= loongson_gpu_remove,
	.suspend	= loongson_gpu_suspend,
	.resume		= loongson_gpu_resume,
	.driver		= {
		   .name = DEVICE_NAME,
		   .bus = &platform_bus_type,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ls_gpu_id_table),
#endif
	}
};

static struct pci_device_id loongson_gpu_devices[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_GPU)},
	{0, 0, 0, 0, 0, 0, 0}
};



static struct resource loongson_gpu_resources[] = {
	[0] = {
		.name	= "gpu_base",
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "gpu_irq",
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.name	= "gpu_mem",
		.start	= 0x0000a000000,
		.end	= 0x0000effffff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device loongson_gpu_device = {
	.name           = "galcore",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(loongson_gpu_resources),
	.resource       = loongson_gpu_resources,

};



static int loongson_gpu_pci_register(struct pci_dev *pdev,
				 const struct pci_device_id *ent)

{
	int ret;

	pr_debug("loongson_gpu_pci_register BEGIN\n");

	/* Enable device in PCI config */
	ret = pci_enable_device(pdev);
	if (ret < 0) {
		printk(KERN_ERR "loongson gpu (%s): Cannot enable PCI device\n",
		       pci_name(pdev));
		goto err_out;
	}

	/* request the mem regions */
	ret = pci_request_region(pdev, 0, "loongson gpu io");
	if (ret < 0) {
		printk( KERN_ERR "loongsongpu fb (%s): cannot request region 0.\n",
			pci_name(pdev));
		goto err_out;
	}

	loongson_gpu_resources[0].start = pci_resource_start (pdev, 0);
	loongson_gpu_resources[0].end = pci_resource_end(pdev, 0);
	loongson_gpu_resources[1].start = pdev->irq;
	loongson_gpu_resources[1].end = pdev->irq;
#ifdef LS_VRAM_UEFI
	if(vuma_vram_addr != 0){
		loongson_gpu_resources[2].start = uma_vram_addr;
		loongson_gpu_resources[2].end = uma_vram_addr + uma_vram_size - 1;
	}
#endif
#ifdef LS_VRAM_ADD_BY_PMON
	loongson_gpu_resources[2].start = pci_resource_start (pdev, 2);
	loongson_gpu_resources[2].end = pci_resource_end(pdev, 2) - 0x01100000;

	ls2k_gpu_resources[2].start = pci_resource_start (pdev, 2);
	ls2k_gpu_resources[2].end = pci_resource_end(pdev, 2);
#endif

#ifndef CONFIG_CPU_LOONGSON2K
	if(loongson_pch->board_type == LS7A){
		vram_type = VRAM_TYPE_UMA_LOW;
		gpu_brust_type = 1;
		lsgpu_hw_coherent = hw_coherentio;
	}
	printk("lsgpu_hw_coherent = %d\r\n",lsgpu_hw_coherent);
	platform_device_register(&loongson_gpu_device);
#endif
	return 0;
err_out:
	return ret;
}

static void loongson_gpu_pci_unregister(struct pci_dev *pdev)
{

	platform_device_unregister(&loongson_gpu_device);
	pci_release_region(pdev, 0);
}

int loongson_gpu_pci_suspend(struct pci_dev *pdev, pm_message_t mesg)
{
	pci_save_state(pdev);
	return 0;
}

int loongson_gpu_pci_resume(struct pci_dev *pdev)
{
	return 0;
}


static struct pci_driver loongson_gpu_pci_driver = {
	.name		= "loongson-gpu",
	.id_table	= loongson_gpu_devices,
	.probe		= loongson_gpu_pci_register,
	.remove		= loongson_gpu_pci_unregister,
#ifdef	CONFIG_SUSPEND
	.suspend = loongson_gpu_pci_suspend,
	.resume	 = loongson_gpu_pci_resume,
#endif
};

static int __init gpu_init(void)
{
	int ret;
	struct pci_dev *pdev = NULL;
	/*if PCIE Graphics card exist,use it as default*/
	pdev = pci_get_device(PCI_VENDOR_ID_ATI, PCI_ANY_ID, NULL);
	if(pdev)
			return 0;
	/*if PCIE Graphics card of aspeed ast2400 exist, use it as default*/
	pdev = pci_get_device(0x1a03, PCI_ANY_ID, NULL);
	if(pdev)
			return 0;
	ret = platform_driver_register(&gpu_driver);
	if(ret) return ret;
	ret = pci_register_driver (&loongson_gpu_pci_driver);
	return ret;

}

static void __exit gpu_exit(void)
{
	platform_driver_unregister(&gpu_driver);
	pci_unregister_driver (&loongson_gpu_pci_driver);
}

module_init(gpu_init);
module_exit(gpu_exit);
MODULE_DESCRIPTION("Loongson Graphics Driver");
MODULE_LICENSE("GPL");
