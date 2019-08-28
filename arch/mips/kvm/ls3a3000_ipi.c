/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * currently used for Loongson3A Virtual IPI interrupt.
 * Mapped into the guest kernel @ KVM_GUEST_COMMPAGE_ADDR.
 *
 * Copyright (C) 2019  Loongson Technologies, Inc.  All rights reserved.
 * Authors: Chen Zhu <zhuchen@loongson.cn>
 */

#include "ls3a3000_ipi.h"
#include "ls3a3000.h"

void ls3a_ipi_lock(struct loongson_kvm_ls3a_ipi *s, unsigned long *flags)
{
	unsigned long tmp;
	spin_lock_irqsave(&s->lock, tmp);
	*flags = tmp;
}

void ls3a_ipi_unlock(struct loongson_kvm_ls3a_ipi *s, unsigned long *flags)
{
	unsigned long tmp;
	tmp = *flags;
	spin_unlock_irqrestore(&s->lock, tmp);
}

int ls3a_gipi_writel(struct loongson_kvm_ls3a_ipi * ipi, gpa_t addr, int len,const void *val)
{
	uint64_t data,offset;
	struct kvm_mips_interrupt irq;
	gipiState * s = &(ipi->ls3a_gipistate);
	uint32_t coreno = (addr >> 8) & 3;
	uint32_t node, no;
	struct kvm * kvm;

	kvm = ipi->kvm;
	node = ( addr >> kvm->arch.node_shift) & 3;
	no = coreno + node * 4;

	data = *(uint64_t *)val;
	offset = addr&0xFF;
	switch (offset) {
		case CORE0_STATUS_OFF:
			printk("CORE0_SET_OFF Can't be write\n");
			break;

		case CORE0_EN_OFF:
			s->core[no].en = data;
			break;

		case CORE0_SET_OFF:
			s->core[no].status |= data;
			irq.cpu = no;
			irq.irq = 6;
			kvm_vcpu_ioctl_interrupt(kvm->vcpus[no],&irq);
			break;

		case CORE0_CLEAR_OFF:
			s->core[no].status ^= data;
			if(!s->core[no].status){
				irq.cpu = no;
				irq.irq = -6;
				kvm_vcpu_ioctl_interrupt(kvm->vcpus[no],&irq);
			}
			break;

		case 0x20 ... 0x3c:
			s->core[no].buf[(offset - 0x20) / 8] = data;
			break;

		default:
			break;
	}
	return 0;
}

uint64_t ls3a_gipi_readl(struct loongson_kvm_ls3a_ipi * ipi, gpa_t addr, int len, void* val)
{
	uint64_t offset;
	uint64_t ret = 0;

	gipiState * s = &(ipi->ls3a_gipistate);
	uint32_t node, no;
	uint32_t coreno;

	coreno = (addr >> 8 ) & 3;
	node = (addr >> ipi->kvm->arch.node_shift) & 3;
	no = coreno + node *4;

	offset = addr&0xFF;

	switch(offset) {
		case CORE0_STATUS_OFF:
			ret = s->core[no].status;
			break;

		case CORE0_EN_OFF:
			ret = s->core[no].en;
			break;

		case CORE0_SET_OFF:
			ret = 0;
			break;

		case CORE0_CLEAR_OFF:
			ret = 0;
			break;

		case 0x20 ... 0x3c:
			ret = s->core[no].buf[(offset - 0x20) / 8];
			break;

		default:
			break;
	}

	*(uint64_t *)val = ret;

	return ret;
}

static int kvm_ls3a_ipi_write(struct kvm_io_device * dev,
		gpa_t addr, int len, const void *val)
{
	struct loongson_kvm_ls3a_ipi *ipi;
	ipi = container_of(dev, struct loongson_kvm_ls3a_ipi, dev_ls3a_ipi);
	return ls3a_gipi_writel(ipi,addr,len,val);
}


static int kvm_ls3a_ipi_read(struct kvm_io_device *dev,
		gpa_t addr, int len, void *val)
{
	struct loongson_kvm_ls3a_ipi *ipi;
	uint64_t result=0;

	ipi = container_of(dev, struct loongson_kvm_ls3a_ipi, dev_ls3a_ipi);
	result = ls3a_gipi_readl(ipi,addr,len,val);
	return 0;
}


static const struct kvm_io_device_ops kvm_ls3a_ipi_ops = {
	.read     = kvm_ls3a_ipi_read,
	.write    = kvm_ls3a_ipi_write,
};



struct loongson_kvm_ls3a_ipi * kvm_create_ls3a_ipi(struct kvm *kvm)
{
	struct loongson_kvm_ls3a_ipi *s;
	int ret;

	s = kzalloc(sizeof(struct loongson_kvm_ls3a_ipi), GFP_KERNEL);
	if (!s)
		return NULL;
	spin_lock_init(&s->lock);
	s->kvm = kvm;

	/*
	 * Initialize PIO device
	 */
	kvm_iodevice_init(&s->dev_ls3a_ipi, &kvm_ls3a_ipi_ops);
	mutex_lock(&kvm->slots_lock);
	ret = kvm_io_bus_register_dev(kvm, KVM_PIO_BUS, SMP_MAILBOX, 0x3FF,
			&s->dev_ls3a_ipi);
	if (ret < 0)
		goto fail_unlock;
	mutex_unlock(&kvm->slots_lock);
	return s;

fail_unlock:
	mutex_unlock(&kvm->slots_lock);
	kfree(s);

	return NULL;
}

int kvm_get_ls3a_ipi(struct kvm *kvm, struct loongson_gipiState *state)
{
	struct loongson_kvm_ls3a_ipi *ipi = ls3a_ipi_irqchip(kvm);
	gipiState *ipi_state =  &(ipi->ls3a_gipistate);
	unsigned long flags;

	ls3a_ipi_lock(ipi, &flags);
	memcpy(state, ipi_state, sizeof(gipiState));
	ls3a_ipi_unlock(ipi, &flags);
	return 0;
}

int kvm_set_ls3a_ipi(struct kvm *kvm, struct loongson_gipiState *state)
{
	struct loongson_kvm_ls3a_ipi *ipi = ls3a_ipi_irqchip(kvm);
	gipiState *ipi_state =  &(ipi->ls3a_gipistate);
	unsigned long flags;

	if (!ipi)
		return -EINVAL;

	ls3a_ipi_lock(ipi, &flags);
	memcpy(ipi_state, state, sizeof(gipiState));
	ls3a_ipi_unlock(ipi, &flags);
	return 0;
}


void kvm_destroy_ls3a_ipi(struct loongson_kvm_ls3a_ipi *vipi)
{
	kvm_io_bus_unregister_dev(vipi->kvm, KVM_PIO_BUS, &vipi->dev_ls3a_ipi);
	kfree(vipi);
}
