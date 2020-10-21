// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>

MODULE_DESCRIPTION("PCI Linux Driver Template");

/*
 * Usage:
 *
 * 1. Prefer to use a hypervisor, for example VirtualBox.
 * 2. Define or select a non-critical PCI device for use.
 *    For example, you can add Audio controller ICH AC97 in VirtualBox VM
 * 3. Using the command lspci, find a corresponding device driver.
 *    For example, ICH AC97 is used by driver snd_intel8x0.
 * 4. Remove the default driver of the selected device: sudo rmmod snd_intel8x0
 * 5. Compile and insert this driver: sudo insmod pci-ldt.ko
 *    The driver tries to bind to every PCI device and succeeds to bind
 *    only to unused devices, including the device selected above.
 * 6. Then remove this driver: sudo rmmod pci-ldt
 * 7. Analyze the kernel log: dmesg
 *    Successful driver usage acquires a base address and calls dummy interrupts.
 * 8. Explore the source.
 * 9. Read https://en.wikibooks.org/wiki/The_Linux_Kernel/PCI
 * 10. Improve the driver and send a pull request.
 */

MODULE_AUTHOR("Costa Shulyupin <costa@makelinux.net>, <constantine.shulyupin@gmail.com>");
MODULE_LICENSE("GPL v2");

#include <linux/version.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/msi.h>

#define CTRACER_ON
#include "ctracer.h"

#undef pr_fmt
#define pr_fmt(fmt)    "%s.c:%d %s " fmt, KBUILD_MODNAME, __LINE__, __func__

/**
 * struct instance_data - Internal data for one instance of device
 * @pcid: PCI device
 */

struct instance_data {
	struct pci_dev *pcid;
	/* Add here custom data of the driver */
	void __iomem *base;
};

static int irqs;

static irqreturn_t pci_ldt_irq(int irq, struct instance_data *d)
{
	if (!irqs) {
		dev_info(&d->pcid->dev, "First IRQ");
#if KERNEL_VERSION(5, 6, 0) <= LINUX_VERSION_CODE
		dev_info(&d->pcid->dev, "%x\n", pci_status_get_and_clear_errors(d->pcid));
#endif
	}
	++irqs;
	/* TODO: process interrupt */

	return IRQ_HANDLED;
}

static void pci_ldt_free(struct pci_dev *pcid)
{
	pci_free_irq_vectors(pcid);
	pcim_iounmap_regions(pcid, pci_select_bars(pcid, IORESOURCE_MEM));
	pci_disable_device(pcid);
}

static int pci_ldt_probe(struct pci_dev *pcid, const struct pci_device_id *ent)
{
	struct instance_data *data;
	u16 status = ~0;
	int ret;
	int bar;

	dev_dbg(&pcid->dev, "Probing %04X:%04X\n", pcid->vendor, pcid->device);

	ret = pci_enable_device_mem(pcid);
	if (ret) {
		dev_err(&pcid->dev, "pci_enable_device_mem %d\n", ret);
		goto error;
	}
	ret = pcim_iomap_regions(pcid, pci_select_bars(pcid, IORESOURCE_MEM), pci_name(pcid));
	if (ret) {
		dev_err(&pcid->dev, "pcim_iomap_regions %d\n", ret);
		goto error;
	}

	bar = ffs(pci_select_bars(pcid, IORESOURCE_MEM)) - 1;
	data = devm_kzalloc(&pcid->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto error;
	}
	data->pcid = pcid;
	pci_set_drvdata(pcid, data);
	ret = pci_alloc_irq_vectors(pcid, 1, 1, PCI_IRQ_ALL_TYPES);
	if (ret < 1) {
		goto error;
	}
	ret = pci_request_irq(pcid, 0, (void *)pci_ldt_irq, NULL, data, "%s", pci_name(pcid));
	if (ret) {
		dev_err(&pcid->dev, "request_irq failed\n");
		goto error;
	}


	dev_notice(&pcid->dev, "Added %04X:%04X\n", pcid->vendor, pcid->device);
	pci_read_config_word(pcid, PCI_STATUS, &status);
	trlvx(status);

	if (bar >= 0) {
		data->base = pcim_iomap_table(pcid)[bar];
		trvp(data->base);
		print_hex_dump(KERN_DEBUG, "regs:", DUMP_PREFIX_OFFSET,
                               16, 2, data->base, 64, 0);
	}

	return 0;
error:
	dev_dbg(&pcid->dev, "Error %d adding %04X:%04X\n", ret, pcid->vendor, pcid->device);
	pci_ldt_free(pcid);
	return ret;
}

static void pci_ldt_remove(struct pci_dev *pcid)
{
	trl();

	trlvd(irqs);
	pci_free_irq(pcid, 0, pci_get_drvdata(pcid));
	pci_ldt_free(pcid);
	dev_notice(&pcid->dev, "Removed %04X:%04X\n", pcid->vendor, pcid->device);
}

static const struct pci_device_id pci_ldt_ids[] = {
	/* For demonstration the driver will be probed with all devices,
	 *  and loaded only for unused by another driver.
	 *  TODO: Limit this IDs for your device.
	 */
	{ PCI_DEVICE(PCI_ANY_ID, PCI_ANY_ID) },
	{ }
};
MODULE_DEVICE_TABLE(pci, pci_ldt_ids);

static struct pci_driver pci_ldt_driver = {
	.name = KBUILD_MODNAME,
	.probe = pci_ldt_probe,
	.remove = pci_ldt_remove,
	.id_table = pci_ldt_ids,
};

module_pci_driver(pci_ldt_driver);
