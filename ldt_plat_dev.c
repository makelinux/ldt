/*
 *	LDT - Linux Driver Template
 *
 *	Copyright (C) 2012 Constantine Shulyupin  http://www.makelinux.net/
 *
 *	Dual BSD/GPL License
 *
 *	platform_device template driver
 *
 *	uses
 *
 *	platform_data
 *	resources
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>

static struct resource ldt_resource[] = {
	{
	 .flags = IORESOURCE_IO,
	 .start = 0x3f8,
	 .end = 0x3ff,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = 4,
	 .end = 4,
	 },
	{
	 .flags = IORESOURCE_MEM,
	 .start = 0,
	 .end = 0,
	 },
};

void ldt_dev_release(struct device *dev)
{
_entry:;
}

static struct platform_device ldt_platform_device = {
	.name = "ldt_device_name",
	.id = 0,
	.num_resources = ARRAY_SIZE(ldt_resource),
	.resource = ldt_resource,
	.dev.platform_data = "test data",
	.dev.release = ldt_dev_release,
};

static int ldt_plat_dev_init(void)
{
	int ret = 0;
_entry:
	ret = platform_device_register(&ldt_platform_device);
	trl_();
	trvd(ret);
	return ret;
}

static void ldt_plat_dev_exit(void)
{
_entry:
	platform_device_unregister(&ldt_platform_device);
}

module_init(ldt_plat_dev_init);
module_exit(ldt_plat_dev_exit);

MODULE_DESCRIPTION("LDT - Linux Driver Template: platform_device");
MODULE_AUTHOR("Constantine Shulyupin <const@makelinux.net>");
MODULE_LICENSE("Dual BSD/GPL");
