/*
 *	LDT - Linux Driver Template
 *
 *	Copyright (C) 2012 Constantine Shulyupin  http://www.makelinux.net/
 *
 *	Dual BSD/GPL License
 *
 *	platform_driver template driver
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include "tracing.h"

static __devinit int ldt_plat_probe(struct platform_device *pdev)
{
	char *data = NULL;
	struct resource *r;
_entry:
	if (pdev)
		data = pdev->dev.platform_data;
	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	return 0;
}

static int __devexit ldt_plat_remove(struct platform_device *pdev)
{
_entry:
	return 0;
}

static struct platform_driver ldt_plat_driver = {
	.driver.name = "ldt_device_name",
	.driver.owner = THIS_MODULE,
	.probe = ldt_plat_probe,
	.remove = __devexit_p(ldt_plat_remove),
};

#ifdef module_platform_driver
module_platform_driver(ldt_plat_driver);
#else
/*
 *	for releases before v3.1-12 without macro module_platform_driver
 */
static int ldt_plat_drv_init(void)
{
	int ret = 0;
_entry:
	ret = platform_driver_register(&ldt_plat_driver);
	return ret;
}

static void ldt_plat_drv_exit(void)
{
_entry:
	platform_driver_unregister(&ldt_plat_driver);
}

module_init(ldt_plat_drv_init);
module_exit(ldt_plat_drv_exit);
#endif

MODULE_DESCRIPTION("LDT - Linux Driver Template: platform_driver template");
MODULE_AUTHOR("Constantine Shulyupin <const@makelinux.net>");
MODULE_LICENSE("Dual BSD/GPL");
