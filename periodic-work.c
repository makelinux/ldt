// SPDX-License-Identifier: GPL-2.0-only

#include <linux/module.h>
MODULE_DESCRIPTION("periodic work with schedule_delayed_work");
#include <linux/device.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#undef pr_fmt
#define pr_fmt(fmt) "%s:%d: %s " fmt, __FILE__, __LINE__, __func__

static struct delayed_work work;

static void work_cb(struct work_struct *wp)
{
	pr_info("\n");
	schedule_delayed_work(&work, HZ);
}

static int periodic_work_init(void)
{
	int ret = 0;
	pr_debug("\n");
	INIT_DELAYED_WORK(&work, work_cb);
	schedule_delayed_work(&work, HZ);
	return ret;
}

static void periodic_work_exit(void)
{
	cancel_delayed_work_sync(&work);
}

module_init(periodic_work_init);
module_exit(periodic_work_exit);
MODULE_LICENSE("GPL");
