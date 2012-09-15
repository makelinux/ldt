/*
 *	LDT - Linux Driver Template
 *
 *	Copyright (C) 2012 Constantine Shulyupin  http://www.makelinux.net/
 *
 *	Dual BSD/GPL License
 *
 *
 *	The driver demonstrates usage of following Linux facilities:
 *
 *	Linux kernel module
 *	file_operations read and write
 *	kfifo
 *	completion
 *	interrupt
 *	tasklet
 *	work
 *	timer
 *	misc device
 *	proc fs
 *	platform_driver and platform_device in another module
 */

#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <linux/kfifo.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/platform_device.h>

int irq = 0;
module_param(irq, int, 0);

/*
 * Prints execution context: hard interrupt, soft interrupt or scheduled task
 */

#define print_context()	\
	printk(KERN_DEBUG"%s:%d %s %s %x\n", __file__, __LINE__, __func__, \
			in_irq()?"hardirq":current->comm,preempt_count());

#define check(a) \
( ret=a,((ret<0)?tracef("%s:%i %s FAIL\n\t%i=%s\n",__FILE__,__LINE__,__FUNCTION__,ret,#a):0),ret)

static int isr_counter;
static int ldt_work_counter;

#define FIFO_SIZE 128		/* should be power of two */
static DEFINE_KFIFO(in_fifo, char, FIFO_SIZE);
static DEFINE_KFIFO(out_fifo, char, FIFO_SIZE);

spinlock_t fifo_lock;

/*	ldt_received - called with data received from HW port
 *	Called from interrupt or emulated function
 */
void ldt_received(void *data, int size)
{
	kfifo_in_spinlocked(&in_fifo, data, size, &fifo_lock);
}

/*	ldt_port_put - emulates sending data to HW port
 *
 */
void ldt_port_put(void *data, int size)
{
	/*
	 * emulate loop back port
	 */
	ldt_received(data, size);
}

static DECLARE_COMPLETION(ldt_comlete);

/* Fictive label _entry is used for tracing  */

int ldt_completed(void)
{
	int ret;
_entry:;
	once(print_context());
	ret = wait_for_completion_interruptible(&ldt_comlete);
	if (ret == -ERESTARTSYS) {
		ret = -EINTR;
		trlm("intrrupred");
	}
	return ret;
}

void ldt_work_func(struct work_struct *work)
{
_entry:;
	once(print_context());
	ldt_work_counter++;
}

DECLARE_WORK(ldt_work, ldt_work_func);

void ldt_tasklet_func(unsigned long data)
{
	int ret;
	char unit;
_entry:;
	once(print_context());
	ret = kfifo_out_spinlocked(&out_fifo, &unit, sizeof(unit), &fifo_lock);
	if (ret) {
		trl_();
		trvd(unit);
		ldt_port_put(&unit, sizeof(unit));
	}
	schedule_work(&ldt_work);
}

DECLARE_TASKLET(ldt_tasklet, ldt_tasklet_func, 0);

irqreturn_t ldt_isr(int irq, void *dev_id, struct pt_regs *regs)
{
_entry:;
	once(print_context());
	isr_counter++;
	tasklet_schedule(&ldt_tasklet);
	return IRQ_NONE;	/* not our IRQ */
	// return IRQ_HANDLED; /* our IRQ */
}

struct timer_list ldt_timer;
void ldt_timer_func(unsigned long data)
{
_entry:;
	tasklet_schedule(&ldt_tasklet);
	mod_timer(&ldt_timer, jiffies + HZ / 10);
}

DEFINE_TIMER(ldt_timer, ldt_timer_func, 0, 0);

static int ldt_open(struct inode *inode, struct file *file)
{
_entry:;
	return 0;
}

static int ldt_release(struct inode *inode, struct file *file)
{
_entry:;
	return 0;
}

static DEFINE_MUTEX(read_lock);

static ssize_t ldt_read(struct file *file, char __user * buf, size_t count, loff_t * ppos)
{
	int ret;
	unsigned int copied;
	// TODO: implement blocking I/O
	if (mutex_lock_interruptible(&read_lock))
		return -EINTR;
	ret = kfifo_to_user(&in_fifo, buf, count, &copied);
	mutex_unlock(&read_lock);
	return ret ? ret : copied;
}

static DEFINE_MUTEX(write_lock);

static ssize_t ldt_write(struct file *file, const char __user * buf, size_t count, loff_t * ppos)
{
	int ret;
	unsigned int copied;

	if (mutex_lock_interruptible(&write_lock))
		return -EINTR;
	ret = kfifo_from_user(&out_fifo, buf, count, &copied);
	mutex_unlock(&write_lock);
	return ret ? ret : copied;
}

struct file_operations ldt_fops = {
	.owner = THIS_MODULE,
	.open = ldt_open,
	.release = ldt_release,
	.read = ldt_read,
	.write = ldt_write,
	.poll = NULL,
};

static struct miscdevice ldt_miscdev = {
	MISC_DYNAMIC_MINOR,
	KBUILD_MODNAME,
	&ldt_fops,
};

static __devinit int ldt_probe(struct platform_device *pdev)
{
	int ret;
	char *data = NULL;
	struct resource *r;
_entry:
	print_context();
	trl_();
	trvs_(KBUILD_MODNAME);
	trvp_(pdev);
	trvd_(irq);
	trln();
	if (pdev) {
		data = pdev->dev.platform_data;
		r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
		if (!irq) {
			irq = r->start;
		}
	}
	trvp(data);
	trvs(data);
	ret = check(misc_register(&ldt_miscdev));
	//ret = check(register_chrdev (0, KBUILD_MODNAME, &ldt_fops));
	if (ret < 0)
		goto exit;
	trvd(ldt_miscdev.minor);
	isr_counter = 0;
	if (irq) {
		ret = check(request_irq(irq, (void *)ldt_isr, IRQF_SHARED, KBUILD_MODNAME, THIS_MODULE));
	}
	proc_create(KBUILD_MODNAME, 0, NULL, &ldt_fops);
	mod_timer(&ldt_timer, jiffies + HZ / 10);
exit:
	trl_();
	trvd(ret);
	return ret;
}

static int __devexit ldt_remove(struct platform_device *pdev)
{
_entry:
	remove_proc_entry(KBUILD_MODNAME, NULL);
	misc_deregister(&ldt_miscdev);
	del_timer(&ldt_timer);
	if (irq) {
		free_irq(irq, THIS_MODULE);
	}
	trvd(isr_counter);
	trvd(ldt_work_counter);
	return 0;
}

#ifdef USE_PLATFORM_DEVICE

/*
 * Folowing code requres platform_device (ldt_plat_dev.*) to work
 */

static struct platform_driver ldt_driver = {
	.driver.name = "ldt_device_name",
	.driver.owner = THIS_MODULE,
	.probe = ldt_probe,
	.remove = __devexit_p(ldt_remove),
};

#ifdef module_platform_driver
module_platform_driver(ldt_driver);
#else

/*
 *	for releases before v3.1-12 without macro module_platform_driver
 */

static int ldt_init(void)
{
	int ret = 0;
_entry:
	ret = platform_driver_register(&ldt_driver);
	return ret;
}

static void ldt_exit(void)
{
_entry:
	platform_driver_unregister(&ldt_driver);
}

module_init(ldt_init);
module_exit(ldt_exit);
#endif // module_platform_driver

#else // ! USE_PLATFORM_DEVICE

/*
 * Standalone module initialization to run without platform_device
 */

static int ldt_init(void)
{
	int ret = 0;
_entry:
	/*
	 * Call probe function directly, bypassing platform_device infrastructure
	 */
	ret = ldt_probe(NULL);
	return ret;
}

static void ldt_exit(void)
{
	int res;
_entry:
	res = ldt_remove(NULL);
}

module_init(ldt_init);
module_exit(ldt_exit);
#endif

MODULE_DESCRIPTION("LDT - Linux Driver Template");
MODULE_AUTHOR("Constantine Shulyupin <const@makelinux.net>");
MODULE_LICENSE("Dual BSD/GPL");
