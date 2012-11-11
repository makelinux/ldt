/*
 *	LDT - Linux Driver Template
 *
 *	Copyright (C) 2012 Constantine Shulyupin http://www.makelinux.net/
 *
 *	Dual BSD/GPL License
 *
 *
 *	The driver demonstrates usage of following Linux facilities:
 *
 *	Linux kernel module
 *	file_operations
 *		read and write (UART)
 *		blocking read
 *		polling
 *		mmap
 *		ioctl
 *	kfifo
 *	completion
 *	interrupt
 *	tasklet
 *	timer
 *	work
 *	kthread
 *	simple single misc device file (miscdevice, misc_register)
 *	multiple char device files (alloc_chrdev_region)
 *	debugfs
 *	platform_driver and platform_device in another module
 *	simple UART driver on port 0x3f8 with IRQ 4
 *	Device Model (class, device)
 *	Power Management (dev_pm_ops)
 *	Device Tree (of_device_id)
 *
 *	TODO:
 *	classic tracing
 *	linked list
 *	private instance state struct
 *
 */

#include <linux/io.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/kfifo.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/serial_reg.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mod_devicetable.h>
#include "tracing.h"

static char ldt_name[] = KBUILD_MODNAME;
static int bufsize = PFN_ALIGN(16 * 1024);
static void *in_buf;
static void *out_buf;
static int uart_detected;
void *port_ptr;

static int port;
module_param(port, int, 0);
static int port_size;
module_param(port_size, int, 0);

static int irq;
module_param(irq, int, 0);

static int loopback;
module_param(loopback, int, 0);

/*
 *	print_context prints execution context:
 *	hard interrupt, soft interrupt or scheduled task
 */

/*
 *	ctracer_cut_path - return filename without path
 */

static int isr_counter;
static int ldt_work_counter;

#define FIFO_SIZE 128		/* should be power of two */
static DEFINE_KFIFO(in_fifo, char, FIFO_SIZE);
static DEFINE_KFIFO(out_fifo, char, FIFO_SIZE);

static DECLARE_WAIT_QUEUE_HEAD(ldt_readable);

static spinlock_t fifo_lock;

/*
 *	ldt_received - called with data received from HW port
 *	Called from tasklet, which is fired from ISR or timer
 */

static void ldt_received(void *data, int size)
{
	kfifo_in_spinlocked(&in_fifo, data, size, &fifo_lock);
	wake_up_interruptible(&ldt_readable);
}

/*
 *	ldt_send - send data to HW port or emulated SW loopback
 */

static void ldt_send(void *data, int size)
{
	if (uart_detected) {
		if (ioread8(port_ptr + UART_LSR) & UART_LSR_THRE)
			iowrite8(*(char *)data, port_ptr + UART_TX);
		else
			trace_msg("overflow");
	} else
		/* emulate loopback  */
	if (loopback)
		ldt_received(data, size);
}

/*
 *	work
 */

/* Fictive label _entry is used for tracing */

static void ldt_work_func(struct work_struct *work)
{
_entry:;
	once(print_context());
	ldt_work_counter++;
}

DECLARE_WORK(ldt_work, ldt_work_func);

/*
 *	tasklet
 */

static DECLARE_COMPLETION(ldt_complete);

#define tx_ready()	(ioread8(port_ptr + UART_LSR) & UART_LSR_THRE)
#define rx_ready()	(ioread8(port_ptr + UART_LSR) & UART_LSR_DR)

static void ldt_tasklet_func(unsigned long d)
{
	char data_out, data_in;
_entry:
	once(print_context());
	if (uart_detected) {
		while (tx_ready() && kfifo_out_spinlocked(&out_fifo, &data_out, sizeof(data_out), &fifo_lock)) {
			trace_loc();
			trace_hex(ioread8(port_ptr + UART_LSR));
			trace_dec(data_out);
			if (data_out >= 32)
				printk(KERN_CONT"'%c' ", data_out);
			ldt_send(&data_out, sizeof(data_out));
		}
		while (rx_ready()) {
			trace_loc();
			trace_hex(ioread8(port_ptr + UART_LSR));
			data_in = ioread8(port_ptr + UART_RX);
			trace_dec(data_in);
			if (data_in >= 32)
				printk(KERN_CONT"'%c' ", data_in);
			ldt_received(&data_in, sizeof(data_in));
		}
	} else {
		while (kfifo_out_spinlocked(&out_fifo, &data_out, sizeof(data_out), &fifo_lock)) {
			trace_loc();
			trace_dec_ln(data_out);
			ldt_send(&data_out, sizeof(data_out));
		}
	}
	schedule_work(&ldt_work);
	complete(&ldt_complete);
}

static DECLARE_TASKLET(ldt_tasklet, ldt_tasklet_func, 0);

/*
 *	interrupt
 */

static irqreturn_t ldt_isr(int irq, void *dev_id, struct pt_regs *regs)
{
_entry:
	/*
	 *      UART interrupt is not fired in loopback mode,
	 *      therefore fire ldt_tasklet from timer too
	 */
	once(print_context());
	isr_counter++;
	trace_loc();
	trace_hex(ioread8(port_ptr + UART_FCR));
	trace_hex(ioread8(port_ptr + UART_IIR));
	trace_ln();
	tasklet_schedule(&ldt_tasklet);
	return IRQ_HANDLED;	/* our IRQ */
}

/*
 *	timer
 */

static struct timer_list ldt_timer;
static void ldt_timer_func(unsigned long data)
{
_entry:
	/*
	 *      this timer is used just to fire ldt_tasklet,
	 *      when there is no interrupt in loopback mode
	 */
	if (loopback)
		tasklet_schedule(&ldt_tasklet);
	mod_timer(&ldt_timer, jiffies + HZ / 100);
}

static DEFINE_TIMER(ldt_timer, ldt_timer_func, 0, 0);

/*
 *	file_operations
 */

static int ldt_open(struct inode *inode, struct file *file)
{
_entry:
	print_context();
	trace_dec(imajor(inode));
	trace_dec(iminor(inode));
	trace_hex(file->f_flags & O_NONBLOCK);
	trace_ln();
	return 0;
}

static int ldt_release(struct inode *inode, struct file *file)
{
_entry:
	print_context();
	trace_dec(imajor(inode));
	trace_dec(iminor(inode));
	trace_dec(isr_counter);
	trace_dec(ldt_work_counter);
	trace_ln();
	return 0;
}

/*
 *	read
 */

static DEFINE_MUTEX(read_lock);

static ssize_t ldt_read(struct file *file, char __user * buf, size_t count, loff_t * ppos)
{
	int ret;
	unsigned int copied;
_entry:
	if (kfifo_is_empty(&in_fifo)) {
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto exit;
		} else {
			trace_msg("waiting");
			ret = wait_event_interruptible(ldt_readable, !kfifo_is_empty(&in_fifo));
			if (ret == -ERESTARTSYS) {
				trace_msg("interrupted");
				ret = -EINTR;
				goto exit;
			}
		}
	}
	if (mutex_lock_interruptible(&read_lock)) {
		trace_msg("interrupted");
		return -EINTR;
	}
	ret = kfifo_to_user(&in_fifo, buf, count, &copied);
	mutex_unlock(&read_lock);
exit:
	return ret ? ret : copied;
}

/*
 *	write
 */

static DEFINE_MUTEX(write_lock);

static ssize_t ldt_write(struct file *file, const char __user * buf, size_t count, loff_t * ppos)
{
	int ret;
	unsigned int copied;
_entry:
	/* TODO: wait_event_interruptible ... ldt_writeable */
	if (mutex_lock_interruptible(&write_lock))
		return -EINTR;
	ret = kfifo_from_user(&out_fifo, buf, count, &copied);
	mutex_unlock(&write_lock);
	tasklet_schedule(&ldt_tasklet);
	return ret ? ret : copied;
}

/*
 *	polling
 */

static unsigned int ldt_poll(struct file *file, poll_table * pt)
{
	unsigned int mask = 0;
_entry:
	poll_wait(file, &ldt_readable, pt);
	/*poll_wait(file, ldt_writeable, pt); TODO */

	if (!kfifo_is_empty(&in_fifo))
		mask |= POLLIN | POLLRDNORM;
	mask |= POLLOUT | POLLWRNORM;
#if 0
	mask |= POLLHUP;	/* on output eof */
	mask |= POLLERR;	/* on output error */
#endif
	trace_loc();
	trace_hex(mask);
	trace_ln();
	return mask;
}

/*
 *	pages_flag - set or clear a flag for sequence of pages
 *
 *	more generic solution instead SetPageReserved, ClearPageReserved etc
 */

void pages_flag(struct page *page, int pages, int mask, int value)
{
	for (; pages; pages--, page++)
		if (value)
			__set_bit(mask, &page->flags);
		else
			__clear_bit(mask, &page->flags);
}

/*
 *	mmap
 */
static int ldt_mmap(struct file *filp, struct vm_area_struct *vma)
{
	void *buf = NULL;
_entry:
	if (vma->vm_flags & VM_WRITE)
		buf = in_buf;
	else if (vma->vm_flags & VM_READ)
		buf = out_buf;
	if (!buf)
		return -EINVAL;
	if (remap_pfn_range(vma, vma->vm_start, virt_to_phys(buf) >> PAGE_SHIFT,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		trace_msg("remap_pfn_range failed");
		return -EAGAIN;
	}
	return 0;
}

/*
 *	ioctl
 */

#define trace_ioctl(nr) printk("ioctl=(%c%c %c #%i %i)\n", \
	(_IOC_READ & _IOC_DIR(nr)) ? 'r' : ' ', (_IOC_WRITE & _IOC_DIR(nr)) ? 'w' : ' ', \
	_IOC_TYPE(nr), _IOC_NR(nr), _IOC_SIZE(nr))

static long ldt_ioctl(struct file *f, unsigned int cmnd, unsigned long arg)
{
	void __user *user = (void *)arg;
_entry:
	trace_loc();
	trace_hex(cmnd);
	trace_hex(arg);
	trace_ln();
	trace_ioctl(cmnd);
	if (_IOC_DIR(cmnd) == _IOC_WRITE) {
		copy_from_user(in_buf, user, _IOC_SIZE(cmnd));
		memcpy(out_buf, in_buf, bufsize);
		memset(in_buf, 0, bufsize);
	}
	if (_IOC_DIR(cmnd) == _IOC_READ) {
		copy_to_user(user, out_buf, _IOC_SIZE(cmnd));
		memset(out_buf, 0, bufsize);
	}
	switch (_IOC_TYPE(cmnd)) {
	case 'A':
		switch (_IOC_NR(cmnd)) {
		case 0:
			break;
		}
		break;
	}
	return 0;
}

static const struct file_operations ldt_fops = {
	.owner = THIS_MODULE,
	.open = ldt_open,
	.release = ldt_release,
	.read = ldt_read,
	.write = ldt_write,
	.poll = ldt_poll,
	.mmap = ldt_mmap,
	.unlocked_ioctl = ldt_ioctl,
};

#ifdef USE_MISCDEV
/*
 *	use miscdevice for single instance device
 */
static struct miscdevice ldt_miscdev = {
	MISC_DYNAMIC_MINOR,
	ldt_name,
	&ldt_fops,
};
#else
/*
 *	used cdev and device for multiple instances device
 */

static int devs = 8;
module_param(devs, int, 0);

static struct cdev ldt_cdev;
static struct class *ldt_class;
static struct device *ldt_dev;
#if 0
static char *ldt_devnode(struct device *dev, umode_t * mode)
{
	if (mode)
		*mode = S_IRUGO | S_IWUGO;
	/* *mode = 0666; */
	return NULL;
}
#endif
#endif

/*
 *	kthread
 */

static int ldt_thread_sub(void *data)
{
	int ret = 0;
_entry:
	/*
	   perform here a useful work in task context
	 */
	return ret;
}

static int ldt_thread(void *data)
{
	int ret = 0;
_entry:
	print_context();
	allow_signal(SIGINT);
	while (!kthread_should_stop()) {
		ret = wait_for_completion_interruptible(&ldt_complete);
		if (ret == -ERESTARTSYS) {
			trace_msg("interrupted");
			ret = -EINTR;
			break;
		}
		ret = ldt_thread_sub(data);
	}
	return ret;
}

/*
 *	UART
 */

static struct resource *port_r;

static int uart_probe(void)
{
	int ret = 0;
	if (port) {
		port_ptr = ioport_map(port, port_size);
		trace_hex(port_ptr);
		port_r = request_region(port, port_size, ldt_name);
		trace_hex(port_r);
		trace_ln();
		/* ignore error */
	}
	if (irq) {
		ret = check(request_irq(irq, (void *)ldt_isr, IRQF_SHARED, ldt_name, THIS_MODULE));
		iowrite8(UART_MCR_RTS | UART_MCR_OUT2 | UART_MCR_LOOP, port_ptr + UART_MCR);
		uart_detected = (ioread8(port_ptr + UART_MSR) & 0xF0) == (UART_MSR_DCD | UART_MSR_CTS);
		trace_hex(ioread8(port_ptr + UART_MSR));
		trace_ln();

		if (uart_detected) {
			/*iowrite8(UART_IER_MSI | UART_IER_THRI |  UART_IER_RDI | UART_IER_RLSI, port_ptr + UART_IER); */
			iowrite8(UART_IER_RDI | UART_IER_RLSI | UART_IER_THRI, port_ptr + UART_IER);
			iowrite8(UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2, port_ptr + UART_MCR);
			iowrite8(UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, port_ptr + UART_FCR);
			trace_dec_ln(loopback);
			if (loopback)
				iowrite8(ioread8(port_ptr + UART_MCR) | UART_MCR_LOOP, port_ptr + UART_MCR);
		}
		if (!uart_detected && loopback) {
			pr_warn("Emulating loopback in software\n");
			ret = -ENODEV;
		}
	}
	trace_hex(uart_detected);
	trace_hex(ioread8(port_ptr + UART_IER));
	trace_hex(ioread8(port_ptr + UART_IIR));
	trace_hex(ioread8(port_ptr + UART_FCR));
	trace_ln();
	trace_hex(ioread8(port_ptr + UART_LCR));
	trace_hex(ioread8(port_ptr + UART_MCR));
	trace_hex(ioread8(port_ptr + UART_LSR));
	trace_hex(ioread8(port_ptr + UART_MSR));
	trace_ln();
	return ret;
}

static struct task_struct *thread;
static struct dentry *debugfs;
static int major;

int chrdev_region_init(char *dev_name)
{
	int ret;
	int d;
	dev_t devid;
_entry:
	devid = MKDEV(major, 0);
	ret = check(alloc_chrdev_region(&devid, 0, devs, dev_name));
	major = MAJOR(devid);
	trace_dec_ln(major);
	cdev_init(&ldt_cdev, &ldt_fops);
	check(cdev_add(&ldt_cdev, MKDEV(major, 0), devs));
	ldt_class = class_create(THIS_MODULE, dev_name);
	/* ldt_class->devnode = ldt_devnode; */
	ldt_dev = device_create(ldt_class, NULL, devid, NULL, "%s", dev_name);
	for (d = 1; d < devs; d++)
		device_create(ldt_class, NULL, MKDEV(major, d), NULL, "%s%d", dev_name, d);
	trace_dec_ln(IS_ERR(ldt_dev));
	trace_hex(ldt_dev);
	trace_ln();
	return major;
}

/*
 *	ldt_probe - main initialization function
 */

static __devinit int ldt_probe(struct platform_device *pdev)
{
	int ret;
	char *data = NULL;
	struct resource *r;
_entry:
	print_context();
	trace_loc();
	printk(KERN_DEBUG"%s %s %s", ldt_name, __DATE__, __TIME__);
	trace_loc();
	printk(KERN_DEBUG"pdev = %p ", pdev);
	trace_dec(irq);
	trace_dec(bufsize);
	trace_ln();
	in_buf = alloc_pages_exact(bufsize, GFP_KERNEL | __GFP_ZERO);
	if (!in_buf) {
		ret = -ENOMEM;
		goto exit;
	}
	pages_flag(virt_to_page(in_buf), PFN_UP(bufsize), PG_reserved, 1);
	out_buf = alloc_pages_exact(bufsize, GFP_KERNEL | __GFP_ZERO);
	if (!out_buf) {
		ret = -ENOMEM;
		goto exit;
	}
	pages_flag(virt_to_page(out_buf), PFN_UP(bufsize), PG_reserved, 1);
	if (pdev) {
		dev_dbg(&pdev->dev, "%s:%d %s attaching driver\n", __file__, __LINE__, __func__);
		trace_hex(pdev->dev.of_node);
		trace_ln();
#ifdef CONFIG_OF_DEVICE
		check(of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev));
#endif
		data = pdev->dev.platform_data;
		printk("%p %s\n", data, data);
		if (!irq)
			irq = platform_get_irq(pdev, 0);
		r = platform_get_resource(pdev, IORESOURCE_IO, 0);
		if (r && !port)
			port = r->start;

		if (r && !port_size)
			port_size = resource_size(r);
	}
	isr_counter = 0;
	uart_probe();
	/* proc_create(ldt_name, 0, NULL, &ldt_fops); depricated */
	mod_timer(&ldt_timer, jiffies + HZ / 10);
	thread = kthread_run(ldt_thread, NULL, "%s", ldt_name);
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		if (ret)
			goto exit;
	}
	debugfs = debugfs_create_file(ldt_name, S_IRUGO, NULL, NULL, &ldt_fops);
#ifdef USE_MISCDEV
	ret = check(misc_register(&ldt_miscdev));
	if (ret < 0)
		goto exit;
	trace_dec_ln(ldt_miscdev.minor);
#else
	chrdev_region_init(ldt_name);
#endif
exit:
	trace_loc();
	trace_dec_ln(ret);
	return ret;
}

/*
 *	ldt_remove - main clean up function
 */

static int __devexit ldt_remove(struct platform_device *pdev)
{
	int d;
_entry:
	if (pdev)
		dev_dbg(&pdev->dev, "%s:%d %s detaching driver\n", __file__, __LINE__, __func__);
	/* remove_proc_entry(ldt_name, NULL); depricated */
	if (debugfs)
		debugfs_remove(debugfs);
#ifdef USE_MISCDEV
	misc_deregister(&ldt_miscdev);
#else
	for (d = 0; d < devs; d++)
		device_destroy(ldt_class, MKDEV(major, d));
	class_destroy(ldt_class);
	cdev_del(&ldt_cdev);
	unregister_chrdev_region(MKDEV(major, 0), devs);
#endif
	if (!IS_ERR_OR_NULL(thread)) {
		send_sig(SIGINT, thread, 1);
		kthread_stop(thread);
	}
	del_timer(&ldt_timer);
	if (port_r)
		release_region(port, port_size);
	if (irq) {
		iowrite8(0, port_ptr + UART_IER);
		iowrite8(0, port_ptr + UART_FCR);
		iowrite8(0, port_ptr + UART_MCR);
		ioread8(port_ptr + UART_RX);
		free_irq(irq, THIS_MODULE);
	}
	tasklet_kill(&ldt_tasklet);
	if (in_buf) {
		pages_flag(virt_to_page(in_buf), PFN_UP(bufsize), PG_reserved, 0);
		free_pages_exact(in_buf, bufsize);
	}
	if (out_buf) {
		pages_flag(virt_to_page(out_buf), PFN_UP(bufsize), PG_reserved, 0);
		free_pages_exact(out_buf, bufsize);
	}
	trace_dec_ln(isr_counter);
	trace_dec_ln(ldt_work_counter);
	if (port_ptr)
		ioport_unmap(port_ptr);
	return 0;
}

#ifdef USE_PLATFORM_DEVICE

/*
 * Following code requires platform_device (ldt_plat_dev.*) to work
 */

#ifdef CONFIG_PM

static int ldt_suspend(struct device *dev)
{
	return 0;
}

static int ldt_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops ldt_pm = {
	.suspend = ldt_suspend,
	.resume = ldt_resume,
};

#define ldt_pm_ops (&ldt_pm)
#else
#define ldt_pm_ops NULL
#endif

static const struct of_device_id ldt_of_match[] = {
	{.compatible = "linux-driver-template",},
	{},
};

MODULE_DEVICE_TABLE(of, ldt_of_match);

static struct platform_driver ldt_driver = {
	.driver = {
		   .name = "ldt_device_name",
		   .owner = THIS_MODULE,
		   .pm = ldt_pm_ops,
		   .of_match_table = of_match_ptr(ldt_of_match),
		   },
	.probe = ldt_probe,
	.remove = __devexit_p(ldt_remove),
};

#ifdef module_platform_driver
module_platform_driver(ldt_driver);
#else

/*
 *	for Linux kernel releases before v3.1-12
 *	without macro module_platform_driver
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
#endif /* module_platform_driver */

#else /* !USE_PLATFORM_DEVICE */

/*
 *	Standalone module initialization to run without platform_device
 */

static int ldt_init(void)
{
	int ret = 0;
_entry:
	/*
	 *      Call probe function directly,
	 *      bypassing platform_device infrastructure
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
