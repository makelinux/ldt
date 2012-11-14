/*
 *	LDT - Linux Driver Template
 *
 *	Copyright (C) 2012 Constantine Shulyupin http://www.makelinux.net/
 *
 *	GPL License
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
 *	Device Model
 *	Power Management (dev_pm_ops)
 *	Device Tree (of_device_id)
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
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/serial_reg.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mod_devicetable.h>

static int bufsize = 8 * PAGE_SIZE;
static void *in_buf;
static void *out_buf;
static int uart_detected;
static void __iomem *port_ptr;

static int port;
module_param(port, int, 0);
MODULE_PARM_DESC(port, "io port number, default 0x3f8 - UART");

static int port_size;
module_param(port_size, int, 0);
MODULE_PARM_DESC(port_size, "number of io ports, default 8");

static int irq;
module_param(irq, int, 0);
MODULE_PARM_DESC(irq, "interrupt request number, default 4 - UART");

static int loopback;
module_param(loopback, int, 0);
MODULE_PARM_DESC(loopback, "loopback mode for testing, default 0");

static int isr_counter;
static int ldt_work_counter;

#define FIFO_SIZE 128		/* must be power of two */
static DEFINE_KFIFO(in_fifo, char, FIFO_SIZE);
static DEFINE_KFIFO(out_fifo, char, FIFO_SIZE);

static DECLARE_WAIT_QUEUE_HEAD(ldt_readable);

static spinlock_t fifo_lock;

/*
 *	pages_flag - set or clear a flag for sequence of pages
 *
 *	more generic solution instead SetPageReserved, ClearPageReserved etc
 *
 * 	Popose to move pages_flag to linux/page-flags.h
 */

extern void pages_flag(struct page *page, int pages, int mask, int value);

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
			pr_err("%s: %s\n",__func__, "UART overflow");
	} else
		/* emulate loopback  */
	if (loopback)
		ldt_received(data, size);
}

/*
 *	work
 */

static void ldt_work_func(struct work_struct *work)
{
	ldt_work_counter++;
}

static DECLARE_WORK(ldt_work, ldt_work_func);

/*
 *	tasklet
 */

static DECLARE_COMPLETION(ldt_complete);

#define tx_ready()	(ioread8(port_ptr + UART_LSR) & UART_LSR_THRE)
#define rx_ready()	(ioread8(port_ptr + UART_LSR) & UART_LSR_DR)

static void ldt_tasklet_func(unsigned long d)
{
	char data_out, data_in;
	if (uart_detected) {
		while (tx_ready() && kfifo_out_spinlocked(&out_fifo, &data_out, sizeof(data_out), &fifo_lock)) {
			pr_debug("UART_LSR=0x%02X\n", ioread8(port_ptr + UART_LSR));
			pr_debug("%s: data_out=%d\n",__func__, data_out);
			if (data_out >= 32)
				pr_debug("data_out = '%c' ", data_out);
			ldt_send(&data_out, sizeof(data_out));
		}
		while (rx_ready()) {
			pr_debug("UART_LSR=0x%02X\n", ioread8(port_ptr + UART_LSR));
			data_in = ioread8(port_ptr + UART_RX);
			pr_debug("%s: data_in=%d\n",__func__, data_in);
			if (data_in >= 32)
				pr_debug("data_out = '%c' ", data_in);
			ldt_received(&data_in, sizeof(data_in));
		}
	} else {
		while (kfifo_out_spinlocked(&out_fifo, &data_out, sizeof(data_out), &fifo_lock)) {
			pr_debug("%s: data_out=%d\n",__func__, data_out);
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
	/*
	 *      UART interrupt is not fired in loopback mode,
	 *      therefore fire ldt_tasklet from timer too
	 */
	isr_counter++;
	pr_debug("UART_FCR=0x%02X\n", ioread8(port_ptr + UART_FCR));
	pr_debug("UART_IIR=0x%02X\n", ioread8(port_ptr + UART_IIR));
	tasklet_schedule(&ldt_tasklet);
	return IRQ_HANDLED;	/* our IRQ */
}

/*
 *	timer
 */

static struct timer_list ldt_timer;
static void ldt_timer_func(unsigned long data)
{
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
	pr_debug("%s: imajor(inode)=%d\n",__func__, imajor(inode));
	pr_debug("%s: iminor(inode)=%d\n",__func__, iminor(inode));
	pr_debug("%s: file->f_flags & O_NONBLOCK=0x%X\n",__func__, file->f_flags & O_NONBLOCK);
	return 0;
}

static int ldt_release(struct inode *inode, struct file *file)
{
	pr_debug("%s: imajor(inode)=%d\n",__func__, imajor(inode));
	pr_debug("%s: iminor(inode)=%d\n",__func__, iminor(inode));
	pr_debug("%s: isr_counter=%d\n",__func__, isr_counter);
	pr_debug("%s: ldt_work_counter=%d\n",__func__, ldt_work_counter);
	return 0;
}

/*
 *	read
 */

static DEFINE_MUTEX(read_lock);

static ssize_t ldt_read(struct file *file, char __user * buf, size_t count, loff_t * ppos)
{
	int ret = 0;
	unsigned int copied;
	if (kfifo_is_empty(&in_fifo)) {
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto exit;
		} else {
			pr_err("%s: ERR=%d %s\n",__func__, ret, "waiting");
			ret = wait_event_interruptible(ldt_readable, !kfifo_is_empty(&in_fifo));
			if (ret == -ERESTARTSYS) {
				pr_err("%s: ERR=%d %s\n",__func__, ret, "interrupted");
				ret = -EINTR;
				goto exit;
			}
		}
	}
	if (mutex_lock_interruptible(&read_lock)) {
		pr_err("%s: ERR=%d %s\n",__func__, ret, "interrupted");
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
	poll_wait(file, &ldt_readable, pt);
	/*poll_wait(file, ldt_writeable, pt); TODO */

	if (!kfifo_is_empty(&in_fifo))
		mask |= POLLIN | POLLRDNORM;
	mask |= POLLOUT | POLLWRNORM;
#if 0
	mask |= POLLHUP;	/* on output eof */
	mask |= POLLERR;	/* on output error */
#endif
	pr_debug("%s: mask=0x%X\n",__func__, mask);
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
	if (vma->vm_flags & VM_WRITE)
		buf = in_buf;
	else if (vma->vm_flags & VM_READ)
		buf = out_buf;
	if (!buf)
		return -EINVAL;
	if (remap_pfn_range(vma, vma->vm_start, virt_to_phys(buf) >> PAGE_SHIFT,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		pr_err("%s: %s\n",__func__, "remap_pfn_range failed");
		return -EAGAIN;
	}
	return 0;
}

/*
 *	ioctl
 */

#define trace_ioctl(nr) pr_debug("ioctl=(%c%c %c #%i %i)\n", \
	(_IOC_READ & _IOC_DIR(nr)) ? 'r' : ' ', (_IOC_WRITE & _IOC_DIR(nr)) ? 'w' : ' ', \
	_IOC_TYPE(nr), _IOC_NR(nr), _IOC_SIZE(nr))

static long ldt_ioctl(struct file *f, unsigned int cmnd, unsigned long arg)
{
	int ret = 0;
	void __user *user = (void __user *)arg;
	pr_debug("%s: cmnd=0x%X\n",__func__, cmnd);
	pr_debug("%s: arg=0x%lX\n",__func__, arg);
	trace_ioctl(cmnd);
	switch (_IOC_TYPE(cmnd)) {
		case 'A':
			switch (_IOC_NR(cmnd)) {
				case 0:
					if (_IOC_DIR(cmnd) == _IOC_WRITE) {
						if(copy_from_user(in_buf, user, _IOC_SIZE(cmnd))) {
							ret = -EFAULT;
							goto exit;
						}
						memcpy(out_buf, in_buf, bufsize);
						memset(in_buf, 0, bufsize);
					}
					if (_IOC_DIR(cmnd) == _IOC_READ) {
						if(copy_to_user(user, out_buf, _IOC_SIZE(cmnd))) {
							ret = -EFAULT;
							goto exit;
						}
						memset(out_buf, 0, bufsize);
					}
					break;
			}
			break;
	}
exit:
	return ret;
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

/*
 *	use miscdevice for single instance device
 */
static struct miscdevice ldt_miscdev = {
	MISC_DYNAMIC_MINOR,
	KBUILD_MODNAME,
	&ldt_fops,
};

/*
 *	kthread
 */

static int ldt_thread_sub(void *data)
{
	int ret = 0;
	/*
	   perform here a useful work in task context
	 */
	return ret;
}

static int ldt_thread(void *data)
{
	int ret = 0;
	allow_signal(SIGINT);
	while (!kthread_should_stop()) {
		ret = wait_for_completion_interruptible(&ldt_complete);
		if (ret == -ERESTARTSYS) {
			pr_err("%s: ERR=%d %s\n",__func__, ret, "interrupted");
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
		pr_debug("%s: port_ptr=%p\n",__func__, port_ptr);
		port_r = request_region(port, port_size, KBUILD_MODNAME);
		/* ignore error */
	}
	if (irq) {
		ret = request_irq(irq, (void *)ldt_isr, IRQF_SHARED, KBUILD_MODNAME, THIS_MODULE);
		iowrite8(UART_MCR_RTS | UART_MCR_OUT2 | UART_MCR_LOOP, port_ptr + UART_MCR);
		uart_detected = (ioread8(port_ptr + UART_MSR) & 0xF0) == (UART_MSR_DCD | UART_MSR_CTS);
		pr_debug("UART_MSR=0x%02X\n", ioread8(port_ptr + UART_MSR));

		if (uart_detected) {
			iowrite8(UART_IER_RDI | UART_IER_RLSI | UART_IER_THRI, port_ptr + UART_IER);
			iowrite8(UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2, port_ptr + UART_MCR);
			iowrite8(UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, port_ptr + UART_FCR);
			pr_debug("%s: loopback=%d\n",__func__, loopback);
			if (loopback)
				iowrite8(ioread8(port_ptr + UART_MCR) | UART_MCR_LOOP, port_ptr + UART_MCR);
		}
		if (!uart_detected && loopback) {
			pr_warn("Emulating loopback in software\n");
			ret = -ENODEV;
		}
	}
	pr_debug("%s: uart_detected=0x%X\n",__func__, uart_detected);
	pr_debug("UART_IER=0x%02X\n", ioread8(port_ptr + UART_IER));
	pr_debug("UART_IIR=0x%02X\n", ioread8(port_ptr + UART_IIR));
	pr_debug("UART_FCR=0x%02X\n", ioread8(port_ptr + UART_FCR));
	pr_debug("UART_LCR=0x%02X\n", ioread8(port_ptr + UART_LCR));
	pr_debug("UART_MCR=0x%02X\n", ioread8(port_ptr + UART_MCR));
	pr_debug("UART_LSR=0x%02X\n", ioread8(port_ptr + UART_LSR));
	pr_debug("UART_MSR=0x%02X\n", ioread8(port_ptr + UART_MSR));
	return ret;
}

static struct task_struct *thread;
static struct dentry *debugfs;

/*
 *	ldt_probe - main initialization function
 */

static __devinit int ldt_probe(struct platform_device *pdev)
{
	int ret;
	char *data = NULL;
	struct resource *r;
	printk(KERN_DEBUG"%s %s %s", KBUILD_MODNAME, __DATE__, __TIME__);
	printk(KERN_DEBUG"pdev = %p ", pdev);
	pr_debug("%s: irq=%d\n",__func__, irq);
	pr_debug("%s: bufsize=%d\n",__func__, bufsize);
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
		dev_dbg(&pdev->dev, "%s attaching device\n", __func__);
		pr_debug("%s: pdev->dev.of_node=%p\n",__func__, pdev->dev.of_node);
#ifdef CONFIG_OF_DEVICE
		of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
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
	/* proc_create(KBUILD_MODNAME, 0, NULL, &ldt_fops); depricated */
	mod_timer(&ldt_timer, jiffies + HZ / 10);
	thread = kthread_run(ldt_thread, NULL, "%s", KBUILD_MODNAME);
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		if (ret)
			goto exit;
	}
	debugfs = debugfs_create_file(KBUILD_MODNAME, S_IRUGO, NULL, NULL, &ldt_fops);
	ret = misc_register(&ldt_miscdev);
	if (ret < 0)
		goto exit;
	pr_debug("%s: ldt_miscdev.minor=%d\n",__func__, ldt_miscdev.minor);
exit:
	pr_debug("%s: ret=%d\n",__func__, ret);
	return ret;
}

/*
 *	ldt_remove - main clean up function
 */

static int __devexit ldt_remove(struct platform_device *pdev)
{
	if (pdev)
		dev_dbg(&pdev->dev, "%s detaching device\n", __func__);
	/* remove_proc_entry(KBUILD_MODNAME, NULL); depricated */
	if (debugfs)
		debugfs_remove(debugfs);
	misc_deregister(&ldt_miscdev);
	if (!IS_ERR_OR_NULL(thread)) {
		send_sig(SIGINT, thread, 1);
		kthread_stop(thread);
	}
	del_timer(&ldt_timer);
	if (port_r)
		release_region(port, port_size);
	if (irq) {
		if (uart_detected) {
			iowrite8(0, port_ptr + UART_IER);
			iowrite8(0, port_ptr + UART_FCR);
			iowrite8(0, port_ptr + UART_MCR);
			ioread8(port_ptr + UART_RX);
		}
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
	pr_debug("%s: isr_counter=%d\n",__func__, isr_counter);
	pr_debug("%s: ldt_work_counter=%d\n",__func__, ldt_work_counter);
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
	ret = platform_driver_register(&ldt_driver);
	return ret;
}

static void ldt_exit(void)
{
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
	res = ldt_remove(NULL);
}

module_init(ldt_init);
module_exit(ldt_exit);
#endif

MODULE_DESCRIPTION("LDT - Linux Driver Template");
MODULE_AUTHOR("Constantine Shulyupin <const@makelinux.net>");
MODULE_LICENSE("GPL");
