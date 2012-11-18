/*
 *	LDT - Linux Driver Template
 *
 *	Copyright (C) 2012 Constantine Shulyupin http://www.makelinux.net/
 *
 *	Licensed under the GPLv2.
 *
 *
 *	The driver demonstrates usage of following Linux facilities:
 *
 *	Linux kernel module
 *	file_operations
 *		read and write (UART)
 *		blocking read and write
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
 *	debugfs
 *	platform_driver and platform_device in another module
 *	simple UART driver on port 0x3f8 with IRQ 4
 *	Power Management (dev_pm_ops)
 *	Device Tree (of_device_id)
 *
 *	Usermode test script and utility are located in tools/testing/ldt/
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
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/serial_reg.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mod_devicetable.h>

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

#define FIFO_SIZE 128		/* must be power of two */

static int bufsize = 8 * PAGE_SIZE;
static int uart_detected;
static void __iomem *port_ptr;

/**
 * struct ldt_data - the driver data
 * @in_buf:	input buffer for mmap interface
 * @out_buf:	outoput buffer for mmap interface
 * @in_fifo:	input queue for write
 * @out_fifo:	output queue for read
 * @fifo_lock:	lock for queues
 * @readable:	waitqueue for blocking read
 * @writeable:	waitqueue for blocking write
 *
 * stored in static global variable for simplicity.
 * Can be also retrieved from platform_device with
 * struct ldt_data *drvdata = platform_get_drvdata(pdev);
 */

struct ldt_data {
	void *in_buf;
	void *out_buf;
	DECLARE_KFIFO(in_fifo, char, FIFO_SIZE);
	DECLARE_KFIFO(out_fifo, char, FIFO_SIZE);
	spinlock_t fifo_lock;
	wait_queue_head_t readable, writeable;
	struct mutex read_lock;
	struct mutex write_lock;
};

static struct ldt_data *drvdata;

/**
 * ldt_received	- puts data to receive queue
 * @data: received data
 */

static void ldt_received(char data)
{
	kfifo_in_spinlocked(&drvdata->in_fifo, &data,
			sizeof(data), &drvdata->fifo_lock);
	wake_up_interruptible(&drvdata->readable);
}

/**
 * ldt_send - sends data to HW port or emulates SW loopback
 * @data: data to send
 */

static void ldt_send(char data)
{
	if (uart_detected)
		iowrite8(data, port_ptr + UART_TX);
	else
		if (loopback)
			ldt_received(data);
}

static inline u8 tx_ready(void)
{
	return ioread8(port_ptr + UART_LSR) & UART_LSR_THRE;
}

static inline u8 rx_ready(void)
{
	return ioread8(port_ptr + UART_LSR) & UART_LSR_DR;
}

/*
 *	work section
 *
 *	empty template function for deferred call in scheduler context
 */

static int ldt_work_counter;


static void ldt_work_func(struct work_struct *work)
{
	ldt_work_counter++;
}

static DECLARE_WORK(ldt_work, ldt_work_func);

/*
 *	tasklet section
 *
 *	template function for deferred call in interrupt context
 */

static DECLARE_COMPLETION(ldt_complete);

static void ldt_tasklet_func(unsigned long d)
{
	char data_out, data_in;

	if (uart_detected) {
		while (tx_ready() && kfifo_out_spinlocked(&drvdata->out_fifo,
					&data_out, sizeof(data_out), &drvdata->fifo_lock)) {
			wake_up_interruptible(&drvdata->writeable);
			pr_debug("%s: data_out=%d %c\n", __func__, data_out, data_out >= 32 ? data_out : 0);
			ldt_send(data_out);
		}
		while (rx_ready()) {
			data_in = ioread8(port_ptr + UART_RX);
			pr_debug("%s: data_in=%d %c\n", __func__, data_in, data_in >= 32 ? data_in : 0);
			ldt_received(data_in);
		}
	} else {
		while (kfifo_out_spinlocked(&drvdata->out_fifo, &data_out, sizeof(data_out),
					&drvdata->fifo_lock)) {
			wake_up_interruptible(&drvdata->writeable);
			pr_debug("%s: data_out=%d\n", __func__, data_out);
			ldt_send(data_out);
		}
	}
	schedule_work(&ldt_work);
	complete(&ldt_complete);
}

static DECLARE_TASKLET(ldt_tasklet, ldt_tasklet_func, 0);

/*
 *	interrupt section
 */

static int isr_counter;

static irqreturn_t ldt_isr(int irq, void *dev_id)
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
 *	timer section
 */

static struct timer_list ldt_timer;

static void ldt_timer_func(unsigned long data)
{
	/*
	 *      this timer is used just to fire ldt_tasklet,
	 *      because there is no interrupts in loopback mode
	 */
	if (loopback)
		tasklet_schedule(&ldt_tasklet);
	mod_timer(&ldt_timer, jiffies + HZ / 100);
}

static DEFINE_TIMER(ldt_timer, ldt_timer_func, 0, 0);

/*
 *	file_operations section
 */

static int ldt_open(struct inode *inode, struct file *file)
{
	pr_debug("%s: from %s\n", __func__, current->comm);
	/* client related data can be allocated here and
	   stored in file->private_data */
	return 0;
}

static int ldt_release(struct inode *inode, struct file *file)
{
	pr_debug("%s: from %s\n", __func__,current->comm);
	/* client related data can be retrived from file->private_data
	   and released here */
	return 0;
}

static ssize_t ldt_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	unsigned int copied;

	pr_debug("%s: from %s\n", __func__,current->comm);
	if (kfifo_is_empty(&drvdata->in_fifo)) {
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			pr_debug("%s: %s\n", __func__, "waiting");
			ret = wait_event_interruptible(drvdata->readable,
					!kfifo_is_empty(&drvdata->in_fifo));
			if (ret == -ERESTARTSYS) {
				pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "interrupted");
				return -EINTR;
			}
		}
	}
	if (mutex_lock_interruptible(&drvdata->read_lock))
		return -EINTR;
	ret = kfifo_to_user(&drvdata->in_fifo, buf, count, &copied);
	mutex_unlock(&drvdata->read_lock);
	return ret ? ret : copied;
}

static ssize_t ldt_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	int ret;
	unsigned int copied;

	pr_debug("%s: from %s\n", __func__,current->comm);
	if (kfifo_is_full(&drvdata->out_fifo)) {
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			ret = wait_event_interruptible(drvdata->writeable,
					!kfifo_is_full(&drvdata->out_fifo));
			if (ret == -ERESTARTSYS) {
				pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "interrupted");
				return -EINTR;
			}
		}
	}
	if (mutex_lock_interruptible(&drvdata->write_lock))
		return -EINTR;
	ret = kfifo_from_user(&drvdata->out_fifo, buf, count, &copied);
	mutex_unlock(&drvdata->write_lock);
	tasklet_schedule(&ldt_tasklet);
	return ret ? ret : copied;
}

static unsigned int ldt_poll(struct file *file, poll_table *pt)
{
	unsigned int mask = 0;
	poll_wait(file, &drvdata->readable, pt);
	poll_wait(file, &drvdata->writeable, pt);

	if (!kfifo_is_empty(&drvdata->in_fifo))
		mask |= POLLIN | POLLRDNORM;
	mask |= POLLOUT | POLLWRNORM;
/*
	if case of output end of file set
	mask |= POLLHUP;
	in case of output error set
	mask |= POLLERR;
*/
	return mask;
}

/*
 *	pages_flag - set or clear a flag for sequence of pages
 *
 *	more generic solution instead SetPageReserved, ClearPageReserved etc
 *
 *	Poposing to move pages_flag to linux/page-flags.h
 */

static void pages_flag(struct page *page, int page_num, int mask, int value)
{
	for (; page_num; page_num--, page++)
		if (value)
			__set_bit(mask, &page->flags);
		else
			__clear_bit(mask, &page->flags);
}

static int ldt_mmap(struct file *filp, struct vm_area_struct *vma)
{
	void *buf = NULL;
	if (vma->vm_flags & VM_WRITE)
		buf = drvdata->in_buf;
	else if (vma->vm_flags & VM_READ)
		buf = drvdata->out_buf;
	if (!buf)
		return -EINVAL;
	if (remap_pfn_range(vma, vma->vm_start, virt_to_phys(buf) >> PAGE_SHIFT,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "remap_pfn_range failed");
		return -EAGAIN;
	}
	return 0;
}

#define trace_ioctl(nr) pr_debug("ioctl=(%c%c %c #%i %i)\n", \
	(_IOC_READ & _IOC_DIR(nr)) ? 'r' : ' ', \
	(_IOC_WRITE & _IOC_DIR(nr)) ? 'w' : ' ', \
	_IOC_TYPE(nr), _IOC_NR(nr), _IOC_SIZE(nr))

static DEFINE_MUTEX(ioctl_lock);

static long ldt_ioctl(struct file *f, unsigned int cmnd, unsigned long arg)
{
	int ret = 0;
	void __user *user = (void __user *)arg;

	if (mutex_lock_interruptible(&ioctl_lock))
		return -EINTR;
	pr_debug("%s:\n", __func__);
	pr_debug("cmnd=0x%X\n", cmnd);
	pr_debug("arg=0x%lX\n", arg);
	trace_ioctl(cmnd);
	switch (_IOC_TYPE(cmnd)) {
	case 'A':
		switch (_IOC_NR(cmnd)) {
		case 0:
			if (_IOC_DIR(cmnd) == _IOC_WRITE) {
				if (copy_from_user(drvdata->in_buf, user,
							_IOC_SIZE(cmnd))) {
					ret = -EFAULT;
					goto exit;
				}
				/* copy data from in_buf to out_buf to emulate loopback for testing */
				memcpy(drvdata->out_buf, drvdata->in_buf, bufsize);
				memset(drvdata->in_buf, 0, bufsize);
			}
			if (_IOC_DIR(cmnd) == _IOC_READ) {
				if (copy_to_user(user, drvdata->out_buf,
							_IOC_SIZE(cmnd))) {
					ret = -EFAULT;
					goto exit;
				}
				memset(drvdata->out_buf, 0, bufsize);
			}
			break;
		}
		break;
	}
exit:
	mutex_unlock(&ioctl_lock);
	return ret;
}

static const struct file_operations ldt_fops = {
	.owner	= THIS_MODULE,
	.open	= ldt_open,
	.release = ldt_release,
	.read	= ldt_read,
	.write	= ldt_write,
	.poll	= ldt_poll,
	.mmap	= ldt_mmap,
	.unlocked_ioctl	= ldt_ioctl,
};

static struct miscdevice ldt_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= KBUILD_MODNAME,
	.fops	= &ldt_fops,
};

/*
 *	kthread section
 */

static int ldt_thread(void *data)
{
	int ret = 0;
	allow_signal(SIGINT);
	while (!kthread_should_stop()) {
		ret = wait_for_completion_interruptible(&ldt_complete);
		if (ret == -ERESTARTSYS) {
			pr_debug("%s: %s\n", __func__, "interrupted");
			return -EINTR;
		}
		/*
		   perform here a useful work in scheduler context
		 */
	}
	return ret;
}

/*
 *	UART initialization section
 */

static struct resource *port_r;

static int uart_probe(void)
{
	int ret = 0;

	if (port) {
		port_r = request_region(port, port_size, KBUILD_MODNAME);
		if (!port_r) {
			pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "request_region failed");
			return -EBUSY;
		}
		port_ptr = ioport_map(port, port_size);
		pr_debug("%s: port_ptr=%p\n", __func__, port_ptr);
		if (!port_ptr) {
			pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "ioport_map failed");
			return -ENODEV;
		}
	}
	if (irq && port_ptr) {
		/*
		 *	Minimal configuration of UART for trivial I/O opertaions
		 *	and ISR just to porform basic tests.
		 *	Some configuration of UART is not touched and reused.
		 *
		 *	This minimal configiration of UART is based on
		 *	full UART driver drivers/tty/serial/8250/8250.c
		 */
		ret = request_irq(irq, ldt_isr,
				IRQF_SHARED, KBUILD_MODNAME, THIS_MODULE);
		if (ret < 0) {
			pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "request_irq failed");
			return ret;
		}
		iowrite8(UART_MCR_RTS | UART_MCR_OUT2 | UART_MCR_LOOP, port_ptr + UART_MCR);
		uart_detected = (ioread8(port_ptr + UART_MSR) & 0xF0) == (UART_MSR_DCD | UART_MSR_CTS);
		pr_debug("UART_MSR=0x%02X\n", ioread8(port_ptr + UART_MSR));
		pr_debug("%s: uart_detected=0x%X\n", __func__, uart_detected);

		if (uart_detected) {
			iowrite8(UART_IER_RDI | UART_IER_RLSI | UART_IER_THRI, port_ptr + UART_IER);
			iowrite8(UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2, port_ptr + UART_MCR);
			iowrite8(UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT,
					port_ptr + UART_FCR);
			pr_debug("%s: loopback=%d\n", __func__, loopback);
			if (loopback)
				iowrite8(ioread8(port_ptr + UART_MCR) | UART_MCR_LOOP,
						port_ptr + UART_MCR);
		}
		if (!uart_detected && loopback)
			pr_warn("Emulating loopback in software\n");
	}
	return ret;
}

/*
 *	main initialization and cleanup section
 */

static struct task_struct *thread;
static struct dentry *debugfs;

static int ldt_cleanup(struct platform_device *pdev)
{
	struct ldt_data *drvdata = platform_get_drvdata(pdev);
	dev_dbg(&pdev->dev, "%s\n", __func__);

	debugfs_remove(debugfs);
	if (ldt_miscdev.this_device)
		misc_deregister(&ldt_miscdev);
	if (!IS_ERR_OR_NULL(thread)) {
		send_sig(SIGINT, thread, 1);
		kthread_stop(thread);
	}
	del_timer(&ldt_timer);
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
	if (drvdata->in_buf) {
		pages_flag(virt_to_page(drvdata->in_buf), PFN_UP(bufsize), PG_reserved, 0);
		free_pages_exact(drvdata->in_buf, bufsize);
	}
	if (drvdata->out_buf) {
		pages_flag(virt_to_page(drvdata->out_buf), PFN_UP(bufsize), PG_reserved, 0);
		free_pages_exact(drvdata->out_buf, bufsize);
	}

	pr_debug("%s: isr_counter=%d\n", __func__, isr_counter);
	pr_debug("%s: ldt_work_counter=%d\n", __func__, ldt_work_counter);
	if (port_ptr)
		ioport_unmap(port_ptr);
	if (port_r)
		release_region(port, port_size);
	kfree(drvdata);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

int ldt_data_init(struct platform_device *pdev)
{
	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	init_waitqueue_head(&drvdata->readable);
	init_waitqueue_head(&drvdata->writeable);
	INIT_KFIFO(drvdata->in_fifo);
	INIT_KFIFO(drvdata->out_fifo);
	platform_set_drvdata(pdev, drvdata);
	mutex_init(&drvdata->read_lock);
	mutex_init(&drvdata->write_lock);
	return 0;
}

static __devinit int ldt_probe(struct platform_device *pdev)
{
	int ret;
	char *data = NULL;
	struct resource *r;
	dev_dbg(&pdev->dev, "%s attaching device\n", __func__);
	pr_debug("MODNAME=%s\n", KBUILD_MODNAME);
	pr_debug("port = %d irq = %d\n", port, irq);

	ret = ldt_data_init(pdev);
	if (ret < 0) {
		pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "ldt_data_init failed");
		goto exit;
	}

	/*
	 *	Allocating buffers and pinning them to RAM
	 *	to be mapped to user space in ldt_mmap
	 */
	drvdata->in_buf = alloc_pages_exact(bufsize, GFP_KERNEL | __GFP_ZERO);
	if (!drvdata->in_buf) {
		ret = -ENOMEM;
		goto exit;
	}
	pages_flag(virt_to_page(drvdata->in_buf), PFN_UP(bufsize), PG_reserved, 1);
	drvdata->out_buf = alloc_pages_exact(bufsize, GFP_KERNEL | __GFP_ZERO);
	if (!drvdata->out_buf) {
		ret = -ENOMEM;
		goto exit;
	}
	pages_flag(virt_to_page(drvdata->out_buf), PFN_UP(bufsize), PG_reserved, 1);
	pr_debug("%s: pdev->dev.of_node=%p\n", __func__, pdev->dev.of_node);
#ifdef CONFIG_OF_DEVICE
	of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
#endif
	data = dev_get_platdata(&pdev->dev);
	pr_debug("%p %s\n", data, data);
	if (!irq)
		irq = platform_get_irq(pdev, 0);
	r = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (r && !port)
		port = r->start;

	if (r && !port_size)
		port_size = resource_size(r);
	isr_counter = 0;
	/*
	 *	This drivers without UART can be sill used
	 *	in emulation mode for testing and demonstation of work
	 */
	ret = uart_probe();
	if (ret < 0) {
		pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "uart_probe failed");
		goto exit;
	}
	mod_timer(&ldt_timer, jiffies + HZ / 10);
	thread = kthread_run(ldt_thread, NULL, "%s", KBUILD_MODNAME);
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		goto exit;
	}
	debugfs = debugfs_create_file(KBUILD_MODNAME, S_IRUGO, NULL, NULL, &ldt_fops);
	if (IS_ERR(debugfs)) {
		ret = PTR_ERR(debugfs);
		pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "debugfs_create_file failed");
		goto exit;
	}
	ret = misc_register(&ldt_miscdev);
	if (ret < 0) {
		pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "misc_register failed");
		goto exit;
	}
	pr_debug("%s: ldt_miscdev.minor=%d\n", __func__, ldt_miscdev.minor);

exit:
	pr_debug("%s: ret=%d\n", __func__, ret);
	if (ret < 0)
		ldt_cleanup(pdev);
	return ret;
}

static int __devexit ldt_remove(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s detaching device\n", __func__);
	ldt_cleanup(pdev);
	return 0;
}

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
	.resume  = ldt_resume,
};

#define ldt_pm_ops (&ldt_pm)
#else
#define ldt_pm_ops NULL
#endif

/*
 *	template for OF FDT ID
 *	(Open Firmware Flat Device Tree)
 */

static const struct of_device_id ldt_of_match[] = {
	{.compatible = "linux-driver-template",},
	{},
};

MODULE_DEVICE_TABLE(of, ldt_of_match);

static struct platform_driver ldt_driver = {
	.driver = {
		   .name	= "ldt_device_name",
		   .owner	= THIS_MODULE,
		   .pm		= ldt_pm_ops,
		   .of_match_table = of_match_ptr(ldt_of_match),
		   },
	.probe	= ldt_probe,
	.remove	= __devexit_p(ldt_remove),
};

module_platform_driver(ldt_driver);

MODULE_DESCRIPTION("LDT - Linux Driver Template");
MODULE_AUTHOR("Constantine Shulyupin <const@makelinux.net>");
MODULE_LICENSE("GPL");
