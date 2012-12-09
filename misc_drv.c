/*
 *	misc-drv
 *
 *	Simple misc device driver sample
 *
 *	Copyright (C) 2012 Constantine Shulyupin http://www.makelinux.net/
 *
 *	Licensed under the GPLv2.
 *
 *	The driver demonstrates usage of following Linux facilities:
 *
 *	Linux kernel module
 *	simple single misc device file (miscdevice, misc_register)
 *	file_operations
 *		read and write
 *		blocking read and write
 *		polling
 *	kfifo
 *	interrupt
 *	tasklet
 *	timer
 *	driven by IRQ 6
 *
 *	Run test script misc-drv-test to test the driver
 *
 */

#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/kfifo.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/serial_reg.h>
#include <linux/cdev.h>
#include <asm/apic.h>

static int port = 0x3f4;
module_param(port, int, 0);
MODULE_PARM_DESC(port, "io port number, default 0x3f4 - floppy");

static int port_size = 2;
module_param(port_size, int, 0);
MODULE_PARM_DESC(port_size, "number of io ports, default 2");

static int irq = 6;
module_param(irq, int, 0);
MODULE_PARM_DESC(irq, "interrupt request number, default 6 - floppy");

#define FIFO_SIZE 128		/* must be power of two */

#define MISC_DRV_TX	0
#define MISC_DRV_RX	0
#define MISC_DRV_TX_FULL	1
#define MISC_DRV_RX_READY	1


/**
 * struct misc_drv_data - the driver data
 * @in_fifo:	input queue for write
 * @out_fifo:	output queue for read
 * @fifo_lock:	lock for queues
 * @readable:	waitqueue for blocking read
 * @writeable:	waitqueue for blocking write
 * @port_ptr:	mapped io port
 *
 * Can be retrieved from platform_device with
 * struct misc_drv_data *drvdata = platform_get_drvdata(pdev);
 */

struct misc_drv_data {
	struct mutex read_lock;
	struct mutex write_lock;
	DECLARE_KFIFO(in_fifo, char, FIFO_SIZE);
	DECLARE_KFIFO(out_fifo, char, FIFO_SIZE);
	spinlock_t fifo_lock;
	wait_queue_head_t readable, writeable;
	struct tasklet_struct misc_drv_tasklet;
	struct timer_list misc_drv_timer;
	//void __iomem *port_ptr;
	void *port_ptr;
	struct resource *port_res;
};

static struct misc_drv_data *drvdata;

static void misc_drv_tasklet_func(unsigned long d)
{
	char data_out, data_in;
	struct misc_drv_data *drvdata = (void*) d;
	int tx_ready, rx_ready;

	tx_ready = !ioread8(drvdata->port_ptr + MISC_DRV_TX_FULL);

	while (tx_ready && kfifo_out_spinlocked(&drvdata->out_fifo,
				&data_out, sizeof(data_out), &drvdata->fifo_lock)) {
		wake_up_interruptible(&drvdata->writeable);
		pr_debug("%s: data_out=%d %c\n", __func__, data_out, data_out >= 32 ? data_out : ' ');
		iowrite8(data_out, drvdata->port_ptr + MISC_DRV_TX);
		/* set flag full and implicitly flag ready */
		iowrite8(1, drvdata->port_ptr + MISC_DRV_TX_FULL);
		/*
		   In regular drivers hardware invokes interrupts.
		   Because this drivers works without real hardware
		   we simulate interrupt invocation with function send_IPI_all.
		   In driver, which works with real hardware this is not required.
		 */
		//apic->send_IPI_all(IRQ0_VECTOR+irq);
	}
	while (ioread8(drvdata->port_ptr + MISC_DRV_RX_READY)) {
		data_in = ioread8(drvdata->port_ptr + MISC_DRV_RX);
		pr_debug("%s: data_in=%d %c\n", __func__, data_in, data_in >= 32 ? data_in : ' ');
		kfifo_in_spinlocked(&drvdata->in_fifo, &data_in,
			sizeof(data_in), &drvdata->fifo_lock);
		wake_up_interruptible(&drvdata->readable);
		/* clear flag ready and implicitly flag full */
		iowrite8(0, drvdata->port_ptr + MISC_DRV_RX_READY);
	}
}

/*
 *	interrupt section
 */

static int isr_counter;

static irqreturn_t misc_drv_isr(int irq, void *d)
{
	struct misc_drv_data *drvdata = (void*) d;
	isr_counter++;
	//tasklet_schedule(&drvdata->misc_drv_tasklet);
	return IRQ_HANDLED;	/* our IRQ */
}

/*
 *	timer section
 */

static void misc_drv_timer_func(unsigned long d)
{
	struct misc_drv_data *drvdata = (void*) d;
	/*
	 *      this timer is used just to fire drvdata->misc_drv_tasklet,
	 *      because there is no interrupts in loopback mode
	 */
	//if (loopback)
	tasklet_schedule(&drvdata->misc_drv_tasklet);
	mod_timer(&drvdata->misc_drv_timer, jiffies + HZ / 100);
}

/*
 *	file_operations section
 */

static int misc_drv_open(struct inode *inode, struct file *file)
{
	pr_debug("%s: from %s\n", __func__, current->comm);
	/* client related data can be allocated here and
	   stored in file->private_data */
	return 0;
}

static int misc_drv_release(struct inode *inode, struct file *file)
{
	pr_debug("%s: from %s\n", __func__,current->comm);
	/* client related data can be retrieved from file->private_data
	   and released here */
	return 0;
}

static ssize_t misc_drv_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	unsigned int copied;

	pr_debug("%s: from %s\n", __func__, current->comm);
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
	trvd(ret);
	trvd(copied);
	return ret ? ret : copied;
}

static ssize_t misc_drv_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	int ret;
	unsigned int copied;
	trl();
	trvp(drvdata);

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
	tasklet_schedule(&drvdata->misc_drv_tasklet);
	return ret ? ret : copied;
}

static unsigned int misc_drv_poll(struct file *file, poll_table *pt)
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

static const struct file_operations misc_drv_fops = {
	.owner	= THIS_MODULE,
	.open	= misc_drv_open,
	.release = misc_drv_release,
	.read	= misc_drv_read,
	.write	= misc_drv_write,
	.poll	= misc_drv_poll,
};

static struct miscdevice misc_drv_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= KBUILD_MODNAME,
	.fops	= &misc_drv_fops,
};

/*
 *	main initialization and cleanup section
 */

static void misc_drv_cleanup(void)
{
	if (misc_drv_dev.this_device)
		misc_deregister(&misc_drv_dev);
	//del_timer(&drvdata->misc_drv_timer);
	if (irq) {
		free_irq(irq, drvdata);
	}
	tasklet_kill(&drvdata->misc_drv_tasklet);

	pr_debug("%s: isr_counter=%d\n", __func__, isr_counter);
	if (drvdata->port_ptr) ioport_unmap(drvdata->port_ptr);
	if (drvdata->port_res)
		release_region(port, port_size);
	kfree(drvdata);
}

struct misc_drv_data * misc_drv_data_init(void)
{
	struct misc_drv_data *drvdata;

	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return NULL;
	trl();
	trvp(drvdata);
	init_waitqueue_head(&drvdata->readable);
	init_waitqueue_head(&drvdata->writeable);
	INIT_KFIFO(drvdata->in_fifo);
	INIT_KFIFO(drvdata->out_fifo);
	mutex_init(&drvdata->read_lock);
	mutex_init(&drvdata->write_lock);
	tasklet_init(&drvdata->misc_drv_tasklet, misc_drv_tasklet_func, (unsigned long)drvdata);
	//setup_timer(&drvdata->misc_drv_timer, misc_drv_timer_func,(unsigned long)drvdata);
	return drvdata;
}


static __devinit int misc_drv_init(void)
{
	int ret = 0;

	pr_debug("MODNAME=%s\n", KBUILD_MODNAME);
	pr_debug("port = %X irq = %d\n", port, irq);

	drvdata = misc_drv_data_init();
	if (!drvdata) {
		pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "misc_drv_data_init failed");
		goto exit;
	}

	drvdata->port_res = request_region(port, port_size, KBUILD_MODNAME);
	if (!drvdata->port_res) {
		pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "request_region failed");
		return -EBUSY;
	}
	//drvdata->port_ptr = a;

	drvdata->port_ptr = ioport_map(port, port_size);
	pr_debug("%s: drvdata->port_ptr=%p\n", __func__, drvdata->port_ptr);
	if (!drvdata->port_ptr) {
		pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "ioport_map failed");
		return -ENODEV;
	}
	trvp(drvdata->port_ptr);

	isr_counter = 0;

	ret = misc_register(&misc_drv_dev);
	if (ret < 0) {
		pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "misc_register failed");
		goto exit;
	}
	pr_debug("%s: misc_drv_dev.minor=%d\n", __func__, misc_drv_dev.minor);
	ret = request_irq(irq, misc_drv_isr, 0, KBUILD_MODNAME, drvdata);
	if (ret < 0) {
		pr_err("%s:%d %s %s\n", __FILE__, __LINE__, __func__, "request_irq failed");
		return ret;
	}

exit:
	pr_debug("%s: ret=%d\n", __func__, ret);
	if (ret < 0)
		misc_drv_cleanup();
	return ret;
}

module_init(misc_drv_init);
module_exit(misc_drv_cleanup);

MODULE_DESCRIPTION("misc-drv Simple misc device driver sample");
MODULE_AUTHOR("Constantine Shulyupin <const@makelinux.net>");
MODULE_LICENSE("GPL");
