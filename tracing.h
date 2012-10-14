
#define multistatement(ms)	ms	/* trick to bypass checkpatch.pl error */
#define _entry multistatement(goto _entry; _entry)

#define ctracer_cut_path(fn) (fn[0] != '/' ? fn : (strrchr(fn, '/') + 1))
#define __file__	ctracer_cut_path(__FILE__)

#define print_context()	\
	pr_debug("%s:%d %s %s 0x%x\n", __file__, __LINE__, __func__, \
			(in_irq() ? "harirq" : current->comm), preempt_count());

#define check(a) \
(ret = a, ((ret < 0) ? printk("%s:%i %s FAIL\n\t%i=%s\n", __file__, __LINE__, __func__, ret, #a) : 0), ret)

#define trace_loc()	printk(KERN_DEBUG"%s:%d %s ", __file__, __LINE__, __func__)
#define trace_hex(h)	printk("%s = %ld ",#h,(long int)h)
#define trace_dec(d)	printk("%s = %ld ",#d,(long int)d)
#define trace_dec_ln(d)	printk("%s = %ld\n",#d,(long int)d)
#define trace_msg(m)	printk(KERN_DEBUG"%s:%d %s %s\n", __file__, __LINE__, __func__,m)
#define once(exp) do{ \
	static int _passed; if (!_passed) {exp; }; _passed = 1;} while (0)

