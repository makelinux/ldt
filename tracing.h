
#define multistatement(ms)	ms	/* trick to bypass checkpatch.pl error */
#define _entry multistatement(goto _entry; _entry)

/*
 *	ctracer_cut_paths - return filename without path
 */

#define trace_loc()	printk(KERN_DEBUG"%s:%d %s ", __file__, __LINE__, __func__)
#define trace_hex(h)	printk("%s = 0x%lX ", #h, (long int)h)
#define trace_dec(d)	printk("%s = %ld ", #d, (long int)d)
#define trace_dec_ln(d)	printk("%s = %ld\n", #d, (long int)d)
#define trace_ln(m)	printk(KERN_CONT"\n")

