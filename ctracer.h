/*
	Tracing utility for C

	implemented in include-file only

	Copyright (C) 2012 Constantine Shulyupin  http://www.makelinux.net/

	Dual BSD/GPL License
*/

/* Configuration flags: */

//#define TRACE_TIME
//#define TRACE_MALLOC

/*
	VI command to include label _entry to each function start for tracing
	:%s/) *\n{ *$/)\r{\t_entry:;/
 */

#ifndef __ASSEMBLY__

//__BEGIN_DECLS
#ifndef CTRACER_H_INCLUDED
#define CTRACER_H_INCLUDED
extern __thread int ret;

#define _entry trllog(); goto _entry; _entry
//#define _entry _trace_enter_exit_();trln(); goto _entry_second;_entry_second
//#define _entry once(trl()); goto _entry; _entry
//#define return trlm("} "); return

#ifdef TRACE_MALLOC
static int malloc_count = 0;
static void *malloc_trace = NULL;
#endif
#define empty_statement() do { } while (0)
#define empty_function()  { } while (0)

#define trace_ioctl(nr) tracef("ioctl=(%c%c %c #%i %i)\n", \
	(_IOC_READ & _IOC_DIR(nr))?'r':' ', (_IOC_WRITE & _IOC_DIR(nr))?'w':' ', \
	_IOC_TYPE(nr), _IOC_NR(nr), _IOC_SIZE(nr))

#define trace_ioctl_(nr) tracef("ioctl=(%i %i %i %i)", _IOC_DIR(nr),_IOC_TYPE(nr), _IOC_NR(nr), _IOC_SIZE(nr))

#ifdef DEBUG
#define _TRACE
#endif

#ifdef CTRACER_ON
#define _TRACE
#endif

#ifdef CTRACER_OFF		// force no tracing
#undef _TRACE
#endif

#ifdef _TRACE
#define IFDBG(x) x

#ifdef __KERNEL__
#undef TRACE_TIME

//static inline void stack_trace(void) { }

//#define TRACE_LINUX_MEMORY_ON
#ifdef TRACE_LINUX_MEMORY_ON
#include <linux/mmzone.h>

extern int free_pages_prev;
#define trace_linux_mem() \
do { \
extern zone_t *zone_table[MAX_NR_ZONES*MAX_NR_NODES]; \
int mem_change = zone_table[0]->free_pages - free_pages_prev; \
if ( mem_change ) { trl_(); trvi_(mem_change ); trvi(zone_table[0]->free_pages); } \
free_pages_prev = zone_table[0]->free_pages; \
} while (0);
#endif

//#define tracef(fmt, args...) { if (get_current()->trace) printk( fmt, ## args); }
#define SOL KERN_DEBUG
#define tracef(fmt, args...) printk(fmt, ##args)

#else // ! __KERNEL__
// _TRACE and not __KERNEL__
#include <stdio.h>

#ifdef __GNUC__
#define tracef(args...) fprintf(stderr,##args)

//#include <signal.h>
//#define BP {trl();kill(0,SIGTRAP);}
//#define BP kill(0,SIGTRAP)

#endif // __GNUC__

#ifndef tracef
#define tracef printf
#endif

#endif // __KERNEL__

#ifndef _hweight16_defined
#ifdef __GNUC__
static inline unsigned int _hweight16(unsigned int n)
{
	int w = 0;;
	while (n) {
		w += n & 1;
		n = n >> 1;
	}
	return w;
}
#endif
#define _hweight16_defined
#endif
#define trllog(args ... ) \
do {  \
	static int num; 			\
 	if ( _hweight16(num) < 2 ) { 		\
		trla("#0x%x\n",(int)num); 	\
	}	num++; 				\
} while (0)

#define trlnum(n, args ... ) \
do {  \
	static int num; 			\
 	if ( num < n ) { 	\
		trl_();	 			\
		tracef("#0x%x",(int)num); 		\
		args; 				\
		trln(); 			\
	}	num++; 				\
} while (0)

#define trleach(n, args ... ) \
do {  \
	static int num; 			\
 	if ( ! ( num % n ) ) { 	\
		trl_();	 			\
		trvi_(num); 		\
		args; 				\
		trln(); 			\
	}	num++; 				\
} while (0)

#else // ! _TRACE
#define trllog(args ... )

#define IFDBG(x) do {} while (0)
#define trace_linux_mem() empty_statement()
#define tracef(fmt, args...) empty_function()
#define stack_trace() empty_statement()

#endif // _TARCE

#ifndef SOL
#define SOL ""
#endif
#define EOL "\n"		// for console
//#define EOL "\r\n" // for com port

// trace variables: integer, hex, string, pointer, float, time value, with and w/o new line
#define trvi_(i) tracef(#i" = %i ",(int)i)
#define trvi(i) tracef(#i" = %i"EOL,(int)i)
#define trvx_(x) tracef(#x" = 0x%x ",(int)x)
#define trvx(x) tracef(#x" = 0x%x"EOL,(int)x)
#define trvlx(x) tracef(#x" = %#llx"EOL,(int)x)
#define trvX(x) tracef(#x" = %#X"EOL,(int)x)
#define trvf(f) tracef(#f" = %f"EOL,f)
#define trvf_(f) tracef(#f" = %f ",f)
#define trvd(d) tracef(#d" = %d"EOL,(int)d)
#define trvtv_(tv) tracef(#tv" = %u.%06u ",(unsigned int)tv.tv_sec,(unsigned int)tv.tv_usec)
#define trvtv(tv) tracef(#tv" = %u.%06u"EOL,(unsigned int)tv.tv_sec,(unsigned int)tv.tv_usec)
#define trvd_(d) tracef(#d" = %d ",(int)d)
#define trvs(s) tracef(#s" = \"%s\""EOL,s)
#define trvs_(s) tracef(#s" = \"%s\" ",s)
#define trvp(p) tracef(#p" = %08x"EOL,(unsigned)p)
#define trvp_(p) tracef(#p" = %08x ",(unsigned)p)
#define trvc(c) tracef(#c" = %c"EOL,c)
#define trv(t,v) tracef(#v" = %"#t##EOL,v)
#define trvdn(d,n) {int i; tracef("%s",#d"[]=");for (i=0;i<n;i++)tracef("%d:%d,",i,(*((int*)d+i))); tracef(EOL);}
#define trvxn(d,n) {int i; tracef("%s",#d"[]=");for (i=0;i<n;i++)tracef("%04x,",(*((int*)d+i))); tracef(EOL);}
#define trvdr(record) trvdn(&record,sizeof(record)/sizeof(int));
#define trvxr(record) trvxn(&record,sizeof(record)/sizeof(int));
#define trvdnz(d) { if(d) tracef(#d" = %d"EOL,(int)d); }
#define trlvpx(p,x) tracef(SOL"%s:%i %s "#p" = 0x%p "#x" = %x"EOL,__FILE__,__LINE__,__FUNCTION__,p,(int)x)
#define trla(fmt, args...) tracef("%s:%i %s "fmt,__FILE__,__LINE__,__FUNCTION__, ## args)

#ifdef MODULE
static inline char *ctracer_file_name_no_path(char *fn)
{
	char *strrchr(const char *s, int c);

	char *p = strrchr(fn, '/');
	if (p)
		return p + 1;
	return fn;
}

#define __file__	ctracer_file_name_no_path(__FILE__)
#else
#define __file__	__FILE__
#endif
// trace location
#define trlm(m) tracef(SOL"%s:%i %s %s"EOL,__file__,__LINE__,__FUNCTION__,m)
#define trf(m) tracef("%s",__FUNCTION__)
#define trf_(m) tracef("%s ",__FUNCTION__)

// macro with '_' doesn't prints new line

#define trlm_(m) tracef(SOL"%s:%i %s %s ",__file__,__LINE__,__FUNCTION__,m)
#define trl() do { trace_time(); trlm(""); } while (0)
#define trl_() tracef(SOL"%s:%i %s ",__file__,__LINE__,__FUNCTION__)
#define trn() tracef(EOL)
#define trm(m) tracef("%s"EOL,m)
#define trm_(m) tracef("%s ",m)
#define trln() tracef(EOL)

#define trl_in() trace_time();trlm("{");
#define trl_out() trace_time();trlm("}");

#define trace_mem(P,N) \
	 IFDBG( { int i=0; tracef("%s=",#P); for (; i < (int)(N) ; i++) \
{ if (i && ( ! ( i % 16 ))) tracef("%i:",i); \
tracef("%02x ",0xFF & *((char*)((void*)(P))+i)); \
if (! ( (i+1) % 4 )) tracef(" "); \
if (! ( (i+1) % 16 )) tracef(EOL); \
};tracef(EOL); } )

#define trace_mem_int_list(P,N) \
IFDBG( { int i=0; for (; i < (int)(N) ; i+= sizeof(int)) \
{ tracef("%i, ",*(int*)((void*)(P)+i)); \
};} )

#define trace_mem_int(P,N) \
IFDBG( { int i=0; for (; i < (int)(N) ; i+= sizeof(int)) \
{ if ( i && ( ! ( i % 16 ) ) ) tracef("%i:",i); \
tracef("%x ",*(int*)((void*)(P)+i)); \
if (! ( (i+1) % 64 )) tracef(EOL); \
};tracef(EOL); } )

#ifdef TRACE_MALLOC

#define malloc(s) \
	(trla("malloc #%i %p %i\n",++malloc_count,malloc_trace=malloc(s),s),\
	malloc_trace)

#define free(p) { free(p); trla("free   #%i %p\n",malloc_count--,(void*)p);}

#define strdup(s) \
	(trla("strdup #%i %p\n",++malloc_count,malloc_trace=(void*)strdup(s)),\
	(char*)malloc_trace)

#endif

#ifdef TRACE_TIME

#include <time.h>
#include <sys/time.h>

#ifndef trace_time_defined
#define trace_time_defined

void trace_time();
/*
extern double time_prev_f;
void static inline trace_time()
{
	time_t time_cur;
	double time_cur_f;
	time(&time_cur);
	struct timeval tv;
	struct timezone tz;
	struct tm* time_tm;
	gettimeofday(&tv, &tz);
	time_tm = localtime(&time_cur);
	time_cur = tv.tv_sec;
	time_cur_f = 0.000001 * tv.tv_usec + time_cur;
	double passed = time_cur_f - time_prev_f;
    if ( passed > 0.001 )
	{
		//static char time_str[30];
		// like ctime, but with other format
		tracef("time=%04d-%02d-%02d %02d:%02d:%02d %02d +%1.4f s\n",
				time_tm->tm_year+1900, time_tm->tm_mon+1, time_tm->tm_mday,
				time_tm->tm_hour,time_tm->tm_min,time_tm->tm_sec, (int)tv.tv_usec,
				passed);
		time_prev_f = time_cur_f;
	}
	//trvf(time_prev_f);
	//trvf(time_cur_f);
}
*/
#endif

#else
#define trace_time() empty_statement()
#endif

#ifdef __GLIBC__XX
#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef stack_trace
#undef stack_trace
#endif
#ifndef stack_trace_difined
#define stack_trace_difined
// only once
static inline void stack_trace(void)
{
	void *array[5];
	size_t size;
	char **strings;
	size_t i;
	size = backtrace(array, sizeof(array) / sizeof(array[0]));
	strings = backtrace_symbols(array, size);
	tracef("Stack:\n");

	for (i = 0; i < size; i++) {
		if (!array[i])
			break;
		tracef("%i %p %s\n", i, array[i], strings[i]);
	}
	free(strings);
}
#endif
#endif // __GLIBC__

//#include <linux/kernel.h>
//#include <linux/mm.h>
//#include <asm/bitops.h>

//#undef current

/* int static inline freeram()
{
	struct sysinfo i;
	static unsigned int last;
	si_meminfo(&i);
	trl_();
	//trvi_(last);
	//trvi(nr_free_pages() << PAGE_SHIFT );
	//if (i.freeram - last) {
	trvi_((i.freeram-last));
	trvi(i.freeram); // pages
	//}
	last = i.freeram;
	return i.freeram << 2 ; // K
} */
#include <linux/kernel.h>

#define freeram() { struct sysinfo i; static unsigned int last; si_meminfo(&i); trl_(); \
	int d = last-i.freeram; int used = i.totalram-i.freeram; \
	trvi_(i.freeram);trvi_(used);  trvi(d); \
	last = i.freeram; }

#define chkz(a) \
(p = a,\
	((!p)?tracef("%s %i %s FAIL %i = %s\n",__FILE__,__LINE__,__FUNCTION__,p,#a):0),\
 	p)

#define chkn(a) \
(ret = a,\
	((ret<0)?tracef("%s:%i %s FAIL\n\t%i=%s\n",__FILE__,__LINE__,__FUNCTION__,ret,#a)\
	 :0),\
 	ret)

#define chkne(a) \
(  /* tracef("calling  %s\n",#a), */ \
 ret = a,\
	((ret<0)?tracef("%s:%i %s FAIL errno = %i \"%s\" %i = %s\n",__FILE__,__LINE__,__FUNCTION__,errno,strerror(errno),ret,#a)\
	 :0),\
 	ret)

#define chkn2(a) \
(ret = a,\
	((ret<0)?tracef("%s %i %s FAIL %i = %s\n",__FILE__,__LINE__,__FUNCTION__,ret,#a)\
	 :tracef("%s %i %s %i = %s\n",__FILE__,__LINE__,__FUNCTION__,ret,#a)),\
 	ret)
#define once(exp) do{static int _passed;if (!_passed){exp;};_passed=1;}while(0)

static inline void _func_exit(void *l)
{
	char **loc = (char **)l;
	if (loc)
		tracef("%s }\n", *loc);
}

#define _trace_enter_exit_() char const *_location __attribute__ (( cleanup (_func_exit) )) = __func__; \
	tracef("%s { ",_location);

//__END_DECLS
#endif // CTRACER_H_INCLUDED
#endif // __ASSEMBLY__
