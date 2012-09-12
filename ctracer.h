//#define TRACE_TIME
#ifndef __ASSEMBLY__

#ifdef  __cplusplus
extern "C" {
#endif

// This file contains simple and usefull tracing fuctions.
// It intended to be cross platform, easy to include to
// any another source for tracing.
// To use this file just include it. No need more
// changes in compilation or linkage.
//
// by Conan, Constantine Shulyupin

// allow multiple inclussion with differen flag (TRACE_OFF)
//#ifndef _TRACE_H_
#ifndef _TRACE_H_
// First time
#define _TRACE_H_
typedef int status_t;
extern __thread status_t status;

// comands:
// :%s/) *\n{ *$/)\r{\t_entry:;/

#define FNSTART trllog()
#define _entry trllog(); goto _entry; _entry
//#define _entry _trace_enter_exit_();trln(); goto _entry_second;_entry_second
//#define _entry once(trl()); goto _entry; _entry
//#define return trlm("} "); return
//#define FNSTART trl()
#define DEBUG_PRINT tracef
#define FNSTARTP(args...) trllog(args)
//#define _MMREAD32(mac,address) gbus_read_uint32 (mac,(RMuint32)(address))
//#define _MMREADFIELD(mac,structaddress,field) (_MMREAD32(mac,(RMuint8 *)(&(structaddress)->field)))

#ifdef TRACE_MALLOC
static int malloc_count=0;
static void * malloc_trace=NULL;
#endif
//static inline int empty_statement() {return 0;};
//#define  empty_statement() do {(0);} while (0) // for sdcc
#define empty_statement() do { } while (0)
#define empty_function()  { } while (0)
//#define empty_statement (1+1)

#else
// Second time
  #undef IFDBG
  #undef tracef
#endif

#define trace_ioctl(nr) tracef("ioctl=(%c%c %c #%i %ib)\n", \
	(_IOC_READ & _IOC_DIR(nr))?'r':' ', (_IOC_WRITE & _IOC_DIR(nr))?'w':' ', \
	_IOC_TYPE(nr), _IOC_NR(nr), _IOC_SIZE(nr))

#define trace_ioctl_(nr) tracef("ioctl=(%i %i %i %i)", _IOC_DIR(nr),_IOC_TYPE(nr), _IOC_NR(nr), _IOC_SIZE(nr))

#ifdef DEBUG // default if debug
#define _TRACE
#endif

#ifdef CTRACER_ON // like debug
#define _TRACE
#endif

#ifdef TRACE_ON // like debug
#define _TRACE
#endif

#ifndef TRACE_OFF
//#define _TRACE // trace by default
#endif

#ifdef CTRACER_OFF // force no tracing
#undef _TRACE
#endif

//#define _TRACE // global force
//#undef _TRACE // for cvs
//#error

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
//extern int printk(const char *s, ...);
//#define tracef(fmt, args...) printk(KERN_DEBUG fmt, ##args)
#define SOL KERN_DEBUG
#define tracef(fmt, args...) printk(fmt, ##args)

#else // ! __KERNEL__
// _TRACE and not __KERNEL__
#include <stdio.h>

#ifdef __GNUC__
// this will work with gcc, and not with vcc
// beacause vcc preprocessor hasn't '...'
// Usefull: __unix__, __GNUC__, __GLIBC__, __linux__

#define tracef(args...) fprintf(stderr,##args)

//#include <signal.h>
//#define BP {trl();kill(0,SIGTRAP);}
//#define BP kill(0,SIGTRAP)

#endif // __GNUC__

#ifndef tracef
#define tracef printf
#endif

#endif // __KERNEL__

#define trfn(i) trvi(i)

#ifndef _hweight16_defined
#ifdef __GNUC__
static inline unsigned int _hweight16(unsigned int n)
{
	int w=0;;
	while (n) {
		w+=n & 1;
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
 	if ( _hweight16(num) < 2 ) { 	\
		trl_();	\
		trvx(num);  \
	}	num++; 				\
} while (0)

#define trlnum(n, args ... ) \
do {  \
	static int num; 			\
 	if ( num < n ) { 	\
		trl_();	 			\
		trvi_(num); 		\
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
#define trfn(i) (i)

#define IFDBG(x) do {} while (0)
#define trace_linux_mem() empty_statement()
#ifdef WIN32
#define tracef if (0) printf  // no print
#else
// for gcc
#define tracef(fmt, args...) empty_function()
#endif

#define stack_trace() empty_statement()

#define trfn(i) (i)

#endif // _TARCE

#ifndef SOL
#define SOL ""
#endif
#define EOL "\n" // for console
//#define EOL "\r\n" // for com port

// trace variables, integer, hex, string, pointer
#ifndef trl
#define trvi_(i) tracef(#i" = %i ",(int)i)
#define trvi(i) tracef(#i" = %i"EOL,(int)i)
#define trvx_(x) tracef(#x" = 0x%x ",(int)x)
#define trvx(x) tracef(#x" = 0x%x"EOL,(int)x)
//#define trvx(x) tracef(#x" = %#x"EOL,(int)x)
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

#endif
/*
not copliled in windows
static inline void show_stack(int a)
{
	void * sp = &a;
	trvp(sp);
}
*/

#ifdef WIN32
#define trla tracef
#else
// for gcc
#define trla(fmt, args...) tracef("%s:%i %s "fmt,__FILE__,__LINE__,__FUNCTION__, ## args)
#endif
#ifdef MODULE
static inline char * ctracer_file_name_no_path(char * fn)
{
	char *strrchr(const char *s, int c);

	char * p = strrchr(fn,'/');
	if (p) return p+1;
	return fn;
}
#define __file__	ctracer_file_name_no_path(__FILE__)
//#define __file__	KBUILD_MODNAME
#else
#define __file__	__FILE__
#endif
// trace location
#define trlm(m) tracef(SOL"%s:%i %s %s"EOL,__file__,__LINE__,__FUNCTION__,m)
#define trf(m) tracef("%s",__FUNCTION__)
#define trf_(m) tracef("%s ",__FUNCTION__)
// macro with '_' doesn't prints new line
//#define trlm_(m) tracef("%s:%i %s %s ",__FILE__,__LINE__,__FUNCTION__,m)
#define trlm_(m) tracef(SOL"%s:%i %s %s ",__file__,__LINE__,__FUNCTION__,m)

#ifndef trl
#define trl() do { trace_time(); trlm(""); } while (0)
#define trn() tracef(EOL)
#define trm(m) tracef("%s"EOL,m)
#define trm_(m) tracef("%s ",m)
#define trln() tracef(EOL)
#define trl_() do { trace_time(); trlm_(""); } while (0)
#endif


//#define trl_in() {trlm("{");IFDBG(stack_trace());}
#define trl_in() trace_time();trlm("{");
#define trl_out() trace_time();trlm("}");

#if 0
#define TraceMem(P,N) \
	 IFDBG( { char * i=(signed char*)P; for (; ((int)i-(int)P) < N ; i++) \
{ tracef("%i:%02X ",(int)i-(int)P,*(unsigned char*)i);};tracef(EOL); } )
#else
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
#endif

#define TraceError(m) \
	IFDBG( tracef("%s:%i %s %s"EOL,__FILE__,__LINE__,m,\
	SysErrorMsgGet(GetLastError())) )

//#ifdef DEBUG
// TODO BUG: double call
//#define TraceExpX(V) (TraceVarX(V),(V))
//#define TraceRetStatus(R) { printf("%s:%i %s <= %s\n",__FILE__,__LINE__ ,SysErrorMsgGet(R),#R); }
//#else
//#define TraceExpX(V) (V)
//#define TraceRetStatus(R) {R;}
//#endif

//##if DEBUG
//#define LOG_ERR_I(f,i) print

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
//double time_prev_f = 0;
//static double time_prev_f __attribute__ ((weak));
//extern double time_prev_f __attribute__ ((weak));


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

#ifdef free
#undef free
#undef char
#undef int
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
	size = backtrace (array, sizeof(array)/sizeof(array[0]));
	strings = backtrace_symbols (array, size);
	tracef("Stack:\n");

	for (i = 0; i < size; i++)
	{
		if ( ! array[i] ) break;
		tracef("%i %p %s\n", i, array[i],strings[i]);
	}
	free (strings);
}
#endif
#endif // __GLIBC__

//#ifndef ERR
//#define ERR(x) ((x)<0)
//#define OK(x) ((x)>=0)
//#endif

#define chck(F) do { int s=F; if (ERR(s)) { trl_();tracef(SOL"FAIL: %i="#F"\n",s); } } while (0)
//#endif //  _TRACE_H_
//##undef DEBUG

//#ifdef define_freeram

//int extern freeram();

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
//#include <linux/printk.h>

#define freeram() { struct sysinfo i; static unsigned int last; si_meminfo(&i); trl_(); \
	int d = last-i.freeram; int used = i.totalram-i.freeram; \
	trvi_(i.freeram);trvi_(used);  trvi(d); \
	last = i.freeram; }
//#endif

#ifdef  __cplusplus
}
#endif

#define chkz(a) \
(p = a,\
	((!p)?tracef("%s %i %s FAIL %i = %s\n",__FILE__,__LINE__,__FUNCTION__,p,#a):0),\
 	p)

#define chkn(a) \
(status = a,\
	((status<0)?tracef("%s:%i %s FAIL\n\t%i=%s\n",__FILE__,__LINE__,__FUNCTION__,status,#a)\
	 :0),\
 	status)

#define chkne(a) \
(  /* tracef("calling  %s\n",#a), */ \
 status = a,\
	((status<0)?tracef("%s:%i %s FAIL errno = %i \"%s\" %i = %s\n",__FILE__,__LINE__,__FUNCTION__,errno,strerror(errno),status,#a)\
	 :0),\
 	status)

#define chkn2(a) \
(status = a,\
	((status<0)?tracef("%s %i %s FAIL %i = %s\n",__FILE__,__LINE__,__FUNCTION__,status,#a)\
	 :tracef("%s %i %s %i = %s\n",__FILE__,__LINE__,__FUNCTION__,status,#a)),\
 	status)
#define once(exp) do{static int _passed;if (!_passed){exp;};_passed=1;}while(0)

static inline void _func_exit(void * l)
{
	char **loc = (char**)l;
	if (loc) tracef("%s }\n",*loc);
}

#define _trace_enter_exit_() char const *_location __attribute__ (( cleanup (_func_exit) )) = __func__; \
	tracef("%s {",_location);

#endif // __ASSEMBLY__
