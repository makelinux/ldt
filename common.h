#ifndef __LDT_COMMON_H__
#define __LDT_COMMON_H__

#include <linux/compiler.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,2,0)

#include <asm/apic.h>

#ifndef ISA_IRQ_VECTOR
#define ISA_IRQ_VECTOR(irq)            (((FIRST_EXTERNAL_VECTOR + 16) & ~15) + irq)
#endif

#endif

/* With Linux 3.8 the __devinit macro and others were removed from <linux/init.h>
   Defining them as empty here to keep the other code portable to older kernel versions. */

#include <linux/init.h>

#ifndef __devinit

    #define __devinit
    #define __devinitdata
    #define __devinitconst
    #define __devexit
    #define __devexitdata
    #define __devexitconst

    #if defined(MODULE) || defined(CONFIG_HOTPLUG)
    #define __devexit_p(x) x
    #else
    #define __devexit_p(x) NULL
    #endif

#endif

#endif
