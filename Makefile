#ccflags-y+=-Wfatal-errors
ccflags-y+=-include $M/ctracer.h
ccflags-y+=-D DEBUG
ccflags-y+=-D CTRACER_ON
ccflags-y+=-D USE_PLATFORM_DEVICE
#ccflags-y+=-D USE_MISCDEV # uncomment to use single misc device instead char devices region

obj-m+= ldt.o
obj-m+= ldt_plat_drv.o # implements platform_driver only
obj-m+= ldt_plat_dev.o # implements platform_device and resource
#obj-m+= chrdev_region_sample.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build

all:	modules dio

modules:
	$(MAKE) -C $(KERNELDIR) M=$$PWD modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$$PWD modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers dio *.tmp *.log

dio: CPPFLAGS+= -D CTRACER_ON -include ctracer.h -g

_src=dio.c  ldt.c  ldt_plat_dev.c  ldt_plat_drv.c ctracer.h

checkpatch:
	/usr/src/linux-headers-$(shell uname -r)/scripts/checkpatch.pl --no-tree --show-types --ignore LONG_LINE,LINE_CONTINUATIONS --terse -f $(_src) Makefile
