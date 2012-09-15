#ccflags-y+=-Wfatal-errors
ccflags-y+=-include $M/ctracer.h
ccflags-y+=-D CTRACER_ON
ccflags-y+=-D USE_PLATFORM_DEVICE
obj-m+= ldt.o
obj-m+= ldt_plat_drv.o # implements platform_driver only
obj-m+= ldt_plat_dev.o # implements platform_device and resource

KERNELDIR ?= /lib/modules/$(shell uname -r)/build

all:	mapper modules

modules:
	$(MAKE) -C $(KERNELDIR) M=$$PWD modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$$PWD modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

depend .depend dep:
	$(CC) $(ccflags-y) -M *.c > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif
