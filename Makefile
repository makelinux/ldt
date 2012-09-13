ifneq ($(KERNELRELEASE),)
ccflags-y+=-Wfatal-errors
ccflags-y+=-include $M/ctracer.h
ccflags-y+=-D CTRACER_ON
obj-m:= ldt.o
else

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

endif

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions


depend .depend dep:
	$(CC) $(ccflags-y) -M *.c > .depend


ifeq (.depend,$(wildcard .depend))
include .depend
endif
