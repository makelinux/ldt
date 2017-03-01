/*

   ctracer.h demonstarion and testing

- Test as user application:

make -B ctracer-test && ./ctracer-test

- Test as kernel module:

KERNELDIR=/lib/modules/$(uname -r)/build; \
make -B -C ${KERNELDIR} M=$PWD modules obj-m=ctracer-test.o; \
sudo insmod ./ctracer-test.ko; \
sudo rmmod ctracer-test

 */


#define CTRACER_ON
#include "ctracer.h"

#ifdef __KERNEL__
#include <linux/module.h>
#endif

int sub(void)
{	_entry:;
	return 0;
}

int main(void)
{	_entry:;
	int i;
	for (i=0; i < 10; i++)
		sub();
	trl();
	trvp(i);
	trvx(i);
	trvd(i);
	trvd(sizeof(long long));
	trvd(sizeof(size_t));
	trvd(sizeof(void*));
	trvd(sizeof(int));
	trvd(sizeof(short));
	trvd(sizeof(char));
	return 0;
}

#ifdef __KERNEL__

module_init(main);

static void mod_exit(void)
{
}
module_exit(mod_exit);

#endif
