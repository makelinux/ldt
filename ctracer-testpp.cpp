/*

   ctracer.h demonstarion and testing

- Test as user application:

make -B ctracer-testpp && ./ctracer-testpp

 */


#include "ctracer.h"
#include <sys/sysinfo.h>

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
	if (0)
		tracef("Should not be printed\n");
	else
		tracef("Should be printed\n");
	if (1)
		tracef("Should be printed\n");
	else
		tracef("Should not be printed\n");

	freeram();
	return 0;
}
