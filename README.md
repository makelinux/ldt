# LDT - Linux Driver Template

It is useful for Linux driver development beginners and as starting point for a new drivers. 
The driver uses following Linux facilities: 
module, file_operations (read/write, mmap, ioctl, blocking and nonblocking mode), kfifo, completion, interrupt, tasklet, work, kthread, timer, misc device, proc fs.

## Files:

Main source file of LDT: 
**[ldt.c](https://github.com/makelinux/ldt/blob/master/ldt.c)**

Test script, run it: **[ldt-test](https://github.com/makelinux/ldt/blob/master/ldt-test)**

Browse the rest code: https://github.com/makelinux/ldt/

## Usage:

Just run

git clone git://github.com/makelinux/ldt.git && cd ldt && ./ldt-test

and explore sources

## Compiled and tested on Linux versions:

v3.6-rc5 

3.2.0-30-generic-pae (Ubuntu 12.04 LTS)

2.6.38-11-generic (Ubuntu 11.04)

v2.6.37-rc8

v2.6.36-rc8

### Failed compilation with:

v2.6.35-rc6 failed because of DEFINE_FIFO
