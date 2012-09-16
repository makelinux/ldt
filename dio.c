/*
 *	DIO - Device Input/Output utility
 *
 *	stdin/stdout <--> dio <--> mmap, ioctl, read/write
 *
 *	Copyright (C) 2012 Constantine Shulyupin <const@makelinux.net> 
 *	http://www.makelinux.net/
 *
 *	Dual BSD/GPL License
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <string.h>
#include <sys/param.h>
#include <sys/poll.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/user.h>
#include <time.h>
#include <fcntl.h>
#include <assert.h>

static enum io_type {
	file_io,
	mmap_io,
	ioctl_io
} io_type;

static void *inbuf, * outbuf;
static void *mm;
int buf_size = 8 * PAGE_SIZE;
static int mmapoffset;
static int mmapsize;
static char *dev_name;
static int ignore_eof;

int output(int dev, void *buf, int len)
{
	if ( dev < 0 )
		return 0;
	switch (io_type) {
	case mmap_io:
		break;
	case file_io:
	default:
		len = write(dev, buf, len);
	}
	return len;
}

int input(int dev, void *buf, int len)
{
	if ( dev < 0 )
		return 0;
	switch (io_type) {
	case mmap_io:
		break;
	case file_io:
	default:
		len = read(dev, buf, len);
	}
	return len;
}

int pipe_start(int dev)
{
	struct pollfd pfd[2] = {{0,},};
	ssize_t res, data_in_len, len_total = 0;
	int i = 0;
	memset(pfd,100,sizeof(pfd));
	trl();
	pfd[0].fd = fileno(stdin);
	pfd[0].events = POLLIN;
	pfd[1].fd = dev;
	pfd[1].events = POLLIN;
	trvd(pfd[1].fd);
	while (poll(pfd, sizeof(pfd) / sizeof(pfd[0]), -1) > 0) {
				usleep(100000);
#if VERBOSE
		trvd_(i);
		trvx_(pfd[0].revents);
		trvx_(pfd[1].revents);
		trln();
#endif
		i++;
		data_in_len = 0;
		if (pfd[0].revents & POLLIN) {
			pfd[0].revents  = 0;
			res = data_in_len = read(fileno(stdin), inbuf, buf_size);
			if (data_in_len < 0) {
				usleep(100000);
				break;
			}
			if (!data_in_len) {
				// read returns 0 on End Of File
				break;
			}
#if VERBOSE
			trvd_(data_in_len);
			trln();
#endif
again:
			chkne(res = output(dev, inbuf, data_in_len));
			if (res < 0 && errno == EAGAIN) {
				usleep(100000);
				goto again;
			}
		}
		if (pfd[1].revents & POLLIN) {
			pfd[1].revents = 0;
			chkne(res = data_in_len = input(dev, outbuf, buf_size));
			if (data_in_len < 0) {
				usleep(100000);
				break;
			}
			if (!data_in_len ) {
				// EOF, dont extect data from the file any more
				// but wee can continue to write
				pfd[1].events = 0;
			}
			if (!data_in_len && !ignore_eof) {
				// read returns 0 on End Of File
				break;
			}
#if VERBOSE
			trl_();
			trvd_(data_in_len);
			trln();
#endif
			write(fileno(stdout), outbuf, data_in_len);
		}
		if (data_in_len > 0)
			len_total += data_in_len;
#if VERBOSE2
		trl_();
		trvd_(i);
		trvd_(len_total);
		trvd_(data_in_len);
		trln();
#endif
		if (pfd[0].revents & POLLHUP || pfd[1].revents & POLLHUP)
			break;
		//usleep(1000 * delay);
	}
	return res;
}

#define add_literal_option(o)  do { options[optnum].name = #o; \
       options[optnum].flag = (void*)&o; options[optnum].has_arg = 1; \
       options[optnum].val = -1; optnum++; } while (0)

#define add_flag_option(n,p,v) do { options[optnum].name = n; \
       options[optnum].flag = (void*)p; options[optnum].has_arg = 0; \
       options[optnum].val = v; optnum++; } while (0)

static struct option options[100];
int optnum;
__thread status_t status;

int options_init()
{
	optnum = 0;
	/* on gcc 64, pointer to variable can be used only on run-time
	 */
	memset(options, 0, sizeof(options));
	add_literal_option(io_type);
	add_literal_option(buf_size);
//	add_literal_option(ignore_eof);
	add_flag_option("ioctl", &io_type, ioctl_io);
	add_flag_option("mmap", &io_type, mmap_io);
	add_flag_option("file", &io_type, file_io);
	add_flag_option("ignore_eof", &ignore_eof, 1);
	options[optnum].name = strdup("help");
	options[optnum].has_arg = 0;
	options[optnum].val = 'h';
	optnum++;
	return optnum;
}

/* expand_arg, return_if_arg_is_equal - utility functions to translate command line parameters
 * from string to numeric values using predefined preprocessor defines
 */

#define return_if_arg_is_equal(entry) if (0 == strcmp(arg,#entry)) return entry

int expand_arg(char *arg)
{
	if (!arg)
		return 0;
	//return_if_arg_is_equal(SOCK_STREAM);

	return atoi(arg);
}

char *usage = "Usage:\n\
       devio <options> <device file>\n\
\n\
options:\n\
\n\
default values are marked with '*'\n\
\n\
       -h | --help\n\
               show this help\n\
\n\
       --buf_size <n> \n\
               I/O buffer size\n\
\n\
Samples:\n\
\n\
TBD\n\
\n\
";

int init(int argc, char *argv[])
{
	int opt = 0;
	int longindex = 0;
	options_init();
	opterr = 0;
	while ((opt = getopt_long(argc, argv, "h", options, &longindex)) != -1) {
		switch (opt) {
		case 0:
			if (options[longindex].val == -1)
				*options[longindex].flag = expand_arg(optarg);
			break;
		case 'h':
			printf("%s", usage);
			exit(0);
			break;
		default:	/* '?' */
			printf("Error in arguments\n");
			exit( EXIT_FAILURE);
		}
	}
	trvd(optind);
	if ( optind < argc ) {
		dev_name = argv[optind];
	}
	trvs(dev_name);
	trvd_(io_type);
	trvd_(buf_size);
	trvd_(ignore_eof);
	trln();
	return 0;
}

int main(int argc, char *argv[])
{
	int dev;
#ifdef TRACE_ON
	fprintf(stderr, "%s compiled " __DATE__ " " __TIME__ "\n", argv[0]);
#endif
	init(argc, argv);
	inbuf = malloc(buf_size);
	outbuf = malloc(buf_size);
	chkne(dev = open(dev_name, O_CREAT | O_RDWR,0666));
	trvd(dev);
	if (io_type == mmap_io) {
		mm = mmap(0, mmapsize, PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, dev, mmapoffset);
		if (mm == MAP_FAILED) {
			fprintf(stderr, "mmap() failed\n");
			goto exit;
		}
	}
	pipe_start(dev);
exit:
	if (mm && mm != MAP_FAILED)
		munmap(mm, mmapsize);
	free(outbuf);
	free(inbuf);
	close(dev);
	exit(EXIT_SUCCESS);
}
