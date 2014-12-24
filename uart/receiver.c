#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <stdbool.h>
#include <fcntl.h>
#include <time.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <signal.h>

// debug log
#define LOG_ERR(fmt, args...) do { fprintf(stderr, "[%s:%d]"fmt, __FUNCTION__, __LINE__,##args); } while(0)
#define LOG_DBG(fmt, args...) do { fprintf(stderr, "[%s:%d]"fmt, __FUNCTION__, __LINE__, ##args); } while(0)

struct termios oldtio;
int fd;

void signal_handler(int signo)
{
    fprintf(stderr, "exiting...\n");
	tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
    exit(0);
}

#include "uart.c"

int main(int argc, char **argv)
{
    if(argc<2) {
        fprintf(stderr, "Usage: %s tty-device\n", argv[0]);
        exit(1);
    }

    signal(SIGINT, signal_handler);

    fd = ubus_uart_init(argv[1], 115200);

    if(fd<0) exit(1);

    fprintf(stderr, "Start receiving\n");
    while(1) {
        int i=0;

        ssize_t s = read(fd, &i, sizeof(i));
        if(s<0) {
            fprintf(stderr, "error reading: %s\n", strerror(errno));
            exit(1);
        }  
        fprintf(stderr, "Got 0x%x\n", i);
    }

    close(fd);
    exit(0);
}
