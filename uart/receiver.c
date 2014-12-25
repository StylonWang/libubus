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
    if (argc < 2) {
        fprintf(stderr, "Usage: %s tty-device\n", argv[0]);
        exit(1);
    }

    signal(SIGINT, signal_handler);

    fd = ubus_uart_init(argv[1], 115200);

    if (fd < 0)
        exit(1);

    fprintf(stderr, "Start receiving\n");
    while (1) {
        unsigned char c[4];
#if 0
        int ret;
        fd_set readfs;
        struct timeval timeout;
        
		timeout.tv_usec = 0L;
		timeout.tv_sec = 0; //50L;
		FD_ZERO(&readfs);
		FD_SET(fd, &readfs);
		ret = select(fd + 1, &readfs, NULL, NULL, &timeout);
        if(ret<0 && errno!=EINTR) {
            LOG_ERR("select failed: %s\n", strerror(errno));
            sleep(2);
            continue;
		}
		else if (ret == 0) {
            fprintf(stderr, "waiting...\n");
            continue; // timeout with no data
		}
#endif
        // read from RS232
        ssize_t s = read(fd, &c, sizeof(c));
        if (s < 0) {
            fprintf(stderr, "error reading: %s\n", strerror(errno));
            exit(1);
        }
        else if(s==0) {
        }
        else {
            int i=0;
            fprintf(stderr, "Got(%d) ", (int)s);
            for(i=0; i<s; ++i) {
                fprintf(stderr, "%d ", c[i]);
            }
            fprintf(stderr, "\n");
        }
    }

    close(fd);
    exit(0);
}
