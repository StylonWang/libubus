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

    fprintf(stderr, "Halt....\n");
    sleep(1);
    while (1) {
        static unsigned char c[4];
        static int i=0;

        c[0] = i++; c[1] = i++; c[2] = i++; c[3] = i++;
        fprintf(stderr, "Sending %d %d %d %d\n", c[0], c[1], c[2], c[3]);
        ssize_t s = write(fd, c, sizeof(c));
        if (s < 0) {
            fprintf(stderr, "error writing: %s\n", strerror(errno));
            exit(1);
        }

        usleep(1200 * 1000);
    }

    close(fd);
    exit(0);
}
