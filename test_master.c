#include <stdio.h>
#include <stdlib.h>

#include "ubus.h"

int main(int argc, char **argv)
{
    int ret;
    char *tty;
    ubus bus;

    if(argc<2) {
        fprintf(stderr, "usage: %s /dev/ttyUSB0\n", argv[0]);
        exit(1);
    }
    tty = argv[1];

    ret = ubus_bus_init(&bus, tty, 115200);
    if(ret) exit(1);

    ubus_bus_exit(bus);
    exit(0);
}

