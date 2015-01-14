#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>

#include "ubus.h"

int main(int argc, char **argv)
{
    int ret;
    char *tty, *filename;
    ubus bus;
    ubus_spipe pipe;
    int fail_rate = 2;
    int fd;

    if(argc<3 || strcmp(argv[1], "-h")==0) {
        fprintf(stderr, "usage: %s /dev/ttyUSB0 filename [fail-rate]\n", argv[0]);
        exit(1);
    }
    tty = argv[1];
    filename = argv[2];

    if(argc>3) fail_rate = atoi(argv[3]);
    fprintf(stderr, "Target fail rate %d\n", fail_rate);

    fd = open(filename, O_WRONLY|O_CREAT|O_TRUNC);
    if(fd<0) {
        fprintf(stderr, "cannot open %s: %s\n", filename, strerror(errno));
        exit(1);
    }

    ret = ubus_bus_init(&bus, tty, 115200);
    if(ret) exit(1);

    ret = ubus_test_set_fail_rate(bus, fail_rate);
    assert(ret>=0);
    ret = ubus_test_enable(bus, UBUS_TEST_MISSING_BYTES);
    assert(ret>=0);
    ret = ubus_test_enable(bus, UBUS_TEST_PACKET_LOSS);
    assert(ret>=0);
    ret = ubus_test_enable(bus, UBUS_TEST_CORRUPTION);
    assert(ret>=0);
    ret = ubus_test_enable(bus, UBUS_TEST_GARBAGE);
    assert(ret>=0);

    pipe = ubus_slave_pipe_new(bus, FAC_2_DM368_REQUEST_SIG, FAC_2_DM368_REPLY_SIG);
    if(NULL==pipe) exit(1);

    while(1) {

        ubus_request_t request;
        ubus_reply_t reply;
        ssize_t sz;

        fprintf(stderr, "Waiting for requests....\n");
        ret = ubus_slave_recv(pipe, &request, 3);
        if(ret<0 && -EAGAIN==ret) {
            continue;
        }
        else if(ret<0) {
            fprintf(stderr, "ubus_slave_recv failed: %d\n", ret);
            break;
        }

        fprintf(stderr, "Received Request:\n");
        fprintf(stderr, "cmd=0x%x, data[0]=0x%x, len=%d\n", 
                request.command, request.data[0], request.data_length);

        reply.command = request.command;
        reply.state = UBUS_STATE_OK;
        reply.data_length = 1;
        reply.data[0] = sz; // return actual bytes saved

        ret = ubus_slave_send(pipe, &reply);
        if(ret<0) {
            fprintf(stderr, "ubus_slave_send failed: %d\n", ret);
            break;
        }

        // end of file
        if(request.data_length==0) {
            break;
        }
        else {
            // save data payload to file
            sz = write(fd, request.data, request.data_length);
            if(sz<0) {
                fprintf(stderr, "write failed: %s\n", strerror(errno));
                exit(1);
            }
            else if(sz<request.data_length) {
                fprintf(stderr, "write byte short: %d\n", (int)sz);
                exit(1);
            }
        }

    } // end of while loop

    ubus_slave_pipe_del(pipe);
    ubus_bus_exit(bus);
    close(fd);
    exit(0);
}

