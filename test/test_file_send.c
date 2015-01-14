#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "ubus.h"

int main(int argc, char **argv)
{
    int ret;
    char *tty, *filename;
    ubus bus;
    ubus_mpipe pipe;
    unsigned long bytes=0, runs=0;
    int fail_rate = 2;
    int fd;

    if(argc<3 || strcmp(argv[1], "-h")==0) {
        fprintf(stderr, "usage: %s /dev/ttyUSB0 filename [fail-rate]\n", argv[0]);
        exit(1);
    }
    tty = argv[1];
    filename = argv[2];

    fprintf(stderr, "File to send: %s\n", filename);
    if(argc>3) fail_rate = atoi(argv[3]);
    fprintf(stderr, "Target fail rate %d\n", fail_rate);

    fd = open(filename, O_RDONLY);
    if(fd<0) {
        fprintf(stderr, "open %s failed: %s\n", filename, strerror(errno));
        exit(1);
    }

    ret = ubus_bus_init(&bus, tty, 115200);
    if(ret) exit(1);

    pipe = ubus_master_pipe_new(bus, FAC_2_DM368_REQUEST_SIG, FAC_2_DM368_REPLY_SIG);
    if(NULL==pipe) exit(1);

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

    while(1) {
        ubus_request_t request;
        ubus_reply_t reply;
        ssize_t sz;

        memset(&request, 0, sizeof(request));

        request.command = 0x88;

        // copy contents of file into requests
        sz = read(fd, request.data, sizeof(request.data));
        if(sz<0) {
            fprintf(stderr, "read failed: %s\n", strerror(errno));
            exit(1);
        }
        request.data_length = sz;

        bytes += sz;
        runs++;
        fprintf(stderr, "\nRun[%ld] %ld %x %d %x\n", runs-1, bytes, request.command, request.data_length, request.data[0]);
        ret = ubus_master_send_recv(pipe, &request, &reply);
        if(ret<0) {
            fprintf(stderr, "failed\n");
            break;
        }

        fprintf(stderr, "state=%s, data[0]=0x%x, len=%d\n", 
                (reply.state==UBUS_STATE_OK)? "ok" : 
                (reply.state==UBUS_STATE_UNKNOWN_COMMAND)? "unknown command" :
                (reply.state==UBUS_STATE_CRC_ERROR)? "crc error" :
                (reply.state==UBUS_STATE_BUSY)? "busy" : "invalid state",
                reply.data[0], reply.data_length);

        if(reply.state!=UBUS_STATE_OK) {
            fprintf(stderr, "reply failed: %d\n", reply.state);
            break;
        }

        if(reply.data[0] != sz) {
            fprintf(stderr, "reply size incorrect: %d\n", reply.data[0]);
            exit(1);
        }

        if(sz==0) { // end of file
            fprintf(stderr, "EOF\n");
            break;
        }
    }

    ubus_master_pipe_del(pipe);
    ubus_test_report(bus);
    fprintf(stderr, "Performed %ld runs, sent %ld bytes\n", runs, bytes);
    ubus_bus_exit(bus);
    close(fd);
    exit(0);
}

