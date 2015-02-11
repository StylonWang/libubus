#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <assert.h>

#include "ubus.h"

int main(int argc, char **argv)
{
    int ret;
    char *tty;
    ubus bus;
    ubus_spipe pipe;
    int fail_rate = 2;

    if(argc<2 || strcmp(argv[1], "-h")==0) {
        fprintf(stderr, "usage: %s /dev/ttyUSB0 [fail-rate]\n", argv[0]);
        exit(1);
    }
    tty = argv[1];

    if(argc>2) fail_rate = atoi(argv[2]);
    fprintf(stderr, "Target fail rate %d\n", fail_rate);

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

        //fprintf(stderr, "Waiting for requests....\n");
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
        reply.data_length = 0;

        ret = ubus_slave_send(pipe, &reply);
        if(ret<0) {
            fprintf(stderr, "ubus_slave_send failed: %d\n", ret);
            break;
        }
    }

    ubus_slave_pipe_del(pipe);
    ubus_bus_exit(bus);
    exit(0);
}

