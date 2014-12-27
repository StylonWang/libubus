#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include "ubus.h"

void generate_request(ubus_request_t *request)
{
    int i;

    // randomly generate requests
    memset(request, 0, sizeof(*request));
    request->command = rand() % 256;
    request->data_length = rand() % 18;
    for(i=0; i<request->data_length; ++i) {
        request->data[i] = (rand() & 0xFF);
    }
}

int main(int argc, char **argv)
{
    int ret;
    char *tty;
    ubus bus;
    ubus_mpipe pipe;
    unsigned long expected_runs = 1024;
    unsigned long runs = 0;
    int fail_rate = 2;

    if(argc<2) {
        fprintf(stderr, "usage: %s /dev/ttyUSB0\n", argv[0]);
        exit(1);
    }
    tty = argv[1];

    if(argc>2) expected_runs = atoi(argv[2]);
    fprintf(stderr, "Target to %ld runs\n", expected_runs);
    if(argc>3) fail_rate = atoi(argv[3]);
    fprintf(stderr, "Target fail rate %d\n", fail_rate);

    srand(time(NULL));

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

    while(runs < expected_runs) {
        ubus_request_t request;
        ubus_reply_t reply;
        // generate request on the fly
        generate_request(&request);

        runs++;
        fprintf(stderr, "\nRun[%ld] %x %x %x\n", runs-1, request.command, request.data_length, request.data[0]);
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
    }

    ubus_master_pipe_del(pipe);
    ubus_test_report(bus);
    fprintf(stderr, "Performed %ld runs\n", runs);
    ubus_bus_exit(bus);
    exit(0);
}

