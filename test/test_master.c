#include <stdio.h>
#include <stdlib.h>

#include "ubus.h"

#include "test_case.h"

int main(int argc, char **argv)
{
    int ret;
    char *tty;
    ubus bus;
    ubus_mpipe pipe;
    int i;

    if(argc<2) {
        fprintf(stderr, "usage: %s /dev/ttyUSB0\n", argv[0]);
        exit(1);
    }
    tty = argv[1];

    ret = ubus_bus_init(&bus, tty, 115200);
    if(ret) exit(1);

    pipe = ubus_master_pipe_new(bus, FAC_2_DM368_REQUEST_SIG, FAC_2_DM368_REPLY_SIG);
    if(NULL==pipe) exit(1);

    for(i=0; i<sizeof(test_cases)/sizeof(test_cases[0]); ++i) {
        ubus_reply_t reply;

        fprintf(stderr, "\nRun test[%d]\n", i);
        ret = ubus_master_send_recv(pipe, &test_cases[i], &reply);
        if(ret<0) {
            fprintf(stderr, "failed\n");
            exit(1);
        }

        fprintf(stderr, "state=%s, data[0]=0x%x, len=%d\n", 
                (reply.state==UBUS_STATE_OK)? "ok" : 
                (reply.state==UBUS_STATE_UNKNOWN_COMMAND)? "unknown command" :
                (reply.state==UBUS_STATE_CRC_ERROR)? "crc error" :
                (reply.state==UBUS_STATE_BUSY)? "busy" : "invalid state",
                reply.data[0], reply.data_length);

        if(reply.state!=UBUS_STATE_OK) {
            fprintf(stderr, "reply failed: %d\n", reply.state);
        }
    }

    ubus_master_pipe_del(pipe);
    ubus_bus_exit(bus);
    exit(0);
}

