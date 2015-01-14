#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>

#include "ubus.h"
#include "test_multi_pipe.h"

ubus g_bus;
 // 5th pipe is used to test the limitation, which is 4 pipes.
ubus_mpipe g_pipes[MAX_TEST_PIPES+1];
pthread_t g_threads[MAX_TEST_PIPES];
unsigned long g_runs[MAX_TEST_PIPES];
unsigned long g_expected_runs = 1024;

bool g_should_stop = false;

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

void *thread_routine(void *data)
{
    int index = (int )data; 
    int ret = 0;

    fprintf(stderr, "Thread[%d] runs\n", index);

    while(g_runs[index] < g_expected_runs && !g_should_stop) {
        ubus_request_t request;
        ubus_reply_t reply;
        // generate request on the fly
        generate_request(&request);

        g_runs[index]++;
        fprintf(stderr, "\nTh[%d] Run[%ld] %x %x %x\n", index, g_runs[index]-1, request.command, request.data_length, request.data[0]);
        ret = ubus_master_send_recv(g_pipes[index], &request, &reply);
        if(ret<0) {
            fprintf(stderr, "Thread[%d] failed\n", index);
            exit(1); // terminate the whole process if failure
        }

        fprintf(stderr, "Thread[%d] state=%s, data[0]=0x%x, len=%d\n", 
                index,
                (reply.state==UBUS_STATE_OK)? "ok" : 
                (reply.state==UBUS_STATE_UNKNOWN_COMMAND)? "unknown command" :
                (reply.state==UBUS_STATE_CRC_ERROR)? "crc error" :
                (reply.state==UBUS_STATE_BUSY)? "busy" : "invalid state",
                reply.data[0], reply.data_length);

        // check reply state
        if(reply.state!=UBUS_STATE_OK) {
            fprintf(stderr, "Thread[%d] reply failed: %d\n", index, reply.state);
            exit(1); // terminate the whole process if failure
        }

        // check reply data length, which is supposed to be 1 bytes less than request
        if(request.data_length>0) {
            
            if(reply.data_length!=(request.data_length-1)) {
                fprintf(stderr, "Thread[%d] reply failed, len mistach: %d/%d \n",
                        index, reply.data_length, request.data_length-1);
                exit(1); // terminate the whole process if failure
            }

            if(0!=memcmp(reply.data, request.data, reply.data_length)) {
                int i;
                fprintf(stderr, "Thread[%d] reply failed, data mistach\n", index);
                fprintf(stderr, "Request is: \n");
                for(i=0; i<reply.data_length; ++i) {
                    fprintf(stderr, "%x %s", request.data[i], ((i%8)==7)? "\n" : "");
                }
                fprintf(stderr, "\nReply is: \n");
                for(i=0; i<reply.data_length; ++i) {
                    fprintf(stderr, "%x %s", reply.data[i], ((i%8)==7)? "\n" : "");
                }
                fprintf(stderr, "\n");
                exit(1); // terminate the whole process if failure
            }
        } // end of if(request.data_length>0)

    } // end of while loop

    fprintf(stderr, "Thread[%d] exits\n", index);
    return NULL;
}

void signal_handler(int signo)
{
    static int counter;

    fprintf(stderr, "Got signal %d\n", signo);
    g_should_stop = true;
    if(++counter>3) exit(0);
}

int main(int argc, char **argv)
{
    int ret;
    char *tty;
    int fail_rate = 2;
    int i;
    ubus_mpipe pipe;

    if(argc<2 || strcmp(argv[1], "-h")==0 ) {
        fprintf(stderr, "usage: %s /dev/ttyUSB0 [runs] [fail-rate]\n", argv[0]);
        exit(1);
    }
    tty = argv[1];

    if(argc>2) g_expected_runs = atoi(argv[2]);
    fprintf(stderr, "Target to %ld runs\n", g_expected_runs);
    if(argc>3) fail_rate = atoi(argv[3]);
    fprintf(stderr, "Target fail rate %d\n", fail_rate);

    srand(time(NULL));
    signal(SIGINT, signal_handler);

    ret = ubus_bus_init(&g_bus, tty, 115200);
    if(ret) exit(1);

    // test parameters
    ret = ubus_test_set_fail_rate(g_bus, fail_rate);
    assert(ret>=0);
    ret = ubus_test_enable(g_bus, UBUS_TEST_MISSING_BYTES);
    assert(ret>=0);
    ret = ubus_test_enable(g_bus, UBUS_TEST_PACKET_LOSS);
    assert(ret>=0);
    ret = ubus_test_enable(g_bus, UBUS_TEST_CORRUPTION);
    assert(ret>=0);
    ret = ubus_test_enable(g_bus, UBUS_TEST_GARBAGE);
    assert(ret>=0);

    // create all the pipes
    for(i=0; i<MAX_TEST_PIPES; ++i) {
        pipe = ubus_master_pipe_new(g_bus, test_signatures[i].request_sig,
                                    test_signatures[i].reply_sig);
        if(NULL==pipe) {
            fprintf(stderr, "Failed to create mpipe[%d].\n", i);
            exit(1);
        }
        g_pipes[i] = pipe;
        g_runs[i] = 0;
    }

    // deliberately create 5th pipe, which is supposed to fail
    pipe = ubus_master_pipe_new(g_bus, test_signatures[MAX_TEST_PIPES].request_sig,
                                test_signatures[MAX_TEST_PIPES].reply_sig);
    if(NULL!=pipe) {
        fprintf(stderr, "Failed: should be \"not enough pipe\".\n");
        exit(1);
    }

    // create all threads
    for(i=0; i<MAX_TEST_PIPES; ++i) {
        ret = pthread_create(&g_threads[i], NULL, thread_routine, 
                            (void *)(unsigned long)i); // cast to ulong to avoid compiler warnings
        if(ret<0) {
            fprintf(stderr, "Failed to create thread[%d]: %s\n", i, strerror(errno));
            exit(1);
        }
    }

    // wait for all threads to stop
    for(i=0; i<MAX_TEST_PIPES; ++i) {
        void *pret;
        pthread_join(g_threads[i], &pret);
        ubus_master_pipe_del(g_pipes[i]);
        fprintf(stderr, "Thread[%d] cleared, performed %ld runs\n", i, g_runs[i]);
    }

    fprintf(stderr, "All threads stopped...\n");

    ubus_test_report(g_bus);
    ubus_bus_exit(g_bus);
    exit(0);
}

