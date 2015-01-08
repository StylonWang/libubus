#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "ubus.h"
#include "test_multi_pipe.h"

ubus g_bus;
 // 5th pipe is used to test the limitation, which is 4 pipes.
ubus_spipe g_pipes[MAX_TEST_PIPES+1];
pthread_t g_threads[MAX_TEST_PIPES];

bool g_should_stop = false;

void *thread_routine(void *data)
{
    int *pi = (int *)data; // avoid int to void * conversion so compiler won't complain
    int index = *pi;
    int ret = 0;

    fprintf(stderr, "Thread[%d] runs\n", index);

    while(!g_should_stop) {
        ubus_request_t request;
        ubus_reply_t reply;

        fprintf(stderr, "Waiting for requests....\n");
        ret = ubus_slave_recv(g_pipes[index], &request, 3);
        if(ret<0 && -EAGAIN==ret) {
            continue;
        }
        else if(ret<0) {
            fprintf(stderr, "ubus_slave_recv failed: %d\n", ret);
            break;
        }

        fprintf(stderr, "Th[%d] eceived Request:\n", index);
        fprintf(stderr, "cmd=0x%x, data[0]=0x%x, len=%d\n", 
                request.command, request.data[0], request.data_length);

        reply.command = request.command;
        reply.state = UBUS_STATE_OK;
        reply.data_length = 0;

        ret = ubus_slave_send(g_pipes[index], &reply);
        if(ret<0) {
            fprintf(stderr, "ubus_slave_send failed: %d\n", ret);
            break;
        }
    } // end of while loop

    fprintf(stderr, "Thread[%d] exits\n", index);
    return NULL;
}

void signal_handler(int signo)
{
    g_should_stop = true;
}

int main(int argc, char **argv)
{
    int ret;
    char *tty;
    int i;
    ubus_spipe pipe;

    if(argc<2) {
        fprintf(stderr, "usage: %s /dev/ttyUSB0\n", argv[0]);
        exit(1);
    }
    tty = argv[1];

    signal(SIGINT, signal_handler);

    ret = ubus_bus_init(&g_bus, tty, 115200);
    if(ret) exit(1);

#if 0
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
#endif

    // create all the pipes
    for(i=0; i<MAX_TEST_PIPES; ++i) {
        pipe = ubus_slave_pipe_new(g_bus, test_signatures[i].request_sig, test_signatures[i].reply_sig);
        if(NULL==pipe) {
            fprintf(stderr, "Failed to create spipe[%d].\n", i);
            exit(1);
        }
        g_pipes[i] = pipe;
    }

    // deliberately create 5th pipe, which is supposed to fail
    pipe = ubus_slave_pipe_new(g_bus, test_signatures[MAX_TEST_PIPES].request_sig,
                                test_signatures[MAX_TEST_PIPES].reply_sig);
    if(NULL!=pipe) {
        fprintf(stderr, "Failed: should be \"not enough pipe\".\n");
        exit(1);
    }

    // create all threads
    for(i=0; i<MAX_TEST_PIPES; ++i) {
        ret = pthread_create(&g_threads[i], NULL, thread_routine, (void *)&i);
        if(ret<0) {
            fprintf(stderr, "Failed to create thread[%d]: %s\n", i, strerror(errno));
            exit(1);
        }
    }

    while(!g_should_stop) {
        sleep(5);
    }

    // wait for all threads to stop
    for(i=0; i<MAX_TEST_PIPES; ++i) {
        void *pret;
        pthread_join(g_threads[i], &pret);
        ubus_slave_pipe_del(g_pipes[i]);
        fprintf(stderr, "Thread[%d] cleared\n", i);
    }

    ubus_test_report(g_bus);
    ubus_bus_exit(g_bus);
    exit(0);
}

