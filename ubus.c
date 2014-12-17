
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
#include "ubus.h"

// -------------------------------------------
// actual data layout in packets flowing on RS232
//
#pragma pack(push)
#pragma pack(1)
typedef struct
{
	unsigned char REQ[2];
	unsigned char COMMAND;
	unsigned char SEQUENCE;
	unsigned char FRAGMENT;
	unsigned char LENGTH;
	unsigned char PAYLOAD[17];
	unsigned int CHECKSUM;
	unsigned char STOP_BYTE;
} UART_REQUEST;
#pragma pack(pop)

#pragma pack(push)
#pragma pack(1)
typedef struct
{
	unsigned char RPL[2];
	unsigned char COMMAND;
	unsigned char STATE;
	unsigned char SEQUENCE;
	unsigned char FRAGMENT;
	unsigned char LENGTH;
	unsigned char PAYLOAD[16]; //reply payload has only 15 bytes in order to fix 24 bytes total
	unsigned int CHECKSUM;
	unsigned char STOP_BYTE;
} UART_REPLY;
#pragma pack(pop)
//
// -------------------------------------------

#define MAX_REQUEST_IN_PIPE (32)
#define MAX_PACKETS_IN_QUEUE (128)

#define MAX_MPIPES (4)
#define MAX_SPIPES (4)

typedef struct _ubus_mpipe
{
    packet_sig_t request_sig;
    packet_sig_t reply_sig;

    pthread_mutex_t pipe_lock;
    struct {
        bool request_sent;
        bool reply_recved;
        UART_REQUEST request; // single request-reply mapping. no packetization.
        UART_REPLY reply;
    } queue[MAX_REQUEST_IN_PIPE];

} ubus_mpipe_t;

typedef struct _ubus_spipe
{
    packet_sig_t request_sig;
    packet_sig_t reply_sig;

    pthread_mutex_t pipe_lock;
    struct {
        bool request_sent;
        bool reply_recved;
        UART_REQUEST request; // single request-reply mapping. no packetization.
        UART_REPLY reply;
    } queue[MAX_REQUEST_IN_PIPE];

} ubus_spipe_t;

typedef struct _ubus_bus_t
{
    int fd; // file descriptor to RS232 device
    unsigned long sequence; // transport sequence

    // all packets received from RS232 are queued here for processing
    struct {
        bool in_use; //
        union {
            UART_REQUEST request;
            UART_REPLY reply;
        } u;
    } packet_queue[MAX_PACKETS_IN_QUEUE];

    pthread_mutex_t queue_lock;

    // related to thread handling
    pthread_t thread;
    bool thread_should_stop;
    pthread_mutex_t thread_lock;
    pthread_cond_t thread_stop_cond;

    struct {
        bool in_use;
        ubus_mpipe pipe; 
    } mpipes[MAX_MPIPES];

    struct {
        bool in_use;
        ubus_spipe pipe; 
    } spipes[MAX_SPIPES];
     
} ubus_bus_t;

typedef enum ubus_pipe_type
{
    UBUS_MASTER=1,
    UBUS_SLAVE,
} ubus_pipe_type;

// debug log
#define LOG_ERR(fmt, args...) do { fprintf(stderr, "[%s:%d]"fmt, __FUNCTION__, __LINE__,##args); } while(0)
#define LOG_DBG(fmt, args...) do { fprintf(stderr, "[%s:%d]"fmt, __FUNCTION__, __LINE__, ##args); } while(0)

//--------------------------------------------------------------
// Internal routines

static int ubus_uart_init(char *uart_device, int baud_rate)
{
    int fd;
    struct termios newtio;

	fd = open(uart_device, O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd < 0) {
        LOG_ERR("unable to init uart port: %s\n", strerror(errno));
        return -1;
	}

	if(fcntl(fd, F_SETFL, 0)<0) {
        LOG_ERR("fcntl failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

	tcflush(fd, TCIOFLUSH);
	if (tcgetattr(fd, &newtio) < 0) {
        LOG_ERR("tcgetattr failed: %s\n", strerror(errno));
		close(fd);
        return -1;
	}
	cfmakeraw(&newtio);
	switch(baud_rate) {
		case 9600:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
		case 19200:
			cfsetispeed(&newtio, B19200);
			cfsetospeed(&newtio, B19200);
			break;
		case 38400:
			cfsetispeed(&newtio, B38400);
			cfsetospeed(&newtio, B38400);
			break;
		case 57600:
			cfsetispeed(&newtio, B57600);
			cfsetospeed(&newtio, B57600);
			break;
		case 115200:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;
		default:
			break;
	}
	if (tcsetattr(fd, TCSANOW, &newtio) < 0) {
        LOG_ERR("tcgetattr failed: %s\n", strerror(errno));
		close(fd);
        return -1;
	}

	tcflush(fd, TCIOFLUSH);
    return fd;
}

static void *ubus_thread_routine(void *data)
{
    ubus_bus_t *bus_obj = (ubus_bus_t *)data;

    while(!bus_obj->thread_should_stop) {

    }

    // signal that thread is done cleaning and about to be closed
    pthread_cond_signal(&bus_obj->thread_stop_cond);

    return 0;
}

// End of internal routines
//--------------------------------------------------------------

int ubus_bus_init(ubus *pbus, char *uart_device, int baud_rate)
{
    ubus_bus_t *bus_obj;
    int fd;
    int ret;

    fd = ubus_uart_init(uart_device, baud_rate);
    if(fd<0) {
        return -1;
    }

    bus_obj = calloc(1, sizeof(ubus_bus_t));
    if(NULL==bus_obj) {
        return -1;
    }

    // init ubus object
    bus_obj->fd = fd;
    bus_obj->sequence = 1;
    ret = pthread_mutex_init(&bus_obj->queue_lock, NULL);
    if(ret) {
        LOG_ERR("unable to create ubus queue lock: %s\n", strerror(ret));
        free(bus_obj);
        return -1;
    }

    ret = pthread_mutex_init(&bus_obj->thread_lock, NULL);
    if(ret) {
        LOG_ERR("unable to create ubus thread lock: %s\n", strerror(ret));
        pthread_mutex_destroy(&bus_obj->queue_lock);
        free(bus_obj);
        return -1;
    }

    ret = pthread_cond_init(&bus_obj->thread_stop_cond, NULL);
    if(ret) {
        LOG_ERR("unable to create ubus thread stop cond: %s\n", strerror(ret));
        pthread_mutex_destroy(&bus_obj->queue_lock);
        pthread_mutex_destroy(&bus_obj->thread_lock);
        free(bus_obj);
        return -1;
    }

    // create bus thread
    ret = pthread_create(&bus_obj->thread, NULL, ubus_thread_routine, bus_obj);
    if(ret) {
        LOG_ERR("unable to create ubus thread: %s\n", strerror(ret));
        pthread_cond_destroy(&bus_obj->thread_stop_cond);
        pthread_mutex_destroy(&bus_obj->queue_lock);
        pthread_mutex_destroy(&bus_obj->thread_lock);
        free(bus_obj);
        return -1;
    }

    *pbus = (ubus)bus_obj;
    return 0;
}

void ubus_bus_exit(ubus bus)
{
    ubus_bus_t *bus_obj = (ubus_bus_t *)bus;
    void *ret;

    LOG_DBG("signal thread to stop\n");
    // signal thread to stop
    bus_obj->thread_should_stop =1;

    //pthread_mutex_lock(&bus_obj->thread_lock);
    //pthread_cond_wait(&bus_obj->thread_stop_cond, &bus_obj->thread_lock);
    //pthread_mutex_unlock(&bus_obj->thread_lock);
    pthread_join(bus_obj->thread, &ret);
    LOG_DBG("thread stopped\n");
    
    pthread_cond_destroy(&bus_obj->thread_stop_cond);
    pthread_mutex_destroy(&bus_obj->queue_lock);
    pthread_mutex_destroy(&bus_obj->thread_lock);
    free(bus_obj);
}

// create and delete master-side pipe
ubus_mpipe * ubus_master_pipe_new(ubus bus, packet_sig_t request_sig, packet_sig_t reply_sig)
{

}

void ubus_master_pipe_del(ubus_mpipe pipe)
{

}

// create and delete slave-side pipe
ubus_spipe * ubus_slave_pipe_new(ubus bus, packet_sig_t request_sig, packet_sig_t reply_sig)
{

}

void ubus_slave_pipe_del(ubus_spipe pipe)
{

}

// Master side: send request and wait for reply
int ubus_master_send(ubus_mpipe pipe, const ubus_request_t *request)
{

}

int ubus_master_recv(ubus_mpipe pipe, ubus_reply_t *reply)
{

}

// Slave side: receive request, process request and send reply
int ubus_slave_recv(ubus_spipe pipe, ubus_request_t *request)
{

}

int ubus_slave_send(ubus_spipe pipe, const ubus_reply_t *reply)
{

}



