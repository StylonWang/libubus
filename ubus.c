
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
#include "ubus.h"

// legacy request/reply signature used in c285 projects
packet_sig_t DM368_2_FAC_REQUEST_SIG = { 0x02, 0x61, 0x06, };
packet_sig_t DM368_2_FAC_REPLY_SIG = { 0x03, 0x62, 0x07, };

packet_sig_t FAC_2_DM368_REQUEST_SIG = { 0x04, 0x63, 0x08, };
packet_sig_t FAC_2_DM368_REPLY_SIG = { 0x05, 0x64, 0x09, };

packet_sig_t DM368_2_NUC100_REQUEST_SIG = { 0x44, 0x54, 0x4F, };
packet_sig_t DM368_2_NUC100_REPLY_SIG = { 0x4D, 0x59, 0x52, };

packet_sig_t NUC100_2_DM368_REQUEST_SIG = { 0x43, 0x53, 0x50, };
packet_sig_t NUC100_2_DM368_REPLY_SIG = { 0x55, 0x4C, 0x45, };

#if 0
#define DM368_2_FAC_MASTER_ID       (0x00026106)
#define DM368_2_FAC_SLAVE_ID        (0x00036207)
#define FAC_2_DM368_MASTER_ID       (0x00046308)
#define FAC_2_DM368_SLAVE_ID        (0x00056409)
#define DM368_2_NUC100_MASTER_ID    (0x0044544F)
#define DM368_2_NUC100_SLAVE_ID     (0x004D5952)
#define NUC100_2_DM368_MASTER_ID    (0x00435350)
#define NUC100_2_DM368_SLAVE_ID     (0x00554C45)
#endif 


// -------------------------------------------
// actual data layout in packets flowing on RS232
//

#define EXPECTED_PACK_SIZE (28)

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
    // reverse mapping back to bus object
    int index;
    void *bus_obj;

    packet_sig_t request_sig;
    packet_sig_t reply_sig;

    pthread_mutex_t pipe_lock;
    pthread_cond_t reply_cond;
    bool reply_received;
    UART_REQUEST request; // single request-reply mapping. no packetization.
    UART_REPLY reply;

} ubus_mpipe_t;

typedef struct _ubus_spipe
{
    // reverse mapping back to bus object
    int index;
    void *bus_obj;

    packet_sig_t request_sig;
    packet_sig_t reply_sig;

    pthread_mutex_t pipe_lock;
    UART_REQUEST request; // single request-reply mapping. no packetization.
    UART_REPLY reply;

} ubus_spipe_t;

typedef struct _ubus_bus_t
{
    pthread_mutex_t fd_lock;
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

#ifdef TEST_MODE
static int ubus_uart_init(char *uart_device, int baud_rate)
{
    //TODO
}
#else // TEST_MODE
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
#endif

int ubus_mpipe_recv_reply(ubus_mpipe_t *pipe, const UART_REPLY *reply)
{
    LOG_DBG("Reply, sig=0x%x:0x%x:0x%x cmd=0x%x state=0x%x len=%d\n",
            reply->RPL[0], reply->RPL[1], reply->STOP_BYTE,
            reply->COMMAND, reply->STATE, reply->LENGTH);

    if(ubus_check_reply_crc(reply)<0) {
        return -1;
    }

    pthread_mutex_lock(&pipe->pipe_lock);

    if(reply->COMMAND == pipe->request.COMMAND) {
        memcpy(pipe->reply, reply, sizeof(*reply));
        pipe->reply_received = true;
        pthread_cond_signal(&pipe->reply_cond);
    }

    pthread_mutex_unlock(&pipe->pipe_lock);

    return 0;
}

int ubus_spipe_recv_request(ubus_spipe_t *pipe, const UART_REQUEST *request)
{
    LOG_DBG("Request, sig=0x%x:0x%x:0x%x cmd=0x%x state=0x%x len=%d\n",
            request->REQ[0], request->REQ[1], request->STOP_BYTE,
            request->COMMAND, request->LENGTH);

    return 0;
}

// currently only handle one request/reply.
static int ubus_dispatch_data(ubus_bus_t *bus_obj, void *data, int data_len)
{
    int i;
    UART_REQUEST *request = (UART_REQUEST *)data;
    UART_REPLY *reply = (UART_REPLY *)reply;

    // check if this is a reply for master pipe or a request for slave pipe
    
    // check on master pipes
    for(i=0; i<MAX_MPIPES; ++i) {
        if(bus_obj->mpipes[i].in_use && 
           bus_obj->mpipes[i].pipe.reply_sig.start_byte_0 == reply->RPL[0] &&
           bus_obj->mpipes[i].pipe.reply_sig.start_byte_1 == reply->RPL[1] &&
           bus_obj->mpipes[i].pipe.reply_sig.stop_byte == reply->STOP_BYTE) {

            return ubus_mpipe_recv_reply(&bus_obj->mpipes[i].pipe, reply);
        }
    }

    // check on slave pipes
    for(i=0; i<MAX_MPIPES; ++i) {
        if(bus_obj->spipes[i].in_use && 
           bus_obj->spipes[i].pipe.request_sig.start_byte_0 == request->REQ[0] &&
           bus_obj->spipes[i].pipe.request_sig.start_byte_1 == request->REQ[1] &&
           bus_obj->spipes[i].pipe.request_sig.stop_byte == request->STOP_BYTE) {

            return ubus_spipe_recv_request(&bus_obj->spipes[i].pipe, request);
        }
    }

    LOG_DBG("unrecognized data received: 0x%x 0x%x 0x%x\n",
                reply->RPL[0],
                reply->RPL[1],
                reply->STOP_BYTE
                );

    return -1;
}

static void *ubus_thread_routine(void *data)
{
    ubus_bus_t *bus_obj = (ubus_bus_t *)data;
    union {
        UART_REQUEST request;
        UART_REPLY reply;
    } u;

    while(!bus_obj->thread_should_stop) {

        int ret = 0;
        ssize_t read_bytes;
        fd_set readfs;
        struct timeval timeout;
        
		timeout.tv_usec = 0L;
		timeout.tv_sec = 5; //50L;
		FD_ZERO(&readfs);
		FD_SET(bus_obj->fd, &readfs);

        // read from RS232
		ret = select(bus_obj->fd + 1, &readfs, NULL, NULL, &timeout);
        if(ret<0 && errno!=EINTR) {
            LOG_ERR("select failed: %s\n", strerror(errno));
            sleep(2);
            continue;
		}
		else if (ret == 0) {
            continue; // timeout with no data
		}

        // have data, process it
        read_bytes = read(bus_obj->fd, &u, sizeof(u)) {

        ubus_dispatch_data(bus_obj, &u, sizeof(u));
    }

    // signal that thread is done cleaning and about to be closed
    pthread_cond_signal(&bus_obj->thread_stop_cond);

    return 0;
}

static int ubus_internal_check(void)
{
    int request_size = sizeof(UART_REQUEST);
    int reply_size = sizeof(UART_REPLY);
    union {
        UART_REQUEST request;
        UART_REPLY reply;
    } u;

    // Check compiler has honored "pragma packing" directive.
    // If not, abort.

    if(request_size != EXPECTED_PACK_SIZE) {
        LOG_ERR("struct packing not working on req.\n");
        return -1;
    }

    if(reply_size != EXPECTED_PACK_SIZE) {
        LOG_ERR("struct packing not working on req.\n");
        return -1;
    }

    if(sizeof(u) != EXPECTED_PACK_SIZE) {
        LOG_ERR("struct packing not working on union.\n");
        return -1;
    }

    return 0;
}

// End of internal routines
//--------------------------------------------------------------

int ubus_bus_init(ubus *pbus, char *uart_device, int baud_rate)
{
    ubus_bus_t *bus_obj;
    int fd;
    int ret;

    if(ubus_internal_check()<0) {
        return -1;
    }

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

    ret = pthread_mutex_init(&bus_obj->fd_lock, NULL);
    if(ret) {
        LOG_ERR("unable to create ubus fd lock: %s\n", strerror(ret));
        free(bus_obj);
        return -1;
    }

    ret = pthread_mutex_init(&bus_obj->queue_lock, NULL);
    if(ret) {
        LOG_ERR("unable to create ubus queue lock: %s\n", strerror(ret));
        pthread_mutex_destroy(&bus_obj->fd_lock);
        free(bus_obj);
        return -1;
    }

    ret = pthread_mutex_init(&bus_obj->thread_lock, NULL);
    if(ret) {
        LOG_ERR("unable to create ubus thread lock: %s\n", strerror(ret));
        pthread_mutex_destroy(&bus_obj->queue_lock);
        pthread_mutex_destroy(&bus_obj->fd_lock);
        free(bus_obj);
        return -1;
    }

    ret = pthread_cond_init(&bus_obj->thread_stop_cond, NULL);
    if(ret) {
        LOG_ERR("unable to create ubus thread stop cond: %s\n", strerror(ret));
        pthread_mutex_destroy(&bus_obj->thread_lock);
        pthread_mutex_destroy(&bus_obj->queue_lock);
        pthread_mutex_destroy(&bus_obj->fd_lock);
        free(bus_obj);
        return -1;
    }

    // create bus thread
    ret = pthread_create(&bus_obj->thread, NULL, ubus_thread_routine, bus_obj);
    if(ret) {
        LOG_ERR("unable to create ubus thread: %s\n", strerror(ret));
        pthread_cond_destroy(&bus_obj->thread_stop_cond);
        pthread_mutex_destroy(&bus_obj->thread_lock);
        pthread_mutex_destroy(&bus_obj->queue_lock);
        pthread_mutex_destroy(&bus_obj->fd_lock);
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
    pthread_mutex_destroy(&bus_obj->fd_lock);
    pthread_mutex_destroy(&bus_obj->queue_lock);
    pthread_mutex_destroy(&bus_obj->thread_lock);
    free(bus_obj);
}

// create and delete master-side pipe
ubus_mpipe * ubus_master_pipe_new(ubus bus, packet_sig_t request_sig, packet_sig_t reply_sig)
{
    ubus_bus_t *bus_obj = (ubus_bus_t *)bus;
    int i;

    for(i=0; i<MAX_MPIPES; ++i) {
        if(!bus_obj->mpipes[i].in_use) {
            // allocate a mpipe slot and do initialization
            LOG_DBG("allocated mpipes[%d]\n", i);
            memset(&bus_obj->mpipes[i], 0, sizeof(bus_obj->mpipes[i]));
            bus_obj->mpipes[i].in_use = true;
            bus_obj->mpipes[i].pipe.request_sig = request_sig;
            bus_obj->mpipes[i].pipe.reply_sig = reply_sig;
            bus_obj->mpipes[i].pipe.bus = bus_obj;
            bus_obj->mpipes[i].pipe.index = i;
            pthread_mutex_init(&bus_obj->mpipes[i].pipe.pipe_lock, NULL);

            return (ubus_mpipe *)&bus_obj->mpipes[i].pipe;
        }
    }

    LOG_ERR("no more available master pipes\n");
    return -1;
}

void ubus_master_pipe_del(ubus_mpipe p)
{
    ubus_mpipe_t *pipe = (ubus_mpipe_t *)p;
    ubus_bus_t *bus_obj = (ubus_bus_t *)pipe->bus_obj;

    LOG_DBG("mpipe[%d] is deleted\n", pipe->index);
    pthread_mutex_destroy(&pipe->pipe_lock);
    bus_obj->mpipes[pipe->index].in_use = false;
}

// create and delete slave-side pipe
ubus_spipe * ubus_slave_pipe_new(ubus bus, packet_sig_t request_sig, packet_sig_t reply_sig)
{
    ubus_bus_t *bus_obj = (ubus_bus_t *)bus;
    int i;

    for(i=0; i<MAX_SPIPES; ++i) {
        if(!bus_obj->spipes[i].in_use) {
            // allocate a spipe slot and do initialization
            LOG_DBG("allocated spipes[%d]\n", i);
            memset(&bus_obj->spipes[i], 0, sizeof(bus_obj->_obj->spipes[pipe->index]pipes[i]));
            bus_obj->spipes[i].in_use = true;
            bus_obj->spipes[i].pipe.request_sig = request_sig;
            bus_obj->spipes[i].pipe.reply_sig = reply_sig;
            bus_obj->spipes[i].pipe.bus = bus_obj;
            bus_obj->spipes[i].pipe.index = i;
            pthread_mutex_init(&bus_obj->spipes[i].pipe.pipe_lock, NULL);

            return (ubus_spipe *)&bus_obj->spipes[i].pipe;
        }
    }

    LOG_ERR("no more available slave pipes\n");
    return -1;
}

void ubus_slave_pipe_del(ubus_spipe pipe)
{
    ubus_spipe_t *pipe = (ubus_spipe_t *)p;
    ubus_bus_t *bus_obj = (ubus_bus_t *)pipe->bus_obj;

    LOG_DBG("spipe[%d] is deleted\n", pipe->index);
    pthread_mutex_destroy(&pipe->pipe_lock);
    bus_obj->spipes[pipe->index].in_use = false;
}

// Master side: send request and wait for reply
int ubus_master_send_recv(ubus_mpipe p, const ubus_request_t *request, ubus_reply_t *reply)
{
    ubus_mpipe_t *pipe = (ubus_mpipe_t *)p;
    ubus_bus_t *bus_obj = (ubus_bus_t *)pipe->bus_obj;
    int max_try = 10;
    int result = 0;

    //----------------------------------
    // begin critical section
    pthread_mutex_lock(&pipe->pipe_lock);

    ubus_generate_request(&pipe->request, request);

    while(1) {

        struct timeval tv;
        struct timespec ts;

        // limit the number of retry
        if(max_try-- <0) {
            LOG_ERR("abort re-try waiting for reply\n");
            result = -1;
            break;
        }

        gettimeofday(&tv, NULL);
        ts.tv_sec = tv.tv_sec + 5; // 5-second timeout
        ts.tv_nsec = 0;

        // send request and wait for reply
        pthread_cond_init(&pipe->reply_cond, NULL);
        pipe->reply_received = false;
        ubus_send_request(bus_obj, &pipe->request);
        pthread_cond_timedwait(&pipe->reply_cond, &pipe->pipe_lock, &ts);

        if(pipe->reply_received) {
            // got reply
            ubus_generate_reply(&pipe->reply, reply);
            result = 0;
            break;
        }
        else {
            LOG_ERR("timeout waiting for reply\n");
            continue; //keep trying
        }
    } // end of while loop

    pthread_mutex_unlock(&pipe->pipe_lock);
    // exit critical section
    //----------------------------------
    return result;
}

// Slave side: receive request, process request and send reply
int ubus_slave_recv(ubus_spipe p, ubus_request_t *request)
{

}

int ubus_slave_send(ubus_spipe p, const ubus_reply_t *reply)
{

}



