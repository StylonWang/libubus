
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
#include <sys/time.h>

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
    pthread_cond_t request_cond;
    bool request_received;
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
        ubus_mpipe_t pipe; 
    } mpipes[MAX_MPIPES];

    struct {
        bool in_use;
        ubus_spipe_t pipe; 
    } spipes[MAX_SPIPES];
     
} ubus_bus_t;

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

static unsigned int crc32_tab[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

static unsigned int do_crc32(unsigned int crc, const void *buf, int size)
{
	const unsigned char *p;

	p = buf;
	crc = crc ^ ~0U;

	while (size--)
	{
		crc = crc32_tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
	}

	return crc ^ ~0U;
}

static int ubus_check_reply_crc(const UART_REPLY *reply)
{
    unsigned int crc32 = do_crc32(0, reply, sizeof(*reply)-sizeof(reply->CHECKSUM)-sizeof(reply->STOP_BYTE));

    if(crc32 != reply->CHECKSUM) {
        return -1;
    }

    return 0;
}

static int ubus_check_request_crc(const UART_REQUEST *request)
{
    unsigned int crc32 = do_crc32(0, request, sizeof(*request)-sizeof(request->CHECKSUM)-sizeof(request->STOP_BYTE));

    if(crc32 != request->CHECKSUM) {
        return -1;
    }

    return 0;
}

static int ubus_mpipe_recv_reply(ubus_mpipe_t *pipe, const UART_REPLY *reply)
{
    LOG_DBG("Reply, sig=0x%x:0x%x:0x%x cmd=0x%x state=0x%x len=%d\n",
            reply->RPL[0], reply->RPL[1], reply->STOP_BYTE,
            reply->COMMAND, reply->STATE, reply->LENGTH);

    if(ubus_check_reply_crc(reply)<0) {
        LOG_ERR("reply crc error\n");
        return -1;
    }

    pthread_mutex_lock(&pipe->pipe_lock);

    if(reply->COMMAND == pipe->request.COMMAND) {
        memcpy(&pipe->reply, reply, sizeof(*reply));
        pipe->reply_received = true;
        pthread_cond_signal(&pipe->reply_cond);
    }

    pthread_mutex_unlock(&pipe->pipe_lock);

    return 0;
}

static int ubus_spipe_recv_request(ubus_spipe_t *pipe, const UART_REQUEST *request)
{
    LOG_DBG("Request, sig=0x%x:0x%x:0x%x cmd=0x%x len=%d\n",
            request->REQ[0], request->REQ[1], request->STOP_BYTE,
            request->COMMAND, request->LENGTH);

    if(ubus_check_request_crc(request)<0) {
        LOG_ERR("request crc error\n");
        // TODO: send back "CRC error" reply
        return -1;
    }

    pthread_mutex_lock(&pipe->pipe_lock);

    memcpy(&pipe->request, request, sizeof(*request));
    pipe->request_received = true;
    pthread_cond_signal(&pipe->request_cond);

    pthread_mutex_unlock(&pipe->pipe_lock);
    return 0;
}

// currently only handle one request/reply.
static int ubus_dispatch_data(ubus_bus_t *bus_obj, void *data, int data_len)
{
    int i;
    UART_REQUEST *request = (UART_REQUEST *)data;
    UART_REPLY *reply = (UART_REPLY *)data;

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
        read_bytes = read(bus_obj->fd, &u, sizeof(u));
        if(read_bytes != sizeof(u)) {
            LOG_ERR("invalid data received\n");
            continue;
        }

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

// translate from API request to RAW request
static int ubus_mpipe_generate_request(ubus_bus_t *bus_obj, ubus_mpipe_t *pipe,
        UART_REQUEST *raw, const ubus_request_t *request)
{
    unsigned int checksum;

    if(request->data_length > sizeof(raw->PAYLOAD)) {
        LOG_ERR("request length %d too long\n", request->data_length);
        return -1;
    }

    raw->REQ[0] = pipe->request_sig.start_byte_0;
    raw->REQ[1] = pipe->request_sig.start_byte_1;
    raw->STOP_BYTE = pipe->request_sig.stop_byte;
    raw->COMMAND = request->command;

    pthread_mutex_lock(&bus_obj->queue_lock);
    raw->SEQUENCE = (bus_obj->sequence++)%0xFF;
    pthread_mutex_unlock(&bus_obj->queue_lock);

    raw->FRAGMENT = 0x00; // no fragment
    raw->LENGTH = request->data_length;
    memset(raw->PAYLOAD, 0, sizeof(raw->PAYLOAD));
    memcpy(raw->PAYLOAD, request->data, request->data_length);

    checksum = do_crc32(0, raw->PAYLOAD, raw->LENGTH);
    raw->CHECKSUM = checksum;

    return 0;
}

// translate from RAW reply into API reply
static int ubus_mpipe_generate_reply(ubus_bus_t *bus_obj, ubus_mpipe_t *pipe,
           ubus_reply_t *reply, const UART_REPLY *raw)
{
    reply->command = raw->COMMAND;
    reply->state = raw->STATE;
    reply->data_length = raw->LENGTH;
    memset(reply->data, 0, sizeof(reply->data));
    memcpy(reply->data, raw->PAYLOAD, raw->LENGTH);

    return 0;
}

// translate from RAW request into API request
static int ubus_spipe_generate_request(ubus_bus_t *bus_obj, ubus_spipe_t *pipe, 
         ubus_request_t *request, const UART_REQUEST *raw)
{
    memcpy((void *)request->data, raw->PAYLOAD, raw->LENGTH);
    request->data_length = raw->LENGTH;
    request->command = raw->COMMAND;
    return 0;
}

// translate from API reply to RAW reply
static int ubus_spipe_generate_reply(ubus_bus_t *bus_obj, ubus_spipe_t *pipe,
           UART_REPLY *raw, const ubus_reply_t *reply)
{
    unsigned int checksum;

    if(reply->data_length > sizeof(raw->PAYLOAD)) {
        LOG_ERR("reply length %d too long\n", reply->data_length);
        return -1;
    }

    raw->RPL[0] = pipe->reply_sig.start_byte_0;
    raw->RPL[1] = pipe->reply_sig.start_byte_1;
    raw->STOP_BYTE = pipe->reply_sig.stop_byte;
    raw->COMMAND = reply->command;
    raw->STATE = reply->state;

    pthread_mutex_lock(&bus_obj->queue_lock);
    raw->SEQUENCE = (bus_obj->sequence++)%0xFF;
    pthread_mutex_unlock(&bus_obj->queue_lock);

    raw->FRAGMENT = 0x00; // no fragment
    raw->LENGTH = reply->data_length;
    memset(raw->PAYLOAD, 0, sizeof(raw->PAYLOAD));
    memcpy(raw->PAYLOAD, reply->data, reply->data_length);

    checksum = do_crc32(0, raw->PAYLOAD, raw->LENGTH);
    raw->CHECKSUM = checksum;

    return 0;
}

static int ubus_send_data(ubus_bus_t *bus_obj, const unsigned char *buf, int length)
{
    int i=0, j=length;
    int result = 0;

    pthread_mutex_lock(&bus_obj->fd_lock);

    while(j) {
        ssize_t s;
        s = write(bus_obj->fd, buf+i, j);
        if(s<0) {
            LOG_ERR("unable to send data: %s\n", strerror(errno));
            result = -1;
            break;
        }

        i+=s;
        j-=s;
    }

    pthread_mutex_unlock(&bus_obj->fd_lock);
    return result;
}

static int ubus_send_request(ubus_bus_t *bus_obj, const UART_REQUEST *request)
{
    return ubus_send_data(bus_obj, (const unsigned char *)request, sizeof(*request));
}

static int ubus_send_reply(ubus_bus_t *bus_obj, const UART_REPLY *reply)
{
    return ubus_send_data(bus_obj, (const unsigned char *)reply, sizeof(*reply));
}

// End of internal routines
//--------------------------------------------------------------
//

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
ubus_mpipe ubus_master_pipe_new(ubus bus, packet_sig_t request_sig, packet_sig_t reply_sig)
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
            bus_obj->mpipes[i].pipe.bus_obj = bus_obj;
            bus_obj->mpipes[i].pipe.index = i;
            pthread_mutex_init(&bus_obj->mpipes[i].pipe.pipe_lock, NULL);

            return (ubus_mpipe )&bus_obj->mpipes[i].pipe;
        }
    }

    LOG_ERR("no more available master pipes\n");
    return NULL;
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
ubus_spipe ubus_slave_pipe_new(ubus bus, packet_sig_t request_sig, packet_sig_t reply_sig)
{
    ubus_bus_t *bus_obj = (ubus_bus_t *)bus;
    int i;

    for(i=0; i<MAX_SPIPES; ++i) {
        if(!bus_obj->spipes[i].in_use) {
            // allocate a spipe slot and do initialization
            LOG_DBG("allocated spipes[%d]\n", i);
            memset(&bus_obj->spipes[i], 0, sizeof(bus_obj->spipes[i]));
            bus_obj->spipes[i].in_use = true;
            bus_obj->spipes[i].pipe.request_sig = request_sig;
            bus_obj->spipes[i].pipe.reply_sig = reply_sig;
            bus_obj->spipes[i].pipe.bus_obj = bus_obj;
            bus_obj->spipes[i].pipe.index = i;
            pthread_mutex_init(&bus_obj->spipes[i].pipe.pipe_lock, NULL);
            pthread_cond_init(&bus_obj->spipes[i].pipe.request_cond, NULL);

            return (ubus_spipe)&bus_obj->spipes[i].pipe;
        }
    }

    LOG_ERR("no more available slave pipes\n");
    return NULL;
}

void ubus_slave_pipe_del(ubus_spipe p)
{
    ubus_spipe_t *pipe = (ubus_spipe_t *)p;
    ubus_bus_t *bus_obj = (ubus_bus_t *)pipe->bus_obj;

    LOG_DBG("spipe[%d] is deleted\n", pipe->index);
    pthread_cond_destroy(&pipe->request_cond);
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

    ubus_mpipe_generate_request(bus_obj, pipe, &pipe->request, request);

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
            ubus_mpipe_generate_reply(bus_obj, pipe, reply, &pipe->reply);
            if(UBUS_STATE_CRC_ERROR==reply->state) {
                LOG_ERR("got reply crc error, retry\n");
                continue;
            }
            else if(UBUS_STATE_BUSY==reply->state) {
                LOG_ERR("got reply busy, retry\n");
                continue;
            }

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
int ubus_slave_recv(ubus_spipe p, ubus_request_t *request, int timeout_sec)
{
    ubus_spipe_t *pipe = (ubus_spipe_t *)p;
    ubus_bus_t *bus_obj = (ubus_bus_t *)pipe->bus_obj;
    int result = 0;
    struct timeval tv;
    struct timespec ts;

    //----------------------------------
    // begin critical section
    pthread_mutex_lock(&pipe->pipe_lock);

    // already have a request standing by
    if(pipe->request_received) {
        ubus_spipe_generate_request(bus_obj, pipe, request, &pipe->request);
        result = 0;
        goto done;
    }

    // no request for now, wait for it.
    gettimeofday(&tv, NULL);
    ts.tv_sec = tv.tv_sec + timeout_sec; // use timeout
    ts.tv_nsec = 0;
    pthread_cond_timedwait(&pipe->request_cond, &pipe->pipe_lock, &ts);
    if(pipe->request_received) {
        ubus_spipe_generate_request(bus_obj, pipe, request, &pipe->request);
        result = 0;
    }
    else {
        result = -1;
        LOG_DBG("no request for now\n");
    }

    pthread_mutex_unlock(&pipe->pipe_lock);
    // exit critical section
    //----------------------------------
    
done:
    return result;
}

int ubus_slave_send(ubus_spipe p, const ubus_reply_t *reply)
{
    ubus_spipe_t *pipe = (ubus_spipe_t *)p;
    ubus_bus_t *bus_obj = (ubus_bus_t *)pipe->bus_obj;
    int result = 0;

    pthread_mutex_lock(&pipe->pipe_lock);

    ubus_spipe_generate_reply(bus_obj, pipe, &pipe->reply, reply);
    ubus_send_reply(bus_obj, &pipe->reply);

    pthread_mutex_unlock(&pipe->pipe_lock);

    return result;
}


