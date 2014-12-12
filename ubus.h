#ifndef __UBUS_H__
#define __UBUS_H__

// ubus is designed to exchange information between 2 CPU or 2 hosts,
// each connected on the same UART(RS232) connection. 
//
// The data are not limited in size. The actual transfer is like IP datagram,
// which all data are split into fix-sized packets.
//
// Error detection/handling is done by CRC checksum. On data corruption, the sender
// is expected to re-send another request when timeout occurs waiting for reply.
//
// This error handling is expected to withstand line noise, such as other debug 
// messages flowing on the UART bus, which is typical case in embedded SoC.
//
// It is possible where each end has more than one logical component 
// sending/receiving packets. This scenario supports multiple logical bus
// using the same UART connection. To acheive this, each logical components
// must identify the other using sender_id and reciever_id in the ubus_init().
//
//

#include <stdint.h>

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

#define MAX_CMD_PAYLOAD_LEN (17)
#define MAX_REPLY_PAYLOAD_LEN (16)

typedef struct _ubus_cmd_t
{
    uint8_t     command;
    uint8_t     *data;
    int         data_length;
} ubus_request_t;

typedef struct _ubus_reply_t
{
    uint8_t     command;
    uint8_t     state;
    uint8_t     *data;
    int         data_length;
} ubus_reply_t;

typedef void * ubus_t;

int ubus_init(ubus_t *pbus, uint16_t sender_id, uint16_t receiver_id, char *uart_device, int baudrate);
void ubus_exit(ubus_t bus);

// synchronous version of command-reply handling
int ubus_send_cmd(ubus_t bus, const ubus_request_t *request, ubus_reply_t *reply);

// asynchronous version of command-reply handling
int ubus_queue_cmd(ubus_t bus, const ubus_request_t *request);
int ubus_wait_reply(ubus_t bus, const ubus_request_t *request, ubus_reply_t *reply);

//C++ guard
#ifdef __cplusplus
};
#endif

#endif //__UBUS_H__
