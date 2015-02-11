#ifndef __UBUS_H__
#define __UBUS_H__

// ubus is designed to exchange information between 2 CPU or 2 hosts,
// each connected on the same UART(RS232) connection. 
//
// Actual use case: C285 DM368<-->NUC100, C285 DM368 <--> factory test program.
//
// The data are not limited in size. The actual transfer is like IP datagram,
// which splits data into fix-sized packets.
//
// Error detection/handling is done by CRC checksum. On data corruption, the sender
// is expected to re-send the request (with different sequence number) when timeout
// occurs waiting for reply.
//
// This error handling is expected to withstand line noise, such as other debug 
// messages flowing on the UART bus, which is typical case in embedded SoC.
//
// It is possible where each end has more than one logical component 
// sending/receiving packets. This scenario supports multiple logical bus 
// (we call "pipes")using the same UART connection. To acheive this, 
// each logical components must identify the other using sender_id and reciever_id.
//
// The sender/receiver signature (id) also serves as packet delimiter so we can detect
// packet boundaries much easier.
//
//
// Deliverables:
// 1. test code: sender and receiver
// 2. documentation and diagrams
// 3. library itself
//
// TODO:
// 1. packetization to support large(>17bytes) data transfer
//

#include <stdint.h>

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

typedef struct ubus_bus_t * ubus; 
typedef struct ubus_mpipe_t * ubus_mpipe; // master pipe
typedef struct ubus_spipe_t * ubus_spipe; // slave pipe

typedef struct _ubus_cmd_t
{
    uint8_t     command;
    uint8_t     data[17];
    int         data_length;
} ubus_request_t;

typedef enum {
    UBUS_STATE_OK = 0,
    UBUS_STATE_UNKNOWN_COMMAND,
    UBUS_STATE_CRC_ERROR,
    UBUS_STATE_BUSY,
} ubus_reply_state;

typedef struct _ubus_reply_t
{
    uint8_t     command;
    ubus_reply_state state;
    uint8_t     data[16];
    int         data_length;
} ubus_reply_t;

typedef struct _packet_signature_t
{
    uint8_t     start_byte_0;
    uint8_t     start_byte_1;
    uint8_t     stop_byte;
} packet_sig_t;

int ubus_bus_init(ubus *pbus, char *uart_device, int baudrate);
void ubus_bus_exit(ubus bus);

// create and delete master-side pipe
ubus_mpipe ubus_master_pipe_new(ubus bus, packet_sig_t request_sig, packet_sig_t reply_sig);
void ubus_master_pipe_del(ubus_mpipe pipe);

// create and delete slave-side pipe
ubus_spipe ubus_slave_pipe_new(ubus bus, packet_sig_t request_sig, packet_sig_t reply_sig);
void ubus_slave_pipe_del(ubus_spipe pipe);

// Master side: send request and wait for reply
int ubus_master_send_recv(ubus_mpipe pipe, const ubus_request_t *request, ubus_reply_t *reply);

// Slave side: receive request, process request and send reply
int ubus_slave_recv(ubus_spipe pipe, ubus_request_t *request, int timeout_sec);
int ubus_slave_send(ubus_spipe pipe, const ubus_reply_t *reply);

typedef enum {
    UBUS_TEST_MISSING_BYTES = 0,
    UBUS_TEST_PACKET_LOSS,
    UBUS_TEST_CORRUPTION,
    UBUS_TEST_GARBAGE,
    UBUS_TEST_ID_MAX,
} ubus_test_id;

int ubus_test_enable(ubus bus, ubus_test_id test_id);
int ubus_test_disable(ubus bus, ubus_test_id test_id);
int ubus_test_set_fail_rate(ubus bus, int fail_percentage);
int ubus_test_report(ubus bus);

// legacy master/slave id used in C285 project.
// DO NOT re-use in non-related projects unless for compatibility.

extern packet_sig_t DM368_2_FAC_REQUEST_SIG;
extern packet_sig_t DM368_2_FAC_REPLY_SIG;

extern packet_sig_t FAC_2_DM368_REQUEST_SIG;
extern packet_sig_t FAC_2_DM368_REPLY_SIG;

extern packet_sig_t DM368_2_NUC100_REQUEST_SIG;
extern packet_sig_t DM368_2_NUC100_REPLY_SIG;

extern packet_sig_t NUC100_2_DM368_REQUEST_SIG;
extern packet_sig_t NUC100_2_DM368_REPLY_SIG;

//C++ guard
#ifdef __cplusplus
};
#endif

#endif //__UBUS_H__
