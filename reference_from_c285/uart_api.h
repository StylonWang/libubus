#ifndef __UART_API_H__
#define __UART_API_H__

#include <fcntl.h>
#include <string.h>
#include <errno.h>

#define UART_S0	"/dev/ttyS0"
#define UART_S1	"/dev/ttyS1"
#define UART_S2	"/dev/ttyS2"

#define BAU_RATE_19200		19200
#define BAU_RATE_38400		38400
#define BAU_RATE_57600 	57600
#define BAU_RATE_115200	115200

//      command 
//DM368 -------> Factory
//      <-------
//       reply
#define FW_SEND_REQUEST_HEAD_BYTE_1  0x02
#define FW_SEND_REQUEST_HEAD_BYTE_2  0x61
#define FW_SEND_REQUEST_STOP_BYTE    0x06 
        
#define FW_GET_REPLY_HEAD_BYTE_1     0x03
#define FW_GET_REPLY_HEAD_BYTE_2     0x62 
#define FW_GET_REPLY_STOP_BYTE       0x07


//    command
//Factory -------> DM368
//    <-------
//     reply
#define FAC_SEND_REQUEST_HEAD_BYTE_1    0x04
#define FAC_SEND_REQUEST_HEAD_BYTE_2    0x63
#define FAC_SEND_REQUEST_STOP_BYTE      0x08

#define FAC_GET_REPLY_HEAD_BYTE_1       0x05
#define FAC_GET_REPLY_HEAD_BYTE_2       0x64
#define FAC_GET_REPLY_STOP_BYTE         0x09

//      command
//DM368 -------> MCU
//      <-------
//       reply
#define DM368_SEND_REQUEST_HEAD_BYTE_1	0x44
#define DM368_SEND_REQUEST_HEAD_BYTE_2	0x54
#define DM368_SEND_REQUEST_STOP_BYTE	0x4F

#define DM368_GET_REPLY_HEAD_BYTE_1	0x4D
#define DM368_GET_REPLY_HEAD_BYTE_2	0x59
#define DM368_GET_REPLY_STOP_BYTE	0x52


//    command
//MCU -------> DM368
//    <-------
//     reply
#define MCU_SEND_REQUEST_HEAD_BYTE_1	0x43
#define MCU_SEND_REQUEST_HEAD_BYTE_2	0x53
#define MCU_SEND_REQUEST_STOP_BYTE	0x50

#define MCU_GET_REPLY_HEAD_BYTE_1 	0x55
#define MCU_GET_REPLY_HEAD_BYTE_2 	0x4C
#define MCU_GET_REPLY_STOP_BYTE		0x45

typedef enum
{
	//don't use 0x00...or may cause some error because of "DIA_OWNER_ANY = 0"
	AVM_EVENT_DM368 = 0x01,
	AVM_EVENT_FAC,
	AVM_EVENT_MCU,
} _EVENT_PORT;

//Ivan 20121110 UART communication protocol stucture
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

#define UART_REQUEST_SIZE sizeof(UART_REQUEST)
#define UART_REPLY_SIZE sizeof(UART_REPLY)
#define UART_TRANSMIT_BYTE_MAX UART_REQUEST_SIZE

unsigned char rbuf[UART_REQUEST_SIZE];

typedef struct
{
	unsigned char length;
	unsigned char ucPara[16];
} COMMAND_PARAMETERS_CONTENT;

typedef struct
{
	unsigned char year;
	unsigned char month;
	unsigned char date;
	unsigned char hour;
	unsigned char minute;
	unsigned char sec;
	unsigned char reserved[2];
} COMMAND_TIME_CONTENT;

typedef struct
{
	unsigned char protocol;
	unsigned char data_code[4];
	unsigned char sepration_period;	//only if protocol is Dream-DVB need it
	unsigned char custom_code[4];		//only if protocol is Dream-DVB need it
	unsigned char reserved[2];
} COMMAND_IR_CONTENT;
//---------------------------------------------------------

enum
{
	NO_ERROR		=	0,
	UNKNOWN_CMD		=	1,
	DATA_ERROR		=	2,
	UART_TIMEOUT	=	3,
	
	REPLY_STATE_MAX
};

typedef enum __flag_type {
	HEADER_CHECK = 0x01,
	CONTENT_CHECK = 0x01 << 1
} _flag_type;

UART_REQUEST DM368_send_request_to_MCU;
UART_REQUEST MCU_send_request_to_DM368;
UART_REQUEST FAC_send_request_to_DM368;
UART_REQUEST DM368_send_request_to_FAC;
UART_REPLY DM368_reply_to_MCU;
UART_REPLY MCU_reply_to_DM368;
UART_REPLY DM368_reply_to_FAC;

#define REQUEST_CRC_LENGTH sizeof(UART_REQUEST)-(sizeof(DM368_send_request_to_MCU.CHECKSUM)+sizeof(DM368_send_request_to_MCU.STOP_BYTE))
#define REPLY_CRC_LENGTH sizeof(UART_REPLY)-(sizeof(MCU_reply_to_DM368.CHECKSUM)+sizeof(MCU_reply_to_DM368.STOP_BYTE))

//void init_uart (int fd, int baundrate);
//int monitor_MCU_reply(int, char *, unsigned int);
int monitor_MCU_reply(int file_fd, unsigned char *buffer_pool, unsigned int pool_size);
char Receive_Reply_From_MCU (int fd);
void UART_initial(int *f_id, char *uart_port, int bau_rate);
void init_uart(int fd, int baundrate);
void insert_data_to_uart_struct(_EVENT_PORT port, unsigned char cmd, unsigned char payload_len, unsigned char *payload_content);
void DM368_send_command(_EVENT_PORT port);

void DM368_send_command_time(_EVENT_PORT port, unsigned char cmd, COMMAND_TIME_CONTENT time_struct);
void DM368_send_command_IR(_EVENT_PORT port, unsigned char cmd, COMMAND_IR_CONTENT ir_struct);
void DM368_send_command_parameter(_EVENT_PORT port, unsigned char cmd, COMMAND_PARAMETERS_CONTENT para_struct);
void DM368_send_command_no_parameter(_EVENT_PORT port, unsigned char cmd);
void DM368_send_reply_no_parameter(_EVENT_PORT port, UART_REQUEST *request_event, unsigned char state);
void DM368_send_reply_parameter(_EVENT_PORT port, UART_REQUEST *request_event, unsigned char state, COMMAND_PARAMETERS_CONTENT para_struct);

//special testing block----------------------------------------------------
void auto_test_send_event_to_DM368(unsigned char cmd, COMMAND_PARAMETERS_CONTENT para_struct, int need_crc);

int get_command_state(UART_REPLY *command);
unsigned char get_uart_sequence_id(void);
void plus_uart_sequence_id(void);

#endif //__UART_API_H__
