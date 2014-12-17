#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include "uart_api.h"
#include "avm_crc32.h"
#include <pthread.h>
//static struct termios newtio;
unsigned char rbuf[UART_REQUEST_SIZE] = { 0 };
unsigned char rbuf_fac[UART_REQUEST_SIZE] = { 0 };
int fd_uart;
int fd_uart_fac;
static struct termios newtio;
unsigned char uart_sequence_id = 0;

pthread_mutex_t write_mutex;

void init_uart(int fd, int baundrate)
{
	tcflush(fd, TCIOFLUSH);
	if (tcgetattr(fd, &newtio) < 0) {
		printf("error1 \n");
		close(fd);
	}
	cfmakeraw(&newtio);
	switch(baundrate) {
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
		printf("error2 \n");
		close(fd);
	}
	tcflush(fd, TCIOFLUSH);
}

/*
return 1 if it need append new event into queue
otherwise, just call this function again.
*/
int monitor_MCU_reply(int file_fd, unsigned char *buffer_pool, unsigned int pool_size)
{
	int ret = 0;
	fd_set readfs;
	struct timeval Timeout;
	ssize_t number_of_data = 0;
	unsigned int data_length = 2, buffer_index = 0;
	unsigned char check_flag = 0x00;

	bzero(rbuf, pool_size);

	while (1) {
		Timeout.tv_usec = 0L;
		Timeout.tv_sec = 50L;
		FD_ZERO(&readfs);
		FD_SET(file_fd, &readfs);

		ret = select(file_fd + 1, &readfs, NULL, NULL, NULL/*&Timeout*/);
		if (ret == -1) {
			// You have to check errno variable and handl it, if you want do best.
			printf("[%s, %s, %d]Call select system call fail!\n", __FILE__, __func__, __LINE__);
			return -1;
		}
		else if (ret == 0) {
			printf("[%s, %s, %d]Timeout!\n", __FILE__, __func__, __LINE__);
			return -1;
		}

		number_of_data = 0;
		number_of_data = read(file_fd, &buffer_pool[buffer_index], data_length);
		if (number_of_data == -1) {
			printf("[%s, %s@%d]Read /dev/sttyS1 error\n", __FILE__, __func__, __LINE__);
			switch (errno) {
			case EAGAIN:
				printf("[%s, %s@%d]EAGIN or EWOULDBLOCK\n", __FILE__, __func__, __LINE__);
				break;
			case EBADF:
				printf("[%s, %s@%d]EBADF\n", __FILE__, __func__, __LINE__);
				break;
			case EFAULT:
				printf("[%s, %s@%d]EFAULT\n", __FILE__, __func__, __LINE__);
				break;
			case EINTR:
				printf("[%s, %s@%d]EINTR\n", __FILE__, __func__, __LINE__);
				break;
			case EINVAL:
				printf("[%s, %s@%d]EINVAL\n", __FILE__, __func__, __LINE__);
				break;
			case EIO:
				printf("[%s, %s@%d]EIO\n", __FILE__, __func__, __LINE__);
				break;
			case EISDIR:
				printf("[%s, %s@%d]EISDIR\n", __FILE__, __func__, __LINE__);
				break;
			}
			return -1;
		} // end of if

/*		do {
			// just for debug
			int i;
			for (i = buffer_index; i < (number_of_data + buffer_index); i++) {
				printf("p[%d]=0x%02x ", i, buffer_pool[i]);
			}
			printf("\n");
		} while (0);
*/
		if (data_length -= number_of_data) {
			buffer_index += number_of_data;
			continue;
		}

		if (!(check_flag & HEADER_CHECK)) {
			if ((buffer_pool[0] == DM368_GET_REPLY_HEAD_BYTE_1
				&& buffer_pool[1] == DM368_GET_REPLY_HEAD_BYTE_2)
				|| (buffer_pool[0] == MCU_SEND_REQUEST_HEAD_BYTE_1
				&& buffer_pool[1] == MCU_SEND_REQUEST_HEAD_BYTE_2)
				|| (buffer_pool[0] == MCU_GET_REPLY_HEAD_BYTE_1
				&& buffer_pool[1] == MCU_GET_REPLY_HEAD_BYTE_2)
				|| (buffer_pool[0] == FAC_SEND_REQUEST_HEAD_BYTE_1
				&& buffer_pool[1] == FAC_SEND_REQUEST_HEAD_BYTE_2)
				|| (buffer_pool[0] == FW_GET_REPLY_HEAD_BYTE_1
				&& buffer_pool[1] == FW_GET_REPLY_HEAD_BYTE_2)) {

				check_flag |= HEADER_CHECK;
				data_length = pool_size - 2;
				buffer_index = 2;
			}
			else if (buffer_pool[1] == DM368_GET_REPLY_HEAD_BYTE_1
				|| buffer_pool[1] == MCU_SEND_REQUEST_HEAD_BYTE_1
				|| buffer_pool[1] == MCU_GET_REPLY_HEAD_BYTE_1
				|| buffer_pool[1] == FAC_SEND_REQUEST_HEAD_BYTE_1
				|| buffer_pool[1] == FW_GET_REPLY_HEAD_BYTE_1) {

				buffer_pool[0] = buffer_pool[1];
				buffer_index = 1;
				data_length = 1;
			}
			else {
				buffer_index = 0;
				data_length = 2;
				check_flag = 0x00;
			} // end of second if
		} 
		else {
			if ((buffer_pool[UART_REPLY_SIZE - 1] == DM368_GET_REPLY_STOP_BYTE
				&& buffer_pool[0] == DM368_GET_REPLY_HEAD_BYTE_1
				&& buffer_pool[1] == DM368_GET_REPLY_HEAD_BYTE_2)
				|| (buffer_pool[UART_REQUEST_SIZE - 1] == MCU_SEND_REQUEST_STOP_BYTE
				&& buffer_pool[0] == MCU_SEND_REQUEST_HEAD_BYTE_1
				&& buffer_pool[1] == MCU_SEND_REQUEST_HEAD_BYTE_2)
				|| (buffer_pool[UART_REQUEST_SIZE - 1] == MCU_GET_REPLY_STOP_BYTE
				&& buffer_pool[0] == MCU_GET_REPLY_HEAD_BYTE_1
				&& buffer_pool[1] == MCU_GET_REPLY_HEAD_BYTE_2)
				|| (buffer_pool[UART_REQUEST_SIZE - 1] == FAC_SEND_REQUEST_STOP_BYTE
				&& buffer_pool[0] == FAC_SEND_REQUEST_HEAD_BYTE_1
				&& buffer_pool[1] == FAC_SEND_REQUEST_HEAD_BYTE_2)
			
				|| (buffer_pool[UART_REQUEST_SIZE - 1] == FW_GET_REPLY_STOP_BYTE
				&& buffer_pool[0] == FW_GET_REPLY_HEAD_BYTE_1
				&& buffer_pool[1] == FW_GET_REPLY_HEAD_BYTE_2)) {
				// add into event queue
				return 1;
			} // end of second if
			else {
				printf("[%s, %s@%d]Not command\n", __FILE__, __func__, __LINE__);
				return -1;
			} // end of second if
		} // end of first if
	} // end of while
} // end of monitor_MCU_reply

char Receive_Reply_From_MCU (int fd)
{
	fd_set readfs;
	struct timeval Timeout;
	int ret = 1, i = 0;//, count, len = 24, read_timeout = 0;

//	unsigned int cnt_c = 0U, cnt_k = 0;
	ssize_t number_of_data = 0;
	unsigned int data_length = 2, buffer_index = 0;
	unsigned char check_flag = 0x00;
	unsigned int pool_size = UART_TRANSMIT_BYTE_MAX;


	memset(rbuf, '0', sizeof(rbuf));
	FD_ZERO(&readfs);
	FD_SET(fd, &readfs);
	Timeout.tv_usec = 0L;
	Timeout.tv_sec = 5L;

	ret = select (fd + 1, &readfs, NULL, NULL, &Timeout);

	switch (ret) {
		case 0:		// timeout
			fprintf(stderr, "Waiting for a while that is timeout.\n");
			break;

		case -1:	// error
			// You have to check errno variable and handl it, if you want do best.
			fprintf(stderr, "Call select system call fail!\n");
			break;

		default:
		{
#if 0
			ret = 0;
			while((count = read (fd, &rbuf[i], len)) < len)
			{
				i += count;
				ret += count;
				len -= count;
				printf(">>>>>>>>>>>>>>only read %d bytes\n", count);
				if(count == 0)
					read_timeout ++;
				if(read_timeout > 2)
					break;
			}

			if(read_timeout <= 2)
				ret += count;
#else
	/*
	//		printf("rbuf=");
			for (i = 0; i < UART_TRANSMIT_BYTE_MAX; ) {
				count = read(fd, &rbuf[i], 1);
				if (count == 0 && i < UART_TRANSMIT_BYTE_MAX)
					continue;
				i++;
			}
	*/
			while(1) {
				number_of_data = read(fd, &rbuf[buffer_index], data_length);
				printf("read count is :%d\n", number_of_data);

				for (i = 0; i < UART_TRANSMIT_BYTE_MAX; i++) {
					printf("p[%d]=0x%X ", i, rbuf[i]);
				}
				printf("\n");

				if (number_of_data == -1) {
					printf("[%s, %s, %d]read error\n", __FILE__, __func__, __LINE__);
					return -1;
				}

				if (data_length -= number_of_data) {
					continue;
				}

				if (!(check_flag & HEADER_CHECK)) {
					if ((rbuf[0] == DM368_GET_REPLY_HEAD_BYTE_1
						&& rbuf[1] == DM368_GET_REPLY_HEAD_BYTE_2)
						|| (rbuf[0] == MCU_SEND_REQUEST_HEAD_BYTE_1
						&& rbuf[1] == MCU_SEND_REQUEST_HEAD_BYTE_2)) {

						check_flag |= HEADER_CHECK;
						data_length = pool_size - 2;
						buffer_index = 2;
					}
					else if (rbuf[1] == DM368_GET_REPLY_HEAD_BYTE_1
						|| rbuf[1] == MCU_SEND_REQUEST_HEAD_BYTE_1) {

						rbuf[0] = rbuf[1];
						buffer_index = 1;
						data_length = 1;
					}
					else {
						buffer_index = 0;
						data_length = 2;
					} // end of second if
				} 
				else {
					if ((rbuf[23] == DM368_GET_REPLY_STOP_BYTE
						&& rbuf[0] == DM368_GET_REPLY_HEAD_BYTE_1
						&& rbuf[1] == DM368_GET_REPLY_HEAD_BYTE_2)
						|| (rbuf[23] == MCU_SEND_REQUEST_STOP_BYTE
						&& rbuf[0] == MCU_SEND_REQUEST_HEAD_BYTE_1
						&& rbuf[1] == MCU_SEND_REQUEST_HEAD_BYTE_2)) {
						// add into event queue
						printf("Got message and put into event queue\n");
						ret = 24;
						break;
					} // end of second if

					//bzero(rbuf, pool_size);
					memset(rbuf, '0', sizeof(rbuf));
					buffer_index = 0;
					data_length = 2;
					check_flag = 0x00;
				} // end of first if
			}

			for (i = 0; i < UART_TRANSMIT_BYTE_MAX; i++) {
				printf("p[%d]=0x%X ", i, rbuf[i]);
			}
			printf("\n");
	//		for (i = 0; i < UART_TRANSMIT_BYTE_MAX; i++) {
	//			printf("p[%d]=%c ", i, rbuf[i]);
	//		}
	//		printf("\n");
			ret = UART_TRANSMIT_BYTE_MAX;
#endif
		} // end of default switch 
	} // end of switch

	if (ret != 24)
	{
		printf ("read error! %d\n", ret);
		return -2;
	}
	else
	{
		//memcpy(&MCU_reply_to_DM368, (UART_REPLY*)&rbuf, UART_REPLY_SIZE);
	//	printf("[receive]: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
	//	for(i = 0 ; i < 24 ; i ++)
	//		printf("%c", rbuf[i]);
	//	printf("\n");
	}
	return 1;
}

void UART_initial(int *f_id, char *uart_port, int bau_rate)
{
	//Initiate UART----------------------------------------
	//fd_uart = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
	*f_id = open(uart_port, O_RDWR | O_NOCTTY | O_NDELAY);

	if (*f_id < 0) {
		printf("[%s:%d] open %s error \n", __FUNCTION__, __LINE__, uart_port);
		fflush(stdout);
	}
	fcntl(*f_id, F_SETFL, 0);

	init_uart(*f_id, bau_rate);
	printf("initial uart done\n");
	//Initiate UART done-----------------------------------
}

void dm368_request_command_reset(_EVENT_PORT port, UART_REQUEST *buff, unsigned char cmd)
{
	if( uart_sequence_id++ == 0xff )
		uart_sequence_id = 0;

	if(port == AVM_EVENT_MCU)
	{
		buff->REQ[0] = DM368_SEND_REQUEST_HEAD_BYTE_1;
		buff->REQ[1] = DM368_SEND_REQUEST_HEAD_BYTE_2;
		buff->STOP_BYTE = DM368_SEND_REQUEST_STOP_BYTE;
		
			printf("\n\n>>>>>>>> \033[31mcommand request [NO. 0x%02x]\033[m <<<<<<<<<\n\n", cmd);
	}
	else if(port == AVM_EVENT_DM368)
	{
		//no this rule
		return;
	}
	else //port == AVM_EVENT_FAC
	{
		buff->REQ[0] = FW_SEND_REQUEST_HEAD_BYTE_1;
		buff->REQ[1] = FW_SEND_REQUEST_HEAD_BYTE_2;
		buff->STOP_BYTE = FW_SEND_REQUEST_STOP_BYTE;
		
			printf("\n\n>>>>>>>> \033[31mfactory command request [NO. 0x%02x]\033[m <<<<<<<<<\n\n", cmd);
	}
	
	buff->COMMAND = cmd;
	buff->SEQUENCE = uart_sequence_id;
	buff->FRAGMENT = 0x00;
	buff->LENGTH = 0;
	buff->CHECKSUM = 0;
}

void insert_data_to_uart_struct(_EVENT_PORT port, unsigned char cmd, unsigned char payload_len, unsigned char *payload_content)
{
	UART_REQUEST *buff;

	if(port == AVM_EVENT_MCU)
	{
		buff = &DM368_send_request_to_MCU;
	}
	else if(port == AVM_EVENT_DM368)
	{
		//no this rule
		return;
	}
	else //port == AVM_EVENT_FAC
	{
		buff = &DM368_send_request_to_FAC;
	}
	
	dm368_request_command_reset(port, buff, cmd);
	
	buff->LENGTH = payload_len;

	if( !((payload_len == 0) || (payload_content == NULL)) )
		memcpy(&(buff->PAYLOAD), payload_content, sizeof(buff->PAYLOAD));
		
	buff->CHECKSUM = avm_crc32(0, buff, REQUEST_CRC_LENGTH);
		
/*	printf("length = %d\n", payload_len);
	for(i = 0 ; i < payload_len ; i ++ )
		printf("\033[35m[%d] = 0x%x,", i, *(payload_content+i));
	printf("\033[m\n");*/
}

void insert_reply_data_to_uart_struct(_EVENT_PORT port, UART_REQUEST *request_event, unsigned char state, unsigned char payload_len, unsigned char *payload_content)
{
	UART_REPLY *buff;
	
	if(port == AVM_EVENT_MCU)
	{
		buff = &DM368_reply_to_MCU;
		memset(buff, 0, sizeof(UART_REPLY));

		buff->RPL[0] = MCU_GET_REPLY_HEAD_BYTE_1;
		buff->RPL[1] = MCU_GET_REPLY_HEAD_BYTE_2;
		buff->SEQUENCE = request_event->SEQUENCE;
		buff->STOP_BYTE = MCU_GET_REPLY_STOP_BYTE;
	}
	else if(port == AVM_EVENT_DM368)
	{
		buff = &MCU_reply_to_DM368;
		memset(buff, 0, sizeof(UART_REPLY));
		
		buff->RPL[0] = DM368_GET_REPLY_HEAD_BYTE_1;
		buff->RPL[1] = DM368_GET_REPLY_HEAD_BYTE_2;
		buff->SEQUENCE = request_event->SEQUENCE;
		buff->STOP_BYTE = DM368_GET_REPLY_STOP_BYTE;
	}
	else //port == AVM_EVENT_FAC
	{
		buff = &DM368_reply_to_FAC;
		memset(buff, 0, sizeof(UART_REPLY));
		
		buff->RPL[0] = FAC_GET_REPLY_HEAD_BYTE_1;
		buff->RPL[1] = FAC_GET_REPLY_HEAD_BYTE_2;
		buff->SEQUENCE = request_event->SEQUENCE;
		buff->STOP_BYTE = FAC_GET_REPLY_STOP_BYTE;
	}
	
//	printf("\n>>>>>>>> \033[32mreply to MCU [NO. 0x%02x]\033[m <<<<<<<<<\n\n", cmd);

	buff->STATE = state;
	buff->COMMAND = request_event->COMMAND;
	buff->FRAGMENT = 0x00;

	buff->LENGTH = payload_len;

	if( !((payload_len == 0) || (payload_content == NULL)) )
		memcpy(&(buff->PAYLOAD), payload_content, payload_len/*sizeof(&(buff->PAYLOAD))*/);

	buff->CHECKSUM = avm_crc32(0, buff, REPLY_CRC_LENGTH);
}

void DM368_send_command(_EVENT_PORT port)
{
	int i, res, fd_buff;
	UART_REQUEST request_buff;

pthread_mutex_lock(&write_mutex);
	
	if(port == 	AVM_EVENT_MCU)
	{
		fd_buff = fd_uart;
		request_buff = DM368_send_request_to_MCU;
	}
	else if(port == AVM_EVENT_DM368)
	{
		//no this rule;
		//it should be send from MCU
		bzero(&request_buff, UART_REPLY_SIZE);
	}
	else //AVM_EVENT_FAC
	{
		fd_buff = fd_uart_fac;
		request_buff = DM368_send_request_to_FAC;
	}
	
	for(i = 0 ; i < UART_REQUEST_SIZE ; i ++)
	{
		res = write (fd_uart, (unsigned char *)&request_buff + i, 1);
		if (res != 1)
			printf ("write error %d\n", i);
		usleep(3000);
	}

pthread_mutex_unlock(&write_mutex);
}

void DM368_send_reply(_EVENT_PORT port)
{
	int i, res, fd_buff;
	UART_REPLY reply_buff;

pthread_mutex_lock(&write_mutex);
	
	if(port == 	AVM_EVENT_MCU)
	{
		fd_buff = fd_uart;
		reply_buff = DM368_reply_to_MCU;
	}
	else if(port == AVM_EVENT_DM368)
	{
		fd_buff = fd_uart;
		reply_buff = MCU_reply_to_DM368;
	}
	else //AVM_EVENT_FAC
	{
		fd_buff = fd_uart_fac;
		reply_buff = DM368_reply_to_FAC;
	}

	for(i = 0 ; i < UART_REPLY_SIZE ; i ++)
	{
		res = write (fd_buff, (unsigned char *)&reply_buff + i, 1);
		if (res != 1)
			printf ("write error %d\n", i);
	//	usleep(3000);
	}

pthread_mutex_unlock(&write_mutex);
}

unsigned char get_uart_sequence_id(void)
{
	return uart_sequence_id;
}

void plus_uart_sequence_id(void)
{
	uart_sequence_id++;
}

int get_command_state(UART_REPLY *command)
{
	if( command->STATE == NO_ERROR )
	{
		return NO_ERROR;
	}
	else if( command->STATE == UNKNOWN_CMD )
	{
		printf("unknown command\n");
		return UNKNOWN_CMD;
	}
	else if( command->STATE == DATA_ERROR )
	{
		printf("data error\n");
		return DATA_ERROR;
	}
	else if( command->STATE == UART_TIMEOUT )
	{
		printf("UART timeout\n");
		return UART_TIMEOUT;
	}
	else
	{
		return DATA_ERROR;
	}
}

//==============================================================================
void DM368_send_command_time(_EVENT_PORT port, unsigned char cmd, COMMAND_TIME_CONTENT time_struct)
{
	unsigned char insert_content[6] = {0};

	memcpy(insert_content, (unsigned char*)&time_struct, sizeof(insert_content));

	insert_data_to_uart_struct(port, cmd, 6, insert_content);

	DM368_send_command(port);
}

void DM368_send_command_IR(_EVENT_PORT port, unsigned char cmd, COMMAND_IR_CONTENT ir_struct)
{
	//TODO: command avm_cmd_SendIrLONGDataToMcu_with_verify need 20 parameters...
	unsigned char insert_content[10] = {0};
	unsigned char para_len = 0;//IR_parameter_length(cmd);

	if(para_len != 0)
		memcpy(insert_content, (unsigned char*)&ir_struct, sizeof(insert_content));

	insert_data_to_uart_struct(port, cmd, para_len, insert_content);

	DM368_send_command(port);
}

void DM368_send_command_parameter(_EVENT_PORT port, unsigned char cmd, COMMAND_PARAMETERS_CONTENT para_struct)
{
	unsigned char insert_content[11] = {0};

	memcpy(insert_content, ((unsigned char*)&para_struct) + 1, sizeof(insert_content));

	insert_data_to_uart_struct(port, cmd, para_struct.length, insert_content);

	DM368_send_command(port);
}

void DM368_send_command_no_parameter(_EVENT_PORT port, unsigned char cmd)
{
	insert_data_to_uart_struct(port, cmd, 0, NULL);

	DM368_send_command(port);
}

void DM368_send_reply_parameter(_EVENT_PORT port, UART_REQUEST *request_event, unsigned char state, COMMAND_PARAMETERS_CONTENT para_struct)
{
	unsigned char insert_content[16] = {0};

//	memcpy(insert_content, ((unsigned char*)&para_struct) + 1, sizeof(insert_content));
	memcpy(insert_content, (unsigned char*)&(para_struct.ucPara), sizeof(insert_content));

	insert_reply_data_to_uart_struct(port, request_event, state, para_struct.length, insert_content);

	DM368_send_reply(port);
}

void DM368_send_reply_no_parameter(_EVENT_PORT port, UART_REQUEST *request_event, unsigned char state)
{
	insert_reply_data_to_uart_struct(port, request_event, state, 0, NULL);

	DM368_send_reply(port);
}

//special testing block----------------------------------------------------
void auto_dm368_request_command_reset(unsigned char cmd)
{
	if( uart_sequence_id++ == 0xff )
		uart_sequence_id = 0;

	printf(">>>>>>>> \033[31mevent testing [NO. 0x%02x]\033[m <<<<<<<<<\n", cmd);

	DM368_send_request_to_MCU.REQ[0] = MCU_SEND_REQUEST_HEAD_BYTE_1;
	DM368_send_request_to_MCU.REQ[1] = MCU_SEND_REQUEST_HEAD_BYTE_2;
	DM368_send_request_to_MCU.STOP_BYTE = MCU_SEND_REQUEST_STOP_BYTE;

	DM368_send_request_to_MCU.COMMAND = cmd;
	DM368_send_request_to_MCU.SEQUENCE = uart_sequence_id;
	DM368_send_request_to_MCU.FRAGMENT = 0x00;
	DM368_send_request_to_MCU.LENGTH = 0;
}

void auto_insert_data_to_uart_struct(unsigned char cmd, unsigned char payload_len, unsigned char *payload_content)
{
// 	int i;
	auto_dm368_request_command_reset(cmd);
	
	DM368_send_request_to_MCU.LENGTH = payload_len;

	if( !((payload_len == 0) || (payload_content == NULL)) )
		memcpy(&DM368_send_request_to_MCU.PAYLOAD, payload_content, sizeof(DM368_send_request_to_MCU.PAYLOAD));
		
	DM368_send_request_to_MCU.CHECKSUM = avm_crc32(0, &DM368_send_request_to_MCU.REQ[0], REQUEST_CRC_LENGTH);
}

void auto_test_send_event_to_DM368(unsigned char cmd, COMMAND_PARAMETERS_CONTENT para_struct, int need_crc)
{
	unsigned char insert_content[11] = {0};

	memcpy(insert_content, ((unsigned char*)&para_struct) + 1, sizeof(insert_content));

	auto_insert_data_to_uart_struct(cmd, para_struct.length, insert_content);

	if(need_crc == 0)
		DM368_send_request_to_MCU.CHECKSUM = 0;

	DM368_send_command(AVM_EVENT_DM368);
}
//special testing block----------------------------------------------------
//==============================================================================
