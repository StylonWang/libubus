
CROSS_COMPILE=
CC=$(CROSS_COMPILE)gcc
CFLAGS=-Wall -I../ -g
LDFLAGS=-lpthread
CROSS_CC=arm-none-linux-gnueabi-gcc
#CROSS_CC=arm-linux-gnueabihf-gcc-4.7

default:: test_master test_slave test_multi_pipe
	#test_slave.arm test_master.arm

test_master:: test_master.c ../ubus.c
	$(CC) $(CFLAGS) -o test_master $^ $(LDFLAGS)

test_slave:: test_slave.c ../ubus.c
	$(CC) $(CFLAGS) -o test_slave $^ $(LDFLAGS)

test_multi_pipe:: test_multi_mpipe.c test_multi_spipe.c
	$(CC) $(CFLAGS) -o test_multi_mpipe test_multi_mpipe.c ../ubus.c $(LDFLAGS)
	$(CC) $(CFLAGS) -o test_multi_spipe test_multi_spipe.c ../ubus.c $(LDFLAGS)

test_slave.arm:: test_slave.c ../ubus.c
	$(CROSS_CC) $(CFLAGS)  -o test_slave.arm $^ $(LDFLAGS)
	scp test_slave.arm bananapi@stylon-bpi.local:/home/bananapi/tmp/.

test_master.arm:: test_master.c ../ubus.c
	$(CROSS_CC) $(CFLAGS)  -o test_master.arm $^ $(LDFLAGS)
	scp test_master.arm bananapi@stylon-bpi.local:/home/bananapi/tmp/.
	
clean::
	rm -f test_master test_slave test_slave.arm test_master.arm

