
CROSS_COMPILE=
CC=$(CROSS_COMPILE)gcc
CFLAGS=-Wall
LDFLAGS=-lpthread
CROSS_CC=arm-none-linux-gnueabi-gcc

default:: libubus.so libubus-arm.so test_master test_slave test_slave.arm test_master.arm

libubus.so:: ubus.c
	$(CC) -Wall -shared -fPIC -o libubus.so $^ $(LDFLAGS)

libubus-arm.so:: ubus.c
	$(CROSS_CC) $(CFLAGS) -shared -fPIC -o libubus-arm.so $^ $(LDFLAGS) 

test_master:: test_master.c libubus.so
	$(CC) -Wall -o test_master $^ 

test_slave:: test_slave.c libubus.so
	$(CC) -Wall -o test_slave $^ 

test_slave.arm:: test_slave.c ubus.c
	$(CROSS_CC) $(CFLAGS)  -o test_slave.arm $^ $(LDFLAGS)
	scp test_slave.arm bananapi@10.1.9.94:/home/bananapi/tmp/.

test_master.arm:: test_master.c ubus.c
	$(CROSS_CC) $(CFLAGS)  -o test_master.arm $^ $(LDFLAGS)
	scp test_master.arm bananapi@10.1.9.94:/home/bananapi/tmp/.
	
clean::
	rm -f libubus.so libubus-arm.so test_master test_slave test_slave.arm

