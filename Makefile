
CROSS_COMPILE=
CC=$(CROSS_COMPILE)gcc
CFLAGS=-Wall
LDFLAGS=-lpthread
CROSS_CC=arm-none-linux-gnueabi-gcc
#CROSS_CC=arm-linux-gnueabihf-gcc-4.7

default:: libubus.so libubus-arm.so 
	@$(MAKE) -C test

libubus.so:: ubus.c
	$(CC) -Wall -shared -fPIC -o libubus.so $^ $(LDFLAGS)

libubus-arm.so:: ubus.c
	$(CROSS_CC) $(CFLAGS) -shared -fPIC -o libubus-arm.so $^ $(LDFLAGS) 

clean::
	rm -f libubus.so libubus-arm.so
	@$(MAKE) -C test clean


