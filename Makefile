
CROSS_COMPILE=
CC=$(CROSS_COMPILE)gcc

default:: libubus.so test

libubus.so:: avm_crc32.c ubus.c
	$(CC) -Wall -shared -o libubus.so $^

test:: main.c libubus.so
	$(CC) -Wall -o test $^ libubus.so
	
clean::
	rm -f test libubus.so

