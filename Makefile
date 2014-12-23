
CROSS_COMPILE=
CC=$(CROSS_COMPILE)gcc

default:: libubus.so test_master test_slave

libubus.so:: ubus.c
	$(CC) -Wall -shared -o libubus.so $^

test_master:: test_master.c libubus.so
	$(CC) -Wall -o test_master $^ libubus.so

test_slave:: test_slave.c libubus.so
	$(CC) -Wall -o test_slave $^ libubus.so
	
clean::
	rm -f test libubus.so

