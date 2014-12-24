CC=gcc
CFLAGS=-Wall
#CROSS_CC=arm-none-linux-gnueabi-gcc
CROSS_CC=arm-linux-gnueabihf-gcc-4.7

default:: sender receiver sender.arm receiver.arm

clean::
	rm -f sender receiver sender.arm receiver.arm

sender:: sender.c
	$(CC) $(CFLAGS) -o $@ $^

receiver:: receiver.c
	$(CC) $(CFLAGS) -o $@ $^

sender.arm:: sender.c
	$(CROSS_CC) $(CFLAGS) -o $@ $^
	scp $@ bananapi@10.1.9.94:/home/bananapi/tmp/.

receiver.arm:: receiver.c
	$(CROSS_CC) $(CFLAGS) -o $@ $^
	scp $@ bananapi@10.1.9.94:/home/bananapi/tmp/.

