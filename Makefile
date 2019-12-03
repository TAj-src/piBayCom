obj-m+=piBayComLKM.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
	$(CC) BayComKISS.c -o BayComKISS -lpthread -lrt -g -D _GNU_SOURCE
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
	rm BayComKISS

