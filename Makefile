obj-m+=piBayComLKM.o

all:
        make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
        $(CC) BayComKISS.c -o BayComKISS -lpthread -lrt -D _GNU_SOURCE
        $(CC) cal.c -o cal
clean:
        make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
        rm BayComKISS
        rm cal
