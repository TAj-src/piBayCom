#include <linux/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

#define BAYCOM_DEV "/dev/piBayCom"

int fd, cmd;

#define MAGIC_NUM 240
#define IOCTL_CAL_DROP _IO( 'q' ,0 )
#define IOCTL_CAL_HIGH _IO( 'q' ,1 )
#define IOCTL_CAL_LOW  _IO( 'q' ,2 )
#define IOCTL_CAL_DIDDLE  _IO( 'q' ,3 )


void bye (int dummy) {

  ioctl(fd, IOCTL_CAL_DROP);
  exit(0);
}


int main(int argc, char **argv)
{
   int ret, running ,v;

   signal(SIGINT, bye);

   fd = open(BAYCOM_DEV, O_RDONLY);
    if (fd == -1) {
        printf("Error opening BayCom kernel module\n");
        perror("open");
        return EXIT_FAILURE;
    }

    running =1;

    while ( running ) {

       printf("Enter mode ( 0 ,1 ,2 , 9(Quit): ");
       v = getchar();
       getchar(); // not needed CR


       switch (v) {

        case '0' :
                ret = ioctl(fd, IOCTL_CAL_DROP);
                if (ret == -1) {
                    perror("ioctl");
                    return EXIT_FAILURE;
                }
                break;
        case '1' :
                ret = ioctl(fd, IOCTL_CAL_HIGH);
                if (ret == -1) {
                    perror("ioctl");
                    return EXIT_FAILURE;
                }
                break;
        case '2' :
                ret = ioctl(fd, IOCTL_CAL_LOW);
                if (ret == -1) {
                    perror("ioctl");
                    return EXIT_FAILURE;
                }
                break;
        case '3' :
                printf("\nSending alternate tones for ~15 seconds\n");
                ret = ioctl(fd, IOCTL_CAL_DIDDLE);
                if (ret == -1) {
                    perror("ioctl");
                    return EXIT_FAILURE;
                }
                break;
        case '9' :
                running = 0;
                printf("Exiting CAL mode\n");
                // drop PTT to be sure
                ret = ioctl(fd, IOCTL_CAL_DROP);
                if (ret == -1) {
                    perror("ioctl");
                    return EXIT_FAILURE;
                }

                break;

       }

    }

    puts("");

    close(fd);
    return EXIT_SUCCESS;
}
