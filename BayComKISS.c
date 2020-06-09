/*********************************************************
 *
 *      G7TAJ BayCom Packet KISS driver for RasPi - Copyright � 2019
 *      Original BayCom code Copyright � 2001-2019 John Wiseman G8BPQ
 *
 *      06- 2019:       v0.1a
 *                      Wokring, stable with cmdline args
 *      24-09-2019:     Aded LastDCD=True to stop TX directly fater TX without CSMA
 *      26-09-2019:     Corrected KISSACK as it was ACKing everything
 *                      even not ACK frames
 *      15/10/2019:     Changed to Linux Kernel Module version, so this attached to that
 *                      and provides a KISS interface out
 *      17/10/2019:     0.1.6a Fixed TCP port disc/re-connect issue
 *      09/06/2020:     0.1.7a - Added Calibration routine via IOCTL and fixed IRQ free issue
 *
 *
 *
 **********************************************************/

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <pthread.h>
#include <sched.h> // for thread priority
#include <time.h>
#include <unistd.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <assert.h>

#include <sys/time.h>

//#define DEBUG
#define VERSION "0.1.7a\0"


typedef int BOOL;
#define TRUE 1
#define FALSE 0

#define LKM_BUFFER_LENGTH 1024              ///< The buffer length (crude but fine)
static short LKM_receive[LKM_BUFFER_LENGTH];     ///< The receive buffer from the LKM


typedef unsigned char UCHAR;
typedef unsigned int UINT;
typedef unsigned short USHORT;

int TXCOUNT=10;

#define FEND  0xC0
#define FESC  0xDB
#define TFEND 0xDC
#define TFESC 0xDD

#define Debugprintf(fmt, ... ) printf(stderr, fmt)

#define MaxFrameLen 340
#define MinFrameLen 17
#define TxBufferLen 4096

#define MaxSamples 1000


#define MAXMSG  512  //max msg on KISS TCP Port

struct TX_Q {
    BOOL ACK_FRAME;                     // True for ACK
    int FrameID_1;                      // KISS Frame ID
    int FrameID_2;                      // KISS Frame ID
    UCHAR TXBuff[MaxFrameLen +10];      // What we want to send
};

struct TX_Q ** TX_Queue = NULL;

#define LOBYTE(v)   ((unsigned char) (v))
#define HIBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))

int show_captures=0;
int client_socket_fd;

volatile int rxbuff[1000];
volatile int bitsRXed;

pthread_mutex_t lock= PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t TX_Q_lock= PTHREAD_MUTEX_INITIALIZER;

struct BAYINFO
{
        int BitTime;                                        // Bit Length in clock ticks - calculated
        int MaxBitTime;
        int MinBitTime;                                 // Limits of PLL clock
        int PLLBitTime;                                 // Actual bit length, adjusted for timing errors by DPLL

        UCHAR RxFrame[MaxFrameLen + 10];                // Receive Buffer
        int RXFrameLen;
        int BitCount;                                   // Bits received
        BOOL RxFrameOK;                                 // Frame not aborted
        UCHAR RxReg;                                    // RX Data Shift register
        int RXOneBits;                                  // Bit count for removing stuffed zeors

        BOOL DCD;                                       // Current carrier state
        BOOL LastDCD;                                   // Last carrier state (only do csma if channel was busy and is now free)

        UINT fromBPQ_Q;                                 // Frames from BPQ32 L2 to modem

        UCHAR TxBuffer[TxBufferLen];
        int TxBlockPtr;

        BOOL TXState;                                   // We are TXing
        int TXOneBits;                                  // Count of consecutive 1 bits (for stuffing)
        int RxZeroBits;                                 // count of consecutive 0 bits
        int RXERRORS;                                   // Numbers of port RX errors
        int RXOverruns;                                 // Number of RX overruns
        int gooddecodes;                                // number of frames we'd decoded OK

        BOOL FULLDUPLEX;                                // fullduplex
        UINT PERSISTANCE;                               // Persistance
        UINT TXDELAY;
        UINT RXBytes;                   // Total bytes RXed
        UINT TXBytes;                   // Total bytes TXed
};

BOOL KISSFRAME_START=FALSE;
UCHAR Frame_to_Send[300];


struct BAYINFO * BayList[33];

unsigned short CRCTAB[256] = {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


//#define KISSPORT 8616

int KISSPORT = 99; //default value
int BAUD = 1200;

#define BUFFER_MAX 50
int fd[32] = {0};
int KISS_CONNECTED = FALSE;

int LKM_fd;             // LKM file handle

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')


void RxBit(struct BAYINFO * Baycom, UCHAR NewBit);
void RemoveFrom_TX_Q(struct BAYINFO * Baycom);


int openSocket() {

        int sockfd;
        struct sockaddr_in my_addr;    /* my address information */


        if ((sockfd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1) {
            perror("socket");
            exit(1);
        }
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &(int){ 1 }, sizeof(int)) < 0)
            perror("setsockopt(SO_REUSEADDR) failed");

        my_addr.sin_family = AF_INET;         /* host byte order */
        my_addr.sin_port = htons(KISSPORT);     /* short, network byte order */
        my_addr.sin_addr.s_addr = INADDR_ANY; /* auto-fill with my IP */
        bzero(&(my_addr.sin_zero), 8);        /* zero the rest of the struct */

        if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) {
            perror("bind");
            exit(1);
        }

        if (listen(sockfd, 1) == -1) {  // listen for 1 connection only
            perror("listen");
            exit(1);
        }

        int flags = fcntl(sockfd, F_GETFL);
        fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

 return sockfd;

}



// when socket gets closed but we still try to write to it...
void sigpipe_handler()
{
    printf("KISS TCP socket error!\n");
    close(KISS_CONNECTED);
    close(client_socket_fd);
    KISS_CONNECTED=0;
    client_socket_fd=0;
/*    if ( !(fd=openSocket()) ) {
        perror("Open KISS socket failed");
    }
    printf("TCP KISS Socket %d listening...\r\n",KISSPORT);
*/
}




void get_toggles(int fd) {
  int ret;
  int x;

   memset(LKM_receive, 0, sizeof( LKM_receive));
   ret = read(fd, (char *)LKM_receive, LKM_BUFFER_LENGTH);        // Read the response from the LKM
   if (ret < 0){
      perror("Failed to read the message from the device.");
      return;
   }
   if (ret == 0){ // no toggles
      return;
   }


   x=0;
   for (x=0;x < LKM_BUFFER_LENGTH; x++) {
        if (LKM_receive[x] == 0) {
                break;
        } else {
        rxbuff[bitsRXed++]=LKM_receive[x];
       }
   }

//   if (x>0) printf("\r\n******************** Got %d toggles (%d bytes)\r\n",x, ret/2);


}



void *zalloc(len) int len;
{
        void *t = (void *) malloc(len);
        if (!t) {
                fprintf(stderr, "Malloc failed---PROGRAM ABORTED\n");
                exit(1);
        }
        memset(t, 0, len);
        return t;
}




void PrepareNewFrame(struct BAYINFO * Baycom)
{
        Baycom->RXFrameLen = Baycom->BitCount = Baycom->RXOneBits = 0;
        Baycom->RxFrameOK = TRUE;
}



unsigned short int compute_crc(unsigned char *buf,int len)
{
        unsigned short fcs = 0xffff;
        int i;

        for(i = 0; i < len; i++)
                fcs = (fcs >>8 ) ^ CRCTAB[(fcs ^ buf[i]) & 0xff];

        return fcs;
}

void AddTXBit(struct BAYINFO * Baycom, UCHAR Byte)
{
        if (Byte)
                Baycom->TXOneBits++;
        else
                Baycom->TXOneBits = 0;

        Baycom->TxBuffer[Baycom->TxBlockPtr++] = Byte;
        // *********** Testing *******
        // RxBit(Baycom, Byte);
}

void AddTxByteDirect(struct BAYINFO * Baycom, UCHAR Byte)                               // Add unstuffed byte to output
{
        int i;
        UCHAR Data = Byte;

        for (i = 0; i < 8; i++)
        {
                AddTXBit(Baycom, Byte & 1);
                Byte >>= 1;
        }
}


void AddTxByteStuffed(struct BAYINFO * Baycom, UCHAR Byte)       // Add unstuffed byte to output
{
        int i;
        UCHAR Data = Byte;

        for (i = 0; i < 8; i++)
        {
                AddTXBit(Baycom, Byte & 1);
                Byte >>= 1;

                if (Baycom->TXOneBits == 5)
                        AddTXBit(Baycom, 0);
        }
}



void SendtoBaycom(struct BAYINFO * Baycom)
{
        int ret, retcode;
        int Ptr = 0;
        int a=0;
        int x;
        pthread_t thread_id = pthread_self();
        struct sched_param params;


#ifdef DEBUG
        printf("Baycom Writing %d bits\r\n", Baycom->TxBlockPtr);
        printf("Sending %d bits (in bytes) to LKM\r\n", Baycom->TxBlockPtr);
#endif

        UINT tmpPtr= Baycom->TxBlockPtr;

#ifdef DEBUG
        int b=0;
        while(tmpPtr!=0) {
           printf("%d", Baycom->TxBuffer[b++]);
           if ( (tmpPtr % 7) == 0 ) printf(" ");
           tmpPtr--;                // continue through
        }
        printf("\r\ntotal=%d\r\n",b);
#endif


        ret = write(LKM_fd, (char *)Baycom->TxBuffer, Baycom->TxBlockPtr); // Send the string to the LKM (byte is 2)
        if (ret < 0){
                perror("Failed to write the message to the device.");
                return;
        }



     // return transmitted frames to node, after setting FRACK timer
     pthread_mutex_lock(&TX_Q_lock);

     if ( TX_Queue[0]->ACK_FRAME) { // ACK frame

             UCHAR tmpbuf[5];
        #ifdef DEBUG
             printf("*** SENDING KISS ACK Frame ID=%02x %02x\r\n", TX_Queue[0]->FrameID_1, TX_Queue[0]->FrameID_2);
        #endif
             tmpbuf[0]=FEND;
             tmpbuf[1]=0x0C; //ACK frame
             tmpbuf[2]=TX_Queue[0]->FrameID_1;
             tmpbuf[3]=TX_Queue[0]->FrameID_2;
             tmpbuf[4]=FEND;


            if ( (send(KISS_CONNECTED, tmpbuf, 5, 0) == -1)) {
                printf("\r\n################################################### KISS Socket closed ####\r\n");
                close(KISS_CONNECTED);
                KISS_CONNECTED=0;
             } else {
        #ifdef DEBUG
              printf("KISS frame sent OK\r\n");
        #endif

             }
      } //send ACK if needed

#ifdef DEBUG
     printf("Frame sent\r\n");
#endif

     Baycom->LastDCD = TRUE; //after a TX dont assume the channel is free (to stop TX ,TX)!
     Baycom->TXState = FALSE;
     pthread_mutex_unlock(&TX_Q_lock);

     RemoveFrom_TX_Q(Baycom);

     fflush(stdout);

 return;
}


void EncodePacket(struct BAYINFO * Baycom, UCHAR * Data, int Len)
{
        USHORT CRC = compute_crc(Data, Len);
        UCHAR * Msg = Data;
        UCHAR * Msgend = Data + Len;

        AddTxByteDirect(Baycom, 0x7e);                  // Start Flag - add without stuffing

        while (Msg < Msgend)
        {
                AddTxByteStuffed(Baycom, *(Msg++));     // Send byte with stuffing
        }

        CRC ^= 0xffff;

        AddTxByteStuffed(Baycom, LOBYTE(CRC));
        AddTxByteStuffed(Baycom, HIBYTE(CRC));
}


void CheckTX(struct BAYINFO * Baycom)
{
  pthread_t thread_id;


        if (Baycom->fromBPQ_Q == 0 || Baycom->TXState) {
                //printf("Nothing to send\r\n");
                return;
        }

        if (Baycom->FULLDUPLEX == 0)
        {
                // Do CSMA

                if (Baycom->DCD) {
#ifdef DEBUG
                        printf("\r\n<*** DCD BLOCKED ***>\r\n");
#endif
                        Baycom->LastDCD =1; // tried but busy, so DCD drops, csma withh happen
                        return;
                }
                // if DCD has just dropped, do csma. If not, just transmit

                if (Baycom->LastDCD)
                {
                        UINT Random = rand();
#ifdef DEBUG
                        printf("checking PERSISTANCE (%d)", LOBYTE(Random));
#endif
                        if (LOBYTE(Random) > Baycom->PERSISTANCE) {
#ifdef DEBUG
                                printf(" - NOPE\n");
#endif
                                return;
                        }
                        Baycom->LastDCD=0;
#ifdef DEBUG
                        printf(" - YEP\n");
#endif

                }
        }

        // ok to send

        Baycom->TXState=TRUE;  //We're now in TXState
        Baycom->TxBlockPtr = 0;


                UCHAR * buffptr=calloc(MaxFrameLen, sizeof(UCHAR));
                memcpy(buffptr, TX_Queue[0]->TXBuff, MaxFrameLen);


                int txlen=buffptr[0]-1; // dont include the size byte [0]

#ifdef DEBUG
                printf("\r\nFramelen=%d\r\n",txlen);fflush(stdout);
#endif

                buffptr++; //move passed frame length at pos 0
                EncodePacket(Baycom, &buffptr[0], txlen);
                buffptr--;
                free(buffptr);

        // EncodePacket adds a starting flag, but not an ending one, so successive packets can be sent with a single flag
        // between them. So at end add a teminating flag and a pad to give txtail

        AddTxByteDirect(Baycom, 0x7e);                  // End Flag - add without stuffing
        AddTxByteDirect(Baycom, 0xff);                  // Tail Padding
        AddTxByteDirect(Baycom, 0xff);                  // Tail Padding

        Baycom->TXBytes += txlen;               // Update total data transfered (dont include start flag & end flag + padding)

        pthread_create(&thread_id, NULL, (void * (*)(void*))SendtoBaycom, (void *)Baycom);
        pthread_detach(thread_id);
}


void send_KISS( UCHAR * Frame) {
     UCHAR * Msg = Frame;
     int Len = Msg[1]; //need to handle >256 pkts
     int x;
     int y=2;
     UCHAR tmpbuf[300];
#ifdef DEBUG
     printf("**** SENDING KISS Frame to %d LEN=%d\r\n", KISS_CONNECTED, Len);
#endif

     Msg++;

     tmpbuf[0]=FEND;
     tmpbuf[1]=0; //data frame
     for (x=2; x< Len+2; x++) {

       switch (Msg[1]) {
        case    FESC:
                tmpbuf[y++]=FESC;
//              printf("%02x ",tmpbuf[y-1]);
                tmpbuf[y]=TFESC;
                break;
        case    FEND:
                tmpbuf[y++]=FESC;
//              printf("%02x ",tmpbuf[y-1]);
                tmpbuf[y]=TFEND;
                break;
        default:
                tmpbuf[y]=Msg[1];
       }

     //  printf("%02x ",tmpbuf[y]);
       Msg++;
       y++;
     }

    tmpbuf[y]=FEND;

#ifdef DEBUG
   // printf("KISS Frame Len=%d\r\n",y);
    for (int h=0; h<= y; h++) {
      printf("%02x ", tmpbuf[h]);
      if ((h % 19)==0 && (h>0)) printf("\n");
    }

  printf("\r\n");
#endif

    if ( (send(KISS_CONNECTED, tmpbuf, y+1, 0) == -1)) {
        printf("#### KISS Socket closed ####\r\n");
        fflush(stdout);
        close(KISS_CONNECTED);
        KISS_CONNECTED=0;
     } else {
#ifdef DEBUG
      printf("KISS frame sent OK\r\n");
#endif

     }


}

void DecodeCalls(UCHAR * RxFrame, int RXFrameLen, int TXRX) {

        // DEBUG PRINT SOME FRAME INFO
        time_t t = time(NULL);
        struct tm *tm = localtime(&t);
        char s[64];
        int a;
        int end;
        UINT ssid;

        assert(strftime(s, sizeof(s), "%c", tm));
        printf("[%s] ", s);
        printf("%s: ", TXRX == 1 ? "T" : "R");
        //from call
        for (a = 7; a <= 12; a++) {
                if ( RxFrame[a] != 0x40 )
                        printf("%c", RxFrame[a] >>1);
        }

        ssid = RxFrame[13];
        ssid = (ssid >>1);
        if ( (ssid & 0x0F) != 0)
        printf("-%d", (ssid & 0x0F) ); //SSID

        printf(" > ");

        //to call
        for (a = 0; a <=5 ; a++) {
                if ( RxFrame[a] != 0x40 )
                        printf("%c", RxFrame[a] >>1);
        }
        ssid = RxFrame[6] >>1;
        if ( (ssid & 0x0F ) != 0)
                printf("-%d", ssid & 0x0F ); //SSID

        int framepos=14;
        if ( (RxFrame[framepos-1] & 0x01) == 0 ) { //dest call has DIGI? LSB==0
                BOOL finished=FALSE;
                while ( !finished) {
                        printf(", ");
                        for (a = framepos; a <= framepos+5; a++) {
                                if ( RxFrame[a] != 0x40 )
                                  printf("%c", RxFrame[a] >>1); //from call
                        }
                        ssid = RxFrame[framepos+6] >>1;
                        if ( (ssid & 0x0F) !=  0)
                                printf("-%d", ssid & 0x0F ); //SSID

                        if (  (RxFrame[framepos+6] & 0x80) == 0x80)
                                        printf("*");

                        if ( (RxFrame[framepos+6] & 0x01) ==1) {
                                finished=TRUE;
                        } else {
                                framepos += 7;
                        }
                }
                framepos += 7;
        }
//printf("framepos=%d, len=%d", framepos,RXFrameLen);
        a=0;
        //Ctrl frame
        if (       ((RxFrame[framepos] &  0x01) == 0)
//              || ( ((RxFrame[framepos] &  0x0f) == 0x0f) && ((RxFrame[framepos] &  0x1f) == 0x1f) )) {
                || ( (RxFrame[framepos] & 0x03) == 3) ) {
                printf(" <I/U/S/C/D>");
                framepos += 2;// iframe so also a PID

                printf("\n");

                if (TXRX) {
                        end=2; //TX includes FCD
                } else {
                        end=3; // RX doesnt
                }

                //currently chopping I frame ends by 1 char. differnet length for UI, <UI R> <UI C> I etc are the issue
                // need to properly impliment frame type. This is no affecting KISS or TX pkts being sent.

                for (a = framepos; a <=((RXFrameLen)-end); a++) { //dont inc FCS
                        if ( (RxFrame[a] & 0x7f ) > 31 )
                                printf("%c", (RxFrame[a] & 0x7F) ); //makes it readable char
                        if ( (RxFrame[a] == 0x0D) && (a != (RXFrameLen-end)) ) {
                                printf("\n");
                        }
                }
        } else {
                printf("\n");
        }
        if (a>0)
           printf("\n");

}


void ProcessFlag(struct BAYINFO * Baycom)
{
        // BitCount should have the 7 bits extracted from the (unstuffed) flag if are on a byte boundary
#ifdef DEBUG

        printf("\r\nBAYCOM Flag RX'ed Framelen %d FrameOK %d BitCount %d\t",
                Baycom->RXFrameLen, Baycom->RxFrameOK, (Baycom->BitCount & 7));
#endif

        if (Baycom->RXFrameLen > 16)
        {
#ifdef DEBUG
                printf("\r\nBAYCOM RX Data \r\n");
                for (int i = 1; i <= Baycom->RXFrameLen-2; i++) { // dont include FCS bytes
                         printf("%02x ", Baycom->RxFrame[i-1]);
                        if (((i % 20)==0) && (i>0)) printf("\r\n");
                }
                printf("\r\n");
                printf("FCS=%02x %02x\r\n",Baycom->RxFrame[Baycom->RXFrameLen], Baycom->RxFrame[Baycom->RXFrameLen-1]);
#endif

        }

        if (Baycom->RXFrameLen >= MinFrameLen && Baycom->RxFrameOK && (Baycom->BitCount & 7) == 7)
        {
                // Check CRC
#ifdef DEBUG
                printf("checking CRC\t");
#endif

                USHORT crc = compute_crc(Baycom->RxFrame, Baycom->RXFrameLen);
                {
                        if (crc == 0xf0b8)              // Good CRC
                        {
                                UCHAR * buffptr=calloc(MaxFrameLen, sizeof(UCHAR));
#ifdef DEBUG
                                printf("**** Good CRC ****\r\n");
#endif

                                if (buffptr)
                                {
                                        memcpy(buffptr+2, Baycom->RxFrame, Baycom->RXFrameLen);
                                        buffptr[1] = Baycom->RXFrameLen - 2; // *** NEED TO HANDLE > 256 ?

                                        Baycom->RXBytes += Baycom->RXFrameLen;

                                        // decode callsigns & display packet data
                                        DecodeCalls(Baycom->RxFrame,Baycom->RXFrameLen, 0);

                                        if (KISS_CONNECTED) {
                                                send_KISS(buffptr);
                                        }
                                        Baycom->gooddecodes++;
                                } //if buffprt
                                free(buffptr);

                        }
                        else
                        {
#ifdef DEBUG
                                // Bad CRC
                                printf("<<< BAD CRC >>>\r\n");
#endif

                                Baycom->RXERRORS++;
                        }
                }
        }

//      printf("\r\nBAD FRAME\r\n");
        // Bad frame, not closing flag no buffers or whatever - set up for next frame
        Baycom->DCD =FALSE;
        PrepareNewFrame(Baycom);
        return;
}





void RxBit(struct BAYINFO * Baycom, UCHAR NewBit)
{

        UCHAR RxShiftReg = Baycom->RxReg >> 1;
        int BitCount;
        if (NewBit)                             // 1 bit
        {
                RxShiftReg |= 0x80;
                Baycom->RXOneBits++;    // Another 1 bit
//              Baycom->RxZeroBits=0; // broke our run of 0's

                if (Baycom->RXOneBits   == 4)                   // 4 ones - start of header DCD =TRUE (or should it be 4 0x7e flags!!??)
                {
                        Baycom->DCD = TRUE;

                }

                if (Baycom->RXOneBits > 6)              // >6 Ones - an abort
                {
                        Baycom->RxFrameOK = Baycom->DCD = FALSE;

                        Baycom->PLLBitTime = Baycom->BitTime;   // Reset PLL
                        return;
                }
        }
        else
        {
                // Zero Bit - check for stuffing
                if (Baycom->RXOneBits   == 6)                   // 6 Ones followed by zero - a flag
                {
                        ProcessFlag(Baycom);
                        return;
                }

                if (Baycom->RXOneBits   == 5)
                {
                        Baycom->RXOneBits = 0;
                        return;                                 // Drop bit
                }
                Baycom->RXOneBits = 0;
/*              Baycom->RxZeroBits++; //see if we get lots of 0 in a row?
                if ( Baycom->RxZeroBits ==8 ) {
                        printf("\r\n\t\t\t###### ALL 0's #######\r\n");
                }
*/
                // No need to or in zero bit
        }

        // See if 8 bits received
        BitCount = ++Baycom->BitCount;
        if ((BitCount & 7) == 0)                // 8 bits
        {
                int Len = Baycom->RXFrameLen;
                Baycom->RxFrame[Len++] = RxShiftReg;

                if (Len > MaxFrameLen) {
                        Baycom->RxFrameOK = FALSE;
                        printf("frame too large (%d)\r\n", Len);
                }
                else
                        Baycom->RXFrameLen = Len;
        }

        Baycom->RxReg = RxShiftReg;
}



void ProcessSample(struct BAYINFO * Baycom, int Interval)
{
        // Interval is time since previous input transition
        int BitTime = Baycom->PLLBitTime;
        int ReasonableError = BitTime / 5;
        int Bits = 0;

        if (Interval < BitTime - ReasonableError)               // Allow 5% error here
                return;                                         // Short sample - assume noise for now
                                                                // at some point try to ignore short spikes

        if (Interval > BitTime * 10)
        {
#ifdef DEBUG
                printf("Bittime = %d interval = %d\r\n", BitTime, Interval);
#endif
                Interval = BitTime * 10;                        // Long idles - 9 ones is plenty to abort and reset frame
                Baycom->DCD = FALSE;
                //printf("DCD=%d\t",Baycom->DCD);
        }

        while (Interval > BitTime + ReasonableError)            // Allow 5% error here
        {
                RxBit(Baycom, 1);
                Interval -= BitTime;
                Bits++;
        }

        RxBit(Baycom, 0);
        Bits++;

        Interval -= BitTime;

        Interval /= Bits;                       // Error per bit

        // Use any residue to converge DPLL, but make sure not spurious transition before updating DPLL
        // Can be between - (BitTime - Reasonable) and Reasonable. Can't be greater, or we would have extraced another 1 bit

        if ((Interval + ReasonableError) < 0)
        {
                // Do we try to ignore spurious? Maybe once synced.
                // for now, ignore, but dont use to converge dpll
//                printf("Bittime = %d Residue interval = %d\r\n", BitTime, Interval);
                Baycom->DCD = FALSE;
                return;
        }

        if (Interval)
        {
                Baycom->PLLBitTime += Interval /16;
                if (Baycom->PLLBitTime > Baycom->MaxBitTime)
                        Baycom->PLLBitTime = Baycom->MaxBitTime;
                else
                        if (Baycom->PLLBitTime < Baycom->MinBitTime)
                                Baycom->PLLBitTime = Baycom->MinBitTime;
        }

}

void BaycomInit (int Port ) {
 int Speed;
 int SysClock;
 uint64_t t;
 struct BAYINFO * Baycom;
 struct timeval start, end;

   Baycom = BayList[Port] = zalloc(sizeof(struct BAYINFO));
   printf("Baycom Serial Modem Port %d\r\n", Port);
   fflush(stdout);

   // calculate BITTIME
   gettimeofday(&start, NULL);
   sleep(1);
   gettimeofday(&end, NULL);
   long seconds = (end.tv_sec - start.tv_sec);
   long micros = ((seconds * 1000000) + end.tv_usec) - (start.tv_usec);
   printf("Time elpased is %d seconds and %d micros\n", seconds, micros);
   SysClock = (double)micros;;
   Speed = BAUD;

   Baycom->BitTime = Baycom->PLLBitTime = (double)SysClock / (double)Speed;

   Baycom->MaxBitTime = 1.05 * SysClock / Speed;
   Baycom->MinBitTime = 0.95 * SysClock / Speed;
   PrepareNewFrame(Baycom);

   printf("Bit time for %d BAUD = %d\tMax bittime =%d\t Min bittime=%d\n\r", BAUD,Baycom->BitTime,Baycom->MaxBitTime, Baycom->MinBitTime);

}

void Ctrl_break_handler(int s){
           printf("\r\nCaught signal %d\n",s);
           printf("\n\nSTATS:\nRX Good pkts= %d\r\n", BayList[1]->gooddecodes);
           printf("RXErrorss   = %d\r\n", BayList[1]->RXERRORS);
           printf("RXOverruns  = %d\r\n", BayList[1]->RXOverruns);
           printf("Closed BCM\r\n");
           exit(1);
}


void RemoveFrom_TX_Q( struct BAYINFO * Baycom) {

        pthread_mutex_lock(&TX_Q_lock);

#ifdef DEBUG
        printf("Trying to free TX_Q : ");
#endif
        if( Baycom->fromBPQ_Q == 1) {
#ifdef DEBUG
          printf("to null\r\n");
#endif
          free(TX_Queue[0]);
          TX_Queue=NULL;
        } else {
#ifdef DEBUG
         printf("to %d\r\n", (Baycom->fromBPQ_Q-1));
#endif
         TX_Queue[0]=TX_Queue[1];
         TX_Queue = realloc(TX_Queue, (Baycom->fromBPQ_Q) * sizeof(struct TX_Q ));
        }

        Baycom->fromBPQ_Q--;
        pthread_mutex_unlock(&TX_Q_lock);

}


void Addto_TX_Q(BOOL ACK_FRAME,int ID1, int ID2,UCHAR * Frame, struct BAYINFO * Baycom) {

#ifdef DEBUG
   printf("Adding new TX Frame - POS %d\r\n", Baycom->fromBPQ_Q);

   for(int a=0;a< Frame[0]; a++) {
     printf("%02x ", Frame[a]);
   }
   printf("\r\n");
#endif

   pthread_mutex_lock(&TX_Q_lock);

   TX_Queue = realloc(TX_Queue, (Baycom->fromBPQ_Q) * sizeof(struct TX_Q ));
   TX_Queue[Baycom->fromBPQ_Q] = malloc(sizeof(struct TX_Q));
   memset( TX_Queue[Baycom->fromBPQ_Q], 0, sizeof(struct TX_Q) );              // Always a good idea to clear malloc'ed memory!

   TX_Queue[Baycom->fromBPQ_Q]->ACK_FRAME = ACK_FRAME;

  //frameID
   TX_Queue[Baycom->fromBPQ_Q]->FrameID_1 = ID1;
   TX_Queue[Baycom->fromBPQ_Q]->FrameID_2 = ID2;

   int Len=Frame[0]; //frame leng at pos[0]

   memcpy( TX_Queue[Baycom->fromBPQ_Q]->TXBuff , Frame ,Len); //copy to TX_QUEUE position (not inc headers) but does inc framelength

#ifdef DEBUG
   for (int x=0 ; x<= Baycom->fromBPQ_Q; x++) {
     printf("TX_Q[%d] (LEN=%d) =>\r\n", x , TX_Queue[x]->TXBuff[0] );
     for (int y=0;y < TX_Queue[x]->TXBuff[0]; y++) {
       printf("%02x ", TX_Queue[x]->TXBuff[y]);
     }
     printf("\r\n");
   }
#endif

   Baycom->fromBPQ_Q++;

   pthread_mutex_unlock(&TX_Q_lock);

}


/*********
/ Send TX Params to LKM
/*********/
void send_TXParam_LKM( struct BAYINFO * Baycom ) {

  int ret;
  UCHAR txd_id[]={0xFF, Baycom->TXDELAY/10};            // TXDELAY /10 so it fits in a byte
#ifdef DEBUG
  printf("Sending %d bytes to LKM\r\n", sizeof(txd_arr));
#endif

  ret = write(LKM_fd, txd_id,sizeof(txd_id));           // Send the txdelay to the LKM
  if (ret < 0){
         perror("Failed to write the message to the device.");
  }
}


void DecodeKISS(UCHAR * Frame, int Len, struct BAYINFO * Baycom) {
int a,x, FrameValid=0;
int framecount=1;
BOOL ACK_FRAME = FALSE;
int ID1 = 0;
int ID2 = 0;
int Frame_start=1;

 if ( (Frame[0] ==0) || (LOBYTE(Frame[0])==0x0C) ) { //DATA Frame or KISS DataFrame

   if (LOBYTE(Frame[0]) ==0x0C) { //KISS ACK FRAME
#ifdef DEBUG
     printf("\nKISS ACK FRAME ID= %02x %02x\r\n", Frame[1], Frame[2]);
#endif
     ID1 = Frame[1];
     ID2 = Frame[2];
     ACK_FRAME = TRUE;
     Frame_start=3;
   } else {
#ifdef DEBUG
     printf("\nNormal KISS DataFrame\r\n");fflush(stdout);
#endif
   }
        for (a=Frame_start; a< Len; a++) {

#ifdef DEBUG
         printf("%02x ",Frame[a]);
         if ( (a >0) && (((a) % 20)==0)) printf("\n");

#endif
          switch (Frame[a]) {


           case FESC:
                        if (Frame[a+1]==TFEND) {
                            Frame_to_Send[framecount++]=FEND; //replace with FEND and inc past TFEND
                            a++;
                        } else {
                          if (Frame[a+1]==TFESC) {
                            Frame_to_Send[framecount++]=FESC;  //replace with FESC and inc past TFESC
                            a++;
                          }
                        }
                        break;
          default:
                    Frame_to_Send[ framecount++ ]=Frame[a];

          }

        }

  Frame_to_Send[0] = framecount ; //length of frame data
  FrameValid=1;
  } else {
   // config frames

   if (LOBYTE(Frame[0]) ==0x01) {
        Baycom->TXDELAY = Frame[1]*10;
        printf("TXDelay = %dms\r\n", Baycom->TXDELAY);
        send_TXParam_LKM(Baycom);
   }
   if (LOBYTE(Frame[0]) == 0x02) {
        Baycom->PERSISTANCE = Frame[1];
        printf("Persistance = %d\r\n", Baycom->PERSISTANCE);
   }
   if (LOBYTE(Frame[0]) == 0x03) {
        printf("SlotTime => Ignored\r\n");
   }
   if (LOBYTE(Frame[0]) == 0x04) {
        printf("TX Tail => Ignored\r\n");
   }
   if (LOBYTE(Frame[0]) == 0x05) {
        Baycom->FULLDUPLEX = Frame[1];
        printf("FULLDUPLEX  = %s\r\n", Baycom->FULLDUPLEX > 0 ? "TRUE" : "FALSE");
   }
 } // frame type

     if (FrameValid) {
        UCHAR * Callbuffptr=calloc(framecount-1, sizeof(UCHAR));
        memcpy(Callbuffptr, Frame_to_Send+1, framecount-1);

        DecodeCalls( Callbuffptr,framecount-1,1);  //take off length bit
        free(Callbuffptr);

#ifdef DEBUG
        printf("\r\nDecocded KISS FRAME B4 add to TX_Q (LEN=%d)\r\n", Frame_to_Send[0]);
        for (x=0;x< framecount ; x++) {
          printf("%02x ", Frame_to_Send[x] );
          if ( (x >0) && (((x+1) % 20)==0)) printf("\n");

        }
        printf("\r\n");
#endif

        Addto_TX_Q(ACK_FRAME,ID1,ID2,Frame_to_Send, Baycom);
    }

    Frame_to_Send[0]=0; // null the sending frame
}


void ShowVer() {

        printf("\nG7TAJ BayCom Packet KISS driver for RasPi - Copyright � 2019 - Version %s \n", VERSION);
        printf("Original BayCom code Copyright � 2001-2020 John Wiseman G8BPQ\n\n");
}

void printHelp() {
        printf("\n\tUSAGE:\n\n"
                   "\t\t-v  \tShow version information\n"
                   "\t\t-kXX\tKISS TCP port\n\n"
                   "\t\t-bXXXX\tModem speed [1200|300] optional default is 1200\n\n"
                   "\te.g. ./BaycomKISS -k8515 [-b1200|300]\n\n");
}


void CheckArgs(int argc, char** argv) {
  int x,m,n,l,ch;
  char s[3];

   for( n = 1; n < argc; n++ )            /* Scan through args. */
     {
       switch( (int)argv[n][0] )            /* Check for option character. */
       {
       case '-':
       case '/':
                 l = strlen( argv[n] );
                 for( m = 1; m < l; ++m ) /* Scan through options. */
                 {
                   ch = (int)argv[n][m];
                   switch( ch )
                   {
                   case '?': printHelp();
                             exit(0);
                             break;

                   case 's':
                   case 'S': printf( "Showing Capture time data\n" );
                             show_captures=1;
                             break;
                   case 'v':              /* String parameter. */
                   case 'V': exit(0);
                             break;
                   case 'K':
                   case 'k': if( m + 1 >= l )
                             {
                               puts( "Illegal syntax -- KISS port!" );
                               exit( 1 );
                             }
                             else
                             {
                               strcpy( s, &argv[n][m+1] );
                               KISSPORT = atoi(s);
                             }
                             x = 1;
                             break;

                   case 'B':
                   case 'b': if( m + 1 >= l )
                             {
                               puts( "Illegal syntax -- BAUD Rate!" );
                               exit( 1 );
                             }
                             else
                             {
                               strcpy( s, &argv[n][m+1] );
                               BAUD = atoi(s);
                             }
                             x = 1;
                             break;


                   default:  printf( "Illegal option code = %c\n", ch );
                             exit( 1 );
                             break;
                   }
                if (x==1) break;
                }
        }
     }
    printf("KISSPORT=%d\n",KISSPORT);
}


int open_LKM() {
 int fd;
   fd = open("/dev/piBayCom", O_RDWR);             // Open the device with read/write access
   if (fd < 0){
      perror("Failed to open the device...");
   }
   return fd;
}




// *****************************************
// **********      MAIN   ******************
// *****************************************
int main(int argc, char** argv) {

  pthread_t thread_id, thread_id2;
  int tmpbits;
  int tmpbuff[1024];
  UCHAR RXbuf[384];
  UCHAR tmpRXbuf[1024];
  int x, main_y, fd;
  int RXCOUNT = 0;
  // instal sigpipe handler
  signal (SIGPIPE,sigpipe_handler);

  struct sockaddr_in their_addr;
  int new_fd;
  int sin_size;
  time_t start_t,end_t,diff_t;



  // to stop swapping - www says this helps!?
  struct sched_param sp;
  memset(&sp, 0, sizeof(sp));
  sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
  sched_setscheduler(0, SCHED_FIFO, &sp);
  mlockall(MCL_CURRENT | MCL_FUTURE);

  ShowVer();
  if (argc>1) {
        CheckArgs(argc, argv);
  } else {
        printHelp();
        exit(0);
  }


  BaycomInit(1);
  printf("Baycom port config OK\r\n");
  BayList[1]->TXDELAY = 300; // in mSec
  BayList[1]->PERSISTANCE = 128 ; // default

  signal (SIGINT,Ctrl_break_handler);

  if ( !(fd=openSocket()) ) {
        perror("Open KISS socket failed");
  }
  printf("TCP KISS Socket %d listening...\r\n",KISSPORT);

  LKM_fd=open_LKM();
  if (LKM_fd < 0 ) {
        printf("Could not open the link to the LKM - is it loaded?\r\n");
        exit(1);
  }
  send_TXParam_LKM(BayList[1]);                                 // Send TX Delay parameter to LKM

  /**** Main Loop ****/
  while (1) {

   tmpbits=0;
    if (bitsRXed >= MinFrameLen) {
//      printf("bits=%d\n",bitsRXed);
        tmpbits = bitsRXed;
        bitsRXed=0;

        for (x=0; x< tmpbits; x++) {
           tmpbuff[x]=rxbuff[x];
        }
     }

     if (tmpbits>0) {

       for (x=0; x< tmpbits; x++) {
          ProcessSample(BayList[1], tmpbuff[x]);
       }
     }

        if (BayList[1]->DCD && tmpbits==0) {
            BayList[1]->DCD = FALSE;
        }



        //TCP part
    if (!KISS_CONNECTED) {
        client_socket_fd = accept(fd, NULL, NULL);
        if (client_socket_fd == -1) {
           if (errno == EWOULDBLOCK) {
           } else {
             perror("error when accepting connection");
             exit(1);
           }
        } else {
      printf("KISS TCP port %d Connected (%d)\n",KISSPORT, client_socket_fd);
      int flags = fcntl(client_socket_fd, F_GETFL);
      fcntl(client_socket_fd, F_SETFL, O_NONBLOCK); // set new port to NON_BLOCK
      KISS_CONNECTED=client_socket_fd;
      }
    }
      if (KISS_CONNECTED) {

        //see if data waiting on port and add to KISSBUFF
        int rxbytes = recv(KISS_CONNECTED, tmpRXbuf, sizeof(tmpRXbuf), 0);
        if (rxbytes != -1)
        {
#ifdef DEBUG
                printf("KISS RXbytes=%d\r\n",rxbytes);
#endif
                for (int x=0; x< rxbytes;x++){
#ifdef DEBUG
                  printf("%02x ",tmpRXbuf[x]);
                  if ( (x >0) && (((x+1) % 20)==0)) printf("\n");
#endif
                  if (tmpRXbuf[x]==FEND) {
                    if (KISSFRAME_START) {
                        //end of frame? - send buff to decode KISS -possibly causing errors on FEND FEND <data> FEND ??
                        DecodeKISS(RXbuf,main_y,BayList[1]);
                        KISSFRAME_START=FALSE;
                        main_y=0;
                    } else {
                      KISSFRAME_START=TRUE;
                      main_y=0;
                    }
                  } else { // FEND
                    RXbuf[main_y++]=tmpRXbuf[x];
                  } // KISS Frame data
                }
        } //If RXBytes != -1

        //check if TCP port has been closed
        if (recv(KISS_CONNECTED, NULL, 0, MSG_PEEK | MSG_DONTWAIT) == 0) {
                printf("KISS Port (ID=%d) Closed!\r\n",KISS_CONNECTED);
                close(KISS_CONNECTED);
                KISS_CONNECTED=0;
        }

      } //If KISSCONNECTED ( TCP Port connected)



        if ( TXCOUNT < 1) {
          CheckTX(BayList[1]);
          TXCOUNT=10;
        } else {
          TXCOUNT--;
        }

        usleep( 10000 );  //10ms
        // 100ms (10x10)-> acts as slottime for TX too

        time(&end_t);
        diff_t = difftime(end_t, start_t);


        if (RXCOUNT >= 5) {
                get_toggles(LKM_fd);
                RXCOUNT=5;
        }
        RXCOUNT++;

        // display stats every 30 seconds
        if (diff_t > 30) {

                printf("\n\nSTATS:\nRX Good pkts: %d\r\n", BayList[1]->gooddecodes);
                printf("TXBytes     :\t%.3fK\n" ,(float)(BayList[1]->TXBytes/(float)1000));
                printf("RXBytes     :\t%.3fK\n" ,(float)(BayList[1]->RXBytes/(float)1000));
                printf("RXErrorss   : %d\n"     , BayList[1]->RXERRORS);
                printf("RXOverruns  : %d\n\n"   , BayList[1]->RXOverruns);
                start_t = end_t;

        }
}

    return (EXIT_SUCCESS);
}
