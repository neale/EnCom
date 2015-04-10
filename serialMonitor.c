#include "serialMonitor.h"
#define PACKET_SIZE 8
#define BUFFER 9
#define SLAVE_0_PORT "/dev/ttyACM0"
#define RECEIVER_0_PORT "/dev/ttyUSB0"

int serial_options(int fd, int baud, int parity){

  struct termios old_serial, serial;

  memset(&serial, 0, sizeof(serial));
  memset(&old_serial, 0, sizeof(old_serial));

  if(tcgetattr (fd, &old_serial)){

    printf("error %d from tcgetattr", errno);
    return -1;
  }
  bzero(&serial, sizeof(serial));                 /*clear for new settings */
  cfsetospeed(&serial, (speed_t)baud);            /* set input speed */
  cfsetispeed(&serial, (speed_t)baud);            /* set output speed */

  serial.c_iflag      &= ~(IXON | IXOFF | IXANY); /* x(on/off) ctrl off */
  serial.c_iflag      &= ~IGNBRK;                 /* disable break handler */

  serial.c_lflag        = 0;      /* no signaling characters, no local echo, */

  /* no canonical processing */
  serial.c_oflag        = 0;      /* no remapping, no delays */
  serial.c_cc[VMIN]     = 1;      /* read doesn't block */
  serial.c_cc[VTIME]    = .5;     /* 0.5s reading timeout */

  /* enable read */
  serial.c_cflag |= (CLOCAL | CREAD);   /* ignore modem control, */
  serial.c_cflag        |= parity;               
  serial.c_cflag        &= ~(PARENB | PARODD); /* no parity */
  serial.c_cflag        &= ~CSTOPB;            /* one stop bit */
  serial.c_cflag        &= ~CRTSCTS;           /* disable flow control */
  serial.c_cflag        = (serial.c_cflag & ~CSIZE) | CS8; /* 8-bit characters */

  /* set control characters */
  serial.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
  serial.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
  serial.c_cc[VERASE]   = 0;     /* del */
  serial.c_cc[VKILL]    = 0;     /* @ */
  serial.c_cc[VEOF]     = 4;     /* Ctrl-d */
  serial.c_cc[VTIME]    = 0;     /* inter-character timer unused */
  serial.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
  serial.c_cc[VSWTC]    = 0;     /* '\0' */
  serial.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
  serial.c_cc[VSTOP]    = 0;     /* Ctrl-s */
  serial.c_cc[VSUSP]    = 0;     /* Ctrl-z */
  serial.c_cc[VEOL]     = 0;     /* '\0' */
  serial.c_cc[VREPRINT] = 0;     /* Ctrl-r */
  serial.c_cc[VDISCARD] = 0;     /* Ctrl-u */
  serial.c_cc[VWERASE]  = 0;     /* Ctrl-w */
  serial.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
  serial.c_cc[VEOL2]    = 0;     /* '\0' */
  tcflush(fd, TCIFLUSH); 
  if(tcsetattr(fd, TCSANOW, &serial)){

    printf("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

void syncronous(int fd, int blocking){

  struct termios serial;
  memset(&serial, 0, sizeof(serial));
  if(tcgetattr(fd, &serial)){

    printf("error %d from tggetattr", errno);
    return;
  }

  serial.c_cc[VMIN] = blocking ? 1 : 0;
  serial.c_cc[VTIME] = .5;   /* 0.5 seconds timeout */

  if(tcsetattr(fd, TCSANOW, &serial))
    printf("error %d, can't set serial options", errno);
}
int init(void){

  const char *port = RECEIVER_0_PORT;
  //const char *port1 = SLAVE_0_PORT;

  printf("opening file...\n");
  int fd = open(port, O_RDWR | O_NOCTTY);
  if(fd < 0){
    printf("error in %d opening %s", errno, port);
    return -1;
  }
  printf("setting serial options...\n");
  serial_options(fd, B38400, 0);                /* 38400 baud, 8n1 */
  syncronous(fd, 0);                            /* set no blocking */
  write(fd, "welcome to EnMonitor\n\0", 20);      /* send 20 character open message */
  usleep(100000);                               /* sleep to transmit the message */
  printf("entered display mode\n");
  printf("Welcome to EnMonitor 0.1.0 Command Shell\r\n\n"
         "Compiled on Feb 21 2015\r\n"
         "Porting Data from %s \r\n\n"
         "Stop with CTRL-C\n\n\n", port);

  return fd;
}
void write_buf(int fd){
  unsigned char terminal[] ="init \n";
  int num_written = 0;
  int space = 0;
  do{  
    num_written = write(fd, &terminal[space], 1);
    space += num_written;
  }
  while((terminal[space - 1] != '\r') && (num_written > 0));
}
void read_buf(int fd){
  int size = 0;
  int space = 0;
  int i = 0;
  char buf = '\0';
  char response[BUFFER];
  memset(response, '\0', sizeof(response));
  do{
    size = read(fd, &buf, 1);
    space += size;
    if(buf != '\0'){
      if(size == 1){
        response[space] = buf;
        fflush(stdout);
      }
      else if(size < 0){
      printf("error in %d handling read\t", errno);
        usleep(100);
      }
      else if(size == 0){
        usleep(100);
      }
      else printf("what");
    }
  }
  while((buf != '\r') && (space < BUFFER-1));
  for(i = 0; i < BUFFER; i++){
    printf("%c", response[i]);
  }
  printf("  ");
}

int process_options(int argc, char * argv[]){

  int baud_flag = 0;
  char parity_flag = '\0';
  char *cvalue = NULL;
  int index;
  int c;

  opterr = 0;

  int baud_rates[19] = {
    B0, B50, B75, B110, B134, B150, B200, B300, B600,
    B1200, B1800, B2400, B4800, B9600, B19200, B38400,
    B57600, B115200, B230400 };

   while ((c = getopt (argc, argv, "abc:")) != -1)
    switch (c){

      case 'b':
        baud_flag = 1;
        break;
      case 'd':
        parity_flag = 1;
        break;
      case 'p':
        cvalue = optarg;
        break;
      case '?':
        if (optopt == 'c')
          fprintf (stderr, "Option -%c requires an argument.\n", optopt);
        else if (isprint(optopt))
          fprintf (stderr, "Unknown option `-%c'.\n", optopt);
        else
          fprintf (stderr,
                   "Unknown option character `\\x%x'.\n",
                   optopt);
        return 1;
      default:
      exit(EXIT_FAILURE);
      }

  printf ("aflag = %d, bflag = %d, cvalue = %s\n",
          baud_flag, parity_flag, cvalue);

  for (index = optind; index < argc; index++)
    printf ("Non-option argument %s\n", argv[index]);
  return 0; 

}
int main(void){
  printf("starting...\n");
  int fd = init();
  write_buf(fd);
  if(fd < 0){
    printf("error %d from open", errno);
  }
  while(1){  
    usleep(500);
    read_buf(fd);
  }
  printf("error sustaining terminal");
  close(fd); 
  return 0;
}
