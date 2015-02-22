#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int serial_options(int fd, int speed, int parity);
void syncronous(int fd, int blocking);
void read_buf(int fd);
void write_buf(int fd);
int process_options(int argc, char * argv[]); 
int init(void);

