#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

#include <pthread.h>
#include <sys/types.h>  // needed for getpid()
#include <unistd.h>     // needed for getpid()




/*
* 
* http://www.makelinux.net/alp/028   (threads)
* 
* gcc -Wall -o k10 k10.c -lwiringPi -lthread
* sudo ./k5
* gcc -Wall -ggdb -o k10 k10.c -lwiringPi -lthread
* sudo gdb ./k5
*
*/

unsigned int i ;
char *a; 
char* b;
	union equiv { unsigned int J; char CJ[4]; } eq;
	char buff[4], buff_rx[4];

int main(void)
{

			buff[0] = 0x1;
			buff[1] = 0x2;
			buff[2] = 0x3;
			buff[3] = 0x4;
			eq.CJ[0] = 0x1;
			eq.CJ[1] = 0x2;
			eq.CJ[2] = 0x3;
			eq.CJ[3] = 0x4;
			printf( " %0x, %0x, %0x, %0x, %0x/n",
			eq.CJ[0],eq.CJ[1],eq.CJ[2],eq.CJ[3], eq.J); 


  a="9";
  b="9";
  printf( " a=%s b=%d\n", a,b);


}
