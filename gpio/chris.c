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


int main(void)
{
  a="9";
  b="9";
  printf( " a=%s b=%d\n", a,b);


}
