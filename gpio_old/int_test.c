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
#include <sys/time.h>




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

unsigned int i, j ;
float a;
struct timeval  tv1, tv2;

int main(void)
{
gettimeofday(&tv1, NULL);

for ( i=0; i<3200000; i++) {
   j = (j + i * 7)/3;
}

gettimeofday(&tv2, NULL);

printf ("Total int time = %f seconds\n",
         (double) (tv2.tv_usec - tv1.tv_usec) / 1000000 +
         (double) (tv2.tv_sec - tv1.tv_sec));

gettimeofday(&tv1, NULL);

for ( i=0; i<3200000; i++) {
   a = (a + i * 7.1)/3.3;
}

gettimeofday(&tv2, NULL);

printf ("Total float time = %f seconds\n",
         (double) (tv2.tv_usec - tv1.tv_usec) / 1000000 +
         (double) (tv2.tv_sec - tv1.tv_sec));



}
