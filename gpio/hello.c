#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <pthread.h>


#include <pigpio.h>		// http://abyz.co.uk/rpi/pigpio/cif.html

/*
gcc -Wall -pthread -o hello hello.c -lpigpio -lrt
sudo ./hello
*/  

unsigned spid_adc, spid_dac, spid_att1, spid_T26, spid_pll, ser_tty, count;
unsigned int x='1', spiBaud=100000, loop_cnt;
char buff[4];
char r[20]="nothin ", c[20]="@253ACK1.234E-1;FF";			// loopback test
char *device;


int main(void)
{
	printf( "starting \n");
	if (gpioInitialise() < 0)
	{
		printf( "gpio did not initiallize");
		return(1);
	}

	gpioSetMode(4, PI_OUTPUT); 


	loop_cnt = 0;

loop:
	loop_cnt = loop_cnt+1;

	printf( "testing gpio 4 \n");
	gpioWrite(4, 0);
	usleep(100000);
	gpioWrite(4, 1);
	usleep(100000);
	if( loop_cnt <= 50 ) {goto loop;}


	gpioTerminate();
	return(0);
}
