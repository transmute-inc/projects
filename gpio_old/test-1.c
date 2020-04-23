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

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>



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

#define RF_en 	  5			// BCM GPIO
#define CS_ADC    8
#define CS_DAC    7
#define CS_ATT    6 
#define CS_PLL   13

#define SPI_CHANNEL 0
#define SPI_SPEED   1000000 // !! Start low 
// MAX2871 PLL 20MHz
// MAX5134 DAC 30MHz
// MAX11254 ADC 8MHz
// PE43711 ATT 10MHz


// byte order must be reversed when sending out via SPI port
// 
// use this equivalence union to access the integer as bytes
//   J    0x11223344	integer & byte alignment
//  CJ[]     3 2 1 0	
union equivs { unsigned int J; unsigned char CJ[4]; };
union equivs eq;

    
// 7=dac_cs(26), 8=adc_cs(24), 9=MISO(21), 10=MOSI(19), 11=s_clk(23), 
int my_pin=4, ce=0, spid;
unsigned int spiChan=0 , spiBaud, spiFlags;
unsigned char buff[4];


int main(void)
{
	
	if (gpioInitialise() < 0)
	{
		printf( "gpio did not initiallize");
		return;
	}

	gpioSetMode(my_pin, PI_OUTPUT); // Set GPIO18 as output.
	
	spid = spiOpen( spiChan, spiBaud, spiFlags)


loop:
	gpioWrite(my_pin, 0);
	usleep(1);
	gpioWrite(my_pin, 1);
	usleep(1); 
	
		buff[0] = 0x02;
		digitalWrite(CS_DAC, 0);
		wiringPiSPIDataRW(ce, buff, 1);
		digitalWrite(CS_DAC, 1);	
	usleep(1);
	goto loop;



	gpioTerminate();
}
