#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <pthread.h>
#include <sys/types.h>  // needed for getpid()
#include <unistd.h>     // needed for getpid()

#include <pigpio.h>		// http://abyz.co.uk/rpi/pigpio/cif.html

/*
* 
* http://www.makelinux.net/alp/028   (threads)
* 
* gcc -Wall -pthread -o test test.c -lpigpio -lrt
* sudo ./k5
* gcc -Wall -ggdb -pthread -o k10 k10.c -lpigpio -lrt
* sudo gdb ./k5
*
*/

/*
Test plan
1. Test gpio Vac & Vac_pump - they require no board mods
2. Test SPI spid_adc ,spid_dac, and spid_pll
3. Test serial port.
4. See if dtoverlay allows avoiding jumpers
5. Test pthreads
5. Merge new code into p5 as p6
6.
*/


/*  http://abyz.co.uk/rpi/pigpio/index.html#Type_3
name  gpio pin#
MOSI  10   19
MISO   9   21
SCLK  11   23
CE0   08   24
CE1   07   26
                 dtoverlay reassignment
dtoverlay=spi1-3cs, cs0_pin=gpio16, cs1_pin=gpio26, cs2_pin=gpio13
mosi  20   38
miso  19   35
sclk  21   40
ce0   18   12	gpio=16
ce1   17   11	gpio=26
ce2   16   36	gpio=13

C documentation: http://abyz.co.uk/rpi/pigpio/cif.html

Download and install pigpio: http://abyz.co.uk/rpi/pigpio/download.html 

*/


#define RF_en 	  6			// BCM GPIO
//#define CS_ADC    8
//#define CS_DAC    7
//#define CS_ATT   26 
//#define CS_PLL   16

#define SPI_CHANNEL 0
#define SPI_SPEED   1000000 // !! Start low 
// MAX2871 PLL 20MHz
// MAX5134 DAC 30MHz
// MAX11254 ADC 8MHz
// PE43711 ATT 10MHz

#define H2_in    04 // RLY_1 moved from p1-11 -> p1-7
#define H2_out   22	// RLY_2 moved from p1-12 -> p1-15
#define Vac      27
#define Vac_pump 23

// byte order must be reversed when sending out via SPI port
// 
// use this equivalence union to access the integer as bytes
//   J    0x11223344	integer & byte alignment
//  CJ[]     3 2 1 0	
union equivs { unsigned int J; unsigned char CJ[4]; };
union equivs eq;

    

unsigned spid_adc, spid_dac, spid_att1, spid_T26, spid_pll, count;
unsigned ser_tty, serBaud=300, serFlags=0;
unsigned int x, spiBaud=100000, loop_cnt;
char buff[4];
char r[20]="nothin ", c[20]="@253ACK1.234E-1;FF";			// loopback test
char *device;


int main(void)
{
	x='1';
	printf( "starting \n");
	if (gpioInitialise() < 0)
	{
		printf( "gpio did not initiallize");
		return(1);
	}

	gpioSetMode(H2_in, PI_OUTPUT); 
 	gpioSetMode(H2_out, PI_OUTPUT); 
 	gpioSetMode(Vac, PI_OUTPUT); 
 	gpioSetMode(Vac_pump, PI_OUTPUT); 
  	gpioSetMode(17, PI_OUTPUT); 	
 	gpioSetMode(18, PI_OUTPUT);  	
	
	spid_adc  = spiOpen(0, spiBaud, 0);		//CE0   08   24	 T11	
	spid_dac  = spiOpen(1, spiBaud, 0);		//CE1   07   26  T12
	spid_pll  = spiOpen(0, spiBaud, 256);	//ce0   18   12	(p1-36->p1-11) 
	spid_att1 = spiOpen(1, spiBaud, 256);	//ce1   17   11	(p1-37->p1-12)

	device = "/dev/serial0";
	ser_tty = serOpen( device, serBaud, serFlags);
	printf( "ser_tty=%d\n", ser_tty);
	if (ser_tty < 0)
	{
		printf( "serial did not initiallize");
		return(1);
	}
	loop_cnt = 0;

loop:
	loop_cnt = loop_cnt+1;
	if (x == '1')
	{ 
		printf( "testing gpio 17, 18, 27, 23 \n");
		gpioWrite(H2_in, 1);
		usleep(100000);
		gpioWrite(H2_in, 0);
		gpioWrite(H2_out, 1);	
		usleep(100000);
		gpioWrite(H2_out, 0);
		gpioWrite(Vac, 1);
		usleep(100000);
		gpioWrite(Vac, 0);
		gpioWrite(Vac_pump, 1);
		usleep(100000);
		gpioWrite(Vac_pump, 0);
		gpioWrite(17, 1);		
		usleep(100000);
		gpioWrite(17, 0);
		gpioWrite(18, 1);		
		usleep(100000);
		gpioWrite(18, 0);
		usleep(100000);
	}	

 	if (x == '2')	//spi
	{
		buff[0] = 0x02;
		spiWrite(spid_dac, buff, 1);
		spiWrite(spid_adc, buff, 1);
		spiWrite(spid_pll, buff, 1);
		spiWrite(spid_att1, buff, 1);
		usleep(1000);
	}	

 	if (x == '3')	//serial
	{
		serWrite(ser_tty, c, 20);
		usleep(100000);
		printf( " read=%s\n", r );
		count = serDataAvailable(ser_tty);
		if( count > 0 )
		{
			serRead( ser_tty, r, count );
			printf( " read=%s\n", r );
		}
	}	



	if( loop_cnt <= 50000 )
	{goto loop;}

	
	spiClose(spid_dac);
	spiClose(spid_adc);
	spiClose(spid_pll);
	spiClose(spid_att1);

	serClose(ser_tty);

	gpioTerminate();
	return(0);
}
