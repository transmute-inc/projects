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

#include <pigpio.h>		// http://abyz.co.uk/rpi/pigpio/cif.html
#include <../gnuplot/gnuplot_i.h>


/*
* gcc -Wall -o p6 p6.c -lpigpio -lthread
* sudo ./p6
* gcc -Wall -ggdb -o p6 p6.c -lpigpio -lthread
* sudo gdb ./p6
*/


#define Vref 2.44	       // vref is common for adc & dac
#define dac_bits 65535

#define SPI_CHANNEL 0
#define SPI_SPEED   100000 // !! Start 0 
// MAX2871 PLL 20MHz
// MAX5134 DAC 30MHz
// MAX11254 ADC 8MHz
// PE43711 ATT 10MHz

#define RF_en 	  6			// BCM GPIO
#define H2_in    04 // RLY_1 moved from p1-11 -> p1-7
#define H2_out   22	// RLY_2 moved from p1-12 -> p1-15
#define Vac      27
#define Vac_pump 23

#define not_running 0		//threadstatus
#define running     1		//threadstatus
#define kill        2		//threadstatus
#define inuse       1		//ADC status
#define available   0		//ADC status
#define idle		0
#define init        1
#define step        2
#define run			3

// byte order must be reversed when sending out via SPI port
// 
// use this equivalence union to access the integer as bytes
//   J    0x11223344	integer & byte alignment
//  CJ[]     3 2 1 0	
union equivs { unsigned int J; unsigned char CJ[4]; };
union equivs eq;

typedef struct vac_gage {
	int thread_status;
	unsigned serBaud;
	unsigned serFlags;	
	int status;
	unsign fd;
	float gage;
	float set_point;
	float deadband;
} vac;

	vac vgage;				// global structure!!
	

typedef struct spi_control {

	int thread_status;
	FILE *fp;
	
	unsigned cs_pll;
	int pll_flag;
	unsigned int PLL[6];	// reg0-reg5 !write in reverse order!
	int frequency;			// in KHz

	unsigned cs_att;
	int att_flag;
	unsigned int ATT;
	float attenuation;				// in dB

	unsigned cs_adc;
	int adc_flag;
	unsigned char adc_reg[6];
	float ADC[6];			// raw values from MAX11254
	float ammeter;			// AIN0P
	float power_fwrd;		// AIN1P
	float power_rev;		// AIN2P
	float power_sniffer;	// AIN3P
	float adc_test;			// AIN4P
	float adc_spare;		// AIN5P

	unsigned cs_dac;
	int dac_flag;
	unsigned char dac_cmd[4];
	unsigned int DAC[4];	// raw values sent to MAX5134
	float cathode;
	float cathode_gain;
	float einzel1;
	float einzel1_gain;
	float einzel2;
	float einzel2_gain;	
	float dac_test;			// drives AIN4P
	float dac_test_gain;
} spi_c;

	spi_c spi;				// global structure!!
	

int   getch(int ms);		// routine calling definitions
void* spi_thread( void* s );
void* gpio_test( );
void* set_att( );
void* set_pll( );
void* read_adc( );
void* write_dac( );
void* vac_thread( void* v );
int   read_gage( );
int   serial_test( );
int   plotit( );
int   help_menu( );

unsigned spid_adc, spid_dac, spid_att1, spid_T26, spid_pll, count;
unsigned ser_tty, serBaud=9600, serFlags=0;
unsigned spiBaud=100000;
int x, rc1, rc2, count=0;
unsigned char c;

int main(void)
{
	if (gpioInitialise() < 0)
	{
		printf( "gpio did not initiallize");
		return(1);
	}
	gpioSetMode(H2_in, PI_OUTPUT); 
 	gpioSetMode(H2_out, PI_OUTPUT); 
 	gpioSetMode(Vac, PI_OUTPUT); 
 	gpioSetMode(Vac_pump, PI_OUTPUT);

	spi.cs_adc = spiOpen(0, spiBaud, 0);	//CE0   08   24	 T11	
	spi.cs_dac = spiOpen(1, spiBaud, 0);	//CE1   07   26  T12
	spi.cs_pll = spiOpen(0, spiBaud, 256);	//ce0   18   12	(p1-36->p1-11) 
	spi.cs_att = spiOpen(1, spiBaud, 256);	//ce1   17   11	(p1-37->p1-12)


    pthread_t thread1, thread2;		

	spi.thread_status = not_running;
	
	vgage.thread_status = not_running;
	vgage.status=0;
	
	help_menu();
	
	do 
	{
		x = getch(500);
  
		if (x == 'q') 							//no threads running, just exit
		{
			if (spi.thread_status == running) 	//kill any running threads
			{
				spi.thread_status = kill;
			}
			if (vgage.thread_status == running) 
			{
				vgage.thread_status = kill;
			}
		}
		if (x == 'a')		// puff of H2
		{
			gpioWrite (H2_in, 1);	usleep(500000);
			gpioWrite (H2_in,  0);	usleep(10000);
			gpioWrite (H2_out,1);	usleep(40000);
			gpioWrite (H2_out, 0);
		}
		if (x == 's')		// suck of vac
		{
			gpioWrite (Vac_pump, 1);	usleep(500000);
			gpioWrite (Vac_pump, 0);	usleep(10000);
			gpioWrite (Vac,1);			usleep(40000);
			gpioWrite (Vac, 0);
		}
		if (x == 'd')		// evacuate
		{
			gpioWrite (Vac_pump, 1);
			gpioWrite (Vac,      1);
		}
		if (x == 'f')		// Vacuum off
		{
			gpioWrite (Vac_pump, 0);
			gpioWrite (Vac,      0);
		}
		if (x == 't')		// Read rs-232 port
		{
			read_gage( );

		}
		if (x == 'g')		// gnuplot
		{
			plotit( );

		}
		if (x == 'h')		// print help menu
		{
			help_menu( );

		}


		if (x == 'w' && spi.thread_status == not_running) // spi
		{
			spi.thread_status = running;		// 	
			rc1=pthread_create( &thread1, NULL, &spi_thread, &spi);
			if( rc1 ) 
			{
				printf( "Failed to create spi thread: %d\n", rc1);
			}
		}

		if (x == 'p' && vgage.thread_status == not_running) //gage
		{

			vgage.thread_status = running;		// 
			vgage.set_point = 10.0;
			vgage.deadband = 2.0;					//2.65 regulates at 3.1T
		
			rc2=pthread_create( &thread2, NULL, &vac_thread, &vgage);
			if( rc2 ) 
			{
				printf( "Failed to create gage thread: %d\n", rc2);
			}
		}
		system("clear");




		printf("\t vac.gage=%.4f\n",vgage.gage);
		c=x;
		printf( "\t %d  x=%c \n", count, c);
		count = count+1;
		usleep(500000);

  
}while (x != 'z');
		
	if (vgage.status == 1) 
	{
		close(vgage.fd);
	}	
	if (spi.thread_status == running) 	//kill any running threads
	{
		spi.thread_status = kill;
	}
	if (vgage.thread_status == running) 
	{
		vgage.thread_status = kill;
	}

	
	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);
    pthread_exit(NULL);
}



///////////////////////////////////////////
void* spi_thread( void* ss )
{
	int count;
	spi_c* s = (spi_c*) ss;

	FILE *fp;
	fp = fopen( "spi.dat", "w+" );
	s->fp = fp;
	count=0;
	
//	gpio_test( );

/*
	s->att_flag = init;
	s->attenuation = -16;
	set_att();


	s->pll_flag = init;
	s->frequency = 1000;
	set_pll();
	gpioWrite (RF_en, 1) ;
PLL:
	if (s->thread_status == kill ) { goto CLOSE; }
	s->pll_flag = step;	
	set_pll();	
	usleep(1000);
	goto PLL;



	s->dac_flag = init;
	write_dac();


DAC:
	if (s->thread_status == kill ) { goto CLOSE; }
	s->cathode = .1;
	s->einzel1 = .2;
	s->einzel2 = .3;
	s->dac_test = .4;
	s->dac_flag = step;
	write_dac();
	usleep(100);

	s->cathode = 1.1;
	s->einzel1 = 1.2;
	s->einzel2 = 1.3;
	s->dac_test = 1.4;
	s->dac_flag = step;
	write_dac();
	usleep(200);	
	goto DAC;
	
	
//	goto CLOSE;


*/
	
	s->adc_flag = init;
	read_adc();	
	fprintf( fp, " I %d, AM=%.4f, FP=%.4f, RP=%.4f, SP=%.4f, TST=%.4f\n", 
		count, spi.ammeter,spi.power_fwrd,spi.power_rev,
		spi.power_sniffer,spi.adc_test);

LOOP:
	if (s->thread_status == kill ) { goto CLOSE; }
	count = count + 1;
	s->adc_flag = step;
	read_adc();
//	fprintf( fp, " S %d, AM=%.4f, FP=%.4f, RP=%.4f, SP=%.4f, TST=%.4f\n",  
//		count, spi.ammeter,spi.power_fwrd,spi.power_rev,
//		spi.power_sniffer,spi.adc_test);
	usleep(500);

	goto LOOP;
	

CLOSE:
							// turn everything off before exit
	s->cathode = 0;
	s->einzel1 = 0;
	s->einzel2 = 0;
	s->dac_test = 0;
	write_dac();
	gpioWrite (RF_en, 0) ;

	
	s->thread_status = not_running ;
	fclose(fp);
	
	pthread_exit(NULL);
}

///////////////////////////////////////////
void* gpio_test( )
{
	// 7=dac_cs(26), 8=adc_cs(24), 9=MISO(21), 10=MOSI(19), 11=s_clk(23), 

	unsigned char buff[4];
	buff[0] = 0x55;
	buff[1] = 0xff;
	buff[2] = 0xcc;
	gpioSetMode(4, PI_OUTPUT); 	


loop:
	gpioWrite(4, 0);
	usleep(100);                                           
	gpioWrite(4, 1); 
	usleep(100); 
		spiWrite(spi.cs_dac, buff, 2);
		spiWrite(spi.cs_adc, buff, 3);
	goto loop;

	
	return 0;
}


///////////////////////////////////////////
void* set_att( )
{

	unsigned char buff[4];

	
	if( spi.att_flag == idle ) {
		goto END;
	}
	
	
	eq.J = abs(spi.attenuation)*4;	// attenuation in quarter dB steps
	buff[0] = eq.CJ[3];
	spiWrite(spi.cs_att, buff, 1);

END:	
	return 0;
}

///////////////////////////////////////////
void* set_pll( )
{
	unsigned char buff[4];
	unsigned int N;
	int n;
	
	
	if( spi.pll_flag == idle ) {
		goto END;
	}
	
	if( spi.pll_flag == init ) {
/*		N = spi.frequency/625 *2;			//  freq is frequency in KHz
		spi.PLL[0] = 1<<31 | N<<15;
		spi.PLL[1] = 0x8000ce21;
		spi.PLL[2] = 0x00040142;
		spi.PLL[3] = 0x0000000b;
		spi.PLL[4] = 0x6090d03c;	  
		spi.PLL[5] = 0x00400005;
*/		
		spi.PLL[0] = 0x00d38070;	//example output=2117.8MHz	
		spi.PLL[1] = 0x200080c9;
		spi.PLL[2] = 0x00004042;
		spi.PLL[3] = 0x0000000b;
		spi.PLL[4] = 0x609c803c;	  
		spi.PLL[5] = 0x00400005;
	
							// must be written in this order
		for ( n=5; n>-1; n--) {
			eq.J = spi.PLL[n];
			buff[0] = eq.CJ[3];
			buff[1] = eq.CJ[2];
			buff[2] = eq.CJ[1];
			buff[3] = eq.CJ[0];
			fprintf( spi.fp, " n= %d, spi= %x  buff= %x, %x, %x, %x\n",
		                      n, eq.J, buff[0], buff[1], buff[2], buff[3] );
			spiWrite(spi.cs_pll, buff, 4);
		}
		usleep(20000);	
		for ( n=5; n>-1; n--) {
			eq.J = spi.PLL[n];
			buff[0] = eq.CJ[3];
			buff[1] = eq.CJ[2];
			buff[2] = eq.CJ[1];
			buff[3] = eq.CJ[0];
			spiWrite(spi.cs_pll, buff, 4); 
		}	
		goto END;
	}
	
	if( spi.pll_flag == step ) {
		n=8;
		N = spi.frequency/625 *2;
		spi.PLL[0] = 0x80000000 | N<<15;
		eq.J = spi.PLL[0];
		buff[0] = eq.CJ[3];
		buff[1] = eq.CJ[2];
		buff[2] = eq.CJ[1];
		buff[3] = eq.CJ[0];
		fprintf( spi.fp, " n= %d, spi= %x  buff= %x, %x, %x, %x\n",
					n, eq.J, buff[0], buff[1], buff[2], buff[3] );
		spiWrite(spi.cs_pll, buff, 4);
	}

END:

	return 0;
}



///////////////////////////////////////////
void* read_adc( )
{
		
	unsigned char buff[4];
	int n;

	if( spi.adc_flag == idle ) {
		goto END;
	}
	
	if( spi.adc_flag == init ) {
		spi.adc_reg[0] = 0xd1;
		spi.adc_reg[1] = 0xd3;
		spi.adc_reg[2] = 0xd5;
		spi.adc_reg[3] = 0xd7;
		spi.adc_reg[4] = 0xd9;
		spi.adc_reg[5] = 0xdb;
/*			
		buff[0] = 0x00;		// DELAY
		buff[1] = 0x00;
			memcpy(buff,&ADC_init,2);
			spiWrite(spi.cs_adc, buff, 2);
 
		buff[0] = 0xc6;		// CTRL3 
			spiWrite(spi.cs_adc, buff, 1);

		buff[0] = 0xc6;		// CHMAP0 
			spiWrite(spi.cs_adc, buff, 1);

		buff[0] = 0xcc;		// CHMAP1 
			spiWrite(spi.cs_adc, buff, 1);

		buff[0] = 0xc4;		// CTRL2 
			spiWrite(spi.cs_adc, buff, 1);
*/


/*SEQ = D0					0    8
 * mux=0 					000
 * mode=1					   0 1
 * gpodren=0				      0
 * mdren=0					       0
 * rdyben=0					        0
*/
		buff[0] = 0xd0;		//SEQ  command
		buff[1] = 0x08;		//SEQ  data
			spiWrite(spi.cs_adc, buff, 2);
			usleep(100); 
			
/*CTL1 =C2                   0    E
 * perform self calibration	00
 * powerdown =NOP			  00
 * unipolar					    1
 * format = offset binary 	     1
 * Scycle = single cycle		  1
 * Contsc = single cycle           0
*/ 
		buff[0] = 0xc2;		// CTRL1
		buff[1] = 0x0e;		// CTRL1 data
			spiWrite(spi.cs_adc, buff, 2);
			usleep(100); 
			
/*Convert					B    E
 * convert code				1011
 * sample rate=64000sps		    1110
*/
		buff[0] = 0xbe;		// Convert at 64000sps
			spiWrite(spi.cs_adc, buff, 1);
			usleep(100); 			

		goto END;
	}

ADC:	
	if( spi.adc_flag == step ) {
		buff[0] = 0xbe;					// Convert! (64000sps)
		spiWrite(spi.cs_adc, buff, 1);
		usleep(100); 		
				
		for ( n=0; n<1; n++) {
			buff[0] = spi.adc_reg[n];	//select a channel to read
			spiWrite(spi.cs_adc, buff, 4);
			eq.CJ[2]=buff[1];
			eq.CJ[1]=buff[2];
			eq.CJ[0]=buff[3];
			spi.ADC[n] = (float) eq.J / 16777215 * Vref;
			fprintf( spi.fp, " acd %d, = %0x,  %f\n",  n, eq.J, spi.ADC[n]);
			usleep(100);
		}
//		goto END;
	}	
	goto ADC;


END:
	return 0;
}


///////////////////////////////////////////
void* write_dac( )
{
	unsigned char buff[3];
	unsigned short I[4];
	int n;

	if( spi.dac_flag == idle ) {
		goto END;
	}
	
	if( spi.dac_flag == init ) {
		spi.cathode = 0;
		spi.cathode_gain = 1 /Vref * dac_bits;
		spi.einzel1 = 0;
		spi.einzel1_gain = 1 /Vref * dac_bits;
		spi.einzel2 = 0;
		spi.einzel2_gain = 1 /Vref * dac_bits;
		spi.dac_test = 0;
		spi.dac_test_gain = 1 /Vref * dac_bits;	

		spi.dac_cmd[0] = 0x31;		// inialize write thru command
		spi.dac_cmd[1] = 0x32;		// for DACs 1-4
		spi.dac_cmd[2] = 0x34;
		spi.dac_cmd[3] = 0x38;

		buff[0] = 0x02;				// software clear
		buff[1] = 0x00;
		buff[2] = 0x00;
		spiWrite(spi.cs_dac, buff, 3);

		goto END;
	}
	
	if( spi.dac_flag == step ) {

		I[0] = spi.cathode * spi.cathode_gain;
		I[1] = spi.einzel1 * spi.einzel1_gain;
		I[2] = spi.einzel2 * spi.einzel2_gain;
		I[3] = spi.dac_test * spi.dac_test_gain;
		fprintf( spi.fp, " dac= %x,  %x,  %x,  %x\n",I[0], I[1], I[2], I[3]  );	

		for( n=0; n<4; n++) {
			eq.J = I[n];
			buff[0] = spi.dac_cmd[n];		// send command first
			buff[1] = eq.CJ[1];
			buff[2] = eq.CJ[0];
		
//			fprintf( spi.fp, "buff= %x, %x, %x\n", buff[0], buff[1], buff[2] );
			spiWrite(spi.cs_dac, buff, 3); 
			usleep(100);
		}
	}
END:

//union equiv { unsigned int J; unsigned char CJ[4] };
//union equiv eq;
//	eq.J = 0x11223344;
//	fprintf ( spi.fp, "J= %x,   CJ= %x, %x, %x, %x \n",
//		eq.J, eq.CJ[0], eq.CJ[1], eq.CJ[2], eq.CJ[3] );


	return 0;
}





///////////////////////////////////////////
void* vac_thread( void* vv )
{
	vac* v = (vac*) vv; 				// cast the void* to struct type
  
	int h2_in_delay=10000, h2_out_delay=5000, pump_delay=100000, vac_delay=40000;
	int dwell_time=800000, count=0;
	FILE *fp;
	float hi=v->set_point+v->deadband/2,0=v->set_point-v->deadband/2;

	fp = fopen( "gage.dat", "w+" );

Measure: 
	if (v->thread_status == kill ) { goto Exit; }

	usleep(dwell_time);	
	read_gage();
	v->gage = vgage.gage;
	count = count + 1;
	fprintf( fp, "%d, %.4f\n", count, v->gage);



	if( v->gage > hi) 
	{									//too 1, pulse vac
		vac_delay = 500000 * (v->gage-hi) / hi;
		if( vac_delay > 2000 ) 
		{
			gpioWrite (Vac_pump, 1);	usleep(pump_delay);
			gpioWrite (Vac_pump,  0);	usleep(10000);
			gpioWrite (Vac,1);			usleep(vac_delay);
			gpioWrite (Vac, 0);
		}	
		goto Measure;
	}
		
	if( v->gage < 0) 
	{									//too 0, pulse H2
		h2_out_delay = 20000 * (0 - v->gage) / 0;
		if( h2_out_delay > 2000 )
		{
			gpioWrite (H2_in, 1);		usleep(h2_in_delay) ;
			gpioWrite (H2_in,  0);		usleep(10000) ;
			gpioWrite (H2_out,1);		usleep(h2_out_delay) ;
			gpioWrite (H2_out, 0) ;
		}
		goto Measure;
	}
	goto Measure;

Exit:									// turn everything off before exit
	gpioWrite (Vac_pump, 0);
	gpioWrite (Vac, 0);
	gpioWrite (H2_in,  0);	
	gpioWrite (H2_out, 0);		
	v->thread_status = not_running ;
	fclose(fp);
	pthread_exit(NULL);
}



///////////////////////////////////////////
int read_gage( )
{
	int i, len, j ;
	const char* device;
	char c[20] = "@253PR4?;FF";					//normal request
//	char c[20] = "@253ACK1.234E-1;FF";			// loopback test
	char d[20] = {'b','i','l','b','o','\0'};
	char e[10] ;
	
	if( vgage.status == 0)		// setup the gage
	{		
		vgage.gage = 0.0;
		vgage.status = 1;
		vgage.serBaud = 9600;
		vgage.serFlags = 9600;
		device = "/dev/serial0";   // for usb, check /dev/serial/by-id/*

		if( ( vgage.fd = serOpen ( device , vgage.serBaud, vgage.serFlags )) <0) 
		{
			perror("serial device not opened\n");
		}
	}


	len = strlen(c);
	write (vgage.fd, c, len) ;
//	printf( " write gate=%s\n", c);
    usleep(300000) ;
    
    if( serialDataAvail( vgage.fd ) )
    { 
		serRead ( vgage.fd, d, 20) ;
		printf( " read=%s\n", d );
		if (d[4] == 'A' )
		{
			j=-1;
			for ( i=7; i<20; i++ )
			{
				j=j+1;
				if ( d[i] != ';' ) {
					e[j] = d[i];
				}
				else {
					e[j] = '\0';
					break;
				}
			}
		    vgage.gage = atof(e);
//			printf ( " string=%s   %.4f\n", e,vgage.gage );
		}
	}

	return 0;
}



///////////////////////////////////////////
int plotit()
{
	
	gnuplot_ctrl    *   h1;

//    printf("*** example of gnuplot control through C ***\n") ;
    h1 = gnuplot_init() ;

    gnuplot_resetplot(h1) ;
    gnuplot_cmd(h1, "set xlabel 'time' ") ;
    gnuplot_cmd(h1, "plot 'gage.dat' using 1:2 with lines") ;
 
    sleep (10 );
    gnuplot_close(h1) ;
    return 0 ;
}

///////////////////////////////////////////
int getch(int ms)
{
    int ret;
    struct termios oldt, newt;
    struct pollfd pfds[1];

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    pfds[0].fd = STDIN_FILENO;
    pfds[0].events = POLLIN;
    poll(pfds, 1, ms);
    if (pfds[0].revents & POLLIN) {
        char ch;
        read(STDIN_FILENO, &ch, 1);
        ret = ch;
    } else {
        ret = 0;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ret;
}


///////////////////////////////////////////
int help_menu()
{
	printf( "a = puff of H2\n" );
	printf( "s = suck of vacuum\n" );
	printf( "d = vacuum\n" );
	printf( "f = valves off\n" );
	printf( "w = start spi thread\n" );	
	printf( "t = read pressure gage\n" );
	printf( "v = voltage pulse on DAC-pin 10\n" );
	printf( "p = regulate vacuum pressure\n" );
	printf( "g = gnuplot gage.dat\n" );
	printf( "q = kill vacuum or pulse process\n" );
	printf( "z = exit program\n" );	
	
	sleep(2);
	return 0 ;
}

/*	Old read adc code
			adcOut = 0;
			adcOut = buff[1];			//shift MSByte over one byte
			adcOut = adcOut << 8;
			adcOut = adcOut & buff[2];	//shift top 2 bytes over
			adcOut = adcOut << 8;
			adcOut = adcOut & buff[3];	//add in the LSByte		 	
			spi.ADC[n] = (float) adcOut / 16777215 * Vref;
			fprintf( spi.fp, "      %d,  %0x,  %f\n",  n, adcOut, spi.ADC[n]);
*/

