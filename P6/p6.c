#include <stdio.h>		//20180521.1738
#include <stdlib.h>
#include <stdint.h>  //int8_t,int16_t,int32_t,uint8_t,uint16_t,uint32_t
#include <termios.h>
#include <poll.h>
#include <string.h>
#include <errno.h>



#include <pthread.h>
#include <sys/types.h>  // needed for getpid()
#include <unistd.h>     // needed for getpid()

#include <../gnuplot/gnuplot_i.h>

#include <pigpio.h>		// http://abyz.co.uk/rpi/pigpio/cif.html

#include "max2871.h"	

/*
* gcc -Wall -o p6 p6.c -lpigpio -lthread
* sudo ./p6
* gcc -Wall -ggdb -o p6 p6.c -lpigpio -lthread
* sudo gdb ./p6
*/



#define Vref		2.44	// vref is common for adc & dac
#define dac_bits	65535
#define adc_bits	16777215



#define RF_en		6		// BCM GPIO
#define H2_in		4		// RLY_1 moved from p1-11 -> p1-7
#define H2_out		22		// RLY_2 moved from p1-12 -> p1-15
#define Vac			27
#define Vac_pump	23
#define att_cs		18		// needed to toggle pe43711 data into latch

#define not_running	0		//threadstatus
#define running		1		//threadstatus
#define kill		2		//threadstatus
#define inuse		1		//ADC status
#define available	0		//ADC status
#define idle		0
#define init		1
#define step		2
#define run			3
#define dev_status	4
#define init_mode1	5
#define init_mode2	6

#define  forward	0		// AIN0P
#define  reflected	1		// AIN1P
#define  sniffer	2		// AIN2P
#define  ammeter	3		// AIN3P
#define  adc_test	4		// AIN4P
#define  adc_spare	5		// AIN5P


typedef struct vac_gage {
	int thread_status;
	unsigned serBaud;
	unsigned serFlags;	
	int status;
	unsigned fd;
	float gage;
	float set_point;
	float deadband;
} vac;

	vac vgage;				// global structure!!
	

typedef struct spi_control {

	int thread_status;
	FILE *fp;
	unsigned char cmd;
	
	uint32_t pll_fd;
	uint32_t pll_reg[6];	// reg0-reg5 !write in reverse order!
	int frequency;			// in KHz

	uint32_t att_fd;
	float attenuation;				// in dB

	uint32_t adc_fd;
	unsigned char adc_reg[6];
	unsigned char adc_status[4];
	float ADC[6];			// adc voltage values

	uint32_t dac_fd;
	unsigned char dac_cmd[4];
	uint32_t DAC[4];	// raw values sent to MAX5134
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
void* spi_thread( void *s );
void* d_a_test( );
void* d_a_bob( );
void* ce_test( );
void* att_set( int cmd_flag );
void* pll_set( int cmd_flag );
void* pll_write( uint32_t data );
void* adc_read( int cmd_flag );
void* dac_write( int cmd_flag );
void* vac_thread( void *v );
int   gage_read( );
int   serial_test( );
int   plotit( );
int   help_menu( );

uint32_t spid_adc, spid_dac, spid_att1, spid_T26, spid_pll;
uint32_t ser_tty, serBaud=9600, serFlags=0;
uint32_t spiBaud=8000000;
// MAX2871  PLL 20MHz
// MAX5134  DAC 30MHz
// MAX11254 ADC  8MHz
// PE43711  ATT 10MHz

int x, rc1, rc2, count=0;
unsigned char c;
char buff[4], buff_rx[4];
union equivs { uint32_t J; unsigned char CJ[4]; } eq;

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
  	gpioSetMode(RF_en, PI_OUTPUT);


	spi.adc_fd = spiOpen(0, spiBaud, 0);	//CE0   08   24	 T11	
	spi.dac_fd = spiOpen(1, spiBaud, 0);	//CE1   07   26  T12
	spi.att_fd = spiOpen(0, spiBaud, 256);	//ce0   18   12	(p1-37->p1-12) 
	gpioSetMode(att_cs, PI_OUTPUT);
	spi.pll_fd = spiOpen(1, spiBaud, 256);	//ce1   17   11	(p1-36->p1-11)

    pthread_t *thread_s, *thread_v;		

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
			gage_read( );

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
			spi.thread_status = running;	
			thread_s = gpioStartThread(spi_thread, &spi );	
		}

		if (x == 'p' && vgage.thread_status == not_running) //gage
		{

			vgage.thread_status = running;		// 
			vgage.set_point = 10.0;
			vgage.deadband = 2.0;					//2.65 regulates at 3.1T
		
			thread_v = gpioStartThread( vac_thread, &vgage );
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
		vgage.status = 0;
	}	
	if (spi.thread_status == running) 	//kill any running threads
	{
		spi.thread_status = kill;
	}
	if (vgage.thread_status == running) 
	{
		vgage.thread_status = kill;
	}


	gpioStopThread( thread_s );
	gpioStopThread( thread_v );
    gpioTerminate();
}

///////////////////////////////////////////
void *spi_thread( void *ss )
{
	int count;
//	spi_c* s = (spi_c*) ss;


	FILE *fp;
	fp = fopen( "spi.dat", "w+" );
	spi.fp = fp;
	count=0;
	
	
// g(pio), t(atten), p(ll), d(ac), a(dc), (s)pi (c)hip enable (b)ob
	spi.cmd='p';  
	
	if(spi.cmd=='s')
	{
		d_a_test();
	} 
	else if(spi.cmd=='b')
	{
		d_a_bob();
	} 
	else if(spi.cmd=='a')		// use DAC to debug ADC
	{
		dac_write( init );
		adc_read( init_mode2 );	
		adc_read( dev_status );
		count = 0;
				
ADC:	//write dac_test waveform to adc
		if (spi.thread_status == kill ) { goto CLOSE; }
		count = count + 1;
		spi.dac_test = .4;
		dac_write( step );			
		
		fprintf( fp, " %d, vin=%.4f  ", count, spi.dac_test);	
		adc_read( step );
		
		spi.dac_test = 1.4;
		dac_write( step );			
		
		fprintf( fp, " %d, vin=%.4f  ", count, spi.dac_test);	
		adc_read( step );
//		fprintf( fp, " S %d, AM=%.4f, FP=%.4f, RP=%.4f, SP=%.4f, TST=%.4f\n",  
//		count, spi.ADC[ammeter],spi.ADC[forward],spi.ADC[reflected],
//		spi.ADC[sniffer],spi.ADC[adc_test]);

		goto ADC;
	}
	else if(spi.cmd=='c')
	{
		ce_test();
	} 
	else if(spi.cmd=='g')
	{
GPIO:
		if (spi.thread_status == kill ) { goto CLOSE; }
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
		usleep(100000);
		goto GPIO;
	}	
	else if(spi.cmd=='t')
	{
		spi.attenuation = -16;
		att_set( init );
	} 
	
	
	else if(spi.cmd=='p')
	{	
	
		dac_write( init );				//write .5v to dac_adc_test
		eq.J = 0.5 /Vref * dac_bits;	//just to insure adc is working
		buff[0] = 0x38;
		buff[1] = eq.CJ[1];
		buff[2] = eq.CJ[0];
		spiWrite(spi.dac_fd, buff, 3);
 			
		spi.frequency = 50;				//50MHz, maybe...
		pll_set( init );
	
		spi.attenuation = -16;
		att_set( init );
	
		adc_read( init_mode1 );
		gpioWrite (RF_en, 1) ;	

PLL:
		spi.attenuation = 0;
		att_set( init );
		if (spi.thread_status == kill ) { goto CLOSE; }
		spi.frequency = 50;				//50MHz
		pll_set( step );	
		usleep(100);
		adc_read( step );
		fprintf( fp, " freq=%d, power out = %.4f\n",
					spi.frequency, spi.ADC[forward]);
		usleep(1000000);
		
		spi.attenuation = 16;
		att_set( init );
		spi.frequency = 100;			//100MHz
		pll_set( step );			
		usleep(100);
		adc_read( step );
		fprintf( fp, " freq=%d, power out = %.4f\n",
					spi.frequency, spi.ADC[forward]);
		usleep(1000000);
		goto PLL;
	}
	
	else if(spi.cmd=='d') {
		dac_write( init );
DAC:
		if (spi.thread_status == kill ) { goto CLOSE; }
		spi.cathode = .1;
		spi.einzel1 = .2;
		spi.einzel2 = .3;
		spi.dac_test = .4;
		dac_write( step );
		usleep(50);

		spi.cathode = 2.1;
		spi.einzel1 = 2.2;
		spi.einzel2 = 2.3;
		spi.dac_test = 2.4;
		dac_write( step );
		usleep(50);	
		goto DAC;
	}

CLOSE:							// turn everything off before exit
	spi.cathode = 0;
	spi.einzel1 = 0;
	spi.einzel2 = 0;
	spi.dac_test = 0;
	dac_write( step );
	gpioWrite (RF_en, 0) ;
	
	spi.thread_status = not_running ;
	fclose(fp);
	
	pthread_exit(NULL);
}

///////////////////////////////////////////	
void* pll_write( uint32_t data ) {
	char buff[4];
	union equivs { uint32_t J; unsigned char CJ[4]; } eq;
	eq.J = data;
	buff[0] = eq.CJ[3];
	buff[1] = eq.CJ[2];
	buff[2] = eq.CJ[1];
	buff[3] = eq.CJ[0];

	spiWrite(spi.pll_fd, buff, 4);
	fprintf( spi.fp, " spi_wr = %x  buff= %x, %x, %x, %x\n",
					eq.J, buff[0], buff[1], buff[2], buff[3] );	
	return 0;
}

///////////////////////////////////////////
void* d_a_test( )
{
	union equivs { uint32_t J; unsigned char CJ[4]; } eq;
	char buff[4];
	float vin, vout;
	
	dac_write( init );
	
	adc_read( dev_status );

	adc_read( init_mode1 );

	adc_read( dev_status );


	usleep(1000); 		

LOOP:
		if (spi.thread_status == kill ) { goto CLOSE; }

		vin = 0.1;
		eq.J = vin /Vref * dac_bits;
		buff[0] = 0x38;					// write through to DAC3
		buff[1] = eq.CJ[1];
		buff[2] = eq.CJ[0];
		spiWrite(spi.dac_fd, buff, 3); 
		usleep(10);

		buff[0] = 0xbe;					// Convert! (6400sps)
		spiWrite(spi.pll_fd, buff, 1);
		usleep(10); 		


		buff[0] = spi.adc_reg[adc_test];		//select channel 4  DAC_ADC_test
		buff[1] = 0x00;
		buff[2] = 0x00;
		buff[3] = 0x00;
		spiXfer(spi.pll_fd, buff, buff_rx, 4);
		eq.CJ[0] = buff_rx[3];
		eq.CJ[1] = buff_rx[2];
		eq.CJ[2] = buff_rx[1];
		eq.CJ[3] = buff_rx[0];
		vout = (float) eq.J / adc_bits * Vref;
		fprintf( spi.fp, " vin=%f, vout = %.4f\n",  vin, vout);
		usleep(200);

		adc_read( dev_status );


		vin = 1.1;
		eq.J = vin /Vref * dac_bits;
		buff[0] = 0x38;					// write through to DAC3
		buff[1] = eq.CJ[1];
		buff[2] = eq.CJ[0];
		spiWrite(spi.dac_fd, buff, 3); 
		usleep(10);

		buff[0] = 0xbe;					// Convert! (64000sps)
		spiWrite(spi.pll_fd, buff, 1);
		usleep(10); 		

		buff[0] = spi.adc_reg[adc_test];//select channel 4  DAC_ADC_test
		spiXfer(spi.pll_fd, buff, buff_rx, 4);
		eq.CJ[0] = buff_rx[3];
		eq.CJ[1] = buff_rx[2];
		eq.CJ[2] = buff_rx[1];
		eq.CJ[3] = buff_rx[0];
		vout = (float) eq.J / 16777215 * Vref;
		fprintf( spi.fp, " vin=%f, vout = %.4f\n",  vin, vout);
		usleep(200);

		adc_read( dev_status );

		goto LOOP;
CLOSE:
	return 0;
}
///////////////////////////////////////////
void* d_a_bob( )
{
//	union equivs { uint32_t J; unsigned char CJ[4]; } eq;
	char buff[4];
//	float vin, vout;
//	int n;
	
//1:	
	adc_read( dev_status );
	dac_write( init );
		spi.adc_reg[0] = 0xdd;
		spi.adc_reg[1] = 0xdf;
		spi.adc_reg[2] = 0xe1;
		spi.adc_reg[3] = 0xe3;
		spi.adc_reg[4] = 0xe5;
		spi.adc_reg[5] = 0xe7;
	
// Initialization stuff



		buff[0] = 0xd0;		// ==> SEQ register
		buff[1] = 0x0a;		//select seq mod 2, enable delay
		spiWrite(spi.adc_fd, buff, 2);
		usleep(1000); 
		
		buff[0] = 0xca;		// ==> Delay register
		buff[1] = 0xf0;		//select chan 1, enable delay
		buff[2] = 0x00; 
		spiWrite(spi.adc_fd, buff, 2);
		usleep(1000); 
				
		buff[0] = 0xce;		// ==> CHMAP0
		buff[1] = 0x0e;		// chan 3
		buff[2] = 0x0a;		// chan 2
		buff[3] = 0x06;		// chan 1
		spiWrite(spi.adc_fd, buff, 4);
		usleep(100); 
		
		buff[0] = 0xcc;		// ==> CHMAP1
		buff[1] = 0x1a;		// chan 6
		buff[2] = 0x16;		// chan 5
		buff[3] = 0x12;		// chan 4
		spiWrite(spi.adc_fd, buff, 4);
		usleep(100); 
		
		buff[0] = 0xc2;		// ==> CTRL1
		buff[1] = 0x2e;		// CTRL1 data
		spiWrite(spi.adc_fd, buff, 2);
		usleep(100); 
		
		adc_read( dev_status );


LOOP:
		if (spi.thread_status == kill ) { goto CLOSE; }
	
		spi.dac_test = .5;
		dac_write( step );
		usleep(1000); 

		adc_read( step );
		usleep(1000); 

		spi.dac_test = 1.0;
		dac_write( step );

		adc_read( step );
																						
		adc_read( dev_status );

		goto LOOP;
CLOSE:
	
	return 0;
}

///////////////////////////////////////////
void* ce_test( )
{
	
	char buff[4], buff_rx[4];	
	
	buff[0] = 0x55;
	buff_rx[0] = 0x99; 
LOOP:
	if (spi.thread_status == kill ) { goto CLOSE; }
	spiWrite(spi.dac_fd, buff, 1);
	usleep(20);
	spiXfer(spi.adc_fd, buff, buff_rx, 1);
	usleep(20);
	spiWrite(spi.pll_fd, buff, 1);
	usleep(20);
	spiWrite(spi.att_fd, buff, 1);
	usleep(20);
	goto LOOP;

CLOSE:
	
	return 0;
}

///////////////////////////////////////////
void* att_set( int cmd_flag )
{
	union equivs { uint32_t J; unsigned char CJ[4]; } eq;
	char buff[4];
		
	if( cmd_flag == idle ) {
		goto END;
	}
	
	eq.J = abs(spi.attenuation)*4;	// attenuation in quarter dB steps
	buff[0] = eq.CJ[3];
	spiWrite(spi.att_fd, buff, 1);
	gpioWrite (att_cs,0);				// clock shift reg data into latch 
	gpioWrite (att_cs,1);

END:	
	return 0;
}

///////////////////////////////////////////
void* pll_set( int cmd_flag )
{
//	char buf8[8], buf8_rx[8];
	int i, n;
	uint32_t Fref,DivA,N;
//	uint32_t RFoutA;
	uint32_t R4;
	uint32_t N_clear, DivA_clear;	
	
	if( cmd_flag == init ) {
		
		spi.pll_reg[0] = 0x0 | EN_INT;
		spi.pll_reg[0] |= N_SET;
		spi.pll_reg[0] |= F_SET;
		spi.pll_reg[0] |= REG_0;
		spi.pll_reg[1] |= CPL;
		spi.pll_reg[1] |= CPT;
		spi.pll_reg[1] |= PHASE;
		spi.pll_reg[1] |= M_SET;
		spi.pll_reg[1] |= REG_1;
		spi.pll_reg[2] |= LDS;
		spi.pll_reg[2] |= SDN;
		spi.pll_reg[2] |= MUX_2;
		spi.pll_reg[2] |= R_DIV;
		spi.pll_reg[2] |= REG4DB;
		spi.pll_reg[2] |= CP_SET;
		spi.pll_reg[2] |= LDF;
		spi.pll_reg[2] |= LDP;
		spi.pll_reg[2] |= PDP;
		spi.pll_reg[2] |= SHDN;
		spi.pll_reg[2] |= RST;
		spi.pll_reg[2] |= REG_2;
		spi.pll_reg[3] |= VCO_SET;
		spi.pll_reg[3] |= VAS_SHDN;
		spi.pll_reg[3] |= VAS_TEMP;
		spi.pll_reg[3] |= CSM;
		spi.pll_reg[3] |= MUTEDEL;
		spi.pll_reg[3] |= CDM;
		spi.pll_reg[3] |= CDIV;
		spi.pll_reg[3] |= REG_3;
		spi.pll_reg[4] |= REG4HEAD;
		spi.pll_reg[4] |= SDLDO;
		spi.pll_reg[4] |= SDDIV;
		spi.pll_reg[4] |= SDREF;
		spi.pll_reg[4] |= BS_MSB;
		spi.pll_reg[4] |= FB;
		spi.pll_reg[4] |= DIVA;
		spi.pll_reg[4] |= BS_LSB;
		spi.pll_reg[4] |= SDVCO;
		spi.pll_reg[4] |= MTLD;
		spi.pll_reg[4] |= BDIV;
		spi.pll_reg[4] |= RFB_EN;
		spi.pll_reg[4] |= BPWR;
		spi.pll_reg[4] |= RFA_EN;
		spi.pll_reg[4] |= APWR;
		spi.pll_reg[4] |= REG_4;
		spi.pll_reg[5] |= VAS_DLY;
		spi.pll_reg[5] |= SDPLL;	
		spi.pll_reg[5] |= F01;
		spi.pll_reg[5] |= LD;
		spi.pll_reg[5] |= MUX_5;
		spi.pll_reg[5] |= ADCS;
		spi.pll_reg[5] |= ADCM;
		spi.pll_reg[5] |= REG_5;
		spi.pll_reg[6] |= REG_6;
		fprintf( spi.fp, " Reg0-5 = %x, %x, %x, %x, %x, %x\n",
				spi.pll_reg[0],spi.pll_reg[1],spi.pll_reg[2],spi.pll_reg[3],spi.pll_reg[4],spi.pll_reg[5] );			
/*
 * 
80A00000,400103E9,10005F42,00001F23,63DE80FC,00400005,00000006
Reg0     Reg1     Reg2     Reg3     Reg4     Reg5     Reg6       Freq
80A00000,800103E9,00005F42,00001F23,63EE81FC,00400005,00000006     50
80A00000,800103E9,00005F42,00001F23,63DE81FC,00400005,00000006    100
80A00000,800103E9,00005F42,00001F23,63CE81FC,00400005,00000006    200
80A00000,800103E9,00005F42,00001F23,63BE81FC,00400005,00000006    400
80A00000,800103E9,00005F42,00001F23,639E81FC,00400005,00000006   1600
81040000,800103E9,00005F42,00001F23,639E81FC,00400005,00000006   2600
80960000,800103E9,00005F42,00001F23,638E81FC,00400005,00000006   3000
 * 
  see     www.maximintegrated.com/en/app-notes/index.mvp/id/5498
byte & nibble msb	31   27		23   19		15   11		7    3
* 
REG0				8    0		A    0		0    0		0    0
	int/frac flag	1
	int div value	.000 0000	1010 0000							(320d)
	reg#												.... .000
	* 

REG1				8    0		0    1		0    3		E    9
	reserved		0
	CP linearity	.00.
	CP test mode	...0 0...
	phase value			  000	0000 0001	0...
	modulus value							.000 0011	1110 1...
	reg#													 .001
	*

REG2				0    0  	0    0		5    F		4    2
	mux_out			...1 00..										<<<<
	RD2 ref doubler 	 ..0.	
	RDIV2			.... ...0 
	R ref div cntr				0000 0000	01..
	double buffer							..0.
	CP current								...1 111.
	LDF lock detect							.... ...1
	LDP lock precision									0... ....
	PDP phase polarity									.1.. ....
	SHDN power down										..0. ....
	TRI CP tridstate									...0 ....
	RST counter											.... 0...
	reg#												.... .010
	* 
REG3				0    0  	0    0		1    F		2    3
	CDIV									.001 1111	0010 0...
	reg#												.... .011
	* 
REG4				6    3  	D    E		8    1		F    C
	max2871			0110 ..11
	RFout divider				1101 ....
	BS band clk div				     1110	1000
	RFV_en									.... ...1
	Bpwr												11.. ....
	RFA_en												..1. ....
	Apwr												...1 1...
	reg#												.... .100
	* 

REG5				0    0  	4    0		0    0		0    5	
	max2871			0110 000.
	F01 int/frac	.... ...0
	Lock Det. pin				01.. ....
	MUX3 pin					..0. ....
	ADC start											.0.. ....
	ADC mode											..00 0...
	reg#												.... .101
	*


		spi.pll_reg[0] = 0x80a00000;	//evkit output=100MHz	
		spi.pll_reg[1] = 0x800103e9;
		spi.pll_reg[2] = 0x00005f42;
		spi.pll_reg[3] = 0x00001f23;
		spi.pll_reg[4] = 0x63de81fc;	  
		spi.pll_reg[5] = 0x00400005;	
*/

		for ( i=0; i<2; i++) {			// write init twice
			for ( n=5; n>-1; n--) {		// registers reverse order
				pll_write( spi.pll_reg[n]);
			}
			usleep(20000);	
		}


/*		buf8[0] = 0x00;		//select reg6 to read  0x00000006
		buf8[1] = 0x00;
		buf8[2] = 0x00;
		buf8[3] = 0x06;
		buf8[4] = 0x00;
		buf8[5] = 0x00;
		buf8[6] = 0x00;
		buf8[7] = 0x00;
		spiXfer(spi.pll_fd, buf8, buf8_rx, 8);
		fprintf( spi.fp, " REG6= %x, %x, %x, %x\n",
		         buf8[4], buf8[5], buf8[6], buf8[7] );		
*/
		goto END;
		
	}

	
	if( cmd_flag == step ) {

		N_clear = 0xffff<<15;
		DivA_clear = 0x7<<20;

		Fref = 10;							//xtal reference frenquency in MHz
		DivA = -1;							// find the DivA that 300<N<600
Loop:
		DivA = DivA + 1;
		N = spi.frequency / Fref * 1<<DivA ;//  freq is frequency in MHz
		if( N < 300 ) goto Loop;

//		RFoutA = N * Fref / (1<<DivA);	
//		printf(  " N = %d,  DivA= %d, RFoutA =  %d\n", N, DivA, RFoutA);
	
	
		spi.pll_reg[0] &= ~N_clear;				// set N into Reg0 & write to PLL
		spi.pll_reg[0] |= N<<15;
		pll_write( spi.pll_reg[0]);	
							
		R4 	= spi.pll_reg[4] & ~DivA_clear;		// set DivA into Reg4 & write to PLL
		R4 	= R4 | DivA<<20;
		if( spi.pll_reg[4] != R4 ) {
			spi.pll_reg[4] = R4;
			pll_write( spi.pll_reg[4] );
		}							
		
	}

END:

	return 0;
}

///////////////////////////////////////////
void* adc_read( int cmd_flag )
{

	union equiv { uint32_t J; char CJ[4]; } eq;
	char buff[4], buff_rx[4];
	int n;

	if( cmd_flag == idle ) {
		goto END;
	}
	
	if( cmd_flag == init_mode1 ) {
		spi.adc_reg[0] = 0xdd;
		spi.adc_reg[1] = 0xdf;
		spi.adc_reg[2] = 0xe1;
		spi.adc_reg[3] = 0xe3;
		spi.adc_reg[4] = 0xe5;
		spi.adc_reg[5] = 0xe7;

/*CTL1 =C2                   2    F
 * perform self calibration	00
 * powerdown =RESET  		  10
 * unipolar					    1
 * format = offset binary 	     1
 * Scycle = single cycle		  1
 * Contsc = single cycle           1
*/ 
		buff[0] = 0xc2;		// CTRL1
		buff[1] = 0x2f;		// CTRL1 data
		spiWrite(spi.adc_fd, buff, 2);
		usleep(200000);
		 

/*SEQ = D0					0    2
 * mux=ch 4					000
 * mode=seq 1				   0 0
 * gpodren=0				      0
 * mdren=0					       1
 * rdyben=0					        0
*/
		buff[0] = 0xd0;		//SEQ  command
		buff[1] = 0x02;		//SEQ  data
		spiWrite(spi.adc_fd, buff, 2);
		usleep(1000); 	

		buff[0] = 0xbe;		// Convert at 6400sps
		spiWrite(spi.adc_fd, buff, 1);
		usleep(100000); 			

		goto END;
	}


	if( cmd_flag == init_mode2 ) {
		spi.adc_reg[0] = 0xdd;
		spi.adc_reg[1] = 0xdf;
		spi.adc_reg[2] = 0xe1;
		spi.adc_reg[3] = 0xe3;
		spi.adc_reg[4] = 0xe5;
		spi.adc_reg[5] = 0xe7;

/*CTL1 =C2                   2    E
 * perform self calibration	00
 * powerdown =STANDBY		  10
 * unipolar					    1
 * format = offset binary 	     1
 * Scycle = single cycle		  1
 * Contsc = single cycle           0  */
		buff[0] = 0xc2;		// CTRL1
		buff[1] = 0x2e;		// CTRL1 data
		spiWrite(spi.adc_fd, buff, 2);
		usleep(100); 

/*SEQ = D0					0    a
 * mux=0 					000
 * mode=2					   0 1
 * gpodren=0				      0
 * mdren=1					       1
 * rdyben=0					        0
*/
		buff[0] = 0xd0;		//SEQ  command
		buff[1] = 0x0a;		//select seq mod 2, enable delay
		spiWrite(spi.adc_fd, buff, 2);
		
		buff[0] = 0xca;		// DELAY
		buff[1] = 0xf0;		// 
		buff[2] = 0x00;
		spiWrite(spi.adc_fd, buff, 3);
			
		buff[0] = 0xce;		// ==> CHMAP0
		buff[1] = 0x0e;		// chan 3
		buff[2] = 0x0a;		// chan 2
		buff[3] = 0x06;		// chan 1
		spiWrite(spi.adc_fd, buff, 4);
		
		buff[0] = 0xcc;		// ==> CHMAP1
		buff[1] = 0x1a;		// chan 6
		buff[2] = 0x16;		// chan 5
		buff[3] = 0x12;		// chan 4
		spiWrite(spi.adc_fd, buff, 4);
		
		goto END;
	}


	if( cmd_flag == step ) {

/*Convert					B    E
 * convert code				1011
 * sample rate=64000sps		    1110  */
		buff[0] = 0xbe;					// Convert! (6400sps)
		spiWrite(spi.adc_fd, buff, 1);
		usleep(10000); 			

		for ( n=0; n<6; n++) {
		
			buff[0] = spi.adc_reg[n];	//select a channel to read
			spiXfer(spi.adc_fd, buff, buff_rx, 4);
			eq.CJ[0] = buff_rx[3];
			eq.CJ[1] = buff_rx[2];
			eq.CJ[2] = buff_rx[1];
			eq.CJ[3] = buff_rx[0];
			spi.ADC[n] = (float) eq.J / adc_bits * Vref;
//			fprintf( spi.fp, " acd %d, = %0x,  %.4f\n",  n, eq.J, spi.ADC[n]);
		}
//		fprintf( spi.fp, "ADC[0-5]= %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n",
//		spi.ADC[0],spi.ADC[1],spi.ADC[2],spi.ADC[3],spi.ADC[4],spi.ADC[5]);


		goto END;
	}	

	if( cmd_flag == dev_status ) {

		buff[0] = 0xc1;					//read status reg.
		spiXfer(spi.adc_fd, buff, buff_rx, 4);
		fprintf(spi.fp, "status= %x, %0x, %x, %x\n",
			buff_rx[0],buff_rx[1],buff_rx[2],buff_rx[3]); 
	}	

END:
	return 0;
}


///////////////////////////////////////////
void* dac_write( int cmd_flag )
{
// 
// use this equivalence union to access the integer as bytes
//   J    0x11223344	integer & byte alignment
//  CJ[]     3 2 1 0	
	union equivs { uint32_t J; unsigned char CJ[4]; } eq;
	char buff[3];
	unsigned short I[4];
	int n;

	if( cmd_flag == idle ) {
		goto END;
	}
	
	 else if( cmd_flag == init ) {
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
		spiWrite(spi.dac_fd, buff, 3);

		goto END;
	}
	
	else if( cmd_flag == step ) {

		I[0] = spi.cathode * spi.cathode_gain;
		I[1] = spi.einzel1 * spi.einzel1_gain;
		I[2] = spi.einzel2 * spi.einzel2_gain;
		I[3] = spi.dac_test * spi.dac_test_gain;
//		fprintf( spi.fp, " dac= %x,  %x,  %x,  %x\n",I[0], I[1], I[2], I[3]  );	

		for( n=0; n<4; n++) {
			eq.J = I[n];
			buff[0] = spi.dac_cmd[n];		// send command first
			buff[1] = eq.CJ[1];
			buff[2] = eq.CJ[0];
//			fprintf( spi.fp, "buff= %x, %x, %x\n", buff[0], buff[1], buff[2] );
			spiWrite(spi.dac_fd, buff, 3); 
			usleep(100);
		}
	}

END:

//union equiv { uint32_t J; unsigned char CJ[4] };
//union equiv eq;
//	eq.J = 0x11223344;
//	fprintf ( spi.fp, "J= %x,   CJ= %x, %x, %x, %x \n",
//		eq.J, eq.CJ[0], eq.CJ[1], eq.CJ[2], eq.CJ[3] );


	return 0;
}





///////////////////////////////////////////
void *vac_thread( void *vv )
{
	vac* v = (vac*) vv; 				// cast the void* to struct type
  
	int h2_in_delay=10000, h2_out_delay=5000, pump_delay=100000, vac_delay=40000;
	int dwell_time=800000, count=0;
	FILE *fp;
	float hi=v->set_point+v->deadband/2,low=v->set_point-v->deadband/2;

	fp = fopen( "gage.dat", "w+" );

Measure: 
	if (v->thread_status == kill ) { goto Exit; }

	usleep(dwell_time);	
	gage_read();
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
		
	if( v->gage < low) 
	{									//too 0, pulse H2
		h2_out_delay = 20000 * (low- v->gage) / low;
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
int gage_read( )
{
	int i, len, j ;
	char *device;
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
    
    if( serDataAvailable( vgage.fd ) )
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
	printf( "h = THIS help menu\n" );	

	
	sleep(1);
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

