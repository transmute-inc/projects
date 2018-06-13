#include <stdio.h>		//20180525.1738
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

#define RF_en		6		// BCM GPIO
#define H2_in		4		// RLY_1 moved from p1-11 -> p1-7
#define H2_out		22		// RLY_2 moved from p1-12 -> p1-15
#define Vac			27
#define Vac_pump	23
#define att_cs		18		// needed to toggle pe43711 data into latch

#define not_running	0		//threadstatus
#define running		1		//threadstatus
#define kill		2		//threadstatus

#define  Vref		2.44	// vref is common for adc & dac
#define  dac_bits	65535
#define  adc_bits	16777215
#define  forward	0		// AIN0P
#define  reflected	1		// AIN1P
#define  sniffer	2		// AIN2P
#define  ammeter	3		// AIN3P
#define  adc_test	4		// AIN4P
#define  adc_spare	5		// AIN5P
#define  cathode	0		// DAC0
#define  einzel1	0		// DAC1
#define  einzel2	0		// DAC2
#define  dac_test	0		// DAC3 -- drives AIN4P



typedef struct vac_gage {
	int32_t thread_status;
	uint32_t serBaud;
	uint32_t serFlags;	
	int32_t status;
	uint32_t fd;
	float gage;
	float set_point;
	float deadband;
} vac;

	vac vgage;				// global structure!!
	

typedef struct spi_control {

	int thread_status;
	FILE *fp;
	char cmd;
	
	uint32_t pll_fd;
	uint32_t pll_reg[6];	// reg0-reg5 !write in reverse order!
	uint32_t frequency;		// in MHz

	uint32_t att_fd;
	float attenuation;		// in dB

	uint32_t adc_fd;
	unsigned char adc_reg[6];
	unsigned char adc_status[4];
	float ADC[6];			// adc voltage values
	float DB[3];			// log amp voltages converted to DB

	uint32_t dac_fd;
	unsigned char dac_cmd[4];
	float DAC[4];			// dac voltage desired
	float GAIN[4];			// dac gain coefficient to get a 0-2.44v range
} spi_c;

	spi_c spi;				// global structure!!
	

int   getch(int32_t ms);		// routine calling definitions
void* spi_thread( void *s );
int d_a_bob( );
int* ce_Test( );

void adc_init_mode1( );
void adc_init_mode2( );
void adc_status( );
void adc_read( );

void att_Set( float atten );
void att_Test( );
void att_Sweep(float start, float end, float df, uint32_t dt);

void dac_init( );
void dac_write( );
void dac_adc_Test( );
void dac_Sweep(float start, float end, float dv, uint32_t dt, uint32_t channel, uint32_t num_pulses);

void pll_Init( );
void pll_write( uint32_t data );
void pll_Test( );
void pll_SetN( uint16_t N );
void pll_SetDIVA( char diva );
void pll_SetR( uint16_t R );
void pll_SetF( uint16_t F );
void pll_SetM( uint16_t M );
void pll_SetFracMode();
void pll_SetIntMode();
void pll_rfa_power( int32_t power );
void pll_rfa_disable();
void pll_rfa_enable();
void pll_SetIntfreq( int32_t megahz );
void pll_IntSweep(uint32_t a, uint32_t b, uint32_t df, uint32_t dt);

void find_resonance(uint32_t start, uint32_t end, uint32_t df, uint32_t dt);
void* logamp_Test( );
void* gpio_Test( );
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



		printf("\t dac_test=%.4f  adc_test=%4f\n", spi.DAC[dac_test], spi.ADC[adc_test]);
//		printf("\t vac.gage=%.4f\n",vgage.gage);
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
//	spi_c* s = (spi_c*) ss;


	FILE *fp;
	fp = fopen( "spi.dat", "w+" );
	spi.fp = fp;
	
	dac_init( );
	adc_init_mode2();	
	pll_Init( );
	
	
// select a letter to call the associated routine
	spi.cmd='d';  
LOOP:
	if (spi.thread_status == kill ) { goto CLOSE; }

	if     (spi.cmd=='a')	{ dac_adc_Test(); }	//cycles dac_test line 0v->2.44v
	else if(spi.cmd=='c')	{ ce_Test(); } 		//chip enables with one byte data
	else if(spi.cmd=='g')	{ gpio_Test( ); }	//valve test
	else if(spi.cmd=='t')	{ att_Test( ); }	//just sets the PE43711 attenuator
	else if(spi.cmd=='l')	{ logamp_Test( ); }	//reads voltages on logamps
	else if(spi.cmd=='d')	{ dac_Sweep( 0.0, 2.44, 0.1, 10000, dac_test,10);}
	else if(spi.cmd=='p')	{ pll_Test( ); }
	else if(spi.cmd=='P')	{ 
		pll_IntSweep( 50, 200, 5, 1000);
		pll_IntSweep( 200, 50, 5, 1000);}
		
	goto LOOP;	
		
CLOSE:	
	spi.thread_status = not_running ;
	fclose(fp);
	
	pthread_exit(NULL);
}

///////////////////////////////////////////	
void find_resonance(uint32_t start, uint32_t end, uint32_t df, uint32_t dt){
						// start, df, end in MHz,   dt in usec
	uint32_t megahz, num_points, for_max_mhz, for_min_mhz, ref_max_mhz, ref_min_mhz;
	float for_ave, for_max, for_min, ref_ave, ref_max, ref_min;
	
	
	megahz = start;
	num_points = (end - start) / df;
	for_ave = 0;
	for_max = 0;
	for_min = 0;
	ref_ave = 0;
	ref_max = 0;
	ref_min = 0;
	

	FILE *res_fp;
	res_fp = fopen( "res.dat", "w+" );

					//sweep up
	while ( megahz < end ) {
		if (spi.thread_status == kill) { return; }
		pll_SetIntfreq( megahz );
		usleep(10);
		adc_read();
		if( spi.ADC[forward] > for_max) {
			spi.ADC[forward] = for_max;
			for_max_mhz = megahz; }
		if( spi.ADC[reflected] > ref_max) {
			spi.ADC[reflected] = ref_max;	
			ref_max_mhz = megahz; }
		if( spi.ADC[forward] < for_min) {
			spi.ADC[forward] = for_min;
			for_min_mhz = megahz; }
		if( spi.ADC[reflected] < ref_min) {
			spi.ADC[reflected] = ref_min;	
			ref_min_mhz = megahz; }
		fprintf( res_fp, " %d %.4f %.4f\n", megahz, spi.ADC[forward], spi.ADC[reflected]);	
		usleep(dt);
		megahz = megahz + df;
	}
	
	fprintf( spi.fp, "for_max %.4f @ %d mhz\n", for_max, for_max_mhz );	
	fprintf( spi.fp, "ref_max %.4f @ %d mhz\n", ref_max, ref_max_mhz );	
	fprintf( spi.fp, "for_min %.4f @ %d mhz\n", for_min, for_min_mhz );	
	fprintf( spi.fp, "ref_min %.4f @ %d mhz\n", ref_min, ref_min_mhz );	

	fclose(res_fp);

	gnuplot_ctrl    *   h1 ;
	h1 = gnuplot_init() ;
	gnuplot_resetplot(h1) ;
	
	gnuplot_setstyle(h1, "lines") ;
	gnuplot_cmd(h1, "set xlabel 'Freq MHz' ") ;
	gnuplot_cmd(h1, "plot 'res.dat' using 1:2 title 'forward DB', 'res.dat' using 1:3 title 'reflected DB'") ;

	sleep (10 );
	gnuplot_close(h1) ;


}




///////////////////////////////////////////	
void pll_write( uint32_t data ) {
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
}

///////////////////////////////////////////
void pll_Test( )
{
//	dac_init( );				//write .5v to dac_adc_test
	eq.J = 0.5 /Vref * dac_bits;	//just to insure adc is working
	buff[0] = 0x38;
	buff[1] = eq.CJ[1];
	buff[2] = eq.CJ[0];
	spiWrite(spi.dac_fd, buff, 3);
 			
	spi.frequency = 100;				//50MHz, maybe...
//	pll_Init( );
	
	spi.attenuation = 0;
	att_Set( spi.attenuation );

//	adc_init_mode1();
	gpioWrite (RF_en, 1) ;	

PLL:
	spi.attenuation = 0;
	att_Set( spi.attenuation );
	if (spi.thread_status == kill ) { goto CLOSE; }
	spi.frequency = 50;				//50MHz
	pll_SetIntfreq( spi.frequency );	
	usleep(100);
//	adc_read( );
//	fprintf( spi.fp, " freq=%d, power out = %.4f\n",
//				spi.frequency, spi.ADC[forward]);
	usleep(100000);
		
	spi.attenuation = 16;
	att_Set( spi.attenuation );
	spi.frequency = 100;			//100MHz
	pll_SetIntfreq( spi.frequency );			
	usleep(100);
//	adc_read( );
//	fprintf( spi.fp, " freq=%d, power out = %.4f\n",
//				spi.frequency, spi.ADC[forward]);
	usleep(100000);
	goto PLL;

CLOSE:
	gpioWrite (RF_en, 0) ;
	
}

///////////////////////////////////////////	
void pll_SetN( uint16_t N ) {		//set 16bit int divisor in reg0[30:15]
	spi.pll_reg[0] &= (~N_MASK); 
	spi.pll_reg[0] |= (N << 15);
	pll_write( spi.pll_reg[0] );
}

///////////////////////////////////////////	
void pll_SetDIVA( char diva ) {	//set 3bit A divider into reg4[22:20]
	spi.pll_reg[4] &= ~DIVA_MASK;
	spi.pll_reg[4] |= diva<<20;
	pll_write( spi.pll_reg[4] );
}
///////////////////////////////////////////	
void pll_SetR( uint16_t R ) {		//set 10bit R divider into reg2[23:14]
	spi.pll_reg[2] &= ~R_MASK;
	spi.pll_reg[2] |= R<<14;
	pll_write( spi.pll_reg[2] );
	pll_write( spi.pll_reg[0] );	//R is double buffered ?????
}

///////////////////////////////////////////	
void pll_SetF( uint16_t F ) {		//set 12bit F divider into reg0[14:3]
	spi.pll_reg[0] &= ~F_MASK;
	spi.pll_reg[0] |= F<<3;
	pll_write( spi.pll_reg[0] );
}

///////////////////////////////////////////	
void pll_SetM( uint16_t M ) {		//set 12bit M divider into reg0[14:3]
	spi.pll_reg[1] &= ~M_MASK;
	spi.pll_reg[1] |= M<<3;
	pll_write( spi.pll_reg[1] );
	pll_write( spi.pll_reg[0] );	//M is double buffered ?????
}

///////////////////////////////////////////	
void pll_SetFracMode() {
	spi.pll_reg[0] &= ~EN_INT;
	spi.pll_reg[2] &= ~LDF;
	pll_write( spi.pll_reg[2] );
	pll_write( spi.pll_reg[0] );
}

///////////////////////////////////////////	
void pll_SetIntMode() {
	spi.pll_reg[0] &= EN_INT;
	spi.pll_reg[2] &= LDF;
	pll_write( spi.pll_reg[2] );
	pll_write( spi.pll_reg[0] );
}

///////////////////////////////////////////	
void pll_rfa_power( int32_t power ) {
	spi.pll_reg[4] &= ~(3 << 3); 		// "clean" power bits 
	spi.pll_reg[4] |= (power << 3); 	// 0=-4dBm, 1=-1dBm, 2=2dBm, 3=5dBm
	pll_write( spi.pll_reg[4] );
}

///////////////////////////////////////////	
void pll_rfa_disable() {
	spi.pll_reg[4] &= ~RFB_EN;
	pll_write( spi.pll_reg[4] );
}
///////////////////////////////////////////	
void pll_rfa_enable() {
	spi.pll_reg[4] &= RFB_EN;
	pll_write( spi.pll_reg[4] );
}


///////////////////////////////////////////
void pll_Init( )  {
//	char buf8[8], buf8_rx[8];
	int i, n;
	
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

		
}

///////////////////////////////////////////
void pll_IntSweep(uint32_t start, uint32_t end, uint32_t df, uint32_t dt){
						// start, df, end in MHz,   dt in usec
	uint32_t megahz;
	
	megahz = start;
	if( start < end ) {					//sweep up
		while ( megahz < end ) {
			if (spi.thread_status == kill) { return; }
			pll_SetIntfreq( megahz );
			usleep(dt);
			megahz = megahz + df;
		}
	}
	else {									//sweep down
		while ( megahz > start ) {
			if (spi.thread_status == kill) { return; }		
			pll_SetIntfreq( megahz );
			usleep(dt);
			megahz = megahz - df;
		}
	}
 
}

///////////////////////////////////////////
void pll_SetIntfreq( int32_t megahz )
{	
		uint32_t Fref,diva,N;
		Fref = 10;						//xtal reference frenquency in MHz
		diva = -1;						// find the DivA that 300<N<600
Loop:
		diva = diva + 1;
		N = megahz / Fref * 1<<diva ;	//  freq is frequency in MHz
		if( N < 300 ) goto Loop;

		pll_SetN(N);
		pll_SetDIVA(diva);
}

///////////////////////////////////////////
int d_a_bob( )
{
	char buff[4];
	
//1:	
	adc_status();
	dac_init( );
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
		
		adc_status();


LOOP:
		if (spi.thread_status == kill ) { return 0;}
	
		spi.DAC[dac_test] = .5;
		dac_write( );
		usleep(1000); 

		adc_read( );
		usleep(1000); 

		spi.DAC[dac_test] = 1.0;
		dac_write( );

		adc_read( );
																						
		adc_status();

		goto LOOP;

}

///////////////////////////////////////////
int* ce_Test( )
{
	char buff[4], buff_rx[4];	
	buff[0] = 0x55;
	buff_rx[0] = 0x99;
	
LOOP:
		if (spi.thread_status == kill ) { return 0; }
		spiWrite(spi.dac_fd, buff, 1);
		usleep(20);
		spiXfer(spi.adc_fd, buff, buff_rx, 1);
		usleep(20);
		spiWrite(spi.pll_fd, buff, 1);
		usleep(20);
		spiWrite(spi.att_fd, buff, 1);
		usleep(20);
		goto LOOP;
}

///////////////////////////////////////////
void att_Set( float atten )
{
	union equivs { uint32_t J; unsigned char CJ[4]; } eq;
	char buff[4];
	
	eq.J = abs(atten)*4;			// attenuation in quarter dB steps
	buff[0] = eq.CJ[3];
	spiWrite(spi.att_fd, buff, 1);
	gpioWrite (att_cs,0);			// clock shift reg data into latch 
	gpioWrite (att_cs,1);

}
///////////////////////////////////////////
void att_Test( )
{
	adc_init_mode2();
	spi.attenuation = -16;
	att_Set( spi.attenuation );
}

///////////////////////////////////////////
void att_Sweep(float start, float end, float df, uint32_t dt){
// start, df, end in dbm,   dt in usec

	float atten;
	atten = start;
	if ( start < end ) {			//sweep up
down:		att_Set( atten );
		usleep(dt);
		atten = atten + df;
		if ( atten < end ) goto down;
	}
	else {
up:	att_Set( atten );
		usleep(dt);
		atten = atten - df;
		if ( atten > end ) goto up;
	}
 
}

///////////////////////////////////////////
void adc_init_mode1( )
{
	char buff[4];

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
}

///////////////////////////////////////////
void adc_init_mode2( )
{
	char buff[4];

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
}

///////////////////////////////////////////
void adc_status( )
{
	char buff[4], buff_rx[4];
	
	buff[0] = 0xc1;					//read status reg.
	spiXfer(spi.adc_fd, buff, buff_rx, 4);
	fprintf(spi.fp, "status= %x, %0x, %x, %x\n",
		buff_rx[0],buff_rx[1],buff_rx[2],buff_rx[3]); 
}

///////////////////////////////////////////
void adc_read( )
{

	union equiv { uint32_t J; char CJ[4]; } eq;
	char buff[4], buff_rx[4];
	int32_t n;

/*Convert					B    E
 * convert code				1011
 * sample rate=64000sps		    1110  */
	buff[0] = 0xbe;					// Convert! (6400sps)
	spiWrite(spi.adc_fd, buff, 1);
	usleep(500); 			

	for ( n=0; n<5; n++) {
		
		buff[0] = spi.adc_reg[n];	//select a channel to read
		spiXfer(spi.adc_fd, buff, buff_rx, 4);
		eq.CJ[0] = buff_rx[3];
		eq.CJ[1] = buff_rx[2];
		eq.CJ[2] = buff_rx[1];
		eq.CJ[3] = buff_rx[0];
		spi.ADC[n] = (float) eq.J / adc_bits * Vref;
//		fprintf( spi.fp, " acd %d, = %0x,  %.4f\n",  n, eq.J, spi.ADC[n]);
	}
//  forward		0		// AIN0P
//  reflected	1		// AIN1P
//  sniffer		2		// AIN2P
		spi.DB[forward] = spi.ADC[forward] /0.021 - 87; //convert to dBv
		spi.DB[reflected] = spi.ADC[reflected] /0.021 - 87;
		spi.DB[sniffer] = spi.ADC[sniffer] /0.021 - 87;	
//	fprintf( spi.fp, "ADC[0-5]= %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n",
//	spi.ADC[0],spi.ADC[1],spi.ADC[2],spi.ADC[3],spi.ADC[4],spi.ADC[5]);
}

///////////////////////////////////////////
void dac_init( )
{
	char buff[3];

	spi.DAC[cathode] = 0;		// init v=0 and default gain
	spi.GAIN[cathode] = 1 /Vref * dac_bits;
	spi.DAC[einzel1] = 0;
	spi.GAIN[einzel1] = 1 /Vref * dac_bits;
	spi.DAC[einzel2] = 0;
	spi.GAIN[einzel2] = 1 /Vref * dac_bits;
	spi.DAC[dac_test] = 0;
	spi.GAIN[dac_test] = 1 /Vref * dac_bits;	

	spi.dac_cmd[0] = 0x31;		// inialize write thru command
	spi.dac_cmd[1] = 0x32;		// for DACs 1-4
	spi.dac_cmd[2] = 0x34;
	spi.dac_cmd[3] = 0x38;

	buff[0] = 0x02;				// software clear
	buff[1] = 0x00;
	buff[2] = 0x00;
	spiWrite(spi.dac_fd, buff, 3);

}


///////////////////////////////////////////
void dac_write( )
{
// 
// use this equivalence union to access the integer as bytes
//   J    0x11223344	integer & byte alignment
//  CJ[]     3 2 1 0	
	union equivs { uint32_t J; unsigned char CJ[4]; } eq;
	char buff[3];
	uint16_t v[4];
	int32_t n;

	v[0] = spi.DAC[cathode] * spi.GAIN[cathode];
	v[1] = spi.DAC[einzel1] * spi.GAIN[einzel1];
	v[2] = spi.DAC[einzel2] * spi.GAIN[einzel2];
	v[3] = spi.DAC[dac_test] * spi.GAIN[dac_test];

	for( n=0; n<4; n++) {
		eq.J = v[n];
		buff[0] = spi.dac_cmd[n];		// send command first
		buff[1] = eq.CJ[1];
		buff[2] = eq.CJ[0];
//		fprintf( spi.fp, "buff= %x, %x, %x\n", buff[0], buff[1], buff[2] );
		spiWrite(spi.dac_fd, buff, 3); 
//		usleep(100);
	}
}

///////////////////////////////////////////
void dac_adc_Test( )  {
	int32_t count;
	count = 0;

DAC:
	if (spi.thread_status != kill ) {
		count = count + 1;
		spi.DAC[cathode] = 0.0;
		spi.DAC[einzel1] = 0.0;
		spi.DAC[einzel2] = 0.0;
		spi.DAC[dac_test] = 0.0;
		dac_write( );
		usleep(1000);
		adc_read( );
		fprintf( spi.fp, " %d, vin=%.4f  vout=%.4f\n", count, spi.DAC[dac_test], spi.ADC[adc_test]);	


		spi.DAC[cathode] = 2.44;
		spi.DAC[einzel1] = 2.44;
		spi.DAC[einzel2] = 2.44;
		spi.DAC[dac_test] = 2.44;
		dac_write( );
		usleep(1000);
		adc_read( );
		fprintf( spi.fp, " %d, vin=%.4f  vout=%.4f\n", count, spi.DAC[dac_test], spi.ADC[adc_test]);	
	}
	else {							//reset voltage = 0 and bail
		spi.DAC[cathode] = 0;
		spi.DAC[einzel1] = 0;
		spi.DAC[einzel2] = 0;
		spi.DAC[dac_test] = 0;
		dac_write( );
		return;
	}	
	goto DAC;

//	fprintf( fp, " S %d, AM=%.4f, FP=%.4f, RP=%.4f, SP=%.4f, TST=%.4f\n",  
//	count, spi.ADC[ammeter],spi.ADC[forward],spi.ADC[reflected],
//	spi.ADC[sniffer],spi.ADC[adc_test]);


}

///////////////////////////////////////////
void dac_Sweep(float start, float end, float dv, uint32_t dt, uint32_t channel, uint32_t num_pulses){
	int32_t count;

/* start, dv, end in dbm,   dt in usec
 * calling routine passes a voltage (0-2.44v) on one of the four dac channels  
 * such as spi.DAC[cathode], spi.DAC[einzel1], spi.DAC[einzel2], or spi.DAC[dac_test_gain].
 * dac_write writes out all four channels. In the future, the DAC and ADC will 
 * just cycle continuously on a 1msec heartbeat so the dac_write and adc_read 
 * on their own thread and will not need to be called.
*/

	count = 0;
	spi.DAC[channel] = start;			


	while ( count < num_pulses ) {
		while ( spi.DAC[channel] < end ) {			//sweep up
			if (spi.thread_status == kill) { goto CLOSE; }
			dac_write( );
			adc_read( );
			usleep(dt);
			adc_read( );
			fprintf( spi.fp, " %d, vin=%.4f  vout=%.4f\n", count, spi.DAC[dac_test], spi.ADC[adc_test]);	
			spi.DAC[channel] = spi.DAC[channel] + dv;
		}
		count = count + 1;
		
		while ( spi.DAC[channel] > start ) {
			if (spi.thread_status == kill) { goto CLOSE; }
			dac_write( );
			adc_read( );
			fprintf( spi.fp, " %d, vin=%.4f  vout=%.4f\n", count, spi.DAC[dac_test], spi.ADC[adc_test]);	
			usleep(dt);
			spi.DAC[channel] = spi.DAC[channel] - dv;
		}
		count = count + 1;
	}

CLOSE:
	spi.DAC[channel] = 0.0;
}

///////////////////////////////////////////
void* logamp_Test( )
{
	adc_init_mode2();	
	adc_status();

			
LOOP:
	if (spi.thread_status == kill ) { return 0; }

	adc_read( );

	printf(" Pfor= %.4f, Prev= %.4f\r", spi.ADC[forward],spi.ADC[reflected]);
	goto LOOP;
 
}

///////////////////////////////////////////
void* gpio_Test( )
{
GPIO:
		if (spi.thread_status == kill ) { return 0; }
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

///////////////////////////////////////////
void *vac_thread( void *vv )
{
	vac* v = (vac*) vv; 				// cast the void* to struct type
  
	int32_t h2_in_delay=10000, h2_out_delay=5000, pump_delay=100000, vac_delay=40000;
	int32_t dwell_time=800000, count=0;
	FILE *fp;
	float hi=v->set_point+v->deadband/2,low=v->set_point-v->deadband/2;

	fp = fopen( "gage.dat", "w+" );

MEASURE: 
	if (v->thread_status == kill ) { goto EXIT; }

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
		goto MEASURE;
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
		goto MEASURE;
	}
	goto MEASURE;

EXIT:									// turn everything off before exit
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
	int32_t i, len, j ;
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
		if (d[4] == 'A' )				//must be ACK
		{
			j=-1;
			for ( i=7; i<20; i++ )		//collect the number
			{
				j=j+1;
				if ( d[i] != ';' ) {	// ';' delimiter
					e[j] = d[i];
				}
				else {
					e[j] = '\0';		// add a trailing zero
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
	
//	gnuplot_ctrl    *   h1;

//    printf("*** example of gnuplot control through C ***\n") ;
 //   h1 = gnuplot_init() ;

 //   gnuplot_resetplot(h1) ;
 //   gnuplot_cmd(h1, "set xlabel 'time' ") ;
 //   gnuplot_cmd(h1, "plot 'gage.dat' using 1:2 with lines") ;
 
    sleep (10 );
  //  gnuplot_close(h1) ;
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
