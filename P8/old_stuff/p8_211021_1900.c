#include <stdio.h>		//20180525.1738
#include <stdlib.h>
#include <stdint.h>  //int8_t,int16_t,int32_t,uint8_t,uint16_t,uint32_t
#include <termios.h>
#include <poll.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <math.h>


#include <pthread.h>
#include <sys/types.h>  // needed for getpid()
#include <unistd.h>     // needed for getpid()

#include <../gnuplot/gnuplot_i.h>

#include <pigpio.h>		// http://abyz.co.uk/rpi/pigpio/cif.html

#include "max2871.h"	

/*
* gcc -Wall -o p7 p7.c -lpigpio -lthread -lm	// be sure to include math libs
* sudo ./p6
* gcc -Wall -ggdb -o p7 p7.c -lpigpio -lthread
* sudo gdb ./p7
*/

// defines for REV6
#define ch0			0
#define ch1			1
#define ch2			2
#define ch3			3
#define range_5ma	0
#define range_50ua	2
#define range_500pa	3




#define RF_en		6		// BCM GPIO
#define Sync_pin	4		// usefull to sync oscope
#define H2_in		27		// RLY_1
#define H2_out		22		// RLY_2 
#define Vac			23		// RLY_3
#define Vac_pump	24		// RLY_4

#define RLY_5		17		// RLY_5  rev6
#define RLY_6		18		// RLY_6  rev6
#define HV_pulse	25		// RLY_7  rev6 also wired to P2 Valves header
#define range_cs	12		// rev6 gpio extender 
#define ltc2664_cs	 7		// rev6 DAC
#define ads8698_cs	 8		// rev6 ADC



#define att1_cs		18		// needed to toggle pe43711 data into latch
#define att2_cs		13		// needed to toggle pe43711 data into latch

#define not_running	0		// threadstatus
#define running		1		// threadstatus
#define kill		2		// threadstatus

#define	Vref		2.534	// vref is common for adc & dac
#define	adc_bits	16777215
#define amp_scale	1.98	// diff-amp scaling factor for ammeter

#define  forward	0		// AIN0P	cathode (vac) gage
#define  gage_Vac	0		// AIN0P	cathode (vac) gage
#define  reflected	1		// AIN1P
#define  ammeter	1		// AIN2P    // board 1 & 2 are both using the reflected pin
#define  adc_3		3		// AIN3P
#define  gage_H2	4		// AIN4P	anode (H2)
#define  adc_spare	5		// AIN5P

int32_t  anode		=0;		// DAC0
int32_t  e3bot		=1;		// DAC1
int32_t  grid		=2;		// DAC2
int32_t  e3top		=3;		// DAC3
int32_t  cathode	=4;		// DAC4
int32_t  e2			=5;		// DAC5
int32_t  e1			=6;		// DAC6
int32_t  dac_spare	=7;		// DAC7

char date_time[32];


typedef struct vac_gage {
	float_t gage;
	float_t gage_hydrogen;
	float_t gage_vacuum;
	float_t set_point;
	float_t deadband;
	int32_t thread_status;
	uint32_t serBaud;
	uint32_t serFlags;	
	int32_t status;
	uint32_t fd;
	char msg[20];
} vac;

	vac vgage;				// global structure!!
	

typedef struct spi_control {

	uint32_t thread_status, task_status;
	uint32_t start, last, end;
	FILE *fp;
	char cmd;
	
	uint32_t pll_fd;
	uint32_t pll_reg[6];	// reg0-reg5 !write in reverse order!
	uint32_t frequency;		// in MHz
	uint32_t sw_freq[1000];	// set up sweep arrays
	float_t sw_forward[1000];
	float_t sw_reflected[1000];
	float_t sw_d_forward[1000];
	float_t sw_d_reflected[1000];

	uint32_t att1_fd;
	uint32_t att2_fd;
	float_t attenuation;		// in dB


	uint32_t adc_fd,  adc_channel_rev6[8];
	uint8_t adc_reg[8];
	uint8_t adc_status[8]; 
	char adc_names[8][8];
	float_t adc[8];			// adc voltage values
	float_t adc_gain[8];
	float_t adc_offset[8];
	float_t DB[3];			// log amp voltages converted to DB

	uint32_t dac_fd, dac_bits,  dac_channel_rev6[8];
	uint32_t dac_cmd[8], dac_ltc2446_cmd[4];
	float_t dac[8];			// dac voltage desired (positive for anode, negative for cathode)
	float_t dac_gain[8];		// dac gain coefficient - see dac_init for details
	float_t dac_offset[8];
	char dac_names[8][8];
	
//REV6
	uint32_t range_fd;		// gpio16
	uint8_t range_out;
	uint8_t range_mask_clear[4];
	
	
} spi_c;

	spi_c spi;				// global structure!!
	

int   getch(int32_t ms);		// routine calling definitions
void* _spi_thread( void *s );
int* ce_Test( );

void adc_init( uint32_t mode );
void adc_status( );
void adc_read( );

void att_Set( float_t atten );
void att_Test( );
void att_Sweep(float_t start, float_t end, float_t df, uint32_t dt);

void dac_init( );
void dac_write(int32_t channel, float_t value );
void dac_adc_Test( );
int  dac_spare_test( );

void range_init( );
void range_set(int32_t channel, int8_t max_current);
void range_test( );
void my_gpio_init( );
void my_gpio_write(int32_t gpio, int32_t val);

void dac_Sweep(float_t start, float_t end, float_t dv, int32_t dt, int32_t channel, int32_t num_pulses, float_t r_proton);

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
void* _vac_thread( void *v );
int   serial_gage_init( );
int   serial_gage_read( );
int   serial_test( );
int   sweep_pressure( );
int   grid_pulse( );
int   sweep_grid( );
int   opamp_test();
void   mass_flow(int32_t number_puffs, int32_t H2_vac);
void   proton_infusion(int32_t adc_channel);
int   sweep_cathode( );
int   help_main_menu( );
int   help_s_menu( );

uint32_t ser_tty, serBaud=9600, serFlags=0;
uint32_t spiBaud=1000000;	// 8meghz for rev2, 10meghz for rev6
// MAX2871  PLL 20MHz
// MAX5134  DAC 30MHz
// MAX11254 ADC  8MHz
// PE43711  ATT 10MHz
// AD5668	DAC 50MHz

int x, rc1, rc2, count=0, n, ch;
uint8_t c;
char main_cmd;
char buff[4], buff_rx[4];
union equivs { uint32_t J; uint8_t CJ[4]; } eq;

	int32_t  board_rev, pelletnumber ;
	float_t  r_sense, Vcathode, temp_exp, temp1, temp2;





int main(void)
{
	clock_t tic = clock();
	spi.start = tic;
	spi.last = tic;

    time_t T= time(NULL);
    struct  tm tm = *localtime(&T);
    sprintf(date_time,"%04d%02d%02d-%02d%02d", tm.tm_year+1900, tm.tm_mon+1,
										tm.tm_mday,tm.tm_hour, tm.tm_min);
	r_sense = 47700;	// ammeter sense R21 board 1&2 are now original 
	printf(" board rev2 or 6? ");
	scanf(" %d", &board_rev);
	printf(" pellet number? ");
	scanf(" %d", &pelletnumber);
	printf(" cathode voltage - ");
	scanf(" %f", &Vcathode);
	Vcathode = fabs(Vcathode);
	printf("pellet = %d  cathode = %f \n",pelletnumber, Vcathode);



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
  	gpioSetMode(Sync_pin, PI_OUTPUT);
  	
	serial_gage_init( );
	if(board_rev == 2) {
		spi.adc_fd =  spiOpen(0, spiBaud, 0);		//CE0   08   24	 T11	
		spi.dac_fd =  spiOpen(1, spiBaud, 0);		//CE1   07   26  T12
	}else {
		spi.adc_fd   = spiOpen(0, spiBaud, 256);	//ce0   18   12
		spi.dac_fd   = spiOpen(1, spiBaud, 256);	//ce1   17   11
		spi.range_fd = spiOpen(2, spiBaud, 256);	//ce2   16   33	
	}

/*	
 	spi.adc_fd =  spiOpen(0, spiBaud, 0);	//CE0   08   24	 T11	
	spi.dac_fd =  spiOpen(1, spiBaud, 0);	//CE1   07   26  T12
	spi.att1_fd = spiOpen(0, spiBaud, 256);	//ce0   18   12
	spi.pll_fd =  spiOpen(1, spiBaud, 256);	//ce1   17   11
	spi.att2_fd = spiOpen(2, spiBaud, 256);	//ce2   13   33
*/
	FILE *fp;
	fp = fopen( "spi.dat", "w+" );
	spi.fp = fp;
	

//	char adc_names[6][8] = {"gage_H2 ", "ammeter ", "dead   ", "gage_Vac  ", "spare   ", "dac_test"};

    pthread_t *thread_s, *thread_v;		

	spi.thread_status = not_running;
	
	vgage.thread_status = not_running;
	vgage.status=0;
	dac_init( );		// we want to use the adc and dac in the main loop
	adc_init( 2 );		// will be reinitiallized in spi_thread
	system("clear");
	help_main_menu();
	

	
	do {
		main_cmd = 0;
		do{
			main_cmd = getch(500);
		}while(main_cmd==0);
		printf("%s\n",&main_cmd);	
  
		if (main_cmd == 'q') {		//quit any running threads
			if (spi.thread_status == running) {	
//				spi.thread_status = kill;	//this will signal spi processes to exit
				spi.task_status = kill;
			}
			if (vgage.thread_status == running) {
				vgage.thread_status = kill;
			}
			system("clear");
			help_main_menu();
		}
		if (main_cmd == 'o')		// vac on
		{
osc:		gpioWrite (Sync_pin, 1);
			gpioWrite (Sync_pin, 0);
			goto osc;
		}	
		if (main_cmd == 'v')		// change cathode voltage
		{
			printf(" cathode voltage - ");
			scanf(" %f", &Vcathode);
			Vcathode = fabs(Vcathode);
			printf("pellet = %d  cathode = %f \n",pelletnumber, Vcathode);
		}			
		if (main_cmd == 'a') {		// puff of H2
			mass_flow(1,H2_in);	//regulator setting on H2 tank
			adc_read( );
			serial_gage_read( );
			printf( "ammeter=%.6f gage=%.3f  H2=%.3f Vac=%3f", spi.adc[ammeter]*1000, vgage.gage, vgage.gage_hydrogen, vgage.gage_vacuum);
			fflush(stdout);
		}
		if (main_cmd == 's') {		// suck of vac
			mass_flow(1,Vac);
			adc_read( );
			serial_gage_read( );
			printf( "ammeter=%.6f gage=%.3f  H2=%.3f Vac=%3f", spi.adc[ammeter]*1000, vgage.gage, vgage.gage_hydrogen, vgage.gage_vacuum);
			fflush(stdout);
		}
	if (main_cmd == 'd')		// vac on
		{
			gpioWrite (Vac_pump, 1);
			gpioWrite (Vac,      1);
			adc_read( );
			serial_gage_read( );
			printf( "ammeter=%.6f gage=%.3f  H2=%.3f Vac=%3f", spi.adc[ammeter]*1000, vgage.gage, vgage.gage_hydrogen, vgage.gage_vacuum);
			fflush(stdout);
		}	
		if (main_cmd == 'f') {		// vac off
			gpioWrite (Vac_pump, 0);
			gpioWrite (Vac,      0);
			adc_read( );
			serial_gage_read( );
			printf( "ammeter=%.6f gage=%.3f  H2=%.3f Vac=%3f", spi.adc[ammeter]*1000, vgage.gage, vgage.gage_hydrogen, vgage.gage_vacuum);
			fflush(stdout);
		}
		if (main_cmd == 'h') {	// print help menu
			help_main_menu( );
		}
		if (main_cmd == 'p' && vgage.thread_status == not_running) { //gage
			vgage.thread_status = running;		// 
			vgage.set_point = 10.0;
			vgage.deadband = 2.0;					//2.65 regulates at 3.1T
			thread_v = gpioStartThread( _vac_thread, &vgage );
		}
		if (main_cmd == 'w')  {	// spi
			system("clear");
			help_s_menu( );
			spi.cmd = 0;
			do{
				spi.cmd = getch(500);
			}while(spi.cmd==0);
			
			printf("%s\n", &spi.cmd);
			if (spi.cmd == 'q') {		//quit any running spi programs
				spi.task_status = kill;
			}else {
				spi.task_status = running;
			}
			
			if(spi.thread_status == not_running) {
				spi.thread_status = running;	
				thread_s = gpioStartThread( _spi_thread, &spi );
			}
			system("clear");
			help_main_menu();
		}

	}while (main_cmd != 'z');


//kill any running threads and cleanup
	
	if (vgage.status == 1) {
		close(vgage.fd);
		vgage.status = 0;
	}	
	if (spi.thread_status == running) {	
		spi.thread_status = kill;
	}
	if (vgage.thread_status == running) {
		vgage.thread_status = kill;
	}

	usleep(100000);

	gpioStopThread( thread_s );
	gpioStopThread( thread_v );
	gpioWrite (H2_in,  0);
	gpioWrite (H2_out, 0);
	gpioWrite (Vac, 0);
	gpioWrite (Vac_pump, 0);
    gpioTerminate();
    fclose(spi.fp);


	clock_t toc = clock();
	printf("Elapsed: %f seconds\n", (double)(toc - tic) / CLOCKS_PER_SEC);

}

///////////////////////////////////////////
void *_spi_thread( void *ss )
{
//	spi_c* s = (spi_c*) ss;
	uint32_t count;
	
    time_t T= time(NULL);
    struct  tm tm = *localtime(&T);
    sprintf(date_time,"%04d%02d%02d-%02d%02d", tm.tm_year+1900, tm.tm_mon+1,
										tm.tm_mday,tm.tm_hour, tm.tm_min);
	count = 0;
	
	dac_init( );
	adc_init( 2 );	// mode 1 code does not work yet, use mode 2
	pll_Init( );
		
	if     (spi.cmd=='a'){spi.cmd=0;dac_adc_Test(); }	//cycles dac_test line 0v->2.44v
	else if(spi.cmd=='c'){spi.cmd=0;ce_Test(); } 		//chip enables with one byte data
	else if(spi.cmd=='v'){spi.cmd=0;gpio_Test( ); }		//valve test
	else if(spi.cmd=='g'){spi.cmd=0;sweep_grid( ); }		//sweep grid
	else if(spi.cmd=='m'){spi.cmd=0;grid_pulse(ammeter); }		//pulse grid
	else if(spi.cmd=='i'){spi.cmd=0;sweep_pressure( ); }	
	else if(spi.cmd=='n'){spi.cmd=0;proton_infusion(ammeter); }	
	else if(spi.cmd=='b'){spi.cmd=0;sweep_cathode( ); }	
	else if(spi.cmd=='k'){spi.cmd=0;dac_spare_test( ); }	
	else if(spi.cmd=='o'){spi.cmd=0;opamp_test( ); }	
	else if(spi.cmd=='s'){spi.cmd=0;serial_test( ); }	//serial i/o test
	else if(spi.cmd=='t'){spi.cmd=0;att_Test( ); }		//just sets the PE43711 attenuator
	else if(spi.cmd=='l'){spi.cmd=0;logamp_Test( ); }	//reads voltages on logamps
//	else if(spi.cmd=='d'){spi.cmd=0;dac_Sweep( 0,  1.1, 0.1, 10000, cathode,2,      981);} 
	else if(spi.cmd=='d'){spi.cmd=0;dac_Sweep( 0,  10, 0.2, 1000000, cathode,20,   1000000);}	// for 1meg
//	else if(spi.cmd=='d'){spi.cmd=0;dac_Sweep( 0,  10,  2, 10000, cathode,4,  10000000);} // for 10meg
//	else if(spi.cmd=='d'){spi.cmd=0;dac_Sweep( 0,  10,  2, 10000, cathode,4,1000000000);} // for 1g
//	else if(spi.cmd=='d'){spi.cmd=0;dac_Sweep( 0, 2.0,0.5, 10000, dac_spare,4, 10000000);} 
	else if(spi.cmd=='p'){spi.cmd=0;pll_Test( ); }	
	else if(spi.cmd=='f'){spi.cmd=0;find_resonance( 100, 200, 5, 1000);}

	usleep(100000);
	clock_t toc = clock();
//	fprintf(spi.fp, "spi: %d,  %s,   %f seconds\n", count, &spi.cmd, 
//	(double)(toc - spi.last) / CLOCKS_PER_SEC);
	spi.last = toc;
	count++;
	
	spi.thread_status = not_running ;
	spi.task_status = not_running;

	pthread_exit(NULL);
}

///////////////////////////////////////////
void *_vac_thread( void *vv )
{
	vac* v = (vac*) vv; 				// cast the void* to struct type
  
	int32_t h2_in_delay=10000, h2_out_delay=5000, pump_delay=100000, vac_delay=40000;
	int32_t dwell_time=800000, count=0;
	FILE *fp;
	float_t hi=v->set_point+v->deadband/2,low=v->set_point-v->deadband/2;

	fp = fopen( "gage.dat", "w+" );

MEASURE: 
	if (v->thread_status == kill ) { goto EXIT; }

	usleep(dwell_time);	
	adc_read( );
	v->gage = vgage.gage_hydrogen;
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
void my_gpio_init( )
{
	
	return;
}
///////////////////////////////////////////
void my_gpio_write(int32_t gpio, int32_t val)
{
	
	return;
}

///////////////////////////////////////////
void range_init( )
{
/* The non-inverting output of the MCP23S08 drives the inverting MC1413 
 * which drives the active low UB2 DPDT switch. So, a GPIO '0' leaves 
 * the switch in the 'off' position, which selects the 470 ohm sense 
 * resistor for the 5ma range. Always start with the 5ma range, then 
 * move the the more sensitive ranges to avoid overdriving the ADC
*/

	spi.range_mask_clear[0] = 0xfc;		// set the 2-bit range masks
	spi.range_mask_clear[0] = 0xf3;		// for each channel
	spi.range_mask_clear[0] = 0xcf;
	spi.range_mask_clear[0] = 0x3f;
	
	buff[0] = 0x40;				// send command first
	buff[1] = 0x00;				// select IO_DIR_REG
	buff[3] = 0x00;				// set IO_DIR_REG to all output
	spiWrite(spi.range_fd, buff, 3); 
	
	spi.range_out = 0x00;
	buff[0] = 0x40;				// send command first
	buff[1] = 0x09;				// Select gpio output latch
	buff[3] = spi.range_out;	// output to all zero (UB2 switchs off)
	spiWrite(spi.range_fd, buff, 3); 

	return;
}

///////////////////////////////////////////
void range_set(int32_t channel, int8_t max_current)
{
/* clears the channel two bit range value in the packed byte 
 * see range_init
*/
	spi.range_out &= spi.range_mask_clear[channel];
	spi.range_out |= max_current << (channel * 2);
	
	buff[0] = 0x40;				// send command first
	buff[1] = 0x09;				// Select gpio output latch
	buff[3] = spi.range_out;	// output to all zero (UB2 switchs off)
	spiWrite(spi.range_fd, buff, 3); 
	
	return;
}

///////////////////////////////////////////
void range_test( )
{
	char buff[4], buff_rx[4];
	
	range_init();
	
	spi.range_out = 0xff;
	range_set(ch0, range_5ma);
	fprintf( spi.fp, "chan%x = %x, %x\n", ch0, range_5ma, spi.range_out );

	buff[0] = 0x41;			// read command
	buff[1] = 0x09;			// read GPIO reg
	spiXfer(spi.range_fd, buff, buff_rx, 2);
	fprintf(spi.fp, "status= %x, %0x\n", buff_rx[0], buff_rx[1]); 
	return;
}
///////////////////////////////////////////

///////////////////////////////////////////
void dac_init( )
{
	char buff[4];
	
// set up channel names
	strcpy(spi.dac_names[0],"anode");
	strcpy(spi.dac_names[1],"e3bot");
	strcpy(spi.dac_names[2],"grid");
	strcpy(spi.dac_names[3],"e3top");
	strcpy(spi.dac_names[4],"cathode");
	strcpy(spi.dac_names[5],"e2");
	strcpy(spi.dac_names[6],"e1");
	strcpy(spi.dac_names[7],"dacspar");
	
	spi.dac_channel_rev6[0] = 0;	//anode   - rev6 has 4 channels
	spi.dac_channel_rev6[1] = 2;
	spi.dac_channel_rev6[2] = 1;	//grid
	spi.dac_channel_rev6[3] = 2;
	spi.dac_channel_rev6[4] = 3;	//cathode 	
	spi.dac_channel_rev6[2] = 2;	// map e1,e2, e3top, e3bot, & dacspare -> 2
	spi.dac_channel_rev6[2] = 2;
	spi.dac_channel_rev6[2] = 2;
	
/*
 * opamp gain is set by ratio of feedback resistor to input resistor
 *  510k/25.5k = 20
 * Measure vout/vin for each opamp channel and edit "20" below
 * 
 * note anode and grid are positive supplies, rest are negative
 * 
 * to calculate dac_offset, set dac_offset to zero and set dac to zero
 * then measure the voltage offset ( it could be positive or negative).
 * Now enter this offset into dac_offset for each channel.
 * 
 * 
 * 
*/
	if( board_rev == 2) spi.dac_bits = 65535;	// 2^16-1bit, ad5668 0->+vref
	if( board_rev == 6) spi.dac_bits = 32767;	// 2^15-1bit, ltc2664 is -vref->+vref

	spi.dac[anode] 			= 0;
	spi.dac_offset[anode] 	= 0;
	spi.dac_gain[anode] 	= spi.dac_bits  / 50 - spi.dac_offset[anode];
	spi.dac[grid] 			= 0;
	spi.dac_offset[grid] 	= 0;
	spi.dac_gain[grid] 		= spi.dac_bits  / 50 - spi.dac_offset[grid];
	spi.dac[e1] 			= 0;
	spi.dac_offset[e1] 		= 0;
	spi.dac_gain[e1] 		= spi.dac_bits  / 50 - spi.dac_offset[e1];
	spi.dac[e2] 			= 0;
	spi.dac_offset[e2] 		= 0;
	spi.dac_gain[e2] 		= spi.dac_bits  / 50 - spi.dac_offset[e2];	
	spi.dac[e3top] 			= 0;
	spi.dac_offset[e3top] 	= 0;
	spi.dac_gain[e3top] 	= spi.dac_bits  / 50 - spi.dac_offset[e3top];	
	spi.dac[e3bot] 			= 0;
	spi.dac_offset[e3bot] 	= 0;
	spi.dac_gain[e3bot] 	= spi.dac_bits  / 50 - spi.dac_offset[e3bot];	
	spi.dac[cathode] 		= 0;
	spi.dac_offset[cathode] = 0;
	spi.dac_gain[cathode] 	= spi.dac_bits  / 50 - spi.dac_offset[cathode];	
	spi.dac[dac_spare] 		= 0;
	spi.dac_offset[dac_spare] = 0;
	spi.dac_gain[dac_spare] = spi.dac_bits  / Vref - spi.dac_offset[dac_spare];	

	spi.dac_cmd[0] = 0x300000;		// inialize write thru command and address
	spi.dac_cmd[1] = 0x310000;		// for DACs 0-7
	spi.dac_cmd[2] = 0x320000;		// This value will be added to the 16bit data
	spi.dac_cmd[3] = 0x330000;		// shifted left by 4
	spi.dac_cmd[4] = 0x340000;
	spi.dac_cmd[5] = 0x350000; 
	spi.dac_cmd[6] = 0x360000;
	spi.dac_cmd[7] = 0x370000;

	buff[0] = 0x07;				// software reset
	buff[1] = 0x00;
	buff[2] = 0x00;
	buff[3] = 0x00;
	spiWrite(spi.dac_fd, buff, 4);

/*	buff[0] = 0x06;				// Override LDAC/ pin 
	buff[1] = 0x00;
	buff[2] = 0x00;
	buff[3] = 0x0f;
	spiWrite(spi.dac_fd, buff, 4);
*/




	return;
}


///////////////////////////////////////////
void dac_write(int32_t channel, float_t value )
{
// 
// use this equivalence union to access the integer as bytes
//   J    0x11223344	integer & byte alignment
//  CJ[]     3 2 1 0	
//	union equivs { uint32_t J; uint8_t CJ[4]; } eq;
	char buff[4];
	int16_t  v_dig;

/*
 * AD5668 DACs are limited to a positive 16bits (unsigned int16)
 * dac_init accounts for positive and negative supplies
 * 
 * LTC2446 has 4 16bit bipolar DACs with a Vref +-2.5V
 * value will be written as a signed integer and can be either 
 * positive or negative outputing +-50V.
 * 
*/
	if(board_rev == 2) {					// fix up unipolar vs bipolar limits
		spi.dac[channel] = value;			// only use +15bits 
		v_dig = value * spi.dac_gain[channel];
		if(v_dig > spi.dac_bits)   v_dig = spi.dac_bits;
			
		eq.J = spi.dac_cmd[channel] + v_dig;
		eq.J = eq.J <<4;		// leading and trailing 4bits are zero (!!)
		buff[0] = eq.CJ[3];		// send command first 
		buff[1] = eq.CJ[2];
		buff[2] = eq.CJ[1];
		buff[3] = eq.CJ[0];
		spiWrite(spi.dac_fd, buff, 4);	
	} 
	
	if(board_rev == 6) {
		channel = spi.dac_channel_rev6[channel];	//use +-15bits
		spi.dac[channel] = value;
		v_dig = value * spi.dac_gain[channel];
		if(v_dig > spi.dac_bits)   v_dig = spi.dac_bits;	
		if(v_dig < -spi.dac_bits)  v_dig = -spi.dac_bits;
		
		eq.J = spi.dac_cmd[channel] + v_dig;
		buff[0] = eq.CJ[3];		// uses same command and address as ad5668
		buff[1] = eq.CJ[2];
		buff[2] = eq.CJ[1];

/*		buff[0] = 0x30 + channel;  //this should make the same word
		buff[1] = v_dig >>8;
		buff[2] = v_dig & 0x00ff;
*/		spiWrite(spi.dac_fd, buff, 3); 	// might have to pad leading zero byte and send 4
	}
		

/*	eq.J = eq.J <<4;
	buff[0] = eq.CJ[3];		// send command first
	buff[1] = eq.CJ[2];
	buff[2] = eq.CJ[1];
	buff[3] = eq.CJ[0];
	spiWrite(spi.dac_fd, buff, 4); 
*/	fprintf( spi.fp, " dac ->%s = %.4f\n",  spi.dac_names[channel], spi.dac[channel] ) ;
	fprintf( spi.fp, "chan%x = %x, %x, %x, %x, %x\n", channel, v_dig, buff[0], buff[1], buff[2], buff[3] );
	
	return;
}

///////////////////////////////////////////
int serial_gage_init( )
/*
 * MKS910 Dual Trans vacuum gage supports 4800, 9600, 
 *      19200, 38400, 57600, 115200 and 230400 baud rates
 * TRS3232 RS232 tranceiver supports up to 250000 bps
 * 
 * Default adress AD! = 253
 * Default baud rate BR! = 9600
 * Default comm delay RSD! = on   (20ms)
 * 
 * 
 * 10 times per second is the fastest recommended pressure request frequency
 *
 * The pressure output query message is @253PR4?;FF which is 10 chars. 
 * The reply is @253ACK1.234E-4;FF which is 18 chars. 
 *  
 * 9600 baud is 960 char/sec. 
 * measurement time = 10/960 + 0.020 + 18/960 
 *                  = 0.0104 + 0.020 + 0.018
 *                  = 48.4ms 
 * so, a delay of 51.6ms would yield 10 pressure readings per second.
*/

{
	char *device;
/*
 * 	if no gage, connect Tx to Rx and uncomment loopback test above
 *  
 */
	vgage.gage = 0.0;
	vgage.status = 1;
	vgage.serBaud = 9600;
	vgage.serFlags = 0;
	device = "/dev/serial0";   // for usb, check /dev/serial/by-id/*
	strcpy(vgage.msg , "@253PR4?;FF" );

	if( ( vgage.fd = serOpen ( device , vgage.serBaud, vgage.serFlags )) <0) 
	{
		perror("serial device not opened\n");
		return 1;
	}
	return 0;
}


///////////////////////////////////////////
int serial_gage_read( )
{
	int32_t i, len, j ;
	char d[20] = {'b','i','l','b','o','\0'};
	char e[10] ;
	
/*
 * 	if no gage, connect Tx to Rx and uncomment loopback test above
 *  
 */

	len = strlen(vgage.msg);
	serWrite (vgage.fd, vgage.msg, len) ;
//	printf( " write gage=%s\n", vgage.msg);
    
    if( serDataAvailable( vgage.fd ) )
    { 
		serRead ( vgage.fd, d, 20) ;
//		printf( " read=%s\n", d );
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
		    adc_read();									// reads gage voltage
//		    vgage.gage_hydrogen = pow( 10, spi.adc[gage_Vac]/0.2446 - 6  );    //ratio of voltage divider
//			printf ( " string=%s   %.4f\n", e,vgage.gage );
//			printf ( " gage=%.4f  voltage= %.5f gage_Vac adc=%.5f temp=%.5f\n", vgage.gage, vgage.gage_hydrogen, spi.adc[4] );
//			printf ( " gage=%.4f  voltage= %.5f\n", vgage.gage, vgage.gage_hydrogen );
		}
	}

	return 0;
}

///////////////////////////////////////////
void   mass_flow(int32_t number_puffs, int32_t H2_vac)
{
	int32_t  puffs;

	puffs = 0;
	while ( puffs < number_puffs) { 

		if( H2_vac == H2_in ) {
			gpioWrite (H2_out, 0) ;
			gpioWrite (H2_in, 1) ;	
			usleep(20000);
			gpioWrite (H2_in, 0) ;				
			gpioWrite (H2_out, 1) ;
			usleep(20000);
			gpioWrite (H2_out, 0) ;
		}
		if( H2_vac == Vac ) {
			gpioWrite (Vac, 0) ;	
			gpioWrite (Vac_pump, 1) ;
			usleep(10000);
			gpioWrite (Vac_pump, 0) ;
			gpioWrite (Vac, 1) ;				
			usleep(10000);
			gpioWrite (Vac, 0) ;
		}
		if( H2_vac == Vac_pump ) {
			gpioWrite (Vac_pump, 1) ;
			gpioWrite (Vac, 1) ;	
			usleep(10000);
			gpioWrite (Vac_pump, 0) ;
			gpioWrite (Vac, 0) ;
		}

		puffs = puffs + 1;
	}
	return ;
}

///////////////////////////////////////////
int   sweep_pressure( )
{
	int32_t  gage_count, Vx100, i;
	float_t i_measured, x;
	char plot_cmd[96], plotfilename[32];

	FILE *I_fp;						// open plot file
	I_fp = fopen( "I.dat", "w+" );
	
	dac_write(cathode, Vcathode );	// ammeter is hooked to cathode

	gpioWrite(Vac, 1);				//suck out any residual H2 
	gpioWrite(Vac_pump, 1);
	usleep(2000000) ;				// 2 sec	

	adc_read( );					//clear out previous read
	
/*
 * mass_flow delivers a number of puffs of H2 
 * mount of H2 delivered depends of number of puffs and regulator setting on H2 tanks
 * or a number of sucks of vacuum
 */


	gage_count = 0;
	while (spi.task_status != kill && gage_count <100) { 

		if( gage_count == 20 ) mass_flow(1,Vac_pump);	// close the vacuum valves
		if( gage_count == 20 ) mass_flow(1,H2_in);		// puff of H2
		if( gage_count == 30 ) mass_flow(1,H2_in);		// puff of H2
		if( gage_count == 40 ) mass_flow(1,H2_in);		// puff of H2
		if( gage_count == 50 ) mass_flow(1,H2_in);		// puff of H2
		if( gage_count == 60 ) mass_flow(4,Vac_pump);		// puff of H2
		if( gage_count == 70 ) mass_flow(4,Vac_pump);		// puff of H2
		if( gage_count == 80 ) mass_flow(4,Vac_pump);		// puff of H2
		if( gage_count == 90 ) {gpioWrite(Vac_pump, 1);gpioWrite(Vac, 1);}	// suck out the H2 until next puff 
			
		i=0;
		while (i < 5) {		
			adc_read( );
			i_measured = spi.adc[ammeter] * 1000000;
			x = (gage_count*10 + i*2);
			x = x /100;			
			fprintf( I_fp, " %.4f %.1f %.8f\n", x, vgage.gage_hydrogen, i_measured);	
			usleep(9500) ;								//0.010sec	
			i++;
		}		
		gage_count++;	

	}
	spi.dac[cathode] = 0.0;	// done - now turn everything off
	gpioWrite(Vac, 0);
	gpioWrite(Vac_pump, 0);

	fclose(I_fp);

	Vx100 = Vcathode * -100;
    sprintf(plotfilename,"P%2dv%04d-%.13s.png", pelletnumber, Vx100,date_time);
	printf("making plot file %s\n",plotfilename);

	gnuplot_ctrl    *h1 ;
	h1 = gnuplot_init() ;
	gnuplot_cmd(h1, "set terminal png size 1200,600") ;

    sprintf(plot_cmd,"set output '%s'",plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
    sprintf(plot_cmd,"set title 'Proton Current at %.1fV %s",Vcathode, plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;

	gnuplot_cmd(h1, "set margins screen 0.1, screen 0.8, screen 0.1, screen 0.94") ;
	gnuplot_cmd(h1, "set xlabel 'Time sec' ") ;

	gnuplot_cmd(h1, "set ylabel 'H2 Pressure - Torr' tc 'web-green' ") ;
	gnuplot_cmd(h1, "set ytics nomirror tc 'web-green' ") ;
	gnuplot_cmd(h1, "set autoscale y ") ;
	gnuplot_cmd(h1, "set y2label 'Proton Current - microamps' tc 'web-blue' ") ;
	gnuplot_cmd(h1, "set y2tics nomirror tc 'web-blue' ") ;
	gnuplot_cmd(h1, "set autoscale y2 ") ;
   	gnuplot_cmd(h1, "set key left top") ;
	gnuplot_cmd(h1, "set grid x y2 ") ;
	gnuplot_cmd(h1, "plot './I.dat' using 1:2 with linespoints title 'pressure' axes x1y1 lw 2 lc 'web-green', \
						  './I.dat' using 1:3 with linespoints title 'current'  axes x1y2 lw 1 lc 'web-blue' ") ;

	gnuplot_close(h1) ;
	printf("done\n");

	return 0;
}

///////////////////////////////////////////
int   grid_pulse()
{
	int32_t  gage_count, Vx100, i;
	float_t i_measured, Vgrid,x;
	char plot_cmd[96], plotfilename[32];


	FILE *I_fp;			// open plot file
	I_fp = fopen( "I.dat", "w+" );
	
//	spi.dac[cathode] = 0.0;		//turn off power supplies
	spi.dac[cathode] = Vcathode;	//turn on power supplies
	Vgrid = 48;

	dac_write(e1, 0.0 );
	dac_write(anode, 0.0 );
	adc_read( );				//clear out previous read

	gpioWrite(Vac, 1);			//suck out any residual H2 
	gpioWrite(Vac_pump, 1);

/*
 * mass_flow delivers a number of puffs of H2 
 * mount of H2 delivered depends of number of puffs and regulator setting on H2 tanks
 * or a number of sucks of vacuum
 */

	gage_count = 0;
	while (spi.task_status != kill && gage_count <60) { 

/*		if( gage_count ==  2 ) dac_write(e1,   Vgrid);
		if( gage_count ==  4 ) dac_write(e1,   0.0  );	
*/		if( gage_count == 0 ) dac_write(grid, Vgrid);	
		if( gage_count == 2 ) dac_write(grid, 0.0);	
		if( gage_count == 4 ) dac_write(anode, 50);
		if( gage_count == 8 ) dac_write(anode, 0.0);		
		if( gage_count == 18 ) {gpioWrite(Vac, 0); gpioWrite(Vac_pump, 0);}	// turn off vacuum
		
		if( gage_count == 29 ) mass_flow(3,H2_in);				// puff of H2
		if( gage_count == 31 ) mass_flow(3,H2_in);				// puff of H2
		if( gage_count == 33 ) mass_flow(2,H2_in);				// puff of H2
		if( gage_count == 35 ) mass_flow(2,H2_in);				// puff of H2
		if( gage_count == 33 ) {dac_write(grid, Vgrid);} 		// pulse grid
		if( gage_count == 37 ) {dac_write(grid, 0.0);}	
//		if( gage_count == 35 ) {dac_write(cathode, Vcathode);} 	// pulse cathode
//		if( gage_count == 43 ) {dac_write(cathode, 0.0);}
		if( gage_count == 35 ) {dac_write(anode, 50);}			// pulse anode
		if( gage_count == 43 ) {dac_write(anode, 0.0);}		
				
//		if( gage_count == 40 ) {gpioWrite(Vac_pump, 1);gpioWrite(Vac, 1);}	// suck out the H2 until the end

		
		i=0;
		while (i < 5) {
			adc_read( );
			i_measured = spi.adc[ammeter] * 1000000;	//microamps
			x = (gage_count*10 + i*2);
			x = x /100;
			fprintf( I_fp, " %.2f %.1f %.3f %.1f %.1f\n", x, vgage.gage_hydrogen, 
						i_measured, spi.dac[grid], spi.dac[anode] );	
			usleep(10000) ;								//0.010sec	
			i++;
		}
		gage_count++;	

	}


	dac_write(e1, 0.0);			// done - now turn everything off
	dac_write(cathode, 0.0);
	dac_write(anode, 0.0);
	gpioWrite(Vac, 0);
	gpioWrite(Vac_pump, 0);

	fclose(I_fp);

	gnuplot_ctrl    *h1 ;
	h1 = gnuplot_init() ;
	
	Vx100 = Vcathode * -100;
    sprintf(plotfilename,"P%2dv%04d-%.13s.png", pelletnumber, Vx100,date_time);
	printf("making plot file %s\n",plotfilename);

	gnuplot_cmd(h1, "set terminal png size 1200,600") ;
    sprintf(plot_cmd,"set output '%s'",plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
    sprintf(plot_cmd,"set title 'Proton Current at %.1fV %s",Vcathode, plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;

	gnuplot_cmd(h1, "set margins screen 0.1, screen 0.8, screen 0.1, screen 0.94") ;
	gnuplot_cmd(h1, "set multiplot") ;

	gnuplot_cmd(h1, "set xlabel 'Time sec' ") ;

	gnuplot_cmd(h1, "set ylabel 'H2 Pressure - Torr' tc 'web-green' ") ;
	gnuplot_cmd(h1, "set ytics nomirror tc 'web-green' ") ;
	gnuplot_cmd(h1, "set autoscale y ") ;
	gnuplot_cmd(h1, "set y2label 'Proton Current - microamps' tc 'web-blue' ") ;
	gnuplot_cmd(h1, "set y2tics nomirror offset -1,0 tc 'web-blue' ") ;
	gnuplot_cmd(h1, "set autoscale y2 ") ;
   	gnuplot_cmd(h1, "set key left top") ;
	gnuplot_cmd(h1, "set grid x y2 ") ;
	gnuplot_cmd(h1, "plot './I.dat' using 1:2 with linespoints title 'pressure' axes x1y1 lw 1 lc 'web-green', \
						  './I.dat' using 1:3 with linespoints title 'current'  axes x1y2 lw 1 lc 'web-blue' ") ;

	gnuplot_cmd(h1, "unset title") ;
	gnuplot_cmd(h1, "unset xlabel") ;
	gnuplot_cmd(h1, "unset ylabel") ;
	gnuplot_cmd(h1, "unset y2label") ;
	gnuplot_cmd(h1, "unset tics") ;
	gnuplot_cmd(h1, "unset grid") ;
   	gnuplot_cmd(h1, "set key right top") ;
	gnuplot_cmd(h1, "set yrange[0:100]") ;
	gnuplot_cmd(h1, "set y2range[0:100]") ;
	gnuplot_cmd(h1, "plot './I.dat' using 1:4 with linespoints title 'Grid'    axes x1y1 lc 'red', \
						  './I.dat' using 1:5 with linespoints title 'Anode' axes x1y2 lc 'dark-violet' ") ;

   	gnuplot_cmd(h1, "set key off") ;
	gnuplot_cmd(h1, "set rmargin at screen 0.9") ;
	gnuplot_cmd(h1, "set border 8") ;
	gnuplot_cmd(h1, "set y2label 'Voltage'  offset -1,0 tc 'red'") ;
	gnuplot_cmd(h1, "set y2tics nomirror offset 0,0 tc 'red'") ;
	gnuplot_cmd(h1, "set yrange[0:100]") ;
	gnuplot_cmd(h1, "plot  sqrt(x*.001) lc 'grey'") ;
	
 	gnuplot_cmd(h1, "unset multiplot") ;
	gnuplot_cmd(h1, "unset output") ;

	gnuplot_close(h1) ;
	printf("done\n");

	return 0;
	
}
///////////////////////////////////////////
void   proton_infusion( int32_t adc_channel )

{
	int32_t  time_count, Vx100, next_off, next_on, time_out, end;
	int32_t  time_at_last_Imax, time_at_last_trigger;
	float_t tictoc, local_Imax, Inow,last_Imax, global_Imax,i_measured;
	char plot_cmd[96], y2label_cmd[80], plotfilename[32], units[4];


/*
	printf(" pellet number? \n");
//	fflush(stdout);
	scanf(" %d\n", &pelletnumber);
//	fflush(stdin);
	printf(" cathode voltage - \n");
//	fflush(stdout);
	scanf(" %f\n", &Vcathode);
//	fflush(stdin);
	if(Vcathode > 0 ) Vcathode = -Vcathode;
	printf(" pellet = %d  cathode = %f \n",pelletnumber, Vcathode);
//	fflush(stdout);
*/


// serial_gage_init is called at top of main
// amp_scale was calculated in dac_Sweep

	Vx100 = abs(Vcathode * 100);
	if( adc_channel == ammeter ) {
		strcpy ( units, "mA" );
		strcpy (y2label_cmd, "set y2label 'Proton Current - mA' ");
		sprintf(plotfilename,"HI%2dv%04d-%.13s.png",pelletnumber, Vx100,date_time);		
	}
	
	FILE *I_fp;			// open plot file
	I_fp = fopen( "I.dat", "w+" );
	
//	Vcathode= -40.0;
//	dac_write(cathode, Vcathode);	// ammeter is hooked to cathode
//	serial_gage_read( );

	gpioWrite(Vac, 1);			//suck out any residual H2 
	gpioWrite(Vac_pump, 1);
	usleep(50000);
	gpioWrite(Vac, 0);
	gpioWrite(Vac_pump, 0);
	usleep(50000);
		
	clock_t tic = clock();

	local_Imax = 0.0 ;
	global_Imax = 0.0 ;
	end = 200;			//duration of this H2 injection - usleep sets the time_count length
	next_off = end;
	next_on = 2;		//first H2 injection
	time_out = 0;		// flag to suspend H2 injection
	time_count = 0;
	while (spi.task_status != kill && time_count < end ) { 

		if( time_count*100 % end == 0 || time_count == 0) {	
			printf("\r H2 = %3d%%   %.6f %s ", time_count * 100 /end, local_Imax,units);
			fflush(stdout);
		}
		
		if( time_count == next_on ) {
			gpioWrite (H2_in, 1) ;	
			gpioWrite (H2_out, 1) ;
			time_at_last_trigger = time_count;	// new cycle
			last_Imax = local_Imax;				
			local_Imax = Inow;					// reset local_max for this cycle
			next_off = time_count + 1;	// schedule off 
			time_out = time_count + 10;	// schedule next cycle evaluation
		}
		if( time_count == next_off ) {
			gpioWrite (H2_in, 0) ;	
			gpioWrite (H2_out, 0) ;
		}

		adc_read( );
		i_measured = spi.adc[adc_channel] * 1000000;
		Inow = i_measured *10;
		clock_t toc = clock();
		tictoc = (double)(toc - tic) / CLOCKS_PER_SEC;
		fprintf( I_fp, " %.4f %.1f %.6f\n", tictoc, vgage.gage_hydrogen, Inow);	
		usleep(1500000) ;
		time_count++;		
			
		if( Inow > local_Imax) {				// current is rising
			local_Imax = Inow;  
			time_at_last_Imax = time_count-1;	//remember when at local max		
			if( local_Imax > global_Imax ) global_Imax = local_Imax;	//promote to global
			
		}else {									// current is falling
			if(time_count > time_out) {
				if( time_at_last_Imax > time_at_last_trigger ){	// had a max this cycle		
					if(Inow < local_Imax * 0.9 ) next_on = time_count;	// trigger H2 injection
					if(time_count-time_at_last_trigger > 30) next_on = time_count;
				} else {						// did not have a Imax this cycle
					if(local_Imax < last_Imax * 0.8 ) next_on = time_count;	// trigger H2 injection
					if(time_count > time_at_last_trigger + 20)  next_on = time_count;
				}
	//			if( fabs(local_Imax - last_Imax)/last_Imax < 0.05 ) time_out=time_count + 10;//snooze if not much change
			}								
		}
	}
	
	gpioWrite (H2_in, 0) ;	
	gpioWrite (H2_out, 0) ;	
	fclose(I_fp);

	printf("\n   making plot file %s\n",plotfilename);
	gnuplot_ctrl    *h1 ;
	h1 = gnuplot_init() ;
	gnuplot_cmd(h1, "set terminal png size 1600,400") ;
    sprintf(plot_cmd,"set output '%s'",plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;	
    sprintf(plot_cmd,"set title 'Proton Infusion max=%.3fmS %s",global_Imax, plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;

	gnuplot_cmd(h1, "set margins screen 0.1, screen 0.8, screen 0.1, screen 0.94") ;
	gnuplot_cmd(h1, "set xlabel 'Time sec' ") ;

	gnuplot_cmd(h1, "set ylabel 'H2 Pressure - Torr' tc 'web-green' ") ;
	gnuplot_cmd(h1, "set ytics nomirror tc 'web-green' ") ;
	gnuplot_cmd(h1, "set autoscale y ") ;
	gnuplot_cmd(h1, "set y2label 'Proton Current - microamps' tc 'web-blue' ") ;
	gnuplot_cmd(h1, "set y2tics nomirror tc 'web-blue' ") ;
	gnuplot_cmd(h1, "set autoscale y2 ") ;
   	gnuplot_cmd(h1, "set key left top") ;
	gnuplot_cmd(h1, "plot './I.dat' using 1:2 with linespoints title 'pressure' axes x1y1 lw 2 lc 'web-green', \
						  './I.dat' using 1:3 with linespoints title 'current'  axes x1y2 lw 1 lc 'web-blue' ") ;

	gnuplot_close(h1) ;
	printf("done\n");

	return ;
}

///////////////////////////////////////////
int   sweep_grid()
{
	int32_t count, Vx100 ;
	char plot_cmd[80], plotfilename[32];

// grid is actually hooked to e1

	FILE *I_fp;			// open plot file
	I_fp = fopen( "I.dat", "w+" );
	Vx100 = fabs(Vcathode *100);
	dac_write(anode, 0.0);
	dac_write(cathode, Vcathode );
	
	count = 0;
	while (count <= 10 ) {
		spi.dac[e1] = Vcathode * (float)(count)/10.0;	// ammeter is hooked to cathode
		dac_write(e1, spi.dac[e1] );
		usleep(200000) ;
		adc_read( );
		fprintf( I_fp, " %.6f %.8f\n", spi.dac[e1], spi.adc[ammeter]*1000000 );	// diff_amp voltage is divided by r_sense in adc_read
		count++ ;
	}

	dac_write(cathode, 0.0);	//turn off power supplies
	dac_write(e1, 0.0);
	dac_write(anode, 0.0);

	fclose(I_fp);


    sprintf(plotfilename,"A%2dv%04d-%.13s.png", pelletnumber, Vx100, date_time);
	printf("making plot file %s\n",plotfilename);
	gnuplot_ctrl    *h1 ;
	h1 = gnuplot_init() ;
	gnuplot_cmd(h1, "set terminal png size 800,600") ;
    sprintf(plot_cmd,"set output '%s'",plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
    sprintf(plot_cmd,"set title ' ammeter vs. grid  - %s", plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;

	gnuplot_cmd(h1, "set xlabel 'Time sec' ") ;

	gnuplot_cmd(h1, "set ylabel 'Proton Current - microamps' tc 'web-blue' ") ;
	gnuplot_cmd(h1, "set ytics nomirror tc 'web-blue' ") ;
   	gnuplot_cmd(h1, "set key left top") ;
	gnuplot_cmd(h1, "set autoscale y ") ;
	gnuplot_cmd(h1, "set grid x y ") ;
	gnuplot_cmd(h1, "plot './I.dat' using 1:2 with linespoints title 'current' axes x1y1 lc 'web-blue' ") ;

	gnuplot_close(h1) ;
	printf("done\n");

	return 0;	
}
///////////////////////////////////////////
int   sweep_cathode( )
{
	int32_t count, Vx100 ;
	char plot_cmd[80], plotfilename[32];
	
	FILE *I_fp;			// open plot file
	I_fp = fopen( "I.dat", "w+" );
	Vx100 = fabs(Vcathode *100);
	adc_read( );						//clear out previous read
	
	count = 0;
	while (count <= 100 ) {
		spi.dac[cathode] = Vcathode * (float)(count)/100.0;	// ammeter is hooked to cathode
		dac_write(cathode, spi.dac[cathode]);
		usleep(5000) ;
		adc_read( );
		fprintf( I_fp, " %.6f %.8f\n", spi.dac[cathode], spi.adc[ammeter]*1000 );	// diff_amp voltage is divided by r_sense in adc_read
		count++ ;
	}

	dac_write(cathode, 0.0 );	//turn off cathode
	
	fclose(I_fp);


    sprintf(plotfilename,"A%2dv%04d-%.13s.png", pelletnumber, Vx100, date_time);
	printf("making plot file %s\n",plotfilename);
	gnuplot_ctrl    *h1 ;
	h1 = gnuplot_init() ;
	gnuplot_cmd(h1, "set terminal png size 800,600") ;
    sprintf(plot_cmd,"set output '%s'",plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
    sprintf(plot_cmd,"set title ' ammeter vs. cathode  - %s", plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
	
	gnuplot_cmd(h1, "set xlabel 'V cathode' ") ;
	gnuplot_cmd(h1, "set ylabel 'I mA' tc 'web-blue' ") ;
	gnuplot_cmd(h1, "set ytics nomirror tc 'web-blue' ") ;
	gnuplot_cmd(h1, "set autoscale y ") ;
	gnuplot_cmd(h1, "set grid x y ") ;
   	gnuplot_cmd(h1, "set key left top") ;
	
	gnuplot_cmd(h1, "plot './I.dat' using 1:2 with linespoints title 'ammeter' axes x1y1 lc 'web-blue'  ") ;
	gnuplot_close(h1) ;
	printf("done\n");


	return 0;
	
}
///////////////////////////////////////////
int   opamp_test()
{
	int32_t  gage_count;
	float_t    Vset;

	Vset = 50;
	dac_write(cathode, Vcathode);	//turn on power supplies

	gage_count = 0;
	while (spi.task_status != kill) { 

loop:	usleep(1000) ;
		gage_count++;
		if( gage_count ==  1 ) gpioWrite (Sync_pin, 1);
		if( gage_count == 20 ) gpioWrite (Sync_pin, 0);
		if( gage_count ==  2 ) {dac_write(e1, Vset);}
		if( gage_count ==  4 ) {dac_write(e1, 0.0);}	
		if( gage_count == 20 ) {dac_write(grid, Vset);}
		if( gage_count == 22 ) {dac_write(grid, Vset);}
		if( gage_count == 24 ) {dac_write(anode, Vset);}
		if( gage_count == 28 ) {dac_write(anode, 0.0);}		

		if( gage_count == 28 ) gage_count = 0;
		goto loop;	

	}

// done - now turn everything off

	dac_write(e1, 0.0);
	dac_write(grid, 0.0);
	dac_write(anode, 0.0);
	dac_write(cathode, 0.0);

	gpioWrite(Vac, 		0);
	gpioWrite(Vac_pump, 0);

	return 0;
	
}
///////////////////////////////////////////

int   dac_spare_test( )
{
	int32_t count, Vx100 ;
	char plot_cmd[80], plotfilename[32];
	float_t Vset;

	FILE *I_fp;			// open plot file
	I_fp = fopen( "I.dat", "w+" );
	Vx100 = abs(Vcathode * 100);
	
	count = 0;
	while (count <= 10 ) {
		Vset = Vcathode * (float)(count)/10.0;	
		dac_write(dac_spare, Vset );
		usleep(20000) ;
		adc_read( );
		fprintf( I_fp, " %.8f %.8f \n", Vset, spi.adc[adc_spare]  );
		count++ ;
	}

	dac_write(dac_spare, 0.0);
	
	fclose(I_fp);


    sprintf(plotfilename,"A%2dv%04d-%.13s.png", pelletnumber, Vx100, date_time);
	printf("making plot file %s\n",plotfilename);
	gnuplot_ctrl    *h1 ;
	h1 = gnuplot_init() ;
	gnuplot_cmd(h1, "set terminal png size 800,600") ;
    sprintf(plot_cmd,"set output '%s'",plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
    sprintf(plot_cmd,"set title ' dac_spare vs adc_spare %s", plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
	
	gnuplot_cmd(h1, "set xlabel 'V dac-spare' ") ;
	gnuplot_cmd(h1, "set y2label 'V adc-spare' tc 'web-blue'  ") ;
	gnuplot_cmd(h1, "set y2tics nomirror tc 'web-blue' ") ;
	gnuplot_cmd(h1, "set grid x y ") ;
	gnuplot_cmd(h1, "set autoscale y2 ") ;
	gnuplot_cmd(h1, "plot './I.dat' using 1:2 with linespoints axes x1y2 lc 'web-blue' ");

	gnuplot_close(h1) ;
	printf("done\n");


	return 0;
	
}

///////////////////////////////////////////	
void find_resonance(uint32_t start, uint32_t end, uint32_t df, uint32_t dt){
						// start, df, end in MHz,   dt in usec
	uint32_t megahz, for_max_mhz, for_min_mhz, ref_max_mhz, ref_min_mhz;
	uint32_t i,num_points;
	float_t for_max, for_min, ref_max, ref_min;
	float_t forward_db, reflected_db;
		
	megahz = start;
	for_max = 0;
	for_min = Vref;
	ref_max = 0;
	ref_min = Vref;
	for_max_mhz = start;
	for_min_mhz = start;
	ref_max_mhz = start;
	ref_min_mhz = start;
	

	FILE *res_fp;
	res_fp = fopen( "res.dat", "w+" );
	FILE *res_d_fp;
	res_d_fp = fopen( "res_d.dat", "w+" );
	i=0;


					//sweep up
	while ( megahz <= end && spi.task_status != kill) {

		i=i+1;
		pll_SetIntfreq( megahz );
		usleep(10);
		adc_read();
		if( spi.adc[forward] > for_max) {
			for_max = spi.adc[forward] ;
			for_max_mhz = megahz; }
		if( spi.adc[reflected] > ref_max) {
			ref_max = spi.adc[reflected] ;	
			ref_max_mhz = megahz; }
		if( spi.adc[forward] < for_min) {
			for_min = spi.adc[forward] ;
			for_min_mhz = megahz; }
		if( spi.adc[reflected] < ref_min) {
			ref_min = spi.adc[reflected] ;	
			ref_min_mhz = megahz; }

		spi.sw_freq[i] = megahz;						//stuff the sweep arrays
		spi.sw_forward[i] = spi.adc[forward];			//do all the calc in voltages
		spi.sw_reflected[i] = spi.adc[reflected];
		
		forward_db = spi.adc[forward] /0.021 - 87; //convert to dBv
		reflected_db = spi.adc[reflected] /0.021 - 87;
		fprintf( res_fp, " %d %.4f %.4f\n", megahz, forward_db, reflected_db);	
		usleep(dt);
		megahz = megahz + df;
	}
	
	num_points = i;				
	spi.sw_forward[0] = spi.sw_forward[1];						// fix up the end points
	spi.sw_reflected[0] = spi.sw_reflected[1];					// so first and last derivatives
	spi.sw_forward[num_points+1] = spi.sw_forward[num_points];	// are reasonable
	spi.sw_reflected[num_points+1] = spi.sw_reflected[num_points];
		
	for ( i=1; i <= num_points; i=i+1) {				// now calculate first derivatives
		spi.sw_d_forward[i] = (spi.sw_d_forward[i+1] - spi.sw_d_forward[i-1]) / 2;
		spi.sw_d_reflected[i] = (spi.sw_d_reflected[i+1] - spi.sw_d_reflected[i-1]) / 2;
		fprintf( res_d_fp, " %d %.4f %.4f\n", spi.sw_freq[i], spi.sw_d_forward[i], spi.sw_d_reflected[i]);				
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
	gnuplot_cmd(h1, "plot './res.dat' using 1:2 with lines title 'forward DB', \
						  './res.dat' using 1:3 with lines title 'reflected DB' \
						  './res_d.dat' using 1:2 with lines title 'forward delta', \
						  './res_d.dat' using 1:3 with lines title 'reflected delta'  ") ;

	sleep (20 );
	gnuplot_close(h1) ;
	return;

}




///////////////////////////////////////////	
void pll_write( uint32_t data ) {
	char buff[4];
	union equivs { uint32_t J; uint8_t CJ[4]; } eq;
	eq.J = data;
	buff[0] = eq.CJ[3];
	buff[1] = eq.CJ[2];
	buff[2] = eq.CJ[1];
	buff[3] = eq.CJ[0];

	spiWrite(spi.pll_fd, buff, 4);
//	fprintf( spi.fp, " spi_wr = %x  buff= %x, %x, %x, %x\n",
//					eq.J, buff[0], buff[1], buff[2], buff[3] );	
}

///////////////////////////////////////////
void pll_Test( ){		
/* 
 * Connect RF-OUT SMA to the FORWARD SMA
 * checks PLL, ATT, and logamp
 * Alternatively, connect RF-OUT to scope
 * 
 */	
	int loop=0;

 	pll_Init( );
//	pll_SetIntfreq( 60 );	
	gpioWrite (RF_en, 1) ;	
	
	while(loop < 20 && spi.task_status != kill) {
//		loop++;
			
		gpioWrite (Sync_pin,  1);		// use gpio4 as a sync for the o´scope
		pll_SetIntfreq( 30 );			
		usleep(1000) ;

		gpioWrite (Sync_pin, 0);		
		pll_SetIntfreq( 50 );			
		usleep(1000) ;
	}
	
	gpioWrite (RF_en, 0) ;
	return ;	
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
//		fprintf( spi.fp, " PLL reg0-5 = %x, %x, %x, %x, %x, %x\n",
//				spi.pll_reg[0],spi.pll_reg[1],spi.pll_reg[2],spi.pll_reg[3],spi.pll_reg[4],spi.pll_reg[5] );			

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
			if (spi.task_status == kill) { return; }
			pll_SetIntfreq( megahz );
			usleep(dt);
			megahz = megahz + df;
		}
	}
	else {									//sweep down
		while ( megahz > start ) {
			if (spi.task_status == kill) { return; }		
			pll_SetIntfreq( megahz );
			usleep(dt);
			megahz = megahz - df;
		}
	}
	spi.cmd = 0;
	return;
}

///////////////////////////////////////////
void pll_SetIntfreq( int32_t megahz )
{	
	uint32_t Fref,diva,N;
	Fref = 10;						//xtal reference frenquency in MHz
	diva = -1;						// find the DivA that 300<N<600
	while( N < 300 ) {
		diva = diva + 1;
		N = megahz / Fref * 1<<diva ;	//  freq is frequency in MHz
	}

	pll_SetN(N);
	pll_SetDIVA(diva);
}

///////////////////////////////////////////
int* ce_Test( )
{
	char buff[4], buff_rx[4];	
	buff[0] = 0x55;
	buff_rx[0] = 0x99;

	while (spi.task_status != kill) { 
		spiWrite(spi.dac_fd, buff, 1);
		usleep(20);
		spiXfer(spi.adc_fd, buff, buff_rx, 1);
		usleep(20);
		spiWrite(spi.att1_fd, buff, 1);
		usleep(20);
		spiWrite(spi.pll_fd, buff, 1);
		usleep(20);
		spiWrite(spi.att2_fd, buff, 1);
		usleep(20);	
		gpioWrite(Sync_pin, 1);
		usleep(20);
		gpioWrite(Sync_pin, 0);
		usleep(20);	}
	return 0;
}

///////////////////////////////////////////
void att_Set( float_t atten )
{

	char buff[4];
	union equivs { uint32_t J; uint8_t CJ[4]; } eq;

	atten = abs(atten);
	if(atten < 0.0) atten = 0.0;
	if(atten > 31.75) atten = 31.75;
	eq.J = (int) (atten*4);				// attenuation in quarter dB steps
	buff[0] = eq.CJ[0];
	spiWrite(spi.att1_fd, buff, 1);
	spiWrite(spi.att2_fd, buff, 1);

	return;
}
///////////////////////////////////////////
void att_Test( ) {
//	Put a -30 db attenuator on forward input and jumper to RF-OUT 
	float_t delta;
	
	pll_SetIntfreq( 100 );
	
	delta = 0.0;
	while(delta <= 32.0){
		att_Set( delta );	
		adc_read();
		fprintf( spi.fp, " forward  delta= %.4f,  db= %.4f\n", delta,spi.DB[forward] );
		delta = delta + 4.0;
		usleep(1000);
	}
	return;
}

///////////////////////////////////////////
void att_Sweep(float_t start, float_t end, float_t df, uint32_t dt){
// start, df, end in dbm,   dt in usec

	float_t atten;
	atten = start;
	if ( start < end ) {			//sweep up
down:	att_Set( atten );
		usleep(dt);
		atten = atten + df;
		if ( atten < end ) goto down;
	}
	else {
up:		att_Set( atten );
		usleep(dt);
		atten = atten - df;
		if ( atten > end ) goto up;
	}
	spi.cmd = 0;
	return;
}

///////////////////////////////////////////
void adc_init( uint32_t mode )
{
	char buff[4];

	spi.adc_reg[0] = 0xdd;
	spi.adc_reg[1] = 0xdf;
	spi.adc_reg[2] = 0xe1;
	spi.adc_reg[3] = 0xe3;
	spi.adc_reg[4] = 0xe5;
	spi.adc_reg[5] = 0xe7;
	
	strcpy(spi.adc_names[0],"forward");
	strcpy(spi.adc_names[1],"reverse");
	strcpy(spi.adc_names[2],"ammeter");
	strcpy(spi.adc_names[3],"spare");
	strcpy(spi.adc_names[4],"amm2");
	strcpy(spi.adc_names[5],"dactest");
	
	
if(mode==1){
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
	
}


	if(mode==2){

/*CTL1 =C2                   2    E
 * perform self calibration	00
 * powerdown =STANDBY		  10
 * unipolar					    1
 * format = offset binary 	     1
 * Scycle = single cycle		  1
 * Contsc = single cycle           0  */
	buff[0] = 0xc2;		// CTRL1
	buff[1] = 0x2e;		// CTRL1 data
//	buff[1] = 0x2f;		// CTRL1 data
	spiWrite(spi.adc_fd, buff, 2);
	usleep(100); 

/*SEQ = D0					0    A
 * mux=0 					000
 * mode=1					   0 1
 * gpodren=0				      0
 * mdren=1					       1
 * rdyben=0					        0
*/
	buff[0] = 0xd0;		//SEQ  command
	buff[1] = 0x0a;		//select seq mod 2, enable delay
	spiWrite(spi.adc_fd, buff, 2);
		
	buff[0] = 0xca;		// DELAY
	buff[1] = 0x00;		//    origianl = f0
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
	int32_t n,num_channels;

/*Convert					B    E
 * convert code				1011
 * sample rate=64000sps		    1110  */
	buff[0] = 0xba;					// Convert! (6400sps)
	spiWrite(spi.adc_fd, buff, 1);
	usleep(2000); 			 			
	num_channels = 6;											//TEMP!!!!! 5->1
	for ( n=0; n<num_channels; n++) {
//		n=2; // ammeter
		buff[0] = spi.adc_reg[n];	//select a channel to read
		spiXfer(spi.adc_fd, buff, buff_rx, 4);
		eq.CJ[0] = buff_rx[3];
		eq.CJ[1] = buff_rx[2];
		eq.CJ[2] = buff_rx[1];
		eq.CJ[3] = buff_rx[0];
		spi.adc[n] = (float) eq.J / adc_bits * Vref;
//		fprintf( spi.fp, " adc<- %s = %.4f\n",  spi.adc_names[n], spi.adc[n]);
		usleep(2000); 		//1000	


	}

//  make any instrument corrections here
//	1-in ammeter_test, don't divide ammeter by rsense on the I.dat write
//  2- run ammeter_test and find the offset and slope factor from I.dat
//	3- divide by 2 - the diffamp circuit doubles the voltage across Rsense
//	4- restore the ammeter_test divide by Rsense.	
		spi.adc[ammeter] = (spi.adc[ammeter] ) * .96 /2.0 / r_sense; 
		spi.adc[adc_spare] = (spi.adc[adc_spare] -0.00358) * 0.99 ; 
		vgage.gage_hydrogen = pow( 10, spi.adc[gage_H2]/0.24485 - 6  );    //ratio of voltage divider
		vgage.gage_vacuum = pow( 10, spi.adc[gage_Vac]/0.24605 - 6  );    //ratio of voltage divider
//		spi.DB[forward] = spi.adc[forward] /0.021 - 87; //convert to dBv
//		spi.DB[reflected] = spi.adc[reflected] /0.021 - 87;
//		spi.DB[sniffer] = spi.adc[sniffer] /0.021 - 87;	

	fprintf( spi.fp, "adc[0-5]= %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",
	spi.adc[0],spi.adc[1],spi.adc[2],spi.adc[3],spi.adc[4],spi.adc[5]);
	
	return;
}

///////////////////////////////////////////

///////////////////////////////////////////
void dac_adc_Test( )  {
	int32_t count;
	float_t r_proton, v_calc, scale;
	float_t i_calc, i_measured, v_measured, ir_proton;

	count = 0;
	r_proton = 981;
	
	while (spi.task_status != kill && count < 10) {
//		count = count + 1;
	
		dac_write(anode, 50);
		dac_write(grid, 50);
		dac_write(cathode, -1.0);
		dac_write(e1, -50);
		dac_write(dac_spare, 1);
		gpioWrite(Sync_pin, 1);

		adc_read( );
/* 
 * comment out the scaling of ammeter in adc read to find out what scale 
 * value to use then update the scale factor in adc_read
*/
		i_calc = (spi.dac[cathode] + 0.06)/ (r_proton+r_sense) ;  // e = r * i(tot)
		v_calc = r_sense * i_calc;
		ir_proton = r_proton * i_calc;
		scale = -  v_calc / spi.adc[ammeter];
		v_measured = spi.adc[ammeter] * scale;  // diffamp = 20x v_drop across r_sense;  needed adjustment
		i_measured = v_measured / r_sense;		
		fprintf( spi.fp, "cathode=%.4f v_meas=%.4f  v_calc=%.4f  i_meas=%10.3e   i_calc=%10.3e v_p=%.4f scale=%.6f\n", 
		           spi.dac[cathode], v_measured,  v_calc,     i_measured,   i_calc, ir_proton, scale  );
		


		usleep(5000000);
		dac_write(anode, 0);
		dac_write(grid, 0);
		dac_write(cathode, 0);
		dac_write(e1, 0);
		dac_write(dac_spare, 0);

		gpioWrite(Sync_pin, 0);

		adc_read( );
		fprintf( spi.fp, "%d, DAC=%.4f  ADC=%.4f\n", 
//		  count, spi.dac[dac_spare], spi.adc[adc_spare]);	
		  count, spi.dac[cathode], spi.adc[ammeter]);	
		usleep(5000000);
		
	}
							

//	fprintf( fp, " S %d, AM=%.4f, FP=%.4f, RP=%.4f, SP=%.4f, TST=%.4f\n",  
//	count, spi.adc[ammeter],spi.adc[forward],spi.adc[reflected],
//	spi.adc[sniffer],spi.adc[adc_test]);

	return;
}

///////////////////////////////////////////
void dac_Sweep(float_t start, float_t end, float_t dv, int32_t dt, int32_t channel, int32_t num_pulses, float_t r_proton){
	int32_t count, n;
	float_t i_calc, v_calc, i_measured, cat_corrected;

/* start, dv, end in dbm,   dt in usec
 * calling routine passes a voltage (0-2.44v) on one of the eight dac channels  
 * such as spi.dac[cathode], spi.dac[einzel1], spi.dac[einzel2], or spi.dac[dac_test_gain].
 * dac_write writes out all four channels. In the future, the DAC and ADC will 
 * just cycle continuously on a 1msec heartbeat so the dac_write and adc_read 
 * on their own thread and will not need to be called.
*/

	count = 1;
	n=0;	
	spi.dac[channel] = start;

	fprintf( spi.fp, "sweep from %.1f to %.1f  across r_proton=%.0f   %s\n",  
				start, end, r_proton, date_time  );


	while ( count <= num_pulses && spi.task_status != kill) {
		while ( spi.dac[channel] < end ) {			//sweep up
			gpioWrite (Sync_pin,  n);		// use gpio4 as a sync for the o´scope
			if(n == 0){n=1;}else{n=0;}		
			dac_write(channel, spi.dac[channel] );
			usleep(50);	
			adc_read( );
			usleep(dt);

			if(channel == cathode) {	
				cat_corrected = spi.dac[cathode] + 0.062;		
				i_calc = cat_corrected/(r_proton + r_sense);	
				v_calc = r_sense * i_calc;  // e = r * i(tot)
				i_measured = spi.adc[ammeter];					
				fprintf( spi.fp, "cathode=%.4f v_calc=%.4f   ammeter=%.4f   i_calc=%.4e  I=%.4e\n",  
							cat_corrected, v_calc, spi.adc[ammeter], i_calc,  i_measured );
			}
			if(channel == dac_spare) {
				fprintf( spi.fp, " %d  %s=%.3f  measured=%.3f\n", 
				count, spi.dac_names[channel], spi.dac[channel], (spi.adc[adc_spare]-0.0034) * 0.99);
			}	
			spi.dac[channel] = spi.dac[channel] + dv;
		}
		
		while ( spi.dac[channel] > start ) {			//sweep down
			gpioWrite (Sync_pin,  n);		// use gpio4 as a sync for the o´scope
			if(n == 0){n=1;}else{n=0;}
			dac_write(channel, spi.dac[channel] );
			usleep(50);	
			adc_read( );
			usleep(dt);

			if(channel == cathode) {
				cat_corrected = spi.dac[cathode] + 0.062;		
				i_calc = cat_corrected/(r_proton + r_sense);	
				v_calc = r_sense * i_calc;  // e = r * i(tot)
				i_measured = spi.adc[ammeter];					
				fprintf( spi.fp, "cathode=%.4f v_calc=%.4f   ammeter=%.4f   i_calc=%.4e  I=%.4e\n",  
							cat_corrected, v_calc, spi.adc[ammeter], i_calc,  i_measured );
			}
			if(channel == dac_spare) {
				fprintf( spi.fp, " %d  %s=%.3f  measured=%.3f\n", 
				count, spi.dac_names[channel], spi.dac[channel], (spi.adc[adc_spare]-0.0034) * 0.99);
			}	
			spi.dac[channel] = spi.dac[channel] - dv;
		}
		count = count + 1;
	}

	dac_write(channel, 0);
	
	return ;
}


///////////////////////////////////////////
void* logamp_Test( )
{
	int32_t count;
	
	count=0;
	while (spi.task_status != kill && count <10) {
//		count++;
		adc_read( );
		printf( " Pfor= %.4f, Prev= %.4f\r", 
//		fprintf( spi.fp," Pfor= %.4f, Prev= %.4f\r", 
			spi.adc[forward],spi.adc[reflected]);
	}
	return 0;
 
}

///////////////////////////////////////////
void* gpio_Test( )
{
	while (spi.task_status != kill) {
		gpioWrite(H2_in, 1);
		usleep(500000);
		gpioWrite(H2_in, 0);
		gpioWrite(H2_out, 1);	
		usleep(500000);
		gpioWrite(H2_out, 0);
		gpioWrite(Vac, 1);
		usleep(500000);
		gpioWrite(Vac, 0);
		gpioWrite(Vac_pump, 1);
		usleep(500000);
		gpioWrite(Vac_pump, 0);
		usleep(500000);
	 }
	return 0; 

}

///////////////////////////////////////////
int   serial_test( )
{
	int32_t  len, count ;
	char d[20] = {'b','i','l','b','o','\0'};

/*
 * 	if no gage, connect Tx to Rx and uncomment loopback test above
 *  
 */
// serial_gage_init is called at top of main
	
	count = 0;
	while (spi.task_status != kill && count <5) { 
		count++;
//		gpioWrite (H2_in, 1) ;	
		len = strlen(vgage.msg);
		serWrite (vgage.fd, vgage.msg, len) ;
		fprintf( spi.fp, " serial write =%s\n", vgage.msg);
		usleep(300000) ;
		
//		gpioWrite (H2_in, 0) ;	 
		if( serDataAvailable( vgage.fd ) ){ 
			serRead ( vgage.fd, d, 20) ;
			fprintf( spi.fp, " serial read=%s\n", d );
			usleep(300000) ;
		}
	}
	return 0;
	
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
int help_main_menu() {
	printf( 
	"a = puff of H2\n"
	"v = change cathode voltage\n"
	"s = suck of vacuum\n"
	"d = vacuum on\n"
	"f = vacuum off\n"
	"w = start spi thread\n"
	"v = voltage pulse on DAC-pin 10\n"
	"p = regulate vacuum pressure\n"
	"o = GPIO4 oscillator test\n"
	"h = THIS help menu\n" 
	"q = just kill spi or vacuum processes\n"
	"z = exit\n\n\n");	

	return 0 ;
}


///////////////////////////////////////////
int help_s_menu() {
	printf(
	"\n"
	"wg = sweep_grid - plots grid vs cathode ammeter @ Vcathode\n"
	"wi = sweep_pressure - plots pressure vs ammeter @ Vcathode\n"
	"wb = sweep_cathode  - plots cathode vs ammeter \n"
	"wm = grid_pulse\n"
	
	"wv = gpio_Test - valve test\n"
	"ws = serial_test - serial i/o test (short GAGE-pins 1&2) (spi.dat)\n"
	"wc = ce_Test - chip enables with one byte data\n"
	"wa = dac_adc_Test - cycles dac_test line 0v->2.44v\n"
	"wk = dac_spare_test - plots dac_spare vs adc_spare \n"
	"wd = dac_Sweep( V0, V1, dV, dT, channel,numpulses)\n"
	"wo = opamp_test\n"

	"wn = proton infusion - plots pressure and ammeter current as cell is saturated with H2\n"
	"wl = logamp_Test - reads logamp voltage (spi.dat)\n"
	"wt = att_Test - sets pll=100mhz & ramps PE43711 from 0to -31.75 db (spi.dat)\n"
	"               insert -30db attenuator between RF-OUT & FORWARD ports\n"	"wp = pll_test\n"
	"wf = find_resonance( 100, 200, 5, 1000)\n"
	"wq = exit spi task\n");

	return 0 ;
}




/*	Old read adc code
			adcOut = 0;
			adcOut = buff[1];			//shift MSByte over one byte
			adcOut = adcOut << 8;
			adcOut = adcOut & buff[2];	//shift top 2 bytes over
			adcOut = adcOut << 8;
			adcOut = adcOut & buff[3];	//add in the LSByte		 	
			spi.adc[n] = (float) adcOut / 16777215 * Vref;
			fprintf( spi.fp, "      %d,  %0x,  %f\n",  n, adcOut, spi.adc[n]);
*/
