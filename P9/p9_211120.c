#include <stdio.h>		//211120
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

#include <pigpio.h>		// http://abyz.me.uk/rpi/pigpio/cif.html

#include "max2871.h"	

/*
* gcc -Wall -o p7 p7.c -lpigpio -lthread -lm	// be sure to include math libs
* sudo ./p6
* gcc -Wall -ggdb -o p7 p7.c -lpigpio -lthread
* sudo gdb ./p7
*/


// GPIOs
#define Sync_pin	4		// usefull to sync oscope
#define H2_in		27		// RLY_1
#define H2_out		22		// RLY_2 
#define Vac			23		// RLY_3
#define Vac_pump	24		// RLY_4
//#define RLY_5		17		// RLY_5  rev6
//#define RLY_6		18		// RLY_6  rev6
#define HV_pulse	25		// RLY_7  rev6 also wired to P2 Valves header
#define range_cs	12		// rev6 gpio extender 
#define ltc2664_cs	 7		// rev6 DAC
#define ads8698_cs	 8		// rev6 ADC

#define not_running	0		// threadstatus
#define running		1		// threadstatus
#define kill		2		// threadstatus


//REV6 ADC channels
#define  am0		0		// AIN0P	anode
#define  am_anode	0		// AIN0P	anode
#define  am1		1		// AIN1P    grid
#define  am_grid	1		// AIN1P    grid
#define  am2		2		// AIN2P	e1
#define  am_e1		2		// AIN2P	e1
#define  am3		3		// AIN3P	cat
#define  am_cat		3		// AIN3P	cat
#define  gage_H2	4		// AIN4P	(H2)
#define  gage_Vac	5		// AIN5P	(vac)
#define  dac_n		5		// AIN5P	dac output for calibration
#define  Rext1		5		// AIN5P	channel output / (510k+25.5k) / 25.5 = 21

#define ch0			0		// defines for REV6
#define ch1			1
#define ch2			2
#define ch3			3
#define range_5ma	0		// 00
#define range_50ua	1		// 01
#define range_500pa	3		// 11

int32_t  anode		=0;		// DAC0
int32_t  grid		=1;		// DAC1
int32_t  e1			=2;		// DAC2
int32_t  cathode	=3;		// DAC3

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

	uint32_t adc_fd, adc_bits;
	uint32_t adc_channel_rev6[8];
	float_t  adc_vref;
	uint8_t adc_reg[8];
	uint8_t adc_status[8]; 
	char adc_names[8][8];
	float_t adc[8], adc_raw[8];			// adc voltage values
	float_t adc_gain[8];
	float_t adc_offset[8];
	float_t DB[3];			// log amp voltages converted to DB

	uint32_t dac_fd, dac_bits,  dac_channel_rev6[8];
	uint32_t dac_cmd[8], dac_ltc2446_cmd[4];
	float_t dac_vref;
	float_t dac[8];			// dac voltage desired (positive for anode, negative for cathode)
	float_t dac_bits_per_volt[8];		// dac gain coefficient - see dac_init for details
	float_t dac_volts_offset[8];
	char dac_names[8][8];
	
//REV6
	uint32_t range_fd;		// gpio16
	uint8_t range_out;
	uint8_t range_mask_clear[4];
	float_t range_r_value[4];
	float_t range_resistor[4][4];
	
	
} spi_c;

	spi_c spi;				// global structure!!
	

int   getch(int32_t ms);		// routine calling definitions
void* _spi_thread( void *s );

void adc_init( uint32_t mode );
void adc_status( );
float_t adc_read(int32_t channel);

void dac_init( );
void dac_write(int32_t channel, float_t value );
void dac_step_test(int32_t channel);
int  adc_dacN_test( int32_t channel );

void range_init( );
void range_set(int32_t channel, uint8_t max_current);
int  range_read( );
void range_test( int32_t channel  );
void my_gpio_init( );
void my_gpio_write(int32_t gpio, int32_t val);

void dac_Sweep(float_t start, float_t end, float_t dv, int32_t dt, int32_t channel, int32_t num_pulses, float_t r_proton);
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
int   sweep_channel( int32_t channel );
int   help_main_menu( );
int   help_s_menu( );

uint32_t ser_tty, serBaud=9600, serFlags=0;
uint32_t spiBaud=1000000;	// 8meghz for rev2, 10meghz for rev6
// MAX11254 ADC   8MHz rev2 6x24bit
// AD5668	DAC  50MHz rev2 8x16bit
// ADS8698	ADC  18MHz rev6 8x18bit
// LTC2664  DAC  50MHz rev6 4x16bit
// MCP23S08 GPIO 10MHz rev6 8 GPIO expander

int x, rc1, rc2, count=0, n, ch;
uint8_t c;
char main_cmd;
union equivs { uint32_t J; uint8_t CJ[4]; } eq;

	int32_t  board_rev, pelletnumber, channelnumber,rangenumber ;
	float_t  r_sense, Vchannel, temp_exp, temp1, temp2;





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
//	printf(" board rev2 or 6? ");	scanf(" %d", &board_rev);
	board_rev = 6;
	printf(" p9_211120 - pellet number? ");		scanf(" %d", &pelletnumber);
	printf(" channel? ");			scanf(" %d", &channelnumber);
	printf(" channel voltage - ");	scanf(" %f", &Vchannel);
	printf("pellet = %d  channel%d = %f \n",pelletnumber, channelnumber, Vchannel);



	if (gpioInitialise() < 0)
	{
		printf( "gpio did not initiallize");
		return(1);
	}
	gpioSetMode(H2_in, PI_OUTPUT); 
 	gpioSetMode(H2_out, PI_OUTPUT); 
 	gpioSetMode(Vac, PI_OUTPUT); 
 	gpioSetMode(Vac_pump, PI_OUTPUT);
  	gpioSetMode(Sync_pin, PI_OUTPUT);
  	


	if(board_rev == 2) {
		serial_gage_init( );	// rev6 gauge does not use serial comm.
		
// MAX11254 ADC   8MHz rev2 6x24bit
// AD5668	DAC  50MHz rev2 8x16bit
		spi.adc_fd =  spiOpen(0,  8000000, 0);		//CE0   08   24	 T11	
		spi.dac_fd =  spiOpen(1, 50000000, 0);		//CE1   07   26  T12
	}
	if(board_rev == 6) {
// ADS8698	ADC  18MHz rev6 8x18bit
// LTC2664  DAC  50MHz rev6 4x16bit
// MCP23S08 GPIO 10MHz rev6 8 GPIO expander
/*		spi.adc_fd   = spiOpen(0, 18000000, 256);	//ce0   18   12
		spi.dac_fd   = spiOpen(1, 50000000, 256);	//ce1   17   11
		spi.range_fd = spiOpen(2, 10000000, 256);	//ce2   16   33	
*/		spi.adc_fd   = spiOpen(0,  100000, 256);	//ce0   18   12	//was 2M
		spi.dac_fd   = spiOpen(1,  100000, 256);	//ce1   17   11
		spi.range_fd = spiOpen(2,  100000, 256);	//ce2   16   33	
	}

	FILE *fp;
	fp = fopen( "spi.dat", "w+" );
	spi.fp = fp;
	


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
		if (main_cmd == 'v')		// change  voltage
		{
			printf(" channel? ");			scanf(" %d", &channelnumber);
			printf(" channel voltage - ");	scanf(" %f", &Vchannel);
			printf(" Range 0(470), 1(47k), 3(4.7M) ? ");		scanf(" %d", &rangenumber);
			range_set( channelnumber, rangenumber);
			printf("pellet = %d  channel%d = %.1f R = %.2e \n",
				pelletnumber, channelnumber, Vchannel, spi.range_resistor[channelnumber] [rangenumber] );
		}			
		if (main_cmd == 'a') {		// puff of H2
			mass_flow(1,H2_in);	//regulator setting on H2 tank
			printf( "am%d=%.3e  H2=%.2e", channelnumber, adc_read(channelnumber)*1000, adc_read(gage_H2) );
			fflush(stdout);
		}
		if (main_cmd == 's') {		// suck of vac
			mass_flow(1,Vac);
			printf( "am%d=%.3e  H2=%.2e", channelnumber, adc_read(channelnumber)*1000, adc_read(gage_H2) );
			fflush(stdout);
		}
	if (main_cmd == 'd')		// vac on
		{
			gpioWrite (Vac_pump, 1);
			gpioWrite (Vac,      1);
			gpioWrite (HV_pulse, 1);
			printf( "am%d=%.3e  H2=%.2e", channelnumber, adc_read(channelnumber)*1000, adc_read(gage_H2) );
			fflush(stdout);
		}	
		if (main_cmd == 'f') {		// vac off
			gpioWrite (Vac_pump, 0);
			gpioWrite (Vac,      0);
			gpioWrite (HV_pulse, 0);
			printf( "am%d=%.3e  H2=%.2e", channelnumber, adc_read(channelnumber)*1000, adc_read(gage_H2) );
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
	
//	dac_init( );  // already done in main
//	adc_init( 2 );	// mode 1 code does not work yet, use mode 2

		
	if     (spi.cmd=='a'){spi.cmd=0;adc_dacN_test(channelnumber); }	//plots dacN to adc(4) to 
	else if(spi.cmd=='b'){spi.cmd=0;sweep_channel(channelnumber); }	//plots V vs V/Rsense
	else if(spi.cmd=='c'){spi.cmd=0;dac_step_test(channelnumber); }	//
	else if(spi.cmd=='v'){spi.cmd=0;gpio_Test( ); }		//valve test
	else if(spi.cmd=='r'){spi.cmd=0;range_test( channelnumber ); }	//
	else if(spi.cmd=='g'){spi.cmd=0;sweep_grid( ); }		//sweep grid
	else if(spi.cmd=='m'){spi.cmd=0;grid_pulse(channelnumber); }		//pulse grid
	else if(spi.cmd=='i'){spi.cmd=0;sweep_pressure( ); }	
	else if(spi.cmd=='n'){spi.cmd=0;proton_infusion(cathode); }	
	else if(spi.cmd=='o'){spi.cmd=0;opamp_test( ); }	
	else if(spi.cmd=='s'){spi.cmd=0;serial_test( ); }	//serial i/o test
//	else if(spi.cmd=='d'){spi.cmd=0;dac_Sweep( 0,  1.1, 0.1, 10000, cathode,2,      981);} 
	else if(spi.cmd=='d'){spi.cmd=0;dac_Sweep( 0,  10, 0.2, 1000000, cathode,20,   1000000);}	// for 1meg
//	else if(spi.cmd=='d'){spi.cmd=0;dac_Sweep( 0,  10,  2, 10000, cathode,4,  10000000);} // for 10meg
//	else if(spi.cmd=='d'){spi.cmd=0;dac_Sweep( 0,  10,  2, 10000, cathode,4,1000000000);} // for 1g
//	else if(spi.cmd=='d'){spi.cmd=0;dac_Sweep( 0, 2.0,0.5, 10000, dac_spare,4, 10000000);} 


	usleep(100000);
	clock_t toc = clock();
//	fprintf(spi.fp, "spi: %d,  %s,   %f seconds\n", count, &spi.cmd, 
//	(double)(toc - spi.last) / CLOCKS_PER_SEC);
	spi.last = toc;
	count++;
	
	fflush(spi.fp);			//spi.dat was opened in main, flush last record before leaving thread

	spi.thread_status = not_running ;
	spi.task_status = not_running;

	pthread_exit(NULL);
}

///////////////////////////////////////////
void *_vac_thread( void *vv )
{
	vac* v = (vac*) vv; 				// cast the void* to struct type
  
//	int32_t h2_in_delay=10000, h2_out_delay=5000, pump_delay=100000, vac_delay=40000;
	int32_t h2_out_delay=5000, vac_delay=40000;
	int32_t dwell_time=800000, count=0;
	FILE *fp;
	float_t hi=v->set_point+v->deadband/2,low=v->set_point-v->deadband/2 ;

	fp = fopen( "gage.dat", "w+" );


	while ( v->thread_status != kill )  {   //   THIS ROUTINE NEEDS WORK, See old version
		usleep(dwell_time);	
		v->gage = adc_read(gage_H2);
		count = count + 1;
		fprintf( fp, "%d, %.4f\n", count, v->gage);

		if( v->gage > hi) {					//too 1, pulse vac
			vac_delay = 500000 * (v->gage-hi) / hi;
			if( vac_delay > 2000 ) mass_flow(2, Vac_pump);
		}else if( v->gage < low) {			//too 0, pulse H2
			h2_out_delay = 20000 * (low- v->gage) / low;
			if( h2_out_delay > 2000 ) mass_flow(2, H2_in);
		}
	}

									// turn everything off before exit
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
	char buff[4];
	int32_t n;

	spi.range_mask_clear[0] = 0xfc;		// set the 2-bit range masks
	spi.range_mask_clear[1] = 0xf3;		// for each channel
	spi.range_mask_clear[2] = 0xcf;
	spi.range_mask_clear[3] = 0x3f;
	
	
/* range#   range2     range1     Rdropping
    0         0          0          470 
    1         0          1          47K 
    2         1          0          470
    3         1          1          4.7M	
*/	
	
	
	spi.range_resistor[0][0] = 470;		// 0  0
	spi.range_resistor[0][1] = 47000;	// 0  1
	spi.range_resistor[0][2] = 470;		// 1  0
	spi.range_resistor[0][3] = 4700000;	// 1  1
	spi.range_resistor[1][0] = 470;
	spi.range_resistor[1][1] = 47000;
	spi.range_resistor[1][2] = 470;
	spi.range_resistor[1][3] = 4700000;	
	spi.range_resistor[2][0] = 470;
	spi.range_resistor[2][1] = 47000;
	spi.range_resistor[2][2] = 470;
	spi.range_resistor[2][3] = 4700000;
	spi.range_resistor[3][0] = 470;
	spi.range_resistor[3][1] = 47000;
	spi.range_resistor[3][2] = 470;
	spi.range_resistor[3][3] = 4700000;	

		
//		uint32_t range_r_value[4];
//		float_t range_resistor[4][4];
	
	buff[0] = 0x40;				// send command first
	buff[1] = 0x00;				// select IO_DIR_REG
	buff[2] = 0x00;				// set IO_DIR_REG to all output
	spiWrite(spi.range_fd, buff, 3); 
	
	spi.range_out = 0x00;
	buff[0] = 0x40;				// send command first
	buff[1] = 0x09;				// Select gpio output latch
	buff[2] = spi.range_out;	// output to all zero (UB2 switchs off)
	spiWrite(spi.range_fd, buff, 3); 

// set the default range to 47K ohms as in rev2
	n=0;
	while( n<=3 ) {
		range_set(n ,range_50ua);	//range_5ma, range_50ua,range_500pa
		n += 1;
	}
	
	return;
}

///////////////////////////////////////////
void range_set(int32_t channel, uint8_t max_current)
{
/* clears the channel two bit range value in the packed byte 
 * see range_init
*/
	char buff[4];
	spi.range_r_value[channel] = spi.range_resistor[channel][max_current];	//select the r_sense value
	spi.range_out = spi.range_out & spi.range_mask_clear[channel];
	spi.range_out = spi.range_out |(max_current << (channel * 2));
	fprintf( spi.fp, " range%d  relay=%02x  R=%.1e\n", channel, max_current, spi.range_r_value[channel]);
	
	buff[0] = 0x40;				// send command first
	buff[1] = 0x09;				// Select gpio output latch
	buff[2] = spi.range_out;	// output to all zero (UB2 switchs off)
	spiWrite(spi.range_fd, buff, 3); 
	return;
}
///////////////////////////////////////////
int range_read( )
{
	char buff[4], buff_rx[4];
		
	buff[0] = 0x41;			// read command
	buff[1] = 0x09;			// read GPIO reg
	spiXfer(spi.range_fd, buff, buff_rx, 3);
	fprintf(spi.fp, "status= %x, %0x, %0x\n", buff_rx[0], buff_rx[1], buff_rx[2]); 
	
	return(buff_rx[2]);
}
///////////////////////////////////////////
void range_test( int32_t channel )
{
/*  Put an ohmmeter between Rext1 & Rext2 (Out) 
 *  Run this test (wr)
 *  the resistance will cycle 470, 47k,4.7M every 5 sec
*/
	int32_t delay = 50;
	int32_t count = 0;	
	
	while (spi.task_status != kill && count < 1) {
//		count = count+1;

/*		gpioWrite(Sync_pin, 1);
		range_set(0,1);		usleep(delay);
		range_set(0,2);		usleep(delay);
		range_set(0,0);		usleep(delay);
		range_set(1,1);		usleep(delay);
		range_set(1,2);		usleep(delay);
		gpioWrite(Sync_pin, 0);
		range_set(1,0);		usleep(delay);
		range_set(2,1);		usleep(delay);
		range_set(2,2);		usleep(delay);
		range_set(2,0);		usleep(delay);
		range_set(3,1);		usleep(delay);
		range_set(3,2);		usleep(delay);
		range_set(3,0);		usleep(delay);

	
		range_set(channel, range_5ma);		//00
		usleep(delay);		
		range_set(channel, range_50ua);		//01
		usleep(delay);
		range_set(channel, range_500pa);	//11
		usleep(delay);
*/
	char buff[4];
	gpioWrite(Sync_pin, 0);
	buff[0] = 0x40;				// send command first
	buff[1] = 0x09;				// Select gpio output latch
	buff[2] = 0x00;				// output to all zero (UB2 switchs off)
	spiWrite(spi.range_fd, buff, 3);
	gpioWrite(Sync_pin, 1);
	buff[0] = 0x40;				// send command first
	buff[1] = 0x09;				// Select gpio output latch
	buff[2] = 0xff;				// output to all zero (UB2 switchs off)
	spiWrite(spi.range_fd, buff, 3); 




	}
	
	return;
}
///////////////////////////////////////////

///////////////////////////////////////////
void dac_init( )
{
	char buff[4], buff_rx[4];
	
// set up channel names
	strcpy(spi.dac_names[0],"anode");
	strcpy(spi.dac_names[1],"grid");
	strcpy(spi.dac_names[2],"e1");
	strcpy(spi.dac_names[3],"cathode");

/*
 * opamp gain is set by ratio of feedback resistor to input resistor
 *  510k/25.5k = 20
 * Measure vout/vin for each opamp channel and edit "20" below
 * 
 * note anode and grid are positive supplies, rest are negative
 * 
 * to calculate dac_volts_offset, set dac_volts_offset to zero and set dac to zero
 * then measure the voltage offset ( it could be positive or negative).
 * Now enter this offset into dac_volts_offset for each channel.
 * dac_volts_offset will be subtracted from the voltage called for.
 * 
 * 
*/

	spi.dac_bits = 65535;	// ltc2664 is -vref->+vref == 0->65535
	spi.dac_vref = 2.5;
	range_init();

	spi.dac[ch0] 				= 0;
	spi.dac_volts_offset[ch0] 	= 20 * (spi.dac_vref -.00122);
	spi.dac_bits_per_volt[ch0] 	= spi.dac_bits  / 100;
	
	spi.dac[ch1] 				= 0;
	spi.dac_volts_offset[ch1] 	= 20 * (spi.dac_vref +.00);
	spi.dac_bits_per_volt[ch1] 	= spi.dac_bits  / 100;
		
	spi.dac[ch2] 				= 0;
	spi.dac_volts_offset[ch2] 	= 20 * (spi.dac_vref +.00);
	spi.dac_bits_per_volt[ch2] 	= spi.dac_bits  / 100;
		
	spi.dac[ch3] 				= 0;
	spi.dac_volts_offset[ch3] 	= 20 * (spi.dac_vref + .00);
	spi.dac_bits_per_volt[ch3] 	= spi.dac_bits  / 100;
	
	spi.dac_cmd[0] = 0x300000;		// inialize write thru command and address
	spi.dac_cmd[1] = 0x310000;		// for DACs 0-7
	spi.dac_cmd[2] = 0x320000;		// This value will be added to the 16bit data
	spi.dac_cmd[3] = 0x330000;		// shifted left by 4


	buff[0] = 0x00;			// write code to all, update all, power up
	buff[1] = 0xc0;	
	buff[2] = 0x00;
	buff[3] = 0x00;
	spiWrite(spi.dac_fd, buff, 4);
	spiXfer(spi.adc_fd, buff, buff_rx, 4);
//	fprintf( spi.fp, "dac-init = buff   = %x, %x, %x, %x\n", buff[0], buff[1], buff[2], buff[3] );
//	fprintf( spi.fp, "dac-init = buff_rx= %x, %x, %x, %x\n", buff_rx[0], buff_rx[1], buff_rx[2], buff_rx[3] );

	dac_write(0 , 0 );
	dac_write(1 , 0 );
	dac_write(2 , 0 );
	dac_write(3 , 0 );

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
	int32_t  v_dig;

/*
 * LTC2446 has 4 16bit bipolar DACs with a Vref +-2.5V
 * 0 = -2.5v, 32768=0, 65535= +2.5v
 * 
*/
	spi.dac[channel] = value; 

	v_dig = ( value + spi.dac_volts_offset[channel] ) * spi.dac_bits_per_volt[channel];
	if( v_dig > spi.dac_bits) v_dig = spi.dac_bits;	
	if( v_dig < 0   ) v_dig = 0;
		
	eq.J = spi.dac_cmd[channel] + v_dig;
	buff[0] = eq.CJ[2];				// uses same command and address as ad5668
	buff[1] = eq.CJ[1];
	buff[2] = eq.CJ[0];
	spiWrite(spi.dac_fd, buff, 3); 	// might have to pad leading zero byte and send 4

/*		buff[0] = 0x30 + channel;  //this should make the same word
		buff[1] = v_dig >>8;
		buff[2] = v_dig & 0x00ff;
*/		
//		fprintf( spi.fp, "dac%x buff[0-2]  %02x, %02x, %02x\n", 
//		                      channel, buff[0],buff[1],buff[2] );

//	fprintf( spi.fp, " dac ->%s = %.4f\n",  spi.dac_names[channel], spi.dac[channel] ) ;
//	fprintf( spi.fp, "dac-chan%x = %x, %x, %x, %x, %x\n", channel, v_dig, buff[0], buff[1], buff[2], buff[3] );
	
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
		    adc_read(gage_H2);									// reads gage voltage
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
	
	dac_write(cathode, Vchannel );	// ammeter is hooked to cathode

	gpioWrite(Vac, 1);				//suck out any residual H2 
	gpioWrite(Vac_pump, 1);
	usleep(2000000) ;				// 2 sec	

	adc_read(am_cat);					//clear out previous read
	
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
			i_measured = adc_read(am_cat) * 1000000;
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

	Vx100 = Vchannel * -100;
    sprintf(plotfilename,"P%2dv%04d-%.13s.png", pelletnumber, Vx100,date_time);
	printf("making plot file %s\n",plotfilename);

	gnuplot_ctrl    *h1 ;
	h1 = gnuplot_init() ;
	gnuplot_cmd(h1, "set terminal png size 1200,600") ;

    sprintf(plot_cmd,"set output '%s'",plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
    sprintf(plot_cmd,"set title 'Proton Current at %.1fV %s",Vchannel, plotfilename);
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
	spi.dac[cathode] = Vchannel;	//turn on power supplies
	Vgrid = 48;

	dac_write(e1, 0.0 );
	dac_write(anode, 0.0 );
	adc_read(am_cat );				//clear out previous read

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
//		if( gage_count == 35 ) {dac_write(cathode, Vchannel);} 	// pulse cathode
//		if( gage_count == 43 ) {dac_write(cathode, 0.0);}
		if( gage_count == 35 ) {dac_write(anode, 50);}			// pulse anode
		if( gage_count == 43 ) {dac_write(anode, 0.0);}		
				
//		if( gage_count == 40 ) {gpioWrite(Vac_pump, 1);gpioWrite(Vac, 1);}	// suck out the H2 until the end

		
		i=0;
		while (i < 5) {
			i_measured = adc_read(am_cat) * 1000000;	//microamps
			x = (gage_count*10 + i*2);
			x = x /100;
			fprintf( I_fp, " %.2f %.1f %.3f %.1f %.1f\n", x, adc_read(gage_H2), 
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
	
	Vx100 = Vchannel * -100;
    sprintf(plotfilename,"P%2dv%04d-%.13s.png", pelletnumber, Vx100,date_time);
	printf("making plot file %s\n",plotfilename);

	gnuplot_cmd(h1, "set terminal png size 1200,600") ;
    sprintf(plot_cmd,"set output '%s'",plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
    sprintf(plot_cmd,"set title 'Proton Current at %.1fV %s",Vchannel, plotfilename);
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


	Vx100 = abs(Vchannel * 100);
	if( adc_channel == am_cat ) {
		strcpy ( units, "mA" );
		strcpy (y2label_cmd, "set y2label 'Proton Current - mA' ");
		sprintf(plotfilename,"HI%2dv%04d-%.13s.png",pelletnumber, Vx100,date_time);		
	}
	
	FILE *I_fp;			// open plot file
	I_fp = fopen( "I.dat", "w+" );
	
//	Vchannel= -40.0;
//	dac_write(cathode, Vchannel);	// ammeter is hooked to cathode
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

		i_measured = adc_read(adc_channel) * 1000000;
		Inow = i_measured *10;
		clock_t toc = clock();
		tictoc = (double)(toc - tic) / CLOCKS_PER_SEC;
		fprintf( I_fp, " %.4f %.1f %.6f\n", tictoc, adc_read(gage_H2), Inow);	
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
	float_t v_set;

// grid is actually hooked to e1

	FILE *I_fp;			// open plot file
	I_fp = fopen( "I.dat", "w+" );
	Vx100 = fabs(Vchannel *100);
	dac_write(anode, 0.0);
	dac_write(cathode, Vchannel );
	
	count = 0;
	while (count <= 10 ) {
		v_set = Vchannel * (float)(count)/10.0;	// ammeter is hooked to cathode
		dac_write(e1, v_set );
		usleep(200000) ;
		fprintf( I_fp, " %.6f %.8f\n", v_set, adc_read(am_cat)*1000000 );	// diff_amp voltage is divided by r_sense in adc_read
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
    sprintf(plot_cmd,"set title ' am_cat vs. grid  - %s", plotfilename);
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
int   sweep_channel( int32_t channel)
{
	int32_t count, Vx100, steps;
	char plot_cmd[80], plotfilename[32];
	float_t v_set, i_sense;
	
	FILE *I_fp;			// open plot file
	I_fp = fopen( "I.dat", "w+" );
	Vx100 = Vchannel *100;
	adc_read(channel);						//clear out previous read
	
	count = 0;
	steps = 50;
	while (count <= steps ) {
		v_set = Vchannel * (float)(count)/steps;	// each channel has an ammeter
		dac_write(channel, v_set);
		usleep(500) ;
		i_sense = adc_read(channel);				//read ammeter on channel
		fprintf( I_fp, " %.6f %.10f\n", v_set, i_sense*1000 );	// diff_amp voltage is divided by r_sense in adc_read
		count++ ;
	}

	dac_write(channel, 0.0 );	//turn off cathode
	fclose(I_fp);


    sprintf(plotfilename,"A%2dv%04d-%.13s.png", pelletnumber, Vx100, date_time);
	printf("making plot file %s\n",plotfilename);
	gnuplot_ctrl    *h1 ;
	h1 = gnuplot_init() ;
	gnuplot_cmd(h1, "set terminal png size 800,600") ;
    sprintf(plot_cmd,"set output '%s'",plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
    sprintf(plot_cmd,"set title ' ammeter vs. channel%d R=%.2e - %s", 
						channel, spi.range_r_value[channel], plotfilename);
	gnuplot_cmd(h1, plot_cmd) ;
	
	gnuplot_cmd(h1, "set xlabel 'V channel' ") ;
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
	dac_write(cathode, Vchannel);	//turn on power supplies

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

int   adc_dacN_test( int32_t channel )
{
	int32_t count, Vx100, steps ;
	char plot_cmd[80], plotfilename[32];
	float_t v_set;

	FILE *I_fp;			// open plot file
	I_fp = fopen( "I.dat", "w+" );
	Vx100 = abs(Vchannel * 100);
	
count = 0;
	steps = 50;
	while (count <= steps ) {
		v_set = Vchannel * (float)(count)/steps;	// each channel has an ammeter
		dac_write(channel, v_set);
		usleep(50) ;
		adc_read(gage_H2);
		fprintf( I_fp, " %.6e %.8e\n", v_set, spi.adc_raw[gage_H2] );	// test loop dac0->adc(gage_H2)
		count++ ;
	}

	dac_write(channel, 0.0);
	fclose(I_fp);

	sprintf(plotfilename,"A%2dv%04d-%.13s.png", pelletnumber, Vx100, date_time);
		printf("making plot file %s\n",plotfilename);
		gnuplot_ctrl    *h1 ;
		h1 = gnuplot_init() ;
	gnuplot_cmd(h1, "set terminal png size 800,600") ;
    sprintf(plot_cmd,"set output '%s'",plotfilename);
		gnuplot_cmd(h1, plot_cmd) ;
    sprintf(plot_cmd,"set title ' ammeter vs. channel%d  R=%d  %s", channel, rangenumber, plotfilename);
		gnuplot_cmd(h1, plot_cmd) ;
	
	gnuplot_cmd(h1, "set xlabel 'Vdac channel' ") ;
	gnuplot_cmd(h1, "set ylabel 'Vadc' tc 'web-blue' ") ;
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
void adc_init( uint32_t mode )
{
	char buff[4];
	int32_t n;

	spi.adc_bits = 262143;				//2^18 -1, 3FFFF
	spi.adc_vref = 4.096 * 0.625;		// internal Vref=4.096, = +-2.53952V  LSB=19.53125uV
	strcpy(spi.adc_names[0],"am0");		//anode
	strcpy(spi.adc_names[1],"am1");		//grid
	strcpy(spi.adc_names[2],"am2");		//e1
	strcpy(spi.adc_names[3],"am3");		//cathode
	strcpy(spi.adc_names[4],"gage_H2");
	strcpy(spi.adc_names[5],"gage_vac");

/* internal Vref is 4.096V	  
 * page 50 data sheet
*/		
	n=0;
	while (n<=5) {
		buff[0] = ( (0x05 + n)<<1 ) + 1 ;	//create range register address + write
		buff[1] = 0x02;						//(0010) to select +-0.625 multiplier
		spiWrite(spi.adc_fd, buff, 2);
		n=n+1;
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
float_t adc_read(int32_t channel)
{
	union equiv { uint32_t J; char CJ[4]; } eq;
	char buff[5], buff_rx[5];
	int32_t n, iter;
	float_t adc_sum;
	
	eq.J = 0;								//current read is based on last channel number
	buff[0] = (0x30 + channel)<<2;			//command for Man_Ch selection
	buff[1] = 0x00;							//pad 16bit command
	spiXfer(spi.adc_fd, buff, buff_rx, 5);	//fake read to reset channel
		
	n=0;	iter=50;		adc_sum=0;
	while( n <= iter ) {					// take the average of n reads
//		usleep(1000); 						// sleep does not seem to make any difference
		eq.J = 0;
		buff[0] = (0x30 + channel)<<2;
		buff[1] = 0x00;
		spiXfer(spi.adc_fd, buff, buff_rx, 5);
		eq.CJ[0] = buff_rx[4];
		eq.CJ[1] = buff_rx[3];
		eq.CJ[2] = buff_rx[2];
		eq.CJ[3] = buff_rx[1];
		adc_sum = adc_sum + (float) (eq.J>>6);
		n = n+1;			 			
	}
	spi.adc_raw[channel] = ( adc_sum / (n) / (spi.adc_bits) * spi.adc_vref*2 - spi.adc_vref)    ;
	
	if(channel <=3 ) spi.adc[channel] = spi.adc_raw[channel] * 0.7 /spi.range_r_value[channel];
	if(channel == gage_H2) spi.adc[gage_H2] = pow( 10, spi.adc_raw[gage_H2]*4.0888 - 6  );    //ratio of voltage divider
	if(channel == gage_Vac) spi.adc[gage_Vac] = pow( 10, spi.adc_raw[gage_Vac]*4.0888 - 6  );    //ratio of voltage divider

//	fprintf( spi.fp, " adc%d %04x, buff_rx  %02x, %02x, %02x, %02x, %0x\n", 
//					channel, eq.J, buff_rx[0],buff_rx[1],buff_rx[2],buff_rx[3],buff_rx[4] );
//	fprintf( spi.fp, " adc%d raw=%.3f adc=%.3e\n",  channel, spi.adc_raw[channel], spi.adc[channel]);
	
	return(spi.adc[channel]);
}

///////////////////////////////////////////

///////////////////////////////////////////
void dac_step_test(int32_t channel)  {
	int32_t count, read_chan, delay;
	float_t i_read;

//	read_chan = channel;	// read the current across Rsense
	read_chan = gage_H2;	// jumper from 20x plug on opamp out to adc
	delay = 50;				//5000000 to measure with meter

	count = 0;
	while (spi.task_status != kill && count < 5) {
		count = count + 1;

		gpioWrite(Sync_pin, 1);
		dac_write(channel, Vchannel);	usleep(delay);
		i_read = adc_read(read_chan);
		i_read = spi.adc_raw[read_chan];
		fprintf( spi.fp, "dac_test%d  v_set=%.1f\ti_read=%.4e \n", count, spi.dac[channel], i_read );
		
		dac_write(channel, 0);			usleep(delay);
		i_read = adc_read(read_chan);
		i_read = spi.adc_raw[read_chan];
		fprintf( spi.fp, "dac_test%d  v_set=%.1f\ti_read=%.4e \n", count, spi.dac[channel], i_read );

		gpioWrite(Sync_pin, 0);
		dac_write(channel, -Vchannel);	usleep(delay);
		i_read = adc_read(read_chan);
		i_read = spi.adc_raw[read_chan];
		fprintf( spi.fp, "dac_test%d  v_set=%.1f\ti_read=%.4e \n", count, spi.dac[channel], i_read );

		dac_write(channel, 0);			usleep(delay);
		i_read = adc_read(read_chan);
		i_read = spi.adc_raw[read_chan];
		fprintf( spi.fp, "dac_test%d  v_set=%.1f\ti_read=%.4e \n", count, spi.dac[channel], i_read );
		fprintf( spi.fp, "\n" );
	}

	return;
}

///////////////////////////////////////////
void dac_Sweep(float_t start, float_t end, float_t dv, int32_t dt, int32_t channel, int32_t num_pulses, float_t r_proton){
	int32_t count, n;
	float_t i_calc, v_calc, i_measured, cat_corrected, v_set;

/* start, dv, end in dbm,   dt in usec
 * calling routine passes a voltage (0-2.44v) on one of the eight dac channels  
 * such as spi.dac[cathode], spi.dac[einzel1], spi.dac[einzel2], or spi.dac[dac_test_gain].
 * dac_write writes out all four channels. In the future, the DAC and ADC will 
 * just cycle continuously on a 1msec heartbeat so the dac_write and adc_read 
 * on their own thread and will not need to be called.
*/

	count = 1;
	n=0;	
	v_set = start;

	fprintf( spi.fp, "sweep from %.1f to %.1f  across r_proton=%.0f   %s\n",  
				start, end, r_proton, date_time  );


	while ( count <= num_pulses && spi.task_status != kill) {
		while ( v_set < end ) {			//sweep up
			gpioWrite (Sync_pin,  n);		// use gpio4 as a sync for the o´scope
			if(n == 0){n=1;}else{n=0;}		
			dac_write(channel, v_set );
			usleep(50);	
			usleep(dt);

			cat_corrected = v_set + 0.062;		
			i_calc = cat_corrected/(r_proton + r_sense);	
			v_calc = r_sense * i_calc;  // e = r * i(tot)
			i_measured = adc_read(am_cat);					
			fprintf( spi.fp, "cathode=%.4f v_calc=%.4f   am_cat=%.4f   i_calc=%.4e \n",  
					cat_corrected, v_calc, i_measured, i_calc );

			v_set = v_set + dv;
		}
		
		while ( v_set > start ) {			//sweep down
			gpioWrite (Sync_pin,  n);		// use gpio4 as a sync for the o´scope
			if(n == 0){n=1;}else{n=0;}
			dac_write(channel, v_set );
			usleep(dt);

			cat_corrected = v_set + 0.062;		
			i_calc = cat_corrected/(r_proton + r_sense);	
			v_calc = r_sense * i_calc;  // e = r * i(tot)
			i_measured = adc_read(am_cat);					
			fprintf( spi.fp, "cathode=%.4f v_calc=%.4f   I=%.4e   i_calc=%.4e\n",  
						cat_corrected, v_calc, i_measured, i_calc  );
	
			v_set = v_set - dv;
		}
		count = count + 1;
	}

	dac_write(channel, 0);
	
	return ;
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
	"v = change channel voltage\n"
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
	"wg = sweep_grid - plots grid vs cathode ammeter @ Vchannel\n"
	"wi = sweep_pressure - plots pressure vs ammeter @ Vchannel\n"
	"wb = sweep_channel  - plots channel vs its ammeter \n"
	"wm = grid_pulse\n\n"
	
	"wa = adc_dacN_test - plots adc vs dacN voltage\n"
	"wc = dac_step_test - 0, V, 0, -V for 5sec \n"
	"wv = gpio_Test - valve test\n"
	"wr = range_test - cycle through Rsense\n"
	"ws = serial_test - serial i/o test (short GAGE-pins 1&2) (spi.dat)\n"
	"wd = dac_Sweep( V0, V1, dV, dT, channel,numpulses)\n"
	"wo = opamp_test\n"
	"wn = proton infusion - plots pressure and ammeter current as cell is saturated with H2\n"
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
