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
//include <wiringPiSPI.h>
//include <wiringSerial.h>



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


unsigned int Fref,DBR,Rdiv2,R,Fpfd,N,F,Mod,DIVA,Fvco,RFoutA;
unsigned int BS, Cdiv;


int main(void)
{
	
	BS=232;					//BandSelect reg4<19:12> & reg4<25:24>
	Cdiv = 996;								//
	
	Fref=10000000;			//xtal reference frenquency
	DBR = 1;				//Fref multiplier	reg2<25>
	Rdiv2 = 1;				//Fref divider		reg2<24>
	R = 1;					//R divider bits	reg<23:14> 
	Fpfd = Fref*(2^DBR) / (2^Rdiv2) / R;	//Comparison frequency


	N = 320;				//integer mult		reg0<30:15>  N<15:0>  
	F = 0;					//fraction 			reg0<14:3>   F<15:0>  
	Mod = 125;				//modulus			reg1<14:3>	 M<11:0>
	DIVA = 6;				//A-divider			reg DIVA<2:0>
	Fvco = (N + (F/Mod)) * Fpfd;			//Voltage Controlled Osc. Freq

	RFoutA =  Fvco / (2^DIVA);
	printf(  " Fpfs = %x,  Fvco= %x, RFoutA =  %x\n",
				Fpfd, Fvco, RFoutA);
	printf(  " Fpfd = %d,  Fvco= %d, RFoutA =  %d\n",
				Fpfd, Fvco, RFoutA);
}
