#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>


unsigned int Frequency,Fref,DivA,N,RFoutA;
unsigned int R0, R0_init=0x80A00000, R4, R4_init=0x63EE81FC;
unsigned int N_clear, DivA_clear;



int main(void)
{
	N_clear = 0xffff<<15;
	DivA_clear = 0x7<<20;
	
	Frequency = 50;
	Fref = 10;			//xtal reference frenquency in MHz
	DivA = -1;
Loop:
	DivA = DivA + 1;
	N = Frequency / Fref * 1<<DivA ;
	if( N < 300 ) goto Loop;

	RFoutA = N * Fref / (1<<DivA);	
	printf(  " N = %d,  DivA= %d, RFoutA =  %d\n",
				N, DivA, RFoutA);
	
	
	R0 = R0_init & ~N_clear;
	R0 = R0 | N<<15;
	
	R4 = R4_init & ~DivA_clear;
	R4 = R4 | DivA<<20;	
//	printf(  " N_clear = %x, %x,    DivA_clear= %x, %x\n",
//				N_clear, ~N_clear, DivA_clear, ~DivA_clear); 
	printf(  " R0 = %x, %x,    R4= %x, %x\n",
				R0_init,R0,R4_init,R4);
}
