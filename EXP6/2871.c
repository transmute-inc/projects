#include <stdint.h>  //int8_t,int16_t,int32_t,uint8_t,uint16_t,uint32_t
#include "MAX2871.h"


typedef struct spi_control {

	int thread_status;
	FILE *fp;
	char cmd;
	
	unsigned cs_pll;
	unsigned int PLL[6];	// reg0-reg5 !write in reverse order!
	int frequency;			// in KHz

	unsigned cs_att;
	unsigned int ATT;
	float attenuation;				// in dB

	unsigned cs_adc;
	unsigned char adc_reg[6];
	unsigned char adc_status[4];
	float ADC[6];			// adc voltage values

	unsigned cs_dac;
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
	
	
		eq.J = spi.PLL[0];
		buff[0] = eq.CJ[3];
		buff[1] = eq.CJ[2];
		buff[2] = eq.CJ[1];
		buff[3] = eq.CJ[0];
		spiWrite(spi.cs_pll, buff, 4);
		fprintf( spi.fp, " pll(s) = %x  buff= %x, %x, %x, %x\n",
						eq.J, buff[0], buff[1], buff[2], buff[3] );	
	
	

void MAX2871_RFA_Disable(){
  /* Activates RF A output. */
  PLL[4] &= ~RFA_EN;
  MAX2871_SPI_tx(MAX2871_Registers[4]);
  digitalWrite(MAX2871_RF_EN, LOW);
}

void MAX2871_RFA_Power(char power){
  /* Sets the power to the RF A output */
  MAX2871_RFA_Disable();
  MAX2871_Registers[4] &= ~(3 << 3); // "clean" power bits (actually set to min)
  MAX2871_Registers[4] |= (power << 3); // Set desired value
  MAX2871_RFA_Enable();
}


int main(void)
{
	


	N = 320;				//integer mult		reg0<30:15>  N<15:0>  
	F = 0;					//fraction 			reg0<14:3>   F<15:0>  
	Mod = 125;				//modulus			reg1<14:3>	 M<11:0>
	DIVA = 6;				//A-divider			reg DIVA<2:0>
	Fvco = (N + (F/Mod)) * Fpfd;			//Voltage Controlled Osc. Freq

	RFoutA =  Fvco / (1<<DIVA);
	
	RFoutA = N * Fref / R / (1<<DIVA);		// for integer mode, eqn reduces to this
	printf(  " Fpfs = %x,  Fvco= %x, RFoutA =  %x\n",
				Fpfd, Fvco, RFoutA);
	printf(  " Fpfd = %d,  Fvco= %d, RFoutA =  %d\n",
				Fpfd, Fvco, RFoutA);










}
