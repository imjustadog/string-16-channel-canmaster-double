#if defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24hxxxx.h>
#endif



#include "collect.h"

/*
#include "delay.h"
#include <delays.h>
*/



void s_transstart(void)
{
	TRIS_DATA=0;
	TRIS_SCK=0;

	DATA=1;
	SCK=0; 
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();

	SCK=1;
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	DATA=0;
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	SCK=0;
	Nop();Nop();Nop();
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	SCK=1;
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	DATA=1;
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	SCK=0;
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
}

void s_connectionreset(void)
{
	unsigned char i;
	TRIS_DATA=0;
	TRIS_SCK=0;
	DATA=1;
	SCK=0; 
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
		for(i=0;i<9;i++) //9 SCK cycles
		{ 
		SCK=1;
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
		SCK=0;
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
		}
	s_transstart(); //transmission start
}

char s_write_byte(unsigned char value)
{
	unsigned char i,error=0;
	TRIS_DATA=0;
	TRIS_SCK=0;

	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
		for (i=0x80;i>0;i/=2) //shift bit for masking
		{ 
		if (i & value) DATA=1; //masking value with i , write to SENSI-BUS
		else DATA=0;
		Nop();Nop();Nop();
		Nop();
		Nop();
		Nop();
		Nop();	
	    Nop();
		SCK=1; //clk for SENSI-BUS
    	Nop();
    	Nop();
		Nop();
	   	Nop();	
	    Nop();
		SCK=0;

		}
	DATA=1; //release DATA-line

	TRIS_DATA=1;
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	SCK=1; //clk #9 for ack
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	error=DATA; //check ack (DATA will be pulled down by SHT11)
	SCK=0;
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	return error;
}

char s_read_byte(unsigned char ack)
{
	unsigned char i,val=0;
	TRIS_DATA=0;
	TRIS_SCK=0;
	DATA=1; //release DATA-line

	TRIS_DATA=1;
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
		for (i=0x80;i>0;i/=2) //shift bit for masking
		{ 
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
		SCK=1; //clk for SENSI-BUS
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
		if (DATA) val=(val | i); //read bit
		SCK=0;

		}	

	TRIS_DATA=0;
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	DATA=!ack; //in case of "ack==1" pull down DATA-Line
	SCK=1; //clk #9 for ack
	Nop();Nop();Nop();
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	SCK=0;
    DATA=1; //release DATA-line
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	

	return val;
}

char s_softreset(void)
{
	unsigned char error=0;
	TRIS_DATA=0;
	TRIS_SCK=0;
	Nop();
	Nop();
	Nop();
	Nop();	
    Nop();
	s_connectionreset(); //reset communication
	error+=s_write_byte(RESET); //send RESET-command to sensor
	return error; //error=1 in case of no response form the sensor
}

char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum)
{
	unsigned char error=0;
	s_transstart(); //transmission start
	error=s_write_byte(STATUS_REG_R); //send command to sensor
	*p_value=s_read_byte(ACK); //read status register (8-bit)
	*p_checksum=s_read_byte(noACK); //read checksum (8-bit)
	return error; //error=1 in case of no response form the sensor
}

char s_write_statusreg(unsigned char *p_value)
{
	unsigned char error=0;
	s_transstart(); //transmission start
	error+=s_write_byte(STATUS_REG_W);//send command to sensor
	error+=s_write_byte(*p_value); //send value of status register
	return error; //error>=1 in case of no response form the sensor
}

char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode)
{
	unsigned char d_h,d_l;	
	unsigned error=0;
	unsigned int i;
	s_transstart(); //transmission start
		switch(mode)//send command to sensor
		{ 
		case TEMP : error+=s_write_byte(MEASURE_TEMP); break;
		case HUMI : error+=s_write_byte(MEASURE_HUMI); break;
		default : break;
		}
	for (i=0;i<65533;i++) 
{    

asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
 asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
asm("nop");	asm("nop");	asm("nop");	asm("nop");	asm("nop");
if(DATA==0)
{ 

break;
 Nop();
}


}
 //wait until sensor has finished the measurement
	if(DATA) error+=1; // or timeout (~2 sec.) is reached
	*(p_value+1) =s_read_byte(ACK); //read the first byte (MSB)
	d_h=s_read_byte(ACK);
	*(p_value)=s_read_byte(ACK); //read the second byte (LSB)
	d_l=s_read_byte(ACK);
	*p_checksum =s_read_byte(noACK); //read checksum
	return error;
}


void calc_sth11(float *p_humidity ,float *p_temperature)
{ 
	const float C1=-4.0; // for 12 Bit
	const float C2=0.0405; // for 12 Bit
	const float C3=-0.0000028; // for 12 Bit
	const float T1=0.01; // for 14 Bit @ 5V
	const float T2=0.00008; // for 14 Bit @ 5V
	float rh=*p_humidity; // rh: Humidity [Ticks] 12 Bit
	float t=*p_temperature; // t: Temperature [Ticks] 14 Bit
	float rh_lin; // rh_lin: Humidity linear
	float rh_true; // rh_true: Temperature compensated humidity
	float t_C; // t_C : Temperature [C]
	t_C=t*0.01-40;///c	alc. Temperature from ticks to [C]
	rh_lin=C3*rh*rh + C2*rh + C1;//calc. Humidity from ticks to [%RH]
	rh_true=(t_C-25)*(T1+T2*rh)+rh_lin; //calc. Temperature compensated humidity [%RH]
	if(rh_true>100)rh_true=100; //cut if the value is outside of
	if(rh_true<0.1)rh_true=0.1; //the physical possible range
	*p_temperature=t_C; //return temperature [C]
	*p_humidity=rh_true; //return humidity[%RH]
}

