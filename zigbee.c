/*��ʱ��ѯģ�鵱ǰ�Ĺ���״̬*/
/*FRAM�п���00000~0001f �洢32�ֽڵ�ָ���ѯָ�� ֮ǰ�Ŀ���ָ�
	��ѯָ���ʽΪ 53 52 44 44 FF 44 FF 00 00 00 00 00 44 FF 45��15�ֽڣ���
	��ѯ����֮ǰ�Ŀ���ָ�������Ϊ 53 52 ���� FF 45��15�ֽڣ�+ 0x22��17����+C M D ��
*/

#if defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24hxxxx.h>
#endif
#include "timer.h"
#include "adc.h"
#include "sci.h"
#include "IC.h"
#include "DS1302.h"
#include "FRAM.h"

_FBS(0xCF);
_FSS(0xCF);
_FGS(0x07);
_FOSCSEL(0xA2);  //Select HS without PLL
_FOSC(0x46);  // HS oscilator, OSC2 is clock output
_FWDT(0x5F);
_FPOR(0xE7);
_FICD(0xC3);

#define CEA LATGbits.LATG0
#define CEB LATGbits.LATG1
#define CEC LATFbits.LATF1
#define CED LATFbits.LATF0
#define CEE LATDbits.LATD7
#define CEF LATDbits.LATD2
#define RESET LATGbits.LATG15 //RESET ������λzigbee��·��/�ն�
//#define WORK_LED LATDbits.LATD1
#define BAT_LED1 LATBbits.LATB14
#define BAT_LED2 LATBbits.LATB13
#define BAT_LED3 LATBbits.LATB12

unsigned int freq[6]; 
//unsigned int temp_freq[6];	
//float freq_float;			  
unsigned int temp[6];//�洢���ɼ�ͨ�����¶�ADֵ
unsigned int bat; //��ص���ADֵ
//unsigned int dummy;
short bat_high, bat_mid, bat_low; // �����������ڶԵ�ص�ѹ�����˲�

//int CE_Enable; // 1->����Ƶ  0->��ֹ��Ƶ 
int Read_Enable; // 1->ʰƵ��ɣ����Զ�ȡ  0->��ֹ��ȡ
int Tick; // ������¼��ʱ�жϴ���

short halt; // �������Ʋ���ʱ����
unsigned int halt_Tick; // ������¼ʱ�䣬��ʱ��Ϊ���η���֮���ʱ����
unsigned int halt_Timeout; // ���η���֮��ļ������λΪms����ͨ����λ������

unsigned char Send_Enable; // ����ָʾ�Ƿ���Է������ݣ��ϵ翪ʼʱ���������ݣ��յ�ZigBeeоƬ���͵�'n'�ַ����������
unsigned char Save_Enable; // ������ʾ�Ƿ���FRAM�д�����
unsigned char Tran_Enable; // ������ʾ�Ƿ�FRAM�е����ݶ���
unsigned char halt_Enable; // ����

unsigned char ID = 5; // ���*****************************************************************************

unsigned char stat1;
unsigned char net_state;//����ָʾ �����ɹ�����zigbee����10��35�ֽڵ�����
unsigned char lq[32];//��Ŵ�FRAM�ж�ȡ��ָ������

//unsigned int end_addr[2]; // �洢������ַ���洢��FRAM 0��1��2��ַ��
//unsigned char temp_addr[3];
unsigned int curr_addr[2];
unsigned int read_count;//������¼��FRAM�ж�ȡ�����ݰ�������ÿ��32�ֽ����ݣ������Ϊ35�ֽ�
//unsigned int read_count; //������¼��FRAM�ֶ�ȡ���ַ�������������������ͷ���������������
void inc_addr(void); // ��curr_addr��1

void Tran_Data_U1(void); // ����λ����������
void Tran_Data_U2(void);// ��С���ִ�������

void read_command_15_U1(void); //��ȡ����FRAM�е�15�ֽڵ�ָ��
void read_command_15_U2(void); //��ȡ����FRAM�е�15�ֽڵ�ָ��

void DELAY(unsigned int t)
{
	unsigned int i,j;
	
  	for(i=0;i<50;i++)
  	{
	  	for(j=0;j<t;j++)
   		{
	   		asm("nop");		
    	}
   } 	
 }//600~9ms

void __attribute__((interrupt,no_auto_psv)) _T6Interrupt(void)  // 1ms interrupt
{
	IFS2bits.T6IF = 0;
	if(halt_Enable == 0)
	{
		if(halt)
		{
			if(halt_Tick<halt_Timeout)//������or����ģʽ�£����Ʋɼ�ʱ����
				halt_Tick++;
			else
			{	StopTimer6();
				halt = 0;
			}
		}
		else
		{
			Tick++;
		//	if(Tick>59) // 60ms
			if(Tick>19)
			{	Tick = 0;
				Read_Enable = 1;	//��֤�����ʱ���ڣ�ʰƵ��׽����
			}
		}//���ڿ��� ʰƵʱ��
	}

}//

/*u1 �����жϣ����ڽ�����λ�������Ŀ���ָ��*/
void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
	unsigned char data[15];
	unsigned int UART_Timeout = 0;
	unsigned char i = 0;
	unsigned char j = 0;
	data[0] = U1RXREG;
	
	if('R'==data[0]) // �ɼ�ģ����յ�·��/�ն˷��������綪ʧ�źź󣬶�·��/�ն˽��и�λ
	{
		RESET = 0;
		DELAY(9000); //135ms
		DELAY(9000);
		DELAY(9000);
		DELAY(9000);
		RESET = 1;
		DELAY(9000);
		DELAY(9000);
		DELAY(9000);
		DELAY(9000);
		RESET = 0;
		DELAY(9000);
		DELAY(9000);
		DELAY(9000);
		DELAY(9000);
		RESET = 1;
	}
	if('n'==data[0]) 	  // �����ã�����n�ſ�ʼ��Ƶ
	{
		net_state = 1;
		for(j=0;j<32;j++) //��ȡFRAM�д洢��ָ���ʽΪ15�ֽ�ָ��+17��0x22����32�ֽ�
		{
			lq[j] = READ(0,0,j);
		}

//lq   0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  ����  30 31			
//     S   R 				                                    E  22  22      22 22
//���� S   R   FF  FF  FF  00  FF  FF  FF  FF  FF  FF  FF  FF   E 
//���� S   R   00  FF  FF  00  FF  FF  FF  FF  FF  FF  FF  FF   E
//���� S   R   55  FF  FF  00  FF  FF  FF  FF  FF  FF  00  FF   E

		//���罨���ɹ��󣬲ɼ�ģ���Զ�����ϴι���״̬����֮ǰΪ����״̬�������������״̬����
		if((lq[0]=='S')&&(lq[1]=='R')&&(lq[14]=='E')&&(lq[31]==0x22))
		{	
			if((lq[2]==0xFF)&&(lq[5]==0x00)&&(lq[12]==0xff)) // ����ģʽ
				{
					Send_Enable = 1;
					Save_Enable = 0;
					Tran_Enable = 0;
					halt_Enable = 0;
				}

		}

	}//if('n'==data[0])
	else//���յ����ڷ�����15�ֽڵ�ָ����ʽΪS ���� E
	{
		switch(data[0])
		{
			case 'S':  
				while(data[i]!='E')
				{
					i++;
					while((0==(U1STA&0x0001))&&(UART_Timeout<5000))
					{UART_Timeout++;}
					if(UART_Timeout>=5000)
					{
						data[i]='E'; // ������ճ�ʱ���˳��ж�
					}
					else
					{
						data[i] = U1RXREG;
					}
					UART_Timeout = 0;
				}
				break;
			default:
				break;
		}
		if(i==14)
		{
			
//data 0   1   2   3   4   5   6   7   8   9   10   11   12   13   14			
//     S   R 				                                       E  
//���� S   R   FF  FF  FF  00  FF  FF  FF  FF  FF   FF   FF   FF   E 
//���� S   R   00  FF  FF  00  FF  FF  FF  FF  FF   FF   FF   FF   E
//���� S   R   55  FF  FF  00  FF  FF  FF  FF  FF   FF   00   FF   E
//�ض� S   R   55  FF  FF  FF  FF  FF  FF  FF  FF   FF   FF   FF   E
//��ѯ S   R   44  44  FF  44  FF  FF  FF  FF  FF   FF   44   FF   E
		
			if((data[2]==0xff)&&(data[5]==0x00)&&(data[12]==0xff)) // ����ģʽ
			{
				Send_Enable = 1;
				Save_Enable = 0;
				Tran_Enable = 0;
				halt_Enable = 0;
			}
			if((data[2]==0x00)&&(data[5]==0x00)&&(data[12]==0xff)) // ����ģʽ
			{	net_state = 0;//ͨ��ָ����������״̬ʱ������zigbee����10��35�ֽڵ�����
				Send_Enable = 0;
				Save_Enable = 1;
				Tran_Enable = 0;
				halt_Enable = 0;
				
				curr_addr[0] = 0;
				curr_addr[1] = 32;		
			}
			if((data[2]==0x55)&&(data[5]==0x00)&&(data[12]==0x00)) //����
			{	net_state = 0;//ͨ��ָ����������״̬ʱ������zigbee����10��35�ֽڵ�����
				Send_Enable = 0;
				Save_Enable = 0;
				Tran_Enable = 0;
				halt_Enable = 1;
			}	
			if(data[4]!=0xff)	//(����or����ģʽ��)���ò������
								//����������������״̬��ͬʱ���ò��������ͬһ��ָ���ڣ�
								//����������������״̬��ͬʱ���ò��������ͬһ��ָ���ڣ�
			{
				halt_Timeout = data[4]*500; 
			}
			
			if(data[5]==0xff) 	//�ض�FRAMֵ
			{
				Send_Enable = 0;
				Save_Enable = 0;
				Tran_Enable = 1;				
				halt_Enable = 0;
			}			
			if(data[6]!=0xff) 	//У׼ʵʱʱ�� 
			{
				ds1302_write_time(6,data[6]);
				ds1302_write_time(4,data[7]);
				ds1302_write_time(3,data[8]);
				ds1302_write_time(2,data[9]);
				ds1302_write_time(1,data[10]);
				ds1302_write_time(0,data[11]);
			}			
					
			
			for(j=0;j<32;j++) //��ȡFRAM�д洢��ָ���ʽΪ15�ֽ�ָ��+17��0x22����32�ֽ�
			{
				lq[j] = READ(0,0,j);
			}
			//д15�ֽڵ�ָ��,��λ��0x22 ������00000~0001f  ,��32�ֽ�
			//��ʽΪ 15�ֽ�ָ��+17��0x22
			if(((data[2]==0xff)&&(data[5]==0x00)&&(data[12]==0xff))||((data[2]==0x00)&&(data[5]==0x00)&&(data[12]==0xff))||((data[2]==0x55)&&(data[5]==0x00)&&(data[12]==0x00)))	
				//ֻ�й���״̬����ָ��Ŵ洢 ������or����or���ߣ�							
			{
				for(j=0;j<15;j++)
				{
						WRITE(0,0,j,(unsigned char)data[j]); //data[0]~data[14] ����15�ֽ� ָ������
				}
				WRITE(0,0,15,ID);
				for(j=16;j<32;j++)
				{	
					WRITE(0,0,j,0x22);  // ����0x22
				}
				
				U1TXREG = 'S';
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = ID;
				while((U1STA&0x0100)==0x0000){asm("nop");}
				for(j=0;j<29;j++)
				{
					U1TXREG = 0xff;
					while((U1STA&0x0100)==0x0000){asm("nop");}
				}
				U1TXREG = data[2];
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = data[5];
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = data[12];
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = 'A';
				while((U1STA&0x0100)==0x0000){asm("nop");} //���յ�15�ֽ�ָ��󣬲���Ӧ��ָ�
													   //���ʽΪ S ID 0xff����0xff XX XX XX A ��35�ֽ�
			}
			else
			{
				
				U1TXREG = 'S';
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = ID;
				while((U1STA&0x0100)==0x0000){asm("nop");}
				for(j=0;j<29;j++)
				{
					U1TXREG = 0xff;
					while((U1STA&0x0100)==0x0000){asm("nop");}
				}
				U1TXREG = lq[2];
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = lq[5];
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = lq[12];
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = 'A';
				while((U1STA&0x0100)==0x0000){asm("nop");} //���յ�15�ֽ�ָ��󣬲���Ӧ��ָ�
													   //���ʽΪ S ID 0xff����0xff XX XX XX A ��35�ֽ�
				DELAY(6000);
			}
			if((data[3]==0x44)&&(data[2]==0x44)&&(data[5]==0x44)&&(data[12]==0x44)) //��ѯ����״̬
			{
				read_command_15_U1();
			}	
		
		}//if(i==14)	
	}
	IFS0bits.U1RXIF = 0;
	return;	
}//

/*u2 �����жϣ����ڽ���С���ַ����Ŀ���ָ��*/
void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void)
{
	unsigned char data[15];
	unsigned int UART_Timeout = 0;
	unsigned char i = 0;
	unsigned char j = 0;
	data[0] = U2RXREG;
	if('n'==data[0]) 	  // �����ã�����n�ſ�ʼ��Ƶ
	{
		net_state = 1;
		for(j=0;j<32;j++) //��ȡFRAM�д洢��ָ���ʽΪ15�ֽ�ָ��+17��0x22����32�ֽ�
		{
			lq[j] = READ(0,0,j);
		}
		
//lq   0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  ����  30 31			
//     S   R 				                                    E  22  22      22 22
//���� S   R   FF  FF  FF  00  FF  FF  FF  FF  FF  FF  FF  FF   E 
//���� S   R   00  FF  FF  00  FF  FF  FF  FF  FF  FF  FF  FF   E
//���� S   R   55  FF  FF  00  FF  FF  FF  FF  FF  FF  00  FF   E

		//���罨���ɹ��󣬲ɼ�ģ���Զ�����ϴι���״̬����֮ǰΪ����״̬�������������״̬����
		if((lq[0]=='S')&&(lq[1]=='R')&&(lq[14]=='E')&&(lq[31]==0x22))
		{	
			if((lq[2]==0xFF)&&(lq[5]==0x00)&&(lq[12]==0xff)) // ����ģʽ
				{
					Send_Enable = 1;
					Save_Enable = 0;
					Tran_Enable = 0;
					halt_Enable = 0;
				}

		}

	}//if('n'==data[0])
	else//���յ����ڷ�����15�ֽڵ�ָ����ʽΪS ���� E
	{
		switch(data[0])
		{
			case 'S':  
				while(data[i]!='E')
				{
					i++;
					while((0==(U2STA&0x0001))&&(UART_Timeout<5000))
					{UART_Timeout++;}
					if(UART_Timeout>=5000)
					{
						data[i]='E'; // ������ճ�ʱ���˳��ж�
					}
					else
					{
						data[i] = U2RXREG;
					}
					UART_Timeout = 0;
				}
				break;
			default:
				break;
		}
		if(i==14)
		{
			for(j=0;j<32;j++) //��ȡFRAM�д洢��ָ���ʽΪ15�ֽ�ָ��+17��0x22����32�ֽ�
			{
				lq[j] = READ(0,0,j);
			}
			
			U2TXREG = 'S';
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = ID;
			while((U2STA&0x0100)==0x0000){asm("nop");}
			for(j=0;j<29;j++)
			{
				U2TXREG = 0xff;
				while((U2STA&0x0100)==0x0000){asm("nop");}
			}
			U2TXREG = lq[2];
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = lq[5];
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = lq[12];
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = 'A';
			while((U2STA&0x0100)==0x0000){asm("nop");} //���յ�15�ֽ�ָ��󣬲���Ӧ��ָ�
													   //���ʽΪ S ID 0xff����0xff xx xx xx A ��35�ֽ�
			DELAY(6000);
						
//data 0   1   2   3   4   5   6   7   8   9   10   11   12   13   14			
//     S   R 				                                       E  
//���� S   R   FF  FF  FF  00  FF  FF  FF  FF  FF   FF   FF   FF   E 
//���� S   R   00  FF  FF  00  FF  FF  FF  FF  FF   FF   FF   FF   E
//���� S   R   55  FF  FF  00  FF  FF  FF  FF  FF   FF   00   FF   E
//�ض� S   R   55  FF  FF  FF  FF  FF  FF  FF  FF   FF   FF   FF   E
//��ѯ S   R   44  44  FF  44  FF  FF  FF  FF  FF   FF   44   FF   E
		
			if((data[2]==0xff)&&(data[5]==0x00)&&(data[12]==0xff)) // ����ģʽ
			{
				Send_Enable = 1;
				Save_Enable = 0;
				Tran_Enable = 0;
				halt_Enable = 0;
			}
			if((data[2]==0x00)&&(data[5]==0x00)&&(data[12]==0xff)) // ����ģʽ
			{	net_state = 0;//ͨ��ָ����������״̬ʱ������zigbee����10��35�ֽڵ�����
				Send_Enable = 0;
				Save_Enable = 1;
				Tran_Enable = 0;
				halt_Enable = 0;
				
				curr_addr[0] = 0;
				curr_addr[1] = 32;		
			}
			if((data[2]==0x55)&&(data[5]==0x00)&&(data[12]==0x00)) //����
			{	net_state = 0;//ͨ��ָ����������״̬ʱ������zigbee����10��35�ֽڵ�����
				Send_Enable = 0;
				Save_Enable = 0;
				Tran_Enable = 0;
				halt_Enable = 1;
			}	
			if(data[4]!=0xff)	//(����or����ģʽ��)���ò������
								//����������������״̬��ͬʱ���ò��������ͬһ��ָ���ڣ�
								//����������������״̬��ͬʱ���ò��������ͬһ��ָ���ڣ�
			{
				halt_Timeout = data[4]*500; 
			}
			
			if(data[5]==0xff) 	//�ض�FRAMֵ
			{
				Send_Enable = 0;
				Save_Enable = 0;
				Tran_Enable = 1;				
				halt_Enable = 0;
			}			
			if(data[6]!=0xff) 	//У׼ʵʱʱ�� 
			{
				ds1302_write_time(6,data[6]);
				ds1302_write_time(4,data[7]);
				ds1302_write_time(3,data[8]);
				ds1302_write_time(2,data[9]);
				ds1302_write_time(1,data[10]);
				ds1302_write_time(0,data[11]);
			}			
			if((data[3]==0x44)&&(data[2]==0x44)&&(data[5]==0x44)&&(data[12]==0x44)) //��ѯ����״̬
			{
				read_command_15_U2();
			}		
			
			//д15�ֽڵ�ָ��,��λ��0x22 ������00000~0001f  ,��32�ֽ�
			//��ʽΪ 15�ֽ�ָ��+17��0x22
			if(((data[2]==0xff)&&(data[5]==0x00)&&(data[12]==0xff))||((data[2]==0x00)&&(data[5]==0x00)&&(data[12]==0xff))||((data[2]==0x55)&&(data[5]==0x00)&&(data[12]==0x00)))	
				//ֻ�й���״̬����ָ��Ŵ洢 ������or����or���ߣ�														
			{
				for(j=0;j<15;j++)
				{
						WRITE(0,0,j,(unsigned char)data[j]); //data[0]~data[14] ����15�ֽ� ָ������
				}
				WRITE(0,0,15,ID);
				for(j=16;j<32;j++)
				{	
					WRITE(0,0,j,0x22);  // ����0x22
				}
			}	
		
		}//if(i==14)	
	}
	IFS1bits.U2RXIF = 0;
	return;	
}//

int main()
{
	int i,k,j,n;
	unsigned char year,month,day,hh,mm,ss;

	Read_Enable = 0;
	
	Send_Enable = 0;
	Save_Enable = 0;
	Tran_Enable = 0;
	halt_Enable = 1;

	bat_high = 0;
	bat_mid = 0;
	bat_low = 0;
	
	Tick = 0; //T6 �жϴ�������ʱ1ms�ж�
	net_state = 0;
	halt_Timeout = 1000; //(����or����״̬��)Ĭ�ϲ���ʱ����Ϊ1000*1ms = 1s

	OSCCON = 0x2200;

	TRISGbits.TRISG1 = 0;
	TRISGbits.TRISG0 = 0;
	TRISFbits.TRISF1 = 0;
	TRISFbits.TRISF0 = 0; 
	TRISDbits.TRISD7 = 0;
	TRISDbits.TRISD2 = 0;
	
	TRISGbits.TRISG15 = 0; //RESET
	RESET = 1;
	
	TRISDbits.TRISD8 = 1;
	TRISDbits.TRISD10 = 1;
	TRISDbits.TRISD11 = 1;
	TRISDbits.TRISD4 = 1;
	TRISDbits.TRISD5 = 1;
	TRISBbits.TRISB5 = 1;
	TRISBbits.TRISB12 = 0;
	TRISBbits.TRISB13 = 0;
	TRISBbits.TRISB14 = 0;	
	TRISGbits.TRISG13 = 0;
	TRISGbits.TRISG14 = 0;
	AD1PCFGLbits.PCFG5 = 1;
	TRISGbits.TRISG13 = 0;
	
	TRISBbits.TRISB15 = 1;
	TRISBbits.TRISB9 = 1;
	TRISBbits.TRISB1 = 1;	
	TRISCbits.TRISC1 = 1;
	TRISBbits.TRISB3 = 1;
	TRISCbits.TRISC2 = 1;
	TRISBbits.TRISB0 = 1;

	InitTimer6();  // Timer6 �ṩ1ms�ж϶�ʱ
	InitTimer2();  // Timer2 �ṩ���벶׽ʱ�ӻ�׼
	InitIC();
	InitADC();
	InitSPI();
	ds1302_init();
	InitSCI();

/*�ɼ�ģ���������Ȼ�ȡ֮ǰ�Ĺ���״̬
��֮ǰ��������״̬���������������������״̬
��֮ǰ��������״̬���������������������״̬
��֮ǰ��������״̬��������������δ�������򱣳�����״̬
*/
	for(j=0;j<32;j++)  
	{
		lq[j] = READ(0,0,j);
	}	
	if((lq[2]==0x00)&&(lq[5]==0x00)&&(lq[12]==0xff)) // ����ģʽ
	{	
		Send_Enable = 0;
		Save_Enable = 1;
		Tran_Enable = 0;
		halt_Enable = 0;
		curr_addr[0] = 0;
		curr_addr[1] = 32;	//һ��ʼ��FRAM�д�Ųɼ����ݴӵ�ַ 0x00020 ��ʼ
			
		}
	if((lq[2]==0x55)&&(lq[5]==0x00)&&(lq[12]==0x00)) //����
	{
		Send_Enable = 0;
		Save_Enable = 0;
		Tran_Enable = 0;
		halt_Enable = 1;
	}
			
	while(1)
	{			
		if(Send_Enable)  //���ߣ�ͨ��zigbee����λ�����Ͳɼ�����
						 //��ʽΪ S ID CHx Ƶ�ʸ߰�λ �Ͱ�λ �¶ȸ߰�λ �Ͱ�λ���� �����߰�λ �Ͱ�λ E ����35�ֽ�
		{
			if(net_state==1)//�����ɹ�����zigbee����10��35�ֽڵ�����
			{
				for(i=0;i<32;i++) //��ȡFRAM�д洢��ָ���ʽΪ15�ֽ�ָ��+17��0x22����32�ֽ�
					{
						lq[i] = READ(0,0,i);
					}
					
				for(j=0;j<10;j++)
				{
					U1TXREG = 'S';
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = ID;
					while((U1STA&0x0100)==0x0000){asm("nop");}
					for(i=0;i<28;i++)
					{
						U1TXREG = 0;
						while((U1STA&0x0100)==0x0000){asm("nop");}							
					}
					U1TXREG = 'A';
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = lq[2];
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = lq[5];
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = lq[12];
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = 'A';
					while((U1STA&0x0100)==0x0000){asm("nop");}
					DELAY(10000);	
				}	//10������ָ��			
				net_state=0;
			}
				
			i = 1000;
			while(i>=200)
			{
				for(k=0;k<3;k++)
				{
            		j = i;
            		CEB = 1; //RG1���Ϊ��
            		CEA = 1;
            		CEC = 1;
            		CED = 1;
            		CEE = 1;
            		CEF = 1;            	
            		while(j)
               		{    
	               		asm("nop");
                    	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
						j--;
                	}             
 
            		j = i;              
            		CEB = 0; //RG1���Ϊ��  
            		CEA = 0;
            		CEC = 0;
            		CED = 0;
            		CEE = 0;
            		CEF = 0;            		
            		while(j)
                	{  
	                	asm("nop");
                    	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
                    	j--;
                	}          
            	} 
				i = i-10;	 
			}
			DELAY(800);  // 600~9ms
			StartTimer6();
			StartTimer2();
			StartIC();
			while(!Read_Enable)	{asm("nop");}
			Read_Enable = 0;
			freq[0] = GetPeriod(1);
			freq[1] = GetPeriod(2);
			freq[2] = GetPeriod(3);
			freq[3] = GetPeriod(4);
			freq[4] = GetPeriod(5);
			freq[5] = GetPeriod(6);
			StopIC();
			StopTimer6();
						
		
			AD1CHS0bits.CH0SA = 15;
			AD1CON1bits.ADON = 1; // Turn on the A/D converter
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			bat = ADC1BUF0;
			AD1CON1bits.ADON = 0;
		
			if(bat>1489)  // >4.0V
			{	
				bat_high++;	bat_mid=0;	bat_low=0;
			}
			else 
				if((bat<=1489)&&(bat>1199))
				{	
					bat_high=0;	bat_mid++;bat_low=0;
				}
				else //<1199
				{	
					bat_high=0;	bat_mid=0;bat_low++;
				}//
		
			if(bat_high>=10)
			{
				bat_high=0;
				BAT_LED1 = 0;BAT_LED2 = 0;BAT_LED3 = 0;
			}
			else 
				if(bat_mid>=10)
				{
				bat_mid=0;
				BAT_LED1 = 1;BAT_LED2 = 0;BAT_LED3 = 0;
				}
				else 
					if(bat_low>=10)
					{
						bat_low=0;
						BAT_LED1 = 1;BAT_LED2 = 1;BAT_LED3 = 0;
					}//
				
/*��6��ͨ���Ĵ������������践�صĵ�ѹ������ADת������������temp[]�� */
		
			AD1CHS0bits.CH0SA = 9; //AN9
			AD1CON1bits.ADON = 1; // Turn on the A/D converter
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1; 		//ADC����ʹ��λ
										//1: ADC����/���ַŴ������ڲ���
										//0: ADC ����/���ַŴ������ֲ������
			while(!AD1CON1bits.DONE){}; //ADC ת��״̬λ
										//1 = ADC ת�����
										//0 = ADC ת����δ��ʼ���ڽ�����			
			temp[0] = ADC1BUF0;	//ÿ��A/D ת���Ľ���洢��ADCxBUF0�Ĵ�����
								//ADC ģ�����һ������ֻ��˫�˿ڼĴ�����ADCxBUF0��
			AD1CON1bits.ADON = 0;//ADC ����ģʽλ
								//1 = ADC ģ�����ڹ���
								//0 = ADC �ر�		
	
			AD1CHS0bits.CH0SA = 1;
			AD1CON1bits.ADON = 1;
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[1] = ADC1BUF0;	
			AD1CON1bits.ADON = 0;
	
			AD1CHS0bits.CH0SA = 16;
			AD1CON1bits.ADON = 1;
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[2] = ADC1BUF0;	
			AD1CON1bits.ADON = 0;
	
			AD1CHS0bits.CH0SA = 3;
			AD1CON1bits.ADON = 1;
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[3] = ADC1BUF0;	
			AD1CON1bits.ADON = 0;

			AD1CHS0bits.CH0SA = 17;
			AD1CON1bits.ADON = 1;
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[4] = ADC1BUF0;	
			AD1CON1bits.ADON = 0;

			AD1CHS0bits.CH0SA = 0;
			AD1CON1bits.ADON = 1;
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[5] = ADC1BUF0;	
			AD1CON1bits.ADON = 0;		


			//���ɼ�����ͨ��zigbee���͸���λ��
			U1TXREG = 'S';
			while((U1STA&0x0100)==0x0000){asm("nop");}
			U1TXREG = ID;
			while((U1STA&0x0100)==0x0000){asm("nop");}
			for(n=0;n<6;n++)
			{
				U1TXREG = n;
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = (unsigned char)((freq[n]&0xff00)>>8);
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = (unsigned char)(freq[n]&0x00ff);
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = (unsigned char)((temp[n]&0xff00)>>8);
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = (unsigned char)(temp[n]&0x00ff);
				while((U1STA&0x0100)==0x0000){asm("nop");}		
			}
			U1TXREG = (unsigned int)((bat&0xff00)>>8);
			while((U1STA&0x0100)==0x0000){asm("nop");}
			U1TXREG = (unsigned int)(bat&0x00ff);
			while((U1STA&0x0100)==0x0000){asm("nop");}
			U1TXREG = 'E';
			while((U1STA&0x0100)==0x0000){asm("nop");}	

	/*			
			//���ɼ�����ͨ��uart2���͸�С������ʾ
			U2TXREG = 'S';
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = ID;
			while((U2STA&0x0100)==0x0000){asm("nop");}
			for(n=0;n<6;n++)
			{
				U2TXREG = n+1;
				while((U2STA&0x0100)==0x0000){asm("nop");}
				U2TXREG = (unsigned char)((freq[n]&0xff00)>>8);
				while((U2STA&0x0100)==0x0000){asm("nop");}
				U2TXREG = (unsigned char)(freq[n]&0x00ff);
				while((U2STA&0x0100)==0x0000){asm("nop");}
				U2TXREG = (unsigned char)((temp[n]&0xff00)>>8);
				while((U2STA&0x0100)==0x0000){asm("nop");}
				U2TXREG = (unsigned char)(temp[n]&0x00ff);
				while((U2STA&0x0100)==0x0000){asm("nop");}		
			}
			U2TXREG = (unsigned int)((bat&0xff00)>>8);
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = (unsigned int)(bat&0x00ff);
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = 'E';
			while((U2STA&0x0100)==0x0000){asm("nop");}	
	*/		
			//������or����ģʽ�£����Ʋɼ�ʱ����		
			halt = 1;
			halt_Tick = 0;
			StartTimer6();
    		while(halt){asm("NOP");} 
			
		}//if(Send_Enable)
	 
		if(Save_Enable)  //���ߣ���FRAM�б�������
						 //��������ݸ�ʽΪ S �� �� �� ʱ �� �� Ƶ�ʸ߰�λ �Ͱ�λ �¶ȸ߰�λ �Ͱ�λ���� E ��32�ֽ�
		{
				
			if(net_state==1)//�����ɹ�����zigbee����10��35�ֽڵ�����
			{
				for(i=0;i<32;i++) //��ȡFRAM�д洢��ָ���ʽΪ15�ֽ�ָ��+17��0x22����32�ֽ�
					{
						lq[i] = READ(0,0,i);
					}
					
				for(j=0;j<10;j++)
				{
					U1TXREG = 'S';
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = ID;
					while((U1STA&0x0100)==0x0000){asm("nop");}
					for(i=0;i<28;i++)
					{
						U1TXREG = 0;
						while((U1STA&0x0100)==0x0000){asm("nop");}							
					}
					U1TXREG = 'A';
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = lq[2];
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = lq[5];
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = lq[12];
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = 'A';
					while((U1STA&0x0100)==0x0000){asm("nop");}
					DELAY(10000);	
				}	//10������ָ��
			
				net_state=0;
			}
			
			i = 1000;
			while(i>=200)
			{
				for(k=0;k<3;k++)
				{
            		j = i;
            		CEB = 1; //RG1���Ϊ��
            		CEA = 1;
            		CEC = 1;
            		CED = 1;
            		CEE = 1;
            		CEF = 1;            	
            		while(j)
               		{    
	               		asm("nop");
                    	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
						j--;
                	}             
 
            		j = i;              
            		CEB = 0; //RG1���Ϊ��  
            		CEA = 0;
            		CEC = 0;
            		CED = 0;
            		CEE = 0;
            		CEF = 0;            		
            		while(j)
                	{  
	                	asm("nop");
                    	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
                    	j--;
                	}          
            	} 
				i = i-10;	 
			}
			DELAY(800);  // 600~9ms
			StartTimer6();
			StartTimer2();
			StartIC();
			while(!Read_Enable)	{asm("nop");}
			Read_Enable = 0;
			freq[0] = GetPeriod(1);
			freq[1] = GetPeriod(2);
			freq[2] = GetPeriod(3);
			freq[3] = GetPeriod(4);
			freq[4] = GetPeriod(5);
			freq[5] = GetPeriod(6);
			StopIC();
			StopTimer6();
		
			AD1CHS0bits.CH0SA = 15;
			AD1CON1bits.ADON = 1; // Turn on the A/D converter
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			bat = ADC1BUF0;
			AD1CON1bits.ADON = 0;
		
			if(bat>1489)  // >4.0V
			{
				bat_high++;	bat_mid=0;	bat_low=0;
			}
			else 
				if((bat<=1489)&&(bat>1199))
				{	
					bat_high=0;	bat_mid++;bat_low=0;
				}
				else //<1199
				{	
					bat_high=0;	bat_mid=0;bat_low++;
				}//

		
			if(bat_high>=10)
			{
				bat_high=0;
				BAT_LED1 = 0;BAT_LED2 = 0;BAT_LED3 = 0;
			}
			else 
				if(bat_mid>=10)
				{
					bat_mid=0;
					BAT_LED1 = 1;BAT_LED2 = 0;BAT_LED3 = 0;
				}
				else 
					if(bat_low>=10)
					{
						bat_low=0;
						BAT_LED1 = 1;BAT_LED2 = 1;BAT_LED3 = 0;
					}//
				
			//��6��ͨ���Ĵ������������践�صĵ�ѹ������ADת������������temp[]�� 		
			AD1CHS0bits.CH0SA = 9;
			AD1CON1bits.ADON = 1; // Turn on the A/D converter
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[0] = ADC1BUF0;
			AD1CON1bits.ADON = 0;
	
			AD1CHS0bits.CH0SA = 1;
			AD1CON1bits.ADON = 1;
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[1] = ADC1BUF0;	
			AD1CON1bits.ADON = 0;
	
			AD1CHS0bits.CH0SA = 16;
			AD1CON1bits.ADON = 1;
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[2] = ADC1BUF0;	
			AD1CON1bits.ADON = 0;
	
			AD1CHS0bits.CH0SA = 3;
			AD1CON1bits.ADON = 1;
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[3] = ADC1BUF0;	
			AD1CON1bits.ADON = 0;

			AD1CHS0bits.CH0SA = 17;
			AD1CON1bits.ADON = 1;
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[4] = ADC1BUF0;	
			AD1CON1bits.ADON = 0;

			AD1CHS0bits.CH0SA = 0;
			AD1CON1bits.ADON = 1;
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			temp[5] = ADC1BUF0;	
			AD1CON1bits.ADON = 0;		
		
			year = ds1302_read_time(6);
			month = ds1302_read_time(4);
			day = ds1302_read_time(3);
			hh = ds1302_read_time(2);
			mm = ds1302_read_time(1);
			ss = ds1302_read_time(0);
			
//			U2TXREG = year;
//			while((U2STA&0x0100)==0x0000){asm("nop");}
//			U2TXREG = month;
//			while((U2STA&0x0100)==0x0000){asm("nop");}
//			U2TXREG = day;
//			while((U2STA&0x0100)==0x0000){asm("nop");}
/*			U2TXREG = hh;
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = mm;
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = ss;
			while((U2STA&0x0100)==0x0000){asm("nop");}
*/			
			
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),'S');
			inc_addr();			
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),year);
			inc_addr();
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),month);
			inc_addr();
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),day);
			inc_addr();
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),hh);
			inc_addr();
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),mm);
			inc_addr();
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),ss);
			inc_addr();
			for(n=0;n<6;n++)
			{
				WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),(unsigned char)((freq[n]&0xff00)>>8));
				inc_addr();
				WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),(unsigned char)(freq[n]&0x00ff));
				inc_addr();
				WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),(unsigned char)((temp[n]&0xff00)>>8));
				inc_addr();
				WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),(unsigned char)(temp[n]&0x00ff));
				inc_addr();
			}
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),'E');
			inc_addr();			
					
			halt = 1;
			halt_Tick = 0;
			StartTimer6();
    		while(halt){asm("NOP");} //������or����ģʽ�£����Ʋɼ�ʱ����
		}//if(Save_Enable)
			 
		if(Tran_Enable)  //�ض�����ȡ��FRAM�е�����
		{
			Tran_Data_U1(); //����λ������FRAM�е�����*******************************************
		//	Tran_Data_U2(); //��С���ִ���FRAM�е�����
			Tran_Enable = 0; //���ͽ����󣬽����ͱ������㣬ȷ��ֻ����һ��
		}
		if(halt_Enable)  //���ߣ���Ҫ��ʾ����
		{	
			if(net_state==1)//�����ɹ�����zigbee����10��35�ֽڵ�����
			{
				for(i=0;i<32;i++) //��ȡFRAM�д洢��ָ���ʽΪ15�ֽ�ָ��+17��0x22����32�ֽ�
					{
						lq[i] = READ(0,0,i);
					}
					
				for(j=0;j<10;j++)
				{
					U1TXREG = 'S';
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = ID;
					while((U1STA&0x0100)==0x0000){asm("nop");}
					for(i=0;i<28;i++)
					{
						U1TXREG = 0;
						while((U1STA&0x0100)==0x0000){asm("nop");}							
					}
					U1TXREG = 'A';
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = lq[2];
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = lq[5];
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = lq[12];
					while((U1STA&0x0100)==0x0000){asm("nop");}
					U1TXREG = 'A';
					while((U1STA&0x0100)==0x0000){asm("nop");}
					DELAY(10000);	
				}	//10������ָ��
			
				net_state=0;
			}
					
			AD1CHS0bits.CH0SA = 15;
			AD1CON1bits.ADON = 1; // Turn on the A/D converter
			asm("nop");asm("nop");asm("nop");asm("nop");
			AD1CON1bits.SAMP = 1;
			while(!AD1CON1bits.DONE){};
			bat = ADC1BUF0;
			AD1CON1bits.ADON = 0;
		
			if(bat>1489)  // >4.0V
			{	
				bat_high++;	bat_mid=0;bat_low=0;
			}
			else 
				if((bat<=1489)&&(bat>1199))
				{	
					bat_high=0;	bat_mid++;	bat_low=0;
				}
				else //<1199
				{	
					bat_high=0;	bat_mid=0;	bat_low++;
				}//

			if(bat_high>=10)
			{
				bat_high=0;
				BAT_LED1 = 0;BAT_LED2 = 0;BAT_LED3 = 0;
			}
			else 
				if(bat_mid>=10)
				{
					bat_mid=0;
					BAT_LED1 = 1;BAT_LED2 = 0;BAT_LED3 = 0;
				}
				else 
					if(bat_low>=10)
					{
						bat_low=0;
						BAT_LED1 = 1;BAT_LED2 = 1;BAT_LED3 = 0;
					}//
					
/***********************					
			U2TXREG = 'E';
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = (unsigned int)((bat&0xff00)>>8);
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = (unsigned int)(bat&0x00ff);
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = 'E';
			while((U2STA&0x0100)==0x0000){asm("nop");}	
*/
		}//if(halt_Enable)
			
		asm("NOP");		
	}//while(1)	
	return 0;
}//main()

void inc_addr(void)
{
	if(curr_addr[1]==0xffff)
	{
		if(curr_addr[0]==1)
		{
			curr_addr[0]=0;
			curr_addr[1]=32;
		}else
		{
			curr_addr[0]=1;
			curr_addr[1]=0;
		}
	}
	else
	{
		curr_addr[1]=curr_addr[1]+1;
	}
}//

/*u1 ��������λ������FRAM����*/
void Tran_Data_U1(void) // *u1 ��������λ������FRAM����*
{
	unsigned char temp,i;
//	unsigned int j;
	curr_addr[0] = 0;
	curr_addr[1] = 32;

	//��ʼ�����ļ�ͷ����ʽΪ S T 0x00....0x00 E �ܹ�35���ֽڣ��м���32��0x00
	U1TXREG = 'S';
	while((U1STA&0x0100)==0x0000){asm("nop");}
	U1TXREG = 'T';
	while((U1STA&0x0100)==0x0000){asm("nop");}
	for(i=0;i<32;i++)
	{
		U1TXREG = 0x00;
		while((U1STA&0x0100)==0x0000){asm("nop");}
	}
	U1TXREG = 'E';
	while((U1STA&0x0100)==0x0000){asm("nop");}
	DELAY(6000);

	read_count = 4096; //��FRAM��ȡread_count������
//	read_count = 10;
	while(read_count)
	{
		temp =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
		U1TXREG = temp;
		while((U1STA&0x0100)==0x0000){asm("nop");}
		inc_addr();
	
		U1TXREG = 0x55;
		while((U1STA&0x0100)==0x0000){asm("nop");}
		U1TXREG = 0x55;
		while((U1STA&0x0100)==0x0000){asm("nop");}
		U1TXREG = 0x55;
		while((U1STA&0x0100)==0x0000){asm("nop");}
		
		for(i=0;i<30;i++)
		{	temp =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
			U1TXREG = temp;
			while((U1STA&0x0100)==0x0000){asm("nop");}
			inc_addr();
		}
		
		temp =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
		U1TXREG = temp;
		while((U1STA&0x0100)==0x0000){asm("nop");}
		inc_addr();
		DELAY(6000);
		
		read_count--;
	}	
		
	// ���ͽ���������ʽΪ S T 0xff...0xff E �м���32��0xff
	U1TXREG = 'S';
	while((U1STA&0x0100)==0x0000){asm("nop");}
	U1TXREG = 'T';
	while((U1STA&0x0100)==0x0000){asm("nop");}
	for(i=0;i<32;i++)
	{
		U1TXREG = 0xff;
		while((U1STA&0x0100)==0x0000){asm("nop");}
	}
	U1TXREG = 'E';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
}//

/*u1 ��������λ������FRAM�д洢��ָ��*/
void read_command_15_U1(void) // 
{
	unsigned char j,temp;
	//��֮ǰ����fram 00000~0001f�е�ָ��ͳ���
	for(j=0;j<32;j++)
	{
		temp =  READ(0,0,j);
		U1TXREG = temp;
		while((U1STA&0x0100)==0x0000){asm("nop");}
	}
	U1TXREG = 'C';
	while((U1STA&0x0100)==0x0000){asm("nop");}
	U1TXREG = 'M';
	while((U1STA&0x0100)==0x0000){asm("nop");}
	U1TXREG = 0x42;
	while((U1STA&0x0100)==0x0000){asm("nop");}

}//


void Tran_Data_U2(void) // *u2 ��������λ��(С����)����FRAM����*
{
	unsigned char temp,i;
//	unsigned int j;
	curr_addr[0] = 0;
	curr_addr[1] = 32; //ÿ�ζ�ȡFRAM�е����ݶ��ӵ�ַ 0x00020 ��ʼ

	//��ʼ�����ļ�ͷ����ʽΪ S T 0x00....0x00 E �ܹ�35���ֽڣ��м���32��0x00
	U2TXREG = 'S';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	U2TXREG = 'T';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	for(i=0;i<32;i++)
	{
		U2TXREG = 0x00;
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	U2TXREG = 'E';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	DELAY(6000);
		
	read_count = 10; //��FRAM��ȡread_count������
	while(read_count)
	{
		temp =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
		U2TXREG = temp;
		while((U2STA&0x0100)==0x0000){asm("nop");}
		inc_addr();
	
		U2TXREG = 0x55;
		while((U2STA&0x0100)==0x0000){asm("nop");}
		U2TXREG = 0x55;
		while((U2STA&0x0100)==0x0000){asm("nop");}
		U2TXREG = 0x55;
		while((U2STA&0x0100)==0x0000){asm("nop");}
		
		for(i=0;i<30;i++)
		{	temp =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
			U2TXREG = temp;
			while((U2STA&0x0100)==0x0000){asm("nop");}
			inc_addr();
		}
		
		temp =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
		U2TXREG = temp;
		while((U2STA&0x0100)==0x0000){asm("nop");}
		inc_addr();
		DELAY(6000);
		
		read_count--;
	}	
		
	// ���ͽ���������ʽΪ S T 0xff...0xff E �м���32��0xff
	U2TXREG = 'S';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	U2TXREG = 'T';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	for(i=0;i<32;i++)
	{
		U2TXREG = 0xff;
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	U2TXREG = 'E';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
}//


void read_command_15_U2(void) // *u2 ��������λ������FRAM�д洢��ָ��*
{
	unsigned char j,temp;
	
	//��֮ǰ����fram 00000~0001f�е�ָ��ͳ���
	for(j=0;j<32;j++)
	{
		temp =  READ(0,0,j);
		U2TXREG = temp;
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	U2TXREG = 'C';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	U2TXREG = 'M';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	U2TXREG = 0x42;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
}//



