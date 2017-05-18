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
#include "ecan.h"

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
#define CEC LATGbits.LATG2
#define CED LATGbits.LATG3
#define CEE LATDbits.LATD7
#define CEF LATDbits.LATD2
#define CEG LATFbits.LATF2
#define CEH LATFbits.LATF3
//#define WORK_LED LATDbits.LATD1
#define BOARD_NUM 2  // Number of board


unsigned int freq[BOARD_NUM][8],lq; 
unsigned int temp_freq[8];	
float freq_float;			  
unsigned int temp[BOARD_NUM][8]; // Temperature
unsigned int bat; 
unsigned int dummy;
short bat_high, bat_mid, bat_low; // �����������ڶԵ�ص�ѹ�����˲�
short halt; // 1-ֹͣ������0-��ʼ����
//unsigned int TxBuffer[16] __attribute__((space(dma)));

int CE_Enable; // 1->����Ƶ  0->��ֹ��Ƶ 
int Read_Enable; // 1->ʰƵ��ɣ����Զ�ȡ  0->��ֹ��ȡ
int Tick; // ������¼��ʱ�жϴ���
int CE_Tick;
unsigned int halt_Tick; // ������¼ʱ�䣬��ʱ��Ϊ���η���֮���ʱ����
unsigned int halt_Timeout; // ���η���֮��ļ������λΪms����ͨ����λ������
unsigned char Send_Enable; // ����ָʾ�Ƿ���Է������ݣ��ϵ翪ʼʱ���������ݣ��յ�ZigBeeоƬ���͵�'n'�ַ����������
unsigned char Save_Enable; // ������ʾ�Ƿ���FRAM�д�����
unsigned char Tran_Enable; // ������ʾ�Ƿ�FRAM�е����ݶ���
unsigned char halt_Enable; // ����
unsigned char ID = 0; // ���******************************************************
unsigned char Read_Timer_En; 
unsigned char CE_Timer_En;
unsigned int filter_Tick; // �˲�����
unsigned char filter_Enable; // �Ƿ����˲������У�1-�ǣ�0-��
unsigned int filter_freq[6]; // �˲����4������ֵ
unsigned char Slave_Message_Count[BOARD_NUM-1]; // ��¼���Ӱ巵�ص����ݸ���
unsigned int filter[6][4]; // ����ƽ���˲����� 

unsigned char stat1;

unsigned int end_addr[2]; // �洢������ַ���洢��FRAM 0��1��2��ַ��
unsigned char temp_addr[3];
unsigned int curr_addr[2];
unsigned int read_count; //������¼��FRAM�ֶ�ȡ���ַ�������������������ͷ���������������

unsigned char state;

mID canTxMessage;
mID canRxMessage;
ECAN1MSGBUF ecan1msgBuf __attribute__((space(dma),aligned(ECAN1_MSG_BUF_LENGTH*16)));

void inc_addr(void); // ��curr_addr��1
void Tran_Data(void); // ����λ����������

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
 }

void __attribute__((interrupt,no_auto_psv)) _T6Interrupt(void)  // 1ms interrupt
{
	IFS2bits.T6IF = 0;
	if(Read_Timer_En)
	{
		Tick++;
		if(Tick>19)//20ms
		{
			Tick = 0;
			Read_Enable = 1;
			Read_Timer_En = 0;	
		}
	}
	if(CE_Timer_En)
	{
		CE_Tick++;
		if(CE_Tick>2000)
		{
			CE_Tick = 0;
			CE_Enable = 1;
			CE_Timer_En = 0;	
		}
	}			
}

void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void)
{
	
	IFS1bits.U2RXIF = 0;
	return;	
}


void __attribute__((interrupt,no_auto_psv))_C1Interrupt(void)  
{
	/* check to see if the interrupt is caused by receive */     	 
    if(C1INTFbits.RBIF)
    {
	    /* check to see if buffer 1 is full */
	    if(C1RXFUL1bits.RXFUL1)
	    {			
			/* set the buffer full flag and the buffer received flag */
			canRxMessage.buffer_status=CAN_BUF_FULL;
			canRxMessage.buffer=1;
				
		}		
		/* check to see if buffer 2 is full */
		else if(C1RXFUL1bits.RXFUL2)
		{
			/* set the buffer full flag and the buffer received flag */
			canRxMessage.buffer_status=CAN_BUF_FULL;
			canRxMessage.buffer=2;
								
		}
		/* check to see if buffer 3 is full */
		else if(C1RXFUL1bits.RXFUL3)
		{
			/* set the buffer full flag and the buffer received flag */
			canRxMessage.buffer_status=CAN_BUF_FULL;
			canRxMessage.buffer=3;
								
		}
		else;
		/* clear flag */
		C1INTFbits.RBIF = 0;
	}
	else if(C1INTFbits.TBIF)
    {
	    /* clear flag */
		C1INTFbits.TBIF = 0;	    
	}
	else;
	
	/* clear interrupt flag */
	IFS2bits.C1IF=0;
}

int main()
{
	int i = 1000; // ɨƵƵ�� 473Hz~7.042kHz
	int k,j;
	int n;
//	int f; // �˲�ѭ������
	unsigned char year,month,day,hh,mm,ss;
	read_count = 0;
	CE_Enable = 1; 
	Read_Enable = 0;
	Send_Enable = 0;
	Save_Enable = 0;
	Tran_Enable = 0;
	halt_Enable = 0;
	CE_Timer_En = 1;
	Read_Timer_En = 0;
	CE_Tick = 0;
	Tick = 0;
	bat_high = 0; 
	bat_mid = 0;
	bat_low = 0;
	halt = 0;
	halt_Timeout = 600; //**************************************************
	filter_Tick = 0;
	filter_Enable = 1; // ��ʼ������ʼ�˲�
	filter_freq[0] = 0; filter_freq[1] = 0; filter_freq[2] = 0; filter_freq[3] = 0; filter_freq[4] = 0; filter_freq[5] = 0;
	OSCCON = 0x2200;
	
	TRISGbits.TRISG1 = 0;
	TRISGbits.TRISG0 = 0;
	TRISGbits.TRISG2 = 0;
	TRISGbits.TRISG3 = 0;
	TRISDbits.TRISD7 = 0;
	TRISDbits.TRISD2 = 0;
	TRISFbits.TRISF2 = 0;
	TRISFbits.TRISF3 = 0;
	
	TRISFbits.TRISF1 = 0;
	TRISFbits.TRISF0 = 1; 
	
	TRISGbits.TRISG15 = 0;
	
	TRISDbits.TRISD8 = 1;
	TRISDbits.TRISD9 = 1;
	TRISDbits.TRISD10 = 1;
	TRISDbits.TRISD11 = 1;
	TRISDbits.TRISD4 = 1;
	TRISDbits.TRISD5 = 1;
	TRISBbits.TRISB5 = 1;
	TRISBbits.TRISB4 = 1;
	
	
	TRISGbits.TRISG13 = 0;
	TRISGbits.TRISG14 = 0;
	AD1PCFGLbits.PCFG5 = 1;
	AD1PCFGLbits.PCFG4 = 1;
	InitTimer6();  // Timer6 �ṩ1ms�ж϶�ʱ
	InitTimer2();  // Timer2 �ṩ���벶׽ʱ�ӻ�׼
	InitIC();
	InitADC();
	InitSCI();
	InitSPI();
	ds1302_init();
	InitSCI();
	initECAN();
	initDMAECAN();
	IEC2bits.C1IE=1;
	C1INTEbits.RBIE=1;
	while(1)
	{
		
		if(CE_Enable) // ��ʼ��Ƶ
		{
			i = 1000;
						
			canTxMessage.message_type=CAN_MSG_DATA;		
			canTxMessage.frame_type=CAN_FRAME_STD;
			canTxMessage.buffer=0;
			canTxMessage.id=0x001;
			canTxMessage.data[0]=0x01;
			canTxMessage.data[1]=0x02;
			canTxMessage.data[2]=0x03;
			canTxMessage.data[3]=0x04;
			canTxMessage.data[4]=0x05;
			canTxMessage.data[5]=0x06;
			canTxMessage.data[6]=0x07;
			canTxMessage.data[7]=0x08;
			canTxMessage.data_length=8;
			sendECAN(&canTxMessage);//���ͱ��ģ�ָʾ����slave��ʼ��Ƶ
			
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
            		CEG = 1;
            		CEH = 1;
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
            		CEG = 0;
            		CEH = 0;
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
			Read_Timer_En = 1;
			while(!Read_Enable)
			{asm("nop");}
				
			if(filter_Tick<5)
				filter_Tick++;
			Read_Enable = 0;
			
			freq[0][0] = GetPeriod(1);
			freq[0][1] = GetPeriod(2);
			freq[0][2] = GetPeriod(3);
			freq[0][3] = GetPeriod(4);
			freq[0][4] = GetPeriod(5);
			freq[0][5] = GetPeriod(6);
			freq[0][6] = GetPeriod(7);
			freq[0][7] = GetPeriod(8);
			StopIC();
			StopTimer6();
/*****filter				for(f=0;f<6;f++)
			{
				filter[f][0]=filter[f][1];
				filter[f][1]=filter[f][2];
				filter[f][2]=filter[f][3];
				filter[f][3]=(freq[f]>>2);
				filter_freq[f]=filter[f][0]+filter[f][1]+filter[f][2]+filter[f][3];
			}
filter********/	

			DELAY(800);
			for(n=0;n<BOARD_NUM-1;n++)
			{				
				canTxMessage.message_type=CAN_MSG_DATA;		
				canTxMessage.frame_type=CAN_FRAME_STD;
				canTxMessage.buffer=0;
				canTxMessage.id=(0x002|(n+1)<<4);
				canTxMessage.data[0]=0x01;
				canTxMessage.data[1]=0x02;
				canTxMessage.data[2]=0x03;
				canTxMessage.data[3]=0x04;
				canTxMessage.data[4]=0x05;
				canTxMessage.data[5]=0x06;
				canTxMessage.data[6]=0x07;
				canTxMessage.data[7]=0x08;
				canTxMessage.data_length=8;
				sendECAN(&canTxMessage);//���ͱ��ģ�ָʾ��n+1����ӷ��زɼ�ֵ
				
				Slave_Message_Count[n] = 0;
				while(Slave_Message_Count[n]<2)  // ÿ���Ӱ���Ҫ�ظ�2֡
				{
					if(canRxMessage.buffer_status==CAN_BUF_FULL)
					{
						if(canRxMessage.buffer==(n+1))
						{
							rxECAN(&canRxMessage);
							canRxMessage.buffer_status=CAN_BUF_EMPTY;
							Slave_Message_Count[n]++;
							if(canRxMessage.id == (0x001|(n+1)<<8))
							{
								freq[n+1][0] = ((unsigned int)(canRxMessage.data[0])<<8)+canRxMessage.data[1];
								freq[n+1][1] = ((unsigned int)(canRxMessage.data[2])<<8)+canRxMessage.data[3];
								freq[n+1][2] = ((unsigned int)(canRxMessage.data[4])<<8)+canRxMessage.data[5];
								freq[n+1][3] = ((unsigned int)(canRxMessage.data[6])<<8)+canRxMessage.data[7];
							}
							else if(canRxMessage.id == (0x002|(n+1)<<8))
							{
								freq[n+1][4] = ((unsigned int)(canRxMessage.data[0])<<8)+canRxMessage.data[1];
								freq[n+1][5] = ((unsigned int)(canRxMessage.data[2])<<8)+canRxMessage.data[3];
								freq[n+1][6] = ((unsigned int)(canRxMessage.data[4])<<8)+canRxMessage.data[5];
								freq[n+1][7] = ((unsigned int)(canRxMessage.data[6])<<8)+canRxMessage.data[7];
							}
						}
					}
				}
				
			}									
		}
		
		for(n=0;n<8;n++)
		{
//filter*****			freq[n] = filter_freq[n]; // ������ֵ�����ۼ�4�κ��ԭ����ֵ
			//TEST = 1;
			if(temp_freq[n]!=0)
			{
				temp_freq[n] = freq[0][n]>>2;
				freq_float = ((float)temp_freq[n] * 0.4f);
				freq_float = freq_float/1000;
				freq_float = 10000/freq_float;
				temp_freq[n] = (unsigned int)freq_float;
			}
			
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
			bat_high++;
			bat_mid=0;
			bat_low=0;
		}
		else if((bat<=1489)&&(bat>1199))
		{	
			bat_high=0;
			bat_mid++;
			bat_low=0;
		}
		else
		{	
			bat_high=0;
			bat_mid=0;
			bat_low++;
		}

		
		if(bat_high>=10)
		{
			bat_high=0;
			
		}else if(bat_mid>=10)
		{
			bat_mid=0;
			
		}else if(bat_low>=10)
		{
			bat_low=0;
			
		}
		/*
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
		*/
		if(filter_Tick>4)
		{
			//WREN();
			//stat1 = RDSR();
			//stat1=READ(0x00,0x00);			
		}
			
		if(Send_Enable)
		{
			/*
			U2TXREG = 'S';
			while((U2STA&0x0100)==0x0000){asm("nop");}
			U2TXREG = 0;
			while((U1STA&0x0100)==0x0000){asm("nop");}
			for(n=0;n<8;n++)
			{
				U1TXREG = n;
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = (unsigned char)((freq[0][n]&0xff00)>>8);
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = (unsigned char)(freq[0][n]&0x00ff);
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = (unsigned char)((temp[0][n]&0xff00)>>8);
				while((U1STA&0x0100)==0x0000){asm("nop");}
				U1TXREG = (unsigned char)(temp[0][n]&0x00ff);
				while((U1STA&0x0100)==0x0000){asm("nop");}		
			}
			U1TXREG = (unsigned int)((bat&0xff00)>>8);
			while((U1STA&0x0100)==0x0000){asm("nop");}
			U1TXREG = (unsigned int)(bat&0x00ff);
			while((U1STA&0x0100)==0x0000){asm("nop");}
			U1TXREG = 'E';
			while((U1STA&0x0100)==0x0000){asm("nop");}
			*/	
		}else if(Save_Enable)  //��FRAM�б�������
		{
			year = ds1302_read_time(6);
			month = ds1302_read_time(4);
			day = ds1302_read_time(3);
			hh = ds1302_read_time(2);
			mm = ds1302_read_time(1);
			ss = ds1302_read_time(0);
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),'S');
			inc_addr();
			 
			//�˶δ洢������Ϣ
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),year);
			inc_addr();
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),month);
			inc_addr();
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),day);
			inc_addr();
			//�˶δ洢������Ϣ
			
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),hh);
			inc_addr();
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),mm);
			inc_addr();
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),ss);
			inc_addr();
			for(n=0;n<8;n++)
			{
				WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),(unsigned char)((freq[0][n]&0xff00)>>8));
				inc_addr();
				WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),(unsigned char)(freq[0][n]&0x00ff));
				inc_addr();
				WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),(unsigned char)((temp[0][n]&0xff00)>>8));
				inc_addr();
				WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),(unsigned char)(temp[0][n]&0x00ff));
				inc_addr();
			}
			WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),'E');
			inc_addr();
			
			if((curr_addr[0]==1)&&(curr_addr[1]>=0xffe3))/////////////////////////////////***
			{curr_addr[0]=0;curr_addr[1]=3;}
			WRITE(0,0,0,(unsigned char)curr_addr[0]);
			WRITE(0,0,1,(unsigned char)(curr_addr[1]>>8));
			WRITE(0,0,2,(unsigned char)(curr_addr[1]&0x00ff));
			stat1 = READ(0,0,2);  //���öϵ�,�ڵڶ��ε�ʱ��stat1��ֵ
		}else if(Tran_Enable)
		{
			Tran_Data(); //����λ������FRAM�е�����
			Tran_Enable = 0; //���ͽ����󣬽����ͱ������㣬ȷ��ֻ����һ��
		}

		CE_Enable = 0;
		CE_Timer_En = 1;
		StartTimer6();
		
		
	}
	
	return 0;
}

void inc_addr(void)
{
	if(curr_addr[1]==0xffff)
	{
		if(curr_addr[0]==1)
		{
			curr_addr[0]=0;
			curr_addr[1]=3;
		}else
		{
			curr_addr[0]=1;
			curr_addr[1]=0;
		}
	}else
	{
		curr_addr[1]=curr_addr[1]+1;
	}
}


void Tran_Data(void)
{
	unsigned char temp1,i;
	unsigned char empty;
	empty = 0; // �洢���ձ�־λ��1��ʾ�洢����
	//unsigned int end_addr[2];
	temp1 =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
	//��ʼ�����ļ�ͷ����ʽΪ S T 0x00....0x00 E �ܹ�32���ֽڣ��м���32��0x00
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
	
	if(temp1 == 'S') // ����FRAM�е������Ѿ������������ڸ��� 
	{
		// ����洢����λ�ã�λ��curr_addr-1
		if((curr_addr[0]==1)&&(curr_addr[1]==0))
		{
			end_addr[0]=0; end_addr[1]=0xffff;
		}else if((curr_addr[0]==0)&&(curr_addr[1]==3))
		{
			end_addr[0]=1; end_addr[1]=0xffe2;
		}else
		{
			end_addr[0]=curr_addr[0]; end_addr[1]=curr_addr[1]-1;
		}
		
		while((curr_addr[0]!=end_addr[0])||(curr_addr[1]!=end_addr[1]))
		{	
			temp1 =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
			U2TXREG = temp1;
			while((U2STA&0x0100)==0x0000){asm("nop");}
			inc_addr();
			if((curr_addr[0]==1)&&(curr_addr[1]>=0xffe3))/////////////////////////////////***
			{curr_addr[0]=0;curr_addr[1]=3;}
			if(read_count==0)
			{
				U2TXREG = 0x55;
				while((U2STA&0x0100)==0x0000){asm("nop");}
				U2TXREG = 0x55;
				while((U2STA&0x0100)==0x0000){asm("nop");}
				U2TXREG = 0x55;
				while((U2STA&0x0100)==0x0000){asm("nop");}
			}
			
			if(read_count<31)
			{
				read_count++;
			}else
			{
				read_count=0;
			}
//			DELAY(500);	// ���ô���С���ַ��ʹ��ڿ��������ȡ�洢��ֵ�Ļ�Ӧ��ɾ�� ��806��
		}
			
	}else //�洢����������δ�����������ڸ���
	{
		if((curr_addr[0]==1)&&(curr_addr[1]==0))
		{
			end_addr[0]=0; end_addr[1]=0xffff;
		}else if((curr_addr[0]==0)&&(curr_addr[1]==3))
		{
			end_addr[0]=0; end_addr[1]=3;  //��ʱ˵���洢����û������
			empty = 1;
		}else
		{
			end_addr[0]=curr_addr[0]; end_addr[1]=curr_addr[1]-1;
		}
		curr_addr[0]=0; curr_addr[1]=3; //����ʼ��ַ��Ϊ�׵�ַ
		while((curr_addr[0]!=end_addr[0])||(curr_addr[1]!=end_addr[1]))
		{	
			temp1 =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
			U2TXREG = temp1;
			while((U2STA&0x0100)==0x0000){asm("nop");}
			inc_addr();
			if((curr_addr[0]==1)&&(curr_addr[1]>=0xffe3))/////////////////////////////////***
			{curr_addr[0]=0;curr_addr[1]=3;}
			if(read_count==0)
			{
				U2TXREG = 0x55;
				while((U2STA&0x0100)==0x0000){asm("nop");}
				U2TXREG = 0x55;
				while((U2STA&0x0100)==0x0000){asm("nop");}
				U2TXREG = 0x55;
				while((U2STA&0x0100)==0x0000){asm("nop");}
			}
			
			if(read_count<31)
			{
				read_count++;
			}else
			{
				read_count=0;
			}
//			DELAY(500);
		}
		
	}
	
	if(empty!=1) //����洢���������ݣ�������һ�����ݵĽ�������������������ݣ��򲻷��͸ý�����
	{
		U2TXREG = 'E';
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	
	// ���ͽ���������ʽΪ S T 0xff...0xff E �м���29��0xff
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
}
