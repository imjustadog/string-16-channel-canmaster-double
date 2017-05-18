
#if defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24hxxxx.h>
#endif

#include "main.h"
#include "timer.h"
#include "adc.h"
#include "sci.h"
#include "IC.h"
#include "DS1302.h"
#include "FRAM.h"
#include <string.h>

_FBS(0xCF);
_FSS(0xCF);
_FGS(0x07);
_FOSCSEL(0xA2);  //Select HS without PLL
_FOSC(0x46);  // HS oscilator, OSC2 is clock output
_FWDT(0x5F);
_FPOR(0xE7);
_FICD(0xC3);

#define CEA LATFbits.LATF1
unsigned char ID1 = 1; // ���****************************************************
unsigned char ID2 = 1;//��λ
unsigned int freq[2]; 		  
unsigned int temp[2];//�洢���ɼ�ͨ�����¶�ADֵ
unsigned int BAT;
unsigned int Read_Enable; // 1->ʰƵ��ɣ����Զ�ȡ  0->��ֹ��ȡ
unsigned int Tick; // ������¼��ʱ�жϴ���
unsigned int Timetick;//1->0.1S
unsigned int timeflag;
//unsigned int Time_long;
unsigned char Send_Enable; // ����ָʾ�Ƿ���Է������ݣ��ϵ翪ʼʱ���������ݣ��յ�ZigBeeоƬ���͵�'n'�ַ����������
int clock=0;
int halt_clock=1;//���Ʋɼ�ʱ����1-1min
int highest_freq_flag=1;//Ĭ�ϲɼ�ʱ������־λ 1-�������ɼ�Ƶ��T=12s����,0-�������õĲɼ��������
int Tran_Enable=0;//ͨ��GPRS���ʹ洢���ڵ����� 1-ʹ�� 0-��ֹ
char MG323_open_flag=1;//MG323������־λ 1-����ʧ�� 0-�ɹ�
char MG323_AT_flag=1;//ATӦ��ָ���־λ	1-Ӧ��ʧ��
char MG323_ATE0_flag=1;//�رջ��Ա�־λ	1-���Թر�ʧ��
char MG323_gprs_flag=1;//�Ƿ�ע�ᵽ�й��ƶ������־λ 1-ע��ʧ�ܣ�0-�ɹ�
char MG323_IOMODE_flag=1;
char MG323_conType_flag=1;
char MG323_apn_flag=1;
char MG323_conId_flag=1;
char MG323_srvType_flag=1;
char MG323_address_flag=1;	
char MG323_TCP_flag =1;//TCP���ӱ�־λ 1-����ʧ�� 0-�ɹ�
char Send_OK_flag=1;//36λ�����Ƿ��ͳɹ� 1-����ʧ�� 0-�ɹ�
//char send_36_flag = 0;//1-����36������	
unsigned int curr_addr[2];//�洢��ַ
unsigned int read_count_GPRS=20;//�Ӵ洢���ж�ȡread_count_GPRS������,��ͨ��GPRS���͸���λ��
unsigned int read_count_COM=0;//�Ӵ洢���ж�ȡ������
unsigned int Acq_Save_count=90; //����TCP�Ĺ����У����Ʋɼ�-�洢ʱ������(ԼΪ30min)
unsigned int COPS_count=0;//

unsigned char recv_17[20],recv[100],test[50],test1[100];
unsigned char send_data[36];
unsigned char send_AT[4]={'A','T',0x0d,0x0a};
unsigned char send_ATE0[6]={'A','T','E','0',0x0d,0x0a};
unsigned char send_COPS[10]={'A','T','+','C','O','P','S','?',0x0d,0x0a};
//unsigned char send_IOMODE[14] = {'A','T','^','I','O','M','O','D','E','=','0',',','0',0x0d};
unsigned char send_IOMODE[14] = {'A','T','^','I','O','M','O','D','E','=','0',',','1',0x0d};//��ʹ�ý��ջ���

unsigned char send_SICS_conType[25]={'A','T','^','S','I','C','S','=','0',',','c','o','n','T','y','p','e',',','G','P','R','S','0',0x0d,0x0a};
unsigned char send_SICS_apn[21]={'a','t','^','s','i','c','s','=','0',',','a','p','n',',','c','m','n','e','t',0x0d,0x0a};
unsigned char send_SISS_conId[19]={'A','T','^','S','I','S','S','=','0',',','c','o','n','I','d',',','0',0x0d,0x0a};
unsigned char send_SISS_srvType[26]={'A','T','^','S','I','S','S','=','0',',','s','r','v','T','y','p','e',',','S','o','c','k','e','t',0x0d,0x0a};
//3000
unsigned char send_SISS_address[59]={'A','T','^','S','I','S','S','=','0',',','a','d','d','r','e','s','s',',',0x22,'s','o','c','k','t','c','p',':','/','/','b','i','t','-','s','m','c','-','g','p','r','s','2','.','x','i','c','p','.','n','e','t',':','3','0','0','0',0x22,0x0d,0x0a};

/*//3000
unsigned char send_SISS_address[58]={'A','T','^','S','I','S','S','=','0',',','a','d','d','r','e','s','s',',',0x22,'s','o','c','k','t','c','p',':','/','/','b','i','t','-','s','m','c','-','g','p','r','s','.','g','i','c','p','.','n','e','t',':','3','0','0','0',0x22,0x0d,0x0a};
//4000
//unsigned char send_SISS_address[58]={'A','T','^','S','I','S','S','=','0',',','a','d','d','r','e','s','s',',',0x22,'s','o','c','k','t','c','p',':','/','/','b','i','t','-','s','m','c','-','g','p','r','s','.','g','i','c','p','.','n','e','t',':','4','0','0','0',0x22,0x0d,0x0a};
//unsigned char send_SISS_address[52]={'A','T','^','S','I','S','S','=','0',',','a','d','d','r','e','s','s',',',0x22,'s','o','c','k','t','c','p',':','/','/','c','s','j','e','d','i','.','o','i','c','p','.','n','e','t',':','4','0','0','0',0x22,0x0d,0x0a};

*/unsigned char send_SISO[11]={'A','T','^','S','I','S','O','=','0',0x0d,0x0a};
unsigned char send_SICI[10]={'A','T','^','S','I','C','I','?',0x0d,0x0a};
unsigned char send_SISW[14]={'A','T','^','S','I','S','W','=','0',',','3','6',0x0d,0x0a};
//unsigned char send_SISR[14]={'A','T','^','S','I','S','R','=','0',',','1','7',0x0d,0x0a};
unsigned char send_SISC[11]={'A','T','^','S','I','S','O','=','0',0x0d,0x0a};

unsigned char send_OK[10]={'M','G','3','2','3',' ','O','K',0x0d,0x0a};
unsigned char send_ERROR[13]={'M','G','3','2','3',' ','E','R','R','O','R',0x0d,0x0a};
unsigned char send_FAIL[12]={'M','G','3','2','3',' ','F','A','I','L',0x0d,0x0a};
unsigned char send_RESTART[18]={'P','L','E','A','S','E',' ','R','E','S','T','A','R','T',' ','!',0x0d,0x0a};

void DELAY(unsigned int t)
{
	unsigned int i,j;	
  	for(i=0;i<50;i++)
  	{
	  	for(j=0;j<t;j++)
   		{asm("nop");}
   } 	
 }//600~9ms
void Delay(unsigned int t) //t=10->1s
{
	StartTimer6();
	IEC2bits.T6IE = 1;
	Timetick = t;
	while(timeflag){};
	timeflag = 1;
	IEC2bits.T6IE = 0;	
	StopTimer6();
}
int time2bcd(int x)//ʱ����Ϣ��10����->BCD��
{
	int bcd;
	if((x>=0)&&(x<10))
	{
		bcd=x;
	}
	else if((x>=10)&&(x<20))
	  {
		bcd=x+6;
	   }
	else if ((x>=20)&&(x<30))
		 {
			 bcd=x+12;
		 }
	else if ((x>=30)&&(x<40))
		 {
			 bcd=x+18;
		 }
	else if ((x>=40)&&(x<50))
		 {
			 bcd=x+24;
		 }
	else if ((x>=50)&&(x<60))
		 {
			 bcd=x+30;
		 }
	return bcd;
	
}//
void __attribute__((interrupt,no_auto_psv)) _T6Interrupt(void)  // 1ms interrupt
{
	IFS2bits.T6IF = 0;	
	if(Tick<(Timetick*100))// Timetick=10,��1s
	{	
		Tick++;
	}
	else 
	{	
		Tick = 0;
		timeflag=0;
	}
		
}//
void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
	unsigned char data1[100];
	int nian,yue,ri,shi,fen,miao;
//	unsigned int UART_Timeout = 0;
//	unsigned int Break = 1;
//	unsigned char i = 0;
	unsigned char j = 0;
	unsigned char dummy;
	
	//while(0==(U1STA&0x0001))
	data1[0] = U1RXREG;
	asm("nop");	
	
	while((U1STA&0x0002)==0x0002)
	{
		U1STAbits.OERR = 0;
		dummy = U1RXREG;
	}

	j=UART1_Recv_Safe_Without_OK();
	asm("nop");

	if( recv[2]=='S' && recv[3]=='R' && recv[4]==':' )//��ʹ�ý��ջ���ʱ�� ^SISR �����ϱ�����
	{
		asm("nop");	
		if(recv[14]=='S'&&recv[30]=='E')//����17λ��������
		{
			if(recv[15]=='A'&&recv[16]=='S'&&recv[17]=='W')
			{
				//���ݷ��ͳɹ�����λ��������Ӧ��ָ���ʾ���ͳɹ�
				Send_OK_flag=0;
			}
			if(recv[15]=='R'&&recv[16]=='S'&&recv[17]=='T')
			{
				//������λ���������������MG323��λ
				MG323_RESTART();
				MG323_TCP_Open();
			}
			if(recv[15]=='O'&&recv[16]=='F')//��ȡ�洢���е�����
			{
				switch(recv[17])
				{
					case '1':  
								read_count_GPRS=50;
								break; 
					case '2':  
								read_count_GPRS=100;
								break;								
					case '3': 
								read_count_GPRS=150;
								break; 
					case '4':  
								read_count_GPRS=200;
								break;								
					case '5': 
					 			read_count_GPRS=250;
								break; 
					case '6':  
								read_count_GPRS=300;
								break;			
					case '7':  
								read_count_GPRS=350;
								break; 
					case '8':  
								read_count_GPRS=400;
								break;			
					case 'F':  
								read_count_GPRS=20;
								break; 					
					break;		
											
				}
				read_count_COM=0;//ͨ��GPRS������ʱ����ֹCOM�����ݡ�
				Tran_Enable=1;
				asm("nop");
			}
			if(recv[15]!='A'&&recv[15]!='R'&&recv[15]!='O'&&recv[17]!=0x39)	//���ò������			
			{
				clock=0;
				if(recv[17]=='f')
				{
					halt_clock = (recv[15]-48)*10+recv[16]-48;
					if(halt_clock==0)
					{
						highest_freq_flag=1;
					}
					else
					{
						highest_freq_flag=0;
					}
				}				
				asm("nop");
			}		
		
			if( (recv[18]!=0x39)&&(recv[19]!=0x39) ) 	//У׼ʵʱʱ�� 
			{
				nian = (int)( (recv[18]-48)*10+(recv[19]-48) );
				yue = (int)( (recv[20]-48)*10+(recv[21]-48) );
				ri = (int)( (recv[22]-48)*10+(recv[23]-48) );
				shi = (int)( (recv[24]-48)*10+(recv[25]-48) );
				fen = (int)( (recv[26]-48)*10+(recv[27]-48) );
				miao = (int)( (recv[28]-48)*10+(recv[29]-48) );	
				
				ds1302_write_time(6,(unsigned char)time2bcd(nian));
				ds1302_write_time(4,(unsigned char)time2bcd(yue));
				ds1302_write_time(3,(unsigned char)time2bcd(ri));
				ds1302_write_time(2,(unsigned char)time2bcd(shi));
				ds1302_write_time(1,(unsigned char)time2bcd(fen));
				ds1302_write_time(0,(unsigned char)time2bcd(miao));
				asm("nop");
			}				
	
		}
		
	/*	if(recv[12]==0x22&&recv[13]=='R'&&recv[14]=='T'&&recv[15]==0x22)
		{
			//������λ���������������MG323��λ
			MG323_RESTART();
			MG323_TCP_Open();
		}
	*/	
	}	
	if(  recv[2]=='S' && recv[3]=='W' &&  recv[8]=='3' && recv[9]=='6' )//��������񻺳���д��36����
	{
		//send_36_flag = 1;
		UART2_Send(send_data,36);
		UART1_Send(send_data,36);
		U1TXREG = 0x0d;
		while((U1STA&0x0100)==0x0000){asm("nop");}
	}
	if( recv[2]=='O' && recv[3]=='R' )//��������񻺳���д������
	{
		MG323_RESTART();
		MG323_TCP_Open();
	}
	if(  (recv[2]=='S'&&recv[3]==':'&&recv[7]=='0' )  )//^SIS �����ϱ�����
	{
		asm("nop");
		MG323_RESTART();
		MG323_TCP_Open();
	}
	if( recv[2]=='S' && recv[3]=='W' )//^SISW�����ϱ�����
	{
		if( recv[8]=='1')//����׼���ý����µ��û�����
		{			
			if(Tran_Enable)
			{
			//	Tran_Data_U1_GPRS_count++;	
			}
		}
	}
	
	
	IFS0bits.U1RXIF = 0;
//	return;	
}//	
void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void)//���ڽ���С���ַ����Ŀ���ָ��
{
	unsigned char data[8];
	unsigned int UART_Timeout = 0;
	unsigned char i = 0;
	unsigned int num;
	data[0] = U2RXREG;
			 
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

	if(i==7)
		{		
			if(data[1]!=0xff) 	//У׼ʵʱʱ�� 
			{
				ds1302_write_time(6,data[1]);
				ds1302_write_time(4,data[2]);
				ds1302_write_time(3,data[3]);
				ds1302_write_time(2,data[4]);
				ds1302_write_time(1,data[5]);
				ds1302_write_time(0,data[6]);
			}
			if(data[1]==0xff)//��ȡ�洢���е�����
			{
				if(data[2]==0xff)
				{
					num = (unsigned int) data[3];
					num = (num<<8) + (unsigned int)data[4];
					if(num>0 && num<3841)
					{
						read_count_COM = num;
					}
				}
				if(data[2]==0x00)
				{
					read_count_COM = 3840;
				}
					
				read_count_GPRS=0;//ͨ��GPRS������ʱ����ֹCOM�����ݡ�
				Tran_Enable=1;
				asm("nop");
			}
			
		}//if(i==7)	
	
	IFS1bits.U2RXIF = 0;
	return;	
}//

int main()
{
	int i,k,j,n;
	unsigned char dummy;
	unsigned int count=0;
	unsigned char year,month,day,hour,minute,second,minute_m;
//	int year1,month1,day1,hour1,minute1,second1;

	minute_m=0;
	
	OSCCON = 0x2200;	
 	timeflag=1;	//Delay()��־λ
	Tick = 0; 	//T6 �жϴ�������ʱ1ms�ж�	
//	Time_long = 30;//Ĭ�ϲɼ�ʱ����3s
		
	
	TRISCbits.TRISC1 = 1;//BAT
	
	TRISGbits.TRISG13 = 0;//ʱ��
	TRISGbits.TRISG14 = 0;			
	
	TRISGbits.TRISG0 = 0;//MG323
	TRISGbits.TRISG1 = 0;	

	send_data[0]='S';
	send_data[1]=ID1+0x30;
	send_data[2]=ID2+0x30;
	for(j=3;j<=34;j++)
	{send_data[j]=0x38;}
	send_data[35]='E';
	
	InitTimer6();  // Timer6 �ṩ1ms�ж϶�ʱ	
	InitTimer2();  // Timer2 �ṩ���벶׽ʱ�ӻ�׼
	InitIC();
	InitADC();
	InitSPI();
	ds1302_init();
	InitSCI();
	
	while((U1STA&0x0001)==0x0001){dummy = U1RXREG;}	
	
	LATGbits.LATG0=0;	//�رո�λ
	LATGbits.LATG1 =1;	//MG323����
	asm("nop");
	Delay(12);	
	LATGbits.LATG1 =0;
	asm("nop");
	
	while((U1STA&0x0002)==0x0002)
	{
		U1STAbits.OERR = 0;
		dummy = U1RXREG;
	}
/*
while(1)
{	
	for(i=0;i<36;i++)
	{
		U2TXREG = send_data[i];
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	asm("nop");
	DELAY(60000);	
}
*/
	while(MG323_open_flag)
	{
		j=UART1_Recv_Without_OK();//^SYSSTART<CR><LF>
		asm("nop");
		
		if(j==11)
		{
			MG323_open_flag=0;//open success
		}
		else
		{
			MG323_open_flag=1;//open fail
			LATGbits.LATG0=1;//��MG323��λ
			Delay(1);//100ms
			LATGbits.LATG0=0;
		}

	}	
	MG323_open_flag=1;
	
	while(MG323_AT_flag)
	{	
		UART1_Send(send_AT,4);//AT		
		j=UART1_Recv_With_OK();
		asm("nop");
		if(j==7 && recv[j-1]==0x0a && recv[j-3]=='K')
		{
			MG323_AT_flag=0;//success
		}else
		{
			//���i��Ӧ��ʧ�ܣ�������MG323
		}
		asm("nop");
	}
	MG323_AT_flag=1;//fail	
	
	while(MG323_ATE0_flag)
	{	
		UART1_Send(send_ATE0,5);//ATE0 //�رջ���		
		j=UART1_Recv_With_OK();
		asm("nop");
		if(j==9 && recv[j-1]==0x0a && recv[j-3]=='K')
		{
			MG323_ATE0_flag=0;//success
		}else
		{
			//�ٴη���ATE0�����i��Ӧ��ʧ�ܣ�������MG323
		}
		asm("nop");
	}
	MG323_ATE0_flag=1;

	while(MG323_gprs_flag)//��ѯ�Ƿ�ע�ᵽ�й��ƶ�����δע��ɹ�������ע��
	{
		UART2_Send(send_COPS,8);
		UART1_Send(send_COPS,9);//AT+COPS?[8] 
		
		j=UART1_Recv_GPRS();
		asm("nop");
		
		if( j==33 && (recv[13]=='H')&&(recv[18]=='M')&&(recv[19]=='O')&&(recv[22]=='L') )
		{				
			UART2_Send(send_COPS,8);
			UART1_Send(send_COPS,9);//AT+COPS?[8] 
			j=UART1_Recv_With_OK();
			asm("nop");
			MG323_gprs_flag = 0;//success	
			COPS_count=0;
		}
		else
		{
			MG323_gprs_flag = 1;//δע�ᵽ�й��ƶ�����
			Delay(15);
			COPS_count++;
			//����ظ�ע����15�Σ�������MG323ģ��
			if(COPS_count>=15)
			{
				COPS_count=0;
				LATGbits.LATG0=1;//��MG323��λ
				Delay(1);//100ms
				LATGbits.LATG0=0;
				
				
				while(MG323_open_flag)
				{
					j=UART1_Recv_CR_LF();
					j=UART1_Recv_CR_LF();//��ȡ������Ϣ "SYSSTART"
					
			//		j=UART1_Recv_Without_OK();//^SYSSTART<CR><LF>
					asm("nop");		
					if(j==9)
					{
						MG323_open_flag=0;//open success
					}
					else
					{
						MG323_open_flag=1;//open fail
						LATGbits.LATG0=1;//��MG323��λ
						Delay(1);//100ms
						LATGbits.LATG0=0;
					}
			
				}	
				MG323_open_flag=1;
					
				while(MG323_AT_flag)
				{	
					UART1_Send(send_AT,4);//AT	
					
					j=UART1_Recv_With_OK();
					asm("nop");
					if(j==7 && recv[j-1]==0x0a && recv[j-3]=='K')
					{
						MG323_AT_flag=0;//success
					}else
					{
						//���i��Ӧ��ʧ�ܣ�������MG323
					}
					asm("nop");
				}
				MG323_AT_flag=1;//fail	
				
				while(MG323_ATE0_flag)
				{	
					UART1_Send(send_ATE0,5);//ATE0 //�رջ���		
					j=UART1_Recv_With_OK();
					asm("nop");
					if(j==9 && recv[j-1]==0x0a && recv[j-3]=='K')
					{
						MG323_ATE0_flag=0;//success
					}else
					{
						//�ٴη���ATE0�����i��Ӧ��ʧ�ܣ�������MG323
					}
					asm("nop");
				}
				MG323_ATE0_flag=1;

			}//
		}
	}
	MG323_gprs_flag = 1;//δע�ᵽ�й��ƶ�����

	while(MG323_IOMODE_flag)
	{
		UART2_Send(send_IOMODE,13);//�������ݸ�ʽ
		UART1_Send(send_IOMODE,14);//AT^IOMODE[13] 
		j=UART1_Recv_With_OK();
		if(j==4)
		{		
			MG323_IOMODE_flag=0;//success
		}else
		{
			//�ٴη��ͣ����i��ʧ�ܣ�������MG323
		}	
	}	
	MG323_IOMODE_flag=1;	
	
	while(MG323_conType_flag)
	{
		UART2_Send(send_SICS_conType,23);//ѡ�� 0 ͨ�� Internet��������Ϊ GPRS 
		UART1_Send(send_SICS_conType,24);//at^sics=0,conType,GPRS0[23] 
		j=UART1_Recv_With_OK();
		if(j==6)
		{		
			MG323_conType_flag=0;//success
		}else
		{
			//�ٴη��ͣ����i��ʧ�ܣ�������MG323
		}	
	}	
	MG323_conType_flag=1;	

	while(MG323_apn_flag)
	{
		UART2_Send(send_SICS_apn,19);//���� 0 ͨ�� APN ΪCMNET 
		UART1_Send(send_SICS_apn,20);//at^sics=0,apn,cmnet[19]
		j=UART1_Recv_With_OK();
		if(j==6)
		{
			MG323_apn_flag=0;//���óɹ�	MG323_apn_flag
		}else
		{
			//�ٴη��ͣ����i��ʧ�ܣ�������MG323
		}
	}
	MG323_apn_flag=1;
	
	while(MG323_conId_flag)
	{
		UART2_Send(send_SISS_conId,17);//���� 0 ͨ��  conId Ϊ 0 
		UART1_Send(send_SISS_conId,18);//at^siss=0,conId,0[17]
		j=UART1_Recv_With_OK();
		if(j==6)
		{
			MG323_conId_flag=0;//���óɹ�	MG323_conId_flag
		}else
		{
			//�ٴη��ͣ����i��ʧ�ܣ�������MG323
		}
	}	
	MG323_conId_flag=1;
		
	while(MG323_srvType_flag)
	{
		UART2_Send(send_SISS_srvType,24);//���� 0 ͨ����������
		UART1_Send(send_SISS_srvType,25);//at^siss=0,srvType,socket[24]
		j=UART1_Recv_With_OK();
		if(j==6)
		{
			MG323_srvType_flag=0;//���óɹ�	MG323_srvType_flag
		}else
		{
			//�ٴη��ͣ����i��ʧ�ܣ�������MG323
		}
	}	
	MG323_srvType_flag=1;
		
	while(MG323_address_flag)
	{
		UART2_Send(send_SISS_address,57);//�������ӷ������� IP�Ͷ˿ں�
		UART1_Send(send_SISS_address,58);//at^siss=0,address,"socktcp://bit-smc-gprs.gicp.net:4000"[56]
		j=UART1_Recv_With_OK();
		if(j==6)
		{
			MG323_address_flag=0;//���óɹ�	MG323_address_flag
		}else
		{
			//�ٴη��ͣ����i��ʧ�ܣ�������MG323 
		}
	}
	MG323_address_flag=1;
	
	MG323_TCP_Open();
	Delay(30);
	
	while((U1STA&0x0002)==0x0002)
	{
		U1STAbits.OERR = 0;
		dummy = U1RXREG;
	}
	
	UART2_Send(send_SICI,8);
	UART1_Send(send_SICI,9);//��ѯ���䵽��IP
	j=UART1_Recv_With_OK();
	asm("nop");	
	if( recv[9]=='0' )
	{
		//˵��internet�Ѿ����嵫û������
		//Ӧ�����½���TCP���� MG323_TCP_flag
	}
	
	if( recv[9]=='2' )
	{
		//˵��internet�Ѿ����ӳɹ�
	}
	

	UART2_Send(send_SISW,12);
	UART1_Send(send_SISW,13);//AT^SISW=0,36 //�����Ļ�����д��36�ֽ�����//д��36������send_data[36]��س���
	j=UART1_Recv_Without_OK();	//����" ^SISW: 0,36,36 " �ĳ��� j
								//���һ���ֽ�ֵ36-�Ѿ����͵�TCP��δȷ�ϵ��ֽ������п��ܲ���36������36�ı���
	asm("nop");		
	UART2_Send(send_data,36);
	UART1_Send(send_data,36);	
	asm("nop");	
	for(j=0;j<20;j++) //<CR><LF>OK<CR><LF><CR><LF>^SISW: 0,1<CR><LF>
	{
		while(0==(U1STA&0x0001)){}
		recv[j] = U1RXREG;
		U2TXREG=recv[j];
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	if( recv[2]=='O' && recv[3]=='K' &&recv[17]=='1' )
	{
		//˵�����ͳɹ�,���Լ�������
	}		
	asm("nop");

	TRISFbits.TRISF1 = 0;//CEA//��Ƶ����	
	TRISDbits.TRISD8 = 1;//PH1//ʰƵ���
	TRISDbits.TRISD10 = 1;//PH2
	TRISBbits.TRISB9 = 1;//TMPOP1//AD�����
	TRISBbits.TRISB1 = 1;//TMPOP2

	curr_addr[0] = 0;
	curr_addr[1] = 0;	//һ��ʼ��FRAM�д�Ųɼ����ݴӵ�ַ 0x00000 ��ʼ
	
	IFS0bits.U1RXIF = 0;
	IEC0bits.U1RXIE = 1; 	// enable UART1 RX interrupt
	while(1)
	{
		if(Tran_Enable)
		{
			if(read_count_GPRS!=0)
			{
				Tran_Data_U1_GPRS();
			}
			if(read_count_COM!=0)
			{
				Tran_Data_U2_COM();
			}
			
			Tran_Enable=0;
		}	
		else
		{	
			year = ds1302_read_time(6);//��ȡʵʱʱ�䣬ʱ�����ݸ�ʽ ASC��,ע���ӳ���DS1302.c  L97��������������𣡣�����
			month = ds1302_read_time(4);
			day = ds1302_read_time(3);
			hour = ds1302_read_time(2);	
			minute = ds1302_read_time(1);
			second = ds1302_read_time(0);
		
			if(highest_freq_flag==1)
			{
							
				i = 2000;
				while(i>=50)
				{
					for(k=0;k<3;k++)
					{
	            		j = i;            		
	            		CEA = 1;//RG1���Ϊ��            	
	            		while(j)
	               		{    
		               		asm("nop");
	                    	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
							j--;
	                	}             
	 
	            		j = i;              
	            		CEA = 0;           		
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
				IEC2bits.T6IE = 1;
				StartTimer2();
				StartIC();
				DELAY(2000);//��֤��׽��ɣ����Ϊ19ms
				freq[0] = GetPeriod(1);
				freq[1] = GetPeriod(2);
				StopIC();
				StopTimer6();	
				
				AD1CHS0bits.CH0SA = 16;
				AD1CON1bits.ADON = 1;
				asm("nop");asm("nop");asm("nop");asm("nop");
				AD1CON1bits.SAMP = 1;
				while(!AD1CON1bits.DONE){};
				BAT = ADC1BUF0;	
				AD1CON1bits.ADON = 0;					
				
		/*��2��ͨ���Ĵ������������践�صĵ�ѹ������ADת������������temp[]�� */		
				AD1CHS0bits.CH0SA = 9; //AN9 == TMPOP1
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
		
				AD1CHS0bits.CH0SA = 1;//AN1 == TMPOP2
				AD1CON1bits.ADON = 1;
				asm("nop");asm("nop");asm("nop");asm("nop");
				AD1CON1bits.SAMP = 1;
				while(!AD1CON1bits.DONE){};
				temp[1] = ADC1BUF0;	
				AD1CON1bits.ADON = 0;
			
				year = ds1302_read_time(6);//��ȡʵʱʱ�䣬ʱ�����ݸ�ʽ ASC��,ע���ӳ���DS1302.c  L97��������������𣡣�����
				month = ds1302_read_time(4);
				day = ds1302_read_time(3);
				hour = ds1302_read_time(2);	
				minute = ds1302_read_time(1);
				second = ds1302_read_time(0);	
		
				send_data[0]='S';
				send_data[1]=ID1+0x30;
				send_data[2]=ID2+0x30;
				
				send_data[3] = year/10+0x30;
				send_data[4] = year%10+0x30;
				send_data[5] = month/10+0x30;
				send_data[6] = month%10+0x30;
				send_data[7] = day/10+0x30;
				send_data[8] = day%10+0x30;
				send_data[9] = hour/10+0x30;
				send_data[10] = hour%10+0x30;
				send_data[11] = minute/10+0x30;
				send_data[12] = minute%10+0x30;
				send_data[13] = second/10+0x30;
				send_data[14] = second%10+0x30;			
	
				send_data[15] = freq[0]/1000+0x30;
				send_data[16] = (freq[0]%1000)/100+0x30;
				send_data[17] = (freq[0]%100)/10+0x30;
				send_data[18] = freq[0]%10+0x30;			
				send_data[19] = temp[0]/1000+0x30;
				send_data[20] = (temp[0]%1000)/100+0x30;
				send_data[21] = (temp[0]%100)/10+0x30;
				send_data[22] = temp[0]%10+0x30;
				
				send_data[23] = freq[1]/1000+0x30;
				send_data[24] = (freq[1]%1000)/100+0x30;
				send_data[25] = (freq[1]%100)/10+0x30;
				send_data[26] = freq[1]%10+0x30;			
				send_data[27] = temp[1]/1000+0x30;
				send_data[28] = (temp[1]%1000)/100+0x30;
				send_data[29] = (temp[1]%100)/10+0x30;
				send_data[30] = temp[1]%10+0x30;
				
				send_data[31] = BAT/1000+0x30;
				send_data[32] = (BAT%1000)/100+0x30;
				send_data[33] = (BAT%100)/10+0x30;
				send_data[34] = BAT%10+0x30;
				
				send_data[35]='E';
	
				while((U1STA&0x0002)==0x0002)
				{
					U1STAbits.OERR = 0;
					dummy = U1RXREG;
				}
				UART2_Send(send_SISW,12);
				UART1_Send(send_SISW,13);//AT^SISW=0,36 //��������//д��36������send_data[36]��س�
				while(Send_OK_flag)
				{
					DELAY(60000);
					DELAY(60000);
					DELAY(60000);
					count++;
					if(count==3)//2*2.7s��û�н��յ���λ����Ӧ��ָ�����Ϊ���綪ʧ�������ݴ洢��FM25V10��
					{
						Send_OK_flag=0;
						count=0;
						
						for(n=0;n<36;n++)//���洢3640������
						{
							WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),send_data[n]);
							inc_addr();	
						}
					}
					
					
				}
				Send_OK_flag=1;
				
				DELAY(60000);
				DELAY(60000);
				DELAY(60000);
			}//if(highest_freq_flag==1)
			else
			{
				if(minute != minute_m)//ͨ���Ƚ�����ʵʱʱ�ӵġ����ӡ�������������β�ͬ����Ϊ��ʱ1min
				{
					minute_m = minute;
					clock++;		
				
					if(clock>=halt_clock)
				//	if(clock>=1)	//�ɼ�ʱ����Ϊ1����
					{						
						clock=0;
						i = 2000;
						while(i>=50)
						{
							for(k=0;k<3;k++)
							{
			            		j = i;            		
			            		CEA = 1;//RG1���Ϊ��            	
			            		while(j)
			               		{    
				               		asm("nop");
			                    	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
									j--;
			                	}             
			 
			            		j = i;              
			            		CEA = 0;           		
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
						IEC2bits.T6IE = 1;
						StartTimer2();
						StartIC();
						DELAY(2000);//��֤��׽��ɣ����Ϊ19ms
						freq[0] = GetPeriod(1);
						freq[1] = GetPeriod(2);
						StopIC();
						StopTimer6();	
						
						AD1CHS0bits.CH0SA = 16;
						AD1CON1bits.ADON = 1;
						asm("nop");asm("nop");asm("nop");asm("nop");
						AD1CON1bits.SAMP = 1;
						while(!AD1CON1bits.DONE){};
						BAT = ADC1BUF0;	
						AD1CON1bits.ADON = 0;					
					
			/*��2��ͨ���Ĵ������������践�صĵ�ѹ������ADת������������temp[]�� */		
						AD1CHS0bits.CH0SA = 9;
						AD1CON1bits.ADON = 1; 
						asm("nop");asm("nop");asm("nop");asm("nop");
						AD1CON1bits.SAMP = 1; 
						while(!AD1CON1bits.DONE){}; 		
						temp[0] = ADC1BUF0;	
						AD1CON1bits.ADON = 0;		
				
						AD1CHS0bits.CH0SA = 1;//AN1 == TMPOP2
						AD1CON1bits.ADON = 1;
						asm("nop");asm("nop");asm("nop");asm("nop");
						AD1CON1bits.SAMP = 1;
						while(!AD1CON1bits.DONE){};
						temp[1] = ADC1BUF0;	
						AD1CON1bits.ADON = 0;
				
						year = ds1302_read_time(6);//��ȡʵʱʱ�䣬ʱ�����ݸ�ʽ ASC��,ע���ӳ���DS1302.c  L97��������������𣡣�����
						month = ds1302_read_time(4);
						day = ds1302_read_time(3);
						hour = ds1302_read_time(2);	
						minute = ds1302_read_time(1);
						second = ds1302_read_time(0);	
				
						send_data[0]='S';
						send_data[1]=ID1+0x30;
						send_data[2]=ID2+0x30;	
	
						send_data[3] = year/10+0x30;
						send_data[4] = year%10+0x30;
						send_data[5] = month/10+0x30;
						send_data[6] = month%10+0x30;
						send_data[7] = day/10+0x30;
						send_data[8] = day%10+0x30;
						send_data[9] = hour/10+0x30;
						send_data[10] = hour%10+0x30;
						send_data[11] = minute/10+0x30;
						send_data[12] = minute%10+0x30;
						send_data[13] = second/10+0x30;
						send_data[14] = second%10+0x30;			
			
						send_data[15] = freq[0]/1000+0x30;
						send_data[16] = (freq[0]%1000)/100+0x30;
						send_data[17] = (freq[0]%100)/10+0x30;
						send_data[18] = freq[0]%10+0x30;			
						send_data[19] = temp[0]/1000+0x30;
						send_data[20] = (temp[0]%1000)/100+0x30;
						send_data[21] = (temp[0]%100)/10+0x30;
						send_data[22] = temp[0]%10+0x30;
						
						send_data[23] = freq[1]/1000+0x30;
						send_data[24] = (freq[1]%1000)/100+0x30;
						send_data[25] = (freq[1]%100)/10+0x30;
						send_data[26] = freq[1]%10+0x30;			
						send_data[27] = temp[1]/1000+0x30;
						send_data[28] = (temp[1]%1000)/100+0x30;
						send_data[29] = (temp[1]%100)/10+0x30;
						send_data[30] = temp[1]%10+0x30;
						
						send_data[31] = BAT/1000+0x30;
						send_data[32] = (BAT%1000)/100+0x30;
						send_data[33] = (BAT%100)/10+0x30;
						send_data[34] = BAT%10+0x30;
			
						send_data[35]='E';
						
						while((U1STA&0x0002)==0x0002)
						{
							U1STAbits.OERR = 0;
							dummy = U1RXREG;
						}
								
						UART2_Send(send_SISW,12);
						UART1_Send(send_SISW,13);//AT^SISW=0,36 //��������//д��36������send_data[36]��س���
						while(Send_OK_flag)
						{
							DELAY(60000);
							DELAY(60000);
							DELAY(60000);
							count++;
							if(count>=2)//2*2.7s��û�н��յ���λ����Ӧ��ָ�����Ϊ���綪ʧ�������ݴ洢��FM25V10��
							{
								Send_OK_flag=0;
								count=0;
								
								for(n=0;n<36;n++)//���洢3640������
								{
									WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),send_data[n]);
									inc_addr();	
								}
							}
							
							
						}
						Send_OK_flag=1;
				

					}
					
					
				}//if(minute != minute_m)
			}
			asm("NOP");	
		}	
	
	}//while(1)	
	return 0;
}//main()

void inc_addr(void)
{
	if(curr_addr[1]==0xffdf)
	{
		if(curr_addr[0]==1)
		{
			curr_addr[0]=0;
			curr_addr[1]=0;
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

void Tran_Data_U1_GPRS(void) //
{
	unsigned char i;
//	unsigned int j;
	curr_addr[0] = 0;
	curr_addr[1] = 0; //ÿ�ζ�ȡFRAM�е����ݶ��ӵ�ַ 0x00000 ��ʼ

	send_data[0]='S';
	send_data[1]=ID1+48;
	send_data[2]=ID2+48;
	for(i=3;i<31;i++){send_data[i]='0';}
	for(i=31;i<35;i++){send_data[i]='1';}
	send_data[35]='E';
	UART2_Send(send_SISW,12);
	UART1_Send(send_SISW,13);//AT^SISW=0,36 //��������
	DELAY(60000);
	DELAY(60000);
	DELAY(60000);
	
	while(read_count_GPRS)
	{

		for(i=0;i<36;i++)
		{
			send_data[i] =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
			inc_addr();
		}
		DELAY(6000);
		UART2_Send(send_SISW,12);
		UART1_Send(send_SISW,13);//AT^SISW=0,36 //��������//д��36������send_data[36]��س�
		DELAY(60000);
		DELAY(60000);
		DELAY(60000);	
		read_count_GPRS--;
	}	

	send_data[0]='S';
	send_data[1]=ID1+48;
	send_data[2]=ID2+48;
	for(i=3;i<31;i++){send_data[i]='0';}
	for(i=31;i<35;i++){send_data[i]='9';}
	send_data[35]='E';
	UART2_Send(send_SISW,12);
	UART1_Send(send_SISW,13);//AT^SISW=0,36 //��������
	
	DELAY(60000);
	DELAY(60000);
	DELAY(60000);
}//
void Tran_Data_U2_COM(void) // 
{
	unsigned char i;
//	unsigned int j;
	curr_addr[0] = 0;
	curr_addr[1] = 0; //ÿ�ζ�ȡFRAM�е����ݶ��ӵ�ַ 0x00000 ��ʼ

	send_data[0]='S';
	send_data[1]=ID1+48;
	send_data[2]=ID2+48;
	for(i=3;i<31;i++){send_data[i]='0';}
	for(i=31;i<35;i++){send_data[i]='1';}
	send_data[35]='E';
	for(i=0;i<36;i++)
	{
		U2TXREG = send_data[i];
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	DELAY(6000);
	while(read_count_COM)
	{

		for(i=0;i<36;i++)
		{
			send_data[i] =  READ(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]&0x00ff));
			U2TXREG = send_data[i];
			while((U2STA&0x0100)==0x0000){asm("nop");}
			inc_addr();
		}
		DELAY(6000);
		
		read_count_COM--;
	}	

	send_data[0]='S';
	send_data[1]=ID1+48;
	send_data[2]=ID2+48;
	for(i=3;i<31;i++){send_data[i]='0';}
	for(i=31;i<35;i++){send_data[i]='9';}
	send_data[35]='E';
	for(i=0;i<36;i++)
	{
		U2TXREG = send_data[i];
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	
	U2TXREG = 0x0a;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
}//
void Acq_Save(void)
{
	int i,k,j,n;
	unsigned char month,day,hour,minute,second;
	
	i = 2000;
	while(i>=50)
	{
		for(k=0;k<3;k++)
		{
          		j = i;            		
          		CEA = 1;//RG1���Ϊ��            	
          		while(j)
             		{    
              		asm("nop");
                  	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
				j--;
              	}             

          		j = i;              
          		CEA = 0;           		
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
	IEC2bits.T6IE = 1;
	StartTimer2();
	StartIC();
	DELAY(2000);//��֤��׽��ɣ����Ϊ19ms
	freq[0] = GetPeriod(1);
	freq[1] = GetPeriod(2);
	StopIC();
	StopTimer6();	
	
	AD1CHS0bits.CH0SA = 16;
	AD1CON1bits.ADON = 1;
	asm("nop");asm("nop");asm("nop");asm("nop");
	AD1CON1bits.SAMP = 1;
	while(!AD1CON1bits.DONE){};
	BAT = ADC1BUF0;	
	AD1CON1bits.ADON = 0;					
	
/*��2��ͨ���Ĵ������������践�صĵ�ѹ������ADת������������temp[]�� */		
	AD1CHS0bits.CH0SA = 9; //AN9 == TMPOP1
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

	AD1CHS0bits.CH0SA = 1;//AN1 == TMPOP2
	AD1CON1bits.ADON = 1;
	asm("nop");asm("nop");asm("nop");asm("nop");
	AD1CON1bits.SAMP = 1;
	while(!AD1CON1bits.DONE){};
	temp[1] = ADC1BUF0;	
	AD1CON1bits.ADON = 0;

//	year = ds1302_read_time(6);//��ȡʵʱʱ�䣬ʱ�����ݸ�ʽ ASC��,ע���ӳ���DS1302.c  L97��������������𣡣�����
	month = ds1302_read_time(4);
	day = ds1302_read_time(3);
	hour = ds1302_read_time(2);	
	minute = ds1302_read_time(1);
	second = ds1302_read_time(0);	

	send_data[0]='S';
	send_data[1]=ID1+0x30;
	send_data[2]=ID2+0x30;
	
	send_data[3] = 0x38;
	send_data[4] = 0x37;
	send_data[5] = month/10+0x30;
	send_data[6] = month%10+0x30;
	send_data[7] = day/10+0x30;
	send_data[8] = day%10+0x30;
	send_data[9] = hour/10+0x30;
	send_data[10] = hour%10+0x30;
	send_data[11] = minute/10+0x30;
	send_data[12] = minute%10+0x30;
	send_data[13] = second/10+0x30;
	send_data[14] = second%10+0x30;			

	send_data[15] = freq[0]/1000+0x30;
	send_data[16] = (freq[0]%1000)/100+0x30;
	send_data[17] = (freq[0]%100)/10+0x30;
	send_data[18] = freq[0]%10+0x30;			
	send_data[19] = temp[0]/1000+0x30;
	send_data[20] = (temp[0]%1000)/100+0x30;
	send_data[21] = (temp[0]%100)/10+0x30;
	send_data[22] = temp[0]%10+0x30;
	
	send_data[23] = freq[1]/1000+0x30;
	send_data[24] = (freq[1]%1000)/100+0x30;
	send_data[25] = (freq[1]%100)/10+0x30;
	send_data[26] = freq[1]%10+0x30;			
	send_data[27] = temp[1]/1000+0x30;
	send_data[28] = (temp[1]%1000)/100+0x30;
	send_data[29] = (temp[1]%100)/10+0x30;
	send_data[30] = temp[1]%10+0x30;
	
	send_data[31] = BAT/1000+0x30;
	send_data[32] = (BAT%1000)/100+0x30;
	send_data[33] = (BAT%100)/10+0x30;
	send_data[34] = BAT%10+0x30;
	
	send_data[35]='E';
	
	for(n=0;n<36;n++)//���洢3640������
	{
		WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),send_data[n]);
		inc_addr();	
	}
						
}
void MG323_RESTART(void)
{
	int j;
	unsigned char dummy;
	//ÿ����40�Σ�ԼΪ30min��ʱ���������ݲɼ�-�洢���ܣ���ʱ�洢�����ݸ�ʽ�н� ������Ϊ87 ������ȡ����87�꼴��Ϊ��������ʱ�ɼ�������
	Acq_Save_count--;
	if(Acq_Save_count==0)
	{		
		Acq_Save_count=40;
		Acq_Save();
	}
	
	while((U1STA&0x0002)==0x0002)
	{
		U1STAbits.OERR = 0;
		dummy = U1RXREG;
	}

	LATGbits.LATG0=1;//��MG323��λ
	Delay(1);//100ms
	LATGbits.LATG0=0;
				
	while(MG323_open_flag)//���MG323û�п����ɹ����߲���Ӧ��ATָ����Ͻ��临λ
	{
		
		j=UART1_Recv_CR_LF();
		j=UART1_Recv_CR_LF();//��ȡ������Ϣ "SYSSTART"
		asm("nop");
		UART1_Send(send_AT,4);//AT	
		j=UART1_Recv_With_OK();
		asm("nop");
		
		if( (recv[j-4]=='O')&&(recv[j-3]=='K')&&(recv[j-1]==0x0a) )
		{
			UART2_Send(send_OK,8);//MG323�ѿ��������ܹ�Ӧ��ATָ��
			U2TXREG=0x0a;
			while((U2STA&0x0100)==0x0000){asm("nop");}
			MG323_open_flag=0;
		}
		else
		{
			LATGbits.LATG0=1;//��MG323��λ
			Delay(1);//100ms
			LATGbits.LATG0=0;	
		}
	
	}	
	MG323_open_flag=1;
	
	UART1_Send(send_ATE0,5);//ATE0 //�رջ���
	j=UART1_Recv_With_OK();
	
	while(MG323_gprs_flag)//��ѯ�Ƿ�ע�ᵽ�й��ƶ�����δע��ɹ�������ע��
	{
		UART2_Send(send_COPS,8);
		UART1_Send(send_COPS,9);//AT+COPS?[8] 					
		j=UART1_Recv_GPRS();
		asm("nop");
	
		if( j==33 && (recv[13]=='H')&&(recv[18]=='M')&&(recv[19]=='O')&&(recv[22]=='L') )
		{				
			UART2_Send(send_COPS,8);
			UART1_Send(send_COPS,9);//AT+COPS?[8] 
			j=UART1_Recv_With_OK();
			MG323_gprs_flag = 0;//success
			COPS_count=0;	
		}
		else
		{
			MG323_gprs_flag= 1;//δע�ᵽ�й��ƶ�����
			Delay(15);
			
			//����ظ�ע����20�Σ�������MG323ģ��
			COPS_count++;
			if(COPS_count>=20)
			{
				COPS_count=0;
				LATGbits.LATG0=1;//��MG323��λ
				Delay(1);//100ms
				LATGbits.LATG0=0;
				
				
				while(MG323_open_flag)
				{
					j=UART1_Recv_CR_LF();
					j=UART1_Recv_CR_LF();//��ȡ������Ϣ "SYSSTART"
					
			//		j=UART1_Recv_Without_OK();//^SYSSTART<CR><LF>
					asm("nop");		
					if(j==9)
					{
						MG323_open_flag=0;//open success
					}
					else
					{
						MG323_open_flag=1;//open fail
						LATGbits.LATG0=1;//��MG323��λ
						Delay(1);//100ms
						LATGbits.LATG0=0;
					}
			
				}	
				MG323_open_flag=1;
					
				while(MG323_AT_flag)
				{	
					UART1_Send(send_AT,4);//AT	
					
					j=UART1_Recv_With_OK();
					asm("nop");
					if(j==7 && recv[j-1]==0x0a && recv[j-3]=='K')
					{
						MG323_AT_flag=0;//success
					}else
					{
						//���i��Ӧ��ʧ�ܣ�������MG323
					}
					asm("nop");
				}
				MG323_AT_flag=1;//fail	
				
				while(MG323_ATE0_flag)
				{	
					UART1_Send(send_ATE0,5);//ATE0 //�رջ���		
					j=UART1_Recv_With_OK();
					asm("nop");
					if(j==9 && recv[j-1]==0x0a && recv[j-3]=='K')
					{
						MG323_ATE0_flag=0;//success
					}else
					{
						//�ٴη���ATE0�����i��Ӧ��ʧ�ܣ�������MG323
					}
					asm("nop");
				}
				MG323_ATE0_flag=1;

			}//
		}
	}
	MG323_gprs_flag = 1;//δע�ᵽ�й��ƶ�����
	
	asm("nop");
	UART2_Send(send_IOMODE,13);//�������ݸ�ʽ
	UART1_Send(send_IOMODE,14);//AT^IOMODE[13] 
	j=UART1_Recv_With_OK();
	
	UART2_Send(send_SICS_conType,23);//ѡ�� 0 ͨ�� Internet��������Ϊ GPRS 
	UART1_Send(send_SICS_conType,24);//at^sics=0,conType,GPRS0[23]
	j=UART1_Recv_With_OK();
	
	UART2_Send(send_SICS_apn,19);//���� 0 ͨ�� APN ΪCMNET 
	UART1_Send(send_SICS_apn,20);//at^sics=0,apn,cmnet[19]
	j=UART1_Recv_With_OK();
	//asm("nop");
				
	UART2_Send(send_SISS_conId,17);//���� 0 ͨ��  conId Ϊ 0 
	UART1_Send(send_SISS_conId,18);//at^siss=0,conId,0[17]
	j=UART1_Recv_With_OK();
	//asm("nop");
	
	UART2_Send(send_SISS_srvType,24);//���� 0 ͨ����������
	UART1_Send(send_SISS_srvType,25);//at^siss=0,srvType,socket[24]
	j=UART1_Recv_With_OK();
	//asm("nop");
	
	UART2_Send(send_SISS_address,57);//�������ӷ������� IP�Ͷ˿ں�
	UART1_Send(send_SISS_address,58);//at^siss=0,address,"socktcp://bit-smc-gprs.gicp.net:4000"[56]
	j=UART1_Recv_With_OK();
//	asm("nop")
}//
void MG323_TCP_Open(void)
{
	int j;
	
	while(MG323_TCP_flag)
	{
		
		UART2_Send(send_SISO,9);
		UART1_Send(send_SISO,10);//AT^SISO=0[9] //��TCP���� 			
		j=UART1_Recv_Without_OK();
	
		if( j==6 && recv[4]==0x0d && recv[2]=='O' && recv[3]=='K' )
		{
			MG323_TCP_flag = 0;//success
		}	
		else
		{
			MG323_RESTART();
		}
	}	
	MG323_TCP_flag=1;
}//
void MG323_CHECK_Internet(void)
{
	int j;
	int check=1;//1-���ӳ�ʧ�� 0-Internet�Ѿ����ӳɹ�
	unsigned char dummy;
	while(check)
	{
		while((U1STA&0x0002)==0x0002)
		{
			U1STAbits.OERR = 0;
			dummy = U1RXREG;
		}
		UART2_Send(send_SICI,8);
		UART1_Send(send_SICI,9);//��ѯinternet �Ƿ����ӳɹ�
		j=UART1_Recv_With_OK();
		asm("nop");	
		if( recv[9]=='0' )
		{
			asm("nop");	
			//˵��internet�Ѿ����嵫û������
			check=1;
			//Ӧ�����½���TCP���� MG323_TCP_flag
			MG323_TCP_Open();
			Delay(30);
		}
		asm("nop");	
		if( recv[9]=='2' )
		{
			//˵��internet�Ѿ����ӳɹ�
			check=0;
		}
		
		asm("nop");	
	}	
}
void UART1_Send(unsigned char str[], int len)
{
	int i;
	for(i=0;i<len;i++)
	{
		U1TXREG = str[i];
		while((U1STA&0x0100)==0x0000){asm("nop");}
	}
}
void UART2_Send(unsigned char str[], int len)
{
	int i;
	U2TXREG = '[';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	for(i=0;i<len;i++)
	{
		U2TXREG = str[i];
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	U2TXREG = ']';
	while((U2STA&0x0100)==0x0000){asm("nop");}
	//U2TXREG = 0x0a;
	//while((U2STA&0x0100)==0x0000){asm("nop");}
}
int UART1_Recv()
{
	int len=0;
	unsigned char recv_temp;
	while(0==(U1STA&0x0001))
	{}
	recv_temp = U1RXREG;
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	while(0==(U1STA&0x0001))
	{}
	recv_temp = U1RXREG;
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	while(0==(U1STA&0x0001))
	{}
	recv_temp = U1RXREG;
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	len++;
	recv[0] = recv_temp;
	while( (len<4) || ((recv[len-1]!=0x0a)||(recv[len-2]!=0x0d)||(recv[len-3]!='K')||(recv[len-4]!='O')) )
	{
		while(0==(U1STA&0x0001))
		{}
		recv_temp = U1RXREG;
		U2TXREG=recv_temp;
		while((U2STA&0x0100)==0x0000){asm("nop");}
		//len++;
		recv[len] = recv_temp;	
		len++;
	}
	U2TXREG = 0x0a;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	return len;	
}//
int UART1_Recv_With_OK()
{
	int len=0;
	unsigned char recv_temp;
	unsigned int UART_Timeout=0;
	
	while(0==(U1STA&0x0001)){}
	recv_temp = U1RXREG;
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	while(0==(U1STA&0x0001)){}
	recv_temp = U1RXREG;
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	while(0==(U1STA&0x0001)){}
	recv_temp = U1RXREG;
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	
	recv[0] = recv_temp;
	len++;
	
	while( (UART_Timeout<5000) && (len<4 || recv[len-1]!=0x0a || recv[len-2]!=0x0d || recv[len-3]!='K' || recv[len-4]!='O' ) )
	{
		UART_Timeout=0;
		while( (0==(U1STA&0x0001)) && (UART_Timeout<5000) )  {UART_Timeout++;}
		if(UART_Timeout<5000)
		{
			recv_temp = U1RXREG;
			U2TXREG=recv_temp;
			while((U2STA&0x0100)==0x0000){asm("nop");}			
			recv[len] = recv_temp;	
			len++;
		}
	}
	U2TXREG = 0x0a;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	return len;	
}
//
int UART1_Recv_Without_OK()
{
	int len=0;
	unsigned char recv_temp;
	unsigned int UART_Timeout=0;
	
	while(0==(U1STA&0x0001)){}
	recv_temp = U1RXREG;
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	while(0==(U1STA&0x0001)){}
	recv_temp = U1RXREG;
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	while(0==(U1STA&0x0001)){}
	recv_temp = U1RXREG;
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	
	recv[0] = recv_temp;
	len++;
	
	while( (UART_Timeout<50000) && ((len<4) || (recv[len-1]!=0x0a) || (recv[len-2]!=0x0d)) )
	{
	//	UART_Timeout=0;
		while( (0==(U1STA&0x0001)) && (UART_Timeout<50000) )  {UART_Timeout++;}
		if(UART_Timeout<50000)
		{
			UART_Timeout=0;
			recv_temp = U1RXREG;
			U2TXREG=recv_temp;
			while((U2STA&0x0100)==0x0000){asm("nop");}			
			recv[len] = recv_temp;	
			len++;
		}
	}
	U2TXREG = 0x0a;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	return len;	
}
//
int UART1_Recv_Safe_Without_OK()
{
	int len=0;
	unsigned char recv_temp;
	unsigned int UART_Timeout=0;
	
	while(0==(U1STA&0x0001) && (UART_Timeout<50000)){UART_Timeout++;}
	if(UART_Timeout<50000)	
	{
		UART_Timeout=0;
		recv_temp = U1RXREG;
		U2TXREG=recv_temp;
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	else
	{
		
	}
	
	while(0==(U1STA&0x0001) && (UART_Timeout<50000)){UART_Timeout++;}
	if(UART_Timeout<50000)	
	{
		UART_Timeout=0;
		recv_temp = U1RXREG;
		U2TXREG=recv_temp;
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	else
	{
		
	}
	
		while(0==(U1STA&0x0001) && (UART_Timeout<50000)){UART_Timeout++;}
	if(UART_Timeout<50000)	
	{
		UART_Timeout=0;
		recv_temp = U1RXREG;
		U2TXREG=recv_temp;
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	else
	{
		
	}
	
	
	recv[0] = recv_temp;
	len++;
	
	while( (UART_Timeout<50000) && ((len<6) || (recv[len-1]!=0x0a) || (recv[len-2]!=0x0d)) )
	{
		//UART_Timeout=0;
		while( (0==(U1STA&0x0001)) && (UART_Timeout<50000) )  {UART_Timeout++;}
		if(UART_Timeout<50000)
		{
			UART_Timeout=0;
			recv_temp = U1RXREG;
			U2TXREG=recv_temp;
			while((U2STA&0x0100)==0x0000){asm("nop");}			
			recv[len] = recv_temp;	
			len++;
		}
	}
	U2TXREG = 0x0a;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	return len;
}
//
int UART1_Recv_17_Safe_Without_OK()
{
	int len=0;
	unsigned char recv_temp;
	unsigned int UART_Timeout=0;
	
	while(0==(U1STA&0x0001) && (UART_Timeout<50000)){UART_Timeout++;}
	if(UART_Timeout<50000)	
	{
		UART_Timeout=0;
		recv_temp = U1RXREG;
		U2TXREG=recv_temp;
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
	else
	{
		
	}
	
	
	recv_17[0] = recv_temp;
	len++;
	
	while( (UART_Timeout<50000) && ((len<6) || (recv_17[len-1]!=0x0a) || (recv_17[len-2]!=0x0d)) )
	{
		//UART_Timeout=0;
		while( (0==(U1STA&0x0001)) && (UART_Timeout<50000) )  {UART_Timeout++;}
		if(UART_Timeout<50000)
		{
			UART_Timeout=0;
			recv_temp = U1RXREG;
			U2TXREG=recv_temp;
			while((U2STA&0x0100)==0x0000){asm("nop");}			
			recv_17[len] = recv_temp;	
			len++;
		}
	}
	U2TXREG = 0x0a;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	return len;
}
//
void UART1_Recv_ATE0()
{
	int len=0;
	unsigned char recv_temp;
	
//	U2TXREG = 0x0a;
//	while((U2STA&0x0100)==0x0000){asm("nop");}
	U2TXREG = 0x0a;
	while((U2STA&0x0100)==0x0000){asm("nop");}

	while(0==(U1STA&0x0001)){}
	recv_temp = U1RXREG;
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	len++;
	recv[0] = recv_temp;
	while( (len<4) || ((recv[len-1]!=0x0a)||(recv[len-2]!=0x0d)||(recv[len-3]!='K')||(recv[len-4]!='O')) )
	{
		while(0==(U1STA&0x0001))
		{}
		recv_temp = U1RXREG;
		U2TXREG=recv_temp;
		while((U2STA&0x0100)==0x0000){asm("nop");}
		recv[len] = recv_temp;	
		len++;
	}
	U2TXREG = 0x0a;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
}
int UART1_Recv_GPRS()
{
	int len=0;
	unsigned char recv_temp;
	unsigned int UART_Timeout=0;
	while(0==(U1STA&0x0001))
	{}
	recv_temp = U1RXREG;
//	U2TXREG=recv_temp;
//	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	while(0==(U1STA&0x0001))
	{}
	recv_temp = U1RXREG;
//	U2TXREG=recv_temp;
//	while((U2STA&0x0100)==0x0000){asm("nop");}
	while(0==(U1STA&0x0001))
	{}
	recv_temp = U1RXREG;
//	U2TXREG=recv_temp;
//	while((U2STA&0x0100)==0x0000){asm("nop");}
	len++;
	recv[0] = recv_temp;
/*	while( (len<4) || ((recv[len-1]!=0x0a)||(recv[len-2]!=0x0d)||(recv[len-3]!='K')||(recv[len-4]!='O')) )
	{
		while(0==(U1STA&0x0001))
		{}
		recv_temp = U1RXREG;
		recv[len] = recv_temp;	
		len++;
	}
*/	
	while( (UART_Timeout<50000) && ((len<4) || (recv[len-1]!=0x0a) || (recv[len-2]!=0x0d) ||(recv[len-3]!='K')||(recv[len-4]!='O')) )
	{
		//UART_Timeout=0;
		while( (0==(U1STA&0x0001)) && (UART_Timeout<50000) )  {UART_Timeout++;}
		if(UART_Timeout<50000)
		{
			UART_Timeout=0;
			recv_temp = U1RXREG;
		//	U2TXREG=recv_temp;
		//	while((U2STA&0x0100)==0x0000){asm("nop");}			
			recv[len] = recv_temp;	
			len++;
		}
	}
	return len;
		
}//

int UART1_Recv_CR_LF()
{
	int len=0;
	unsigned char recv_temp;
	
	while(0==(U1STA&0x0001)){}
	recv_temp = U1RXREG;	//0x0d
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	while(0==(U1STA&0x0001)){}
	recv_temp = U1RXREG;	//0x0a
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	while(0==(U1STA&0x0001)){}
	recv_temp = U1RXREG;	//^
	U2TXREG=recv_temp;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	
	
	recv[0] = recv_temp;	//^
	len++;
	while(  (recv[len-1]!=0x0a) || (recv[len-2]!=0x0d)  )
	{
		while(0==(U1STA&0x0001)){}
		recv_temp = U1RXREG;
		U2TXREG=recv_temp;
		while((U2STA&0x0100)==0x0000){asm("nop");}
		//len++;
		recv[len] = recv_temp;	
		len++;
	}
	U2TXREG = 0x0a;
	while((U2STA&0x0100)==0x0000){asm("nop");}
	return len;	
}//



