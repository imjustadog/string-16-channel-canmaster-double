
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
unsigned char ID1 = 1; // 板号****************************************************
unsigned char ID2 = 1;//个位
unsigned int freq[2]; 		  
unsigned int temp[2];//存储各采集通道的温度AD值
unsigned int BAT;
unsigned int Read_Enable; // 1->拾频完成，可以读取  0->禁止读取
unsigned int Tick; // 用来记录定时中断次数
unsigned int Timetick;//1->0.1S
unsigned int timeflag;
//unsigned int Time_long;
unsigned char Send_Enable; // 用来指示是否可以发送数据，上电开始时不发送数据，收到ZigBee芯片发送的'n'字符后才允许发送
int clock=0;
int halt_clock=1;//控制采集时间间隔1-1min
int highest_freq_flag=1;//默认采集时间间隔标志位 1-按照最大采集频率T=12s工作,0-按照设置的采集间隔工作
int Tran_Enable=0;//通过GPRS传送存储器内的数据 1-使能 0-禁止
char MG323_open_flag=1;//MG323开机标志位 1-开机失败 0-成功
char MG323_AT_flag=1;//AT应答指令标志位	1-应答失败
char MG323_ATE0_flag=1;//关闭回显标志位	1-回显关闭失败
char MG323_gprs_flag=1;//是否注册到中国移动网络标志位 1-注册失败，0-成功
char MG323_IOMODE_flag=1;
char MG323_conType_flag=1;
char MG323_apn_flag=1;
char MG323_conId_flag=1;
char MG323_srvType_flag=1;
char MG323_address_flag=1;	
char MG323_TCP_flag =1;//TCP连接标志位 1-连接失败 0-成功
char Send_OK_flag=1;//36位数据是否发送成功 1-发送失败 0-成功
//char send_36_flag = 0;//1-发送36个数据	
unsigned int curr_addr[2];//存储地址
unsigned int read_count_GPRS=20;//从存储器中读取read_count_GPRS组数据,并通过GPRS发送给上位机
unsigned int read_count_COM=0;//从存储器中读取组数据
unsigned int Acq_Save_count=90; //重连TCP的过程中，控制采集-存储时间间隔，(约为30min)
unsigned int COPS_count=0;//

unsigned char recv_17[20],recv[100],test[50],test1[100];
unsigned char send_data[36];
unsigned char send_AT[4]={'A','T',0x0d,0x0a};
unsigned char send_ATE0[6]={'A','T','E','0',0x0d,0x0a};
unsigned char send_COPS[10]={'A','T','+','C','O','P','S','?',0x0d,0x0a};
//unsigned char send_IOMODE[14] = {'A','T','^','I','O','M','O','D','E','=','0',',','0',0x0d};
unsigned char send_IOMODE[14] = {'A','T','^','I','O','M','O','D','E','=','0',',','1',0x0d};//不使用接收缓存

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
int time2bcd(int x)//时间信息的10进制->BCD码
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
	if(Tick<(Timetick*100))// Timetick=10,即1s
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

	if( recv[2]=='S' && recv[3]=='R' && recv[4]==':' )//不使用接收缓冲时的 ^SISR 主动上报命令
	{
		asm("nop");	
		if(recv[14]=='S'&&recv[30]=='E')//接收17位控制命令
		{
			if(recv[15]=='A'&&recv[16]=='S'&&recv[17]=='W')
			{
				//数据发送成功后，上位机产生的应答指令，表示发送成功
				Send_OK_flag=0;
			}
			if(recv[15]=='R'&&recv[16]=='S'&&recv[17]=='T')
			{
				//接收上位机发来的重启命令，MG323复位
				MG323_RESTART();
				MG323_TCP_Open();
			}
			if(recv[15]=='O'&&recv[16]=='F')//读取存储器中的数据
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
				read_count_COM=0;//通过GPRS传数据时，禁止COM传数据。
				Tran_Enable=1;
				asm("nop");
			}
			if(recv[15]!='A'&&recv[15]!='R'&&recv[15]!='O'&&recv[17]!=0x39)	//设置采样间隔			
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
		
			if( (recv[18]!=0x39)&&(recv[19]!=0x39) ) 	//校准实时时钟 
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
			//接收上位机发来的重启命令，MG323复位
			MG323_RESTART();
			MG323_TCP_Open();
		}
	*/	
	}	
	if(  recv[2]=='S' && recv[3]=='W' &&  recv[8]=='3' && recv[9]=='6' )//允许向服务缓冲区写入36数据
	{
		//send_36_flag = 1;
		UART2_Send(send_data,36);
		UART1_Send(send_data,36);
		U1TXREG = 0x0d;
		while((U1STA&0x0100)==0x0000){asm("nop");}
	}
	if( recv[2]=='O' && recv[3]=='R' )//不能向服务缓冲区写入数据
	{
		MG323_RESTART();
		MG323_TCP_Open();
	}
	if(  (recv[2]=='S'&&recv[3]==':'&&recv[7]=='0' )  )//^SIS 主动上报命令
	{
		asm("nop");
		MG323_RESTART();
		MG323_TCP_Open();
	}
	if( recv[2]=='S' && recv[3]=='W' )//^SISW主动上报命令
	{
		if( recv[8]=='1')//服务准备好接收新的用户数据
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
void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void)//用于接收小助手发来的控制指令
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
			data[i]='E'; // 如果接收超时，退出中断
		}
		else
		{
			data[i] = U2RXREG;
		}
		UART_Timeout = 0;
	}

	if(i==7)
		{		
			if(data[1]!=0xff) 	//校准实时时钟 
			{
				ds1302_write_time(6,data[1]);
				ds1302_write_time(4,data[2]);
				ds1302_write_time(3,data[3]);
				ds1302_write_time(2,data[4]);
				ds1302_write_time(1,data[5]);
				ds1302_write_time(0,data[6]);
			}
			if(data[1]==0xff)//读取存储器中的数据
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
					
				read_count_GPRS=0;//通过GPRS传数据时，禁止COM传数据。
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
 	timeflag=1;	//Delay()标志位
	Tick = 0; 	//T6 中断次数，定时1ms中断	
//	Time_long = 30;//默认采集时间间隔3s
		
	
	TRISCbits.TRISC1 = 1;//BAT
	
	TRISGbits.TRISG13 = 0;//时钟
	TRISGbits.TRISG14 = 0;			
	
	TRISGbits.TRISG0 = 0;//MG323
	TRISGbits.TRISG1 = 0;	

	send_data[0]='S';
	send_data[1]=ID1+0x30;
	send_data[2]=ID2+0x30;
	for(j=3;j<=34;j++)
	{send_data[j]=0x38;}
	send_data[35]='E';
	
	InitTimer6();  // Timer6 提供1ms中断定时	
	InitTimer2();  // Timer2 提供输入捕捉时钟基准
	InitIC();
	InitADC();
	InitSPI();
	ds1302_init();
	InitSCI();
	
	while((U1STA&0x0001)==0x0001){dummy = U1RXREG;}	
	
	LATGbits.LATG0=0;	//关闭复位
	LATGbits.LATG1 =1;	//MG323开机
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
			LATGbits.LATG0=1;//将MG323复位
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
			//如果i次应答失败，则重启MG323
		}
		asm("nop");
	}
	MG323_AT_flag=1;//fail	
	
	while(MG323_ATE0_flag)
	{	
		UART1_Send(send_ATE0,5);//ATE0 //关闭回显		
		j=UART1_Recv_With_OK();
		asm("nop");
		if(j==9 && recv[j-1]==0x0a && recv[j-3]=='K')
		{
			MG323_ATE0_flag=0;//success
		}else
		{
			//再次发送ATE0，如果i次应答失败，则重启MG323
		}
		asm("nop");
	}
	MG323_ATE0_flag=1;

	while(MG323_gprs_flag)//查询是否注册到中国移动，若未注册成功，不断注册
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
			MG323_gprs_flag = 1;//未注册到中国移动网络
			Delay(15);
			COPS_count++;
			//如果重复注册了15次，则重启MG323模块
			if(COPS_count>=15)
			{
				COPS_count=0;
				LATGbits.LATG0=1;//将MG323复位
				Delay(1);//100ms
				LATGbits.LATG0=0;
				
				
				while(MG323_open_flag)
				{
					j=UART1_Recv_CR_LF();
					j=UART1_Recv_CR_LF();//读取开机信息 "SYSSTART"
					
			//		j=UART1_Recv_Without_OK();//^SYSSTART<CR><LF>
					asm("nop");		
					if(j==9)
					{
						MG323_open_flag=0;//open success
					}
					else
					{
						MG323_open_flag=1;//open fail
						LATGbits.LATG0=1;//将MG323复位
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
						//如果i次应答失败，则重启MG323
					}
					asm("nop");
				}
				MG323_AT_flag=1;//fail	
				
				while(MG323_ATE0_flag)
				{	
					UART1_Send(send_ATE0,5);//ATE0 //关闭回显		
					j=UART1_Recv_With_OK();
					asm("nop");
					if(j==9 && recv[j-1]==0x0a && recv[j-3]=='K')
					{
						MG323_ATE0_flag=0;//success
					}else
					{
						//再次发送ATE0，如果i次应答失败，则重启MG323
					}
					asm("nop");
				}
				MG323_ATE0_flag=1;

			}//
		}
	}
	MG323_gprs_flag = 1;//未注册到中国移动网络

	while(MG323_IOMODE_flag)
	{
		UART2_Send(send_IOMODE,13);//设置数据格式
		UART1_Send(send_IOMODE,14);//AT^IOMODE[13] 
		j=UART1_Recv_With_OK();
		if(j==4)
		{		
			MG323_IOMODE_flag=0;//success
		}else
		{
			//再次发送，如果i次失败，则重启MG323
		}	
	}	
	MG323_IOMODE_flag=1;	
	
	while(MG323_conType_flag)
	{
		UART2_Send(send_SICS_conType,23);//选择 0 通道 Internet连接类型为 GPRS 
		UART1_Send(send_SICS_conType,24);//at^sics=0,conType,GPRS0[23] 
		j=UART1_Recv_With_OK();
		if(j==6)
		{		
			MG323_conType_flag=0;//success
		}else
		{
			//再次发送，如果i次失败，则重启MG323
		}	
	}	
	MG323_conType_flag=1;	

	while(MG323_apn_flag)
	{
		UART2_Send(send_SICS_apn,19);//设置 0 通道 APN 为CMNET 
		UART1_Send(send_SICS_apn,20);//at^sics=0,apn,cmnet[19]
		j=UART1_Recv_With_OK();
		if(j==6)
		{
			MG323_apn_flag=0;//设置成功	MG323_apn_flag
		}else
		{
			//再次发送，如果i次失败，则重启MG323
		}
	}
	MG323_apn_flag=1;
	
	while(MG323_conId_flag)
	{
		UART2_Send(send_SISS_conId,17);//设置 0 通道  conId 为 0 
		UART1_Send(send_SISS_conId,18);//at^siss=0,conId,0[17]
		j=UART1_Recv_With_OK();
		if(j==6)
		{
			MG323_conId_flag=0;//设置成功	MG323_conId_flag
		}else
		{
			//再次发送，如果i次失败，则重启MG323
		}
	}	
	MG323_conId_flag=1;
		
	while(MG323_srvType_flag)
	{
		UART2_Send(send_SISS_srvType,24);//设置 0 通道服务类型
		UART1_Send(send_SISS_srvType,25);//at^siss=0,srvType,socket[24]
		j=UART1_Recv_With_OK();
		if(j==6)
		{
			MG323_srvType_flag=0;//设置成功	MG323_srvType_flag
		}else
		{
			//再次发送，如果i次失败，则重启MG323
		}
	}	
	MG323_srvType_flag=1;
		
	while(MG323_address_flag)
	{
		UART2_Send(send_SISS_address,57);//设置连接服务器的 IP和端口号
		UART1_Send(send_SISS_address,58);//at^siss=0,address,"socktcp://bit-smc-gprs.gicp.net:4000"[56]
		j=UART1_Recv_With_OK();
		if(j==6)
		{
			MG323_address_flag=0;//设置成功	MG323_address_flag
		}else
		{
			//再次发送，如果i次失败，则重启MG323 
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
	UART1_Send(send_SICI,9);//查询分配到的IP
	j=UART1_Recv_With_OK();
	asm("nop");	
	if( recv[9]=='0' )
	{
		//说明internet已经定义但没有连接
		//应该重新建立TCP连接 MG323_TCP_flag
	}
	
	if( recv[9]=='2' )
	{
		//说明internet已经连接成功
	}
	

	UART2_Send(send_SISW,12);
	UART1_Send(send_SISW,13);//AT^SISW=0,36 //向服务的缓冲区写入36字节数据//写入36个数据send_data[36]后回车，
	j=UART1_Recv_Without_OK();	//返回" ^SISW: 0,36,36 " 的长度 j
								//最后一个字节值36-已经发送但TCP层未确认的字节数，有可能不是36，而是36的倍数
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
		//说明发送成功,可以继续发送
	}		
	asm("nop");

	TRISFbits.TRISF1 = 0;//CEA//激频出口	
	TRISDbits.TRISD8 = 1;//PH1//拾频入口
	TRISDbits.TRISD10 = 1;//PH2
	TRISBbits.TRISB9 = 1;//TMPOP1//AD输入口
	TRISBbits.TRISB1 = 1;//TMPOP2

	curr_addr[0] = 0;
	curr_addr[1] = 0;	//一开始向FRAM中存放采集数据从地址 0x00000 开始
	
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
			year = ds1302_read_time(6);//获取实时时间，时间数据格式 ASC码,注意子程序DS1302.c  L97与其他程序的区别！！！！
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
	            		CEA = 1;//RG1输出为高            	
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
				DELAY(2000);//保证捕捉完成，最短为19ms
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
				
		/*将2个通道的传感器热敏电阻返回的电压量进行AD转化，并存在于temp[]中 */		
				AD1CHS0bits.CH0SA = 9; //AN9 == TMPOP1
				AD1CON1bits.ADON = 1; // Turn on the A/D converter
				asm("nop");asm("nop");asm("nop");asm("nop");
				AD1CON1bits.SAMP = 1; 		//ADC采样使能位
											//1: ADC采样/保持放大器正在采样
											//0: ADC 采样/保持放大器保持采样结果
				while(!AD1CON1bits.DONE){}; //ADC 转换状态位
											//1 = ADC 转换完成
											//0 = ADC 转换尚未开始或在进行中			
				temp[0] = ADC1BUF0;	//每次A/D 转换的结果存储在ADCxBUF0寄存器中
									//ADC 模块包含一个单字只读双端口寄存器（ADCxBUF0）
				AD1CON1bits.ADON = 0;//ADC 工作模式位
									//1 = ADC 模块正在工作
									//0 = ADC 关闭		
		
				AD1CHS0bits.CH0SA = 1;//AN1 == TMPOP2
				AD1CON1bits.ADON = 1;
				asm("nop");asm("nop");asm("nop");asm("nop");
				AD1CON1bits.SAMP = 1;
				while(!AD1CON1bits.DONE){};
				temp[1] = ADC1BUF0;	
				AD1CON1bits.ADON = 0;
			
				year = ds1302_read_time(6);//获取实时时间，时间数据格式 ASC码,注意子程序DS1302.c  L97与其他程序的区别！！！！
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
				UART1_Send(send_SISW,13);//AT^SISW=0,36 //发送数据//写入36个数据send_data[36]后回车
				while(Send_OK_flag)
				{
					DELAY(60000);
					DELAY(60000);
					DELAY(60000);
					count++;
					if(count==3)//2*2.7s内没有接收到上位机的应答指令，则认为网络丢失，将数据存储到FM25V10中
					{
						Send_OK_flag=0;
						count=0;
						
						for(n=0;n<36;n++)//共存储3640组数据
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
				if(minute != minute_m)//通过比较两次实时时钟的“分钟”，如果相邻两次不同，则为定时1min
				{
					minute_m = minute;
					clock++;		
				
					if(clock>=halt_clock)
				//	if(clock>=1)	//采集时间间隔为1分钟
					{						
						clock=0;
						i = 2000;
						while(i>=50)
						{
							for(k=0;k<3;k++)
							{
			            		j = i;            		
			            		CEA = 1;//RG1输出为高            	
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
						DELAY(2000);//保证捕捉完成，最短为19ms
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
					
			/*将2个通道的传感器热敏电阻返回的电压量进行AD转化，并存在于temp[]中 */		
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
				
						year = ds1302_read_time(6);//获取实时时间，时间数据格式 ASC码,注意子程序DS1302.c  L97与其他程序的区别！！！！
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
						UART1_Send(send_SISW,13);//AT^SISW=0,36 //发送数据//写入36个数据send_data[36]后回车，
						while(Send_OK_flag)
						{
							DELAY(60000);
							DELAY(60000);
							DELAY(60000);
							count++;
							if(count>=2)//2*2.7s内没有接收到上位机的应答指令，则认为网络丢失，将数据存储到FM25V10中
							{
								Send_OK_flag=0;
								count=0;
								
								for(n=0;n<36;n++)//共存储3640组数据
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
	curr_addr[1] = 0; //每次读取FRAM中的数据都从地址 0x00000 开始

	send_data[0]='S';
	send_data[1]=ID1+48;
	send_data[2]=ID2+48;
	for(i=3;i<31;i++){send_data[i]='0';}
	for(i=31;i<35;i++){send_data[i]='1';}
	send_data[35]='E';
	UART2_Send(send_SISW,12);
	UART1_Send(send_SISW,13);//AT^SISW=0,36 //发送数据
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
		UART1_Send(send_SISW,13);//AT^SISW=0,36 //发送数据//写入36个数据send_data[36]后回车
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
	UART1_Send(send_SISW,13);//AT^SISW=0,36 //发送数据
	
	DELAY(60000);
	DELAY(60000);
	DELAY(60000);
}//
void Tran_Data_U2_COM(void) // 
{
	unsigned char i;
//	unsigned int j;
	curr_addr[0] = 0;
	curr_addr[1] = 0; //每次读取FRAM中的数据都从地址 0x00000 开始

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
          		CEA = 1;//RG1输出为高            	
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
	DELAY(2000);//保证捕捉完成，最短为19ms
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
	
/*将2个通道的传感器热敏电阻返回的电压量进行AD转化，并存在于temp[]中 */		
	AD1CHS0bits.CH0SA = 9; //AN9 == TMPOP1
	AD1CON1bits.ADON = 1; // Turn on the A/D converter
	asm("nop");asm("nop");asm("nop");asm("nop");
	AD1CON1bits.SAMP = 1; 		//ADC采样使能位
								//1: ADC采样/保持放大器正在采样
								//0: ADC 采样/保持放大器保持采样结果
	while(!AD1CON1bits.DONE){}; //ADC 转换状态位
								//1 = ADC 转换完成
								//0 = ADC 转换尚未开始或在进行中			
	temp[0] = ADC1BUF0;	//每次A/D 转换的结果存储在ADCxBUF0寄存器中
						//ADC 模块包含一个单字只读双端口寄存器（ADCxBUF0）
	AD1CON1bits.ADON = 0;//ADC 工作模式位
						//1 = ADC 模块正在工作
						//0 = ADC 关闭		

	AD1CHS0bits.CH0SA = 1;//AN1 == TMPOP2
	AD1CON1bits.ADON = 1;
	asm("nop");asm("nop");asm("nop");asm("nop");
	AD1CON1bits.SAMP = 1;
	while(!AD1CON1bits.DONE){};
	temp[1] = ADC1BUF0;	
	AD1CON1bits.ADON = 0;

//	year = ds1302_read_time(6);//获取实时时间，时间数据格式 ASC码,注意子程序DS1302.c  L97与其他程序的区别！！！！
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
	
	for(n=0;n<36;n++)//共存储3640组数据
	{
		WRITE(curr_addr[0],(unsigned char)(curr_addr[1]>>8),(unsigned char)(curr_addr[1]),send_data[n]);
		inc_addr();	
	}
						
}
void MG323_RESTART(void)
{
	int j;
	unsigned char dummy;
	//每重启40次（约为30min）时，启动数据采集-存储功能，此时存储的数据格式中将 年设置为87 ，即读取到的87年即认为是在重启时采集的数据
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

	LATGbits.LATG0=1;//将MG323复位
	Delay(1);//100ms
	LATGbits.LATG0=0;
				
	while(MG323_open_flag)//如果MG323没有开机成功或者不能应答AT指令，不断将其复位
	{
		
		j=UART1_Recv_CR_LF();
		j=UART1_Recv_CR_LF();//读取开机信息 "SYSSTART"
		asm("nop");
		UART1_Send(send_AT,4);//AT	
		j=UART1_Recv_With_OK();
		asm("nop");
		
		if( (recv[j-4]=='O')&&(recv[j-3]=='K')&&(recv[j-1]==0x0a) )
		{
			UART2_Send(send_OK,8);//MG323已开机，且能够应答AT指令
			U2TXREG=0x0a;
			while((U2STA&0x0100)==0x0000){asm("nop");}
			MG323_open_flag=0;
		}
		else
		{
			LATGbits.LATG0=1;//将MG323复位
			Delay(1);//100ms
			LATGbits.LATG0=0;	
		}
	
	}	
	MG323_open_flag=1;
	
	UART1_Send(send_ATE0,5);//ATE0 //关闭回显
	j=UART1_Recv_With_OK();
	
	while(MG323_gprs_flag)//查询是否注册到中国移动，若未注册成功，不断注册
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
			MG323_gprs_flag= 1;//未注册到中国移动网络
			Delay(15);
			
			//如果重复注册了20次，则重启MG323模块
			COPS_count++;
			if(COPS_count>=20)
			{
				COPS_count=0;
				LATGbits.LATG0=1;//将MG323复位
				Delay(1);//100ms
				LATGbits.LATG0=0;
				
				
				while(MG323_open_flag)
				{
					j=UART1_Recv_CR_LF();
					j=UART1_Recv_CR_LF();//读取开机信息 "SYSSTART"
					
			//		j=UART1_Recv_Without_OK();//^SYSSTART<CR><LF>
					asm("nop");		
					if(j==9)
					{
						MG323_open_flag=0;//open success
					}
					else
					{
						MG323_open_flag=1;//open fail
						LATGbits.LATG0=1;//将MG323复位
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
						//如果i次应答失败，则重启MG323
					}
					asm("nop");
				}
				MG323_AT_flag=1;//fail	
				
				while(MG323_ATE0_flag)
				{	
					UART1_Send(send_ATE0,5);//ATE0 //关闭回显		
					j=UART1_Recv_With_OK();
					asm("nop");
					if(j==9 && recv[j-1]==0x0a && recv[j-3]=='K')
					{
						MG323_ATE0_flag=0;//success
					}else
					{
						//再次发送ATE0，如果i次应答失败，则重启MG323
					}
					asm("nop");
				}
				MG323_ATE0_flag=1;

			}//
		}
	}
	MG323_gprs_flag = 1;//未注册到中国移动网络
	
	asm("nop");
	UART2_Send(send_IOMODE,13);//设置数据格式
	UART1_Send(send_IOMODE,14);//AT^IOMODE[13] 
	j=UART1_Recv_With_OK();
	
	UART2_Send(send_SICS_conType,23);//选择 0 通道 Internet连接类型为 GPRS 
	UART1_Send(send_SICS_conType,24);//at^sics=0,conType,GPRS0[23]
	j=UART1_Recv_With_OK();
	
	UART2_Send(send_SICS_apn,19);//设置 0 通道 APN 为CMNET 
	UART1_Send(send_SICS_apn,20);//at^sics=0,apn,cmnet[19]
	j=UART1_Recv_With_OK();
	//asm("nop");
				
	UART2_Send(send_SISS_conId,17);//设置 0 通道  conId 为 0 
	UART1_Send(send_SISS_conId,18);//at^siss=0,conId,0[17]
	j=UART1_Recv_With_OK();
	//asm("nop");
	
	UART2_Send(send_SISS_srvType,24);//设置 0 通道服务类型
	UART1_Send(send_SISS_srvType,25);//at^siss=0,srvType,socket[24]
	j=UART1_Recv_With_OK();
	//asm("nop");
	
	UART2_Send(send_SISS_address,57);//设置连接服务器的 IP和端口号
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
		UART1_Send(send_SISO,10);//AT^SISO=0[9] //打开TCP连接 			
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
	int check=1;//1-连接成失败 0-Internet已经连接成功
	unsigned char dummy;
	while(check)
	{
		while((U1STA&0x0002)==0x0002)
		{
			U1STAbits.OERR = 0;
			dummy = U1RXREG;
		}
		UART2_Send(send_SICI,8);
		UART1_Send(send_SICI,9);//查询internet 是否连接成功
		j=UART1_Recv_With_OK();
		asm("nop");	
		if( recv[9]=='0' )
		{
			asm("nop");	
			//说明internet已经定义但没有连接
			check=1;
			//应该重新建立TCP连接 MG323_TCP_flag
			MG323_TCP_Open();
			Delay(30);
		}
		asm("nop");	
		if( recv[9]=='2' )
		{
			//说明internet已经连接成功
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



