/*随时查询模块当前的工作状态*/
/*FRAM中开辟00000~0001f 存储32字节的指令（查询指令 之前的控制指令）
	查询指令格式为 53 52 44 44 FF 44 FF 00 00 00 00 00 44 FF 45（15字节）；
	查询到的之前的控制指令的内容为 53 52 …… FF 45（15字节）+ 0x22（17个）+C M D ；
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
#define RESET LATGbits.LATG15 //RESET 用来复位zigbee的路由/终端
//#define WORK_LED LATDbits.LATD1
#define BAT_LED1 LATBbits.LATB14
#define BAT_LED2 LATBbits.LATB13
#define BAT_LED3 LATBbits.LATB12

unsigned int freq[6]; 
//unsigned int temp_freq[6];	
//float freq_float;			  
unsigned int temp[6];//存储各采集通道的温度AD值
unsigned int bat; //电池电量AD值
//unsigned int dummy;
short bat_high, bat_mid, bat_low; // 计数器，用于对电池电压进行滤波

//int CE_Enable; // 1->允许激频  0->禁止激频 
int Read_Enable; // 1->拾频完成，可以读取  0->禁止读取
int Tick; // 用来记录定时中断次数

short halt; // 用来控制采样时间间隔
unsigned int halt_Tick; // 用来记录时间，该时间为两次发送之间的时间间隔
unsigned int halt_Timeout; // 两次发送之间的间隔，单位为ms，可通过上位机设置

unsigned char Send_Enable; // 用来指示是否可以发送数据，上电开始时不发送数据，收到ZigBee芯片发送的'n'字符后才允许发送
unsigned char Save_Enable; // 用来表示是否向FRAM中存数据
unsigned char Tran_Enable; // 用来表示是否将FRAM中的数据读出
unsigned char halt_Enable; // 休眠

unsigned char ID = 5; // 板号*****************************************************************************

unsigned char stat1;
unsigned char net_state;//用来指示 组网成功后，向zigbee传送10组35字节的数据
unsigned char lq[32];//存放从FRAM中读取的指令内容

//unsigned int end_addr[2]; // 存储结束地址，存储于FRAM 0，1，2地址处
//unsigned char temp_addr[3];
unsigned int curr_addr[2];
unsigned int read_count;//用来记录从FRAM中读取的数据包个数，每包32字节数据，打包后为35字节
//unsigned int read_count; //用来记录从FRAM种读取的字符个数，用来鉴别数据头，并进行数据填充
void inc_addr(void); // 将curr_addr加1

void Tran_Data_U1(void); // 向上位机传输数据
void Tran_Data_U2(void);// 向小助手传输数据

void read_command_15_U1(void); //读取存在FRAM中的15字节的指令
void read_command_15_U2(void); //读取存在FRAM中的15字节的指令

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
			if(halt_Tick<halt_Timeout)//（在线or离线模式下）控制采集时间间隔
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
				Read_Enable = 1;	//保证在这段时间内，拾频捕捉结束
			}
		}//用于控制 拾频时间
	}

}//

/*u1 串口中断，用于接收上位机发来的控制指令*/
void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
	unsigned char data[15];
	unsigned int UART_Timeout = 0;
	unsigned char i = 0;
	unsigned char j = 0;
	data[0] = U1RXREG;
	
	if('R'==data[0]) // 采集模块接收到路由/终端发来的网络丢失信号后，对路由/终端进行复位
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
	if('n'==data[0]) 	  // 调试用，发送n才开始激频
	{
		net_state = 1;
		for(j=0;j<32;j++) //读取FRAM中存储的指令，格式为15字节指令+17个0x22，共32字节
		{
			lq[j] = READ(0,0,j);
		}

//lq   0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  ……  30 31			
//     S   R 				                                    E  22  22      22 22
//在线 S   R   FF  FF  FF  00  FF  FF  FF  FF  FF  FF  FF  FF   E 
//离线 S   R   00  FF  FF  00  FF  FF  FF  FF  FF  FF  FF  FF   E
//休眠 S   R   55  FF  FF  00  FF  FF  FF  FF  FF  FF  00  FF   E

		//网络建立成功后，采集模块自动检测上次工作状态，若之前为在线状态，则继续以在线状态工作
		if((lq[0]=='S')&&(lq[1]=='R')&&(lq[14]=='E')&&(lq[31]==0x22))
		{	
			if((lq[2]==0xFF)&&(lq[5]==0x00)&&(lq[12]==0xff)) // 在线模式
				{
					Send_Enable = 1;
					Save_Enable = 0;
					Tran_Enable = 0;
					halt_Enable = 0;
				}

		}

	}//if('n'==data[0])
	else//接收到串口发来的15字节的指令，其格式为S …… E
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
						data[i]='E'; // 如果接收超时，退出中断
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
//在线 S   R   FF  FF  FF  00  FF  FF  FF  FF  FF   FF   FF   FF   E 
//离线 S   R   00  FF  FF  00  FF  FF  FF  FF  FF   FF   FF   FF   E
//休眠 S   R   55  FF  FF  00  FF  FF  FF  FF  FF   FF   00   FF   E
//回读 S   R   55  FF  FF  FF  FF  FF  FF  FF  FF   FF   FF   FF   E
//查询 S   R   44  44  FF  44  FF  FF  FF  FF  FF   FF   44   FF   E
		
			if((data[2]==0xff)&&(data[5]==0x00)&&(data[12]==0xff)) // 在线模式
			{
				Send_Enable = 1;
				Save_Enable = 0;
				Tran_Enable = 0;
				halt_Enable = 0;
			}
			if((data[2]==0x00)&&(data[5]==0x00)&&(data[12]==0xff)) // 离线模式
			{	net_state = 0;//通过指令设置离线状态时，不向zigbee发送10组35字节的数据
				Send_Enable = 0;
				Save_Enable = 1;
				Tran_Enable = 0;
				halt_Enable = 0;
				
				curr_addr[0] = 0;
				curr_addr[1] = 32;		
			}
			if((data[2]==0x55)&&(data[5]==0x00)&&(data[12]==0x00)) //休眠
			{	net_state = 0;//通过指令设置休眠状态时，不向zigbee发送10组35字节的数据
				Send_Enable = 0;
				Save_Enable = 0;
				Tran_Enable = 0;
				halt_Enable = 1;
			}	
			if(data[4]!=0xff)	//(在线or离线模式下)设置采样间隔
								//即：可在设置在线状态的同时设置采样间隔（同一条指令内）
								//即：可在设置离线状态的同时设置采样间隔（同一条指令内）
			{
				halt_Timeout = data[4]*500; 
			}
			
			if(data[5]==0xff) 	//回读FRAM值
			{
				Send_Enable = 0;
				Save_Enable = 0;
				Tran_Enable = 1;				
				halt_Enable = 0;
			}			
			if(data[6]!=0xff) 	//校准实时时钟 
			{
				ds1302_write_time(6,data[6]);
				ds1302_write_time(4,data[7]);
				ds1302_write_time(3,data[8]);
				ds1302_write_time(2,data[9]);
				ds1302_write_time(1,data[10]);
				ds1302_write_time(0,data[11]);
			}			
					
			
			for(j=0;j<32;j++) //读取FRAM中存储的指令，格式为15字节指令+17个0x22，共32字节
			{
				lq[j] = READ(0,0,j);
			}
			//写15字节的指令,余位补0x22 ，存在00000~0001f  ,共32字节
			//格式为 15字节指令+17个0x22
			if(((data[2]==0xff)&&(data[5]==0x00)&&(data[12]==0xff))||((data[2]==0x00)&&(data[5]==0x00)&&(data[12]==0xff))||((data[2]==0x55)&&(data[5]==0x00)&&(data[12]==0x00)))	
				//只有工作状态设置指令才存储 （在线or离线or休眠）							
			{
				for(j=0;j<15;j++)
				{
						WRITE(0,0,j,(unsigned char)data[j]); //data[0]~data[14] ——15字节 指令内容
				}
				WRITE(0,0,15,ID);
				for(j=16;j<32;j++)
				{	
					WRITE(0,0,j,0x22);  // ——0x22
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
				while((U1STA&0x0100)==0x0000){asm("nop");} //接收到15字节指令后，产生应答指令，
													   //其格式为 S ID 0xff……0xff XX XX XX A ，35字节
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
				while((U1STA&0x0100)==0x0000){asm("nop");} //接收到15字节指令后，产生应答指令，
													   //其格式为 S ID 0xff……0xff XX XX XX A ，35字节
				DELAY(6000);
			}
			if((data[3]==0x44)&&(data[2]==0x44)&&(data[5]==0x44)&&(data[12]==0x44)) //查询工作状态
			{
				read_command_15_U1();
			}	
		
		}//if(i==14)	
	}
	IFS0bits.U1RXIF = 0;
	return;	
}//

/*u2 串口中断，用于接收小助手发来的控制指令*/
void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void)
{
	unsigned char data[15];
	unsigned int UART_Timeout = 0;
	unsigned char i = 0;
	unsigned char j = 0;
	data[0] = U2RXREG;
	if('n'==data[0]) 	  // 调试用，发送n才开始激频
	{
		net_state = 1;
		for(j=0;j<32;j++) //读取FRAM中存储的指令，格式为15字节指令+17个0x22，共32字节
		{
			lq[j] = READ(0,0,j);
		}
		
//lq   0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  ……  30 31			
//     S   R 				                                    E  22  22      22 22
//在线 S   R   FF  FF  FF  00  FF  FF  FF  FF  FF  FF  FF  FF   E 
//离线 S   R   00  FF  FF  00  FF  FF  FF  FF  FF  FF  FF  FF   E
//休眠 S   R   55  FF  FF  00  FF  FF  FF  FF  FF  FF  00  FF   E

		//网络建立成功后，采集模块自动检测上次工作状态，若之前为在线状态，则继续以在线状态工作
		if((lq[0]=='S')&&(lq[1]=='R')&&(lq[14]=='E')&&(lq[31]==0x22))
		{	
			if((lq[2]==0xFF)&&(lq[5]==0x00)&&(lq[12]==0xff)) // 在线模式
				{
					Send_Enable = 1;
					Save_Enable = 0;
					Tran_Enable = 0;
					halt_Enable = 0;
				}

		}

	}//if('n'==data[0])
	else//接收到串口发来的15字节的指令，其格式为S …… E
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
						data[i]='E'; // 如果接收超时，退出中断
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
			for(j=0;j<32;j++) //读取FRAM中存储的指令，格式为15字节指令+17个0x22，共32字节
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
			while((U2STA&0x0100)==0x0000){asm("nop");} //接收到15字节指令后，产生应答指令，
													   //其格式为 S ID 0xff……0xff xx xx xx A ，35字节
			DELAY(6000);
						
//data 0   1   2   3   4   5   6   7   8   9   10   11   12   13   14			
//     S   R 				                                       E  
//在线 S   R   FF  FF  FF  00  FF  FF  FF  FF  FF   FF   FF   FF   E 
//离线 S   R   00  FF  FF  00  FF  FF  FF  FF  FF   FF   FF   FF   E
//休眠 S   R   55  FF  FF  00  FF  FF  FF  FF  FF   FF   00   FF   E
//回读 S   R   55  FF  FF  FF  FF  FF  FF  FF  FF   FF   FF   FF   E
//查询 S   R   44  44  FF  44  FF  FF  FF  FF  FF   FF   44   FF   E
		
			if((data[2]==0xff)&&(data[5]==0x00)&&(data[12]==0xff)) // 在线模式
			{
				Send_Enable = 1;
				Save_Enable = 0;
				Tran_Enable = 0;
				halt_Enable = 0;
			}
			if((data[2]==0x00)&&(data[5]==0x00)&&(data[12]==0xff)) // 离线模式
			{	net_state = 0;//通过指令设置离线状态时，不向zigbee发送10组35字节的数据
				Send_Enable = 0;
				Save_Enable = 1;
				Tran_Enable = 0;
				halt_Enable = 0;
				
				curr_addr[0] = 0;
				curr_addr[1] = 32;		
			}
			if((data[2]==0x55)&&(data[5]==0x00)&&(data[12]==0x00)) //休眠
			{	net_state = 0;//通过指令设置休眠状态时，不向zigbee发送10组35字节的数据
				Send_Enable = 0;
				Save_Enable = 0;
				Tran_Enable = 0;
				halt_Enable = 1;
			}	
			if(data[4]!=0xff)	//(在线or离线模式下)设置采样间隔
								//即：可在设置在线状态的同时设置采样间隔（同一条指令内）
								//即：可在设置离线状态的同时设置采样间隔（同一条指令内）
			{
				halt_Timeout = data[4]*500; 
			}
			
			if(data[5]==0xff) 	//回读FRAM值
			{
				Send_Enable = 0;
				Save_Enable = 0;
				Tran_Enable = 1;				
				halt_Enable = 0;
			}			
			if(data[6]!=0xff) 	//校准实时时钟 
			{
				ds1302_write_time(6,data[6]);
				ds1302_write_time(4,data[7]);
				ds1302_write_time(3,data[8]);
				ds1302_write_time(2,data[9]);
				ds1302_write_time(1,data[10]);
				ds1302_write_time(0,data[11]);
			}			
			if((data[3]==0x44)&&(data[2]==0x44)&&(data[5]==0x44)&&(data[12]==0x44)) //查询工作状态
			{
				read_command_15_U2();
			}		
			
			//写15字节的指令,余位补0x22 ，存在00000~0001f  ,共32字节
			//格式为 15字节指令+17个0x22
			if(((data[2]==0xff)&&(data[5]==0x00)&&(data[12]==0xff))||((data[2]==0x00)&&(data[5]==0x00)&&(data[12]==0xff))||((data[2]==0x55)&&(data[5]==0x00)&&(data[12]==0x00)))	
				//只有工作状态设置指令才存储 （在线or离线or休眠）														
			{
				for(j=0;j<15;j++)
				{
						WRITE(0,0,j,(unsigned char)data[j]); //data[0]~data[14] ——15字节 指令内容
				}
				WRITE(0,0,15,ID);
				for(j=16;j<32;j++)
				{	
					WRITE(0,0,j,0x22);  // ——0x22
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
	
	Tick = 0; //T6 中断次数，定时1ms中断
	net_state = 0;
	halt_Timeout = 1000; //(在线or离线状态下)默认采样时间间隔为1000*1ms = 1s

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

	InitTimer6();  // Timer6 提供1ms中断定时
	InitTimer2();  // Timer2 提供输入捕捉时钟基准
	InitIC();
	InitADC();
	InitSPI();
	ds1302_init();
	InitSCI();

/*采集模块重启后，先获取之前的工作状态
若之前处于离线状态，则重启后继续保持离线状态
若之前处于休眠状态，则重启后继续保持休眠状态
若之前处于在线状态，且重启后网络未建立，则保持休眠状态
*/
	for(j=0;j<32;j++)  
	{
		lq[j] = READ(0,0,j);
	}	
	if((lq[2]==0x00)&&(lq[5]==0x00)&&(lq[12]==0xff)) // 离线模式
	{	
		Send_Enable = 0;
		Save_Enable = 1;
		Tran_Enable = 0;
		halt_Enable = 0;
		curr_addr[0] = 0;
		curr_addr[1] = 32;	//一开始向FRAM中存放采集数据从地址 0x00020 开始
			
		}
	if((lq[2]==0x55)&&(lq[5]==0x00)&&(lq[12]==0x00)) //休眠
	{
		Send_Enable = 0;
		Save_Enable = 0;
		Tran_Enable = 0;
		halt_Enable = 1;
	}
			
	while(1)
	{			
		if(Send_Enable)  //在线，通过zigbee向上位机传送采集数据
						 //格式为 S ID CHx 频率高八位 低八位 温度高八位 低八位…… 电量高八位 低八位 E ，共35字节
		{
			if(net_state==1)//组网成功，向zigbee发送10组35字节的数据
			{
				for(i=0;i<32;i++) //读取FRAM中存储的指令，格式为15字节指令+17个0x22，共32字节
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
				}	//10条虚拟指令			
				net_state=0;
			}
				
			i = 1000;
			while(i>=200)
			{
				for(k=0;k<3;k++)
				{
            		j = i;
            		CEB = 1; //RG1输出为高
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
            		CEB = 0; //RG1输出为低  
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
				
/*将6个通道的传感器热敏电阻返回的电压量进行AD转化，并存在于temp[]中 */
		
			AD1CHS0bits.CH0SA = 9; //AN9
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


			//将采集数据通过zigbee发送给上位机
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
			//将采集数据通过uart2发送给小助手显示
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
			//（在线or离线模式下）控制采集时间间隔		
			halt = 1;
			halt_Tick = 0;
			StartTimer6();
    		while(halt){asm("NOP");} 
			
		}//if(Send_Enable)
	 
		if(Save_Enable)  //离线，向FRAM中保存数据
						 //保存的数据格式为 S 年 月 日 时 分 秒 频率高八位 低八位 温度高八位 低八位…… E ，32字节
		{
				
			if(net_state==1)//组网成功，向zigbee发送10组35字节的数据
			{
				for(i=0;i<32;i++) //读取FRAM中存储的指令，格式为15字节指令+17个0x22，共32字节
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
				}	//10条虚拟指令
			
				net_state=0;
			}
			
			i = 1000;
			while(i>=200)
			{
				for(k=0;k<3;k++)
				{
            		j = i;
            		CEB = 1; //RG1输出为高
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
            		CEB = 0; //RG1输出为低  
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
				
			//将6个通道的传感器热敏电阻返回的电压量进行AD转化，并存在于temp[]中 		
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
    		while(halt){asm("NOP");} //（在线or离线模式下）控制采集时间间隔
		}//if(Save_Enable)
			 
		if(Tran_Enable)  //回读，读取将FRAM中的数据
		{
			Tran_Data_U1(); //向上位机传送FRAM中的数据*******************************************
		//	Tran_Data_U2(); //向小助手传送FRAM中的数据
			Tran_Enable = 0; //传送结束后，将传送标致清零，确保只传送一次
		}
		if(halt_Enable)  //休眠，但要显示电量
		{	
			if(net_state==1)//组网成功，向zigbee发送10组35字节的数据
			{
				for(i=0;i<32;i++) //读取FRAM中存储的指令，格式为15字节指令+17个0x22，共32字节
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
				}	//10条虚拟指令
			
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

/*u1 串口向上位机传输FRAM数据*/
void Tran_Data_U1(void) // *u1 串口向上位机传输FRAM数据*
{
	unsigned char temp,i;
//	unsigned int j;
	curr_addr[0] = 0;
	curr_addr[1] = 32;

	//开始传送文件头，格式为 S T 0x00....0x00 E 总共35个字节，中间有32个0x00
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

	read_count = 4096; //从FRAM读取read_count组数据
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
		
	// 发送结束符，格式为 S T 0xff...0xff E 中间有32个0xff
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

/*u1 串口向上位机传输FRAM中存储的指令*/
void read_command_15_U1(void) // 
{
	unsigned char j,temp;
	//将之前存在fram 00000~0001f中的指令发送出来
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


void Tran_Data_U2(void) // *u2 串口向上位机(小助手)传输FRAM数据*
{
	unsigned char temp,i;
//	unsigned int j;
	curr_addr[0] = 0;
	curr_addr[1] = 32; //每次读取FRAM中的数据都从地址 0x00020 开始

	//开始传送文件头，格式为 S T 0x00....0x00 E 总共35个字节，中间有32个0x00
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
		
	read_count = 10; //从FRAM读取read_count组数据
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
		
	// 发送结束符，格式为 S T 0xff...0xff E 中间有32个0xff
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


void read_command_15_U2(void) // *u2 串口向上位机传输FRAM中存储的指令*
{
	unsigned char j,temp;
	
	//将之前存在fram 00000~0001f中的指令发送出来
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



