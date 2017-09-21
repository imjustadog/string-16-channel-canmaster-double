#if defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24hxxxx.h>
#endif
#include "timer.h"
#include "adc.h"
#include "sci.h"
 
 
 
#include "ecan.h"
#include "collect.h"

#include "stdio.h"

_FBS(0xCF);
_FSS(0xCF);
_FGS(0x07);
_FOSCSEL(0xA2);  //Select HS without PLL
_FOSC(0x46);  // HS oscilator, OSC2 is clock output
_FWDT(0xDF);
_FPOR(0xE7);
//_FICD(0xC3);
#define CLRWDT {__asm__ volatile ("CLRWDT");}




/****
四大指示灯
**/

#define COMM LATCbits.LATC14
#define WORK LATCbits.LATC13
#define STAT LATDbits.LATD0
#define FAIL LATDbits.LATD11

#define SWITCH1 PORTBbits.RB2
#define SWITCH2 PORTBbits.RB3
#define SWITCH3 PORTBbits.RB4
#define SWITCH4 PORTBbits.RB5
 
 /**
 网口复位引脚
 
 ****/

#define Nrest LATFbits.LATF6
 
/****
MIAN_ID与IP地址的最后两位对应
**********/

unsigned char MAIN_ID = 5;
#define BOARD_NUM 2 // Number of board

//ooo1234567

int count_rx = 0; 



unsigned char send_data[200]={1,2,3,4,5,6,7,8,9,10};
char send_ascii[250];

char flag_ascii_or_bin = 'b';

unsigned int freq[BOARD_NUM][8][5];  			  
unsigned int temp[BOARD_NUM][8][5];
unsigned int freq_temp = 0;
  
int Tick_40S=0; // 用来记录定时中断次数
 
unsigned char work_enable = 0;//采集模块工作使能位
unsigned char uart2_enable = 0;//串口使能位
unsigned char uart1_enable = 0;//网口使能位
 
unsigned char Slave_Message_Count[BOARD_NUM]; // 记录各从板返回的数据个数
 
//unsigned char state;

mID canTxMessage;
mID canRxMessage;
ECAN1MSGBUF ecan1msgBuf __attribute__((space(dma),aligned(ECAN1_MSG_BUF_LENGTH*16)));

 

void DELAY(unsigned int t)
{
	unsigned int i,j;
	
  	for(i=0;i<50;i++)
  	{
	  		CLRWDT
	  	for(j=0;j<t;j++)
   		{
	   		asm("nop");
	   		
    	}
   } 	
 } 
void UART2_Send(unsigned char str[], int len)
{
	int i;
	for(i=0;i<len;i++)
	{
		U2TXREG = str[i];
		while((U2STA&0x0100)==0x0000){asm("nop");}
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


void __attribute__((interrupt,no_auto_psv)) _T6Interrupt(void)  // 0.8s interrupt
{
	IFS2bits.T6IF = 0;
	FAIL = ~FAIL;
	WORK = FAIL;
	Tick_40S++;//加一次是0.8s
	if(Tick_40S > 2714) //36min
	{
	    Nrest=1;	
		Tick_40S = 0;
	}
	else if(Tick_40S == 2712)
	{
		Nrest=0;
	}
	else if(Tick_40S == 2363) //31.5min
	{
		InitSCI();
		work_enable = 1;
		uart2_enable = 1;
		uart1_enable = 1;				
	}
}

/******
网口中断


**********/

void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF = 0;
	unsigned char data[17];
	unsigned int UART_Timeout = 0;
	unsigned char i = 0;
	int nian,yue,ri,shi,fen,miao;
	//unsigned int num;
	data[0] = U1RXREG;
			 
	while(data[i]!='E')
	{
		i++;
		while((0==(U1STA&0x0001))&&(UART_Timeout<50000))
		{UART_Timeout++;}
		if(UART_Timeout>=50000)
		{
			data[i]='E'; // 如果接收超时，退出中断
		}
		else
		{
			data[i] = U1RXREG;
		}
		UART_Timeout = 0;
	}
	
	if( (i==16)&&(data[2]==0X03)&&(data[3]==0X04)&&(data[0]=='S') )
	{	
		    flag_ascii_or_bin = 'b';
			work_enable = 1;//握手指令中的ID与箱号一致时，才会开启该采集箱激频拾频
			uart1_enable =1;
			Tick_40S=0;		
	}//if(i==16)	
	
	return;	
}//





/******
外部串口中断


**********/
int start_judge = 0;

void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void)
{
	unsigned int UART_Timeout = 0;
	unsigned char i = 0;
	unsigned char dat;
	unsigned char receive_buf[60];
	
	IFS1bits.U2RXIF = 0;
	dat = U2RXREG;
	
		if((start_judge == 0) && (dat == 'S'))
	    {
			start_judge = 1;
			receive_buf[0] = dat;
	    }
		else if((start_judge == 0) && (dat == 'C'))
	    {
			start_judge = 3;
	    }
	    else if(start_judge == 1)
		{
			while(receive_buf[i]!='E')
			{
				if(i == 0)
				{
					i = 1;
					receive_buf[i] = dat;
				}	
				i++;
				while((0==(U2STA&0x0001))&&(UART_Timeout<50000))
				{UART_Timeout++;}
				if(UART_Timeout>=50000)
				{
					receive_buf[i]='E'; // 如果接收超时，退出中断
				}
				else
				{
					receive_buf[i] = U2RXREG;
				}
				UART_Timeout = 0;
			}	
			start_judge = 0; 
			if((i==4)&&(receive_buf[1]==0X88)&&(receive_buf[2]==0X88)) 
			{	
				work_enable = 1;//握手指令中的ID与箱号一致时，才会开启该采集箱激频拾频
				uart2_enable =1;
				flag_ascii_or_bin = 'b';
				count_rx = 0;	
				Tick_40S = 0;
			}
		}
		
		else if((start_judge == 0) && (dat == '5'))
	    {
			start_judge = 2;
			receive_buf[0] = dat;
	    }
	    else if(start_judge == 2)
		{
			while((receive_buf[i] != '5') || (i == 0))
			{
				if(i == 0)
				{
					i = 1;
					receive_buf[i] = dat;
				}
				i++;
				while((0==(U2STA&0x0001))&&(UART_Timeout<50000))
				{UART_Timeout++;}
				if(UART_Timeout>=50000)
				{
					receive_buf[i]='5'; // 如果接收超时，退出中断
				}
				else
				{
					receive_buf[i] = U2RXREG;
				}
				UART_Timeout = 0;
			}	
			start_judge = 0; 
			if((i==49)&&(receive_buf[7]=='1')&&(receive_buf[10]=='2')) 
			{	
				work_enable = 1;//握手指令中的ID与箱号一致时，才会开启该采集箱激频拾频
				uart2_enable =1;
				flag_ascii_or_bin = 'a';
                count_rx = 0;	
			}
		}
		else if(start_judge == 3)
		{
			start_judge = 0;
			if(dat == 'Q')
			{
				IEC1bits.U2RXIE = 0; // Enable UART2 RX interrupt
	            IEC1bits.U2TXIE = 0;
				IEC0bits.U1RXIE = 0; //  Enable UART1 RX interrupt
				IEC0bits.U1TXIE = 0;
				IEC2bits.T6IE = 0;
				IEC2bits.C1IE=0;
				C1INTEbits.RBIE=0;
				while(1); //饿狗，让狗来重启
			}
		}
		
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
unsigned char CAN_FLAG=0;
int main()
{
	int p;
	int i = 1000; // 扫频频率 473Hz~7.042kHz
	int k,j,q;
	int n,m,w,v,x;
	int s=0;
	int temp1;
	unsigned int mid_temp;;
    unsigned int humi_val_i,temp_val_i;
	unsigned char error,checksum;
    unsigned char read_temp;
    CLRWDT
	OSCCON = 0x2200;
 
 	TRISCbits.TRISC13 = 0;//LIGNT
 	TRISCbits.TRISC14 = 0; 
	TRISDbits.TRISD11 = 0; 
	TRISDbits.TRISD0  = 0; 
    
    TRISBbits.TRISB2  = 1; //switch
	TRISBbits.TRISB3  = 1; 
	TRISBbits.TRISB4  = 1; 
	TRISBbits.TRISB5  = 1; 
 
 
 	TRISFbits.TRISF6 = 0; //interet
 
 
 
	TRISFbits.TRISF1 = 0;//CAN
	TRISFbits.TRISF0 = 1; 
	

	s_connectionreset();
   /*temp hr   */
	
	AD1PCFGLbits.PCFG5 = 1;//端口引脚处于数字模式，使能端口读输入
	AD1PCFGLbits.PCFG4 = 1;
	AD2PCFGLbits.PCFG5 = 1;//端口引脚处于数字模式，使能端口读输入
	AD2PCFGLbits.PCFG4 = 1;
	

    WORK=1;//COMMM1
    STAT=0;
    COMM=0;//COMMM1
    FAIL=1;
    Nrest=1;//
	InitTimer6();  //// Timer6 提供0.8s中断定时
    StartTimer6();
	InitSCI();

	initECAN();
	initDMAECAN();
	IEC2bits.C1IE=1;
	C1INTEbits.RBIE=1;
	CLRWDT

 	DELAY(200);  // 600~9ms

    //MAIN_ID = SWITCH1 + (SWITCH2<<1) + (SWITCH3<<2) + (SWITCH4<<3);    
	//sprintf(initial,"main id is %d",MAIN_ID);

	while(1)
	{
		CLRWDT
		
		if((U1STA & 0x000E) != 0x0000)
		{
			read_temp = U1RXREG;
			U1STAbits.OERR = 0;
        }
		
		if((U2STA & 0x000E) != 0x0000)
		{
			read_temp = U2RXREG;
			U2STAbits.OERR = 0;
        }
		
		if(work_enable == 1) 
		{
			COMM=1;

			for(w = 0; w < 5; w ++)
			{
	 
				CLRWDT
               			
				canTxMessage.message_type=CAN_MSG_DATA;		
				canTxMessage.frame_type=CAN_FRAME_STD;
				canTxMessage.buffer=0;
				canTxMessage.id=0X01;
				canTxMessage.data[0]=0x01;
				canTxMessage.data[1]=0x02;
				canTxMessage.data[2]=0x03;
				canTxMessage.data[3]=0x04;
				canTxMessage.data[4]=0x05;
				canTxMessage.data[5]=0x06;
				canTxMessage.data[6]=0x07;
				canTxMessage.data[7]=0x08;
				canTxMessage.data_length=8;
				sendECAN(&canTxMessage);//发送报文，指示第n+1块板子返回采集值
			

             	DELAY(60000);  // 600~9ms	
             	DELAY(60000);  // 600~9ms	


				for(n=0;n<BOARD_NUM;n++)//获取slave的采集数据
				{	  
				    DELAY(800);  // 600~9ms
					CLRWDT
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
					sendECAN(&canTxMessage);//发送报文，指示第n+1块板子返回采集值
					
					Slave_Message_Count[n] = 0;
					while(Slave_Message_Count[n]<4)  // 每个从板需要回复2帧
					{
						if(canRxMessage.buffer_status==CAN_BUF_FULL)
						{
							rxECAN(&canRxMessage);
							canRxMessage.buffer_status=CAN_BUF_EMPTY;
							Slave_Message_Count[n]++;
							if(canRxMessage.id == (0x001|(n+1)<<8))
							{
								freq[n][0][w] = ((unsigned int)(canRxMessage.data[0])<<8)+canRxMessage.data[1];
								temp[n][0][w] = ((unsigned int)(canRxMessage.data[2])<<8)+canRxMessage.data[3];
								freq[n][1][w] = ((unsigned int)(canRxMessage.data[4])<<8)+canRxMessage.data[5];
								temp[n][1][w] = ((unsigned int)(canRxMessage.data[6])<<8)+canRxMessage.data[7];
							}
							else if(canRxMessage.id == (0x002|(n+1)<<8))
							{
								freq[n][2][w] = ((unsigned int)(canRxMessage.data[0])<<8)+canRxMessage.data[1];
								temp[n][2][w] = ((unsigned int)(canRxMessage.data[2])<<8)+canRxMessage.data[3];
								freq[n][3][w] = ((unsigned int)(canRxMessage.data[4])<<8)+canRxMessage.data[5];
								temp[n][3][w] = ((unsigned int)(canRxMessage.data[6])<<8)+canRxMessage.data[7];
							}
							else if(canRxMessage.id == (0x003|(n+1)<<8))
							{
								freq[n][4][w] = ((unsigned int)(canRxMessage.data[0])<<8)+canRxMessage.data[1];
								temp[n][4][w] = ((unsigned int)(canRxMessage.data[2])<<8)+canRxMessage.data[3];
								freq[n][5][w] = ((unsigned int)(canRxMessage.data[4])<<8)+canRxMessage.data[5];
								temp[n][5][w] = ((unsigned int)(canRxMessage.data[6])<<8)+canRxMessage.data[7];
							}
							else if(canRxMessage.id == (0x004|(n+1)<<8))
							{
								freq[n][6][w] = ((unsigned int)(canRxMessage.data[0])<<8)+canRxMessage.data[1];
								temp[n][6][w] = ((unsigned int)(canRxMessage.data[2])<<8)+canRxMessage.data[3];
								freq[n][7][w] = ((unsigned int)(canRxMessage.data[4])<<8)+canRxMessage.data[5];
								temp[n][7][w] = ((unsigned int)(canRxMessage.data[6])<<8)+canRxMessage.data[7];
							}
						}
					}
	            	CAN_FLAG=1;
				}	
		        CLRWDT
			}
			
			for(n = 0;n < BOARD_NUM; n ++)
			{
				for(v = 0;v < 8;v ++)
				{
					for(w = 0;w < 4;w ++)
					{
						for(x = w + 1;x < 5;x ++)
						{
							if(freq[n][v][x] < freq[n][v][w])
							{
								freq_temp = freq[n][v][w];
								freq[n][v][w] = freq[n][v][x];
								freq[n][v][x] = freq_temp;
							}
 						}
					}
				}
			}

	        error=0;
	     	error+=s_measure((unsigned char*) &humi_val_i,&checksum,HUMI); 
	     	error+=s_measure((unsigned char*) &temp_val_i,&checksum,TEMP); 
	
			if(error!=0)
	        	s_connectionreset(); 
			else
			{ 
		    	error=0;
			}

			COMM=0;	
            STAT=~STAT;		
 	
			work_enable = 0;

			if((uart1_enable ==1) || (uart2_enable == 1))
			{
		    	for(q = 0;q < BOARD_NUM/2 ; q ++)
				{
		
					send_data[0]='S';
					send_data[1]=1;
					send_data[2]=2;	
			
					send_data[3] = 3;
					send_data[4] =4;
					send_data[5] = 5;
					send_data[6] = 6;
					send_data[7] = MAIN_ID + q;
			        s=8;
					for(m=0;m<2;m++)
					{   
		                send_data[s]=m;s++;
						for(n=0;n<8;n++)
						{
							send_data[s] = freq[m + q * 2][n][2]>>8;s++;
							send_data[s] = freq[m + q * 2][n][2];s++;
							send_data[s] = temp[m + q * 2][n][2]>>8;s++; 
						    send_data[s] = temp[m + q * 2][n][2];s++;					
			
						}
					}
		
					send_data[s] = 0x00;s++;
					send_data[s] = 0x00;s++;
		
					send_data[s] = humi_val_i>>8;s++;
					send_data[s] = humi_val_i;s++;
					
					send_data[s] = temp_val_i>>8;s++;
					send_data[s] =temp_val_i;s++;
					
					send_data[s]=  'E';s++;
					
					if(flag_ascii_or_bin == 'a') 
					{
						for(p = 0;p < s;p ++)
						{
							sprintf(&send_ascii[p * 2],"%02x",send_data[p]);
						}
						if(uart2_enable ==1)
						{
							UART2_Send(send_ascii,s * 2);
						}
						if(uart1_enable ==1)
		   	        	{
		   	        		UART1_Send(send_ascii,s * 2);
		   	        	}	
					}	
				    
					else
					{
						if(uart2_enable ==1)
						{
							UART2_Send(send_data,s);
						}
						if(uart1_enable ==1)
		   	        	{
		   	        		UART1_Send(send_data,s);
		   	        	}	
					}
		            DELAY(6000);
					CLRWDT
				}
				uart2_enable =0;
				uart1_enable =0;
			}

		}

	}
	
	return 0;
}

