#ifndef _collect_h
#define _collect_h
/*
#define DATA  PORTCbits.RC4
#define TRIS_DATA  TRISCbits.TRISC4
#define SCK  PORTCbits.RC3
#define TRIS_SCK  TRISCbits.TRISC3

/#define CEA LATGbits.LATG0
 #define TRISCBITS TRISCbits.TRISC13
 #define TRISCBITS TRISCbits.TRISC13
 #define TRISCBITS TRISCbits.TRISC13
 #define TRISCBITS TRISCbits.TRISC13

*/

//clk  RG2
//data  RG3

#define DATA  PORTGbits.RG3
#define TRIS_DATA  TRISGbits.TRISG3
#define SCK  PORTGbits.RG2
#define TRIS_SCK  TRISGbits.TRISG2
/*
#define DATA  LATGbits.LATG3
#define TRIS_DATA  TRISGbits.TRISG3
#define SCK  LATGbits.LATG2
#define TRIS_SCK  TRISGbits.TRISG2
*/


#define noACK 0
#define ACK 1
enum {TEMP,HUMI};



//adr command r/w
#define STATUS_REG_W 0x06 //000 0011 0
#define STATUS_REG_R 0x07 //000 0011 1
#define MEASURE_TEMP 0x03 //000 0001 1
#define MEASURE_HUMI 0x05 //000 0010 1
#define RESET 0x1e //000 1111 0

char s_write_byte(unsigned char value);
char s_read_byte(unsigned char ack);
void s_transstart(void);
void s_connectionreset(void);
char s_softreset(void);
char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum);
char s_write_statusreg(unsigned char *p_value);
char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode);
void calc_sth11(float *p_humidity ,float *p_temperature);

#endif