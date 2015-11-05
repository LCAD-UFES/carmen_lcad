/*
    graphic LCD backpack code
        
        set up for the big LCD, 160x128
        
        10/16/08, everything works. Pixel, line, circle, set x, set y, print_char, clear_screen. All good.
        
        11/14/08, block erase, box, converted to ATMega168
        
        11/19/08 added reverse mode
        
        11/20/08 added backlight modulation
        
        11/21/08 Added changable baud rate, also added EEPROM settings for
                baudrate, reverse mode, spash screen on/off, and backlight
*/


#include <avr/io.h>
#include "rprintf.h"
#include <math.h>
#include <avr/interrupt.h>

#define FOSC 16000000// Clock Speed
//#define BAUD 115200
//#define MYUBRR FOSC/16/(BAUD/2)-1

#define WR 0        //PC0
#define RD 1        //PC1
#define CE 2        //PC2
#define CD 3        //PC3
#define HALT 4        //PC4
#define RST 5        //PC5

#define BL_EN 2        //PB2

#define X_ENDPOINT 159
#define Y_ENDPOINT 127

#define BPS 0
#define BACKLIGHT 1
#define SPLASH 2
#define REV 3


//Define functions
//======================
void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);

void ioinit(void);      //Initializes IO
void delay_ms(uint16_t x); //General purpose delay
void delay_us(uint8_t x);
void USART_Init( unsigned int ubrr);
void set_baud(char b);
void put_char(char byte);
int rnd(float number);

void set_data(char data);//sets the data port
char read(char D_S);//reads data (D_S = 1) or status (D_S = anything else)
void write(char D_C, char byte);//writes data or command
void display_init(void);//initializes the display
void set_backlight(unsigned char dutycycle);


void clear_screen(void);

void print_char(char S_R, char txt);
void del_char(char endpoint);
void pixel(char S_R, char x, char y);
void line(char S_R, char x1, char y1, char x2, char y2);
void circle(char S_R, int x, int y, int r);
void demo(void);

void erase_block(char x1, char y1, char x2, char y2);
void box(char x1, char y1, char x2, char y2);

//======================
char x_offset = 0;
char y_offset = 127;
char reverse = 0;
char baud_rate = 6;//115200 by default

unsigned char RX_array[416];
volatile unsigned short RX_in = 0;
unsigned short RX_read = 0;
unsigned char BL_dutycycle = 100;
unsigned char splash_screen = 1;

static char logo[30] = {0x01,0xC0,0x03,0x80,0x03,0x80,0x01,0xD0,
                                                0x01,0xF8,0x0C,0xF8,0x18,0xF8,0x1F,0xF8,
                                                0x1F,0xF8,0x1F,0xF0,0x1F,0xE0,0x1F,0xC0,
                                                0x1C,0x00,0x18,0x00,0x10,0x00};

//Jacked from Sinister 7 code
static char text_array[475] = {0x00,0x00,0x00,0x00,0x00,/*space*/
                              0x00,0xF6,0xF6,0x00,0x00,/*!*/
                              0x00,0xE0,0x00,0xE0,0x00,/*"*/
                              0x28,0xFE,0x28,0xFE,0x28,/*#*/
                              0x00,0x64,0xD6,0x54,0x08,/*$*/
                              0xC2,0xCC,0x10,0x26,0xC6,/*%*/
                              0x4C,0xB2,0x92,0x6C,0x0A,/*&*/
                              0x00,0x00,0xE0,0x00,0x00,/*'*/
                              0x00,0x38,0x44,0x82,0x00,/*(*/
                              0x00,0x82,0x44,0x38,0x00,/*)*/
                              0x88,0x50,0xF8,0x50,0x88,/***/
                              0x08,0x08,0x3E,0x08,0x08,/*+*/
                              0x00,0x00,0x05,0x06,0x00,/*,*/
                              0x08,0x08,0x08,0x08,0x08,/*-*/
                              0x00,0x00,0x06,0x06,0x00,/*.*/
                              0x02,0x0C,0x10,0x60,0x80,/*/*/
                              0x7C,0x8A,0x92,0xA2,0x7C,/*0*/
                              0x00,0x42,0xFE,0x02,0x00,/*1*/
                              0x42,0x86,0x8A,0x92,0x62,/*2*/
                              0x44,0x82,0x92,0x92,0x6C,/*3*/
                              0x10,0x30,0x50,0xFE,0x10,/*4*/
                              0xE4,0xA2,0xA2,0xA2,0x9C,/*5*/
                              0x3C,0x52,0x92,0x92,0x0C,/*6*/
                              0x80,0x86,0x98,0xE0,0x80,/*7*/
                              0x6C,0x92,0x92,0x92,0x6C,/*8*/
                              0x60,0x92,0x92,0x94,0x78,/*9*/
                              0x00,0x00,0x36,0x36,0x00,/*:*/
                              0x00,0x00,0x35,0x36,0x00,/*;*/
                              0x10,0x28,0x44,0x82,0x00,/*<*/
                              0x28,0x28,0x28,0x28,0x28,/*=*/
                              0x00,0x82,0x44,0x28,0x10,/*>*/
                              0x40,0x80,0x8A,0x90,0x60,/*?*/
                              0x7C,0x82,0xBA,0xBA,0x62,/*@*/
                              0x3E,0x48,0x88,0x48,0x3E,/*A*/
                              0xFE,0x92,0x92,0x92,0x6C,/*B*/
                              0x7C,0x82,0x82,0x82,0x44,/*C*/
                              0xFE,0x82,0x82,0x82,0x7C,/*D*/
                              0xFE,0x92,0x92,0x92,0x82,/*E*/
                              0xFE,0x90,0x90,0x90,0x80,/*F*/
                              0x7C,0x82,0x82,0x8A,0x4E,/*G*/
                              0xFE,0x10,0x10,0x10,0xFE,/*H*/
                              0x82,0x82,0xFE,0x82,0x82,/*I*/
                              0x84,0x82,0xFC,0x80,0x80,/*J*/
                              0xFE,0x10,0x28,0x44,0x82,/*K*/
                              0xFE,0x02,0x02,0x02,0x02,/*L*/
                              0xFE,0x40,0x20,0x40,0xFE,/*M*/
                              0xFE,0x60,0x10,0x0C,0xFE,/*N*/
                              0x7C,0x82,0x82,0x82,0x7C,/*O*/
                              0xFE,0x90,0x90,0x90,0x60,/*P*/
                              0x7C,0x82,0x82,0x86,0x7E,/*Q*/
                              0xFE,0x90,0x98,0x94,0x62,/*R*/
                              0x64,0x92,0x92,0x92,0x4C,/*S*/
                              0x80,0x80,0xFE,0x80,0x80,/*T*/
                              0xFC,0x02,0x02,0x02,0xFC,/*U*/
                              0xF8,0x04,0x02,0x04,0xF8,/*V*/
                              0xFC,0x02,0x0C,0x02,0xFC,/*W*/
                              0xC6,0x28,0x10,0x28,0xC6,/*X*/
                              0xC0,0x20,0x1E,0x20,0xC0,/*Y*/
                              0x86,0x8A,0x92,0xA2,0xC2,/*Z*/
                              0x00,0x00,0xFE,0x82,0x00,/*[*/
                              0x80,0x60,0x10,0x0C,0x02,/*this should be / */
                              0x80,0x60,0x10,0x0C,0x02,/*]*/
                              0x20,0x40,0x80,0x40,0x20,/*^*/
                              0x01,0x01,0x01,0x01,0x01,/*_*/
                              0x80,0x40,0x20,0x00,0x00,/*`*/
                              0x04,0x2A,0x2A,0x2A,0x1E,/*a*/
                              0xFE,0x12,0x22,0x22,0x1C,/*b*/
                              0x1C,0x22,0x22,0x22,0x14,/*c*/
                              0x1C,0x22,0x22,0x12,0xFE,/*d*/
                              0x1C,0x2A,0x2A,0x2A,0x18,/*e*/
                              0x10,0x7E,0x90,0x80,0x40,/*f*/
                              0x18,0x25,0x25,0x25,0x1E,/*g*/
                              0xFE,0x10,0x10,0x10,0x0E,/*h*/
                              0x00,0x12,0x5E,0x02,0x00,/*i*/
                              0x02,0x01,0x01,0x11,0x5E,/*j*/
                              0xFE,0x08,0x08,0x14,0x22,/*k*/
                              0x00,0x82,0xFE,0x02,0x00,/*l*/
                              0x3E,0x20,0x1C,0x20,0x1E,/*m*/
                              0x3E,0x20,0x20,0x20,0x1E,/*n*/
                              0x1C,0x22,0x22,0x22,0x1C,/*o*/
                              0x3F,0x24,0x24,0x24,0x18,/*p*/
                              0x18,0x24,0x24,0x3F,0x01,/*q*/
                              0x3E,0x10,0x20,0x20,0x10,/*r*/
                              0x12,0x2A,0x2A,0x2A,0x04,/*s*/
                              0x00,0x10,0x3C,0x12,0x04,/*t*/
                              0x3C,0x02,0x02,0x02,0x3E,/*u*/
                              0x30,0x0C,0x02,0x0C,0x30,/*v*/
                              0x38,0x06,0x18,0x06,0x38,/*w*/
                              0x22,0x14,0x08,0x14,0x22,/*x*/
                              0x38,0x05,0x05,0x05,0x3E,/*y*/
                              0x22,0x26,0x2A,0x32,0x22,/*z*/
                              0x00,0x10,0x6C,0x82,0x82,/*{*/
                              //0x00,0x00,0xFF,0x00,0x00,/*|*/
                              0x04,0x02,0xFF,0x02,0x04,/*|, arrow*/
                              0x82,0x82,0x6C,0x10,0x00,/*}*/
                              0x08,0x10,0x18,0x08,0x10};/*~*/
                                                          
                                                          

ISR (SIG_USART_RECV)//USART Receive Interrupt
{
        cli();//Disable Interrupts
        RX_array[RX_in] = UDR0;
        
        RX_in++;
        
        if (RX_in >= 416) RX_in = 0;
        
        sei();//Enable Interrupts
        
}

ISR (TIMER0_COMPA_vect)
{
        unsigned char y;
        
        //cli();//Disable Interrupts
        
        TCCR0B = 0;
        TIMSK0 = 0;//disable timer ints
        
        TIFR0 = 0x02;//clear the interrupt
        TCNT0 = 0;
        
        //PORTB |= (1<<BL_EN);//off
        
        
        y = PINB;
        if (y & (1<<BL_EN))//on
        {
                PORTB &= (~(1<<BL_EN));
                OCR0A = BL_dutycycle;
        }
        
        else//off
        {
                PORTB |= (1<<BL_EN);
                OCR0A = 100 - BL_dutycycle;
        }
        
        //PORTB |= (1<<BL_EN);
        
        TIMSK0 = 0x02;//enable OCR0A
        TCCR0B = 0x02;
        //sei();//Enable Interrupts
}




int main (void)
{
        char x, y, temp, q;
        
    ioinit(); //Setup IO pins and defaults
        //USART_Init( MYUBRR);
        set_baud(6);//115200
        rprintf_devopen(put_char); // init rrprintf 
        
        //check for existing preset values==============================================================
        temp = EEPROM_read((unsigned int)BPS);
        
        if ((temp < 1) | (temp > 6))//BPS will only be 1-6
        {
                cli();//Disable Interrupts
                
                EEPROM_write((unsigned int) BPS, 6);
                EEPROM_write((unsigned int) BACKLIGHT, 100);
                EEPROM_write((unsigned int) SPLASH, 1);
                EEPROM_write((unsigned int) REV, 0);
                
                sei();//Enable Interrupts
                
                BL_dutycycle = 100;
                baud_rate = 6;
                splash_screen = 1;
                reverse = 0;
        }
        
        else
        {
                baud_rate = temp;
                BL_dutycycle = EEPROM_read((unsigned int)BACKLIGHT);
                splash_screen = EEPROM_read((unsigned int)SPLASH);
                reverse = EEPROM_read((unsigned int)REV);
        }
        
        //reset the display
        delay_ms(1);
        PORTC |= (1<<RST);
        
        //initialize the display
        display_init();
        
        set_backlight(BL_dutycycle);

        clear_screen();
        
        //Backlight on
        PORTB &= (~(1<<BL_EN));
        
        //Logo==========================================================
        if (splash_screen == 1)
        {
                y = 72;
                
                for (q = 0; q < 30; q++)
                {
                        temp = logo[q];
                        for (x = 72; x < 80; x++)
                        {
                                if (temp & 0x80) pixel(1,x,y);
                                
                                temp <<= 1;
                        }
                        
                        q++;
                        
                        temp = logo[q];
                        for (x = 80; x < 88; x++)
                        {
                                if (temp & 0x80) pixel(1,x,y);
                                
                                temp <<= 1;
                        }
                        y--;
        
                }        
        }        
        
        RX_in = 0;
        
        delay_ms(1000);
        clear_screen();
        
        if (RX_in > 0)//revert to 115200
        {
                print_char(1,'1');
                print_char(1,'1');
                print_char(1,'5');
                print_char(1,'2');
                print_char(1,'0');
                print_char(1,'0');
                
                baud_rate = 6;
                set_baud(6);//115200
                
                cli();
                
                EEPROM_write((unsigned int) BPS, 6);
                
                sei();//Enable Interrupts
        }
        
        else (set_baud(baud_rate));
        
        delay_ms(1000);
        clear_screen();
        
        //main loop===================================================
        while(1)
        {
                if(RX_in != RX_read)
                {
                        x = RX_array[RX_read];
                        RX_read++;
                        if(RX_read >= 416) RX_read = 0;
                        
                        //Backspace===================================================
                        if(x == 8) del_char(0);
                        
                        //Special commands
                        else if (x == 124)
                        {        
                                //make sure the next byte is there
                                while(RX_in == RX_read);
                                
                                //0, clear screen======================================================
                                if(RX_array[RX_read] == 0)//^@
                                {
                                        clear_screen();
                                        RX_read++;
                                        if(RX_read >= 416) RX_read = 0;
                                }
                                
                                //demo mode
                                else if(RX_array[RX_read] == 4)//^d
                                {
                                        RX_in = 0, RX_read = 0;
                                        demo();
                                        clear_screen();
                                        RX_in = 0;
                                }
                                
                                //reverse mode
                                else if(RX_array[RX_read] == 18)//^r
                                {
                                        reverse ^= 1;
                                        clear_screen();
                                        RX_read++;
                                        if(RX_read >= 416) RX_read = 0;
                                        
                                        cli();
                                        EEPROM_write((unsigned int) REV, reverse);
                                        sei();
                                }
                                
                                //toggle spasl screen
                                else if(RX_array[RX_read] == 19)//^s
                                {
                                        splash_screen ^= 1;
                                        //clear_screen();
                                        RX_read++;
                                        if(RX_read >= 416) RX_read = 0;
                                        
                                        cli();
                                        EEPROM_write((unsigned int) SPLASH, splash_screen);
                                        sei();
                                }
                                
                                else
                                {
                                        //set backlight (0 to 100)=========================================================
                                        if(RX_array[RX_read] == 2)//^b
                                        {
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;
                                                while(RX_in == RX_read);//wait for byte
                                                BL_dutycycle = RX_array[RX_read];
                                                
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;
                                                
                                                set_backlight(BL_dutycycle);
                                                
                                                cli();
                                                EEPROM_write((unsigned int) BACKLIGHT, BL_dutycycle);
                                                sei();
                                                
                                                /*
                                                if(BL_dutycycle >= 100)
                                                {
                                                        TCCR0B = 0;
                                                        TIMSK0 = 0;//disable timer ints
                                                        
                                                        //Backlight on
                                                        PORTB &= (~(1<<BL_EN));
                                                }
                                                else if (BL_dutycycle == 0)
                                                {
                                                        TCCR0B = 0;
                                                        TIMSK0 = 0;//disable timer ints
                                                        
                                                        //Backlight off
                                                        PORTB |= (1<<BL_EN);
                                                }
                                                        
                                                else
                                                {
                                                        TCCR0B = 0;
                                                        TIMSK0 = 0;//disable timer ints
                                                        
                                                        OCR0A = 100 - BL_dutycycle;
                                                        
                                                        TIMSK0 = 0x02;//enable match on A
                                                        TCCR0B = 0x02;
                                                        
                                                        SREG |= 0x80;
                                                        
                                                }
                                                */

                                        }
                                        
                                        
                                        //change baud rate=========================================================
                                        if(RX_array[RX_read] == 7)//^g
                                        {
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;
                                                while(RX_in == RX_read);//wait for byte
                                                //if (RX_array[RX_read] == '1') USART_Init( 1000000/2400-1);//4800
                                                //else if (RX_array[RX_read] == '2') USART_Init( 1000000/4800-1);//9600
                                                //else if (RX_array[RX_read] == '3') USART_Init( 1000000/9600-1);//19200
                                                //else if (RX_array[RX_read] == '4') USART_Init( 1000000/19200-1);//38400
                                                //else if (RX_array[RX_read] == '5') USART_Init( 1000000/28800-1);//57600
                                                //else if (RX_array[RX_read] == '6') USART_Init( 1000000/57600-1);//115200
                                                
                                                if ((RX_array[RX_read] > '0') * (RX_array[RX_read] < '7')) baud_rate = (RX_array[RX_read]) - 48;
                                                
                                                set_baud(baud_rate);
                                                
                                                cli();
                                                EEPROM_write((unsigned int) BPS, baud_rate);
                                                sei();
                                                
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;
                                                
                                        }        
                                        
                                        
                                        //set x or y=========================================================
                                        if((RX_array[RX_read] == 24) | (RX_array[RX_read] == 25))//^x or ^y
                                        {
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;
                                                while(RX_in == RX_read);//wait for byte
                                                if (RX_array[RX_read-1] == 24) x_offset = RX_array[RX_read];
                                                else if (RX_array[RX_read-1] == 25) y_offset = RX_array[RX_read];
                                                
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;
                                                
                                                if (x_offset > 159) x_offset = 159;
                                                if (y_offset > 127) y_offset = 127;

                                        }

                                        //set pixel=========================================================
                                        if (RX_array[RX_read] == 16)//^p
                                        {
                                                //need 3 bytes
                                                for (y = 0; y < 3; y++)
                                                {
                                                        RX_read++;
                                                        if(RX_read >= 416) RX_read = 0;
                                                        while(RX_in == RX_read);//wait for byte
                                                }
                                                
                                                pixel(RX_array[RX_read], RX_array[RX_read-2], RX_array[RX_read-1]);
                                                
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;

                                        }

                                        
                                        //<ctrl>c, circle======================================================
                                        if(RX_array[RX_read] == 3)//^c
                                        {
                                                //need 4 bytes
                                                for (y = 0; y < 4; y++)
                                                {
                                                        RX_read++;
                                                        if(RX_read >= 416) RX_read = 0;
                                                        while(RX_in == RX_read);//wait for byte
                                                }
                                                
                                                circle(RX_array[RX_read], RX_array[RX_read-3], RX_array[RX_read-2], RX_array[RX_read-1]);
                                                
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;
                                        }
                                        
                                        
                                        //<ctrl>e, erase block======================================================
                                        if(RX_array[RX_read] == 5)//^e
                                        {
                                                //need 4 bytes
                                                for (y = 0; y < 4; y++)
                                                {
                                                        RX_read++;
                                                        if(RX_read >= 416) RX_read = 0;
                                                        while(RX_in == RX_read);//wait for byte
                                                }
                                                
                                                erase_block(RX_array[RX_read-3], RX_array[RX_read-2], RX_array[RX_read-1], RX_array[RX_read]);
                                                
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;
                                        }
                                        
                                        
                                        //box======================================================
                                        if(RX_array[RX_read] == 15)//^o
                                        {
                                                //need 4 bytes
                                                for (y = 0; y < 4; y++)
                                                {
                                                        RX_read++;
                                                        if(RX_read >= 416) RX_read = 0;
                                                        while(RX_in == RX_read);//wait for byte
                                                }
                                                
                                                box(RX_array[RX_read-3], RX_array[RX_read-2], RX_array[RX_read-1], RX_array[RX_read]);
                                                
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;
                                        }


                                        //line========================================================
                                        else if (RX_array[RX_read] == 12)//^l
                                        {
                                                //need 5 bytes
                                                for (y = 0; y < 5; y++)
                                                {
                                                        RX_read++;
                                                        if(RX_read >= 416) RX_read = 0;
                                                        while(RX_in == RX_read);//wait for byte
                                                }
                                                
                                                line(RX_array[RX_read], RX_array[RX_read-4], RX_array[RX_read-3], RX_array[RX_read-2], RX_array[RX_read+-1]);
                                                RX_read++;
                                                if(RX_read >= 416) RX_read = 0;
                                        }
                                        
                                        
                                }
        
                        }
                        
                        //print character to the screen===============================================
                        else
                        {
                                del_char(1);
                                print_char(1, x);
                        }
                }
                
        }
        
        

}

void ioinit (void)
{
        
    //1 = output, 0 = input
        
        /*
        WR        //PC0
        RD        //PC1
        CE        //PC2
        C_D //PC3
        HALT        //PC4
        RST        //PC5
        */

        PORTB |= (1<<BL_EN);//Backlight off
        DDRB |= (1<<BL_EN);//set PB2 as output
        
        PORTC = ((1<<WR) | (1<<RD) | (1<<CE) | (1<<CD) | (1<<HALT));
        PORTC &= (~(1<<RST));//set the reset line low at power up
        DDRC = ((1<<WR) | (1<<RD) | (1<<CE) | (1<<CD) | (1<<HALT) | (1<<RST));
        
        //Init timer 2
    TCCR2B = (1<<CS21); //Set Prescaler to 8. CS21=1

        //Set up Timer 0
        TCCR0A = 0x02;//CTC mode
        //TCCR0B = 0x02;
        TIMSK0 = 0x02;//enable OCR0A
        //OCR0B = 255 - BL_dutycycle;
        
        //OCR0A = 255 - (100 - BL_dutycycle);
        OCR0A = BL_dutycycle;
        
        //SREG |= 0x80;
}

//General short delays
void delay_ms(uint16_t x)
{
        for (; x > 0 ; x--)
    {
        delay_us(250);
        delay_us(250);
        delay_us(250);
        delay_us(250);
    }
        
}

//General short delays
void delay_us(uint8_t x)
{
        char temp;
        
        if (x == 0) temp = 1;
        else temp = x;
        
        TIFR2 |= 0x01;//Clear any interrupt flags on Timer2
    
    TCNT2 = 256 - temp; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click

        while(!(TIFR2 & 0x01));
        
        if (x == 0) return;//this is for display timing        
        
        //The prescaler doesn't allow for a setting of 16, just 8 or 32. So, we do this twice.
        TIFR2 |= 0x01;
    
    TCNT2 = 256 - temp; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click

        while(!(TIFR2 & 0x01));
        
}

void USART_Init( unsigned int ubrr)
{
        // Set baud rate 
        UBRR0H = (unsigned char)(ubrr>>8);
        UBRR0L = (unsigned char)ubrr;
        
        // Enable receiver and transmitter 
        UCSR0A = (1<<U2X0);
        UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);        //Enable Interrupts on receive character
        
        UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);
        sei();
}

void put_char(char byte)
{
        /* Wait for empty transmit buffer */
        while ( !( UCSR0A & (1<<UDRE0)) );
        /* Put data into buffer, sends the data */
        UDR0 = byte;
}


//set data port
void set_data(char data)
{
        //PORTB
        //DB0 = PB0
        //DB1 = PB1
        
        PORTB &= 0xFC;
        
        //PORTD
        //DB2 = PD2
        //DB3 = PD3
        //DB4 = PD4
        //DB5 = PD5
        //DB6 = PD6
        //DB7 = PD7
        
        PORTD &= 0x03;
        
        PORTB |= (data & 0x03);
        PORTD |= (data & 0xFC);

}


//Reads data or status
//for data D_S = 1, for status D_S = 0
//returns the value of the data bus
char read(char D_S)
{
        char data1 = 0, data2 = 0;
        
        DDRB &= 0xFC;//PB0 and PB1 inputs
        DDRD &= 0x02;//everything but PD1 as input
        
        PORTC &= ~((1 << RD) | (1 << CE));//CD high for status
        if (D_S == 1) PORTC &= ~(1 << CD);//CD down for data
        
        delay_us(0);
        
        data1 = PINB;
        data1 &= 0x03;
        
        data2 = PIND;
        data2 &= 0xFC;
        
        data1 |= data2;
        
        PORTC |= ((1 << CD) | (1 << RD) | (1 << CE));//all up
        
        delay_us(0);
        
        return data1;

}


//Writes data (D_C = 1) or command (D_C = anything else)
void write(char D_C, char byte)
{
        DDRB |= 0x03; //PB0 and PB1 are outs
        DDRD |= 0xFC; //PD2-PD7 are also outs.  Ports B and D are the data bus
        
        set_data(byte);
        
        if (D_C == 1) PORTC &= ~((1 << WR) | (1 << CE) | (1 << CD));//down
        else PORTC &= ~((1 << WR) | (1 << CE));//down
        
        delay_us(0);
        PORTC |= ((1 << CD) | (1 << WR) | (1 << CE));//all up
        delay_us(0);
        DDRB &= 0xFC;//PB0 and PB1 inputs
        DDRD &= 0x02;//everything but PD1 as input
        
        delay_us(0);

}



void display_init(void)
{
        //set graphics home address to 0
        while(!(read(0) & 3));//read status
        write(1, 0);
        while(!(read(0) & 3));//read status
        write(1, 0);
        while(!(read(0) & 3));//read status
        write(0, 0x42);

        //set graphics area
        while(!(read(0) & 3));//read status
        write(1, 20);//20 bytes, 160/8
        while(!(read(0) & 3));//read status
        write(1, 0);
        while(!(read(0) & 3));//read status
        write(0, 0x43);
        
        //set mode
        while(!(read(0) & 3));//read status
        write(0, 0x80);//Or, with internal character generator
        
        //set display mode
        while(!(read(0) & 3));//read status
        write(0, 0x98);//Graphics on

}





void clear_screen(void)
{
        int x;
        
        //set address pointer to 0, start of graphics
        while(!(read(0) & 3));//read status
        write(1, 0);
        while(!(read(0) & 3));//read status
        write(1, 0);
        while(!(read(0) & 3));//read status
        write(0, 0x24);
        
        for(x = 0; x < 0xA00; x++)
        {
                while(!(read(0) & 3));//read status
                if (reverse == 1) write(1,0xFF);
                else if (reverse == 0) write(1, 0);                
                while(!(read(0) & 3));//read status
                write(0, 0xC0);
        }
        
        x_offset = 0;
        y_offset = 127;
}


//sets (S_R = 1) or resets (S_R = 0) at x, y
void pixel(char S_R, char x, char y)
{
        short address = 0;
        char byte = 0;
        
        if (reverse == 1) S_R ^= 1;
                
        //don't try to print something outside of our range
        if (x > 159) return;
        if (y > 127) return;
        
        address = ((127-y) * 20) + (x / 8);
        
        //set address pointer
        while(!(read(0) & 3));//read status
        byte = (char)(address & 0xFF);

        write(1, byte);//20 bytes, 160/8
        
        while(!(read(0) & 3));//read status
        byte = (char)((address & 0xFF00) >> 8);

        write(1, byte);
        
        while(!(read(0) & 3));//read status
        write(0, 0x24);
        
        byte = ~(x % 8);

        byte |= 0xF8;
        if (S_R == 0) byte &= 0xF7;
        
        //if (reverse == 1) byte = ~(byte);
        
        //set-reset bit
        while(!(read(0) & 3));//read status
        write(0, byte);
        
}


//draws (S_R = 1) or erases (S_R = 0) a line from x1, y1 to x2, y2
void line(char S_R, char x1, char y1, char x2, char y2)
{
        float m, q;
        int x_dif, y_dif;
        int a, b, c;
        
        if ((x1 > X_ENDPOINT) | (x2 > X_ENDPOINT)) return;
        if ((y1 > Y_ENDPOINT) | (y2 > Y_ENDPOINT)) return;
        
        x_dif = x2 - x1;
        y_dif = y2 - y1;
        if (y_dif < 0) y_dif *= (-1);
        

        m = (float)(y2 - y1) / (float)(x2 - x1);
        
        b = y1-(m*x1);
        
        if(x_dif >= y_dif)
        {
                for (a = x1; a <= x2; a++)
                {
                        pixel(S_R, (char)a, (char)((m*a)+b));

                }
        }
        
        else
        {
                if (y2 > y1)
                {
                        for (a = y1; a <= y2; a++)
                        {
                                if (x_dif == 0) c = x1;
                                else
                                {
                                        q = (((float)(a-b))/m);
                                        c = rnd(q);
                                }
                                
                                pixel(S_R, (char)c, (char)a);

                        }
                }
                
                else if (y1 > y2)
                {
                        for (a = y1; a >= y2; a--)
                        {
                                if (x_dif == 0) c = x1;
                                else 
                                {
                                        q = (((float)(a-b))/m);
                                        c = rnd(q);
                                }
                        
                                pixel(S_R, (char)c, (char)a);

                        }
                }
        }
                
}


//draws (S_R = 1) or erases (S_R = 0) a circle ar x, y with radius r
void circle(char S_R, int x, int y, int r)
{
        int x1 = 0, x2 = 0;
        int x_line = 0, y_line = 0;
        int temp_y;
        int temp_x;
        
        x1 = x - r;
        x2 = x + r;
        
        for (temp_x = x1; temp_x <= x2; temp_x++)
        {
                temp_y = ((sqrt((r*r) - ((temp_x - x)*(temp_x - x)))) - y);
                
                temp_y *= (-1);
                
                if (temp_x > x1)
                {
                        line(S_R, (char)x_line, (char)y_line, (char)temp_x, (char)temp_y);
                        line(S_R, (char)x_line, (char)(2*y - y_line), (char)temp_x, (char)(2*y - temp_y));
                }
                        
                else 
                {
                        pixel(S_R, (char)temp_x, (char)temp_y);
                        pixel(S_R, (char)temp_x, (char)(y + y - temp_y));
                }
                
                x_line = temp_x;
                y_line = temp_y;
                
        }
        

}

//rounds a floar to the nearest int
int rnd(float number)
{
        int a;
        float b;
        
        a = number / 1;
        b = number - a;
        
        if (b >= 0.5) a++;
        
        return a;

}

//prints (S_R = 1) or erases (S_R = 0) a character to the screen
//at x_offset, y_offset. Automatically augments offsets for next write
void print_char(char S_R, char txt)
{
    short text_array_offset = (txt - 32)*5, j;
    char x, k;
        
        
    for (j = text_array_offset; j < text_array_offset+5; j++)
    {
                k = text_array[j];
                
                for (x = 0; x < 8; x++)
                {
                        if(k & 0x80) pixel(S_R, x_offset, y_offset - x);
                        k <<= 1;
                }
                        
                x_offset++;
                
    }
        
        x_offset++;
        
    if ((x_offset + 6) > 160)
        {
                x_offset = 0;
                if (y_offset <= 7)
                {
                        y_offset = 127;
                        //clear_screen();
                }
                else y_offset -= 8;
                
        }
        
}


//demonstration code
void demo(void)
{
        char x, y, temp;
        int q = 0;
        
        while(1)
        {        
                x_offset = 0;
                y_offset = 127;
        
                for (y = 0; y < 5; y++)
                {
                        for (x = 32; x < 123; x++)
                        {        
                                del_char(1);
                                print_char(1, x);
                                if (RX_in > 0) return;
                        }
                }
                
                clear_screen();
                
                for (y = 0; y < 5; y++)
                {
                        for (x = 32; x < 123; x++)
                        {
                                //x_offset += 4;
                                y_offset -= 6;
                                if (y_offset <= 8) y_offset = 127;
                                del_char(1);
                                print_char(1, x);
                                if (RX_in > 0) return;
                        }
                }
                
                clear_screen();
                
                //draw circles================================
                for (x = 5; x < 120; x += 5)
                {
                        circle(1,80,64,x);
                        if (RX_in > 0) return;
                }
                
                
                //draw lines===================================
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 20)
                {
                        line(1,0,y,x,0);
                        y -= 16;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 20)
                {
                        line(1,x,0,X_ENDPOINT,y);
                        y += 16;
                }
                
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 20)
                {
                        line(1,x,Y_ENDPOINT,X_ENDPOINT,y);
                        y -= 16;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 20)
                {
                        line(1,0,y,x,Y_ENDPOINT);
                        y += 16;
                }
                
                
                //erase circles================================
                for (x = 5; x < 120; x += 5)
                {
                        circle(0,80,64,x);
                        if (RX_in > 0) return;
                }

                //erase lines===================================
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 20)
                {
                        line(0,0,y,x,0);
                        y -= 16;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 20)
                {
                        line(0,x,0,X_ENDPOINT,y);
                        y += 16;
                }
                
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 20)
                {
                        line(0,x,Y_ENDPOINT,X_ENDPOINT,y);
                        y -= 16;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 20)
                {
                        line(0,0,y,x,Y_ENDPOINT);
                        y += 16;
                }
                
                if (RX_in > 0) return;
                
                //Boxes=================================================================
                y = 111;
                for (x = 0; x <= 140; x += 10)
                {
                        erase_block(x, y, x+16, y+16);
                        box(x, y, x+16, y+16);
                        y -= 7;
                }
                
                
                //x = 110;
                y = 28;
                //Logo=================================================================
                q = 0;
                while(q < 30)
                {
                        temp = logo[q];
                        for (x = 140; x < 148; x++)
                        {
                                if (temp & 0x80) pixel(1,x,y);
                                
                                temp <<= 1;
                        }
                        q++;
                        temp = logo[q];
                        for (x = 148; x < 156; x++)
                        {
                                if (temp & 0x80) pixel(1,x,y);
                                
                                temp <<= 1;
                        }
                        y--;
                        q++;
                }        
                
                delay_ms(1000);
                reverse ^= 1;
                clear_screen();
                
        }
        

}

//Deletes a full character space. Endpoint == 0 for a backwards delete,
//Endpoint != 0 to erase spot for a new character write
void del_char(char endpoint)
{
        char a, y;
        
        if (endpoint == 0)//Backwards delete
        {
                if (x_offset <= 5)
                {                        
                        x_offset += 120;
                        y_offset += 8;
                        
                        if (y_offset > 63) y_offset -= 64;
                }
                
                else x_offset -= 6;
        }
        
        for (a = x_offset; a < x_offset + 6; a++)
        {                                        
                for (y = y_offset - 7; y <= y_offset; y++)
                {
                        pixel(0, a, y);

                }
        }
                
}


//erases a block of the screen. Block is decribed
//by a diagonal line from x, y1 to x2, y2
void erase_block(char x1, char y1, char x2, char y2)
{
        static char temp_x = 0, temp_y = 0;
        
        for (temp_y = y2; temp_y >= y1; temp_y--)
        {
                for (temp_x = x1; temp_x <= x2; temp_x++)
                {
                        pixel(0, temp_x, temp_y);
                        
                }
        }        
        
        

}

//draws a box. The box is decribed
//by a diagonal line from x, y1 to x2, y2
void box(char x1, char y1, char x2, char y2)
{
        line(1, x1, y1, x1, y2);
        line(1, x1, y1, x2, y1);
        line(1, x2, y1, x2, y2);
        line(1, x1, y2, x2, y2);

}

void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
/* Wait for completion of previous write */
while(EECR & (1<<EEPE))
;
/* Set up address and Data Registers */
EEAR = uiAddress;
EEDR = ucData;
/* Write logical one to EEMPE */
EECR |= (1<<EEMPE);
/* Start eeprom write by setting EEPE */
EECR |= (1<<EEPE);
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
/* Wait for completion of previous write */
while(EECR & (1<<EEPE))
;
/* Set up address register */
EEAR = uiAddress;
/* Start eeprom read by writing EERE */
EECR |= (1<<EERE);
/* Return data from Data Register */
return EEDR;
}


void set_baud(char b)
{
        if (b == 1) USART_Init( 1000000/2400-1);//4800
        else if (b == 2) USART_Init( 1000000/4800-1);//9600
        else if (b == 3) USART_Init( 1000000/9600-1);//19200
        else if (b == 4) USART_Init( 1000000/19200-1);//38400
        else if (b == 5) USART_Init( 1000000/28800-1);//57600
        else if (b == 6) USART_Init( 1000000/57600-1);//115200
        
}


void set_backlight(unsigned char dutycycle)
{
        //Set up Timer 0
        TCCR0A = 0x02;//CTC mode
        //TCCR0B = 0x02;
        //TIMSK0 = 0x02;//enable OCR0A
        //OCR0B = 255 - BL_dutycycle;
        
        //OCR0A = 255 - (100 - BL_dutycycle);
        //OCR0A = dutycycle;
        
        //SREG |= 0x80;
        
        if(BL_dutycycle >= 100)
        {
                TCCR0B = 0;
                TIMSK0 = 0;//disable timer ints
                
                //Backlight on
                PORTB &= (~(1<<BL_EN));
        }
        else if (BL_dutycycle == 0)
        {
                TCCR0B = 0;
                TIMSK0 = 0;//disable timer ints
                
                //Backlight off
                PORTB |= (1<<BL_EN);
        }
                
        else
        {
                TCCR0B = 0;
                TIMSK0 = 0;//disable timer ints
                
                OCR0A = 100 - BL_dutycycle;
                
                TIMSK0 = 0x02;//enable match on A
                TCCR0B = 0x02;
                
                SREG |= 0x80;
                
        }

}



        
        

