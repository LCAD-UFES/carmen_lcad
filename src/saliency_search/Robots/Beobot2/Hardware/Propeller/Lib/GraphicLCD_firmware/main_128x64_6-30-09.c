/*
    graphic LCD backpack test code
        
        10/7/08, text and lines work, need to set up UART and circle functions
        
        All functions work per the 160x128, seemingly all glitches gone
        Next step is to merge the 128x64 code (this code) with the 160x128 code...
        
*/

#include <avr/io.h>
#include "rprintf.h"
#include <math.h>
#include <avr/interrupt.h>


#define FOSC 16000000// Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#define EN 0        //PC0
#define RS 1        //PC1, D_I?
#define R_W 2        //PC2
#define RESET 3        //PC3
#define CS1 4        //PC4
#define CS2 5        //PC5

#define BL_EN 2        //PB2

#define X_ENDPOINT 127
#define Y_ENDPOINT 63

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
void delay(void);//display timing
void USART_Init( unsigned int ubrr);
void put_char(char byte);

void set_data(char data);
void set_x(char x_spot);
void set_page(char page);
void clear_screen(void);
void write_byte(char byte, char side);
char read_byte(char byte, char side);
void display_on(void);
void set_backlight(unsigned char dutycycle);

//unsigned char print_char(char txt);
void set_baud(char b);
void print_char(char S_R, char txt);
void del_char(char endpoint);
void pixel(char S_R, char x, char y);
void line(char S_R, char x1, char y1, char x2, char y2);
int rnd(float number);
void circle(char S_R, int x, int y, int r);
void demo(void);
void erase_block(char x1, char y1, char x2, char y2);
void box(char x1, char y1, char x2, char y2);

//======================


char x_offset = 0;
char y_offset = 63;
char side_mark = 1;//left side default
char page_mark = 0;
char baud_rate = 6;//115200 by default
char reverse = 0;

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
                              0x00,0x00,0x00,0x00,0x00,/*this should be / */
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
        rprintf_devopen(put_char); /* init rrprintf */
        
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
        
        
        //Reset the display
        PORTC &= ~(1 << RESET);
        delay_ms(50);
        PORTC |= (1 << RESET);
        //delay_ms(500);

        
        clear_screen();

        set_page(0);
        
        set_x(0);
        
        display_on();
        
        //set display start line to 0
        //set control lines
        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
        
        set_data(0xC0);
        //set_data(0xFF);
        delay();
        PORTC |= (1 << EN);//up
        delay();
        PORTC &= ~(1 << EN);//down
        delay();
        PORTC |= ((1 << EN) | (1 << R_W) | (1 << RS));//all high
        
        delay();
        
        x_offset = 0;

        set_page(0);
        
        DDRB |= (1<<BL_EN);//set PB2 as output
        
        set_backlight(BL_dutycycle);
        
        //Logo==========================================================
        if (splash_screen == 1)
        {
                y = 40;
                
                for (q = 0; q < 30; q++)
                {
                        temp = logo[q];
                        for (x = 56; x < 64; x++)
                        {
                                if (temp & 0x80) pixel(1,x,y);
                                
                                temp <<= 1;
                        }
                        
                        q++;
                        
                        temp = logo[q];
                        for (x = 64; x < 72; x++)
                        {
                                if (temp & 0x80) pixel(1,x,y);
                                
                                temp <<= 1;
                        }
                        y--;
        
                }        
        }
        
        pixel(0,0,0);//cheat
        
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
        
        //demo();
        


    
}

void ioinit (void)
{
        
    //1 = output, 0 = input
   
        DDRB = 0b00000011; //PB0 and PB1 are outs
        
        PORTB |= (1<<BL_EN);//Backlight off
        DDRB |= (1<<BL_EN);//set PB2 as output
        
        DDRC = ((1<<EN) | (1<<RS) | (1<<R_W) | (1<<RESET) | (1<<CS1) | (1<<CS2));
        PORTC = ((1<<EN) | (1<<RS) | (1<<R_W) | (1<<RESET) | (1<<CS1) | (1<<CS2));
        DDRD = 0b11111100; //PD2-PD7 are also outs.  Ports B and D are the data bus.
        
        //Init timer 2
    TCCR2B = (1<<CS21); //Set Prescaler to 8. CS21=1

        //Set up Timer 0
        TCCR0A = 0x02;//CTC mode
        //TCCR0B = 0x02;
        TIMSK0 = 0x02;//enable OCR0A
        //OCR0B = 255 - BL_dutycycle;
        
        //OCR0A = 255 - (100 - BL_dutycycle);
        OCR0A = BL_dutycycle;

}

//General short delays
void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 165 ; y++){
      for ( z = 0 ; z < 18 ; z++){
        asm volatile ("nop");
      }
    }
  }
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

//delay for display timing
void delay(void)
{
        char y;
        
        //for(y = 0; y < 20; y++)
        for(y = 0; y < 30; y++)
        {
                asm volatile ("nop");
                
        }
        
        /*
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        */
        
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

void clear_screen(void)
{
        char x, y;
        
        delay();
        
        for (x = 0; x < 8; x++)
        {
                //set x address
                //set control lines
                PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
                
                set_data(0xB8 | x);
                delay();
                PORTC |= (1 << EN);//up
                delay();
                PORTC &= ~(1 << EN);//down
                delay();
                PORTC |= ((1 << EN) | (1 << R_W) |(1 << RS));//all high
                
                delay();
                
                //Set y address to zero
                //set control lines
                PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
                
                set_data(0x40);
                delay();
                PORTC |= (1 << EN);//up
                delay();
                PORTC &= ~(1 << EN);//down
                delay();
                PORTC |= ((1 << EN) | (1 << R_W) | (1 << RS));//all high
                
                if (reverse == 1) set_data(0xFF);
                else set_data(0);
                
                for (y = 0; y < 64; y++)
                {
                        delay();
                        //y address increments after every write
                        //write data, CS1, left half of screen
                        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << CS1));//down
                        delay();
                        PORTC |= (1 << EN);//up
                        delay();
                        PORTC &= ~(1 << EN);//down
                        delay();
                        PORTC |= ((1 << EN) | (1 << R_W) | (1 << CS1));//all high
                        
                        delay();
                        
                        //write data, CS2, right half of screen
                        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << CS2));//down
                        delay();
                        PORTC |= (1 << EN);//up
                        delay();
                        PORTC &= ~(1 << EN);//down
                        delay();
                        PORTC |= ((1 << EN) | (1 << R_W) | (1 << CS2));//all high
                }
                
        }
        
        x_offset = 0;
        y_offset = 63;

}

//sets horizontal position (data sheet calls this "y". Man, I just wanna punch these guys in the mouth...)
void set_x(char x_spot)
{
        //char a;
        
        //a = x_spot;
        //if (a == 0) a = 63;
        //else a--;
        
        if (x_spot == 0) x_spot = 63;
        else x_spot--;
        
        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
                        
        set_data(0x40 | x_spot);
        delay();
        PORTC |= (1 << EN);//up
        delay();
        PORTC &= ~(1 << EN);//down
        delay();
        PORTC |= ((1 << EN) | (1 << R_W) | (1 << RS));//all high
        delay();
        
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
        
    if ((x_offset + 6) > 127)
        {
                x_offset = 0;
                if (y_offset <= 7)
                {
                        y_offset = 63;
                        //clear_screen();
                }
                else y_offset -= 8;
                
        }
        
        pixel(0,0,0);//cheat
        
}


void set_page(char page)
{
        //set control lines
        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
        
        set_data(0xB8 | page);
        delay();
        PORTC |= (1 << EN);//up
        delay();
        PORTC &= ~(1 << EN);//down
        delay();
        PORTC |= ((1 << EN) | (1 << R_W) |(1 << RS));//all high
        
        delay();
}


void write_byte(char byte, char side)
{        
        PORTC |= (1 << RS);//make sure this thing is high
        set_data(byte);
        
        delay();
        //y address increments after every write
        //write data, CS1, left half of screen
        if (side == 1) PORTC &= ~((1 << EN) | (1 << R_W) | (1 << CS1));//down
        else if (side == 2) PORTC &= ~((1 << EN) | (1 << R_W) | (1 << CS2));//down
        delay();
        PORTC |= (1 << EN);//up
        delay();
        PORTC &= ~(1 << EN);//down
        delay();
        PORTC |= ((1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2));//all high
        
        
}


//display on
void display_on(void)
{
        set_data(0x3F);
        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
        delay();
        PORTC |= (1 << EN);//up
        delay();
        PORTC &= ~(1 << EN);//down
        PORTC |= ((1 << EN) | (1 << R_W) | (1 << RS));//all high
        
}
 
//mapping to cartesian coordinates, (0,0) is in the lower left corner, (127,63) is in the upper right
void pixel(char S_R, char x, char y)
{
        static char temp_page, temp_side, temp_x = 0, temp_data1 = 0, temp_data2 = 0;
        
        //don't try to print something outside of our range
        if (x > 127) return;
        if (y > 63) return;
        
        if (x >= 64) temp_side = 2, temp_x = x - 64;
        else temp_side = 1, temp_x = x;
        
        temp_page = 7 - (y >> 3);
        
        //data = (1 << (y - ((7 - temp_page) * 8)));
        temp_data1 = (1 << (7 - (y - ((7 - temp_page) * 8))));
        
        set_page(temp_page);
        set_x(temp_x);
        //set_x(0);
        
        //need to read the existing byte here, then or it with the new byte
        temp_data2 = read_byte(temp_x, temp_side);
        
        if (reverse == 1) S_R ^= 1;
        
        if (S_R == 0)
        {
                temp_data1 = ~temp_data1;
                temp_data1 &= temp_data2;
        }
        
        else temp_data1 |= temp_data2;
        
        set_x(temp_x);//reset this...
        
        write_byte(temp_data1, temp_side);
        
        

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
        
        pixel(0,0,0);//cheat
}



char read_byte(char byte, char side)
{
        char data1 = 0, data2 = 0;
        
        if (byte == 0) byte = 63;
        else byte--;
        
        set_x(byte);
        
        PORTC |= ((1 << RESET) | (1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2) | (1 << RS));//all high, just to make sure
        
        DDRB &= 0xFC;//PB0 and PB1 as inputs
        DDRD = 0;
        
        //PORTB |= 0x03;//pullups...?
        //PORTD |= 0XFC;
        //PORTB = 0;
        //PORTD = 0;
        
        delay();
        delay();
        
        if (side == 1) PORTC &= ~((1 << EN) | (1 << CS1));//down
        else if (side == 2) PORTC &= ~((1 << EN) | (1 << CS2));//down
        //PORTC &= ~((1 << EN) | (1 << CS1));//down
        
        
        
        delay();
        delay();
        PORTC |= (1 << EN);//up
        
        
        delay();
        delay();
        
        /*
        data1 = PINB;
        data1 &= 0x03;
        
        data2 = PIND;
        data2 &= 0xFC;
        
        data1 |= data2;
        */
        
        PORTC &= ~(1 << EN);//down
        
        
        
        delay();
        delay();
        
        PORTC |= ((1 << RESET) | (1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2) | (1 << RS));//all high

        //DDRB = 0b00000011; //PB0 and PB1 are outs
        //DDRD = 0b11111100; //PD2-PD7 are also outs.  Ports B and D are the data bus.
        
        
        
        delay();
        delay();
        
        //PORTC &= ~((1 << EN) | (1 << CS1));//down
        if (side == 1) PORTC &= ~((1 << EN) | (1 << CS1));//down
        else if (side == 2) PORTC &= ~((1 << EN) | (1 << CS2));//down
        
        
        
        delay();
        delay();
        PORTC |= (1 << EN);//up
        
        //we can read here...

        delay();
        delay();
        data1 = PINB;
        data1 &= 0x03;
        
        data2 = PIND;
        data2 &= 0xFC;
        
        data1 |= data2;
        //while(1);
        
        PORTC &= ~(1 << EN);//down
        
        //cannot read here...
        //while(1);

        PORTC |= ((1 << RESET) | (1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2) | (1 << RS));//all high

        DDRB |= 0x03; //PB0 and PB1 are outs
        DDRD |= 0xFC; //PD2-PD7 are also outs.  Ports B and D are the data bus.
        
        delay();
        
        return data1;

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
        
        read_byte(0,1);
        
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
        
        pixel(0,0,0);//cheat
}

//demonstration code
void demo(void)
{
        char x, y, temp;
        int q = 0;
        
        while(1)
        {        
                x_offset = 0;
                y_offset = 63;
                
                
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
                                if (y_offset <= 8) y_offset = 63;
                                del_char(1);
                                print_char(1, x);
                                if (RX_in > 0) return;
                        }
                }
                
                clear_screen();
                
                //draw circles================================
                for (x = 5; x < 64; x += 5)
                {
                        circle(1,64,32,x);
                        if (RX_in > 0) return;
                }
                
                
                //draw lines===================================
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 16)
                {
                        line(1,0,y,x,0);
                        y -= 8;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 16)
                {
                        line(1,x,0,X_ENDPOINT,y);
                        y += 8;
                }
                
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 16)
                {
                        line(1,x,Y_ENDPOINT,X_ENDPOINT,y);
                        y -= 8;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 16)
                {
                        line(1,0,y,x,Y_ENDPOINT);
                        y += 8;
                }
                
                
                //erase circles================================
                for (x = 5; x < 64; x += 5)
                {
                        circle(0,64,32,x);
                        if (RX_in > 0) return;
                }

                //erase lines===================================
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 16)
                {
                        line(0,0,y,x,0);
                        y -= 8;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 16)
                {
                        line(0,x,0,X_ENDPOINT,y);
                        y += 8;
                }
                
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 16)
                {
                        line(0,x,Y_ENDPOINT,X_ENDPOINT,y);
                        y -= 8;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 16)
                {
                        line(0,0,y,x,Y_ENDPOINT);
                        y += 8;
                }
                
                if (RX_in > 0) return;
                
                //Boxes=================================================================
                y = 47;
                for (x = 0; x <= 100; x += 10)
                {
                        erase_block(x, y, x+16, y+16);
                        box(x, y, x+16, y+16);
                        y -= 4;
                }
                
                
                
                y = 22;
                //Logo=================================================================
                q = 0;
                while(q < 30)
                {
                        temp = logo[q];
                        for (x = 100; x < 108; x++)
                        {
                                if (temp & 0x80) pixel(1,x,y);
                                
                                temp <<= 1;
                        }
                        q++;
                        temp = logo[q];
                        for (x = 108; x < 116; x++)
                        {
                                if (temp & 0x80) pixel(1,x,y);
                                
                                temp <<= 1;
                        }
                        y--;
                        q++;
                }        
                
                //read_byte(0,1);
                pixel(0,0,0);
                
                delay_ms(1000);
                reverse ^= 1;
                clear_screen();
                
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
        
        line(1, x2, y2, x2, y1);
        line(1, x1, y2, x2, y2);
        line(1, x1, y2, x1, y1);
        line(1, x1, y1, x2, y1);
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



