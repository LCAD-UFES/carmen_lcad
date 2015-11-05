/*
    graphic LCD backpack test code
        
        set up for the big LCD, 160x128
        
        10/16/08, everything works. Pixel, line, circle, set x, set y, print_char, clear_screen. All good.
*/

/*
Stuff from Nate to be incorporated...

In IOINIT:

    //Init timer 2
    //8,000,000 / 8 = 1,000,000
    TCCR2B = (1<<CS21); //Set Prescaler to 8. CS21=1


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
    TIFR2 = 0x01; //Clear any interrupt flags on Timer2
    
    TCNT2 = 256 - x; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click

    while( (TIFR2 & (1<<TOV2)) == 0);
}
*/

#include <avr/io.h>
#include "rprintf.h"
#include <math.h>
#include <avr/interrupt.h>

#define FOSC 16000000// Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#define WR 0        //PC0
#define RD 1        //PC1
#define CE 2        //PC2
#define CD 3        //PC3
#define HALT 4        //PC4
#define RST 5        //PC5

#define BL_EN 2        //PB2

//#define        CS20 0
//#define        CS21 1
//#define        CS22 2


//Define functions
//======================
void ioinit(void);      //Initializes IO
void delay_ms(uint16_t x); //General purpose delay
void delay(void);
void delay_us(uint8_t x);
void USART_Init( unsigned int ubrr);
void put_char(char byte);
void print_num(short num);
int rnd(float number);

void set_data(char data);
char read(char D_S);//reads data (D_S = 1) or status (D_S = anything else)
void write(char D_C, char byte);//writes data or command
void display_init(void);


void clear_screen(void);

void print_char(char S_R, char txt);
void del_char(char endpoint);
void pixel(char S_R, char x, char y);
void line(char S_R, char x1, char y1, char x2, char y2);
void circle(char S_R, int x, int y, int r);
void demo(void);

//======================
char x_offset = 0;
char y_offset = 127;

unsigned char RX_array[256];
volatile unsigned short RX_in = 0;
unsigned short RX_read = 0;

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
                                                          
                                                          

ISR (SIG_UART_RECV)                                                                        //USART Receive Interrupt
{
        cli();//Disable Interrupts
        RX_array[RX_in] = UDR;
        
        RX_in++;
        
        if (RX_in >= 256) RX_in = 0;
        
        sei();//Enable Interrupts
        
}


int main (void)
{
        char x, y, a;
        //short a, b;
        
        //char a = 0;
    ioinit(); //Setup IO pins and defaults
        USART_Init( MYUBRR);
        rprintf_devopen(put_char); /* init rrprintf */
        
        //reset the display
        delay_ms(1);
        PORTC |= (1<<RST);
        
        //initialize the display
        display_init();

        clear_screen();
        
        //Backlight on
        PORTB &= (~(1<<BL_EN));
        
        while(1)
        {
                if(RX_in != RX_read)
                {
                        x = RX_array[RX_read];
                        RX_read++;
                        if(RX_read >= 256) RX_read = 0;
                        
                        //Backspace===================================================
                        if(x == 8) del_char(0);
                        
                        //Special commands
                        else if (x == 124)
                        {        
                                //make sure the next byte is there
                                while(RX_in == RX_read);
                                
                                //0, clear screen======================================================
                                if(RX_array[RX_read] == 0)
                                {
                                        clear_screen();
                                        RX_read++;
                                        if(RX_read >= 256) RX_read = 0;
                                }
                                
                                //demo mode
                                else if(RX_array[RX_read] == 4)
                                {
                                        RX_in = 0, RX_read = 0;
                                        demo();
                                        clear_screen();
                                        RX_in = 0;
                                }
                                
                                else
                                {                                
                                        //set x or y=========================================================
                                        if((RX_array[RX_read] == 24) | (RX_array[RX_read] == 25))
                                        {
                                                RX_read++;
                                                if(RX_read >= 256) RX_read = 0;
                                                while(RX_in == RX_read);//wait for byte
                                                if (RX_array[RX_read-1] == 24) x_offset = RX_array[RX_read];
                                                else if (RX_array[RX_read-1] == 25) y_offset = RX_array[RX_read];
                                                
                                                RX_read++;
                                                if(RX_read >= 256) RX_read = 0;
                                                
                                                if (x_offset > 159) x_offset = 159;
                                                if (y_offset > 127) y_offset = 127;

                                        }

                                        //set pixel=========================================================
                                        if (RX_array[RX_read] == 16)
                                        {
                                                //need 3 bytes
                                                for (y = 0; y < 3; y++)
                                                {
                                                        RX_read++;
                                                        if(RX_read >= 256) RX_read = 0;
                                                        while(RX_in == RX_read);//wait for byte
                                                }
                                                
                                                pixel(RX_array[RX_read], RX_array[RX_read-2], RX_array[RX_read-1]);
                                                
                                                RX_read++;
                                                if(RX_read >= 256) RX_read = 0;

                                        }

                                        //<ctrl>c, circle======================================================
                                        if(RX_array[RX_read] == 3)
                                        {
                                                //need 4 bytes
                                                for (y = 0; y < 4; y++)
                                                {
                                                        RX_read++;
                                                        if(RX_read >= 256) RX_read = 0;
                                                        while(RX_in == RX_read);//wait for byte
                                                }
                                                
                                                circle(RX_array[RX_read], RX_array[RX_read-3], RX_array[RX_read-2], RX_array[RX_read-1]);
                                                
                                                RX_read++;
                                                if(RX_read >= 256) RX_read = 0;
                                        }
                                        

                                        //<ctrl>L, line========================================================
                                        else if (RX_array[RX_read] == 12)
                                        {
                                                //need 5 bytes
                                                for (y = 0; y < 5; y++)
                                                {
                                                        RX_read++;
                                                        if(RX_read >= 256) RX_read = 0;
                                                        while(RX_in == RX_read);//wait for byte
                                                }
                                                
                                                line(RX_array[RX_read], RX_array[RX_read-4], RX_array[RX_read-3], RX_array[RX_read-2], RX_array[RX_read+-1]);
                                                RX_read++;
                                                if(RX_read >= 256) RX_read = 0;
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
        
        //set these in the read/write functions instead of here
        //DDRB = 0b00000011; //PB0 and PB1 are outs
        //DDRD = 0b11111100; //PD2-PD7 are also outs.  Ports B and D are the data bus.

        PORTC = ((1<<WR) | (1<<RD) | (1<<CE) | (1<<CD) | (1<<HALT));
        PORTC &= (~(1<<RST));//set the reset line low at power up
        DDRC = ((1<<WR) | (1<<RD) | (1<<CE) | (1<<CD) | (1<<HALT) | (1<<RST));
        
        //Init timer 2
    //8,000,000 / 8 = 1,000,000
    TCCR2 = (1<<CS21); //Set Prescaler to 8. CS21=1
        //TCCR2 = ((1<<CS20) | (1<<CS22) | (1<<CS22));
        
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
    //TIFR = 0x01; //Clear any interrupt flags on Timer2
        TIFR |= 0x40;
    
    TCNT2 = 256 - temp; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click

    //while( (TIFR & (1<<TOV2)) == 0);
        while(!(TIFR & 0x40));
        
        if (x == 0) return;//this is for display timing
        
        
        //The prescaler doesn't allow for a setting of 16, just 8 or 32. So, we do this twice.
        TIFR |= 0x40;
    
    TCNT2 = 256 - temp; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click

    //while( (TIFR & (1<<TOV2)) == 0);
        while(!(TIFR & 0x40));
        
}

void USART_Init( unsigned int ubrr)
{
        /* Set baud rate */
        UBRRH = (unsigned char)(ubrr>>8);
        UBRRL = (unsigned char)ubrr;
        /* Enable receiver and transmitter */
        //UCSRB = (1<<RXEN)|(1<<TXEN);
        UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);        //Enable Interrupts on receive character
        /* Set frame format: 8data, 2stop bit */
        //UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);
        UCSRC = (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);
        sei();
}

void put_char(char byte)
{
        /* Wait for empty transmit buffer */
        while ( !( UCSRA & (1<<UDRE)) );
        /* Put data into buffer, sends the data */
        UDR = byte;
}

//delay for display timing
void delay(void)
{
        char y;
        
        for(y = 0; y < 20; y++)
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
        delay();
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
                write(1, 0);
                while(!(read(0) & 3));//read status
                write(0, 0xC0);
        }
        
        x_offset = 0;
        y_offset = 127;
}

/*
void set_cursor(char x_spot, char y_spot)
{
        //set address pointer to 0, start of graphics
        while(!(read(0) & 3));//read status
        write(1, x_spot);
        while(!(read(0) & 3));//read status
        write(1, y_spot);
        while(!(read(0) & 3));//read status
        write(0, 0x21);

}
*/

void pixel(char S_R, char x, char y)
{
        short address = 0;
        char byte = 0;
                
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
        
        //set-reset bit
        while(!(read(0) & 3));//read status
        write(0, byte);
        
}



void line(char S_R, char x1, char y1, char x2, char y2)
{
        float m, q;
        int x_dif, y_dif;
        int a, b, c;
        
        if ((x1 > 160 | (x2 > 160)) return;
        //if ((x1 < 0) | (x2 < 0)) return;
        if ((y1 > 128) | (y2 > 128)) return;
        //if ((y1 < 0) | (y2 < 0)) return;
        
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
                        //delay_ms(25);
                }
        }
        
        else
        {
                if (y2 > y1)
                {
                        for (a = y1; a <= y2; a++)
                        {
                                q = (((float)(a-b))/m);
                                c = rnd(q);
                                
                                pixel(S_R, (char)c, (char)a);
                                //delay_ms(25);
                        }
                }
                
                else if (y1 > y2)
                {
                        for (a = y1; a >= y2; a--)
                        {
                                q = (((float)(a-b))/m);
                                c = rnd(q);
                                
                                pixel(S_R, (char)c, (char)a);
                                //delay_ms(25);
                        }
                }
        }
                
}



void circle(char S_R, int x, int y, int r)
{
        int x1 = 0, x2 = 0;
        int x_line = 0, y_line = 0;
        int temp_y;
        int temp_x;
        //float x_float, y_float, r_float;
        
        x1 = x - r;
        x2 = x + r;
        
        for (temp_x = x1; temp_x <= x2; temp_x++)
        {
                temp_y = ((sqrt((r*r) - ((temp_x - x)*(temp_x - x)))) - y);
                
                temp_y *= (-1);
                
                /*
                print_num((short)temp_x);
                put_char(9);
                print_num((short)temp_y);
                put_char(10);
                put_char(13);
                */
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
                
                //
                
                
        }
        

}


int rnd(float number)
{
        int a;
        float b;
        
        a = number / 1;
        b = number - a;
        
        if (b >= 0.5) a++;
        
        return a;

}


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
                        
                //if (j == text_array_offset+5) x_offset++;//blank byte for letter spacing

                        
                x_offset++;
                
    }
        
        x_offset++;
        
    if ((x_offset + 6) > 159)
        {
                x_offset = 0;
                if (y_offset <= 7) y_offset = 127;
                else y_offset -= 8;
                
        }
        
}



void print_num(short num)
{
        short a;
        char b, c;
        a = num;
        
        //print hex
        for (c = 0; c < 4; c++)
        {
                b = (char)((a & 0xF000) >> 12);
                if (b < 10) put_char(b+48);
                else put_char(b+55);
                a <<= 4;
        }
        
        //decimal
        /*
        b = a/100;
        putchr(b+48);
        a -= b*100;
        b = a/10;
        putchr(b+48);
        a -= b*10;
        putchr(a+48);
        */
        
}


void demo(void)
{
        char x, y;
        
        while(1)
        {        
                x_offset = 0;
                y_offset = 127;
        
                for (y = 0; y < 10; y++)
                {
                        for (x = 32; x < 123; x++)
                        {        
                                del_char(1);
                                print_char(1, x);
                                if (RX_in > 0) return;
                        }
                }
                
                clear_screen();
                
                for (y = 0; y < 20; y++)
                {
                        for (x = 32; x < 123; x++)
                        {
                                //x_offset += 4;
                                y_offset -= 6;
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
                y = 128;
                
                for (x = 0; x < 160; x += 20)
                {
                        line(1,0,y,x,0);
                        y -= 16;
                }
                
                y = 0;
                
                for (x = 0; x < 160; x += 20)
                {
                        line(1,x,0,159,y);
                        y += 16;
                }
                
                y = 128;
                
                for (x = 0; x < 160; x += 20)
                {
                        line(1,x,127,159,y);
                        y -= 16;
                }
                
                y = 0;
                
                for (x = 0; x < 160; x += 20)
                {
                        line(1,0,y,x,127);
                        y += 16;
                }
                
                
                //erase circles================================
                for (x = 5; x < 120; x += 5)
                {
                        circle(0,80,64,x);
                        if (RX_in > 0) return;
                }

                //erase lines===================================
                y = 128;
                
                for (x = 0; x < 160; x += 20)
                {
                        line(0,0,y,x,0);
                        y -= 16;
                }
                
                y = 0;
                
                for (x = 0; x < 160; x += 20)
                {
                        line(0,x,0,159,y);
                        y += 16;
                }
                
                y = 128;
                
                for (x = 0; x < 160; x += 20)
                {
                        line(0,x,127,159,y);
                        y -= 16;
                }
                
                y = 0;
                
                for (x = 0; x < 160; x += 20)
                {
                        line(0,0,y,x,127);
                        y += 16;
                }
                
                if (RX_in > 0) return;
                
        }
        

}

//Deletes a character. Endpoint == 0 for a backwards delete,
//Endpoint != 0 to erase spot for a new character write
void del_char(char endpoint)
{
        char a, y;
        
        if (endpoint == 0)//Backwards delete
        {
                if (x_offset <= 5)
                {
                        x_offset = 150;
                        if (y_offset >= 120) y_offset = 7;
                        else y_offset += 8;
                }
                
                else x_offset -= 6;
        }
                
        for (a = 0; a < 6; a++)
        {                                        
                for (y = 0; y < 8; y++)
                {
                        pixel(0, x_offset + a, y_offset - y);
                        
                }
        }
        
                
}






