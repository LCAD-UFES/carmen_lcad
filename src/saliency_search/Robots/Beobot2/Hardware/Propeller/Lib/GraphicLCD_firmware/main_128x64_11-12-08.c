/*
    graphic LCD backpack test code
        
        10/7/08, text and lines work, need to set up UART and circle functions
        11/12/08, box, block_erase, circle, pixel, logo...everything done for this one.
        
*/

#include <avr/io.h>
#include "rprintf.h"
#include <math.h>
#include <avr/interrupt.h>

#define FOSC 16000000// Clock Speed
#define BAUD 57600
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

#define DISPLAY_DELAY 6



//Define functions
//======================
void ioinit(void);      //Initializes IO
void delay_ms(uint16_t x); //General purpose delay
void delay_us(uint8_t x);
void delay(void);
void USART_Init( unsigned int ubrr);
void put_char(char byte);
void demo(void);

void set_data(char data);
void set_x(char x_spot);
void set_page(char page);
void clear_screen(void);
void write_byte(char byte, char side);
char read_byte(char byte, char side);
void display_on(void);
void set_port_out(void);
void set_port_in(void);

//unsigned char print_char(char txt);
void print_char(char S_R, char txt);
void del_char(char endpoint);
void pixel(char S_R, char x, char y);
void line(char S_R, char x1, char y1, char x2, char y2);
void circle(char S_R, int x, int y, int r);
void erase_block(char x1, char y1, char x2, char y2);
void box(char x1, char y1, char x2, char y2);
int rnd(float number);

//======================


char x_offset = 0;
char y_offset = 63;
//char y_offset = 0;
char side_mark = 1;//left side default
char page_mark = 0;

unsigned char RX_array[256];
volatile unsigned short RX_in = 0;
unsigned short RX_read = 0;

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
        char x, y;
        //char a = 0;
    ioinit(); //Setup IO pins and defaults
        USART_Init( MYUBRR);
        rprintf_devopen(put_char); /* init rrprintf */
        
        //set_data(0x55);

        //PORTC |= ((1 << RESET) | (1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2) | (1 << RS));//all high
        
        //while(1);
        
        /*
        while(1)
        {
                PORTC &= ~((1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2));//down
                delay_ms(500);
                //PORTC |= ((1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2));//all high
                PORTC |= ((1 << RESET) | (1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2) | (1 << RS));//all high
                delay_ms(500);
        }
        */
        /*
        DDRC = 0b00000001;
        
        while(1)
        {
                PORTC |= 0b00000001;
                delay_1uS();
                PORTC &= 0b11111110;
                delay_1uS();
        
        }
        */
        
        //Reset the display
        //PORTC = 0b11110111;
        PORTC &= ~(1 << RESET);
        delay_ms(50);
        PORTC |= (1 << RESET);
        delay_ms(500);
        
        clear_screen();
        
        set_page(0);
        set_x(0);
        
        display_on();
        
        //set display start line to 0
        //set control lines
        
        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
        set_port_out();
        //set_data(0xC0);
        set_data(0xC0);
        delay_us(4);
        PORTC |= (1 << EN);//up
        delay_us(4);
        PORTC &= ~(1 << EN);//down
        delay_us(4);
        PORTC |= ((1 << EN) | (1 << R_W) | (1 << RS));//all high
        
        set_port_in();
        delay_us(4);
        
        
        x_offset = 0;

        set_page(0);
        
        //Backlight on
        PORTB &= (~(1<<BL_EN));
        
        //demo();  
        //put_char('X');
        
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
                                
                                
                                //Backlight on/off
                                else if(RX_array[RX_read] == 2)
                                {
                                        y = PINB;
                                        if (y & (1<<BL_EN)) PORTB &= (~(1<<BL_EN));
                                        else PORTB |= (1<<BL_EN);
                                        RX_read++;
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
                                                
                                                if (x_offset > 127) x_offset = 127;
                                                if (y_offset > 63) y_offset = 63;

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
                                        
                                        
                                        //<ctrl>e, erase block======================================================
                                        if(RX_array[RX_read] == 5)
                                        {
                                                //need 4 bytes
                                                for (y = 0; y < 4; y++)
                                                {
                                                        RX_read++;
                                                        if(RX_read >= 256) RX_read = 0;
                                                        while(RX_in == RX_read);//wait for byte
                                                }
                                                
                                                erase_block(RX_array[RX_read-3], RX_array[RX_read-2], RX_array[RX_read-1], RX_array[RX_read]);
                                                
                                                RX_read++;
                                                if(RX_read >= 256) RX_read = 0;
                                        }
                                        
                                        
                                        //<ctrl>o, box, running out of meaningful letters======================================================
                                        if(RX_array[RX_read] == 15)
                                        {
                                                //need 4 bytes
                                                for (y = 0; y < 4; y++)
                                                {
                                                        RX_read++;
                                                        if(RX_read >= 256) RX_read = 0;
                                                        while(RX_in == RX_read);//wait for byte
                                                }
                                                
                                                box(RX_array[RX_read-3], RX_array[RX_read-2], RX_array[RX_read-1], RX_array[RX_read]);
                                                
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
                                //put_char('L');
                                print_char(1, x);
                                
                        }
                        
                        //set_data(0xFF);
                        //set_port_in();
                        //display_on();
                        //y = PINB;
                        //PORTB = y;
                }
                
        }
        
}

void ioinit (void)
{
        
    //1 = output, 0 = input
   
        //DDRB = 0b00000011; //PB0 and PB1 are outs
        PORTB |= (1<<BL_EN);//Backlight off
        DDRB |= (1<<BL_EN);//set PB2 as output
        
        DDRC = ((1<<EN) | (1<<RS) | (1<<R_W) | (1<<RESET) | (1<<CS1) | (1<<CS2));
        PORTC = ((1<<EN) | (1<<RS) | (1<<R_W) | (1<<RESET) | (1<<CS1) | (1<<CS2));
        //DDRD = 0b11111100; //PD2-PD7 are also outs.  Ports B and D are the data bus.
        
        //Init timer 2
    //8,000,000 / 8 = 1,000,000
    TCCR2 = (1<<CS21); //Set Prescaler to 8. CS21=1

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

void clear_screen(void)
{
        char x, y;
        set_port_out();
        //delay();
        delay_us(DISPLAY_DELAY);
        
        for (x = 0; x < 8; x++)
        {
                //set x address
                //set control lines
                PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down

                set_data(0xB8 | x);
                //delay();
                delay_us(DISPLAY_DELAY);
                PORTC |= (1 << EN);//up
                //delay();
                delay_us(DISPLAY_DELAY);
                PORTC &= ~(1 << EN);//down
                //delay();
                delay_us(DISPLAY_DELAY);
                PORTC |= ((1 << EN) | (1 << R_W) |(1 << RS));//all high
                
                //delay();
                delay_us(DISPLAY_DELAY);
                
                //Set y address to zero
                //set control lines
                PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
                
                set_data(0x40);
                //delay();
                delay_us(DISPLAY_DELAY);
                PORTC |= (1 << EN);//up
                //delay();
                delay_us(DISPLAY_DELAY);
                PORTC &= ~(1 << EN);//down
                //delay();
                delay_us(DISPLAY_DELAY);
                PORTC |= ((1 << EN) | (1 << R_W) | (1 << RS));//all high
                
                set_data(0);
                
                for (y = 0; y < 64; y++)
                {
                        //delay();
                        delay_us(DISPLAY_DELAY);
                        //y address increments after every write
                        //write data, CS1, left half of screen
                        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << CS1));//down
                        //delay();
                        delay_us(DISPLAY_DELAY);
                        PORTC |= (1 << EN);//up
                        //delay();
                        delay_us(DISPLAY_DELAY);
                        PORTC &= ~(1 << EN);//down
                        //delay();
                        delay_us(DISPLAY_DELAY);
                        PORTC |= ((1 << EN) | (1 << R_W) | (1 << CS1));//all high
                        
                        //delay();
                        delay_us(DISPLAY_DELAY);
                        
                        //write data, CS2, right half of screen
                        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << CS2));//down
                        //delay();
                        delay_us(DISPLAY_DELAY);
                        PORTC |= (1 << EN);//up
                        //delay();
                        delay_us(DISPLAY_DELAY);
                        PORTC &= ~(1 << EN);//down
                        //delay();
                        delay_us(DISPLAY_DELAY);
                        PORTC |= ((1 << EN) | (1 << R_W) | (1 << CS2));//all high
                }
                
        }
        
        set_port_in();

}

//sets horizontal position (data sheet calls this "y". Man, I just wanna punch these guys in the mouth...)
void set_x(char x_spot)
{
        //char a;
        
        //a = x_spot;
        //if (a == 0) a = 63;
        //else a--;
        
        set_port_out();
        
        if (x_spot == 0) x_spot = 63;
        else x_spot--;
        
        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
                        
        set_data(0x40 | x_spot);
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC |= (1 << EN);//up
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC &= ~(1 << EN);//down
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC |= ((1 << EN) | (1 << R_W) | (1 << RS));//all high
        //delay();
        set_port_in();
        delay_us(DISPLAY_DELAY);
        
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
        
    if ((x_offset + 6) > X_ENDPOINT)
        {
                x_offset = 0;
                if (y_offset <= 7) y_offset = Y_ENDPOINT;
                else y_offset -= 8;
                
        }
        
}



void set_page(char page)
{
        //set control lines
        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
        
        set_port_out();
        
        set_data(0xB8 | page);
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC |= (1 << EN);//up
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC &= ~(1 << EN);//down
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC |= ((1 << EN) | (1 << R_W) |(1 << RS));//all high
        
        set_port_in();
        
        //delay();
        delay_us(DISPLAY_DELAY);
}


void write_byte(char byte, char side)
{        
        PORTC |= (1 << RS);//make sure this thing is high
        
        set_port_out();
        
        set_data(byte);
        
        //delay();
        delay_us(DISPLAY_DELAY);
        //y address increments after every write
        //write data, CS1, left half of screen
        if (side == 1) PORTC &= ~((1 << EN) | (1 << R_W) | (1 << CS1));//down
        else if (side == 2) PORTC &= ~((1 << EN) | (1 << R_W) | (1 << CS2));//down
        
        //PORTC &= ~((1 << EN) | (1 << R_W));
        //if (side == 1) PORTC &= (~(1 << CS1));
        //else if (side == 2) PORTC &= (~(1 << CS2));
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC |= (1 << EN);//up
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC &= ~(1 << EN);//down
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC |= ((1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2));//all high
        delay_us(DISPLAY_DELAY);
        
        set_port_in();
        set_data(0xFF);
                
}


//display on
void display_on(void)
{
        set_port_out();
        set_data(0x3F);
        PORTC &= ~((1 << EN) | (1 << R_W) | (1 << RS));//down
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC |= (1 << EN);//up
        //delay();
        delay_us(DISPLAY_DELAY);
        PORTC &= ~(1 << EN);//down
        PORTC |= ((1 << EN) | (1 << R_W) | (1 << RS));//all high
        
        set_port_in();
        
}
 
//mapping to cartesian coordinates, (0,0) is in the lower left corner, (127,63) is in the upper right
void pixel(char S_R, char x, char y)
{
        static char temp_page, temp_side, temp_x = 0, temp_data1 = 0, temp_data2 = 0;
        
        if ((x > X_ENDPOINT) | (y > Y_ENDPOINT)) return;
        //if ((x < 0) | (y < 0)) return;
        
        if (x >= 64) temp_side = 2, temp_x = x - 64;
        else temp_side = 1, temp_x = x;
        
        temp_page = 7 - (y >> 3);
        
        //data = (1 << (y - ((7 - temp_page) * 8)));
        temp_data1 = (1 << (7 - (y - ((7 - temp_page) * 8))));
        if (S_R == 0) temp_data1 = ~temp_data1;
        //if (S_R == 1) temp_data1 = (1 << (7 - (y - ((7 - temp_page) * 8))));
        
        set_page(temp_page);
        set_x(temp_x);
        //set_x(0);
        
        //need to read the existing byte here, then or it with the new byte
        temp_data2 = read_byte(temp_x, temp_side);
        
        if (S_R == 1) temp_data1 |= temp_data2;
        else if (S_R == 0) temp_data1 &= temp_data2;
        set_x(temp_x);//reset this...
        
        write_byte(temp_data1, temp_side);
        
        //display_on();

}


/*
//y = mx + b
void line(char x1, char y1, char x2, char y2)
{
        float m;
        char a, b;
        
        m = (float)(y2 - y1) / (float)(x2 - x1);
        
        b = y1-(m*x1);
        
        for (a = x1; a <= x2; a++)
        {
                pixel(a, (char)((m*a)+b));
                //delay_ms(25);
        }

}
*/

void line(char S_R, char x1, char y1, char x2, char y2)
{
        float m, q;
        int x_dif, y_dif;
        int a, b, c;
        
        if ((x1 > X_ENDPOINT) | (x2 > X_ENDPOINT)) return;
        //if ((x1 < 0) | (x2 < 0)) return;
        if ((y1 > Y_ENDPOINT) | (y2 > Y_ENDPOINT)) return;
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
                                if (x_dif == 0) c = x1;
                                else
                                {
                                        q = (((float)(a-b))/m);
                                        c = rnd(q);
                                }
                                
                                pixel(S_R, (char)c, (char)a);
                                //delay_ms(25);
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
                                //delay_ms(25);
                        }
                }
        }
                
}



char read_byte(char byte, char side)
{
        char data1 = 0, data2 = 0;
        
        if (byte == 0) byte = 63;
        else byte--;
        
        set_x(byte);
        
        PORTC |= ((1 << RESET) | (1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2) | (1 << RS));//all high, just to make sure
        
        //DDRB = 0x04;//all but PB2 inputs
        //DDRD = 0;
        
        set_port_in();
        
        delay_us(DISPLAY_DELAY);
        
        if (side == 1) PORTC &= ~((1 << EN) | (1 << CS1));//down
        else if (side == 2) PORTC &= ~((1 << EN) | (1 << CS2));//down
        
        delay_us(DISPLAY_DELAY);
        PORTC |= (1 << EN);//up
        
        delay_us(DISPLAY_DELAY);
        
        PORTC &= ~(1 << EN);//down
        
        delay_us(DISPLAY_DELAY);
        
        PORTC |= ((1 << RESET) | (1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2) | (1 << RS));//all high

        delay_us(DISPLAY_DELAY);
        
        if (side == 1) PORTC &= ~((1 << EN) | (1 << CS1));//down
        else if (side == 2) PORTC &= ~((1 << EN) | (1 << CS2));//down
        
        delay_us(DISPLAY_DELAY);
        PORTC |= (1 << EN);//up
        
        delay_us(DISPLAY_DELAY);
        data1 = PINB;
        data1 &= 0x03;
        
        data2 = PIND;
        data2 &= 0xFC;
        
        data1 |= data2;
        
        PORTC &= ~(1 << EN);//down
        
        PORTC |= ((1 << RESET) | (1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2) | (1 << RS));//all high

        //DDRB = 0b00000111; //PB0 and PB1 are outs
        //DDRD = 0b11111100; //PD2-PD7 are also outs.  Ports B and D are the data bus.

        delay_us(DISPLAY_DELAY);
        
        return data1;

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
                                y_offset -= 3;
                                if (y_offset <= 8) y_offset = 63;
                                del_char(1);
                                print_char(1, x);
                                if (RX_in > 0) return;
                        }
                }
                
                clear_screen();
                
                //draw circles================================
                for (x = 5; x < 60; x += 5)
                {
                        circle(1,64,32,x);
                        if (RX_in > 0) return;
                }
                
                //draw lines===================================
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 8)
                {
                        line(1,0,y,x,0);
                        y -= 8;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 8)
                {
                        line(1,x,0,X_ENDPOINT,y);
                        y += 8;
                }
                
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 8)
                {
                        line(1,x,Y_ENDPOINT,X_ENDPOINT,y);
                        y -= 8;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 8)
                {
                        line(1,0,y,x,Y_ENDPOINT);
                        y += 8;
                }
                        
                //erase circles================================
                for (x = 5; x < 60; x += 5)
                {
                        circle(0,64,32,x);
                        if (RX_in > 0) return;
                }

                //erase lines===================================
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 8)
                {
                        line(0,0,y,x,0);
                        y -= 8;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 8)
                {
                        line(0,x,0,X_ENDPOINT,y);
                        y += 8;
                }
                
                y = Y_ENDPOINT;
                
                for (x = 0; x < X_ENDPOINT; x += 8)
                {
                        line(0,x,Y_ENDPOINT,X_ENDPOINT,y);
                        y -= 8;
                }
                
                y = 0;
                
                for (x = 0; x < X_ENDPOINT; x += 8)
                {
                        line(0,0,y,x,Y_ENDPOINT);
                        y += 8;
                }
                
                if (RX_in > 0) return;
                
                //Boxes=================================================================
                y = 47;
                for (x = 0; x <= 110; x += 10)
                {
                        erase_block(x, y, x+16, y+16);
                        box(x, y, x+16, y+16);
                        y -= 4;
                }
                
                
                //x = 110;
                y = 18;
                //Logo=================================================================
                q = 0;
                while(q < 30)
                {
                        temp = logo[q];
                        for (x = 110; x < 118; x++)
                        {
                                if (temp & 0x80) pixel(1,x,y);
                                
                                temp <<= 1;
                        }
                        q++;
                        temp = logo[q];
                        for (x = 118; x < 126; x++)
                        {
                                if (temp & 0x80) pixel(1,x,y);
                                
                                temp <<= 1;
                        }
                        y--;
                        q++;
                }        
                
                delay_ms(3000);
                clear_screen();
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
                        x_offset += 120;
                        y_offset += 8;
                        
                        if (y_offset > 63) y_offset -= 64;
                        
                }
                
                else x_offset -= 6;
        }
        
        //erase_block(x_offset, y_offset - 8, x_offset + 5, y_offset);
        
        
        for (a = x_offset; a < x_offset + 6; a++)
        {                                        
                for (y = y_offset - 7; y <= y_offset; y++)
                {
                        pixel(0, a, y);
                        //put_char('L');
                }
        }
        
        pixel(0, 0, 0);
                
}

void erase_block(char x1, char y1, char x2, char y2)
{
        static char temp_x = 0, temp_y = 0;
        
        for (temp_y = y2; temp_y >= y1; temp_y--)
        {
                for (temp_x = x1; temp_x <= x2; temp_x++)
                {
                        pixel(0, temp_x, temp_y);
                        
                        //rprintf("%d ",temp_x);
                        //rprintf("%d \r\n",temp_y);
                        //delay_ms(1000);
                        
                }
        }        
        
        

}

void box(char x1, char y1, char x2, char y2)
{
        //static char temp_x = 0, temp_y = 0;
        
        line(1, x1, y1, x1, y2);
        line(1, x1, y1, x2, y1);
        line(1, x2, y1, x2, y2);
        line(1, x1, y2, x2, y2);

}


void set_port_out(void)
{
        DDRB = 0x07;//PB0, PB1 and PB2 outs
        DDRD |= 0xFC;//PD2-PD7 outs
}


void set_port_in(void)
{
        DDRB = 0x04;//PB0 & PB1 are inputs, PB2 out
        DDRD &= 0x03;//PD2-PD7 inputs
}





