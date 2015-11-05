/*
    graphic LCD backpack test code
        
        10/7/08, text and lines work, need to set up UART and circle functions
        
*/

#include <avr/io.h>
#include "rprintf.h"


#define FOSC 16000000// Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#define EN 0        //PC0
#define RS 1        //PC1, D_I?
#define R_W 2        //PC2
#define RESET 3        //PC3
#define CS1 4        //PC4
#define CS2 5        //PC5


//Define functions
//======================
void ioinit(void);      //Initializes IO
void delay_ms(uint16_t x); //General purpose delay
void delay(void);
void USART_Init( unsigned int ubrr);
void put_char(char byte);

void set_data(char data);
void set_x(char x_spot);
void set_page(char page);
void clear_screen(void);
void write_byte(char byte, char side);
char read_byte(char byte, char side);
void display_on(void);

unsigned char print_char(char txt);
void pixel(char x, char y);
void line(char x1, char y1, char x2, char y2);

//======================


char x_offset = 0;
//char y_offset = 0;
char side_mark = 1;//left side default
char page_mark = 0;

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
        //PORTC = 0b11111111;
        PORTC |= (1 << RESET);
        delay_ms(500);
        
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
        
        /*
        set_x(0);
        write_byte(0X55, 1);
        
        set_x(1);
        write_byte(0X55, 1);
        
        //set_x(0);
        x = read_byte(1,1);
        
        set_x(2);
        write_byte(x, 1);
        set_x(3);
        write_byte(x, 1);
        */
        

        //set_x(63);//???
        
        //x = read_byte(1);
        
        //set_x(3);
        
        //rite_byte(x, 1);
        
        //set_x(0);
        
        //write_byte(x, 1);
        
        //while(1);
        
        while(1)
        {
                line(0, 0, 63, 63);
                line(0, 10, 63, 52);
                line(0, 21, 63, 42);
                line(0, 32, 63, 32);
                line(0, 42, 63, 21);
                line(0, 52, 63, 10);
                line(0, 63, 63, 0);
        
                
                line(64, 0, 127, 63);
                line(64, 10, 127, 52);
                line(64, 21, 127, 42);
                line(64, 32, 127, 32);
                line(64, 42, 127, 21);
                line(64, 52, 127, 10);
                line(64, 63, 127, 0);
                
                //set_x(20);
                x_offset = 20;
                set_page(0);
                
                print_char('C');
                print_char('O');
                print_char('C');
                print_char('K');
                
                x_offset = 82;
                print_char('B');
                print_char('L');
                print_char('O');
                print_char('C');
                print_char('K');
                
                x_offset = 14;
                set_page(7);
                
                print_char('M');
                print_char('O');
                print_char('T');
                print_char('H');
                print_char('E');
                print_char('R');
                
                x_offset = 79;
                print_char('F');
                print_char('U');
                print_char('C');
                print_char('K');
                print_char('E');
                print_char('R');
                
                delay_ms(1000);
                clear_screen();
        }
        
        //line(0, 20, 127, 63);
        
        //line(20, 0, 127, 63);
        
        //line(0, 63, 127, 0);
        
        //pixel(127,63);
        //pixel(0,0);
        while(1);
        
        while(1)
        {
                for (y = 0; y < 64; y++)
                {
                        //set_x(0);
                        
                        //pixel(0,y);
                        //display_on();
                        //delay_ms(500);
                        
                        for(x = 0; x < 128; x++)
                        {
                                pixel(x,y);
                                //delay_ms(10);
                                display_on();
                        }
                
                }
                
                delay_ms(1000);
                clear_screen();
        
        }
        
        
        /*
        
        
        while(1);
        */
        
        /*
        for (a = 0; a < 64; a++)
        {
                set_x(a);
                write_byte(a,2);
                delay_ms(500);
        }
        */
        
        /*
        set_x(63);
        write_byte(0xFF,1);
        
        set_x(0);
        write_byte(0x55,1);
        
        while(1);
        */
        
        /*
        while(1)
        {
                for (a = 32; a < 128; a++)
                {
                        print_char(a);
                        
                        display_on();
                        delay_ms(50);
                }
        }        
        */

    
}

void ioinit (void)
{
        
    //1 = output, 0 = input
   
        DDRB = 0b00000011; //PB0 and PB1 are outs
        DDRC = ((1<<EN) | (1<<RS) | (1<<R_W) | (1<<RESET) | (1<<CS1) | (1<<CS2));
        PORTC = ((1<<EN) | (1<<RS) | (1<<R_W) | (1<<RESET) | (1<<CS1) | (1<<CS2));
        DDRD = 0b11111100; //PD2-PD7 are also outs.  Ports B and D are the data bus.

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
        /* Set baud rate */
        UBRRH = (unsigned char)(ubrr>>8);
        UBRRL = (unsigned char)ubrr;
        /* Enable receiver and transmitter */
        UCSRB = (1<<RXEN)|(1<<TXEN);
        /* Set frame format: 8data, 2stop bit */
        UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);
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
                
                set_data(0);
                
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


unsigned char print_char(char txt)
{
    short text_array_offset = (txt - 32)*5, j;
    char k, temp;
    
        /*
    if (txt == 10)
    {
        y_offset += 8;
        return(0);
    }

    else if (txt == 13)
    {
        x_offset = 0;
        return(0);
    }
        */
        

    for (j = text_array_offset; j < text_array_offset+6; j++)
    {
                if (x_offset >= 128)
                {
                        x_offset = 0;
                        page_mark++;
                        set_page(page_mark);
                }
        
                if (x_offset >= 64) 
                {
                        side_mark = 2;
                        set_x(x_offset - 64);
                }
                else
                {
                        side_mark = 1;
                        set_x(x_offset);
                }
                
                k = text_array[j];
                //The bit patern is reversed in the GDM12864H display (the lookup table was made for the Nokia display)
                //Rather than rewrite the table, we cheat...
                temp = (((k & 1) << 7) | ((k & 2) << 5) | ((k & 4) << 3) | ((k & 8) << 1) | ((k & 16) >> 1) | ((k & 32) >> 3) | ((k & 64) >> 5) | ((k & 128) >> 7));
        //temp = k;
                if (j == text_array_offset + 5) temp = 0;//blank byte for letter spacing
                write_byte(temp, side_mark);
                
                x_offset++;
                
    }

    if ((x_offset + 6) > 128)
        {
                x_offset = 0;
                side_mark = 1;
                page_mark++;
                
                if (page_mark >= 8) page_mark = 0;
                
                set_page(page_mark);
        }
        
    //else x_offset++;
        
        /*
    if ((y_offset + 8) > 132)
    {
        //clear_screen();
        return 1;
    }
    else return 0;
        */
        
        return 0;

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
void pixel(char x, char y)
{
        static char temp_page, temp_side, temp_x = 0, temp_data1 = 0, temp_data2 = 0;
        
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
        
        temp_data1 |= temp_data2;
        
        set_x(temp_x);//reset this...
        
        write_byte(temp_data1, temp_side);

}

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


char read_byte(char byte, char side)
{
        char data1 = 0, data2 = 0;
        
        if (byte == 0) byte = 63;
        else byte--;
        
        set_x(byte);
        
        PORTC |= ((1 << RESET) | (1 << EN) | (1 << R_W) | (1 << CS1) | (1 << CS2) | (1 << RS));//all high, just to make sure
        
        DDRB = 0;//all inputs
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

        DDRB = 0b00000011; //PB0 and PB1 are outs
        DDRD = 0b11111100; //PD2-PD7 are also outs.  Ports B and D are the data bus.
        
        delay();
        
        return data1;

}




