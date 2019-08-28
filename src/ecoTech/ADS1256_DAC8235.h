#ifndef _ADS1256_DAC8235_H_
#define _ADS1256_DAC8235_H_

#include <wiringPi.h>
#include <bcm2835.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>     //exit()
#include <signal.h>     //signal()
#include <time.h>
#include <string.h>


#define DAC0   0x30
#define DAC1   0x34

#define DAC_Value_MAX  65535
#define DAC_VREF  3.3
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

#define DEV_RST_PIN                   18
#define DEV_CS_PIN                    22
#define DEV_CS1_PIN                   23
#define DEV_DRDY_PIN                  17
#define SIGNAL_RELAY_PIN               7
#define CONTROL_ON_OFF_RELAY_PIN       3
#define SIGNAL_ON_OFF_RELAY_PIN        2

#define DEV_Digital_Write(_pin, _value) bcm2835_gpio_write(_pin, _value)
#define DEV_Digital_Read(_pin) bcm2835_gpio_lev(_pin)
#define DEV_SPI_WriteByte(__value) bcm2835_spi_transfer(__value)
#define DEV_SPI_ReadByte() bcm2835_spi_transfer(0xff)
#define DEV_Delay_ms(__xms) bcm2835_delay(__xms)


typedef enum
{
	ADS1256_GAIN_1			= 0,	// GAIN   1
	ADS1256_GAIN_2			= 1,	// GAIN   2
	ADS1256_GAIN_4			= 2,	// GAIN   4
	ADS1256_GAIN_8			= 3,	// GAIN   8
	ADS1256_GAIN_16			= 4,	// GAIN  16
	ADS1256_GAIN_32			= 5,	// GAIN  32
	ADS1256_GAIN_64			= 6,	// GAIN  64
}ADS1256_GAIN;


typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,
    
	ADS1256_DRATE_MAX
}ADS1256_DRATE;


typedef enum                      // Register address, followed by reset the default values
{
	REG_STATUS = 0,	// x1H
	REG_MUX    = 1, // 01H
	REG_ADCON  = 2, // 20H
	REG_DRATE  = 3, // F0H
	REG_IO     = 4, // E0H
	REG_OFC0   = 5, // xxH
	REG_OFC1   = 6, // xxH
	REG_OFC2   = 7, // xxH
	REG_FSC0   = 8, // xxH
	REG_FSC1   = 9, // xxH
	REG_FSC2   = 10,// xxH
}ADS1256_REG;


typedef enum
{
	CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
}ADS1256_CMD;


static const uint8_t ADS1256_DRATE_E[ADS1256_DRATE_MAX] =
{
	0xF0,		// Reset the default values
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};

UBYTE
ADS1256_init(void);

void
ADS1256_SetMode(UBYTE Mode);

void
ADS1256_ConfigADC(ADS1256_GAIN gain, ADS1256_DRATE drate);

UDOUBLE
ADS1256_GetChannalValue(UBYTE Channel);

void
ADS1256_GetAll(UDOUBLE *ADC_Value);

void
Write_DAC8532(UBYTE channel, UWORD Data);

void
DAC8532_Out_Voltage(UBYTE Channel, float Voltage);

UBYTE
DEV_ModuleInit(void);

void
DEV_ModuleExit(void);


#endif
