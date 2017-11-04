#include "mcp_dac.h"

MCP_DAC::MCP_DAC(int _CS)
{
    MCPCS = _CS;
	config[0] = 0x3000;
	config[1] = 0xB000;
    pinMode(MCPCS, OUTPUT);
    digitalWrite(MCPCS,HIGH);
}

void MCP_DAC::begin(){
	SPI.begin();
	off(DAC_A);
	off(DAC_B);
}

void MCP_DAC::gain(bool gain, int channel){
	if (gain)
		config[channel] = (!(0x2000) & config[channel]);
	else
		config[channel] = (0x2000 | config[channel]);
}

void MCP_DAC::buffered(bool buf, int channel){
	if (buf)
		config[channel] = (!(0x4000) & config[channel]);
	else
		config[channel] = (0x4000 | config[channel]);
}

void MCP_DAC::off(int channel){
	SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
	digitalWrite(MCPCS,LOW);
	SPI.transfer16(channel << 15);
	digitalWrite(MCPCS,HIGH);
	SPI.endTransaction();
}


void MCP_DAC::write(uint16_t data, int channel){
	SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
	digitalWrite(MCPCS,LOW);
	SPI.transfer16(config[channel] | data);
	digitalWrite(MCPCS,HIGH);
	SPI.endTransaction();
}

