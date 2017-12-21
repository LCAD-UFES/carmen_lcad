#include "mcp_dac.h"

int16_t sin_lookup_t[180];


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

void MCP_DAC::initSin(int channel, int min, int max){
/*
	set time interrupts
*/
	for (int i=0; i<(180*(max-min)/4096); i++){
		sin_lookup_table[channel][i] = sin_lookup_t[i*(4096/(max-min))] * ((max-min)/2) + ((max+min)/2);
	}
}

void MCP_DAC::sin(int channel, int freq){
	if (!freq){
		//Disable timer interrupts
	}else{
		//enable interrupts
		//set_pre scaler and counters
	}
}

void MCP_DAC::gain(int channel ,bool gain){
	if (gain)
		config[channel] = (!(0x2000) & config[channel]);
	else
		config[channel] = (0x2000 | config[channel]);
}

void MCP_DAC::buffered(int channel, bool buf){
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

void MCP_DAC::write(int channel, uint16_t data){
	SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
	digitalWrite(MCPCS,LOW);
	SPI.transfer16(config[channel] | data);
	digitalWrite(MCPCS,HIGH);
	SPI.endTransaction();
}

