#include <mcp_dac.h>
#include <SPI.h>

// CAN0 INT and CS
MCP_DAC DAC(9);                               // Set CS to pin 10


void setup()
{
  Serial.begin(115200);  // CAN is running at 500,000BPS; 115,200BPS is SLOW, not FAST, thus 9600 is crippling.
  
  DAC.begin();
  
  Serial.println("MCP4922 Library Simple analog Write...");
}

void loop()
{
  for (int i = 0; i < 0x0FFF; i++){
  	DAC.write(i,DAC_A);
	delay(10);
  }
  for (int i = 0x0FFF; i > 0; i--){
  	DAC.write(i,DAC_A);
	delay(10);
  }
}
