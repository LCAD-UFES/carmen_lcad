// CAN Receive Example
//

#include <mcp_can.h>
#include <mcp_dac.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 2                              // Set INT to pin 2
#define DAC_CS 9
MCP_CAN CAN0(10);                               // Set CS to pin 10

void analogWriteSPI(int value, int port) { //0 DAC_A,1 DAC_B
  // take the SS pin low to select the chip:
  digitalWrite(DAC_CS, LOW);
  //  send in the address and value via SPI:
  SPI.transfer16((port<<15)|0x3000|value);
  // take the SS pin high to de-select the chip:
  digitalWrite(DAC_CS, HIGH);
}

void setup()
{
  Serial.begin(115200);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_STD, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.init_Mask(0,0x07FF0000);
  CAN0.init_Mask(1,0x07FF0000);

  CAN0.init_Filt(0,0x01140000);
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(DAC_CS,OUTPUT);
  digitalWrite(DAC_CS, HIGH);
  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

  SPI.begin();
}

void loop()
{
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
      Serial.print("ID: ");
      Serial.print(rxId, HEX);
      Serial.print(" Data: ");
      for(int i = 0; i<len; i++)           // Print each byte of the data
      {
        if(rxBuf[i] < 0x10)                // If data byte is less than 0x10, add a leading zero
        {
          Serial.print("0");
        }
        Serial.print(rxBuf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      int value = (((rxBuf[0]<<8) | rxBuf[1])&0x0FFF);
      Serial.println(value);
      analogWriteSPI(value,0);
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
