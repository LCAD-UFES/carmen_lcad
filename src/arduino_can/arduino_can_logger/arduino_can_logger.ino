#include "src/MCP_CAN/mcp_can.h"
#include "src/MCP_DAC/mcp_dac.h"
#include <SPI.h>

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10

int sensors[3];
byte data[6];

void setup()
{
  Serial.begin(115200);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_STD, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
 
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                     // Configuring pin for /INT input

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
}

void loop()
{
  sensors[0] = analogRead(A0);
  sensors[1] = analogRead(A1);
  sensors[2] = analogRead(A2);

  Serial.print(sensors[0]);
  Serial.print(", ");
  Serial.print(sensors[1]);
  Serial.print(", ");
  Serial.println(sensors[2]);

  data[0]= (byte) (sensors[0]>>8)&0x3;
  data[1]= (byte) sensors[0];

  data[2]= (byte) (sensors[1]>>8)&0x3;
  data[3]= (byte) sensors[1];

  data[4]= (byte) (sensors[2]>>8)&0x3;
  data[5]= (byte) sensors[2];
  
  byte sndStat = CAN0.sendMsgBuf(0x100, 0, 6, data);
  if(sndStat == CAN_OK){
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }
  delay(100);   // send data per 100ms

}
