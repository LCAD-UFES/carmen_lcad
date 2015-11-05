#define VOLT_IN   0             // A1
float p = 3.1415926;
float analogValue = 0.0;
float panalogValue = 0.0;
float vout = 0.0;
float pvout = 0.0;
float vin = 0.0;
float pvin = 0.0;
float R1 = 21400.0;    // !! resistance of R1 !!
float R2 = 2145.0;     // !! resistance of R2 !!
float AtoV = 4.9902;//adc with usb

#include <SoftwareSerial.h>

SoftwareSerial softSerial(3, 2); // RX, TX


// ######################################
void setup(void) {
  uint16_t time = millis();
  Serial.begin(57600);

  Serial.println("Arduino Start");
  Serial.println(time, DEC);
  delay(500);
  Serial.println("done");
  delay(1000);
  softSerial.begin(9600);

}
// ######################################
void loop() {
  analogValue = analogRead(VOLT_IN);
  vout = (analogValue * AtoV)/1024.0;
  vin = vout / (R2/(R1+R2));
  Serial.println(analogValue,DEC);
  Serial.println(vin, DEC);
  
  sendFloat(vin);
  delay(1000);  
}
// ######################################
void sendFloat(float f)
{
  uint32_t v = f*1000000;
  byte * b = (byte *) &v;
  byte s = 4;
  softSerial.print('f');//102
  softSerial.write(s);
  softSerial.write(b[0]);
  softSerial.write(b[1]);
  softSerial.write(b[2]);
  softSerial.write(b[3]);
}

