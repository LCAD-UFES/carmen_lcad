
#include <WProgram.h>
#include <AFMotor/AFMotor.h>

AF_Stepper motor(48, 2);

/*
   Output pins from the GP2D02
   1) Ground (black)
   2) Input (Green) --> A1 (pClk)
   3) +5V (Red)
   4) Output (Yellow) --> A0 (pDat)

*/

int pDat = 14; //A0
int pClk = 15; //A1

int numSteps = 0;

void setupSensor()
{
  pinMode(pClk, OUTPUT);
  pinMode(pDat, INPUT);
  digitalWrite(pClk, LOW);
}

byte readSensor()
{
  byte dataIn = 0;
  digitalWrite(pClk, LOW);
  while(digitalRead(pDat) != HIGH);

  digitalWrite(pClk, HIGH);
  for(int i=0; i<8; i++)
  {
    digitalWrite(pClk, LOW);
    dataIn = (dataIn << 1) | digitalRead(pDat);
    digitalWrite(pClk, HIGH);

  }
  delay(2);
  digitalWrite(pClk, LOW);
  return dataIn;
}

void setupMotor()
{
  motor.setSpeed(10);  // 10 rpm
  motor.release();
  delay(1000);
}

void setup() {
  beginSerial(19200);
  setupSensor();
  setupMotor();
  Serial.println("Setup Done");
}

void printDist(byte i, byte dist)
{
  Serial.print((byte)255);
  Serial.print((byte)0);
  Serial.print((byte)255);
  Serial.print((byte)i);
  Serial.print((byte)dist);
}



void loop() {

  int nStep = 10;
  int scan = 200;

  for(int i=0; i<20; i++)
  {
    byte dist = readSensor();
    printDist(i,dist);
    motor.step(10, FORWARD, SINGLE);
  }

  for(int i=20; i>0; i--)
  {
    motor.step(10, BACKWARD, SINGLE);
    byte dist = readSensor();
    printDist(i,dist);
  }

}

int main()
{
  init();
  setup();

  for(;;)
    loop();

  return 0;
}


