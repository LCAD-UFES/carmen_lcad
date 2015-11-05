
#include <WProgram.h>

/*
   Output pins from the GP2D02
   1) Ground (black)
   2) Input (Green) --> A1 (pClk)
   3) +5V (Red)
   4) Output (Yellow) --> A0 (pDat)

*/

int pDat = 14; //A0
int pClk = 15; //A1


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
  delay(1);
  return dataIn;
}

void setup() {
  beginSerial(19200);
  setupSensor();
  Serial.println("Setup Done");
}

void loop() {

  int dist = readSensor();
  Serial.print("IR: ");
  Serial.println(dist);

}

int main()
{
  init();
  setup();

  for(;;)
    loop();

  return 0;
}


