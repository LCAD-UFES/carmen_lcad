#include <WProgram.h>
#include <pins_arduino.h>

#define rcOut 9 //pin to output ppm signal
#define pushButton 2
#define rightSwitch 3
#define leftSwitch 4

unsigned int chIn[4]; //Data from pots
unsigned int serOutPls[10]; //Pulse width for each channel

enum SERIAL_SM {HEADER1, HEADER2, MSB, LSB};

//For ppm values
enum PPM_STATE {PPM_FRAME_LOW, PPM_FRAME_HIGH, PPM_HIGH, PPM_LOW};
PPM_STATE ppmState = PPM_FRAME_LOW;
int ppmChan = 0;

void setupPPMTimer()
{

  TCCR1A = 0x00; //Normal timer mode

  //Set the timer prescaler to 2, this gives us a precision of 1/2 uS
  TCCR1B = (1<<CS11) | (0<<CS10);

  //Timer2 Overflow Interrupt Enable
  TIMSK1 = 1<<TOIE2;

  //load the timer for its first cycle
  TCNT1=0;
}

ISR(TIMER1_OVF_vect) {
  //Capture the current timer value. This is how much error we
  //have due to interrupt latency and the work in this function
  //unsigned char latency=TCNT2;

  int delay = 0;
  switch (ppmState)
  {
    case PPM_FRAME_LOW:
      digitalWrite(rcOut,LOW);
      delay = 304; //10 ms
      ppmChan = 0;
      ppmState = PPM_FRAME_HIGH;
      break;
    case PPM_FRAME_HIGH:
      digitalWrite(rcOut,HIGH);
      delay = 10000; //10 ms
      ppmState = PPM_LOW;
      break;
    case PPM_LOW:
      digitalWrite(rcOut,LOW);
      delay = 300; //304 microseconds
      ppmState = PPM_HIGH;
      break;
    case PPM_HIGH:
      digitalWrite(rcOut,HIGH);
      delay = serOutPls[ppmChan];
      ppmChan++;
      if (ppmChan > 8)
        ppmState = PPM_FRAME_LOW;
      else
        ppmState = PPM_LOW;
      break;
  }

  //Reload the timer and correct for latency.
  TCNT1=0xFFFF-((delay*2)-16);
}

void setup()
{
  Serial.begin(115200);
  pinMode(rcOut, OUTPUT); //Pin rcOut as output
  digitalWrite(rcOut, HIGH);

  pinMode(rightSwitch, INPUT);
  pinMode(leftSwitch, INPUT);
  pinMode(pushButton, INPUT);
  digitalWrite(rightSwitch, HIGH); //enable pullup resistor
  digitalWrite(leftSwitch, HIGH); //enable pullup resistor
  digitalWrite(pushButton, HIGH); //enable pullup resistor

  setupPPMTimer();

  //Reset all channel to low values
  for(int i=0; i<9; i++)
    serOutPls[i] = 500;

}

void outputData()
{
  //Serial.print((char)255);
  Serial.print((char)0);
  Serial.print((char)9);
  for(int i=0; i<4; i++)//Loop to print and clear all the channel readings
  {
    Serial.print((char)chIn[i]);
    Serial.print((char)(chIn[i] >> 8));
    //Serial.print((char) serOutPls[i]);
    //Serial.print((char)(serOutPls[i] >> 8));
  }
  char switches =  *portInputRegister(digitalPinToPort(0)) & 0xFC;
  Serial.print(switches);
  Serial.print((char)255);
}

void getData()
{
  static SERIAL_SM serialSM = HEADER1;
  static int data = 0;
  static int numDataRead = 0;
#define MAX_NUM_DATA 4


  int na = Serial.available();

  for(int i=0; i<na; i++)
  {
    int inData = Serial.read();
    if (inData == -1) break;

    //State machine to read data
    switch (serialSM)
    {
      //Get headers
      case HEADER1:
        if (inData == 255) serialSM = HEADER2;
        break;
      case HEADER2:
        if (inData == 255) serialSM = MSB;
        break;
      case MSB:
        data = inData << 8;
        serialSM = LSB;
        break;
      case LSB:
        data += inData;

        //senity check
        if (data > 300 && data < 2000)
          serOutPls[numDataRead] = data;
        numDataRead++;
        if (numDataRead < MAX_NUM_DATA)
        {
          serialSM = MSB;
        } else {
          numDataRead = 0;
          serialSM = HEADER1;
        }
        break;
    }

  }


}

void outputPPM()
{
  //Output servo pulse
  for(int i=0; i<9; i++)
  {
    digitalWrite(rcOut, LOW);
    delayMicroseconds(304);
    digitalWrite(rcOut, HIGH);
    delayMicroseconds(serOutPls[i]);
  }
  digitalWrite(rcOut, LOW);
  delayMicroseconds(304);
  digitalWrite(rcOut, HIGH);

}

void loop()
{
  //Get A2D Values and convert to ppm pulse width
  chIn[0] = 500 + analogRead(2);
  chIn[1] = 500 + analogRead(0);
  chIn[2] = 500 + analogRead(1);
  chIn[3] = 500 + (1023-analogRead(3));

  //outputPPM(); //output the ppm signal

  ////Output serial data


  outputData();

  //Get data from computer if switch is enabled
  if (digitalRead(rightSwitch)==LOW)
  {
    getData();
  } else {
    //Read from pots
    serOutPls[0] = chIn[0];    // Throtle
    serOutPls[1] = chIn[1];    // Ailron
    serOutPls[2] = chIn[2];    //Elevator
    serOutPls[3] = chIn[3];    // Rader
  }
}

int main(void)
{
  init();
  setup();

  for (;;)
    loop();

  return 0;
}

