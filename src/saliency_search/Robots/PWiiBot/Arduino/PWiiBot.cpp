#include <WProgram.h>
#include <Wire/Wire.h>
#include <string.h>
#include <stdio.h>
#include <AFMotor/AFMotor.h>

AF_DCMotor motor1(1, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm


uint8_t outbuf[6]; // array to store arduino output
char storebuf[200]; //array to store arduino input (transmitted from PC)
int cnt = 0;
int ledPin = 13;

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
  char
nunchuk_encode_byte (char x)
{
  x = x - 0x17;
  x = (x ^ 0x17);
  return x;
}

void receiveEvent (int howMany)
{

  while (Wire.available ())
  {
    if (howMany == 2)
    {
      int addr = Wire.receive ();        // receive byte as an integer
      int data = Wire.receive ();        // receive byte as an integer

      if (addr == 1)
      {
        Serial.print("Set motor 1 dir to ");
        Serial.print(data);
        Serial.print("\r\n");
        motor1.run(data&(FORWARD|BACKWARD|BRAKE|RELEASE));
      }

      if (addr == 2)
      {
        Serial.print("Set motor 1 to ");
        Serial.print(data);
        Serial.print("\r\n");
        motor1.setSpeed(data);     // set the speed to 200/255
      }

      if (addr == 3)
      {
        Serial.print("Set motor 2 dir to ");
        Serial.print(data);
        Serial.print("\r\n");
        motor2.run(data&(FORWARD|BACKWARD|BRAKE|RELEASE));

      }

      if (addr == 4)
      {
        Serial.print("Set motor 2 to ");
        Serial.print(data);
        Serial.print("\r\n");
        motor2.setSpeed(data);     // set the speed to 200/255
      }

    }
    //digitalWrite (ledPin, HIGH);        // sets the LED on
  }
  //strncat (storebuf, ":", 1);  // add a break after each set of input data
}

void requestEvent ()
{
  // Send some data back to the wiimote to be transmitted back to PC
  outbuf[0] = nunchuk_encode_byte (125);        // joystick X
  outbuf[1] = nunchuk_encode_byte (126);        // joystick Y
  outbuf[2] = nunchuk_encode_byte (227);        // Axis X
  outbuf[3] = nunchuk_encode_byte (241);        // Axis Y
  outbuf[4] = nunchuk_encode_byte (140);        // Axis Z
  outbuf[5] = nunchuk_encode_byte (1);        // Press C button, byte[5] is buttons
  //C,Z and accelaration data
  //outbuf[5] = nunchuk_encode_byte(2); // Press Z button
  //outbuf[5] = nunchuk_encode_byte(0); // Press Z and C button

  Wire.send (outbuf, 6);        // send data packet
}

  void
setup ()
{
  beginSerial (19200);
  Serial.print ("Finished setup\n");
  Wire.begin (0x52);                // join i2c bus with address 0x52
  Wire.onReceive (receiveEvent);        // register event
  Wire.onRequest (requestEvent);        // register event

  motor1.setSpeed(0);     // set the speed to 200/255
  motor2.setSpeed(0);     // set the speed to 200/255

}



  void
loop ()
{
  delay (100);
}




int main()
{
  init();
  setup();

  for(;;)
    loop();

  return 0;
}


