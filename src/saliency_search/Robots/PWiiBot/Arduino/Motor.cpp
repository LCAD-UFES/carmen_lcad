
#include <WProgram.h>
#include <AFMotor/AFMotor.h>

AF_Stepper motor(400, 2);

void setup() {

  motor.setSpeed(40);  // 10 rpm

  //motor.step(100, FORWARD, SINGLE);
  motor.release();
  delay(1000);
}

void loop() {
  delay(1000);
  motor.step(200, FORWARD, INTERLEAVE);
  delay(1000);
  motor.step(200, BACKWARD, INTERLEAVE);
}

int main()
{
  init();
  setup();

  for(;;)
    loop();

  return 0;
}


