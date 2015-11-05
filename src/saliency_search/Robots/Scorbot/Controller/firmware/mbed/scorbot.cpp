#include "scorbot.h"
#include "feedback_motor.h"

//I2C Scorbot::g_i2c_to_pwm_driver_(I2C_SDA, I2C_SCL);

Scorbot::Scorbot() :
  propeller_(PROPELLER_RX, PROPELLER_TX, PROPELLER_DATA_SIZE)
{
  // Instantiate motors
  //                            Interna PWM, Ext Spd Pin, Int Spd Pin, Microswitch Pin, Microswitch Mode
  motor_[0] = new FeedbackMotor(true,        0,           PWM_1_PIN,   MS_1_PIN,        PullUp);
  motor_[1] = new FeedbackMotor(true,        0,           PWM_2_PIN,   MS_2_PIN,        PullUp);
  motor_[2] = new FeedbackMotor(true,        0,           PWM_3_PIN,   MS_3_PIN,        PullUp);
  motor_[3] = new FeedbackMotor(true,        0,           PWM_4_PIN,   MS_4_PIN,        PullUp);
  motor_[4] = new FeedbackMotor(true,        0,           PWM_5_PIN,   MS_5_PIN,        PullUp);
  motor_[5] = new FeedbackMotor(false,       11,          PWM_6_PIN,   MS_6_PIN,        PullUp);
  motor_[6] = new FeedbackMotor(true,        0,           PWM_7_PIN,   MS_7_PIN);
  
  //Set all PWMs to 0
  for (int i = 0; i < NUM_MOTORS; i++)
    motor_[i]->setPWMDuty(0);
  
  //Setup serial communication with Propeller
  propeller_.baud(115200);
  controller_status_ = 0;
  
  //ensure the motors are off
  propeller_.putc(PROPELLER_STOP_MOTORS);
  
  //Set the update step to occur at 100Hz (UPDATE_PERIOD = 10000us, defined in motor.h)
  propeller_.setFullCallback(this, &Scorbot::updateStep);
  request_update_.attach_us(this, &Scorbot::requestUpdate, UPDATE_PERIOD);
}

Scorbot::~Scorbot()
{
  for (int index = 0; index < NUM_MOTORS; index++)
    delete motor_[index];
}

uint8_t Scorbot::readMicroswitches()
{
  uint8_t switches = 0;
  for (int index = 0; index < 7; index++)
    switches |= (motor_[index]->readMicroswitch() << index);
    
  return switches;
}

uint8_t Scorbot::readMicroswitch(uint8_t motor_index)
{
  return motor_[motor_index]->readMicroswitch();
}

long Scorbot::readEncoder(uint8_t motor_index)
{
  return motor_[motor_index]->readEncoder();
}

float Scorbot::readPWMDuty(uint8_t motor_index)
{
  return motor_[motor_index]->readPWMDuty();
}

uint8_t Scorbot::readControllerStatus()
{
  return controller_status_;
}

uint8_t * Scorbot::readControlParameters(uint8_t motor_index, uint8_t & length)
{
  return motor_[motor_index]->readControlParameters(length);
}

long Scorbot::readTargetPosition(uint8_t motor_index)
{
  return motor_[motor_index]->readTargetPosition();
}

long Scorbot::readTargetVelocity(uint8_t motor_index)
{
  return motor_[motor_index]->readTargetVelocity();
}

void Scorbot::resetEncoders()
{
  propeller_.putc(PROPELLER_RESET_ENCODERS);
}

void Scorbot::setDestination(uint8_t motor_index, long position, long duration)
{
  motor_[motor_index]->setDestination(position, duration);
}

bool Scorbot::setControlParameters(uint8_t motor_index, uint8_t length, uint8_t *params)
{
  return motor_[motor_index]->setControlParameters(length, params);
}

void Scorbot::enableMotors()
{
  propeller_.putc(PROPELLER_START_MOTORS);
}

void Scorbot::disableMotors()
{
  propeller_.putc(PROPELLER_STOP_MOTORS);
}

void Scorbot::requestUpdate()
{
  //get rid of any lingering data
  propeller_.flushBuffer();
  //request an update
  propeller_.putc(PROPELLER_GET_VALUES);
}

void Scorbot::updateStep()
{
  //if the message is the correct length, and the sentinel character
  //is correct, then we may proceed to process the data
  if (propeller_.getc() == PROPELLER_SENTINEL_CHARACTER)
  {
    /********************************************
     *      Parse the Propeller Message         *
     ********************************************/
    //parse the encoder values
    for (int index = 0; index < 7; index++)
      motor_[index]->setEncoder(propeller_.readLong());
    //parse the controller status
    controller_status_ = propeller_.getc();
    
    /********************************************
     *          Perform Control Update          *
     ********************************************/
    for (int index = 0; index < 7; index++)
      motor_[index]->updateMotor();
  }
}

/*void Scorbot::setExternalPWMDuty(uint8_t external_pwm_address, uint8_t pwm_duty)
{
  char i2c_message[2];
  i2c_message[0] = external_pwm_address;
  i2c_message[1] = pwm_duty;
  g_i2c_to_pwm_driver_.write(I2C_TO_PWM_DRIVER_ADDRESS, i2c_message, 2);
}*/