#include "motor.h"
#include "scorbot.h"

Serial g_external_pwm(p28,p27);

Motor::Motor(bool internal_pwm, uint8_t external_speed_pin, 
             PinName internal_speed_pin, PinName microswitch_pin,
             PinMode microswitch_mode) : 
  speed_(internal_speed_pin),
  microswitch_(microswitch_pin)
{
  speed_.period_us(PWM_PERIOD);
  
  if (microswitch_mode != PullNone)
    microswitch_.mode(microswitch_mode);
    
  g_external_pwm.baud(115200);
  
  internal_pwm_ = internal_pwm;
  external_speed_pin_ = external_speed_pin;
  external_speed_ = 0;
  update_in_progress_ = false;
  encoder_ = 0;
  desired_move_duration_ = 0;
  desired_distance_ = 0;
  desired_encoder_ = 0;
}

Motor::~Motor()
{
}

uint8_t Motor::readMicroswitch()
{
  return !microswitch_;
}

long Motor::readEncoder()
{
  return encoder_;
}

float Motor::readPWMDuty()
{
  if (internal_pwm_)
    return 2.0*speed_ - 1.0;
  else
    return ((float)external_speed_)/127.0 - 1.0;
}

void Motor::setPWMDuty(float duty)
{
  if(duty > 1.0)       duty = 1.0;
  else if(duty < -1.0) duty = -1.0;
  
  if (internal_pwm_)
    speed_ = duty/2.0 + 0.5;
  else
  {
    external_speed_ = (uint8_t) ((duty + 1.0)*127.0);
    //Scorbot::setExternalPWMDuty(external_speed_pin_, external_speed_);
    g_external_pwm.printf("A%c%cZ", external_speed_pin_, external_speed_);
    //wait_us(200);
  }
}

void Motor::setEncoder(long position)
{
  encoder_ = position;
}

void Motor::setDestination(long encoder_position, long duration_ms)
{
  if (duration_ms > 0)
  {
    desired_encoder_ = encoder_position;
    desired_distance_ = encoder_position - encoder_;
    desired_move_duration_ = ((float) duration_ms) / (1000.0);
    move_duration_.start();
  }
}