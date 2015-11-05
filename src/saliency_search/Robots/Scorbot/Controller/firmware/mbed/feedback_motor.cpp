#include "feedback_motor.h"

extern Serial pc;

FeedbackMotor::FeedbackMotor(bool internal_pwm, uint8_t external_speed_pin, 
                             PinName internal_speed_pin, PinName microswitch_pin,
                             PinMode microswitch_mode) :
  Motor(internal_pwm, external_speed_pin, internal_speed_pin, microswitch_pin, microswitch_mode)
{
  dt_ = 0;
  position_error_ = 0;
  target_position_ = 0;
  target_velocity_ = 0;
  integral_error_ = 0;
  control_enabled_ = false;
  p_gain_ = 0;
  i_gain_ = 0;
  d_gain_ = 0;
  i_saturation_ = 0;
  max_pwm_ = 0;
  velocity_threshold_ = 0;
  pwm_ = 0;
}

FeedbackMotor::~FeedbackMotor() {}

void FeedbackMotor::updateMotor()
{
  if (control_enabled_ && !update_in_progress_ && desired_move_duration_ > 0)
  {
    dt_ = move_duration_.read();
    if(dt_ > desired_move_duration_)
        dt_ = desired_move_duration_;
    
    //find the base trajectory term
    float base_term = (dt_ * desired_distance_) / (desired_move_duration_ * desired_move_duration_);
    //find the position trajectory
    target_position_ = (desired_encoder_ - desired_distance_) +
                       (3.0) * (dt_ * base_term) -
                       (2.0) * (dt_ * dt_ * base_term) / (desired_move_duration_);
    //find the velocity trajectory
    target_velocity_ = (6.0) * (base_term) -
                       (6.0) * (dt_ * base_term) / (desired_move_duration_);

    //find position error
    position_error_ = target_position_ - encoder_;

    //find integral error                       
    integral_error_ = integral_error_ + position_error_;
    if (integral_error_ > i_saturation_)
      integral_error_ = i_saturation_;
    else if (integral_error_ < - i_saturation_)
      integral_error_ = - i_saturation_;   

    //compute pwm                       
    pwm_ = p_gain_*position_error_ + i_gain_*integral_error_ + d_gain_*target_velocity_;
    //add a constant velocity offset if required by static friction
    if (velocity_threshold_ > 0)
    {
      if (pwm_ > 0)
        pwm_ += velocity_threshold_;
      else if (pwm_ < 0)
        pwm_ -= velocity_threshold_;
    }
    
    //limit the maximum and minimum pwm
    if (pwm_ > max_pwm_)
      pwm_ = max_pwm_;
    else if (pwm_ < - max_pwm_)
      pwm_ = - max_pwm_;
    
    setPWMDuty(pwm_);
  }
}

bool FeedbackMotor::setControlParameters(uint8_t length, uint8_t *data)
{
  bool success = false;
  if (length == PARAMETER_MESSAGE_LENGTH)
  {
    control_enabled_ = true;
    update_in_progress_ = true;
    success = true;
    
    p_gain_             = ((float) ((data[0]  << 24) | (data[1]  << 16) | (data[2]  << 8) | data[3] )) / 100000.0;
    i_gain_             = ((float) ((data[4]  << 24) | (data[5]  << 16) | (data[6]  << 8) | data[7] )) / 100000.0;
    d_gain_             = ((float) ((data[8]  << 24) | (data[9]  << 16) | (data[10] << 8) | data[11])) / 100000.0;
    i_saturation_       = ((float) ((data[12] << 24) | (data[13] << 16) | (data[14] << 8) | data[15])) / 100000.0;
    max_pwm_            = ((float) ((data[16] << 24) | (data[17] << 16) | (data[18] << 8) | data[19])) / 100000.0;
    velocity_threshold_ = ((float) ((data[20] << 24) | (data[21] << 16) | (data[22] << 8) | data[23])) / 100000.0;
    
    update_in_progress_ = false;
  }
  delete[] data;
  return success;
}

uint8_t * FeedbackMotor::readControlParameters(uint8_t & length)
{
  length = PARAMETER_MESSAGE_LENGTH;
  uint8_t * data = new uint8_t[PARAMETER_MESSAGE_LENGTH];
  int index = 0;
  long parameter = 0;
  
  //p_gain_
  parameter = (long) (p_gain_*100000.0);
  data[index++] = (parameter >> 24) & 0xFF;
  data[index++] = (parameter >> 16) & 0xFF;
  data[index++] = (parameter >>  8) & 0xFF;
  data[index++] = (parameter      ) & 0xFF;
  //i_gain_
  parameter = (long) (i_gain_*100000.0);
  data[index++] = (parameter >> 24) & 0xFF;
  data[index++] = (parameter >> 16) & 0xFF;
  data[index++] = (parameter >>  8) & 0xFF;
  data[index++] = (parameter      ) & 0xFF;
  //d_gain_
  parameter = (long) (d_gain_*100000.0);
  data[index++] = (parameter >> 24) & 0xFF;
  data[index++] = (parameter >> 16) & 0xFF;
  data[index++] = (parameter >>  8) & 0xFF;
  data[index++] = (parameter      ) & 0xFF;
  //i_saturation_
  parameter = (long) (i_saturation_*100000.0);
  data[index++] = (parameter >> 24) & 0xFF;
  data[index++] = (parameter >> 16) & 0xFF;
  data[index++] = (parameter >>  8) & 0xFF;
  data[index++] = (parameter      ) & 0xFF;
  //max_pwm_
  parameter = (long) (max_pwm_*100000.0);
  data[index++] = (parameter >> 24) & 0xFF;
  data[index++] = (parameter >> 16) & 0xFF;
  data[index++] = (parameter >>  8) & 0xFF;
  data[index++] = (parameter      ) & 0xFF;
  //velocity_threshold_
  parameter = (long) (velocity_threshold_*100000.0);
  data[index++] = (parameter >> 24) & 0xFF;
  data[index++] = (parameter >> 16) & 0xFF;
  data[index++] = (parameter >>  8) & 0xFF;
  data[index++] = (parameter      ) & 0xFF;

  return data;
}

long FeedbackMotor::readTargetPosition()
{
  return target_position_;
}

long FeedbackMotor::readTargetVelocity()
{
  return target_velocity_;
}  

void FeedbackMotor::setDestination(long encoderPosition, long duration)
{
  integral_error_ = 0;
  Motor::setDestination(encoderPosition, duration);
}