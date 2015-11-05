#ifndef FEEDBACK_MOTOR_H
#define FEEDBACK_MOTOR_H

#include "motor.h"

#define PARAMETER_MESSAGE_LENGTH    24

class FeedbackMotor : public Motor
{
public:
  FeedbackMotor(bool internal_pwm, uint8_t external_speed_pin, 
                PinName internal_speed_pin, PinName microswitch_pin,
                PinMode microswitch_mode = PullNone);
  virtual ~FeedbackMotor();
  virtual void updateMotor();
  virtual bool setControlParameters(uint8_t length, uint8_t *data);
  virtual uint8_t * readControlParameters(uint8_t & length);
  virtual long readTargetPosition();
  virtual long readTargetVelocity();
  virtual void setDestination(long encoderPosition, long duration);
  
protected:
  float dt_, target_position_, position_error_, target_velocity_, integral_error_;
  float p_gain_, i_gain_, d_gain_, i_saturation_, max_pwm_, velocity_threshold_;
  float pwm_;
  bool control_enabled_;
};

#endif /* FEEDBACK_MOTOR_H */