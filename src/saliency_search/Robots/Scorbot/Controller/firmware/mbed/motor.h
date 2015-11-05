/*
 * Defines main functions, serial commands, and data types
 *
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

#define UNASSIGNED_PWM_PIN      LED4
#define UNASSIGNED_MS_PIN       p20
#define NUM_MOTORS              7

///Period of the PWM signals in microseconds
#define PWM_PERIOD              64
#define UPDATE_PERIOD           5000

class Motor
{
public:
  virtual ~Motor();
  
  /**
   * Update the control algorithm's parameters
   *
   * Takes in raw data from the serial port, checks the length,
   * and deserializes the data.
   * Note: it is the implementer's responsibility to deallocate
   * the data using the delete operator.
   *
   * @param int length
   *  The number of bytes in the data buffer
   * @param uint8_t *data
   *  The serialized parameters
   * @return bool
   *  true  if the data was deserialized successfully
   *  false otherwise
   */
  virtual bool setControlParameters(uint8_t length, uint8_t *data) = 0;
  
  virtual uint8_t * readControlParameters(uint8_t & length) = 0;
  
  virtual long readTargetPosition() = 0;
  
  virtual long readTargetVelocity() = 0;
  
  /**
   * Runs a single step of the control algorithm
   *
   * Updates the PWM value for the given motor
   */
  virtual void updateMotor() = 0;
  
  uint8_t readMicroswitch();
  long    readEncoder();
  float   readPWMDuty();
  void    setEncoder(long position);
  virtual void    setDestination(long encoderPosition, long duration);
  void    setPWMDuty(float duty);

protected:
  Motor();
  Motor(bool internal_pwm, uint8_t external_speed_pin, 
        PinName internal_speed_pin, PinName microswitch_pin,
        PinMode microswitch_mode);
        
  uint8_t external_speed_pin_;
  bool internal_pwm_;
  uint8_t external_speed_;
  /// Hardware PWM controller
  PwmOut speed_;
  /// Microswitch input
  DigitalIn microswitch_;
  /// Reports if either parameters or positions in the process of
  /// being updated over serial
  bool update_in_progress_;
  /// Current encoder value
  long encoder_;
  /// Desired encoder value
  long desired_encoder_;
  /// Desired move distance
  long desired_distance_;
  /// Time since the move began
  Timer move_duration_;
  /// How long the move should take in seconds
  float desired_move_duration_;
};

#endif /* MOTOR_H */