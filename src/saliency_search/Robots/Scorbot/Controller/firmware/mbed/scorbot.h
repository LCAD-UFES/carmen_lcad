/**
 * Defines main functions, serial commands, and data types
 *
 **/

#ifndef SCORBOT_H
#define SCORBOT_H

#include "mbed.h"
#include "buffered_serial.h"
#include "motor.h"

/******************************************
 *            IO Pin Mappings             *
 ******************************************/
#define MS_1_PIN                        p11
#define MS_2_PIN                        p12
#define MS_3_PIN                        p13
#define MS_4_PIN                        p14
#define MS_5_PIN                        p15
#define MS_6_PIN                        p16
#define MS_7_PIN                        p17
#define PWM_1_PIN                       p21
#define PWM_2_PIN                       p22
#define PWM_3_PIN                       p23
#define PWM_4_PIN                       p24
#define PWM_5_PIN                       p25
#define PWM_6_PIN                       LED1
#define PWM_7_PIN                       p26

/******************************************
 *      Propeller Related Constants       *
 ******************************************/
#define PROPELLER_TX                    p10
#define PROPELLER_RX                    p9
//the length of the propeller message
#define PROPELLER_DATA_SIZE             30
#define PROPELLER_SENTINEL_CHARACTER    'A'
//propeller messages:
#define PROPELLER_GET_VALUES            10
#define PROPELLER_START_MOTORS          14
#define PROPELLER_STOP_MOTORS           15
#define PROPELLER_RESET_ENCODERS        20

/******************************************
 *      I2C to PWM Driver Constants       *
 ******************************************/
#define I2C_SDA                         p28
#define I2C_SCL                         p27
//the length of the propeller message
#define I2C_TO_PWM_DRIVER_ADDRESS       0x20



class Scorbot
{
public:
  Scorbot();
  virtual ~Scorbot();
  
  uint8_t   readMicroswitches();
  uint8_t   readMicroswitch(uint8_t motor_index);
  long      readEncoder(uint8_t motor_index);
  float     readPWMDuty(uint8_t motor_index);
  uint8_t   readControllerStatus();
  uint8_t * readControlParameters(uint8_t motor_index, uint8_t & length);
  long      readTargetPosition(uint8_t motor_index);
  long      readTargetVelocity(uint8_t motor_index);
  void      resetEncoders();
  void      setDestination(uint8_t motor_index, long position, long duration);
  bool      setControlParameters(uint8_t motor_index, uint8_t length, uint8_t *params);
  void      enableMotors();
  void      disableMotors();
  
  
  //static void setExternalPWMDuty(uint8_t external_pwm_address, uint8_t duty);
  //static I2C  g_i2c_to_pwm_driver_;

private:
  void    requestUpdate();
  void    updateStep();
  
  BufferedSerial propeller_;
  Motor * motor_[7];
  Ticker request_update_;
  uint8_t controller_status_;
};

#endif /* SCORBOT_H */