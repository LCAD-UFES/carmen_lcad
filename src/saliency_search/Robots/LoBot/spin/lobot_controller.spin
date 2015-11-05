'  enum MOTOR {SPEED,DIRECTION};
'  enum COMMANDS {SAFE_MODE=9, SET_MOTORS=10, MOTORS_OFF=11, GET_PWM=14};

CON

  _clkmode = xtal1 + pll16x    'Run at the full 80MHz
  _xinfreq = 5_000_000
  Protocol_OK = 128
  SAFE_MODE = 9
  SET_MOTORS = 10
  MOTORS_OFF = 11
  GET_PWM = 14
    
VAR
        byte Index
         '
OBJ 
 serial : "FullDuplexSerial"
 'read   : "ServoInput"  
 num    : "Numbers"
 SERVO : "Servo32v3"
DAT
  
  pins        LONG 0, 1, 2 
  pulseWidths LONG 1, 1, 1
  servoPins   LONG 0,14,15
  cp LONG 1500, 1500, 1500   'Current Pluse
  'center LONG 1500, 1762, 1500 'middle value
  'range 2000~1000 for speed
  'range 1972~1432 for direction
  center LONG 1500, 1500, 1500 'middle value
PUB start | dataIn, counter, pin, motor, data    
  serial.start(31, 30, 0,38400)
  num.init 
  'read.start(@pins,3,@pulseWidths)
  
  serial.str(string("System Start!!!"))
  serial.str(string(10,13)) 
  repeat pin from 0 to 2 
    servo.Set(servoPins[pin],center[pin])  
  servo.Start
  'servo.start(cp[0],23,cp[1],22,cp[2],21,0,20)  
  repeat
    'counter := cnt + 10_000_000  'Run our loop at ~8Hz (80e6/10e6)  
    dataIn := serial.rxtime(10)
    'isRCopen
     
    'if(pulseWidths[2] > 1500)
      'updateServos
                           
    case dataIn
      'Speed 
      "1":
        incDuty(0)
      "q":
        decDuty(0)
      "a":
        forwardOrHold(0)         
      "z":
        reverseOrHold(0)        
      'Direction
      "2":
        incDuty(1)
      "w":
        decDuty(1)
      "x":
        forwardOrHold(1)       
      "s":
        reverseOrHold(1)        
       'Gear
      'third motor 
      "3":
        incDuty(2)
      "e":
        decDuty(2)
      " ":
        stopAllServos
      "b":
        normalStopAllServos  
      "v":  
        showServosValues
        
      'Listen from computer program  
      SET_MOTORS:    '10 drive motors       
            motor := serial.rxtime(1) 'Speed or Direction            
            data := serial.rxtime(1) ' -100 ~ 100       
            setServoValue(motor,data*5+center[motor])
            serial.tx(Protocol_OK)
      MOTORS_OFF:    '11 Stop All Motor
        stopAllServos
        serial.tx(Protocol_OK)            

'Display the motor value
PUB showServosValues  | i2
repeat i2 from 0 to 2
  'Index := 0
  serial.str(string("["))
  serial.dec(cp[i2])
  serial.str(string(" | "))   
  serial.dec(pulseWidths[i2])
  serial.str(string("]"))    
  serial.str(string(":"))
  serial.dec(i2)
  serial.str(string("  |"))
serial.str(string(10,13))


PUB incDuty(I)
  serial.str(string("Start Inc servo",10,13))        
  if(cp[I]<center[I]+501)
    cp[I] := cp[I]+10
    servo.Set(servoPins[I], cp[I])
  showServoValue(I)   
  serial.str(string("Finish Inc servo",10,13))  
PUB decDuty(I)
  if(cp[I] > center[I]-501)
    cp[I] := cp[I]-10
  servo.Set(servoPins[I], cp[I])
  showServoValue(I)

'The motor drives as same way as servo
'This function needs which motors and the servo value 1000~2000
PUB setServoValue(motor,cycle)
  cp[motor] := cycle
  servo.Set(servoPins[motor],cp[motor])  

PUB forwardOrHold(I)

PUB reverseOrHold(I)

PUB normalStopAllServos |i5
  repeat i5 from 0 to 2
    servo.Set(servoPins[i5], center[i5])
    cp[i5] := center[i5]
       
PUB stopAllServos | i3
  repeat i3 from 0 to 2
    'E-Break     
    if(Index == 0)
      if(cp[i3] > center[i3]+59)' When it is forwarding
        servo.Set(servoPins[i3], 4300 - 2*cp[i3])
        waitcnt(20000_000 + cnt) 'wait ????ms to reverse motor
      elseif(cp[i3] < center[i3]-69)'when it's backward
        servo.Set(servoPins[i3], 4600 - 2*cp[i3])
        waitcnt(9000_000 + cnt) 'wait ?000ms to reverse motor
                              
    servo.Set(servoPins[i3], center[i3])
    cp[i3] := center[i3]
  'serial.str(string("Finish Stop all servos",10,13))        
 
PUB showServoValue(I)
  serial.str(string("ServoMove["))
  serial.dec(cp[I])
  serial.str(string(" | "))
  serial.dec(pulseWidths[I])          
  serial.str(string("]"))
  serial.str(string(10,13))

PUB isRCopen :yesno
  if(pulseWidths[0]>center[0]-1000 and pulseWidths[0] < center[0]+1000)
    serial.str(string("RC signal found"))
    serial.str(string(10,13))     
    return TRUE
  serial.str(string("RC signal lost"))
  serial.str(string(10,13))
  'stopAllServos
  return FALSE          