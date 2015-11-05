
CON

        _clkmode        = xtal1 + pll16x
        _xinfreq        = 5_000_000

        Right_Wheel_PWM1 = 10
        Right_Wheel_PWM2 = 11

        Left_Wheel_PWM1 = 8
        Left_Wheel_PWM2 = 9

        Protocol_startHeader = 255
        Protocol_OK = 128
        

        

VAR
        byte duty[2], mdir[2], hold_duty[2]

OBJ                  
  serial : "FullDuplexSerial"
  'motorCtrl : "Simple_Serial"
  motorCtrl : "FullDuplexSerial"  
  adc   :     "CD_ADC0834"

  

PUB start | dataIn, i, command, data
  serial.start(31, 30, 0,115200)

  motorCtrl.start(9, 8,0,9600)

  
  waitcnt(100_000_000 + cnt)
  motorCtrl.tx(170)

  serial.rx
  
  'Send a start cmd to the motor controller
  repeat i from 120 to 255
     motorCtrl.tx(i)
     serial.tx(i)
     waitcnt(30_000_000 + cnt)

  'command := 0
  'data := 20 
  
  'motorCtrl.tx(130)          
  'motorCtrl.tx(0)
  'motorCtrl.tx(64)
  'motorCtrl.tx(66)


  repeat
  
   'A2D Init
  adc.start(0)
  

  
  'Note: motor command is unsigned
  '      so it rolled over from 255 back to 0
  
  repeat
    dataIn := serial.rxtime(1)

    if dataIn == 255   'Start Header
      dataIn := serial.rxtime(1)    
      case dataIn
        10:    ' drive motors
 
          command := serial.rxtime(1)
          data := serial.rxtime(1)

          motorCtrl.tx(130)          
          motorCtrl.tx(command)
          motorCtrl.tx(data)
          motorCtrl.tx( (130 + command + data) & %01111111 )
          serial.tx(Protocol_OK)
               
           
        11:    ' all motors off
          allMotorsOff
          serial.tx(Protocol_OK)

        20:   'Get Battery Voltage
          data := adc.GetADC(0)
          data := (data * 126)/100 + 3  'Convert to voltage
          serial.tx(data)
          serial.tx(Protocol_OK)
        21: 
          data := adc.GetADC(1)
          serial.tx(data)
          serial.tx(Protocol_OK)
        22: 
          data := adc.GetADC(2)
          serial.tx(data)
          serial.tx(Protocol_OK)
        23: 
          data := adc.GetADC(3)
          serial.tx(data)
          serial.tx(Protocol_OK)
           


PUB allMotorsOff
     motorCtrl.tx(128)          
     motorCtrl.tx(0)
     motorCtrl.tx(0)
     motorCtrl.tx( (128 + 0 + 0 ) & %01111111 )

     motorCtrl.tx(128)          
     motorCtrl.tx(4)
     motorCtrl.tx(0)
     motorCtrl.tx( (128 + 4 + 0 ) & %01111111 )
