CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000


  TRIGGER_PIN0 = 2
  TRIGGER_PIN1 = 5
  TRIGGER_PIN2 = 8
  TRIGGER_PIN3 = 11
  TRIGGER_PIN4 = 14
  TRIGGER_PIN5 = 23
  TRIGGER_PIN6 = 20
  TRIGGER_PIN7 = 15

  PWM_PIN0 = 1
  PWM_PIN1 = 4
  PWM_PIN2 = 7
  PWM_PIN3 = 10
  PWM_PIN4 = 13
  PWM_PIN5 = 22
  PWM_PIN6 = 19
  PWM_PIN7 = 17
  
OBJ
  serial : "FullDuplexSerial"
  sonar  : "Mini-S_Sonar"     
PUB Main | time0, time1, time2, time3, time4, cmd

  serial.start(31,30,0,115200)

  dira[TRIGGER_PIN0]~~
  dira[TRIGGER_PIN1]~~
  dira[TRIGGER_PIN2]~~
  dira[TRIGGER_PIN3]~~
  dira[TRIGGER_PIN4]~~
  dira[TRIGGER_PIN5]~~
  dira[TRIGGER_PIN6]~~
  dira[TRIGGER_PIN7]~~
  
  dira[PWM_PIN0]~
  dira[PWM_PIN1]~
  dira[PWM_PIN2]~
  dira[PWM_PIN3]~
  dira[PWM_PIN4]~
  dira[PWM_PIN5]~
  dira[PWM_PIN6]~
  dira[PWM_PIN7]~

  outa[TRIGGER_PIN0] := 0
  outa[TRIGGER_PIN1] := 0
  outa[TRIGGER_PIN2] := 0
  outa[TRIGGER_PIN3] := 0
  outa[TRIGGER_PIN4] := 0
  outa[TRIGGER_PIN5] := 0
  outa[TRIGGER_PIN6] := 0
  outa[TRIGGER_PIN7] := 0

  repeat
		cmd := serial.rx

		if (cmd == 0)
			serial.tx(0)
			serial.tx(12)
			serial.str(string("coolingboard"))
			serial.tx(255)

		if (cmd == 97)
    	time0 := sonar.getReading(PWM_PIN0, TRIGGER_PIN0)
    	time1 := sonar.getReading(PWM_PIN1, TRIGGER_PIN1)
    	time2 := sonar.getReading(PWM_PIN2, TRIGGER_PIN2)
    	time3 := sonar.getReading(PWM_PIN3, TRIGGER_PIN3)
    	time4 := sonar.getReading(PWM_PIN4, TRIGGER_PIN4)
    	
			serial.tx(cmd)
			serial.tx(20)
			sendLong(time0)
			sendLong(time1)
			sendLong(time2)
			sendLong(time3)
			sendLong(time4)
			serial.tx(255)
			
    
PUB sendLong(txLong)
	serial.tx((txLong >> 24))
	serial.tx((txLong >> 16))
	serial.tx((txLong >> 08))
	serial.tx((txLong >> 00))

    

  
   
                                                  
    
