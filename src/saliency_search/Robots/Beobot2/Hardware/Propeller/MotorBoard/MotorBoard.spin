CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
	
	ENCODER_PINS  = 20 'Left(20 21) Right(22 23)
DAT
  remote_pins        LONG 6, 5, 4, 3, 2, 1, 0
  remote_pulseWidths LONG 1, 1, 1, 1, 1, 1, 1
VAR
	long encoder_value[2]
	long voltage
	byte fByte[4]
OBJ
  serial 		 : "ComputerLink"
	sabertooth : "SaberTooth"
  encoder		 : "Quadrature Encoder"
	Arduino		 : "FullDuplexSerial"
PUB Main | cmd, motor, speed, timeout, chan

	timeout := CLKFREQ/2

	serial.start(31,30,0,115200)

	sabertooth.Start(15,14,128, timeout)

	encoder.start(ENCODER_PINS,2,2,@encoder_value)
	
	Arduino.start (13,12,0, 9600) 'rx receive from arduino 
	voltage := 0
  repeat
		cmd := serial.rx
			
		if(cmd == 0)
			serial.frameStr(cmd, string("motorboard"))

		elseif(cmd == 100)
			'Set Rotational Speed
			speed := serial.rx
			~speed
			sabertooth.SetRotationalSpeed(speed)

		elseif(cmd == 101)
			'Set Translational Speed
			speed := serial.rx
			~speed
			sabertooth.SetTranslationalSpeed(speed)
			
		elseif(cmd == 103)
			'Get FailSafe Mode 
			serial.frameByte(cmd, sabertooth.getEnabled)

		elseif(cmd == 104)
			chan := serial.rx			
			serial.frameLong(cmd,sabertooth.getChannel(chan))

		elseif(cmd == 105)
			'Get Remote Mode
			serial.frameByte(cmd, sabertooth.getRemoteMode)

		elseif(cmd == 106)
			'Get Remote Speed 
			serial.frameLong(cmd, sabertooth.getRCSpeed)

		elseif(cmd == 107)
			'Get all data
			serial.tx(cmd)
			serial.tx(30)'data size 7 * 2 + 4 + 2*4 + 4*4 + 4
			
			'Send all of the channels
			repeat chan from 0 to 6
				serial.sendShort(sabertooth.getChannel(chan))

			serial.tx(sabertooth.getEmergencyMode)
			serial.tx(sabertooth.getRemoteMode)
			serial.tx(sabertooth.getMotor1Speed)
			serial.tx(sabertooth.getMotor2Speed)

			serial.sendLong(encoder_value[0])
			serial.sendLong(encoder_value[1])

'			serial.sendLong(sabertooth.getTranslationalSpeed)
'			serial.sendLong(sabertooth.getRotationalSpeed)
'			serial.sendLong(sabertooth.getRcTranslationalSpeed)
'			serial.sendLong(sabertooth.getRcRotationalSpeed)
			
			serial.sendLong(voltage)
			serial.tx(255)
		


		elseif(cmd == 108)
			encoder_value[0] := 0
			encoder_value[1] := 0
			encoder.start(ENCODER_PINS,2,2,@encoder_value)

		elseif(cmd == 109) 'get voltage
			serial.frameLong(cmd,voltage)

		updateVoltage



pub updateVoltage|fail,i,psize,volt,inByte

		inByte := Arduino.rxtime(1)
		if(inByte == 102)' char f
			psize := Arduino.rxtime(1)
			if(psize == 4)
				repeat i from 0 to psize-1
					fByte[i] := Arduino.rxtime(1)
					if(fByte[i] == 255)
						fail := true					
				if(fail <> true)
					volt := ((fByte[3]& $ff)<< 24)|((fByte[2] & $ff) << 16) | ((fByte[1] & $ff) << 8)|(fByte[0] & $ff)
					voltage:= volt
					
