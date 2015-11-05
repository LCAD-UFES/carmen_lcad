CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
	
	ENCODER_PINS  = 20 'Left(20 21) Right(22 23)
DAT
  remote_pins        LONG 6, 5, 4, 3, 2, 1, 0
  remote_pulseWidths LONG 1, 1, 1, 1, 1, 1, 1
VAR
	long encoder_value[2]
OBJ
  serial 		 : "ComputerLink"
  bserial 	 : "ComputerLink"
	sabertooth : "SaberTooth"
  encoder		 : "Quadrature Encoder"

PUB Main | cmd, bcmd, motor, speed, timeout, chan

	timeout := CLKFREQ/2

	serial.start(31,30,0,115200)'USB Serial
	bserial.start(25,24,0,115200)'Bluetooth Serial

	sabertooth.Start(15,14,128, timeout)

	encoder.start(ENCODER_PINS,2,2,@encoder_value)
	
  repeat
	  'Check USB serial for 100ms,if time out we will check bluetooth serial
		cmd := serial.rxtime(10)
		if(cmd <> -1)
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
				serial.tx(26)'data size 7 * 2 + 4 + 2*4 + 4*4
				
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
				
				serial.tx(255)
			elseif(cmd == 108)
				encoder_value[0] := 0
				encoder_value[1] := 0
				encoder.start(ENCODER_PINS,2,2,@encoder_value)

		'check bluetooth serial	
		bcmd := bserial.rxtime(10)
		if(bcmd <> -1)
			if(bcmd == 0)
				bserial.frameStr(bcmd, string("motorboard"))
	
			elseif(bcmd == 100)
				'Set Rotational Speed
				speed := bserial.rx
				~speed
				sabertooth.SetRotationalSpeed(speed)
	
			elseif(bcmd == 101)
				'Set Translational Speed
				speed := bserial.rx
				~speed
				sabertooth.SetTranslationalSpeed(speed)
				
			elseif(bcmd == 103)
				'Get FailSafe Mode 
				bserial.frameByte(bcmd, sabertooth.getEnabled)
	
			elseif(bcmd == 104)
				chan := bserial.rx			
				bserial.frameLong(bcmd,sabertooth.getChannel(chan))
	
			elseif(bcmd == 105)
				'Get Remote Mode
				bserial.frameByte(bcmd, sabertooth.getRemoteMode)
	
			elseif(bcmd == 106)
				'Get Remote Speed 
				bserial.frameLong(bcmd, sabertooth.getRCSpeed)
	
			elseif(bcmd == 107)
				'Get all data
				bserial.tx(bcmd)
				bserial.tx(26)'data size 7 * 2 + 4 + 2*4 + 4*4
				
				'Send all of the channels
				repeat chan from 0 to 6
					bserial.sendShort(sabertooth.getChannel(chan))
	
				bserial.tx(sabertooth.getEmergencyMode)
				bserial.tx(sabertooth.getRemoteMode)
				bserial.tx(sabertooth.getMotor1Speed)
				bserial.tx(sabertooth.getMotor2Speed)
	
				bserial.sendLong(encoder_value[0])
				bserial.sendLong(encoder_value[1])
	
	'			bserial.sendLong(sabertooth.getTranslationalSpeed)
	'			bserial.sendLong(sabertooth.getRotationalSpeed)
	'			bserial.sendLong(sabertooth.getRcTranslationalSpeed)
	'			bserial.sendLong(sabertooth.getRcRotationalSpeed)
				
				bserial.tx(255)
			elseif(bcmd == 108)
				encoder_value[0] := 0
				encoder_value[1] := 0
				encoder.start(ENCODER_PINS,2,2,@encoder_value)
