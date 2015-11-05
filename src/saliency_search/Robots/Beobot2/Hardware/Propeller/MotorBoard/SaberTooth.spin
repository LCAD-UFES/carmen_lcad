{
        SetMotor(1, 64) -> Turn off motor 1
        SetMotor(2, 64) -> Turn off motor 2
        SetMotor(1, 1)  -> Turn motor 1 to full reverse
        SetMotor(1, 1)  -> Turn motor 2 to full reverse
        SetMotor(1, 127)        -> Turn motor 1 to full forward
        SetMotor(1, 127)        -> Turn motor 2 to full forward
}

CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

				RAMP_INC		 = 5
        CH_ROTATION  = 0 'CH1 1109~1932
        CH_SPEED     = 1 'CH2 1044~1864
        CH_SPEED_CAP = 2 'CH3 1042~1863
        CH_ESTOP     = 4 'CH5 SW-F 899,2005
        CH_VR        = 5 'CH6 965~2072
        CH_MODE      = 6 'CH7 SW-G 2073,1520,966

				BUMPER_PINS = 18 'we connect bumper to both I/O
				BUMPER_GND  = 19 'because of screw block,so set one to GND
        PW_FAILSAFE = 1900
        LCD_PIN = 16
DAT
  remote_pins        LONG 6, 5, 4, 3, 2, 1, 0
  remote_pulseWidths LONG 1, 1, 1, 1, 1, 1, 1

VAR
        long stack[128]
        long motor1speed
        long motor2speed
        long rcspeed
				long rcdirection 
        long timestamp
        byte remoteMode
        byte motorEnable
        long emergencyMode
        long rot_vel_req
        long tran_vel_req
				long ramp_cnt

OBJ
        SabertoothSerial : "Simple_Serial"
        Serial 					 : "Simple_Serial"
        remote           : "ServoInput"
        lcd              : "GraphicLcd"

'Start up the sabertooth runtime controller
PUB Start(EStopPin, TxPin, Address, Timeout) : success | cog
        'Start the sabertooth runtime in a new cog
        success := (cog := cognew(SaberToothRuntime(EStopPin, TxPin, Address, Timeout), @stack) +1)

PUB GetRemoteMode : mode
        timestamp := cnt
        return remoteMode

PUB GetChannel(chan) : chanVal
        timestamp := cnt
        return remote_pulseWidths[chan]

PUB GetEnabled : en
        timestamp := cnt
        return motorEnable

PUB GetRCSpeed : rcs
        timestamp := cnt
        return rcspeed 

PUB GetEmergencyMode : em
        timestamp := cnt
        return emergencyMode
'for bumper
PUB SetEmergencyMode(mode) 
				emergencyMode := mode 
        timestamp := cnt

PUB GetMotor1Speed :spd
        timestamp := cnt
        return motor1speed

PUB GetMotor2Speed :spd
        timestamp := cnt
        return motor2speed

'Request a change in rotational speed
PUB SetRotationalSpeed(Speed) 

        if(Speed > 100)
                Speed := 100
        elseif(Speed < -100)
                Speed := -100

        rot_vel_req := Speed

        'Reset the watchdog timestamp
        timestamp := cnt

'Request a change in translational speed
PUB SetTranslationalSpeed(Speed)

        if(Speed > 100)
                Speed := 100
        elseif(Speed < -100)
                Speed := -100

        tran_vel_req := Speed

        'Reset the watchdog timestamp
        timestamp := cnt

'The sabertooth runtime controller - takes care of servicing speed
'change requests, and minding the timeout
PRI SaberToothRuntime(EStopPin, TxPin, Address, Timeout)|speed,direction, rc_speed_cap, trans_vel, rot_vel, rot_vel_cap,targetSpeed1, targetSpeed2, estop, emergencyTimer,lcdcount,channel' emergencyMode

        'Turn off the e-stop
        dira[EStopPin] := 1
        outa[EStopPin] := 1

				'Initialize the bumper
				dira[BUMPER_PINS]:= 0 'set pin as input
				dira[BUMPER_GND] := 1 'set pin as output
				outa[BUMPER_GND] := 0	'set to low
        
				motor1speed  := 64
        motor2speed  := 64
        targetSpeed1 := 64
        targetSpeed2 := 64

        'Initialize the serial port
        SabertoothSerial.init(0, TxPin, 2400)
        Serial.init(31, 30, 115200)

        remote.start(@remote_pins,7,@remote_pulseWidths)

        estop := True 
        emergencyMode := True 

        repeat
								'check bumper first
								if ina[BUMPER_PINS] == 0
									setMotor(64,64,emergencyMode)'then kill it right away
									waitcnt(clkfreq/4+cnt)
									if(!emergencyMode)
										setMotor(54,74,False)'when hit bumper,reverse first
										waitcnt(clkfreq/8+cnt)
										setMotor(64,64,emergencyMode)'then kill it right away
									estop := True
									emergencyMode := True

								'Safety Check on Channel 3
								if(remote_pulseWidths[CH_SPEED_CAP] > PW_FAILSAFE)
									estop := True
									emergencyMode := True
								if(remote_pulseWidths[CH_SPEED_CAP] < 800)
									estop := True
									emergencyMode := True

								'All 7 channel validation
								repeat channel from 0 to 5
									if(remote_pulseWidths[channel] < 700)
										estop := True
										emergencyMode := True
									
									
								'Determine the remote mode
                'if(remote_pulseWidths[CH_MODE] > 2500 or remote_pulseWidths[CH_MODE] <30)
                '        remoteMode := -1 'it's out of range
                if(remote_pulseWidths[CH_MODE] > 1900 or remote_pulseWidths[CH_MODE] <700)
                        remoteMode := 1 'Manual Mode
                elseif(remote_pulseWidths[CH_MODE] > 1300 and remote_pulseWidths[CH_MODE] < 1700)
                        remoteMode := 2 'Semi-Auto Mode
                elseif(remote_pulseWidths[CH_MODE] > 700 and remote_pulseWidths[CH_MODE] < 1300)
                        remoteMode := 3 'Auto Mode
                
                'Calculate the speed limit for the robot
                rc_speed_cap := remote_pulseWidths[CH_SPEED_CAP]
                rc_speed_cap := 100-((rc_speed_cap - 1041) * 100)/(1750 - 1041)
                if(rc_speed_cap > 100)
                        rc_speed_cap := 100
                if(rc_speed_cap < 0)
                        rc_speed_cap := 0

                'Calculate the speed limit for the robot
                rot_vel_cap := ((remote_pulseWidths[CH_VR] - 900) * 100)/(2073- 900)
                if(rot_vel_cap> 100)
                        rot_vel_cap:= 100
                if(rot_vel_cap< 0)
                        rot_vel_cap:= 0

								'Recover from emergency state
                'The user can get out of emergency mode by holding down the emergency stop
                'button for two seconds with the speed cap at 0
                if(emergencyMode == True)
                        waitcnt(clkfreq/2+cnt)
                        if(estop == False or rc_speed_cap > 0)
                                emergencyTimer := cnt
                        elseif(cnt > emergencyTimer+clkfreq/4)
                                repeat while remote_pulseWidths[CH_ESTOP] > 1500
                                emergencyMode := False
                                waitcnt(clkfreq/2+cnt)

								'Remote Kill Switch!!! Very important
                if(remote_pulseWidths[CH_ESTOP] > 1500)
                        estop := True
                else
                        estop := False


                if(estop == True and emergencyMode == False)
                        emergencyMode := True
								''''======================Finish Check=================================''''
								
                ''''Grab the translational velocity stick
                trans_vel := remote_pulseWidths[CH_SPEED]
                trans_vel := (200 - ((trans_vel - 1041) * 200)/(1874 - 1041))-100

                'Kill the middle range of the stick
                'if(trans_vel < 2 and trans_vel > -2)
                '        trans_vel := 0

                ''''Grab the turning of stick
								rot_vel := remote_pulseWidths[CH_ROTATION]
                rot_vel := (200 - ((rot_vel - 951) * 200)/(1780 - 951))-100

                        'Kill the middle range of the stick
                'if(rot_vel < 2 and rot_vel > -2)
                 '	rot_vel := 0

                'If we are in fully manual mode:
                if(remoteMode == 1)
                        rot_vel := remote_pulseWidths[CH_ROTATION]
                        rot_vel := (200 - ((rot_vel - 951) * 200)/(1780 - 951))-100
												
                        'Kill the middle range of the stick
                        'if(rot_vel < 2 and rot_vel > -2)
                        '        rot_vel := 0

                'If we are in any kind of auto mode:
                elseif( remoteMode > 1)
                        'Check to see if our timeout has expired, or we are in a motor disabled state
                        if((cnt > timestamp + Timeout))
                                'If the timeout has expired, then kill the motors
                                targetSpeed1 := 64
                                targetSpeed2 := 64
                                tran_vel_req := 0
                                rot_vel_req  := 0
                                trans_vel    := 0
                                rot_vel      := 0
                                timestamp    := cnt
												
                        'If we're in semi-auto mode, then pass the requested speed through
                        if( remoteMode == 2 )
                                trans_vel := tran_vel_req
                                rot_vel := rot_vel_req

                        'If we're in fully auto mode, then pass the requested speeds through
                        elseif(remoteMode == 3)
                                rot_vel   := rot_vel_req
                                trans_vel := tran_vel_req
                                
                if(trans_vel < -100)
                        trans_vel := -100
                elseif(trans_vel > 100)
                        trans_vel := 100

                if(rot_vel < -100)
                        rot_vel := -100
                elseif(rot_vel > 100)
                        rot_vel := 100

								'Limit rotation speed by rot_vel_cap
'								rot_vel   := (rot_vel * rot_vel_cap) / 100
'								trans_vel := (trans_vel * rc_speed_cap) / 100

                targetSpeed1 :=  ((trans_vel * rc_speed_cap)/100 + (rot_vel*rot_vel_cap)/100)
                targetSpeed2 := -((trans_vel * rc_speed_cap)/100 - (rot_vel*rot_vel_cap)/100)

'								if(targetSpeed1 == 0 or targetSpeed2 == 0)

                targetSpeed1 := (targetSpeed1 + 100) * 128 / 200
                targetSpeed2 := (targetSpeed2 + 100) * 128 / 200

								rampMotor(targetSpeed1,targetSpeed2,emergencyMode)
								setMotor(motor1speed,motor2speed,emergencyMode)


PUB rampMotor(target_m1,target_m2,emergency)|temp_m1,temp_m2,current_m1,current_m2,d1,d2
'	current_m1 := (motor1speed * 200 /128) -100
'	current_m2 := (motor2speed * 200 /128) -100

	if (target_m1 < motor1speed)
		motor1speed := motor1speed - RAMP_INC
	if (target_m1 > motor1speed)
		motor1speed := motor1speed + RAMP_INC
	if (target_m2 < motor2speed)
		motor2speed := motor2speed - RAMP_INC
	if (target_m2 > motor2speed)
		motor2speed := motor2speed + RAMP_INC

	if(||(motor1speed - target_m1) < RAMP_INC)
		motor1speed := target_m1
	if(||(motor2speed - target_m2) < RAMP_INC)
		motor2speed := target_m2




PUB setMotor(m1,m2,emergency)|m1cmd,m2cmd
								m1cmd := m1
								m2cmd := m2            

                if(m1cmd > 127)
                        m1cmd := 127
                elseif(m1cmd < 0)
                        m1cmd := 0

                if(m2cmd > 127)
                        m2cmd := 127
                elseif(m2cmd < 0)
                        m2cmd := 0

                if(emergency == True)
                        m1cmd := 64
                        m2cmd := 64

                motor1speed := m1cmd
                motor2speed := m2cmd

                SabertoothSerial.tx(170) 

								'Drive _motor 1
                SabertoothSerial.tx(128)            											  'Address
                SabertoothSerial.tx(6)              											  'Command
                SabertoothSerial.tx(m1cmd)   											  'Data
                SabertoothSerial.tx((128 + 6 + m1cmd) & %01111111)   'Checksum

                'Drive _motor 2
                SabertoothSerial.tx(128)                                    'Address
                SabertoothSerial.tx(7)                                      'Command
                SabertoothSerial.tx(m2cmd)                           'Data
                SabertoothSerial.tx((128 + 7 + m2cmd) & %01111111)   'Checksum

