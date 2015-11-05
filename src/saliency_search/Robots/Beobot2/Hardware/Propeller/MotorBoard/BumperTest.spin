CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
	
	ENCODER_PINS  = 20 'Left(20 21) Right(22 23)
	BUMPER_PINS = 18 'we connect bumper to both I/O
	BUMPER_GND  = 19 'because of screw block,so set one to GND
DAT
VAR
OBJ
  serial 		 : "ComputerLink"

PUB Main | cmd, motor, speed, timeout, chan

	timeout := CLKFREQ/2

	serial.start(31,30,0,115200)


	
	dira[BUMPER_PINS]:= 0 'set pin as input
	dira[BUMPER_GND] := 1 'set pin as output
	outa[BUMPER_GND] := 0	'set to low
  repeat
		'check bumper first,FIXXX
		if ina[BUMPER_PINS] == 0
			cmd := 0
			serial.frameStr(cmd,string("Bumper Hit"))
			serial.str(string(13))
		else
			serial.str(string("No Bumper"))
			serial.str(string(13))

    'waitcnt (clkfreq + cnt) 'delay 1s
		waitcnt(clkfreq/2+cnt)
'		cmd := serial.rx
			
'		if(cmd == 0)
'			serial.frameStr(cmd, string("motorboard"))

