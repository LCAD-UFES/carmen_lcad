CON
	_clkmode = xtal1 + pll4x
	_xinfreq = 5_000_000
	PING_Pin = 10
OBJ

	serial:"FullDuplexSerial"
	ping:"ping"

PUB Main | dist,cmd

    	serial.start(31,30,0,115200)

	repeat
		dist := ping.Millimeters(PING_Pin)
		serial.tx(255)		
		sendLong(dist)
		serial.tx(255)
		'serial.dec(dist)	
		'serial.str(string(13,10))		

PUB sendLong(txLong)
	serial.tx((txLong >> 24))
	serial.tx((txLong >> 16))
	serial.tx((txLong >> 08))
	serial.tx((txLong >> 00))
	
