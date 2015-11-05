
OBJ
	serial : "FullDuplexSerial"

PUB Start(rxpin, txpin, mode, baudrate)
	serial.start(rxpin, txpin, mode, baudrate)

PUB Stop
	serial.stop

PUB rxcheck : rxbyte
	return serial.rxcheck

PUB rx  : rxbyte
	return serial.rx

PUB rxtime(ms)  : rxbyte | t
	return serial.rxtime(ms)

PUB tx(txbyte)
	serial.tx(txbyte)

PUB str(stringptr)
	serial.str(stringptr)
	
PUB dec(value)
	serial.dec(value)

PUB frameStr(cmd, stringptr)
	serial.tx(cmd)
	serial.tx(strsize(stringptr))
	serial.str(stringptr)
	serial.tx(255)

PUB frameLong(cmd, longVal)
	serial.tx(cmd)
	serial.tx(4)
	sendLong(longVal)
	serial.tx(255)

PUB frameByte(cmd, byteVal)
	serial.tx(cmd)
	serial.tx(1)
	serial.tx(byteVal)
	serial.tx(255)

PUB sendLong(value)
	serial.tx((value >> 24))
	serial.tx((value >> 16))
	serial.tx((value >> 08))
	serial.tx((value >> 00))

PUB sendShort(value)
	serial.tx((value >> 08))
	serial.tx((value >> 00))

	
