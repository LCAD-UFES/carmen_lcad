PORT=/dev/ttyUSB0
BSTC=bstc.linux
OBJDIR=-L/usr/local/share/propeller/lib -L../Lib
PROG=TestSerial
PROGBT=MotorBoard_BTserial
PROGBUMPER=BumperTest

all:
	sudo $(BSTC) $(OBJDIR) $(PROG).spin

upload:
	echo "Uploading to RAM on Port $(PORT)"
	sudo $(BSTC) -d $(PORT) -p0 $(OBJDIR) $(PROG).spin

uploadE:
	echo "Uploading to EEPROM on Port $(PORT)"
	sudo $(BSTC) -d $(PORT) -p2 $(OBJDIR) $(PROG).spin

bt:
	echo "Uploading to EEPROM on Port $(PORT)"
	sudo $(BSTC) -d $(PORT) -p2 $(OBJDIR) $(PROGBT).spin
bump:
	echo "Upload bumper test to EEPROM on Port $(PORT)"
	sudo $(BSTC) -d $(PORT) -p2 $(OBJDIR) $(PROGBUMPER).spin
display:
	sudo $(BSTC) $(OBJDIR) DisplayTest.spin
	sudo $(BSTC) -d $(PORT) -p0 $(OBJDIR) DisplayTest.spin

displayE:
	sudo $(BSTC) $(OBJDIR) DisplayTest.spin
	sudo $(BSTC) -d $(PORT) -p2 $(OBJDIR) DisplayTest.spin
