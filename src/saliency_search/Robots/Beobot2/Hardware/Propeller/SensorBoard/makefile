PORT=/dev/ttyUSB0
BSTC=bstc.linux
OBJDIR=-L/usr/local/share/propeller/lib -L../Lib
PROG=SensorBoard

all:
	sudo $(BSTC) $(OBJDIR) $(PROG).spin

upload:
	echo "Uploading to RAM on Port $(PORT)"
	sudo $(BSTC) -d $(PORT) -p0 $(OBJDIR) $(PROG).spin

uploadE:
	echo "Uploading to EEPROM on Port $(PORT)"
	sudo $(BSTC) -d $(PORT) -p2 $(OBJDIR) $(PROG).spin

display:
	sudo $(BSTC) $(OBJDIR) DisplayTest.spin
	sudo $(BSTC) -d $(PORT) -p2 $(OBJDIR) DisplayTest.spin
