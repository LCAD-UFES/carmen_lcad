PORT=/dev/ttyUSB0
BSTC=bstc.linux
OBJDIR=/usr/local/share/propeller/lib
PROG=CoolingBoard

all:
	sudo $(BSTC) -L$(OBJDIR) $(PROG).spin

upload:
	echo "Uploading to RAM on Port $(PORT)"
	sudo $(BSTC) -d $(PORT) -p0 -L$(OBJDIR) $(PROG).spin

uploadE:
	echo "Uploading to EEPROM on Port $(PORT)"
	sudo $(BSTC) -d $(PORT) -p2 -L$(OBJDIR) $(PROG).spin
