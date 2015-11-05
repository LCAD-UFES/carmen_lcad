
PORT=/dev/ttyUSB0
BSTC=bstc.linux
OBJDIR=/usr/local/share/propeller/lib
PROG=Scorbot-ER4_4PC_Controller 

all:
	$(BSTC) -L$(OBJDIR) $(PROG).spin

upload:
	echo "Stop Controller"
	/home/ihead/saliency/bin/test-Scorbot --out=none 15
	#/home/ihead/saliency/bin/test-Scorbot --out=none 15
	#/home/ihead/saliency/bin/test-Scorbot --out=none 15
	sleep 1
	echo "Uploading to RAM on Port $(PORT)"
	$(BSTC) -d $(PORT) -p0 -L$(OBJDIR) $(PROG).spin

uploadE:
	echo "Stop Controller"
	/home/ihead/saliency/bin/test-Scorbot --out=none 15
	/home/ihead/saliency/bin/test-Scorbot --out=none 15
	/home/ihead/saliency/bin/test-Scorbot --out=none 15
	sleep 1
	echo "Uploading to EEPROM on Port $(PORT)"
	$(BSTC) -d $(PORT) -p2 -L$(OBJDIR) $(PROG).spin
