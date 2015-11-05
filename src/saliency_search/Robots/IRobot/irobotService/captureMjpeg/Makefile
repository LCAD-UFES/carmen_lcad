##############################
# spcaview Makefile
##############################

INSTALLROOT=$(PWD)

CC=/usr/local/bin/gumgcc
CPP=/usr/local/bin/gumg++
INSTALL=install
APP_BINARY=luvcview
BIN=/usr/local/bin
#LIBX11FLAGS= -I/usr/X11R6/include -L/usr/X11R6/lib
VERSION = 0.2.4

#WARNINGS = -Wall \
#           -Wundef -Wpointer-arith -Wbad-function-cast \
#           -Wcast-align -Wwrite-strings -Wstrict-prototypes \
#           -Wmissing-prototypes -Wmissing-declarations \
#           -Wnested-externs -Winline -Wcast-qual -W \
#           -Wno-unused
#           -Wunused

CFLAGS += -O2 -DLINUX -DVERSION=\"$(VERSION)\"  $(WARNINGS)
CPPFLAGS = $(CFLAGS)

OBJECTS= luvcview.o color.o utils.o v4l2uvc.o gui.o avilib.o
		

all:	luvcview

clean:
	@echo "Cleaning up directory."
	rm -f *.a *.o $(APP_BINARY) core *~ log errlog *.avi

# Applications:
luvcview:	$(OBJECTS)
	$(CC)	$(CFLAGS) $(OBJECTS) $(X11_LIB) $(XPM_LIB)\
		$(MATH_LIB) \
		-o $(APP_BINARY)
	chmod 755 $(APP_BINARY)


install: luvcview
	$(INSTALL) -s -m 755 -g root -o root $(APP_BINARY) $(BIN) 
	rm -f $(BIN)/uvcview
