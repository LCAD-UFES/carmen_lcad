TARGET	= 	./bin/ojWheeltec
TARGETSO = 	./lib/libojWheeltec.so

CCFLAGS	=	-fPIC -Wall -fno-strict-aliasing -c -g -I ./include/ -I ../libjaus/include/ -I ../libopenJaus/include/ -I ../ojTorc/include
LFLAGS	=	-L ../libjaus/lib/ -L ../libopenJaus/lib/ -L ../ojTorc/lib/
LIBS	=	-lopenJaus -ljaus -lpthread -lcurses -lojTorc

SOURCES =	./src/main.c \
		./src/pd.c \
		./src/mpd.c \
		./src/sd.c \
		./src/can_utils.c \
		./src/vss.c

OBJECTS =	./Build/main.o \
		./Build/pd.o \
		./Build/mpd.o \
		./Build/sd.o \
		./Build/can_utils.o \
		./Build/vss.o

default : all

all :  $(TARGETSO) $(TARGET) download

clean :
	rm -f ./Build/*.o
	rm -f $(TARGET)
	rm -f $(TARGETSO)

download :
ifeq (,$(wildcard $(CARMEN_HOME)/bin/ford_escape/trailer-cargo.obj))
	$(info )
	$(info **************** **********************************************************************************************)
	$(info *	Dowloading 3D model trailer-cargo.obj)
	$(info * Dont worry if the trailer-cargo.mtl returns error, the gdown doesn't work properly wtth small files *)
	$(info * Check the file and if necessary, download it direct from the link in this makefie *)
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1nzTxAIa6XDspEGTdLzR143JpCNDJA2At/view?usp=sharing $(CARMEN_HOME)/bin/ford_escape/trailer-cargo.obj
#	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1HR0jUa7_goJIRxe8cUxXshXmSH6jwJvL/view?usp=sharing $(CARMEN_HOME)/bin/ford_escape/trailer-cargo.mtl
else
	$(info * 3D model trailer-cargo.obj already exists *)
endif

$(TARGET) : $(OBJECTS)
	g++ $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

$(TARGETSO) : $(OBJECTS)
	g++ -shared $(LFLAGS) -o $(TARGETSO) $(OBJECTS) $(LIBS)

./Build/main.o : ./src/main.c
	gcc $(CCFLAGS) -o ./Build/main.o ./src/main.c

./Build/pd.o : ./src/pd.c
	gcc $(CCFLAGS) -o ./Build/pd.o ./src/pd.c

./Build/mpd.o : ./src/mpd.c
	gcc $(CCFLAGS) -o ./Build/mpd.o ./src/mpd.c

./Build/sd.o : ./src/sd.c
	gcc $(CCFLAGS) -o ./Build/sd.o ./src/sd.c

./Build/can_utils.o : ./src/can_utils.c
	gcc $(CCFLAGS) -o ./Build/can_utils.o ./src/can_utils.c

./Build/vss.o : ./src/vss.c
	gcc $(CCFLAGS) -o ./Build/vss.o ./src/vss.c

