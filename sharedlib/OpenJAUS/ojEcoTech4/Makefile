TARGET	= 	./bin/ojEcoTech4
TARGETSO = 	./lib/ojEcoTech4.so

CCFLAGS	=	-fPIC -Wall -fno-strict-aliasing -c -g -I ./include/ -I ../libjaus/include/ -I ../libopenJaus/include/ -I ../ojTorc/include
LFLAGS	=	-L ../libjaus/lib/ -L ../libopenJaus/lib/ -L ../ojTorc/lib/
LIBS	=	-lopenJaus -ljaus -lpthread -lcurses -lojTorc -lpigpio

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

all :  $(TARGETSO) $(TARGET)

clean :
	rm -f ./Build/*.o
	rm -f $(TARGET)
	rm -f $(TARGETSO)

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
