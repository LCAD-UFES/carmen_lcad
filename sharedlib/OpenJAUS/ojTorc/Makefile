TARGET	= 	./bin/ojTorc
TARGETSO = 	./lib/libojTorc.so

CCFLAGS	=	-Wall -std=c99 -fPIC -fno-strict-aliasing -c -g -I ./include/ -I ../libjaus/include/ -I ../libopenJaus/include/
LFLAGS	=	-L ../libjaus/lib/ -L ../libopenJaus/lib/
LIBS	=	-lopenJaus -ljaus -lpthread -lcurses -lm
SOURCES =	./src/torcInterface.c \
		./src/torc.c \
		./src/torcComm.c \
		./src/setMotionProfileMessage.c \
		./src/reportCurvatureMessage.c \
		./src/reportWheelsSpeedMessage.c \
		./src/reportSignalsMessage.c \
		./src/setSignalsMessage.c \
		./src/reportErrorCountMessage.c \
		./src/reportComponentStatusMessage.c

OBJECTS =	./Build/torcInterface.o \
		./Build/torc.o \
		./Build/torcComm.o \
		./Build/setMotionProfileMessage.o \
		./Build/reportCurvatureMessage.o \
		./Build/reportWheelsSpeedMessage.o \
		./Build/reportSignalsMessage.o \
		./Build/setSignalsMessage.o \
		./Build/reportErrorCountMessage.o \
		./Build/reportComponentStatusMessage.o

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

./Build/torcInterface.o : ./src/torcInterface.c ./include/torcInterface.h
	gcc $(CCFLAGS) -o ./Build/torcInterface.o ./src/torcInterface.c

./Build/torc.o : ./src/torc.c
	gcc $(CCFLAGS) -o ./Build/torc.o ./src/torc.c

./Build/torcComm.o : ./src/torcComm.c
	gcc $(CCFLAGS) -o ./Build/torcComm.o ./src/torcComm.c

./Build/setMotionProfileMessage.o : ./src/setMotionProfileMessage.c ./include/setMotionProfileMessage.h
	gcc $(CCFLAGS) -o ./Build/setMotionProfileMessage.o ./src/setMotionProfileMessage.c

./Build/reportCurvatureMessage.o : ./src/reportCurvatureMessage.c ./include/reportCurvatureMessage.h
	gcc $(CCFLAGS) -o ./Build/reportCurvatureMessage.o ./src/reportCurvatureMessage.c

./Build/reportWheelsSpeedMessage.o : ./src/reportWheelsSpeedMessage.c ./include/reportWheelsSpeedMessage.h
	gcc $(CCFLAGS) -o ./Build/reportWheelsSpeedMessage.o ./src/reportWheelsSpeedMessage.c

./Build/reportSignalsMessage.o : ./src/reportSignalsMessage.c ./include/reportSignalsMessage.h
	gcc $(CCFLAGS) -o ./Build/reportSignalsMessage.o ./src/reportSignalsMessage.c
	
./Build/reportComponentStatusMessage.o : ./src/reportComponentStatusMessage.c ./include/reportComponentStatusMessage.h
	gcc $(CCFLAGS) -o ./Build/reportComponentStatusMessage.o ./src/reportComponentStatusMessage.c

./Build/setSignalsMessage.o : ./src/setSignalsMessage.c ./include/setSignalsMessage.h
	gcc $(CCFLAGS) -o ./Build/setSignalsMessage.o ./src/setSignalsMessage.c

./Build/reportErrorCountMessage.o : ./src/reportErrorCountMessage.c ./include/reportErrorCountMessage.h
	gcc $(CCFLAGS) -o ./Build/reportErrorCountMessage.o ./src/reportErrorCountMessage.c
