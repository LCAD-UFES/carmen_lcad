TARGET	= 	./bin/ojSteeringBypass
CCFLAGS	=	-Wall -fno-strict-aliasing -c -g -I ./include/ -I ../libjaus/include/ -I ../libopenJaus/include/ -I ../ojTorc/include
LFLAGS	=	-L ../libjaus/lib/ -L ../libopenJaus/lib/ -L ../ojTorc/lib/
LIBS	=	-lopenJaus -ljaus -lpthread -lcurses -lojTorc

SOURCES =	./src/getLine.c \
		./src/main.c \
		./src/can_utils.c

OBJECTS = ./Build/main.o \
		./Build/can_utils.o
		
default : all

all : $(TARGET)

clean :
	rm -f ./Build/*.o
	rm -f $(TARGET)

$(TARGET) : $(OBJECTS)
	g++ $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

./Build/main.o : ./src/main.c
	gcc $(CCFLAGS) -o ./Build/main.o ./src/main.c

./Build/can_utils.o : ./src/can_utils.c
	gcc $(CCFLAGS) -o ./Build/can_utils.o ./src/can_utils.c
