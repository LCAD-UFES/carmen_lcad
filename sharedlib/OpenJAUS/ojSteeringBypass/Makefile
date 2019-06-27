TARGET	= ./bin/ojSteeringBypass
CCFLAGS	= -Wall -fno-strict-aliasing -c -g -I ./include/ -I ../libjaus/include/ -I ../libopenJaus/include/ -I ../ojIARASim/include
LFLAGS	= -L ../libjaus/lib/ -L ../libopenJaus/lib/ -L ../ojIARASim/lib/ -L ../ojTorc/lib/
LIBS	= -lpthread -lopenJaus -ljaus -lojTorc -lojIARASim

SOURCES = ./src/main.c

OBJECTS = ./Build/main.o
		
default : all

all : $(TARGET)

clean :
	rm -f ./Build/*.o
	rm -f $(TARGET)

$(TARGET) : $(OBJECTS)
	g++ $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

./Build/main.o : ./src/main.c
	gcc $(CCFLAGS) -o ./Build/main.o ./src/main.c

