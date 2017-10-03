TARGET	= 	./bin/ojIARASim
CCFLAGS	=	-Wall -fno-strict-aliasing -c -g -I ./include/ -I ../libjaus/include/ -I ../libopenJaus/include/ -I ../ojTorc/include
LFLAGS	=	-L ../libjaus/lib/ -L ../libopenJaus/lib/ -L ../ojTorc/lib/
LIBS	=	-lopenJaus -ljaus -lpthread -lcurses -lojTorc

SOURCES =	./src/getLine.c \
		./src/main.c \
		./src/properties.c \
		./src/pd.c \
		./src/vss.c

OBJECTS =	./Build/getLine.o \
		./Build/main.o \
		./Build/properties.o \
		./Build/pd.o \
		./Build/vss.o

default : all

all : $(TARGET)

clean :
	rm -f ./Build/*.o
	rm -f $(TARGET)

$(TARGET) : $(OBJECTS)
	g++ $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

./Build/getLine.o : ./src/getLine.c
	gcc $(CCFLAGS) -o ./Build/getLine.o ./src/getLine.c

./Build/main.o : ./src/main.c
	gcc $(CCFLAGS) -o ./Build/main.o ./src/main.c

./Build/properties.o : ./src/properties.c
	gcc $(CCFLAGS) -o ./Build/properties.o ./src/properties.c

./Build/pd.o : ./src/pd.c
	gcc $(CCFLAGS) -o ./Build/pd.o ./src/pd.c

./Build/vss.o : ./src/vss.c
	gcc $(CCFLAGS) -o ./Build/vss.o ./src/vss.c

