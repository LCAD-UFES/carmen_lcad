IDIR=
ODIR=obj

CC=g++
CFLAGS=-Wall
DEPENDENCIES=
LIBS=-lcurl

DIRS= $(IDIR) $(ODIR)

TARGET=directions

_OBJ = $(TARGET).o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

.PHONY: all clean

all: $(DIRS) $(TARGET)
	echo $(DIRS)
	echo $(TARGET)

clean:
	rm -Rf $(DIRS) $(TARGET)

$(ODIR)/%.o: %.cpp $(DEPENDENCIES)
	$(CC) -c -o $@ $< $(CFLAGS)

$(DIRS):
	mkdir -p $(DIRS)

$(TARGET): $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)
