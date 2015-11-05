CC = g++

CFLAGS = -W -Wall `pkg-config --cflags opencv` -O4
LFLAGS = `pkg-config --libs opencv`

PROG = create_division
SOURCE = division.cpp

build: $(PROG)
	@echo Done
	
$(PROG): $(SOURCE)
	$(CC) -o $(PROG) $(SOURCE) $(CFLAGS) $(LFLAGS)

clean:
	rm -rf *.o $(PROG)
