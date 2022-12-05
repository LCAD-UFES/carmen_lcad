CC=g++
CFLAGS = -lm -w `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv libswresample libavformat libavcodec libswscale libavutil`
DEPS = 
OBJ = check_intelbras.o

all: check clean

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

check: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

clean:
	rm -f *.o