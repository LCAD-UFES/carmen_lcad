CC=g++
CFLAGS=-I. -Wall -g
LFLAGS= -lrt -lncurses -lpthread
DEPS = cmt1.h 
OBJSTATIC = cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o

all: static

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<

example: $(OBJSTATIC) example_linux.cpp
	$(CC) $(CFLAGS) $(OBJSTATIC) example_linux.cpp -o example $(LFLAGS)

example1s: $(OBJSTATIC) example1s_mt_linux.cpp
	$(CC) $(CFLAGS) $(OBJSTATIC) example1s_mt_linux.cpp -o example1s $(LFLAGS)

example2s: $(OBJSTATIC) example2s_mt_linux.cpp
	$(CC) $(CFLAGS) $(OBJSTATIC) example2s_mt_linux.cpp -o example2s $(LFLAGS)

example3s: $(OBJSTATIC) example3s_mt_linux.cpp
	$(CC) $(CFLAGS) $(OBJSTATIC) example3s_mt_linux.cpp -o example3s $(LFLAGS)


static: $(OBJSTATIC)
	ar rcs libcmt.a $^

clean:
	rm -f example
	rm -f example1s
	rm -f example2s
	rm -f example3s
	rm -f *.o
	rm -f libcmt.a
	rm -f libcmt.so
