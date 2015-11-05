CC = g++

CFLAGS = -W -Wall `pkg-config --cflags opencv` -O4
LFLAGS = `pkg-config --libs opencv`

build: haar training testing result crop_roi fp fn image_gray split_channels
	@echo Done

haar: haar.cpp
	$(CC) -o haar haar.cpp $(CFLAGS) $(LFLAGS)

training: training.cpp
	$(CC) -o training training.cpp $(CFLAGS) $(LFLAGS)

testing: testing.cpp
	$(CC) -o testing testing.cpp $(CFLAGS) $(LFLAGS)

result: result.cpp
	$(CC) -o result result.cpp $(CFLAGS) $(LFLAGS)

crop_roi: crop_image.cpp
	$(CC) -o crop_roi crop_image.cpp $(CFLAGS) $(LFLAGS)

fp: fp.cpp
	$(CC) -o fp fp.cpp $(CFLAGS) $(LFLAGS)

fn: fn.cpp
	$(CC) -o fn fn.cpp $(CFLAGS) $(LFLAGS)

image_gray: image_gray.cpp
	$(CC) -o image_gray image_gray.cpp $(CFLAGS) $(LFLAGS)

split_channels: split_channels.cpp
	$(CC) -o split_channels split_channels.cpp $(CFLAGS) $(LFLAGS)

clean:
	rm -rf *.o haar result training testing crop_roi fp fn image_gray 
