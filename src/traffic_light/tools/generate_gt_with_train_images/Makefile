CC = g++

CFLAGS = -W -Wall `pkg-config --cflags opencv` -O4
LFLAGS = `pkg-config --libs opencv`

PROG = generate_gt_with_train_images
SOURCE = generate_gt_with_train_images.cpp

PROG2 = generate_negatives
SOURCE2 = generate_negatives_from_dataset.cpp

build: $(PROG) $(PROG2)
	@echo Done
	
$(PROG): $(SOURCE)
	$(CC) -o $(PROG) $(SOURCE) $(CFLAGS) $(LFLAGS)

$(PROG2): $(SOURCE2)
	$(CC) -o $(PROG2) $(SOURCE2) $(CFLAGS) $(LFLAGS)

clean:
	rm -rf *.o $(PROG)
