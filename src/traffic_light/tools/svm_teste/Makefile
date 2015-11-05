CC = g++

CFLAGS = -W -Wall `pkg-config --cflags opencv` -O4
LFLAGS = `pkg-config --libs opencv`

build: svm_train svm_validation
	@echo Done

svm_train: svm_test.cpp
	$(CC) -o svm_train svm_test.cpp $(CFLAGS) $(LFLAGS)

svm_validation: svm_val.cpp
	$(CC) -o svm_validation svm_val.cpp $(CFLAGS) $(LFLAGS)

clean:
	rm -rf *.o svm_train svm_validation
