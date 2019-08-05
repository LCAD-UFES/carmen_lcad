all:conversor

conversor: main.o tinyxml.o tinystr.o tinyxmlparser.o tinyxmlerror.o
	g++  -o conversor main.o tinyxml.o tinystr.o tinyxmlparser.o tinyxmlerror.o  -lglut -lGL -lGLU -lm

tinyxml.o: tinyxml.cpp
	g++ -c tinyxml.cpp

main.o: main.cpp
	g++  -c main.cpp

tinystr.o: tinystr.cpp
	g++ -c tinystr.cpp

tinyxmlparser.o: tinyxmlparser.cpp
	g++ -c tinyxmlparser.cpp 	

tinyxmlerror.o: tinyxmlerror.cpp
	g++ -c tinyxmlerror.cpp

clean:	
	rm -f tinyxmlerror.o tinyxmlparser.o tinystr.o main.o tinyxml.o


