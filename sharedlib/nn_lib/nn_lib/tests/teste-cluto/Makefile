all:
	g++ -O3 -o main main.cpp -L . -lcluto `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv`
	g++ -g -o main-live main-live.cpp -L . -lcluto `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv`	
clean:
	rm -rf main main-live
	
