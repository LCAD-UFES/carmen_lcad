
# Migração Ubuntu 26.04: faltavam os -I do OpenCV (headers agora em /usr/include/opencv4).
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`
#Simple clean Makefile

clean:
	rm -rf *.o
	rm -rf *.o~
	rm -rf *.h~
	rm -rf *.c~
