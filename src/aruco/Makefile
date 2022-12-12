include ../Makefile.conf

MODULE_NAME = ARUCO
MODULE_COMMENT = Aruco pose estimation module

all: cmake cp-bin

cmake:
	[ -f build/CMakeCache.txt ] && [ "$$(pwd)" != "$$(grep 'CMAKE_HOME_DIRECTORY:INTERNAL' build/CMakeCache.txt | cut -d '=' -f 2)" ] \
&& rm -rf build || true
	mkdir -p build && cd build && cmake .. -Wno-dev
	make -C build
	rm -rf build

cp-bin:
	cp aruco $(CARMEN_HOME)/bin/

# clean:
# 	rm -rf build

include ../Makefile.rules
