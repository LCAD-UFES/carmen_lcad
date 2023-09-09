include ../Makefile.conf

MODULE_NAME = ARUCO Position Tracker
MODULE_COMMENT = Aruco pose estimation module

PUBLIC_BINARIES_COPY = aruco_theta_tracker

TARGETS = cmake

cmake:
	[ -f build/CMakeCache.txt ] && [ "$$(pwd)" != "$$(grep 'CMAKE_HOME_DIRECTORY:INTERNAL' build/CMakeCache.txt | cut -d '=' -f 2)" ] \
&& rm -rf build || true
	mkdir -p build && cd build && cmake .. -Wno-dev
	make -C build
	rm -rf build

clear:
	rm -rf build
	rm -f aruco_theta_tracker
	rm -f create_board
	rm -f Makefile.depend


include ../Makefile.rules
