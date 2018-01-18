#ifndef VIRTUAL_SCAN_LOGGING_H
#define VIRTUAL_SCAN_LOGGING_H

#include <iostream>

#ifdef DEBUG
	#define LOG(message) std::cout << message << std::endl
#else
	#define LOG(message)
#endif

#endif
