#ifndef CARMEN_CPP_DEBUG_LOG_H
#define CARMEN_CPP_DEBUG_LOG_H

/**
 * When adding debug logging to a module, you'll want to add the following
 * block to its `Makefile`:
 *
 * 	# Add logging dependencies as required
 * 	ifeq ("$(CARMEN_CPP_DEBUG_LOG)","ON")
 * 	CXXFLAGS += -DBOOST_LOG_DYN_LINK -DCARMEN_CPP_DEBUG_LOG_ON
 * 	LFLAGS += \
 * 		-lboost_log_setup \
 * 		-lboost_log \
 * 		-lboost_thread \
 * 		-lboost_system
 * 	endif
 *
 * So that you can conditionally enable debug logging by compiling the module
 * with:
 *
 * 	make CARMEN_CPP_DEBUG_LOG="ON"
*/

#ifdef CARMEN_CPP_DEBUG_LOG_ON
	#include <boost/log/trivial.hpp>
	#include <boost/log/utility/setup/file.hpp>

	#define CARMEN_LOG_TO_FILE(PATH) boost::log::add_file_log(PATH)

	#define CARMEN_LOG(LEVEL, MESSAGE) BOOST_LOG_TRIVIAL(LEVEL) << __FILE__ << ": " << MESSAGE
#else
	#define CARMEN_LOG_TO_FILE(PATH)

	#define CARMEN_LOG(LEVEL, MESSAGE)
#endif

#endif
