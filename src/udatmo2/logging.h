#ifndef CARMEN_LOGGING_H
#define CARMEN_LOGGING_H

#ifdef CARMEN_LOGGING_ON
	#include <boost/log/trivial.hpp>
	#include <boost/log/utility/setup/file.hpp>

	#define CARMEN_LOG_TO_FILE(PATH) boost::log::add_file_log(PATH)

	#define CARMEN_LOG(LEVEL, MESSAGE) BOOST_LOG_TRIVIAL(LEVEL) << MESSAGE
#else
	#define CARMEN_LOG_TO_FILE(PATH)

	#define CARMEN_LOG(LEVEL, MESSAGE)
#endif

#endif
