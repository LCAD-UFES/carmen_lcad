/*
 * tf_time_exception.h
 *
 *  Created on: Apr 18, 2012
 *      Author: lauro
 */

#ifndef TF_TIME_EXCEPTION_H_
#define TF_TIME_EXCEPTION_H_


#include <stdexcept>

namespace tf
{

class Exception : public std::runtime_error
{
public:
	Exception(const std::string& what) : std::runtime_error(what){}};
}

#endif /* TF_TIME_EXCEPTION_H_ */
