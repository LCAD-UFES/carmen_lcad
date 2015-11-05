#include "xsens_exception.h"

namespace xsens {

Exception::Exception() :
		std::exception()
{
}

Exception::Exception(const std::string &what) :
		std::exception(),
		m_what(what)
{

}

Exception::Exception(const Exception &ex) :
		std::exception(ex),
		m_what(ex.m_what)
{

}

Exception::~Exception() throw()
{
}

const char *Exception::what() const throw()
{
	return m_what.c_str();
}

} // namespace xsens
