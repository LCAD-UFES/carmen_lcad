#ifndef XSENS_EXCEPTION_H
#define XSENS_EXCEPTION_H

#include <exception>
#include <string>

namespace xsens {

class Exception : public std::exception
{
public:
	Exception();
	explicit Exception(const std::string &what);
	Exception(const Exception &ex);

	virtual ~Exception() throw();

	virtual const char *what() const throw();

private:
	std::string m_what;
};

} // namespace xsens

#endif // XSENS_EXCEPTION_H
