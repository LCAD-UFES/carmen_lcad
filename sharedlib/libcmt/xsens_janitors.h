#ifndef XSENS_MONOLITHIC
/*! \file
	\brief	Contains the Janitor class-interfaces and implementations

	This file contains a number of janitor classes. These classes can be used to perform
	simple actions upon leaving scope, such as deleting an object.
	This greatly simplifies exit code. Functions that have lots of exit points can benefit
	greatly from janitors.
	
	Each janitor is named after its main functionality, eg Restore, Free, Delete...

	\section FileCopyright Copyright Notice 
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.
*/
#endif

#ifndef XSENS_JANITORS_H
#define XSENS_JANITORS_H

// required for older gnu c++ compiler versions due to difference in attribute declarations
#if defined(__GNUC__) && !defined(HAVE_CDECL)
#   define __cdecl __attribute__(()) // Atributo retirado para evitar warnings!
#   define __stdcall __attribute__((stdcall))
#endif

namespace xsens {


//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Class function calling janitor class

	This class can be used to make sure that the given class function is called when the
	janitor leaves scope.
*/
template <class T, typename R = void>
class JanitorClassFunc {
public:
	typedef R (T::*t_func_JanitorClassFunc)(void);
private:
	const JanitorClassFunc& operator = (const JanitorClassFunc&);

	T& m_control;
	t_func_JanitorClassFunc m_funcJCF;
	bool m_enabled;
public:
	
	JanitorClassFunc<T,R>(T& control, t_func_JanitorClassFunc func, bool enabl = true) :
		m_control(control), m_funcJCF(func), m_enabled(enabl)
	{
	}
	~JanitorClassFunc<T,R>()
	{
		if (m_enabled)
			(m_control.*m_funcJCF)();
	}
	
	void disable(void)
		{ m_enabled = false; }
	
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Class function calling janitor class with a parameter

	This class can be used to make sure that the given class function is called when the
	janitor leaves scope.
*/
template <class T, typename P1, typename R = void>
class JanitorClassFuncP1 {
public:
	typedef R (T::*t_func_JanitorClassFunc)(P1);
private:
	const JanitorClassFuncP1& operator = (const JanitorClassFuncP1&);

	T& m_control;
	P1 m_param1;
	t_func_JanitorClassFunc m_funcJCF;
	bool m_enabled;
public:
	
	JanitorClassFuncP1<T,P1,R>(T& control, P1 p1, t_func_JanitorClassFunc func, bool enabl = true) :
		m_control(control), m_param1(p1), m_funcJCF(func), m_enabled(enabl)
	{
	}
	~JanitorClassFuncP1<T,P1,R>()
	{
		if (m_enabled)
			(m_control.*m_funcJCF)(m_param1);
	}
	
	void disable(void)
		{ m_enabled = false; }
	
	void enable(void)
		{ m_enabled = true; }
};


}	// end of xsens namespace

#endif	// XSENS_JANITORS_H
