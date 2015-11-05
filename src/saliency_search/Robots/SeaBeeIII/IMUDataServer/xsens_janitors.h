#ifndef _CMT_MONOLITHIC
/*! \file
        \brief        Contains the Janitor class-interfaces and implementations

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

#ifndef _JANITORS_H_2006_05_01
#define _JANITORS_H_2006_05_01

// required for older gnu c++ compiler versions due to difference in attribute declarations
#if defined(__GNUC__) && !defined(HAVE_CDECL)
#   define __cdecl __attribute__((cdecl))
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
        typedef R (T::*t_func_JanitorClasssFunc)(void);
private:
        T& m_control;
        t_func_JanitorClasssFunc m_funcJCF;
        bool m_enabled;
public:
        
        JanitorClassFunc<T,R>(T& control, t_func_JanitorClasssFunc func, bool enabl = true) :
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


}        // end of xsens namespace

#endif        // _JANITORS_H_2006_05_01
