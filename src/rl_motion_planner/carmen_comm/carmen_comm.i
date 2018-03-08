
%module carmen_comm
%{
// ***************************************************************
// File(s) containing classes and functions we want to expose.
// ***************************************************************
#include "carmen_comm.h"
%}

// The following lines are responsible for automatically translating std::vector<float> to python tuples.
%include "typemaps.i"
%include "std_vector.i" 

namespace std
{
    %template(FloatVector) vector<double>;
}

// **********************************************************
// List of function and classes we want to expose.
// By including the file the whole content is exposed.
// **********************************************************
%include "carmen_comm.h"
