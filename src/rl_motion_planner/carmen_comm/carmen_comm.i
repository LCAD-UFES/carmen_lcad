
%module carmen_comm

%include "/usr/share/swig3.0/python/typemaps.i"
%include "/usr/share/swig3.0/python/std_vector.i" 

namespace std
{
    %template(FloatVector) vector<double>;
}

%{
// The following lines are responsible for automatically translating std::vector<float> to python tuples.
#include "carmen_comm.h"
%}


// **********************************************************
// List of function and classes we want to expose.
// By including the file the whole content is exposed.
// **********************************************************
%include "carmen_comm.h"


