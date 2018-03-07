
%module carmen_comm
%{
#include "carmen_comm.h"
%}

%include "typemaps.i"
%include "std_vector.i" 

namespace std
{
    %template(FloatVector) vector<float>;
}

%include "carmen_comm.h"
