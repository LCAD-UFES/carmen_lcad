
%module pycarmen_panel

%{
// The following lines are responsible for automatically translating std::vector<float> to python tuples.
#include "panel.h"
%}

// **********************************************************
// List of function and classes we want to expose.
// By including the file the whole content is exposed.
// **********************************************************
%include "panel.h"


