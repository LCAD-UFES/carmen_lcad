include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIBCARMEN_UTIL
MODULE_COMMENT = Library of utilities functions

CXXFLAGS += -Wno-sign-compare -std=c++11
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

LFLAGS += -lboost_program_options 
SOURCES = command_line.cpp  carmen_param_file.cpp util_io.cpp util_math.cpp util_strings.cpp util_time.cpp #util_strings_test.cpp

PUBLIC_INCLUDES = command_line.h carmen_param_file.h util_io.h util_math.h util_strings.h util_time.h
PUBLIC_LIBRARIES = libcarmen_util.a
TARGETS = libcarmen_util.a #util_strings_test

libcarmen_util.a: util_io.o util_math.o util_strings.o util_time.o command_line.o carmen_param_file.o 
#util_strings_test: util_strings_test.o util_strings.o

include $(CARMEN_HOME)/src/Makefile.rules
