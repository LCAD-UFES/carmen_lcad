
########### next target ###############

SET(sickldmrs_LIB_SRCS
   commands.c
   print.c
   sickldmrs.c
   sickldmrs-private.h
)

add_library(sickldmrs SHARED ${sickldmrs_LIB_SRCS})

target_link_libraries(sickldmrs rt)

set_target_properties(sickldmrs PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS sickldmrs DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#lib_LTLIBRARIES = libsickldmrs.la
#
#libsickldmrs_la_SOURCES = \
#	commands.c \
#	print.c \
#	sickldmrs.c \
#	sickldmrs-private.h
#
#LIBS = $(SICKLDMRS_LIBS)
#
#AM_CPPFLAGS = -Wall -I$(top_srcdir)
#AM_CFLAGS = -g -Wall $(SICKLDMRS_CFLAGS)
