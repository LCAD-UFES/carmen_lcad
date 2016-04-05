
########### next target ###############

SET(sickldmrstest_SRCS
   sickldmrstest.c
)

add_executable(sickldmrstest ${sickldmrstest_SRCS})

target_link_libraries(sickldmrstest sickldmrs pthread m)

install(TARGETS sickldmrstest DESTINATION bin)


########### install files ###############




#original Makefile.am contents follow:

#bin_PROGRAMS = sickldmrstest 
#
#AM_CPPFLAGS = -I$(top_srcdir)
#AM_CFLAGS = -g -Wall $(SICKLDMRS_CFLAGS)
#
#sickldmrstest_SOURCES = sickldmrstest.c
#sickldmrstest_LDADD =  $(top_builddir)/src/libsickldmrs.la \
#        $(SICKLDMRS_LIBS) -lpthread -lm
