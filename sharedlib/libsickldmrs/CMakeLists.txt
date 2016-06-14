cmake_minimum_required(VERSION 3.0)

include_directories(include)

add_subdirectory(src)
add_subdirectory(test)


########### install files ###############

install(FILES  sickldmrs-libs.pc DESTINATION bin)
install(FILES  DESTINATION include)
install(FILES  $(top_srcdir)/include/sickldmrs.h DESTINATION bin)



#original Makefile.am contents follow:

#
#AUTOMAKE_OPTIONS = foreign
#SUBDIRS = src test
#
#pkgconfigdir = $(libdir)/pkgconfig
#pkgconfig_DATA = sickldmrs-libs.pc
#
#ACLOCAL_AMFLAGS = -I m4
#
#libsickldmrs_includedir = $(includedir)/sickldmrs
#nobase_include_HEADERS = \
#	$(top_srcdir)/sickldmrs/sickldmrs.h
