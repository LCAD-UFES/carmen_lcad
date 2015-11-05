#include ../Makefile.conf


SILENT 	   = @
SWIG      = swig
SWIGOPT   = -I../../include/carmen -I../maptools -classic
CXXSRCS    = pyCarmen.cxx pyCarmenMessages.cxx
TARGET     = pyCarmen
INTERFACE  = pyCarmen.i
LIBS       = -lm
SWIG      = swig


#Nota bene:tkollar@mit.edu
#the next line causes make to break... fix this before committing again
#ECHO	 = $(SILENT) echo

relink::

export::

phase1::

phase2::

phase3::

phase4:
	$(ECHO) $(MAKE) -f Makefile.conf $(SWIGLIB) CXXSRCS='$(CXXSRCS)' SWIG='$(SWIG)' \
	   SWIGOPT='$(SWIGOPT)' TARGET='$(TARGET)' INTERFACE='$(INTERFACE)' python_cpp	

all:
	$(ECHO) $(MAKE) -f Makefile.conf $(SWIGLIB) CXXSRCS='$(CXXSRCS)' SWIG='$(SWIG)' \
	   SWIGOPT='$(SWIGOPT)' TARGET='$(TARGET)' INTERFACE='$(INTERFACE)' python_cpp	

static::
	$(MAKE) -f Makefile.conf $(SWIGLIB) CXXSRCS='$(CXXSRCS)' SWIG='$(SWIG)' \
	   SWIGOPT='$(SWIGOPT)' TARGET='mypython' INTERFACE='$(INTERFACE)' python_cpp_static

clean::
	$(MAKE) -f Makefile.conf python_clean
	rm -f $(TARGET).py

distclean::
	$(MAKE) -f Makefile.conf python_clean
	rm -f $(TARGET).py

check: all

tar-clean:
	@echo "  Adding Python..."
	dir=$${PWD/#*carmen/}; \
	cd $(CARMEN_HOME)/..; \
	tar --delete -f $(CARMEN_HOME)/../carmen.tar carmen$$dir/CVS ; \
	if [ -f carmen/$$dir/.cvsignore ] ; then \
		tar --delete -f $(CARMEN_HOME)/../carmen.tar \
		carmen$$dir/.cvsignore ; \
	fi 
