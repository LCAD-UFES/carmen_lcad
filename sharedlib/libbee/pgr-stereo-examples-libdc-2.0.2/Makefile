
PKG =	pgrlibdcstereo	\
	simplegrab \
	simplestereo \
#	hdrgrab

all: 
	for pkg in $(PKG) ; do \
	    ( echo $$pkg:$@ ; cd $$pkg ; $(MAKE) $(MFLAGS) ) ;\
	done

clean:
	for pkg in $(PKG) ; do \
	    ( echo $$pkg:$@ ; cd $$pkg ; $(MAKE) $(MFLAGS) $@ ) ;\
	done

