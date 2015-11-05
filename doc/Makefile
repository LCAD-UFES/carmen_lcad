include ../src/Makefile.conf

MODULE_NAME = DOCUMENTATION
MODULE_COMMENT = The CARMEN Documentation

SOURCES = 
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =
PUBLIC_BINARIES = 

ifndef NO_DOXYGEN
TARGETS = carmen.doxygen  copy_files
endif

TO_HTML = tree.html tree-bin.html tree-src.html carmen-logo-tiny.png doxygen.css 

copy_to_html =   $(SILENT) \
	for i in $(TO_HTML) ; do \
		echo "    ---- Copying $$i to html/" ; \
		$(CP) -f $$i html/; \
	done 


rm_html =   $(SILENT) \
	if [ -d html ] ; then \
		echo "    ---- Removing html/" ; \
		$(RM) -rf html/; \
        fi



all: $(TARGETS)

carmen.doxygen:  carmen.dox  

copy_files:  
	$(copy_to_html)


clean:
	$(rm_html)

%.doxygen:	
	$(ECHO) "    ---- Compiling $< (Doxygen)"
	$(SILENT) $(DOXYGEN) $<

