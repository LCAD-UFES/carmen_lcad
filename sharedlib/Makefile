include $(CARMEN_HOME)/src/Makefile.conf

PACKAGES = alglib darknet gl_plotter libbee libclustering libcontrol libelas libkinfu libminorucalib libsickldmrs2 libstereovgram \
	   libtf/src libvelodyne libviso2.3/src OpenJAUS pso libcarmodel libcmt libkalman libminoru libsickldmrs libstereo prob_models Deeplab

RECURSE = $(SILENT) \
	for i in $(PACKAGES) xxxx ; do \
		if [ -d $$i ] ; then \
			if ! $(MAKE) -C $$i $@ ; then \
                                exit -1; \
                        fi; \
		fi \
	done

clean:
	$(ECHO)
	$(ECHO) "       *********"
	$(ECHO) "       C L E A N"
	$(ECHO) "       *********"
	$(ECHO)
	$(RECURSE)

