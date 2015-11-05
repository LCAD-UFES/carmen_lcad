EXAMPLES = classification face_parts_recognition face_recognition \
	stereo_matching visualization

examples_recurse = \
        for i in $(EXAMPLES) xxxx ; do \
                if [ -d $$i ] ; then \
                        if ! $(MAKE) -C $$i $@ ; then \
                                exit -1; \
                        fi; \
                fi \
        done

all:
	$(examples_recurse)
clean:
	$(examples_recurse)
