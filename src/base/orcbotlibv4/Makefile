include ../../Makefile.conf

SUBDIRS = orclib

ifndef CYGWIN
CFLAGS += -fPIC 
endif
CFLAGS += -g -DBASE_HAS_ARM 
IFLAGS +=
LFLAGS += 

MODULE_NAME = "ORC_V4"
MODULE_COMMENT = "Legacy main for controlling ORC (v4) robots"

SOURCES = base_main.c 

PUBLIC_LIBRARIES =
PUBLIC_BINARIES = 
MAN_PAGES = 

orc4_main.o: base_main.c
	$(ECHO) "    ---- Compiling $< to $@ (C)"
	$(SILENT) $(CC) $(CFLAGS) $(IFLAGS) -c $< -o $@

include ../../Makefile.rules
