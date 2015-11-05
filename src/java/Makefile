include ../Makefile.conf

SUBDIRS = LDLoad Carmen IPC

IFLAGS += -I$(JAVA_HOME)/include 
IFLAGS += -I$(JAVA_HOME)/include/linux 
IFLAGS += -I/usr/include/bsd
IFLAGS += -I ../ipc
LFLAGS += -L ../../lib
CFLAGS += -DREDHAT_52 -DREDHAT_6 -DREDHAT_71 
CFLAGS += -Wno-unused-parameter

ifeq ($(PROCESSOR),x86_64)
CFLAGS += -fPIC
endif

MODULE_NAME = JAVA LIBRARIES
MODULE_COMMENT = Module for interfacing java programs with Carmen!

SOURCES = ipcjava.c
PUBLIC_LIBRARIES = libipcjava.so
TARGETS = libipcjava.so Carmen-1.0.jar

libipcjava.so: ipcjava.o
	$(ECHO) "    ---- Creating shared library (C)"
	$(SILENT) ld -o $@ -G -dy $^ -L ../ipc -lipc

JAVA_FILES := $(wildcard $(patsubst %,%/*.java,$(SUBDIRS)))


Carmen-1.0.jar: $(JAVA_FILES)
	$(MAKE) -C LDLoad
	$(MAKE) -C IPC
	$(MAKE) -C Carmen
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(JAVA_HOME)/bin/jar cf Carmen-1.0.jar Carmen/*.class IPC/*.class LDLoad/*.class libipcjava.so
#	$(SILENT) $(JAVA_HOME)/bin/jar cf Carmen-1.0.jar Carmen/*.class IPC/*.class libipcjava.so

%.class: %.java
	$(ECHO) "    ---- Compiling $@ (Java)"
	$(SILENT) $(JAVA_HOME)/bin/javac -classpath Carmen-1.0.jar:. -d . $<

# rules

#IPC.h : IPC.java
#	javah -classpath . -force -jni -o IPC.h IPC.IPC

#formatters.h : formatters.java
#	javah -classpath . -force -jni -o formatters.h IPC.formatters

#primFmttrs.h : primFmttrs.java
#	javah -classpath . -force -jni -o primFmttrs.h IPC.primFmttrs

include ../Makefile.rules
