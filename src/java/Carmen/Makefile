include ../../Makefile.conf

MODULE_NAME = "Carmen"
MODULE_COMMENT = "Java class files for Carmen"

JAVA_FILES = $(wildcard *.java)
CLASS_FILES := $(JAVA_FILES:.java=.class) $(wildcard *.class)

TARGETS = ../Carmen-1.0.jar

../Carmen-1.0.jar: $(JAVA_FILES)
	$(ECHO) "    ---- Compiling Carmen (Java)"
	$(SILENT) $(JAVA_HOME)/bin/javac -classpath .. -d .. $(JAVA_FILES)

include ../../Makefile.rules
