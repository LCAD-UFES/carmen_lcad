include ../Makefile.conf

CFLAGS = -Wall -Wstrict-prototypes -Wnested-externs -Wmissing-prototypes -Wmissing-declarations -O3 -DREDHAT_52 -DREDHAT_6 -DREDHAT_71 -g

ifeq ($(PROCESSOR),x86_64)
CFLAGS += -fPIC
endif

ifndef NO_TCPD
CFLAGS += -DACCESS_CONTROL
LFLAGS = -lnsl -lwrap
endif

MODULE_NAME = IPC
MODULE_COMMENT = Flexible communications package written by Reid Simmons

SOURCES = ipc.c queryResponse.c marshall.c timer.c list.c behaviors.c \
	tcModError.c datamsg.c formatters.c hash.c idtable.c key.c \
	primFmttrs.c reg.c sendMsg.c tcaMem.c tcaRef.c comModule.c com.c \
	globalM.c globalMUtil.c strList.c modLogging.c modVar.c resMod.c \
	parseFmttrs.c lex.c printData.c	comServer.c dispatch.c msgTap.c \
	recvMsg.c res.c tcerror.c logging.c globalS.c centralIO.c \
	globalVar.c central.c test_generate.c test_receive.c multiThread.c

PUBLIC_INCLUDES = ipc.h
PUBLIC_LIBRARIES = libipc.a 
PUBLIC_BINARIES = central
TARGETS = central  libipc.a test_generate test_receive ipc-endian-test ipc-die-test

PUBLIC_LIBRARIES_SO =  libipc.so
ifndef NO_PYTHON
TARGETS += libipc.so.1
endif

libipc.a: ipc.o queryResponse.o marshall.o timer.o list.o behaviors.o \
	tcModError.o datamsg.o formatters.o hash.o idtable.o key.o \
	primFmttrs.o reg.o sendMsg.o tcaMem.o tcaRef.o comModule.o com.o \
	globalM.o globalMUtil.o strList.o modLogging.o modVar.o resMod.o \
	parseFmttrs.o lex.o printData.o 

libipc.so.1: ipc.o queryResponse.o marshall.o timer.o list.o behaviors.o \
	tcModError.o datamsg.o formatters.o hash.o idtable.o key.o \
	primFmttrs.o reg.o sendMsg.o tcaMem.o tcaRef.o comModule.o com.o \
	globalM.o globalMUtil.o strList.o modLogging.o modVar.o resMod.o \
	parseFmttrs.o lex.o printData.o 

central: comServer.o dispatch.o msgTap.o recvMsg.o res.o tcerror.o logging.o \
	globalS.o centralIO.o globalVar.o central.o libipc.a

test_generate: test_generate.o libipc.a
test_receive: test_receive.o libipc.a

ipc-endian-test: ipc-endian-test.o libipc.a

ipc-die-test: ipc-die-test.o libipc.a

include ../Makefile.rules


