include ../../Makefile.conf

MODULE_NAME = tracker_opentld/3rdparty
MODULE_COMMENT = tracker_opentld/3rdparty

LINK = g++

# Migração Ubuntu 26.04: o libconfig vendorizado usa dynamic exception specifications
# (`throw(ParseException)` nas assinaturas), removidas do C++17 — que é o padrão default do
# GCC 15. Este diretório é código de terceiros congelado; compilar em C++14 (onde isso é só
# deprecated) evita reescrever a lib inteira.
CXXFLAGS += -std=gnu++14
# Migração Ubuntu 26.04: o libconfig vendorizado aqui tem código gerado por bison/flex com
# protótipos no estilo K&R (`int f()` = "argumentos não especificados"). O GCC 15 usa C23 por
# padrão, onde `f()` passou a significar `f(void)` — e chamar com argumentos virou erro
# ("too many arguments to function"). Compilar este diretório em C17 mantém a semântica
# antiga sem precisar regenerar o parser.
CFLAGS += -std=gnu17
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv` -I cvblobs  
LFLAGS += `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv`
SUBDIRS += 

SOURCES = cvblobs/BlobContour.cpp \
	cvblobs/blob.cpp \
	cvblobs/ComponentLabeling.cpp \
	cvblobs/BlobProperties.cpp \
	cvblobs/BlobResult.cpp \
	cvblobs/BlobOperators.cpp \
	libconfig/scanner.c \
	libconfig/libconfig.c \
	libconfig/grammar.c \
	libconfig/libconfigcpp.c++ \
	libconfig/libconfigcpp.cc \
	libconfig/strbuf.c \
	libconfig/scanctx.c 

    
PUBLIC_INCLUDES = 
    
PUBLIC_LIBRARIES = lib3rdparty.a
TARGETS = lib3rdparty.a
PUBLIC_BINARIES = 

lib3rdparty.a: cvblobs/BlobContour.o \
	cvblobs/blob.o \
	cvblobs/ComponentLabeling.o \
	cvblobs/BlobProperties.o \
	cvblobs/BlobResult.o \
	cvblobs/BlobOperators.o \
	libconfig/scanner.o \
	libconfig/libconfig.o \
	libconfig/grammar.o \
	libconfig/libconfigcpp.o \
	libconfig/libconfigcpp.o \
	libconfig/strbuf.o \
	libconfig/scanctx.o 

include ../../Makefile.rules
