#############################################################################
#       $Id: GNUmakefile,v 1.4 1997/06/17 12:34:56 reids Exp $
# $Revision: 1.4 $
#     $Date: 1997/06/17 12:34:56 $
#   $Author: reids $
#    $State: Exp $
#   $Locker:  $
#
# PROJECT:	TCA
#
# FILE:		GNUmakefile
#		Based on Erebus prototypical Makefile
#		Makefile.proto,v 1.1 1992/06/16 22:50:21 jwest Exp 
#
# DESCRIPTION:
# Makes the tca libraries and central server.
#
# EXPORTS:
#	The following targets are available as part of Makefile.std:
#		all bins libs privs objs
#		clean install
#		lint depend
#
# HISTORY:
#
# $Log: GNUmakefile,v $
# Revision 1.4  1997/06/17 12:34:56  reids
# Making system more portable -- removing hard-coded path names.
#
# Revision 1.3  97/05/29  16:01:06  whelan
# Messages sent from and to the same module are represented as a triangle.
# 
# Revision 1.2  1996/08/23  17:05:57  rich
# Typos.
#
# Revision 1.1  1996/08/23  17:02:17  rich
# Added makefile.
#
#
#
#############################################################################

#############################################################################
# Module Configuration Macros
#############################################################################

IPC_DIR		= ../..

INSTALL_DIR	= $(IPC_DIR)
BASE_DIR	:= $(shell cd $(INSTALL_DIR);/bin/pwd)
MODULE_DIR	= comview/doc
PROJECT		= tca
MODULE		= comview
SUBDIRS		= 
DEPEND_FILE	= 
DEPEND_PREFIX	= 
VERSION_FILE	= 
VERSION_PREFIX	= 

#############################################################################
# Where are the standard files?
#############################################################################

ifndef FAST_COMPILE
PWD 	:= $(shell /bin/pwd)
endif

ifndef STD_FILE
STD_FILE := $(shell ls $(BASE_DIR)/etc/GNUmakefile.std | grep -v "not found")
ifeq ($(STD_FILE),)
STD_FILE := $(PROJECT_DIR)/etc/GNUmakefile.std
endif
DEFS_FILE := $(shell ls $(BASE_DIR)/etc/GNUmakefile.defs | grep -v "not found")
ifeq ($(DEFS_FILE),)
DEFS_FILE := $(PROJECT_DIR)/etc/GNUmakefile.defs
endif
endif

##########
# Include standard definitions
##########

include $(DEFS_FILE)

##########
# Override defaults here.
##########

##########
# File Definition Macros.  Deletion of generated files assumed
#    to be o.k.  Source files will never be deleted.
##########

# Generated files - installed
PUBLIC_BINS		= 
PUBLIC_LIBS		= 
PUBLIC_LINTS		= 

MODULE_LIBS		= 
MODULE_LINTS		= 

# Source files - installed
PUBLIC_INCS		= 
PUBLIC_MANS		= 
PUBLIC_DOCS		= 

MODULE_INCS		= 
MODULE_DOCS		= comview.pdf

INCS			= $(MODULE_INCS) $(PUBLIC_INCS)

# Generated files - not installed
PRIVS	= 

FIGURES		= comview1.ps.gz gomsg.ps.gz \
		gotime.ps.gz inform.ps.gz modlist.ps.gz msglist.ps.gz\
		pend.ps.gz query.ps.gz search.ps.gz showall.ps.gz

MISC		= GNUmakefile comview.tex $(FIGURES)

##########
# Default Target
##########

all:: srcs libs bins privs

##########
# User Defined Targets
##########

# Define rules for all files listed in BINS, LIBS, and LINTS in this section.
# Example executable, object, library, and combined objects are shown below:
#
#  $(BDIR)/sample: file1.o file2.o $(LIBDEP)
#	$(LINK.c) -o $@ file1.o file2.o $(LIBRARIES)
#
#  $(ODIR)/alternateName.o: file1.c
#	$(COMPILE.c) -o $@ file1.c
#
#  $(LDIR)/libsample.a: file1.o file2.o
#	-$(RM) $@
#	$(AR) $(ARFLAGS) $@ file1.o file2.o
#	$(RANLIB) $@
#
#  $(ODIR)/combined.o: $(ODIR)/part1.o $(ODIR)/part2.o
#	$(LD) -r -o $@ $(ODIR)/part1.o $(ODIR)/part2.o; chmod a-x $@

LATEX = latex

DELETES += comview.aux comview.log comview.dvi comview.ps.gz comview.toc

# Run latex twice to get references correct
comview.pdf: comview.tex $(FIGURES)
	gunzip $(FIGURES)
	$(LATEX) comview.tex
	$(LATEX) comview.tex
	dvipdf comview.dvi
	gzip *.ps

##########
# Include standard target definitions and RCS rules
##########

include $(STD_FILE)
