#############################################################################
#       $Id: GNUmakefile,v 1.14 1999/04/05 10:28:47 reids Exp $
# $Revision: 1.14 $
#     $Date: 1999/04/05 10:28:47 $
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
# Revision 1.14  1999/04/05 10:28:47  reids
# Changes to enable TCA compile under Linux 5.2
#
# Revision 1.13  1997/06/17 12:35:27  reids
# Making system more portable -- removing hard-coded path names.
#
# Revision 1.12  97/05/29  22:07:40  reids
# Add all the needed .tcl files to the MISC var
# 
# Revision 1.11  97/05/29  16:00:36  whelan
# Messages sent from and to the same module are represented as a triangle.
# 
# Revision 1.10  97/05/01  17:19:19  whelan
# nothing. absolutely nothing.
# 
# Revision 1.9  97/05/01  16:05:42  reids
# Removed extraneous characters.
# 
# Revision 1.8  97/05/01  15:44:49  whelan
# Small bug fixes (can't even remember what they are now).
# 
# Revision 1.7  97/01/20  11:03:08  reids
# Added all the relevant comview source files to the GNUmakefile.
# 
# Revision 1.6  96/08/26  19:03:36  whelan
# 	o Bug fixes and code refinement (unfortunately this doesn't
# 	  show much of a performance increase, but the code is much
# 	  cleaner).
#
# 	o Additional keyboard control commands.
#
# 	o Menu option to toggle logging all data (ie. same as
# 	  comview's command line "-l" option).
#
# 	o Ability to toggle from the "Message" menu whether a
# 	  particular message is ignored (formerly could only be done
# 	  from the settings file, and not changed dynamically).
#
# 	o A pending bar that can be clicked to highlight corresponding
# 	  message that is actually "pending".
#
# 	o Feature to run through log file until a regular expression
# 	  is found in the log file.
#
# 	o Improved control over colors.
#
# Revision 1.5  1996/07/19  19:05:22  reids
# Added doc as a subdirectory.
#
# Revision 1.4  1996/07/19  14:26:43  whelan
# Comview directory specified from compilation directory.
#
# Revision 1.3  1996/07/18  16:39:58  whelan
# COMVIEW_DIRECTORY environment variable is no longer neccessary.
# It is passed to the C code as a define in the makefile (as the directory
# which the program is made).
#
# Revision 1.2  1996/07/18  13:09:00  whelan
# *** empty log message ***
#
# Revision 1.1  1996/06/04  18:24:53  whelan
# First release of comview.
#
# Revision 1.14  1996/02/11  21:35:27  rich
# Updated GNUmakefiles for faster complilation.  Use FAST_COMPILE=1 for
# routine recompiles.
#
# Revision 1.13  1996/02/01  04:04:45  rich
# Generalized updateVersion and added recursion.
#
# Revision 1.12  1996/01/31  22:56:00  reids
# Added automatic updating of (micro) version control numbers.
# Also, removed dependence in makefile on libtca.a
#
# Revision 1.11  1996/01/12  00:54:13  rich
# Simplified GNUmakefiles.
#
# Revision 1.10  1995/12/15  01:26:25  rich
# Fixed the includes.
#
# Revision 1.9  1995/10/07  19:10:24  rich
# Pre-alpha release of tca-8.2.
# Added PROJECT_DIR.
#
# Revision 1.8  1995/07/08  17:51:48  rich
# Linux Changes.  Also added GNUmakefile.defs.
#
# Revision 1.7  1995/07/06  21:21:09  rich
# Solaris and Linux changes.
#
# Revision 1.6  1995/06/14  18:04:11  rich
# Fixed MODULE_DIR.
#
# Revision 1.5  1995/06/14  03:24:32  rich
# Added DBMALLOC_DIR.
#
# Revision 1.4  1995/04/10  14:25:18  reids
# Changed colors for the task states, and added support for statically
# linked versions of tview.
#
# Revision 1.3  1995/04/09  20:31:23  rich
# Added /usr/local/include and /usr/local/lib to the paths for compiling
# for sunOS machines. (Support for new vendor OS).
#
# Revision 1.2  1995/04/07  05:07:26  rich
# Fixed GNUmakefiles to find the release directory.
# Fixed problems found by sgi cc compiler.  It would not compile.
#
# Revision 1.1  1995/04/05  18:31:54  rich
# Moved tview files to a subdirectory.
#
# Revision 1.4.2.1  1995/04/04  18:11:01  rich
# Added support for compiling on sgi machines.  Use "gmake -k CC=cc install"
# Fixed bugs in the declarations for some functions.
#
# Revision 1.7  1995/03/30  15:50:37  rich
# DBMALLOC works.  To use "gmake -k -w DBMALLOC=DBMALLOC install"
# Do not return pointers to local variables.
#
# Revision 1.6  1995/03/16  18:06:09  rich
# Merged in changes to the 7.9 branch.
# Changed the VERSION_ to TCA_VERSION_
#
# Revision 1.5  1995/01/31  02:05:24  rich
# Fixed CFLAGS so -gstabs comes after -g for i386_mach.
#
# Revision 1.4  1995/01/25  00:04:09  rich
# Release of tca 7.9.  Mostly speed improvements.
# The cvs binaries may now be located in /usr/local.
# Formatting changes.
#
# Revision 1.3  1994/11/02  21:38:49  rich
# Now works for linux machines (i486).
# Got afs to work on alpha (and hopefully other vendor OS's)
# Added generic Makefile and asynchronous sender/receiver.
# Renamed some X11 files and modified routines so we don't get library
# conflicts.
#
# Revision 1.2  1994/06/07  02:33:13  rich
# Removed static added for compiling for remote sites.
#
# Revision 1.1  1994/05/31  03:25:52  rich
# Moved Domingo's tview tool into the main tca module.
#
#
#############################################################################

#############################################################################
# Module Configuration Macros
#############################################################################

# Change this to point to the top level IPC directory
IPC_DIR         = ..

INSTALL_DIR	= $(IPC_DIR)
BASE_DIR	:= $(shell cd $(INSTALL_DIR);/bin/pwd)
MODULE_DIR	= comview
PROJECT		= ipc
MODULE		= comview
SUBDIRS		= doc
DEPEND_FILE	= $(PWD)/GNUmakefile.$(THIS_MACHINE)
DEPEND_PREFIX	= $(ODIR)/ 
VERSION_FILE	= main.h
VERSION_PREFIX	= COMVIEW
COMPILE_DIR	= $(shell pwd)

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
DEFS_FILE :=  $(shell ls $(BASE_DIR)/comview/GNUmakefile.defs | grep -v "not found")
#DEFS_FILE := $(shell ls $(BASE_DIR)/etc/GNUmakefile.defs | grep -v "not found")
#ifeq ($(DEFS_FILE),)
#DEFS_FILE := $(PROJECT_DIR)/etc/GNUmakefile.defs
#endif
endif

##########
# Include standard definitions
##########

include $(DEFS_FILE)

##########
# Override defaults here.
##########

DEFINES		+= -DCOLOR_WORKSTATION -DCOMVIEW_DIRECTORY=\"$(COMPILE_DIR)\"

CFLAGS		= -g -O0 $(CFLAGS_$(CC)) $(CFLAGSD_$(DEBUG)) \
		  $(CFLAGS_$(THIS_MACHINE)) \
		  $(CFLAGSM_$(DBMALLOC)) $(CFLAGS_EXT) -I/usr/include/tcl

LIBRARIES	= -ltk -ltcl -lX11 -lm -ldl -lXaw -lXmu \
		  -lXext -lXt \
		   $(LIBS_$(THIS_OS)) $(LIBS_$(DBMALLOC))

##########
# File Definition Macros.  Deletion of generated files assumed
#    to be o.k.  Source files will never be deleted.
##########

# Generated files - installed
PUBLIC_BINS		= comview
PUBLIC_LIBS		= 
PUBLIC_LINTS		= 

MODULE_LIBS		= 
MODULE_LINTS		= 

# Source files - installed
PUBLIC_INCS		= 
PUBLIC_MANS		= 
PUBLIC_DOCS		= 

MODULE_INCS		= 
MODULE_DOCS		= 

INCS			= $(MODULE_INCS) $(PUBLIC_INCS)

# Generated files - not installed
PRIVS	= 

_OBJECTS	= Top.o comview.o tclsetup.o main.o \
		  Array.o Debug.o List.o MyString.o Parser.o TaskTree.o \
		  y.tab.o

OBJS            = $(patsubst %,$(ODIR)/%,$(_OBJECTS))

# Source files - not installed

SRCS            = Top.c comview.c tclsetup.c main.c \
		  Array.c Debug.c List.c MyString.c Parser.c TaskTree.c \
		  parse.l parse.y

PRIVATE_INCS	= Top.h Handler.h comview.h tclsetup.h tcltk.h main.h \
		  Debug.h Memory.h

MISC		= GNUmakefile README \
		colors.tcl FSBox.tcl options.tcl orig-colors.tcl comview.tcl \
		queues.tcl config.tcl sampleuser.tcl cregexp.tcl \
		set-colors.tcl tca-header.tcl keyboard.tcl menu.tcl mouse.tcl \
		layout.tcl logwindow.tcl main.tcl vars.tcl vistool.tcl

DELETES		+= lex.yy.c y.tab.c

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

#LEX = lex
LEX = flex
# YACC = yacc -t
YACC = bison -b y

lex.yy.c: parse.l
	$(LEX) parse.l

y.tab.c: parse.y lex.yy.c
	$(YACC) parse.y

$(BDIR)/comview: $(OBJS) $(LIBDEP)
	$(LINK.c) -o $@ $(OBJS) $(LIBRARIES)

$(BDIR)/comview.static: $(OBJS) $(LIBDEP)
	$(LINK.c) -static -o $@ $(OBJS) $(LIBRARIES)

##########
# Include standard target definitions and RCS rules
##########

include $(STD_FILE)
