#
# Makefile -- build instructions for Robolocust's control program for
#             iRobot Create's Command Module
#
# This makefile builds the control program used by the Robolocust/lobot
# project to talk to the iRobot Create's Command Module. This program
# acts as an intermediary between the higher (C++) layers of the lobot
# controller and the iRobot Create's sensorimotor system. That is,
# instead of talking directly to the iRobot Create, the higher layers of
# the controller will issue "high-level" motor commands to the Command
# Module's control program, which, in turn, will take care of issuing the
# appropriate sequence of bytes comprising the equivalent Open Interface
# commands that the iRobot Create understands.
#
# To build this control program, we will need the AVR build tools
# installed, viz., avr-gcc, avr-libc and avrdude.
#
# DEVNOTE: This makefile has been adapted from the ones available with
# the example programs that come with the iRobot Create's Command Module.
# It requires GNU make.
#

# To build the control program and load it to the iRobot Create Command
# in one step, issue the following command: make load


########################################################################
#                                                                      #
# The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   #
# by the University of Southern California (USC) and the iLab at USC.  #
# See http:#iLab.usc.edu for information about this project.           #
#                                                                      #
########################################################################
#                                                                      #
# Major portions of the iLab Neuromorphic Vision Toolkit are protected #
# under the U.S. patent ``Computation of Intrinsic Perceptual Saliency #
# in Visual Environments, and Applications'' by Christof Koch and      #
# Laurent Itti, California Institute of Technology, 2001 (patent       #
# pending; application number 09/912,225 filed July 23, 2001; see      #
# http:#pair.uspto.gov/cgi-bin/final/home.pl for current status).      #
#                                                                      #
########################################################################
#                                                                      #
# This file is part of the iLab Neuromorphic Vision C++ Toolkit.       #
#                                                                      #
# The iLab Neuromorphic Vision C++ Toolkit is free software; you can   #
# redistribute it and/or modify it under the terms of the GNU General  #
# Public License as published by the Free Software Foundation; either  #
# version 2 of the License, or (at your option) any later version.     #
#                                                                      #
# The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  #
# that it will be useful, but WITHOUT ANY WARRANTY; without even the   #
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      #
# PURPOSE.  See the GNU General Public License for more details.       #
#                                                                      #
# You should have received a copy of the GNU General Public License    #
# along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   #
# to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   #
# Boston, MA 02111-1307 USA.                                           #
#                                                                      #
########################################################################

# Primary maintainer for this file: Manu Viswanathan mviswana usc edu
# $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/Makefile $
# $Id: Makefile 12805 2010-02-09 22:42:47Z mviswana $

#----------------------------- BUILD TOOLS ------------------------------

SHELL = /bin/bash
TEST  = /usr/bin/test

CC = /usr/bin/avr-gcc
NM = /usr/bin/avr-nm

OBJCOPY = /usr/bin/avr-objcopy
OBJDUMP = /usr/bin/avr-objdump
AVRDUDE = /usr/bin/avrdude

SIZE = /usr/bin/avr-size

#-------------- iROBOT CREATE COMMAND MODULE BUILD SPECS ----------------

# Microcontroller
MCU = atmega168

# Processor frequency
#
# DEVNOTE: This will define a symbol, F_CPU, in all source code files.
# You can then use this symbol in your source code to calculate timings.
# Do NOT tack on a 'UL' at the end, this will be done automatically to
# create a 32-bit value in your source code.
F_CPU = 18432000

# Output format (can be srec, ihex, binary)
OUTPUT_FORMAT = ihex

#---------- LOBOT COMMAND MODULE CONTROL PROGRAM BUILD SPECS ------------

# Final target name (without extension)
TARGET = LoCreateCmdModule

# Extra directories to look for include files
#
# DEVNOTE: Each directory must be seperated by a space. Use forward
# slashes for directory separators. For a directory that has spaces,
# enclose it in quotes.
EXTRAINCDIRS =

# Extra -I options
CINCS =

# Extra #define/#undef macros
CDEFS = -DF_CPU=${F_CPU}UL

# To which communications port is the AVR microcontroller programmer
# connected?
AVRDUDE_PORT = /dev/ttyUSB0

# Whether to use minimal printf/scanf or to have floating point support
# built into them. If these variables are not defined, the standard
# versions of these functions will be used.
PRINTF_LIB =
#PRINTF_LIB = ${PRINTF_LIB_MIN}
#PRINTF_LIB = ${PRINTF_LIB_FLOAT}

SCANF_LIB =
#SCANF_LIB = ${SCANF_LIB_MIN}
#SCANF_LIB = ${SCANF_LIB_FLOAT}

# Optimization level: 0/1/2/3/s
#
# DEVNOTE: 0 ==> turn off optimization
#          s ==> optimize for size
#
# DEVNOTE: 3 is not always the best optimization level. See avr-libc FAQ.
OPT = s

# C source files
#
# DEVNOTE: C dependencies are automatically generated.
SRC = $(wildcard *.c)

# Assembler sources
#
# DEVNOTE: Remember to always end assembler source file names in a
# capital .S. Files ending in a lowercase .s will not be considered
# source files but generated files (assembler output from the compiler),
# and will be deleted any time you make clean.
ASRC = $(wildcard *.S)

# Debugging format
#
# DEVNOTE: Native formats for AVR-GCC's -g are dwarf-2 [default] or
# stabs. AVR Studio 4.10 requires dwarf-2. AVR [Extended] COFF format
# requires stabs, plus an avr-objcopy run.
DEBUG = stabs

# Compiler flag to set the C Standard level
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu99

#-------------------------- COMPILER OPTIONS ----------------------------

# -g      ==> generate debugging information
# -O      ==> optimization level
# -f      ==> tuning, see GCC manual and avr-libc documentation
# -Wall   ==> warning level
# -Wa,    ==> tell GCC to pass this to the assembler
# -adhlns ==> create assembler listing

CFLAGS = -g${DEBUG} ${CDEFS} ${CINCS} $(patsubst %,-I%,${EXTRAINCDIRS})   \
         -O${OPT} ${CSTANDARD} -Wall -Wstrict-prototypes                  \
         -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
         -Wa,-adhlns=$(<:.c=.lst)

# Compiler flags to generate dependencies
DEPFLAGS = -MD -MP -MF .dep/$(@F).d

# Combine all necessary and optional flags, including target processor
ALL_CFLAGS = -mmcu=${MCU} -I. ${CFLAGS} ${DEPFLAGS}

#-------------------------- ASSEMBLER OPTIONS ---------------------------

# -Wa,    ==> tell GCC to pass this to the assembler
# -ahlms  ==> create listing
# -gstabs ==> have assembler create line number information
#             NOTE: for use in COFF files, additional information about
#             filenames and function names needs to be present in the
#             assembler source files (see avr-libc docs)

ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs

# Combine all necessary and optional flags, including target processor
ALL_ASFLAGS = -mmcu=${MCU} -I. -x assembler-with-cpp ${ASFLAGS}

#--------------------------- LINKER OPTIONS -----------------------------

# -Wl,   ==> tell GCC to pass this to linker
# -Map   ==> create map file
# --cref ==> add cross reference to map file

# Linker options for a minimalistic printf/scanf
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min
SCANF_LIB_MIN  = -Wl,-u,vfscanf -lscanf_min

# Linker options for a maximalistic printf/scanf (i.e., with floating
# point support)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt
SCANF_LIB_FLOAT  = -Wl,-u,vfscanf -lscanf_flt

# Linker option for linking with the standard math library
#
# NOTE: The maximalistic printf and scanf need to link with this library.
MATH_LIB = -lm

# External memory options

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--defsym=__heap_start=0x801100,--defsym=__heap_end=0x80ffff

EXTMEMOPTS =

# The final linker options
LDFLAGS = -Wl,-Map=${TARGET}.map,--cref ${EXTMEMOPTS} \
          ${PRINTF_LIB} ${SCANF_LIB} ${MATH_LIB}

#----------------- MICROCONTROLLER PROGRAMMER OPTIONS -------------------

# Programming hardware: alf avr910 avrisp bascom bsd dt006 pavr picoweb
#                       pony-stk200 sp12 stk200 stk500
AVRDUDE_PROGRAMMER = stk500v1

# Options for writing code to onboard flash memory or EEPROM
AVRDUDE_WRITE_FLASH = -U flash:w:${TARGET}.hex
#AVRDUDE_WRITE_EEPROM = -U eeprom:w:${TARGET}.eep

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level. Please use this when submitting bug reports
# about avrdude. See http://savannah.nongnu.org/projects/avrdude to
# submit bug reports.
#AVRDUDE_VERBOSE = -v -v

AVRDUDE_FLAGS = -p ${MCU} -P ${AVRDUDE_PORT} -c ${AVRDUDE_PROGRAMMER} \
                ${AVRDUDE_NO_VERIFY} ${AVRDUDE_VERBOSE}               \
                ${AVRDUDE_ERASE_COUNTER}

#---------------------------- BUILD OBJECTS -----------------------------

# Object files
OBJ = $(SRC:.c=.o) $(ASRC:.S=.o)

# Listing files
LST = $(SRC:.c=.lst) $(ASRC:.S=.lst)

#---------------------------- PATTERN RULES -----------------------------

# Compile C sources into assembler sources
%.s: %.c
	@echo "Compiling $< to assembly"
	@${CC} -S ${ALL_CFLAGS} $< -o $@

# Assemble assembler sources into their corresponding object modules
%.o: %.S
	@echo "Assembling $<"
	@${CC} -c ${ALL_ASFLAGS} $< -o $@

# Compile C sources to their corresponding object modules
%.o: %.c
	@echo "Compiling $<"
	@${CC} -c ${ALL_CFLAGS} $< -o $@

# Link all object modules to create ELF executable
%.elf: ${OBJ}
	@echo "Linking $@"
	@${CC} ${ALL_CFLAGS} $^ --output $@ ${LDFLAGS}

# Convert the .elf generated by GCC to a .hex file suitable for loading
# to the microcontroller's flash memory.
%.hex: %.elf
	@echo "Converting $< to $@ for loading to flash"
	@if ${TEST} -e "$@" ; then \
	    echo ; echo "Code size from previous build:" ;\
	    ${SIZE} --target=${OUTPUT_FORMAT} $@ ;\
	 fi
	@${OBJCOPY} -O ${OUTPUT_FORMAT} -R .eeprom $< $@
	@echo; echo "Code size:"
	@${SIZE} --target=${OUTPUT_FORMAT} $@
	@echo

# Convert the .elf generated by GCC to a .eep file suitable for loading
# to the microcontroller's EEPROM.
%.eep: %.elf
	@echo "Converting $< to $@ for loading to EEPROM"
	-@${OBJCOPY} -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	             --change-section-lma .eeprom=0 -O ${OUTPUT_FORMAT} $< $@

# Creating an extended listing file from GCC's ELF output
%.lss: %.elf
	@echo "Creating extended listing $@ from $<"
	@${OBJDUMP} -h -S $< > $@

# Creating a symbol table from the ELF output
%.sym: %.elf
	@echo "Creating symbol table $@ from $<"
	@${NM} -n $< > $@

# Create preprocessed source for use in sending a bug report
%.i: %.c
	@echo "Creating preprocessed source $@"
	@${CC} -E -mmcu=${MCU} -I. ${CFLAGS} $< -o $@

#-------------------------- BUILT-IN TARGETS ----------------------------

# Instruct make not to waste time looking for these targets in the file
# system.
.PHONY : all elf hex eep lss sym load clean

# Instruct make not delete the object files or the ELF file built by GCC
.PRECIOUS  : ${OBJ}
.SECONDARY : ${TARGET}.elf

#----------------------------- BUILD RULES ------------------------------

# Default target: build everything
all: elf hex eep lss sym

# Rules/targets for building individual output files
elf: ${TARGET}.elf
hex: ${TARGET}.hex
eep: ${TARGET}.eep
lss: ${TARGET}.lss
sym: ${TARGET}.sym

# Load the program into the microcontroller
load: ${TARGET}.hex ${TARGET}.eep
	${AVRDUDE} ${AVRDUDE_FLAGS} ${AVRDUDE_WRITE_FLASH} \
        ${AVRDUDE_WRITE_EEPROM}

# Get rid of build related gunk
clean:
	${RM} ${TARGET}.{elf,hex,eep,map,lss,sym}
	${RM} ${OBJ}
	${RM} ${LST}
	${RM} ${SRC:.c=.s}
	${RM} ${SRC:.c=.d}
	${RM} .dep/*

# Include the dependency files
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
