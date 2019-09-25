include ../Makefile.conf

##SUBDIRS = pioneerlib rflexlib scoutlib xr4000_base segway_base orcbotlibv4 orcbotlib

SUBDIRS = pioneerlib

#for robs in $(ROBOT_TYPES); do 	\
#	ifeq ($$robs,pioneer) 	SUBDIRS += pioneerlib ; endif ; \
# 	ifeq ($$robs,rflex)    	SUBDIRS += rflexlib ; endif ; \
#	ifeq ($$robs,scout) 	SUBDIRS += scoutlib ; endif ; \
#	ifeq ($$robs,xr4000) 	SUBDIRS += xr4000_base ; endif ; \
#	ifeq ($$robs,seqway) 	SUBDIRS += segway_base ; endif ; \
#	ifeq ($$robs,orc4) 	SUBDIRS += orcbotlibv4 ; endif ; \
#	ifeq ($$robs,orc5) 	SUBDIRS += orcbotlib ; 	endif ; \
#done

CFLAGS += 
IFLAGS += -I../robot -I../laser
LFLAGS += -lglobal -lipc -lparam_interface -lcarmenserial -lgeometry  -lm -lbase_ackerman_interface -lobstacle_avoider_interface

MODULE_NAME = BASE
MODULE_COMMENT = Generic interface to all robot bases

SOURCES = base_interface.c base_test.c drive_main.c arm_main.c base_main.c arm_test.c
PUBLIC_INCLUDES = base_messages.h arm_messages.h base_interface.h \
	arm_interface.h arm_low_level.h drive_low_level.h
PUBLIC_LIBRARIES = libbase_interface.a libarm_interface.a 

#PUBLIC_INCLUDES = localize_ackerman_messages.h localize_ackerman_interface.h localize_ackerman_core.h \
	          localize_ackerman_likelihood_map.h localize_ackerman_motion.h localize_ackerman_velodyne.h  
#PUBLIC_LIBRARIES = liblocalize_ackerman_interface.a liblocalize_ackerman_core.a liblocalize_ackerman_motion.a
#PUBLIC_BINARIES = localize_ackerman localize_ackerman_particle_viewer


TARGETS =  libbase_interface.a libarm_interface.a  base_test arm_test

TARGETS += $(ROBOT_TYPES)
PUBLIC_BINARIES += $(ROBOT_TYPES)

PUBLIC_LIBRARIES_SO = libbase_interface.so libarm_interface.so
ifndef NO_PYTHON
TARGETS += libbase_interface.so.1 libarm_interface.so.1
endif

# rules

libbase_interface.a:	base_interface.o

libbase_interface.so.1:	base_interface.o

libarm_interface.a:	arm_interface.o

libarm_interface.so.1:	arm_interface.o

base_test:		base_test.o libbase_interface.a

arm_test:		arm_test.o libarm_interface.a

# mains for each robot
scout: base_main.o scoutlib/libscout.a 

pioneer: base_main.o pioneerlib/libpioneer.a 

orc4: orcbotlibv4/orc4_main.o orcbotlibv4/orclib/liborc.a

orc5: drive_main.o orcbotlib/orclib_v5/liborc.a orcbotlib/liborcbot.a

orc5_arm: arm_main.o orcbotlib/orclib_v5/liborc.a orcbotlib/liborcbot.a

rflex: base_main.o rflexlib/librflex.a

segway: 
	$(MAKE) -C segway_base

xr4000: 
	$(MAKE) -C xr4000_base

# libraries for each robot
scoutlib/libscout.a: 
	$(MAKE) -C scoutlib 

pioneerlib/libpioneer.a: 
	$(MAKE) -C pioneerlib

orcbotlibv4/orclib/liborc.a:
	$(MAKE) -C orcbotlibv4/orclib

orcbotlibv4/orc4_main.o:
	$(MAKE) -C orcbotlibv4 orc4_main.o 

orcbotlib/orclib_v5/liborc.a:
	$(MAKE) -C orcbotlib/orclib_v5

orcbotlib/liborcbot.a:
	$(MAKE) -C orcbotlib

rflexlib/librflex.a: 
	$(MAKE) -C rflexlib

include ../Makefile.rules
