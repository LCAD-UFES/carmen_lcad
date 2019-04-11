total:	home export phase1 phase2 phase3 phase4

include Makefile.conf

PACKAGES = ipc global param_daemon proccontrol robot_ackerman_gui laser-new  	\
	   obstacle_avoider simulator_ackerman localize_ackerman robot_ackerman \
	   maptools mapeditor logger dynamic_object_detector \
	   navigator_ackerman joystick pantilt laser gps gps_xyz imu localize_neural	\
	   fused_odometry moving_objects bumblebee_basic laser-ldmrs viewer_3D 		\
	   stereo road_finding v_disparity stereo_mapping grid_mapping 		\
	   visual_odometry_package2 stereo_point_cloud xsens xsens_MTi-G	\
	   rrt_planner subdir_module_example velodyne log_filter rddf 		\
	   download_map parking_assistant navigator_astar 			\
	   navigator_spline navigator_gui navigator_gui2		\
	   mapper obstacle_distance_mapper map_server car_panel_gui		\
	   v_disparity stereo_velodyne motion_planner udatmo1 behavior_selector \
	   base_ackerman driving_playback ford_escape_hybrid			\
	   path_interpolator_planner kinect ultrasonic 		\
	   odometry_calibration utilities visual_tracker	\
	   visual_car_tracking model_predictive_planner utilities \
	   moving_objects_simulator zed_camera2 graphslam hypergraphsclam pi_imu pi_camera \
	   neural_object_detector road_mapper can_dump traffic_light # tracker_opentld tracker voice_interface 

ifndef NO_CPP
PACKAGES += carmenpp
endif

TAR_PACKAGE_LIST = $(filter-out laser, $(PACKAGES)) laser-new laser-old

ifndef NO_PYTHON
PACKAGES += python
else
TAR_PACKAGE_LIST += python
endif

ifdef JAVA_HOME
PACKAGES += java 
else
TAR_PACKAGE_LIST += python
endif


RECURSE = $(SILENT) \
	for i in $(PACKAGES) xxxx ; do \
		if [ -d $$i ] ; then \
			if ! $(MAKE) -C $$i $@ ; then \
                                exit -1; \
                        fi; \
		fi \
	done

INSTALL_RECURSE = $(SILENT) \
	for i in $(PACKAGES) xxxx ; do \
		if [ -d $$i ] ; then \
			if ! $(MAKE) install -C $$i $@ ; then \
                                exit -1; \
                        fi; \
		fi \
	done

loud:
	$(SILENT) $(MAKE) MAKEFLAGS= LOUD=1 total

home: 
	$(SILENT) $(MKDIR) $(LIB_DIR)
	$(SILENT) $(MKDIR) $(SHARED_DIR)
	$(SILENT) $(MKDIR) $(BIN_DIR)
	$(SILENT) $(MKDIR) $(INC_DIR)/carmen
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(SILENT) $(MKDIR) $(MAN_DIR)
	if ! test -f carmen.ini ; then \
	   echo "    ---- Copying global/carmen-std.ini to carmen.ini " ; \
	   $(CP) global/carmen-std.ini carmen.ini ; \
	fi

export:
	$(ECHO)
	$(ECHO) "       ***********"
	$(ECHO) "       E X P O R T"
	$(ECHO) "       ***********"
	$(ECHO)
	$(RECURSE)

dep depend:
	$(ECHO)
	$(ECHO) "       ***********"
	$(ECHO) "       D E P E N D"
	$(ECHO) "       ***********"
	$(ECHO)
	$(RECURSE)

phase1:
	$(ECHO)
	$(ECHO) "       *****************"
	$(ECHO) "       L I B R A R I E S"
	$(ECHO) "       *****************"
	$(ECHO)
	$(RECURSE)

phase2:
	$(ECHO)
	$(ECHO) "       ***************"
	$(ECHO) "       B I N A R I E S"
	$(ECHO) "       ***************"
	$(ECHO)
	$(RECURSE)
	$(ECHO) "Done making binaries..."


phase3:
ifndef NO_PYTHON
	$(ECHO)
	$(ECHO) "       *******************************"
	$(ECHO) "       S H A R E D   L I B R A R I E S"
	$(ECHO) "       *******************************"
	$(ECHO)
	$(RECURSE)
endif

phase4:
ifndef NO_PYTHON
	$(ECHO)
	$(ECHO) "       ***********"
	$(ECHO) "       P Y T H O N"
	$(ECHO) "       ***********"
	$(ECHO)
	$(SILENT) make -C python phase4
endif

install:
	$(ECHO)
	$(ECHO) "	*************"
	$(ECHO) "	I N S T A L L"
	$(ECHO) "	*************"
	$(ECHO)
	$(SILENT) $(MKDIR) $(GLOBAL_INC_DIR)/carmen
	$(SILENT) $(MKDIR) $(GLOBAL_BIN_DIR)
	$(SILENT) $(MKDIR) $(GLOBAL_LIB_DIR)
	$(SILENT) $(MKDIR) $(GLOBAL_MAN_DIR)
	$(INSTALL_RECURSE)

tar:
	$(ECHO) $(TAR_PACKAGE_LIST)
	$(SILENT) cp Makefile.vars Makefile.vars.tmp
	$(MAKE) distclean	
	$(SILENT) mv Makefile.vars.tmp Makefile.vars
	$(ECHO)
	$(ECHO) "  Adding carmen/data..."
	$(SILENT) tar -cf $(CARMEN_HOME)/../carmen.tar \
		      -C $(CARMEN_HOME)/.. carmen/data
	$(SILENT) tar --delete -f $(CARMEN_HOME)/../carmen.tar \
		      -C $(CARMEN_HOME)/.. carmen/data/CVS
	cd $(CARMEN_HOME)/.. ; \
	if [ -f carmen/data/.cvsignore ] ; then \
		tar --delete -f carmen.tar carmen/data/.cvsignore ; \
	fi
	$(ECHO) "  Adding carmen/doc..."
	$(SILENT) tar -r -f $(CARMEN_HOME)/../carmen.tar \
		      -C $(CARMEN_HOME)/.. carmen/doc
	$(SILENT) tar --delete -f $(CARMEN_HOME)/../carmen.tar \
		      -C $(CARMEN_HOME)/.. carmen/doc/CVS
	$(SILENT) tar --delete -f $(CARMEN_HOME)/../carmen.tar \
		      -C $(CARMEN_HOME)/.. carmen/doc/binaries/CVS
	cd $(CARMEN_HOME)/.. ; \
	if [ -f carmen/doc/.cvsignore ] ; then \
		tar --delete -f carmen.tar carmen/doc/.cvsignore ; \
	fi
	$(ECHO) "  Adding carmen/src..."
	cd $(CARMEN_HOME)/.. ; \
	for i in $(notdir $(wildcard $(CARMEN_HOME)/src/*)) xxxx ; do \
		if [ -f carmen/src/$$i ] ; then \
			tar -f carmen.tar -r carmen/src/$$i ; \
		fi \
	done
	$(SILENT) tar --delete -f $(CARMEN_HOME)/../carmen.tar \
		-C $(CARMEN_HOME)/.. carmen/src/carmen.ini 
	$(ECHO) "  Adding subdirs..."
	cd $(CARMEN_HOME)/.. ; \
	for i in $(TAR_PACKAGE_LIST) xxxx ; do \
		if [ -d carmen/src/$$i ] ; then \
			tar -f carmen.tar -r carmen/src/$$i ; \
			$(MAKE) -C carmen/src/$$i tar-clean ; \
		fi; \
	done
	$(SILENT) tar --delete -f $(CARMEN_HOME)/../carmen.tar \
		-C $(CARMEN_HOME)/.. carmen/src/Makefile.vars
	mv $(CARMEN_HOME)/../carmen.tar $(CARMEN_HOME)
	$(SILENT) gzip -f9 $(CARMEN_HOME)/carmen.tar
	$(ECHO)
	$(ECHO) "  --->  $(CARMEN_HOME)/carmen.tar.gz"
	$(ECHO)

clean:
	$(ECHO)
	$(ECHO) "       *********"
	$(ECHO) "       C L E A N"
	$(ECHO) "       *********"
	$(ECHO)
	$(RECURSE)

relink:
	$(ECHO)
	$(ECHO) "       ***********"
	$(ECHO) "       R E L I N K"
	$(ECHO) "       ***********"
	$(ECHO)
	$(RECURSE)

distclean:
	$(ECHO)
	$(ECHO) "       *****************"
	$(ECHO) "       D I S T C L E A N"
	$(ECHO) "       *****************"
	$(ECHO)
	$(RECURSE)
	$(SILENT) $(RM) $(LIB_DIR)
	$(SILENT) $(RM) $(SHARED_DIR)
	$(SILENT) $(RM) $(INC_DIR)/carmen
	$(SILENT) $(RM) $(INC_DIR)
	$(SILENT) $(RM) $(BIN_DIR)
	$(SILENT) $(RM) $(MAN_DIR)
	$(SILENT) $(RM) $(DOC_DIR)/*.log $(DOC_DIR)/*.aux  $(DOC_DIR)/*.dvi
	$(SILENT) $(RM) $(CARMEN_HOME)/src/Makefile.vars
	$(SILENT) $(RM) $(CARMEN_HOME)/src/*~
	$(SILENT) $(RM) $(CARMEN_HOME)/src/.\#*
	$(SILENT) $(RM) $(CARMEN_HOME)/src/\#*
	$(SILENT) $(RM) $(CARMEN_HOME)/src/core
	$(ECHO) "    ---- Cleaning up bin, include, lib, and src dirs"

debug:
	$(SILENT) $(MAKE) CARMEN_DEBUG=1 total

checkup:
	$(SILENT) $(MAKE) CARMEN_TEST=1 total
	$(ECHO)
	$(ECHO) "       ***********"
	$(ECHO) "       TEST RESULT"
	$(ECHO) "       ***********"
	$(ECHO)
	$(RECURSE)

.PHONY:	  debug
