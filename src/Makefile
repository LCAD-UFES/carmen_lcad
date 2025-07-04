total:	home export phase1 phase2 phase3 phase4

export:	| home

phase1:	| export

phase2:	| phase1

phase3:	| phase2

phase4:	| phase3


include Makefile.conf

PACKAGES = ipc global param_daemon proccontrol argos_lidar_driver\
	   obstacle_avoider simulator_ackerman localize_ackerman robot_ackerman \
	   maptools mapeditor logger dynamic_object_detector \
	   navigator_ackerman joystick pantilt laser gps gps_xyz imu \
	   fused_odometry bumblebee_basic laser_ldmrs viewer_3D \
	   stereo road_finding v_disparity grid_mapping \
	   visual_odometry_package2 stereo_point_cloud xsens xsens_MTi-G \
	   rrt_planner subdir_module_example xyz_pointcloud_lidar velodyne log_filter rddf \
	   download_map parking_assistant navigator_astar \
	   tracker navigator_spline navigator_gui \
	   camera_boxes_to_world navigator_gui2 stereo_mapping \
	   mapper obstacle_distance_mapper map_server car_panel_gui	\
	   v_disparity stereo_velodyne motion_planner udatmo1 behavior_selector \
	   base_ackerman driving_playback ford_escape_hybrid \
	   path_interpolator_planner \
	   odometry_calibration utilities \
	   visual_car_tracking model_predictive_planner utilities \
	   moving_objects_simulator pi_imu pi_camera \
	   road_mapper can_dump imu_viewer \
	   tracker_opentld  joystick_vehicle \
	   velodyne_camera_calibration graphslam # ipc_watcher carmen_map_converter argos_globalpos argos_ros_mapper zed_driver deep_vgl traffic_light robot_ackerman_gui 
	   
	   # hypergraphsclam neural_object_detector3

PACKAGES_LIB = sensors/kinect visual_tracker lane_detector voice_interface localize_neural \
	   ../sharedlib/prob_models ../sharedlib/libcarmodel ../sharedlib/libclustering  moving_objects 

ifndef NO_CPP
PACKAGES += carmenpp
endif

TAR_PACKAGE_LIST = $(filter-out laser, $(PACKAGES)) sensors/laser_new sensors/laser_old

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

RECURSE_LIB = $(SILENT) \
	for i in $(PACKAGES_LIB) xxxx ; do \
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

INSTALL_RECURSE_LIB = $(SILENT) \
	for i in $(PACKAGES_LIB) xxxx ; do \
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
	$(RECURSE_LIB)

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
	$(RECURSE_LIB)

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
	$(INSTALL_RECURSE_LIB)

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
