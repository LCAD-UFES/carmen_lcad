20200501 carmen_lcad/doc/README_HISTORY.txt

Na data acima sera criado o branch history0 que contem todos os modulos do carmen_lcad/src ateh a mencionada data. Sao eles:

arduino_can/                     logtools/                    neural_slam/                            road_network_generator/  		      unv_camera_ipc322/
base/                            fused_odometry/              mapeditor/                              obstacle_avoider/                       robosense/               utilities/
base_ackerman/                   fused_odometry_kalman/       mapper/                                 obstacle_avoider_multi_height/          robot_ackerman/          v_disparity/
baseline_extension/              global/                      mapper2/                                obstacle_distance_mapper/               robot_ackerman_gui/      vehicle_tracker/
behavior_selector/               gps/                         mapper3/                                obstacle_distance_mapper_multi_height/  robotgui/                velodyne/
behavior_selector2/              gps_xyz/                     mapper_datmo/                           occupancy_grid/                         velodyne_camera_calibration/
behavior_selector_multi_height/  graphslam/                   mapper_multi_height/                    odometry_calibration/                   rrt_planner/             velodyne_icp/
bumblebee_basic/                 grid_mapping/                mapper_multi_height2/                   odometry_calibration_nogps/             saliency_search/         velodyne_odometry/
call_iara_app/                   hypergraphsclam/             map_server/                             ompl_planner/                           segmap/                  vergence/
camera/                          imu/                         map_server_multi_height/                online_path_planner/                    semantic_segmentation/   viewer_3D/
camera_boxes_to_world/           imu_viewer/                  maptools/                               pantilt/                                sensors/                 virtual_scan/
camera_ip_driver/                ipc/                         model_predictive_planner/               param_daemon/                           sharedlib_example/       virtual_scan2/
camera_viewer/                   java/                        model_predictive_planner2/              parking_assistant/                      shared_memory_test/      visual_car_tracking/
can_dump/                        joystick/                    model_predictive_planner_multi_height/  path_interpolator_planner/              simple_2d_mapper/        visual_graphslam/
car_lai/                         kinect/                      motion_planner/                         path_planner/                           simple_module_example/   visual_memory/
carmen_log_to_dataset/           kitti2carmen/                moving_objects/                         path_planner2/                          simulator_ackerman/      visual_odometry/
carmenpp/                        landmark_localization/       moving_objects2/                        path_planner_astar/                     simulator_ackerman2/     visual_odometry_package2/
carmen_ros_communication/        lane_analysis/               moving_objects3/                        path_planner_lib/                       skeleton_package/        visual_search/
carmen_ros_interface/            lane_detection/              moving_objects_simulator/               pi_camera/                              slam/                    visual_search_thin/
car_panel_gui/                   lane_detector/               moving_obstacle_detector/               pi_imu/                                 slam6d/                  visual_tracker/
clean_map/                       laser/                       multicentral_bridge/                    polar_slam/                             slam_icp/                voice_interface/
cvis/                            laser_calibration/           mvog_package/                           post_slam/                              sound/                   voice_recognition/
cyton_arm/                       laser_ldmrs/                 navigator_ackerman/                     probabilistic_robotics_course/          stehs_planner/           volume_measurement/
download_map/                    laser-ldmrs/                 navigator_astar/                        proccontrol/                            stereo/                  voslam/
download_map_streetview/         laser-new/                   navigator_gui/                          python/                                 stereo_mapping/          web_cam/
download_map_streetview_ofi/     laserviewer/                 navigator_gui2/                         qt_gui/                                 stereo_point_cloud/      xsens/
dqn/                             laslam/                      navigator_spline/                       rddf/                                   stereo_velodyne/         xsens_MTi-G/
driving_playback/                latency_estimation_package/  neural_global_localizer/                rddf2/                                  subdir_module_example/   xsensOdometer/
dynamic_object_detector/         legacy500_socket_interface/  neural_localizer/                       rddf_from_image/                        tracker/                 yolo_save_prediction/
dynamic_window/                  lidars/                      neural_mapper/                          rddf_graph/                             tracker_opentld/         zed_camera/
ecoTech/                         line_follower/               neural_object_detector/                 realsense/                              traffic_light/           zed_camera2/
ekf_odometry/                    localize_ackerman/           neural_object_detector2/                rl_motion_planner/                      traffic_light_yolo/      zed_opencv/
examples/                        localize_neural/             neural_object_detector3/                road_finding/                           udatmo1/
facial_greeting/                 log_filter/                  neural_object_detector_fovea/           road_map_path_planning/                 udatmo2/
ford_escape_hybrid/              logger/                      neural_object_detector_point_cloud/     road_mapper/                            ultrasonic/


Este branch possuirah, entao, todos os modulos acima. Para tornar mais facil o trabalho de desenvolvimento, o branch origin/master vai deixar de conter os seguintes modulos:

behavior_selector2
behavior_selector_multi_height
mapper2
mapper3
mapper_multi_height
mapper_multi_height2
map_server_multi_height
model_predictive_planner2
model_predictive_planner_multi_height
moving_objects2
moving_objects3
navigator_gui
navigator_ackerman
navigator_astar
neural_object_detector2
neural_object_detector3
obstacle_avoider_multi_height
obstacle_distance_mapper_multi_height
path_planner2
rddf2
simulator_ackerman2
udatmo2

Caso alguem esteja trabalhando em algum destes modulos acima favor se manifestar e defender sua permanencia no origin/master. Lembro que todos os modulos ainda poderao ser
acessados no branch history0.

Abraccos,

Alberto.
