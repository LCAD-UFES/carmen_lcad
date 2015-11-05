
After run the stereo_calibration.launch, you can find a file in calibrationdata.tar.gz/ost.txt.
Copy each section to left_camera.ini and right_camera.ini
===== Convert ini file to yaml file ====
rosrun camera_calibration_parsers convert camera4.ini camera4.yaml 
