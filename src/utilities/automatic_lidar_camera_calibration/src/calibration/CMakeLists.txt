# SOURCE
set (SRC
  main.cpp 
  Calibration.cpp  Calibration.h
  Calibration_deprecated.cpp 
  )

add_executable (mi-extrinsic-calib ${SRC})
target_link_libraries (mi-extrinsic-calib
  ${Glib_LIBRARIES}
  ${GSL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  common
  )
include_directories (${Glib_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIR} 
  ${OpenCV_INCLUDE_DIRS}
  ${SRC_DIR}/common 
  )
