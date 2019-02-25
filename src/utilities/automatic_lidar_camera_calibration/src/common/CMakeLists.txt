# SOURCE
add_definitions(-std=gnu99)
set (SRC
  config.c                     config.h    
  fasttrig.c                   fasttrig.h  
  gsl_util.c                   gsl_util.h  
  small_linalg.c               small_linalg.h 
                               so3.h  
  ssc.c                        ssc.h
  timestamp.c                  timestamp.h
  )

add_library (common ${SRC})
target_link_libraries (common
  ${Glib_LIBRARIES}
  ${GSL_LIBRARIES}
  ${OpenCV_LIBS}
  )
include_directories (${Glib_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIR} 
  ${OpenCV_INCLUDE_DIRS}
  ${SRC_DIR} 
  )
