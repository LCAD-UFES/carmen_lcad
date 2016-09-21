##############################
PROJECT( vanishingPoint )
##############################
cmake_minimum_required (VERSION 2.6)

# Packages
find_package( OpenCV REQUIRED )


# Includes
include_directories(	
	${OPENCV_INCLUDE_DIR}
	${PROJECT_BINARY_DIR}
)

set(SOURCE_FILES 
	main.cpp
	MSAC.cpp
	MSAC.h
	errorNIETO.h
	errorNIETO.cpp
	lmmin.h
	lmmin.cpp
)


# Add executable and target link libraries
ADD_EXECUTABLE( vanishingPoint ${SOURCE_FILES})
TARGET_LINK_LIBRARIES( vanishingPoint ${OpenCV_LIBS})


