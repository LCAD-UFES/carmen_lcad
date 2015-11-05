# - Config file for the OctoMap package
# (example from http://www.vtk.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file)
# It defines the following variables
#  OCTOVIS_INCLUDE_DIRS - include directories for OctoMap viewer
#  OCTOVIS_LIBRARY_DIRS - library directories for OctoMap viewer 
#  OCTOVIS_LIBRARIES    - libraries to link against
 
set(OCTOVIS_INCLUDE_DIRS /home/lauro/robotics/code/carmen/octomap/octovis/src/extern/QGLViewer /home/lauro/robotics/code/carmen/octomap/octovis/include)
set(OCTOVIS_LIBRARY_DIRS /home/lauro/robotics/code/carmen/octomap/octovis/src/extern/QGLViewer /home/lauro/robotics/code/carmen/octomap/lib)
 
set(OCTOVIS_LIBRARIES QGLViewer optimized;/usr/lib64/libQtOpenGL.so;debug;/usr/lib64/libQtOpenGL_debug.so;optimized;/usr/lib64/libQtGui.so;debug;/usr/lib64/libQtGui_debug.so;optimized;/usr/lib64/libQtXml.so;debug;/usr/lib64/libQtXml_debug.so;optimized;/usr/lib64/libQtCore.so;debug;/usr/lib64/libQtCore_debug.so octovis)
