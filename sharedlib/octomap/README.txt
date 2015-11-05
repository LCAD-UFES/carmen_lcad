
Octomap
- A probabilistic, flexible, and compact 3D mapping library for robotic systems.

Authors: K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009-2011.
http://octomap.sourceforge.net/

Further Contributors:
S. Osswald, University of Freiburg
R. Schmitt, University of Freiburg
R. Bogdan Rusu, Willow Garage Inc.

License: 
  * New BSD License (see "octomap/LICENSE.txt")
  * GPL for the viewer "octovis" and related libraries (see "octovis/LICENSE.txt").

OVERVIEW
############################

OctoMap now consists of two separated libraries each in its own subfolder:
octomap, the actual library, and octovis, our visualization libraries and tools.

You can build each separately with CMake by running it from the subdirectories, 
or build octomap and octovis together from this top-level directory. E.g., to
only compile the library, run:

  cd octomap
  mkdir build
  cd build
  cmake ..
  make
  
To compile the complete package, run:
  cd build
  cmake ..
  make
  
Binaries and libs will end up in the directories "bin" and "lib" of the 
top-level directory where you started the build.


See "octomap/README.txt" and "octovis/README.txt" for 
details about compilation and hints on compiling under Windows.

A list of changes is available in "octomap/CHANGELOG.txt"

Getting started

Jump right in and have a look at the example src/octomap/simple.cpp or start the 3D viewer bin/octovis. You will find an example scan to load at src/examples/scan.dat.bz2 (please bunzip2 it first). The Maps section below contains some finished real-world maps to look at. 

Importing Data
Laser Scan Data

Plain-text laser scan data can be imported from the following file format:

NODE x y z roll pitch yaw
x y z
x y z
[...]
NODE x y z roll pitch yaw
x y z
[...]

The keyword NODE is followed by the 6D pose of the laser origin of the 3D scan (roll, pitch, and yaw angles are around the axes x, y, z respectively). After a NODE-line, the laser endpoints of the scan originating at that scan node are listed as 3D points, in the coordinate frame of the scan node. The next NODE keyword specifies the start of a new 3D scan. Lines starting with '#' or empty lines are ignored.

Our tool "log2graph" converts these plain-text log files into a binary scan graph file, which can be directly opened and converted in the viewer "octovis", or converted to an octree map from the command line with "graph2tree".

