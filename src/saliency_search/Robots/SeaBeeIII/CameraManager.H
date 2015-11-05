#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

// This class is used to manage the list of cameras supported by dc1394

#include <stdio.h>
#include <stdint.h>
#include <dc1394/dc1394.h>
#include <stdlib.h>
#include <inttypes.h>
#include <vector>

class Camera;

class CameraManager {

public:

  CameraManager();
  ~CameraManager();
  std::vector<uint64_t> getCameraList();
  Camera* getCamera(uint64_t guid);
  bool setupSuccessful(); // returns true if CameraManager was setup successfully

private:

  dc1394error_t error; // error code during setup.  Equals DC1394_SUCCESS if no errors
  dc1394_t* d;  // dc1394 library
  //std::vector<dc1394camera_t*> freeCameraList;
  dc1394camera_list_t* list;

  dc1394error_t initialize();  // Need to call this if no arguments were passed into constructor
};
#endif
