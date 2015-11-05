#include "CameraManager.H"
#include "Camera.H"

CameraManager::CameraManager() : error(DC1394_SUCCESS), d(NULL) {
  error = initialize();
}

CameraManager::~CameraManager() {
  /*for (std::vector<uint64_t>::size_type i = 0; i < freeCameraList.size(); i++) {
    dc1394_capture_stop(freeCameraList[i]);
    dc1394_camera_free(freeCameraList[i]);
  }*/
  if (d) dc1394_free(d);
}

dc1394error_t CameraManager::initialize() {
  dc1394error_t err;

  d = dc1394_new();
  if (!d) {
    err = DC1394_FAILURE;
    DC1394_ERR_RTN(err, "Failed to create new dc1394");
  }

  err = dc1394_camera_enumerate(d, &list);
  DC1394_ERR_RTN(err, "Failed to enumerate cameras");

  if (list->num == 0) {
    err = DC1394_CAMERA_NOT_INITIALIZED;
    DC1394_ERR_RTN(err, "No cameras found");
  }

  return DC1394_SUCCESS;
}

std::vector<uint64_t> CameraManager::getCameraList() {
  std::vector<uint64_t> tempList;
  for (uint32_t i = 0; i < list->num; i++) {
    tempList.push_back(list->ids[i].guid);
  }
  return tempList;
}

Camera* CameraManager::getCamera(uint64_t guid) {
  dc1394camera_t* camera1394 = NULL;
  dc1394error_t err;
  if (!setupSuccessful())  return NULL;
  for (uint32_t i = 0; i < list->num; i++) {
    if (list->ids[i].guid == guid) {
      
      camera1394 = dc1394_camera_new(d, list->ids[i].guid);
      if (!camera1394) {
        err = DC1394_CAMERA_NOT_INITIALIZED;
        DC1394_WRN(err, "Failed to initialize camera");
      } else {
        err = dc1394_reset_bus(camera1394);
        DC1394_WRN(err, "Failed to reset bus for camera");
      }
      break;
    }
  }

  if (!camera1394)  return NULL;
  else              return new Camera(camera1394);
}

bool CameraManager::setupSuccessful() {
  return error == DC1394_SUCCESS;
}

