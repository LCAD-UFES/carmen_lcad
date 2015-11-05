#ifndef CAMERA_H
#define CAMERA_H

#include <dc1394/dc1394.h>
#include "Ice/ImageIce.ice.H"
#include "Image/Image.H"

// Wrapper class that holds a dc1394 camera and provides convenient way to extract
// frames.

class Camera {

public:

  Camera(dc1394camera_t* _camera = NULL);
  ~Camera();
  Image<byte> getImage();
  ImageIceMod::ImageIce getIceImage();
  ImageIceMod::ImageIce dc2Ice(dc1394video_frame_t* input);
  dc1394video_frame_t getdc1394Frame();

private:

  dc1394camera_t* camera;
  dc1394error_t error;
  dc1394error_t setupCameraDefaults();
};
#endif
