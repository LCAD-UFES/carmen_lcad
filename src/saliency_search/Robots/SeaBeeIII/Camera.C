#include "Camera.H"
#include <iostream>

Camera::Camera(dc1394camera_t* _camera) :
camera(_camera), error(DC1394_SUCCESS)
{
  if (camera) error = setupCameraDefaults();
  else error = DC1394_CAMERA_NOT_INITIALIZED;  
}

Camera::~Camera() {
  if (camera) {
    dc1394_video_set_transmission(camera, DC1394_OFF);
    dc1394_capture_stop(camera);
  }
}

Image<byte> Camera::getImage() {

  dc1394video_frame_t* frame = NULL;
  dc1394error_t err;
  Image<byte> output;
  int width = 0;
  int height = 0;


  
  err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
  DC1394_WRN(err, "Could not capture frame");
  if (err != DC1394_SUCCESS)  return output;
  
  width = (int)frame->size[0];
  height = (int)frame->size[1];
  output.attach(frame->image, width, height);

	err = dc1394_capture_enqueue(camera, frame);
  DC1394_WRN(err, "Could not free frame");
  return output.deepcopy(); 
}

ImageIceMod::ImageIce Camera::getIceImage() {
  dc1394video_frame_t* frame = NULL;
  dc1394error_t err;
  ImageIceMod::ImageIce output;

  
  err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
  DC1394_WRN(err, "Could not capture frame");
  if (err != DC1394_SUCCESS)  return output;
  //printf("Frames behind: %d\n", frame->frames_behind);
	output = dc2Ice(frame);
	err = dc1394_capture_enqueue(camera, frame);
  DC1394_WRN(err, "Could not free frame");
  return output; 

    /*while (!frame) {
      dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &frame);
      if (!frame) usleep(20000);
    }


    dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &frame);
    int numFrames = frame->frames_behind;

    dc1394_capture_enqueue(camera, frame);
    for (int i = 0; i < numFrames; i++) {
      dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &frame);
      if (frame != NULL) {
        printf("%d\n", frame->frames_behind);
        dc1394_capture_enqueue(camera, frame);
      }
    }*/
}

ImageIceMod::ImageIce Camera::dc2Ice(dc1394video_frame_t* input) {
	ImageIceMod::ImageIce output;
	int height, width, pixSize;
  width = (int)input->size[0];
  height = (int)input->size[1];
  pixSize = (int)input->data_depth / 8;
  //std::cout << input->data_depth << std::endl;
  //printf("Width: %d", width);
  //printf("Height: %d", height);
	//dc1394_get_image_size_from_video_mode(camera, cameraMode, &width, &height);
	output.height = height;
	output.width = width;
	output.pixSize = pixSize;
   	
 	int size = height*width*pixSize;
  output.data.resize(size);
  std::copy(input->image, input->image + size, output.data.begin());
  /*
  if (size == EXPECTED_SIZE*ICE_IMAGE_DEPTH) {
    output.data.resize(size);
    std::copy(input->image, input->image + size, output.data.begin());
  }
  */
	return output;
}

dc1394video_frame_t Camera::getdc1394Frame() {
  dc1394video_frame_t* frame;
  dc1394video_frame_t output; 
  dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
  output = *frame;
  //output = dc2ice(frame);
  dc1394_capture_enqueue(camera, frame);
  return output;
}

dc1394error_t Camera::setupCameraDefaults() {
  dc1394error_t err; 
  err=dc1394_video_set_framerate(camera, DC1394_FRAMERATE_30); //7_5,15, 30, 60
  DC1394_ERR_RTN(err, "Failed to set framerate");
  err=dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_400);
  DC1394_ERR_RTN(err, "Failed to set iso speed");
  err = dc1394_video_set_mode(camera, DC1394_VIDEO_MODE_640x480_MONO8);
  DC1394_ERR_RTN(err, "Failed to set video mode");
  err = dc1394_capture_setup(camera, 5, DC1394_CAPTURE_FLAGS_DEFAULT);
  DC1394_ERR_RTN(err, "Failed to setup capture");
  err = dc1394_video_set_transmission(camera, DC1394_ON);
  DC1394_ERR_RTN(err, "Failed to start camera iso transmission");
  return DC1394_SUCCESS;
}

