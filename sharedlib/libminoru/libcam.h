/*
 * Copyright (C) 2009 Giacomo Spigler
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */

#ifndef __LIBCAM__H__
#define __LIBCAM__H__

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <linux/videodev2.h>

struct buffer {
        void *                  start;
        size_t                  length;
};

typedef enum {
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR
} io_method;

class Camera {
private:
  void Open();
  void Close();

  void Init();
  void UnInit();

  void Start();
  void Stop();

  void init_userp(unsigned int buffer_size);
  void init_mmap();
  void init_read(unsigned int buffer_size);

  bool initialised;

  const char *name;  //dev_name

  int width;
  int height;
  int fps;

  int w2;

  unsigned char *data;

  unsigned int bufferIndex;

  io_method io;
  int fd;
  buffer *buffers;
  int n_buffers;

  int mb, Mb, db, mc, Mc, dc, ms, Ms, ds, mh, Mh, dh, msh, Msh, dsh;
  bool ha;

public:
  Camera(const char *name, int w, int h, int fps=30);
  ~Camera();

  int ReadFrame();
  int GrabFrame();
  unsigned char *RetrieveFrame();
  unsigned char *QueryFrame();

  void toIplImage(IplImage *im);


  void StopCam();

  int minBrightness();
  int maxBrightness();
  int defaultBrightness();
  int minContrast();
  int maxContrast();
  int defaultContrast();
  int minSaturation();
  int maxSaturation();
  int defaultSaturation();
  int minHue();
  int maxHue();
  int defaultHue();
  bool isHueAuto();
  int minSharpness();
  int maxSharpness();
  int defaultSharpness();

  int setBrightness(int v);
  int setContrast(int v);
  int setSaturation(int v);
  int setHue(int v);
  int setHueAuto(bool v);
  int setSharpness(int v);



};


class StereoCamera {
	Camera *left;
	Camera *right;
	const char *left_name;  //dev_name
	const char *right_name;  //dev_name
	int width;
	int height;
	int fps;

public:
	StereoCamera(const char *l, const char *r, int w, int h, int fps=30);
	~StereoCamera();

	void StopCam();

	bool GrabFrames();

	void RetrieveLeftImage(IplImage *im);
	void RetrieveRightImage(IplImage *im);
};


#endif
