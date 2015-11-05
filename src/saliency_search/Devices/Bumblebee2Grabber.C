/*!@file Devices/Bumblebee2Grabber.C BumbleBee2 grabber class based on libdc1394 version 2.x */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Bumblebee2Grabber.C $
// $Id: Bumblebee2Grabber.C 14376 2011-01-11 02:44:34Z kai$
//

#ifndef DEVICES_BUMBLEBEE2GRABBER_C_DEFINED
#define DEVICES_BUMBLEBEE2GRABBER_C_DEFINED


//Just for debuging
#include "Devices/Bumblebee2Grabber.H"

#include "Devices/DeviceOpts.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Transport/TransportOpts.H"
#include "Util/Assert.H"
#include "Util/SimTime.H"
#include "Util/sformat.H"
#include "Video/VideoFrame.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"
#include "Image/ColorOps.H"

#include <unistd.h>

// wait when polling for a frame (in us):
#define IEEE1394WAIT 1

#ifdef HAVE_DC1394V2
// ######################################################################
void* Bumblebee2Grabber_run(void *r0)
{
  Bumblebee2Grabber *r = (Bumblebee2Grabber*)r0;
  r->run(); return NULL;
}
#endif
// ######################################################################


// This code is somewhat inspired from the grab_gray_image.c example
// code provided with the libdc1394 distribution by Gord Peters, and
// from the Coriander source code.

// ######################################################################
Bumblebee2Grabber::Bumblebee2Grabber(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName) :
  FrameIstream(mgr, descrName, tagName),
  // NOTE that contrary to the common case, we give USE_MY_VAL here
  // when we construct the OModelParam objects; that means that we
  // push our values into the ModelManager as the new default values,
  // rather than having our param take its value from the
  // ModelManager's default
  itsShowInputDetails(&OPT_ShowInputDetails, this),
  itsDevName(&OPT_FrameGrabberDevice, this, "/dev/video1394/0", USE_MY_VAL), // device
  itsChannel(&OPT_FrameGrabberChannel, this, 0, USE_MY_VAL), // first firewire card
  itsSubChan(&OPT_FrameGrabberSubChan, this, 0, USE_MY_VAL),
  itsDims(&OPT_FrameGrabberDims, this, Dims(640, 240), USE_MY_VAL),
  itsGrabMode(&OPT_FrameGrabberMode, this, VideoFormat::VIDFMT_GREY, USE_MY_VAL), // grab mode
  itsByteSwap(&OPT_FrameGrabberByteSwap, this, false, USE_MY_VAL), // no byte-swapping
  itsFPS(&OPT_FrameGrabberFPS, this, 30.0, USE_MY_VAL),
  itsNbuf(&OPT_FrameGrabberNbuf, this, 4, USE_MY_VAL), // number of grab buffers
  itsBrightness(&OPT_FrameGrabberBrightness, this, 32768, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsHue(&OPT_FrameGrabberHue, this, 32768, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsSaturation(&OPT_FrameGrabberSaturation, this, 90, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsExposure(&OPT_FrameGrabberExposure, this, 511, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsSharpness(&OPT_FrameGrabberSharpness, this, 80, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsWhiteBalBU(&OPT_FrameGrabberWhiteBalBU, this, 95, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsWhiteBalRV(&OPT_FrameGrabberWhiteBalRV, this, 87, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsGamma(&OPT_FrameGrabberGamma, this, 1, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsShutter(&OPT_FrameGrabberShutter, this, 6, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsGain(&OPT_FrameGrabberGain, this, 87, USE_MY_VAL | ALLOW_ONLINE_CHANGES)
#ifdef HAVE_DC1394V2
  , itsDC1394(NULL)
  , itsCam(NULL)
#endif
#ifdef HAVE_PCL
  ,itsViewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
  ,itsCloud(new pcl::PointCloud<pcl::PointXYZ>)
#endif
  ,itsImageReady(false)

{
  itsFrameCount = 0;
  pthread_mutex_init(&itsLock, NULL);
#ifdef HAVE_PCL

  itsViewer->setBackgroundColor (0, 0, 0);
  itsViewer->addPointCloud<pcl::PointXYZ>(itsCloud,"simple cloud");
  itsViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "simple cloud");
  itsViewer->addCoordinateSystem (1.0);
  itsViewer->initCameraParameters ();
#endif
}

// ######################################################################
void Bumblebee2Grabber::start1()
{
#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use Bumblebee2Grabber");
#else
  ASSERT(itsDC1394 == 0);

  itsDC1394 = dc1394_new();
  dc1394error_t		err;
	dc1394camera_list_t * list;

   // Enumerate cameras connected to the PC
   err = dc1394_camera_enumerate (itsDC1394, &list);

   if ( err != DC1394_SUCCESS )
   {
		 fprintf( stderr, "Unable to look for cameras\n\n"
				 "Please check \n"
				 "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' "
				 "are loaded \n"
				 "  - if you have read/write access to /dev/raw1394\n\n");
		 LFATAL("dc1394_new() failed");
   }

    if (list->num == 0)
    {
        LFATAL( "No cameras found!");
    }
		LINFO( "There were %d camera(s) found attached to your PC", list->num  );

		unsigned int nThisCam;
		// Identify cameras. Use the first stereo camera that is found
		for ( nThisCam = 0; nThisCam < list->num; nThisCam++ )
		{
			itsCam = dc1394_camera_new(itsDC1394, list->ids[nThisCam].guid);

			if(!itsCam)
			{
				LINFO("Failed to initialize camera with guid %llx", (long long unsigned int)list->ids[nThisCam].guid);
				continue;
			}

			LINFO( "Camera %d model = '%s'\n", nThisCam, itsCam->model );

			const char* bb2 = "Bumblebee2";
			if (!strncmp(itsCam->model,bb2,strlen(bb2)))
			{
				LINFO( "Using this camera\n" );
				break;
			}
			dc1394_camera_free(itsCam);
		}
		if ( nThisCam == list->num )
		{
			LFATAL( "No stereo cameras were detected\n" );
		}

   // Free memory used by the camera list
   dc1394_camera_free_list (list);


#ifdef HAVE_BUMBLEBEE2
   // query information about this stereo camera
   err = queryStereoCamera( itsCam, &itsStereoCamera );
   if ( err != DC1394_SUCCESS )
   {
      fprintf( stderr, "Cannot query all information from camera\n" );
      cleanup_and_exit( itsCam );
   }

   // set the capture mode
   printf( "Setting stereo video capture mode\n" );
   err = setStereoVideoCapture( &itsStereoCamera );
   if ( err != DC1394_SUCCESS )
   {
      fprintf( stderr, "Could not set up video capture mode\n" );
      cleanup_and_exit( itsStereoCamera.camera );
   }
	err = startTransmission( &itsStereoCamera );
   if ( err != DC1394_SUCCESS )
   {
      fprintf( stderr, "Unable to start camera iso transmission\n" );
      cleanup_and_exit( itsStereoCamera.camera );
   }

		int w = itsStereoCamera.nCols;
		int h = itsStereoCamera.nRows;

    itsDims.setVal(Dims(320,240));
    Image<byte> frame(itsDims.getVal(), NO_INIT);
    itsCurrentFrame = VideoFrame(frame);

    itsCurrentRectMap = Image<byte>(itsDims.getVal(), ZEROS);
    itsCurrentDisparityMap =Image<uint16>(itsDims.getVal(), ZEROS);
    itsLeftImage = Image<byte>(itsDims.getVal(),ZEROS);
    itsRightImage = Image<byte>(itsDims.getVal(),ZEROS);

    //triclopsInpus Init
    itsTriclopsInput.inputType 	= TriInp_RGB;
    itsTriclopsInput.nrows	= h;
    itsTriclopsInput.ncols	= w;
    itsTriclopsInput.rowinc	= w;

    //TriclopsError e;
    //e = getTriclopsContextFromCamera( &itsStereoCamera, &itsTriclops );

    //if ( e != TriclopsErrorOk )
    //{
    //  LINFO("Can't get context from camera\n" );
    //  cleanup_and_exit( itsCam );
    //}
    // make sure we are in subpixel mode
    //e = triclopsSetSubpixelInterpolation( itsTriclops, 1 );
    //e = triclopsSetDisparityMapping(itsTriclops,0,60);
    //e = triclopsSetDisparityMappingOn(itsTriclops,1);
    //e = triclopsSetUniquenessValidationMapping(itsTriclops,0);
    //e = triclopsSetTextureValidationMapping(itsTriclops,0);

  // start the capture thread
  // start thread for run():

  pthread_create(&itsRunThread, NULL, &Bumblebee2Grabber_run, (void *)this);
#endif // HAVE_BUMBLEBEE2
#endif // HAVE_DC1394V2
}

// ######################################################################
void Bumblebee2Grabber::stop2()
{
#ifndef HAVE_DC1394V2
  // don't LFATAL() in stop() since it may be called in a destructor chain
  LERROR("you must have libdc1394 version 2.x in order to use Bumblebee2Grabber");
#else
#ifdef HAVE_BUMBLEBEE2
	cleanup_and_exit(itsStereoCamera.camera);
#endif // HAVE_BUMBLEBEE2
#endif // HAVE_DC1394V2
}
//=============================================================================
// cleanup_and_exit()
// This is called when the program exits and destroys the existing connections
// to the 1394 drivers
#ifdef HAVE_DC1394V2
void Bumblebee2Grabber::cleanup_and_exit( dc1394camera_t* camera )
{
   dc1394_capture_stop( camera );
   dc1394_video_set_transmission( camera, DC1394_OFF );
   dc1394_camera_free( camera );
	 LFATAL("Clean up and exit");
}
#endif // HAVE_DC1394V2


// ######################################################################
Bumblebee2Grabber::~Bumblebee2Grabber()
{ 
#ifdef HAVE_DC1394V2
#ifdef HAVE_BUMBLEBEE2
	cleanup_and_exit(itsStereoCamera.camera);
#endif // HAVE_BUMBLEBEE2
#endif // HAVE_DC1394V2
 }

// ######################################################################
GenericFrameSpec Bumblebee2Grabber::peekFrameSpec()
{
  GenericFrameSpec result;


  result.nativeType = GenericFrame::RGBD;
  result.videoFormat = VIDFMT_RGB24;
  result.videoByteSwap = false;
  result.dims = itsDims.getVal();
  result.floatFlags = 0;

//  result.nativeType = GenericFrame::VIDEO;
//  result.videoFormat = itsGrabMode.getVal();
//  result.videoByteSwap = itsByteSwap.getVal();
//  result.dims = itsDims.getVal();
//  result.floatFlags = 0;

  return result;
}

// ######################################################################
SimTime Bumblebee2Grabber::getNaturalFrameTime() const
{
  return SimTime::HERTZ(itsFPS.getVal());
}

// ######################################################################
GenericFrame Bumblebee2Grabber::readFrame()
{
  //return GenericFrame(this->grabRaw());

#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use Bumblebee2Grabber");
  /* can't happen */ return GenericFrame();
#else
#ifdef HAVE_BUMBLEBEE2
    makeStereo();
    return GenericFrame(toRGB(itsCurrentRectMap),itsCurrentDisparityMap);
  //return GenericFrame(grabRaw());
#else
  LFATAL("you must have bumblebee2 driver installed");
  /* can't happen */ return GenericFrame();
#endif // HAVE_BUMBLEBEE2


#endif // HAVE_DC1394V2

}

// ######################################################################
VideoFrame Bumblebee2Grabber::grabRaw()
{
#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use Bumblebee2Grabber");
  /* can't happen */ return VideoFrame();
#else
   VideoFrame result;
   pthread_mutex_lock(&itsLock);
   result = itsCurrentFrame;
   pthread_mutex_unlock(&itsLock);

  return result;

#endif // HAVE_DC1394V2
}

//void Bumblebee2Grabber::grabFrame(Image<byte> left,Image<> )
// ######################################################################
void Bumblebee2Grabber::run()
{
#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use Bumblebee2Grabber");
#else
#ifdef HAVE_BUMBLEBEE2

  itsThreadRunning = true;
  //Grab the frame
  int w = itsStereoCamera.nCols;
  int h = itsStereoCamera.nRows;
  int nb= itsStereoCamera.nBytesPerPixel;
  itsBufferSize = w*h*nb;

  // size of capture buffer
  LINFO("Image Size: width %d height %d Byte Per Pixel %d",w,h,nb);

  while(itsThreadRunning)
  {
	 // get the images from the capture buffer and do all required processing
	 // note: produces a TriclopsInput that can be used for stereo processing
   dc1394error_t err;
   // RC7
   dc1394video_frame_t* frame = NULL;
    //block untill data becomes available
   err = dc1394_capture_dequeue( itsStereoCamera.camera,
       DC1394_CAPTURE_POLICY_WAIT,
       &frame );

   // allocate a buffer to hold the de-interleaved images
   unsigned char* intBuf= new unsigned char[ itsBufferSize ];
   unsigned char* grabBuf= frame->image;

	 unsigned char* right;
	 unsigned char* left;
	 // de-interlace the 16 bit data into 2 mono images
	 dc1394_deinterlace_stereo( grabBuf, intBuf, w, 2*h );
	 right = intBuf;
	 left  = intBuf+ w*h;

   // return buffer for use
   dc1394_capture_enqueue( itsStereoCamera.camera, frame );
   Dims dim(w,h);

   Image<byte> leftImg(left,dim);
   Image<byte> rightImg(right,dim);

   itsTriclopsInput.u.rgb.red   = right;
   itsTriclopsInput.u.rgb.green = left;
   itsTriclopsInput.u.rgb.blue  = left;
    //convert the frame
   if((err == DC1394_SUCCESS)&&frame!=NULL)
    {
      pthread_mutex_lock(&itsLock);
      itsLeftImage = leftImg;
      itsRightImage = rightImg;
      itsCurrentFrame = VideoFrame(rescale(concatX(leftImg,rightImg),itsDims.getVal()));
      itsImageReady = true;
      pthread_mutex_unlock(&itsLock);
    }

   delete[] intBuf;
   usleep(10000);
  }

  pthread_exit(0);

#endif // HAVE_BUMBLEBEE2
#endif // HAVE_DC1394V2
}
// ######################################################################
void Bumblebee2Grabber::makeRect()
{
#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use Bumblebee2Grabber");
#else
#ifdef HAVE_BUMBLEBEE2
   TriclopsInput  input; 
   TriclopsError e;
   
   TriclopsContext triclops;
    e = getTriclopsContextFromCamera( &itsStereoCamera, &triclops );
    e = triclopsSetSubpixelInterpolation( triclops, 1 );
		int w = itsStereoCamera.nCols;
		int h = itsStereoCamera.nRows;

    //triclopsInpus Init
    input.inputType 	= TriInp_RGB;
    input.nrows	  = h;
    input.ncols	  = w;
    input.rowinc	= w;
   unsigned char* intBuf= new unsigned char[ itsBufferSize ];

   //copy the left and right image
   pthread_mutex_lock(&itsLock);
    memcpy(intBuf,itsTriclopsInput.u.rgb.red,sizeof(byte)*itsBufferSize);
   pthread_mutex_unlock(&itsLock);

   input.u.rgb.red   = intBuf;
   input.u.rgb.green = intBuf+w*h;
   input.u.rgb.blue  = intBuf+w*h;

   e = triclopsRectify( triclops, &input);
   if ( e != TriclopsErrorOk )
   {
      LINFO("triclopsRectify failed!" );
      triclopsDestroyContext( triclops );
      cleanup_and_exit( itsCam );
   }
   // get and save the rectified and disparity images
   TriclopsImage image;
   triclopsGetImage( triclops, TriImg_RECTIFIED, TriCam_REFERENCE, &image );
   Image<byte> rectImg(image.data,Dims(image.ncols,image.nrows));
   itsCurrentRectMap = rectImg;

   triclopsDestroyContext( triclops );
   delete[] intBuf;

#endif // HAVE_BUMBLEBEE2
#endif // HAVE_DC1394V2
}
// ######################################################################
void Bumblebee2Grabber::makeStereo()
{
#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use Bumblebee2Grabber");
#else
#ifdef HAVE_BUMBLEBEE2
   TriclopsInput  input; 
   TriclopsError e;
   
   TriclopsContext triclops;
    e = getTriclopsContextFromCamera( &itsStereoCamera, &triclops );
    e = triclopsSetSubpixelInterpolation( triclops, 1 );
		int w = itsStereoCamera.nCols;
		int h = itsStereoCamera.nRows;

    //triclopsInpus Init
    input.inputType 	= TriInp_RGB;
    input.nrows	  = h;
    input.ncols	  = w;
    input.rowinc	= w;
   unsigned char* intBuf= new unsigned char[ itsBufferSize ];

   //copy the left and right image
   pthread_mutex_lock(&itsLock);
    memcpy(intBuf,itsTriclopsInput.u.rgb.red,sizeof(byte)*itsBufferSize);
   pthread_mutex_unlock(&itsLock);

   input.u.rgb.red   = intBuf;
   input.u.rgb.green = intBuf+w*h;
   input.u.rgb.blue  = intBuf+w*h;

   e = triclopsRectify( triclops, &input);
   e = triclopsStereo( triclops );
   if ( e != TriclopsErrorOk )
   {
      LINFO("triclopsStereo failed!" );
      triclopsDestroyContext( triclops );
      cleanup_and_exit( itsCam );
   }
   // get and save the disparity images
   TriclopsImage image;
   triclopsGetImage( triclops, TriImg_RECTIFIED, TriCam_REFERENCE, &image );
   Image<byte> rectImg(image.data,Dims(image.ncols,image.nrows));
   itsCurrentRectMap = rectImg;

   TriclopsImage16 image16;
   triclopsGetImage16( triclops, TriImg16_DISPARITY, TriCam_REFERENCE, &image16 );
   Image<uint16> disparityImg(image16.data,Dims(image16.ncols,image16.nrows));
   itsCurrentDisparityMap = disparityImg;

   triclopsDestroyContext( triclops );
   delete[] intBuf;
#endif // HAVE_BUMBLEBEE2
#endif // HAVE_DC1394V2
}
// ######################################################################
void Bumblebee2Grabber::toPointCloud(Image<uint16> disparity)
{
#ifdef HAVE_BUMBLEBEE2 
  int w = disparity.getWidth();
  int h = disparity.getHeight();
  float x,y,z;
#ifdef HAVE_PCL
  itsCloud->clear();
  //itsCloud(new pcl::PointCloud<pcl::PointXYZI>());
  //pcl::PointCloud<pcl::PointXYZI>cloud;//(new pcl::PointCloud<pcl::PointXYZI>);
#endif
  for(int i = 0 ;i < h; i++)
  {
    for(int j = 0;j < w ;j++)
    {
    uint16 pixel = disparity.getVal(j,i);
    //float lum = itsCurrentRectMap.getVal(j,i);
    //filter invalid points
    if(pixel < 0xFF00)
    {
      triclopsRCD16ToXYZ(itsTriclops,i,j,pixel,&x,&y,&z);
#ifdef HAVE_PCL

      pcl::PointXYZ pt(x,y,z);
      //pt.x = x;pt.y = y; pt.z = z;
      itsCloud->push_back(pt);
      //LINFO("ijd(%d %d %f) -> (%f,%f,%f) %f",i,j,(float)pixel,x,y,z,lum);
#else
#endif
    }
    }
  }
#ifdef HAVE_PCL


  itsViewer->updatePointCloud(itsCloud,"simple cloud"); 

  // save PCD files
  //  std::string fname = sformat("stereoPointCloud_%06d.pcd",itsFrameCount++);
  //  pcl::io::savePCDFileASCII( fname, *itsCloud ); 
#endif

#endif // HAVE_BUMBLEBEE2
}
// ######################################################################
void Bumblebee2Grabber::paramChanged(ModelParamBase* const param,
                                  const bool valueChanged,
                                  ParamClient::ChangeStatus* status)
{
#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use Bumblebee2Grabber");
#else

  FrameIstream::paramChanged(param, valueChanged, status);

  // just handle online changes here (i.e. changes that come while we
  // are already started); if the change happens before start() then
  // we will pick up the proper dc1394 settings in start() so we don't
  // need to handle the change here

#define HANDLE_PARAM(FVAL, MODELPARAM, FNAME)           \
      if (valueChanged && param == &MODELPARAM)         \
        {                                               \
          LDEBUG("online change of " FNAME " from %s",  \
                 MODELPARAM.getName().c_str());         \
          if (dc1394_feature_set_value                  \
              (itsCam, DC1394_FEATURE_ ## FVAL,         \
               MODELPARAM.getVal())                     \
              != DC1394_SUCCESS)                        \
            {                                           \
              *status = ParamClient::CHANGE_REJECTED;   \
              LERROR("Unable to set " FNAME);           \
            }                                           \
        }

  HANDLE_PARAM(BRIGHTNESS, itsBrightness, "brightness");
  HANDLE_PARAM(EXPOSURE, itsExposure, "exposure");
  HANDLE_PARAM(SHARPNESS, itsSharpness, "sharpness");
  HANDLE_PARAM(HUE, itsHue, "hue");
  HANDLE_PARAM(SATURATION, itsSaturation, "saturation");
  HANDLE_PARAM(GAMMA, itsGamma, "gamma");
  HANDLE_PARAM(SHUTTER, itsShutter, "shutter");
  HANDLE_PARAM(GAIN, itsGain, "gain");

#undef HANDLE_PARAM

  if (valueChanged
      && (param == &itsWhiteBalBU || param == &itsWhiteBalRV))
    {
      LDEBUG("online change of white balance from %s",
             param->getName().c_str());

      if (dc1394_feature_whitebalance_set_value
          (itsCam, itsWhiteBalBU.getVal(), itsWhiteBalRV.getVal())
          != DC1394_SUCCESS)
        {
          *status = ParamClient::CHANGE_REJECTED;
          LERROR("Unable to set white balance");
        }
    }

#endif // HAVE_DC1394V2
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // DEVICES_BUMBLEBEE2GRABBER_C_DEFINED
