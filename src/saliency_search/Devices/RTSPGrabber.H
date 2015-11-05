/*!@file Devices/RTSPGrabber.H video stream grabber froman rtsp source using LiveMedia */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/RTSPGrabber.H $
// $Id: RTSPGrabber.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef RTSPGrabber_H_DEFINED
#define RTSPGrabber_H_DEFINED


#include "Component/ModelParam.H"
#include "Image/Dims.H"
#include "Transport/FrameIstream.H"
#include "Util/Types.H"
#include "Video/VideoFormat.H"
#include "Video/VideoFrame.H"
#include "Media/FfmpegDecoder.H"
#include <pthread.h>

#ifdef INVT_HAVE_LIVEMEDIA
#include <liveMedia/liveMedia.hh>
#include <BasicUsageEnvironment/BasicUsageEnvironment.hh>
#include <UsageEnvironment/UsageEnvironment.hh>
#endif

#ifdef HAVE_FFMPEG_AVCODEC_H
extern "C"
{
#include <ffmpeg/avcodec.h>
#include <ffmpeg/avformat.h>
}
#endif

#ifdef HAVE_LIBAVCODEC_AVCODEC_H
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}
#endif

class VideoFrame;

template <class T> class PixRGB;
template <class T> class Image;

class RTSPGrabber : public FrameIstream
{
public:
  //! Constructor
  RTSPGrabber(OptionManager& mgr,
            const std::string& descrName="RTSP Frame Grabber Driver",
            const std::string& tagName = "RTSPFrameGrabber",
            const ParamFlag flags = USE_MY_VAL,
            unsigned int bufferSize = 200000);

  //! Destructor
  virtual ~RTSPGrabber();

  //! Return the specifications of the next frame to be returned
  virtual GenericFrameSpec peekFrameSpec();

  //! Get the inter-frame time that matches our video mode
  virtual SimTime getNaturalFrameTime() const;

  //! Get the next frame from the frame-grabber
  /*! Returns grabbed frame. This call will block until a frame is
      ready and has been grabbed. */
  virtual GenericFrame readFrame();

  //! Override from FrameIstream; just set the url
  virtual void setConfigInfo(const std::string& url);

  //! get the size of the buffer
  unsigned int getFrameBufferSize() { return itsFrameBufferSize; }

  //! do not use!! used to implement synchronus reads
  char blockingFlag;
  //! set the frame size
  unsigned int itsFrameSize;

  //! the capture thread
  void run();
protected:
  //! Grab raw data (the raw bayer format data)
  /*! Don't call this directly; use readFrame() instead. */
  VideoFrame grabRaw();

  //! get started
  virtual void start1();

  //! get stopped
  virtual void stop2();

  //! Create a decoder
  void createDecoder();


private:
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  void shutdown();

  //! width of grabbed frames
  OModelParam<Dims> itsDims;

  //! grab mode that the hardware should use
  /*! Grabbed frames will internally be converted to Image<
    PixRGB<byte> > whatever that mode is, but playing with it may
    influence image quality, maximum achievable framerate, and amounts
    of CPU doing those conversions to RGB. */
  OModelParam<VideoFormat> itsGrabMode;

  //! determines whether byte swapping is done during conversion to RGB
  OModelParam<bool> itsByteSwap;

  //! for white balance
  OModelParam<int> itsWhiteBalTarR;
  OModelParam<int> itsWhiteBalTarG;
  OModelParam<int> itsWhiteBalTarB;
  OModelParam<int> itsWhiteBalRefR;
  OModelParam<int> itsWhiteBalRefG;
  OModelParam<int> itsWhiteBalRefB;

  //! frames per second
  OModelParam<float> itsFPS;

  //! bit depth
  int itsBitDepth;

  //! the url to stream from
  std::string itsURL;

  unsigned int itsFrameBufferSize;
  unsigned char *itsFrameBuffer; //the frame buffer

#ifdef INVT_HAVE_LIVEMEDIA
  UsageEnvironment* itsEnv;
  Medium* itsClient;
  RTSPClient* itsRTSPClient;
  MediaSession* itsSession;
  FramedSource* itsFramedSource;
#endif

#ifdef INVT_HAVE_AVCODEC
  AVFormatContext* itsFormatContext;
  AVCodecContext* itsCodecContext;
  AVFrame itsPicture;
  int itsGotPicture;
  VideoFrame itsCurrentFrame;
#endif

  pthread_t itsRunThread;
  pthread_mutex_t itsLock;
  bool itsThreadRunning;


};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
