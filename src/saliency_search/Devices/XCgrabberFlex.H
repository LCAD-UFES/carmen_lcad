
/*!@file Devices/XCgrabberFlex.H Definition and access functions for
  Silicon Imaging HD Camera grabber */

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
// Primary maintainer for this file: Farhan Baluch <fbaluch@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/XCgrabberFlex.H $
// $Id: XCgrabberFlex.H 12965 2010-03-06 03:23:30Z irock $
//

#ifndef XCGRABBERFLEX_H_DEFINED
#define XCGRABBERFLEX_H_DEFINED

#ifdef HAVE_XCLIB
// xclib defines a bunch of types which conflicts with other packages;
// namely: uint64 which we define in Util/Types.H, and, more
// importantly, float4 which is defined by CUDA includes. So we just
// resort to using it in a new xclib namespace. That means that all
// the function calls and uses of xclib datatypes now must use the
// xclib:: prefix:
namespace xclib {
#include <xclib/xcliball.h>
}
#endif


#include "Component/ModelParam.H"
#include "Image/Dims.H"
#include "Transport/FrameIstream.H"
#include "Util/Types.H"
#include "Video/VideoFormat.H"

#include <pthread.h>

class VideoFrame;
template <class T> class PixRGB;
template <class T> class Image;

class XCgrabberFlex : public FrameIstream
{
public:
  //! Constructor
  XCgrabberFlex(OptionManager& mgr,
            const std::string& descrName="XC Frame Grabber Driver",
            const std::string& tagName = "XCFrameGrabber",
            const ParamFlag flags = USE_MY_VAL);

  //! Destructor
  virtual ~XCgrabberFlex();

  //! Return the specifications of the next frame to be returned
  virtual GenericFrameSpec peekFrameSpec();

  //! Get the inter-frame time that matches our video mode
  virtual SimTime getNaturalFrameTime() const;

  //! White balance
  void WhiteBalance();

  //! Get the next frame from the frame-grabber
  /*! Returns grabbed frame. This call will block until a frame is
      ready and has been grabbed.

      Beware that the integrity of the GenericFrame object may not
      last "very long"; basically, try to be finished using the
      GenericFrame object before you attempt to grab the next frame in
      the stream. If you need it for longer than that, then you should
       use GenericFrame::deepCopyOf() to make a copy of the frame that
     can be safely held indefinitely. */
  virtual GenericFrame readFrame();

#ifdef HAVE_XCLIB
  xclib::pxbuffer_t getCurrBufID();
#endif

protected:
  //! Grab raw data (the raw bayer format data)
  /*! Don't call this directly; use readFrame() instead. */
  VideoFrame grabRaw();

  //! get started
  virtual void start1();

  //! get stopped
  virtual void stop2();

private:
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

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


  //! gamma  - highly dependent on your driver
  OModelParam<float> itsGamma;

  //! frames per second
  OModelParam<float> itsFPS;

  //! bit depth
  int itsBitDepth;

  pthread_mutex_t qmutex_buf;

// check whether the configure script found XClib and XC camera; if not, then
// just stub out the core of the XCgrabber class and issue
// LFATAL()s if somebody actually tries to use it
#ifdef HAVE_XCLIB
  OModelParam<std::string> itsFormatFile;
  bool itsCameraOk;           // the camera is initialized and transmitting
  xclib::pxbuffer_t itsLastBuf;
  byte* itsImgBuf;
  struct xclib::xclibs itsXclib;
  struct xclib::pxvidstate *itsStatep;
  int itsStateid;
#endif // HAVE_XCLIB

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
