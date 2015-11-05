/*!@file Devices/OpenNIGrabber.H Definition and access functions for OpenNI grabber */

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
// Primary maintainer for this file: Eric Hu <ehu@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/OpenNIGrabber.H $
// $Id: OpenNIGrabber.H 14046 2010-09-25 05:58:24Z itti $
//

#ifndef OPENNIGRABBER_H_DEFINED
#define OPENNIGRABBER_H_DEFINED

#ifdef INVT_HAVE_OPENNI
// The following is a fix for OpenNI with c++0x
// c++0x does not define "linux" as a preprocessor directive
#ifdef __linux__
#ifndef linux
#define linux 1
#endif // linux
#endif // __linux__
//OPENNI Libraries
#pragma GCC diagnostic ignored "-Wreorder"
#include <XnOS.h>
#include <XnCppWrapper.h>
#pragma GCC diagnostic error "-Wreorder"

#include "Component/ModelParam.H"
#include "Transport/FrameIstream.H"
#include "Raster/GenericFrame.H"
//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../Data/SamplesConfig.xml"

#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480



//! Definition and access functions for Kinect frame grabber
class OpenNIGrabber : public FrameIstream
{
public:
  //! Constructor
  OpenNIGrabber(OptionManager& mgr,
                const std::string& descrName = "OpenNI Frame Grabber Driver",
                const std::string& tagName = "OpenNIFrameGrabber",
                const ParamFlag flags = USE_MY_VAL);

  //! Destructor
  virtual ~OpenNIGrabber();

  /// Install a FrameListener
  /** We call the listener's onRawFrame() inside each readFrame(). */
  virtual void setListener(rutz::shared_ptr<FrameListener> listener);

  //! Return the specifications of the next frame to be returned
  virtual GenericFrameSpec peekFrameSpec();

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

  xn::Context* getContext();

protected:
  //! get started
  virtual void start1();

  //! get stopped
  virtual void stop2();


private:
  OModelParam<Dims> itsDims; //!< Dims of grabbed frames
  rutz::shared_ptr<FrameListener> itsListener;

  //OPENNI related variables
  XnUInt16* NI_DMap;
  unsigned int NI_DMapX;
  unsigned int NI_DMapY;

  xn::Context NI_context;
  xn::DepthGenerator NI_depth;
  xn::ImageGenerator NI_image;
  xn::DepthMetaData NI_depthMD;
  xn::ImageMetaData NI_imageMD;

  Image<PixRGB<byte> > itsColorImage;
  Image<uint16> itsDepthImage;
};

#endif

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
