/*!@file Channels/ForegroundDetectionChannel.C Wrapper around OpenCV implementation
 * of "Foreground Object Detection from Videos Containing Complex Background" by Huang,
 * et. al. in ACMMM 2003*/
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
// Primary maintainer for this file: Randolph Voorhies
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ForegroundDetectionChannel.C $
// $Id: ForegroundDetectionChannel.C 14605 2011-03-15 02:25:06Z dparks $
//


#include "Channels/ForegroundDetectionChannel.H"

#include "Image/DrawOps.H"
#include "Image/Kernels.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Channels/ChannelOpts.H"
#include "Component/ModelOptionDef.H"


ForegroundDetectionChannel::ForegroundDetectionChannel(OptionManager& mgr) :
  SingleChannel(mgr, "ForegroundDetectionChannel", "ForegroundDetectionChannel", FOREGROUND, rutz::shared_ptr<PyrBuilder<float> >()),
  itsMap(),
  itsLevelSpec(&OPT_LevelSpec, this)
{
#ifdef HAVE_OPENCV
  itsStatModel_cv = NULL;
#else
  LFATAL("OpenCV is needed for Foreground Detection Channel!");
#endif
}

// ######################################################################
ForegroundDetectionChannel::~ForegroundDetectionChannel()
{  }

// ######################################################################
bool ForegroundDetectionChannel::outputAvailable() const
{ return itsMap.initialized(); }

// ######################################################################
uint ForegroundDetectionChannel::numSubmaps() const
{
  return 1;
}

// ######################################################################
Dims ForegroundDetectionChannel::getMapDims() const
{
  if (!this->hasInput())
    LFATAL("Oops! I haven't received any input yet");

  const Dims indims = this->getInputDims();

  return Dims(indims.w() >> itsLevelSpec.getVal().mapLevel(),
              indims.h() >> itsLevelSpec.getVal().mapLevel());

}

// ######################################################################
void ForegroundDetectionChannel::getFeatures(const Point2D<int>& locn,
                              std::vector<float>& mean) const
{
  LFATAL("not implemented");
}

// ######################################################################
void ForegroundDetectionChannel::getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                   std::vector<std::vector<float> > *mean,
                                   int *count) const
{
  LFATAL("not implemented");
}


// ######################################################################
void ForegroundDetectionChannel::doInput(const InputFrame& inframe)
{

#ifdef HAVE_OPENCV
  ASSERT(inframe.colorByte().initialized());
  LINFO("Input to Foreground Detection Channel ok.");

  //Convert the input frame to opencv format
  IplImage* inFrame_cv = img2ipl(inframe.colorByte());

  //If the statistics model has not been created (i.e. this is the first frame),
  //then create it.
  if(itsStatModel_cv == NULL)
    itsStatModel_cv = cvCreateFGDStatModel( inFrame_cv );

  //Update the statistics model
  cvUpdateBGStatModel( inFrame_cv, itsStatModel_cv );

  //Assign the foreground and background maps
  //OpenCV clears out the foreground and background memory at the beginning
  //of every icvUpdateFGDStatModel, so let's do a deep copy of the image
  //data just to be safe.
  //Also, because the source image is byte-valued, let's divide by 255 to get
  //a true probability
  itsForegroundMap = ipl2gray(itsStatModel_cv->foreground).deepcopy() / 255.0;

  //Rescale the image to the correct dimensions
  itsMap = rescale(itsForegroundMap, this->getMapDims());

  float mi, ma;
  getMinMax(itsMap,mi, ma);
  LINFO("FOREGROUND MAP RANGE: [%f .. %f]", mi, ma);

  //Free the memory allocated to the input frame - OpenCV makes it's own deep
  //copy of this data internally.
  cvReleaseImage( &inFrame_cv );

#endif


}

// ######################################################################
Image<float> ForegroundDetectionChannel::getSubmap(const uint index) const
{
  if (index != 0)
    LFATAL("got submap index = %u, but I have only one submap", index);

  return itsMap;
}

// ######################################################################
Image<float> ForegroundDetectionChannel::getRawCSmap(const uint idx) const
{
  return Image<float>();
}

// ######################################################################
std::string ForegroundDetectionChannel::getSubmapName(const uint index) const
{
  return std::string("ForegroundOutput");
}


// ######################################################################
Image<float> ForegroundDetectionChannel::getOutput()
{ return itsMap; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
