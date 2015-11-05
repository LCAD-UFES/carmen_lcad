/*!@file Channels/SIFTChannel.C */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SIFTChannel.C $
// $Id: SIFTChannel.C 7434 2006-11-11 02:15:19Z rjpeters $
//

#ifndef SIFTCHANNEL_C_DEFINED
#define SIFTCHANNEL_C_DEFINED

#include "Channels/SIFTChannel.H"
#include "SIFT/ScaleSpace.H"
#include "SIFT/Keypoint.H"
#include "Image/ColorOps.H"

// ######################################################################
// SIFTChannel member definitions:
// ######################################################################

SIFTChannel::SIFTChannel(OptionManager& mgr) :
  SingleChannel(mgr, "SIFT Channel", "SIFT keypoints", SIFT,
                rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsMap()
{
  itsTakeAbs.setVal(true);
  itsNormalizeOutput.setVal(true);
}

// ######################################################################
SIFTChannel::~SIFTChannel() {}

// ######################################################################
bool SIFTChannel::outputAvailable() const
{ return itsMap.initialized(); }

// ######################################################################
void SIFTChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.grayFloat().initialized());
  Image<float> lum = inframe.grayFloat();

  const int nums = 3;        // recommended by David Lowe
  const double sigma = 1.6F; // recommended by David Lowe
  //TODO: add interpolation  (effects on pyramid?)
  float octscale = 1.0F;     // since we did not doubled the image

  //int numkp = 0;
  //for (int lev = 0; lev < depth; ++lev) //we dont want to process the 1x1 image
  {

     Image<float> imgOutput;
     if (lum.getWidth() > 24 and lum.getHeight() > 24){
        ImageSet<float> inImg(3);
        inImg[ScaleSpace::LUM_CHANNEL] = lum;
        std::vector< rutz::shared_ptr<Keypoint> > keypoints; // keypoints

        ScaleSpace ss(inImg, octscale, nums, sigma, false); //no color
        int nkp = ss.findKeypoints(keypoints);
        LDEBUG("Found %d keypoints in ScaleSpace %d", nkp, 1);

        // get ready for next ScaleSpace:
    //    lum = decXY(ss.getTwoSigmaImage(ScaleSpace::LUM_CHANNEL));

        octscale *= 2.0F;

        itsMap = ss.getKeypointImage(keypoints);
     }

     // result[lev] = imgOutput;

  }


}

// ######################################################################
Image<float> SIFTChannel::getOutput()
{ return itsMap; }




// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIFTCHANNEL_C_DEFINED
