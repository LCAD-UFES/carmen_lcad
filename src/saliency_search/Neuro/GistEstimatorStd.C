/*!@file Neuro/GistEstimatorStd.C extract estimated gist
         using available features of the image                          */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorStd.C $
// $Id: GistEstimatorStd.C 14596 2011-03-14 17:32:24Z sophie $
//

// ######################################################################
/*! Extract gist of image                                               */

#include "Neuro/GistEstimatorStd.H"

#include "Component/ModelManager.H"
#include "Channels/ChannelMaps.H"
//#include "Channels/BlueYellowChannel.H"
//#include "Channels/ColorChannel.H"
//#include "Channels/GaborChannel.H"
//#include "Channels/IntensityChannel.H"
//#include "Channels/OrientationChannel.H"
//#include "Channels/RedGreenChannel.H"
#include "GUI/XWinManaged.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Neuro/gistParams.H"
#include "Raster/Raster.H"

#include "Simulation/SimEventQueue.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimEvents.H"
#include "Channels/ChannelOpts.H"
#include "Channels/SingleChannel.H"

#include "Util/Timer.H"
#include "Util/StringUtil.H"


// ######################################################################
GistEstimatorStd::GistEstimatorStd(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName) :
  GistEstimatorAdapter(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventVisualCortexOutput),
  itsRawGistEstimStd(new RawGistEstimatorStd(mgr)) 
{
  itsGistVector.resize(1,NUM_GIST_FEAT * NUM_GIST_CHAN, NO_INIT);
  ////mgr.setOptionValString(&OPT_SingleChannelComputeFullPyramidForGist, "true");
 addSubComponent(itsRawGistEstimStd);
}

// ######################################################################
GistEstimatorStd::~GistEstimatorStd()
{ }


// ######################################################################
void GistEstimatorStd::
onSimEventVisualCortexOutput(SimEventQueue& q, rutz::shared_ptr<SimEventVisualCortexOutput>& e)
{
  //Grab the channel maps from the visual cortex
  rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
  q.request(vcxm); // VisualCortex is now filling-in the maps...
  rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

  //Compute the full size gist feature vector
  Image<float> GistVect;
  GistVect = itsRawGistEstimStd->compute(chm);
  itsGistVector = GistVect;


  // post an event so that anyone interested in gist can grab it:
  rutz::shared_ptr<SimEventGistOutput>
    ew(new SimEventGistOutput(this, itsGistVector)); //GistVect)); //itsGistVector));
  q.post(ew);
}

// ######################################################################
Image<double> GistEstimatorStd::getGist()
{
   return itsGistVector;
}

// ######################################################################
void GistEstimatorStd::start1()
{
  getManager().setOptionValString(&OPT_SingleChannelComputeFullPyramidForGist,"true");
  GistEstimatorAdapter::start1();
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
