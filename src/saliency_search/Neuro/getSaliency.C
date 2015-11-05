/*!@file Neuro/getSaliency.C Interface for obtaining lists of salient
  locations */

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
// Primary maintainer for this file: Dirk Walther <walther@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/getSaliency.C $
// $Id: getSaliency.C 13065 2010-03-28 00:01:00Z itti $
//

#include "Neuro/getSaliency.H"

#include "Channels/ChannelMaps.H"
#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/VisualCortex.H"
#include "Util/log.H"
#include "Image/MathOps.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include <string>

// ######################################################################
GetSaliency::GetSaliency(OptionManager& mgr) :
  StdBrain(mgr),
  SIMCALLBACK_INIT(SimEventWTAwinner),
  itsQ(new SimEventQueue(mgr)),
  itsSalmap(), itsCoords(), itsTimes()
{
  this->addSubComponent(itsQ);

  // by default, only use CIO channels and no TRM:
//  mgr.setOptionValString(&OPT_RawVisualCortexChans, "CIO");
//  mgr.setOptionValString(&OPT_TaskRelevanceMapType, "None");
}


// ######################################################################
GetSaliency::~GetSaliency()
{ }

// ######################################################################
void GetSaliency::
onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  itsCoords.push_back(e->winner().p);
  itsTimes.push_back(q.now());
}

// ######################################################################
const int GetSaliency::compute(const Image< PixRGB<byte> >& img,
                               const SimTime& max_time)
{
  LINFO("Start at %.2fms", itsQ->now().msecs());

  SimTime end_time = itsQ->now() + max_time;

  // post the input as an event:
  rutz::shared_ptr<SimEventInputFrame> e(new SimEventInputFrame(this, GenericFrame(img), 0));
  itsQ->post(e);

  // start with empty lists of coords and times:
  itsCoords.clear(); itsTimes.clear();

  // time-loop, evolve until we have anough attention shifts:
  while (itsQ->now() < end_time) itsQ->evolve();

  // obtain the saliency map to return it with getSalmap
  if (SeC<SimEventVisualCortexOutput> e = itsQ->check<SimEventVisualCortexOutput>(this))
    itsSalmap = e->vco();

  // return number of winners
  return itsTimes.size();
}

// ######################################################################
const Image<float>& GetSaliency::getSalmap()
{ return itsSalmap; }

// ######################################################################
const std::vector<Point2D<int> >& GetSaliency::getCoords()
{ return itsCoords; }

// ######################################################################
const std::vector<SimTime>& GetSaliency::getTimes()
{ return itsTimes; }

// ######################################################################
const std::vector<subMap>& GetSaliency::getSubMaps()
{
  // grab all the VisualCortex maps:
  rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
  itsQ->request(vcxm); // VisualCortex is now filling-in the maps...
  rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

  uint numSubmaps = chm->numSubmaps();
  Dims mapDims = chm->getMap().getDims();
  std::vector<subMap> theSubMaps(numSubmaps);

  for(uint i=0;i < numSubmaps; i++)
    {
      NamedImage<float> tempMap = chm->getRawCSmap(i);
      theSubMaps[i].itsSubMap = tempMap;  // conversion to Image
      theSubMaps[i].itsSubMapName = tempMap.name();

      if (theSubMaps[i].itsSubMap.getWidth() > mapDims.w())
        theSubMaps[i].itsSubMap = downSize(theSubMaps[i].itsSubMap, mapDims);
      else if (theSubMaps[i].itsSubMap.getWidth() < mapDims.w())
        theSubMaps[i].itsSubMap = rescale(theSubMaps[i].itsSubMap, mapDims);
    }
  itsSubMaps = theSubMaps;
  return itsSubMaps;
}

Image<float> GetSaliency::getVCXmap(const Image<PixRGB<byte> > &img)
{
  SimStatus status = itsQ->evolve();
  if (status != SIM_CONTINUE) LFATAL("Quitting! Queue Status is %d", status);

  // Create a new Retina Image from the inputImage, and post it to the queue
  itsQ->post(rutz::make_shared(
        new SimEventRetinaImage(
          this,
          InputFrame(InputFrame::fromRgb(&img, itsQ->now())),
          Rectangle(Point2D<int>(0,0), img.getDims()),
          Point2D<int>(0,0)
          )
        )
      );

  Image<float> vcMap;
  //Get the raw, unnormalized visual cortex output map
  if (SeC<SimEventVisualCortexOutput> e = itsQ->check<SimEventVisualCortexOutput>(this, SEQ_ANY))
    vcMap = e->vco(1.0F);


  return vcMap;
}



// ######################################################################



/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
