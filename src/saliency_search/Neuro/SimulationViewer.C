/*!@file Neuro/SimulationViewer.C visualize various model simulations */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewer.C $
// $Id: SimulationViewer.C 14508 2011-02-15 21:09:43Z dberg $
//

#include "Neuro/SimulationViewer.H"
#include "Component/OptionManager.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Media/MediaSimEvents.H"
#include "Simulation/SimEventQueue.H"

// ######################################################################
// implementation of SimulationViewer
// ######################################################################
SimulationViewer::SimulationViewer(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName) :
  SimModule(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventSceneDescription),
  SIMCALLBACK_INIT(SimEventObjectDescription),
  itsDisplayInterp(&OPT_SVdisplayInterp, this),
  itsMapFactor(&OPT_SVdisplayMapFactor, this),
  itsMapType(&OPT_SVdisplayMapType, this)
{ }

// ######################################################################
SimulationViewer::~SimulationViewer()
{ }

// ######################################################################
void SimulationViewer::
onSimEventSceneDescription(SimEventQueue& q, rutz::shared_ptr<SimEventSceneDescription>& e)
{
  LINFO("Scene is: %s ", e->getSceneData()->description.c_str());
}

// ######################################################################
void SimulationViewer::
onSimEventObjectDescription(SimEventQueue& q, rutz::shared_ptr<SimEventObjectDescription>& e)
{
  LINFO("Object is: %s id %i", e->getObjData()->description.c_str(), e->getObjData()->id);
}

// ######################################################################
Image<float> SimulationViewer::getMap(SimEventQueue& q,
                                      const bool warn) const
{
  const float fac = itsMapFactor.getVal();
  const std::string typ = itsMapType.getVal();
  Image<float> ret;

  if (typ.compare("SM") == 0)
    {
      if (SeC<SimEventSaliencyMapOutput> e =
          q.check<SimEventSaliencyMapOutput>(this, SEQ_ANY))
        ret = e->sm(fac);
      else if (warn) LERROR("Could not get a Saliency Map!");
    }
  else if  (typ.compare("TRM") == 0)
    {
      if (SeC<SimEventTaskRelevanceMapOutput> e =
          q.check<SimEventTaskRelevanceMapOutput>(this, SEQ_ANY))
        ret = e->trm(fac);
      else if (warn) LERROR("Could not get a Task Relevance Map!");
    }
  else if (typ.compare("AGM") == 0)
    {
      if (SeC<SimEventAttentionGuidanceMapOutput> e =
          q.check<SimEventAttentionGuidanceMapOutput>(this, SEQ_ANY))
        ret = e->agm(fac);
      else if (warn) LERROR("Could not get an Attention Guidance Map!");
    }
  else if  (typ.compare("VCO") == 0)
    {
      if (SeC<SimEventVisualCortexOutput> e =
          q.check<SimEventVisualCortexOutput>(this, SEQ_ANY))
        ret = e->vco(fac);
      else if (warn) LERROR("Could not get a Visual Cortex Output Map!");
    }
  else LFATAL("Unknown desired map type '%s'", typ.c_str());

  //inverse the map, will only happen if necessary default does nothing
  ret = inverseMap(ret);
  
  return ret;
}

// ######################################################################
// implementation of SimulationViewerAdapter
// ######################################################################
SimulationViewerAdapter::SimulationViewerAdapter(OptionManager& mgr, 
                                                 const std::string& descrName, 
                                                 const std::string& tagName) :
  SimulationViewer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventWTAwinner),
  itsInverseRetinal(&OPT_SVInverseTransform, this),
  itsInput(),
  itsCurrFOA(WTAwinner::NONE()),
  itsPrevFOA(WTAwinner::NONE()),
  itsTransform(new SpaceVariantModule(mgr))
{
  addSubComponent(itsTransform);
}

// ######################################################################
void SimulationViewerAdapter::reset1()
{
  itsInput.freeMem();
  itsCurrFOA = WTAwinner::NONE();
  itsPrevFOA = WTAwinner::NONE();
  itsTransform->clear();
  
 // propagate to our base class:
  SimulationViewer::reset1();
}

// ######################################################################
void SimulationViewerAdapter::
onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  // keep a copy of the image
  itsInput = e->frame().colorByte();
  
  //if we are a retinal type that produces a transform, then lets invert the image if the flag is set
  if (e->getRetTransform().is_valid() && e->getMapTransform().is_valid())
    {
      //update the local versions of the transform
      itsTransform->setTransform(e->getRetTransform());
      itsTransform->setMapTransform(e->getMapTransform());
      
      //transform the retinal image
      if (itsInverseRetinal.getVal())
        itsInput = itsTransform->inverseTransform(itsInput);      
    }

  doEventRetinaImage(q, e);//derived classes can do there own thing
}

// ######################################################################
void SimulationViewerAdapter::
onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  itsPrevFOA = itsCurrFOA;
  itsCurrFOA = e->winner();//get the current winner
  if (itsCurrFOA.isValid())
    {
      fromRetinal(itsCurrFOA.p);//inverse our retinal winner point if necessary
      fromRetinalMap(itsCurrFOA.smpos);//inverse the position in saliency map if necessary
    }
  doEventWTAwinner(q, e);//let derived classes do there own thing
}

// ######################################################################
Image<float> SimulationViewerAdapter::inverseMap(const Image<float>& map_image) const
{
  Image<float> map = map_image;
  
  //if we are a retinal type that produces a transform, then lets inverse the image if the flag is set
  if (itsTransform->validTransforms() && itsInverseRetinal.getVal())
    map = itsTransform->inverseMap(map);      
  return map;
}

// ######################################################################
Image<float> SimulationViewerAdapter::inverseRetinal(const Image<float>& ret_image) const
{
  Image<float> ret = ret_image;

  //if we are a retinal type that produces a transform, then lets inverse the image if the flag is set
  if (itsTransform->validTransforms() && itsInverseRetinal.getVal())
    ret = itsTransform->inverseTransform(ret);      
  return ret;
}

// ######################################################################
void SimulationViewerAdapter::toRetinal(Point2D<int>& point) const
{
  if (itsTransform->validTransforms())
    itsTransform->toSvCoords(point);
}

// ######################################################################
void SimulationViewerAdapter::fromRetinal(Point2D<int>& point) const
{
  if (itsTransform->validTransforms() && itsInverseRetinal.getVal())
    itsTransform->fromSvCoords(point);
}

// ######################################################################
void SimulationViewerAdapter::fromRetinalMap(Point2D<int>& point) const
{
  if (itsTransform->validTransforms() && itsInverseRetinal.getVal())
    itsTransform->fromSvCoordsMap(point);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
