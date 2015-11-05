/*!@file Robots2/Beobot2/Localization/Beobot2GistSalMaster.H
  Vision robot localization using a combination saliency and gist.
  Run app-Beobot2GistSalMaster at X1 to run Gist-Saliency model
  Run app-Beobot2_GistSalLocalizerMaster at X1 to run SIFT recognition master    
  Run app-Beobot2_GistSalLocalizerWorker at X[2 ... 8] to run SIFT recognition workers    
  see Siagian_Itti09tr                                                  */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Localization/Beobot2GistSalMaster.H $
// $Id: Beobot2GistSalMaster.H 15441 2012-11-14 21:28:03Z kai $
//

#ifndef BEOBOT2GISTSALMASTERI_H
#define BEOBOT2GISTSALMASTERI_H

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/BeobotEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>


#include "Beobot/BeobotBrainMT.H"
#include "Beobot/beobot-GSnav-def.H"

#include "Media/MediaOpts.H"

class Beobot2GistSalMasterI : public RobotBrainComponent
{
public:

  Beobot2GistSalMasterI(OptionManager& mgr,
      const std::string& descrName = "Beobot2GistSalMaster",
      const std::string& tagName = "Beobot2GistSalMaster");

  ~Beobot2GistSalMasterI();

  //! Get started. See ModelComponent.
  virtual void start1();

  virtual void evolve();

  //!Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  virtual void registerTopics();

  inline nub::soft_ref<InputFrameSeries> getIfs();
  inline void setInputFrameRate(uint64 inputFrameRate);

private:


  bool haveNextFrame();
  Image<PixRGB<byte> > getNextFrame(int& msg_id);

  void printStatistics();
  void abort();

  nub::soft_ref<InputFrameSeries> itsIfs;      // input image
  nub::soft_ref<BeobotBrainMT>    itsBbmt;     // saliency and gist
  rutz::shared_ptr<XWinManaged>   itsMwin;     // display window
  Image<PixRGB<byte> >            itsDisp;     // display image

  ImageSet<float>                 itsCmap;     // conspicuity map
  std::vector<Point2D<int> >      itsClmpt;    // currently followed landmarks
  std::vector<Point2D<int> >      itsNlmpt;    // currently searched landmarks

  // frame number and timing related
  IceUtil::Mutex its_Current_Image_mutex;        //!< locking current input image 
  IceUtil::Mutex its_bbmt_processing_mutex;      //!< locking bbmt processing results
  Image<PixRGB<byte> > itsCurrImage;             //!< current received image
  int                  itsCurrImageID;
  int itsLastProcessedImageMessageID;
  uint64 itsInputFrameRate;                    // in micro-seconds
  bool itsNextFrameFlag;
  Timer itsInputTimer;
  std::vector<float> itsTimes;

  bool itsReceivedCameraMessage;
};

// ######################################################################
// Implementation for Beobot2GistSalMaster inline functions
// ######################################################################
inline nub::soft_ref<InputFrameSeries>  Beobot2GistSalMasterI::getIfs()
{ return itsIfs; }

inline void Beobot2GistSalMasterI::setInputFrameRate(uint64 inputFrameRate)
{ itsInputFrameRate = inputFrameRate; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
