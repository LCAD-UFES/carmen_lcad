/*!@file GUI/GeneralGUI.H  A utility to create general guis */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/GeneralGUI.H $
// $Id: GeneralGUI.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef GeneralGUI_H_DEFINED
#define GeneralGUI_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Transport/TransportOpts.H"
#include "Media/FrameSeries.H"
#include "rutz/shared_ptr.h"
#include "Util/Types.H"
#include "Util/WorkThreadServer.H"
#include "Util/JobWithSemaphore.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/DrawOps.H"
#include "GUI/PrefsWindow.H"
#include "GUI/XWinManaged.H"
#include "Image/Layout.H"

static const ModelOptionDef OPT_ExtendedDisplay =
  { MODOPT_ARG(byte), "Nv2ExtendedDisplay", &MOC_OUTPUT, OPTEXP_CORE,
    "Amount of extra info to display in the GUI:\n"
    "    level 0: just input + saliency map\n"
    "    level 1: add meters underneath input+salmap\n"
    "    level 2: add single row of small internal maps\n"
    "    level 3: add double row of medium-sized internal maps",
    "extended-display", '\0', "<int>", "2" };

struct MeterInfo
{
        const int *valPtr;
        std::string label;
        int valmax;
        PixRGB<byte> color;
};

// ######################################################################
class GeneralGUI : public ModelComponent
{
public:

  // ######################################################################
  /*! @name Constructors and Destructors */
  //@{

  //! Constructor
  GeneralGUI(OptionManager& mgr,
             const std::string& descrName = "GeneralGUI",
             const std::string& tagName = "GeneralGUI",
             Dims d=Dims(512,512));

  //! Destructor
  ~GeneralGUI();

  void start2();

        void addMeter(const int* valPtr, const std::string label,
                 const int valMax, const PixRGB<byte> color);
        void addImage(const Image<PixRGB<byte> > *img);


  void setupGUI(ModelComponent* comp, bool recurse = false);
  void update(); //update the window

  void startThread(nub::soft_ref<OutputFrameSeries> &ofs);
  void stopThread();


  //@}

  // ######################################################################
  /*! @name member functions */
  //@{

private:
  std::vector<PrefsWindow*> itsPWins;
  rutz::shared_ptr<WorkThreadServer> itsThreadServer;
  XWinManaged *itsXwin;
  Dims itsWinDims;
  ModelComponent* itsModelComponent;
  std::vector<MeterInfo> itsMeters;
  std::vector<const Image<PixRGB<byte> >*> itsImages;
  Image<PixRGB<byte> > makeMeters(const size_t nx,
                                  const Dims& meterdims);
  nub::soft_ref<OutputFrameSeries> itsOfs;


};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
