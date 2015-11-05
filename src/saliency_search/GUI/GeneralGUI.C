/*!@file GUI/GeneralGUI.C  A utility to create general guis */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/GeneralGUI.C $
// $Id: GeneralGUI.C 10794 2009-02-08 06:21:09Z itti $
//

#include "GUI/GeneralGUI.H"
#include "Component/ModelOptionDef.H"
#include "Component/ModelParam.H"
#include "Image/CutPaste.H"
#include "GUI/DebugWin.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"

#include "Util/JobWithSemaphore.H"
#include "Util/StringUtil.H"
#include "Util/WorkThreadServer.H"
#include "Util/sformat.H"
#include "rutz/compat_snprintf.h"

#include <ctype.h>
#include <deque>
#include <iterator>
#include <stdlib.h> // for atoi(), malloc(), free()
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <vector>


namespace
{
  class GeneralGUILoop : public JobWithSemaphore
  {
    public:
      GeneralGUILoop(GeneralGUI* genGUI)
        :
          itsGeneralGUI(genGUI),
          itsPriority(1),
          itsJobType("GUI Loop")
    {}

      virtual ~GeneralGUILoop() {}

      virtual void run()
      {
        ASSERT(itsGeneralGUI);
        while(1)
        {
          itsGeneralGUI->update();
          usleep(10000);
        }
      }

      virtual const char* jobType() const
      { return itsJobType.c_str(); }

      virtual int priority() const
      { return itsPriority; }

    private:
      GeneralGUI* itsGeneralGUI;
      const int itsPriority;
      const std::string itsJobType;
  };
}





// ######################################################################
GeneralGUI::GeneralGUI(OptionManager& mgr,
           const std::string& descrName,
           const std::string& tagName,
           Dims d):
  ModelComponent(mgr, descrName, tagName),
  itsPWins(0),
  itsWinDims(d),
  itsModelComponent(NULL),
  itsMeters(0)

{

}

// ######################################################################
GeneralGUI::~GeneralGUI()
{
  for(unsigned int i=0; i<itsPWins.size(); i++)
    if (itsPWins[i] != NULL)
      delete itsPWins[i];
}

void GeneralGUI::start2()
{
}

void GeneralGUI::startThread(nub::soft_ref<OutputFrameSeries> &ofs)
{
  LINFO("Starting Gui thread");
  //start a worker thread
  itsThreadServer.reset(new WorkThreadServer("GuiThread",1)); //start a single worker thread
  itsThreadServer->setFlushBeforeStopping(false);
  rutz::shared_ptr<GeneralGUILoop> j(new GeneralGUILoop(this));
  itsThreadServer->enqueueJob(j);

  itsOfs = ofs;
}

void GeneralGUI::stopThread()
{
  for(unsigned int i=0; i<itsPWins.size(); i++)
    if (itsPWins[i] != NULL)
      delete itsPWins[i];
 //TODO stop threads

}




void GeneralGUI::setupGUI(ModelComponent* comp, bool recurse)
{
  PrefsWindow* pWin = new PrefsWindow("Control", SimpleFont::FIXED(8));
  ASSERT(pWin);
  pWin->setValueNumChars(16);
  pWin->addPrefsForComponent(comp, false);

  itsPWins.push_back(pWin);

}

void GeneralGUI::update()
{
  for (unsigned int i=0; i<itsPWins.size(); i++)
    if (itsPWins[i] != NULL)
      itsPWins[i]->update(); // handle pending preference window events

  Image<PixRGB<byte> > disp(itsWinDims, ZEROS);
  Image<PixRGB<byte> > meters =
    makeMeters(2, Dims(disp.getDims().w() / 2, 13));
  inplacePaste(disp, meters, Point2D<int>(0,0));

  for(unsigned int i=0; i<itsImages.size(); i++) {
    const size_t ax = i % 2;
    const size_t ay = i / 2;
    Point2D<int> pasteLoc(Point2D<int>(256*ax,
          meters.getHeight()+ (ay * itsImages[i]->getHeight())));

    inplacePaste(disp, *itsImages[i], pasteLoc);
  }

  itsOfs->writeRGB(disp, "GUIDisplay",
      FrameInfo("GeneralGUI Display", SRC_POS));


}

void GeneralGUI::addMeter(const int* valPtr, const std::string label,
                 const int valMax, const PixRGB<byte> color)
{

        if (valPtr != NULL)
        {
                const MeterInfo minfo = {valPtr, label, valMax, color};
                itsMeters.push_back(minfo);
        }
}

void GeneralGUI::addImage(const Image<PixRGB<byte> > *imgPtr)
{
        if (imgPtr != NULL)
                itsImages.push_back(imgPtr);
}



Image<PixRGB<byte> > GeneralGUI::makeMeters(const size_t nx,
                const Dims& meterdims)
{
        if (itsMeters.size() == 0)
                return Image<PixRGB<byte> >();

        ASSERT(meterdims.w() > 0);
        ASSERT(meterdims.h() > 0);

        size_t maxlabelsize = itsMeters[0].label.size();
        for(unsigned int i=0; i<itsMeters.size(); i++)
                if (itsMeters[i].label.size() > maxlabelsize)
                        maxlabelsize = itsMeters[i].label.size();

        const SimpleFont f = SimpleFont::fixedMaxHeight(meterdims.h());

        const int meterx = f.w() * (maxlabelsize + 7);
        const int maxmeterlen = meterdims.w() - meterx;

        const size_t ny = (itsMeters.size() + nx-1) / nx;

        Image<PixRGB<byte> > result(meterdims.w() * nx, meterdims.h() * ny, ZEROS);

        for (unsigned int i = 0; i < itsMeters.size(); ++i)
        {
                const size_t ay = i % ny;
                const size_t ax = i / ny;

                const std::string txt =
                        sformat("%*s %i",
                                        int(maxlabelsize),
                                        itsMeters[i].label.c_str(),
                                        *itsMeters[i].valPtr);

                writeText(result, Point2D<int>(meterdims.w() * ax, meterdims.h() * ay),
                                txt.c_str(),
                                PixRGB<byte>(255), PixRGB<byte>(0), f);

                int meterlen =
                        clampValue(int(maxmeterlen * abs(*itsMeters[i].valPtr) / abs(itsMeters[i].valmax)),
                                        1, maxmeterlen);
                //if (itsMeters[i].valmax < 0) meterlen += (maxmeterlen/2);

                Image<PixRGB<byte> >::iterator itr =
                        result.beginw()
                        + meterdims.w()*ax + meterx + ay*meterdims.h()*result.getWidth();

                const int rowskip = result.getWidth() - maxmeterlen;

                const PixRGB<byte> c1(itsMeters[i].color);
                const PixRGB<byte> c2(c1/2);
                const PixRGB<byte> c3(c2/3);
                const PixRGB<byte> c4(c3/2);

                for (int y = 0; y < meterdims.h()-1; ++y)
                {
                        for (int x = 0; x < meterlen; ++x)
                                *itr++ = (x & 1) ? c2 : c1;
                        for (int x = meterlen; x < maxmeterlen; ++x)
                                *itr++ = (x & 1) ? c4 : c3;
                        itr += rowskip;
                }
        }

        return result;
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
