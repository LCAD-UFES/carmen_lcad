/*!@file AppMedia/app-framecount.C  */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/AppMedia/app-framecount.C $

#ifndef APPMEDIA_APP_FRAMECOUNT_C_DEFINED
#define APPMEDIA_APP_FRAMECOUNT_C_DEFINED

#include "Media/MgzDecoder.H"
#include "Psycho/EyeTrace.H"
#include "Image/PixelsTypes.H"
#include <iostream>
#include <fstream>

class FrameCount
{
  public:
    FrameCount(uint const chans, uint const offset, std::string const & et_filename, std::string const & mgz_filename,  std::string const & outfile) : itsEye(), itsMgz(), numFrame(0), numChan(chans), itsOffset(offset), itsOut(outfile)
    { 
      itsEye = rutz::make_shared(new EyeTrace(et_filename, PixRGB<byte>(0,0,0)));

      {
        std::ifstream filem(mgz_filename);
        if (!filem.is_open())
          return;
      }

      itsMgz = rutz::make_shared(new MgzDecoder(mgz_filename));

      while (itsMgz->skipFrame())
        ++numFrame;
    }
    
    ~FrameCount() 
    { 
      uint esize = itsEye->size();
      if ((esize != (numFrame/numChan - itsOffset)) || (numFrame == 0))
      {
        std::ofstream stimFile(itsOut);
        if (!stimFile.is_open())
          LFATAL("output file couldn't be opened");
        
        stimFile << itsEye->filename() << std::endl;
        stimFile.close();
      }
    }
    
  private:
    rutz::shared_ptr<EyeTrace> itsEye;
    rutz::shared_ptr<MgzDecoder> itsMgz;

    uint numFrame;
    uint numChan;
    uint itsOffset;
    std::string itsOut;
};

int main(const int argc, const char **argv)
{
  FrameCount s(atoi(argv[1]), atoi(argv[2]), argv[3], argv[4], argv[5]);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // APPMEDIA_APP_FRAMECOUNT_C_DEFINED
