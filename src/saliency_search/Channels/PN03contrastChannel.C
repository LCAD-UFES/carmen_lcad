/*!@file Channels/PN03contrastChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/PN03contrastChannel.C $
// $Id: PN03contrastChannel.C 7434 2006-11-11 02:15:19Z rjpeters $
//

#ifndef PN03CONTRASTCHANNEL_C_DEFINED
#define PN03CONTRASTCHANNEL_C_DEFINED

#include "Channels/PN03contrastChannel.H"

// ######################################################################
// PN03contrast channel member definitions
// ######################################################################
PN03contrastChannel::PN03contrastChannel(OptionManager& mgr) :
  SingleChannel(mgr, "PN03contrast", "pn03contrast", PN03CONTRAST,
                rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsMap()
{ }

// ######################################################################
PN03contrastChannel::~PN03contrastChannel()
{ }

// ######################################################################
bool PN03contrastChannel::outputAvailable() const
{ return itsMap.initialized(); }

// ######################################################################
void PN03contrastChannel::doInput(const InputFrame& inframe)
{
//    LERROR("\n START doInput of PN03contrast \n");

  const LevelSpec ls = itsLevelSpec.getVal();
  ASSERT(ls.levMin() == ls.levMax());
  ASSERT(ls.delMin() == 0 && ls.delMax() == 0);
  ASSERT(ls.levMin() == ls.mapLevel());
  ASSERT(inframe.grayFloat().initialized());

  const uint lev = ls.mapLevel();

  // figure out the tile and map sizes:
  const int siz = 1 << lev;
  const int w = inframe.getWidth();
  const int h = inframe.getHeight();
  itsMap.resize(w >> lev, h >> lev);
  const float siz2 = float(siz * siz);

  // let's loop over the tiles and compute contrast for each:
  Image<float>::iterator dest = itsMap.beginw();
  for (int j = 0; j < h; j += siz)
  {
      const int jmax = std::min(j + siz, h);
      for (int i = 0; i < w; i += siz)
      {
           const int imax = std::min(i + siz, w);
           float patch_sum=0;
           double patch_sumsq=0;
           for (int jj = j; jj < jmax; jj ++)
           {
               for (int ii = i; ii < imax; ii ++) {
                   float pix_val = inframe.grayFloat().getVal(ii,jj);
                   patch_sum+=pix_val;
                   patch_sumsq+=pix_val*pix_val;
               }
           }
//           patch_sum = std::accumulate(inframe.grayFloat().begin() + i+j*w, inframe.grayFloat().begin() + imax+jmax*w, 0.0); // includes out-of-patch pixels!
           const double patch_mean =  patch_sum / double(siz2);
           const double patch_var = patch_sumsq / double(siz2) - patch_mean*patch_mean;

               // calculate the sd of pixel intensities (for long_patch_var)
//            double patch_diffsum=0;
//            if (i==0 && j==0) LERROR("patch mean: %.2f\n", patch_mean);
//            if (i==0 && j==0) LERROR( "[ " );
//            for (int jj = j; jj < jmax; jj ++)
//            {
//                for (int ii = i; ii < imax; ii ++) {
//                    double pix_val = inframe.grayFloat().getVal(ii,jj);
//                    if (i==0 && j==0) std::cerr << pix_val << " ";
//                    double pix_diff = pix_val-patch_mean;
//                    double pix_var = double(pix_diff*pix_diff);
//                    patch_diffsum+=pix_var;
//                }
//                if (i==0 && j==0) std::cerr << "\n  ";
//            }
//            if (i==0 && j==0) LERROR(" ]\n");

          // now compute the contrast:

//           if (i==0 && j==0) LERROR("short patch var %.2f\n", patch_var);
//            double long_patch_var=patch_diffsum/double(siz2);
//            if (i==0 && j==0) LERROR("long patch var %.2f\n", long_patch_var);

           const double patch_cntrst = sqrt(patch_var);
           if (i==0 && j==0) LERROR( "\nfirst %ix%i patch contrast: %.2f\n\n", siz, siz, patch_cntrst);
           *dest++ = patch_cntrst;
        }
    }
//  LERROR("\n END doInput of PN03contrast \n");
}

// ######################################################################
Image<float> PN03contrastChannel::getOutput()
{ return itsMap; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // PN03CONTRASTCHANNEL_C_DEFINED
