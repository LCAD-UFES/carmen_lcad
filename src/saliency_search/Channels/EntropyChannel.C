/*!@file Channels/EntropyChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/EntropyChannel.C $
// $Id: EntropyChannel.C 7434 2006-11-11 02:15:19Z rjpeters $
//

#ifndef ENTROPYCHANNEL_C_DEFINED
#define ENTROPYCHANNEL_C_DEFINED

#include "Channels/EntropyChannel.H"

#include "Image/Transforms.H" // for highThresh()

// ######################################################################
// Entropy channel member definitions
// ######################################################################
EntropyChannel::EntropyChannel(OptionManager& mgr) :
  SingleChannel(mgr, "Entropy", "entropy", ENTROPY,
                rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsQstep("EntropyChannelQstep", this, 4),
  itsMap()
{ }

// ######################################################################
EntropyChannel::~EntropyChannel()
{ }

// ######################################################################
bool EntropyChannel::outputAvailable() const
{ return itsMap.initialized(); }

// ######################################################################
void EntropyChannel::doInput(const InputFrame& inframe)
{
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

  // prepare a histogram for our quantized patch values:
  const int nsteps = 256 / itsQstep.getVal();
  int histo[nsteps];
  Image<byte> quant = inframe.grayFloat() / nsteps;  // quantize
  quant = highThresh(quant, byte(nsteps - 1), byte(nsteps - 1)); // saturate

  // let's loop over the tiles and compute entropy for each:
  Image<float>::iterator dest = itsMap.beginw();

  for (int j = 0; j <= h-siz; j += siz)
    {
      const int jmax = std::min(j + siz, h);
      for (int i = 0; i <= w-siz; i += siz)
        {
          const int imax = std::min(i + siz, w);
          Image<byte>::const_iterator src = quant.begin() + i + j * w;
          memset(histo, 0, nsteps * sizeof(int));

          // accumulate the histogram of values:
          for (int jj = j; jj < jmax; jj ++)
            {
              for (int ii = i; ii < imax; ii ++)
                histo[*src++] ++;

              // skip to next input row:
              src += w - imax + i;
           }

          // now compute the entropy:
          float e = 0.0f;
          for (int k = 0; k < nsteps; k ++)
            if (histo[k])
              {
                const float freq = float(histo[k]) / siz2;
                e -= freq * logf(freq);
              }
          // store entropy in output:
          *dest++ = e;
        }
    }
}

// ######################################################################
Image<float> EntropyChannel::getOutput()
{ return itsMap; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // ENTROPYCHANNEL_C_DEFINED
