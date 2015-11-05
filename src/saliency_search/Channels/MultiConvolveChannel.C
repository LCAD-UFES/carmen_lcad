/*!@file Channels/MultiConvolveChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MultiConvolveChannel.C $
// $Id: MultiConvolveChannel.C 9308 2008-02-22 19:04:41Z rjpeters $
//

#ifndef MULTICONVOLVECHANNEL_C_DEFINED
#define MULTICONVOLVECHANNEL_C_DEFINED

#include "Channels/MultiConvolveChannel.H"

#include "Channels/ConvolveChannel.H"
#include "Image/ColorOps.H"
#include "Util/sformat.H"

// ######################################################################
// MultiConvolveChannel member definitions:
// ######################################################################
MultiConvolveChannel::MultiConvolveChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "MultiConvolve", "multiconv", CONVOLVE),
  itsFilterFname("MultiConvolveChannelFilterFname", this, ""),
  itsLumThresh("MultiConvolveChannelLuminanceThreshold", this, 25.5F)
{
  // since we only support 4 subchannels, let's build and name them,
  // though we don't know what their filters are yet; we'll assign a
  // filter to them in start1():
  for (int n = 0; n < 4; n ++)
    {
      nub::soft_ref<ConvolveChannel> cc(new ConvolveChannel(getManager()));

      // let's change our subchan's name:
      cc->setDescriptiveName(sformat("Convolve%d", n));
      cc->setTagName(sformat("conv%d", n));

      // add cc as one of our subchannels:
      addSubChan(cc);
    }
}

// ######################################################################
MultiConvolveChannel::~MultiConvolveChannel()
{ }

// ######################################################################
void MultiConvolveChannel::start1()
{
  // let's configure our subchannels by reading their kernel from
  // text file:
  const char *fname = itsFilterFname.getVal().c_str();
  FILE *f = fopen(fname, "r");
  if (f == NULL) LFATAL("Cannot open %s", fname);

  int w, h, d;
  if (fscanf(f, "%d %d %d\n", &w, &h, &d) != 3)
    LFATAL("Bogus first line in %s", fname);

  if (d != 4) LFATAL("Only 4 subchannels are supported");

  LINFO("Building %dx%dx%d kernel from '%s'", w, h, d, fname);
  for (int n = 0; n < d; n ++)
    {
      Image<float> ker(w, h, NO_INIT);
      for (int j = 0; j < h; j ++)
        for (int i = 0; i < w; i ++) {
          float coeff;
          if (fscanf(f, "%f\n", &coeff) != 1)
            LFATAL("Bogus coeff in %s at (%d, %d)", fname, i, j);
          ker.setVal(i, j, coeff);
        }

      // ok, let's assign this filter to one of our subchannels:
      (dynCast<ConvolveChannel>(subChan(n)))->setFilter(ker, CONV_BOUNDARY_ZERO);
    }
  fclose(f);
}

// ######################################################################
void MultiConvolveChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.colorFloat().initialized());

  Image<float> r, g, b, y;
  getRGBY(inframe.colorFloat(), r, g, b, y, itsLumThresh.getVal());

  ASSERT(numChans() == 4); // only 4 subchannels currently supported
  subChan(0)->input(InputFrame::fromGrayFloat(&r, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  subChan(1)->input(InputFrame::fromGrayFloat(&g, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  subChan(2)->input(InputFrame::fromGrayFloat(&b, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  subChan(3)->input(InputFrame::fromGrayFloat(&y, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MULTICONVOLVECHANNEL_C_DEFINED
