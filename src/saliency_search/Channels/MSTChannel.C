/*!@file Channels/MSTChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MSTChannel.C $
// $Id: MSTChannel.C 14356 2011-01-04 21:30:52Z dberg $
//

#ifndef MSTCHANNEL_C_DEFINED
#define MSTCHANNEL_C_DEFINED

#include "Channels/MSTChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/ComplexChannel.H"
#include "Channels/DirectionChannel.H"
#include "Channels/MotionChannel.H"
#include "Component/OptionManager.H"
#include "Image/FilterOps.H"
#include "Util/sformat.H"
#include "rutz/trace.h"

// ######################################################################
// MST Channel member definitions:
// ######################################################################
MSTChannel::MSTChannel(OptionManager& mgr,
                                 nub::soft_ref<MotionChannel> oc,
                                 const VisualFeature vs,
                                 const int r0, const int r1, const int r2,
                                 const int r3, const int r4, const int r5,
                                 const int r6, const int r7) :
  SingleChannel(mgr, "MSTChannel", "MSTChannel", vs,
                rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsFull(vs == FOEMST ? &OPT_EFullImplementation :
          (ModelOptionDef*) 0,
          this),  // see Channels/ChannelOpts.C
  itsDelta(vs == FOEMST ? &OPT_EndPointChannelDelta :
           (ModelOptionDef*) 0,
           this),
  itsOriChan(oc),
  R0(r0), R1(r1), R2(r2), R3(r3), R4(r4), R5(r5), R6(r6), R7(r7)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsTakeAbs.setVal(true);
 
  setDescriptiveName(sformat("MST(%d%d%d%d%d%d%d%d)",
                             r0, r1, r2, r3, r4, r5, r6, r7));

  setTagName(sformat("MST_%d%d%d%d%d%d%d%d",
                     r0, r1, r2, r3, r4, r5, r6, r7));
LINFO("***mst**");
}

// ######################################################################
MSTChannel::~MSTChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void MSTChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // access the orientation channel
  ASSERT(itsOriChan.get() != 0);
  const uint num = itsOriChan->getModelParamVal<uint>("NumDirections");
  ASSERT(num == 4);

  LINFO("**doing input in *mst**");
  
  // access the gabor pyramids at the relevant orientations: 0, 45,
  // 90, 135
  const ImageSet<float>& ori0_pyr =
    dynCast<SingleChannel>(itsOriChan->subChan(0))->pyramid(0);
  const ImageSet<float>& ori45_pyr =
    dynCast<SingleChannel>(itsOriChan->subChan(1))->pyramid(0);
  const ImageSet<float>& ori90_pyr =
    dynCast<SingleChannel>(itsOriChan->subChan(2))->pyramid(0);
  const ImageSet<float>& ori135_pyr =
    dynCast<SingleChannel>(itsOriChan->subChan(3))->pyramid(0);

  ImageSet<float> result(ori0_pyr.size());
  SingleChannel::killCaches();
  SingleChannel::setClipPyramid(inframe.clipMask());

  const bool R[8] = { (bool)R0, (bool)R1, (bool)R2, (bool)R3, (bool)R4, 
                      (bool)R5, (bool)R6, (bool)R7 };

  const uint dx = itsDelta.getVal();
  const uint dy = itsDelta.getVal();

  //LINFO("MST Delta %d",itsDelta.getVal());

  // combine the orientation pyramids to form the L pyramid


  if (itsFull.getVal())
  {

    for (uint i = 0; i < ori0_pyr.size(); i ++)
      result[i] = MSTFilterFull(ori0_pyr[i],  ori45_pyr[i],
                                ori90_pyr[i], ori135_pyr[i],
                                R, dx, dy);
  }
  else
  {
    for (uint i = 0; i < ori0_pyr.size(); i ++)
      result[i] = MSTFilterPartial(ori0_pyr[i],  ori45_pyr[i],
                                   ori90_pyr[i], ori135_pyr[i],
                                   R, dx, dy);
  }
   // store the result
   SingleChannel::storePyramid(result, inframe.time());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MSTCHANNEL_C_DEFINED
