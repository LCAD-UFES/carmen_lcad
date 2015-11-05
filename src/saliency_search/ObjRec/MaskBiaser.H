/*!@file ObjRec/MaskBiaser.H */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/MaskBiaser.H $
// $Id: MaskBiaser.H 7833 2007-02-01 23:01:39Z rjpeters $
//

#ifndef OBJREC_MASKBIASER_H_DEFINED
#define OBJREC_MASKBIASER_H_DEFINED

#include "Channels/ChannelVisitor.H"
#include "Image/Image.H"

class MaskBiaser : public ChannelVisitor
{
public:
  /// Construct with settings for the algorithm
  MaskBiaser(Image<float>& mask,
                 const bool dobias);

  /// Virtual destructor
  virtual ~MaskBiaser();

  /// Visit the given ChannelBase node.
  virtual void visitChannelBase(ChannelBase& chan);

  /// Visit the given SingleChannel node.
  virtual void visitSingleChannel(SingleChannel& chan);

  /// Visit the given ComplexChannel node.
  virtual void visitComplexChannel(ComplexChannel& chan);

private:
  Image<float>& itsBiasMask;
  const int itsDoBias;
  int itsIndex;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // OBJREC_BAYESIANBIASER_H_DEFINED
