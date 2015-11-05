/*!@file Neuro/EnvObjDetection.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EnvObjDetection.H $
// $Id: EnvObjDetection.H 14605 2011-03-15 02:25:06Z dparks $
//

#ifndef NEURO_EnvObjDetection_H_DEFINED
#define NEURO_EnvObjDetection_H_DEFINED

#include "Component/ModelParam.H"
#include "Neuro/EnvSegmenter.H"

#include "Image/OpenCVUtil.H"

class EnvObjDetection : public EnvSegmenter
{
public:
  EnvObjDetection(OptionManager& mgr);

  virtual ~EnvObjDetection();

  virtual Rectangle getFoa(const Image<PixRGB<byte> >& rgbin,
                           const Point2D<int>& center,
                           Image<byte>* foamask,
                           Image<PixRGB<byte> >* segmentdisp) const;

private:
  OModelParam<std::string> itsCascadeFile;

#ifdef HAVE_OPENCV
  CvMemStorage* itsStorage;
  CvHaarClassifierCascade* itsCascade;
#endif
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_EnvObjDetection_H_DEFINED
