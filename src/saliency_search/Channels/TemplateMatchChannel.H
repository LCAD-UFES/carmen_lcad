/*!@file Channels/TemplateMatchChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/TemplateMatchChannel.H $
// $Id: TemplateMatchChannel.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef TEMPLATEMATCHCHANNEL_H_DEFINED
#define TEMPLATEMATCHCHANNEL_H_DEFINED

#include "Channels/SingleChannel.H"

// ######################################################################
//! Convolves each level of its pyramid with a feature template.
/*! This type of channel is useful for searching for a specific feature
    shape at multiple scales. For example, it is used by VisualCortex to
    search for possible pedestrians in an input image. */
class TemplateMatchChannel : public SingleChannel
{
public:
  //! Construct with the standard params, plus a filter specification.
  /*! The filter is specified by a C array, which will be interpreted to
      have dimensions \a filt_x and \a filt_y. A private copy of the filter
      will be made, so the \a filt pointer does not have to be
      persistent. */
  TemplateMatchChannel(OptionManager& mgr, const Image<float>& filt);

  //! Virtual destructor.
  virtual ~TemplateMatchChannel();

  //! Look for the most likely template match in a given rectangular region.
  /*! The position of the match in 0-level coordinates is returned in
      \a matchpos, the pyramid level at which the match was found is
      returned in \a match_lev, and a score rating the goodness of the
      match is returned in \a score. */
  void findBestMatch(const Rectangle& r, Point2D<int>& matchpos,
                     int& match_lev, float& score);

  //! Draws a box in \a traj around the last best template match found
  virtual void drawResults(Image< PixRGB<byte> >& traj,
                           const Rectangle& foa);

protected:
  virtual void start1();

  Point2D<int> itsBestMatchPos;
  int itsBestMatchLev;
  float itsBestMatchScore;
  Dims itsFiltDims;         // only used by drawResults()
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TEMPLATEMATCHCHANNEL_H_DEFINED
