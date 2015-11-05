/*!@file Neuro/getSaliency.H Interface for obtaining lists of salient locations */

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
// Primary maintainer for this file: Dirk Walther <walther@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/getSaliency.H $
// $Id: getSaliency.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef GETSALIENCY_H_DEFINED
#define GETSALIENCY_H_DEFINED

#include "Image/Image.H"
#include "Neuro/StdBrain.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimEventQueue.H"
#include <vector>

//simple struct to store a submap and its name

typedef struct /// this should now be a NamedImage<float>
{
    Image<float> itsSubMap;
    std::string itsSubMapName;
} subMap;



//! This class allows for easy access to the list of salient points in an image
/*! It uses the Brain class in order to create a list of the coordinates and
  evolution times for the salient spots of an image. */
class GetSaliency : public StdBrain {
public:

  //! Constructor
  GetSaliency(OptionManager& mgr);

  //! Destructor
  ~GetSaliency();

  //! Compute the table of salient spots
  /*! Required parameters:
    @param inp input color image
    @param targets binary map of the same size as the input, in which
    targets for the focus of attention are 255 and the rest is
    zero. Pass an uninitialized image if you have no targets to look for.
    @return The number of salient spots found within max_time seconds. */
  const int compute(const Image< PixRGB<byte> >& img,
                    const SimTime& max_time);

  //! Returns the initial saliency map <B>prior</B> to the evolution.
  /*! Should be called after compute.
    @return The saliency map prior to the evolutions, normalized
    between 0.0 and 1.0 - the size is the same as the input image.*/
  const Image<float>& getSalmap();


  //! Returns the coordinates of the salient spots.
  /*!@return A vector of Point2D<int> objects - the x and y coordinates of
    the salient spots.*/
  const std::vector<Point2D<int> >& getCoords();


  //! Returns the evolution times.
  /*!@return A vector of <TT>double</TT> numbers that contains the
    evolution times for the salient spots. */
  const std::vector<SimTime>& getTimes();

  //!Returns the submaps
  const std::vector<subMap>& getSubMaps();

  //!Return the vc map
  Image<float> getVCXmap(const Image<PixRGB<byte> > &img);


protected:
  //! Callback for when a new attention shift occurs
  SIMCALLBACK_DECLARE(GetSaliency, SimEventWTAwinner);

  nub::ref<SimEventQueue> itsQ;
  Image<float> itsSalmap;
  std::vector<Point2D<int> > itsCoords;
  std::vector<SimTime> itsTimes;
  std::vector<subMap> itsSubMaps;

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
