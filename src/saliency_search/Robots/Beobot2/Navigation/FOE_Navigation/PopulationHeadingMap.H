/*!@file Robots/Beobot2/Navigation/FOE_Navigation/PopulationHeadingMap.H
  Lappe&Rauschecker 1993's Population Heading Map algorithm
 */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/PopulationHeadingMap.C $
// $Id: $
//

#ifndef ROBOTS_BEOBOT2_NAVIGATION_FOENAVIGATION_POPULATIONHEADINGMAP_DEFINED
#define ROBOTS_BEOBOT2_NAVIGATION_FOENAVIGATION_POPULATIONHEADINGMAP_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"

#include "GUI/XWinManaged.H"

#include "Robots/Beobot2/Navigation/FOE_Navigation/MotionOps.H"

//! Lappe&Rauschecker's Population Heading Map algorithm
class PopulationHeadingMap
{
public:
  // ######################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! Constructor 
  PopulationHeadingMap(float focalLength);

  //! blank constructor with an optional name entry
  PopulationHeadingMap(const std::string& name = "");

  //! function to populate the weights
  void initialize(Dims dims);

  //! function to load the weights from a file
  //bool loadFrom(const std::string& fname);

  //! function to save weights
  //bool saveTo(const std::string& fname);

  //! Destructor
  ~PopulationHeadingMap();

  //@}

  // ######################################################################
  //! @name Access functions
  //@{

  void setFocalLength(float focalLength);
  
  //@}

  // ######################################################################
  //! @name Compute functions
  //@{

  //! return FOE from image of movement between the two input images
  void getFOE(Image<byte> stim1, Image<byte> stim2);

  //! return 
  void getFOE(Image<byte> stim);

  //@}

private:

  //! the current image
  Image<byte> itsCurrentImage;
  
  //! the previous image
  Image<byte> itsPreviousImage;

  //! its input OpticFlow
  rutz::shared_ptr<OpticalFlow> itsOpticalFlow;

  //! display optical flow image
  Image<PixRGB<byte> > itsOpticalFlowImage;

  //! MT features: encode direction firing rate
  std::vector<Image<float> > itsMTfeatures;

  //! MT features: encode direction firing rate
  //! NOTE: NOT USED CURRENTLY
  std::vector <Image<float> > itsMToptimalShift;

  //! name of the landmark
  std::vector<Image<float> > itsJ;

  //! focal length
  float itsFocalLength;

  //! debug window
  rutz::shared_ptr<XWinManaged> itsWin;

  //! the heading direction map
  //! the maximum is the most likely heading direction
  Image<float> itsHeadingDirectionMap; 

  // ######################################################################
  //! @name Compute functions (all the inner-working functions)
  //@{

  //! compute the population heading map
  void computePopulationHeading();

  //! select n random locations from the optic flow
  std::vector<uint> pickRandomLocations(uint m);
  std::vector<Point2D<float> > getLocations(std::vector<uint> lind);
  std::vector<Point2D<float> > getFlowVectors(std::vector<uint> lind);
  
  //! compute C(T) 
  Image<double> computeC
  (std::vector<Point2D<float> > points, uint jx, uint jy);

  //! compute the orthogonal complement matrix C_perp(T)
  Image<double> computeOrthogonalComplement(Image<double> C);

  //! simple sigmoid function
  double sigmoid(double x); 

  void print(Image<double> img, const std::string& name);

  //@}
};

// ######################################################################
// Implementation for PopulationHeadingMap inline functions
// ######################################################################


#endif // ROBOTS_BEOBOT2_NAVIGATION_FOENAVIGATION_POPULATIONHEADINGMAP_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
