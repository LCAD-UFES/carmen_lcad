/*!@file Gist/ContourBoundaryDetector.H Detect meaningful contours by
   weighting longer countour chains more */
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
// Primary maintainer for this file: Christian Siagian <siagian@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/ContourBoundaryDetector.H $
// $Id: $
//
//////////////////////////////////////////////////////////////////////////
//
// Detect meaningful contours by weighting longer countour chains more
// 
// Based on Hidayat's Variance Ridge boundary detector. 
// However, we thin and parameterize the boundary to contours.
//
//
// Real-time texture boundary detection from ridges
// in the standard deviation space
// Ray Hidayat and Richard Green
// BMCV 2009

#ifndef CONTOUR_BOUNDARY_DETECTOR
#define CONTOUR_BOUNDARY_DETECTOR

#define NUM_GRADIENT_DIRECTIONS   8
#define NUM_RIDGE_DIRECTIONS      NUM_GRADIENT_DIRECTIONS/2
#define BOUNDARY_STEP_SIZE        NUM_GRADIENT_DIRECTIONS
#define DEFAULT_NEIGHBORHOOD_RAD  8

#include "Image/Image.H"
#include "Raster/Raster.H"

#include "GUI/XWinManaged.H"

struct Edgel
{
  Edgel() { };

  Edgel(const Point2D<int> inPt,
        const float inAngle,
        const int inAngleIndex,
        const float inVal) :
    pt(inPt),
    angle(inAngle),
    angleIndex(inAngleIndex),
    val(inVal)
  {
  }

  Point2D<int> pt;
  float angle;
  int angleIndex; // -1 means no value
  float val;

  bool operator < (const Edgel& e)
  {
    return val < e.val;
  }
};

struct Contour
{
  Contour() { };

  Contour(const std::vector<rutz::shared_ptr<Edgel> > inEdgels) :
    edgels(inEdgels)
  {
  }

  std::vector<rutz::shared_ptr<Edgel> > edgels;
};


//! Contour boundary detector implementing:
//! Real-time texture boundary detection from ridges
//! in the standard deviation space
//! Ray Hidayat and Richard Green
//! BMCV 2009
class ContourBoundaryDetector
{
public:

  // ######################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! constructor
  ContourBoundaryDetector();

  //! destructor
  ~ContourBoundaryDetector();

  //@}

  // ######################################################################
  //! @name Compute functions
  //@{

  //! the full procedure to calculate
  //! contour boundary 
  void computeContourBoundary
  (Image<PixRGB<byte> > ima, int r = -1);

  //! get the variance ridge boundary
  Image<float> getVarianceRidgeBoundaryMap
  (Image<PixRGB<byte> > ima, int r = -1);

  //! get the contour boundary map
  //! assume computeContourBoundary or
  //! or other compute functions that includes
  //! variance ridge boundary map computation
  //! is already called on the same image
  Image<float> getVarianceRidgeBoundaryMap();

  //! get the Non-max Suppressed 
  //! Variance Ridge (VR) boundary
  Image<float> getNmsBoundaryMap
  (Image<PixRGB<byte> > ima, int r = -1);

  //! get the non-max suppressed VR boundary map
  //! assume computeContourBoundary or
  //! or other necessary compute functions, which includes
  //! Non-max Suppressed VR boundary map computation
  //! is already called on the same image
  Image<float> getNmsBoundaryMap();

  // get the contour boundary edgels 
  Image<float> getEdgelBoundaryMap
  (Image<PixRGB<byte> > ima, int r = -1);

  //! get the non-max suppressed VR boundary map
  //! assume computeContourBoundary or
  //! or other necessary compute functions, which includes
  //! edgel boundary map computation
  //! is already called on the same image
  Image<float> getEdgelBoundaryMap();

  //! get the contour boundary map
  Image<PixRGB<byte> > getContourBoundaryMap
  (Image<PixRGB<byte> > ima, int r = -1);

  //! get the contour boundary map
  //! assume computeContourBoundary or
  //! or other necessary compute functions, which includes
  //! contour boundary map computation
  //! are already called on the same image
  Image<PixRGB<byte> > getContourBoundaryMap();

  //! display the gradient magnitude image 
  void displayGradImage(std::vector<Image<float> > gradImg);

  //@}

  // ######################################################################
  //! @name Access functions
  //@{
  
  //! get contour boundaries
  inline std::vector<rutz::shared_ptr<Contour> > getContourBoundaries();

  //! get the edgels
  inline Image<rutz::shared_ptr<Edgel> > getEdgels();

  //@}

private:

  // ######################################################################
  //! @name Compute functions (all the inner-working functions)
  //@{

  //! compute the Variance Ridge Boundary map
  void computeVarianceRidgeBoundaryMap();

  //! compute the non-max suppressed
  //! variance ridge boundary map
  void computeNmsBoundaryMap();

  //! compute the contour boundary edgels
  void computeContourBoundaryEdgels();

  //! compute the contour boundary map
  void computeContourBoundaryMap();

  //! calculate the local standard deviation of the image
  Image<float> getLabStandardDeviation
  (Image<PixRGB<byte> > ima, int r = 8);

  //! VarianceRidgeDetector: box-blur the image for local mean
  Image<float> boxBlur
  (Image<float> ridgeImg, int rad);

  //! VarianceRidgeDetector: square combine the Lab channels
  Image<float> sqCombine
  (Image<float> a, Image<float> b, Image<float> c);

  //!  VarianceRidgeDetector: 
  //!    calculate gradient of the standard dev. image
  std::vector<Image<float> > calculateGradient
  (Image<float> varImg, int r = 8);

  //!  VarianceRidgeDetector: 
  //!    compute the ridge of the gradient image
  Image<float>  getRidge
  (std::vector<Image<float> > gradImg, int r = 8);

  //! substract the gradient magnitude from the ridge image
  Image<float> substractGradImg
  (Image<float> ridgeImg, std::vector<Image<float> >  gradImg);

  //! compute the contour boundary edgels
  Image<float> getContourBoundaryEdgels();

  //! compute the non-max suppresion image 
  Image<float> getNonMaxSuppression(Image<float> bImg);

  //! group the boundary edgels to contours
  void connectContourEdgels();

  //! check if the contour can be lengthened with the new edgel
  bool addEdgelToContour
  (int &i, int &j, 
   rutz::shared_ptr<Edgel> &edgel, 
   rutz::shared_ptr<Contour> &contour,
   int sign);

  //! get the location of where to look for next edgels
  //! given a direction
  void getEdgelDirectionNeighborhood
  (int ci, int cj, int dir, int sign, 
   int &ni1, int &nj1, int &ni2, int &nj2, int &ni3, int &nj3);

  //! given two edgels 
  //! figure out if the they can be connected
  bool isLegalDirection
  (int ci, int cj, int cd, int sign, 
   int ni, int nj, int nd);

  //! create an image representation of the contours
  Image<PixRGB<byte> > getContourBoundaryImage();

  //! display contour boundary 1 contour at a time
  void displayContourBoundary();

  //@}

  //! its current input image
  Image<PixRGB<byte> > itsImage;
 
  //! the radius of considered neighborhood
  int itsRadius;

  //! the ridge direction map
  std::vector<Image<float> > itsRidgeDirection;

  //! the maximum values (for each pixel) of the ridge direction map
  std::vector<Image<float> > itsRidgeDirectionMax;

  //! the winning index (for each pixel) of the ridge direction map
  Image<float> itsRidgeDirectionIndex;
  
  //! boundary map image (from Variance Ridge Detector)
  Image<float> itsBoundaryImage;

  //! the ridge direction map
  std::vector<Image<float> > itsRidgeDirectionNMS;

  //! the non-max suppressed version of the boundary map
  //! We use our own 5x5 kernel 
  Image<float> itsBoundaryImageNMS;

  //! the raw gradient standard deviation values 
  std::vector<std::vector<Image<float> > > itsDVin; 

  //! list of edgels in all RF edgel locations
  Image<std::vector<rutz::shared_ptr<Edgel> > > itsCompleteEdgels;

  //! list of just the max edgels in each RF location
  Image<rutz::shared_ptr<Edgel> > itsEdgels;

  //! edgel boundary map image
  Image<float> itsEdgelBoundaryImage;

  //! the list of contour Boundaries
  std::vector<rutz::shared_ptr<Contour> > itsContourBoundaries;

  //! contour storage based on location
  Image<rutz::shared_ptr<Contour> > itsContourMap;

  //! contour boundary image
  Image<PixRGB<byte> > itsContourBoundaryImage;

  //! region segmentation image 
  Image<PixRGB<byte> > itsRegionSegmentationImage;

  //! debug window
  rutz::shared_ptr<XWinManaged> itsWin;
};

// ######################################################################
// Implementation for ContourBoundaryDetector inline functions
// ######################################################################
inline std::vector<rutz::shared_ptr<Contour> > 
ContourBoundaryDetector::getContourBoundaries()
{ return itsContourBoundaries; }

inline Image<rutz::shared_ptr<Edgel> > 
ContourBoundaryDetector::getEdgels()
{ return itsEdgels; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
