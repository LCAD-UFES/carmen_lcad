/*!@file Gist/CenterSurroundHistogramSegmenter.H segment out object
  depicted by the salient point using varying center surround
  histogram differencing */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/CenterSurroundHostogramSegmenter.H $
// $Id: $
//
//////////////////////////////////////////////////////////////////////////

#ifndef CENTER_SURROUND_HISTOGRAM_SEGMENTER
#define CENTER_SURROUND_HISTOGRAM_SEGMENTER

#include "Image/Image.H"
#include "Raster/Raster.H"
#include "Image/Pixels.H"
#include "SIFT/Histogram.H"

#include "GUI/XWinManaged.H"

//! center surround histogram segmenter
//! varies shape of center surround histogram differencing 
//! to estimata shape of object
class CenterSurroundHistogramSegmenter
{
public:

  // ######################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! constructor
  CenterSurroundHistogramSegmenter();

  //! destructor
  ~CenterSurroundHistogramSegmenter();

  //! set the input image
  void setImage(Image<PixRGB<byte> > image);

  //! get the salient region given the salient point 
  Image<float> getSalientRegion(Point2D<int> pt);

  //@}

  // ######################################################################
  //! @name Access functions
  //@{

  //! image getter
  inline Image<PixRGB<byte> > getImage();

  //@}

private:

  //! original input image
  Image<PixRGB<byte> > itsImage;

  //! debug window
  rutz::shared_ptr<XWinManaged> itsWin;

  //! list of histogram entries for various image features:
  //! FIXXX_NOTE: currently just brightness and color (from CIELab)
  std::vector<Image<int> > itsImageHistogramEntries;

  //! stored grid histogram
  Image<rutz::shared_ptr<Histogram> > itsGridHistogram;

  //! stored integral image
  Image<rutz::shared_ptr<Histogram> > itsIntegralHistogram;

  //! center surround shape templates 
  //! in a form of vector points
  std::vector<std::pair<std::vector<Point2D<int> >,
                        std::vector<Point2D<int> > > > 
  itsCSpoints;

  //! center surround shape templates 
  //! in a form of rectangles
  std::vector<std::pair<Rectangle,Rectangle> > itsCStemplates;

  //! center and surround beliefs
  Image<float> itsGridCenterBelief;
  Image<float> itsGridSurroundBelief;

  //! the final rectangle result
  Rectangle itsCSrectangle;

  //! smoothing kernel for histogram smoothing
  Image<float> itsLKernel;
  Image<float> itsAKernel;
  Image<float> itsBKernel;

  // ######################################################################
  //! @name Compute functions (all the inner-working functions)
  //@{

  //! set the histogram of feature values
  void setImageFeatureHistogramValues();

  //! quantize the input feature map 
  //! to the specified bin numbers
  Image<int> quantize_values(Image<float> image, int num_bins);

  //! compute histogram for each grid location 
  void computeGridHistogram();

  //! compute the integral image histogram
  void computeHistogramIntegralImage();

  //! compute the various center surround shape combination
  //! in rectangle shapes
  void computeCStemplates();
  
  //! estimate salient region 
  //! using vector of center surround points
  void ptsSalientRegion(Point2D<int> pt);

  //! estimate salient region 
  //! using rectangles of center surround points
  void csTemplateSalientRegion(Point2D<int> pt);

  //! display the current shape estimator belief
  void drawCurrentCSbelief
  (Point2D<int> pt = Point2D<int>(-1,-1), 
   Rectangle grC   = Rectangle(), 
   Rectangle grS   = Rectangle() );

  //! calculate the image distribution 
  //! from the given area
  rutz::shared_ptr<Histogram> getHistogramDistribution
  (Point2D<int> pt, int grid_size);

  //! get the histogram distribution 
  //! from the specified grid locations
  rutz::shared_ptr<Histogram> getGridHistogramDistribution
  (Rectangle r);

  //! smooth and normalize the feature vector distribution
  Histogram smoothAndNormalize(Histogram h, int numPoints);

  //! estimate center surround region
  //! from the center and surround belief map
  Rectangle growCSregion(Point2D<int> pt);

  //@}
};

// ######################################################################
// Implementation for CenterSurroundHistogramSegmenter inline functions
// ######################################################################
inline Image<PixRGB<byte> > CenterSurroundHistogramSegmenter::getImage()
{ return itsImage; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
