/*!@file VFAT/featureClusterFilters.H Filters for featureClusterVision */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/featureClusterFilters.H $
// $Id: featureClusterFilters.H 14376 2011-01-11 02:44:34Z pez $
//

#ifndef FEATURECLUSTERFILTERS_H_DEFINED
#define FEATURECLUSTERFILTERS_H_DEFINED

// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itti itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################


#include "Util/Assert.H"
#include "Image/Image.H"

#include <vector>

//! this method will apply a filter at a location and return a value
/*! This method is useful if your need to filter an image in a
  non-linear method or only need to extract filtered patches.
  Other than that, you should use the other methods above.
*/
template <class T_or_RGB> inline
T_or_RGB filterAtLocation(Image<T_or_RGB> *image,
                      Image<float> *kernel, Point2D<int> *location);

//! this method will apply a filter at a location and return a value
/*! This method is useful if your need to filter an image in a
  non-linear method or only need to extract filtered patches.
  Other than that, you should use the other methods above.
  This method differs from filterAtLocation in that it accepts
  a batch of locations.
*/
template <class T_or_RGB> inline
void filterAtLocationBatch(const Image<T_or_RGB> *image,
                           const Image<float> *kernel,
                           const std::vector<Point2D<int>*> *location,
                           std::vector<T_or_RGB> *results,
                           const unsigned int size);

//! This method applies filter on an image at multiple scales at select locals
/*! When called this method will run each of the supplied filters on the
  image at the scales ranging from full scale to size/2^n. To keep the
  workings straight, supply a name for each filter and then you can
  be sure how the results allign in the output vector. The method
  for each filter is basic convolution, but it limited to each location
  rather than the whole image.
  @param image this is the input raw image
  @param location This is a list of the locations at which to apply filters
  @param locations This is how many locations are in the list
  @param filterNames a list of what each filter is called for labels
  @param filters these are several kernels for filter use
  @param scales how man scales down do you want to filter each image
  @param quarterSize scale image by sqrt(.5)(quarter-ish) or 1/2
  @param normalize normalize over the filter outputs
  @param results what you get back from all this
  @param resultsLables User supplied descriptors for each filter
*/
template <class T_or_RGB> inline
void multiScaleBatchFilter(const Image<T_or_RGB> *image,
                           const std::vector<Point2D<int>*> *location,
                           const unsigned int *locations,
                           const std::vector<Image<float> > *filters,
                           const unsigned int scales,
                           const bool quarterSize,
                           const bool normalize,
                           std::vector<std::vector<T_or_RGB> > *results);

//! this will find junctions after calling multiScaleBatchFilter
/*! This is optional and may be called to find junctions if
  multiScaleBatchFilter was called using gabor filters at
  0,45,90 and 135 degrees. Call this after calling multiScaleBatchFilter
  if wanted.
*/
template <class T> inline
void multiScaleJunctionFilter(const unsigned int size,
                              const unsigned int scales,
                              const std::vector<std::vector<PixH2SV2<T> > >
                              *input,
                              std::vector<std::vector<PixH2SV2<T> > > *results);

//! call this optionally after multiScaleBatchFilter
template <class T> inline
void multiScaleJunctionFilter(const unsigned int size,
                              const unsigned int scales,
                              const std::vector<std::vector<T> >
                              *input,
                              std::vector<std::vector<T> > *results);

// ######################################################################
// ######################################################################
// ##### Inline function definitions
// ######################################################################
// ######################################################################

// ######################################################################
template <class T_or_RGB> inline
T_or_RGB filterAtLocation(Image<T_or_RGB> *image,
                          Image<float> *kernel, Point2D<int> *location)
{
  ASSERT((kernel->getWidth() % 2 == 1) && (kernel->getHeight() % 2 == 1));
  int w        = image->getWidth();
  int h        = image->getHeight();
  int kw       = kernel->getWidth();
  int kh       = kernel->getHeight();
  int kx       = (kw-1)/2;
  int ky       = (kh-1)/2;
  int kstartx  = 0;
  int kstarty  = 0;

  // do some extra work to account for overlapping on edge
  // e.g. bounds checking

  int startx = location->i - kx;
  if(startx < 0){ kstartx = abs(startx); startx = 0;}
  int endx   = location->i + kx;
  if(endx >= w) { endx = w - 1; }

  int starty = location->j - ky;
  if(starty < 0){ kstarty = abs(starty); starty = 0;}
  int endy   = location->j + ky;
  if(endy >= h) { endy = h - 1; }

  T_or_RGB result = T_or_RGB(0);
  T_or_RGB zero   = T_or_RGB(0);
  float sum = 0;
  //LINFO("C");
  //LINFO("%d %d %d %d",startx,endx,starty,endy);
  //LINFO("%d %d",kstartx,kstarty);
  typename Image<float>::iterator    ker;
  typename Image<T_or_RGB>::iterator img;
  for(int iy = starty; iy <= endy; iy++)
  {
    // start at kernel offset for any edges
    ker = kernel->beginw() +
      (kw*(kstarty+iy-starty)) + kstartx;
    //LINFO("%d", (kw*(kstarty+iy-starty))+ kstartx);
    //LINFO("%d", w*iy + startx);
    //LINFO("%d", w*iy + endx);
    // start in image offset for any edges
    for(img = image->beginw() + w*iy + startx;
        img != image->beginw() + w*iy + endx; ++img, ++ker)
    {
      result += clamped_convert<T_or_RGB>((*img) * (*ker));
      //LINFO("C2");
      sum += *ker;
      //LINFO("C3");
    }
  }
  // normalize results
  if(sum != 0)
  {
    result = clamped_convert<T_or_RGB>(result/sum);
  }
  else
  {
    result = zero;
  }
  return result;
}

// ######################################################################
template <class T_or_RGB> inline
void filterAtLocationBatch(const Image<T_or_RGB> *image,
                           const Image<float> *kernel,
                           const std::vector<Point2D<int>*> *location,
                           std::vector<T_or_RGB> *results,
                           const unsigned int size)
{
  ASSERT((kernel->getWidth() % 2 == 1) && (kernel->getHeight() % 2 == 1));
  ASSERT(results->size() >= size);
  int w        = image->getWidth();
  int h        = image->getHeight();
  int kw       = kernel->getWidth();
  int kh       = kernel->getHeight();
  int kx       = (kw-1)/2;
  int ky       = (kh-1)/2;
  int kstartx  = 0;
  int kstarty  = 0;

  typename std::vector<T_or_RGB>::iterator resultsItr = results->begin();

  std::vector<Point2D<int>*>::const_iterator locationItr = location->begin();

  // do some extra work to account for overlapping on edge
  // e.g. bounds checking

  int startx,endx,starty,endy;

  typename Image<float>::const_iterator ker;
  typename Image<T_or_RGB>::const_iterator img;
  bool useSlow;
  T_or_RGB zero = T_or_RGB(0);

  for(unsigned int i = 0; i < size; i++, ++resultsItr, ++locationItr)
  {
    useSlow = false;
    startx = (*locationItr)->i - kx;
    if(startx < 0){ kstartx = abs(startx); startx = 0; useSlow = true; }
    endx   = (*locationItr)->i + kx;
    if(endx >= w) { endx = w - 1; useSlow = true; }

    starty = (*locationItr)->j - ky;
    if(starty < 0){ kstarty = abs(starty); starty = 0; useSlow = true; }
    endy   = (*locationItr)->j + ky;
    if(endy >= h) { endy = h - 1; useSlow = true; }

    float sum = 0;
    // use slow if the kernal runs off the image
    // otherwise use a slightly faster iterator incr.
    *resultsItr = zero;

    if(useSlow == true)
    {
      //LINFO("SLOW");
      for(int iy = starty; iy <= endy; iy++)
      {
        // start at kernel offset for any edges
        ker = kernel->begin() +
          (kw*(kstarty+iy-starty)) + kstartx;
        // start in image offset for any edges
        for(img = image->begin() + w*iy + startx;
            img != image->begin() + w*iy + endx; ++img, ++ker)
        {
          *resultsItr += clamped_convert<T_or_RGB>((*img) * (*ker));
          sum += *ker;
        }
      }
      // normalize results
      if(sum != 0)
      {
        *resultsItr = clamped_convert<T_or_RGB>((*resultsItr)/sum);
      }
      else
      {
        *resultsItr = zero;
      }
    }
    else
    {
      //LINFO("FAST");
      ker = kernel->begin();
      for(int iy = starty; iy <= endy; iy++)
      {
        for(img = image->begin() + w*iy + startx;
            img != image->begin() + w*iy + endx; ++img, ++ker)
        {
          *resultsItr += clamped_convert<T_or_RGB>((*img) * (*ker));
          sum += *ker;
        }
      }
      if(sum != 0)
      {
        *resultsItr = clamped_convert<T_or_RGB>((*resultsItr)/sum);
      }
      else
      {
        *resultsItr = zero;
      }
    }
  }
}

// ######################################################################
template <class T_or_RGB> inline
void multiScaleBatchFilter(const Image<T_or_RGB> *image,
                           const std::vector<Point2D<int>*> *location,
                           const unsigned int *locations,
                           const std::vector<Image<float> > *filters,
                           const unsigned int scales,
                           const bool quarterSize,
                           const bool normalize,
                           std::vector<std::vector<T_or_RGB> > *results)
{

  // set up a scan line map
  T_or_RGB zero = T_or_RGB(0);
  //PixH2SV<T_or_RGB> zero = PixH2SV<T_or_RGB>(0);

  // this is how much to reduce each image by in fact for img pyramid
  const double rooty = sqrt(.5);

  // An operable copt of the image locations
  std::vector<Point2D<int> > currentLocation1;
  std::vector<Point2D<int> > currentLocation2;
  std::vector<Point2D<int> > *currentLocation;

  // A working copy of the current image
  Image<T_or_RGB> currentImage1 = *image;
  Image<T_or_RGB> currentImage2;
  Image<T_or_RGB> *currentImage;

  // Copy all pertinent locations over
  //LINFO("COPY LOCATIONS");

  std::vector<Point2D<int>*>::const_iterator ilocation = location->begin();

  for(unsigned int i = 0; i < *locations; i++, ++ilocation)
    {
      currentLocation1.push_back(**ilocation);
    }
  if(quarterSize)
    {
      Point2D<int> pp;
      currentImage2 = rescaleBilinear(currentImage1,
                              (int)floor(currentImage1.getWidth()*rooty),
                              (int)floor(currentImage1.getHeight()*rooty));
      currentLocation2.resize(currentLocation1.size(),pp);
      std::vector<Point2D<int> >::iterator icur1 =  currentLocation1.begin();
      std::vector<Point2D<int> >::iterator icur2 =  currentLocation2.begin();
      while(icur1 != currentLocation1.end())
        {
          icur2->i = (int)floor(icur1->i * rooty);
          icur2->j = (int)floor(icur1->j * rooty);
          ++icur1; ++icur2;
        }
    }

  int startx,starty;
  unsigned int endx,endy;

  Image<float>::const_iterator ker;
  typename Image<T_or_RGB>::iterator img;

  typename std::vector<std::vector<T_or_RGB> >::iterator iresults =
    results->begin();

  //LINFO("START");
  // iterate over each scale
  for(unsigned int sc = 0; sc < scales; sc++)
    {
      if((!quarterSize) || (sc%2 == 0) || (sc == 0))
        {
          currentLocation = &currentLocation1;
          currentImage    = &currentImage1;
        }
      else
        {
          currentLocation = &currentLocation2;
          currentImage    = &currentImage2;
        }

      const unsigned int w        = currentImage->getWidth();
      const unsigned int h        = currentImage->getHeight();
      unsigned int kstartx        = 0;
      unsigned int kstarty        = 0;

      for(std::vector<Image<float> >::const_iterator
            ifilters = filters->begin();
          ifilters != filters->end(); ++ifilters, ++iresults)
        {
          // iterate over each location

          const unsigned int kw       = ifilters->getWidth();
          const unsigned int kh       = ifilters->getHeight();
          const unsigned int kx       = (kw-1)/2;
          const unsigned int ky       = (kh-1)/2;

          typename std::vector<T_or_RGB>::iterator iiresults =
            iresults->begin();
          std::vector<Point2D<int> >::iterator icurrentLocation
            = currentLocation->begin();
          for(unsigned int l = 0; l < *locations; l++,
                ++icurrentLocation, ++iiresults)
            {
              startx = icurrentLocation->i - kx;
              if(startx < 0)
                { kstartx = abs(startx); startx = 0; }
              endx   = icurrentLocation->i + kx;
              if(endx >= w) { endx = w - 1; }

              starty = icurrentLocation->j - ky;
              if(starty < 0)
                { kstarty = abs(starty); starty = 0; }
              endy   = icurrentLocation->j + ky;
              if(endy >= h) { endy = h - 1; }

              float sum = 0;
              // use slow if the kernal runs off the image
              // otherwise use a slightly faster iterator incl

              for(unsigned int iy = starty; iy <= endy; iy++)
                {
                  // start at kernel offset for any edges
                  ker = ifilters->begin() +
                    (kw*(kstarty+iy-starty)) + kstartx;
                  // start in image offset for any edges
                  for(img  = currentImage->beginw() + w*iy + startx;
                      img != currentImage->beginw() + w*iy + endx; ++img, ++ker)
                    {
                      *iiresults += clamped_convert<T_or_RGB>((*img) * (*ker));
                      sum += fabs(*ker);
                    }
                }
              // normalize results
              if(normalize)
              {
                if(sum != 0)
                  {
                    *iiresults = clamped_convert<T_or_RGB>((*iiresults)/sum);
                  }
                else
                  {
                    *iiresults = zero;
                  }
              }
              kstarty = 0;
              kstartx = 0;
            }
        }

      // reduce the image using decimation and
      // adjust the coordinate selection accordingly
      *currentImage    = decXY(*currentImage);
      // note rooty = sqrt(.5);

      for(std::vector<Point2D<int> >::iterator icurrentLocation
            = currentLocation->begin();
          icurrentLocation != currentLocation->end();
          ++icurrentLocation)
        {
          *icurrentLocation = *icurrentLocation/2;
        }
    }
}

// ######################################################################
template <class T> inline
void multiScaleJunctionFilter(const unsigned int size,
                              const unsigned int scales,
                              const std::vector<std::vector<PixH2SV2<T> > >
                              *input,
                              std::vector<std::vector<PixH2SV2<T> > > *results)
{
  const PixH2SV2<T> zero(T(0));

  typename std::vector<std::vector<PixH2SV2<T> > >::const_iterator
    iinput1a = input->begin();
  typename std::vector<std::vector<PixH2SV2<T> > >::const_iterator
    iinput2a = input->begin()+1;
  typename std::vector<std::vector<PixH2SV2<T> > >::const_iterator
    iinput1b = input->begin()+2;
  typename std::vector<std::vector<PixH2SV2<T> > >::const_iterator
    iinput2b = input->begin()+3;
  typename std::vector<std::vector<PixH2SV2<T> > >::iterator
    iresults = results->begin();

  for(unsigned int s = 0; s < scales; s++)
    {
      typename std::vector<PixH2SV2<T> >::const_iterator
        iiinput1a = iinput1a->begin();
      typename std::vector<PixH2SV2<T> >::const_iterator
        iiinput2a = iinput2a->begin();
      typename std::vector<PixH2SV2<T> >::const_iterator
        iiinput1b = iinput1b->begin();
      typename std::vector<PixH2SV2<T> >::const_iterator
        iiinput2b = iinput2b->begin();
      typename std::vector<PixH2SV2<T> >::iterator iiresults = iresults->begin();

      PixH2SV2<T> max1a = PixH2SV2<T>(0); PixH2SV2<T> max2a = PixH2SV2<T>(0);
      PixH2SV2<T> max1b = PixH2SV2<T>(0); PixH2SV2<T> max2b = PixH2SV2<T>(0);

      for(unsigned int i = 0; i < size; i++,++iiinput1a)
        {
          max1a = maxmerge(*iiinput1a,max1a);
        }
      for(unsigned int i = 0; i < size; i++,++iiinput2a)
        {
          max2a = maxmerge(*iiinput2a,max2a);
        }
      for(unsigned int i = 0; i < size; i++,++iiinput1b)
        {
          max1b = maxmerge(*iiinput1b,max1b);
        }
      for(unsigned int i = 0; i < size; i++,++iiinput2b)
        {
          max2b = maxmerge(*iiinput2b,max2b);
        }

      iiinput1a = iinput1a->begin();
      iiinput2a = iinput2a->begin();
      iiinput1b = iinput1b->begin();
      iiinput2b = iinput2b->begin();

      const PixH2SV2<T> norm1( max1a + max1b );
      const PixH2SV2<T> norm2( max2a + max2b );


      if((norm1 != zero) && (norm2 != zero))
        {
          for(unsigned int i = 0; i < size; i++,
                ++iiinput1a,++iiinput2a,
                ++iiinput1b,++iiinput2b,
                ++iiresults)
            {
              const PixH2SV2<T> c1mix( abs((*iiinput1a)-(*iiinput1b))/(norm1) );
              const PixH2SV2<T> c2mix( abs((*iiinput2a)-(*iiinput2b))/(norm2) );
              // compute crossyness at location (strength of junction)
              const PixH2SV2<T> c1Gmix( ((*iiinput1a)+(*iiinput1b))/(norm1) );
              const PixH2SV2<T> c2Gmix( ((*iiinput2a)+(*iiinput2b))/(norm2) );
              // Indicates general intensity of the output
              const PixH2SV2<T> iOutA( (c1Gmix+c2Gmix)/2 );
              // Finish computing crossyness by subtracting lineyness
              // indicates the crispness of lines
              const PixH2SV2<T> iOutB( abs(c1mix-c2mix)/2 );
              // indicates the crispness of junctions
              *iiresults            = PixH2SV2<T>(iOutA - iOutB);
              // Include the explicit instantiations
            }
        }
      else
        {
          LINFO("WARNING - AT %d ALL VALUES ARE ZERO",s);
          for(unsigned int i = 0; i < size; i++,++iiresults)
            {
              *iiresults = zero;
            }
        }
      iinput1a = iinput1a + 4;
      iinput2a = iinput2a + 4;
      iinput1b = iinput1b + 4;
      iinput2b = iinput2b + 4;
      ++iresults;
    }
}

// ######################################################################
template <class T> inline
void multiScaleJunctionFilter(const unsigned int size,
                              const unsigned int scales,
                              const std::vector<std::vector<T> >
                              *input,
                              std::vector<std::vector<T> > *results)
{
  const T zero = T(0);

  typename std::vector<std::vector<T> >::const_iterator
    iinput1a = input->begin();
  typename std::vector<std::vector<T> >::const_iterator
    iinput2a = input->begin()+1;
  typename std::vector<std::vector<T> >::const_iterator
    iinput1b = input->begin()+2;
  typename std::vector<std::vector<T> >::const_iterator
    iinput2b = input->begin()+3;
  typename std::vector<std::vector<T> >::iterator
    iresults = results->begin();

  for(unsigned int s = 0; s < scales; s++)
    {
      typename std::vector<T>::const_iterator
        iiinput1a = iinput1a->begin();
      typename std::vector<T>::const_iterator
        iiinput2a = iinput2a->begin();
      typename std::vector<T>::const_iterator
        iiinput1b = iinput1b->begin();
      typename std::vector<T>::const_iterator
        iiinput2b = iinput2b->begin();
      typename std::vector<T>::iterator iiresults = iresults->begin();

      T max1a = T(0); T max2a = T(0);
      T max1b = T(0); T max2b = T(0);

      for(unsigned int i = 0; i < size; i++,++iiinput1a)
        {
          max1a = maxmerge(*iiinput1a,max1a);
        }
      for(unsigned int i = 0; i < size; i++,++iiinput2a)
        {
          max2a = maxmerge(*iiinput2a,max2a);
        }
      for(unsigned int i = 0; i < size; i++,++iiinput1b)
        {
          max1b = maxmerge(*iiinput1b,max1b);
        }
      for(unsigned int i = 0; i < size; i++,++iiinput2b)
        {
          max2b = maxmerge(*iiinput2b,max2b);
        }

      iiinput1a = iinput1a->begin();
      iiinput2a = iinput2a->begin();
      iiinput1b = iinput1b->begin();
      iiinput2b = iinput2b->begin();

      const T norm1 = max1a + max1b;
      const T norm2 = max2a + max2b;


      if((norm1 != 0) && (norm2 != 0))
        {
          for(unsigned int i = 0; i < size; i++,
                ++iiinput1a,++iiinput2a,
                ++iiinput1b,++iiinput2b,
                ++iiresults)
            {
              const T c1mix = T(fabs((*iiinput1a)-(*iiinput1b))/(norm1));
              const T c2mix = T(fabs((*iiinput2a)-(*iiinput2b))/(norm2));
              // compute crossyness at location (strength of junction)
              const T c1Gmix = (((*iiinput1a)+(*iiinput1b))/(norm1));
              const T c2Gmix = (((*iiinput2a)+(*iiinput2b))/(norm2));
              // Indicates general intensity of the output
              const T iOutA  = (c1Gmix+c2Gmix)/2;
              // Finish computing crossyness by subtracting lineyness
              // indicates the crispness of lines
              const T iOutB  = T(fabs(c1mix-c2mix)/2);
              // indicates the crispness of junctions
              *iiresults            = iOutA - iOutB;
              // Include the explicit instantiations
            }
        }
      else
        {
          LINFO("WARNING - AT %d ALL VALUES ARE ZERO",s);
          for(unsigned int i = 0; i < size; i++,++iiresults)
            {
              *iiresults = zero;
            }
        }
      iinput1a = iinput1a + 4;
      iinput2a = iinput2a + 4;
      iinput1b = iinput1b + 4;
      iinput2b = iinput2b + 4;
      ++iresults;
    }
}

#endif // !FEATURECLUSTERFILTERS_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
