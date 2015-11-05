/*!@file Gist/test-SuperPixel.C testing SuperPixel segmentation algorithm */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/SuperPixel.H $
// $Id: SuperPixel.H 15342 2012-07-24 22:41:24Z kai $
//
//////////////////////////////////////////////////////////////////////////
//
// Implementation of the segmentation algorithm described in:
//
// Efficient Graph-Based Image Segmentation
// Pedro F. Felzenszwalb and Daniel P. Huttenlocher
// International Journal of Computer Vision, 59(2) September 2004.
// Copyright (C) 2006 Pedro Felzenszwalb

#ifndef SUPERPIXEL_SEGMENT_IMAGE_H
#define SUPERPIXEL_SEGMENT_IMAGE_H

#include <cstdlib>

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Image/MathOps.H"
#include "Image/ColorOps.H"
#include "Image/Point3D.H"

#include "Gist/SuperPixel_helper.H"

#include "Util/Timer.H"

#include <map>

// random color
inline PixRGB<byte> random_rgb(){ 
  PixRGB<byte> c((byte)random(),
                 (byte)random(),
                 (byte)random());
  return c;
}

// dissimilarity measure between pixelsRGB
static inline float 
diff(Image<float> r, Image<float> g, Image<float> b,
     int x1, int y1, int x2, int y2) 
{
  return sqrt(pow(r.getVal(x1, y1) - r.getVal(x2, y2), 2.0) +
              pow(g.getVal(x1, y1) - g.getVal(x2, y2), 2.0) +
              pow(b.getVal(x1, y1) - b.getVal(x2, y2), 2.0));
}
// dissimilarity measure between pixels (multiple channels)
static inline float 
diff(std::vector<Image<float> > channels,
     int x1, int y1, int x2, int y2) 
{
  float difference = 0.0;
  std::vector<Image<float> >::iterator 
    it = channels.begin(), 
    stop = channels.end();
  for (;it!=stop; ++it) 
    {
      Image<float> ch = *it;
      difference+= pow(ch.getVal(x1, y1) - ch.getVal(x2, y2), 2.0) ;
    }
  
  return sqrt(difference);
}

// dissimilarity measure between pixels in 3D
static inline float 
diff(std::vector<std::vector<Image<float> > >channels3d,
     int x1, int y1, int z1, int x2, int y2, int z2) 
{
  float difference = 0.0;
  std::vector<Image<float> > channel1 = channels3d[z1];	
  std::vector<Image<float> > channel2 = channels3d[z2];	


  std::vector<Image<float> >::iterator 
    it = channel1.begin(), 
    stop = channel1.end();
  std::vector<Image<float> >::iterator 
    it2 = channel2.begin(); 
  for (;it!=stop; ++it,++it2) 
    {
      Image<float> ch = *it;
      Image<float> ch2 = *it2;
      difference+= pow(ch.getVal(x1, y1) - ch2.getVal(x2, y2), 2.0) ;
    }
  
  return sqrt(difference);
}



//! Segment an image using Super Pixel Segmentation.
/*!
 * Returns an integer image where the value at each pixel denotes the
 * group it belongs to.  Additionally, if a groups vector is supplied,
 * it will be filled with vectors of points belonging to each group.
 *
 * \param im image to segment.
 * \param sigma to smooth the image.
 * \param c constant for treshold function.
 * \param min_size minimum component size (enforced by post-processing stage).
 * \param num_ccs number of connected components in the segmentation.
 * \param channels :instead use default RGB channels, it takes all channels as graph base 
 *
 */

// ######################################################################
inline Image<int> SuperPixelSegment
(Image<PixRGB<byte> > im, 
 float sigma, float c, int min_size, int &num_ccs, 
 std::vector<std::vector<Point2D<int> > > *groups , 
 std::vector<Image<float> > channels )
{
  //smooth out each channel
  std::vector<Image<float> > smooth_channels;
  std::vector<Image<float> >::iterator 
    it = channels.begin(), 
    stop = channels.end();
  for (;it!=stop; ++it) {
	  Image<float> ch = *it;
	  Image<float> smooth_temp = smooth(ch, sigma);
	  smooth_channels.push_back(smooth_temp);
  }
 
  int width  = im.getWidth();
  int height = im.getHeight();

  // build graph
  edge *edges = new edge[width*height*4];
  int num = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (x < width-1) {
        edges[num].a = y * width + x;
        edges[num].b = y * width + (x+1);
        edges[num].w = diff(smooth_channels, x, y, x+1, y);
        num++;
      }

      if (y < height-1) {
        edges[num].a = y * width + x;
        edges[num].b = (y+1) * width + x;
        edges[num].w = diff(smooth_channels, x, y, x, y+1);
        num++;
      }

      if ((x < width-1) && (y < height-1)) {
        edges[num].a = y * width + x;
        edges[num].b = (y+1) * width + (x+1);
        edges[num].w = diff(smooth_channels, x, y, x+1, y+1);
        num++;
      }

      if ((x < width-1) && (y > 0)) {
        edges[num].a = y * width + x;
        edges[num].b = (y-1) * width + (x+1);
        edges[num].w = diff(smooth_channels, x, y, x+1, y-1);
        num++;
      }
    }
  }

  // segment
  universe *u = segment_graph(width*height, num, edges, c);

  // post process small components
  for (int i = 0; i < num; i++) {
    int a = u->find(edges[i].a);
    int b = u->find(edges[i].b);
    if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
      u->join(a, b);
  }
  delete [] edges;
  num_ccs = u->num_sets();

  Image<int> groupImage(width, height, NO_INIT);

  // If a group vector is provided, then let's resize it and fill it along
  // with the output image
  groups->resize(u->num_sets());
  
  // The group numbers created by the algorithm are more or less random. We
  // need to compress them so we can easily insert the group points into
  // vectors.
  std::map<int, int> groupLabels; 
  int labelCount = 0;
  for (int y = 0; y < height; y++) 
    for (int x = 0; x < width; x++) 
      {
        int groupUnorderedId = u->find(y * width + x);
        
        // If we haven't seen this group before, 
        // create a new ordered label for it
        if(groupLabels.find(groupUnorderedId) == groupLabels.end())
          groupLabels[groupUnorderedId] = labelCount++;
        
        int groupOrderedId = groupLabels[groupUnorderedId];
        
        if(groups != NULL)
          (*groups)[groupOrderedId].push_back(Point2D<int>(x,y));
        groupImage.setVal(x, y, groupOrderedId);
      }
  
  delete u;
  
  return groupImage;
}

// ######################################################################
inline Image<int> SuperPixelSegment
(Image<PixRGB<byte> > im, 
 float sigma, float c, int min_size, int &num_ccs, 
 std::vector<std::vector<Point2D<int> > > *groups,bool useLAB = false)
{
  std::vector<Image<float> > channels;

	if(useLAB)
	{
		LINFO("Using LAB Color Space");
		//Image<float> l, a, b; getNormalizedLAB(im, l, a, b);
		Image<float> l, a, b; getLAB(im, l, a, b);
		channels.push_back(l);	
		channels.push_back(a);	
		channels.push_back(b);	

	}else
	{
		Image<byte> r, g, b; getComponents(im, r, g, b);
		channels.push_back(r);	
		channels.push_back(g);	
		channels.push_back(b);	
	}


  return SuperPixelSegment
    (im,sigma,c,min_size,num_ccs,groups,channels);
}

// ######################################################################
inline Image<int> SuperPixelSegment3D
(Image<PixRGB<byte> > im1,
Image<PixRGB<byte> > im2,
 
 float sigma, float c, int min_size, int &num_ccs, 
 std::vector<std::vector<Point3D<int> > > *groups , 
 std::vector<std::vector<Image<float> > > channels3D )
{
  //smooth out each channel
  std::vector<std::vector<Image<float> > > smooth_channels3D;
  std::vector<std::vector<Image<float> > > ::iterator 
    it3D = channels3D.begin(), 
    stop3D = channels3D.end();
  for (;it3D!=stop3D; ++it3D) {


	std::vector<Image<float> > smooth_channels;
	std::vector<Image<float> > chs = *it3D;
        std::vector<Image<float> > ::iterator 
		it = chs.begin(), 
		   stop = chs.end();
  	for (;it!=stop; ++it) {

	  Image<float> ch = *it;
	  Image<float> smooth_temp = smooth(ch, sigma);
	  smooth_channels.push_back(smooth_temp);
	}

	smooth_channels3D.push_back(smooth_channels);
  }
 
  int width  = im1.getWidth();
  int height = im1.getHeight();
  int layers = (int)smooth_channels3D.size();
  // build graph
  edge *edges = new edge[width*height*4*layers];
  int num = 0;
  int z1 = 0;
for(int z = 0; z < layers;z++){

	//First graph layer is flat (x,y,0) to (x,y,0)
	//Second and more graph layer is (x,y,z-1) to (x,y,z)
	if(z > 0)
		z1 = z -1;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

    	      

      if (x < width-1) {
        edges[num].a = z1 * width * height + y * width + x;
        edges[num].b = z * width * height + y * width + (x+1);
        edges[num].w = diff(smooth_channels3D, x, y, z1 , x+1, y, z);
        num++;
      }

      if (y < height-1) {
        edges[num].a = z1 * width * height + y * width + x;
        edges[num].b = z * width * height +(y+1) * width + x;
        edges[num].w = diff(smooth_channels3D, x, y, z1, x, y+1, z);
        num++;
      }

      if ((x < width-1) && (y < height-1)) {
        edges[num].a = z1 * width * height + y * width + x;
        edges[num].b = z * width * height + (y+1) * width + (x+1);
        edges[num].w = diff(smooth_channels3D, x, y, z1, x+1, y+1, z);
        num++;
      }

      if ((x < width-1) && (y > 0)) {
        edges[num].a = z1 * width * height + y * width + x;
        edges[num].b = z * width * height + (y-1) * width + (x+1);
        edges[num].w = diff(smooth_channels3D, x, y, z1, x+1, y-1, z);
        num++;
      }
    }
  }
}
  // segment
  universe *u = segment_graph(width*height*layers, num, edges, c);

  // post process small components
  for (int i = 0; i < num; i++) {
    int a = u->find(edges[i].a);
    int b = u->find(edges[i].b);
    if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
      u->join(a, b);
  }
  delete [] edges;
  num_ccs = u->num_sets();

  Image<int> groupImage(width, height, NO_INIT);

  // If a group vector is provided, then let's resize it and fill it along
  // with the output image
  groups->resize(u->num_sets());
  
  // The group numbers created by the algorithm are more or less random. We
  // need to compress them so we can easily insert the group points into
  // vectors.
  std::map<int, int> groupLabels; 
  int labelCount = 0;
  
 for (int z = 0; z < layers; z++) 
  for (int y = 0; y < height; y++) 
    for (int x = 0; x < width; x++) 
      {
        int groupUnorderedId = u->find(z*width*height + y * width + x);
        
        // If we haven't seen this group before, 
        // create a new ordered label for it
        if(groupLabels.find(groupUnorderedId) == groupLabels.end())
          groupLabels[groupUnorderedId] = labelCount++;
        
        int groupOrderedId = groupLabels[groupUnorderedId];
        
        if(groups != NULL)
          (*groups)[groupOrderedId].push_back(Point3D<int>(x,y,z));
        groupImage.setVal(x, y, groupOrderedId);
      }
  
  delete u;
  
  return groupImage;
}
// ######################################################################
inline Image<int> SuperPixelSegmentRGB3D
(
Image<PixRGB<byte> > im1, 
Image<PixRGB<byte> > im2, 
 float sigma, float c, int min_size, int &num_ccs, 
 std::vector<std::vector<Point3D<int> > > *groups)
{


  std::vector<std::vector<Image<float> > >channel3D;

  Image<byte> r, g, b; getComponents(im1, r, g, b);

  std::vector<Image<float> > channel1;
  channel1.push_back(r);	
  channel1.push_back(g);	
  channel1.push_back(b);	
	
channel3D.push_back(channel1);

  Image<byte> r2, g2, b2; getComponents(im2, r2, g2, b2);

  std::vector<Image<float> > channel2;
  channel2.push_back(r2);	
  channel2.push_back(g2);	
  channel2.push_back(b2);	

  channel3D.push_back(channel2);

  return SuperPixelSegment3D
    (im1,im2,sigma,c,min_size,num_ccs,groups,channel3D);
}
// ######################################################################
namespace
{ std::vector<PixRGB<byte> > groupColors; }

// ######################################################################
inline Image<int> SuperPixelRegionSizeImage 
(std::vector<std::vector<Point2D<int> > > const& groups, Image<int> groupImage)
{
	Image<int> debugSizeImage(groupImage.getDims(), NO_INIT);

  for(size_t grpIdx=0; grpIdx < groups.size(); grpIdx++)
  {
    int regionSize =  (int) groups[grpIdx].size();
    for(size_t pntIdx=0; pntIdx<groups[grpIdx].size(); pntIdx++)
      debugSizeImage.setVal(groups[grpIdx][pntIdx], regionSize);
  }
  
  return debugSizeImage;
}

// ######################################################################
inline Image<PixRGB<byte> > SuperPixelDebugImage(Image<int> groupImage)
{
  // pick random colors for each component
  if(groupColors.size() == 0)
    {
      groupColors.resize(255);
      std::vector<PixRGB<byte> >::iterator 
        it = groupColors.begin(), 
        stop = groupColors.end();
      for (;it!=stop; ++it) *it = random_rgb();
    }
  
  Image<PixRGB<byte> > debugImage(groupImage.getDims(), NO_INIT);
  
  for(size_t i=0; i<groupImage.size(); ++i)
    debugImage.setVal
      (i,groupColors[groupImage.getVal(i)%groupColors.size()]);
  
  return debugImage;
}

// ######################################################################
inline Image<PixRGB<byte> > SuperPixelDebugImage
(std::vector<std::vector<Point2D<int> > > const& groups, 
 Image<PixRGB<byte> > originalImage)
{
  Image<PixRGB<byte> > debugImage(originalImage.getDims(), NO_INIT);
  
  for(size_t grpIdx=0; grpIdx < groups.size(); grpIdx++)
  {
    PixRGB<long> avgColor(0,0,0);
    for(size_t pntIdx=0; pntIdx<groups[grpIdx].size(); pntIdx++)
      avgColor += originalImage.getVal(groups[grpIdx][pntIdx]);
    avgColor /= groups[grpIdx].size();
    for(size_t pntIdx=0; pntIdx<groups[grpIdx].size(); pntIdx++)
      debugImage.setVal(groups[grpIdx][pntIdx], avgColor);
  }
  
  return debugImage;
}

// ######################################################################
inline Image<PixRGB<byte> > SuperPixelDebugImage
(std::vector<std::vector<Point3D<int> > > const& groups, 
 Image<PixRGB<byte> > originalImage)
{
  Image<PixRGB<byte> > debugImage(originalImage.getDims(), NO_INIT);
  
  for(size_t grpIdx=0; grpIdx < groups.size(); grpIdx++)
  {
    PixRGB<long> avgColor(0,0,0);
    for(size_t pntIdx=0; pntIdx<groups[grpIdx].size(); pntIdx++)
      avgColor += originalImage.getVal(groups[grpIdx][pntIdx].x,groups[grpIdx][pntIdx].y);
    avgColor /= groups[grpIdx].size();
    for(size_t pntIdx=0; pntIdx<groups[grpIdx].size(); pntIdx++)
      debugImage.setVal(groups[grpIdx][pntIdx].x,groups[grpIdx][pntIdx].y, avgColor);
  }
  
  return debugImage;
}
/*
 * Segment an image
 *
 * Returns a color image representing the segmentation.
 *
 * im: image to segment.
 * sigma: to smooth the image.
 * c: constant for treshold function.
 * min_size: minimum component size (enforced by post-processing stage).
 * num_ccs: number of connected components in the segmentation.


 :::::NOTE::::: This method is now deprecated. 
 Please use SuperPixelSegment, and SuperPixelDebugImage instead.

 */
// ######################################################################
inline Image<PixRGB<byte> > segment_image
(Image<PixRGB<byte> > im, float sigma, float c, int min_size,
 int &num_ccs)
{
	//LINFO("This method is now deprecated. "
        //      "Please use SuperPixelSegment "
        //      "and SuperPixelDebugImage instead.");
	std::vector<std::vector<Point2D<int> > > groups;
	Image<int> groupImage = 
          SuperPixelSegment
          (im, sigma, c, min_size, num_ccs, &groups);
	//return SuperPixelDebugImage(groups,im);
	return SuperPixelDebugImage(groupImage);
}

// ######################################################################
inline Image<PixRGB<byte> > segment_image3D
(Image<PixRGB<byte> > im, float sigma, float c, int min_size,
 int &num_ccs)
{
	LINFO("This method is now deprecated. "
              "Please use SuperPixelSegment "
              "and SuperPixelDebugImage instead.");
	std::vector<std::vector<Point3D<int> > > groups;
	Image<int> groupImage = 
          SuperPixelSegmentRGB3D
          (im,im, sigma, c, min_size, num_ccs, &groups);
	return SuperPixelDebugImage(groups,im);
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
