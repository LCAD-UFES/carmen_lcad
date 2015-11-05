/*!@file Gist/Clustering.C various clustering techniques */
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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/Clustering.C $
// $Id: test-KMeans.C 14770 2011-05-19 04:19:46Z kai $
//
//////////////////////////////////////////////////////////////////////////

//#include "Image/OpenCVUtil.H"
#include "Gist/Clustering.H"

// K-means parameters
#ifndef TT_KMEANS_ITERATIONS
   #define TT_KMEANS_ITERATIONS (10)
#endif
#ifndef TT_KMEANS_PRECISION
   #define TT_KMEANS_PRECISION (1)
#endif


// ######################################################################
void MAT_ELEM(CvMat *mat, int x, int y, int channel, int val)   
{   
    ((float*)(mat->data.ptr+mat->step*x))[y*mat->step+channel]= val;     
}

// ######################################################################
//img : input image in RGB space
//K   : number of groups create by kmean
//dist: distance weight when doing the segment, higher weight will make
//      each region group by its neighbor pixel
// ######################################################################
Image<PixRGB<byte> > getKMeans(Image<PixRGB<byte> > img,int K,float dist)
{
  //convert iNVT image to cvImage
  IplImage *cvImg = img2ipl(img);

  int h = img.getHeight();
  int w = img.getWidth();
  LINFO("Image Width %d Height %d cv W %d,H %d",w,h,cvImg->width,cvImg->height);
  CvMat *sample  = cvCreateMat(w*h, 1, CV_32FC(5));   
  //CvMat *sample  = cvCreateMat(w*h, 1, CV_32FC(3));   
  CvMat *cluster = cvCreateMat(w*h, 1, CV_32SC1);   
  for(int y = 0; y < h; y++)
    { 
      for(int x = 0; x < w; x++)
        {     
          int idx= y*w+x;   
          int idxpix= y*w*3+x*3;   
          MAT_ELEM(sample,idx,0,0,x*dist);   
          MAT_ELEM(sample,idx,0,1,y*dist);   
          MAT_ELEM(sample,idx,0,2, *(cvImg->imageData + idxpix + 0));
          MAT_ELEM(sample,idx,0,3, *(cvImg->imageData + idxpix + 1));
          MAT_ELEM(sample,idx,0,4, *(cvImg->imageData + idxpix + 2));
        }   
    }   

  //Doing cvKmean
  cvKMeans2(sample, K, cluster,
            cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
                           TT_KMEANS_ITERATIONS, TT_KMEANS_PRECISION)) ;

  IplImage 	*dst = cvCreateImage(cvGetSize(cvImg),8,3);   
  cvZero(dst);   

  std::vector<std::vector<Point2D<int> > >  groups;
  groups.resize(K);

  // Put Pixel Color to each labeled bin
  for(int y = 0; y < h; y++)
    {   
      for(int x = 0; x < w; x++)
        {   
          int idx = cluster->data.i[y*w+x];   
          groups[idx].push_back(Point2D<int>(x,y));
        }   
    }
   
  // Given a int label map, we will create a average color map
  Image<PixRGB<byte> > output(img.getDims(), ZEROS);

  //Compute avg color for each region
  for(size_t grpIdx=0; grpIdx < groups.size(); grpIdx++)
    {
      //Compute Average Color
      PixRGB<long> avgColor(0,0,0);
      for(size_t pntIdx=0; pntIdx<groups[grpIdx].size(); pntIdx++)
        avgColor +=  img.getVal(groups[grpIdx][pntIdx]);

      if(groups[grpIdx].size() != 0)
        avgColor /= groups[grpIdx].size();

      //Asign avg color to region pixels
      for(size_t pntIdx=0; pntIdx<groups[grpIdx].size(); pntIdx++)
        output.setVal(groups[grpIdx][pntIdx],avgColor);
    }

  cvReleaseMat(&sample);   
  cvReleaseMat(&cluster);         
  cvReleaseImage(&cvImg);  
  return output;	
}

// ######################################################################
Image<int> getKMeansLabel(Image<PixRGB<byte> > img,int K,float dist)
{
  //convert iNVT image to cvImage
  IplImage *cvImg = img2ipl(img);

  int h = img.getHeight();
  int w = img.getWidth();
  LINFO("Image Width %d Height %d cv W %d,H %d",w,h,cvImg->width,cvImg->height);
  CvMat *sample  = cvCreateMat(w*h, 1, CV_32FC(5));   
  //CvMat *sample  = cvCreateMat(w*h, 1, CV_32FC(3));   
  CvMat *cluster = cvCreateMat(w*h, 1, CV_32SC1);   
  for(int y = 0; y < h; y++)
    { 
      for(int x = 0; x < w; x++)
        {     
          int idx= y*w+x;   
          int idxpix= y*w*3+x*3;   
          MAT_ELEM(sample,idx,0,0,x*dist);   
          MAT_ELEM(sample,idx,0,1,y*dist);   
          MAT_ELEM(sample,idx,0,2, *(cvImg->imageData + idxpix + 0));
          MAT_ELEM(sample,idx,0,3, *(cvImg->imageData + idxpix + 1));
          MAT_ELEM(sample,idx,0,4, *(cvImg->imageData + idxpix + 2));
        }   
    }   

  //Doing cvKmean
  cvKMeans2(sample, K, cluster,
            cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
                           TT_KMEANS_ITERATIONS, TT_KMEANS_PRECISION)) ;

  IplImage 	*dst = cvCreateImage(cvGetSize(cvImg),8,3);   
  cvZero(dst);   

  std::vector<std::vector<Point2D<int> > >  groups;
  groups.resize(K);

  // Put Pixel Color to each labeled bin
  for(int y = 0; y < h; y++)
    {   
      for(int x = 0; x < w; x++)
        {   
          int idx = cluster->data.i[y*w+x];   
          groups[idx].push_back(Point2D<int>(x,y));
        }   
    }
   
  // Given a int label map, we will create a average color map
  Image<int> output(img.getDims(), ZEROS);

  //Compute avg color for each region
  for(size_t grpIdx=0; grpIdx < groups.size(); grpIdx++)
    {
      //Asign avg color to region pixels
      for(size_t pntIdx=0; pntIdx<groups[grpIdx].size(); pntIdx++)
        output.setVal(groups[grpIdx][pntIdx], int(grpIdx));
    }

  cvReleaseMat(&sample);   
  cvReleaseMat(&cluster);         
  cvReleaseImage(&cvImg);  
  return output;	
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

