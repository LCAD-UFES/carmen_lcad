/*!@file Neuro/EnvSegmenterCannyContour.C */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EnvSegmenterCannyContour.C $
// $Id: EnvSegmenterCannyContour.C 12782 2010-02-05 22:14:30Z irock $
//

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

#include "Neuro/EnvSegmenterCannyContour.H"

#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "GUI/DebugWin.H"

#ifdef HAVE_OPENCV

namespace
{
  const int thresh = 50;

  // helper function:
  // finds a cosine of angle between vectors
  // from pt0->pt1 and from pt0->pt2
  double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 )
  {
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
  }

  CvSeq* findSquares(const Image<PixRGB<byte> >& in, CvMemStorage* storage,
                  const int minarea, const int maxarea, const double mincos)
  {
    const int N = 11;

    IplImage* img = img2ipl(in);

    CvSize sz = cvSize( img->width & -2, img->height & -2 );
    IplImage* timg = cvCloneImage( img ); // make a copy of input image
    IplImage* gray = cvCreateImage( sz, 8, 1 );
    IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );
    // create empty sequence that will contain points -
    // 4 points per square (the square's vertices)
    CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );

    // select the maximum ROI in the image
    // with the width and height divisible by 2
    cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height ));

    // down-scale and upscale the image to filter out the noise
    cvPyrDown( timg, pyr, 7 );
    cvPyrUp( pyr, timg, 7 );
    IplImage* tgray = cvCreateImage( sz, 8, 1 );

    // find squares in every color plane of the image
    for (int c = 0; c < 3; ++c)
      {
        // extract the c-th color plane
        cvSetImageCOI( timg, c+1 );
        cvCopy( timg, tgray, 0 );

        // try several threshold levels
        for (int l = 0; l < N; ++l)
          {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
              {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cvCanny( tgray, gray, 0, thresh, 5 );
                // dilate canny output to remove potential
                // holes between edge segments
                cvDilate( gray, gray, 0, 1 );
              }
            else
              {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY );
              }

            // find contours and store them all as a list
            CvSeq* contours = 0;
            cvFindContours( gray, storage, &contours, sizeof(CvContour),
                            CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

            // test each contour
            while( contours )
              {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                CvSeq* result =
                  cvApproxPoly( contours, sizeof(CvContour), storage,
                                CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                const double area = fabs(cvContourArea(result,CV_WHOLE_SEQ));
                if (result->total == 4 &&
                    area >= minarea && area <= maxarea &&
                    cvCheckContourConvexity(result))
                  {
                    double s = 0;

                    for (int i = 0; i < 4; ++i)
                      {
                        // find minimum angle between joint
                        // edges (maximum of cosine)
                        const double t =
                          fabs(angle((CvPoint*)cvGetSeqElem( result, i % 4 ),
                                     (CvPoint*)cvGetSeqElem( result, (i-2) % 4 ),
                                     (CvPoint*)cvGetSeqElem( result, (i-1) % 4 )));
                        s = s > t ? s : t;
                      }


                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrangle
                    // vertices to resultant sequence
                    if (s < mincos)
                    {
                      for (int i = 0; i < 4; ++i)
                        cvSeqPush(squares,
                                  (CvPoint*)cvGetSeqElem( result, i ));
                    //  LINFO("area=%f, mincos=%f", area, s);
                    }
                  }

                // take the next contour
                contours = contours->h_next;
              }
          }
      }

    // release all the temporary images
    cvReleaseImage( &gray );
    cvReleaseImage( &pyr );
    cvReleaseImage( &tgray );
    cvReleaseImage( &timg );
    cvReleaseImageHeader( &img );

    return squares;
  }

  // the function draws all the squares in the image
  Image<PixRGB<byte> > drawSquares(const Image<PixRGB<byte> >&in, CvSeq* squares)
  {
    Image<PixRGB<byte> > out(in);

    CvSeqReader reader;

    // initialize reader of the sequence
    cvStartReadSeq(squares, &reader, 0);

    // read 4 sequence elements at a time (all vertices of a square)
    for (int i = 0; i < squares->total; i += 4)
      {
        CvPoint pt[4];

        // read 4 vertices
        CV_READ_SEQ_ELEM( pt[0], reader );
        CV_READ_SEQ_ELEM( pt[1], reader );
        CV_READ_SEQ_ELEM( pt[2], reader );
        CV_READ_SEQ_ELEM( pt[3], reader );

        for (int j = 0; j < 4; ++j)
          drawLine(out,
                   Point2D<int>(pt[j].x, pt[j].y),
                   Point2D<int>(pt[(j+1)%4].x, pt[(j+1)%4].y),
                   PixRGB<byte>(0, 255, 0),
                   2);
      }

    return out;
  }

  Rectangle getRectangle(CvSeq* cards)
  {
    CvSeqReader reader;
    // initialize reader of the sequence
    cvStartReadSeq( cards, &reader, 0 );

    Rectangle result;

    if (cards->total > 0)
      {
        CvPoint pt[4];
        // read 4 vertices
        CV_READ_SEQ_ELEM( pt[0], reader );
        CV_READ_SEQ_ELEM( pt[1], reader );
        CV_READ_SEQ_ELEM( pt[2], reader );
        CV_READ_SEQ_ELEM( pt[3], reader );

        //Find the bounding box
        Point2D<int> tl(pt[0].x, pt[0].y), br(pt[0].x, pt[0].y);
        for(int i=1; i<4; i++)
          {
            if (pt[i].x < tl.i) tl.i = pt[i].x;
            else if (pt[i].x > br.i) br.i = pt[i].x;

            if (pt[i].y < tl.j) tl.j = pt[i].y;
            else if (pt[i].y > br.j) br.j = pt[i].y;
          }
        tl.i -= 10; tl.j -= 10;
        br.i += 10; br.j += 10;

        result = Rectangle::tlbrO(tl.j, tl.i, br.j, br.i);
      }

    return result;
  }
}

#endif //HAVE_OPENCV

// ######################################################################
EnvSegmenterCannyContour::EnvSegmenterCannyContour(OptionManager& mgr)
  :
  EnvSegmenter(mgr, "Embeddable Canny Contour FOA Segmenter",
               "EnvSegmenterCannyContour"),
  itsMinArea("CannyMinArea", this, 500, ALLOW_ONLINE_CHANGES),
  itsMaxArea("CannyMaxArea", this, 7000, ALLOW_ONLINE_CHANGES),
  itsMinCos("CannyMinCos", this, 0.1, ALLOW_ONLINE_CHANGES)
#ifdef HAVE_OPENCV
  ,
  itsStorage(cvCreateMemStorage(0))
#endif
{
#ifndef HAVE_OPENCV
  LFATAL("OpenCV must be installed in order to use this function");
#else
  ASSERT(itsStorage != 0);
#endif
}

// ######################################################################
EnvSegmenterCannyContour::~EnvSegmenterCannyContour()
{
#ifndef HAVE_OPENCV
  LERROR("OpenCV must be installed in order to use this function");
#else
  cvReleaseMemStorage(&itsStorage);
#endif
}

// ######################################################################
Rectangle EnvSegmenterCannyContour::getFoa(const Image<PixRGB<byte> >& rgbin,
                                           const Point2D<int>& center,
                                           Image<byte>* foamask,
                                           Image<PixRGB<byte> >* segmentdisp) const
{
#ifndef HAVE_OPENCV
  LFATAL("OpenCV must be installed in order to use this function");
  return Rectangle();
#else

  CvSeq* cards = findSquares(rgbin, itsStorage, itsMinArea.getVal(), itsMaxArea.getVal(), itsMinCos.getVal());

  const Rectangle result = getRectangle(cards);

  const Image<PixRGB<byte> > out = drawSquares(rgbin, cards);

  if (foamask)
    {
      *foamask = Image<byte>(rgbin.getDims(), ZEROS);

      if (result.isValid())
        inplaceClearRegion(*foamask, result, byte(255));
      else
        foamask->setVal(center, 255);
    }

  if (segmentdisp)
    *segmentdisp = out;

  cvClearMemStorage(itsStorage);

  return result.getOverlap(rgbin.getBounds());
#endif
}

// ######################################################################
std::vector<Rectangle> EnvSegmenterCannyContour::getSquares(const Image<PixRGB<byte> >& rgbin, Image<PixRGB<byte> >* segmentDisp)
{
#ifndef HAVE_OPENCV
  LFATAL("OpenCV must be installed in order to use this function");
  return std::vector<Rectangle>();
#else

  CvSeq* squares = findSquares(rgbin, itsStorage, itsMinArea.getVal(), itsMaxArea.getVal(), itsMinCos.getVal());


  if (segmentDisp)
    *segmentDisp = drawSquares(rgbin, squares);

  //const Rectangle square = getRectangle(squares);
  std::vector<Rectangle> results;


  CvSeqReader reader;
  // initialize reader of the sequence
  cvStartReadSeq(squares, &reader, 0);

  // read 4 sequence elements at a time (all vertices of a square)
  for (int i = 0; i < squares->total; i += 4)
  {
    CvPoint pt[4];

    // read 4 vertices
    CV_READ_SEQ_ELEM( pt[0], reader );
    CV_READ_SEQ_ELEM( pt[1], reader );
    CV_READ_SEQ_ELEM( pt[2], reader );
    CV_READ_SEQ_ELEM( pt[3], reader );

    //Find the bounding box
    Point2D<int> tl(pt[0].x, pt[0].y), br(pt[0].x, pt[0].y);
    for(int i=1; i<4; i++)
    {
      if (pt[i].x < tl.i) tl.i = pt[i].x;
      else if (pt[i].x > br.i) br.i = pt[i].x;

      if (pt[i].y < tl.j) tl.j = pt[i].y;
      else if (pt[i].y > br.j) br.j = pt[i].y;
    }
    tl.i -= 10; tl.j -= 10;
    br.i += 10; br.j += 10;
    Rectangle rect  = Rectangle::tlbrO(tl.j, tl.i, br.j, br.i);
    results.push_back(rect.getOverlap(rgbin.getBounds()));
  }

  cvClearMemStorage(itsStorage);

  return results;
#endif
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
