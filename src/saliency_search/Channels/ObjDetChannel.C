/*!@file Channels/ObjDetChannel.C object detection channel using opecv cascade detector */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ObjDetChannel.C $
// $Id: ObjDetChannel.C 12821 2010-02-11 07:15:07Z itti $
//

#include "Channels/ObjDetChannel.H"
#include "Image/DrawOps.H"
#include "Image/Kernels.H"  // for gaussianBlob()
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Channels/ChannelOpts.H"
#include "Component/ModelOptionDef.H"
#include "Component/GlobalOpts.H"

static const ModelOptionDef OPT_CascadeFilePath =
  { MODOPT_ARG_STRING, "Cascade file path", &MOC_CHANNEL, OPTEXP_CORE,
    "Name of directory containing the description of a trained cascade classifier."
    "Used in making faces salient or any other object.  ",
    "cascade-file", '\0', "<filename>.xml",
    //"/usr/local/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml"};
    "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml"};

ObjDetChannel::ObjDetChannel(OptionManager& mgr, const std::string & descrName,
                             const std::string& tagName) :
  ChannelBase(mgr, descrName, tagName, FACE),
  itsMap(),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsCascadeFile(&OPT_CascadeFilePath, this),
  itsNormType(&OPT_MaxNormType, this), // see Channels/ChannelOpts.{H,C}
  itsOutputRangeMin(&OPT_ChannelOutputRangeMin, this),
  itsOutputRangeMax(&OPT_ChannelOutputRangeMax, this),
  itsUseRandom(&OPT_UseRandom, this),
  itsNormalizeOutput("SingleChannelNormalizeOutput", this, false)
{

#ifdef HAVE_OPENCV
  cascade = (CvHaarClassifierCascade*)cvLoad( itsCascadeFile.getVal().c_str(), 0, 0, 0 );

  if( !cascade )
    LFATAL("ERROR: Could not load classifier cascade (%s)\n", itsCascadeFile.getVal().c_str() );
  storage = cvCreateMemStorage(0);
#else
  LFATAL("OpenCV is needed for ObjDet channel");
#endif


}

// ######################################################################
ObjDetChannel::~ObjDetChannel()
{  }

// ######################################################################
bool ObjDetChannel::outputAvailable() const
{ return itsMap.initialized(); }

// ######################################################################
uint ObjDetChannel::numSubmaps() const
{
  return 1;
}

// ######################################################################
Dims ObjDetChannel::getMapDims() const
{
  if (!this->hasInput())
    LFATAL("Oops! I haven't received any input yet");

  const Dims indims = this->getInputDims();

  return Dims(indims.w() >> itsLevelSpec.getVal().mapLevel(),
              indims.h() >> itsLevelSpec.getVal().mapLevel());

}

// ######################################################################
void ObjDetChannel::getFeatures(const Point2D<int>& locn,
                              std::vector<float>& mean) const
{
  if (!this->outputAvailable())
    { CLDEBUG("I have no input yet -- RETURNING ZEROS"); mean.push_back(0.0F); return; }

  // The coordinates we receive are at the scale of the original
  // image, and we will need to rescale them to the size of the
  // various submaps we read from. The first image in our first
  // pyramid has the dims of the input:
  const Dims indims = this->getInputDims();
  mean.push_back(itsMap.getValInterpScaled(locn, indims));
}

// ######################################################################
void ObjDetChannel::getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                     std::vector<std::vector<float> > *mean,
                                     int *count) const
{
  if (!this->outputAvailable())
    {
      CLDEBUG("I have no input yet -- RETURNING ZEROS");
      std::vector<std::vector<float> >::iterator imean = mean->begin();
      for (int i = 0; i < *count; i++, ++imean) imean->push_back(0.0);
      return;
    }

  // The coordinates we receive are at the scale of the original
  // image, and we will need to rescale them to the size of the
  // various submaps we read from. The first image in our first
  // pyramid has the dims of the input:
  const Dims indims = this->getInputDims();

  std::vector<Point2D<int>*>::iterator ilocn = locn->begin();
  std::vector<std::vector<float> >::iterator imean = mean->begin();

  for (int i = 0; i < *count; ++i, ++ilocn, ++imean)
    imean->push_back(itsMap.getValInterpScaled(**ilocn, indims));
}

// ######################################################################
void ObjDetChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.grayFloat().initialized());
  Image<byte> lum = inframe.grayFloat();

#ifdef HAVE_OPENCV
  const double scale = 1.3;
  IplImage* small_img =
    cvCreateImage(cvSize(cvRound(lum.getWidth() / scale), cvRound(lum.getHeight() / scale)), 8, 1 );

  cvResize(img2ipl(lum), small_img, CV_INTER_LINEAR);
  cvEqualizeHist(small_img, small_img);
  cvClearMemStorage(storage);

  if (cascade)
    {
      double t = double(cvGetTickCount());
      CvSeq* objects = cvHaarDetectObjects(small_img, cascade, storage,
                                           1.1, 2, 0/*CV_HAAR_DO_CANNY_PRUNING*/,
                                           cvSize(30, 30));
      t = double(cvGetTickCount()) - t;
      LDEBUG( "detection time = %gms", t / (double(cvGetTickFrequency())*1000.0));

      itsMap = Image<float>(lum.getDims(), ZEROS);
      for (int i = 0; i < (objects ? objects->total : 0); ++i )
        {
          CvRect* r = (CvRect*)cvGetSeqElem(objects, i);

          Rectangle objRect(Point2D<int>(int(r->x*scale), int(r->y*scale)),
                            Dims(int(r->width*scale), int(r->height*scale)));

          const Point2D<int> objCenter = Point2D<int>(objRect.topLeft().i + objRect.width()/2,
                                                      objRect.topLeft().j + objRect.height()/2);

          Image<float> objBlob =
            gaussianBlobUnnormalized<float>(lum.getDims(), objCenter,
                                            float(objRect.width())/2, float(objRect.height())/2);

          itsMap += objBlob;
        }
    }

  inplaceRectify(itsMap); // eliminate any possible negative values (from rounding)
  itsMap = rescale(itsMap, this->getMapDims()); // scale to final dims

  // normalize range and add background noise if desired:
  inplaceNormalize(itsMap, 0.0F, 255.0F);
  if (itsUseRandom.getVal()) inplaceAddBGnoise(itsMap, 255.0F);

  // apply max-normalization on the output as needed:
  if (itsNormalizeOutput.getVal())
    {
      LDEBUG("%s: Normalizing output: %s(%f .. %f)", tagName().c_str(),
             maxNormTypeName(itsNormType.getVal()), itsOutputRangeMin.getVal(),
             itsOutputRangeMax.getVal());

            itsMap = maxNormalize(itsMap, itsOutputRangeMin.getVal(),
                           itsOutputRangeMax.getVal(), itsNormType.getVal());
    }

  cvReleaseImage(&small_img);
#endif
}

// ######################################################################
Image<float> ObjDetChannel::getSubmap(const uint index) const
{
  if (index != 0)
    LFATAL("got submap index = %u, but I have only one submap", index);

  return itsMap;
}

// ######################################################################
std::string ObjDetChannel::getSubmapName(const uint index) const
{
  return "ObjDet";
}

// ######################################################################
std::string ObjDetChannel::getSubmapNameShort(const uint index) const
{
  return "ObjDet";
}


// ######################################################################
Image<float> ObjDetChannel::getOutput()
{ return itsMap; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
