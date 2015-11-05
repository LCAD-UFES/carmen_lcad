/*!@file Channels/DescriptorVec.C descriptor vector generator for obj rec */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DescriptorVec.C $
// $Id: DescriptorVec.C 9412 2008-03-10 23:10:15Z farhan $
//

#include "Channels/DescriptorVec.H"

#include "Channels/ChannelOpts.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Image/CutPaste.H"
#include "SIFT/VisualObject.H"
#include "SIFT/Keypoint.H"
#include "Image/ColorOps.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Image/fancynorm.H"

#include "GUI/DebugWin.H"
#include "Image/DrawOps.H"

#include <cstdlib>
#include <iostream>

// Used by: Retina
static const ModelOptionDef OPT_DescriptorVecFOV =
  { MODOPT_ARG(Dims), "DescriptorVecFOV", &MOC_CHANNEL, OPTEXP_CORE,
    "Use the given fovea size for constructing DescriptorVec.",
    "descriptor-vec-fov", '\0', "<w>x<h>", "75x75" };

// ######################################################################
DescriptorVec::DescriptorVec(OptionManager& mgr,
    const std::string& descrName,
    const std::string& tagName,
    ComplexChannel *cc)
: ModelComponent(mgr, descrName, tagName),
  itsComplexChannel(cc), itsFoveaSize(&OPT_DescriptorVecFOV, this),
  itsFeatureHistogram(100),
  itsFEngine(0),
  itsFV(0)
{
  itsFoveaSize.setVal(Dims(80,80));
}

// ######################################################################
DescriptorVec::~DescriptorVec()
{}

// ######################################################################
void DescriptorVec::setInputImg(const Image<PixRGB<byte> >& img)
{
  itsInputImg = img;
}

// ######################################################################
void DescriptorVec::setFovea(Point2D<int> loc)
{
  int locX = loc.i;
  int locY = loc.j;
  int foveaW = itsFoveaSize.getVal().w();
  int foveaH = itsFoveaSize.getVal().h();

  LINFO("Getting descriptor from loc %i,%i", loc.i, loc.j);
  LINFO("Fovea size: %ix%i", foveaW, foveaH);
  if (itsComplexChannel->hasInput()){
    //get the input image
    Image<PixRGB<byte> > img = itsInputImg;
    int imgW = img.getWidth();
    int imgH = img.getHeight();

    LINFO("Image size: %ix%i", imgW, imgH);

    //Adjest the fovea location so we dont go outside the image
    int tl_x = locX - ((foveaW-1)/2);
    int tl_y = locY - ((foveaH-1)/2);

    //Sift the fovea position if nessesary
    if (tl_x < 0) tl_x = 0; if (tl_y < 0) tl_y = 0;
    if (tl_x+foveaW > imgW) tl_x = imgW - foveaW;
    if (tl_y+foveaH > imgH) tl_y = imgH - foveaH;

    //adjust locX/locX if tl_x/tl_y has changed
    locX = tl_x + ((foveaW-1)/2);
    locY = tl_y + ((foveaH-1)/2);

    itsFoveaLoc = Point2D<int>(locX, locY);

    //get only the fovea region
    if (itsFoveaSize.getVal().w() < img.getWidth()) //crop if our fovea is smaller then img
    {
      img= crop(img, Point2D<int>(tl_x, tl_y), itsFoveaSize.getVal());
    }

    //float angle = getDominateOrientation(luminance(img));

    //Rotate the image around the main angle
    //itsFoveaImg = rotate(img, img.getWidth()/2, img.getHeight()/2, angle);
    itsFoveaImg = img; //rotate(img, img.getWidth()/2, img.getHeight()/2, angle);

    //itsFoveaImg = toRGB(img);

  } else {
    LINFO("No input image in VC");
  }


}

// ######################################################################
void DescriptorVec::setFoveaSize(Dims &d)
{
  itsFoveaSize.setVal(d);
}

// ######################################################################
void DescriptorVec::setFoveaSize(int foveaRadius)
{
  Dims fr(foveaRadius, foveaRadius);
  itsFoveaSize.setVal(fr);
}

// ######################################################################
Dims DescriptorVec::getFoveaSize()
{
  return itsFoveaSize.getVal();
}

// ######################################################################
float DescriptorVec::getDominateOrientation(const Image<float> &img){
  //get main orientation
  const int ORIENTARRAY = 36;
  Image<float> gradmag, gradori;
  gradientSobel(luminance(img), gradmag, gradori);

  //Add orientations to the histogram
  Histogram OV(ORIENTARRAY);
  for (int y=0; y<img.getHeight(); y++){
    for(int x=0; x<img.getWidth(); x++){
      const float gradVal = gradmag.getVal(x, y);
      float angle = gradori.getVal(x, y) + M_PI;

      angle = 0.5F * angle * ORIENTARRAY / M_PI;
      while (angle < 0.0F) angle += ORIENTARRAY;
      while (angle >= ORIENTARRAY) angle -= ORIENTARRAY;

      OV.addValueInterp(angle, 1 * gradVal);

    }
  }

  // smooth the orientation histogram 3 times:
  for (int i = 0; i < 3; i++) OV.smooth();

  // find the max in the histogram:
  float maxPeakVal; int maxLoc;
  OV.findMax(maxLoc, maxPeakVal);

  // get value to the left of current value
  const float leftval = OV.getValue((maxLoc == 0) ? ORIENTARRAY-1 : maxLoc-1);

  // get value to the right of current value
  const float rightval = OV.getValue((maxLoc == ORIENTARRAY-1) ? 0 : maxLoc+1);

  // interpolate the values to get the orientation of the peak:
  //  with f(x) = ax^2 + bx + c
  //   f(-1) = x0 = leftval
  //   f( 0) = x1 = midval
  //   f(+1) = x2 = rightval
  //  => a = (x0+x2)/2 - x1
  //     b = (x2-x0)/2
  //     c = x1
  // f'(x) = 0 => x = -b/2a
  const float a  = 0.5f * (leftval + rightval) - maxPeakVal;

  const float b  = 0.5f * (rightval - leftval);
  float realangle = float(maxLoc) - 0.5F * b / a;

  realangle *= 2.0F * M_PI / ORIENTARRAY; // [0:36] to [0:2pi]
  realangle -= M_PI;                      // [0:2pi] to [-pi:pi]


  /*
     float realangle;

  // Find orientation peak:
  for (int bin = 0; bin < ORIENTARRAY; bin++)
  {
  // consider the peak centered around 'bin':
  const float midval = OV.getValue(bin);

  // if current value much smaller than global peak, forget it:
  if (midval < 0.8F * maxPeakValue) continue;

  // get value to the left of current value
  const float leftval = OV.getValue((bin == 0) ? ORIENTARRAY-1 : bin-1);

  // get value to the right of current value
  const float rightval = OV.getValue((bin == ORIENTARRAY-1) ? 0 : bin+1);

  // only consider local peaks:
  if (leftval >= midval) continue;
  if (rightval >= midval) continue;

  // interpolate the values to get the orientation of the peak:
  //  with f(x) = ax^2 + bx + c
  //   f(-1) = x0 = leftval
  //   f( 0) = x1 = midval
  //   f(+1) = x2 = rightval
  //  => a = (x0+x2)/2 - x1
  //     b = (x2-x0)/2
  //     c = x1
  // f'(x) = 0 => x = -b/2a
  const float a  = 0.5f * (leftval + rightval) - midval;

  const float b  = 0.5f * (rightval - leftval);
  realangle = float(bin) - 0.5F * b / a;

  realangle *= 2.0F * M_PI / ORIENTARRAY; // [0:36] to [0:2pi]
  realangle -= M_PI;                      // [0:2pi] to [-pi:pi]
  break;
  }*/

  return realangle;

}

// ######################################################################
//! Return the image under the fovea
Image<PixRGB<byte> > DescriptorVec::getFoveaImage() {


  if (itsFoveaImg.initialized()){
    /*rutz::shared_ptr<VisualObject>
      obj(new VisualObject("NewObject", "NewObject", itsFoveaImg));

      itsFoveaImg = obj->getKeypointImage();*/

  } else {
    LINFO("No input image in VC");
  }

  return itsFoveaImg;
}

// ######################################################################
//! Return the image of the histogram under the fovea
Image<PixRGB<byte> > DescriptorVec::getHistogramImage() {

  //return Image<PixRGB<byte> >();
  return itsFoveaImg;

}

// ######################################################################
//! Build a SIFT descriptor vector
void DescriptorVec::buildSIFTDV()
{
  rutz::shared_ptr<VisualObject>
    obj(new VisualObject("NewObject", "NewObject", itsFoveaImg));

  std::vector< rutz::shared_ptr<Keypoint> > keypoints = obj->getKeypoints();

  //add the keypoints to our DV
  for (unsigned int  i=0; i<keypoints.size(); i++)
  {

    std::vector<double> fv;

    for(unsigned int j=0; j<keypoints[i]->getFVlength(); j++)
      fv.push_back((byte)keypoints[i]->getFVelement(j));
    //itsFV.push_back(fv);
  }

}

void DescriptorVec::buildFftDV()
{

  /*
     Image<float> gradmag, gradori;
  //TODO: This was already computed in DomnateOri, can be opt
  gradientSobel(luminance(itsFoveaImg), gradmag, gradori);


  //   itsFoveaImg = toRGB(Image<byte>(gradmag));

  if (itsFEngine == 0)
  itsFEngine = new FourierEngine<double>(gradmag.getDims());
  itsFTImg = itsFEngine->fft(gradmag);

  if (!itsFV.initialized()){
  itsFV = logmagnitude(itsFTImg);
  // itsFV = phase(itsFTImg);
  } else {
  //itsFV = phase(itsFTImg);
  itsFV = logmagnitude(itsFTImg);
  }

  //inplaceNormalize(itsFV, 0.0f, 255.0f);

*/
  /*inplaceNormalize(img, 0.0f, 99.0f);
    Point2D<int> p; float maxval, minval;
    findMax(img, p, maxval);
    findMin(img, p, minval);
    LINFO("Max %f min %f", maxval, minval);
    itsFeatureHistogram.clear();
    for (int y=0; y<img.getHeight(); y++){
    for(int x=0; x<img.getWidth(); x++){
    float val = img.getVal(x, y);
    printf("%0.2f ", val);
    itsFeatureHistogram.addValueInterp(val, 1);
    }
    }
    printf("\n");*/

  //itsFeatureHistogram((Image<byte>)img);

}
// ######################################################################
void DescriptorVec::buildParticleCountDV()
{

  //Count the saliency values in the fovea region
  ComplexChannel& cc = dynamic_cast<ComplexChannel&>(*itsComplexChannel->subChan(0));

  const LevelSpec lspec = itsComplexChannel->getModelParamVal<LevelSpec>("LevelSpec");
  const int smlevel = lspec.mapLevel();
  int x=int(itsFoveaLoc.i / double(1 << smlevel) + 0.49);
  int y=int(itsFoveaLoc.j / double(1 << smlevel) + 0.49);

  itsFV.clear();
  int FVi = 0;
  for (uint i = 0; i < cc.numChans(); i++)
  {
    nub::ref<ChannelBase> cb = cc.subChan(i);
    if (dynamic_cast<ComplexChannel*>(cb.get()) != 0) //we have a complex channel
    {
      LINFO("Complex channel %i %s", i, cb->descriptiveName().c_str());
    } else {
      SingleChannel& sc = dynamic_cast<SingleChannel&>(*cb);
      LINFO("Single channel %i", i);
      for (uint j=0; j< sc.numSubmaps(); j++)
      {
        Image<float> submap = sc.getSubmap(j);

        itsFV.push_back(submap.getVal(x, y));
        LINFO("FV%i %s:%i", FVi++, sc.descriptiveName().c_str(), j);
        /*submap = crop(submap, //get only the fovea region
          Point2D<int>(itsFoveaLoc.i - ((itsFoveaSize.getVal().w()-1)/2),
          itsFoveaLoc.j - ((itsFoveaSize.getVal().h())-1)/2),
          itsFoveaSize.getVal());*/


        /*submap = maxNormalize(submap, 0.0F, 10.0F, VCXNORM_MAXNORM);
          Point2D<int> p; float maxVal, minVal, midVal;
          findMax(submap, p, maxVal);
          findMin(submap, p, minVal);
          midVal = (maxVal-minVal)/2;

          int nParticles = countThresh(submap, midVal); //countParticles(submap, 1.0F);

          LINFO("Channel %s:%i max is %0.2f min is %0.2f mid %0.2f, p=%i",
          sc.descriptiveName().c_str(),j, maxVal, minVal, midVal,
          nParticles);
          printf("%i ", nParticles);
        //SHOWIMG(submap);  */
      }

    }
  }
}

// ######################################################################
void DescriptorVec::buildDV()
{
  const LevelSpec lspec = itsComplexChannel->getModelParamVal<LevelSpec>("LevelSpec");
  const int smlevel = lspec.mapLevel();

  int x=int(itsFoveaLoc.i / double(1 << smlevel) + 0.49);
  int y=int(itsFoveaLoc.j / double(1 << smlevel) + 0.49);

  int foveaW = int(itsFoveaSize.getVal().w() / double(1 << smlevel) + 0.49);
  int foveaH = int(itsFoveaSize.getVal().h() / double(1 << smlevel) + 0.49);

  //Adjest the fovea location so we dont go outside the image
  int tl_x = x - (foveaW/2);
  int tl_y = y - (foveaH/2);


  //Go through all the submaps building the DV
  itsFV.clear(); //clear the FV
  uint numSubmaps = itsComplexChannel->numSubmaps();
  for (uint i = 0; i < numSubmaps; i++)
  {
    Image<float> submap = itsComplexChannel->getSubmap(i);
    //Image<float> submap = itsComplexChannel->getRawCSmap(i);


    //itsFV.push_back(submap.getVal(x,y));

    //get only the fovea region
    if (foveaW < submap.getWidth()) //crop if our fovea is smaller
      submap = crop(submap, Point2D<int>(tl_x, tl_y), Dims(foveaW, foveaH));
    //submap = maxNormalize(submap, 0.0F, 10.0F, VCXNORM_MAXNORM);



    /*   Point2D<int> p; float maxVal, minVal, midVal;
         findMax(submap, p, maxVal);
         findMin(submap, p, minVal);
         midVal = (maxVal-minVal)/2;
         int nParticles = countThresh(submap, 1.0F); //countParticles(submap, 1.0F);
         itsFV.push_back(nParticles); */

    float maxVal; Point2D<int> maxLoc;
    findMax(submap, maxLoc, maxVal);

    //SHOWIMG(rescale(submap, 255, 255));

    itsFV.push_back(maxVal);

  }
}

// ######################################################################
void DescriptorVec::buildRawDV()
{

  bool salientLocationWithinSubmaps = true;
  Point2D<int> objSalientLoc(-1,-1);  //the feature location

  const LevelSpec lspec = itsComplexChannel->getModelParamVal<LevelSpec>("LevelSpec");
  const int smlevel = lspec.mapLevel();

  int x=int(itsFoveaLoc.i / double(1 << smlevel) + 0.49);
  int y=int(itsFoveaLoc.j / double(1 << smlevel) + 0.49);

  int foveaW = int(itsFoveaSize.getVal().w() / double(1 << smlevel) + 0.49);
  int foveaH = int(itsFoveaSize.getVal().h() / double(1 << smlevel) + 0.49);

  int tl_x = x - (foveaW/2);
  int tl_y = y - (foveaH/2);

  Dims mapDims = itsComplexChannel->getSubmap(0).getDims();

  //Shift the fovea location so we dont go outside the image
  //Sift the fovea position if nessesary
  if (tl_x < 0) tl_x = 0; if (tl_y < 0) tl_y = 0;
  if (tl_x+foveaW > mapDims.w()) tl_x = mapDims.w() - foveaW;
  if (tl_y+foveaH > mapDims.h()) tl_y = mapDims.h() - foveaH;

  if (!salientLocationWithinSubmaps)
  {
    //Find the most salient location within the fovea
    Image<float> SMap = itsComplexChannel->getOutput();

    Image<float> tmp = SMap; //TODO need to resize to fovea
    //Find the max location within the fovea

    float maxVal; Point2D<int> maxLoc;
    findMax(tmp, maxLoc, maxVal);
    //convert back to original SMap cordinates
   // objSalientLoc.i=tl_x+maxLoc.i;
   // objSalientLoc.j=tl_y+maxLoc.j;
    objSalientLoc.i=x;
    objSalientLoc.j=y;
    itsAttentionLoc = objSalientLoc;
  }

  //Go through all the submaps building the DV
  itsFV.clear(); //clear the FV
  uint numSubmaps = itsComplexChannel->numSubmaps();
  for (uint i = 0; i < numSubmaps; i++)
  {
    //Image<float> submap = itsComplexChannel->getSubmap(i);
    Image<float> submap = itsComplexChannel->getRawCSmap(i);

    // resize submap to fixed scale if necessary:
    if (submap.getWidth() > mapDims.w())
      submap = downSize(submap, mapDims);
    else if (submap.getWidth() < mapDims.w())
      submap = rescale(submap, mapDims); //TODO convert to  quickInterpolate


    if (salientLocationWithinSubmaps) //get the location from the salient location within each submap
    {
      Image<float> tmp = submap;
      //get only the fovea region

      if (foveaW < tmp.getWidth()) //crop if our fovea is smaller
        tmp = crop(tmp, Point2D<int>(tl_x, tl_y), Dims(foveaW, foveaH));
     // tmp = maxNormalize(tmp, 0.0F, 10.0F, VCXNORM_MAXNORM);  //find salient locations

      //Find the max location within the fovea
      float maxVal; Point2D<int> maxLoc; findMax(tmp, maxLoc, maxVal);
      //LINFO("%i: Max val %f, loc(%i,%i)", i, maxVal, maxLoc.i, maxLoc.j);

      objSalientLoc.i=tl_x+maxLoc.i;
      objSalientLoc.j=tl_y+maxLoc.j;

    }

    if (objSalientLoc.i < 0) objSalientLoc.i = 0;
    if (objSalientLoc.j < 0) objSalientLoc.j = 0;

    if (objSalientLoc.i > submap.getWidth()-1) objSalientLoc.i = submap.getWidth()-1;
    if (objSalientLoc.j > submap.getHeight()-1) objSalientLoc.j = submap.getHeight()-1;



   // LINFO("Location from %i,%i: (%i,%i)", objSalientLoc.i, objSalientLoc.j,
    //    submap.getWidth(), submap.getHeight());
    float featureVal = submap.getVal(objSalientLoc.i,objSalientLoc.j);
    itsFV.push_back(featureVal);
 //   SHOWIMG(rescale(submap, 255, 255));

  }
}

// ######################################################################
void DescriptorVec::buildLocalDV()
{

  Point2D<int> objSalientLoc(-1,-1);  //the feature location


  //Go through all the submaps building the DV
  itsFV.clear(); //clear the FV
  uint numSubmaps = itsComplexChannel->numSubmaps();
  for (uint i = 0; i < numSubmaps; i++)
  {
    //Image<float> submap = itsComplexChannel->getSubmap(i);
    Image<float> submap = itsComplexChannel->getRawCSmap(i);

    //Find the max location within the fovea
    float maxVal; Point2D<int> maxLoc; findMax(submap, maxLoc, maxVal);
    //SHOWIMG(submap);

    float featureVal = submap.getVal(maxLoc.i,maxLoc.j);
    itsFV.push_back(featureVal);
 //   SHOWIMG(rescale(submap, 255, 255));

  }
}
// ######################################################################
void DescriptorVec::buildSingleChannelFV(SingleChannel &sc)
{
  const LevelSpec lspec = itsComplexChannel->getModelParamVal<LevelSpec>("LevelSpec");
  const int smlevel = lspec.mapLevel();
  int x=int(itsFoveaLoc.i / double(1 << smlevel) + 0.49);
  int y=int(itsFoveaLoc.j / double(1 << smlevel) + 0.49);

  //LINFO("Single channel %s", sc.descriptiveName().c_str());
  for (uint j=0; j< sc.numSubmaps(); j++)
  {
    Image<float> submap = sc.getSubmap(j);
    itsFV.push_back(submap.getVal(x, y));
  }

}

uint DescriptorVec::getFVSize()
{
  ASSERT(itsComplexChannel != NULL);
  return itsComplexChannel->numSubmaps();
}

// ######################################################################
const std::vector<double>& DescriptorVec::getFV() const
{
  return itsFV;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
