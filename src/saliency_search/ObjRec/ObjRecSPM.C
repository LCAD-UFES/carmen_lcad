/*!@file ObjRec/ObjRecSPM.C Obj Reconition using spatial pyramid matching
 algorithem from Lzebnik, Schmid and Ponce
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/ObjRecSPM.C $
// $Id: ObjRecSPM.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Image/Kernels.H"
#include "Image/Convolutions.H"
#include "GUI/DebugWin.H"
#include "ObjRec/ObjRecSPM.H"
#include "SIFT/FeatureVector.H"


// ######################################################################
ObjRecSPM::ObjRecSPM(OptionManager& mgr, const std::string& descrName,
    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsNumOri(4),
  itsNumScales(2),
  itsNumOriArray(36), //for sift descriptor
  itsObjects(0),
  itsUseSaliency(false)
{

  float stdmin = 1.75F;
  float stdstep = 0.5F;
  int fsmin = 3;
  int fsstep = 1;

  //create the filters
  for(int scale = 0; scale < itsNumScales; scale++)
    for(int ori = 0; ori < itsNumOri; ori++)
    {

      Image<float> filter = dogFilter<float>(stdmin + stdstep * scale,
          (float)ori * 180.0F / (float)itsNumOri,
          fsmin + fsstep * scale);

      //  fsmin + fsstep * scale);
      // normalize to zero mean:
      filter -= mean(filter);

      // normalize to unit sum-of-squares:
      filter /= sum(squared(filter));

      itsFilters.push_back(filter);
    }

    if (itsUseSaliency)
    {
      itsGetSaliency = nub::soft_ref<GetSaliency>(new GetSaliency(mgr));
      addSubComponent(itsGetSaliency);
    }

}

void ObjRecSPM::start2()
{


}

ObjRecSPM::~ObjRecSPM()
{
}


void ObjRecSPM::train(const Image<PixRGB<byte> > &img, const std::string label)
{

  Image<float> input = luminance(img); //convert to gray

  if (itsUseSaliency)
  {
    Image<PixRGB<byte> > inputImg = rescale(img, 256, 256);
    itsGetSaliency->compute(inputImg, SimTime::MSECS(3.0));
    Image<float> smap = itsGetSaliency->getSalmap();
    smap = rescale(smap, img.getDims());
    inplaceNormalize(smap, 0.0F, 1.0F);
    input *= smap; //weigh the input by the saliency map
  }

  //Descriptor desc = extractFeatures(input);
  Descriptor desc = extractSiftFeatures(input);

  uint objId = getObject(label);

  itsObjects[objId].model.push_back(desc);

}

void ObjRecSPM::finalizeTraining()
{
  LINFO("Training done");

  for(uint i=0; i<itsObjects.size(); i++)
    for(uint j=0; j<itsObjects[i].model.size(); j++)
    {
      printf("Obj %u model %u size %" ZU ,
          i, j, itsObjects[i].model[j].featureLevelHist.size());

    }


}


uint ObjRecSPM::getObject(const std::string name)
{
  //Find the object
  //TODO can use hash function
  uint i=0;
  for(i=0; i<itsObjects.size(); i++)
    if (itsObjects[i].name == name)
      return i;

  //Object not found. create a new one
  Object obj;
  obj.id = i;
  obj.name = name;
  obj.model.clear();

  itsObjects.push_back(obj);

  return i;
}


ObjRecSPM::Descriptor ObjRecSPM::extractFeatures(const Image<float> &input)
{

  double normSum = 0; //the numalization constant

  //Compute the various low level features
  ImageSet<float> featuresValues(itsFilters.size());
  for(uint i=0; i<itsFilters.size(); i++)
  {
    Image<float> tmp = convolve(input, itsFilters[i], CONV_BOUNDARY_CLEAN); //No normalization
    //Image<float> tmp = convolveHmax(input, itsFilters[i]); // normalize by image energy
    tmp = abs(tmp);
    normSum += sum(tmp);
    featuresValues[i] = tmp;
  }

  //get the histograms

  Descriptor desc;
  for(uint feature=0; feature<featuresValues.size(); feature++) //for each feature m
  {
    Image<float> featureVal = featuresValues[feature];

    std::vector<Histogram> levelHists;
    for(int level = 0; level < 4; level++)
    {
      int levelSize = 1<<level;
      Histogram hist(levelSize*levelSize);

      int xSpace = (featureVal.getWidth()/levelSize)+1;
      int ySpace = (featureVal.getHeight()/levelSize)+1;

      for(int y=0; y<featureVal.getHeight(); y++)
        for(int x=0; x<featureVal.getWidth(); x++)
        {
          int binPos = (int)(x/xSpace + 2*(y/ySpace));
          hist.addValue(binPos, featureVal.getVal(x,y));
        }
      hist.normalize(normSum); //normalize across the sum of all feature outputs
      levelHists.push_back(hist);
    }
    desc.featureLevelHist.push_back(levelHists);
  }

  return desc;
}

ObjRecSPM::Descriptor ObjRecSPM::extractSiftFeatures(const Image<float> &input)
{

  SHOWIMG(input);
  Descriptor desc;
  for(int y=10; y<input.getHeight()-10; y+=10)
    for(int x=10; x<input.getWidth()-10; x+=10)
    {
      SiftKeypoint kp;
      kp.x = x;
      kp.y = y;
      kp.fv = getSiftDescriptor(input, x,y,2);
      desc.siftDescriptors.push_back(kp);
      LINFO("%ix%i", x, y);
      //for(uint i=0; i<fv.size(); i++)
      //{
      //  FeatureVector fvtmp;
      //
      //  SHOWIMG(fvtmp.getFeatureVectorImage(fv[i]));
      //}
    }


  return desc;
}

std::string ObjRecSPM::predict(const Image<PixRGB<byte> > &img)
{

  Image<float> input = luminance(img); //convert to gray

  if (itsUseSaliency)
  {
    Image<PixRGB<byte> > inputImg = rescale(img, 256, 256);
    itsGetSaliency->compute(inputImg, SimTime::MSECS(3.0));
    Image<float> smap = itsGetSaliency->getSalmap();
    smap = rescale(smap, img.getDims());
    inplaceNormalize(smap, 0.0F, 1.0F);
    input *= smap; //weigh the input by the saliency map
  }

  //Descriptor desc = extractFeatures(input);
  Descriptor desc = extractSiftFeatures(input);

  //find matches

 // Descriptor objDec = itsObjects[0].model[0];

  for(uint kp_i=0; kp_i<desc.siftDescriptors.size(); kp_i++)
  {
    SiftKeypoint kp = desc.siftDescriptors[kp_i];

    for(uint fv_i=0; fv_i<kp.fv.size(); fv_i++)
    {
      std::vector<byte> fv = kp.fv[fv_i];
      FeatureVector tmpFv;

      SHOWIMG(tmpFv.getFeatureVectorImage(fv));
    }
  }






  int objId = findObject(desc);

  if (objId != -1)
    return itsObjects[objId].name;

  return std::string("Unknown");
}

//NN search
int ObjRecSPM::findObject(const Descriptor &desc)
{
  int objId = -1;

  double minDist = std::numeric_limits<double>::max();
  //Find the object with the closest distance
  for(uint i=0; i<itsObjects.size(); i++)
  {
    //Find the closes model match with the data
    for(uint j=0; j<itsObjects[i].model.size(); j++)
    {
      double dist = matchDescriptor(itsObjects[i].model[j], desc);
      if (dist < minDist)
      {
        minDist = dist;
        objId = i;
      }
    }

  }

  return objId;

}

double ObjRecSPM::matchDescriptor(const Descriptor &descA, const Descriptor &descB)
{

  double sum = 0;
  for(uint feature=0; feature<descA.featureLevelHist.size(); feature++)
  {
    sum += matchKernel(descA.featureLevelHist[feature], descB.featureLevelHist[feature]);
  }

  return sum;



}

double ObjRecSPM::matchKernel(const std::vector<Histogram>& A, const std::vector<Histogram>& B)
{


  if (B.size() > A.size())
    LFATAL("Incompatibale histograms");
  double dist = 0;

  //Equlideian distance between histograms
  for(uint level=0; level<A.size(); level++)
  {
    Histogram modelHist = A[level];
    Histogram testHist = B[level];

    double weight;
    if (level == 0)
      weight = 1.0F/(double)(1<<(A.size()));
    else
      weight = 1.0F/(double)(1<<(A.size()-level+1));
    dist += weight*modelHist.getDistance(testHist);

  }

  //histogram intersection
  //for(uint i=0; i<A.size(); i++)
  //  dist += std::min(A[i],B[i]);

  //KL distance
  //for(uint i=0; i<A.size(); i++)
  //{
  //  //Insure that no probabiliy is set to 0
  //  double pA = (A[i] > 0) ? A[i] : 0.0001;
  //  double qB = (B[i] > 0) ? B[i] : 0.0001;

  //  dist += pA * log(pA/qB);
  //}


  return dist;

}

std::vector<std::vector<byte> >  ObjRecSPM::getSiftDescriptor(const Image<float> &lum,
    const float x, const float y, const float s)
{

  Image<float> mag, ori;
  gradientSobel(lum, mag, ori, 3);

  Histogram OV(36);

  // 1. Calculate main orientation vector
  calculateOrientationVector(x, y, s, mag, ori, OV);

  //Image<byte> histImg = OV.getHistogramImage(256, 256, 0, -1);

  // 2. Create feature vector and keypoint for each significant
  // orientation peak:
  return createVectorsAndKeypoints(x, y, s, mag, ori, OV);

}

void ObjRecSPM::calculateOrientationVector(const float x, const float y, const float s,
                const Image<float>& gradmag, const Image<float>& gradorie, Histogram& OV) {


        // compute the effective blurring sigma corresponding to the
        // fractional scale s:
        const float sigma = s;

        const float sig = 1.5F * sigma, inv2sig2 = - 0.5F / (sig * sig);
        const int dimX = gradmag.getWidth(), dimY = gradmag.getHeight();

        const int xi = int(x + 0.5f);
        const int yi = int(y + 0.5f);

        const int rad = int(3.0F * sig);
        const int rad2 = rad * rad;


        // search bounds:
        int starty = yi - rad; if (starty < 0) starty = 0;
        int stopy = yi + rad; if (stopy >= dimY) stopy = dimY-1;

        // 1. Calculate orientation vector
        for (int ind_y = starty; ind_y <= stopy; ind_y ++)
        {
                // given that y, get the corresponding range of x values that
                // lie within the disk (and remain within the image):
                const int yoff = ind_y - yi;
                const int bound = int(sqrtf(float(rad2 - yoff*yoff)) + 0.5F);
                int startx = xi - bound; if (startx < 0) startx = 0;
                int stopx = xi + bound; if (stopx >= dimX) stopx = dimX-1;

                for (int ind_x = startx; ind_x <= stopx; ind_x ++)
                {
                        const float dx = float(ind_x) - x, dy = float(ind_y) - y;
                        const float distSq = dx * dx + dy * dy;

                        // get gradient:
                        const float gradVal = gradmag.getVal(ind_x, ind_y);

                        // compute the gaussian weight for this histogram entry:
                        const float gaussianWeight = expf(distSq * inv2sig2);

                        // add this orientation to the histogram
                        // [-pi ; pi] -> [0 ; 2pi]
                        float angle = gradorie.getVal(ind_x, ind_y) + M_PI;

                        // [0 ; 2pi] -> [0 ; 36]
                        angle = 0.5F * angle * itsNumOriArray / M_PI;
                        while (angle < 0.0F) angle += itsNumOriArray;
                        while (angle >= itsNumOriArray) angle -= itsNumOriArray;

                        OV.addValueInterp(angle, gaussianWeight * gradVal);
                }
        }


        // smooth the orientation histogram 3 times:
        for (int i = 0; i < 3; i++) OV.smooth();
}

// ######################################################################


std::vector<std::vector<byte> > ObjRecSPM::createVectorsAndKeypoints(const float x,
    const float y, const float s,
    const Image<float>& gradmag, const Image<float>& gradorie, Histogram& OV)
{

  const float sigma = s; //itsSigma * powf(2.0F, s / float(itsLumBlur.size() - 3));

  // find the max in the histogram:
  float maxPeakValue = OV.findMax();

  const int xi = int(x + 0.5f);
  const int yi = int(y + 0.5f);

  uint numkp = 0;

  std::vector<std::vector<byte> > descriptor;

  // 2. Create feature vector and keypoint for each significant
  // orientation peak:
  for (int bin = 0; bin < itsNumOriArray; bin++)
  {
    // consider the peak centered around 'bin':
    const float midval = OV.getValue(bin);

    // if current value much smaller than global peak, forget it:
    if (midval < 0.8F * maxPeakValue) continue;

    // get value to the left of current value
    const float leftval = OV.getValue((bin == 0) ? itsNumOriArray-1 : bin-1);

    // get value to the right of current value
    const float rightval = OV.getValue((bin == itsNumOriArray-1) ? 0 : bin+1);

    // only consider local peaks:
    if (leftval > midval) continue;
    if (rightval > midval) continue;

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
    float realangle = float(bin) - 0.5F * b / a;

    realangle *= 2.0F * M_PI / itsNumOriArray; // [0:36] to [0:2pi]
    realangle -= M_PI;                      // [0:2pi] to [-pi:pi]

    // ############ Create keypoint:

    // compute the feature vector:
    FeatureVector fv;

    const float sinAngle = sin(realangle), cosAngle = cos(realangle);

    // check this scale
    const int radius = int(5.0F * sigma + 0.5F); // NOTE: Lowe uses radius=8?
    const float gausssig = float(radius); // 1/2 width of descript window
    const float gaussfac = - 0.5F / (gausssig * gausssig);


    // Scan a window of diameter 2*radius+1 around the point of
    // interest, and we will cumulate local samples into a 4x4 grid
    // of bins, with interpolation. NOTE: rx and ry loop over a
    // square that is assumed centered around the point of interest
    // and rotated to the gradient orientation (realangle):

    int scale = abs(int(s));
    scale = scale > 5 ? 5 : scale;

    for (int ry = -radius; ry <= radius; ry++)
      for (int rx = -radius; rx <= radius; rx++)
      {
        // rotate the point:
        const float newX = rx * cosAngle - ry * sinAngle;
        const float newY = rx * sinAngle + ry * cosAngle;

        // get the coords in the image frame of reference:
        const float orgX = newX + float(xi);
        const float orgY = newY + float(yi);

        // if outside the image, forget it:
        if (gradmag.coordsOk(orgX, orgY) == false) continue;

        // find the fractional coords of the corresponding bin
        // (we subdivide our window into a 4x4 grid of bins):
        const float xf = 2.0F + 2.0F * float(rx) / float(radius);
        const float yf = 2.0F + 2.0F * float(ry) / float(radius);


        // find the Gaussian weight from distance to center and
        // get weighted gradient magnitude:
        const float gaussFactor = expf((newX*newX+newY*newY) * gaussfac);
        const float weightedMagnitude =
          gaussFactor * gradmag.getValInterp(orgX, orgY);

        // get the gradient orientation relative to the keypoint
        // orientation and scale it for 8 orientation bins:
        float gradAng = gradorie.getValInterp(orgX, orgY) - realangle;

        gradAng=fmod(gradAng, 2*M_PI); //bring the range from 0 to M_PI

        //convert from -M_PI to M_PI
        if (gradAng < 0.0) gradAng+=2*M_PI; //convert to -M_PI to M_PI
        if (gradAng >= M_PI) gradAng-=2*M_PI;
        //split to eight bins
        const float orient = (gradAng + M_PI) * 8 / (2 * M_PI);

        /*
        //reflect the angle to convert from 0 to M_PI
        if (gradAng >= M_PI) gradAng-=M_PI;
        //split to four bins
        const float orient = (gradAng + M_PI) * 4 / (2 * M_PI);
         */

        // will be interpolated into 2 x 2 x 2 bins:

        fv.addValue(xf, yf, orient, weightedMagnitude);

      }

    // normalize, clamp, scale and convert to byte:
    std::vector<byte> oriVec;

    fv.toByteKey(oriVec);

    double mag = fv.getMag();
    if (oriVec.size() > 0 && mag > 0)
      descriptor.push_back(oriVec);
    //The key point

    ++ numkp;

  }
  return descriptor;
}





// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
