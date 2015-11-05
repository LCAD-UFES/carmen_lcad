/*!@file HMAX/HmaxFL.C Riesenhuber & Poggio's HMAX model for object recognition */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/HMAX/HmaxFL.C $
// $Id: HmaxFL.C 14376 2011-01-11 02:44:34Z pez $
//

#include <iostream>
#include "HMAX/HmaxFL.H"
#include "HMAX/Hmax.H"
#include "Image/FilterOps.H" // for convolve() etc.
#include "Image/Image.H"
#include "Image/Kernels.H"   // for dogFilter()
#include "Image/MathOps.H"
#include "Image/Normalize.H"
#include "Image/CutPaste.H"
#include "Raster/Raster.H"
#include "Raster/RasterFileFormat.H"
#include "Util/MathFunctions.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "Util/safecopy.H"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <limits>
#include <stdexcept>

// ######################################################################
HmaxFL::HmaxFL()
{ initialized = false; }

// ######################################################################
HmaxFL::HmaxFL(const int nori, const std::vector<int>& spacess,
               const std::vector<int>& scaless, const int c1spaceol,
               const bool angleflag, const float s2t, const float s2s,
               const float gamma, const float divstart, const float divstep,
               const int fsmin, const int fsstep)
{  
  initialized = false;
  init(nori, spacess, scaless, c1spaceol, angleflag, s2t, s2s, gamma, divstart, divstep, fsmin, fsstep);
  initialized = true;
}

// ######################################################################
void HmaxFL::init(const int nori, const std::vector<int>& spacess,
                  const std::vector<int>& scaless, const int c1spaceol,
                  const bool angleflag, const float s2t, const float s2s,
                  const float gamma, const float divstart, const float divstep,
                  const int fsmin, const int fsstep)
{
  Hmax::init(nori,spacess,scaless,c1spaceol,angleflag,s2t,s2s);
  int curNSWB;
  c1Patches = 0;
  nswb = 0;
  // Determine the number of scales within band
  for (int sb = 0; sb < nsb; sb ++) {
    curNSWB = 0;
    for(int s = scaleSS[sb]; s < scaleSS[sb + 1]; s++) {
      curNSWB+=1;
    }
    nswb = std::max(nswb,curNSWB);
  }
  initFilters(gamma,divstart,divstep,fsmin,fsstep);
}

void HmaxFL::initFilters(const float gamma, const float divstart, const float divstep, const int fsmin, const int fsstep)
{
  // create the filters:
  typedef Image<float>* FloatImagePtr;
  filter = new FloatImagePtr[ns];
  for(int s = 0; s < ns; s++)
    {
      filter[s] = new Image<float>[no];
      for(int o = 0; o < no; o ++)
        {
          // create DoG filter:
          filter[s][o] = dogFilterHmax<float>( (float)o * 180.0F / (float)no, gamma,
                                               fsmin + fsstep * s, divstart + divstep * s);
          // normalize to zero mean:
          filter[s][o] -= mean(filter[s][o]);

          // normalize to sqrt of sum-of-squares:
          filter[s][o] /= sqrt(sum(squared(filter[s][o])));
        }
    }
}

// ######################################################################
void HmaxFL::freeMem()
{
  if (initialized)
    {
      initialized = false;
      freeC1Patches();
      for(int s = 0; s < ns; s++) delete [] filter[s];
      delete [] filter;
    }
}

// ######################################################################
HmaxFL::~HmaxFL()
{ freeMem(); }



void HmaxFL::freeC1Patches()
{
  if(c1Patches != 0){
    for(unsigned int i=0;i<c1PatchSizes.size();i++){
      delete [] c1Patches[i];
    }
    delete [] c1Patches;
    c1Patches = 0;
  }
}


void HmaxFL::setC1Patches(Image<float>***&patches,std::vector<int> patchSizes,int nPatchesPerSize)
{
  c1Patches = patches;
  c1PatchSizes = patchSizes;
  nC1PatchesPerSize = nPatchesPerSize;
}

void HmaxFL::writeOutC1Patch(std::string dirName, Image<float>& patch, int i, int j,int k)
{
  std::string fname;
  std::stringstream fnamestr;
  fnamestr.str("");
  fnamestr << dirName << C1_PATCH_FILE_NAME << "." << i << "." << j << "." << k;
  fname = fnamestr.str();
  Raster::WriteFloat(patch,FLOAT_NORM_0_255,fname,RASFMT_PNM);
}

void HmaxFL::writeOutC1Patches(std::string dirName)
{
  if(c1Patches == 0){
    throw std::runtime_error("Cannot write out patches, patches undefined");
  }
  std::string fname;
  std::stringstream fnamestr;
  for(unsigned int i=0;i<c1PatchSizes.size();i++) {
    for(int j=0;j<nC1PatchesPerSize;j++) {
      for(int k=0;k<no;k++) {
        writeOutC1Patch(dirName,c1Patches[i][j][k],i,j,k);
      }
    }
  }
}

void HmaxFL::readInC1Patches(std::string dirName)
{
  std::string fname;
  std::stringstream fnamestr;

  int i,j,k;

  // mi - patchSizes
  // mj - number of patches per size
  // mk - number of orientations
  int mi,mj,mk;
  Image<byte> input;

  i=j=k=0;
  mi=mj=mk=0;
  // Size the number of patches first (very inefficient)
  while(1){
    while(1){
      while(1){
        fnamestr.str("");
        fnamestr << dirName << C1_PATCH_FILE_NAME << "." << i << "." << j << "." << k;
        fname = fnamestr.str();
        if(Raster::fileExists(fname,RASFMT_PNM))
          k++;
        else
          break;
      }
      if(mk==0 && k>0){
        mk=k;
      }
      if(k == 0){
        j--; // This j was too large
        break; // Either done, or j is also done
      }
      else if(mk != k){
        throw std::runtime_error("Unexpected jagged array of patches in 3rd dimension");
      }
      else {
        k=0;
        j++;
      }
    }
    if(mj==0 && j>0){
      mj=j;
    }
    if(j<=0){
      i--; // This i was too large
      break; // Should be done
    }
    else if(mj != j){
      throw std::runtime_error("Unexpected jagged array of patches in 2nd dimension");
    }
    else{
      j=0;
      i++;
    }
  }
  if(mi==0 && i>0){
    mi=i;
  }
  // The loop by design will end up with -1 -1 0 if the list is empty, this corrects
  mi+=1;
  mj+=1;

  if(mi==0 || mj==0 || mk==0){
    throw std::runtime_error("No patches found");
  }

  std::vector<int> patchSizes(mi);

  // Allocate the space for the patch 3D array
  Image<float> ***patches = new Image<float>**[mi];
  for(i=0;i<mi;i++){
    patches[i] = new Image<float>*[mj];
    for(j=0;j<mj;j++) {
      patches[i][j] = new Image<float>[mk];
    }
  }

  std::cout << mi << " " << mj << " " << mk << std::endl;

  // Load the patches
  int w,h;
  for(i=0;i<mi;i++){
    for(j=0;j<mj;j++){
      for(k=0;k<mk;k++){
        fnamestr.str("");
        fnamestr << dirName << C1_PATCH_FILE_NAME << "." << i << "." << j << "." << k;
        fname = fnamestr.str();
        input=Raster::ReadGray(fname,RASFMT_PNM);
        patches[i][j][k]=input;
        w=patches[i][j][k].getWidth();
        h=patches[i][j][k].getHeight();
        if(w != h){
          // I'm leaking, patches not freed!
          throw std::runtime_error("Patches are expected to be square");
        }
        if(j==0 && k==0){
          patchSizes[i] = w;
        }
        else if(patchSizes[i] != w){
          // I'm leaking, patches not freed!
          throw std::runtime_error("Patches in same band should all be the same size");
        }
      }
    }
  }
  // Free the existing patches class member to prevent a leak
  freeC1Patches();
  // Now set the new patches class member
  setC1Patches(patches,patchSizes,mj);
}

Image<float>***& HmaxFL::getC1Patches()
{
  return c1Patches;
}

std::vector<int> HmaxFL::getC1PatchSizes()
{
  return c1PatchSizes;
}

int HmaxFL::getC1PatchesPerSize()
{
  return nC1PatchesPerSize;
}

void HmaxFL::extractRandC1Patch(std::string dirname, Image<float> & image, int index, std::vector<int> patchSizes)
{
  Image<float> **c1Res;
  initC1(c1Res);
  getC1(image,c1Res);
  int sb = 0; // Only one scale band

  int bsize1 = c1Res[sb][0].getWidth();
  int bsize2 = c1Res[sb][0].getHeight();
  printf("C1 Activation size is %d %d ScaleSS %d ScaleOL %d\n",bsize1,bsize2,scaleSS[0],c1SpaceOL);
  Image<float> patch;
  for(unsigned int i=0;i<patchSizes.size();i++)
    {
      int xy1 = int(floor((rand()-1.0F)/RAND_MAX*(bsize1-patchSizes[i])));
      int xy2 = int(floor((rand()-1.0F)/RAND_MAX*(bsize2-patchSizes[i])));
      Rectangle r = Rectangle::tlbrI(xy2,xy1,xy2+patchSizes[i]-1,xy1+patchSizes[i]-1);
      for(int k=0;k<no;k++) {
        patch = crop(c1Res[sb][k],r);
        patch *= 255*10;
        writeOutC1Patch(dirname,patch,i,index,k);
      }
    }
  clearC1(c1Res);
}

void HmaxFL::extractRandC1Patches(std::string dirname, Image<float> *&  posTrainingImages, int numPosTrainImages, std::vector<int> patchSizes, int nPatchesPerSize)
{
  std::srand(time(0));
  for(int i=0;i<nPatchesPerSize;i++){
    // Randomly select an image from the list
    unsigned int imInd = static_cast<unsigned int>(floor((rand()-1.0F)/RAND_MAX*numPosTrainImages));
    extractRandC1Patch(dirname,posTrainingImages[imInd],i,patchSizes);
  }
  readInC1Patches(dirname);
}

//computes the euclidean distance between each patch and all crops of images of
//similar size.
void HmaxFL::windowedPatchDistance(Image <float>* &images, int nimages, Image <float> *& patches, int npatches, Image<float>& D)
{

  // sum_over_p(W(p)-I(p))^2 is factored as
  // sum_over_p(W(p)^2) + sum_over_p(I(p)^2) - 2*(W(p)*I(p));

  // Im and Patch must have the same number of channels
  if(nimages != npatches) {
    // Error
    throw std::invalid_argument("Number of layers must be equal between patches and images");
  }

  Image<float> imSq(images[0].getWidth(),images[0].getHeight(),ZEROS);
  Image<float> imSqFiltered;
  Image<float> pi(images[0].getWidth(),images[0].getHeight(),ZEROS);
  float sumPSq=0;

  int s1 = patches[0].getWidth();
  int s2 = patches[0].getHeight();

  for(int i=0;i<nimages;i++) {

    //s = size(Patch);
    //Psqr = sum(sum(sum(Patch.^2)));
    sumPSq += sum(patches[i]*patches[i]);
    //Imsq = Im.^2;
    //Imsq = sum(Imsq,3);
    imSq += images[i]*images[i];
  }

  // tt, ll, bb, rr
  Rectangle sumSupport = Rectangle::tlbrI(int(ceil(s2/2)-1),int(ceil(s1/2)-1),int(floor(s2/2)),int(floor(s1/2)));
  //Imsq = sumfilter(Imsq,sum_support);
  sumFilter(imSq,sumSupport,imSqFiltered);
  for(int i=0;i<nimages;i++) {
    //PI = PI + conv2(Im(:,:,i),Patch(:,:,i), 'same');
    pi += convolve(images[i],patches[i],CONV_BOUNDARY_ZERO);
  }

  //D = Imsq - 2 * PI + Psqr + 10^-10;
  D = imSqFiltered - pi*2 + sumPSq + 10E-10F;

}


// ######################################################################
void HmaxFL::getC2(const Image<float>& input, float**& c2Res)
{
  if(c1Patches == 0){
    throw std::runtime_error("C1Patches must be initialized before determining c2 layer response");
  }

  // allocate buffers for intermediary results:

  Image<float> **c1Res;
  initC1(c1Res);

  // ******************************
  // *** Compute S1/C1 output:
  // ******************************
  getC1(input,c1Res);

  // ******************************
  // *** Compute S2/C2 output:
  // ******************************
  // New way, use c1Patches and windowedPatchDistance

  Image<float> s2Res;

  for(unsigned int ps = 0; ps < c1PatchSizes.size(); ps++) {
    for(int pi = 0; pi <nC1PatchesPerSize; pi++) {
      c2Res[ps][pi] = FLT_MAX;
      for(int sb = 0; sb < nsb; sb++){
        windowedPatchDistance(c1Res[sb],no,c1Patches[ps][pi],no,s2Res);
        //printCorners("s2Res",s2Res,ps==0&&pi==0&&sb==0);
        float mi, ma; getMinMax(s2Res,mi,ma);
        c2Res[ps][pi] = std::min(c2Res[ps][pi],mi);
      }
    }
  }

  clearC1(c1Res);
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
