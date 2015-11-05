/*!@file HMAX/Hmax.C Riesenhuber & Poggio's HMAX model for object recognition */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/HMAX/Hmax.C $
// $Id: Hmax.C 15310 2012-06-01 02:29:24Z itti $
//

#include "HMAX/Hmax.H"

#include "Image/FilterOps.H" // for convolve() etc.
#include "Image/Image.H"
#include "Image/Kernels.H"   // for dogFilter()
#include "Image/MathOps.H"
#include "Util/MathFunctions.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "Util/safecopy.H"

#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <limits>

// ######################################################################
Hmax::Hmax()
{ initialized = false; }

// ######################################################################
Hmax::Hmax(const int nori, const std::vector<int>& spacess,
           const std::vector<int>& scaless, const int c1spaceol,
           const bool angleflag, const float s2t, const float s2s,
           const float stdmin, const float stdstep,
           const int fsmin, const int fsstep)
{
  initialized = false;
  init(nori, spacess, scaless, c1spaceol, angleflag, s2t, s2s);
  initFilters(stdmin,stdstep,fsmin,fsstep);
  initialized = true; 
}

// ######################################################################
void Hmax::init(const int nori, const std::vector<int>& spacess,
                const std::vector<int>& scaless, const int c1spaceol,
                const bool angleflag, const float s2t, const float s2s)
{
  freeMem(); ns = scaless[scaless.size() - 1]; no = nori;
  c1SpaceOL = c1spaceol; angleFlag = angleflag; s2Target = s2t; s2Sigma = s2s;
  spaceSS.resize(spacess.size()); scaleSS.resize(scaless.size());

 // detrmine number of scale bands from length of vector scaleSS:
  nsb = scaleSS.size() - 1;

  for (unsigned int i = 0; i < spacess.size(); i ++) spaceSS[i] = spacess[i];
  for (unsigned int i = 0; i < scaless.size(); i ++) scaleSS[i] = scaless[i];

}

void Hmax::initFilters(const float stdmin, const float stdstep, const int fsmin, const int fsstep)
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
          filter[s][o] = dogFilter<float>(stdmin + stdstep * s,
                                          (float)o * 180.0F / (float)no,
                                          fsmin + fsstep * s);
          // normalize to zero mean:
          filter[s][o] -= mean(filter[s][o]);

          // normalize to unit of sum-of-squares:
          filter[s][o] /= sum(squared(filter[s][o]));
        }
    }
}

int Hmax::getNumOrientations()
{
  return no;
}

// ######################################################################
void Hmax::freeMem()
{
  if (initialized)
    {
      initialized = false;
      for(int s = 0; s < ns; s++) delete [] filter[s];
      delete [] filter;
    }
}

// ######################################################################
Hmax::~Hmax()
{ freeMem(); }

// ######################################################################
std::vector<std::string> Hmax::readDir(std::string inName)
{
        DIR *dp = opendir(inName.c_str());
        if(dp == NULL)
        {
          LFATAL("Directory does not exist %s",inName.c_str());
        }
        dirent *dirp;
        std::vector<std::string> fList;
        while ((dirp = readdir(dp)) != NULL ) {
                if (dirp->d_name[0] != '.')
                        fList.push_back(inName + '/' + std::string(dirp->d_name));
        }
        LINFO("%" ZU " files in the directory\n", fList.size());
        LINFO("file list : \n");
        for (unsigned int i=0; i<fList.size(); i++)
                LINFO("\t%s", fList[i].c_str());
        std::sort(fList.begin(),fList.end());
        return fList;
}

// ######################################################################
std::vector<std::string> Hmax::readList(std::string inName)
{
  std::ifstream inFile;
  inFile.open(inName.c_str(),std::ios::in);
  if(!inFile){
    LFATAL("Unable to open image path list file: %s",inName.c_str());
  }
  std::string sLine;
  std::vector<std::string> fList;
  while (std::getline(inFile, sLine)) {
      std::cout << sLine << std::endl;
      fList.push_back(sLine);
  }
  LINFO("%" ZU " paths in the file\n", fList.size());
  LINFO("file list : \n");
  for (unsigned int i=0; i<fList.size(); i++)
    LINFO("\t%s", fList[i].c_str());
  inFile.close();
  return fList;
}


// ######################################################################
// This is the code of Max Riesenhuber almost straight out of the box.
// Minor modifications have been made, to:
// - use float rather than double throughout
// - interface with our Image class rather than Matlab matrices
// - limit lines of code at 80 chars
// - allocate memory through malloc rather than Matlab functions
// - use Image<float> rather than Matlab matrices for result
// - changed fSiz, spaceSS, scaleSS from float to int (avoid compiler warnings)
// - made all pointers to input parameters pointers to const data
// - switched scaleSS to zero-based rather than Matlab's one-based
// - switched to using our built-in array of filters
Image<float> Hmax::origGetC2(const Image<float>& input)
{
  int offTab[8]={0,0,0,-1,-1,0,-1,-1};
  int imgy,imgx;
  int i,j,k,x,y,bufX,bufY;
  float *buf,*bStart,*currB;
  int fx,fy;
  float res;
  float imgLen;
  float *retPtr,*rPtr2;
  Image<float> retC2;
  float *ret,*c2Ptr;
  float *sBuf,*sBufPtr,*sPtr2;
  float *c1Act,s2Resp;
  int maxFS,sSS,scaleInd,numScaleBands,numSimpleFilters,numPos;
  int currScale,yS,xS,sFInd;
  int f1,f2,f3,f4,c2Ind,maxXY;
  int poolRange;

  Image<float>::const_iterator image = input.begin();
  imgy = input.getHeight();
  imgx = input.getWidth();

  maxFS = filter[ns - 1][0].getWidth(); // LI: use our built-in filters
  //for(maxFS=0,i=fSizY*fSizX-1;i>=0;i--) /* get maximum filter size */
  //  maxFS=fSiz[i]>maxFS?fSiz[i]:maxFS;

  /* for each interval in scaleSS, filter type, go through spaceSS,
     allocate enough mem to calc all these response fields in S1 &
     then do max over the required range */
  numScaleBands=scaleSS.size()-1; /* convention: last element in
                                        c1ScaleSS is max index + 1 */

  //  numScales=scaleSS[numScaleBands];
  /* last index in scaleSS contains scale index where next band would
     start, i.e., 1 after highest scale!! */

  numSimpleFilters = no; // LI: use our built-in filters
  // numSimpleFilters=fSizY*fSizX/numScales;

  /* calculate number of positions for each C-cell */
  /* base number of positions on smallest pooling range */
  numPos=(int)(ceil((double)(imgy/spaceSS[0]))*ceil((double)(imgx/spaceSS[0])))*
    c1SpaceOL*c1SpaceOL;

  /* be a little wasteful: in each filter band, allocate enough for
     smallest scale */
  retC2.resize(numSimpleFilters*numSimpleFilters,
               numSimpleFilters*numSimpleFilters);
  c2Ptr=retC2.getArrayPtr();
  /* allocate memory for C1 activations */
  ret=(float*)malloc(numPos*numScaleBands*numSimpleFilters*sizeof(float));

 /* s1 activations before pooling over space (sBuf already pools over
    *scale*) */
  sBuf=(float*)malloc(numSimpleFilters*imgy*imgx*sizeof(float));

  /* buf is buffer to perform convolutions in (with zero padding) */
  bufX=imgx+maxFS;
  bufY=imgy+maxFS;
  buf=(float*)malloc(bufX*bufY*sizeof(float));

  /* copy image and pad with zeros to half max filter size */
  memset(buf,0,bufX*bufY*sizeof(float));
  Image<float>::const_iterator currI = image;
  for(currB=buf+(maxFS>>1)*bufX+(maxFS>>1),i=0;i<imgy;i++,
        currI+=imgx,currB+=bufX)
    safecopy(currB,currI,imgx);

  for(scaleInd=0;scaleInd<numScaleBands;scaleInd++){
    memset(sBuf,0,numSimpleFilters*imgy*imgx*sizeof(float));
    for(currScale=scaleSS[scaleInd];currScale<scaleSS[scaleInd+1];currScale++){
      for(sBufPtr=sBuf,sFInd=0;sFInd<numSimpleFilters;sFInd++){
        fx = filter[currScale][sFInd].getWidth();
        fy = filter[currScale][sFInd].getHeight();
        for(y=0,bStart=buf+(maxFS>>1)*bufX+(maxFS>>1);y<imgy;y++,bStart+=
              bufX-imgx){
          for(x=0;x<imgx;x++,bStart++,sBufPtr++){
            /* center filter on current image point */
            Image<float>::const_iterator currF =
              filter[currScale][sFInd].begin();
            for(res=0,imgLen=0,currB=bStart-fy/2*bufX-fx/2,
                  j=0;j<fy;j++,currB+=bufX-fx)
              for(k=0;k<fx;k++){
                imgLen+=*currB**currB;
                res += *currB++**currF++;
              }
            if(angleFlag && (imgLen>0)) res/=sqrt(imgLen);
            res=fabs(res);
            *sBufPtr = *sBufPtr>res?*sBufPtr:res; /*already do max over scale*/
          }
        }
      }
    }

    /* now pool over space, take overlap into account */
    /* take ceiling here otherwise might get more than c1SpaceOL times */
    for(retPtr=ret+numPos*numSimpleFilters*scaleInd,sSS=(int)
          ceil((float)spaceSS[scaleInd]/c1SpaceOL),
          poolRange=spaceSS[scaleInd],sFInd=0;sFInd<numSimpleFilters;sFInd++)
      for(rPtr2=retPtr+numPos*sFInd,yS=0;yS<imgy;yS+=sSS)
        for(xS=0;xS<imgx;xS+=sSS,rPtr2++)
          /* eee still have same pooling range!!
             division by c1SpaceOL only in stepping of start pos! */
          for(*rPtr2=0.0,sBufPtr=sBuf+imgx*(imgy*sFInd+yS)+xS,y=yS;
              (y-yS<poolRange)&&(y<imgy);y++)
            for(sPtr2=sBufPtr+(y-yS)*imgx,x=xS;(x-xS<poolRange)&&(x<imgx);
                x++,sPtr2++)
              *rPtr2=*rPtr2>*sPtr2?*rPtr2:*sPtr2;
  }

  /* now: do S2 calculation by doing all combinations of features  */
  /* to make things a little more efficient, the outer loop runs over
     the 4 filters that a S2 cell combines, the inner loop does the calculation
     for all pos & filter bands & takes the max (ret contains C2 then w/out
     exp) */
  for(c1Act=ret,c2Ind=0,f1=0;f1<numSimpleFilters;f1++)
    for(f2=0;f2<numSimpleFilters;f2++)
      for(f3=0;f3<numSimpleFilters;f3++)
        for(f4=0;f4<numSimpleFilters;f4++,c2Ind++){
          for(c2Ptr[c2Ind]=res=-1e10,scaleInd=0;scaleInd<numScaleBands;
              scaleInd++){
            /* eee assume square image */
            for(maxXY=(int)ceil(imgy/ceil((float)
                                          spaceSS[scaleInd]/c1SpaceOL)),
                  y=c1SpaceOL;y<maxXY;y++)
              for(x=c1SpaceOL;x<maxXY;x++){
                /* use the fact that exp is monotonous in abs(arg):
                   just pass back max of neg. dist (arg of exp) */
                s2Resp=squareOf<float>(c1Act[numPos*(scaleInd*numSimpleFilters+f1)+
                                y+maxXY*x]-s2Target)+
                  squareOf<float>(c1Act[numPos*(scaleInd*numSimpleFilters+f2)+y+
                           c1SpaceOL*offTab[2]+
                           maxXY*(x+c1SpaceOL*offTab[3])]-s2Target)+
                  squareOf<float>(c1Act[numPos*(scaleInd*numSimpleFilters+f3)+y+
                           c1SpaceOL*offTab[4]+
                           maxXY*(x+c1SpaceOL*offTab[5])]-s2Target)+
                  squareOf<float>(c1Act[numPos*(scaleInd*numSimpleFilters+f4)+y+
                           c1SpaceOL*offTab[6]+
                           maxXY*(x+c1SpaceOL*offTab[7])]-s2Target);
                res=s2Resp>res?s2Resp:res;
              }
            c2Ptr[c2Ind]=c2Ptr[c2Ind]>res?c2Ptr[c2Ind]:res; /*max over scale*/
          }
        }
    free(sBuf);
    free(buf);
    free(ret);
    return hmaxActivation(retC2, s2Sigma);
}


// ######################################################################
void Hmax::getC1(const Image<float>& input, Image<float>** &c1Res)
{
  Image<float> *c1Buff = new Image<float>[no];
  Image<float> s1Res;
  // loop over scale bands:


  for(int sb = 0; sb < nsb; sb ++) {

    // clear our buffers:
    for(int i = 0; i < no; i ++)
      c1Buff[i].resize(input.getWidth(), input.getHeight(), true);;

    // loop over scales within current scale band:
    for(int s = scaleSS[sb]; s < scaleSS[sb + 1]; s++) {
      // loop over orientations:
      for(int o = 0; o < no; o ++) {
        // convolve image by filter at current orient & scale:
        if (angleFlag) {
          s1Res = convolveHmax(input, filter[s][o]); // normalize by image energy
          //printCorners("s1Res",s1Res,s==scaleSS[0]&&sb==0&&o==0);
        }
        else {
          s1Res = convolve(input, filter[s][o], CONV_BOUNDARY_CLEAN); // no normalization
          // take absolute value of the convolution result:
          s1Res = abs(s1Res);
        }

        // take max between this convolution and previous ones for
        // that orientation but other scales within current scale band:
        c1Buff[o] = takeMax<float>(c1Buff[o], s1Res);
      }
    }

    // compute RF spacing (c1R) and pool range (c1PR):
    int c1R = (int)ceil((float)spaceSS[sb] / (float)c1SpaceOL);
    int c1PR = spaceSS[sb];

    // pool over space for each orientation (and scale band):
    for(int o = 0; o < no; o ++){
      c1Res[sb][o] = spatialPoolMax(c1Buff[o], c1R, c1R, c1PR, c1PR);
      //printCorners("c1Res",c1Res[sb][o],sb==0&&o==0);
    }

  }

  delete [] c1Buff;
}

void Hmax::initC1(Image<float> **&c1Res)
{
  c1Res = new Image<float>*[nsb];
  for (int sb = 0; sb < nsb; sb ++) c1Res[sb] = new Image<float>[no];
}

void Hmax::clearC1(Image<float> **&c1Res)
{
   for (int sb = 0; sb < nsb; sb ++) delete [] c1Res[sb];
   delete [] c1Res;
}

void Hmax::printCorners(const char name[], const Image<float>& im, bool cond)
{
  if(cond) {
    printf("%s\n",name);
    int w = im.getWidth();
    int h = im.getHeight();
    /*
    std::string s;
    if(w>2 && h>2) {
      printf("%f\t%f\t%f\t%f\t%f\n",im.getVal(0,0),im.getVal(1,0),im.getVal(2,0),im.getVal(w-2,0),im.getVal(w-1,0));
      printf("%f\t%f\t\t\t%f\t%f\n\n", im.getVal(0,1),im.getVal(1,1),im.getVal(w-2,1),im.getVal(w-1,1));
      printf("%f\t%f\t\t\t%f\t%f\n", im.getVal(0,h-2),im.getVal(1,h-2),im.getVal(w-2,h-2),im.getVal(w-1,h-2));
      printf("%f\t%f\t%f\t%f\t%f\n",im.getVal(0,h-1),im.getVal(1,h-1),im.getVal(2,h-1),im.getVal(w-2,h-1),im.getVal(w-1,h-1));
    }
    else if(w>1 && h>1) {
      printf("%f\t%f\n",im.getVal(0,0),im.getVal(1,0));
      printf("%f\t%f\n", im.getVal(0,1),im.getVal(1,1));
    }
    else if(w>0 && h>0){
      printf("%f\n",im.getVal(0,0));
    }
    */
    std::cout << "Mean of " << name << " " << mean(im) << std::endl;
    std::cout << "Var of " << name << " " << (stdev(im)*stdev(im)) << std::endl;
    std::cout << "Width of " << w << " and height of " << h << std::endl;
    //float mi,ma; getMinMax(input,mi,ma);
    //writeOutImage(inputf,name);
    //std::cout << "Min " << mi << " Max " << ma << std::endl;
  }
}

void Hmax::writeOutImage(const Image<float>& im,std::string & fName)
{
  std::ofstream oFile;
  Image<float> d;
  oFile.open(fName.c_str(),std::ios::out);
  d = im;
  int w,h;
  w = d.getWidth();
  h = d.getHeight();
  for(int i=0;i<w;i++){
    for(int j=0;j<h;j++){
      oFile << d.getVal(i,j) <<" ";
    }
    if(i!=w-1)
      oFile << std::endl;
  }
  oFile.close();

}


// ######################################################################
Image<float> Hmax::getC2(const Image<float>& input)
{
  // detrmine number of scale bands from length of vector scaleSS:
  int nsb = scaleSS.size() - 1;

  // allocate buffers for intermediary results:
  Image<float> **c1Res;
  initC1(c1Res);

  // ******************************
  // *** Compute S1/C1 output:
  // ******************************
  getC1(input, c1Res);

  // ******************************
  // *** Compute S2/C2 output:
  // ******************************
  Image<float> c2Res(no * no, no * no, NO_INIT);
  c2Res.clear(-1.0E10F);
  int idx = 0;

  // loop over four filters giving inputs to an S2 cell:
  for (int f1 = 0; f1 < no; f1++)
    for (int f2 = 0; f2 < no; f2++)
      for (int f3 = 0; f3 < no; f3++)
        for (int f4 = 0; f4 < no; f4++) {

          float c2r = -1.0E10;
          // loop over scale band:
          for (int sb = 0; sb < nsb; sb ++) {

            float s2r = featurePoolHmax(c1Res[sb][f1], c1Res[sb][f2],
                                        c1Res[sb][f3], c1Res[sb][f4],
                                        c1SpaceOL, c1SpaceOL, s2Target);
            if (s2r > c2r) c2r = s2r;
          }
          c2Res.setVal(idx, c2r); idx ++;
        }

  // free allocated temporary images:
  clearC1(c1Res);

  return hmaxActivation(c2Res, s2Sigma);
}

void Hmax::sumFilter(const Image<float>& image, const float radius, Image<float>& newImage)
{
  Rectangle sumSupport = Rectangle::tlbrI(0,0,int(radius*2.0F),int(radius*2.0F));
  sumFilter(image,sumSupport,newImage);
}

void Hmax::sumFilter(const Image<float>& image, const Rectangle& support, Image<float>& newImage)
{

  Dims d(support.top()+support.bottomI()+1,support.left()+support.rightI()+1);
  Image<float> a(d,NO_INIT);
  a.clear(1.0F);

  //convolve the image with a matrix of 1's that is as big as the rectangle
  // This two step process is doing effectively the same thing by taking the center part of the convolution
  //I2 = conv2(ones(1,radius(2)+radius(4)+1), ones(radius(1)+radius(3)+1,1), I);
  //I3 = I2((radius(4)+1:radius(4)+size(I,1)), (radius(3)+1:radius(3)+size(I,2)));
  //Image<float> i;
  //i = convolution(image,a,MATLAB_STYLE_CONVOLUTION);
  //Rectangle r = Rectangle::tlbrI(support.bottomI()+1,support.rightI()+1,support.bottomI()+image.getHeight(),support.rightI()+image.getWidth());
  //newImage = crop(i,r);
  // Can be done in one step
  newImage = convolve(image,a,CONV_BOUNDARY_ZERO);
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
