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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-SuperPixel.C $
// $Id: test-SuperPixel.C 14770 2011-05-19 04:19:46Z kai $
//
//////////////////////////////////////////////////////////////////////////
//
// Implementation of the segmentation algorithm described in:
//
// Efficient Graph-Based Image Segmentation
// Pedro F. Felzenszwalb and Daniel P. Huttenlocher
// International Journal of Computer Vision, 59(2) September 2004.
// Copyright (C) 2006 Pedro Felzenszwalb


#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameIstream.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/LowPass.H"
#include "Image/ShapeOps.H"
#include "Image/DrawOps.H" //for writeText and SimpleFont
#include "Image/ImageSet.H" //for ImageSet
#include "Image/PyrBuilder.H"
#include "Image/PyramidOps.H" //for buildPyrGabor
#include "GUI/ImageDisplayStream.H"
#include "Util/SortUtil.H" //for sortindex() 
#include <cstdio>//for sprintf
// #include <cstdlib>
// #include <image.h>
// #include <misc.h>
// #include <pnmfile.h>

#include "Gist/SuperPixel.H"
#include "Util/Timer.H"
template<typename T>
class frameQue{
        // Prevent copy and assignment
        frameQue(const frameQue&);
        frameQue& operator=(const frameQue&);
        //Max size of data
        int m_size;
        //Current size total
        int m_total;
        std::deque<T> m_data;

public:
        frameQue():m_size(0)
        {
		m_total = 0;
        }
        frameQue(int n):m_size(0)
        {
                m_size = n;
		m_total = 0;
        }
        void add(const T& data){
                // if the queue is full yet, we pop the oldest data out
                if(m_total >= m_size)
                {
                        m_data.pop_back();
                }else{
                        m_total++;
                }
                m_data.push_front(data);
        }
	void setSize(int n)
	{
		m_size = n;
	}
        std::vector<T> getVector()
        {
                return std::vector<T>(m_data.begin(),m_data.end());
        }
        ~frameQue(){}
};

Image<float> averageImageSet(ImageSet<float> iset){

	uint numLevels = iset.size();
	uint w = iset[0].getWidth();
	uint h = iset[0].getHeight();
	Image<float> avgIma(w, h, ZEROS);
	uint totalUsableLayers = 0;
	for (uint i = 0; i < numLevels; i++)
	{
		uint scale = pow(2.0, i);

		if(w == uint(iset[i].getWidth()  * scale) && 
				h == uint(iset[i].getHeight() * scale) )
		{
			Image<float> temp = iset[i];
			inplaceNormalize(temp, 0.0F, 255.0F);
			Image<float> tempZoom = zoomXY(temp,scale);
			//LINFO("zoom w %d h %d",tempZoom.getWidth(),tempZoom.getHeight());
			avgIma+=tempZoom;
			totalUsableLayers ++;
		}
		else{ LINFO("Size not fit. skip"); }
	}
	return avgIma/totalUsableLayers;

}
float averageRange(std::vector<float> v,uint start,uint end){
	float sum = 0.0; uint count = 0 ;
	for(uint i = start; i < end ; i++){
		sum += v[i];
		count++;
	}
	float avg = sum/count;
	
	return avg;

}
Image<float> confImage(ImageSet<float> iset){

	uint numLevels = iset.size();
	uint w = iset[0].getWidth();
	uint h = iset[0].getHeight();
	Image<float> confMap(w, h, ZEROS);
	for (uint x = 0 ; x < w ; x++)
		for (uint y = 0 ; y < h ; y++)
		{
			//build pixel array
			std::vector<float> vec,sorted_vec;
			std::vector<size_t> ix;
			for(uint z = 0 ; z < numLevels ;z++) vec.push_back(iset[z].getVal(x,y));

			util::sortindex (vec,sorted_vec,ix);

			
//			for(size_t di = 0 ;di < sorted_vec.size();di++){
//				printf("vec[%d]=%f ",(int)di,sorted_vec[di]);
//			}
//			printf("\n");

			float avgR5_R15 = averageRange(sorted_vec,numLevels-14,numLevels-4);
			float max = sorted_vec[numLevels-1];
			float conf = 1 - (avgR5_R15/max);
			//LINFO("%d,%d:avgr5~r15=%f,max=%f,conf %f",x,y,avgR5_R15,max,conf);			
			//Raster::waitForKey();

			confMap.setVal(x,y,conf);

		}
	inplaceNormalize(confMap,0.0F,1.0F);
	return confMap;	

}
Image<float> thresholdConf(Image<float> confMap,float t){
	
	uint w = confMap.getWidth();
	uint h = confMap.getHeight();
	Image<float> thresholdConfMap(w, h, ZEROS);
	for (uint x = 0 ; x < w ; x++){
		for (uint y = 0 ; y < h ; y++)
		{
			float val = confMap.getVal(x,y);
			
			if(val > t)
				thresholdConfMap.setVal(x,y,val);
			//printf("%3.3f ",val);
		}
		//printf("\n");
	}
	return thresholdConfMap;


}

int main(int argc, char **argv)
{

  // instantiate a model manager:
  ModelManager manager("test SuperPixel");

  // Instantiate our various ModelComponents:

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  manager.exportOptions(MC_RECURSE);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv,
                               "[save_output_to_file] "
                               "[pyr_size] [angle] [min_size]",
                               0, 4)
      == false) return(1);

  // get the operation mode
  bool saveOutput = false;
  std::string outFilename;
  if(manager.numExtraArgs() >  0)
    {
      saveOutput = true;
      outFilename = manager.getExtraArg(0);
      LINFO("save to: %s", outFilename.c_str());
    }

  // default parameters for the Superpixel segmentation
  //if(manager.numExtraArgs() > 1)
  //  sigma = manager.getExtraArgAs<float>(1);
  //if(manager.numExtraArgs() > 2)
  //  k = manager.getExtraArgAs<uint>(2);
  //if(manager.numExtraArgs() > 3)
  //  minSize = manager.getExtraArgAs<uint>(3);

  // let's do it!
  manager.start();
  frameQue<std::vector<Image<float> > > channel3Dque ;//= frameQue(5);//use 5 frame per segment
  channel3Dque.setSize(2);
  //std::vector<std::vector<Image<float> > >channel3D;



   bool keepGoing = true; uint index = 0;
    while(keepGoing)
      {

  Timer timer(1000000); float time;
      
  ifs->updateNext();
  Image<PixRGB<byte> > ima = ifs->readRGB();
  if(!ima.initialized()) { keepGoing = false; }
  else
    {
      timer.reset();

         
      time = timer.get()/1000.0F;
      LINFO("time: %f", time);
      
      	char buffer[200];

	float oriIntDegree = 5;//every 5 degree out of 180
	float initAngle = 0.0f;
	uint oriAngleNum = 180/oriIntDegree;//180/5=36
	uint oriAngleNumHalf = oriAngleNum/2;
	uint w = ima.getWidth();
	uint h = ima.getHeight();
      Image<PixRGB<byte> > dispIma(w*oriAngleNumHalf, h*4, NO_INIT);
	Image<float> const fIma = luminance(ima);

ImageSet<float> gaborOriSet(oriAngleNum);
for(uint angIndex = 0;angIndex < oriAngleNum ;angIndex++){
  float angle = oriIntDegree*angIndex + initAngle;
  ImageSet<float> pyr =
    buildPyrGabor(fIma, 0, 4, angle, 2.20, 1.0, 5);
	//uint numLevels = pyr.size();
	Image<PixRGB<byte> > disp(w, h, ZEROS);

	//Only for display
	/*
	LINFO("Depth: %d Dim(%d,%d)", numLevels, w, h);  
	for (uint i = 0; i < numLevels; i++)
	{
		uint scale = pow(2.0, i);

		if(w >= uint(pyr[i].getWidth()  * scale) && 
				h >= uint(pyr[i].getHeight() * scale) )
		{
			Image<float> temp = pyr[i];
			//float mn,mx; getMinMax(images[i], mn,mx);
			//LINFO("min: %f, max: %f", mn, mx);

			inplaceNormalize(temp, 0.0F, 255.0F);
			//Image<PixRGB<byte> > tempRGB = toRGB(zoomXY(temp,scale));
			Image<PixRGB<byte> > tempRGB = toRGB(temp);
			inplacePaste(disp, tempRGB , Point2D<int>(w*i,  0));

			sprintf(buffer,"Pyr %d",i);
			writeText(disp,Point2D<int>(w*i,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
		}
		else{ LINFO("Too big. Not drawn."); }
	}*/

	Image<float > avgFimage = averageImageSet(pyr);
	Image<PixRGB<byte> > avgImage = toRGB(avgFimage);
	sprintf(buffer,"Angle %2.2f",angle);
	writeText(avgImage,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
	int ph = angIndex/oriAngleNumHalf;

        inplacePaste(dispIma, avgImage, Point2D<int>(w*(angIndex%oriAngleNumHalf), ph*h));

	gaborOriSet[angIndex] = avgFimage;
}
	Image<float> confMap = confImage(gaborOriSet); 
	Image<float > tconfMap = thresholdConf(confMap,0.5);
	

	inplaceNormalize(confMap, 0.0F, 255.0F);
	Image<PixRGB<byte> > color_confMap = toRGB(confMap);
        inplacePaste(dispIma, color_confMap, Point2D<int>(0, 2*h));
	inplaceNormalize(tconfMap, 0.0F, 255.0F);
	Image<PixRGB<byte> > color_tconfMap = toRGB(tconfMap);
        inplacePaste(dispIma, color_tconfMap, Point2D<int>(w, 2*h));
//	sprintf(buffer,"Input %3d",index);
//	writeText(ima,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

//        inplacePaste(dispIma, ima, Point2D<int>(0, 0));




      ofs->writeRGB(dispIma, "Gabor Pyr");
      ofs->updateNext();
      
      if(saveOutput)
        Raster::WriteRGB(dispIma, outFilename);
      
    }
     index++;
   }
   Raster::waitForKey();


  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
