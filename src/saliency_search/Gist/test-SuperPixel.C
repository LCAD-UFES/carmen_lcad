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
// $Id: test-SuperPixel.C 15342 2012-07-24 22:41:24Z kai $
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
#include "GUI/ImageDisplayStream.H"
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


int main(int argc, char **argv)
{

	// instantiate a model manager:
	ModelManager manager("test SuperPixel");

	// Instantiate our various ModelComponents:

	nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
	manager.addSubComponent(ifs);

	//nub::ref<GraphBasedSegmenter> sps(new SuperPixelSegmenter(manager));
	//manager.addSubComponent(sps);

	nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
	manager.addSubComponent(ofs);

	manager.exportOptions(MC_RECURSE);

	// Parse command-line:
	if (manager.parseCommandLine(argc, argv,
				"[save_output_to_file] "
				"[sigma] [k] [min_size]",
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
	float sigma = .5; uint k = 500; uint minSize = 20;
	if(manager.numExtraArgs() > 1)
		sigma = manager.getExtraArgAs<float>(1);
	if(manager.numExtraArgs() > 2)
		k = manager.getExtraArgAs<uint>(2);
	if(manager.numExtraArgs() > 3)
		minSize = manager.getExtraArgAs<uint>(3);

	// let's do it!
	manager.start();
	frameQue<std::vector<Image<float> > > channel3Dque ;//= frameQue(5);//use 5 frame per segment
	channel3Dque.setSize(1);
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
			int num_ccs;



			Image<byte> r, g, b; getComponents(ima, r, g, b);

			std::vector<Image<float> > channel;
			channel.push_back(r);	
			channel.push_back(g);	
			channel.push_back(b);	

			channel3Dque.add(channel);


			std::vector<std::vector<Point3D<int> > > groups;
			Image<int> groupImage = SuperPixelSegment3D (ima,ima,sigma,k,minSize,num_ccs,&groups,channel3Dque.getVector());
			Image<PixRGB<byte> > output = SuperPixelDebugImage(groups,ima);

			Image<PixRGB<byte> > output2 = 
				segment_image(ima, sigma, k, minSize, num_ccs);//Single Image

			Image<float> ll, aa, bb; getNormalizedLAB(ima, ll, aa, bb);//Test LAB Color Space





			time = timer.get()/1000.0F;
			LINFO("time: %f", time);
			int w = ima.getWidth();
			int h = ima.getHeight();
			Image<PixRGB<byte> >
				dispIma(ima.getWidth()*3, ima.getHeight()*2, NO_INIT);
			char buffer[200];
			sprintf(buffer,"Input %3d",index);
			writeText(ima,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
			sprintf(buffer,"3D SuperPixel Z=%2d",(int)channel3Dque.getVector().size());
			writeText(output,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
			sprintf(buffer,"Single SuperPixel");
			writeText(output2,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));


			inplacePaste(dispIma, ima, Point2D<int>(0, 0));
			inplacePaste(dispIma, output, Point2D<int>(ima.getWidth(), 0));
			inplacePaste(dispIma, output2, Point2D<int>(ima.getWidth()*2, 0));

			inplacePaste(dispIma, (Image<PixRGB<byte> >)toRGB(normalizeFloat(ll, true)), Point2D<int>(ima.getWidth()*0, ima.getHeight()));
			inplacePaste(dispIma, (Image<PixRGB<byte> >)toRGB(normalizeFloat(aa, true)), Point2D<int>(ima.getWidth()*1, ima.getHeight()));
			inplacePaste(dispIma, (Image<PixRGB<byte> >)toRGB(normalizeFloat(bb, true)), Point2D<int>(ima.getWidth()*2, ima.getHeight()));
			sprintf(buffer,"L");
			writeText(dispIma,Point2D<int>(0,h),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
			sprintf(buffer,"A");
			writeText(dispIma,Point2D<int>(w,h),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
			sprintf(buffer,"B");
			writeText(dispIma,Point2D<int>(w*2,h),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
			//       inplacePaste(dispIma, output2, Point2D<int>(2*ima.getWidth(), 0));



			ofs->writeRGB(dispIma, "SuperPixel Segmentation");
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
