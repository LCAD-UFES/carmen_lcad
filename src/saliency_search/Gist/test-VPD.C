/*!@file Gist/test-VPD.C testing Vanishing Point Detector algorithm */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-VPD.C $
// $Id: test-VPD.C 14770 2011-05-19 04:19:46Z kai $
//
//////////////////////////////////////////////////////////////////////////

// Implementation of the algorithm described in:
//
// General Road Detection From a Single Image
//
// Hui Kong, Jean-Yves Audibert, and Jean Ponce,
// IEEE TRANSACTIONS ON IMAGE PROCESSING, VOL. 19, NO. 8, AUGUST 2010


#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameIstream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/DrawOps.H" //for writeText and SimpleFont
#include "Image/ColorOps.H" //for luminance
#include "Image/MathOps.H" //for inplaceNormalize() 
#include "Image/Normalize.H" //for inplaceNormalize() 
#include "GUI/ImageDisplayStream.H"
#include <cstdio>//for sprintf
#include "Gist/VanishingPointDetector.H" 
#include "Util/Timer.H"



Image<PixRGB<byte> > toColor(Image<float> input)
{
	return toRGB(normalizeFloat(input, true));
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
	if (manager.parseCommandLine(argc, argv, "[pyr_size] [angle] [min_size]", 0, 4) == false) return(1);


	// let's do it!
	manager.start();




	bool keepGoing = true; uint index = 0;
	//Point2D<int> current_vp = Point2D<int>(0,0);
	//Dims vp_region;  
	//Image<float> voteMap; 
	rutz::shared_ptr<VanishingPointDetector>vpd (new VanishingPointDetector());

	Timer timer(1000000); float time = 0.0;
	while(keepGoing)
	{
		ifs->updateNext();
		Image<PixRGB<byte> > ima = ifs->readRGB();

		if(!ima.initialized()) { keepGoing = false; }
		else
		{

			//char buffer[200];

			uint w = ima.getWidth();
			uint h = ima.getHeight();


			if(w > 320 ||w/h != 320/240){
				LINFO("rescale input to 160/120");
				ima = rescale(ima,160,120);
				w = ima.getWidth();
				h = ima.getHeight();
			}


			Image<PixRGB<byte> > dispIma(w*10, h*4, NO_INIT);

			timer.reset();
			time = timer.get()/1000.0F;
			vpd->updateImage(ima);
			LINFO("Total time for one frame: %f", timer.get()/1000.0F - time);time = timer.get()/1000.0F;
			



			Image<float> confMap = vpd->getConfidenceMapImg();
//			Image<int> indexMap = vpd->getIndexMap();
			Image<float> voteMap = vpd->getVoteMapImg();
			Image<PixRGB<byte> > oriSet = vpd->getOrientationSetDisplay();//FIXXX
                        vpd->computeOutputImage(ima.getDims());
			Image<PixRGB<byte> > outputIma = vpd->getOutputImage();
			Image<PixRGB<byte> > outputFiltedIma = vpd->getFilteredImage();
			Image<float> searchMemoryMap = vpd->getSearchMemoryMap();
			
			Dims x2 = ima.getDims()*2;
			inplacePaste(dispIma, rescale(outputFiltedIma,x2), Point2D<int>(0, 0));
			inplacePaste(dispIma, rescale(toColor(confMap),x2), Point2D<int>(2*w, 0));
			inplacePaste(dispIma, rescale(toColor(voteMap),x2), Point2D<int>(0, 2*h));
			inplacePaste(dispIma, rescale(toColor(searchMemoryMap),x2), Point2D<int>(2*w, 2*h));

			time = timer.get()/1000.0F;

                        LINFO("%d %d --> %d %d", outputIma.getWidth(),  outputIma.getHeight(), dispIma.getWidth(), dispIma.getHeight());



			inplacePaste(dispIma, rescale(outputIma,x2*2), Point2D<int>(6*w, 0));
			LINFO("Compute Boundary time: %f", timer.get()/1000.0F - time);time = timer.get()/1000.0F;



			//Auto scale down if image is too big
			if(oriSet.getWidth() > (int)w*12){
				float res = (float)oriSet.getWidth()/(float)(w*6.0);
				inplacePaste(dispIma, rescale(oriSet,oriSet.getDims()/res), Point2D<int>(4*w, 0));
			}else
				inplacePaste(dispIma, rescale(oriSet,oriSet.getDims()/2), Point2D<int>(4*w, 0));
			ofs->writeRGB(dispIma, "Vanishing Point Detector");
			ofs->updateNext();

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
