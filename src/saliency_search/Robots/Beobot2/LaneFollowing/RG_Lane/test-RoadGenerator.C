/*!@file Gist/test-RoadGenerator.C testing Road Generator */
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
//
//////////////////////////////////////////////////////////////////////////



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
#include "Image/ShapeOps.H" //for rescale() 
#include "GUI/ImageDisplayStream.H"
#include <cstdio>//for sprintf
#include "Robots/Beobot2/LaneFollowing/RG_Lane/RoadGenerator.H" 
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

	//nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
	//manager.addSubComponent(ifs);

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
	rutz::shared_ptr<RoadGenerator>rg(new RoadGenerator(320,240));

	Timer timer(1000000); float time = 0.0;
	while(keepGoing)
	{
		//ifs->updateNext();
		Image<PixRGB<byte> > ima = rg->getRoad();
		int w = ima.getWidth();
		int h = ima.getHeight();



			Image<PixRGB<byte> > dispIma(w, h, NO_INIT);

			timer.reset();
			time = timer.get()/1000.0F;
			LINFO("Total time for one frame: %f", timer.get()/1000.0F - time);time = timer.get()/1000.0F;
			inplacePaste(dispIma, ima, Point2D<int>(0, 0));

			ofs->writeRGB(dispIma, "Road Generator");
			ofs->updateNext();

		}
	Raster::waitForKey();
	index++;


	return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
