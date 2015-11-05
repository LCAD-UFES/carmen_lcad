/*!@file Gist/test-Arc.C testing drawArc algorithm */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-Arc.C $
// $Id: test-Arc.C 14770 2011-05-19 04:19:46Z kai $
//
//////////////////////////////////////////////////////////////////////////

// Implementation of the algorithm described in:


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
#include "Util/Timer.H"



Image<PixRGB<byte> > toColor(Image<float> input)
{
	return toRGB(normalizeFloat(input, true));
}
int main(int argc, char **argv)
{

	// instantiate a model manager:
	ModelManager manager("test DrawArc");

	// Instantiate our various ModelComponents:


	nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
	manager.addSubComponent(ofs);

	manager.exportOptions(MC_RECURSE);

	// Parse command-line:
	if (manager.parseCommandLine(argc, argv, "[pyr_size] [angle] [min_size]", 0, 4) == false) return(1);


	// let's do it!
	manager.start();
	
	for(int s = 0;s < 360;s++)
	{
		for(int e = 0;e < 360;e++)
		{
						Image<PixRGB<byte> > dispIma(800, 600, ZEROS);
						drawArc(dispIma,Point2D<int>(150,150),100,PixRGB<byte>(255,0,0),s/180.0*M_PI,e/180.0*M_PI);
						ofs->writeRGB(dispIma, "Test DrawArc");
						ofs->updateNext();

		}


	}
	//drawArc(dispIma,Point2D<int>(400,300),Point2D<int>(400,500),PixRGB<byte>(255,0,0),ang30,ang130);
	Raster::waitForKey();


	return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
