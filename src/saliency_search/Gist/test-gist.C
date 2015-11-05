/*!@file Gist/test-gist.C extract gist from an image
         using available features from the image                          */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-gist.C $
// $Id: test-gist.C 14884 2011-08-17 02:26:20Z kai $
//

// ######################################################################
/*! display and save gist information                                   */

#include "Component/ModelManager.H"
#include "Component/RawGistEstimatorGen.H"
#include "Channels/RawVisualCortex.H"
#include "GUI/XWinManaged.H"
#include "Image/DrawOps.H"
#include "Image/Pixels.H"
#include "Neuro/gistParams.H"
#include "Raster/Raster.H"
#include "Util/Types.H"
#include "Media/MPEGStream.H"
#include "Media/MediaOpts.H"
#include "Channels/ChannelOpts.H"
#include "Channels/InputFrame.H"

#include "Media/FrameSeries.H"
#include "Transport/FrameIstream.H"


#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <math.h>
#include <stdio.h>

// ######################################################################
void saveData(Image<double> data, std::string fName);

// ######################################################################
// training procedure
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // Instantiate a ModelManager:
  ModelManager manager("test-gist Model");

  // Instantiate our various ModelComponents:
	nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

	nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
	manager.addSubComponent(ofs);

  nub::soft_ref<RawVisualCortex> vcx(new RawVisualCortex(manager));
	manager.addSubComponent(vcx);
  // get the GistEstimator
  nub::soft_ref<RawGistEstimatorGen> ge(new RawGistEstimatorGen(manager));
	manager.addSubComponent(ge);

  manager.setOptionValString(&OPT_SingleChannelSaveRawMaps, "true");
  manager.setOptionValString(&OPT_GistEstimatorType,"Std");

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv,"<filename stem> ", 0, 1) == false)
    return(1);


  // do post-command-line configs:
  //   GenericFrameSpec fs =ifs->peekFrameSpec();
  //   int w = fs.getWidth(),  h = fs.getHeight();
  //   XWinManaged iWin(img.getDims(),0,0, "win");
  //lzc int s = 20;
  //lzc XWinManaged gWin(Dims(NUM_GIST_COL * s, NUM_GIST_FEAT * s), 0,0,"Histogram");

  // let's get all our ModelComponent instances started:
  manager.start();

  bool keepGoing = true;
  // main loop:
	while(keepGoing)
	{
		ifs->updateNext();
		Image<PixRGB<byte> > ima = ifs->readRGB();

		if(!ima.initialized()) { keepGoing = false; }
		else
		{
			const InputFrame ifr = InputFrame::fromRgb(&ima);
			vcx->input(ifr);
			rutz::shared_ptr<ChannelMaps> chm = rutz::make_shared(new ChannelMaps(vcx.get()));
		  Image<double> gistV = ge->compute(chm);

//
			// save data to a file
			if(manager.numExtraArgs() > 0)
			{
				std::string name
					(sformat("%s_%06d.gist", manager.getExtraArg(0).c_str(),ifs->frame()));
				LINFO("saving: %s", name.c_str());
				saveData(gistV, name);
			}

			ofs->writeRGB(ima, "Vanishing Point Detector");
			ofs->updateNext();

		}
	}
  // stop all our ModelComponents
  //lzc Raster::waitForKey();
  manager.stop();
}

// ######################################################################
// save the gist feature vector to a file
void saveData(Image<double> data, std::string fName)
{
  // change the extension to .gist
  int ldpos = fName.find_last_of('.');
  std::string gName = fName.substr(0, ldpos) + std::string(".gist");
  LINFO("gist file name: %s", gName.c_str());

  // write the data to the gist file
  Image<double>::iterator aptr = data.beginw();
  FILE *gfp; if((gfp = fopen(gName.c_str(),"wb")) != NULL)
    {
      for(int i = 0; i < data.getSize(); i++)
        { double val = *aptr++; fwrite(&val, sizeof(double), 1, gfp); }
      fclose(gfp);
    }
  else LFATAL("can't write: %s", gName.c_str());

//   double in[NUM_GIST_COL*NUM_GIST_FEAT];
//   gfp = fopen(gName,"rb");
//   fread(in,sizeof(double), NUM_GIST_COL*NUM_GIST_FEAT, gfp);
//   fclose(gfp);

//   for(int i = 0; i < NUM_GIST_COL; i++)
//     {
//       for(int j = 0; j < NUM_GIST_FEAT; j++)
//      printf("%5.3f ",in[i*NUM_GIST_FEAT+j]);
//       printf("\n");
//     }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
