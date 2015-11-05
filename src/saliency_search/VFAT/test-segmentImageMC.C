/*!@file VFAT/test-segmentImageMC.C track color in grabbed frame        */
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
// Primary maintainer for this file:  T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/test-segmentImageMC.C $
// $Id: test-segmentImageMC.C 6988 2006-08-11 17:15:38Z rjpeters $
//

#include "Component/ModelManager.H"
#include "Devices/FrameGrabberConfigurator.H"
#include "GUI/XWindow.H"
#include "Raster/Raster.H"
#include "Transport/FrameIstream.H"
#include "VFAT/segmentImageTrackMC.H"
#include "rutz/shared_ptr.h"

#include <cstdio>
#include <cstdlib>

// number of frames over which framerate info is averaged:
#define NAVG 20

// ######################################################################
//! A basic test program for segmentImageMC tracker
int main(const int argc, const char **argv)
{
  // instantiate a model manager:
  ModelManager manager("Frame Grabber Tester");

  // Instantiate our various ModelComponents:
  nub::soft_ref<FrameGrabberConfigurator>
    gbc(new FrameGrabberConfigurator(manager));
  manager.addSubComponent(gbc);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false) return(1);

  // do post-command-line configs:
  nub::soft_ref<FrameIstream> gb = gbc->getFrameGrabber();
  if (gb.isInvalid())
    LFATAL("You need to select a frame grabber type via the "
           "--fg-type=XX command-line option for this program "
           "to be useful");
  int width = gb->getWidth(), height = gb->getHeight();

  // let's get all our ModelComponent instances started:
  manager.start();
  XWindow wini(Dims(width, height), 0, 0, "test-input window");
  XWindow wino(Dims(width/4, height/4), 0, 0, "test-output window 2");
  XWindow winAux(Dims(500, 450), 0, 0, "Channel levels");
  Timer tim; Image< PixRGB<byte> > ima; Image< PixRGB<float> > fima;
  Image< PixRGB<byte> > display;
  uint64 t[NAVG]; unsigned int frame = 0;
  Image<PixH2SV2<float> > H2SVimage;

  /****************************************************************************/
  /* create 2 trackers that are bound together (e.g. 2 trackers in the same
     camera input image
  */
 /*
  //! Mean color to track (ideal color)
  std::vector<float> color(3,0.0F);
  color[0] = 20.0F; color[1] = 0.25F; color[2] = 156.0F;

  //! +/- tollerance value on mean for track
  std::vector<float> std(3,0.0F);
  std[0] = 30.0F; std[1] = 0.30F; std[2] = 60.0F;

  //! normalizer over color values (highest value possible)
  std::vector<float> norm(3,0.0F);
  norm[0] = 360.0F; norm[1] = 1.0F; norm[2] = 255.0F;

  //! how many standard deviations out to adapt, higher means less bias
  std::vector<float> adapt(3,0.0F);
  adapt[0] = 3.5F; adapt[1] = 3.5F; adapt[2] = 3.5F;

  //! highest value for color adaptation possible (hard boundary)
  std::vector<float> upperBound(3,0.0F);
  upperBound[0] = 50.0F; upperBound[1] = 0.4F ; upperBound[2] = 180.0F;

  //! lowest value for color adaptation possible (hard boundary)
  std::vector<float> lowerBound(3,0.0F);
  lowerBound[0] = 12.0F; lowerBound[1] = 0.1F; lowerBound[2] = 120.0F;
*/
  /****************************************************************************/
  //! Mean color to track (ideal color for red feducial)
  /*  std::vector<float> color(3,0.0F);
  color[0] = 10.0F; color[1] = 0.80F; color[2] = 156.0F;

  //! +/- tollerance value on mean for track
  std::vector<float> std(3,0.0F);
  std[0] = 30.0F; std[1] = 0.30F; std[2] = 60.0F;

  //! normalizer over color values (highest value possible)
  std::vector<float> norm(3,0.0F);
  norm[0] = 360.0F; norm[1] = 1.0F; norm[2] = 255.0F;

  //! how many standard deviations out to adapt, higher means less bias
  std::vector<float> adapt(3,0.0F);
  adapt[0] = 3.5F; adapt[1] = 3.5F; adapt[2] = 3.5F;

  //! highest value for color adaptation possible (hard boundry)
  std::vector<float> upperBound(3,0.0F);
  upperBound[0] = 50.0F; upperBound[1] = 1.0F ; upperBound[2] = 255.0F;

  //! lowest value for color adaptation possible (hard boundry)
  std::vector<float> lowerBound(3,0.0F);
  lowerBound[0] = 0.0F; lowerBound[1] = 0.1F; lowerBound[2] = 10.0F;
  */
  /****************************************************************************/
  //! Mean color to track (ideal color for blue feducial)

  /*
  std::vector<float> color(3,0.0F);
  //"PINK"
  //color[0] = 0.0F; color[1] = 0.88F; color[2] = 180.0F;
  //BLUE
  color[0] = 250.0F; color[1] = 0.50F; color[2] = 156.0F;

  //! +/- tollerance value on mean for track
  std::vector<float> std(3,0.0F);
  std[0] = 60.0F; std[1] = 0.30F; std[2] = 60.0F;

  //! normalizer over color values (highest value possible)
  std::vector<float> norm(3,0.0F);
  norm[0] = 360.0F; norm[1] = 1.0F; norm[2] = 255.0F;

  //! how many standard deviations out to adapt, higher means less bias
  std::vector<float> adapt(3,0.0F);
  adapt[0] = 3.5F; adapt[1] = 3.5F; adapt[2] = 3.5F;

  //! highest value for color adaptation possible (hard boundry)
  std::vector<float> upperBound(3,0.0F);
  upperBound[0] = 360.0F; upperBound[1] = 1.0F ; upperBound[2] = 255.0F;

  //! lowest value for color adaptation possible (hard boundry)
  std::vector<float> lowerBound(3,0.0F);
  lowerBound[0] = 200.0F; lowerBound[1] = 0.1F; lowerBound[2] = 10.0F;
  */

  /****************************************************************************/
  //! extracted signature

  // signature extracted for Nathan's mahogany shirt

  // H1 - H2 - S - V
  std::vector<float> color(4,0.0F);
  color[0] = 0.350962; color[1] = 0.645527; color[2] = 0.313523; color[3] = 0.720654;

  //! +/- tollerance value on mean for track
  std::vector<float> std(4,0.0F);
  std[0] = 0.339556; std[1] = 0.368726; std[2] = 0.609608; std[3] = 0.34012;

  //! normalizer over color values (highest value possible)
  std::vector<float> norm(4,0.0F);
  norm[0] = 1.0F; norm[1] = 1.0F; norm[2] = 1.0F; norm[3] = 1.0F;

  //! how many standard deviations out to adapt, higher means less bias
  std::vector<float> adapt(4,0.0F);
  adapt[0] = 3.5F; adapt[1] = 3.5F; adapt[2] = 3.5F; adapt[3] = 3.5F;

  //! highest value for color adaptation possible (hard boundary)
  std::vector<float> upperBound(4,0.0F);
  upperBound[0] = color[0] + 0.45F; upperBound[1] = color[1] + 0.45F;
  upperBound[2] = color[2] + 0.55F; upperBound[3] = color[3] + 0.55F;

  //! lowest value for color adaptation possible (hard boundary)
  std::vector<float> lowerBound(4,0.0F);
  lowerBound[0] = color[0] - 0.45F; lowerBound[1] = color[1] - 0.45F;
  lowerBound[2] = color[2] - 0.55F; lowerBound[3] = color[3] - 0.55F;


  //int zero = 0;
  int wi = width/4;
  int hi = height/4;

  segmentImageTrackMC<float,unsigned int, 4> segmenter(wi*hi);

  segmenter.SITsetTrackColor(&color,&std,&norm,&adapt,&upperBound,&lowerBound);

  /* This limits the area of consideration to an area smaller than
     the image size. That is, it creates a boundery in the image
     outside of which it will not consider pixes (i.e. a frame)
  */
  segmenter.SITsetFrame(&wi,&hi);


  /* Set display colors for output of tracking. Strictly asthetic */
  segmenter.SITsetCircleColor(255,255,0);
  segmenter.SITsetBoxColor(255,255,0,0,255,255);
  segmenter.SITsetUseSmoothing(true,10);
  //unsigned long counter = 0;

  while(1) {

    tim.reset();
    ima = gb->readRGB();
    uint64 t0 = tim.get();  // to measure display time

    Image<PixRGB<byte> > Aux;
    Aux.resize(100,450,true);

    /* Take in the image and color segment it */
    H2SVimage = ima;
    display = ima;

    /******************************************************************/
    // SEGMENT IMAGE ON EACH INPUT FRAME

    segmenter.SITtrackImageAny(H2SVimage,&display,&Aux,true);

    /* Retrieve and Draw all our output images */
    Image<byte> temp = segmenter.SITreturnCandidateImage();

    wini.drawImage(display);
    wino.drawImage(temp);
    winAux.drawImage(Aux);

    /******************************************************************/
    // Uncomment these lines to write each frame to the hard drive
    /******************************************************************/
    /*
    LINFO("COUNT %d",counter);
    // TRACKER DISPLAY
    Raster::WriteRGB(display,sformat("out.display.%d.ppm",counter));
    // BOOL CANDIDATE MAP
    Raster::WriteRGB(temp,sformat("out.temp.%d.ppm",counter));
    // ADAPTIVE THRESHOLDING BARS
    Raster::WriteRGB(Aux,sformat("out.Aux.%d.ppm",counter));
    // BLOB ID MAP
    Image<byte> blobs = segmenter.SITreturnBlobMap();
    inplaceNormalize(blobs, 0,255);
    Raster::WriteRGB(blobs,sformat("out.Blobs.%d.ppm",counter));
    counter++;
    */
    /******************************************************************/

    t[frame % NAVG] = tim.get();
    t0 = t[frame % NAVG] - t0;
    if (t0 > 28) LINFO("Display took %llums", t0);
    // compute and show framerate over the last NAVG frames:
    if (frame % NAVG == 0 && frame > 0)
    {
      uint64 avg = 0; for (int i = 0; i < NAVG; i ++) avg += t[i];
      float avg2 = 1000.0 / (float)avg * NAVG;
      printf("Framerate: %.1f fps\n", avg2);
    }
    frame ++;
  }

  manager.stop();
  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
