/*!@file Gist/test-Gist-Sal-Nav.C navigation using a combination saliency and
  gist. Input is either the camera or an MPEGStream */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-Gist-Sal-Nav.C $
// $Id: test-Gist-Sal-Nav.C 14762 2011-05-03 01:13:16Z siagian $
//
////////////////////////////////////////////////////////
// test-Gist-Sal-Nav.C <input.mpg/CAMERA> <input_train.txt> [output_directory] [index]

// This is an ongoing project for robotics navigation. Currently it is able to
// recognize places through the use of gist features. It accepts an input video
// clip <input.mpg> and a pre-trained neural network via a training file
// <input_train.txt> - the same file is used in the training phase by train-FFN.C.

// At the start, the component manager enables a standard GistEstimator and
// then a neural net place recognizer is instantiated. In the main while
// loop, at each time step the place recognizer hypothesized the location
// based on the gist features.

// Later on we will incorporate saliency to get a better spatial resolution
// as well as accuracy of a location.

// Related files of interest: GistEstimator.C (and .H) and
// GistEstimatorConfigurator.C (and .H) used by Brain.C to compute gist features.
// test-Gist.C uses GistEstimator to extract gist features from a single image.

#include "Channels/ChannelOpts.H"
#include "Component/GlobalOpts.H"
#include "Component/ModelManager.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Devices/FrameGrabberConfigurator.H"
#include "Devices/DeviceOpts.H"
#include "GUI/XWinManaged.H"
#include "Gist/FFN.H"
#include "Gist/trainUtils.H"
#include "Image/ColorOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/ImageCache.H"
#include "Image/MathOps.H"
#include "Image/MatrixOps.H"
#include "Image/Pixels.H"
#include "Image/Pixels.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Media/MPEGStream.H"
#include "Media/MediaOpts.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/GistEstimatorStd.H"
#include "Neuro/GistEstimatorFFT.H"
#include "Neuro/InferoTemporal.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/Retina.H"
#include "Neuro/ShapeEstimator.H"
#include "Neuro/ShapeEstimatorModes.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/StdBrain.H"
#include "Neuro/gistParams.H"
#include "Raster/Raster.H"
#include "SIFT/Histogram.H"
#include "Transport/FrameIstream.H"
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectDB.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Util/Timer.H"

//! number of frames over which frame rate is computed
#define NAVG 20

#define W_ASPECT_RATIO  320 // ideal minimum width for display
#define H_ASPECT_RATIO  240 // ideal minimum height for display

rutz::shared_ptr<FeedForwardNetwork> ffn_place;
Image<double> pcaIcaMatrix;

CloseButtonListener wList;
XWinManaged *inputWin;
XWinManaged *salWin;
XWinManaged *gistWin;

int wDisp, hDisp, sDisp, scaleDisp;
int wDispWin,  hDispWin;

// gist display
int pcaW = 16, pcaH = 5;
int winBarW = 5, winBarH = 25;

// ######################################################################
void                  setupDispWin     (int w, int h);
Image< PixRGB<byte> > getGistDispImg   (Image< PixRGB<byte> > img, Image<float> gistImg,
                                        Image<float> gistPcaImg, Image<float> outHistImg);
void                  processSalCue    (Image<PixRGB<byte> > inputImg,
                                        nub::soft_ref<StdBrain> brain, Point2D<int> winner, int fNum,
const Image<float>& semask, const std::string& selabel);
// ######################################################################
// Main function
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // Instantiate a ModelManager:
  ModelManager manager("Place Localization Model");

  // we cannot use saveResults() on our various ModelComponent objects
  // here, so let's not export the related command-line options.
  manager.allowOptions(OPTEXP_ALL & (~OPTEXP_SAVE));

  // Instantiate our various ModelComponents:
  // either an MPEGStream
  nub::soft_ref<SimEventQueueConfigurator>
    seqc(new SimEventQueueConfigurator(manager));
  manager.addSubComponent(seqc);

  nub::soft_ref<InputMPEGStream>
    ims(new InputMPEGStream(manager, "Input MPEG Stream", "InputMPEGStream"));
  manager.addSubComponent(ims);

  // or a FrameGrabber
  nub::soft_ref<FrameGrabberConfigurator>
    gbc(new FrameGrabberConfigurator(manager));
  manager.addSubComponent(gbc);

  nub::soft_ref<StdBrain> brain(new StdBrain(manager));
  manager.addSubComponent(brain);

  nub::ref<SpatialMetrics> metrics(new SpatialMetrics(manager));
  manager.addSubComponent(metrics);

  manager.exportOptions(MC_RECURSE);
  metrics->setFOAradius(30); // FIXME
  metrics->setFoveaRadius(30); // FIXME
  manager.setOptionValString(&OPT_MaxNormType, "FancyOne");
  manager.setOptionValString(&OPT_UseRandom, "false");
  //  manager.setOptionValString("ShapeEstimatorMode","SaliencyMap");
  //  manager.setOptionValString(&OPT_ShapeEstimatorMode,"ConspicuityMap");
  manager.setOptionValString(&OPT_ShapeEstimatorMode, "FeatureMap");
  manager.setOptionValString(&OPT_ShapeEstimatorSmoothMethod, "Chamfer");
  //manager.setOptionValString(&OPT_ShapeEstimatorSmoothMethod, "Gaussian");
  manager.setOptionValString(&OPT_RawVisualCortexChans,"OIC");
  manager.setOptionValString(&OPT_IORtype, "Disc");

  // set up the GIST ESTIMATOR: Std or Fft: IN COMMAND LINE
  //manager.setOptionValString(&OPT_GistEstimatorType,"Std");

  // set up the INFEROTEMPORAL
  manager.setOptionValString(&OPT_InferoTemporalType,"Std");
  manager.setOptionValString(&OPT_AttentionObjRecog,"yes");
  manager.setOptionValString(&OPT_MatchObjects,"false");
  // Request a bunch of option aliases (shortcuts to lists of options):
  REQUEST_OPTIONALIAS_NEURO(manager);

  // frame grabber setup
  // NOTE: don't have to put the option --fg-type=1394
//   manager.setOptionValString(&OPT_FrameGrabberType, "1394");
//   manager.setOptionValString(&OPT_FrameGrabberDims, "160x120");
//   manager.setOptionValString(&OPT_FrameGrabberMode, "YUV444");
//   manager.setOptionValString(&OPT_FrameGrabberNbuf, "20");

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "<input.mpg/CAMERA> <input_train.txt>"
                               "[output_directory] [index]",
                               2, 4) == false)
    return(1);

  nub::soft_ref<SimEventQueue> seq = seqc->getQ();

  // NOTE: this could now be controlled by a command-line option
  // --preload-mpeg=true
  manager.setOptionValString(&OPT_InputMPEGStreamPreload, "true");

  // do post-command-line configs:
  int w; int h;
  nub::soft_ref<FrameIstream> gb ;
  std::string camera("CAMERA");

  // compare returns zero if they are equal
  if(!manager.getExtraArg(0).compare(camera))
    {
      gb = gbc->getFrameGrabber();
      if (gb.isInvalid())
        LFATAL("You need to select a frame grabber type via the "
               "--fg-type=XX command-line option for this program "
               "to be useful -- ABORT");
      w = gb->getWidth(); h = gb->getHeight();
      std::string dims = convertToString(Dims(w, h));
      manager.setOptionValString(&OPT_InputFrameDims, dims);
      LINFO("Camera");

      // get the frame grabber to start streaming:
      gb->startStream();
    }
  else
    {
      ims->setFileName(manager.getExtraArg(0));

      Dims iDims = ims->peekDims();
      manager.setOptionValString(&OPT_InputFrameDims,
                                 convertToString(ims->peekDims()));
      // Added the sony cropping
      w = iDims.w() - 50 + 1; h = iDims.h();
      LINFO("Mpeg");
    }

  // setup  display  at the start of stream
  // NOTE: wDisp, hDisp, and sDisp are modified
  LINFO("Frame w: %d, h: %d",w, h);
  setupDispWin(w, h);

  // offset number for the saved images (for mpeg_encode)
  int fNumOffset = 0;
  if (manager.numExtraArgs() > 3)
      fNumOffset = manager.getExtraArgAs<int>(3);

  // frame delay in seconds
  double rtdelay = 33.3667/1000.0;    // real time
  double fdelay  = rtdelay*3;           // 3 times slower than real time

  // let's get all our ModelComponent instances started:
  manager.start();

  // get the GistEstimator
  LFATAL("FIXME");
  nub::soft_ref<GistEstimatorStd> ge;//////// =
  ////////    dynCastWeak<GistEstimatorStd>(brain->getGE());

  // main loop:
  SimTime prevstime = SimTime::ZERO(); int fNum = 0;
  Image< PixRGB<byte> > inputImg;
  Image< PixRGB<byte> > gistDispImg;

  // get place classifier parameters
  FFNtrainInfo pcInfo(manager.getExtraArg(1));

  // instantiate a 3-layer feed-forward network
  // initialize with the provided parameters
  ffn_place.reset(new FeedForwardNetwork());
  ffn_place->init3L(pcInfo.h1Name, pcInfo.h2Name, pcInfo.oName,
                    pcInfo.redFeatSize, pcInfo.h1size, pcInfo.h2size,
                    pcInfo.nOutput, 0.0, 0.0);

  // setup the PCA eigenvector
  pcaIcaMatrix = setupPcaIcaMatrix
    (pcInfo.trainFolder+pcInfo.evecFname,
     pcInfo.oriFeatSize, pcInfo.redFeatSize);

  // MAIN LOOP
  Timer tim(1000000); uint64 t[NAVG]; float frate = 0.0f;
  while(true)
  {
    // has the time come for a new frame?
    // LATER ON GIST WILL DECIDE IF WE WANT TO SLOW THINGS DOWN
    if (fNum == 0 ||
        (seq->now() - 0.5 * (prevstime - seq->now())).secs() - fNum * fdelay > fdelay)
      {
        tim.reset();

        // load or grab new frame
        if(!manager.getExtraArg(0).compare(camera))
          {
            inputImg = gb->readRGB();
              //Raster::ReadRGB("/lab/tmpi6/u/christian/beobotData/data_04_06_2006/test_011_000402.ppm");
          }
        else
          {
            inputImg = ims->readRGB();

            // take out frame borders NOTE: ONLY FOR SONY CAMCORDER
            inputImg = crop(inputImg, Rectangle::tlbrI(0, 25, h-1, 25 + w - 1));
          }

        if (inputImg.initialized() == false) break;  // end of input stream

        // pass input to brain:
        rutz::shared_ptr<SimEventInputFrame>
          e(new SimEventInputFrame(brain.get(), GenericFrame(inputImg), 0));
        seq->post(e); // post the image to the brain
        LINFO("new frame :%d\n",fNum);

        // if we don't have a GE then we have to skip Gist extraction
        if (!ge.isInvalid())
        {
          // get the gist feature vector
          // reduce feature dimension (if available)
          Image<double> cgist =  ge->getGist();
          Image<double> in = cgist;
          if(pcInfo.isPCA) in = matrixMult(pcaIcaMatrix, cgist);

          // analyze the gist features to recognize the place
          Image<double> out = ffn_place->run3L(in);
          rutz::shared_ptr<Histogram> resHist(new Histogram(pcInfo.nOutput));

          for(uint i = 0; i < pcInfo.nOutput; i++)
            {
              LINFO("pl[%3d]: %.4f",i, out.getVal(i));
              resHist->addValue(i, out.getVal(i));
            }

          // FIX FOR DISPLAY
          // // display or save the visuals
          // gistDispImg = getGistDispImg(inputImg,
          //                              ge->getGistImage(sDisp),
          //                              getPcaIcaFeatImage(in, pcaW, pcaH,sDisp*2),
          //                              resHist->getHistogramImage(wDisp,sDisp*2*pcaH, 0.0, 1.0));

          // if (manager.numExtraArgs() > 2)
          //   Raster::WriteRGB(gistDispImg, sformat("%s%07d.ppm", manager.getExtraArg(2).c_str(),
          //                                         fNum + fNumOffset));
          // else
          //   {
          //     inputWin->drawImage(inputImg,0,0);
          //     gistWin->drawImage(gistDispImg,0,0);
          //     //Raster::waitForKey();
          //   }
        }
        else
          LINFO("Cannot compute gist without a Gist Estimator");

        // compute and show framerate over the last NAVG frames:
        t[fNum % NAVG] = tim.get();
        if (fNum % 5 == 0)
          {
            uint64 avg = 0ULL; for (int i = 0; i < NAVG; i ++) avg += t[i];
            frate = 1000000.0F / float(avg) * float(NAVG);
            printf("[%6d] Frame rate: %f fps -> %f ms/frame \n",fNum,frate, 1000.0/frate);
          }

       // increment frame count
        fNum++;
      }

    // evolve brain:
    prevstime = seq->now(); // time before current step
    const SimStatus status = seq->evolve();

    // FIX THIS LATER
    // process if SALIENT location is found
    if (SeC<SimEventWTAwinner> e = seq->check<SimEventWTAwinner>(0))
      {
        // localize using the salient cue
        //const Point2D<int> winner = brain->getLastCovertPos();
        // use Shape estimator to focus on the attended region
        Image<float> fmask; std::string label;
        if (SeC<SimEventShapeEstimatorOutput>
            e = seq->check<SimEventShapeEstimatorOutput>(0))
          { fmask = e->smoothMask(); label = e->winningLabel(); }
        //processSalCue(inputImg, brain, winner, fNum, fmask, label);
      }

    if (SIM_BREAK == status) // Brain decided it's time to quit
      break;
  }

  //uint64 t = tim.get();
  //printf("It takes %.3fms to process %d frame = %.3f ms/frame\n",
  //       float(t)* 0.001F, fNum, float(t)/float(fNum)* 0.001F);

  // stop all our ModelComponents
  manager.stop();

  // all done!
  return 0;
}

// ######################################################################
// process salient cues
void processSalCue(Image<PixRGB<byte> > inputImg,
                   nub::soft_ref<StdBrain> brain, Point2D<int> winner, int fNum,
                   const Image<float>& semask, const std::string& selabel)
{
  // use Shape estimator to focus on the attended region
  Image<float> roiImg;Image<PixRGB<byte> > objImg;
  if (semask.initialized())
    {
      float mn, mx; getMinMax(semask,mn,mx);
      Rectangle r = findBoundingRect(semask, mx*.05f);
      objImg = crop(inputImg, r);
      roiImg = semask * luminance(inputImg);
    }
  else
    {
      objImg = inputImg;
      roiImg = luminance(inputImg);
    }

  // we need a Visual Cortex to obtain a feature vector
  LFATAL("fixme");
  nub::soft_ref<VisualCortex> vc;///////// = brain->getVC();
  ///////  std::vector<float> fvec; vc->getFeatures(winner, fvec);

  //   SIFT key-point
  // create a new VisualObject. Since we give it no keypoints, they
  // will be automatically computed:
  //rutz::shared_ptr<VisualObject>
  //  obj(new VisualObject("NewObject", "NewObject", roiImg, fvec));

  // ----------------------------------------------
  // match the salient Region

  // WITH DATA BASE AND DESCRIPTION

  // ----------------------------------------------

  // draw the results
  drawCircle(roiImg, winner, 10, 0.0f, 1);
  drawPoint(roiImg, winner.i, winner.j, 0.0f);
  LINFO("\nFrame: %d, winner: (%d,%d) in %s\n\n",
        fNum, winner.i, winner.j, selabel.c_str());
  salWin->drawImage(roiImg,0,0);
  salWin->drawImage(objImg,inputImg.getWidth(),0);
  Raster::waitForKey();

}

// ######################################################################
// setup display window for visualization purposes
void setupDispWin(int w, int h)
{

  inputWin = new XWinManaged(Dims(w, h), 2*w, 0, "Original Input Image" );
  wList.add(inputWin);

  // figure out the best display w, h, and scale for gist

  // check if both dimensions of the image
  // are much smaller than the desired resolution
  scaleDisp = 1;
  while (w*scaleDisp < W_ASPECT_RATIO*.75 && h*scaleDisp < H_ASPECT_RATIO*.75)
    scaleDisp++;

  // check if the height is longer aspect-ratio-wise
  // this is because the whole display is setup wrt/ to it
  wDisp = w*scaleDisp; hDisp = h*scaleDisp;
  if(wDisp/(0.0 + W_ASPECT_RATIO) > hDisp/(0.0 + H_ASPECT_RATIO))
    hDisp = (int)(wDisp / (0.0 + W_ASPECT_RATIO) * H_ASPECT_RATIO)+1;
  else
    wDisp = (int)(hDisp / (0.0 + H_ASPECT_RATIO) * W_ASPECT_RATIO)+1;

  // add slack so that the gist feature entry is square
  sDisp = (hDisp/NUM_GIST_FEAT + 1);
  hDisp =  sDisp * NUM_GIST_FEAT;

  // add space for all the visuals
  wDispWin = wDisp + sDisp * NUM_GIST_COL;
  hDispWin = hDisp + sDisp * pcaH * 2;

  gistWin  = new XWinManaged(Dims(wDispWin, hDispWin), 0, 0, "Gist Related");
  wList.add(gistWin);

  salWin   = new XWinManaged(Dims(2*w, h), 0, 2*h, "Saliency Related" );
  wList.add(salWin);
}

// ######################################################################
// get display image for visualization purposes
Image< PixRGB<byte> > getGistDispImg (Image< PixRGB<byte> > img,
                                      Image<float> gistImg,
                                      Image<float> gistPcaImg,
                                      Image<float> outHistImg)
{
  Image< PixRGB<byte> > gistDispImg(wDispWin, hDispWin, ZEROS);
  int w = img.getWidth(); int h = img.getHeight();

  // grid the displayed input image
  Image< PixRGB<byte> > tImg = img;
  drawGrid(tImg, w/4,h/4,1,1,PixRGB<byte>(255,255,255));
  inplacePaste(gistDispImg, tImg,        Point2D<int>(0, 0));

  // display the gist features
  inplaceNormalize(gistImg, 0.0f, 255.0f);
  inplacePaste(gistDispImg, Image<PixRGB<byte> >(gistImg),    Point2D<int>(wDisp, 0));

  // display the PCA gist features
  inplaceNormalize(gistPcaImg, 0.0f, 255.0f);
  inplacePaste(gistDispImg, Image<PixRGB<byte> >(gistPcaImg), Point2D<int>(wDisp, hDisp));

  // display the classifier output histogram
  inplaceNormalize(outHistImg, 0.0f, 255.0f);
  inplacePaste(gistDispImg, Image<PixRGB<byte> >(outHistImg), Point2D<int>(0, hDisp));

  // draw lines delineating the information
  drawLine(gistDispImg, Point2D<int>(0,hDisp),
           Point2D<int>(wDispWin,hDisp),
           PixRGB<byte>(255,255,255),1);
  drawLine(gistDispImg, Point2D<int>(wDisp-1,0),
           Point2D<int>(wDisp-1,hDispWin-1),
           PixRGB<byte>(255,255,255),1);
  return gistDispImg;
}

// ######################################################################
// canonical gray is (128, 128, 128)
Image< PixRGB<byte> > greyWorldNormalize(Image< PixRGB<byte> > img)
{
  Image<byte> rImg;
  Image<byte> gImg;
  Image<byte> bImg;
//   getComponents(img, rImg, gImg, bImg);

  //int rMin, rMax, gMin, gMax, bMin, gMax;
  double rMean = mean(rImg);
  double gMean = mean(gImg);
  double bMean = mean(bImg);
  printf("mean = [%f,%f,%f]\n",rMean, gMean, bMean);

  Image<float> rtImg = (rImg * (128.0/rMean)) + .5;
  Image<float> gtImg = (gImg * (128.0/gMean)) + .5;
  Image<float> btImg = (bImg * (128.0/bMean)) + .5;
  inplaceClamp(rtImg, 0.0f,255.0f);
  inplaceClamp(gtImg, 0.0f,255.0f);
  inplaceClamp(btImg, 0.0f,255.0f);

  Image< PixRGB <byte> > res = makeRGB(rtImg, gtImg, btImg);
  return res;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
