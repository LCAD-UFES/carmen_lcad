/*!@file VFAT/fzvision2.C  simplified version of vision.C with feature analysis
 */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/fzvision2.C $
// $Id: fzvision2.C 14376 2011-01-11 02:44:34Z pez $
//
// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itti itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################

//#include "saliency.H"
#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "GUI/XWindow.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/StdBrain.H"
#include "Neuro/NeuroSimEvents.H"
#include "VFAT/featureClusterVision.H"
#include "VFAT/segmentImageTrackMC2.H"
#include "Simulation/SimEventQueueConfigurator.H"

// define standard deviation multiplier for simple tracker
#define H1_STD 2.0F
#define H2_STD 2.0F
#define S_STD  2.0F
#define V_STD  2.0F

// DV Size

#define IMAGE_SIZE_X    640
#define IMAGE_SIZE_Y    480

#define GAMMA1    0
#define GAMMA2    1
#define GAMMA3    2

//#define SPATX     8
//#define SPATY     9

#define MOTION    3
//#define MOTION    0

#define HUE1      6
//#define HUE1      3
#define HUE2      7
//#define HUE2      4
#define SAT       4
//#define  SAT      1
#define VALUE     5
//#define VALUE     2

// Small NTSC Size

//#define IMAGE_SIZE_X 320
//#define IMAGE_SIZE_Y 240

// maximum value currently is 6
#define TRACKER_NUMBER  3
#define USE_LOGS        true
#define SAVE_TRACK_DRAW true




//! Basic entry and test binary to featureClusterVision

int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  bool use_logs        = USE_LOGS;
  bool save_track_draw = SAVE_TRACK_DRAW;

  // Instantiate a ModelManager:
  ModelManager manager("Attention Model");

  // Instantiate our various ModelComponents:
  nub::soft_ref<SimEventQueueConfigurator>
    seqc(new SimEventQueueConfigurator(manager));
  manager.addSubComponent(seqc);

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  nub::soft_ref<StdBrain> brain(new StdBrain(manager));
  manager.addSubComponent(brain);

  // feature analysis part of model
  const std::string name = "featureCluster";
  const std::string tag  = "fCV";
  Image< PixRGB<byte> > input;
  Image< PixRGB<float> > finput;
  std::vector<covHolder<double> > covHolder;

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "<image>", 1, 1) == false)
    return(1);

  nub::soft_ref<featureClusterVision<float> >
    fCV(new featureClusterVision<float>(manager,name,tag,brain,ifs,
                                        manager.getExtraArg(0)));
  manager.addSubComponent(fCV);
  nub::soft_ref<SimEventQueue> seq = seqc->getQ();

  // let's get all our ModelComponent instances started:
  manager.start();
  // main loop:
  //************************************************************
  // set up output x windows

  Image<PixRGB<float> > fima1; Image<PixRGB<byte> > bima1;
  XWindow win2(Dims(IMAGE_SIZE_X, IMAGE_SIZE_Y), 0, 0, "CLASSES TEMPORAL");
  Image<PixRGB<float> > fima2; Image<PixRGB<byte> > bima2;
  XWindow win3(Dims(IMAGE_SIZE_X, IMAGE_SIZE_Y), 0, 0, "TARGETS TEMPORAL");
  Image<PixRGB<float> > fima3; Image<PixRGB<byte> > bima3;
  Image<float> fima4;  Image<byte> bima4;


  XWindow wini(Dims(IMAGE_SIZE_X, IMAGE_SIZE_Y),
               0, 0, "Test-input window");
  XWindow wino(Dims(IMAGE_SIZE_X/SIT_GLOBAL_DEC,
                    (IMAGE_SIZE_Y/SIT_GLOBAL_DEC)*TRACKER_NUMBER),
               0, 0, "Test-output window");
  XWindow winAux(Dims(500, 450),
                 0, 0, "Channel levels");
  unsigned short wi = IMAGE_SIZE_X;
  unsigned short hi = IMAGE_SIZE_Y;
  //! Holds H2SV representation of image for simple tracker
  Image<PixH2SV2<float> > H2SVimage;
  //! output display image
  Image< PixRGB<byte> > display;

  //************************************************************
  // Initalize translation values and other const values from
  // covHolder to segHolder

  // how many standard deviations out to adapt, higher means less bias
  std::vector<float> adapt(4,0.0F);
  adapt[0] = 3.5F; adapt[1] = 3.5F; adapt[2] = 3.5F; adapt[3] = 3.5F;
  // set mkodifer over standard deviation.
  // A larger number means less bias
  std::vector<float> STDmod(4,0.0F);
  STDmod[0] = H1_STD; STDmod[1] = H2_STD;
  STDmod[2] = S_STD;  STDmod[3] = V_STD;
  // input feature names to be helpful
  std::vector<std::string> fname(4,"");
  fname[0] = "Hue1"; fname[1] = "Hue2";
  fname[2] = "Sat";  fname[3] = "Val";
  // Hard upperBound modifier
  std::vector<float> lowerBoundMod(4,0.0F);
  lowerBoundMod[0] = 0.45F; lowerBoundMod[1] = 0.45F;
  lowerBoundMod[2] = 0.45F; lowerBoundMod[3] = 0.55F;
  // Hard lower bound modifer
  std::vector<float> upperBoundMod(4,0.0F);
  upperBoundMod[0] = 0.45F; upperBoundMod[1] = 0.45F;
  upperBoundMod[2] = 0.45F; upperBoundMod[3] = 0.55F;
  // maps the channels in covHolder to the channels in segHolder
  // since segHolder may have fewer channels this is needed
  std::vector<unsigned short> channelMap(4,0);
  channelMap[0] = HUE1;  channelMap[1] = HUE2;
  channelMap[2] = SAT;   channelMap[3] = VALUE;
  // This holds color infomation for output on screen
  unsigned char oc[6][6] = {
    {128,0,0     ,255,0,0} ,
    {0,128,0     ,0,255,0} ,
    {0,0,128     ,0,0,255}   ,
    {128,128,0   ,255,255,0} ,
    {0,128,128   ,0,255,255} ,
    {128,0,128   ,255,0,255}
  };

  //************************************************************
  // Set up trackers and tracker signature holders
  segmentImageTrackMC<float,unsigned int,4> _segmenter(wi*hi);
  segHolder<float,unsigned int,4> _segHolder;

  std::vector<segmentImageTrackMC<float,unsigned int,4> >
    segmenters(TRACKER_NUMBER,_segmenter);
  std::vector<segHolder<float,unsigned int,4> >
    segHolder(TRACKER_NUMBER,_segHolder);

  // Set up each tracker
  // here we set up inital values for each tracker and
  // each channel

  for(int i = 0; i < TRACKER_NUMBER; i++)
  {
    segmenters[i].SITuseLog(true);
    segmenters[i].SITsetFrame(&wi,&hi);
    segmenters[i].SITsetCircleColor(0,255,0);
    segmenters[i].SITsetUseSmoothing(true,10);
    segmenters[i].SITsetBoxColor(oc[i][0], oc[i][1], oc[i][2],
                                 oc[i][3], oc[i][4], oc[i][5]);
    segHolder[i].imageSizeX       = wi;
    segHolder[i].imageSizeY       = hi;
    segHolder[i].baseID           = i;

    for(int j = 0; j < 4; j++)
    {
      segHolder[i].channelAdapt[j]  = adapt[j];
      segHolder[i].STDmod[j]        = STDmod[j];
      segHolder[i].upperBoundMod[j] = upperBoundMod[j];
      segHolder[i].lowerBoundMod[j] = lowerBoundMod[j];
      segHolder[i].channelMap[j]    = channelMap[j];
      segHolder[i].featureName[j]   = fname[j];
    }
  }

  //************************************************************
  //std::vector<bool>         noTargetYet(TRACKER_NUMBER,true);
  //std::vector<unsigned int> contTrackTime(TRACKER_NUMBER,0);

  //************************************************************
  // Set up log files
  if(use_logs)
  {
    std::ofstream outfileStart1("tracker.fzvision.log",std::ios::out);
    outfileStart1 << "\n*************************************************\n\n";
    outfileStart1.close();
    std::ofstream outfileStart2("blobs.fzvision.log",std::ios::out);
    outfileStart2 << "\n*************************************************\n\n";
    outfileStart2.close();
  }

  //************************************************************
  while(1) {
    // write outputs or quit?
    bool shouldsave = false;
    bool shouldquit = false;
    bool gotcovert = false;
    if (seq->check<SimEventWTAwinner>(0)) gotcovert = true;
    const FrameState os = ofs->update(seq->now(), gotcovert);

    if (os == FRAME_NEXT || os == FRAME_FINAL)
      shouldsave = true;

    if (os == FRAME_FINAL)
      shouldquit = true;

    if (shouldsave)
    {
      SimModuleSaveInfo sinfo(ofs, *seq);
      brain->save(sinfo);
      int foo = ifs->frame();
      std::string Myname;
      std::string a = manager.getExtraArg(0);
      std::string b = ".";
      char c[100];
      if(foo == 1)
        ; //init = false;
      if(foo < 10)
        sprintf(c,"00000%d",foo);
      else if(foo < 100)
        sprintf(c,"0000%d",foo);
      else if(foo < 1000)
        sprintf(c,"000%d",foo);
      else if(foo < 10000)
        sprintf(c,"00%d",foo);
      else if(foo < 100000)
        sprintf(c,"0%d",foo);
      else
        sprintf(c,"%d",foo);
      Myname = a + b + c;
      std::cout << "******************************************************\n";
      std::cout << "******************************************************\n";
      // NOTE: added '0' at the end here because there was no value
      // matching the final '%d'
      std::cout << "RUNNING FRAME " << foo    << "\n";
      std::cout << "NAME          " << Myname << "\n";
      // Upload a frame to the classifier
      fCV->fCVuploadImage(input,Myname);
      // classify and cluster this image
      fCV->fCVclusterImage();
      std::cout << "******************************************************\n";
      std::cout << "******************************************************\n";
      // get back image data
      fCV->fCVgetClusterImages(&fima1,&fima2,&fima3,&fima4);
      bima1 = fima1; bima2 = fima2; bima3 = fima3; bima4 = fima4*255.0F;
      bima4 = rescale(bima4,bima1.getWidth(),bima1.getHeight());
      //Raster::WriteRGB(bima1,sformat("classes.out.%s.ppm",c));
      //Raster::WriteRGB(bima2,sformat("temporal.out.%s.ppm",c));
      Raster::WriteRGB(bima3,sformat("target.out.%s.ppm",c));
      //Raster::WriteGray(bima4,sformat("salmap.out.%s.pgm",c));
      //win1.drawImage(bima1);
      win2.drawImage(bima2);
      win3.drawImage(bima3);
      // optional SIGNATURE DATA FROM THIS CLASS (see covHolder.H)
      covHolder     = fCV->fCVgetCovHolders();
      // optional Size of the covHolder Vector

      // See covHolder.H to see how data is stored
      // See covEstimate.C matchPmeanAccum to see how these
      // numbers are computed.
      std::ofstream outfile("tracker.fzvision.log",std::ios::app);
      if(use_logs)
      {
        outfile << ">>> FRAME " << foo;
      }
      // Update each tracker as to the current frame number
      for(int i = 0; i < TRACKER_NUMBER; i++)
      {
        segmenters[i].SITsetFrameNumber(foo);
      }

      //************************************************************
      /* For each potental target from the complex tracker, check
         to see if it is good to track. If so, assign it to a simple
         tracker, but check that the tracker is free and/or not
         already tracking this exact same thing.
      */

      // for each interesting object in the complex tracker
      for(int x = 0; x < (signed)fCV->fCVgetCovHolderSize(); x++)
      {
        // for each of the simple trackers
        for(int i = 0; i < TRACKER_NUMBER; i++)
        {
          // if the simple tracker is not tracking anything yet
          if(segHolder[i].noTargetYet == true)
          {
            // if an object in the complex tracker is moving
            if(covHolder[x].mean[MOTION] > 2.5F)
            {
              if(use_logs)
                outfile << "\t- (" << x
                        << ") MASS OK " << covHolder[x].mean[MOTION];
              bool unique = true;
              // For all the other simple trackers
              for(int j = 0; j < TRACKER_NUMBER; j++)
              {
                // if the other tracker is tracking a target
                if(segHolder[j].noTargetYet == false)
                {
                  //if(segHolder[j].LOT == false)
                  //{

                  // check to see that the target we want to track
                  // is not already being tracked
                    unique = segmenters[j].
                      SITintersection(&covHolder[x],30,30);
                    if(use_logs)
                      outfile << "\n\t- CHECKED " << i
                              << " WITH " << j << " : ";
                    //}
                }
              }
              // if the target is new and not being tracked
              if(unique == true)
              {
                if(use_logs)
                  outfile << "\t- Samples " << covHolder[x].samples << " ... ";
                // make sure it is large enough to track
                if(covHolder[x].samples > 25)
                {
                  if(use_logs)
                  {
                    outfile << "OK ";
                    outfile << " Variance " << covHolder[x].STD[HUE1]  * H1_STD
                            << " "          << covHolder[x].STD[HUE2]  * H2_STD
                            << " ... ";
                    outfile << "OK\n";

                    outfile << "<<<NEW TRACKING>>> " << x            << "\t"
                            << "FRAME " << foo                       << "\t"
                            << "TRACKER " << i                       << "\t"
                            << "LIFE SPAN " << covHolder[x].lifeSpan << "\t"
                            << "("     << covHolder[x].posX
                            << ","     << covHolder[x].posY        << ")\t"
                            << "Max (" << covHolder[x].maxX
                            << ","     << covHolder[x].maxY        << ")\t"
                            << "Min (" << covHolder[x].minX
                            << ","     << covHolder[x].minY        << ")\n";
                  }
                  // assign target to a simple tracker
                  segHolder[i].noTargetYet = false;
                  // What colors should the target have?
                  segmenters[i].SITsetTrackSignature(&covHolder[x],
                                                     &segHolder[i]);
                }
              }
            }
          }
        }
      }
      if(use_logs)
      {
        outfile << "\n";
        outfile.close();
      }
      display = input;
      Image<PixRGB<byte> > Aux;
      Aux.resize(100,450,true);
      Image<byte> tempPaste;
      tempPaste.resize(IMAGE_SIZE_X/SIT_GLOBAL_DEC,
                       (IMAGE_SIZE_Y/SIT_GLOBAL_DEC)*TRACKER_NUMBER,true);

      //************************************************************
      /* For all simple trackers which are tracking a target,
         check to make sure that it is tracking what it should
         be tracking.
      */

      // for each simple tracker
      for(int i = 0; i < TRACKER_NUMBER; i++)
      {
        Image<byte> temp;
        temp.resize(IMAGE_SIZE_X/SIT_GLOBAL_DEC,
                    IMAGE_SIZE_Y/SIT_GLOBAL_DEC,true);
        // if the simple tracker is tracking
        if(segHolder[i].noTargetYet == false)
        {
          H2SVimage = input;
          // track the current image
          segmenters[i].SITtrackImage(H2SVimage,&display);
          // Retrieve and Draw all our output images
          temp = segmenters[i].SITreturnCandidateImage();
          // Get back the current signature of the target being tracked
          segmenters[i].SITgetTrackSignature(&segHolder[i]);

          if(use_logs)
          {
            char whynotcomeupwithacoolname[100];
            sprintf(whynotcomeupwithacoolname,"blobs.%d.fzvision.log",i);

            std::ofstream outfile2(whynotcomeupwithacoolname,std::ios::app);
            outfile2 << "FRAME "      << foo
                     << "\t"
                     << "TOTAL TRACK TIME " << segHolder[i].totalLifeSpan
                     << "\t"
                     << "TRACK TIME " << segHolder[i].lifeSpan
                     << "\t"
                     << "BLOBS "      << segHolder[i].blobNumber
                     << "\t"
                     << "LOT status " << segHolder[i].LOTtypeName
                     << "\t"
                     << "LOT count "  << segHolder[i].LOTcount
                     << "\t"
                     << "MASS "       << segHolder[i].mass
                     << "\n";
            for(unsigned int ii = 0; ii < segHolder[i].blobNumber; ii++)
            {
              outfile2 << "\tBLOB NUMBER " << ii                    << "\t"
                       << segmenters[i].SITgetBlobReasonForKill(ii)
                       << " ... \t"
                       << segmenters[i].SITgetBlobMass(ii)          << "\t"
                       << segmenters[i].SITgetBlobPosX(ii)          << "\t"
                       << segmenters[i].SITgetBlobPosY(ii)          << "\n";
            }
            outfile2 << "\n";
            outfile2.close();
          }
          if(use_logs)
          {
            std::ofstream outfile("tracker.fzvision.log",std::ios::app);
            outfile << "OVERLAP - \n";
          }
          // for all the other simple trackers
          for(int j = 0; j < TRACKER_NUMBER; j++)
          {
            if(i != j)
            {
              // if the other simple tracker is tracking
              if(segHolder[j].noTargetYet == false)
              {
                // if the other simple tracker is not in Loss Of Track
                if(segHolder[j].LOT == false)
                {
                  float overlap, thisarea, otherarea;
                  // check if our targets are overlaping
                  segmenters[j].SITpercentOverlap(&segHolder[i],&overlap,
                                                    &thisarea,&otherarea);
                  if(use_logs)
                  {
                    std::ofstream outfile("tracker.fzvision.log",std::ios::app);
                    outfile << i <<  " - " << j
                            << " : i / j = " << overlap
                            << "\n";
                    outfile.close();
                  }
                  // if there is alot of overlap, we are most
                  // likely tracking the exact same thing
                  if(overlap > 0.75F)
                  {
                    // kill the younger of the overlappers
                    // if same age, kill the smaller
                    if(segHolder[j].lifeSpan == segHolder[i].lifeSpan)
                    {
                      if(thisarea > otherarea)
                        segHolder[i].noTargetYet = true;
                      else
                        segHolder[j].noTargetYet = true;
                    }
                    else if(segHolder[j].lifeSpan > segHolder[i].lifeSpan)
                      segHolder[i].noTargetYet = true;
                    else
                      segHolder[j].noTargetYet = true;
                    if(use_logs)
                    {
                      std::ofstream outfile("tracker.fzvision.log",std::ios::app);
                      outfile << i << " OVERLAPS WITH " << j << "\n";
                      outfile.close();
                    }
                  }
                }
              }
            }
          }
          // for this tracker, if to many Loss of Tracks have
          // been encountered in a row, the tracker gives up and
          // resets.
          if(segHolder[i].LOTandReset == true)
          {
            segHolder[i].noTargetYet   = true;
            std::cout << "LOSS OF TRACK RESET" << "\n";
            if(use_logs)
            {
              std::ofstream outfile("tracker.fzvision.log",std::ios::app);
              outfile << ">>>LOSS OF TRACK " << i
                      << " RESET AT FRAME "  << foo << "\n";
              outfile.close();
            }
          }
        }
        pasteImage(tempPaste,temp,
                   (byte)128,Point2D<int>(0,(IMAGE_SIZE_Y/SIT_GLOBAL_DEC)*i));
      }

      wino.drawImage(tempPaste);
      winAux.drawImage(Aux);
      wini.drawImage(display);
      if(save_track_draw)
      {
        Raster::WriteGray(tempPaste,
                          sformat("tracker.candidates.out.%s.pgm",c));
        Raster::WriteRGB(Aux,sformat("tracker.levels.out.%s.ppm",c));
        Raster::WriteRGB(display,sformat("tracker.out.%s.ppm",c));
      }
    }

    if (shouldquit) break;              // done

    // why do we handle the output before the input? That's because
    // both the input and output frame series will switch to the next
    // frame at the exact same time, if input and output framerates
    // are equal. When the input series switches to a new frame, it
    // will reset any drawings that were obtained on the previous
    // frame. So we need to make sure we have saved those results
    // before we read the new frame in.

    // if we displayed a bunch of images, let's pause:
    if (ifs->shouldWait() || ofs->shouldWait())
      Raster::waitForKey();

    // read new image in?
    const FrameState is = ifs->update(seq->now());
    if (is == FRAME_COMPLETE) break; // done
    if (is == FRAME_NEXT || is == FRAME_FINAL) // new frame
    {
      input = ifs->readRGB();

      // empty image signifies end-of-stream
      if (input.initialized())
        {
           rutz::shared_ptr<SimEventInputFrame>
            e(new SimEventInputFrame(brain.get(), GenericFrame(input), 0));
          seq->post(e); // post the image to the brain

          // show memory usage if in debug mode:
          if (MYLOGVERB >= LOG_DEBUG)
            SHOWMEMORY("MEMORY USAGE: frame %d t=%.1fms", ifs->frame(),
                       seq->now().msecs());
        }
    }

    // evolve brain:
    const SimStatus status = seq->evolve();

    if (SIM_BREAK == status) // Brain decided it's time to quit
      break;
  }


  //fCV->checkMixing();
  //fCV->checkICA();
  //std::string fileName = "features.txt";
  fCV->fCVfeaturesToFile(manager.getExtraArg(0),false);
  fCV->fCVICAfeaturesToFile(manager.getExtraArg(0));
  // stop all our ModelComponents

  manager.stop();

  // all done!
  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
