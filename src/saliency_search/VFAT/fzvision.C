/*!@file VFAT/fzvision.C  simplified version of vision.C with feature analysis
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/fzvision.C $
// $Id: fzvision.C 14376 2011-01-11 02:44:34Z pez $
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
#include "VFAT/segmentImageTrackMC.H"
#include "Simulation/SimEventQueueConfigurator.H"

// define standard deviation multiplier for simple tracker
#define H1_STD 3.5F
#define H2_STD 3.5F
#define S_STD  3.0F
#define V_STD  2.5F

// DV Size

#define IMAGE_SIZE_X 640
#define IMAGE_SIZE_Y 480
#define TRACKER_NUMBER 3


// Small NTSC Size

//#define IMAGE_SIZE_X 320
//#define IMAGE_SIZE_Y 240

//! Basic entry and test binary to featureClusterVision

int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

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
  const std::string tag = "fCV";
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
  bool init = true;
  // main loop:

  //XWindow win1(Dims(IMAGE_SIZE_X, IMAGE_SIZE_Y), 0, 0, "CLASSES");
  Image<PixRGB<float> > fima1; Image<PixRGB<byte> > bima1;
  XWindow win2(Dims(IMAGE_SIZE_X, IMAGE_SIZE_Y), 0, 0, "CLASSES TEMPORAL");
  Image<PixRGB<float> > fima2; Image<PixRGB<byte> > bima2;
  XWindow win3(Dims(IMAGE_SIZE_X, IMAGE_SIZE_Y), 0, 0, "TARGETS TEMPORAL");
  Image<PixRGB<float> > fima3; Image<PixRGB<byte> > bima3;
  Image<float> fima4;

  //************************************************************
  // SIMPLE TRACKER STUFF

  XWindow wini(Dims(IMAGE_SIZE_X, IMAGE_SIZE_Y),     0, 0, "Test-input window");
  XWindow wino(Dims(IMAGE_SIZE_X/4, (IMAGE_SIZE_Y/4)*TRACKER_NUMBER), 0, 0, "Test-output window");
  XWindow winAux(Dims(500, 450),   0, 0, "Channel levels");
  int wi = IMAGE_SIZE_X/4;
  int hi = IMAGE_SIZE_Y/4;
  //! Holds H2SV representation of image for simple tracker
  Image<PixH2SV2<float> > H2SVimage;
  //! output display image
  Image< PixRGB<byte> > display;
  //! Holds color to track for simple tracker
  std::vector<float> color(4,0.0F);
  //! +/- tollerance value on mean for track
  std::vector<float> std(4,0.0F);
  //! normalizer over color values (highest value possible)
  std::vector<float> norm(4,0.0F);
  norm[0] = 1.0F; norm[1] = 1.0F; norm[2] = 1.0F; norm[3] = 1.0F;
  //! how many standard deviations out to adapt, higher means less bias
  std::vector<float> adapt(4,0.0F);
  adapt[0] = 3.5F; adapt[1] = 3.5F; adapt[2] = 3.5F; adapt[3] = 3.5F;
  //! highest value for color adaptation possible (hard boundry)
  std::vector<float> upperBound(4,0.0F);
  //! lowest value for color adaptation possible (hard boundry)
  std::vector<float> lowerBound(4,0.0F);

  segmentImageTrackMC<float,unsigned int,4> _segmenter(wi*hi);

  std::vector<segmentImageTrackMC<float,unsigned int,4> >
    segmenters(TRACKER_NUMBER,_segmenter);

  for(int i = 0; i < TRACKER_NUMBER; i++)
  {
    segmenters[i].SITsetFrame(&wi,&hi);
    segmenters[i].SITsetCircleColor(0,255,0);
    segmenters[i].SITsetUseSmoothing(true,10);
  }

  segmenters[0].SITsetBoxColor(128,0,0     ,255,0,0);
  segmenters[1].SITsetBoxColor(0,128,0     ,0,255,0);
  segmenters[2].SITsetBoxColor(0,0,128     ,0,0,255);
  segmenters[3].SITsetBoxColor(128,128,0   ,255,255,0);
  segmenters[4].SITsetBoxColor(0,128,128   ,0,255,255);
  segmenters[5].SITsetBoxColor(128,0,128   ,255,0,255);

  std::vector<bool> noTargetYet(TRACKER_NUMBER,true);
  std::vector<unsigned int> contTrackTime(TRACKER_NUMBER,0);

  //************************************************************
  std::ofstream outfileStart1("tracker.fzvision.log",std::ios::out);
  outfileStart1 << "\n*************************************************\n\n";
  outfileStart1.close();
  std::ofstream outfileStart2("blobs.fzvision.log",std::ios::out);
  outfileStart2 << "\n*************************************************\n\n";
  outfileStart2.close();
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
        init = false;
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
      std::cerr << "******************************************************\n";
      std::cerr << "******************************************************\n";
      // NOTE: added '0' at the end here because there was no value
      // matching the final '%d'
      LINFO("RUNNING FRAME %d NTARG %d",foo,0);
      LINFO("NAME %s",Myname.c_str());
      // Upload a frame to the classifier
      //Raster::VisuRGB(input,"input.ppm");
      fCV->fCVuploadImage(input,Myname);
      // classify and cluster this image
      fCV->fCVclusterImage();
      std::cerr << "******************************************************\n";
      std::cerr << "******************************************************\n";
      // get back image data
      fCV->fCVgetClusterImages(&fima1,&fima2,&fima3,&fima4);
      //Raster::VisuRGB(finput,"finput.ppm");
      bima1 = fima1; bima2 = fima2; bima3 = fima3;
      //win1.drawImage(bima1);
      win2.drawImage(bima2);
      win3.drawImage(bima3);

      //Raster::WriteRGB(bima1,sformat("features.out.%s.ppm",c));
      Raster::WriteRGB(bima2,sformat("classes.out.%s.ppm",c));
      //Raster::WriteRGB(bima3,sformat("targets.out.%s.ppm",c));
      // optional output of bayesian classification
      //fCV->fCVprintOutBayesClass();
      // optional output of nearest neighbor classification
      //fCV->fCVprintOutNeighborClass();

      //std::string outFile = manager.getExtraArg(0) + c;

      //fCV->fCVfeaturesToFile(outFile,true);
      // optional dump out all features post ICA to file
      //fCV->fCVICAfeaturesToFile(outFile);
      // optional check all the combined Motions
      //fCV->fCVcheckMotionCombined(foo);

      //std::string file = "covMatrix";
      // optional A file contaning signature data
      //fCV->fCVdumpCovMatrix(file);
      //fCV->fCVprintOutClusters();

      // optional SIGNATURE DATA FROM THIS CLASS (see covHolder.H)
      covHolder = fCV->fCVgetCovHolders();
      // optional Size of the covHolder Vector

      // See covHolder.H to see how data is stored
      // See covEstimate.C matchPmeanAccum to see how these
      // numbers are computed.
      std::ofstream outfile("tracker.fzvision.log",std::ios::app);
      outfile << ">>> FRAME " << foo;
      for(int i = 0; i < TRACKER_NUMBER; i++)
      {
        segmenters[i].SITsetFrameNumber(foo);
      }

      for(int x = 0; x < (signed)fCV->fCVgetCovHolderSize(); x++)
      {
        covHolder[x].dumpMeToFile(a,c,init);
        for(int i = 0; i < TRACKER_NUMBER; i++)
        {
          if(noTargetYet[i] == true)
          {
            LINFO("%d,%f",x,covHolder[x].mean[3]);
            if(covHolder[x].mean[3] > 1.5F)
            {
              bool unique = true;
              for(int j = 0; j < TRACKER_NUMBER; j++)
              {
                if(noTargetYet[j] == false)
                {
                  unsigned int umaxX,umaxY,uminX,uminY,uposX,uposY;
                  int maxX,maxY,minX,minY;
                  bool isSet = false;
                  outfile << "\t\n- CHECKED " << i << " WITH " << j << " : ";
                  if(contTrackTime[j] > 1)
                  {
                    segmenters[j].SITgetMinMaxBoundry(&uminX,&umaxX,&uminY,&umaxY);
                    uposX = 0; uposY = 0;
                    outfile << "REAL BOUNDARY ";
                    uminX = uminX/4; uminY = uminY/4;
                    umaxX = umaxX/4; umaxY = umaxY/4;
                  }
                  else
                  {
                    segmenters[j].SITgetExpectedTargetPosition(&uposX,&uposY,&umaxX,&umaxY,&uminX,&uminY,&isSet);
                    // NOTE: I changed these from posX,posY to
                    // uposX,uposY, because posX,posY haven't been
                    // initialized yet at this point and were
                    // generating compiler warnings:
                    outfile << "EXPECTED BOUNDARY (" << uposX << "," << uposY << ") : ";
                  }

                  const int fudge = 15; // get ride of this
                  minX = (signed)uminX; minY = (signed)uminY;
                  maxX = (signed)umaxX; maxY = (signed)umaxY;
                  bool Xok = false;
                  bool Yok = false;
                  if((((signed)covHolder[x].minX/4 < minX-fudge) && ((signed)covHolder[x].maxX/4 < minX-fudge)) ||
                     (((signed)covHolder[x].minX/4 > maxX+fudge) && ((signed)covHolder[x].maxX/4 > maxX+fudge)))
                  {
                    Xok = true;
                  }
                  if((((signed)covHolder[x].minY/4 < minY-fudge) && ((signed)covHolder[x].maxY/4 < minY-fudge)) ||
                     (((signed)covHolder[x].minY/4 > maxY+fudge) && ((signed)covHolder[x].maxY/4 > maxY+fudge)))
                  {
                    Yok = true;
                  }
                  if((Xok == false) && (Yok == false))
                  {
                    unique = false;
                  }

                  outfile << covHolder[x].minX/4 << " < " << minX-fudge << " && "
                          << covHolder[x].maxX/4 << " < " << minX-fudge << " || "
                          << covHolder[x].minX/4 << " > " << maxX+fudge << " && "
                          << covHolder[x].maxX/4 << " > " << maxX+fudge << " \n\tAND\n\t"
                          << covHolder[x].minY/4 << " < " << minY-fudge << " && "
                          << covHolder[x].maxY/4 << " < " << minY-fudge << " || "
                          << covHolder[x].minY/4 << " > " << maxY+fudge << " && "
                          << covHolder[x].maxY/4 << " > " << maxY+fudge << " : isSet = " << isSet
                          << " : Unique = " << unique << "\n";
                }
              }
              if(unique == true)
              {
                PixRGB<float>   pRGB;
                PixH2SV2<float> pH2SV2;
                pH2SV2.p[0] = covHolder[x].mean[6]/covHolder[x].bias[6];
                pH2SV2.p[1] = covHolder[x].mean[7]/covHolder[x].bias[7];
                pH2SV2.p[2] = covHolder[x].mean[4]/covHolder[x].bias[4];
                pH2SV2.p[3] = covHolder[x].mean[5]/covHolder[x].bias[5];
                pRGB = PixRGB<float>(pH2SV2);
                outfile << "\n\t- "            << x
                        << " Moving Target " << covHolder[x].mean[3] << "\t"
                        << covHolder[x].featureName[3].c_str() << "\t"
                        << covHolder[x].mean[3]                << "\t"
                        << covHolder[x].STD[3]                 << "\t"
                        << covHolder[x].bias[3]                << "\t"
                        << covHolder[x].featureName[4].c_str() << "\t"
                        << covHolder[x].mean[4]                << "\t"
                        << covHolder[x].STD[4]                 << "\t"
                        << covHolder[x].bias[4]                << "\t"
                        << covHolder[x].featureName[5].c_str() << "\t"
                        << covHolder[x].mean[5]                << "\t"
                        << covHolder[x].STD[5]                 << "\t"
                        << covHolder[x].bias[5]                << "\t"
                        << covHolder[x].featureName[6].c_str() << "\t"
                        << covHolder[x].mean[6]                << "\t"
                        << covHolder[x].STD[6]                 << "\t"
                        << covHolder[x].bias[6]                << "\t"
                        << covHolder[x].featureName[7].c_str() << "\t"
                        << covHolder[x].mean[7]                << "\t"
                        << covHolder[x].STD[7]                 << "\t"
                        << covHolder[x].bias[6]                << "\n\t- "
                        << "RED "   << pRGB.p[0]               << "\t"
                        << "GREEN " << pRGB.p[1]               << "\t"
                        << "BLUE "  << pRGB.p[2]               << "\n";
                LINFO("%s",covHolder[x].featureName[3].c_str());
                LINFO("%s",covHolder[x].featureName[6].c_str());
                LINFO("%s",covHolder[x].featureName[7].c_str());
                LINFO("%s",covHolder[x].featureName[4].c_str());
                LINFO("%s",covHolder[x].featureName[5].c_str());
                LINFO("..%lx",covHolder[x].samples);
                outfile << "\t- Samples " << covHolder[x].samples << " ... ";
                if(covHolder[x].samples > 25)
                {
                  outfile << "OK ";
                  outfile << " Variance " << covHolder[x].STD[6]  * H1_STD
                          << " "          << covHolder[x].STD[7]  * H2_STD
                          << " ... ";
                  /* if((((covHolder[x].STD[6]  * H1_STD) < 0.15F) ||
                     ((covHolder[x].STD[7]  * H2_STD) < 0.15F) ||
                     ((covHolder[x].STD[4]  * S_STD) < 0.15F)) &&
                     ((covHolder[x].STD[5]  * V_STD) > 0.15F))
                     {*/

                  outfile << "OK\n";
                  //if(covHolder[x].lifeSpan > 5)
                  //{

                  outfile << "<<<NEW TRACKING>>> " << x            << "\t"
                          << "FRAME " << foo                       << "\t"
                          << "TRACKER " << i                       << "\t"
                          << "LIFE SPAN " << covHolder[x].lifeSpan << "\t"
                          << "("     << covHolder[x].posX/4
                          << ","     << covHolder[x].posY/4        << ")\t"
                          << "Max (" << covHolder[x].maxX/4
                          << ","     << covHolder[x].maxY/4        << ")\t"
                          << "Min (" << covHolder[x].minX/4
                          << ","     << covHolder[x].minY/4        << ")\n";

                  LINFO("....%lx",covHolder[x].lifeSpan);

                  noTargetYet[i]   = false;

                  color[0]      = covHolder[x].mean[6]/covHolder[x].bias[6];
                  color[1]      = covHolder[x].mean[7]/covHolder[x].bias[7];
                  color[2]      = covHolder[x].mean[4]/covHolder[x].bias[4];
                  color[3]      = covHolder[x].mean[5]/covHolder[x].bias[5];

                  std[0]        = (covHolder[x].STD[6]*(1/covHolder[x].bias[6]))
                    * H1_STD;
                  std[1]        = (covHolder[x].STD[7]*(1/covHolder[x].bias[7]))
                    * H2_STD;
                  std[2]        = (covHolder[x].STD[4]*(1/covHolder[x].bias[4]))
                    * S_STD;
                  std[3]        = (covHolder[x].STD[5]*(1/covHolder[x].bias[5]))
                    * V_STD;

                  upperBound[0] = covHolder[x].mean[6]/covHolder[x].bias[6]
                    + 0.45F;
                  upperBound[1] = covHolder[x].mean[7]/covHolder[x].bias[7]
                    + 0.45F;
                  upperBound[2] = covHolder[x].mean[4]/covHolder[x].bias[4]
                    + 0.45F;
                  upperBound[3] = covHolder[x].mean[5]/covHolder[x].bias[5]
                    + 0.45F;

                  lowerBound[0] = covHolder[x].mean[6]/covHolder[x].bias[6]
                    - 0.45F;
                  lowerBound[1] = covHolder[x].mean[7]/covHolder[x].bias[7]
                    - 0.45F;
                  lowerBound[2] = covHolder[x].mean[4]/covHolder[x].bias[4]
                    - 0.45F;
                  lowerBound[3] = covHolder[x].mean[5]/covHolder[x].bias[5]
                    - 0.45F;

                  // reset the frame to the center
                  segmenters[i].SITsetFrame(&wi,&hi);
                  // where should the target appear?
                  segmenters[i].SITsetExpectedTargetPosition(covHolder[x].posX/4,
                                                             covHolder[x].posY/4,
                                                             covHolder[x].maxX/4,
                                                             covHolder[x].maxY/4,
                                                             covHolder[x].minX/4,
                                                             covHolder[x].minY/4);
                  // What colors should the target have?

                  segmenters[i].SITsetTrackColor(&color,&std,&norm,&adapt,
                                                 &upperBound,&lowerBound,false,true);
                  //}
                }
              }
            }
          }
        }
      }
      outfile << "\n";
      outfile.close();
      display = input;
      Image<PixRGB<byte> > Aux;
      Aux.resize(100,450,true);
      Image<byte> tempPaste;
      tempPaste.resize(IMAGE_SIZE_X/4, (IMAGE_SIZE_Y/4)*TRACKER_NUMBER,true);
      for(int i = 0; i < TRACKER_NUMBER; i++)
      {
        Image<byte> temp;
        temp.resize(IMAGE_SIZE_X/4, IMAGE_SIZE_Y/4,true);
        if(noTargetYet[i] == false)
        {
          H2SVimage = input;
          segmenters[i].SITtrackImageAny(H2SVimage,&display,&Aux,true);
          /* Retrieve and Draw all our output images */
          temp = segmenters[i].SITreturnCandidateImage();


          char foofoo[100];
          sprintf(foofoo,"blobs.%d.fzvision.log",i);

          std::ofstream outfile2(foofoo,std::ios::app);
          outfile2 << "FRAME "      << foo                                  << "\t"
                   << "TRACK TIME " << contTrackTime[i]                     << "\t"
                   << "BLOBS "      << segmenters[i].SITnumberBlobs()       << "\t"
                   << "LOT status " << segmenters[i].SITreturnLOTtypeName() << "\t"
                   << "LOT count "  << segmenters[i].SITgetLOTcount()       << "\t"
                   << "MASS "       << segmenters[i].SITgetMass()           << "\n";
          for(unsigned int ii = 0; ii < segmenters[i].SITnumberBlobs(); ii++)
          {
            outfile2 << "\tBLOB NUMBER " << ii                << "\t"
                     << segmenters[i].SITgetBlobReasonForKill(ii) << " ... \t"
                     << segmenters[i].SITgetBlobMass(ii)          << "\t"
                     << segmenters[i].SITgetBlobPosX(ii)          << "\t"
                     << segmenters[i].SITgetBlobPosY(ii)          << "\n";
          }
          outfile2 << "\n";
          outfile2.close();

          if(segmenters[i].SIT_LOTandRESET == true)
          {
            noTargetYet[i]   = true;
            contTrackTime[i] = 0;
            LINFO("LOSS OF TRACK RESET");
            std::ofstream outfile("tracker.fzvision.log",std::ios::app);
            outfile << ">>>LOSS OF TRACK " << i
                    << " RESET AT FRAME "  << foo << "\n";
            outfile.close();
          }
          else
          {
            contTrackTime[i]++;
          }
        }
        pasteImage(tempPaste,temp,(byte)128,Point2D<int>(0,(IMAGE_SIZE_Y/4)*i));
      }
      Raster::WriteGray(tempPaste,sformat("tracker.candidates.out.%s.pgm",c));
      wino.drawImage(tempPaste);
      Raster::WriteRGB(Aux,sformat("tracker.levels.out.%s.ppm",c));
      winAux.drawImage(Aux);
      Raster::WriteRGB(display,sformat("tracker.out.%s.ppm",c));
      wini.drawImage(display);
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
