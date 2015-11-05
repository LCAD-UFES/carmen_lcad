/*!@file Gist/extractGist.C Extract gist features
  from MPEGStream or gistList file                                      */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/extractGist.C $
// $Id: extractGist.C 14376 2011-01-11 02:44:34Z pez $
//

#include "Channels/ChannelOpts.H"
#include "Component/GlobalOpts.H"
#include "Component/ModelManager.H"
#include "Component/ModelOptionDef.H"
#include "Image/CutPaste.H"
#include "Image/Dims.H"
#include "Image/DrawOps.H"
#include "Image/Pixels.H"
#include "Media/MPEGStream.H"
#include "Media/MediaOpts.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/GistEstimatorStd.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/StdBrain.H"
#include "Neuro/gistParams.H"
#include "Raster/Raster.H"
#include "Simulation/SimEventQueueConfigurator.H"

#include "GUI/XWinManaged.H"

#define SQ_SIZE  20 //the size of a square in the histogram

CloseButtonListener wList;
XWinManaged *inputWin;
XWinManaged *gistWin;

// ######################################################################
void getGistFileList(std::string fName,
                     std::vector<std::string>& tag,
                     std::vector<int>& start,
                     std::vector<int>& num);

void saveData(Image<double> data, std::string fName, int num);
// ######################################################################

// Main function
int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // Instantiate a ModelManager:
  ModelManager manager("Gist Features Extraction");

  // we cannot use saveResults() on our various ModelComponent objects
  // here, so let's not export the related command-line options.
  manager.allowOptions(OPTEXP_ALL & (~OPTEXP_SAVE));

  // Instantiate our various ModelComponents:
  nub::soft_ref<SimEventQueueConfigurator>
    seqc(new SimEventQueueConfigurator(manager));
  manager.addSubComponent(seqc);

  nub::soft_ref<InputMPEGStream>
    ims(new InputMPEGStream(manager, "Input MPEG Stream", "InputMPEGStream"));
  manager.addSubComponent(ims);

  nub::soft_ref<StdBrain> brain(new StdBrain(manager));
  manager.addSubComponent(brain);

  nub::ref<SpatialMetrics> metrics(new SpatialMetrics(manager));
  manager.addSubComponent(metrics);

  manager.exportOptions(MC_RECURSE);
  metrics->setFOAradius(30); // FIXME
  metrics->setFoveaRadius(30); // FIXME

  // setting up the GIST ESTIMATOR
  manager.setOptionValString(&OPT_GistEstimatorType,"Std");
  //manager.setOptionValString(&OPT_GistEstimatorType,"FFT");

  // Request a bunch of option aliases (shortcuts to lists of options):
  REQUEST_OPTIONALIAS_NEURO(manager);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "<*.mpg or *_gistList.txt>",
                               1, 1) == false)
    return(1);

  nub::soft_ref<SimEventQueue> seq = seqc->getQ();

  // if the file passed ends with _gistList.txt
  // we have a different protocol
  bool isGistListInput = false;
  int ifLen = manager.getExtraArg(0).length();
  if(ifLen > 13 &&
     manager.getExtraArg(0).find("_gistList.txt",ifLen - 13) !=
     std::string::npos)
    isGistListInput = true;

  // NOTE: this could now be controlled by a command-line option
  // --preload-mpeg=true
  manager.setOptionValString(&OPT_InputMPEGStreamPreload, "true");

  // do post-command-line configs:
  std::vector<std::string> tag;
  std::vector<int> start;
  std::vector<int> num;
  unsigned int cLine = 0; int cIndex = 0;
  if(isGistListInput)
    {
      LINFO("we have a gistList input");
      getGistFileList(manager.getExtraArg(0).c_str(), tag, start, num);
      cIndex = start[0];
    }
  else
    {
      LINFO("we have an mpeg input");
      ims->setFileName(manager.getExtraArg(0));
      manager.setOptionValString(&OPT_InputFrameDims,
                                 convertToString(ims->peekDims()));
    }

  // frame delay in seconds
  //double fdelay = 33.3667/1000.0; // real time
  double fdelay = 3.3667/1000.0;

  // let's get all our ModelComponent instances started:
  manager.start();

  // get the GistEstimator
  nub::soft_ref<GistEstimatorStd> ge;////// =
  ///////    dynCastWeak<GistEstimatorStd>(brain->getGE());
  LFATAL("fixme");
  if (ge.isInvalid()) LFATAL("I am useless without a GistEstimator");

  // MAIN LOOP
  SimTime prevstime = SimTime::ZERO();
  int fNum = 0;
  Image< PixRGB<byte> > inputImg;  Image< PixRGB<byte> > dispImg;
  Image<double> cgist;
  std::string folder =  "";
  std::string::size_type sPos = manager.getExtraArg(0).rfind("/",ifLen);
  if(sPos != std::string::npos)
    folder = manager.getExtraArg(0).substr(0,sPos+1);

  LINFO("let's start");
  while(1)
  {
    // has the time come for a new frame?
    if (fNum == 0 ||
        (seq->now() - 0.5 * (prevstime - seq->now())).secs() - fNum * fdelay > fdelay)
    {
      // load new frame
      std::string fName;
      if(isGistListInput)
        {
          if (cLine >= tag.size()) break;  // end of input list

          // open the current file
          char tNumStr[100]; sprintf(tNumStr,"%06d",cIndex);
          fName = folder + tag[cLine] + std::string(tNumStr) + ".ppm";

          inputImg = Raster::ReadRGB(fName);
          cIndex++;

          if(cIndex >= start[cLine] + num[cLine])
            {
              cLine++;
              if (cLine < tag.size()) cIndex = start[cLine];
            }

          // reformat the file name to a gist name
          int fNameLen = fName.length();
          unsigned int uPos = fName.rfind("_",fNameLen);
          fName = fName.substr(0,uPos)+ ".ppm";

        }
      else
        {
          fName = manager.getExtraArg(0);
          inputImg = ims->readRGB(); //Raster::ReadRGB(manager.getExtraArg(1));
          if (inputImg.initialized() == false) break;  // end of input stream
          // format new frame
          inputImg = crop(inputImg,
                          Rectangle(Point2D<int>(0,25),
                                    Dims(inputImg.getHeight(),
                                         inputImg.getWidth()-25+1)));
          cIndex = fNum+1;
        }

      dispImg = inputImg;
      LINFO("\nnew frame :%d",fNum);

      // pass input to brain:
      rutz::shared_ptr<SimEventInputFrame>
        e(new SimEventInputFrame(brain.get(), GenericFrame(inputImg), 0));
      seq->post(e); // post the image to the brain

      // get the gist feature vector
      cgist = ge->getGist();
      //for(uint k = 0; k < cgist.getSize(); k++) LINFO("%d: %f",k, cgist.getVal(0,k));

//       // setup display at the start of stream
//       if (fNum == 0)
//       {
//         int s = SQ_SIZE;
//         inputWin = new XWinManaged(Dims(w, h), 0, 0, manager.getExtraArg(0).c_str());
//         wList.add(inputWin);
//         gistWin = new XWinManaged(Dims(NUM_GIST_COL * s, NUM_GIST_FEAT * s), 0,0, "Gist");
//         wList.add(gistWin);
//       }

//       // display the input image and the gist histogram
//       drawGrid(dispImg, w/4,h/4,1,1,PixRGB<byte>(255,255,255));
//       inputWin->drawImage(dispImg,0,0);
//       gistWin->drawImage(ge->getGistHistogram(SQ_SIZE),0,0);

      // SAVE GIST FEATURES TO A FILE
      saveData(cgist, fName, cIndex-1);
      //LINFO("\nFrame number just saved:%d",fNum);Raster::waitForKey();

      // increase frame count
      fNum++;
    }

    // evolve brain:
    prevstime = seq->now(); // time before current step
    const SimStatus status = seq->evolve();
    if (SIM_BREAK == status) // Brain decided it's time to quit
      break;

  }

  // stop all our ModelComponents
  manager.stop();

  // all done!
  return 0;
}

// ######################################################################
void getGistFileList(std::string fName,  std::vector<std::string>& tag,
                     std::vector<int>& start, std::vector<int>& num)
{
  char comment[200]; FILE *fp;  char inLine[100];

  // open the file
  if((fp = fopen(fName.c_str(),"rb")) == NULL)
    LFATAL("samples file: %s not found",fName.c_str());
  LINFO("fName: %s",fName.c_str());

  // get number of samples
  int nSamples; if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets failed"); sscanf(inLine, "%d %s", &nSamples, comment);

  // the number of categories
  int tNcat; if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets failed"); sscanf(inLine, "%d %s", &tNcat, comment);

  // get the type of ground truth
  char gtOpt[100]; if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets failed"); sscanf(inLine, "%s %s", gtOpt, comment);

  // skip column headers
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets failed");

  char cName[100]; char ext[100];  int cStart, cNum; int gTruth;
  while(fgets(inLine, 1000, fp) != NULL)
  {
    // get the files in this category and ground truth
    sscanf(inLine, "%s %d %d %d %s", cName, &cStart, &cNum,  &gTruth, ext);
    LINFO("    sName: %s %d %d %d %s",cName, cStart, cNum, gTruth, ext);

    tag.push_back(cName);
    start.push_back(cStart);
    num.push_back(cNum);
  }
  fclose(fp);
}

// ######################################################################
// save the gist feature vector to a file
void saveData(Image<double> data, std::string fName, int num)
{
  // change the extension to .gist
  char temp[100]; char gName[100];
  int tp = strcspn(fName.c_str(),".");
  strncpy(temp, fName.c_str(), tp); temp[tp] = '\0';
  //sprintf(gName,"%s_FFT_%03d%s", temp,num,".gist");
  sprintf(gName,"%s_%03d%s", temp,num,".gist");
  LINFO("gist file name: %s", gName);

  // write the data to the gist file
  Image<double>::iterator aptr = data.beginw();
  FILE *gfp; if((gfp = fopen(gName,"wb")) != NULL)
    {
      for(int i = 0; i < data.getSize(); i++)
        { double val = *aptr++; fwrite(&val, sizeof(double), 1, gfp); }
      fclose(gfp);
    }
  else LFATAL("can't write: %s", gName);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

