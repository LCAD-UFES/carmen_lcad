/*!@file Gist/faceDetection.C saliency based face detection */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/faceDetection.C $
// $Id: faceDetection.C 10982 2009-03-05 05:11:22Z itti $
//

// ######################################################################
/*! face detection program                                              */

#include "Channels/ChannelOpts.H"
#include "Component/GlobalOpts.H"
#include "Component/ModelManager.H"
#include "GUI/XWinManaged.H"
#include "Gist/FFN.H"
#include "Image/FFTWWrapper.H"
#include "Gist/ModelFace.H"
#include "Image/CutPaste.H"    // for inplacePaste()
#include "Image/DrawOps.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Kernels.H"     // for gaborFilter()
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Image/PyramidOps.H"
#include "Image/ShapeOps.H"    // for zoomXY() etc.
#include "Image/Transforms.H"
#include "Media/FrameSeries.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/ShapeEstimator.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/StdBrain.H"
#include "Raster/Raster.H"
#include "Simulation/SimEventQueueConfigurator.H"


#include "Channels/BlueYellowChannel.H"
#include "Channels/ColorChannel.H"
#include "Channels/GaborChannel.H"
#include "Channels/IntensityChannel.H"
#include "Channels/OrientationChannel.H"
#include "Channels/RedGreenChannel.H"


CloseButtonListener wList;
XWinManaged *imgWin;
XWinManaged *fftWin;
XWinManaged *salWin;
XWinManaged *workWin;

ImageSet<float> pyrImg;

// for Gist
FFTWWrapper *fftw;
rutz::shared_ptr<FeedForwardNetwork> ffn_pl;
int    primeLev      = 0;
double *input        = NULL;
Image<double> gft;
double *iptr         = NULL;
double **outFftw     = NULL;
double ****gaborMask = NULL;
Image<float> gaborMaskImg[NUM_G_LEV][NUM_G_DIR];
void setupPrimeLevEstimator();
void setupFFTW(Image<float> img);
int* getPrimeLev(Image<float> img);
void setupGaborMask(Image<float> img);
Image<float> normalize(Image<float> img);
Image<float> getFftImage(double **outFft, int w, int h);
void fftCompute(Image<float> img, double **outFft);
void getFeatureVector
(double **outFft, Image<double> &res, int w, int h);
void getFeatureVector2
(nub::soft_ref<StdBrain> brain, Image<double> &vec, int w, int h);
void freeFftwData(Image<float> img);

// for part detections
rutz::shared_ptr<FeedForwardNetwork> ffn_e;
rutz::shared_ptr<FeedForwardNetwork> ffn_n;
rutz::shared_ptr<FeedForwardNetwork> ffn_m;

// CURRENT IMAGE BEING PROCESS - MINIMIZING PROCESSING
// IN REAL APPLICATION A BRAIN WOULD DO THIS
// Change the option from ORISteerable
Image<float> currImg;
ImageSet<float> currGaPyr[NUM_DIR];

void         setupPartDetectors();
Rectangle    getWindow         (Image<float> img, Point2D<int> p, int s);
void         getFacePartProb   (Point2D<int> p, float ang, double *pPart);
void         correctGabors     (Image<float> **Img, float ang);
void         getFacePartFeatures(Rectangle r, FacePart part, Image<double> &features);
//void         getFeature        (Image<float> **img, FacePart part,
//                                Image<double> &features);
void         getEyeFeature     (Rectangle r, Image<double> &features);
void         getEyeFeature     (Image<float> **img, Image<double> &features);
void         getNoseFeature    (Rectangle r, Image<double> &features);
void         getNoseFeature    (Image<float> **img, Image<double> &features);
void         getMouthFeature   (Rectangle r, Image<double> &features);
void         getMouthFeature   (Image<float> **img, Image<double> &features);

// face detection main path
void         detectFace        (Image<float> img, nub::soft_ref<StdBrain> brain,
                                Point2D<int> winner);
Point2D<int>      getIntSalPt       (Image<float> img, Point2D<int> p);
Point2D<int>      getIntSalPt       (Image<float> img, Image<float>& vis, Point2D<int> p);
float        getSalG           (Point2D<int> p, int lev);
void         getLocalSalPt     (Point2D<int> pWinner, int max,
                                Point2D<int> **pt, double **ang, int *np);
void         crop              (Image<float>& img, Point2D<int> p);
void         getContextPt      (Point2D<int> *attPt, float *dir, double **pProb,
                                int n, int max,
                                Point2D<int> **pt, double **ang, int *np);
double       getAng            (Point2D<int> p);
float        getPotVal         (Point2D<int> *attPt, float *dir,
                                double *pProb[NUM_FACE_PART], int n,
                                Point2D<int> pt, float a, double
                                tProb[NUM_FACE_PART]);
float        getGeomRelProb    (FacePart o,FacePart c, Point2D<int> oPt, float oD,
                                Point2D<int> cPt, float cA);
bool         locateFace        (Point2D<int> *attPt, float *dir,
                                double *pProb[NUM_FACE_PART], int n);
void         drawFace          (Image<float>& img);
void         printRegion       (Image<float> img,int sX,int eX,int dX,
                                int sY,int eY, int dY);
void         displayPartImage  (Image<float> img, Rectangle pr);

// location of the face
Point2D<int> eyeP, eye2P, noseP, mouthP;
float   eyeD, eye2D, noseD, mouthD;

// ######################################################################
// START detecting faces :-)
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // Instantiate a ModelManager:
  ModelManager manager("Face Detection Model");

  nub::soft_ref<SimEventQueueConfigurator>
    seqc(new SimEventQueueConfigurator(manager));
  manager.addSubComponent(seqc);

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  // Instantiate our various ModelComponents:
  nub::soft_ref<StdBrain> brain(new StdBrain(manager));
  manager.addSubComponent(brain);

  manager.setOptionValString(&OPT_MaxNormType, "FancyOne");
  manager.setOptionValString(&OPT_UseRandom, "false");
  //  manager.setOptionValString(&OPT_ShapeEstimatorMode,"SaliencyMap");
  //  manager.setOptionValString(&OPT_ShapeEstimatorMode,"ConspicuityMap");
  manager.setOptionValString(&OPT_ShapeEstimatorMode, "FeatureMap");
  manager.setOptionValString(&OPT_ShapeEstimatorSmoothMethod, "Chamfer");
  //manager.setOptionValString(&OPT_ShapeEstimatorSmoothMethod, "Gaussian");
  manager.setOptionValString(&OPT_RawVisualCortexChans,"OIC");
  //manager.setOptionValString(&OPT_IORtype, "ShapeEstFM");
  manager.setOptionValString(&OPT_IORtype, "Disc");

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "<image>", 1, 1) == false)
    return(1);

  LFATAL("FIXME");
  /*
  nub::soft_ref<SimEventQueue> seq = seqc->getQ();

  Image<float> img//; img.resize(240,240);
  = Raster::ReadGray(manager.getExtraArg(0));
  const Dims dims = img.getDims();
  int w = img.getWidth(); int h = img.getHeight();
  //img = crop(img,Rectangle::tlbrI(0,0,h-1,w/2-1));
  //w = img.getWidth(); h = img.getHeight();

  printf("\nImage size: %d by %d\n",w,h);

//   for(int i = 0; i < 240; i++)
//     for(int j = 0; j < 240; j++)
//     {
//       //float k = sqrt((i - 120.0)*(i - 120.0) + (j-120.0)*(j-120.0));
//       img.setVal(i,j, 255.0*sin(j*2*M_PI/30.0)*sin(i*2*M_PI/30.0));
//     }
//    Point2D<int>  a(100,120);
//    int rx = 0, ry = 0; int mL = 0;
//    if(w/40 < h/50) mL = w/40; else mL = h/50;
//    printf("ML: %d\n", mL);
//    for(int i = 1; i < mL; i*=2)
//  drawRect(img, Rectangle::tlbrI(ry, rx, ry+i*50-1, rx+i*40-1), 255, 1);

//   Point2D<int>  b(140,120);
//   Point2D<int>  c(120, 80);
//   Point2D<int>  d(120,160);
//   Point2D<int> ac( 80, 80);
//   Point2D<int> ad( 80,160);
//   Point2D<int> bc(160, 80);
//   Point2D<int> bd(160,160);
//   //drawLine(img, a,b,255,10);
//   //drawDisk(img, a,48,255);
//   drawDisk(img, a,4,255);
//   drawPatch(img, a, 16, 200); //pt,rad,intens
//   drawPatch(img, b, 2, 200); //pt,rad,intens
//   drawPatch(img, c, 2, 200); //pt,rad,intens
//   drawPatch(img, d, 2, 200); //pt,rad,intens
//   drawPatch(img, ac, 2, 200); //pt,rad,intens
//   drawPatch(img, ad, 2, 200); //pt,rad,intens
//   drawPatch(img, bc, 2, 200); //pt,rad,intens
//   drawPatch(img, bd, 2, 200); //pt,rad,intens

  imgWin  = new XWinManaged(dims,    0,    0, manager.getExtraArg(0).c_str());
  wList.add(imgWin);
  fftWin  = new XWinManaged(dims, w+15,    0, "FFT"                         );
  wList.add(fftWin);
  salWin  = new XWinManaged(dims,    0, h+25, "Sal"                         );
  wList.add(salWin);
  workWin = new XWinManaged(dims, w+15, h+25, "Scan Path"                   );
  wList.add(workWin);

  // let's get all our ModelComponent instances started:
  LINFO("\nSTART\n");
  manager.start();

  // take out the motion and flicker channel
  double wIntens = 1.0, wOrient = 1.0, wColor = 1.0;

  nub::soft_ref<VisualCortex>        vc;////////// = brain->getVC();
  nub::soft_ref<ShapeEstimator>      se;////////// = brain->getSE();
  vc->setSubchanTotalWeight("color", wColor);
  vc->setSubchanTotalWeight("intensity", wIntens);
  vc->setSubchanTotalWeight("orientation", wOrient);
  //vc->setSubchanTotalWeight("flicker", 0.0);
  //vc->setSubchanTotalWeight("motion", 0.0);

  // setup the networks to detect each face parts
  setupPartDetectors();
  setupPrimeLevEstimator();

  // setup the Fourier Transform feature vector
  gft.resize(1, NUM_G_LEV*NUM_G_DIR, NO_INIT);

  // main loop:
  LINFO("\nMAIN_LOOP\n");

  // are we training or testing?
  bool isTesting = true;

  // training protocol to obtain samples
  // actual training done on train-FFN
  if (!isTesting)
    {
      // get scale samples
      //    NEED FOR LOOP
      std::string sName("sample fileName");
      const Image<float> sImg = Raster::ReadGray(sName);
      const int w = sImg.getWidth();
      const int h = sImg.getHeight();
      const Image<float> nsImg = normalize(sImg);
      //iWin->drawImage(sImg,0,0); //Raster::waitForKey();

      // extract the gist features
      bool useFFT = true;
      Image<double> inputVal(1,1,NO_INIT);
      if(useFFT)
        {
          // resize the fftw properly
          setupFFTW(nsImg);

          //compute Fourier Transform of the image
          fftCompute(nsImg, outFftw);

          // compute the feature vector
          getFeatureVector(outFftw, inputVal, w, h);

          //sXW[i%4] = new XWinManaged(Dims(w,h),0,0,"ft");
          //iXW[i%4] = new XWinManaged(Dims(w,h),0,0,inLine);
          Image<float> tImg = getFftImage(outFftw,w,h);
          //float min, max; getMinMax(tImg, min,max);
          //LINFO("min: %f, max: %f", min, max);
          //sXW[i%4]->drawImage(tImg,0,0);
          //iXW[i%4]->drawImage(nsImg,0,0);
          //Raster::waitForKey();

          // free the data for FFTW computation
          freeFftwData(nsImg);
        }
      else
        {
          rutz::shared_ptr<SimEventInputFrame>
            e(new SimEventInputFrame(brain.get(), GenericFrame(Image<byte>(nsImg)), 0));
          seq->post(e); // post the image to the brain
          (void) seq->evolve();

          // compute the feature vector
          getFeatureVector2(brain, inputVal, w, h);
        }

      // save it to a file
//       Image<double>::iterator aptr = inputVal.beginw();
//       Image<double>::iterator stop = inputVal.endw();
//       if((ifp = fopen(iName,"wb")) != NULL)
//         {
//           while(aptr != stop)
//             {
//               double val = *aptr++; fwrite(&val,sizeof(double), 1, ifp);
//             }
//           fclose(ifp);
//         }

      // get face parts samples
      //NOTE: all face part training done by Gist/train-faceParts.C
    }

  int* probLev = NULL;
  while(isTesting)
  {
    // read new image in?
    const FrameState is = ifs->update(seq->now());
    if (is == FRAME_COMPLETE) break; // done
    if (is == FRAME_NEXT || is == FRAME_FINAL) // new frame
    {
      Image< PixRGB<byte> > input = ifs->readRGB();

      // empty image signifies end-of-stream
      if (!input.initialized())
        break;

      rutz::shared_ptr<SimEventInputFrame>
        e(new SimEventInputFrame(brain.get(), GenericFrame(Image<byte>(img)), 0));
      seq->post(e); // post the image to the brain

      // prepare gabor filter
      currImg = normalize(img);
      pyrImg  = buildPyrGaussian(currImg, 0, 9, 3);

      for(int i = 0; i< NUM_DIR; i++)
        currGaPyr[i] = buildPyrGabor(currImg,0,NUM_H_LEV+3,i*45,7,1,9,0);

      // for every new image (new dimensions)
      // the fourier transform arrays has to be resized as well
      setupFFTW(img);

      // estimate the level to work with
      // with respect to the size of our model head
      probLev = getPrimeLev(currImg);
      for(int i = 0; i < NUM_H_LEV; i++)
        printf("PL* %d: %d  \n",i,probLev[i]);
    }

    // evolve brain:
    const SimStatus status = seq->evolve();

    // call face detection method if salient location is selected
    if (SeC<SimEventWTAwinner> e = seq->check<SimEventWTAwinner>(0)) {
      const Point2D<int> winner = e->winner().p;

      // display the input image
      Image<float> dispImg = img;
      inplaceNormalize(dispImg, 0.0f, 255.0f);
      imgWin->drawImage(dispImg,0,0);
      //Raster::waitForKey();

      // setup the necessary objects from the brain
      LFATAL("fixme");
      nub::soft_ref<ShapeEstimator> se;///////// = brain->getSE();
      //nub::soft_ref<OrientationChannel> oriChan;
      //dynCastWeakToFrom(oriChan, vc->subChan("orientation"));

      // use Shape estimator to focus on the attended region
      Image<float> fmask; std::string label;
      if (SeC<SimEventShapeEstimatorOutput>
          e = seq->check<SimEventShapeEstimatorOutput>(0))
        { fmask = e->smoothMask(); label = e->winningLabel(); }

      Image<float> roiImg;
      if (fmask.initialized())
        roiImg = img*fmask;
      else
        { roiImg = img; fmask.resize(img.getDims(),ZEROS); }
      drawCircle(roiImg, winner, 10, 0.0f, 1);
      writeText(roiImg, Point2D<int>(0,0), label.c_str());
      salWin->drawImage(roiImg,0,0);

      printf("want to try this salient point\n");
      char spC = getchar();getchar();
      if(spC == 'y')
      {
        printf("trying the salient point\n");
        for(int i = 0; i < NUM_H_LEV; i++)
        {
          primeLev = probLev[i];
          printf("want to try primeLev: %d \n",primeLev);
          char plC = getchar();getchar();
          if(plC == 'y')
          {
            printf("trying: %d\n",primeLev);
            detectFace(img, brain, winner);
          }
          else
            printf("skipping: %d\n",primeLev);
        }
      }
      else
        printf("skipping the salient point\n");

    }

    if (SIM_BREAK == status) // Brain decided it's time to quit
      break;

    // write outputs or quit?
    bool gotcovert = false;
    if (seq->check<SimEventWTAwinner>(0)) gotcovert = true;
    const FrameState os = ofs->update(seq->now(), gotcovert);

    if (os == FRAME_NEXT || os == FRAME_FINAL)
      brain->save(SimModuleSaveInfo(ofs, *seq));

    if (os == FRAME_FINAL) break;             // done

    // if we displayed a bunch of images, let's pause:
    if (ifs->shouldWait() || ofs->shouldWait())
      Raster::waitForKey();
  }
  */

  while(!(wList.pressedAnyCloseButton()))sleep(1);
  // stop all our ModelComponents
  manager.stop();

  // all done!
  printf("All done\n");
  return 0;
}

// ######################################################################
// setup neural networks for part detectors
void setupPartDetectors()
{
  // instantiate a feed forward network for each face part
  ffn_e.reset(new FeedForwardNetwork());
  ffn_n.reset(new FeedForwardNetwork());
  ffn_m.reset(new FeedForwardNetwork());

  // get the weight values for each part
  std::string h1EName(sformat("%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden1EYE.dat"));
  std::string h2EName(sformat("%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden2EYE.dat"));
  std::string  oEName(sformat("%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"outEYE.dat"));

  std::string h1NName(sformat("%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden1NOSE.dat"));
  std::string h2NName(sformat("%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden2NOSE.dat"));
  std::string  oNName(sformat("%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"outNOSE.dat"));

  std::string h1MName(sformat("%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden1MOUTH.dat"));
  std::string h2MName(sformat("%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden2MOUTH.dat"));
  std::string  oMName(sformat("%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"outMOUTH.dat"));

  // initialize the size of network
  // use 3-weight layer network
  ffn_e->init3L(h1EName, h2EName, oEName,
                RE_INPUT, RE_HIDDEN_1, RE_HIDDEN_2, 1, 0.0, 0.0);
  ffn_n->init3L(h1NName, h2NName, oNName,
                 N_INPUT,  N_HIDDEN_1,  N_HIDDEN_2, 1, 0.0, 0.0);
  ffn_m->init3L(h1MName, h2MName, oMName,
                 M_INPUT,  M_HIDDEN_1,  M_HIDDEN_2, 1, 0.0, 0.0);
}

// ######################################################################
// setup neural networks to train
void setupPrimeLevEstimator()
{
  // instantiate a feed forward network for prime level training
  ffn_pl.reset(new FeedForwardNetwork());

  // get the weight values for each part
  std::string h1PLName(sformat("%s%s",WEIGHT_DATA_FOLDER,"hidden1PL.dat"));
  std::string h2PLName(sformat("%s%s",WEIGHT_DATA_FOLDER,"hidden2PL.dat"));
  std::string  oPLName(sformat("%s%s",WEIGHT_DATA_FOLDER,"outPL.dat"));

  // initialize the size of network
  // use 3-weight layer network
  ffn_pl->init3L(h1PLName, h2PLName, oPLName,
                 NUM_G_LEV*NUM_G_DIR, PL_HIDDEN_1, PL_HIDDEN_2, NUM_H_LEV,
                 0.0, 0.0);
}

// ######################################################################
// detect face
void detectFace(Image<float> img, nub::soft_ref<StdBrain> brain, Point2D<int> winner)
{
  LFATAL("FIXME");
  /*
  printf("detectFace \n");

  Image<float> wImg;
  int scale = int(pow(2,primeLev));

  // move about the local neighborhood to search
  // for possible face part sites

  // get dominant salient feature of the location currently attended
  LFATAL("fixme");
  nub::soft_ref<ShapeEstimator> se;//////// = brain->getSE();
  int lev=3, delta=3; char chanName[30] = "";

  //FIXME//sscanf(se->getWinningLabel().c_str(),
  //FIXME//       "%s %*s %d %*s %d", chanName, &lev, &delta);
  printf("name: %s lev: %d, delta %d\n",chanName,lev,delta);

  Point2D<int> pWinner; float angR = 0.0;

  // if the winner is an intensity channel
  if(chanName[0] == 'I' && chanName[1] == 'n')
  {
    // get an edge around the attended location
    pWinner.i = int(winner.i/pow(2,primeLev));
    pWinner.j = int(winner.j/pow(2,primeLev));
    //pWinner = getIntSalPt(pyrImg[primeLev], pWinner);
    angR = getAng(pWinner);
  }
  // if the winner is a Gabor Channel
  else if(chanName[0] == 'G' && chanName[1] == 'a')
  {
    // starting point of the face finding
    pWinner.i = int(winner.i/pow(2,primeLev));
    pWinner.j = int(winner.j/pow(2,primeLev));

    // get the dominant angle at the attended location
    int angD; sscanf(chanName,"Gabor(%d)",&angD);
    angR = (float)(angD)/180.0 * M_PI;
  }
  else
    LINFO("Neither Gabor Channel nor Intensity Channel");

  // shift the attention location to look for a face
  // limit our shifts to 10 (for now)
  Point2D<int> *attPt = new Point2D<int>[10];
  float dir[10]; dir[0] = angR;
  attPt[0] = pWinner; int nShift = 1;
  typedef double* doubleptr;
  double **pProb = new doubleptr[10];
  for(int i = 0; i < 10; i++) pProb[i] = new double[NUM_FACE_PART];

  // check up if the first salient location has a face part
  // get a vector: probability for each tested part
  getFacePartProb(pWinner, dir[0], pProb[0]);

  // while there is still an edge to follow or HAS SHIFT LESS THAN 10 TIMES
  // limit edges to follow to 20
  int nMax = 1; int maxPt = 20;
  Point2D<int> *pt = new Point2D<int>[maxPt];
  double *ang = new double[maxPt];
  int np;
  int faceFound = 0;

  while((nMax > 0) && (nShift < 10) && !faceFound)
  {
    printf(" winner[%d]: (%d,%d,%f)\n",  nShift-1,
      pWinner.i,pWinner.j, dir[nShift-1]);
    nMax = 0;

    wImg = pyrImg[primeLev];
    inplaceNormalize(wImg, 0.0f, 128.0f);
    // draw scanpath in the work image
    for(int i = 0; i< nShift; i++)
    {
      drawCircle(wImg, attPt[i], 2, 255.0f, 1);
      Point2D<int> ePt(int(6*cos(dir[i])),int(6*sin(dir[i])));
      drawLine(wImg, attPt[i] - ePt, attPt[i] + ePt, 1.0f, 1);
    }

    // get surrounding salient points - limit to maxPt/2
    getLocalSalPt(pWinner, maxPt/2, &pt, &ang, &np);
    int nsal = np;

    // add context points given that we have found some other parts
    getContextPt(attPt, dir, pProb, nShift, maxPt, &pt, &ang, &np);

    // check up all the salient points
    // pick which one we want as the next focus of attention
    double tProb[NUM_FACE_PART];
    double tProbMax[NUM_FACE_PART];
    float maxPVal = 0.0, pVal = 0.0; Point2D<int> maxLoc(0,0); float maxA = 0.0;
    for(int i = 0; i < np; i++)
    {
      // use it to guide next search area
      getFacePartProb(pt[i], ang[i], tProb);
      pVal = getPotVal(attPt,dir,pProb,nShift,pt[i],ang[i],tProb);

      if(i < nsal) printf("SAL "); else printf("CTX ");
      printf("(%d,%d,%5.3f) ",pt[i].i, pt[i].j,ang[i]);
      printf("-> pVal : %f ", pVal);
      printf("( ");
      for(int nPa = 0; nPa < NUM_FACE_PART; nPa++)
        printf("%f ",tProb[nPa]);
      printf(")\n");

      // if this edge is more promising than the previous one
      if(pVal > maxPVal)
        {
          maxPVal = pVal; maxLoc = pt[i]; maxA = ang[i];nMax = 1;
          for(int nPa = 0; nPa < NUM_FACE_PART; nPa++)
            tProbMax[nPa] = tProb[nPa];
        }
      drawCircle(wImg, pt[i], 1, 255.0f, 1);

        //---
        workWin->drawImage(zoomXY(wImg,scale,-1),0,0);
        //printf("PT: ");Raster::waitForKey();
        //---
    }

    // if we found a part in the neighborhood
    if(nMax > 0)
    {
      printf("MAX pt:(%d,%d), ang: %f MVAL: %f\n",
        maxLoc.i,maxLoc.j,maxA,maxPVal);

      attPt[nShift] = maxLoc; dir[nShift] = maxA;
      printf("( ");
      for(int nPa = 0; nPa < NUM_FACE_PART; nPa++)
      {
        pProb[nShift][nPa] = tProbMax[nPa];
        printf("%f ",tProbMax[nPa]);
      }
      printf(")\n");
      nShift++; pWinner = maxLoc;
    }

    // display current progress
    workWin->drawImage(zoomXY(wImg,scale,-1),0,0);
    Raster::waitForKey();

    // hypothesize on the recovered parts if we have found a face
    if(locateFace(attPt,dir,pProb,nShift))
    {
      faceFound = 1;

      // draw the last visited point
      drawCircle(wImg, pWinner, 2, 255.0f, 1);
      workWin->drawImage(zoomXY(wImg,scale,-1),0,0);
      printf("Found a face "); Raster::waitForKey();

      // clean the working image
      // draw the eyes, nose, and mouth and outer face
      wImg = pyrImg[primeLev];
      drawFace(wImg);
    }
  }

  // draw all results
  workWin->drawImage(zoomXY(wImg,scale,-1),0,0);
  printf("end of detectFace \n"); Raster::waitForKey();
  */
}

// ######################################################################
Point2D<int> getIntSalPt(Image<float> img, Point2D<int> p)
{
  // use this image to note that the coordinate has been visited
  Image<float> vis(img.getWidth(), img.getHeight(), ZEROS);

  Point2D<int> a =  getIntSalPt(img,vis,p);
//   XWinManaged xw(vis.getDims(), 700,0,"getIntSalPt");
//   drawImage(xw, vis,0,0);
//   Raster::waitForKey();

  return a;
}

// ######################################################################
Point2D<int> getIntSalPt(Image<float> img, Image<float>& vis, Point2D<int> p)
{
  float a = img.getVal(p); float b;
  if(vis.getVal(p) == 255.0)  return p;
  else vis.setVal(p,255.0);
  Point2D<int> n; Point2D<int> mn; float mc = 0.0;
  int lev = primeLev;

  // recurse outward to find the boundary of the region
  // stop only if the change in value is not too steep
  int w = img.getWidth(); int h = img.getHeight();
  if(p.i > 0)
    {
      n = p + Point2D<int>(-1,0);      b = img.getVal(n);
      if(fabs(a - b)/a < .5 && b > 0.001) n = getIntSalPt(img, vis, n);
      mn = n; mc = getSalG(n,lev);
    }
  if(p.i < w-1)
    {
      n = p + Point2D<int>(1,0);      b = img.getVal(n);
      if(fabs(a - b)/a < .5 && b > 0.001) n = getIntSalPt(img, vis, n);
      if(mc < getSalG(n,lev))
      {    mn = n; mc = getSalG(n,lev); }
    }

  if(p.j > 0)
    {
      n = p + Point2D<int>(0,-1);      b = img.getVal(n);
      if(fabs(a - b)/a < .5 && b > 0.001) n = getIntSalPt(img, vis, n);
      if(mc < getSalG(n,lev))
      {    mn = n; mc = getSalG(n,lev); }
    }
  if(p.j < h-1)
    {
      n = p + Point2D<int>(0,1);      b = img.getVal(n);
      if(fabs(a - b)/a < .5 && b > 0.001) n = getIntSalPt(img, vis, n);
      if(mc < getSalG(n,lev))
      {    mn = n; mc = getSalG(n,lev); }
    }

  return mn;
}

// ######################################################################
float getSalG(Point2D<int> p, int lev)
{
  float max = 0.0;
  for(int i = 0; i< NUM_DIR; i++)
  {
    if(currGaPyr[i][lev].getVal(p) > max)
      max = currGaPyr[i][lev].getVal(p);
  }
  return max;
}

// ######################################################################
XWinManaged *tXW; int txwC = 0;
void getLocalSalPt(Point2D<int> pWinner, int max,
                   Point2D<int> **pt, double **ang, int *np)
{
  // get the region of interest at the primeLev
  *np = 0;
  Image<float> temp  = pyrImg[primeLev];

  //---
  //  int w = temp.getWidth(), h = temp.getHeight();
  //   if(txwC == 0)
  //     {tXW= new XWinManaged(Dims(2*3*w, 3*h), 0,0,"GetFaceProb"); wList.add(*tXW); txwC = 1;}
  //---
  // FIX THE RECTANGLE ERROR
  // get rectangle bounding the local neighborhood
  Rectangle r = getWindow(pyrImg[primeLev], pWinner, 40);
  r = Rectangle::tlbrO(r.top(),r.left(),
                       (r.bottomO()/4)*4,(r.rightO()/4)*4);
  printf("r(%d,%d,%d,%d) = PI[%d,%d],pW(%d,%d) \n",
         r.top(),r.left(),r.bottomI(),r.rightI(),
         pyrImg[primeLev].getWidth(),pyrImg[primeLev].getHeight(),pWinner.i,pWinner.j);

  Image<float> sumS;
  Image<float> sMap;  sMap.resize(r.width(),r.height(),true);

  Image<float> gI[NUM_DIR];
  for(int i = 0; i< NUM_DIR; i++) gI[i].resize(r.width(),r.height(),true);

  // center-surround to localize sharpest edges
  for(int c = 0; c < 3; c++)
    for(int s = c+1; s< 3; s++)
    {
      for(int i = 0; i < NUM_DIR; i++)
      {
        // NOTE: using crop() from Image/CutPaste.H instead of copy(),
        // since copy() was a dup of crop():
        gI[i] = crop(zoomXY(currGaPyr[i][primeLev+c],int(pow(2.0,c)),-1),r) -
                crop(zoomXY(currGaPyr[i][primeLev+s],int(pow(2.0,s)),-1),r)   ;
      }

      sumS = gI[0]+gI[1]+gI[2]+gI[3];
      inplaceNormalize(sumS, 0.0f, 255.0f);
      sumS = squared(squared(sumS));
      sMap+= sumS;

      //---
//       tXW->drawImage(zoomXY(sumS,2*2,-1),2*w,r.height()*4);
//       tXW->drawImage(zoomXY(sMap,2*2,-1),2*w+r.width()*4,r.height()*4);
//       Raster::waitForKey();
      //----
    }

  //----
//    Image<float> temp2 = crop(temp,r);
//    inplaceNormalize(temp2, 0,255.0);
//    drawRect(temp, r, 255,1);
//    tXW->drawImage(zoomXY(temp ,2*1,-1),0,0);
//    tXW->drawImage(zoomXY(temp2,2*2,-1),2*w,0);
//    tXW->drawImage(zoomXY(sumS,2*2,-1),2*w,r.height()*4);
//    Raster::waitForKey();
  //---

  // get each salient points
  // crop out the whole region belonging to that salient point
  sMap = squared(sMap);
  inplaceNormalize(sMap, 0.0f, 255.0f);
  float t;
  for(int i = 0; i < max; i++)
  {
    findMax(sMap, (*pt)[i],t);

    // break when saliency value is below the chosen constant
    if(t < 20.0) break;
    crop(sMap,(*pt)[i]);

    // do not accept points that are to close to other salient points
    for(int j = 0; j < i; j++)
      if((*pt)[j].distance((*pt)[i]) < 6.0)
      {
        j = i+1; // break from the for loop
        i--;(*np)--;
      }

    (*ang)[i] = getAng((*pt)[i]+Point2D<int>(r.left(),r.top()));

    //---
//     printf("I: %d i: %d j: %d val: %f\n",
//            i,(*pt)[i].i,(*pt)[i].j, t);
//     temp2.setVal((*pt)[i],2*255.0);
//     tXW->drawImage(zoomXY(sMap,2*2,-1),2*w+r.width()*4,r.height()*4);
//     tXW->drawImage(zoomXY(temp2,2*2,-1),2*w,0);
//     Raster::waitForKey();
    //---

    (*np)++;
  }
  // add the offset to make points be locations in whole image
  for(int i = 0; i< *np; i++) (*pt)[i] = (*pt)[i] + Point2D<int>(r.left(),r.top());

  //  printf("getLocalSalPt ");Raster::waitForKey();
}

// ######################################################################
// recursive cropping of current salient neighboorhood
void crop(Image<float>& img, Point2D<int> p)
{
  // crop current location
  float a = img.getVal(p);
  float b;
  img.setVal(p,0.0);
  int w = img.getWidth(); int h = img.getHeight();

  // recurse after checking boundary condition
  // crop only if the change in value is not too steep
  if(p.i > 0)
    {
      b = img.getVal(p + Point2D<int>(-1,0));
      if(fabs(a - b)/a < .5 && b > 0.001) crop(img, p+Point2D<int>(-1,0));
    }
  if(p.i < w-1)
    {
      b = img.getVal(p + Point2D<int>(1,0));
      if(fabs(a - b)/a < .5 && b > 0.001) crop(img, p+Point2D<int>(1,0));
    }
  if(p.j > 0)
    {
      b = img.getVal(p + Point2D<int>(0,-1));
      if(fabs(a - b)/a < .5 && b > 0.001) crop(img, p+Point2D<int>(0,-1));
    }
  if(p.j < h-1)
    {
      b = img.getVal(p + Point2D<int>(0,1));
      if(fabs(a - b)/a < .5 && b > 0.001) crop(img, p+Point2D<int>(0,1));
    }
}

// ######################################################################
double getAng(Point2D<int> p)
{
  // get the dominant orientation of the current location
  int iMax = 0; float max = 0.0;
  for(int i = 0; i< NUM_DIR; i++)
  {
    if(currGaPyr[i][primeLev].getVal(p) > max)
      {max = currGaPyr[i][primeLev].getVal(p); iMax = i;}
  }
  return iMax*M_PI/NUM_DIR;
}

// ######################################################################
void getContextPt(Point2D<int> *attPt, float *dir, double **pProb, int n, int max,
                  Point2D<int> **pt, double **ang, int *np)
{
  // use the last attended point
  int i = n-1;

  //printf("getContextPt n:%d max:%d np:%d \n",n,max,*np);
  //printf("PPROB[%d]: (%5.3f,%5.3f,%5.3f)\n",
  //     i, pProb[i][EYE], pProb[i][NOSE], pProb[i][MOUTH]);

  float dx, dy; int dxR, dyR; Point2D<int> temp;
  Rectangle r = pyrImg[primeLev].getBounds();

  // if the likelihood of a part is above 20%
  // make sure this value is not too high
  // so that we won't discard reasonable locations
  if(pProb[i][EYE] > .08)
    {
      float c = cos(dir[i]); float s = sin(dir[i]);

      // add points for the other eye
      dx = 24.0; dy = 0.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = dir[i];
          (*np)++;
        }
      dx = -24.0; dy = 0.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = dir[i];
          (*np)++;
        }

      // add points for the nose
      dx = 12.0; dy = 8.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = fmod(dir[i]+M_PI/2.0, M_PI);
          (*np)++;
        }
      dx = -12.0; dy = 8.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = fmod(dir[i]+M_PI/2.0, M_PI);
          (*np)++;
        }

      // add points for the mouth
      dx = 12.0; dy = 28.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = dir[i];
          (*np)++;
        }
      dx = -12.0; dy = 28.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = dir[i];
          (*np)++;
        }
    }

  if(pProb[i][NOSE] > .08)
    {
      float c = cos(fmod(dir[i]+M_PI/2.0, M_PI));
      float s = sin(fmod(dir[i]+M_PI/2.0, M_PI));

      // add points for the other eye
      dx = 12.0; dy = -8.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = fmod(dir[i]+M_PI/2.0, M_PI);
          (*np)++;
        }
      dx = -12.0; dy = -8.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = fmod(dir[i]+M_PI/2.0, M_PI);
          (*np)++;
        }

      // add points for the mouth
      dx = 0.0; dy = 20.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = fmod(dir[i]+M_PI/2.0, M_PI);
          (*np)++;
        }
    }

  if(pProb[i][MOUTH] > .08)
    {
      float c = cos(dir[i]); float s = sin(dir[i]);

      // add points for the other eye
      dx = 12.0; dy = -28.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = dir[i];
          (*np)++;
        }
      dx = -12.0; dy = -28.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = dir[i];
          (*np)++;
        }

      // add points for the nose
      dx = 0.0; dy = -20.0;
      dxR = int(round( dx*c + -dy*s));
      dyR = int(round( dx*s +  dy*c));
      temp = attPt[i] + Point2D<int>(dxR,dyR);
      if(r.contains(temp) & (*np < max))
        {
          (*pt)[*np]  = temp;
          (*ang)[*np] = fmod(dir[i]+M_PI/2.0, M_PI);
          (*np)++;
        }
    }
}

// ######################################################################
float getPotVal(Point2D<int> *attPt, float *dir, double *pProb[NUM_FACE_PART],
                int n, Point2D<int> pt, float a, double tProb[NUM_FACE_PART])
{
  //printf("pt: (%d,%d) n: %d \n",pt.i,pt.j,n);
  //for(int i = 0; i < n; i++)
  //  printf("attPt[%d]: (%d,%d,%f) \n",i,attPt[i].i,attPt[i].j,dir[i]);

  // check if this location has been visited (CYCLE CHECK)
  // no point of revisiting that point
  for(int i = 0; i < n; i++)
    if(pt.distance(attPt[i]) <= 3.0) return 0.0;

  // relate the point with each previously visited points
  float mVal = 0.0;
  for(int i = 0; i < n; i++)
  {
    // find the best fit relationship between the two points
    for(int o = 0; o < NUM_FACE_PART; o++)
      for(int c = 0; c < NUM_FACE_PART; c++)
      {
        float val = getGeomRelProb((FacePart)o,(FacePart)c,attPt[i],dir[i],pt,a);
        //printf("(%d,%d) -> %f * %f * %f = %f\n",o,c,val,
        //  pProb[i][o],  tProb[c], val * pProb[i][o] * tProb[c]);
        val *= pProb[i][o] * tProb[c];

        if (val > mVal) mVal = val;
      }
    //printf("i: %d\n",i);
  }

  return mVal;
}

// ######################################################################
float getGeomRelProb(FacePart o,FacePart c, Point2D<int> oPt, float oD,
                     Point2D<int> cPt, float cA)
{
  // get the geometric relationship between the two points
  int dx = cPt.i - oPt.i;  int dy = cPt.j - oPt.j;
  float dist = cPt.distance(oPt);
  float dir  = atan2(dy,dx) + oD;
  if(dir > M_PI) dir -= 2.0*M_PI;

  // probability of the agreement of
  // the orientation of the two parts
  float pOri[NUM_DIR] = {0.35, 0.25, 0.005, 0.25};

  // probability relationship between the 2 parts
  // in direction, distance and agreement in part orientation
  // (aProb and dProb , respectively)
  switch(o)
  {
  case EYE: case L_EYE: case R_EYE:
    switch(c)
    {
    case EYE: case L_EYE: case R_EYE:
      {
        // probability of agreement of orientation of the two parts
        int dOri = int(round(4.0 * fabs(oD - cA)/M_PI));

        // probability of distance
        float mean = 24.0; float stdev = 6.0;
        float dProb = 1.0/(stdev*sqrt(2*M_PI))*
          pow(M_E,-1*pow(dist-mean, 2.0)/(2.0*stdev*stdev));

        // probability of direction
        float aProb;
        if(fabs(dir) <= M_PI/2.0)
          aProb = (-fabs(dir)/(M_PI/2) + 1.0)/2.0;
        else
          aProb = ((fabs(dir) - M_PI)/(M_PI/2) + 1.0)/2.0;

        return pOri[dOri] * dProb * aProb;
      }
    case NOSE:
      {
        // probability of agreement of orientation of the two parts
        int dOri = int(round(4.0 * fabs(fmod(oD + M_PI/2.0, M_PI) - cA)/M_PI));

        // probability of distance
        float mean = 15.0; float stdev = 7.0;
        float dProb = 1.0/(stdev*sqrt(2*M_PI))*
          pow(M_E,-1*pow(dist-mean, 2.0)/(2.0*stdev*stdev));

        // probability of direction
        float nDir = fmod(fabs(dir),M_PI/2);
        float aProb = (-fabs(nDir-M_PI/4)/(M_PI/4) + 1.0)/4.0;

        return pOri[dOri] * dProb * aProb;
      }
    case MOUTH:
      {
        // probability of agreement of orientation of the two parts
        int dOri = int(round(4.0 * fabs(oD - cA)/M_PI));

        // probability of distance
        float mean = 30.0; float stdev = 8.0;
        float dProb = 1.0/(stdev*sqrt(2*M_PI))*
          pow(M_E,-1*pow(dist-mean,2.0)/(2.0*stdev*stdev));

        // probability of direction
        float nDir = fmod(fabs(dir),M_PI/2);
        float aProb = (-fabs(nDir-M_PI/4)/(M_PI/4) + 1.0)/4.0;

        return pOri[dOri] * dProb * aProb;
      }
    }

  case NOSE:
    switch(c)
    {
    case EYE: case L_EYE: case R_EYE:
      {
        // probability of agreement of orientation of the two parts
        int dOri = int(round(4.0 * fabs(fmod(oD + M_PI/2.0, M_PI) - cA)/M_PI));

        // probability of distance
        float mean = 15.0; float stdev = 6.0;
        float dProb = 1.0/(stdev*sqrt(2*M_PI))*
          pow(M_E,-1*pow(dist-mean,2.0)/(2.0*stdev*stdev));

        // probability of direction
        float nDir = fmod(fabs(dir),M_PI/2);
        float aProb = (-fabs(nDir-M_PI/4)/(M_PI/4) + 1.0)/4.0;

        return pOri[dOri] * dProb * aProb;
      }
    case NOSE:
      {
        return 0.0;
      }
    case MOUTH:
      {
        // probability of agreement of orientation of the two parts
        int dOri = int(round(4.0 * fabs(fmod(oD + M_PI/2.0, M_PI) - cA)/M_PI));

        // probability of distance
        float mean = 20.0; float stdev = 5.0;
        float dProb = 1.0/(stdev*sqrt(2*M_PI))*
          pow(M_E,-1*pow(dist-mean, 2.0)/(2.0*stdev*stdev));

        // probability of direction
        float nDir = fabs(dir);
        float aProb = (-fabs(nDir-M_PI/2)/(M_PI/2) + 1.0)/2.0;

        return pOri[dOri] * dProb * aProb;
      }
    }

  case MOUTH:
    switch(c)
    {
    case EYE: case L_EYE: case R_EYE:
      {
        // probability of agreement of orientation of the two parts
        int dOri = int(round(4.0 * fabs(oD - cA)/M_PI));

        // probability of distance
        float mean = 30.0; float stdev = 6.0;
        float dProb = 1.0/(stdev*sqrt(2*M_PI))*
          pow(M_E,-1*pow(dist-mean, 2.0)/(2.0*stdev*stdev));

        // probability of direction
        float nDir = fmod(fabs(dir),M_PI/2);
        float aProb = (-fabs(nDir-M_PI/4)/(M_PI/4) + 1.0)/4.0;

        return pOri[dOri] * dProb * aProb;
      }
    case NOSE:
      {
        // probability of agreement of orientation of the two parts
        int dOri = int(round(4.0 * fabs(fmod(oD + M_PI/2.0, M_PI) - cA)/M_PI));

        // probability of distance
        float mean = 20.0; float stdev = 5.0;
        float dProb = 1.0/(stdev*sqrt(2*M_PI))*
          pow(M_E,-1*pow(dist-mean, 2.0)/(2.0*stdev*stdev));

        // probability of direction
        float nDir = fabs(dir);
        float aProb = (-fabs(nDir-M_PI/2)/(M_PI/2) + 1.0)/2.0;

        return pOri[dOri] * dProb * aProb;
      }
    case MOUTH:
      {
        return 0.0;
      }
    }
  }
  return 0.0;
}

// ######################################################################
bool locateFace(Point2D<int> *attPt, float *dir, double *pProb[NUM_FACE_PART],
                int n)
{
  if(n < 4) return false;
  //for(int i = 0; i < n; i++)
  //  printf("PT[%d]: (%d,%d,%f) :(%5.3f,%5.3f,%5.3f) \n",
  //    i,attPt[i].i, attPt[i].j,dir[i],
  //    pProb[i][EYE],pProb[i][NOSE],pProb[i][MOUTH]);

  // store the parts individually
  int nE = 0, nM  = 0, nN = 0;
  Point2D<int> pE[n], pM[n], pN[n];
  float   dE[n], dM[n], dN[n];

  for(int i = 2; i < n; i++)
  {
    if(pProb[i][EYE] > .10)
    { pE[nE] = attPt[i]; dE[nE] = dir[i]; nE++; }

    if(pProb[i][MOUTH] > .10)
    { pM[nM] = attPt[i]; dM[nM] = dir[i]; nM++; }

    if(pProb[i][NOSE] > .0005 && fabs(dir[i] - M_PI/2)<.01 )
    { pN[nN] = attPt[i]; dN[nN] = dir[i]; nN++; }
  }

  printf("-+-+-+-(%d,%d,%d)\n",nE,nM,nN);
  // set face when the configuration of parts is satisfied
  for(int i = 0; i < nE-1; i++)
    for(int i2 = i+1; i2 < nE; i2++)
      for(int j = 0; j < nM; j++)
        for(int k = 0; k < nN; k++)
          {
            // the 2 eyes relationship
            float ee2 = getGeomRelProb(EYE, EYE  , pE[i], dE[i], pE[i2], dE[i2]);

            // the eye-mouth relationship
            float em  = getGeomRelProb(EYE, MOUTH, pE[i], dE[i], pM[j], dM[j]);

            // the eye-nose relationship
            float en  = getGeomRelProb(EYE, NOSE , pE[i], dE[i], pN[k], dN[k]);

            // the nose-mouth relationship
            float nm  = getGeomRelProb(NOSE, MOUTH, pN[k], dN[k], pM[j], dM[j]);

         printf("[%d,%d,%d,%d] ee2: %f, en: %f em: %f nm: %f\n",
               i,i2,k,j,ee2,en,em,nm);
            // condition for a face
            // condition threshold:
            // [Dir][Dist][ori]
            // [.05][  .1][ .2] = .001
            if(ee2 > .001 && em > .001 && en >.002)
            {
              eyeP   = pE[i];  eyeD   = 0.0;//dE[i];
              eye2P  = pE[i2]; eye2D  = 0.0;//dE[i2];
              mouthP = pM[j];  mouthD = 0.0;//dM[j];
              noseP  = pN[k];  noseD  = dN[k];
              return true;
            }
          }
  return false;
}

// ######################################################################
void drawFace (Image<float>& img)
{
  inplaceNormalize(img, 0.0f, 128.0f);

  // draw the eyes
  drawCircle(img, eyeP , 2, 255.0f, 1);
  Point2D<int> ePt(int(5*cos(eyeD)),int(5*sin(eyeD)));
  drawLine(img, eyeP - ePt, eyeP + ePt, 255.0f, 1);
  drawCircle(img, eye2P, 2, 255.0f, 1);
  Point2D<int> e2Pt(int(5*cos(eye2D)),int(5*sin(eye2D)));
  drawLine(img, eye2P - e2Pt, eye2P + e2Pt, 255.0f, 1);

  // draw the nose
  Point2D<int> nPt(int(10*cos(noseD)),int(10*sin(noseD)));
  drawLine(img, noseP - nPt, noseP + nPt, 255.0f, 1);

  // draw the mouth
  drawCircle(img, mouthP , 5, 255.0f, 1);
  Point2D<int> mPt(int(8*cos(mouthD)),int(8*sin(mouthD)));
  drawLine(img, mouthP - mPt, mouthP + mPt, 255.0f, 1);

  // draw the face border
  //drawCircle(img, noseP , 25, 255.0f, 1);
}

// ######################################################################
// normalize image by subtracting it with mean and dividing by stdev
Image<float> normalize(Image<float> img)
{
  Image<float> tImg = img;
  float tMean  = float(mean(img));
  float tStdev = float(stdev(img));
  tImg -= tMean;
  tImg /= tStdev;

  return tImg;
}

// ######################################################################
// setup input and output array for FFT
void setupFFTW(Image<float> img)
{
  int w = img.getWidth(), h = img.getHeight();

  fftw = new FFTWWrapper(w,h);
  input = new double[w*h];
  fftw->init(input);

  // setup output array
  outFftw = new double*[h];
  for(int i = 0; i < h; i++) outFftw[i] = new double[w/2+1];

  // setup the log-gabor masks
  setupGaborMask(img);
}

// ######################################################################
void setupGaborMask(Image<float> img)
{
  // calculate the appropriate gabor filter constant
  int w        = img.getWidth(), h = img.getHeight();
  int dim      = (w<h)?w:h;
  float stdev  = float(floor((dim - 3.0)/2.0/sqrt(10.0)));
  float period = float(stdev * 2.0);

  // setup storage for gabor masks
  gaborMask = new double***[NUM_G_LEV];
  for(int i = 0; i < NUM_G_LEV; i++)
    gaborMask[i] = new double**[NUM_G_DIR];

  //----
  //XWinManaged *xwSGM = new XWinManaged(Dims(2*w,h),0, 0,"xwSGM");
  //---

  // traverse through the different frequency and orientation
  float scale = 1.0;
  for(int  i = 0; i < NUM_G_LEV; i++)
  {
    for(int j = 0; j < NUM_G_DIR; j++)
    {
      // setup the gabor kernel
      Image<float> gaborF = gaborFilter<float>(stdev/scale,period/scale,0,
                                               (j*180.0)/NUM_G_DIR);
      Image<float> temp (img.getDims(), ZEROS);
      inplacePaste(temp, gaborF, Point2D<int>(0,0));

      // allocate space for the solution
      gaborMask[i][j] = new double*[h];
      for(int k = 0; k < h; k++)
        gaborMask[i][j][k] = new double[w/2+1];

      // perform the fourier transform and save the results
      fftCompute(temp, gaborMask[i][j]);
      gaborMaskImg[i][j] = getFftImage(gaborMask[i][j], w, h);

      //---
      //xwSGM->drawImage(temp,0,0);
      //xwSGM->drawImage(gaborMaskImg[i][j],w,0);
      //printf("xwSGM ");Raster::waitForKey();
      //---
    }
    scale *= 2;
  }
}

// ######################################################################
// convert image to array to input to FFT
void fftCompute(Image<float> img, double **outFft)
{
  int w = img.getWidth(), h = img.getHeight();

  iptr = input;
  for(int j = 0; j < h; j++)
    for(int i = 0; i < w; i++)
      *iptr++ = (double)img.getVal(i,j);

  fftw->compute(outFft);
}

// ######################################################################
void getFeatureVector(double **outFft, Image<double> &res, int w, int h)
{
  double sum = 0.0; int c = 0;

  // go through each gabor masks
  for(int i = 0; i < NUM_G_LEV; i++)
    for(int j = 0; j < NUM_G_DIR; j++)
    {
      // weighted sum of fft for a feature
      sum = 0.0;
      for(int k = 0; k < h; k++)
        for(int l = 0; l < w/2+1; l++)
          sum += log(1.0 + gaborMask[i][j][k][l]) * log(1.0 + outFft[k][l]);
      res[c] = sum/(h*w/2+1);
      c++;
    }
}

// ######################################################################
void getFeatureVector2
(nub::soft_ref<StdBrain> brain, Image<double> &vec, int w, int h)
{
  LFATAL("fixme");
  /*
  int w_fm = w/4;
  int h_fm = h/4;

  printf("getFeatureVector\n");
  nub::soft_ref<VisualCortex> vc;/////////// = brain->getVC();
  uint nc = vc->numChans();
  printf("Num Channels: %d\n",nc);

  nub::soft_ref<ColorChannel> cc;
  dynCastWeakToFrom(cc, vc->subChan("color"));
  printf("CC numChans: %d \n",cc->numChans());
  RedGreenChannel& cc_rg = cc->rg();
  BlueYellowChannel& cc_by = cc->by();

  nub::soft_ref<IntensityChannel> ic;
  dynCastWeakToFrom(ic, vc->subChan("intensity"));

  nub::soft_ref<OrientationChannel> oc;
  dynCastWeakToFrom(oc, vc->subChan("orientation"));
  printf("OC numChans: %d \n",oc->numChans());
  GaborChannel& oc_gc000 = oc->gabor(0);
  GaborChannel& oc_gc045 = oc->gabor(1);
  GaborChannel& oc_gc090 = oc->gabor(2);
  GaborChannel& oc_gc135 = oc->gabor(3);

  // use numsubmap
  Image<float> img0 = oc_gc000.getImage(2);
  Image<float> img1 = oc_gc045.getImage(2);
  Image<float> img2 = oc_gc090.getImage(2);
  Image<float> img3 = oc_gc135.getImage(2);
  Image<float> img4 = cc_rg.getImage(2);
  Image<float> img5 = cc_by.getImage(2);
  Image<float> img6 = ic->getImage(2);
  XWinManaged *fWin;

  fWin = new XWinManaged(Dims(w_fm,7*(h_fm)),100,100,"Image");
  wList.add(*fWin);
  fWin->drawImage(img0,0,0);
  fWin->drawImage(img1,0,h_fm);
  fWin->drawImage(img2,0,2*h_fm);
  fWin->drawImage(img3,0,3*h_fm);
  fWin->drawImage(img4,0,4*h_fm);
  fWin->drawImage(img5,0,5*h_fm);
  fWin->drawImage(img6,0,6*h_fm);
  Raster::waitForKey();
  */
}

// ######################################################################
int* getPrimeLev(Image<float> img)
{
  // at this point just use the finest resolution
  int w = img.getWidth(), h = img.getHeight();

  //compute Fourier Transform of the image
  fftCompute(img, outFftw);

  // compute the feature vector
  getFeatureVector(outFftw, gft, w, h);

  // print feature vector
//   for(int i = 0; i < NUM_G_LEV; i++)
//   {
//     for(int j = 0; j < NUM_G_DIR; j++)
//       printf("%12.3f ",gft[i*NUM_G_DIR+j]);
//     printf("\n");
//   }
//   printf("\n\n");

  // run the feature vector on the neural network
  Image<double> vout = ffn_pl->run3L(gft);
  float mVal;  int m; double tLev[NUM_H_LEV];
  int* probLev = new int[NUM_H_LEV];

  for(int i = 0; i < NUM_H_LEV; i++)
    tLev[i] = vout[i];

  for(int i = 0; i < NUM_H_LEV; i++)
  {
    mVal = tLev[0];    m = 0;
    for(int j = 1; j < NUM_H_LEV; j++)
    {
      if (mVal <= tLev[j])
      {
        mVal = tLev[j];  m = j;
      }
    }
    probLev[i] = m;
    tLev[m] = 0.0;
  }

  // display the image of the transform
  Image<float> ftImg = getFftImage(outFftw, w, h);
  float min,max; getMinMax(ftImg, min,max);
  printf("ftIMg: Min: %f, Max: %f\n",min,max);
  inplaceNormalize(ftImg, 0.0f, 255.0f);
  fftWin->drawImage(zoomXY(ftImg,1,-1),0,0);
  return probLev;
}

// ######################################################################
Image<float> getFftImage(double **outFft, int w, int h)
{
  // scaling the large dynamic range of the image Fourier Transform
  // using log: ln(1.0 + |F(i,j)|)
  // origin is centered to the middle of the image
  float ln2 = log(2.0);

  // gamma correction
  float gc = 0.75;

  // redraw the images
  Image<float> ftImg; ftImg.resize(w,h);
  for(int i = 0; i < w/2; i++)
    for(int j = 0; j < h/2; j++)
      ftImg.setVal(i+w/2, h/2-j,
                   pow(log(1.0 + outFft[j][i+1])/ln2, gc));

  for(int i = 0; i < w/2; i++)
    for(int j = h/2; j < h; j++)
      ftImg.setVal(i+w/2, 3*h/2-1-j,
                   pow(log(1.0 + outFft[j][i+1])/ln2, gc));

  for(int i = 0; i < w/2; i++)
    for(int j = 0; j < h; j++)
      ftImg.setVal(i, j, ftImg.getVal(w-1-i, h-1-j));
  return ftImg;
}

// ######################################################################
// free the old block of data whenever a new sample is in
void freeFftwData(Image<float> img)
{
  int h = img.getHeight();

  free(input);

  for(int i = 0; i < h; i++) free(outFftw[i]);
  free(outFftw);

  for(int  i = 0; i < NUM_G_LEV; i++)
    for(int j = 0; j < NUM_G_DIR; j++)
      for(int k = 0; k < h; k++)
        free(gaborMask[i][j][k]);
  for(int  i = 0; i < NUM_G_LEV; i++)
    for(int j = 0; j < NUM_G_DIR; j++)
      free(gaborMask[i][j]);
  for(int i = 0; i < NUM_G_LEV; i++) free(gaborMask[i]);
  free(gaborMask);
}

// ######################################################################
XWinManaged *xwi[2][4]; int xwiC = 0;
void getFacePartProb(Point2D<int> p, float ang, double *pPart)
{
  ang = 0.0;

  // set of gabor images for the inputted window
  Image<float> **gImg = new Image<float>*[2];
  for(int i = 0; i < 2; i++) gImg[i] = new Image<float>[NUM_DIR];

  // use NOSE size (biggest part) + slack
  // to ensure window is big enough to capture a part

  int w = 2+MODEL_NOSE_DIAGONAL, h = 2+MODEL_NOSE_DIAGONAL;
  //  printf("****** p: (%d,%d), w: %d PI[%d,%d]\n",
  //          p.i,p.j,w,pyrImg[primeLev].getWidth(),pyrImg[primeLev].getHeight());
  Rectangle r = getWindow(pyrImg[primeLev], p, w);
  Rectangle r2 =
    Rectangle::tlbrI(r.top()/2               , r.left()/2             ,
                    r.top()/2+r.height()/2-1, r.left()/2+r.width()/2-1);

  // if the image is cut off by the border, add zeros to the edges
  if(r.width() < w || r.height() < h || r2.width() < w/2 || r2.height() < h/2)
  {
    int ph  = r.left()  - p.i   + w/2;
    int pv  = r.top()   - p.j   + h/2;
    int ph2 = r2.left() - p.i/2 + w/4;
    int pv2 = r2.top()  - p.j/2 + h/4;
    //---
//     printf("PI.w: %d, PI.h: %d\n",
//            pyrImg[primeLev].getWidth(),pyrImg[primeLev].getHeight());
//     printf("r.w: %d, r.h: %d,r2.w: %d, r2.h: %d\n",
//         r.width(),r.height(),r2.width(),r2.height());
//     printf("p: (%d,%d), r.l: %d, r.t: %d, r.b: %d, r.r: %d, r2.l: %d, r2.t: %d r2.b: %d, r2.r: %d,w: %d, h: %d\n",
//       p.i,p.j,r.left(),r.top(), r.bottomI(),r.rightI(),
//      r2.left(),r2.top(),r2.bottomI(),r2.rightI(), w,h);
//     printf("ph: %d, pv: %d, ph2: %d, pv2: %d\n",ph,pv,ph2,pv2);

    for(int i = 0; i < NUM_DIR; i++)
    {
      gImg[0][i].resize(w, h, true);
      for(int  j = ph; j < ph + r.width(); j++)
        for(int  k = pv; k < pv + r.height(); k++)
          gImg[0][i].setVal(j,k,
            currGaPyr[i][primeLev].getVal(r.left()+j-ph, r.top()+k-pv));

      gImg[1][i].resize(w/2, h/2, true);
      for(int  j = ph2; j < ph2 + r2.width(); j++)
        for(int  k = pv2; k < pv2 + r2.height(); k++)
          gImg[1][i].setVal(j,k,
            currGaPyr[i][primeLev+1].getVal(r2.left()+j-ph2, r2.top()+k-pv2));
    }
  }
  // else just use the copy operation to fill gabor array to be inputted
  else
    for(int i = 0; i < NUM_DIR; i++)
    {
      gImg[0][i] = crop(currGaPyr[i][primeLev  ], r );
      gImg[1][i] = crop(currGaPyr[i][primeLev+1], r2);
    }

  //---
//   int scl = int(pow(2.0,2));
//    for (int i = 0; i< 2; i++)
//      for (int j = 0; j< 4; j++)
//      {
//        if(xwiC == 0)
//        xwi[i][j] = new XWinManaged(Dims(scl*w,scl*h),scl*j*(h+6),scl*(i+3)*(w+4),"hi");
//        xwi[i][j]->drawImage(zoomXY(gImg[i][j],scl*int(pow(2.0,i)),-1),0,0);
//      }
//    xwiC = 1;
//   Raster::waitForKey();
  //--

  //---
//   float min,max;
//   XWinManaged tXW(Dims(2*2*pyrImg[primeLev].getWidth(),
//                2*pyrImg[primeLev].getHeight()), 0,0,"GetFaceProb");
//   wList.add(tXW);

//   Rectangle tr = Rectangle::tlbrI (h/2 - MODEL_NOSE_HEIGHT/2,     w/2 - MODEL_NOSE_WIDTH/2,
//              h/2 + MODEL_NOSE_HEIGHT/2 - 1, w/2 + MODEL_NOSE_WIDTH/2 - 1);
//   Rectangle tr2 = Rectangle::tlbrI(tr.top()/2                , tr.left()/2             ,
//              tr.top()/2+tr.height()/2-1, tr.left()/2+tr.width()/2-1);
//   printf("t: %d, b: %d, l: %d, r: %d\n",
//       r.top()+tr.top(),r.top()+tr.bottomI(),
//          r.top()+tr.left(),r.top()+tr.rightI());
//   printf("t: %d, b: %d, l: %d, r: %d\n",
//       r2.top()+tr2.top(),r2.top()+tr2.bottomI(),
//          r2.top()+tr2.left(),r2.top()+tr2.rightI());
//   printf("Angle to rotate: %f\n",ang);

//   Image<float> pic  = pyrImg[primeLev];
//   getMinMax(pic, min,max);
//   drawRect(pic, r, max, 1);
//   tXW.drawImage(zoomXY(pic ,2*1,-1),0,0);

//   Image<float> temp  = pyrImg[primeLev];
//   temp = crop(temp,r);
//   //temp = rotate(temp, w/2, h/2,M_PI-ang);
//   getMinMax(temp, min,max);
//   drawRect(temp, tr, max,1);

//   Image<float> temp2 = pyrImg[primeLev+1];
//   temp2 = crop(temp2,r2);
//   //temp2 = rotate(temp2, w/4, h/4, M_PI-ang);
//   getMinMax(temp2, min,max);
//   drawRect(temp2, tr2, max, 1);

//   tXW.drawImage(zoomXY(temp ,2*1,-1),2*pyrImg[primeLev].getWidth(),0);
//   Raster::waitForKey();
//   tXW.drawImage(zoomXY(temp2,2*2,-1),
//     2*pyrImg[primeLev].getWidth(),2*temp.getHeight());
//   Raster::waitForKey();
  //--

  // correct the gabor images (for EYE and MOUTH angle)
  // corrective planar rotation to make face upright
  int a = int(4*ang/M_PI);
  float cAng = ((4-a)%4)*M_PI/4.0;
  //printf("## Ang: %f -> %f a: %d\n\n",ang,cAng,a);
  correctGabors(gImg,cAng);

  //---
//    scl = int(pow(2.0,2));
//    for (int i = 0; i< 2; i++)
//      for (int j = 0; j< 4; j++)
//        xwi[i][j]->drawImage(zoomXY(gImg[i][j],scl*int(pow(2.0,i)),-1),0,0);
//   printf("EYE AND MOUTH "); Raster::waitForKey();
  //--

  // check if it is an eye
  Image<double> eFeat(1, RE_INPUT + 1, NO_INIT);
  Rectangle er =
    Rectangle::tlbrI (h/2 - MODEL_EYE_HEIGHT/2,     w/2 - MODEL_EYE_WIDTH/2,
                     h/2 + MODEL_EYE_HEIGHT/2 - 1, w/2 + MODEL_EYE_WIDTH/2 - 1);
  Rectangle er2 =
    Rectangle::tlbrI(er.top()/2                , er.left()/2             ,
                    er.top()/2+er.height()/2-1, er.left()/2+er.width()/2-1);

  // Eye image array to be inputted
  Image<float> **eImg = new Image<float>*[2];
  for(int i = 0; i < 2; i++) eImg[i] = new Image<float>[NUM_DIR];
  for(int j = 0; j < NUM_DIR; j++)
  {
    eImg[0][j] = crop(gImg[0][j], er );
    eImg[1][j] = crop(gImg[1][j], er2);
  }
  getEyeFeature(eImg, eFeat);
  Image<double> evout = ffn_e->run3L(eFeat);
  pPart[EYE] = evout[0];

  // check if it is a mouth
  Image<double> mFeat(1, M_INPUT + 1, NO_INIT);
  Rectangle mr =
    Rectangle::tlbrI(h/2 - MODEL_MOUTH_HEIGHT/2,     w/2 - MODEL_MOUTH_WIDTH/2,
                    h/2 + MODEL_MOUTH_HEIGHT/2 - 1, w/2 + MODEL_MOUTH_WIDTH/2 - 1);
  Rectangle mr2 =
    Rectangle::tlbrI(mr.top()/2                , mr.left()/2             ,
                    mr.top()/2+mr.height()/2-1, mr.left()/2+mr.width()/2-1);

  // Mouth image array to be inputted
  Image<float> **mImg = new Image<float>*[2];
  for(int i = 0; i < 2; i++) mImg[i] = new Image<float>[NUM_DIR];
  for(int j = 0; j < NUM_DIR; j++)
  {
    mImg[0][j] = crop(gImg[0][j], mr );
    mImg[1][j] = crop(gImg[1][j], mr2);
  }
  getMouthFeature(mImg, mFeat);
  Image<double> mvout =ffn_m->run3L(mFeat);
  pPart[MOUTH] = mvout[0];

  // correct the gabor images (for NOSE angle)
  //a = int(4*tAng/M_PI);
  //cAng = ((4-a)%4)*M_PI/4.0;
  correctGabors(gImg,M_PI/2.0);
  //correctGabors(gImg,cAng);

  //---
//    scl = int(pow(2.0,2));
//    for (int i = 0; i< 2; i++)
//      for (int j = 0; j< 4; j++)
//        xwi[i][j]->drawImage(zoomXY(gImg[i][j],scl*int(pow(2.0,i)),-1),0,0);
//   printf("NOSE "); Raster::waitForKey();
  //--

  // check if it is a nose
  Image<double> nFeat(1, N_INPUT + 1, NO_INIT);
  Rectangle nr =
    Rectangle::tlbrI(h/2 - MODEL_NOSE_HEIGHT/2,     w/2 - MODEL_NOSE_WIDTH/2,
                    h/2 + MODEL_NOSE_HEIGHT/2 - 1, w/2 + MODEL_NOSE_WIDTH/2 - 1);
  Rectangle nr2 =
    Rectangle::tlbrI(nr.top()/2                , nr.left()/2             ,
                    nr.top()/2+nr.height()/2-1, nr.left()/2+nr.width()/2-1);

//   printf("t: %d, b: %d, l: %d, r: %d\n",
//       nr.top(),nr.bottomI(),nr.left(),nr.rightI());
//   printf("t: %d, b: %d, l: %d, r: %d\n",
//       nr2.top(),nr2.bottomI(),nr2.left(),nr2.rightI());
//   printf("Angle to rotate: %f\n",ang);

  // Nose image array to be inputted
  Image<float> **nImg = new Image<float>*[2];
  for(int i = 0; i < 2; i++) nImg[i] = new Image<float>[NUM_DIR];
  for(int j = 0; j < NUM_DIR; j++)
  {
    nImg[0][j] = crop(gImg[0][j], nr );
    nImg[1][j] = crop(gImg[1][j], nr2);
  }

//   printRegion(nImg[0][0],0,11,1,0,13,1);
//   printf(" pr ");Raster::waitForKey();

  getNoseFeature(nImg, nFeat);
  Image<double> nvout = ffn_n->run3L(nFeat);
  pPart[NOSE] = nvout[0];
  //  printf("NOSE: %f \n",pPart[NOSE]);

  // inverse Nose image array to be inputted (in case nose is upside down)
  correctGabors(gImg,M_PI/2.0);
  correctGabors(gImg,M_PI/2.0);

  //---
//    scl = int(pow(2.0,2));
//    for (int i = 0; i< 2; i++)
//      for (int j = 0; j< 4; j++)
//        xwi[i][j]->drawImage(zoomXY(gImg[i][j],scl*int(pow(2.0,i)),-1),0,0);
//   printf("NOSE "); Raster::waitForKey();
  //--

  for(int j = 0; j < NUM_DIR; j++)
  {
    nImg[0][j] = crop(gImg[0][j], nr );
    nImg[1][j] = crop(gImg[1][j], nr2);
  }
  getNoseFeature(nImg, nFeat);
  Image<double> n2vout = ffn_n->run3L(nFeat);
  if(pPart[NOSE] <  n2vout[0])
    pPart[NOSE] = n2vout[0];

  //  printf("p EYE:%5.3f, NOSE: %5.3f MOUTH:%5.3f \n",
  //       pPart[EYE],pPart[NOSE],pPart[MOUTH]);
}

// ######################################################################
// frame a salient point with a rectangle to used for part testing
Rectangle getWindow(Image<float> img, Point2D<int> p, int s)
{
  int w = img.getWidth(), h = img.getHeight();

  int t = (p.j - s/2 < 0        )?     0 : p.j - s/2;
  int l = (p.i - s/2 < 0        )?     0 : p.i - s/2;
  int b = (p.j + s/2 - 1 > h - 1)? h - 1 : p.j + s/2 - 1;
  int r = (p.i + s/2 - 1 > w - 1)? w - 1 : p.i + s/2 - 1;

  return Rectangle::tlbrI(t,l,b,r);
}

// ######################################################################
XWinManaged *xwCG[2][4];
void correctGabors(Image<float> **gImg, float ang)
{
//    int w = gImg[0][0].getWidth(), h = gImg[0][1].getHeight();
//    int scl = int(pow(2.0,2));
//    for (int i = 0; i< 2; i++)
//      for (int j = 0; j< 4; j++)
//      {
//        xwCG[i][j] = new XWinManaged(Dims(scl*w,scl*h),scl*i*(w+4),scl*j*(h+6),"hello");
//       xwCG[i][j]->drawImage(zoomXY(gImg[i][j],scl*int(pow(2.0,i)),-1),0,0);
//      }
//  Raster::waitForKey();

  // rotate each of the gabor output
  for(int j = 0; j < NUM_DIR; j++)
  {
    gImg[0][j] = rotate(gImg[0][j], MODEL_NOSE_DIAGONAL/2+1,
                        MODEL_NOSE_DIAGONAL/2+1   , ang);
    gImg[1][j] = rotate(gImg[1][j], (MODEL_NOSE_DIAGONAL/2+1)/2,
                       (MODEL_NOSE_DIAGONAL/2+1)/2, ang);
  }

//   for (int i = 0; i< 2; i++)
//     for (int j = 0; j< 4; j++)
//       xwCG[i][j]->drawImage(zoomXY(gImg[i][j],scl*int(pow(2.0,i)),-1),0,0);
//   Raster::waitForKey();

  Image<float> tG[2][4];
  for (int i = 0; i< 2; i++)
    for (int j = 0; j< 4; j++)
      tG[i][j] = gImg[i][j];

  int i = int(round(ang*4/M_PI));
  for(int j = 0; j < NUM_DIR; j++)
  {
    gImg[0][j] = tG[0][(i+j)%4];
    gImg[1][j] = tG[1][(i+j)%4];
  }

//   for (int i = 0; i< 2; i++)
//     for (int j = 0; j< 4; j++)
//       xwCG[i][j]->drawImage(zoomXY(gImg[i][j],scl*int(pow(2.0,i)),-1),0,0);
//   Raster::waitForKey();

}

// ######################################################################
// get the face part features
void getFacePartFeatures(Rectangle r, FacePart part, Image<double> &features)
{
  switch(part)
    {
    case EYE:   getEyeFeature  (r, features);     break;
    case NOSE:  getNoseFeature (r, features);     break;
    case MOUTH: getMouthFeature(r, features);     break;
    default:
      LFATAL("Unknown face part");
    }
}

// ######################################################################
void getEyeFeature(Rectangle r, Image<double> &features)
{
  //displayPartImage(currImg,r);
  Image<float> **img = new Image<float>*[2];
  for(int i = 0; i < 2; i++) img[i] = new Image<float>[NUM_DIR];

  Rectangle r2 =
    Rectangle::tlbrI(r.top()/2               , r.left()/2              ,
                    r.top()/2+r.height()/2-1, r.left()/2+r.width()/2-1 );

  // image array to be inputted
  for(int j = 0; j < NUM_DIR; j++)
    img[0][j] = crop(currGaPyr[j][primeLev  ], r );
  for(int j = 0; j < NUM_DIR; j++)
    img[1][j] = crop(currGaPyr[j][primeLev+1], r2);
  getEyeFeature(img, features);
}

// ######################################################################
void getEyeFeature(Image<float> **img, Image<double> &features)
{
  // the eye model is 12 by 6
  // we will extract information from specific points only
  // so as to reduce the input numbers

  // for level 1
  // ______________
  // |    XXXX    |
  // |X  X    X  X|
  // |X  X    X  X|
  // |X  X    X  X|
  // |X  X    X   |
  // |____XXXX____|

  int totalF1 = 24;
  int xF1[24] = { 4, 5, 6, 7, 4, 5, 6, 7, 0, 3, 8,11,
                  0, 3, 8,11, 0, 3, 8,11, 0, 3, 8,11 };
  int yF1[24] = { 0, 0, 0, 0, 5, 5, 5, 5, 1, 1, 1, 1,
                  2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4 };

  // for level 2
  // ________
  // |  XX  |
  // |X XX X|
  // |__XX__|

  int totalF2 = 8;
  int xF2[8]  = { 2, 3, 0, 2, 3, 5, 2, 3 };
  int yF2[8]  = { 0, 0, 1, 1, 1, 1, 2, 2 };

  int k = 0;
  // get feature from all angle of gabor response
  for(int n = 0; n < NUM_DIR; n++)
  {
    for(int i = 0; i< totalF1; i++)
    {
        features[k] = double(img[0][n].getVal(xF1[i], yF1[i]));
        k++;
    }

    for(int i = 0; i< totalF2; i++)
    {
        features[k] = double(img[1][n].getVal(xF2[i], yF2[i]));
        k++;
    }
  }
}

// ######################################################################
void getNoseFeature(Rectangle r, Image<double> &features)
{
  //displayPartImage(currImg,r);
  Image<float> **img = new Image<float>*[2];
  for(int i = 0; i < 2; i++) img[i] = new Image<float>[NUM_DIR];

  Rectangle r2 =
    Rectangle::tlbrI(r.top()/2               , r.left()/2              ,
                    r.top()/2+r.height()/2-1, r.left()/2+r.width()/2-1 );

  // image array to be inputted
  for(int j = 0; j < NUM_DIR; j++)
    img[0][j] = crop(currGaPyr[j][primeLev  ], r );
  for(int j = 0; j < NUM_DIR; j++)
    img[1][j] = crop(currGaPyr[j][primeLev+1], r2);
  getNoseFeature(img, features);
}

// ######################################################################
void getNoseFeature(Image<float> **img, Image<double> &features)
{
  // the Nose model is 20 by 10
  // we will extract information from specific points only
  // so as to reduce the number of inputs

  // for level 1
  // ________________
  // |              |
  // |              |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |   XXXXXXXX   |
  // |   X      X   |
  // |   X      X   |
  // |   X      X   |
  // |   X      X   |
  // |   XXXXXXXX   |
  // |              |
  // |______________|

  int totalF1 = 44;
  int xF1[44] = { 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7,
                  3, 4, 5, 6, 7, 8, 9,10, 3,10, 3,10, 3,10, 3,10, 3, 4, 5, 6,
                  7, 8, 9,10 };
  int yF1[44] = { 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9,10,10,11,11,
                 12,12,12,12,12,12,12,12,13,13,14,14,15,15,16,16,17,17,17,17,
                 17,17,17,17 };

  // for level 2
  // _________
  // |       |
  // |   X   |
  // |   X   |
  // |   X   |
  // |   X   |
  // |   X   |
  // | XXXXX |
  // | X   X |
  // | XXXXX |
  // |_______|

  int totalF2 = 17;
  int xF2[17]  = { 3, 3, 3, 3, 3, 1, 2, 3, 4, 5, 1, 5, 1, 2, 3, 4, 5 };
  int yF2[17]  = { 1, 2, 3, 4, 5, 6, 6, 6, 6, 6, 7, 7, 8, 8, 8, 8, 8 };

  int k = 0;

  // get feature from all angle of gabor response
  for(int n = 0; n < NUM_DIR; n++)
  {
    for(int i = 0; i< totalF1; i++)
    {
        features[k] = double(img[0][n].getVal(xF1[i], yF1[i]));
        k++;
    }

    for(int i = 0; i< totalF2; i++)
    {
        features[k] = double(img[1][n].getVal(xF2[i], yF2[i]));
        k++;
    }
  }
}

// ######################################################################
void getMouthFeature(Rectangle r, Image<double> &features)
{
  //displayPartImage(currImg,r);
  Image<float> **img = new Image<float>*[2];
  for(int i = 0; i < 2; i++) img[i] = new Image<float>[NUM_DIR];

  Rectangle r2 =
    Rectangle::tlbrI(r.top()/2               , r.left()/2              ,
                    r.top()/2+r.height()/2-1, r.left()/2+r.width()/2-1 );

  // image array to be inputted
  for(int j = 0; j < NUM_DIR; j++)
    img[0][j] = crop(currGaPyr[j][primeLev  ], r );
  for(int j = 0; j < NUM_DIR; j++)
    img[1][j] = crop(currGaPyr[j][primeLev+1], r2);
  getMouthFeature(img, features);
}

// ######################################################################
void getMouthFeature(Image<float> **img, Image<double> &features)
{
  // the Mouth model is 20 by 10
  // we will extract information from specific points only
  // so as to reduce the number of inputs

  // for level 1
  // ______________________
  // |                    |
  // |                    |
  // |  X    XXXXXX   X   |
  // |  X      XX     X   |
  // |  X      XX     X   |
  // |  X      XX     X   |
  // |  X      XX     X   |
  // |  X    XXXXXX   X   |
  // |                    |
  // |____________________|

  int totalF1 = 32;
  int xF1[32] = { 2, 7, 8, 9,10,11,12,17, 2, 9,10,17, 2, 9,10,17,
                  2, 9,10,17, 2, 9,10,17, 2, 7, 8, 9,10,11,12,17 };
  int yF1[32] = { 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4,
                  5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7 };

  // for level 2
  // ____________
  // |          |
  // | X XXXX X |
  // | X  XX  X |
  // | X XXXX X |
  // |__________|

  int totalF2 = 16;
  int xF2[16]  = { 1, 3, 4, 5, 6, 8, 1, 4, 5, 8, 1, 3, 4, 5, 6, 8 };
  int yF2[16]  = { 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3 };

  int k = 0;

  // get feature from all angle of gabor response
  for(int n = 0; n < NUM_DIR; n++)
  {
    for(int i = 0; i< totalF1; i++)
    {
        features[k] = double(img[0][n].getVal(xF1[i], yF1[i]));
        k++;
    }

    for(int i = 0; i< totalF2; i++)
    {
        features[k] = double(img[1][n].getVal(xF2[i], yF2[i]));
        k++;
    }
  }
}

// ######################################################################
XWinManaged *partWin;
int start = 0;
void displayPartImage(Image<float> img, Rectangle pr)
{
  // for debugging
  if(start ==  0)
  {
    partWin = new XWinManaged(img.getDims(),1000,700,"Image"); wList.add(*partWin);
    start = 1;
  }

  drawRect(img, pr, 255.0f, 1);
  partWin->drawImage(img,0,0);
  Raster::waitForKey();
}

// ######################################################################
// simple image printing procedure
void printRegion(Image<float> img,int sX,int eX,int dX, int sY,int eY, int dY)
{
  for(int j = sY; j<=eY; j+=dY)
  {
    for(int i = sX; i<=eX; i+=dX)
      printf("%8.3f ", img.getVal(i,j));
    printf(" \n");
  }
  printf("\n");
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
