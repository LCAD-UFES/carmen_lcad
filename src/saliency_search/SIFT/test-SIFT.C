/*! @file SIFT/test-SIFT.C test the SIFT alg */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/test-SIFT.C $
// $Id: test-SIFT.C 14376 2011-01-11 02:44:34Z pez $
//


#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Media/FrameSeries.H"
#include "Util/Timer.H"
#include "GUI/XWinManaged.H"
#include "SIFT/ScaleSpace.H"
#include "SIFT/VisualObject.H"
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObjectDB.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include "Image/DrawOps.H"
#include "Media/TestImages.H"
#include "Raster/Raster.H"
#include "Transport/FrameInfo.H"
#include "Raster/Raster.H"
#include "Raster/GenericFrame.H"

#define DISPSCALE 1
#define USECOLOR false

//the visual database
VisualObjectDB vdb;
#define WIDTH 384
#define HEIGHT 288

std::string matchObject(Image<PixRGB<byte> > &ima);

/*
XWinManaged xwin(Dims(WIDTH,HEIGHT*2), 1, 1, "Test SIFT");


rutz::shared_ptr<VisualObject> objTop, objBottom;

void showObjs(rutz::shared_ptr<VisualObject> obj1, rutz::shared_ptr<VisualObject> obj2){
        //return ;

        Image<PixRGB<byte> > keyIma = rescale(obj1->getKeypointImage(),
                        WIDTH, HEIGHT);
        objTop = obj1;

        if (obj2.is_valid()){
                keyIma = concatY(keyIma, rescale(obj2->getKeypointImage(),
                                        WIDTH, HEIGHT));
                objBottom = obj2;
        }

        xwin.drawImage(keyIma);
}

void showKeypoint(rutz::shared_ptr<VisualObject> obj, int keypi,
                Keypoint::CHANNEL channel = Keypoint::ORI){

        char winTitle[255];
        switch(channel){
                case Keypoint::ORI:
                        sprintf(winTitle, "Keypoint view (Channel ORI)");
                        break;
                case Keypoint::COL:
                        sprintf(winTitle, "Keypoint view (Channel COL)");
         break;
                default:
                        sprintf(winTitle, "Keypoint view (Channel   )");
                        break;
        }


        rutz::shared_ptr<Keypoint> keyp = obj->getKeypoint(keypi);
        float x = keyp->getX();
        float y = keyp->getY();
        float s = keyp->getS();
        float o = keyp->getO();
        float m = keyp->getM();

        uint FVlength = keyp->getFVlength(channel);
        if (FVlength<=0) return; //dont show the Keypoint if we dont have a FV

        XWinManaged *xwinKey = new XWinManaged(Dims(WIDTH*2,HEIGHT), -1, -1, winTitle);


        //draw the circle around the keypoint
        const float sigma = 1.6F * powf(2.0F, s / float(6 - 3));
        const float sig = 1.5F * sigma;
        const int rad = int(3.0F * sig);

        Image<PixRGB<byte> > img = obj->getImage();
        Point2D<int> loc(int(x + 0.5F), int(y + 0.5F));
        drawCircle(img, loc, rad, PixRGB<byte>(255, 0, 0));
        drawDisk(img, loc, 2, PixRGB<byte>(255,0,0));

        s=s*5.0F; //mag for scale
        if (s > 0.0f) drawLine(img, loc,
                        Point2D<int>(int(x + s * cosf(o)  + 0.5F),
                                int(y + s * sinf(o) + 0.5F)),
                        PixRGB<byte>(255, 0, 0));

        char info[255];
        sprintf(info, "(%0.2f,%0.2f) s=%0.2f o=%0.2f m=%0.2f", x, y, s, o, m);

        writeText(img, Point2D<int>(0, HEIGHT-20), info,
                        PixRGB<byte>(255), PixRGB<byte>(127));


        //draw the vectors from the features vectors

        Image<PixRGB<byte> > fvDisp(WIDTH, HEIGHT, NO_INIT);
        fvDisp.clear(PixRGB<byte>(255, 255, 255));
        int xBins = int((float)WIDTH/4);
        int yBins = int((float)HEIGHT/4);

        drawGrid(fvDisp, xBins, yBins, 1, 1, PixRGB<byte>(0, 0, 0));



        switch (channel){
                case Keypoint::ORI:
                        for (int xx=0; xx<4; xx++){
                                for (int yy=0; yy<4; yy++){
                                        for (int oo=0; oo<8; oo++){
                                                Point2D<int> loc(xBins/2+(xBins*xx), yBins/2+(yBins*yy));
                                                byte mag = keyp->getFVelement(xx*32+yy*8+oo, channel);
                                                mag = mag/4;
                                                drawDisk(fvDisp, loc, 2, PixRGB<byte>(255, 0, 0));
                                                drawLine(fvDisp, loc,
                                                                Point2D<int>(int(loc.i + mag*cosf(oo*M_PI/4)),
                                                                        int(loc.j + mag*sinf(oo*M_PI/4))),
                                                                PixRGB<byte>(255, 0, 0));
                                        }
                                }
                        }
                        break;

                case Keypoint::COL:
                        for (int xx=0; xx<4; xx++){
                                for (int yy=0; yy<4; yy++){
                                        for (int cc=0; cc<3; cc++){
                                                Point2D<int> loc(xBins/2+(xBins*xx), yBins/2+(yBins*yy));
                                                byte mag = keyp->getFVelement(xx*12+yy*3+cc, channel);
                                                mag = mag/4;
                                                drawDisk(fvDisp, loc, 2, PixRGB<byte>(255, 0, 0));
                                                drawLine(fvDisp, loc,
                                                                Point2D<int>(int(loc.i + mag*cosf(-1*cc*M_PI/2)),
                                                                        int(loc.j + mag*sinf(-1*cc*M_PI/2))),
                                                                PixRGB<byte>(255, 0, 0));
                                        }
                                }
                        }
                        break;
                default:
                        break;
        }



        Image<PixRGB<byte> > disp = img;
        disp = concatX(disp, fvDisp);


        xwinKey->drawImage(disp);

        while(!xwinKey->pressedCloseButton()){
                usleep(100);
        }
        delete xwinKey;

}



void analizeImage(){
   int key = -1;

        while(key != 24){ // q to quit window
                key = xwin.getLastKeyPress();
                Point2D<int>  point = xwin.getLastMouseClick();
                if (point.i > -1 && point.j > -1){

                        //get the right object
                        rutz::shared_ptr<VisualObject> obj;
                        if (point.j < HEIGHT){
                                obj = objTop;
                        } else {
                                obj = objBottom;
                                point.j = point.j - HEIGHT;
                        }
                        LINFO("ClickInfo: key = %i, p=%i,%i", key, point.i, point.j);

                        //find the keypoint
                        for(uint i=0; i<obj->numKeypoints(); i++){
                                rutz::shared_ptr<Keypoint> keyp = obj->getKeypoint(i);
                                float x = keyp->getX();
                                float y = keyp->getY();

                                if ( (point.i < (int)x + 5 && point.i > (int)x - 5) &&
                                          (point.j < (int)y + 5 && point.j > (int)y - 5)){
                                        showKeypoint(obj, i, Keypoint::ORI);
                                        showKeypoint(obj, i, Keypoint::COL);
                                }

                        }

                }
        }

}
*/
int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager manager("Test SIFT");



  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);



  if (manager.parseCommandLine(
        (const int)argc, (const char**)argv, "<database file> <trainingLabel>", 2, 2) == false)
    return 0;

  manager.start();

  Timer masterclock;                // master clock for simulations
  Timer timer;

  const char *vdbFile = manager.getExtraArg(0).c_str();
  const char *trainingLabel = manager.getExtraArg(1).c_str();

  int numMatches = 0; //the number of correct matches
  int totalObjects = 0; //the number of objects presented to the network
  int uObjId = 0; //a unique obj id for sift

  bool train = false;
  //load the database file
 // if (!train)
  vdb.loadFrom(std::string(vdbFile));

  while(1)
  {
    Image< PixRGB<byte> > inputImg;
    const FrameState is = ifs->updateNext();
    if (is == FRAME_COMPLETE)
      break;

    //grab the images
    GenericFrame input = ifs->readFrame();
    if (!input.initialized())
      break;
    inputImg = input.asRgb();
    totalObjects++;

    ofs->writeRGB(inputImg, "Input", FrameInfo("Input", SRC_POS));


    if (train)
    {
      //add the object to the database
      char objName[255]; sprintf(objName, "%s_%i", trainingLabel, uObjId);
      uObjId++;
      rutz::shared_ptr<VisualObject>
        vo(new VisualObject(objName, "NULL", inputImg,
              Point2D<int>(-1,-1),
              std::vector<float>(),
              std::vector< rutz::shared_ptr<Keypoint> >(),
              USECOLOR));

      vdb.addObject(vo);
    } else {

      //get the object classification
      std::string objName;
      std::string tmpName = matchObject(inputImg);
      int i = tmpName.find("_");
      objName.assign(tmpName, 0, i);
      LINFO("Object name %s", objName.c_str());
      printf("%i %s\n", ifs->frame(), objName.c_str());

      if (objName == trainingLabel)
        numMatches++;

      //printf("objid %i:class %i:rate=%0.2f\n",
      //    objData.description.c_str(), objData.id, cls,
      //    (float)numMatches/(float)totalObjects);
    }
  }

  if (train)
  {
    printf("Trained on %i objects\n", totalObjects);
    printf("Object in db %i\n" , vdb.numObjects());
    vdb.saveTo(std::string(vdbFile));
  } else {
    printf("Classification Rate: %i/%i %0.2f\n",
        numMatches, totalObjects,
        (float)numMatches/(float)totalObjects);
  }


}

std::string matchObject(Image<PixRGB<byte> > &ima){

  //find object in the database
  std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;
  rutz::shared_ptr<VisualObject>
    vo(new VisualObject("PIC", "PIC", ima,
                        Point2D<int>(-1,-1),
                        std::vector<float>(),
                        std::vector< rutz::shared_ptr<Keypoint> >(),
                        USECOLOR));

  const uint nmatches = vdb.getObjectMatches(vo, matches, VOMA_SIMPLE,
      10000U, //max objs to return
      0.5F, //keypoint distance score default 0.5F
      0.5F, //affine distance score default 0.5F
      1.0F, //minscore  default 1.0F
      3U, //min # of keypoint match
      100U, //keypoint selection thershold
      false //sort by preattentive
      );

  std::string objName;
  //LINFO("Found %i", nmatches);
  if (nmatches > 0 ){
    rutz::shared_ptr<VisualObject> obj; //so we will have a ref to the last matches obj
    rutz::shared_ptr<VisualObjectMatch> vom;
    //for(unsigned int i=0; i< nmatches; i++){
    for(unsigned int i=0; i< 1; i++){
      vom = matches[i];
      obj = vom->getVoTest();

//      LINFO("### Object match with '%s' score=%f ID:%i",
//          obj->getName().c_str(), vom->getScore(), objId);
      objName = obj->getName();
    }

  }

  return objName;
}
