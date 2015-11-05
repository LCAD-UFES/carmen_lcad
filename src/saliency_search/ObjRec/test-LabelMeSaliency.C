/*! @file ObjRec/test-LabelMeSaliency.C get saliency information from  */
/* the label me data set */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-LabelMeSaliency.C $
// $Id: test-LabelMeSaliency.C 10982 2009-03-05 05:11:22Z itti $
//


#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/ShapeOps.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/ColorOps.H"
#include "Image/Transforms.H"
#include "Image/MathOps.H"
#include "Neuro/StdBrain.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/TaskRelevanceMap.H"
#include "Neuro/SaliencyMap.H"
#include "Media/TestImages.H"
#include "Media/SceneGenerator.H"
#include "Media/MediaSimEvents.H"
#include "Channels/DescriptorVec.H"
#include "Channels/ComplexChannel.H"
#include "Simulation/SimEventQueue.H"
#include "Simulation/SimulationOpts.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Channels/SubmapAlgorithmBiased.H"
#include "GUI/DebugWin.H"
#include "ObjRec/MaskBiaser.H"



Point2D<int> evolveBrain(Image<PixRGB<byte> > &img, Image<float> &SMap, float *interestLevel,
                nub::soft_ref<SimEventQueue> seq);

ModelManager *mgr;
void biasVC(ComplexChannel &vc, Image<float> &mask);
bool debug = 1;
int main(const int argc, const char **argv)
{

        MYLOGVERB = LOG_INFO;
        mgr = new ModelManager("Test LabelMeSaliency");

        nub::soft_ref<SimEventQueueConfigurator>
                seqc(new SimEventQueueConfigurator(*mgr));
        mgr->addSubComponent(seqc);

        //our brain
        nub::ref<StdBrain>  brain(new StdBrain(*mgr));
        mgr->addSubComponent(brain);


        mgr->exportOptions(MC_RECURSE);
        mgr->setOptionValString(&OPT_RawVisualCortexChans, "IOC");
        //mgr.setOptionValString(&OPT_RawVisualCortexChans, "I");
        //mgr->setOptionValString(&OPT_RawVisualCortexChans, "GNO");
        //mgr.setOptionValString(&OPT_RawVisualCortexChans, "N");
        //manager.setOptionValString(&OPT_UseOlderVersion, "false");
        // set the FOA and fovea radii
        mgr->setOptionValString(&OPT_SaliencyMapType, "Fast");
        mgr->setOptionValString(&OPT_SMfastInputCoeff, "1");

        mgr->setOptionValString(&OPT_WinnerTakeAllType, "Fast");
        mgr->setOptionValString(&OPT_SimulationTimeStep, "0.2");

        mgr->setModelParamVal("FOAradius", 128, MC_RECURSE);
        mgr->setModelParamVal("FoveaRadius", 128, MC_RECURSE);

        mgr->setOptionValString(&OPT_IORtype, "Disc");

        if (mgr->parseCommandLine(
                                (const int)argc, (const char**)argv, "<path to images>", 1, 1) == false);

        nub::soft_ref<SimEventQueue> seq = seqc->getQ();

        mgr->start();
        //nub::ref<StdBrain>  brain = dynCastWeak<StdBrain>(mgr->subComponent("Brain"));

        //"/lab/ilab15/tmp/objectsDB/mit/labelMe/05june05_static_indoor",

        ComplexChannel *cc =
                &*dynCastWeak<ComplexChannel>(brain->getVC());

        TestImages testImages(mgr->getExtraArg(0).c_str(), TestImages::MIT_LABELME);

        Image<float> allObjImg = Raster::ReadFloat("allObjImg.pfm", RASFMT_PFM);
        inplaceNormalize(allObjImg, 0.0F, 1.0F);

        printf("## \"Filename\", \"Size\",\"fovea Radius\",\"Number of objects\",\"Salient Location\", \"Hits\",");
        printf("\"Obj Saliency Max\",\"Obj Saliency Min\",\"Obj Saliency Sum\",\"Obj Saliency Area\"");
        printf("\"Dist Saliency Max\",\"Dist Saliency Min\",\"Dist Saliency Sum\",\"Dist Saliency Area\"");
        printf("\n");

        for(uint scene=0; scene<testImages.getNumScenes(); scene++)
        {

                //get the image
                LINFO("Get scene %i", scene);
                Image<PixRGB<byte> > img = testImages.getScene(scene);
                std::string sceneFile = testImages.getSceneFilename(scene);
                LINFO("Size %ix%i", img.getWidth(), img.getHeight());


                //set the fovea and foa radius to be 1/4 the size of the image width
                int fRadius = 128; //(img.getWidth()/16);
                //TODO: fixme
                //brain->getSM()->setFoveaRadius(fRadius);
                //brain->getSM()->setFOAradius(fRadius);

                initRandomNumbers();

                if (testImages.getNumObj() > 0)  //if we have any labled objects
                {

                        //bias the vc
                        Image<float> mask = rescale(allObjImg, img.getDims());
                        biasVC(*cc, mask);
                        //evolve the brain

                        Image<float> SMap;

                        rutz::shared_ptr<SimEventInputFrame> //place the image in the queue
                                e(new SimEventInputFrame(brain.get(), GenericFrame(img), 0));
                        seq->post(e);

                        //set the task relevance map

                        Point2D<int> winner;
                        float interestLevel=100.0F;
                        int nHits=0;
                        int nTimes=95;

                        printf("[ ");
                        Point2D<int> lastLoc(-1,-1);
                        while(interestLevel > 0.01F && nTimes < 100) //do until no more activation
                        {
                                nTimes++;
                                LINFO("InterestLevel %f", interestLevel);
                                Point2D<int> currentWinner = evolveBrain(img, SMap, &interestLevel, seq);

                                if (debug)
                                {
                                        if (lastLoc.isValid())
                                        {
                                                        drawLine(img, lastLoc, currentWinner,
                                                                         PixRGB<byte>(0, 255, 0), 4);
                                        } else {
                                                drawCircle(img, currentWinner, fRadius-10, PixRGB<byte>(255,0,0), 3);
                                        }
                                        lastLoc = currentWinner;


                                         drawCircle(img, currentWinner, fRadius, PixRGB<byte>(0,255,0), 3);
                                }

                                //check if the winner is inside an object (all objects)
                                int hit = -1;
                                for (uint obj=0; obj<testImages.getNumObj(); obj++)
                                {
                                        int lineWidth = int(img.getWidth()*0.003);
                                        std::vector<Point2D<int> > objPoly = testImages.getObjPolygon(obj);
                                        if (debug)
                                        {
                                                Point2D<int> p1 = objPoly[0];
                                                for(uint i=1; i<objPoly.size(); i++)
                                                {
                                                        drawLine(img, p1, objPoly[i], PixRGB<byte>(255, 0, 0), lineWidth);
                                                        p1 = objPoly[i];
                                                }
                                                drawLine(img, p1, objPoly[0], PixRGB<byte>(255, 0, 0), lineWidth); //close the polygon
                                        }

                                        // if (testImages.pnpoly(objPoly, winner))
                                        //  hit = 1;
                                        if (testImages.pnpoly(objPoly, currentWinner))
                                        {
                                                hit = obj;
                                        }

                                }
                                printf("%i ", hit);
                                if (hit != -1)
                                {
                                        winner = currentWinner;
                                        nHits++;
                                }

                        }

                        if (debug)
                        {
                                                        Raster::WriteRGB(img, "IORSaliency.ppm");
                                Image<PixRGB<byte> > tmp  = rescale(img, 512, 512);
                                SHOWIMG(tmp);
                        }
                        printf("] ");
                        printf("\"%s\",\"%ix%i\",\"%i\",\"%i\",\"(%i,%i)\",\"%i\"",
                                        sceneFile.c_str(), img.getWidth(), img.getHeight(), fRadius,
                                        testImages.getNumObj(), winner.i, winner.j, nHits);
                        printf("\n");

                        if (debug)
                        {
                                Image<PixRGB<byte> > tmp  = rescale(img, 512, 512);
                                SHOWIMG(tmp);

                        }


                        //Compute the saliency ratio
                        /*Image<byte> imgMask;
                        //get the obj mask
                        for (uint obj=0; obj<testImages.getNumObj(); obj++)
                        {
                        LINFO("Adding obj %i", obj);
                        Image<byte> objMask = testImages.getObjMask(obj);
                        if (imgMask.initialized())
                        imgMask += objMask;
                        else
                        imgMask = objMask;
                        }
                        if (debug) SHOWIMG(rescale((Image<float>)imgMask, 512, 512));

                        LINFO("Mask %ix%i", imgMask.getWidth(), imgMask.getHeight());
                        Image<float> distMask = chamfer34(imgMask, (byte)255);
                        Image<float> objMask = binaryReverse(distMask, 255.0F);

                        //normalize mask from 0 to 1
                        inplaceNormalize(objMask, 0.0F, 1.0F);
                        inplaceNormalize(distMask, 0.0F, 1.0F);

                        if (debug) SHOWIMG(rescale((Image<float>)objMask, 512, 512));
                        if (debug) SHOWIMG(rescale((Image<float>)distMask, 512, 512));

                        //resize the saliency map to the orig img size
                        SMap = rescale(SMap, imgMask.getDims());

                        float objMin, objMax, objSum, objArea;
                        getMaskedMinMaxSumArea(SMap, objMask, objMin, objMax, objSum, objArea);

                        float distMin, distMax, distSum, distArea;
                        getMaskedMinMaxSumArea(SMap, distMask, distMin, distMax, distSum, distArea);

                        printf("\"%f\",\"%f\",\"%f\",\"%f\",\"%f\",\"%f\",\"%f\",\"%f\"",
                        objMax, objMin, objSum, objArea,
                        distMax, distMin, distSum, distArea);
                        printf("\n");
                        */
                } else {
                        printf("##%s has no objects \n", sceneFile.c_str());
                }
        }

}

Point2D<int> evolveBrain(Image<PixRGB<byte> > &img, Image<float> &SMap, float *interestLevel,
                nub::soft_ref<SimEventQueue> seq)
{

        nub::ref<StdBrain>  brain = dynCastWeak<StdBrain>(mgr->subComponent("Brain"));


        *interestLevel = 0.0F;
        if (mgr->started() && img.initialized()){       //give the image to the brain


                SimTime end_time = seq->now() + SimTime::MSECS(3.0);
                while (seq->now() < end_time)
                {
                        brain->evolve(*seq); //evolve the brain

                        // Any new WTA winner?
                        if (SeC<SimEventWTAwinner> e = seq->check<SimEventWTAwinner>(brain.get()))
                        {
                                const Point2D<int> winner = e->winner().p;
                                const float winV = e->winner().sv;
                                *interestLevel = (winV * 1000.0f) * 1;
                                LINFO("##### Winner (%d,%d) at %fms : %.4f #####\n",
                                                winner.i, winner.j, seq->now().msecs(), winV * 1000.0f);

                                //get the saliency map output
                                if (debug)
                                {
                                        if (SeC<SimEventSaliencyMapOutput> smo =
                                                        seq->check<SimEventSaliencyMapOutput>(brain.get(), SEQ_ANY))
                                        {
                                                //Image<float> img = smo->sm();
                                                //SHOWIMG(img);
                                                //SHOWIMG(rescale(img, img.getWidth()*16, img.getHeight()*16));
                                        }
                                }
                                seq->evolve();

                                return winner;
                        }
                        seq->evolve();
                }
        }

        return Point2D<int>();

}

void biasVC(ComplexChannel &vc, Image<float> &mask)
{
        //Set mean and sigma to bias submap
        MaskBiaser mb(mask, true);
        vc.accept(mb);

        setSubmapAlgorithmBiased(vc);
}

