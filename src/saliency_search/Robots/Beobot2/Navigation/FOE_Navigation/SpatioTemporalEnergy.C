/*!@file Robots/Beobot2/Navigation/FOE_Navigation/SpatioTemporalEnergy.C 
  detect motion in an image stream   */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/SpatioTemporalEnergy.C  $
// $Id: $
//

#include "Robots/Beobot2/Navigation/FOE_Navigation/SpatioTemporalEnergy.H"

// ######################################################################
// ##### SpatioTemporalEnergyPyrBuilder Functions:
// ######################################################################

//debuging
//#define DEBUG_SpatioTemporalEnergy

#ifdef DEBUG_SpatioTemporalEnergy
#include "GUI/DebugWin.H"
#endif

#include "Raster/Raster.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include "Image/Kernels.H"

// ######################################################################
template <class T>
SpatioTemporalEnergyPyrBuilder<T>::SpatioTemporalEnergyPyrBuilder
(const PyramidType typ,
 const float gabor_theta,
 const float speed,
 const uint depth,
 const int timeDomainSize) :
  PyrBuilder<T>(),
  itsPtype(typ),
  itsDepth(depth),
  itsGaborAngle(gabor_theta),
  itsSpeed(speed),
  itsTimeDomainSize(timeDomainSize)
{
  ASSERT(itsTimeDomainSize == 3 || itsTimeDomainSize == 5);

  setSpatioTemporalFilters();
  itsSpatioTemporalEnergyOptimalShift.reset(depth);
}

// ######################################################################
template <class T>
void SpatioTemporalEnergyPyrBuilder<T>::setSpatioTemporalFilters()
{
  float angle = 90.0; //  NOTE: MAKE THIS 0.0 for the Aperture Problem solution
  float filter_period = 2.5; //3.75;
  float elongation = 1.0;  
  uint size = 5;

  double major_stddev = filter_period / 3.0;
  double minor_stddev = major_stddev * elongation;

  itsSpatTemp0filter  = gaborFilter3(major_stddev, minor_stddev,
                                     filter_period, 0.0f, angle, size);
  itsSpatTemp90filter = gaborFilter3(major_stddev, minor_stddev,
                                     filter_period, 90.0f, angle, size);

  LDEBUG("angle = %.2f, period = %.2f pix, size = %dx%d pix",
        angle, filter_period, 
        itsSpatTemp0filter.getWidth(), itsSpatTemp0filter.getHeight());

  
//   if(itsGaborAngle == 0.0)
//     {
//       for(uint y = 0; y < size; y++)
//         {
//           for(uint x = 0; x < size; x++)           
//             LINFO("%7.3f ", itsSpatTemp0filter.getVal(x,y));//*-128 + 127);
//           LINFO("\n");
//         }
//       LINFO("\n");
   
//       for(uint y = 0; y < size; y++)
//         {
//           for(uint x = 0; x < size; x++)           
//             LINFO("%7.3f ", itsSpatTemp90filter.getVal(x,y));//*128 + 127);
//           LINFO("\n");
//         }
//       LINFO("\n");   
//       if(itsWin.is_invalid())
//         {
//           std::string ntext(sformat("t=%5.2f", itsGaborAngle));
//           itsWin.reset(new XWinManaged(Dims(300,300), 0,0, ntext.c_str()));
//         }
//       else itsWin->setDims(Dims(300, 300));

//       float mn1,mx1,mn2,mx2;
//       getMinMax(itsSpatTemp0filter, mn1, mx1);
//       LDEBUG("g0  min: %f, max: %f", mn1, mx1);
//       getMinMax(itsSpatTemp90filter, mn2, mx2);
//       LDEBUG("g90 min: %f, max: %f", mn2, mx2);
      
//       Image<float> disp0 (300,300,ZEROS);
//       Image<float> disp90(300,300,ZEROS);

//       disp0  = (disp0 +1.0)*mx1;
//       disp90 = (disp90+1.0)*mx2;
//       for(int i = 0; i < 5; i++)
//         {
//           std::string ntexth(sformat("%d", i-2));
//           writeText(disp0,  Point2D<int>(70+40*i-10,25), ntexth.c_str(),mn1,mx1);
//           writeText(disp90, Point2D<int>(70+40*i-10,25), ntexth.c_str(),mn2,mx2);
//         }
//       std::string ntexthl(sformat("dx"));
//       writeText(disp0,  Point2D<int>(250,25), ntexthl.c_str(),mn1,mx1);
//       writeText(disp90, Point2D<int>(250,25), ntexthl.c_str(),mn2,mx2);

//       for(int i = 0; i < 5; i++)
//         {
//           std::string ntextv(sformat("%d", i));
//           writeText(disp0,  Point2D<int>(25, 70+40*i-10), ntextv.c_str(),mn1,mx1);
//           writeText(disp90, Point2D<int>(25, 70+40*i-10), ntextv.c_str(),mn2,mx2);
//         }
//       std::string ntextvl(sformat("dt"));
//       writeText(disp0,  Point2D<int>(25,250), ntextvl.c_str(),mn1,mx1);
//       writeText(disp90, Point2D<int>(25,250), ntextvl.c_str(),mn2,mx2);

//       inplacePaste(disp0,  zoomXY(itsSpatTemp0filter , 40), Point2D<int>(50,50));
//       inplacePaste(disp90, zoomXY(itsSpatTemp90filter , 40), Point2D<int>(50,50));
//       inplaceNormalize(disp0,  0.0F, 255.0F);
//       inplaceNormalize(disp90, 0.0F, 255.0F);
//       Raster::WriteRGB(disp0, "disp0.ppm");
//       Raster::WriteRGB(disp90,"disp90.ppm");
//       //itsWin->drawImage(disp0, 0,0); Raster::waitForKey();
//       //itsWin->drawImage(disp90,0,0); Raster::waitForKey();


//       PixRGB<byte> bl(0,  0,  0);
//       PixRGB<byte> wh(255,255,255);
//       uint wST = 100 + 19*40;
//       uint hST = 100 + 11*40;
//       Image<PixRGB<byte> > dispST(wST, hST,ZEROS);
//       dispST = (dispST+ wh);
//       std::string ntexth(sformat("dt"));
      
//       // 1px/fr
//       uint ri = 50 + 40 * 7; uint rj = 50 + 40*9; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255,  0,  0));
//       ri = 50 + 40 * 8; rj = 50 + 40*8; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255,  0,  0));
//       ri = 50 + 40 * 10; rj = 50 + 40*6; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255,  0,  0));
//       ri = 50 + 40 * 11; rj = 50 + 40*5; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255,  0,  0));

//       //2 px/fr
//       ri = 50 + 40 * 7; rj = 50 + 40*8; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(0,  255,  0));
//       ri = 50 + 40 * 5; rj = 50 + 40*9; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(0,  255,  0));
//       ri = 50 + 40 * 11; rj = 50 + 40*6; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(0,  255,  0));
//       ri = 50 + 40 * 13; rj = 50 + 40*5; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(0,  255,  0));

//       // 4px/fr
//       ri = 50 + 40 * 5; rj = 50 + 40*8; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(0,  0,   255));
//       ri = 50 + 40 * 1; rj = 50 + 40*9; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(0,  0,   255));
//       ri = 50 + 40 * 13; rj = 50 + 40*6; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(0,  0,   255));
//       ri = 50 + 40 * 17; rj = 50 + 40*5; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(0,  0,   255));

//       // 1/2 px/fr
//       ri = 50 + 40 * 11; rj = 50 + 40*1; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255, 255,  0));
//       ri = 50 + 40 * 10; rj = 50 + 40*3; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255, 255,  0));
//       ri = 50 + 40 * 9; rj = 50 + 40*5; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255, 255,  0));
//       ri = 50 + 40 * 8; rj = 50 + 40*7; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255, 255,  0));
//       ri = 50 + 40 * 7; rj = 50 + 40*9; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255, 255,  0));

//       // 1/4 px/fr
// //       ri = 50 + 40 * 11; rj = 50 + 40*1; 
// //       drawFilledRect
// //         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255, 255,  0));
// //       ri = 50 + 40 * 10; rj = 50 + 40*3; 
// //       drawFilledRect
// //         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255, 255,  0));
//       ri = 50 + 40 * 9; rj = 50 + 40*1; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255, 0,  255));
//       ri = 50 + 40 * 8; rj = 50 + 40*5; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255, 0, 255));
//       ri = 50 + 40 * 7; rj = 50 + 40*9; 
//       drawFilledRect
//         (dispST, Rectangle::tlbrI(rj, ri, rj+40-1, ri+40-1), PixRGB<byte>(255, 0, 255));




//       uint ri1 = 50 + 40 * 7 +20; uint rj1 = 50 + 40*9+20; 
//       uint ri2 = 50 + 40 * 11+20; uint rj2 = 50 + 40*5+20; 
//       drawLine(dispST, Point2D<int>(ri1, rj1), Point2D<int>(ri2, rj2), bl);      
//       ri = 50 + 40 * 9; rj = 50 + 40*7; 
//       drawLine(dispST, Point2D<int>(ri, rj), Point2D<int>(ri+40, rj+40), bl);
//       drawLine(dispST, Point2D<int>(ri, rj+40), Point2D<int>(ri+40, rj), bl);      
//       ri1 = 50 + 40 * 5 +20; rj1 = 50 + 40*9+20; 
//       ri2 = 50 + 40 * 13+20; rj2 = 50 + 40*5+20; 
//       drawLine(dispST, Point2D<int>(ri1, rj1), Point2D<int>(ri2, rj2), bl);
//       ri1 = 50 + 40 * 1 +20; rj1 = 50 + 40*9+20; 
//       ri2 = 50 + 40 * 17+20; rj2 = 50 + 40*5+20; 
//       drawLine(dispST, Point2D<int>(ri1, rj1), Point2D<int>(ri2, rj2), bl);
//       ri1 = 50 + 40 * 11 +20; rj1 = 50 + 40*1 +20; 
//       ri2 = 50 + 40 * 7  +20; rj2 = 50 + 40*9 +20; 
//       drawLine(dispST, Point2D<int>(ri1, rj1), Point2D<int>(ri2, rj2), bl);
//       ri1 = 50 + 40 * 11 +20; rj1 = 50 + 40*-7 +20; 
//       ri2 = 50 + 40 * 7  +20; rj2 = 50 + 40*9 +20; 
//       drawLine(dispST, Point2D<int>(ri1, rj1), Point2D<int>(ri2, rj2), bl);



//       writeText(dispST,  Point2D<int>(25, 25), ntexth.c_str(), bl, wh);
//       for(int i = 0; i < 20; i++)
//         {
//           drawLine(dispST, 
//                    Point2D<int>(50+i*40, 50), 
//                    Point2D<int>(50+i*40, 50+10*40), bl);
//           std::string ntext(sformat("%2d", i-8));
//           if(i != 19) 
//             writeText(dispST,  Point2D<int>(70+40*i-10, 50+10*40+5), ntext.c_str(), bl, wh);
//         }

//       std::string ntextv(sformat("dx"));
//       writeText(dispST,  Point2D<int>(70+40*19-10, 50+10*40+5), ntextv.c_str(), bl, wh);
//       for(int j = 0; j < 11; j++)
//         {
//           drawLine(dispST, 
//                    Point2D<int>(50,      50+j*40), 
//                    Point2D<int>(50+19*40,50+j*40), bl);

//           std::string ntext(sformat("%2d", j-9));
//           if(j != 10) 
//             writeText(dispST,  Point2D<int>(25, 70+40*j-10), ntext.c_str(), bl, wh);
//         }

//       //inplaceNormalize(dispST,  0.0F, 255.0F);
//       Raster::WriteRGB(dispST, "ST.ppm");
//       itsWin->setDims(Dims(wST, hST));
//       //itsWin->drawImage(dispST,0,0); Raster::waitForKey();


//       uint wRF = 100 + 28*20;
//       uint hRF = 100 + 28*20;
//       Image<PixRGB<byte> > dispRF (wRF,hRF,ZEROS);
//       dispRF  = (dispRF + PixRGB<byte>(255,255,255));

//       ri = 50 + 20 * 2*4; rj = 50 + 20*2*4; 
//       drawFilledRect
//         (dispRF, Rectangle::tlbrI(rj, ri, rj+240-1, ri+240-1), PixRGB<byte>(255, 255, 0));
//       ri = 50 + 20 * 3*4; rj = 50 + 20*3*4; 
//       drawFilledRect
//         (dispRF, Rectangle::tlbrI(rj, ri, rj+80-1, ri+80-1), PixRGB<byte>(255, 0, 0));


//       for(int i = 0; i < 29; i++)
//         {
//           drawLine(dispRF, 
//                    Point2D<int>(50+i*20, 50), 
//                    Point2D<int>(50+i*20, 50+28*20), bl, 1);
//         }

//       for(int j = 0; j < 29; j++)
//         {
//           drawLine(dispRF, 
//                    Point2D<int>(50,      50+j*20), 
//                    Point2D<int>(50+28*20,50+j*20), bl, 1);
//         }

//       for(int i = 0; i < 29; i+=2)
//         {
//           drawLine(dispRF, 
//                    Point2D<int>(50+i*20, 50), 
//                    Point2D<int>(50+i*20, 50+28*20), bl, 2);
//         }

//       for(int j = 0; j < 29; j+=2)
//         {
//           drawLine(dispRF, 
//                    Point2D<int>(50,      50+j*20), 
//                    Point2D<int>(50+28*20,50+j*20), bl, 2);
//         }

//       for(int i = 0; i < 29; i+=4)
//         {
//           drawLine(dispRF, 
//                    Point2D<int>(50+i*20, 50), 
//                    Point2D<int>(50+i*20, 50+28*20), bl, 3);
//         }

//       for(int j = 0; j < 29; j+=4)
//         {
//           drawLine(dispRF, 
//                    Point2D<int>(50,      50+j*20), 
//                    Point2D<int>(50+28*20,50+j*20), bl, 3);
//         }

//       Raster::WriteRGB(dispRF,"RF.ppm");
//       itsWin->setDims(Dims(wRF, hRF));
//       itsWin->drawImage(dispRF,0,0); Raster::waitForKey();
//     }  

  // old filters with size increase detection
  itsSpatTemp2filter = Image<float>(5,5,NO_INIT);
  itsSpatTemp2filter.setVal(0,0, 0.01);
  itsSpatTemp2filter.setVal(0,1, 0.05);
  itsSpatTemp2filter.setVal(0,2, 0.40);
  itsSpatTemp2filter.setVal(0,3, 0.05);
  itsSpatTemp2filter.setVal(0,4, 0.01);

  itsSpatTemp2filter.setVal(1,0, 0.01);
  itsSpatTemp2filter.setVal(1,1, 0.10);
  itsSpatTemp2filter.setVal(1,2, 0.40);
  itsSpatTemp2filter.setVal(1,3, 0.10);
  itsSpatTemp2filter.setVal(1,4, 0.01);
  
  itsSpatTemp2filter.setVal(2,0, 0.03);
  itsSpatTemp2filter.setVal(2,1, 0.20);
  itsSpatTemp2filter.setVal(2,2, 0.40);
  itsSpatTemp2filter.setVal(2,3, 0.20);
  itsSpatTemp2filter.setVal(2,4, 0.03);

  itsSpatTemp2filter.setVal(3,0, 0.05);
  itsSpatTemp2filter.setVal(3,1, 0.25);
  itsSpatTemp2filter.setVal(3,2, 0.40);
  itsSpatTemp2filter.setVal(3,3, 0.25);
  itsSpatTemp2filter.setVal(3,4, 0.05);

  itsSpatTemp2filter.setVal(4,0, 0.10);
  itsSpatTemp2filter.setVal(4,1, 0.30);
  itsSpatTemp2filter.setVal(4,2, 0.40);
  itsSpatTemp2filter.setVal(4,3, 0.30);
  itsSpatTemp2filter.setVal(4,4, 0.10);
}

// ######################################################################
template <class T>
ImageSet<float> SpatioTemporalEnergyPyrBuilder<T>::getOptimalShift()
{
  return itsSpatioTemporalEnergyOptimalShift;
}
// ######################################################################
template <class T>
ImageSet<float> SpatioTemporalEnergyPyrBuilder<T>::getSpatioTemporalEnergy()
{
  return itsSpatioTemporalEnergy;
}

// ######################################################################
template <class T>
ImageSet<T> SpatioTemporalEnergyPyrBuilder<T>::build
(const Image<T>& image,
 const int firstlevel,
 const int depth,
 PyramidCache<T>* cache)
{
  // WAS: Gaussian5
//   itsMotionPyrs.reset
//     (new SpatioTemporalEnergyPyrBuilder<byte>(Oriented5, 0.0f, 10.0f, itsPyrLevel));
  
  // create a pyramid with the input image
  //ImageSet<T> pyr = buildPyrGeneric(image, 0, itsDepth, Gaussian5,
  //                                  itsGaborAngle);

  //ImageSet<float> pyr =
  //  buildPyrGabor(image, 0, itsDepth, itsGaborAngle+90.0, 3.75, 1.0, 7);

  ImageSet<float> pyr =
    buildPyrGabor(image, 0, itsDepth, itsGaborAngle+90.0, 2.50, 1.0, 5);

  //DEBUG
  //print(pyr, si, ei, sj, ej, false);
  //display(pyr);

  imgPyrQ.push_back(pyr); //build the time domain

  if (imgPyrQ.size() > itsTimeDomainSize)        //limit the image time queue
    imgPyrQ.pop_front();

  // if we have too few imgs in the Q, just return an empty image:
  if (imgPyrQ.size() < itsTimeDomainSize)
    {
      LDEBUG("not enough images yet: %" ZU , imgPyrQ.size());
      return ImageSet<float>();
    }

  const int pdepth = imgPyrQ[0].size();
  ImageSet<float> result(pdepth);

  // compute the motion detection at each location
  for (int scale = 0; scale < pdepth; scale++)
    result[scale] = buildMotion(scale);

  itsSpatioTemporalEnergy = result;

  return result;
}

// // ######################################################################
// // get the motion by detecting edge orientations in the space and time
// template <class T>
// Image<float> SpatioTemporalEnergyPyrBuilder<T>::buildMotion(int scale)
// {
//   // if we have too few imgs in the Q, just return an empty image:
//   uint currQueueSize = imgPyrQ.size();

//   // go through each spatial shift 
//   bool motionComputed = false;
//   std::vector<Image<float> > results;
//   std::vector<float> shifts;

//   //LINFO("num of spatial shifts: %d", itsSpatialShift);

//   for(uint i = 0; i < itsSpatialShift; i++)
//     {
//       uint neededSize = itsTimeDomainSize;
//       if (neededSize > currQueueSize)
//         {
//           LINFO("Not enough images: %d < %d for spatial shift %d", 
//                 currQueueSize, neededSize, i);
//         }
//       else
//         {
//           uint sshift = uint(pow(2.0, i));
//           results.push_back(buildMotion(scale, sshift, 1));
//           shifts.push_back(float(sshift*pow(2.0, scale)));
//           motionComputed = true;
//         }
//     }

//   // FIX: NOTES maybe nested loop later on these two
//   // go through each temporal shift
//   for(uint j = 0; j < itsTemporalShift; j++)
//     {
//       //uint            neededSize =  9; // speed: 1/2 pix/fr
//       //if(j == 1)      neededSize = 13; // speed: 1/3 pix/fr
//       //else if(j == 2) neededSize = 17; // speed: 1/4 pix/fr
//       uint neededSize = 1 + 4*(j+2);

//       if (neededSize > currQueueSize)
//         {
//           LINFO("Not enough images: %d < %d for temporal shift %d", 
//                 currQueueSize, neededSize, j);
//         }
//       else
//         {
//           float sshift = 1.0/(j+2.0);
//           results.push_back(buildMotion(scale, 1, j+1));
//           shifts.push_back(sshift*pow(2.0, scale));
//           motionComputed = true;
//         }
//     }

//   // find max among all the spatial and temporal shifts
//   // --> basically max among all the shifting values
//   if(motionComputed)
//     {
//       uint width  = results[0].getWidth();
//       uint height = results[0].getHeight();
//       Image<float> result(width, height, ZEROS);

//       itsSpatioTemporalEnergyOptimalShift[scale] = 
//         Image<float>(width, height, ZEROS);

//       typename Image<float>::iterator 
//         resultT = result.beginw();

//       typename Image<float>::iterator 
//         shiftT = itsSpatioTemporalEnergyOptimalShift[scale].beginw(); 

//       typename Image<float>::iterator resT[results.size()];
//       for (unsigned int i=0; i < results.size(); i++)
//           resT[i] = results[i].beginw();

//       for(uint i = 0; i < width; i++)
//         {
//           for(uint j = 0; j < height; j++)
//             {
//               float max = 0.0; float optShift = 0.0;
//               uint size = results.size();

//               for(uint r = 0; r < size; r++)
//                 {
//                   float val = *resT[r]++; 
//                   if(max < val) 
//                     {
//                       max = val;
//                       optShift = shifts[r];
//                     }
//                 }
//               *resultT++ = max;
//               *shiftT++  = optShift;
//             }
//         }

// //       if((scale == 1))
// //         {
// //           uint div = uint(pow(2.0,scale));
// //           LINFO("COMB: Ang: %f sc: %d", itsGaborAngle, scale);      
// //           print(result, 166/div, 196/div, 55/div, 75/div, false);
// //         }


//       return result;
//     }
//   else
//     return Image<float>();
// }

// ######################################################################
template <class T>
Image<float> SpatioTemporalEnergyPyrBuilder<T>::buildMotion(int scale)
{
  int width  = imgPyrQ[0][scale].getWidth();
  int height = imgPyrQ[0][scale].getHeight();

  // build the time domain pointers
  /*typename Image<float>::iterator imgT[itsTimeDomainSize];
  for (unsigned int i=0; i < itsTimeDomainSize; i++)
    {
      imgT[i] = imgPyrQ[i][scale].beginw();
    }*/

  // to make the operation look nice and readable
  // the parenthesis around the x are needed
  // otherwise a bug happens when using PixH(0,i-1)
#define Pix(t,x,y) (*(imgT[t] + (y*width)+(x)))

  float cosAng = cos(itsGaborAngle * M_PI/180.0); 
  float sinAng = sin(itsGaborAngle * M_PI/180.0); 

//   LINFO("Bef  %f %f ss:%d ts:%d scale: %d ang: %f", 
//         cosAng,sinAng, spatialShift, temporalShift, scale, itsGaborAngle);

//   if(cosAng >  .1 && cosAng <  0.9) cosAng =  1.0F;
//   if(sinAng >  .1 && sinAng <  0.9) sinAng =  1.0F;
//   if(cosAng < -.1 && cosAng > -0.9) cosAng = -1.0F;
//   if(sinAng < -.1 && sinAng > -0.9) sinAng = -1.0F;

  // NOTE: FIX THIS UNFORTUNATE SITUATION
  // find the limit on the XY plane of the spatiotemporal space
  uint dx = 2*itsSpeed + 2; // 2; //uint(spatialShift * cosAng * (itsTimeDomainSize/2));
  uint dy = 2*itsSpeed + 2; // 2; //uint(spatialShift * sinAng * (itsTimeDomainSize/2));
//   LINFO("%f c,s(%10.3f, %10.3f) dx: %7d, dy: %7d", 
//         itsGaborAngle, cosAng, sinAng, dx, dy);

  Image<float> motion(width, height, ZEROS); // motion place holder

  // go through all the active area
  for (uint j = dy; j < (height-dy); j++)
    {
      typename Image<float>::iterator mot = 
        motion.beginw() + (j*width) + dx;
        
      for (uint i = dx; i < (width-dx); i++)
        {
          //LINFO("[%d %d]",i ,j, );

          // calculate the firing for displacement & size increase
          // based on timedomainsize
          switch(itsTimeDomainSize)
            {
              // calculate values from 3 most recent frames 
            case 3: // do something simpler later
              break;
              
              // 5 values from 5 most recent frames 
            case 5:
              {
                uint speed  = 1;
                uint tshift = 1;
                if(itsSpeed >= 1.0) speed = uint(itsSpeed);
                else
                  {
                    // FIXXX: add cases for 1/2, 1/3, 1/4
                  }

                float val = 
                  getSpatioTemporalVal
                  (cosAng, sinAng, scale, i, j, speed, tshift);
                //motion.setVal(i,j,val);
                *mot = val; mot++; // saves 3ms compared to setVal
              }
              break;

            default: LFATAL("invalid itsTimeDomainSize: %d", itsTimeDomainSize);
              break;
            }
        }
    }

  // DEBUG
  //  uint div = uint(pow(2.0, scale));
//   if((itsGaborAngle == 0.0 || itsGaborAngle == 180.0) &&
//      (scale == 0) && (spatialShift == 1) && (temporalShift == 1))
//     {
//   if((scale == 0))// && (spatialShift == 1) && (temporalShift == 1))
//     {
//       LINFO("Ang: %f sc: %d, ss: %d ts: %d", 
//             itsGaborAngle, scale, spatialShift, temporalShift);      
//       print(motion, 158, 169, 55, 75, false);
//     }
  //print(motion, 165/div, 175/div, 45/div, 75/div, true);
  //print(motion, 175/div, 185/div, 45/div, 75/div, true);
  //print(motion, 185/div, 195/div, 45/div, 75/div, true);

  return motion;
}

// ######################################################################
template <class T> 
float SpatioTemporalEnergyPyrBuilder<T>::getSpatioTemporalVal
(float cosAng, float sinAng, uint scale, uint i, uint j,
 uint spatialShift, uint temporalShift)
{
  
  typename Image<float>::iterator f0 =
    itsSpatTemp0filter.beginw();
  typename Image<float>::iterator f90 =
    itsSpatTemp90filter.beginw();

//   typename Image<float>::iterator f2 =
//     itsSpatTemp2filter.beginw();

  // to make the operation look nice and readable
  // the parenthesis around the x are needed
  // otherwise a bug happens when using PixH(0,i-1)
#define v0(x,y)  (*(f0  + (y*itsTimeDomainSize)+(x)))
#define v90(x,y) (*(f90 + (y*itsTimeDomainSize)+(x)))
  //#define v2(x,y)  (*(f2  + (y*itsTimeDomainSize)+(x)))

  float tsum0 = 0.0; float tsum90 = 0.0;
  std::vector<float> sum0(5);
  std::vector<float> sum90(5);

  std::vector<float> spatShift(5);
  spatShift[0] = -2.0 * spatialShift;
  spatShift[1] = -1.0 * spatialShift;
  spatShift[2] =  0.0;
  spatShift[3] =  1.0 * spatialShift;
  spatShift[4] =  2.0 * spatialShift;

  // check temporal shifting  
  uint tempLimit = 1 + temporalShift*(itsTimeDomainSize-1); 
  uint ft = 0; 
  for(uint t = 0; t < tempLimit; t+= temporalShift)
    {      
      sum0[t]  = getFilterResponseVal
        (spatShift[ft], cosAng, sinAng, t, scale,
         i, j, v0(ft,0), v0(ft,1), v0(ft,2), v0(ft,3), v0(ft,4));
      
      sum90[t] = getFilterResponseVal
        (spatShift[ft], cosAng, sinAng, t, scale,
         i, j, v90(ft,0), v90(ft,1), v90(ft,2), v90(ft,3), v90(ft,4));

      tsum0  += sum0[t];
      tsum90 += sum90[t];
//       sum[t] = getSpatTempVal
//         (shift[t], cosAng, sinAng, t, scale,
//          i, j, v2(t,0), v2(t,1), v2(t,2), v2(t,3), v2(t,4));      

//        LINFO("s: %f (%f %f) t:%d s:%d [%d %d] %f %f %f %f %f", 
//              shift[t], cosAng, sinAng, t, scale,
//              i, j, v2(t,0), v2(t,1), v2(t,2), v2(t,3), v2(t,4));    

      //sum2 += sum[t];

//       LINFO("%10.3f %10.3f %10.3f %10.3f %10.3f ", 
//             v0(ft,0), v0(ft,1), v0(ft,2), v0(ft,3), v0(ft,4));

      ft++;
    }

  float sum = tsum0*tsum0 + tsum90*tsum90;
  
//   // grab values from frame t - 4
//   float v0 = getSpatTempVal
//     (-2.0, cosAng, sinAng, 0, scale,
//      i, j, 0.01, 0.05, 0.40, 0.05, 0.01);

//   // grab values from frame t - 3
//   float v1 = getSpatTempVal
//     (-1.0, cosAng, sinAng, 1, scale,
//      i, j, 0.01, 0.10, 0.40, 0.10, 0.01);

//   // grab values from frame t - 2
//   float v2 = getSpatTempVal
//     (0.0, cosAng, sinAng, 2, scale,
//      i, j, 0.03, 0.20, 0.40, 0.20, 0.03);

//   // grab values from frame t - 1
//   float v3 = getSpatTempVal
//     (1.0, cosAng, sinAng, 3, scale,
//      i, j, 0.05, 0.25, 0.40, 0.25, 0.05);

//   // grab values from frame t 
//   float v4 = getSpatTempVal
//     (2.0, cosAng, sinAng, 4, scale,
//      i, j, 0.10, 0.30, 0.40, 0.30, 0.10);

//   if(i > 150 && i < 170 && j ==60)
//     {
//       LINFO(" 0(%d %d) %10.3f,%10.3f,%10.3f,%10.3f,%10.3f = %10.3f", 
//             i, j, sum0[0], sum0[1], sum0[2], sum0[3], sum0[4], tsum0);
//       LINFO("90(%d %d) %10.3f,%10.3f,%10.3f,%10.3f,%10.3f = %10.3f", 
//             i, j, sum90[0], sum90[1], sum90[2], sum90[3], sum90[4], tsum90);

//       LINFO("%f^2 + %f^2  = %f + %f = %f ", 
//             tsum0, tsum90, tsum0*tsum0, tsum90*tsum90, sum);
//       //LINFO("(%d %d) %10.3f,%10.3f,%10.3f,%10.3f,%10.3f = %10.3f", 
//       //      i, j, v0, v1, v2, v3, v4, v0+v1+v2+v3+v4);
//     }  
  
  //   return  v0 + v1 + v2 + v3 + v4;
  return  sum;
}

// ######################################################################
template <class T> 
float SpatioTemporalEnergyPyrBuilder<T>::getFilterResponseVal
(float dshift, float cosAng, float sinAng, uint time, uint scale,
 uint i, uint j,
 float wm2, float wm1, float w, float wp1, float wp2)
{

  uint width = imgPyrQ[time][scale].getWidth();

  // to make the operation look nice and readable
  // the parenthesis around the x are needed
  // otherwise a bug happens when using Px(0,i-1)
  //#define Px(t,x,y) (*(imgT[t] + ((y)*width)+(x))
#define Px(x,y) (*(imgT + ((y)*width)+(x)))

  // build the time domain pointers
//   typename Image<float>::iterator imgT[itsTimeDomainSize];
//   for (unsigned int ind = 0; ind < itsTimeDomainSize; ind++)
//     {
//       imgT[ind] = imgPyrQ[ind][scale].beginw();
//     }

  typename Image<float>::iterator imgT = imgPyrQ[time][scale].beginw();

  float ii = i + (dshift * cosAng);
  float jj = j + (dshift * sinAng);

  float pi = cosAng;  // old: sinAng
  float pj = sinAng;  // old: cosAng

//   float iim2 = ii - 2*pi;
//   float jjm2 = jj - 2*pj;

//   float iim1 = ii - pi;
//   float jjm1 = jj - pj;

//   float iip1 = ii + pi;
//   float jjp1 = jj + pj;

//   float iip2 = ii + 2*pi;
//   float jjp2 = jj + 2*pj;

//   float v1 = wm2 * Px(time, int(iim2),int(jjm2));
//   float v2 = wm1 * Px(time, int(iim1),int(jjm1));
//   float v3 = w   * Px(time, int(ii),  int(jj)  );
//   float v4 = wp1 * Px(time, int(iip1),int(jjp1));
//   float v5 = wp2 * Px(time, int(iip2),int(jjp2));

  float v1 = wm2 * Px(int(ii - 2*pi),int(jj - 2*pj));
  float v2 = wm1 * Px(int(ii - pi),  int(jj - pj));
  float v3 = w   * Px(int(ii),       int(jj)  );
  float v4 = wp1 * Px(int(ii + pi),  int(jj + pj));
  float v5 = wp2 * Px(int(ii + 2*pi),int(jj + 2*pj));

  float val  = v1 + v2 + v3 + v4 + v5;

//    if(i > 158 && i < 170 && j ==60 && scale == 0 && 
//       pi != 0.0 && pj != 0.0)
//    {
//      LINFO("[%d] i,  j:  %d %d", time, i,  j);
//      LINFO("ii, jj: %f %f", ii, jj);
//      LINFO("pi, pj: %f %f", pi, pj);

// //     LINFO("%d %d: %f x",           int(iim2),int(jjm2), wm2);
// //     LINFO("%d %d: %f x",           int(iim1),int(jjm1), wm1);
// //     LINFO("%d %d: %f x",           int(ii),  int(jj), w);
// //     LINFO("%d %d: %f x",           int(iip1),int(jjp1), wp1);
// //     LINFO("%d %d: %f x",           int(iip2),int(jjp2), wp2);

//     LINFO("%d %d: %f x %f", int(iim2),int(jjm2), Px(int(iim2),int(jjm2)), wm2);
//     LINFO("%d %d: %f x %f", int(iim1),int(jjm1), Px(int(iim1),int(jjm1)), wm1);
//     LINFO("%d %d: %f x %f", int(ii),  int(jj),   Px(int(ii),  int(jj)  ), w);
//     LINFO("%d %d: %f x %f", int(iip1),int(jjp1), Px(int(iip1),int(jjp1)), wp1);
//     LINFO("%d %d: %f x %f", int(iip2),int(jjp2), Px(int(iip2),int(jjp2)), wp2);

//     LINFO("%f %f %f %f %f = %f", v1, v2, v3, v4, v5, val);
//    }

  return val;
  
}

// ######################################################################
template <class T>
float SpatioTemporalEnergyPyrBuilder<T>::DrawVectors
(Image<T> &img, Image<float> &motion)
{
  //TODO: should check the mag is the same size as dir

  Image<float>::const_iterator mag_ptr = motion.begin();
  Image<float>::const_iterator mag_stop = motion.end();

  int inx=0;
  int avg_i = 0;
  double avg_angle = 0;

  while (mag_ptr != mag_stop)
    {
      int y = inx/motion.getWidth();
      int x = inx - (y*motion.getWidth());

      if (*mag_ptr != 0) {
        avg_i++;
        //avg_angle += (*mag_ptr+M_PI/2);
        avg_angle += (*mag_ptr);

        int scale_x = x * (img.getWidth()/motion.getWidth());
        int scale_y = y * (img.getHeight()/motion.getHeight());
        drawLine(img, Point2D<int>(scale_x,scale_y),
                 Point2D<int>((int)(scale_x+25*cos((*mag_ptr))),
                         (int)(scale_y-25*sin((*mag_ptr)))),
                 (T)0);
      }
      mag_ptr++;
      inx++;
    }

  if (avg_i > 0){
    int xi = img.getWidth()/2;
    int yi = img.getHeight()/2;

    drawLine(img,Point2D<int>(xi, yi),
             Point2D<int>((int)(xi+75*cos(avg_angle/avg_i)),
                     (int)(yi-75*sin(avg_angle/avg_i))),
             (T)0, 3);
    return avg_angle/avg_i;
  }

  return -999;
}

// ######################################################################
template <class T>
SpatioTemporalEnergyPyrBuilder<T>* SpatioTemporalEnergyPyrBuilder<T>::clone() const
{ return new SpatioTemporalEnergyPyrBuilder<T>(*this); }

// ######################################################################
template <class T>
void SpatioTemporalEnergyPyrBuilder<T>::reset()
{
  // imgPyrQ.clear();
}

// ######################################################################
template <class T>
void SpatioTemporalEnergyPyrBuilder<T>::print
(Image<float> image, uint si, uint ei, uint sj, uint ej, float stop)
{
  for(uint j = sj; j <= ej; j++)
    {
      for(uint i = si; i <= ei; i++)           
        //if(image.getVal(i,j) > 0.0) 
          //LINFO("[%3d %3d]: %10.3f", i,j,  image.getVal(i,j));
          LINFO("%9.3f ", image.getVal(i,j));
      LINFO("\n");
    }
  LINFO("\n");
  if(stop) Raster::waitForKey();
}

// ######################################################################
template <class T>
void SpatioTemporalEnergyPyrBuilder<T>::print
(ImageSet<float> images, uint si, uint ei, uint sj, uint ej, float stop)
{
  for(uint k = 0; k < images.size(); k++)
    {
      LINFO("level: %d", k);
      uint div = uint(pow(2.0,k));
      for(uint j = sj/div; j <= ej/div; j++)
        {
          for(uint i = si/div; i <= ei/div; i++)           
            //if(image.getVal(i,j) > 0.0) 
            //LINFO("[%3d %3d]: %10.3f", i,j,  image.getVal(i,j));
            LINFO("%9.3f ", images[k].getVal(i,j));
          LINFO("\n");
        }
      LINFO("\n");
    }

  if(stop) Raster::waitForKey();
}


// ######################################################################
// template <class T>
// void SpatioTemporalEnergyPyrBuilder<T>::print
//(Image<float> image, std::vector<uint> si, std::vector<uint> ei, 
// std::vector<uint> sj, std::vector<uint> ej, std::vector<uint> levels, float stop)
// {
//   for (uint i = 0; i < itsDepth; i++)
//     {
//       if(i == 0)
//         { 
//           LINFO("level 0");
//           for(uint y = 35; y < 85; y++)
//             {
//               for(uint x = 155; x < 168; x++)           
//                 LINFO("%7.3f ",pyr[i].getVal(x,y));
//                 //LINFO("%3d ",pyr[i].getVal(x,y));
//               LINFO("\n");
//             }
//           LINFO("\n");
//         }

//       if(i == 1)
//         {
//           LINFO("level 1");
//           for(uint y = 10; y < 50; y++)
//             {
//               for(uint x = 75; x < 88; x++)           
//                 LINFO("%7.3f ",pyr[i].getVal(x,y));
//                 //LINFO("%3d ",pyr[i].getVal(x,y));
//               LINFO("\n");
//             }
//           LINFO("\n");
//         }

//       if(i == 2)
//         {
//           LINFO("level 2");
//           for(uint y = 5; y < 25; y++)
//             {
//               for(uint x = 35; x < 48; x++)           
//                 LINFO("%7.3f ",pyr[i].getVal(x,y));
//                 //LINFO("%3d ",pyr[i].getVal(x,y));
//               LINFO("\n");
//             }
//           LINFO("\n");
//         }

//       if(i == 3)
//         {
//           LINFO("level 3");
//           for(uint y = 0; y < 20; y++)
//             {
//               for(uint x = 13; x < 27; x++)           
//                 LINFO("%7.3f ",pyr[i].getVal(x,y));
//                 //LINFO("%3d ",pyr[i].getVal(x,y));
//               LINFO("\n");
//             }
//           LINFO("\n");
//         }

//       if(i == 4)
//         {
//           LINFO("level 4");
//           for(uint y = 0; y < 15; y++)
//             {
//               for(uint x = 5; x < 18; x++)           
//                 LINFO("%7.3f ",pyr[i].getVal(x,y));
//               //LINFO("%3d ",pyr[i].getVal(x,y));
//               LINFO("\n");
//             }
//           LINFO("\n");
//         }
//   }
//}

// ######################################################################
template <class T>
void SpatioTemporalEnergyPyrBuilder<T>::display(Image<float> image)
{
  uint w = image.getWidth();
  uint h = image.getHeight();

  if(itsWin.is_invalid())
    {
      std::string ntext(sformat("t=%5.2f", itsGaborAngle));
      itsWin.reset(new XWinManaged(Dims(w,h), 0,0, ntext.c_str()));
    }

  itsWin->setDims(Dims(w, h));
  LINFO("Dims: (%d,%d)", w, h);

  float mn,mx; getMinMax(image, mn,mx);
  LINFO("min: %f, max: %f", mn, mx);
  itsWin->drawImage(image,0,0); 
  Raster::waitForKey();
}

// ######################################################################
template <class T>
void SpatioTemporalEnergyPyrBuilder<T>::display(ImageSet<float> images)
{
  uint w         = images[0].getWidth();
  uint h         = images[0].getHeight();

  if(itsWin.is_invalid())
    {
      std::string ntext(sformat("t=%5.2f", itsGaborAngle));
      itsWin.reset(new XWinManaged(Dims(w,h), 0,0, ntext.c_str()));
    }

  uint numLevels = images.size();
  itsWin->setDims(Dims(w, h));
  Image<float> disp(w, h, ZEROS);
  LINFO("Depth: %d Dim(%d,%d)", numLevels, w, h);  
  for (uint i = 0; i < numLevels; i++)
    {
      uint scale = pow(2.0, i);

      if(w >= uint(images[i].getWidth()  * scale) && 
         h >= uint(images[i].getHeight() * scale) )
        {
          Image<float> temp = images[i];
          float mn,mx; getMinMax(images[i], mn,mx);
          LINFO("min: %f, max: %f", mn, mx);

          inplaceNormalize(temp, 0.0F, 255.0F);
          inplacePaste(disp, zoomXY(temp, scale) , Point2D<int>(0,  0));
          itsWin->drawImage(disp,0,0); Raster::waitForKey();
        }
      else{ LINFO("Too big. Not drawn."); }
    }
}


template class SpatioTemporalEnergyPyrBuilder<byte>;
template class SpatioTemporalEnergyPyrBuilder<float>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
