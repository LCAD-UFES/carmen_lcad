/*!@file  Robots/Beobot2/Navigation/FOE_Navigation/PopulationHeadingMap.C
  Lappe&Rauschecker 1993's Population Heading Map algorithm */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/PopulationHeadingMap.C
// $Id: $
//
#ifndef ROBOTS_BEOBOT2_NAVIGATION_FOENAVIGATION_POPULATIONHEADINGMAP_C_DEFINED
#define ROBOTS_BEOBOT2_NAVIGATION_FOENAVIGATION_POPULATIONHEADINGMAP_C_DEFINED

#define PHM_NUM_DIRECTIONS                  4
#define PHM_FOE_STEP                       16
#define PHM_NUM_MSTD_POPULATION_NEURONS    20
#define PHM_NUM_MSTD_INPUT_NEURONS         30

#define PHM_MU                             .20F  

#include "Robots/Beobot2/Navigation/FOE_Navigation/PopulationHeadingMap.H"

#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/MatrixOps.H"
#include "Image/LinearAlgebra.H"
#include "Image/ShapeOps.H"

#include "Util/Timer.H"

#include <stdio.h>

// ######################################################################
PopulationHeadingMap::PopulationHeadingMap(float focalLength):
  itsFocalLength(focalLength)
{

}

// ######################################################################
void PopulationHeadingMap::setFocalLength(float focalLength)
{
  itsFocalLength = focalLength;
}

// ######################################################################
PopulationHeadingMap::~PopulationHeadingMap()
{ }

// ######################################################################
// it would be nice if this function runs fast
// so that we can reinitialize 
void PopulationHeadingMap::initialize(Dims dims)
{

  // find the orthogonal complement of C(Tj)
  //   find A: bases of C(Tj)
  //   find the nullspace of A: this is the orthogonal complement
  
  

  
  // also the matched pair neuron
  // J_(i,j,k,l)' = - J_(i,j,k,l)

}

// ######################################################################
//! return FOE from image of movement between the two input images
void PopulationHeadingMap::getFOE(Image<byte> stim1, Image<byte> stim2)
{
  itsCurrentImage = stim1;
  getFOE(stim2);
}

// ######################################################################
//! return  FOE from image of movement 
//! between this image and the previous one
void PopulationHeadingMap::getFOE(Image<byte> stim)
{
  // NOTE: Particle filtering to run multiple hypotheses

  // don't do anything if this is the first frame
  if(!itsCurrentImage.initialized())
    {
      itsCurrentImage  = stim;
      itsPreviousImage = stim;
      LINFO("NOT ENOUGH INPUT YET");
      return;
    }

  LINFO("Focal Length: %f", itsFocalLength);

  itsPreviousImage = itsCurrentImage;
  itsCurrentImage  = stim;

  // compute the optic flow for the input MT neurons
  // : spatiotemporal, Lucas&Kanade, etc.
  itsOpticalFlow =
    getCleanOpticFlow(Image<byte>(itsPreviousImage.getDims(),ZEROS));
  //getLucasKanadeOpticFlow(itsPreviousImage, itsCurrentImage);  
  itsOpticalFlowImage = 
    drawOpticFlow(itsPreviousImage, itsOpticalFlow);

  //getCleanOpticFlow3(Image<byte>(itsPreviousImage.getDims(),ZEROS));

  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(itsCurrentImage.getDims(), 
                                 0, 0, "Pop Heading Map"));
  itsWin->setDims(itsCurrentImage.getDims());
  itsWin->drawImage(itsOpticalFlowImage,0,0);
  
  LINFO("got the optic flow");
  Raster::waitForKey();


  //itsMTfeatures = itsOpticalFlow->getMTfeatures();

  // compute output for each MSTd neuron
  computePopulationHeading(); 

  // find the most likely heading

}
// ######################################################################
void PopulationHeadingMap::computePopulationHeading()
{
  itsHeadingDirectionMap = 
    Image<float>(itsCurrentImage.getWidth() /PHM_FOE_STEP, 
                 itsCurrentImage.getHeight()/PHM_FOE_STEP, ZEROS);

  uint border = 2;
  uint hdw = itsHeadingDirectionMap.getWidth();
  uint hdh = itsHeadingDirectionMap.getHeight();

  // the number of inputs that each MSTd cells received
  uint m = PHM_NUM_MSTD_INPUT_NEURONS; // can change it to 20 from 30

  // the number of neurons that each direction draw from
  uint n = PHM_NUM_MSTD_POPULATION_NEURONS;

  // compute likelihood for each direction Tj
  for(uint jx = border; jx < hdw - border; jx++)
    {
      for(uint jy = border; jy < hdh - border; jy++)
        {
          float U = 0.0;

          //Timer tim(1000000); tim.reset();

          // pick m random flows
          std::vector<uint> lind = pickRandomLocations(m);
          std::vector<Point2D<float> > points = getLocations(lind);
          std::vector<Point2D<float> > flows  = getFlowVectors(lind); 

          //uint64 t1 = tim.get();

          // compute C(T)
          Image<float> C = computeC(points, jx, jy);

          //uint64 t2 = tim.get();

          // compute orthogonal complement: c_perp 
          // using the above random points 
          Image<float> c_perp = computeOrthogonalComplement(C);
          
          //LINFO("time: %f", t1/1000.0);
          //LINFO("time: %f", (t2 -t1)/1000.0);
          //LINFO("time: %f", tim.get()/1000.0);
          //Raster::waitForKey();
          // for each MT node in the Tj direction population
          for(uint l = 0; l < n; l++)
            {
              // number of null space dimension
              uint nndims = c_perp.getWidth();

              // pick a random vector c_perp_l from c_perp
              uint rvindex = uint(nndims * float(rand())/(RAND_MAX + 1.0F));
              Image<double> c_perp_l = 
                crop(c_perp, Point2D<int>(rvindex, 0), Dims(1,2*m));

              Image<double>::iterator cpli = c_perp_l.beginw();

              // for each input MT neuron to the node
              double total = 0.0;
              for(uint i = 0; i < m; i++)
                {
                  // calculate the neuron i firing rate
                  double dx = flows[i].i;
                  double dy = flows[i].j;                  

                  total += dx*(*cpli++); //c_perp_l.getVal(0, i*2); //(*cpli++); 
                  total += dy*(*cpli++); //c_perp_l.getVal(0, i*2+1); //(*cpli++); 
                  
                  // for each direction in the location
                  //for(uint k = 0; k < PHM_NUM_DIRECTIONS; k++)
                  //  {
                      //float J_ijkl =  itsWeights[jx][jy][k].getVal(pt);
                      //float s_ik   = itsMTfeatures[k].getVal(pt);

                      //total +=  * s_ik;
                  //  }                  
                } 

              // firing rate using sigmoid function
              // do the opposite as well to get an optimal result 
              // near zero
              double ul = sigmoid(PHM_MU - total) + sigmoid(PHM_MU + total) -1.0;
              //double ul = -fabs(total);

              //LINFO("U[%d] Total: %f -> %f", l, total, ul);
              U += ul;//-pow(ul,2.0);
            }
          LINFO("j[%3d %3d]: U: %f",jx,jy, U/n);
          
          //if(jx != 2 & jy != ) U = 0.0;

          itsHeadingDirectionMap.setVal(jx,jy, U/n);
        }
    }

  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(itsCurrentImage.getDims(), 0, 0, "Pop Heading Map"));
  itsWin->setDims(itsCurrentImage.getDims());
  itsWin->drawImage(zoomXY(itsHeadingDirectionMap, PHM_FOE_STEP),0,0);
  LINFO("map");
  Raster::waitForKey();

}

// ######################################################################
double PopulationHeadingMap::sigmoid(double x) 
{
  return 1.0F/(1.0F + pow(M_E,-x));
} 

// ######################################################################
std::vector<uint> PopulationHeadingMap::pickRandomLocations(uint m)
{  
  std::vector<uint> locIndexes(m);

  std::vector<Point2D<float> > flowLocations = 
    itsOpticalFlow->getFlowLocations();
  uint numFlows = flowLocations.size();

  // make sure there is no duplicate locations
  std::vector<bool> picked(numFlows);
  for(uint i = 0; i < numFlows; i++) picked[i] = false;

  // get n random locations
  for(uint i = 0; i < m; i++)
    {      
      uint rindex = 
        uint(numFlows * float(rand())/(RAND_MAX + 1.0F));

      while(picked[rindex])
          rindex = uint(numFlows * float(rand())/(RAND_MAX + 1.0F));

      locIndexes[i] = rindex;
      picked[rindex] = true;
    }

  return locIndexes;
}

// ######################################################################
std::vector<Point2D<float> > 
PopulationHeadingMap::getLocations(std::vector<uint> lind)
{  
  uint m = lind.size();
  std::vector<Point2D<float> > locs(m);

  std::vector<Point2D<float> > flowLocations = 
    itsOpticalFlow->getFlowLocations();

  //Image<PixRGB<byte> > img = itsOpticalFlowImage;

  // get n random locations
  for(uint i = 0; i < m; i++)
    {      
      locs[i] = flowLocations[lind[i]];

      //drawDisk(img, Point2D<int>(locs[i].i, locs[i].j), 2, PixRGB<byte>(255,255,0));
    }

  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(img.getDims(), 0, 0, "Pop Heading Map"));
  // itsWin->setDims(img.getDims());
  // itsWin->drawImage(img,0,0);
  // LINFO("random points");
  // Raster::waitForKey();


  return locs;
}

// ######################################################################
std::vector<Point2D<float> > 
PopulationHeadingMap::getFlowVectors(std::vector<uint> lind)
{  
  uint m = lind.size();
  std::vector<Point2D<float> > flows(m);

  std::vector<rutz::shared_ptr<FlowVector> > fv = 
    itsOpticalFlow->getFlowVectors();

  // get n random locations
  for(uint i = 0; i < m; i++)
    {      
      flows[i] = 
        Point2D<float>
        (fv[lind[i]]->p2.i -  fv[lind[i]]->p1.i,
         fv[lind[i]]->p2.j -  fv[lind[i]]->p1.j);
    }

  return flows;
}

// ######################################################################
Image<double> 
PopulationHeadingMap::computeC
(std::vector<Point2D<float> > points, uint jx, uint jy)
{
  // NOTE_FIXXX: compute the focal length from the two initial images
  float f = itsFocalLength;
  //LINFO("Focal Length: %f", f);

  int w = itsCurrentImage.getWidth();
  int h = itsCurrentImage.getHeight();

  // create C(T)
  uint m = points.size();
  Image<double> C(m+3, 2*m, ZEROS);

  double Tx = double(jx)*PHM_FOE_STEP - double(w/2);
  double Ty = double(jy)*PHM_FOE_STEP - double(h/2);
  double Tz = f;

  double wh = w/2;
  double hh = h/2;

  // fill matrix for every point 
  for(uint i = 0; i < m; i++)
    {
      // make the center 
      double x =  points[i].i - wh;
      double y =  points[i].j - hh; 

      //LINFO("[%f %f]:{%f, %f, %f}", x,y, Tx,Ty,Tz);

      // compute A(x_i,y_i)T
      C.setVal(i, i*2,   -f*Tx + x*Tz);
      C.setVal(i, i*2+1, -f*Ty + y*Tz);

      // compute B(x_i,y_i)
      C.setVal(m,   i*2,   x*y/f      );
      C.setVal(m+1, i*2,   -f-(x*x/f) );
      C.setVal(m+2, i*2,   y          );
      C.setVal(m,   i*2+1, f + (y*y/f));
      C.setVal(m+1, i*2+1, -x*y/f     );
      C.setVal(m+2, i*2+1, -x         );
    }
  
  // for(int j = 0; j < C.getHeight(); j++)
  //   {
  //     for(int i = 0; i < C.getWidth(); i++)
  //       {
  //         printf("%9.3f ", C.getVal(i,j));
  //       }
  //     printf("\n");
  //   }
  // Raster::waitForKey();

  return C;
}

// ######################################################################
Image<double> 
PopulationHeadingMap::computeOrthogonalComplement(Image<double> C)
{
  // perform SVD on C; C = [U][S]([V]T)
  Image<double> U;
  Image<double> S;
  Image<double> V;
  svd(C, U, S, V, SVD_LAPACK, SVD_FULL);
  
  // LINFO("C[%3d %3d] =  U[%3d %3d] S[%3d %3d] V[%3d %3d]",
  //       C.getHeight(), C.getWidth(),
  //       U.getHeight(), U.getWidth(),
  //       S.getHeight(), S.getWidth(),
  //       V.getHeight(), V.getWidth() );
  
  uint m = C.getHeight();
  uint n = C.getWidth();
  uint nindex = n;
  for(uint i = 0; i < n; i++) 
    {
      double val = S.getVal(i,i);
      //LINFO("[%3d]: %f", i, val);

      // figure out how many dimension the nullspace is
      if(val < 0.00000001) nindex = i;
    }

  uint nspacedims = m - nindex;

  //LINFO("nindex: %d nspacedims: %d", nindex, nspacedims);

  Image<double> Cperp = 
    crop(U, Point2D<int>(nindex,0),Dims(nspacedims,m));

  
  //Image<double> res = matrixMult(transpose(C),Cperp);  
  //print(res,std::string("res"));
  //Raster::waitForKey();


  return Cperp;
}

// ######################################################################
void PopulationHeadingMap::print(Image<double> img, const std::string& name)
{
  LINFO("%s:  %d by %d", name.c_str(), img.getWidth(),  img.getHeight());
  for(int j = 0; j < img.getHeight(); j++)
    {
      for(int i = 0; i < img.getWidth(); i++)
        {
          printf("%13.5f ", img.getVal(i,j));
        }
      printf("\n");
    }
}
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // ROBOTS_BEOBOT2_NAVIGATION_FOENAVIGATION_POPULATIONHEADINGMAP_C_DEFINED
