/*!@file Component/RawGistEstimatorHelper.H helper function
         using available features of the image                          */
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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/RawGistEstimatorHelper.H $
// $Id: RawGistEstimatorHelper.H $

#ifndef RAWGISTESTIMATORHELPER_H_DEFINED
#define RAWGISTESTIMATORHELPER_H_DEFINED
// ######################################################################
// get subsum of an image - more efficient implementation
inline
Image<float> getSubSumGen(Image<float> img)
{
  Image<float> res(1, 21, ZEROS);

  int w = img.getWidth();  int h = img.getHeight();
  std::vector<float> tempRes(21);
  for (int i = 0; i < 21; i++) tempRes[i] = 0.0;
  std::vector<int> counts(16);
  for (int i = 0 ; i < 16; i++) counts[i] = 0;

  Image<float>::const_iterator itr = img.begin();
  for (int y = 0; y < h; ++y)
    {
      int suby = (4*y)/h;
      for (int x = 0; x < w; ++x)
        {
          int subx = (4*x)/w;
          int subpos = 4*suby + subx;
          tempRes[subpos+5] += *itr;

          ++(counts[subpos]);
          ++itr;
        }

    }

  int order[] = { 5,6,9,10, 7,8,11,12, 13,14,17,18, 15,16,19,20 };
  for (int i = 0 ; i < 16; ++i)
    if (counts[i] > 0)
        res[i+5] =  tempRes[order[i]] / (counts[order[i] - 5]+ 0.0);

   float tre1 = tempRes[5] + tempRes[6] + tempRes[9] + tempRes[10];
   int ct1 = counts[0] + counts[1] + counts[4] + counts[5];

   float tre2 = tempRes[7] + tempRes[8] + tempRes[11] + tempRes[12];
   int ct2 = counts[2] + counts[3] + counts[6] + counts[7];

   float tre3 = tempRes[13] + tempRes[14] + tempRes[17] + tempRes[18];
   int ct3 = counts[8] + counts[9] + counts[12] + counts[13];

   float tre4 = tempRes[15] + tempRes[16] + tempRes[19] + tempRes[20];
   int ct4 = counts[10] + counts[11] + counts[14] + counts[15];

  res[1] = tre1/ct1;
  res[2] = tre2/ct2;
  res[3] = tre3/ct3;
  res[4] = tre4/ct4;

  res[0] = (tre1 + tre2 + tre3 + tre4)/(ct1 + ct2 + ct3 + ct4);

  LDEBUG("lev1   : %14.4f", res[0]);
  LDEBUG("lev2   : %14.4f, %14.4f", res[1], res[2]);
  LDEBUG("       : %14.4f, %14.4f", res[3], res[4]);
  LDEBUG("lev3   : %14.4f, %14.4f, %14.4f, %14.4f",
         res[ 5], res[ 6], res[ 9], res[10]);
  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f",
         res[ 7], res[ 8], res[11], res[12]);
  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f",
         res[13], res[14], res[17], res[18]);
  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f",
         res[15], res[16], res[19], res[20]);

  return res;
};


//Following is copy from RawGistEstimatorStd
// ######################################################################
// get subsum of an image - more efficient implementation
inline
Image<double> getSubSum(Image<float> img)
{
  Image<float> res(1, 21, ZEROS);

  int w = img.getWidth();  int h = img.getHeight();
  std::vector<float> tempRes(21);
  for (int i = 0; i < 21; i++) tempRes[i] = 0.0;
  std::vector<int> counts(16);
  for (int i = 0 ; i < 16; i++) counts[i] = 0;

  Image<float>::const_iterator itr = img.begin();
  for (int y = 0; y < h; ++y)
    {
      int suby = (4*y)/h;
      for (int x = 0; x < w; ++x)
        {
          int subx = (4*x)/w;
          int subpos = 4*suby + subx;
          tempRes[subpos+5] += *itr;

          ++(counts[subpos]);
          ++itr;
        }

    }

  int order[] = { 5,6,9,10, 7,8,11,12, 13,14,17,18, 15,16,19,20 };
  for (int i = 0 ; i < 16; ++i)
    if (counts[i] > 0)
        res[i+5] =  tempRes[order[i]] / (counts[order[i] - 5]+ 0.0);

   float tre1 = tempRes[5] + tempRes[6] + tempRes[9] + tempRes[10];
   int ct1 = counts[0] + counts[1] + counts[4] + counts[5];

   float tre2 = tempRes[7] + tempRes[8] + tempRes[11] + tempRes[12];
   int ct2 = counts[2] + counts[3] + counts[6] + counts[7];

   float tre3 = tempRes[13] + tempRes[14] + tempRes[17] + tempRes[18];
   int ct3 = counts[8] + counts[9] + counts[12] + counts[13];

   float tre4 = tempRes[15] + tempRes[16] + tempRes[19] + tempRes[20];
   int ct4 = counts[10] + counts[11] + counts[14] + counts[15];

  res[1] = tre1/ct1;
  res[2] = tre2/ct2;
  res[3] = tre3/ct3;
  res[4] = tre4/ct4;

  res[0] = (tre1 + tre2 + tre3 + tre4)/(ct1 + ct2 + ct3 + ct4);

  LDEBUG("lev1   : %14.4f", res[0]);
  LDEBUG("lev2   : %14.4f, %14.4f", res[1], res[2]);
  LDEBUG("       : %14.4f, %14.4f", res[3], res[4]);
  LDEBUG("lev3   : %14.4f, %14.4f, %14.4f, %14.4f",
         res[ 5], res[ 6], res[ 9], res[10]);
  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f",
         res[ 7], res[ 8], res[11], res[12]);
  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f",
         res[13], res[14], res[17], res[18]);
  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f",
         res[15], res[16], res[19], res[20]);

  return res;
};


// ######################################################################
// get subsum of an image - less efficient but can change to stdev easily
/*Image<double> getSubSum2(Image<float> img)
{
  Image<double> res(1, 21, ZEROS);

  int w = img.getWidth();  int h = img.getHeight();

  int useMean = 1;
  // first level
  res[0] = mean(img); //sum(img);

  // second level
  // 1 2
  // 3 4
  Image<float> img1 = crop(img, Rectangle::tlbrI(0  , 0  , h/2-1,  w/2-1));
  Image<float> img2 = crop(img, Rectangle::tlbrI(0  , w/2, h/2-1,  w-1  ));
  Image<float> img3 = crop(img, Rectangle::tlbrI(h/2, 0  , h-1  ,  w/2-1));
  Image<float> img4 = crop(img, Rectangle::tlbrI(h/2, w/2, h-1  ,  w-1  ));

  if(useMean)
    {
      res[1] = mean(img1); //sum(img1);
      res[2] = mean(img2); //sum(img2);
      res[3] = mean(img3); //sum(img3);
      res[4] = mean(img4); //sum(img4);
    }
  else
    {
      res[1] = stdev(img1);//mean(img1); //sum(img1);
      res[2] = stdev(img2);//mean(img2); //sum(img2);
      res[3] = stdev(img3);//mean(img3); //sum(img3);
      res[4] = stdev(img4);//mean(img4); //sum(img4);
    }

  // third level
  // 1 2
  // 3 4
  int wX = img1.getWidth();  int hX = img1.getHeight();
  Image<float> imgX_1 = crop(img1, Rectangle::tlbrI(0   , 0   , hX/2-1,  wX/2-1));
  Image<float> imgX_2 = crop(img1, Rectangle::tlbrI(0   , wX/2, hX/2-1,  wX-1  ));
  Image<float> imgX_3 = crop(img1, Rectangle::tlbrI(hX/2, 0   , hX-1  ,  wX/2-1));
  Image<float> imgX_4 = crop(img1, Rectangle::tlbrI(hX/2, wX/2, hX-1  ,  wX-1  ));
  if(useMean)
    {
      res[5] = mean(imgX_1); //sum(imgX_1);
      res[6] = mean(imgX_2); //sum(imgX_2);
      res[7] = mean(imgX_3); //sum(imgX_3);
      res[8] = mean(imgX_4); //sum(imgX_4);
    }
  else
    {
      res[5] = stdev(imgX_1);//mean(imgX_1); //sum(imgX_1);
      res[6] = stdev(imgX_2);//mean(imgX_2); //sum(imgX_2);
      res[7] = stdev(imgX_3);//mean(imgX_3); //sum(imgX_3);
      res[8] = stdev(imgX_4);//mean(imgX_4); //sum(imgX_4);
    }

  wX = img2.getWidth();  hX = img2.getHeight();
  imgX_1 = crop(img2, Rectangle::tlbrI(0     , 0     , hX/2-1,  wX/2-1));
  imgX_2 = crop(img2, Rectangle::tlbrI(0     , wX/2  , hX/2-1,  wX-1  ));
  imgX_3 = crop(img2, Rectangle::tlbrI(hX/2  , 0     , hX-1  ,  wX/2-1));
  imgX_4 = crop(img2, Rectangle::tlbrI(hX/2  , wX/2  , hX-1  ,  wX-1  ));
  if(useMean)
    {
      res[ 9] = mean(imgX_1); //sum(imgX_1);
      res[10] = mean(imgX_2); //sum(imgX_2);
      res[11] = mean(imgX_3); //sum(imgX_3);
      res[12] = mean(imgX_4); //sum(imgX_4);
    }
  else
    {
      res[ 9] = stdev(imgX_1);//mean(imgX_1); //sum(imgX_1);
      res[10] = stdev(imgX_2);//mean(imgX_2); //sum(imgX_2);
      res[11] = stdev(imgX_3);//mean(imgX_3); //sum(imgX_3);
      res[12] = stdev(imgX_4);//mean(imgX_4); //sum(imgX_4);
    }

  wX = img3.getWidth();  hX = img3.getHeight();
  imgX_1 = crop(img3, Rectangle::tlbrI(0     , 0     , hX/2-1,  wX/2-1));
  imgX_2 = crop(img3, Rectangle::tlbrI(0     , wX/2  , hX/2-1,  wX-1  ));
  imgX_3 = crop(img3, Rectangle::tlbrI(hX/2  , 0     , hX-1  ,  wX/2-1));
  imgX_4 = crop(img3, Rectangle::tlbrI(hX/2  , wX/2  , hX-1  ,  wX-1  ));
  if(useMean)
    {
      res[13] = mean(imgX_1); //sum(imgX_1);
      res[14] = mean(imgX_2); //sum(imgX_2);
      res[15] = mean(imgX_3); //sum(imgX_3);
      res[16] = mean(imgX_4); //sum(imgX_4);
    }
  else
    {
      res[13] = stdev(imgX_1);//mean(imgX_1); //sum(imgX_1);
      res[14] = stdev(imgX_2);//mean(imgX_2); //sum(imgX_2);
      res[15] = stdev(imgX_3);//mean(imgX_3); //sum(imgX_3);
      res[16] = stdev(imgX_4);//mean(imgX_4); //sum(imgX_4);
    }

  wX = img4.getWidth();  hX = img4.getHeight();
  imgX_1 = crop(img4, Rectangle::tlbrI(0     , 0     , hX/2-1,  wX/2-1));
  imgX_2 = crop(img4, Rectangle::tlbrI(0     , wX/2  , hX/2-1,  wX-1  ));
  imgX_3 = crop(img4, Rectangle::tlbrI(hX/2  , 0     , hX-1  ,  wX/2-1));
  imgX_4 = crop(img4, Rectangle::tlbrI(hX/2  , wX/2  , hX-1  ,  wX-1  ));
  if(useMean)
    {
      res[17] = mean(imgX_1); //sum(imgX_1);
      res[18] = mean(imgX_2); //sum(imgX_2);
      res[19] = mean(imgX_3); //sum(imgX_3);
      res[20] = mean(imgX_4); //sum(imgX_4);
    }
  else
    {
      res[17] = stdev(imgX_1);//mean(imgX_1); //sum(imgX_1);
      res[18] = stdev(imgX_2);//mean(imgX_2); //sum(imgX_2);
      res[19] = stdev(imgX_3);//mean(imgX_3); //sum(imgX_3);
      res[20] = stdev(imgX_4);//mean(imgX_4); //sum(imgX_4);
    }

  LDEBUG("lev1   : %14.4f", res[0]);
  LDEBUG("lev2   : %14.4f, %14.4f", res[1], res[2]);
  LDEBUG("       : %14.4f, %14.4f", res[3], res[4]);
  LDEBUG("lev3   : %14.4f, %14.4f, %14.4f, %14.4f", res[ 5], res[ 6], res[ 9], res[10]);
  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f", res[ 7], res[ 8], res[11], res[12]);
  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f", res[13], res[14], res[17], res[18]);
  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f", res[15], res[16], res[19], res[20]);

  return res;
}
*/

// ######################################################################
// get the mean of the image insige the polygon
inline
double getMeanPoly(Image<float> img, std::vector<Point2D<int> > polygon)
{
  double res;
  double sum = 0;
  int countP = 0;
  //res = mean(img); 
  Point2D<int> P;
  for (int i = 0; i < img.getWidth(); i++)
    {
      for (int j = 0; j < img.getHeight(); j++)
        {
          P = Point2D<int> (i,j);
          if (pnpoly(polygon,P))
            {
              sum = sum + img.getVal(P);
              countP ++;
            }
        }
    }

  res = sum/countP; 
  return res;
};

//Following is copy from RawGistEstimatorStd
//// ######################################################################
//// get subsum of an image - more efficient implementation
//Image<double> getSubSum(Image<float> img)
//{
//  Image<float> res(1, 21, ZEROS);
//
//  int w = img.getWidth();  int h = img.getHeight();
//  std::vector<float> tempRes(21);
//
//  for (int i = 0; i < 21; i++) tempRes[i] = 0.0;
//  std::vector<int> counts(16);
//  for (int i = 0 ; i < 16; i++) counts[i] = 0;
//
//  Image<float>::const_iterator itr = img.begin();
//  for (int y = 0; y < h; ++y)
//    {
//      int suby = (4*y)/h;
//      for (int x = 0; x < w; ++x)
//        {
//          int subx = (4*x)/w;
//          int subpos = 4*suby + subx;
//          tempRes[subpos+5] += *itr;
//
//          ++(counts[subpos]);
//          ++itr;
//        }
//    }
//
//  int order[] = { 5,6,9,10, 7,8,11,12, 13,14,17,18, 15,16,19,20 };
//  for (int i = 0 ; i < 16; ++i)
//    if (counts[i] > 0)
//        res[i+5] =  tempRes[order[i]] / (counts[order[i] - 5]+ 0.0);
//
//   float tre1 = tempRes[5] + tempRes[6] + tempRes[9] + tempRes[10];
//   int ct1 = counts[0] + counts[1] + counts[4] + counts[5];
//
//   float tre2 = tempRes[7] + tempRes[8] + tempRes[11] + tempRes[12];
//   int ct2 = counts[2] + counts[3] + counts[6] + counts[7];
//
//   float tre3 = tempRes[13] + tempRes[14] + tempRes[17] + tempRes[18];
//   int ct3 = counts[8] + counts[9] + counts[12] + counts[13];
//
//   float tre4 = tempRes[15] + tempRes[16] + tempRes[19] + tempRes[20];
//   int ct4 = counts[10] + counts[11] + counts[14] + counts[15];
//
//  res[1] = tre1/ct1;
//  res[2] = tre2/ct2;
//  res[3] = tre3/ct3;
//  res[4] = tre4/ct4;
//
//  res[0] = (tre1 + tre2 + tre3 + tre4)/(ct1 + ct2 + ct3 + ct4);
//
//  LDEBUG("lev1   : %14.4f", res[0]);
//  LDEBUG("lev2   : %14.4f, %14.4f", res[1], res[2]);
//  LDEBUG("       : %14.4f, %14.4f", res[3], res[4]);
//  LDEBUG("lev3   : %14.4f, %14.4f, %14.4f, %14.4f",
//         res[ 5], res[ 6], res[ 9], res[10]);
//  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f",
//         res[ 7], res[ 8], res[11], res[12]);
//  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f",
//         res[13], res[14], res[17], res[18]);
//  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f",
//         res[15], res[16], res[19], res[20]);
//
//  return res;
//}
//
//// ######################################################################
//// get subsum of an image - less efficient but can change to stdev easily
///*Image<double> getSubSum2(Image<float> img)
//{
//  Image<double> res(1, 21, ZEROS);
//
//  int w = img.getWidth();  int h = img.getHeight();
//
//  int useMean = 1;
//  // first level
//  res[0] = mean(img); //sum(img);
//
//  // second level
//  // 1 2
//  // 3 4
//  Image<float> img1 = crop(img, Rectangle::tlbrI(0  , 0  , h/2-1,  w/2-1));
//  Image<float> img2 = crop(img, Rectangle::tlbrI(0  , w/2, h/2-1,  w-1  ));
//  Image<float> img3 = crop(img, Rectangle::tlbrI(h/2, 0  , h-1  ,  w/2-1));
//  Image<float> img4 = crop(img, Rectangle::tlbrI(h/2, w/2, h-1  ,  w-1  ));
//
//  if(useMean)
//    {
//      res[1] = mean(img1); //sum(img1);
//      res[2] = mean(img2); //sum(img2);
//      res[3] = mean(img3); //sum(img3);
//      res[4] = mean(img4); //sum(img4);
//    }
//  else
//    {
//      res[1] = stdev(img1);//mean(img1); //sum(img1);
//      res[2] = stdev(img2);//mean(img2); //sum(img2);
//      res[3] = stdev(img3);//mean(img3); //sum(img3);
//      res[4] = stdev(img4);//mean(img4); //sum(img4);
//    }
//
//  // third level
//  // 1 2
//  // 3 4
//  int wX = img1.getWidth();  int hX = img1.getHeight();
//  Image<float> imgX_1 = crop(img1, Rectangle::tlbrI(0   , 0   , hX/2-1,  wX/2-1));
//  Image<float> imgX_2 = crop(img1, Rectangle::tlbrI(0   , wX/2, hX/2-1,  wX-1  ));
//  Image<float> imgX_3 = crop(img1, Rectangle::tlbrI(hX/2, 0   , hX-1  ,  wX/2-1));
//  Image<float> imgX_4 = crop(img1, Rectangle::tlbrI(hX/2, wX/2, hX-1  ,  wX-1  ));
//  if(useMean)
//    {
//      res[5] = mean(imgX_1); //sum(imgX_1);
//      res[6] = mean(imgX_2); //sum(imgX_2);
//      res[7] = mean(imgX_3); //sum(imgX_3);
//      res[8] = mean(imgX_4); //sum(imgX_4);
//    }
//  else
//    {
//      res[5] = stdev(imgX_1);//mean(imgX_1); //sum(imgX_1);
//      res[6] = stdev(imgX_2);//mean(imgX_2); //sum(imgX_2);
//      res[7] = stdev(imgX_3);//mean(imgX_3); //sum(imgX_3);
//      res[8] = stdev(imgX_4);//mean(imgX_4); //sum(imgX_4);
//    }
//
//  wX = img2.getWidth();  hX = img2.getHeight();
//  imgX_1 = crop(img2, Rectangle::tlbrI(0     , 0     , hX/2-1,  wX/2-1));
//  imgX_2 = crop(img2, Rectangle::tlbrI(0     , wX/2  , hX/2-1,  wX-1  ));
//  imgX_3 = crop(img2, Rectangle::tlbrI(hX/2  , 0     , hX-1  ,  wX/2-1));
//  imgX_4 = crop(img2, Rectangle::tlbrI(hX/2  , wX/2  , hX-1  ,  wX-1  ));
//  if(useMean)
//    {
//      res[ 9] = mean(imgX_1); //sum(imgX_1);
//      res[10] = mean(imgX_2); //sum(imgX_2);
//      res[11] = mean(imgX_3); //sum(imgX_3);
//      res[12] = mean(imgX_4); //sum(imgX_4);
//    }
//  else
//    {
//      res[ 9] = stdev(imgX_1);//mean(imgX_1); //sum(imgX_1);
//      res[10] = stdev(imgX_2);//mean(imgX_2); //sum(imgX_2);
//      res[11] = stdev(imgX_3);//mean(imgX_3); //sum(imgX_3);
//      res[12] = stdev(imgX_4);//mean(imgX_4); //sum(imgX_4);
//    }
//
//  wX = img3.getWidth();  hX = img3.getHeight();
//  imgX_1 = crop(img3, Rectangle::tlbrI(0     , 0     , hX/2-1,  wX/2-1));
//  imgX_2 = crop(img3, Rectangle::tlbrI(0     , wX/2  , hX/2-1,  wX-1  ));
//  imgX_3 = crop(img3, Rectangle::tlbrI(hX/2  , 0     , hX-1  ,  wX/2-1));
//  imgX_4 = crop(img3, Rectangle::tlbrI(hX/2  , wX/2  , hX-1  ,  wX-1  ));
//  if(useMean)
//    {
//      res[13] = mean(imgX_1); //sum(imgX_1);
//      res[14] = mean(imgX_2); //sum(imgX_2);
//      res[15] = mean(imgX_3); //sum(imgX_3);
//      res[16] = mean(imgX_4); //sum(imgX_4);
//    }
//  else
//    {
//      res[13] = stdev(imgX_1);//mean(imgX_1); //sum(imgX_1);
//      res[14] = stdev(imgX_2);//mean(imgX_2); //sum(imgX_2);
//      res[15] = stdev(imgX_3);//mean(imgX_3); //sum(imgX_3);
//      res[16] = stdev(imgX_4);//mean(imgX_4); //sum(imgX_4);
//    }
//
//  wX = img4.getWidth();  hX = img4.getHeight();
//  imgX_1 = crop(img4, Rectangle::tlbrI(0     , 0     , hX/2-1,  wX/2-1));
//  imgX_2 = crop(img4, Rectangle::tlbrI(0     , wX/2  , hX/2-1,  wX-1  ));
//  imgX_3 = crop(img4, Rectangle::tlbrI(hX/2  , 0     , hX-1  ,  wX/2-1));
//  imgX_4 = crop(img4, Rectangle::tlbrI(hX/2  , wX/2  , hX-1  ,  wX-1  ));
//  if(useMean)
//    {
//      res[17] = mean(imgX_1); //sum(imgX_1);
//      res[18] = mean(imgX_2); //sum(imgX_2);
//      res[19] = mean(imgX_3); //sum(imgX_3);
//      res[20] = mean(imgX_4); //sum(imgX_4);
//    }
//  else
//    {
//      res[17] = stdev(imgX_1);//mean(imgX_1); //sum(imgX_1);
//      res[18] = stdev(imgX_2);//mean(imgX_2); //sum(imgX_2);
//      res[19] = stdev(imgX_3);//mean(imgX_3); //sum(imgX_3);
//      res[20] = stdev(imgX_4);//mean(imgX_4); //sum(imgX_4);
//    }
//
//  LDEBUG("lev1   : %14.4f", res[0]);
//  LDEBUG("lev2   : %14.4f, %14.4f", res[1], res[2]);
//  LDEBUG("       : %14.4f, %14.4f", res[3], res[4]);
//  LDEBUG("lev3   : %14.4f, %14.4f, %14.4f, %14.4f", res[ 5], res[ 6], res[ 9], res[10]);
//  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f", res[ 7], res[ 8], res[11], res[12]);
//  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f", res[13], res[14], res[17], res[18]);
//  LDEBUG("       : %14.4f, %14.4f, %14.4f, %14.4f", res[15], res[16], res[19], res[20]);
//
//  return res;
//}
//*/
//
//// ######################################################################
//// get the mean of the image insige the polygon
//double getMeanPoly(Image<float> img, std::vector<Point2D<int> > polygon)
//{
//  double res;
//  double sum = 0;
//  int countP = 0;
//  //res = mean(img); 
//  Point2D<int> P;
//  for (int i = 0; i < img.getWidth(); i++)
//    {
//      for (int j = 0; j < img.getHeight(); j++)
//        {
//          P = Point2D<int> (i,j);
//          if (pnpoly(polygon,P))
//            {
//              sum = sum + img.getVal(P);
//              countP ++;
//            }
//        }
//    }
//
//  res = sum/countP; 
//  return res;
//}
//
//
//
#endif
