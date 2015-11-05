/*!@file SIFT/ScaleSpace.C Keypoint computation for SIFT obj recognition */

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
// Primary maintainer for this file: James Bonaiuto <bonaiuto@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/ScaleSpace.C $
// $Id: ScaleSpace.C 10423 2008-11-12 22:26:07Z mviswana $
//

#include "SIFT/ScaleSpace.H"

#include "Image/FilterOps.H"
#include "Image/Kernels.H"
#include "Image/MathOps.H"
#include "Image/MatrixOps.H"
#include "Image/DrawOps.H"
#include "SIFT/Histogram.H"
#include "SIFT/Keypoint.H"
#include "SIFT/FeatureVector.H"
#include "rutz/compat_cmath.h"
#include "Image/fancynorm.H"

const int   EXT          = 10;     //!< keep out edge width
const float R_EDGE       = 8.0F;   //!< edge response threshold
const float PEAK_THRESH  = 2.0F;   //!< peak below that doesn't count
const int   ORIENTARRAY  = 36;     //!< size of orientation array


// ######################################################################
ScaleSpace::ScaleSpace(const ImageSet<float>& in, const float octscale,
                const int s, const float sigma, bool useColor) :
        itsOctScale(octscale), itsSigma(sigma), itsLumBlur(s + 3), itsRGBlur(s+3),
        itsBYBlur(s+3), itsDog(s + 2), itsUseColor(useColor)
{
        ASSERT(s > 0); ASSERT(octscale > 0.0F); ASSERT(sigma > 0.0F);

        const float k = powf(2.0f, 1.0f / s);

        // our bottom image is just the original. We here assume that the
        // caller has pre-blurred it if necessary. See the constructor of
        // VisualObject for what we mean by that:
        itsLumBlur[0] = in[LUM_CHANNEL];

        if (itsUseColor){
                itsRGBlur[0] = in[RG_CHANNEL];
                itsBYBlur[0] = in[BY_CHANNEL];
        }

        // compute the additional gaussian blurred maps:
        for (int ss = 1; ss < s+3; ++ss)
        {
                // compute the stdev of a Gaussian we should convolve the
                // previous blurred image by, so as to multiply the total
                // effective blurring sigma by a factor k:
                const float std = sigma * powf(k, float(ss-1)) * sqrt(k*k - 1.0F);

                // get a Gaussian kernel by that stdev, normalized to unit sum:
                Image<float> kernel = gaussian<float>(1.0F, std, itsLumBlur[0].getWidth(), 1.0F);
                kernel = kernel / float(sum(kernel));

                // do the convolution:
                itsLumBlur[ss] = sepFilter(itsLumBlur[ss-1], kernel, kernel,
                                           CONV_BOUNDARY_CLEAN);

                if (itsUseColor){
                  itsRGBlur[ss] = sepFilter(itsRGBlur[ss-1], kernel, kernel,
                                            CONV_BOUNDARY_CLEAN);
                  itsBYBlur[ss] = sepFilter(itsBYBlur[ss-1], kernel, kernel,
                                            CONV_BOUNDARY_CLEAN);
                }
        }

        // compute the difference of gaussian images:
        for (int ss = 0; ss < s+2; ++ss)
                itsDog[ss] = itsLumBlur[ss+1] - itsLumBlur[ss];


        /*
        //take the max normalize
#define NORMTYPE VCXNORM_MAXNORM
for (int ss=0; ss < s+3; ++ss){
itsLumBlur[ss] = maxNormalize(itsLumBlur[ss], 0.0F, 2.0F, NORMTYPE);
itsRGBlur[ss] = maxNormalize(itsRGBlur[ss], 0.0F, 2.0F, NORMTYPE);
itsBYBlur[ss] = maxNormalize(itsBYBlur[ss], 0.0F, 2.0F, NORMTYPE);
}*/
                }

// ######################################################################
ScaleSpace::~ScaleSpace()
{ }

// ######################################################################
Image<float> ScaleSpace::getTwoSigmaImage(int channel) const
{
        //LINFO("Channel select");
        switch (channel){
                case LUM_CHANNEL:
                        return itsLumBlur.getImage(itsLumBlur.size() - 3);
                case RG_CHANNEL:
                        return itsRGBlur.getImage(itsRGBlur.size() - 3);
                case BY_CHANNEL:
                        return itsRGBlur.getImage(itsBYBlur.size() - 3);
        }

        LINFO("Invalid channel %i", channel);
        ASSERT(false);
        return itsLumBlur.getImage(itsLumBlur.size() - 3);
}


// ######################################################################
uint ScaleSpace::getNumBlurredImages() const
{ return itsLumBlur.size(); }

// ######################################################################
Image<float> ScaleSpace::getBlurredImage(const uint idx) const
{ return itsLumBlur.getImage(idx); }

// ######################################################################
uint ScaleSpace::getNumDoGImages() const
{ return itsDog.size(); }

// ######################################################################
Image<float> ScaleSpace::getDoGImage(const uint idx) const
{ return itsDog.getImage(idx); }

// ######################################################################
uint ScaleSpace::findKeypoints(std::vector< rutz::shared_ptr<Keypoint> >& keypoints)
{
        int w = itsLumBlur[0].getWidth(), h = itsLumBlur[0].getHeight();
        Image<byte> analyzed(w, h, ZEROS); // keep track of visited locations
        uint numkp = 0;

        // loop through blurred images in scale space:
        for (uint sc = 1; sc <= itsLumBlur.size()-3; sc++)
        {
                LDEBUG("Processing octave %.2f scale %d/%d", itsOctScale,
                                sc, itsLumBlur.size()-3);

                // compute magnitude and orientation of the gradient of the
                // blurred image for the scale of interest:
                ImageSet<float> gradmag(3), gradori(3);
                gradient(itsLumBlur[sc], gradmag[LUM_CHANNEL], gradori[LUM_CHANNEL]);

                if (itsUseColor){
                        gradient(itsRGBlur[sc], gradmag[RG_CHANNEL], gradori[RG_CHANNEL]);
                        gradient(itsBYBlur[sc], gradmag[BY_CHANNEL], gradori[BY_CHANNEL]);

                }


                // for every pixels in image except for edges
                for (int x = EXT; x < w - EXT; x++)
                        for (int y = EXT; y < h - EXT; y++)
                                // check for maximum or minimum of DoG in scale space;
                                // if found, trigger accurate localization and pruning
                                // of keypoint:
                                if (checkForMinMax(x, y, itsDog[sc-1], itsDog[sc], itsDog[sc+1]))
                                        numkp += accurateLocalizationAndPruning(x, y, sc, analyzed,
                                                        gradmag, gradori,
                                                        keypoints);
        }
        return numkp;
}

// ######################################################################

// Custom and highly irregular version of SIFT keypoint creation. This
// was written specifically for the Lazebnik Beyond Bags-of-Features
// paper. Works only for level zero. Thou hast been warned.
//
// For more info, see src/Neuro/GistEstimatorBeyondBoF.H and .C; also,
// src/Gist/train-bbof.C.
static void
pixelPatchCreateKeypoint(const float x, const float y, const float sigma,
                         const float dogmag, const Image<float>& gradmag,
                         const Image<float>& gradorie,
                         std::vector<rutz::shared_ptr<Keypoint> >& keypoints)
{
   const int xi = int(x + 0.5f);
   const int yi = int(y + 0.5f);

   FeatureVector fv;

   // check this scale
   const int radius = int(5.0f * sigma + 0.5f); // NOTE: Lowe uses radius=8?
   const float gausssig = float(radius); // 1/2 width of descript window
   const float gaussfac = -0.5f/(gausssig * gausssig);

   // Scan a window of diameter 2*radius+1 around the point of
   // interest, and we will cumulate local samples into a 4x4 grid
   // of bins, with interpolation. NOTE: rx and ry loop over a
   // square that is assumed centered around the point of interest.
   for (int ry = -radius; ry < radius; ry++)
      for (int rx = -radius; rx < radius; rx++)
      {
         // get the coords in the image frame of reference:
         const float orgX = rx + float(xi);
         const float orgY = ry + float(yi);

         if (! gradmag.coordsOk(orgX, orgY)) // outside image
            continue; // forget this coordinate

         // find the fractional coords of the corresponding bin
         // (we subdivide our window into a 4x4 grid of bins):
         const float xf = 2.0f + 2.0f * float(rx)/float(radius);
         const float yf = 2.0f + 2.0f * float(ry)/float(radius);

         // find the Gaussian weight from distance to center and
         // get weighted gradient magnitude:
         const float gaussFactor = expf((rx*rx + ry*ry) * gaussfac);
         const float weightedMagnitude =
            gaussFactor * gradmag.getValInterp(orgX, orgY);

         // get the gradient orientation relative to the keypoint
         // orientation and scale it for 8 orientation bins:
         float gradAng = gradorie.getValInterp(orgX, orgY);
         gradAng=fmod(gradAng, 2*M_PI); //bring the range from 0 to M_PI

         //convert from -M_PI to M_PI
         if (gradAng < 0.0)
            gradAng += 2*M_PI; //convert to -M_PI to M_PI
         if (gradAng >= M_PI)
            gradAng -= 2*M_PI;
         //split to eight bins
         const float orient = (gradAng + M_PI) * 8 / (2 * M_PI);

         // will be interpolated into 2 x 2 x 2 bins:
         fv.addValue(xf, yf, orient, weightedMagnitude);
      }

   // normalize, clamp, scale and convert to byte:
   std::vector<byte> oriVec;
   fv.toByteKey(oriVec);

   rutz::shared_ptr<Keypoint>
      newkey(new Keypoint(oriVec, x, y, sigma, 0, dogmag));
   keypoints.push_back(newkey);
}

uint
ScaleSpace::
getGridKeypoints(std::vector<rutz::shared_ptr<Keypoint> >& keypoints)
{
   // compute magnitude and orientation of the gradient of the
   // blurred image for the scale of interest:
   Image<float> gradmag, gradori;
   gradient(itsLumBlur[0], gradmag, gradori);

   int w = itsLumBlur[0].getWidth(),
       h = itsLumBlur[0].getHeight();
   uint numkp = 0;
   const int radius = int(5.0f * itsSigma + 0.5f);
   for (int y = radius; y < h - radius + 1; y += radius)
      for (int x = radius; x < w - radius + 1; x += radius) {
         pixelPatchCreateKeypoint(x, y, itsSigma, 1,
                                  gradmag, gradori, keypoints);
         ++numkp ;
      }

   return numkp ;
}

// ######################################################################
uint ScaleSpace::
accurateLocalizationAndPruning(const int x, const int y, const int s,
                Image<byte>& analyzed,
                const ImageSet<float>& gradmag,
                const ImageSet<float>& gradori,
                std::vector< rutz::shared_ptr<Keypoint> >& keypoints)
{
        // Interactive 3D interpolation (x direction, y direction, and scale
        // direction). We do a 3d fit of this extremum. The resulting dpos
        // contains the accurate fractional location of the extremum
        // relative to (new_x, new_y, s), and dDog is the derivative of DoG
        // at that point:
        int new_x = x, new_y = y, new_s = s;
        Image<float> dDog, dpos; float dx2, dy2, dxdy;
        dpos = fit3D(new_x, new_y, s, dDog, dx2, dy2, dxdy);

        // give up on some overflow cases:
        if (fabs(dpos.getVal(0)) > 1.5F) return 0;
        if (fabs(dpos.getVal(1)) > 1.5F) return 0;
        if (fabs(dpos.getVal(2)) > 1.5F) return 0;

        // If dpos is larger than 0.5 in x or y direction, we should move
        // our keypoint to a better location and try our 3D fit again:
        bool moved = false;
        if (dpos.getVal(0) > 0.5)       { ++new_x; moved = true; }
        else if (dpos.getVal(0) < -0.5) { --new_x; moved = true; }

        if (dpos.getVal(1) > 0.5)       { ++new_y; moved = true; }
        else if (dpos.getVal(1) < -0.5) { --new_y; moved = true; }

        if (dpos.getVal(2) > 0.5) {
                if (new_s >= int(itsDog.size()) - 2) return 0; // can't go higher in scale
                ++new_s; moved = true;
        } else if (dpos.getVal(2) < -0.5) {
                if (new_s <= 1) return 0; // can't go lower in scale
                --new_s; moved = true;
        }

        // if we moved, recompute the 3D fit:
        if (moved)
        {
                dpos = fit3D(new_x, new_y, new_s, dDog, dx2, dy2, dxdy);

                // give up if we can't get close:
                if (fabs(dpos.getVal(0)) > 0.5F) return 0;
                if (fabs(dpos.getVal(1)) > 0.5F) return 0;
                if (fabs(dpos.getVal(2)) > 0.5F) return 0;
        }

        // if already analyzed then quit, otherwise mark as analyzed:
        if (analyzed.getVal(new_x, new_y)) return 0;
        analyzed.setVal(new_x, new_y, 255);

        // here are our new fractional coordinates:
        const float xf = float(new_x) + float(dpos.getVal(0));
        const float yf = float(new_y) + float(dpos.getVal(1));
        const float sf = float(new_s) + float(dpos.getVal(2));

        // calculate new peak height, using linear approximation:
        const float peak_height =
                itsDog[new_s].getVal(new_x, new_y) + 0.5F * dotprod(dpos, dDog);

        // forget about that keypoint if the DoG value is too small
        // (low-contrast keypoint):
        if (fabsf(peak_height) < PEAK_THRESH ) return 0;

        // pruning big edge response (this will be used as denominator in
        // the following test):
        if (fabs(dx2 * dy2 - dxdy * dxdy) < 1.0e-5F) return 0;

        // calculate edge test, to eliminate keypoints along strong edges:
        if ((dx2+dy2)*(dx2+dy2) / fabs(dx2 * dy2 - dxdy * dxdy) >=
                        (R_EDGE + 1.0F) * (R_EDGE + 1.0F) / R_EDGE) return 0;

        // if we have reached this point, we got a stable keypoint!  Create
        // its feature vector:
        return createKeypoints(xf, yf, sf, peak_height, gradmag, gradori, keypoints);
}

// ######################################################################
bool ScaleSpace::checkForMinMax(const int x, const int y,
                const Image<float>& im0,
                const Image<float>& im1,
                const Image<float>& im2) const
{
        // get value:
        float val = im1.getVal(x, y);

        // if below peak threshold just give up:
        if (fabsf(val) < PEAK_THRESH) return false;

        // verify for max or min:
        if (val < im1.getVal(x-1, y))
        {
                // ok, then let's check for a minimum:
                if (val >= im1.getVal(x-1, y-1)) return false;
                if (val >= im1.getVal(x-1, y+1)) return false;
                if (val >= im1.getVal(x  , y-1)) return false;
                if (val >= im1.getVal(x  , y+1)) return false;
                if (val >= im1.getVal(x+1, y-1)) return false;
                if (val >= im1.getVal(x+1, y  )) return false;
                if (val >= im1.getVal(x+1, y+1)) return false;

                // check for min level -1:
                if (val >= im0.getVal(x-1, y-1)) return false;
                if (val >= im0.getVal(x-1, y  )) return false;
                if (val >= im0.getVal(x-1, y+1)) return false;
                if (val >= im0.getVal(x  , y-1)) return false;
                if (val >= im0.getVal(x  , y  )) return false;
                if (val >= im0.getVal(x  , y+1)) return false;
                if (val >= im0.getVal(x+1, y-1)) return false;
                if (val >= im0.getVal(x+1, y  )) return false;
                if (val >= im0.getVal(x+1, y+1)) return false;

                // check for min level +1:
                if (val >= im2.getVal(x-1, y-1)) return false;
                if (val >= im2.getVal(x-1, y  )) return false;
                if (val >= im2.getVal(x-1, y+1)) return false;
                if (val >= im2.getVal(x  , y-1)) return false;
                if (val >= im2.getVal(x  , y  )) return false;
                if (val >= im2.getVal(x  , y+1)) return false;
                if (val >= im2.getVal(x+1, y-1)) return false;
                if (val >= im2.getVal(x+1, y  )) return false;
                if (val >= im2.getVal(x+1, y+1)) return false;

                return true;
        }
        else if (val > im1.getVal(x-1, y))
        {
                // check for maximum:
                if (val <= im1.getVal(x-1, y-1)) return false;
                if (val <= im1.getVal(x-1, y+1)) return false;
                if (val <= im1.getVal(x  , y-1)) return false;
                if (val <= im1.getVal(x  , y+1)) return false;
                if (val <= im1.getVal(x+1, y-1)) return false;
                if (val <= im1.getVal(x+1, y  )) return false;
                if (val <= im1.getVal(x+1, y+1)) return false;

                // check for max level -1:
                if (val <= im0.getVal(x-1, y-1)) return false;
                if (val <= im0.getVal(x-1, y  )) return false;
                if (val <= im0.getVal(x-1, y+1)) return false;
                if (val <= im0.getVal(x  , y-1)) return false;
                if (val <= im0.getVal(x  , y  )) return false;
                if (val <= im0.getVal(x  , y+1)) return false;
                if (val <= im0.getVal(x+1, y-1)) return false;
                if (val <= im0.getVal(x+1, y  )) return false;
                if (val <= im0.getVal(x+1, y+1)) return false;

                // check for max level +1:
                if (val <= im2.getVal(x-1, y-1)) return false;
                if (val <= im2.getVal(x-1, y  )) return false;
                if (val <= im2.getVal(x-1, y+1)) return false;
                if (val <= im2.getVal(x  , y-1)) return false;
                if (val <= im2.getVal(x  , y  )) return false;
                if (val <= im2.getVal(x  , y+1)) return false;
                if (val <= im2.getVal(x+1, y-1)) return false;
                if (val <= im2.getVal(x+1, y  )) return false;
                if (val <= im2.getVal(x+1, y+1)) return false;

                return true;
        }

        return false;
}

// ######################################################################
//
// Fit in 3D
//   in the (X, Y, S) space, fit the peak, to find it's best extrenum
//   return solution and dDog as first derivative of Dog
//
Image<float> ScaleSpace::fit3D(const int x, const int y, const int s,
                Image<float>& dDog, float& dX2,
                float& dY2, float& dXdY) const
{
        Image<float> below = itsDog[s - 1];
        Image<float> cur   = itsDog[s];
        Image<float> above = itsDog[s + 1];

        // standard approximations of first derivatives of the DoG:
        float dX = 0.5F * (cur.getVal(x+1, y) - cur.getVal(x-1, y));
        float dY = 0.5F * (cur.getVal(x, y+1) - cur.getVal(x, y-1));
        float dS = 0.5F * (above.getVal(x, y) - below.getVal(x, y));

        // standard approximations of second derivatives of the DoG:
        dX2 = cur.getVal(x-1, y) - 2.0F*cur.getVal(x, y) + cur.getVal(x+1, y);
        dY2 = cur.getVal(x, y-1) - 2.0F*cur.getVal(x, y) + cur.getVal(x, y+1);
        float dS2 = below.getVal(x, y) - 2.0F*cur.getVal(x, y) + above.getVal(x, y);

        // standard approximation of crossed derivatives of the DoG:
        dXdY  = 0.25F * (cur.getVal(x+1, y+1) - cur.getVal(x-1, y+1)) -
                0.25F * (cur.getVal(x+1, y-1) - cur.getVal(x-1, y-1));
        float dSdX = 0.25F * (above.getVal(x+1, y) - above.getVal(x-1, y)) -
                0.25F * (below.getVal(x+1, y) - below.getVal(x-1, y));
        float dSdY = 0.25F * (above.getVal(x, y+1) - above.getVal(x, y-1)) -
                0.25F * (below.getVal(x, y+1) - below.getVal(x, y-1));

        // matrix to solve is:
        // dX         [ dX2  dXdY dXdS ]
        // dY = [ V ] [ dXdY dY2  dYdS ]
        // dS         [ dXdS dYdS dS2  ]
        dDog.resize(1, 3);
        dDog.setVal(0, -dX); dDog.setVal(1, -dY); dDog.setVal(2, -dS);

        Image<float> mat(3, 3, NO_INIT);
        mat.setVal(0, 0,  dX2); mat.setVal(1, 1,  dY2); mat.setVal(2, 2,  dS2);
        mat.setVal(1, 0, dXdY); mat.setVal(0, 1, dXdY);
        mat.setVal(2, 0, dSdX); mat.setVal(0, 2, dSdX);
        mat.setVal(2, 1, dSdY); mat.setVal(1, 2, dSdY);

        Image<float> result;
        try
          {
            mat = matrixInv(mat);
            result = matrixMult(mat, dDog);
          }
        catch (SingularMatrixException& e)
          {
            LDEBUG("Couldn't invert matrix -- RETURNING [ 2.0 2.0 2.0 ]^T");
            result.resize(1, 3);
            result.setVal(0, 0, 2.0);
            result.setVal(0, 1, 2.0);
            result.setVal(0, 2, 2.0);
          }
        return result;
}


// ######################################################################

void ScaleSpace::calculateOrientationVector(const float x, const float y, const float s,
                const ImageSet<float>& gradmag, const ImageSet<float>& gradorie, Histogram& OV) {


        // compute the effective blurring sigma corresponding to the
        // fractional scale s:
        const float sigma = itsSigma * powf(2.0F, s / float(itsLumBlur.size() - 3));

        const float sig = 1.5F * sigma, inv2sig2 = - 0.5F / (sig * sig);
        const int dimX = gradmag[LUM_CHANNEL].getWidth(), dimY = gradmag[LUM_CHANNEL].getHeight();

        const int xi = int(x + 0.5f);
        const int yi = int(y + 0.5f);

        const int rad = int(3.0F * sig);
        const int rad2 = rad * rad;


        // search bounds:
        int starty = yi - rad; if (starty < 0) starty = 0;
        int stopy = yi + rad; if (stopy >= dimY) stopy = dimY-1;

        // 1. Calculate orientation vector
        for (int ind_y = starty; ind_y <= stopy; ind_y ++)
        {
                // given that y, get the corresponding range of x values that
                // lie within the disk (and remain within the image):
                const int yoff = ind_y - yi;
                const int bound = int(sqrtf(float(rad2 - yoff*yoff)) + 0.5F);
                int startx = xi - bound; if (startx < 0) startx = 0;
                int stopx = xi + bound; if (stopx >= dimX) stopx = dimX-1;

                for (int ind_x = startx; ind_x <= stopx; ind_x ++)
                {
                        const float dx = float(ind_x) - x, dy = float(ind_y) - y;
                        const float distSq = dx * dx + dy * dy;

                        // get gradient:
                        const float gradVal = gradmag[LUM_CHANNEL].getVal(ind_x, ind_y);

                        // compute the gaussian weight for this histogram entry:
                        const float gaussianWeight = expf(distSq * inv2sig2);

                        // add this orientation to the histogram
                        // [-pi ; pi] -> [0 ; 2pi]
                        float angle = gradorie[LUM_CHANNEL].getVal(ind_x, ind_y) + M_PI;

                        // [0 ; 2pi] -> [0 ; 36]
                        angle = 0.5F * angle * ORIENTARRAY / M_PI;
                        while (angle < 0.0F) angle += ORIENTARRAY;
                        while (angle >= ORIENTARRAY) angle -= ORIENTARRAY;

                        OV.addValueInterp(angle, gaussianWeight * gradVal);
                }
        }


        // smooth the orientation histogram 3 times:
        for (int i = 0; i < 3; i++) OV.smooth();
}

// ######################################################################


uint ScaleSpace::createVectorsAndKeypoints(const float x, const float y, const float s,
                const float dogmag, const ImageSet<float>& gradmag,
                const ImageSet<float>& gradorie, std::vector < rutz::shared_ptr<Keypoint> >& keypoints, Histogram& OV)
{

        const float sigma = itsSigma * powf(2.0F, s / float(itsLumBlur.size() - 3));

        // find the max in the histogram:
        float maxPeakValue = OV.findMax();

        const int xi = int(x + 0.5f);
        const int yi = int(y + 0.5f);

        uint numkp = 0;

        // 2. Create feature vector and keypoint for each significant
        // orientation peak:
        for (int bin = 0; bin < ORIENTARRAY; bin++)
        {
                // consider the peak centered around 'bin':
                const float midval = OV.getValue(bin);

                // if current value much smaller than global peak, forget it:
                if (midval < 0.8F * maxPeakValue) continue;

                // get value to the left of current value
                const float leftval = OV.getValue((bin == 0) ? ORIENTARRAY-1 : bin-1);

                // get value to the right of current value
                const float rightval = OV.getValue((bin == ORIENTARRAY-1) ? 0 : bin+1);

                // only consider local peaks:
                if (leftval >= midval) continue;
                if (rightval >= midval) continue;

                // interpolate the values to get the orientation of the peak:
                //  with f(x) = ax^2 + bx + c
                //   f(-1) = x0 = leftval
                //   f( 0) = x1 = midval
                //   f(+1) = x2 = rightval
                //  => a = (x0+x2)/2 - x1
                //     b = (x2-x0)/2
                //     c = x1
                // f'(x) = 0 => x = -b/2a
                const float a  = 0.5f * (leftval + rightval) - midval;
                const float b  = 0.5f * (rightval - leftval);
                float realangle = float(bin) - 0.5F * b / a;

                realangle *= 2.0F * M_PI / ORIENTARRAY; // [0:36] to [0:2pi]
                realangle -= M_PI;                      // [0:2pi] to [-pi:pi]

                // ############ Create keypoint:

                // compute the feature vector:
                FeatureVector fv;
                FeatureVector Colfv(4, 4, 3, false); //create 4x4x3 bids with no wrap

                const float sinAngle = sin(realangle), cosAngle = cos(realangle);

                // check this scale
                const int radius = int(5.0F * sigma + 0.5F); // NOTE: Lowe uses radius=8?
                const float gausssig = float(radius); // 1/2 width of descript window
                const float gaussfac = - 0.5F / (gausssig * gausssig);


                // Scan a window of diameter 2*radius+1 around the point of
                // interest, and we will cumulate local samples into a 4x4 grid
                // of bins, with interpolation. NOTE: rx and ry loop over a
                // square that is assumed centered around the point of interest
                // and rotated to the gradient orientation (realangle):

                int scale = abs(int(s));
                scale = scale > 5 ? 5 : scale;

                float maxRG=-9999, minRG=9999, maxBY=-9999, minBY=9999, maxBW=-9999, minBW=9999;

                for (int ry = -radius; ry <= radius; ry++)
                        for (int rx = -radius; rx <= radius; rx++)
                        {
                                // rotate the point:
                                const float newX = rx * cosAngle - ry * sinAngle;
                                const float newY = rx * sinAngle + ry * cosAngle;

                                // get the coords in the image frame of reference:
                                const float orgX = newX + float(xi);
                                const float orgY = newY + float(yi);

                                // if outside the image, forget it:
                                if (gradmag[LUM_CHANNEL].coordsOk(orgX, orgY) == false) continue;

                                // find the fractional coords of the corresponding bin
                                // (we subdivide our window into a 4x4 grid of bins):
                                const float xf = 2.0F + 2.0F * float(rx) / float(radius);
                                const float yf = 2.0F + 2.0F * float(ry) / float(radius);


                                // find the Gaussian weight from distance to center and
                                // get weighted gradient magnitude:
                                const float gaussFactor = expf((newX*newX+newY*newY) * gaussfac);
                                const float weightedMagnitude =
                                        gaussFactor * gradmag[LUM_CHANNEL].getValInterp(orgX, orgY);

                                // get the gradient orientation relative to the keypoint
                                // orientation and scale it for 8 orientation bins:
                                float gradAng = gradorie[LUM_CHANNEL].getValInterp(orgX, orgY) - realangle;

                                gradAng=fmod(gradAng, 2*M_PI); //bring the range from 0 to M_PI

                                //convert from -M_PI to M_PI
                                if (gradAng < 0.0) gradAng+=2*M_PI; //convert to -M_PI to M_PI
                                if (gradAng >= M_PI) gradAng-=2*M_PI;
                                //split to eight bins
                                const float orient = (gradAng + M_PI) * 8 / (2 * M_PI);

                                /*
                                //reflect the angle to convert from 0 to M_PI
                                if (gradAng >= M_PI) gradAng-=M_PI;
                                //split to four bins
                                const float orient = (gradAng + M_PI) * 4 / (2 * M_PI);
                                */

                                // will be interpolated into 2 x 2 x 2 bins:
                                fv.addValue(xf, yf, orient, weightedMagnitude);


                                if (itsUseColor){

                                        /*        float bw = gradmag[LUM_CHANNEL].getValInterp(orgX, orgY);
                                                                                float rg = gradmag[RG_CHANNEL].getValInterp(orgX, orgY);
                                                                                float by = gradmag[BY_CHANNEL].getValInterp(orgX, orgY); */

                                        float bw = itsLumBlur[scale].getValInterp(orgX, orgY);
                                        float rg = itsRGBlur[scale].getValInterp(orgX, orgY);
                                        float by = itsBYBlur[scale].getValInterp(orgX, orgY);


                                        Colfv.addValue(xf, yf, 0.5, bw*gaussFactor);
                                        Colfv.addValue(xf, yf, 1.5, rg*gaussFactor);
                                        Colfv.addValue(xf, yf, 2.5, by*gaussFactor);


                                        //find the max and min of the colors
                                        if (bw > maxBW) maxBW = bw;
                                        if (bw < minBW) minBW = bw;

                                        if (rg > maxRG) maxRG = rg;
                                        if (rg < minRG) minRG = rg;

                                        if (by > maxBY) maxBY = by;
                                        if (by < minBY) minBY = by;


                                }


                        }

                // normalize, clamp, scale and convert to byte:
                std::vector<byte> oriVec;
                fv.toByteKey(oriVec);


                if (itsUseColor){

                        std::vector<byte> colVec;
                        Colfv.toByteKey(colVec, -1, true);
                        /*//put the max and min colors
                                colVec.push_back(byte(maxBW));
                                colVec.push_back(byte(minBW));
                                colVec.push_back(byte(maxRG));
                                colVec.push_back(byte(minRG));
                                colVec.push_back(byte(maxBY));
                                colVec.push_back(byte(minBY));*/

                        const float alpha = 0.2; //1.0F/(s+1); //have alpha as a function of scale
                        float oriWeight = 1.0F * (1.0F-alpha);
                        float colWeight = 2.67F * alpha; //21.34 * alpha; //2.67F * alpha;
                        rutz::shared_ptr<Keypoint>
                                newkey(new Keypoint(oriVec, colVec, x * itsOctScale,
                                                        y * itsOctScale, sigma * itsOctScale, realangle, dogmag,
                                                        oriWeight, colWeight)); //set the weights for the ori and col
                        // add to list of keys:
                        keypoints.push_back(newkey);
                        ++ numkp;

                } else {


                        // create a keypoint:
                        rutz::shared_ptr<Keypoint>
                                newkey(new Keypoint(oriVec, x * itsOctScale, y * itsOctScale,
                                                        sigma * itsOctScale, realangle, dogmag));
                        // add to list of keys:
                        keypoints.push_back(newkey);
                        ++ numkp;
                }

        }
        return numkp;
}


// ######################################################################
uint ScaleSpace::createKeypoints(const float x, const float y, const float s,
                const float dogmag, const ImageSet<float>& gradmag,
                const ImageSet<float>& gradorie,
                std::vector< rutz::shared_ptr<Keypoint> >& keypoints)
{
        uint numkp = 0;


        // orientation histogram:
        Histogram OV(ORIENTARRAY);

        // radius and gaussian window calculus:


        // 1. Calculate orientation vector
        calculateOrientationVector(x, y, s, gradmag, gradorie, OV);


        // 2. Create feature vector and keypoint for each significant
        // orientation peak:
        numkp = createVectorsAndKeypoints(x, y, s, dogmag, gradmag, gradorie, keypoints, OV);


        return numkp;
}

// ######################################################################
Image<float> ScaleSpace::getKeypointImage(std::vector< rutz::shared_ptr<Keypoint> >& keypoints)
{
        Image<float> img(itsLumBlur[0].getDims(), ZEROS);

   int width = itsLumBlur[0].getWidth();
   int height = itsLumBlur[0].getHeight();

        //go over keypoints and set that value to 255
        std::vector<rutz::shared_ptr<Keypoint> >::const_iterator k = keypoints.begin(),
                stop = keypoints.end();
        while(k != stop)
        {
                float x = (*k)->getX();
                float y = (*k)->getY();
                const float s = (*k)->getS();
                // const float o = (*k)->getO();

      if (x > width) x = width-1;
      if (y > height) y = height-1;
      drawDisk(img, Point2D<int>(int(x), int(y)), int(s), 255.0F);
                //img.setVal(int(x), int(y), 255.0F);
      k++;
        }

        return img;

}

// ######################################################################
        /* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
