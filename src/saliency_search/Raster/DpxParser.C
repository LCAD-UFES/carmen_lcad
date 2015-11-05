/*!@file Raster/DpxParser.C */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/DpxParser.C $
// $Id: DpxParser.C 9229 2008-02-07 02:31:25Z rjpeters $
//

#ifndef RASTER_DPXPARSER_C_DEFINED
#define RASTER_DPXPARSER_C_DEFINED

#include "Raster/DpxParser.H"

#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Image/Range.H"
#include "Raster/DpxFile.H"
#include "Raster/GenericFrame.H"

#include <vector>

namespace
{
  Image<PixRGB<float> > deBayer(const Image<float>& bayer)
  {
    Image<PixRGB<float> > result(bayer.getDims(), NO_INIT);

    const int w = bayer.getWidth();
    const int h = bayer.getHeight();

    ASSERT(w % 2 == 0);
    ASSERT(h % 2 == 0);

    Image<float>::const_iterator sptr = bayer.begin();
    Image<PixRGB<float> >::iterator dptr = result.beginw();

    for (int y = 0; y < h; y += 2)
      {
        for (int x = 0; x < w; x += 2)
          {
            if (x < 2 || x >= w-2 || y < 2 || y >= h-2)
              {
                const float r = sptr[0];
                const float g = 0.5f * (sptr[1] + sptr[w]);
                const float b = sptr[w+1];

                dptr[0].p[0] = r;
                dptr[1].p[0] = r;
                dptr[w].p[0] = r;
                dptr[w+1].p[0] = r;

                dptr[0].p[1] = g;
                dptr[1].p[1] = g;
                dptr[w].p[1] = g;
                dptr[w+1].p[1] = g;

                dptr[0].p[2] = b;
                dptr[1].p[2] = b;
                dptr[w].p[2] = b;
                dptr[w+1].p[2] = b;
              }
            else
              {
                /*

                  +------+------+------+------+
                  :-w-1  :-w    :-w+1  :-w+2  :
                  :      :      :      :      :
                  :    B :    G :    B :    G :
                  +------+======+======+------+
                  :-1    |0     |1     |2     :
                  :      |      |      |      :
                  :    G |    R |    G |    R :
                  +------+======+======+------+
                  :w-1   |w     |w+1   |w+2   :
                  :      |      |      |      :
                  :    B |    G |    B |    G :
                  +------+======+======+------+
                  :w*2-1 :w*2   :w*2+1 :w*2+2 :
                  :      :      :      :      :
                  :    G :    R :    G :    R :
                  +------+------+------+------+

                */

                dptr[0].p[0] = sptr[0];
                dptr[1].p[0] = 0.5f * (sptr[0] + sptr[2]);
                dptr[w].p[0] = 0.5f * (sptr[0] + sptr[w*2]);
                dptr[w+1].p[0] = 0.25f * (sptr[0] + sptr[2] + sptr[w*2] + sptr[w*2+2]);

                dptr[0].p[1] = 0.25f * (sptr[-w] + sptr[-1] + sptr[1] + sptr[w]);
                dptr[1].p[1] = 0.5f * sptr[1] + 0.125f * (sptr[-w] + sptr[-w+2] + sptr[w] + sptr[w+2]);
                dptr[w].p[1] = 0.5f * sptr[w] + 0.125f * (sptr[-1] + sptr[1] + sptr[w*2-1] + sptr[w*2+1]);
                dptr[w+1].p[1] = 0.25f * (sptr[1] + sptr[w] + sptr[w+2] + sptr[w*2+1]);

                dptr[0].p[2] = 0.25f * (sptr[-w-1] + sptr[-w+1] + sptr[w-1] + sptr[w+1]);
                dptr[1].p[2] = 0.5f * (sptr[w+1] + sptr[-w+1]);
                dptr[w].p[2] = 0.5f * (sptr[w+1] + sptr[w-1]);
                dptr[w+1].p[2] = sptr[w+1];
              }

            sptr += 2;
            dptr += 2;
          }

        sptr += w;
        dptr += w;
      }

    return result;
  }

  void showStats(const Image<uint16>& img, float sclip_lo, float sclip_hi)
  {
    const int w = img.getWidth();
    const int h = img.getHeight();

    const int w1 = w/4;
    const int w2 = (w*3)/4;
    const int h1 = h/4;
    const int h2 = (h*3)/4;

    const uint16* sptr = img.getArrayPtr();
    Range<uint16> srng;
    Range<uint16> srng_center;

    int total_lo_clips = 0;
    int total_hi_clips = 0;
    int center_lo_clips = 0;
    int center_hi_clips = 0;

    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
        {
          if (*sptr < sclip_lo)
            ++total_lo_clips;
          else if (*sptr > sclip_hi)
            ++total_hi_clips;

          srng.merge(*sptr);
          if (x >= w1 && x < w2 && y >= h1 && y < h2)
            {
              srng_center.merge(*sptr);
              if (*sptr < sclip_lo)
                ++center_lo_clips;
              else if (*sptr > sclip_hi)
                ++center_hi_clips;
            }
          ++sptr;
        }

    LINFO("srng = %u .. %u, srng[center] = %u .. %u, sclip_lo = %g [%g%%, %g%%], sclip_hi = %g [%g%%, %g%%]",
          srng.min(), srng.max(),
          srng_center.min(), srng_center.max(),
          sclip_lo, (100.0 * total_lo_clips) / (w*h), (100.0 * center_lo_clips) / (w*h/4),
          sclip_hi, (100.0 * total_hi_clips) / (w*h), (100.0 * center_hi_clips) / (w*h/4));
  }

  class ColorTable
  {
  public:
    ColorTable() {}

    void init(const size_t size,
              const float gamma,
              const bool do_log,
              const float sigm_contrast,
              const float sigm_thresh,
              const float sclip_lo,
              const float sclip_hi)
    {
      this->tab.resize(size);

      const float cmin = do_log ? logf(sclip_lo > 0.0f ? sclip_lo : 1.0f) : sclip_lo;
      const float crng = (do_log ? logf(sclip_hi) : sclip_hi) - cmin;

      const float sigmin = 1.0f / (1.0f + expf(sigm_contrast * sigm_thresh));
      const float sigrng = 1.0f / (1.0f + expf(sigm_contrast * (sigm_thresh-1.0f)))
        - sigmin;

      for (size_t i = 0; i < size; ++i)
        {
          float val = i;
          if (val < sclip_lo) val = sclip_lo;
          else if (val > sclip_hi) val = sclip_hi;

          if (do_log)
            val = logf(val > 0.0f ? val : 1.0f);

          val = (val - cmin) / crng;

          if (sigm_contrast != 0.0f)
            {
              val = 1.0f / (1.0f + expf(sigm_contrast * (sigm_thresh - val)));
              val = (val - sigmin) / sigrng;
            }

          val = 255.0f * powf(val, gamma);

          this->tab[i] = val;
        }
    }

    std::vector<float> tab;
  };

  float defaultGamma = 0.6f;
  float defaultSigmoidContrast = 10.0f;
  float defaultSigmoidThreshold = 0.1f;
  float defaultSrcClipLo = 0.0f;
  float defaultSrcClipHi = 5351.0f;
}

// ######################################################################
void DpxParser::setDefaultGamma(float val)            { defaultGamma = val; }
void DpxParser::setDefaultSigmoidContrast(float val)  { defaultSigmoidContrast = val; }
void DpxParser::setDefaultSigmoidThreshold(float val) { defaultSigmoidThreshold = val; }
void DpxParser::setDefaultSrcClipLo(float val)        { defaultSrcClipLo = val; }
void DpxParser::setDefaultSrcClipHi(float val)        { defaultSrcClipHi = val; }

// ######################################################################
DpxParser::DpxParser(const char* filename)
  :
  dpx(new DpxFile(filename)),
  itsGamma(defaultGamma),
  itsDoLog(false),
  itsSigmoidContrast(defaultSigmoidContrast),
  itsSigmoidThresh(defaultSigmoidThreshold),
  itsSclipLo(defaultSrcClipLo),
  itsSclipHi(defaultSrcClipHi)
{}

// ######################################################################
DpxParser::DpxParser(const char* filename,
                     float gamma,
                     bool do_log,
                     float sigm_contrast,
                     float sigm_thresh,
                     float sclip_lo,
                     float sclip_hi)
  :
  dpx(new DpxFile(filename)),
  itsGamma(gamma),
  itsDoLog(do_log),
  itsSigmoidContrast(sigm_contrast),
  itsSigmoidThresh(sigm_thresh),
  itsSclipLo(sclip_lo),
  itsSclipHi(sclip_hi)
{}

// ######################################################################
DpxParser::~DpxParser()
{
  delete dpx;
}

// ######################################################################
GenericFrameSpec DpxParser::getFrameSpec() const
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGB_F32;
  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = dpx->dims;
  result.floatFlags = 0;

  return result;
}

// ######################################################################
std::string DpxParser::getComments() const
{
  return std::string();
}

// ######################################################################
uint DpxParser::getTagCount() const
{
  return 0;
}

// ######################################################################
bool DpxParser::getTag(uint tag, std::string& name, std::string& value) const
{
  return false;
}

// ######################################################################
GenericFrame DpxParser::getFrame()
{
  ASSERT(dpx->ih.ImageElement[0].BitSize == 16);
  ASSERT(dpx->ih.ImageElement[0].EndOfLinePadding == 0xFFFF);

  showStats(dpx->rawimage, itsSclipLo, itsSclipHi);

  ColorTable ctab;
  ctab.init(1<<16, itsGamma, itsDoLog,
            itsSigmoidContrast, itsSigmoidThresh,
            itsSclipLo, itsSclipHi);

  Image<float> fresult(dpx->dims, NO_INIT);

  const uint16* sptr = dpx->rawimage.getArrayPtr();
  for (Image<float>::iterator fptr = fresult.beginw(),
         stop = fresult.endw(); fptr != stop; ++fptr, ++sptr)
    {
      *fptr = ctab.tab[*sptr];
    }

  const Image<PixRGB<float> > cimg = deBayer(fresult);

  return GenericFrame(cimg, /* flags = */ 0);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_DPXPARSER_C_DEFINED
