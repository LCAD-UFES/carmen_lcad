/*!@file Transport/GameOfLifeInput.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/GameOfLifeInput.C $
// $Id: GameOfLifeInput.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef TRANSPORT_GAMEOFLIFEINPUT_C_DEFINED
#define TRANSPORT_GAMEOFLIFEINPUT_C_DEFINED

#include "Transport/GameOfLifeInput.H"

#include "Component/ModelOptionDef.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include "Raster/GenericFrame.H"
#include "Transport/TransportOpts.H"

namespace
{
  Image<byte> gameOfLifeUpdate(const Image<byte>& in,
                               int* nlive)
  {
    Image<byte> out(in.getDims(), ZEROS);

    Image<byte>::const_iterator sptr = in.begin();
    Image<byte>::iterator dptr = out.beginw();

    const int w = in.getWidth();
    const int h = in.getHeight();

    *nlive = 0;

    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
        {
          int nn = 0; // number of live neighbors

          if (y > 1)
            {
              if (x > 1   && sptr[-1-w] > 0) ++nn;
              if (           sptr[ 0-w] > 0) ++nn;
              if (x < w-1 && sptr[ 1-w] > 0) ++nn;
            }

          if (x > 1   && sptr[-1] > 0) ++nn;
          if (x < w-1 && sptr[ 1] > 0) ++nn;

          if (y < h-1)
            {
              if (x > 1   && sptr[-1+w] > 0) ++nn;
              if (           sptr[ 0+w] > 0) ++nn;
              if (x < w-1 && sptr[ 1+w] > 0) ++nn;
            }

          if (*sptr > 0) // cell is currently live
            {
              *dptr =
                (nn == 2 || nn == 3)
                ? 255 // stay alive
                : 0;  // die
            }
          else // cell is currently dead
            {
              *dptr =
                (nn == 3)
                ? 255 // come alive
                : 0;  // stay dead
            }

          if (*dptr > 0)
            ++*nlive;

          ++sptr;
          ++dptr;
        }

    return out;
  }
}

// Used by: GameOfLifeInput
const ModelOptionDef OPT_GameOfLifeFillFraction =
  { MODOPT_ARG(double), "GameOfLifeFillFraction", &MOC_INPUT, OPTEXP_CORE,
    "In the \"Game of Life\" (with --in=life), this is the fraction "
    "of cells to be filled as \"live\" when creating the initial "
    "random game board, and when re-seeding the board after it becomes "
    "stuck.",
    "life-fill-fraction", '\0', "float", "0.4" };

// Used by: GameOfLifeInput
const ModelOptionDef OPT_GameOfLifeCellSize =
  { MODOPT_ARG(uint), "GameOfLifeCellSize", &MOC_INPUT, OPTEXP_CORE,
    "In the \"Game of Life\" (with --in=life), this is the size, in "
    "pixels, of each cell.",
    "life-cell-size", '\0', "uint", "4" };

// ######################################################################
GameOfLifeInput::GameOfLifeInput(OptionManager& mgr)
  :
  FrameIstream(mgr, "Random Input", "GameOfLifeInput"),
  itsFillFraction(&OPT_GameOfLifeFillFraction, this),
  itsCellSize(&OPT_GameOfLifeCellSize, this),
  itsDims(320,240), // if you change this default value, also update
                    // the documentation of OPT_InputFrameSource in
                    // Media/MediaOpts.C
  itsGenerator(0),
  itsBoard(),
  itsStarted(false)
{
  itsGenerator.seed(time(NULL));
}

// ######################################################################
GameOfLifeInput::~GameOfLifeInput()
{}

// ######################################################################
void GameOfLifeInput::setConfigInfo(const std::string& dimsstring)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  if (dimsstring.size() == 0)
    return;

  Dims d; convertFromString(dimsstring, d);
  this->setDims(d);
}

// ######################################################################
GenericFrameSpec GameOfLifeInput::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGB_U8;
  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = itsDims;
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame GameOfLifeInput::readFrame()
{
  if (!itsStarted)
    {

#if 0

#define i 0
#define Q 255
      const byte gosper_glider_gun[] =
        {
          //    0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5
          /*0*/ i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,Q,i,i,i,i,i,i,i,i,i,i,i,
          /*1*/ i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,Q,i,Q,i,i,i,i,i,i,i,i,i,i,i,
          /*2*/ i,i,i,i,i,i,i,i,i,i,i,i,Q,Q,i,i,i,i,i,i,Q,Q,i,i,i,i,i,i,i,i,i,i,i,i,Q,Q,
          /*3*/ i,i,i,i,i,i,i,i,i,i,i,Q,i,i,i,Q,i,i,i,i,Q,Q,i,i,i,i,i,i,i,i,i,i,i,i,Q,Q,
          /*4*/ Q,Q,i,i,i,i,i,i,i,i,Q,i,i,i,i,i,Q,i,i,i,Q,Q,i,i,i,i,i,i,i,i,i,i,i,i,i,i,
          /*5*/ Q,Q,i,i,i,i,i,i,i,i,Q,i,i,i,Q,i,Q,Q,i,i,i,i,Q,i,Q,i,i,i,i,i,i,i,i,i,i,i,
          /*6*/ i,i,i,i,i,i,i,i,i,i,Q,i,i,i,i,i,Q,i,i,i,i,i,i,i,Q,i,i,i,i,i,i,i,i,i,i,i,
          /*7*/ i,i,i,i,i,i,i,i,i,i,i,Q,i,i,i,Q,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,
          /*8*/ i,i,i,i,i,i,i,i,i,i,i,i,Q,Q,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i
        };

      const byte block_engine_1[] =
        {
          //    0,1,2,3,4,5,6,7,
          /*0*/ i,i,i,i,i,Q,i,Q,
          /*1*/ i,i,i,i,i,Q,i,i,
          /*2*/ i,i,i,Q,i,i,i,i,
          /*3*/ i,Q,i,Q,i,i,i,i,
          /*4*/ Q,Q,i,Q,i,i,i,i,
          /*5*/ i,Q,i,i,i,i,i,i,
        };

      const byte block_engine_2[] =
        {
          //    0,1,2,3,4,
          /*0*/ Q,i,Q,i,Q,
          /*1*/ Q,i,Q,Q,i,
          /*2*/ Q,Q,i,i,i,
          /*3*/ i,i,i,i,Q,
          /*4*/ Q,i,Q,Q,Q,
        };

      const byte block_engine_3[] =
        {
          //    0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8
          /*0*/ Q,Q,Q,Q,Q,Q,Q,Q,i,Q,Q,Q,Q,Q,i,i,i,Q,Q,Q,i,i,i,i,i,i,Q,Q,Q,Q,Q,Q,Q,i,Q,Q,Q,Q,Q
        };

#undef i
#undef Q

#endif

      itsBoardHist.resize(0);
      itsBoard = Image<byte>(itsDims.w() / itsCellSize.getVal(),
                             itsDims.h() / itsCellSize.getVal(),
                             ZEROS);

#if 0
      if (0)
        const Image<byte> pattern(&gosper_glider_gun[0], 36, 9);
      if (0)
        const Image<byte> pattern(&block_engine_1[0], 8, 6);
      if (0)
        const Image<byte> pattern(&block_engine_2[0], 5, 5);
      if (0)
        const Image<byte> pattern(&block_engine_3[0], 39, 1);

      inplacePaste(itsBoard, pattern, Point2D<int>(60,60));
#endif

      const double t = itsFillFraction.getVal();

      for (Image<byte>::iterator
             p = itsBoard.beginw(), stop = itsBoard.endw();
           p != stop; ++p)
        *p =
          (itsGenerator.fdraw() < t)
          ? 255
          : 0;

      itsStarted = true;
    }
  else
    {
      int nlive = 0;
      itsBoard = gameOfLifeUpdate(itsBoard, &nlive);
      if (nlive == 0)
        // oops, everybody died, so let's restart on the next frame:
        itsStarted = false;
    }

  for (size_t i = 0; i  < itsBoardHist.size(); ++i)
    if (itsBoard == itsBoardHist[i])
      {
        // we've started cycling, so let's break the cycle and start
        // fresh on the next frame:
        LINFO("cycle detected with period %" ZU ,
              itsBoardHist.size() - i);
        itsStarted = false;
        break;
      }

  itsBoardHist.push_back(itsBoard);

  while (itsBoardHist.size() >= 8)
    itsBoardHist.pop_front();

  const Image<byte> result =
    rescale(itsBoard, itsDims, RESCALE_SIMPLE_NOINTERP);

  return GenericFrame(result);
}

// ######################################################################
void GameOfLifeInput::setDims(const Dims& s)
{
  itsDims = s;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_GAMEOFLIFEINPUT_C_DEFINED
