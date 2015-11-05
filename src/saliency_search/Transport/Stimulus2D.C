/*!@file Transport/Stimulus2D.C generate stimulus from file */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/Stimulus2D.C $

#include "Transport/Stimulus2D.H"
#include "Transport/TransportOpts.H"   // for MOC_INPUT
#include "Component/ModelOptionDef.H"
#include "Image/DrawOps.H"
#include "Image/Normalize.H"          // for FLOAT_NORM_PRESERVE
#include "Raster/GenericFrame.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/SimTime.H"
#include <fstream>

const ModelOptionDef OPT_Stimulus2DRadius =
  { MODOPT_ARG(int), "Stimulus2DRadius", &MOC_INPUT, OPTEXP_CORE,
    "The radius of the point to be drawn when reading from a stimulus "
    "2D file (.stim)",
    "stim2d-radius", '\0', "<integer>", "1" };

//######################################################################
/*!
Class to read a text file of  2d time varying input

File must have the following format:
Sampling rate
Image Size
X Y VALUE ....repeat for multiple locations
X Y VALUE ....
.
.

The first line is the Sampling period (eg .001s). The
second line is the image size (e.g. 640x480) Remaining lines are time
steps. The number of columns in each line will be a multiple of
three. Each triplit will represent the horizontal (integer), vertical
(integer) and intensity (float) of a stimulus. So two triplets, or six
columns, would represent 2 stimuli.
*/
// ######################################################################
Stimulus2D::Stimulus2D(OptionManager& mgr,
                       const std::string& descrName,
                       const std::string& tagName) :
  FrameIstream(mgr, descrName, tagName),
  itsRadius(&OPT_Stimulus2DRadius, this),
  itsStim(), itsSR(SimTime::ZERO()), itsT(SimTime::ZERO()), itsDims(0,0)
{ };

// ######################################################################
void Stimulus2D::setConfigInfo(const std::string& fileName)
{
  //load our file
  std::ifstream *itsFile;
  itsFile = new std::ifstream(fileName.c_str());

  //error if no file
  if (itsFile->is_open() == false)
    LFATAL("Cannot open '%s' for reading",fileName.c_str());

  //get the number of samples
  std::string line;
  int linenum = 0;
  while (!itsFile->eof())
    {
      getline(*itsFile, line);
      ++linenum;
    }
  itsFile->clear();
  itsFile->seekg(0,std::ios_base::beg);
  linenum -= 3;

  //get sampling rate line 1
  if (getline(*itsFile, line) < 0)//kill if empty
    LFATAL("Bad Format '%s'",fileName.c_str());
  itsSR = SimTime(SimTime::SECS(fromStr<double>(line)));
  LINFO("Period:%3.8f",itsSR.secs());

  //get screen size line 2
  if (getline(*itsFile, line) < 0)//kill if empty
    LFATAL("Bad Format '%s'",fileName.c_str());
  std::vector<std::string> tmptok;//tokenizer
  split(line, "x", std::back_inserter(tmptok));
  if (tmptok.size() < 2)//kill if screen size wrong format
    LFATAL("Bad Format '%s', Screen size", fileName.c_str());

  //set image output dims
  itsDims = Dims(fromStr<int>(tmptok[0]),
                 fromStr<int>(tmptok[1]));
  LINFO("Image Size: %dx%d",itsDims.w(),itsDims.h());

  std::streampos strpos = itsFile->tellg();
  //read our first line for the number of stims
  getline(*itsFile, line);
  // let's tokenize it:
  std::vector<std::string> tok;
  split(line," ", std::back_inserter(tok));
  //crash if wrong format
  if ((tok.size() % 3) != 0)
    LFATAL("Error parsing '%s', line %d only %d columns",
           fileName.c_str(), 3, (int)tok.size());

  //store number of stims and reset file position pointer
  const int numstims = tok.size();
  itsStim = Image<float>(numstims,linenum,ZEROS);
  itsFile->seekg(strpos);

  //loop until end counting lines
  linenum = 0;
  while (!itsFile->eof())
    {
      // get a line
      getline(*itsFile, line);

      // let's tokenize it:
      std::vector<std::string> strtok;
      split(line, " ", std::back_inserter(strtok));
      //crash if wrong format
      if ((tok.size() % 3) != 0)
        LFATAL("Error parsing '%s', line %d only %d columns",
               fileName.c_str(), 3, (int)strtok.size());

      //valid line, so lets grab our data
      for (size_t jj = 0; jj < strtok.size(); jj++)
        itsStim.setVal(jj,linenum,fromStr<float>(strtok[jj]));

      linenum++;
    }
  itsFile->close();
}

// ######################################################################
bool Stimulus2D::setFrameNumber(int n)
{
  return skipTo(SimTime::SECS(n * itsSR.secs()));
}

// ######################################################################
GenericFrameSpec Stimulus2D::peekFrameSpec()
{
  if (itsStim.initialized())
    {
      GenericFrameSpec fs;
      fs.nativeType = GenericFrame::GRAY_F32;
      fs.videoFormat = VIDFMT_AUTO;
      fs.videoByteSwap = false;
      fs.dims = getDims();
      fs.floatFlags = FLOAT_NORM_PRESERVE;
      fs.frameRate = getSR().hertz();
      return fs;
    }
  else
    {
      LFATAL("Don't be greedy, you can't peek the next frame until"
             " you've loaded a file.");
      return GenericFrameSpec();
    }
}

// ######################################################################
SimTime Stimulus2D::getNaturalFrameTime() const
{
  return getSR();
}

// ######################################################################
GenericFrame Stimulus2D::readFrame()
{
  return GenericFrame( next(itsRadius.getVal()), FLOAT_NORM_PRESERVE);
}

// ######################################################################
const SimTime Stimulus2D::getTime() const
{
  return itsT;
}

// ######################################################################
const SimTime Stimulus2D::getTotalTime() const
{
  double time = ((double)itsStim.getDims().h()-1) * itsSR.secs();
  SimTime t(SimTime::SECS(time));
  return t;
}

// ######################################################################
const SimTime Stimulus2D::getSR() const
{
  return itsSR;
}

// ######################################################################
const Dims Stimulus2D::getDims() const
{
  return itsDims;
}

// ######################################################################
Image<float> Stimulus2D::next(const int rad)
{
  if (itsStim.initialized())
    {
      Image<float> retImage(itsDims,ZEROS);
      int pos = int(itsT.secs() / itsSR.secs());
      for (int jj = 0; jj < itsStim.getDims().w();jj+=3){
        int x,y;
        float in = itsStim.getVal(jj,pos);
        x = (int)itsStim.getVal(jj+1,pos);
        y = (int)itsStim.getVal(jj+2,pos);
        drawDisk(retImage,Point2D<int>(x,y),rad,in);
      }
      itsT+=itsSR;
      return retImage;
    }
  else
    {
      LFATAL("Must call setInputFile before requesting any data!");
      return Image<float>();//will never execute.
    }
}

// ######################################################################
bool Stimulus2D::skipTo(const SimTime t)
{
  if (t > getTotalTime())
    return false;
  else
    itsT = t;
  return true;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
