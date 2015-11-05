/*!@file ModelNeuron/PlotBuffer.C implementation for PlotBuffer*/

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/ModelNeuron/PlotBuffer.C$

#include "ModelNeuron/PlotBuffer.H"
#include "ModelNeuron/SimUnit.H"
#include "Image/DrawOps.H"
#include "Image/Transforms.H" //for composite()
#include "Util/StringConversions.H"

// ######################################################################
// ! implementation of PlotBuffer
// ######################################################################
nsu::PlotBuffer::PlotBuffer()
    : itsData(), itsExtraData(), names(), units(), itsXlabel("Samples"), 
      itsCount(0), itsChildCount(0), itsTotal(0), itsDepth(0)
{
}

// ######################################################################
void nsu::PlotBuffer::push(const SimUnit& nsm, const uint length, 
                           const uint depth, const bool usedisplayout)
{
  //if the depth has changed or we haven't initialized, do it now
  if ((itsTotal == 0) || (depth != itsDepth))
  {
    reset();
    getSubInfo(nsm, depth);
    ++itsTotal;
    itsData.resize(itsTotal);
    itsDepth = depth;
  }
  
  //recursively inspect the objects submodules and store the data
  addData(nsm, length, depth, usedisplayout);
}

// ######################################################################
void nsu::PlotBuffer::push(const SimUnit& nsm, const double& data, const uint length, 
                           const uint depth, const bool usedisplayout)
{
  if ((uint)itsExtraData.size() < length)
    itsExtraData.resize(length,0);
  
  push(nsm, length, depth, usedisplayout);
  itsExtraData.push_back(data);

  //chop any extra data off the buffer
  while ( ((uint)itsExtraData.size() > length) && (length != 0) )
    itsExtraData.pop_front();
}

// ######################################################################
void nsu::PlotBuffer::addData(const SimUnit& nsm, const uint length, 
                              const uint depth, const bool usedisplayout)
{
  if (depth > 0)
    for (int i =  nsm.numSubs()-1; i >=0; --i)
    {
      const SimUnit& temp = nsm.getSub(i);
      ++itsCount;
      //recursion hurts your head, but is so nice. 
      addData(temp, length, depth - 1, usedisplayout);
    }

  //base case
  if ((uint)itsData[itsChildCount].size() < length)
    itsData[itsChildCount].resize(length,0);

  //save our current value
  const double outval = usedisplayout ? nsm.getDisplayOutput() : nsm.getOutput();  
  itsData[itsChildCount].push_back(outval);

  //chop any extra data off the buffer
  while ( ((uint)itsData[itsChildCount].size() > length) && (length != 0) )
    itsData[itsChildCount].pop_front();

  if (itsCount > 0)
  {
    ++itsChildCount;
    --itsCount;
  }
  else
    itsChildCount = 0;
}

// ######################################################################
void nsu::PlotBuffer::reset()
{
  itsData.clear();
  itsExtraData.clear();
  itsCount = 0;
  itsChildCount = 0; 
  itsTotal = 0;
  itsDepth = 0;
}

// ######################################################################
void nsu::PlotBuffer::draw(Image<PixRGB<byte> >& img, const bool clear, const uint w, const uint h, const Range<double>& range)
{
  img = draw(clear, w, h, range).render(); 
}

// ######################################################################
Layout<PixRGB<byte> > nsu::PlotBuffer::draw(const bool clear, const uint w, const uint h, const Range<double>& range)
{
  Layout<PixRGB<byte> > layout;
  if ((itsData[0].size() > 1) && (itsTotal != 0))
  {
    uint height = h / (uint)itsData.size();
    std::vector<std::deque<double> >::reverse_iterator 
      iter(itsData.rbegin()), end(itsData.rend());
          
    std::vector<std::string>::const_reverse_iterator 
      nameiter(names.rbegin()), unititer(units.rbegin());
          
    if (itsExtraData.size() > 0)
    {
      Image<PixRGB<byte> > n = linePlot(*iter, w, height, range.min(), range.max(), nameiter->c_str(), unititer->c_str(), itsXlabel.c_str());      
      Image<PixRGB<byte> > d = linePlot(itsExtraData, w, height, range.min(), range.max(), "", " ", "", PixRGB<byte>(0,255,0), PixRGB<byte>(255,255,255), 0, true);
      layout = vcat(layout, composite(n, d, PixRGB<byte>(255,255,255)));
      ++iter; ++nameiter; ++unititer;      
    }
          
    while (iter != end)
    {
      layout = vcat(layout, linePlot(*iter, w, height, range.min(), range.max(), 
                                     nameiter->c_str(), unititer->c_str(), itsXlabel.c_str()));
      ++iter; ++nameiter; ++unititer;
    }

    if (clear)
    {
      itsData.clear();
      itsExtraData.clear();
    }
                
    return layout;
  }
  else
    return Layout<PixRGB<byte> > (Image<PixRGB<byte> >(w,h,ZEROS));
}

// ######################################################################
void nsu::PlotBuffer::getSubInfo(const SimUnit& nsm, const uint depth)
{
  if (depth > 0)
    for (int i = nsm.numSubs() - 1; i >= 0; --i)
    {
      const SimUnit& temp = nsm.getSub(i);
      ++itsTotal;
      this->getSubInfo(temp, depth - 1); //recursion
    }
  names.push_back(nsm.getName());
  units.push_back(nsm.getUnits());
}

// ######################################################################
void nsu::PlotBuffer::setSamplingRate(const SimTime& time)
{
  itsXlabel = nsu::PlotBuffer::SimTimeToSI(time);
}

// ######################################################################
const uint nsu::PlotBuffer::getTotal()
{
  return itsTotal;
}

// ######################################################################
std::string nsu::PlotBuffer::SimTimeToSI(const SimTime& time)
{
  std::string t(time.toString());
  int pos,epos;
  //for no decimal or exponent
  pos = t.find('.');
  epos = t.find('e');

  //for a decimal
  int magnitude = 0;
  std::string output;
  if (pos != (int)std::string::npos)
  {
    int cnt = pos + 1; 
    while (t[cnt] == '0')
      ++cnt;
    output = t.substr(cnt, t.size() - cnt - 1) + " ";
    magnitude = cnt - pos;
  }

  //for exponent
  else if (epos != (int)std::string::npos)
  {
    //get text before exponent
    output = t.substr(0,epos) + " ";
    std::string str = t.substr(epos+2,t.size() - 3 - epos);
    magnitude = fromStr<int>(str);
  }
  
  switch (magnitude)
  {
  case 1 :
    if (output.compare("1 ") == 0)//if the multuple is only 1, ignore the text
      output = "";
    output += "decisec";
    break;
  case 2 :
    if (output.compare("1 ") == 0)//if the multuple is only 1, ignore the text
      output = "";
    output += "centisec";
    break;
  case 3 :
    if (output.compare("1 ") == 0)//if the multuple is only 1, ignore the text
      output = "";
    output += "ms";
    break;
  case 4 :
    output = "." + output + "ms";
    break;
  case 5 :
    output = ".0" + output + "ms";
    break;
  case 6 :
    if (output.compare("1 ") == 0)//if the multuple is only 1, ignore the text
      output = "";
    output += "microsec";
    break;
  case 7 :
    output = "." + output + "microsec";
    break;
  case 8 :
    output = ".0" + output + "microsec";
    break;
  case 9 :
    if (output.compare("1 ") == 0)//if the multuple is only 1, ignore the text
      output = "";
    output += "nanosec";
    break;
  default:
    output = t.substr(0,t.size() - 1) + " sec";
    break;
  }
  return output;
}

// ######################################################################
// implementation of PlotBufferList
// ######################################################################
void nsu::PlotBufferList::push(const std::vector<const SimUnit*>& nsm, 
                               const uint length, const uint depth, 
                               const bool usedisplayout) 
{
  if (nsm.size() != itsPb.size())
  {
    itsPb.clear();
    itsPb.resize(nsm.size());
  }
  std::vector<const SimUnit*>::const_iterator begin(nsm.begin());
  std::vector<PlotBuffer>::iterator beginPb(itsPb.begin());
  while (begin != nsm.end())
  {
    beginPb->push(**begin, length, depth, usedisplayout);
    ++beginPb; ++begin;
  }
}

// ######################################################################
void nsu::PlotBufferList::push(const std::vector<const SimUnit*>& nsm, const double& data, 
                               const uint length, const uint depth, 
                               const bool usedisplayout) 
{
  if (nsm.size() != itsPb.size())
  {
    itsPb.clear();
    itsPb.resize(nsm.size());
  }
  std::vector<const SimUnit*>::const_iterator begin(nsm.begin());
  std::vector<PlotBuffer>::iterator beginPb(itsPb.begin());

  while (begin != nsm.end())
  {
    beginPb->push(**begin, data, length, depth, usedisplayout);
    ++beginPb; ++begin;
  }
}

// ######################################################################
void nsu::PlotBufferList::clear() 
{
  itsPb.clear(); 
}
 
// ######################################################################
Layout<PixRGB<byte> > nsu::PlotBufferList::draw(bool clear, const uint w, 
                                                const uint h, const Dims& d, const Range<double>& range) 
{
  if (itsPb.size() < 1)
  {
    return Layout<PixRGB<byte> >();
  }

  Layout<PixRGB<byte> > lay;
  if (d == Dims(0,1))
  {
    const uint height = h / itsPb.size();
    std::vector<PlotBuffer>::iterator begin(itsPb.begin());
    while (begin != itsPb.end())
      lay = vcat(lay, (begin++)->draw(clear, w, height, range));
  }  
  else if (d == Dims(1,0))
  {   
    const uint width = w / itsPb.size();
    std::vector<PlotBuffer>::iterator begin(itsPb.begin());
    while (begin != itsPb.end())
      lay = hcat(lay, (begin++)->draw(clear, width, h, range));
  }
  else
  {
    Layout<PixRGB<byte> > layr;
    const uint height = h / d.h();
    const uint width = w / d.w();
    std::vector<PlotBuffer>::iterator begin(itsPb.begin());
    for (int r = 0; (r < d.h()) || (begin == itsPb.end()); ++r)
    {
      for (int c = 0; (c < d.w()) || (begin == itsPb.end()); ++c)
        layr = hcat(layr, (begin++)->draw(clear, width, height, range)); 
      lay = vcat(lay,layr);
    }
  }
  return lay;
}

// ######################################################################
void nsu::PlotBufferList::draw(Image<PixRGB<byte> >& img, bool clear, 
                               const uint w, const uint h, const Dims& d, const Range<double>& range) 
{
  img = draw(clear, w, h, d, range).render();
}

// ######################################################################
void nsu::PlotBufferList::setSamplingRate(const SimTime& time)
{
  std::vector<PlotBuffer>::iterator iter(itsPb.begin());
  while (iter != itsPb.end())
    (iter++)->setSamplingRate(time);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
