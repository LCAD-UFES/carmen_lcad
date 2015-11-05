/** @file Psycho/ArrayCreator.C jittered array of search elements */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/ArrayCreator.C $

#ifndef PSYCHO_ARRAYCREATOR_C_DEFINED
#define PSYCHO_ARRAYCREATOR_C_DEFINED

#include "Psycho/ArrayCreator.H"

#include "Component/ModelOptionDef.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Image/DrawOps.H"

#include <iostream>
#include <fstream>

const ModelOptionCateg MOC_PSYCHOARRAYCREATOR = {
  MOC_SORTPRI_2, "ArrayCreator realted options" };

// Used by: ArrayCreator
const ModelOptionDef OPT_ACFileName =
  { MODOPT_ARG_STRING, "ACFileName", &MOC_PSYCHOARRAYCREATOR, OPTEXP_CORE,
    "File name of configuration file for ArrayCreator",
    "ac-filename", '\0', "<filename>", "array_config.conf" };

// Used by: ArrayCreator
const ModelOptionDef OPT_ACItemRadius =
  { MODOPT_ARG(uint), "ACItemRadius", &MOC_PSYCHOARRAYCREATOR,
    OPTEXP_CORE,
    "default radius to use for array items, in degrees of visual angle",
    "ac-itemradius", '\0', "<uint>", "2" };

// Used by: ArrayCreator
const ModelOptionDef OPT_ACJitterLevel =
  { MODOPT_ARG(float), "ACJitterLevel", &MOC_PSYCHOARRAYCREATOR,
    OPTEXP_CORE, "The default off-grid  jitter of the items, "
    "in degrees of visual angle", "ac-jitter", '\0', "<float>", "1" };

// Used by: ArrayCreator
const ModelOptionDef OPT_ACBackgroundColor =
  { MODOPT_ARG(PixRGB<byte>), "ACBackgroundColor", &MOC_PSYCHOARRAYCREATOR,
    OPTEXP_CORE, "value of the default background pixel, in RGB",
    "ac-bgcolor", '\0', "<byte,byte,byte>", "128,128,128" };

// Used by: ArrayCreator
const ModelOptionDef OPT_ACPpdX =
  { MODOPT_ARG(float), "ACPpdX", &MOC_PSYCHOARRAYCREATOR,
    OPTEXP_CORE, "pixels per degree for x direction",
    "ac-ppdx", '\0', "<float>", "10.0" };

// Used by: ArrayCreator
const ModelOptionDef OPT_ACPpdY =
  { MODOPT_ARG(float), "ACPpdY", &MOC_PSYCHOARRAYCREATOR,
    OPTEXP_CORE, "pixels per degree for y direction",
    "ac-ppdy", '\0', "<float>", "10.0" };

// Used by: ArrayCreator
const ModelOptionDef OPT_ACPermuteTargs =
  { MODOPT_FLAG, "ACPermuteTargs", &MOC_PSYCHOARRAYCREATOR,
    OPTEXP_CORE, "Display more than one target at a time?",
    "ac-perm-targs", '\0', "<bool>", "true" };


// ######################################################################
// implementation of for ItemType and ColorSpace
// ######################################################################
std::string convertToString(const ItemType& item)
{
  switch (item)
    {
    case BARH:
      return std::string("BARH");
    case BARV:
      return std::string("BARV");
    case CIRCLE:
      return std::string("CIRCLE");
    case SQUARE:
      return std::string("SQUARE");
    case BLOB:
      return std::string("BLOB");
    default:
      {
        LFATAL("No such item type");
        return std::string();
      }
    }
}

// ######################################################################
void convertFromString(const std::string& str,  ItemType& itemtype)
{
  if (str.compare("SQUARE") == 0)
    itemtype = SQUARE;
  else if (str.compare("BLOB") == 0)
    itemtype = BLOB;
  else if (str.compare("CIRCLE") == 0)
    itemtype = CIRCLE;
  else if (str.compare("BARH") == 0)
    itemtype = BARH;
  else if (str.compare("BARV") == 0)
    itemtype = BARV;
  else
    LFATAL("No such search array item option.");
}

// ######################################################################
std::string convertToString(const ColorSpace& colorspace)
{
  switch (colorspace)
    {
    case RGB:
      return std::string("RGB");
    case HSV:
      return std::string("HSV");
    case DKL:
      return std::string("DKL");
    default:
      {
        LFATAL("No ColorSpace set");
        return std::string();
      }
    }
}

// ######################################################################
void convertFromString(const std::string& str,  ColorSpace& itemtype)
{
  if (str.compare("HSV") == 0)
    itemtype = HSV;
  else if (str.compare("RGB") == 0)
    itemtype = RGB;
  else if (str.compare("DKL") == 0)
    itemtype = DKL;
  else
    LFATAL("No such color space option.");
}

// ######################################################################
// Implementation of ArrayItem and derivitives
// ######################################################################
  template <typename PIXEL_TYPE>
  ArrayItem::ArrayItem(const uint xpos, const uint ypos, const uint radius,
                       const PIXEL_TYPE& color, const float& orientation) :
    itsPos(xpos,ypos), itsRad(radius), itsCol(color), itsOrient(orientation)
  {
  }

// ######################################################################
ArrayItem::~ArrayItem()
{
}

// ######################################################################
std::string ArrayItem::toString() const
{
  std::string str = toStr<uint>(itsPos.x()) + ":" + toStr<uint>(itsPos.y()) +
    ":" + toStr<int>(itsRad) + ":" + toStr<PixRGB<byte> >(itsCol) + ":" +
    toStr<float>(itsOrient) + ":" + getShapeName();
  return str;
}

// ######################################################################
template <class PIXEL_TYPE>
BarItem::BarItem(const uint xpos, const uint ypos, const uint radius,
                 const PIXEL_TYPE& color, const float& orientation) :
  ArrayItem(xpos,ypos,radius,color,orientation)
{
}

// ######################################################################
BarItem::~BarItem()
{
}

// ######################################################################
std::string BarItem::getShapeName() const
{
  return "BAR";
}

// ######################################################################
void BarItem::drawItem(Image<PixRGB<byte> >& dst) const
{
  int tt = (int)(getPos().y() - getRad());
  int bb = (int)(getPos().y() + getRad());
  int ll = (int)((float)getPos().x() - getRad()/3.0f);
  int rr = (int)((float)getPos().x() + getRad()/3.0f);
  while ( ((bb - tt) > 0) && ((rr - ll) > 0) )
    {
      tt++; bb--; ll++; rr--;
      Rectangle r = Rectangle::tlbrO(tt, ll, bb, rr);
      drawRectOR(dst, r, getCol(), 1, getOrient());
    }
}

// ######################################################################
template <class PIXEL_TYPE>
CircleItem::CircleItem(const uint xpos, const uint ypos, const uint radius,
                       const PIXEL_TYPE& color, const float& orientation) :
  ArrayItem(xpos,ypos,radius,color,orientation)
{
}

// ######################################################################
CircleItem::~CircleItem()
{
}

// ######################################################################
std::string CircleItem::getShapeName() const
{
  return "CIRCLE";
}

// ######################################################################
void CircleItem::drawItem(Image<PixRGB<byte> >& dst) const
{
  drawDisk(dst, Point2D<int>((int)getPos().x(),(int)getPos().y()),
           getRad(), getCol());
}

// ######################################################################
template <class PIXEL_TYPE>
SquareItem::SquareItem(const uint xpos, const uint ypos, const uint radius,
                       const PIXEL_TYPE& color, const float& orientation) :
  ArrayItem(xpos,ypos,radius,color,orientation)
{
}

// ######################################################################
SquareItem::~SquareItem()
{
}

// ######################################################################
std::string SquareItem::getShapeName() const
{
  return "SQUARE";
}

// ######################################################################
void SquareItem::drawItem(Image<PixRGB<byte> >& dst) const
{
  drawPatch(dst, Point2D<int>((int)getPos().x(),(int)getPos().y()),
            getRad(), getCol());
}

// ######################################################################
template <class PIXEL_TYPE>
BlobItem::BlobItem(const uint xpos, const uint ypos, const uint radius,
                   const PIXEL_TYPE& color, const float& orientation) :
  ArrayItem(xpos,ypos,radius,color,orientation)
{
}

// ######################################################################
BlobItem::~BlobItem()
{
}

// ######################################################################
std::string BlobItem::getShapeName() const
{
  return "BLOB";
}

// ######################################################################
void BlobItem::drawItem(Image<PixRGB<byte> >& dst) const
{
  float stddev = (float)getRad()/2;
  int psize = stddev*2;

  int ux = int(getPos().x()) - psize;
  int step = dst.getWidth() - 2 * psize-1;

  Image<PixRGB<byte> >::iterator dptr = dst.beginw() +
    int(getPos().y() - psize) * dst.getWidth() + ux;

  for (int y = -1*psize; y <= psize; y++)
    {
      float resultX,resultY;
      resultY = exp(-(float(y*y)/(stddev*stddev)));
      for (int x = -1*psize; x <= psize; x++)
        {
          resultX = exp(-((x*x)/(stddev*stddev)));
          float g = resultX*resultY;

          //the current pixel will be g*col + (1-g)*oldcol
          PixRGB<byte> col = getCol();
          col *= g;
          *dptr *= (1-g);
          (*dptr++) += col;
        }
      dptr += step;
    }
}

// ######################################################################
// Implementation of ArrayCreator
// ######################################################################
ArrayCreator::ArrayCreator(const std::string& filename, const float& radius,
                           const float& noise, const Dims& dims,
                           const float& ppdx, const float& ppdy,
                           const PixRGB<byte>& background,
                           const float& fix_size, const bool permTargs) :
  itsRF(), itsColT(), itsColD(), itsDist(), itsShape(),
  itsStims(), itsDescr(), itsDims(dims), itsRad(radius), itsNoise(noise),
  itsPpdx(ppdx), itsPpdy(ppdy), itsPpd(sqrt(ppdx*ppdx + ppdy*ppdy)), 
  itsBackgroundColor(background), itsFixSize(fix_size), rnum((int)time(NULL)),
  itsPermTargs(permTargs)
{
  parseFile(filename);//read the config file
  computeSequence(); //compute the stim permutations, item positions,
                     //and descriptive string.
}

// ######################################################################
ArrayCreator::~ArrayCreator()
{
  for (size_t ii = 0; ii < itsStims.size(); ++ii)
    for (size_t jj = 0; jj < itsStims.size(); ++jj)
      delete itsStims[ii][jj];
}

// ######################################################################
uint ArrayCreator::size() const
{
  return (uint)itsStims.size();
}

// ######################################################################
Image<PixRGB<byte> > ArrayCreator::draw(const uint permnum) const
{
  Image<PixRGB<byte> > img(itsDims, ZEROS);
  img += itsBackgroundColor;
  draw(img, permnum);
  return img;
}

// ######################################################################
void ArrayCreator::draw(Image<PixRGB<byte> >& dst, const uint permnum) const
{
  if (dst.getDims() != itsDims)
    LFATAL("Image dimensions must be the same as those specified at "
           "this objects creation. ");

  std::vector<ArrayItem*> temp = itsStims[permnum];
  std::vector<ArrayItem*>::iterator iter = temp.begin();
  while (iter != temp.end())
    (*iter++)->drawItem(dst);

  if (itsFixSize > 0)
    {
      int size = int(itsFixSize * itsPpd);
      if ((size % 2) == 0)
        size = size / 2;
      else
        size = (size - 1) / 2;
      const int i = itsDims.w()/2-1,j = itsDims.h()/2-1;
      Rectangle frect = Rectangle::tlbrI(j-size+1, i-size+1,
                                         j + size, i + size);
      drawFilledRect(dst, frect, PixRGB<byte>(255,255,255));
    }
}

// ######################################################################
std::string ArrayCreator::toString(const uint permnum) const
{
  return itsDescr[permnum];
}

// ######################################################################
void ArrayCreator::parseFile(const std::string& file)
{
  //open file
  std::ifstream f(file.c_str());
  if (!f.is_open())
    LFATAL("Couldn't open '%s' for reading.",file.c_str());
  LINFO("%s opened for reading",file.c_str());

  //read in RF and other targt locations
  std::string line = readLine(f);
  while (line.find(':') == std::string::npos)
    {
      std::vector<std::string> tok;//tokenizer
      split(line, ",", std::back_inserter(tok));
      int xpix = int((float)fromStr<int>(tok[0]) * itsPpdx + itsDims.w()/2);
      int ypix = int((float)fromStr<int>(tok[1]) * itsPpdy + itsDims.h()/2);
      if (((int)xpix < itsDims.w()) && ((int)ypix < itsDims.h()) && 
          (xpix >=0) && (ypix >=0))
        {
          float size = fromStr<float>(tok[2]) * itsPpd;
          RF r = {geom::vec2<uint>((uint)xpix,(uint)ypix), size};
          itsRF.push_back(r);
        }
      line = readLine(f);
    }
  LINFO("%d RF's and other target locations",(int)itsRF.size());

  //parse the color list
  std::vector<std::string> tok;//tokenizer
  split(line, " ", std::back_inserter(tok));
  for (size_t ii = 0; ii < tok.size(); ++ii)
    {
      std::vector<std::string> tok1;//tokenizer
      split(tok[ii], ":", std::back_inserter(tok1));
      std::string type = tok1[0];
      ColorSpace cs = RGB;
      convertFromString(tok1[1], cs);
      switch (cs)
        {
        case RGB:
          {
            (type.compare("T") == 0)?
              itsColT.push_back(fromStr<PixRGB<byte> >(tok1[2])):
              itsColD.push_back(fromStr<PixRGB<byte> >(tok1[2]));
            break;
          }
        case HSV:
          {
            PixHSV<byte> thsv(fromStr<PixHSV<byte> >(tok1[2]));
            PixRGB<byte> tmp(thsv);
            (type.compare("T") == 0)?
              itsColT.push_back(tmp):
              itsColD.push_back(tmp);
            break;
          }
        case DKL:
          {
            LFATAL("DKL Colorspace options are currently unfunctional, "
                   "please use HSV, or RGB");
            //PixDKL<byte> tdkl(fromStr<PixDKL<byte> >(tok1[2]));
            //PixRGB<byte> tmp(tdkl); //DKL->RGB not supported
            break;
          }
        default:
          {
            LFATAL("No ColorSpace set");
            break;
          }
        }
    }

  LINFO("%d targets and %d distractors color conditions",
        (int)itsColT.size(), (int)itsColD.size());

  //get distractor density list
  line = readLine(f);
  tok.clear();
  split(line, " ", std::back_inserter(tok));
  for (size_t ii = 0; ii < tok.size(); ++ii)
    itsDist.push_back(fromStr<uint>(tok[ii]));
  LINFO("%d distractor set sizes",(int)itsDist.size());

  //get the shape list
  line = readLine(f);
  tok.clear();
  split(line, " ", std::back_inserter(tok));
  for (size_t ii = 0; ii < tok.size(); ++ii)
    {
      std::vector<std::string> ttok;//tokenizer
      split(tok[ii], "-", std::back_inserter(ttok));
      if (ttok.size() > 2)
        LFATAL("Cannot have more than two paired shapes");

      std::vector<ItemType> tempv;
      for (size_t jj = 0; jj < ttok.size(); ++jj)
        {
          ItemType it = SQUARE;
          convertFromString(ttok[jj], it);
          tempv.push_back(it);
        }
      itsShape.push_back(tempv);
    }
  LINFO("%d shape conditions", (int)itsShape.size());
  f.close();
}

// ######################################################################
std::string ArrayCreator::readLine(std::ifstream& fs)
{
  using namespace std;
  string line = "#";

  if (fs.eof())
    return line;

  while (line[0] == '#')
      getline(fs, line);

  return line;
}

// ######################################################################
void ArrayCreator::computeSequence()
{
  //Basically, here we are going to compute itsRF.size() choose ii
  //permutations (without repeats, ie 1-2 is the same subset as
  //2-1). We are going to select all the combinations of ii members of
  //the set with size itsRF.size(), and store them in one large
  //vector. This will give us all possible combintations of RF on/off
  //for all # of targets
  std::vector<std::vector<int> > idx; //index of on targets for each
                                      //condition.

  if (itsPermTargs)
    {
      std::vector<int> t;
      t.push_back(0);
      idx.push_back(t);
      subset(idx, (int)itsRF.size());
      t.resize(0);
      t.push_back(-1);
      idx.push_back(t);
    }
  else
    for (uint ii = 0; ii < itsRF.size(); ++ii)
      {
        std::vector<int> t;
        t.push_back(ii);
        idx.push_back(t);
      }

  //Now loop through all the conditions and compute target and
  //distractors for different color, shape and positions,
  //counterbalanced. See ArrayCreator.H for more info on the design.

  if (itsColT.size() > 2)
    LFATAL("currently only two target types are supported");

  //loop through all the distractor colors.
  for (uint ccd = 0; ccd < itsColD.size(); ++ccd)//colors distractor
    for (uint cct1 = 0; cct1 < itsColT.size(); ++cct1)//colors target
      for (uint cct2 = cct1; cct2 < itsColT.size(); ++cct2)//colors target1
        for (uint dd = 0; dd < itsDist.size(); ++dd)//distractor sets
          for (uint ss = 0; ss < itsShape.size(); ++ss)//shapes
            for (uint sst = 0; sst < itsShape[ss].size(); ++sst)//paired Targs
              for (uint ssd = 0; ssd < itsShape[ss].size(); ++ssd)//distractor
                if ((sst != ssd) || (itsShape[ss].size() == 1))
                  for (uint id = 0; id < idx.size(); ++id)//target on/of subset
                    for (uint tp = 0; tp < idx[id].size(); ++tp)//col targ pos
                      {
                        //continue if we are on a zero destractor set,
                        //and we are on the present nothing in RF -
                        //as nothing on the screen is not
                        //very interesting.
                        if ((itsDist[dd] == 0) && (idx[id][0] == -1))
                          continue;

                        //if we only have one thing on the screen, there
                        //is no reason to conterbalance colors, othewise
                        //we get repeats.
                        if ((idx[id].size() == 1) && (cct1 != cct2))
                          continue;

                        //if target and other targets are the same color
                        //there is no need have different target
                        //positions as it gives us repeats
                        if ((cct1 == cct2) && (tp > 0))
                          continue;

                        //get possible distractor locations, excluding
                        //ones near our on RF's for this trial.
                        std::vector<geom::vec2<uint> > grid = getGrid(idx[id]);

                        //choose a distractor set size of these points to be on
                        //by shuffle and resizing to the set size.
                        std::random_shuffle(grid.begin(), grid.end());
                        if (itsDist[dd] < grid.size())
                          grid.resize(itsDist[dd]);

                        std::vector<ArrayItem*> array;
                        std::string arraydescr, conddescr;

                        //distractor positions
                        for (uint ii = 0; ii < grid.size(); ++ii)
                          {
                            ArrayItem* ai = createItem(itsShape[ss][ssd],
                                                       grid[ii],
                                                       itsColD[ccd]);
                            array.push_back(ai);
                            arraydescr += "::D:" + ai->toString();
                          }


                        //Target positions
                        std::vector<int>::iterator iter = idx[id].begin();
                        uint otc(0), rfc(0);
                        while (iter != idx[id].end())
                          {
                            if (*iter == -1)
                              {
                                ++iter;
                                continue;
                              }

                            ArrayItem* ai = NULL;
                            if (*iter == idx[id][tp])
                              ai = createItem(itsShape[ss][sst],
                                              itsRF[*iter].pos,
                                              itsColT[cct1]);
                            else
                              ai = createItem(itsShape[ss][sst],
                                              itsRF[*iter].pos,
                                              itsColT[cct2]);

                            array.push_back(ai);
                            if (itsRF[*iter].size == 0)
                              {
                                arraydescr += "::OT:" + ai->toString();
                                ++otc;
                              }
                            else
                              {
                                arraydescr += "::RF:" + ai->toString();
                                ++rfc;
                              }
                            ++iter;
                          }

                        //ok lets add some description of the condition
                        //target count
                        conddescr += toStr<uint>(rfc) + ":";
                        //other target cound
                        conddescr += toStr<uint>(otc) + ":";
                        //target color
                        conddescr += toStr<PixRGB<byte> >(itsColT[cct1]) + ":";
                        //other target color
                        conddescr += toStr<PixRGB<byte> >(itsColT[cct2]) + ":";
                        //distractor color
                        conddescr += toStr<PixRGB<byte> >(itsColD[ccd]) + ":";
                        //setsize
                        conddescr += toStr<uint>(itsDist[dd]) + ":";
                        //target shape
                        conddescr += toStr<ItemType>(itsShape[ss][sst]) + ":";
                        //distractor shape
                        conddescr += toStr<ItemType>(itsShape[ss][ssd]);

                        //store array and description
                        itsDescr.push_back(conddescr + arraydescr);
                        itsStims.push_back(array);

                      }
}

// ######################################################################
void ArrayCreator::subset(std::vector<std::vector<int> >& sub, const uint N)
{
  uint tu = sub.back().back()+1;
  if (tu < N)
    {
      std::vector<int> tv = sub.back();
      tv.push_back(tu);
      sub.push_back(tv);
      this->subset(sub, N);
    }
  else
    {
      std::vector<int> tv = sub.back();
      if (tv.size() > 1)
        {
          tv.pop_back();
          tv.back()+=1;
          sub.push_back(tv);
          this->subset(sub, N);
        }
    }
}

// ######################################################################
std::vector<geom::vec2<uint> >
ArrayCreator::getGrid(const std::vector<int>& onpos)
{
  std::vector<int> pos;
  if (onpos[0] == -1)
    for (size_t ii = 0; ii < itsRF.size(); ++ii)
      pos.push_back((int)ii);
  else
    pos = onpos;

  //create a grid
  const int rad = int(itsRad * itsPpd + 0.5f);//conver to pixels
  const int noise = int(itsNoise * itsPpd + 0.5f);
  const int fxsize = int(itsFixSize * itsPpd + 0.5f);
  const int sz = rad*2 + noise;
  const int sz2 = rad*2 + 2*noise;

  Dims gsize(itsDims.w() - sz, itsDims.h() - sz);
  Dims nums(int((float)gsize.w() / (float)(sz2)),
            int((float)gsize.h() / (float)(sz2)));

  int tx = nums.w() * sz2;
  int ty = nums.h() * sz2;

  int lx = int(float(itsDims.w() - tx)/2.0f);
  int ly = int(float(itsDims.h() - ty)/2.0f);

  geom::vec2<int> c(itsDims.w()/2 - 1, itsDims.h()/2 - 1);
  std::vector<geom::vec2<uint> > grid;
  for (int xx = lx; xx <= itsDims.w() - lx; xx += sz2)
    for (int yy = ly; yy <= itsDims.h() - ly; yy += sz2)
      {
        geom::vec2<float> polar;
        polar.set_polar_rad(rnum.fdraw_range(0, noise),
                            rnum.fdraw_range(0,2*M_PI));
        geom::vec2<int> tv(xx + polar.x(), yy + polar.y());

        bool use = true;
        std::vector<int>::const_iterator op = pos.begin();
        while (op != pos.end())
          {
            geom::vec2<int> rf((int)itsRF[*op].pos.x(),
                               (int)itsRF[*op].pos.y());
            float d = tv.distance_to(rf);
            if (d <= rad*2)
              use = false;
            ++op;
          }

        float dc = 10000;
        if (fxsize > 0)
          dc = tv.distance_to(c);

        if (use && (dc > (rad + fxsize/2 + noise)))
          grid.push_back(geom::vec2<uint>((uint)tv.x(), (uint)tv.y()));
      }
  return grid;
}

// ######################################################################
ArrayItem* ArrayCreator::createItem(const ItemType item,
                                    const geom::vec2<uint>& pos,
                                    const PixRGB<byte>& color)
{
  const uint rad = uint(itsRad * itsPpd + 0.5f);
  switch (item)
    {
    case BARH:
      return new BarItem(pos.x(), pos.y(), rad, color, float(M_PI/2.0));
    case BARV:
      return new BarItem(pos.x(), pos.y(), rad, color, 0);
    case CIRCLE:
      return new CircleItem(pos.x(), pos.y(), rad, color, 0);
    case SQUARE:
      return new SquareItem(pos.x(), pos.y(), rad, color, 0);
    case BLOB:
      return new BlobItem(pos.x(), pos.y(), rad, color, 0);
    default:
      {
        LFATAL("No such item type");
        return NULL;
      }
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_ARRAYCREATOR_C_DEFINED
