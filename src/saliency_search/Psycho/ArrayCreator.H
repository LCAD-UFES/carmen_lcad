/** @file Psycho/ArrayCreator.H create search arrays */

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
// Primary maintainer for this file: David J Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/ArrayCreator.H $


#ifndef PSYCHO_ARRAYCREATOR_H_DEFINED
#define PSYCHO_ARRAYCREATOR_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/vec2.h"
#include "rutz/rand.h"

#include <string>
#include <vector>

struct ModelOptionCateg;
struct ModelOptionDef;

// ######################################################################
//! Command line options that could be used with the ArrayCreator object
// ######################################################################
//@{

//! ArrayCreator options
extern const ModelOptionCateg MOC_PSYCHOARRAYCREATOR;

extern const ModelOptionDef OPT_ACFileName;
extern const ModelOptionDef OPT_ACItemRadius;
extern const ModelOptionDef OPT_ACJitterLevel;
extern const ModelOptionDef OPT_ACBackgroundColor;
extern const ModelOptionDef OPT_ACPpdX;
extern const ModelOptionDef OPT_ACPpdY;
extern const ModelOptionDef OPT_ACPermuteTargs;
//@}

// ######################################################################
// Some enumerated types for convenience. One for the item type -
// circle, bars, etc... and one for the colorspace - RGB, HSV,
// DKL
// ######################################################################
enum ItemType {BARV, BARH, CIRCLE, SQUARE, BLOB}; //different item types

//! Dims overload: format is "<ItemType>"
std::string convertToString(const ItemType& itemtype);

//! Dims overload: format is "<ItemType>"
void convertFromString(const std::string& str,  ItemType& itemtype);

enum ColorSpace {RGB, HSV, DKL}; //different color spaces

//! Dims overload: format is "<ColorSpace>"
std::string convertToString(const ColorSpace& colorspace);

//! Dims overload: format is "<ColorSpace>"
void convertFromString(const std::string& str,  ColorSpace& colorspace);

// ######################################################################
// ArrayItem: A template class to hold information about an array
// item, which contains information about shape, position (pixels),
// size(pixels), color, and orientation(degrees). The template
// parameter is the type of pixel.
// ######################################################################
class ArrayItem
{
public:
  //constructor for template type T, which must be a pixel type
  template <typename PIXEL_TYPE>
  ArrayItem(const uint xpos, const uint ypos, const uint radius,
            const PIXEL_TYPE& color, const float& orientation);

  //virtual destructor for propper inheritance
  virtual ~ArrayItem();

  //draw the item on an image
  virtual void drawItem(Image<PixRGB<byte> >& dst) const = 0;

  //get the name of the shape
  virtual std::string getShapeName() const = 0;

  //!display internal information as a string
  std::string toString() const;
protected:
  //accessor functions
  const geom::vec2<uint>& getPos() const {return itsPos;};
  const uint getRad() const {return itsRad;};
  const PixRGB<byte>& getCol() const {return itsCol;};
  const float& getOrient() const {return itsOrient;};

private:
  geom::vec2<uint> itsPos;//position
  uint itsRad; //radius in pixels of longest axis of item
  PixRGB<byte> itsCol;//color of shape
  float itsOrient; //orientation in degrees of longest axis
};

// ######################################################################
// Classes for different array item shapes. This includes oriented
// bar, filled circle, square, gaussian blob.
// ######################################################################
class BarItem : public ArrayItem
{
public:
  template <class PIXEL_TYPE>
  BarItem(const uint xpos, const uint ypos, const uint radius,
          const PIXEL_TYPE& color, const float& orientation);
  ~BarItem();
  std::string getShapeName() const;
  void drawItem(Image<PixRGB<byte> >& dst) const;
};

// ######################################################################
class CircleItem : public ArrayItem
{
public:
  template <class PIXEL_TYPE>
  CircleItem(const uint xpos, const uint ypos, const uint radius,
             const PIXEL_TYPE& color, const float& orientation);
  ~CircleItem();
  std::string getShapeName() const;
  void drawItem(Image<PixRGB<byte> >& dst) const;
};

// ######################################################################
class SquareItem : public ArrayItem
{
public:
  template <class PIXEL_TYPE>
  SquareItem(const uint xpos, const uint ypos, const uint radius,
             const PIXEL_TYPE& color, const float& orientation);
  ~SquareItem();
  std::string getShapeName() const;
  void drawItem(Image<PixRGB<byte> >& dst) const;
};

// ######################################################################
class BlobItem : public ArrayItem
{
public:
  template <class PIXEL_TYPE>
  BlobItem(const uint xpos, const uint ypos, const uint radius,
           const PIXEL_TYPE& color, const float& orientation);
  ~BlobItem();
  std::string getShapeName() const;
  void drawItem(Image<PixRGB<byte> >& dst) const;
};

// ######################################################################
// ArrayCreator: A class for creating search arrays where targets may
// appear in the receptive fields of multiple cells being recorded
// simultaneosly. All possible combinations of target in receptive
// field of 1, 2 ... N of the cells, and any additional conditions
// where the target is outside the RF (possibly with other targets in
// the cells RF). For each of these conditions we will perform
// counterbalanced presentations where the target and distractor are
// different colors (including intensity), density and shape. The
// distractor positions are randomized every trial. This parameters
// are stored in a text file, see parseFile below for the
// format. Radius and noise are in degrees of visual angle.
// ######################################################################
class ArrayCreator
{
public:
  //!constructor
  ArrayCreator(const std::string& filename, //config file name
               const float& radius,  // radius of stimuli in degrees
               const float& noise,  //maximum noise in degrees
               const Dims& dims, //dimensions of image in pixels
               const float& ppdx, //pixels per degree fo visual angle x
               const float& ppdy,//pixels per degree fo visual angle y 
               const PixRGB<byte>& background, //background color
               const float& fixation_size,
               const bool permTargs = true);

  //!destructor
  ~ArrayCreator();

  //!get the total number of stims
  uint size() const;

  //! draw the specified array in the permutation sequence
  Image<PixRGB<byte> > draw(const uint permnum) const;

  //! draw the specified array in the permutation sequence
  void draw(Image<PixRGB<byte> >& dst, const uint permnum) const;

  //! Write out our whole array as a string. First we will have a ':'
  //seperated list describing the condition, with the following
  //format: <#RF targets>:<#Other targets>:<target color>:<distractor
  //color>:<target shape>:<distractor shape>:<distractor set size>
  //
  //The next section will be seperated by a '::', and will contain
  //information about each item in the array. Items will be seperated
  //by '::'. The format goes like this: <item type (RF, OT,
  //D)>:<position x>:<position y>:<radius>:<rgb color>:<orientation>
  std::string toString(const uint permnum) const;

private:
  //!Parse a file to get receptive field positions, distractor
  //!densities, color, and shape conditions. The format goes like
  //!this:

  //1: The first lines should all be receptive fields locations where
  //targts are to be presented (or other locations you want targets to
  //be presented). A RF size of zero indicates that the position will
  //have a target, but that it is not actually a cell's RF. Although
  //this is not needed to create the conditions, it is useful to have
  //in the descriptive string. These will be counter balanced with all
  //possible subset combinations of RF's with/without target (see
  //computeSequence below). One line for each space seperated by ','
  //Ex "x,y,size"

  //2: Colors (could also be a contrast change of course) to be used
  //for targets and distractors in either RGB, HSV, or DKL
  //space. First put the type 'T' for target, and 'D' for
  //distractor. ':' and then the colorspace followed by a ':' and a
  //comma seperated list of color space components. Multiple color
  //conditions should be sperated by spaces. Target colors will
  //be counter balanced between the target on/of position. So that
  //every RF position will get to be a target of a given color, while
  //all other RF positions will be targets of a different color
  //Ex: "T:RGB:r,g,b T:RGB:r,g,b D:DKL:d,k,l"

  //3: Distractor set size in space seperated list. Ex 0 1 3 5 10

  //4: Shape conditions using the enum data type labels above. Pairs
  //can be defined by a '-' seperated list. Each pair will be counter
  //balanced so each one is a target and one a distractor. If pairs
  //are not used, the target and distractor will be rendered the same
  //shape in that condition. Seperate shape conditions are sperated by
  //spaces Ex "CIRCLE-SQUARE BLOB"
  void parseFile(const std::string& file);

  //!simple helper function to read lines without comments
  std::string readLine(std::ifstream& fs);

  //!Compute an internal list of all possible permutations of all
  //!conditions, counterbalanced.
  //
  //First, compute all the 'itsRF.size() choose ii' permutations
  //(without repeats, ie 1-2 = 2-1), for all ii = 0:itsRF.size(). We
  //are going to select all the combinations of ii members of the set
  //with size itsRF.size() and store them in one large vector. This
  //will give us all the possible combintations of RF on/off locations
  //for all RF's (and any other locations we added in the RF section
  //of the text file)
  //
  //Next, for all of these conditions compute all the possible target
  //color and target-distractor color combinations from the color
  //conditions list. If two targets are defined, then Next, add in all
  //the possible distractor density conditions. Finally, for the Shape
  //condition, target and distractor are only created with paired '-'
  //shapes from the config file. Otherwise, target and distractor will
  //be of the same shape.
  //
  void computeSequence();

  //!A recursive funcion to compute all the N choose K subsets
  //!(without repeats, ie consider 1,2 and 2,1 the same subset of
  //!N choose 2) For K = 1...N
  void subset(std::vector<std::vector<int> >& sub, const uint N);

  //Generate a list of points for possible distractors. Grid locations
  //closest to the targets are not considered. The grid is designed at
  //maximum density given the current radius.
  std::vector<geom::vec2<uint> > getGrid(const std::vector<int>& onpos);

  //create an array item from an ItemType
  ArrayItem* createItem(const ItemType item, const geom::vec2<uint>& pos,
                        const PixRGB<byte>& color);

  struct RF
  {
    geom::vec2<uint> pos; //position
    float size; //size in degrees
  };

  //to store infor about our different conditions from file.
  std::vector<RF> itsRF;
  std::vector<PixRGB<byte> > itsColT, itsColD;
  std::vector<uint> itsDist;
  std::vector<std::vector<ItemType> > itsShape;

  //store our list of created stims and informative strings
  std::vector<std::vector<ArrayItem*> > itsStims;
  std::vector<std::string> itsDescr;

  //for default values
  Dims itsDims;
  float itsRad, itsNoise, itsPpdx, itsPpdy, itsPpd;
  PixRGB<byte> itsBackgroundColor;
  float itsFixSize;
  rutz::urand rnum;
  bool itsPermTargs;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_ARRAYCREATOR_H_DEFINED
