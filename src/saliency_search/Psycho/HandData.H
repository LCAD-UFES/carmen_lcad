/*!@file Psycho/HandData.H Struct for hand(joystick/steering wheel) data with a wrapped ParamMap */
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
// Primary maintainer for this file: Dicky Nauli Sihite <sihite@usc.edu>
// $HeadURL:
// $Id:

#ifndef PSYCHO_HANDDATA_H_DEFINED
#define PSYCHO_HANDDATA_H_DEFINED
#include <iostream>

#include "Image/Point2D.H"
//#include "Psycho/SaccadeState.H"
#include "Util/Types.H"
#include "Util/StringUtil.H"
#include "Util/log.H"
#include "rutz/compat_cmath.h" // for isnan()
#include "rutz/shared_ptr.h"
#include "Component/ParamMap.H"
#include <cmath>

//! Simple struct for hand movement data
/*! Different joystick/steering wheel trackers may or may not fill all data
    members. Members that should always be filled are x and y. Note that this
    class is costly in terms of storage, so typically one would want to use it
    only to pass data samples around, but not to store a long series of samples
    in memory. For example, Psycho/HandTrace.H uses a more compact format to
    internally store eye data, and uses HandData objects to deliver that data
    to whoever wants it. 
    Originally this function is for joystick only, therefore to maintain code
    consistency, the naming without mouse or keyboard will refer to joystick
*/
class HandData 
{
public:
  /*! HandData Constructors & Destructors */
  //{@
  //!!! Fully initialized constructor, no extra data

  // Joystick only
  HandData(const int x_, const int y_, const std::vector<bool> b_);
  // Mouse only
  HandData(const int mx_, const int my_, const int nmx_, const int nmy_,
           const bool mlb_, const bool mmb_, const bool mrb_);
  // Keyboard only
  HandData(const char* kbch_);

  // Keyboard + Mouse
  HandData(const char* kbch_,
           const int mx_, const int my_, const int nmx_, const int nmy_,
           const bool mlb_, const bool mmb_, const bool mrb_);
  // Keyboard + Mouse + Joystick = impossible?
  HandData(const char* kbch_,
           const int mx_, const int my_, const int nmx_, const int nmy_,
           const bool mlb_, const bool mmb_, const bool mrb_,
           const int x_, const int y_, const std::vector<bool> b_);

  //!!! Fully initialized constructor, extra data in ParamMap
  // Joystick only
  HandData(const int x_, const int y_, const std::vector<bool> b_,
           rutz::shared_ptr<ParamMap> dat);
  // Mouse only
  HandData(const int mx_, const int my_, const int nmx_, const int nmy_,
           const bool mlb_, const bool mmb_, const bool mrb_,
           rutz::shared_ptr<ParamMap> dat);
  // Keyboard only
  HandData(const char* kbch_, rutz::shared_ptr<ParamMap> dat);
  // Keyboard + Mouse
  HandData(const char* kbch_,
           const int mx_, const int my_, const int nmx_, const int nmy_,
           const bool mlb_, const bool mmb_, const bool mrb_,
           rutz::shared_ptr<ParamMap> dat);
  // Keyboard + Mouse + Joystick = impossible?
  HandData(const char* kbch_,
           const int mx_, const int my_, const int nmx_, const int nmy_,
           const bool mlb_, const bool mmb_, const bool mrb_,
           const int x_, const int y_, const std::vector<bool> b_,
           rutz::shared_ptr<ParamMap> dat);

  
  //! Destructor
  ~HandData();

  //@}
  // ######################################################################
  /*! @name Helper functions */
  //@{

  // _ ____ _   _ ____ ___ _ ____ _  _ 
  // | |  |  \_/  [__   |  | |    |_/  
  //_| |__|   |   ___]  |  | |___ | \_ 
  //

  //! Is the n-th button is pressed?
  bool isPressed(uint& b_) const;

  //! Is there any pressed button?
  bool isAnyPressed() const;

  //! Do we have buttons?
  bool isButtonEmpty() const;

  //! Do we have valid (x,y) coordinates?
  bool isValid() const;

  //! Are our coordinates within some dims?
  bool isWithin(const Dims& dims) const;

  /* _  _ ____ _  _ ____ ____ 
  ** |\/| |  | |  | [__  |___ 
  ** |  | |__| |__| ___] |___ 
  */
                           
  //! Is mouse position valid?
  bool isMouseValid() const;

  //! Is there any pressed mouse button?
  bool isMouseAnyPressed() const;

  /* _  _ ____ _   _ ___  ____ ____ ____ ___  
  ** |_/  |___  \_/  |__] |  | |__| |__/ |  \ 
  ** | \_ |___   |   |__] |__| |  | |  \ |__/ 
  */ 

  //! Is keyboard input empty?
  bool isKeyboardEmpty() const;

  /* _  _ ____ ___ ____ ___  ____ ___ ____ 
  ** |\/| |___  |  |__| |  \ |__|  |  |__| 
  ** |  | |___  |  |  | |__/ |  |  |  |  | 
  */

  //! Do we have valid extra metadata?
  /*! Only a very few samples in an eye trace have this extra data,
    possibly none. The data may be present at the onset of each
    event and contains information about each event. */
  bool hasMetaData(const std::string field) const;

  bool hasSpecialMetaData(const std::string field) const;

  //! Is this field name a special field?
  bool isSpecialField(std::string field) const;

  //@}

  // ######################################################################
  /*! @name Access functions */
  //@{

  //!-> If these features are not needed, consider removing them

  /*  _ ____ _   _ ____ ___ _ ____ _  _ 
  **  | |  |  \_/  [__   |  | |    |_/  
  ** _| |__|   |   ___]  |  | |___ | \_ 
  */

  //! Check number of buttons
  size_t numButton() const;

  /*! These are for single value of wheel's accl,brake,left,right */
  //! Get Accelerator value 
  int getAccelerator() const;    // (   0 ~ 127 )
  float getAcceleratorF() const; // ( 0.0 ~ 1.0 )

  //! Get Brake value
  int getBrake() const;    // (   0 ~ 128 )
  float getBrakeF() const; // ( 0.0 ~ 1.0 )

  //! Get Wheel Left value 
  int getWheelLeft() const;   // (   0 ~ 127 )
  float getWheelLeftF() const;// ( 0.0 ~ 1.0 )

  //! Get Wheel Right value 
  int getWheelRight() const;   // (   0 ~ 128 )
  float getWheelRightF() const;// ( 0.0 ~ 1.0 )


  /*! These are combined value of wheel's axises (accl-brake, left-right) */
  //! Get Wheel value 
  int getWheel() const;    // (    0 ~ 127 ~ 255 ) -->  127 left - 128 right
  float getWheelF() const; // ( -1.0 ~ 0.0 ~ 1.0 ) --> -1.0 left - 1.0 right
  int getX() const;        // Same as getWheel

  //! Get Both Accl & Brake value 
  int getAcclBrake() const;    // (    0 ~ 127 ~ 255 ) -->  127 accl - 128 brake
  float getAcclBrakeF() const; // ( -1.0 ~ 0.0 ~ 1.0 ) --> -1.0 left - 1.0 right
  int getY() const;            // Same as getAcclBrake

  /*! These are combined value of all wheel's axis */
  //! Get All joystick/wheel position 
  Point2D<int> getPosition() const; // ( 0 ~ 255 ) for all axis
  Point2D<float> getPositionF() const; // ( -1.0 ~ 1.0 ) for all axis

  /* _  _ ____ _  _ ____ ____ 
  ** |\/| |  | |  | [__  |___ 
  ** |  | |__| |__| ___] |___ 
  */

  //! Get raw data
  int getMouseX() const;
  int getMouseY() const;
  int getNativeX() const;
  int getNativeY() const;
  bool getMouseBL() const;
  bool getMouseBM() const;
  bool getMouseBR() const;

  /* _  _ ____ _   _ ___  ____ ____ ____ ___  
  ** |_/  |___  \_/  |__] |  | |__| |__/ |  \ 
  ** | \_ |___   |   |__] |__| |  | |  \ |__/ 
  */ 
  
  //! Get raw data
  char *getKeyboard() const;

  //@}

  // ######################################################################
  /*! @name Extra data management */
  //{@
  //! Get all metadata
  rutz::shared_ptr<ParamMap> getMetaData() const;
  
  //! Get list of metadata fields
  std::vector<std::string> getMetaDataList() const;

  //! Get one field of metadata
  double getMetaDataField(std::string field) const;

  //@}
  // ######################################################################
  /*! Misc */
  //! Get text output report
  void outputData(std::ostream &out) const;
  

  // ######################################################################

private:
  //! Joystick
  const int x, y;             // Joystick/steering wheel position
                              // for steering wheel, it has the following:
                              // x= 127, 0 left, 255 right
                              // y= 127, 0 accl, 255 brake
  const std::vector<bool> b;  // buttons of joystick/steering wheel

  //! Mouse
  const int mx, my; // Mouse X & Y
  const int nmx, nmy; // Native resolution
  const bool mlb, mmb, mrb; // The 3 mouse buttons

  //! Keyboard
  const char* kbch; // Stores the character inputted
  
  //Optional data in tree of ParamMaps
  const rutz::shared_ptr<ParamMap> itsExtraData;

  //! To add/remove special field
  std::string special(std::string field) const;
  std::string unspecial(std::string field) const;
};

// ######################################################################
// ######################################################################
// ##########               inlined functions                ############
// ######################################################################
// ######################################################################


// ######################################################################
/*! Constructors and Destructors*/

//!!! Without Param Map

// Joystick only
inline HandData::HandData(const int x_, const int y_,
                          const std::vector<bool> b_) :
  x(x_), y(y_), b(b_),
  mx(-1), my(-1), nmx(-1), nmy(-1), mlb(false), mmb(false), mrb(false),
  kbch("")
{ }

// Mouse only
inline HandData::HandData(const int mx_, const int my_,
                          const int nmx_, const int nmy_,
                          const bool mlb_, const bool mmb_, const bool mrb_) :
  x(-1), y(-1), b(),
  mx(mx_), my(my_), nmx(nmx_), nmy(nmy_), mlb(mlb_), mmb(mmb_), mrb(mrb_),
  kbch("")
{ }

// Keyboard only
inline HandData::HandData(const char* kbch_) :
  x(-1), y(-1), b(),
  mx(-1), my(-1), nmx(-1), nmy(-1), mlb(false), mmb(false), mrb(false),
  kbch(kbch_)
{ }

// Keyboard + Mouse
inline HandData::HandData(const char* kbch_,
                          const int mx_, const int my_,
                          const int nmx_, const int nmy_,
                          const bool mlb_, const bool mmb_, const bool mrb_) :
  x(-1), y(-1), b(),
  mx(mx_), my(my_), nmx(nmx_), nmy(nmy_), mlb(mlb_), mmb(mmb_), mrb(mrb_),
  kbch(kbch_)
{ }
// Keyboard + Mouse + Joystick = impossible?
inline HandData::HandData(const char* kbch_,
                          const int mx_, const int my_,
                          const int nmx_, const int nmy_,
                          const bool mlb_, const bool mmb_, const bool mrb_,
                          const int x_, const int y_,
                          const std::vector<bool> b_) :
  x(x_), y(y_), b(b_),
  mx(mx_), my(my_), nmx(nmx_), nmy(nmy_), mlb(mlb_), mmb(mmb_), mrb(mrb_),
  kbch(kbch_)
{ }

//!!! With param map

// Joystick only
inline HandData::HandData(const int x_, const int y_,
                          const std::vector<bool> b_,
                          rutz::shared_ptr<ParamMap> dat) :
  x(x_), y(y_), b(b_),
  mx(-1), my(-1), nmx(-1), nmy(-1), mlb(false), mmb(false), mrb(false),
  kbch("")
{ }

// Mouse only
inline HandData::HandData(const int mx_, const int my_,
                          const int nmx_, const int nmy_,
                          const bool mlb_, const bool mmb_, const bool mrb_,
                          rutz::shared_ptr<ParamMap> dat) :
  x(-1), y(-1), b(),
  mx(mx_), my(my_), nmx(nmx_), nmy(nmy_), mlb(mlb_), mmb(mmb_), mrb(mrb_),
  kbch("")
{ }

// Keyboard only
inline HandData::HandData(const char* kbch_, rutz::shared_ptr<ParamMap> dat) :
  x(-1), y(-1), b(),
  mx(-1), my(-1), nmx(-1), nmy(-1), mlb(false), mmb(false), mrb(false),
  kbch(kbch_)
{ }

// Keyboard + Mouse
inline HandData::HandData(const char* kbch_,
                          const int mx_, const int my_,
                          const int nmx_, const int nmy_,
                          const bool mlb_, const bool mmb_, const bool mrb_,
                          rutz::shared_ptr<ParamMap> dat) :
  x(-1), y(-1), b(),
  mx(mx_), my(my_), nmx(nmx_), nmy(nmy_), mlb(mlb_), mmb(mmb_), mrb(mrb_),
  kbch(kbch_)
{ }
// Keyboard + Mouse + Joystick = impossible?
inline HandData::HandData(const char* kbch_,
                          const int mx_, const int my_,
                          const int nmx_, const int nmy_,
                          const bool mlb_, const bool mmb_, const bool mrb_,
                          const int x_, const int y_,
                          const std::vector<bool> b_,
                          rutz::shared_ptr<ParamMap> dat) :
  x(x_), y(y_), b(b_),
  mx(mx_), my(my_), nmx(nmx_), nmy(nmy_), mlb(mlb_), mmb(mmb_), mrb(mrb_),
  kbch(kbch_)
{ }


inline HandData::~HandData()
{ }

// ######################################################################
/*! @name Helper Functions */

//! Joystick

inline bool HandData::isPressed(uint& b_) const
{
  try { // Check if the data is available and return it
    return b[b_];}
  catch (char *str) { // invalid data, just return false 
    // Should be LFATAL instead?
    LERROR("Data %d is out of range, we only have %d buttons",
           b_, int(b.size()));
    return false;}
}

inline bool HandData::isAnyPressed() const {
  for (uint i=0; i<b.size(); i++)
    if (b[i]) return true;
  return false;
}

inline bool HandData::isButtonEmpty() const {
  return b.empty(); }

inline bool HandData::isValid() const {
  return !(isnan(x) || isnan(y)) && x > -1 && y > -1; }

inline bool HandData::isWithin(const Dims& dims) const {
  if (isValid() == false) return false;
  return (x >= 0 && x < dims.w() && y >= 0 && y < dims.h());
}

//! Mouse

inline bool HandData::isMouseValid() const {
  return !(isnan(mx) || isnan(my)) && mx > -1 && my > -1;
}

inline bool HandData::isMouseAnyPressed() const {
  return (mlb || mmb || mrb);
}

//! Keyboard

inline bool HandData::isKeyboardEmpty() const {
  return (std::string(kbch).empty());
}

//! Metadata

inline bool HandData::hasMetaData(const std::string field = "any") const
{ if(field.compare("any") == 0) return itsExtraData.is_valid();
  else return (itsExtraData.is_valid() && 
               (itsExtraData->hasParam(field) ||
                itsExtraData->hasParam(special(field)) )); }

inline bool HandData::hasSpecialMetaData(const std::string field) const
{ 
  return (hasMetaData("any") && itsExtraData->hasParam(special(field)));    
}


inline bool HandData::isSpecialField(std::string field) const
{
   return field[0]=='*';
}

// ######################################################################
/*! Access Functions */

/* _ ____ _   _ ____ ___ _ ____ _  _ 
** | |  |  \_/  [__   |  | |    |_/  
**_| |__|   |   ___]  |  | |___ | \_ 
*/

inline size_t HandData::numButton() const {
  return b.size(); }

inline int HandData::getAccelerator() const {
  return (y<127)?(127-y):0; }
inline float HandData::getAcceleratorF() const {
  return (y<127)?(127-y)/127.0:0.0; }

inline int HandData::getBrake() const {
  return (y>127)?(y-127):0; }
inline float HandData::getBrakeF() const {
  return (y>127)?(y-127)/128.0:0; }

inline int HandData::getWheelLeft() const {
  return (x<127)?(127-x):0; }
inline float HandData::getWheelLeftF() const {
  return (x<127)?(127-x)/127.0:0.0; }

inline int HandData::getWheelRight() const {
  return (x>127)?(x-127):0; }
inline float HandData::getWheelRightF() const {
  return (x>127)?(x-127)/128.0:0; }

inline int HandData::getWheel() const {
  return x; }
inline float HandData::getWheelF() const {
  return (x*2.0/255.0) - 1.0; }
inline int HandData::getX() const {
  return x; }

inline int HandData::getAcclBrake() const {
  return y; }
inline float HandData::getAcclBrakeF() const {
  return (y*2.0/255.0) - 1.0; }
inline int HandData::getY() const {
  return y; }

inline Point2D<int> HandData::getPosition() const {
  return Point2D<int> (x,y); }
inline Point2D<float> HandData::getPositionF() const {
  return Point2D<float> (getWheelF(),getAcclBrakeF()); }

/* _  _ ____ _  _ ____ ____ 
** |\/| |  | |  | [__  |___ 
** |  | |__| |__| ___] |___ 
*/

inline int HandData::getMouseX() const {
  return mx;}
inline int HandData::getMouseY() const {
  return my;}
inline int HandData::getNativeX() const {
  return nmx;}
inline int HandData::getNativeY() const {
  return nmy;}
inline bool HandData::getMouseBL() const {
  return mlb;}
inline bool HandData::getMouseBM() const {
  return mmb;}
inline bool HandData::getMouseBR() const {
  return mrb;}

/* _  _ ____ _   _ ___  ____ ____ ____ ___  
** |_/  |___  \_/  |__] |  | |__| |__/ |  \ 
** | \_ |___   |   |__] |__| |  | |  \ |__/ 
*/ 

inline char * HandData::getKeyboard() const {
  return (char*)kbch;}



// ######################################################################
/*! Extra Data Management */
inline rutz::shared_ptr<ParamMap> HandData::getMetaData() const
{ return itsExtraData; }

inline std::vector<std::string> HandData::getMetaDataList() const 
{
  std::vector<std::string> argList;
  if(hasMetaData()) 
    for (ParamMap::key_iterator iter = getMetaData()->keys_begin();
         iter != getMetaData()->keys_end();
         ++iter)
      argList.push_back(*iter);
 
  return argList; }
    
inline double HandData::getMetaDataField(std::string field) const
{ 
  if (hasSpecialMetaData(field)) 
    return itsExtraData->getDoubleParam(special(field));
  else if(hasMetaData(field)) 
    return itsExtraData->getDoubleParam(field);
  else LFATAL("No data field %s in HandData",field.c_str());
  return -1;
}

inline void HandData::outputData(std::ostream &out) const
{ 
  out << "(x,y) = (" << x << "," << y << ")\t"<< "buttons =";
  for (uint i = 0; i < b.size(); i++)
    out << " " << b[i];
  out << "\n";
  if(hasMetaData())
    {
      std::vector<std::string> metaFields = getMetaDataList();
      for(std::vector<std::string>::iterator iter = metaFields.begin();
          iter != metaFields.end();
          ++iter) 
        out << *iter << " = " << getMetaDataField(*iter) << "\n";
    }
};

inline std::string HandData::special(std::string field) const
{
  if(field[0]=='*') return field;
  else return "*"+field;
}

inline std::string HandData::unspecial(std::string field) const
{
  if(field[0]=='*') return field.erase(0,1);
  else return field;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_HANDDATA_H_DEFINED
