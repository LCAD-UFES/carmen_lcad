/*!@file GUI/PrefsWindow.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/PrefsWindow.H $
// $Id: PrefsWindow.H 8759 2007-09-06 19:24:13Z rjpeters $
//

#ifndef GUI_PREFSWINDOW_H_DEFINED
#define GUI_PREFSWINDOW_H_DEFINED

#include "Image/SimpleFont.H"
#include "Util/StringConversions.H"

#include <deque>
#include <vector>
#include <X11/Xlib.h>

class ModelComponent;
class ModelParamBase;
class XWinManaged;

class PrefItem;

/// A simple X window to allow online manipulation of preferences values
/** The window will show a list of your preference items. One of the
    items will always be the current item, highlighted by a different
    color.

    In "scrolling" mode, the default mode, you can do the following:
    - press up/down arrows to scroll to the previous/next item
    - press left/right arrows to change the current item's value by -1/+1
    - press < or > to change the current item's value by -10/+10
    - press return or enter to enter "editing" mode

    In "editing" mode, you can do the following:
    - enter a new value for the current item
    - press return or enter to accept the new value (or leave the
      current item's value unchanged if no new value was entered) and
      return to "scrolling" mode

    See PrefItem, PrefItemNum and PrefItemStr for useful preference
    item types that can be used with PrefsWindow.
 */
class PrefsWindow
{
public:

  /// Construct with a list of prefence descriptors
  PrefsWindow(const std::string& wintitle = "preferences",
              const SimpleFont& font = SimpleFont::FIXED(10));

  ~PrefsWindow();

  /// Specify the number of characters in the value editing area
  void setValueNumChars(int n);

  /// Change the window's font
  void setFont(const SimpleFont& font);

  /// Call this periodically to retrieve new values of the preferences
  void update();

  /// Add preference items corresponding to the editable params of comp
  void addPrefForParam(ModelParamBase* mp, ModelComponent* comp);

  /// Add preference items corresponding to the editable params of comp
  void addPrefsForComponent(ModelComponent* comp, bool recurse = false);

private:
  PrefsWindow(const PrefsWindow&);
  PrefsWindow& operator=(const PrefsWindow&);

  friend class PrefItem;

  void addItem(PrefItem* item, bool takeOwnership); // called by PrefItem's constructor

  bool handleKeysym(KeySym ks, const std::string& s);
  bool handleButtonPress(XButtonEvent* ev);

  void redraw();

  enum State { EDITING, SCROLLING };

  std::string itsWinTitle;
  XWinManaged* itsWin;
  SimpleFont itsFont;
  std::vector<PrefItem*> itsItems;
  std::vector<PrefItem*> itsOwnedItems;
  std::string itsEditBuffer;
  size_t itsCurrentItem;
  State itsState;
  int itsNameWidth;
  int itsNumWidth;
  bool itsDirty;
};

/// Base class for preference items to be used in a PrefsWindow
class PrefItem
{
public:
  PrefItem(PrefsWindow* pwin, const std::string& nm,
           bool pwinTakesOwnership = false);

  virtual ~PrefItem();

  virtual std::string getName() const;

  virtual void fromString(const std::string& s) = 0;

  virtual std::string toString() const = 0;

  virtual void adjust(int val) = 0;

  bool isValueChanged();

  virtual bool isDisabled() const = 0;

protected:
  std::string name;
  bool isChanged;
  bool wasDisabled;
};

/// Template class for numeric preference items
template <class T>
class PrefItemNum : public PrefItem
{
private:
  T val;
  bool disabled;

  template <class U>
  static void adjustHelper(U* val, int v)
  { *val += v; }

  static void adjustHelper(bool* val, int v)
  { *val = (int(*val) + v) % 2; }

public:
  PrefItemNum(PrefsWindow* pwin, const std::string& nm, T v,
              bool pwinTakesOwnership = false)
    : PrefItem(pwin, nm, pwinTakesOwnership), val(v), disabled(false) {}

  void set(T v) { if (this->val != v) { this->val = v; this->isChanged = true; } }
  T get() const { return this->val; }

  virtual void fromString(const std::string& s)
  {
    if (s.size() == 0) return; // ignore empty strings
    try { this->set(fromStr<T>(s)); }
    catch (conversion_error& e) {} // ignore error; leave value unchanged
  }

  virtual std::string toString() const
  { return toStr(this->val); }

  virtual void adjust(int v)
  { adjustHelper(&this->val, v); this->isChanged = true; }

  virtual bool isDisabled() const { return disabled; }

  void setDisabled(bool v) { if (v != disabled) { disabled = v; isChanged = true; } }
};

typedef PrefItemNum<double> PrefItemDbl;
typedef PrefItemNum<int>    PrefItemInt;
typedef PrefItemNum<uint>   PrefItemUin;
typedef PrefItemNum<byte>   PrefItemByt;
typedef PrefItemNum<bool>   PrefItemBln;

/// String preference item; includes value history
class PrefItemStr : public PrefItem
{
private:
  std::deque<std::string> itsHist;
  const size_t itsMaxSize;
  size_t itsPos;
  bool disabled;

public:
  PrefItemStr(PrefsWindow* pwin, const std::string& nm, const std::string& v,
              bool pwinTakesOwnership = false);

  void set(const std::string& v);

  std::string get() const;

  virtual std::string getName() const;

  virtual void fromString(const std::string& s);

  virtual std::string toString() const;

  /// Step through our value history by the specified amount
  virtual void adjust(int val);

  virtual bool isDisabled() const { return disabled; }

  void setDisabled(bool v) { if (v != disabled) { disabled = v; isChanged = true; } }
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // GUI_PREFSWINDOW_H_DEFINED
