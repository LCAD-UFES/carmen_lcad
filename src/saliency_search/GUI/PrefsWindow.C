/*!@file GUI/PrefsWindow.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/PrefsWindow.C $
// $Id: PrefsWindow.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef GUI_PREFSWINDOW_C_DEFINED
#define GUI_PREFSWINDOW_C_DEFINED

#include "GUI/PrefsWindow.H"

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "GUI/XWinManaged.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/DrawOps.H"
#include "Util/log.H"
#include "Util/sformat.H"

#include <X11/keysym.h>

/// Template class for numeric preference items based on OModelParam or NModelParam
template <class MP, class T>
class PrefItemMPNum : public PrefItem
{
private:
  nub::ref<ModelComponent> comp; // hold a reference to the param's
                                 // owner so that the param won't be
                                 // destroyed until after we are
  MP* param;

  template <class U>
  static U adjustHelper(U val, int v)
  { return val + v; }

  static bool adjustHelper(bool val, int v)
  { return (int(val) + v) % 2; }

public:
  PrefItemMPNum(PrefsWindow* pwin, MP* p, ModelComponent* c,
                bool pwinTakesOwnership)
    : PrefItem(pwin, p->getNameWithSpaces().c_str(), pwinTakesOwnership),
      comp(c),
      param(p)
  {}

  void set(T v) { if (param->getVal() != v) { param->setVal(v); this->isChanged = true; } }
  T get() const { return param->getVal(); }

  virtual void fromString(const std::string& s)
  {
    if (s.size() == 0) return; // ignore empty strings
    const T oldval = param->getVal();
    try { param->setValString(s); }
    catch (std::exception& e) { REPORT_CURRENT_EXCEPTION; } // report+swallow error
    if (oldval != param->getVal()) this->isChanged = true;
  }

  virtual std::string toString() const
  { return param->getValString(); }

  virtual void adjust(int v)
  { this->set(adjustHelper(this->get(), v)); }

  virtual bool isDisabled() const
  { return param->getInactive(); }
};

/// Template class for non-numeric preference items based on OModelParam or NModelParam
class PrefItemMPStr : public PrefItem
{
private:
  nub::ref<ModelComponent> comp; // hold a reference to the param's
                                 // owner so that the param won't be
                                 // destroyed until after we are

  ModelParamBase* param;

public:
  PrefItemMPStr(PrefsWindow* pwin, ModelParamBase* p, ModelComponent* c,
                bool pwinTakesOwnership)
    : PrefItem(pwin, p->getNameWithSpaces().c_str(), pwinTakesOwnership),
      comp(c),
      param(p)
  {}

  virtual void fromString(const std::string& s)
  {
    if (s.size() == 0) return; // ignore empty strings
    const std::string oldval = param->getValString();
    try { param->setValString(s); }
    catch (std::exception& e) { REPORT_CURRENT_EXCEPTION; } // report+swallow error
    if (oldval != param->getValString()) this->isChanged = true;
  }

  virtual std::string toString() const
  { return param->getValString(); }

  virtual void adjust(int v)
  { /* do nothing; adjustments not supported for non-numeric params */ }

  virtual bool isDisabled() const
  { return param->getInactive(); }
};

// ######################################################################
PrefItem::PrefItem(PrefsWindow* pwin, const std::string& nm,
                   bool pwinTakesOwnership)
  :
  name(nm), isChanged(true), wasDisabled(false)
{
  if (pwin != 0)
    pwin->addItem(this, pwinTakesOwnership);
  else if (pwinTakesOwnership)
    // we can't have the PrefsWindow take ownership if we don't
    // actually have a PrefsWindow:
    LFATAL("I can't pass ownership of PrefItem '%s' "
           "to a null PrefsWindow*", nm.c_str());
}

// ######################################################################
PrefItem::~PrefItem()
{}

// ######################################################################
std::string PrefItem::getName() const
{
  return this->name;
}

// ######################################################################
bool PrefItem::isValueChanged()
{
  const bool ret =
    this->isChanged || (this->wasDisabled != this->isDisabled());
  this->isChanged = false;
  this->wasDisabled = this->isDisabled();
  return ret;
}

// ######################################################################
PrefItemStr::PrefItemStr(PrefsWindow* pwin, const std::string& nm,
                         const std::string& v,
                         const bool pwinTakesOwnership)
  : PrefItem(pwin, nm, pwinTakesOwnership),
    itsHist(), itsMaxSize(100), itsPos(0),
    disabled(false)
{
  itsHist.push_back(v);
  itsPos = itsHist.size() - 1;
}

// ######################################################################
void PrefItemStr::set(const std::string& v)
{
  ASSERT(itsPos < itsHist.size());
  if (itsHist[itsPos] != v)
    {
      if (itsHist.back() != v)
        {
          itsHist.push_back(v);
          while (itsHist.size() > itsMaxSize)
            itsHist.pop_front();
        }
      itsPos = itsHist.size() - 1;
      this->isChanged = true;
    }
}

// ######################################################################
std::string PrefItemStr::get() const
{
  ASSERT(itsPos < itsHist.size());
  return itsHist[itsPos];
}

// ######################################################################
std::string PrefItemStr::getName() const
{
  ASSERT(itsPos < itsHist.size());

  return sformat("%s#%02" ZU , this->name.c_str(), itsPos);
}

// ######################################################################
void PrefItemStr::fromString(const std::string& s)
{ this->set(s); }

// ######################################################################
std::string PrefItemStr::toString() const
{ return this->get(); }

// ######################################################################
void PrefItemStr::adjust(int val)
{
  ASSERT(itsHist.size() > 0);

  // adjust the current history position by val, wrapping around in
  // either direction as necessary

  if (val > 0)
    {
      itsPos += val;
      itsPos = itsPos % itsHist.size();
    }
  else if (val < 0)
    {
      const size_t sub = -val;
      for (size_t i = 0; i < sub; ++i)
        {
          if (itsPos == 0) itsPos = itsHist.size() - 1;
          else --itsPos;
        }
    }

  this->isChanged = true;
}

// ######################################################################
PrefsWindow::PrefsWindow(const std::string& wintitle,
                         const SimpleFont& font)
  :
  itsWinTitle(wintitle),
  itsWin(0),
  itsFont(font),
  itsItems(),
  itsEditBuffer(),
  itsCurrentItem(0),
  itsState(SCROLLING),
  itsNameWidth(1),
  itsNumWidth(10),
  itsDirty(true)
{}

// ######################################################################
PrefsWindow::~PrefsWindow()
{
  delete itsWin;

  while (itsOwnedItems.size() > 0)
    {
      delete itsOwnedItems.back();
      itsOwnedItems.pop_back();
    }
}

// ######################################################################
void PrefsWindow::setValueNumChars(const int n)
{
  if (n < 1)
    LFATAL("expected n to be at least 1, but got n=%d", n);

  if (n != itsNumWidth)
    {
      itsNumWidth = n;
      itsDirty = true;
    }
}

// ######################################################################
void PrefsWindow::setFont(const SimpleFont& font)
{
  if (font != itsFont)
    {
      itsDirty = true;
      itsFont = font;
    }
}

// ######################################################################
void PrefsWindow::update()
{
  if (itsItems.size() == 0)
    // don't show any window or try to handle any events if we don't
    // have any pref items
    return;

  if (itsWin == 0)
    {
      this->redraw();
      LINFO("Redraw");
      ASSERT(itsWin != 0);
      return;
    }

  for (size_t i = 0; i < itsItems.size(); ++i)
    {
      if (itsItems[i]->isValueChanged())
        {
          itsDirty = true;

          // don't break the loop early here even though we already
          // know that itsDirty will be set to true, because we want
          // to call isValueChanged() on every item so that we clear
          // the 'isChanged' field in ALL items that have changed,
          // rather than just the first one; otherwise, we will only
          // catch one change per update() and we will end up with a
          // series of multiple redraws when we could have done it all
          // in a single redraw call
        }
    }

  KeySym ks;
  std::string s;
  while ((ks = itsWin->getLastKeySym(&s)) != NoSymbol)
    {
      if (this->handleKeysym(ks, s))
        itsDirty = true;
    }

  XButtonEvent ev;
  while (itsWin->getLastButtonEvent(&ev))
    {
      if (this->handleButtonPress(&ev))
        itsDirty = true;
    }

  if (itsDirty)
    {
      this->redraw();
    }

  // postcondition for update() is that we leave ourselves in a
  // non-dirty state...
  ASSERT(itsDirty == false);
}

// ######################################################################
void PrefsWindow::addPrefForParam(ModelParamBase* mp, ModelComponent* comp)
{
  // OK, OK, we are relying on a bunch of ugly dynamic_cast<>s here;
  // but somehow we have to dig down and figure out what the
  // underlying numeric type is, in order to be able to support
  // numeric adjust() on the pref items that we create. A possible
  // alternative would be to just move the adjust() function into the
  // ModelParamBase class, in which case we wouldn't need to care
  // about the exact type here; we could just treat everything as a
  // ModelParamBase.

#define HANDLE_NUM_PARAM_TYPE(T)                                        \
      if (OModelParam<T>* p = dynamic_cast<OModelParam<T>*>(mp))        \
        {                                                               \
          new PrefItemMPNum<OModelParam<T>, T>                          \
            (this, p, comp,                                             \
             /* takeOwnership = */ true);                               \
          return;                                                       \
        }                                                               \
      if (NModelParam<T>* p = dynamic_cast<NModelParam<T>*>(mp))        \
        {                                                               \
          new PrefItemMPNum<NModelParam<T>, T>                          \
            (this, p, comp,                                             \
             /* takeOwnership = */ true);                               \
          return;                                                       \
        }

  HANDLE_NUM_PARAM_TYPE(int);
  HANDLE_NUM_PARAM_TYPE(float);
  HANDLE_NUM_PARAM_TYPE(double);
  HANDLE_NUM_PARAM_TYPE(unsigned int);
  HANDLE_NUM_PARAM_TYPE(long);
  HANDLE_NUM_PARAM_TYPE(unsigned long);
  HANDLE_NUM_PARAM_TYPE(byte);
  HANDLE_NUM_PARAM_TYPE(bool);

  // ok, the param doesn't correspond to a known numeric type, so
  // let's just treat it as a string param:
  new PrefItemMPStr(this, mp, comp, /* takeOwnership = */ true);

#undef HANDLE_NUM_PARAM_TYPE
}

// ######################################################################
void PrefsWindow::addPrefsForComponent(ModelComponent* comp, bool recurse)
{
  if (recurse)
    {
      const uint n = comp->numSubComp();
      for (uint i = 0; i < n; ++i)
        this->addPrefsForComponent(comp->subComponent(i).get(), recurse);
    }

  const size_t n = comp->getNumModelParams();
  for (size_t i = 0; i < n; ++i)
    {
      ModelParamBase* mp = comp->getModelParam(i);

      if (!mp->allowsOnlineChanges())
        continue;

      this->addPrefForParam(mp, comp);
    }
}

// ######################################################################
void PrefsWindow::addItem(PrefItem* item, bool takeOwnership)
{
  if (item->getName().length() > size_t(itsNameWidth))
    itsNameWidth = item->getName().length();

  itsItems.push_back(item);

  if (takeOwnership)
    itsOwnedItems.push_back(item);
}

// ######################################################################
bool PrefsWindow::handleKeysym(KeySym ks, const std::string& s)
{
  LDEBUG("keysym is %s", XKeysymToString(ks));

  bool dirty = true;

  if (itsState == EDITING)
    {
      switch (ks)
        {
        case XK_Return: case XK_KP_Enter:
          // try to accept the new value, then exit EDITING mode
          {
            if (itsEditBuffer.size() > 0)
              {
                itsItems[itsCurrentItem]->fromString(itsEditBuffer);
                itsEditBuffer = "";
              }

            itsState = SCROLLING;
          }
          break;

        case XK_Escape:
          // exit EDITING mode without changing the value
          {
            itsEditBuffer = "";
            itsState = SCROLLING;
          }
          break;

        case XK_Delete: case XK_BackSpace:
          // delete the last character in the current edit buffer
          {
            if (itsEditBuffer.size() > 0)
              {
                itsEditBuffer.resize(itsEditBuffer.size() - 1);
              }
          }
          break;

        default:
          {
             if (s.length() == 1 && isprint(s[0]))
              {
                itsEditBuffer += s[0];
              }
            else
              dirty = false;
          }
          break;
        }
    }
  else if (itsState == SCROLLING)
    {
      switch (ks)
        {
        case XK_Up:
          itsCurrentItem =
            (itsCurrentItem + itsItems.size() - 1) % itsItems.size();
          break;
        case XK_Down:
          itsCurrentItem =
            (itsCurrentItem + 1) % itsItems.size();
          break;

        case XK_Left:
          if (!itsItems[itsCurrentItem]->isDisabled())
            itsItems[itsCurrentItem]->adjust(-1);
          break;

        case XK_Right:
          if (!itsItems[itsCurrentItem]->isDisabled())
            itsItems[itsCurrentItem]->adjust(1);
          break;

        case XK_less:
          if (!itsItems[itsCurrentItem]->isDisabled())
            itsItems[itsCurrentItem]->adjust(-10);
          break;

        case XK_greater:
          if (!itsItems[itsCurrentItem]->isDisabled())
            itsItems[itsCurrentItem]->adjust(10);
          break;

        case XK_Return: case XK_KP_Enter:
          if (!itsItems[itsCurrentItem]->isDisabled())
            itsState = EDITING;
          break;

        default:
          dirty = false;
          break;
        }
    }

  return dirty;
}

// ######################################################################
bool PrefsWindow::handleButtonPress(XButtonEvent* ev)
{
  bool dirty = true;

  if (itsState == SCROLLING)
    {
      switch (ev->button)
        {
        case Button1:
          {
            const int n = (ev->y - 1) / (itsFont.h() + 2);
            itsCurrentItem = clampValue(n, 0, int(itsItems.size() - 1));
          }
          break;

        case Button2:
        case Button3:
          // do nothing
          dirty = false;
          break;

        case Button4: // mousewheel-up/scroll-up
          if (itsCurrentItem > 0)
            --itsCurrentItem;
          break;
        case Button5: // mousewheel-down/scroll-down
          if (itsItems.size() > 0
              && itsCurrentItem + 1 < itsItems.size())
            ++itsCurrentItem;
          break;
        }
    }

  return dirty;
}

// ######################################################################
void PrefsWindow::redraw()
{
  const Dims d((itsNameWidth + itsNumWidth + 10) * itsFont.w() + 2,
               2 + itsItems.size() * (itsFont.h() + 2));

  Image<PixRGB<byte> > img(d, ZEROS);

  for (size_t i = 0; i < itsItems.size(); ++i)
    {
      std::string line;
      PixRGB<byte> col;

      if (i == itsCurrentItem)
        {
          if (itsState == EDITING)
            {
              col = PixRGB<byte>(255, 0, 0);

              line = sformat(">> %*s ?? %-*s <<",
                             itsNameWidth, itsItems[i]->getName().c_str(),
                             itsNumWidth, itsEditBuffer.c_str());
            }
          else
            {
              col = itsItems[i]->isDisabled()
                ? PixRGB<byte>(127, 127, 127) : PixRGB<byte>(0, 255, 0);

              line = sformat(">> %*s -> %-*s <<",
                             itsNameWidth, itsItems[i]->getName().c_str(),
                             itsNumWidth, itsItems[i]->toString().c_str());
            }
        }
      else
        {
          col =
            itsState == EDITING
            ? PixRGB<byte>(64, 64, 64)
            : PixRGB<byte>(255, 255, 255);

          if (itsItems[i]->isDisabled())
            col /= 2;

          line = sformat("   %*s    %-*s   ",
                         itsNameWidth, itsItems[i]->getName().c_str(),
                         itsNumWidth, itsItems[i]->toString().c_str());
        }

      writeText(img, Point2D<int>(1, 1 + i*(itsFont.h()+2)),
                line.c_str(), col, PixRGB<byte>(0, 0, 0), itsFont);
    }

  if (itsWin == 0)
    itsWin = new XWinManaged(img.getDims(), -1, -1, itsWinTitle.c_str());
  else if (itsWin->getDims() != img.getDims())
    itsWin->setDims(img.getDims());

  ASSERT(itsWin->getDims() == img.getDims());

  itsWin->drawImage(img);

  itsDirty = false;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // GUI_PREFSWINDOW_C_DEFINED
