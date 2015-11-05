/*!@file GUI/QtImageStack.Q */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/QtImageStack.Q $
// $Id: QtImageStack.Q 12269 2009-12-17 01:23:34Z itti $
//

#ifndef GUI_QTIMAGESTACK_Q_DEFINED
#define GUI_QTIMAGESTACK_Q_DEFINED

#ifdef INVT_HAVE_QT3

#include "GUI/QtImageStack.H"

#include "GUI/QtImageFrame.H"
#include "Raster/GenericFrame.H"
#include "Util/Pause.H"
#include "Util/log.H"

#include <qcheckbox.h>
#include <qlistbox.h>
#include <qsplitter.h>
#include <qwidgetstack.h>
#include <vector>

/* TODO:

   * change QtImageFrame to have toggles for resizing, zooming, etc.

 */

struct QtImageStack::Impl
{
  Impl()
    :
    pausebutton(0),
    listbox(0),
    widgstack(0),
    isClosed(false),
    frameNumber(0),
    entries(),
    preferredDims(),
    preferredMaxDims()
  {}

  struct Entry
  {
    Entry(QListBoxItem* listitem_,
          QtImageFrame* imageframe_,
          std::string name_)
      :
      listitem(listitem_),
      imageframe(imageframe_),
      name(name_)
    {}

    QListBoxItem* listitem;
    QtImageFrame* imageframe;
    std::string name;
  };

  Entry* findOrMakeEntry(const std::string& name)
  {
    // do we already have an Entry for this name?
    for (unsigned int i = 0; i < this->entries.size(); ++i)
      if (this->entries[i].name.compare(name) == 0)
        // found it:
        return &(this->entries[i]);

    // ok, no existing Entry, so let's set up a new one:

    QListBoxText* t = new QListBoxText(this->listbox, QString(name));
    this->listbox->sort();

    QtImageFrame* f = new QtImageFrame(this->widgstack,
                                       this->preferredDims,
                                       this->preferredMaxDims);
    this->widgstack->addWidget(f);

    if (this->entries.size() == 0)
      {
        this->listbox->setCurrentItem(t);
        this->widgstack->raiseWidget(f);
      }

    this->entries.push_back(Impl::Entry(t, f, name));

    return &(this->entries.back());
  }

  void removeEntry(const std::string& name)
  {
    for (unsigned int i = 0; i < this->entries.size(); ++i)
      if (this->entries[i].name.compare(name) == 0)
        // found it:
        {
          this->widgstack->removeWidget(this->entries[i].imageframe);
          delete this->entries[i].imageframe;
          delete this->entries[i].listitem;
          this->entries.erase(this->entries.begin() + i);
          return;
        }
  }

  QCheckBox* pausebutton;
  QListBox* listbox;
  QWidgetStack* widgstack;
  bool isClosed;
  int frameNumber;
  std::vector<Entry> entries;
  Dims preferredDims;
  Dims preferredMaxDims;
};

QtImageStack::QtImageStack(QWidget* parent)
  :
  QVBox(parent, "QtImageStack", (WFlags) 0),
  rep(new Impl)
{
  QHBox* buttons = new QHBox(this);
  rep->pausebutton = new QCheckBox(buttons);
  rep->pausebutton->setText("Pause (click to pause)");
  rep->pausebutton->setPaletteForegroundColor(QColor(96, 96, 0));
  QFrame* hspacer = new QFrame(buttons);
  buttons->setStretchFactor(rep->pausebutton, 0);
  buttons->setStretchFactor(hspacer, 1);

  connect(rep->pausebutton, SIGNAL(toggled(bool)),
          this, SLOT(togglePause(bool)));

  QSplitter* splitter = new QSplitter(this);
#if QT_VERSION >= 0x030200
  // setChildrenCollapsible() is not present before qt 3.2.0
  splitter->setChildrenCollapsible(false);
#endif
  rep->listbox = new QListBox(splitter);
  rep->widgstack = new QWidgetStack(splitter);

  this->setStretchFactor(buttons, 0);
  this->setStretchFactor(splitter, 1);

  connect(rep->listbox, SIGNAL(highlighted(QListBoxItem*)),
          this, SLOT(listSelection(QListBoxItem*)));
}

QtImageStack::~QtImageStack()
{
  delete rep;
}

void QtImageStack::addFrame(const GenericFrame& frame,
                            const std::string& name,
                            const FrameInfo& auxinfo)
{
  Impl::Entry* e = rep->findOrMakeEntry(name);

  ASSERT(e != 0);

  e->imageframe->setFrame(frame, name, rep->frameNumber, auxinfo);
}

void QtImageStack::removeFrame(const std::string& name)
{
  rep->removeEntry(name);
}

bool QtImageStack::setFrameNumber(int n)
{
  rep->frameNumber = n;

  return true;
}

bool QtImageStack::isClosed() const
{
  return rep->isClosed;
}

void QtImageStack::setPreferredDims(const Dims& dims)
{
  rep->preferredDims = dims;
}

void QtImageStack::setPreferredMaxDims(const Dims& dims)
{
  rep->preferredMaxDims = dims;
}

void QtImageStack::listSelection(QListBoxItem* item)
{
  for (unsigned int i = 0; i < rep->entries.size(); ++i)
    if (rep->entries[i].listitem == item)
      rep->widgstack->raiseWidget(rep->entries[i].imageframe);
}

void QtImageStack::togglePause(bool on)
{
  if (on)
    {
      rep->pausebutton->setText("Paused (click to resume)");
      rep->pausebutton->setPaletteForegroundColor(QColor(0, 96, 0));
      setPause(true);
    }
  else
    {
      rep->pausebutton->setText("Pause (click to pause)");
      rep->pausebutton->setPaletteForegroundColor(QColor(96, 96, 0));
      setPause(false);
    }
}

void QtImageStack::closeEvent(QCloseEvent* e)
{
  QVBox::closeEvent(e);
  rep->isClosed = true;
}

#endif // INVT_HAVE_QT3

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // GUI_QTIMAGESTACK_Q_DEFINED
