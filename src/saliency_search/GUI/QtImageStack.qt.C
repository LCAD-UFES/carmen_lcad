/*!@file GUI/QtImageStack.qt.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/QtImageStack.qt.C $
// $Id: QtImageStack.qt.C 12962 2010-03-06 02:13:53Z irock $
//

#include "GUI/QtImageStack.qt.H"

#include "GUI/QtImageFrame.qt.H"
#include "Raster/GenericFrame.H"
#include "Util/Pause.H"
#include "Util/log.H"

#include <QtGui/QFrame>
#include <QtGui/QCloseEvent>
#include <QtGui/QCheckBox>
#include <QtGui/QListWidget>
#include <QtGui/QListWidgetItem>
#include <QtGui/QSplitter>
#include <QtGui/QStackedWidget>
#include <QtGui/QVBoxLayout>

#include <vector>

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

  // ######################################################################
  struct Entry
  {
    Entry(QListWidgetItem* listitem_, QtImageFrame* imageframe_, std::string name_) :
      listitem(listitem_), imageframe(imageframe_), name(name_)
    { }

    QListWidgetItem* listitem;
    QtImageFrame* imageframe;
    std::string name;
  };

  // ######################################################################
  Entry* findOrMakeEntry(const std::string& name)
  {
    // do we already have an Entry for this name?
    for (unsigned int i = 0; i < this->entries.size(); ++i)
      if (this->entries[i].name.compare(name) == 0) return &(this->entries[i]); // found it

    // ok, no existing Entry, so let's set up a new one:
    QListWidgetItem* t = new QListWidgetItem(QString(name.c_str()), this->listbox);
    ////this->listbox->sortItems();

    QtImageFrame* f = new QtImageFrame(this->widgstack, this->preferredDims, this->preferredMaxDims);
    this->widgstack->addWidget(f);

    if (this->entries.size() == 0)
      {
        this->listbox->setCurrentItem(t);
        this->widgstack->setCurrentWidget(f);
      }

    this->entries.push_back(Impl::Entry(t, f, name));

    return &(this->entries.back());
  }

  // ######################################################################
  void removeEntry(const std::string& name)
  {
    for (unsigned int i = 0; i < this->entries.size(); ++i)
      if (this->entries[i].name.compare(name) == 0) // found it
        {
          this->widgstack->removeWidget(this->entries[i].imageframe);
          delete this->entries[i].imageframe;
          delete this->entries[i].listitem;
          this->entries.erase(this->entries.begin() + i);
          return;
        }
  }

  QCheckBox* pausebutton;
  QListWidget* listbox;
  QStackedWidget* widgstack;
  bool isClosed;
  int frameNumber;
  std::vector<Entry> entries;
  Dims preferredDims;
  Dims preferredMaxDims;
};

// ######################################################################
QtImageStack::QtImageStack(QWidget* parent) :
  QWidget(parent), rep(new Impl)
{
  QVBoxLayout *main = new QVBoxLayout(this);
  main->setSpacing(4);

  QHBoxLayout* buttons = new QHBoxLayout;

  rep->pausebutton = new QCheckBox(this);
  rep->pausebutton->setText("Pause (click to pause)");
  connect(rep->pausebutton, SIGNAL(toggled(bool)), this, SLOT(togglePause(bool)));

  QPalette palette;
  palette.setColor(rep->pausebutton->foregroundRole(), QColor(96, 96, 0));
  rep->pausebutton->setPalette(palette);

  buttons->addWidget(rep->pausebutton);

  buttons->addStretch(1);

  main->addLayout(buttons);

  QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
  splitter->setChildrenCollapsible(false);

  rep->listbox = new QListWidget(this);
  rep->listbox->setMinimumWidth(150);
  splitter->addWidget(rep->listbox);

  rep->widgstack = new QStackedWidget(this);
  connect(rep->listbox, SIGNAL(currentRowChanged(int)), rep->widgstack, SLOT(setCurrentIndex(int)));
  splitter->addWidget(rep->widgstack);

  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 7);  // preferentially stretch the image pane over the text list pane


  main->addWidget(splitter);

  this->setLayout(main);
}

// ######################################################################
QtImageStack::~QtImageStack()
{
  delete rep;
}

// ######################################################################
void QtImageStack::addFrame(const GenericFrame& frame, const std::string& name, const FrameInfo& auxinfo)
{
  Impl::Entry* e = rep->findOrMakeEntry(name);

  ASSERT(e != 0);

  e->imageframe->setFrame(frame, name, rep->frameNumber, auxinfo);
}

// ######################################################################
void QtImageStack::removeFrame(const std::string& name)
{
  rep->removeEntry(name);
}

// ######################################################################
bool QtImageStack::setFrameNumber(int n)
{
  rep->frameNumber = n;
  return true;
}

// ######################################################################
bool QtImageStack::isClosed() const
{
  return rep->isClosed;
}

// ######################################################################
void QtImageStack::setPreferredDims(const Dims& dims)
{
  rep->preferredDims = dims;
}

// ######################################################################
void QtImageStack::setPreferredMaxDims(const Dims& dims)
{
  rep->preferredMaxDims = dims;
}

// ######################################################################
void QtImageStack::togglePause(bool on)
{
  if (on)
    {
      rep->pausebutton->setText("Paused (click to resume)");

      QPalette palette;
      palette.setColor(rep->pausebutton->foregroundRole(), QColor(0, 96, 0));
      rep->pausebutton->setPalette(palette);
      setPause(true);
    }
  else
    {
      rep->pausebutton->setText("Pause (click to pause)");
      QPalette palette;
      palette.setColor(rep->pausebutton->foregroundRole(), QColor(96, 96, 0));
      rep->pausebutton->setPalette(palette);
      setPause(false);
    }
}

// ######################################################################
void QtImageStack::closeEvent(QCloseEvent* e)
{
  QWidget::closeEvent(e);
  rep->isClosed = true;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
