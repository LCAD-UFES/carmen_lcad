/*!@file GUI/QtDisplayStream.C FrameOstream subclass that displays images in a QWidgetStack */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/QtDisplayStream4.C $
// $Id: QtDisplayStream4.C 15310 2012-06-01 02:29:24Z itti $
//

#include "GUI/QtDisplayStream4.H"

#include "Component/GlobalOpts.H"
#include "Component/ModelOptionDef.H"
#include "GUI/QtImageStack.qt.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "QtUtil/ImageConvert4.H"
#include "QtUtil/Util.H" // for argv2qt()
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstreamFactory.H"
#include "Transport/TransportOpts.H"
#include "Util/sformat.H"

#include <QtGui/QApplication>
#include <QtGui/QPixmap>
#include <QtCore/QReadWriteLock>
#include <unistd.h> // for usleep()

// Used by: QtDisplayStream
const ModelOptionDef OPT_QdisplayEcho =
  { MODOPT_ARG_STRING, "QdisplayEcho", &MOC_OUTPUT, OPTEXP_CORE,
    "Optional output destination for screen-grabs of the Qt display frames.",
    "qdisplay-echo", '\0', "<raster|display|mpeg|none>", "none" };

// Used by: QtDisplayStream
const ModelOptionDef OPT_QdisplayPrefDims =
  { MODOPT_ARG(Dims), "QdisplayPrefDims", &MOC_OUTPUT, OPTEXP_CORE,
    "Preferred scaled dimensions for Qt display frames, "
    "or 0x0 for no preference",
    "qdisplay-pref-dims", '\0', "<WxH>", "640x480" };

// Used by: QtDisplayStream
const ModelOptionDef OPT_QdisplayPrefMaxDims =
  { MODOPT_ARG(Dims), "QdisplayPrefMaxDims", &MOC_OUTPUT, OPTEXP_CORE,
    "If an image larger than these dimensions is sent to a Qt display "
    "frame, initially scale it down to smaller than these dimensions "
    "(you can always scale it up later by using the buttons on the "
    "Qt display widget",
    "qdisplay-pref-max-dims", '\0', "<WxH>", "1600x1200" };

#define EVT_WRITEFRAME (QEvent::User)
#define EVT_REMOVEFRAME ((QEvent::Type)(int(QEvent::User) + 1))

namespace
{
  // ######################################################################
  void *qapp_thread_run(void *c)
  {
    QtDisplayStream *d = reinterpret_cast<QtDisplayStream *>(c);
    return d->run();
  }

  // ######################################################################
  class QEventWriteFrame : public QEvent
  {
  public:
    QEventWriteFrame(const GenericFrame& fram, const std::string& shortnam, const FrameInfo& auxinf)
      : QEvent(EVT_WRITEFRAME), frame(fram), shortname(shortnam), auxinfo(auxinf) { }
    virtual ~QEventWriteFrame() { }

    const GenericFrame frame;
    const std::string shortname;
    const FrameInfo auxinfo;
  };

  // ######################################################################
  class QEventRemoveFrame : public QEvent
  {
  public:
    QEventRemoveFrame(const std::string& shortnam) : QEvent(EVT_REMOVEFRAME), shortname(shortnam) { }
    virtual ~QEventRemoveFrame() { }

    const std::string shortname;
  };
}

// ######################################################################
//! class to filter events
class EventFilt : public QObject {
public:
  EventFilt(QtDisplayStream *master) : m(master) { }

  virtual ~EventFilt() { }

protected:
  virtual bool eventFilter(QObject *obj, QEvent *event)
  {
    if (obj == m->itsWidget) {
      if (event->type() == EVT_WRITEFRAME)
        {
          QEventWriteFrame *e = static_cast<QEventWriteFrame*>(event);
          m->itsWidget->addFrame(e->frame, e->shortname, e->auxinfo);
          if (!m->itsShown && !m->itsTestMode.getVal()) { m->itsShown = true; m->itsWidget->show(); }
          return true;  // the event queue will delete the event
        }
      else if (event->type() == EVT_REMOVEFRAME)
        {
          QEventRemoveFrame *e = static_cast<QEventRemoveFrame*>(event);
          m->itsWidget->removeFrame(e->shortname);
          return true;  // the event queue will delete the event
        }
      else
        return false;
    }

    // else, pass the event on to the parent class:
    return QObject::eventFilter(obj, event);
  }

private:
  QtDisplayStream *m;
};


// ######################################################################
QtDisplayStream::QtDisplayStream(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName) :
  FrameOstream(mgr, descrName, tagName),
  itsQtEcho(&OPT_QdisplayEcho, this),
  itsPreferredDims(&OPT_QdisplayPrefDims, this),
  itsPreferredMaxDims(&OPT_QdisplayPrefMaxDims, this),
  itsTestMode(&OPT_TestMode, this),
  itsWidget(0), itsFilter(0),
  itsShown(false),
  itsFrameNumber(-1),
  itsEcho(), itsReady(false)
{ }

// ######################################################################
void QtDisplayStream::start1()
{
  // get our thread going and our app started:
  if (0 != pthread_create(&runner, NULL, &qapp_thread_run, this)) LFATAL("couldn't create thread for QApplication");

  while(itsReady == false) usleep(10000);
}

// ######################################################################
QtDisplayStream::~QtDisplayStream()
{
  itsWidget->deleteLater();
}

// ######################################################################
void* QtDisplayStream::run()
{
  // this runs in a thread dedicated to handling the display and events
  const char* dpy = getenv("DISPLAY");
  const bool havedpy = (dpy != 0 && dpy[0] != '\0');
  if (!havedpy) {
    if (setenv("DISPLAY", ":0.0", 1) == 0)  LINFO("DISPLAY environment variable not set; assuming DISPLAY=\":0.0\"");
    else LFATAL("DISPLAY environment variable not set; can't use Qt without a DISPLAY");
  }

  // create a QApplication:
  int argc = 1; const char* argv[1] = { "QtDisplayStream" };
  QApplication *a = new QApplication(argc, argv2qt(argc, argv));

  // Create our widget:
  itsWidget = new QtImageStack;
  itsWidget->setPreferredDims(itsPreferredDims.getVal());
  itsWidget->setPreferredMaxDims(itsPreferredMaxDims.getVal());

  // Install an event filter so that we get called (in this thread) when new images come in:
  itsFilter = new EventFilt(this);
  itsWidget->installEventFilter(itsFilter);

  // ready to rock:
  itsReady = true;

  // main loop for QApplication:
  const int result = a->exec();
  LDEBUG("QApplication exit status %d", result);
  return reinterpret_cast<void*>(result);
}

// ######################################################################
void QtDisplayStream::paramChanged(ModelParamBase* const param,
                                   const bool valueChanged,
                                   ParamClient::ChangeStatus* status)
{
  FrameOstream::paramChanged(param, valueChanged, status);

  if (param == &itsQtEcho && itsQtEcho.getVal().length() > 0)
    {
      if (itsQtEcho.getVal().compare("none") == 0)
        {
          if (itsEcho.is_valid()) this->removeSubComponent(*itsEcho);
          itsEcho = nub::soft_ref<FrameOstream>();
        }
      else
        {
          itsEcho = makeFrameOstream(itsQtEcho.getVal(), this->getManager());
          this->addSubComponent(itsEcho);
          itsEcho->exportOptions(MC_RECURSE);
        }
    }
}

// ######################################################################
bool QtDisplayStream::setFrameNumber(int n)
{
  if (itsFrameNumber != n)
    {
      if (itsEcho.is_valid())
        {
          QPixmap pixmap = QPixmap::grabWidget(itsWidget);
          Image<PixRGB<byte> > img = convertToImage4(pixmap);
          itsEcho->setFrameNumber(n);
          itsEcho->writeRGB(img, "qtecho");
        }

      itsFrameNumber = n;
    }

  return itsWidget->setFrameNumber(n);
}

// ######################################################################
void QtDisplayStream::writeFrame(const GenericFrame& frame,
                                 const std::string& shortname,
                                 const FrameInfo& auxinfo)
{
  QEventWriteFrame *event = new QEventWriteFrame(frame, shortname, auxinfo);
  QApplication::postEvent(itsWidget, event);
}

// ######################################################################
bool QtDisplayStream::isVoid() const
{
  return itsWidget->isClosed();
}

// ######################################################################
void QtDisplayStream::closeStream(const std::string& shortname)
{
  QEventRemoveFrame *event = new QEventRemoveFrame(shortname);
  QApplication::postEvent(itsWidget, event);
}

// ######################################################################
bool QtDisplayStream::isClosed() const
{
  return itsWidget->isClosed();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
