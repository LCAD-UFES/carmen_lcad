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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/QtDisplayStream.C $
// $Id: QtDisplayStream.C 12269 2009-12-17 01:23:34Z itti $
//

#ifndef GUI_QTDISPLAYSTREAM_C_DEFINED
#define GUI_QTDISPLAYSTREAM_C_DEFINED

#ifdef INVT_HAVE_QT3

#include "GUI/QtDisplayStream.H"

#include "Component/GlobalOpts.H"
#include "Component/ModelOptionDef.H"
#include "GUI/QtImageStack.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "QtUtil/ImageConvert.H"
#include "QtUtil/Util.H" // for argv2qt()
#include "Transport/FrameOstreamFactory.H"
#include "Transport/TransportOpts.H"
#include "Util/sformat.H"

#include <qapplication.h>
#include <qpixmap.h>

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

namespace
{
  QApplication* qapp = 0;

  pthread_once_t qapp_init_once = PTHREAD_ONCE_INIT;
  pthread_t qapp_thread;

  void* qapp_thread_run(void* app)
  {
    QApplication* a = static_cast<QApplication*>(app);

    //a->connect(a, SIGNAL(lastWindowClosed()),
    //           a, SLOT(quit()));

    const int result = a->exec();
    LDEBUG("QApplication exit status %d", result);
    return reinterpret_cast<void*>(result);
  }

  void qapp_thread_init()
  {
    ASSERT(qapp == 0);

    const char* dpy = getenv("DISPLAY");
    const bool havedpy = (dpy != 0 && dpy[0] != '\0');
    if (!havedpy)
      {
        if (setenv("DISPLAY", ":0.0", 1) == 0)
          LINFO("the DISPLAY environment variable is not set; "
                "assuming DISPLAY=\":0.0\"");
        else
          LFATAL("the DISPLAY environment variable is not set; "
                 "can't use Qt without a DISPLAY");
      }

    // NOTE, there is a QApplication constructor that takes a 'bool
    // usegui' parameter that specifies whether or not a DISPLAY is
    // available; however we can't just pass usegui=false here,
    // because we'd still need a way to avoid constructing any QWidget
    // objects

    int argc = 1;
    const char* argv[2] = { "QtDisplayStream", 0 };
    qapp = new QApplication(argc, argv2qt(argc, argv));

    if (0 != pthread_create(&qapp_thread, NULL, &qapp_thread_run, qapp))
      LFATAL("couldn't create thread for QApplication");
  }
}

// ######################################################################
class QAppLockClass
{
public:
  QAppLockClass(QApplication* qapp) : itsApp(qapp)
  {
    itsApp->lock();
  }

  ~QAppLockClass()
  {
    itsApp->unlock();
  }

private:
  QAppLockClass(const QAppLockClass&);
  QAppLockClass& operator=(const QAppLockClass&);

  QApplication* itsApp;
};

// ######################################################################
#define QAPP_LOCK(qapp) QAppLockClass anonymous_qapp_lock_(qapp)

// ######################################################################
QtDisplayStream::QtDisplayStream(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName)
  :
  FrameOstream(mgr, descrName, tagName),
  itsQtEcho(&OPT_QdisplayEcho, this),
  itsPreferredDims(&OPT_QdisplayPrefDims, this),
  itsPreferredMaxDims(&OPT_QdisplayPrefMaxDims, this),
  itsTestMode(&OPT_TestMode, this),
  itsWidget(0),
  itsShown(false),
  itsFrameNumber(-1),
  itsEcho()
{
  pthread_once(&qapp_init_once, &qapp_thread_init);

  ASSERT(qapp != 0);

  QAPP_LOCK(qapp);
  itsWidget = new QtImageStack;
}

// ######################################################################
QtDisplayStream::~QtDisplayStream()
{
  itsWidget->deleteLater();
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
          if (itsEcho.is_valid())
            this->removeSubComponent(*itsEcho);

          itsEcho = nub::soft_ref<FrameOstream>();
        }
      else
        {
          itsEcho = makeFrameOstream(itsQtEcho.getVal(),
                                     this->getManager());
          this->addSubComponent(itsEcho);
          itsEcho->exportOptions(MC_RECURSE);
        }
    }
  else if (param == &itsPreferredDims)
    {
      itsWidget->setPreferredDims(itsPreferredDims.getVal());
    }
  else if (param == &itsPreferredMaxDims)
    {
      itsWidget->setPreferredMaxDims(itsPreferredMaxDims.getVal());
    }
}

// ######################################################################
bool QtDisplayStream::setFrameNumber(int n)
{
  if (itsFrameNumber != n)
    {
      if (itsEcho.is_valid())
        {
          QAPP_LOCK(qapp);
          QPixmap pixmap = QPixmap::grabWidget(itsWidget);
          Image<PixRGB<byte> > img = convertToImage(pixmap);
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
  QAPP_LOCK(qapp);
  itsWidget->addFrame(frame, shortname, auxinfo);
  if (!itsShown && !itsTestMode.getVal())
    { itsShown = true; itsWidget->show(); }
}

// ######################################################################
bool QtDisplayStream::isVoid() const
{
  return itsWidget->isClosed();
}

// ######################################################################
void QtDisplayStream::closeStream(const std::string& shortname)
{
  QAPP_LOCK(qapp);
  itsWidget->removeFrame(shortname);
}

// ######################################################################
bool QtDisplayStream::isClosed() const
{
  return itsWidget->isClosed();
}

#endif // INVT_HAVE_QT3

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // GUI_QTDISPLAYSTREAM_C_DEFINED
