/*!@file GUI/QtImageFrame.Q */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/QtImageFrame.Q $
// $Id: QtImageFrame.Q 14314 2010-12-16 00:02:44Z lior $
//

#ifndef GUI_QTIMAGEFRAME_Q_DEFINED
#define GUI_QTIMAGEFRAME_Q_DEFINED

#ifdef INVT_HAVE_QT3

#include "GUI/QtImageFrame.H"

#include "Image/ColorOps.H"
#include "Image/FilterOps.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/Normalize.H"
#include "Image/Pixels.H"
#include "Image/Range.H"
#include "Image/ShapeOps.H"
#include "QtUtil/ImageConvert.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Transport/FrameInfo.H"
#include "Util/MathFunctions.H"
#include "Util/sformat.H"
#include "rutz/demangle.h"
#include "rutz/trace.h"

#include <qerrormessage.h>
#include <qfiledialog.h>
#include <qlabel.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <typeinfo>

namespace
{
  const int MAXZOOM = 6;
  const int MINZOOM = -6;

  // escape any characters from the given string so that it will
  // render cleanly as Qt::RichText
  static std::string richTextEncode(const std::string& s)
  {
    std::string result;

    for (std::string::size_type i = 0; i < s.size(); ++i)
      {
        switch (s[i])
          {
          case '<': result += "&lt;"; break;
          default:  result += s[i];   break;
          }
      }

    return result;
  }

  static std::string buildTitle(const GenericFrame& img,
                                const std::string& title,
                                const int frameNumber,
                                const FrameInfo& auxinfo)
  {
    std::string result;

    if (auxinfo.description.length() > 0)
      result += sformat("<b>%s</b>", auxinfo.description.c_str());

    result += "<br>";

    result +=
      sformat("<b>%s</b> [frame #%06d]"
              "<br><i>%dx%d %s</i>",
              title.c_str(), frameNumber,
              img.getDims().w(), img.getDims().h(),
              richTextEncode(img.nativeTypeName()).c_str());

    result += "<br>";

    if (auxinfo.srcpos.m_file_name != 0)
      result += sformat("from %s:%d",
                        auxinfo.srcpos.m_file_name,
                        auxinfo.srcpos.m_line_no);

    result += "<br>";

    if (img.nativeType() == GenericFrame::GRAY_F32)
      {
        Range<float> r = rangeOf(img.asFloat());
        result += sformat("range=[%g .. %g]", r.min(), r.max());
      }
    else if (img.nativeType() == GenericFrame::RGB_F32)
      {
        float mi, ma;
        getMinMaxC(img.asRgbF32(), mi, ma);
        result += sformat("range=[%g .. %g]", mi, ma);
      }

    return result;
  }
}

struct QtImageFrame::Impl
{
  Impl()
    :
    title(0),
    zoom(0),
    frame(0),
    genFrame(),
    titlestring(),
    preferredDims(),
    preferredMaxDims(),
    doInitialDims(true),
    isUpdated(false)
  {}

  QLabel* title;
  QSpinBox* zoom;
  QFrame* frame;

  GenericFrame genFrame;
  std::string shortname;
  std::string titlestring;
  Dims preferredDims;
  Dims preferredMaxDims;
  bool doInitialDims;

  bool isUpdated;

  Dims rawDims() const
  {
    return this->genFrame.getDims();
  }

  Dims scaledDims() const
  {
    const Dims raw = this->rawDims();

    if (raw.isEmpty())
      return raw;

    else if (this->zoom->value() >= 0)
      return Dims(raw.w() << this->zoom->value(),
                  raw.h() << this->zoom->value());

    // else...
    return Dims(raw.w() >> -(this->zoom->value()),
                raw.h() >> -(this->zoom->value()));
  }

  void setInitialDims()
  {
    if (this->doInitialDims)
      {
        if (this->preferredDims.isNonEmpty())
          {
            Dims dims = this->scaledDims();

            while (dims.w() < this->preferredDims.w() &&
                   dims.h() < this->preferredDims.h() &&
                   this->zoom->value() < this->zoom->maxValue())
              {
                this->zoom->blockSignals(true);
                this->zoom->stepUp();
                this->zoom->blockSignals(false);
                dims = this->scaledDims();
              }
          }

        if (this->preferredMaxDims.isNonEmpty())
          {
            Dims dims = this->scaledDims();

            while (dims.w() > this->preferredMaxDims.w() ||
                   dims.h() > this->preferredMaxDims.h())
              {
                this->zoom->blockSignals(true);
                this->zoom->stepDown();
                this->zoom->blockSignals(false);
                dims = this->scaledDims();
              }
          }

        this->doInitialDims = false;
      }
  }

  void updateSize()
  {
    this->setInitialDims();

    const Dims dims = this->scaledDims();

    if (dims.isNonEmpty())
      this->frame->setFixedSize(dims.w(), dims.h());
  }

  template <class T>
  Image<T> scaleImage(const Image<T>& img)
  {
    if (this->zoom->value() >= 0)
      return zoomXY(img, (1 << this->zoom->value()), (1 << this->zoom->value()));
    else
      {
        Image<T> result = img;
        for (int i = 0; i < -this->zoom->value(); ++i)
          result = decXY(lowPass3(result));
        return result;
      }
  }

  void update()
  {
    if (this->isUpdated)
      return;

    GVX_TRACE(__PRETTY_FUNCTION__);

    this->setInitialDims();

    QPixmap pixmap;

    switch (this->genFrame.nativeType())
      {
      case GenericFrame::NONE:
        break;

      case GenericFrame::RGB_U8:
      case GenericFrame::RGB_F32:
      case GenericFrame::VIDEO:
        pixmap = convertToQPixmap(scaleImage(this->genFrame.asRgbU8()));
        break;

      case GenericFrame::GRAY_U8:
      case GenericFrame::GRAY_F32:
        pixmap = convertToQPixmap(scaleImage(this->genFrame.asGrayU8()));
        break;

      case GenericFrame::RGB_U16:
        break;
      case GenericFrame::GRAY_U16:
        break;
      case GenericFrame::RGBD:
        break;
      }

    this->title->setText(titlestring.c_str());
    this->title->setTextFormat(Qt::RichText);
    this->title->setAlignment(Qt::AlignAuto | Qt::AlignVCenter |
                              ExpandTabs | Qt::SingleLine);
    this->frame->setPaletteBackgroundPixmap(pixmap);

    this->updateSize();

    this->isUpdated = true;
  }
};

QtImageFrame::QtImageFrame(QWidget* parent, const Dims& preferredDims,
                           const Dims& preferredMaxDims)
  :
  QVBox(parent, "QtImageFrame", (WFlags) 0),
  rep(new Impl)
{
  this->setFrameShape(QFrame::Panel);
  this->setFrameShadow(QFrame::Sunken);
  this->setLineWidth(2);
  this->setMargin(2);

  rep->preferredDims = preferredDims;
  rep->preferredMaxDims = preferredMaxDims;

  QVBox* header = new QVBox(this);
  rep->title = new QLabel(header);
  QHBox* spingroup = new QHBox(header);
  QLabel* spinlabel = new QLabel("zoom:", spingroup);
  rep->zoom = new QSpinBox(MINZOOM, MAXZOOM, 1, spingroup);
  rep->zoom->setButtonSymbols(QSpinBox::PlusMinus);
  rep->zoom->setValue(0);
  rep->zoom->setPrefix("pow(2, ");
  rep->zoom->setSuffix(")");
  QFrame* hdummy = new QFrame(spingroup);
  spingroup->setStretchFactor(spinlabel, 0);
  spingroup->setStretchFactor(rep->zoom, 0);
  spingroup->setStretchFactor(hdummy, 1);
  spingroup->setSpacing(4);

  header->setStretchFactor(rep->title, 0);
  header->setStretchFactor(spingroup, 0);
  header->setSpacing(4);

  QHBox* buttons = new QHBox(this);
  QPushButton* savebutton = new QPushButton(buttons);
  savebutton->setText("Save this image");
  QFrame* hspacer = new QFrame(buttons);
  buttons->setStretchFactor(savebutton, 0);
  buttons->setStretchFactor(hspacer, 1);

  this->connect(savebutton, SIGNAL(clicked()),
                this, SLOT(saveImage()));

  QFrame* hline = new QFrame(this);
  rep->frame = new QFrame(this);
  QFrame* vdummy = new QFrame(this);

  hline->setFrameShape(QFrame::HLine);
  hline->setFrameShadow(QFrame::Raised);
  hline->setLineWidth(2);

  this->setStretchFactor(header, 0);
  this->setStretchFactor(buttons, 0);
  this->setStretchFactor(hline, 0);
  this->setStretchFactor(vdummy, 1);
  this->setSpacing(4);

  this->connect(rep->zoom, SIGNAL(valueChanged(int)),
                this, SLOT(setZoom(int)));
}

QtImageFrame::~QtImageFrame()
{
  delete rep;
}

void QtImageFrame::setFrame(const GenericFrame& frame,
                            const std::string& title,
                            const int frameNumber,
                            const FrameInfo& auxinfo)
{
  rep->genFrame = frame;
  rep->genFrame.setFloatFlags(frame.floatFlags() | FLOAT_NORM_0_255);

  rep->shortname = sformat("%s%06d", title.c_str(), frameNumber);
  rep->titlestring = buildTitle(frame, title, frameNumber, auxinfo);

  rep->isUpdated = false;

  if (this->isVisible())
    rep->update();
  else
    rep->updateSize();
}

void QtImageFrame::setZoom(int z)
{
  rep->isUpdated = false;

  if (this->isVisible())
    rep->update();
}

void QtImageFrame::saveImage()
{
  QString s = QFileDialog::getSaveFileName
    (sformat("./%s.png", rep->shortname.c_str()),
     QString::null, /* e.g., "Images (*.png *.xpm *.jpg)" */
     this,
     "save file dialog",
     "Choose a filename to save under");

  if (!s.isEmpty())
    {
      try
        {
          Raster::WriteFrame(rep->genFrame, s);
        }
      catch (std::exception& e)
        {
          QErrorMessage* dlg = new QErrorMessage(this);
          dlg->message(e.what());
        }
    }
}

void QtImageFrame::showEvent(QShowEvent* event)
{
  rep->update();
}

#endif // INVT_HAVE_QT3

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // GUI_QTIMAGEFRAME_Q_DEFINED
