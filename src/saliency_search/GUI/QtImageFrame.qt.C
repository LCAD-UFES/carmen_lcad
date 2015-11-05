/*!@file GUI/QtImageFrame.qt.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/QtImageFrame.qt.C $
// $Id: QtImageFrame.qt.C 14290 2010-12-01 21:44:03Z itti $
//

#include "GUI/QtImageFrame.qt.H"

#include "Image/ColorOps.H"
#include "Image/FilterOps.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/Normalize.H"
#include "Image/Pixels.H"
#include "Image/Range.H"
#include "Image/ShapeOps.H"
#include "QtUtil/ImageConvert4.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Transport/FrameInfo.H"
#include "Util/MathFunctions.H"
#include "Util/sformat.H"
#include "rutz/demangle.h"
#include "rutz/trace.h"

#include <QtGui/QFrame>
#include <QtGui/QShowEvent>
#include <QtGui/QPixmap>
#include <QtGui/QErrorMessage>
#include <QtGui/QFileDialog>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QHBoxLayout>

#include <typeinfo>

namespace
{
  const int MAXZOOM = 6;
  const int MINZOOM = -6;

  // ######################################################################
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

  // ######################################################################
  static std::string buildTitle(const GenericFrame& img, const std::string& title,
                                const int frameNumber, const FrameInfo& auxinfo)
  {
    std::string result;

    if (auxinfo.description.length() > 0)
      result += sformat("<b>%s</b>", auxinfo.description.c_str());

    result += "<br>";

    result += sformat("<b>%s</b> [frame #%06d]<br><i>%dx%d %s</i>",
                      title.c_str(), frameNumber, img.getDims().w(), img.getDims().h(),
                      richTextEncode(img.nativeTypeName()).c_str());

    result += "<br>";

    if (auxinfo.srcpos.m_file_name != 0)
      result += sformat("from %s:%d", auxinfo.srcpos.m_file_name, auxinfo.srcpos.m_line_no);

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

// ######################################################################
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
  QLabel* frame;

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

    if (raw.isEmpty()) return raw;
    else if (this->zoom->value() >= 0) return Dims(raw.w() << this->zoom->value(), raw.h() << this->zoom->value());

    // else...
    return Dims(raw.w() >> -(this->zoom->value()), raw.h() >> -(this->zoom->value()));
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
                   this->zoom->value() < this->zoom->maximum())
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

    if (dims.isNonEmpty()) this->frame->setFixedSize(dims.w(), dims.h());
  }

  template <class T>
  Image<T> scaleImage(const Image<T>& img)
  {
    if (this->zoom->value() >= 0)
      return zoomXY(img, (1 << this->zoom->value()), (1 << this->zoom->value()));
    else
      {
        Image<T> result = img;
        for (int i = 0; i < -this->zoom->value(); ++i) result = decXY(lowPass3(result));
        return result;
      }
  }

  void update()
  {
    if (this->isUpdated) return;

    GVX_TRACE(__PRETTY_FUNCTION__);

    this->setInitialDims();

    QPixmap pixmap;

    switch (this->genFrame.nativeType())
      {
      case GenericFrame::NONE:
        break;

      case GenericFrame::RGB_U8:
      case GenericFrame::RGBD:
      case GenericFrame::RGB_F32:
      case GenericFrame::VIDEO:
        pixmap = convertToQPixmap4(scaleImage(this->genFrame.asRgbU8()));
        break;

      case GenericFrame::GRAY_U8:
      case GenericFrame::GRAY_F32:
        pixmap = convertToQPixmap4(scaleImage(this->genFrame.asGrayU8()));
        break;

      case GenericFrame::RGB_U16:
        break;
      case GenericFrame::GRAY_U16:
        break;
      }

    this->title->setText(titlestring.c_str());
    this->title->setTextFormat(Qt::RichText);
    this->title->setAlignment(Qt::Alignment(Qt::AlignLeft|Qt::AlignVCenter|Qt::TextExpandTabs|Qt::TextSingleLine));

    this->frame->setPixmap(pixmap);

    this->updateSize();

    this->isUpdated = true;
  }
};

// ######################################################################
QtImageFrame::QtImageFrame(QWidget* parent, const Dims& preferredDims, const Dims& preferredMaxDims) :
  QWidget(parent), rep(new Impl)
{
  rep->preferredDims = preferredDims;
  rep->preferredMaxDims = preferredMaxDims;

  QVBoxLayout *main = new QVBoxLayout(this);
  main->setSpacing(4);
  main->setMargin(2);

  QVBoxLayout* header = new QVBoxLayout;
  main->addLayout(header);

  rep->title = new QLabel(this);
  header->addWidget(rep->title);

  QHBoxLayout* spingroup = new QHBoxLayout;
  spingroup->setSpacing(4);

  QLabel* spinlabel = new QLabel("zoom:", this);
  spingroup->addWidget(spinlabel);

  rep->zoom = new QSpinBox(this);
  spingroup->addWidget(rep->zoom);
  rep->zoom->setMinimum(MINZOOM);
  rep->zoom->setMaximum(MAXZOOM);
  rep->zoom->setSingleStep(1);
  rep->zoom->setButtonSymbols(QSpinBox::PlusMinus);
  rep->zoom->setValue(0);
  rep->zoom->setPrefix("pow(2, ");
  rep->zoom->setSuffix(")");

  spingroup->addStretch(1);

  QPushButton* savebutton = new QPushButton(this);
  savebutton->setText("Save this image");
  spingroup->addWidget(savebutton);

  spingroup->addStretch(1);

  this->connect(savebutton, SIGNAL(clicked()), this, SLOT(saveImage()));

  header->addLayout(spingroup);

  QFrame* hline = new QFrame(this);
  hline->setFrameShape(QFrame::HLine);
  hline->setFrameShadow(QFrame::Raised);
  hline->setLineWidth(2);
  main->addWidget(hline);

  main->addStretch(1);

  QHBoxLayout* himage = new QHBoxLayout;
  himage->addStretch(1);
  rep->frame = new QLabel(this);
  himage->addWidget(rep->frame);
  himage->addStretch(1);

  main->addLayout(himage);

  main->addStretch(1);

  this->connect(rep->zoom, SIGNAL(valueChanged(int)), this, SLOT(setZoom(int)));

  this->setLayout(main);
}

QtImageFrame::~QtImageFrame()
{
  delete rep;
}

void QtImageFrame::setFrame(const GenericFrame& frame, const std::string& title,
                            const int frameNumber, const FrameInfo& auxinfo)
{
  rep->genFrame = frame;
  rep->genFrame.setFloatFlags(frame.floatFlags() | FLOAT_NORM_0_255);

  rep->shortname = sformat("%s%06d", title.c_str(), frameNumber);
  rep->titlestring = buildTitle(frame, title, frameNumber, auxinfo);

  rep->isUpdated = false;

  if (this->isVisible()) rep->update();
  else rep->updateSize();
}

void QtImageFrame::setZoom(int z)
{
  rep->isUpdated = false;

  if (this->isVisible()) rep->update();
}

void QtImageFrame::saveImage()
{
  QString s = QFileDialog::getSaveFileName(this, QString("Select filename to save to"),
                                           QString(sformat("./%s.png", rep->shortname.c_str()).c_str()));

  if (!s.isEmpty())
    {
      try
        {
          Raster::WriteFrame(rep->genFrame, s.toStdString());
        }
      catch (std::exception& e)
        {
          QErrorMessage* dlg = new QErrorMessage(this);
          dlg->showMessage(e.what());
        }
    }
}

void QtImageFrame::showEvent(QShowEvent* event)
{
  rep->update();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
