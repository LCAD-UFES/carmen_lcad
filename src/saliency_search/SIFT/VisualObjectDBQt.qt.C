/*!@file SIFT/VisualObjectDBQt.qt.C Qt GUI around VisualObjectDB */

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
// Primary maintainer for this file: Laurent Itti
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Apps/BorderWatch/BorderWatchQt.qt.C $
// $Id: BorderWatchQt.qt.C 13059 2010-03-26 08:14:32Z itti $
//

#include "SIFT/VisualObjectDBQt.qt.H"

#include <QtCore/QTimer>
#include <QtGui/QLabel>
#include <QtGui/QVBoxLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QListWidget>
#include <QtGui/QSplitter>
#include <QtGui/QFrame>
#include <QtGui/QAction>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QFileDialog>
#include <QtGui/QStatusBar>
#include <QtGui/QMessageBox>

#include "QtUtil/ImageConvert4.H"
#include "Raster/Raster.H"
#include "Util/FileUtil.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "Util/StringUtil.H"

#include <dirent.h>

// ######################################################################
VisualObjectDBQt::VisualObjectDBQt(QWidget* parent) :
  QMainWindow(parent), itsVDBchanged(false)
{
  QAction *loadaction = new QAction("&Load VisualObjectDB", this);
  QAction *saveaction = new QAction("&Save VisualObjectDB", this);
  QAction *quitaction = new QAction("&Quit", this);

  QMenu *file = menuBar()->addMenu("&File");
  file->addAction(loadaction); connect(loadaction, SIGNAL(triggered()), this, SLOT(load()));
  file->addAction(saveaction); connect(saveaction, SIGNAL(triggered()), this, SLOT(save()));
  file->addAction(quitaction); connect(quitaction, SIGNAL(triggered()), this, SLOT(close()));

  QAction *importaction = new QAction("I&mport Image", this);
  QAction *rimportaction = new QAction("&Recursive Import Images", this);

  QMenu *import = menuBar()->addMenu("&Import");
  import->addAction(importaction); connect(importaction, SIGNAL(triggered()), this, SLOT(import()));
  import->addAction(rimportaction); connect(rimportaction, SIGNAL(triggered()), this, SLOT(rimport()));

  QAction *matchaction = new QAction("&Match against Image", this);

  QMenu *process = menuBar()->addMenu("&Processing");
  process->addAction(matchaction); connect(matchaction, SIGNAL(triggered()), this, SLOT(match()));

  QVBoxLayout *main = new QVBoxLayout();
  main->setSpacing(4);
  main->setMargin(2);

  QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
  splitter->setChildrenCollapsible(false);

  itsListWidget = new QListWidget(this);
  itsListWidget->setMinimumWidth(180);
  itsListWidget->setSortingEnabled(true); // sort list entries

  splitter->addWidget(itsListWidget);
  connect(itsListWidget, SIGNAL(currentRowChanged(int)), this, SLOT(listChanged(int)));

  itsFrameWidget = new QLabel(this);
  splitter->addWidget(itsFrameWidget);

  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 7);  // preferentially stretch the image pane over the text list pane

  splitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  main->addWidget(splitter);

  QWidget *centralWidget = new QWidget;
  setCentralWidget(centralWidget);
  centralWidget->setLayout(main);

  statusBar(); // initialize the statusbar
  statusBar()->showMessage("Status: Idle. Use the File menu to load a VisualObjectDB.");
}

// ######################################################################
VisualObjectDBQt::~VisualObjectDBQt()
{ }

// ######################################################################
bool VisualObjectDBQt::load()
{
  asksave();

  QString fileName = QFileDialog::getOpenFileName(this, tr("Select VisualObjectDB File to Load"),
                                                  QString(""), tr("VisualObjectDB Files (*.vdb)"));
  statusBar()->showMessage(sformat("Loading: %s ...", fileName.toLatin1().data()).c_str()); repaint();
  LINFO("Loading: %s", fileName.toLatin1().data());

  if (itsVDB.loadFrom(fileName.toStdString(), false)) {
    itsFileName = fileName.toStdString();
    LINFO("Loaded VisualObjectDB with %u objects.", itsVDB.numObjects());
  } else {
    itsFileName = "";
    LINFO("Error loading VisualObjectDB");
    QMessageBox::warning(this, tr("VisualObjectDBQt"), tr("Error loading VisualObjectDB"),
                             QMessageBox::Ok | QMessageBox::Default);
    return false;
  }

  // refresh our list:
  refreshList();

  return true;
}

// ######################################################################
void VisualObjectDBQt::refreshList()
{
  itsListWidget->clear();
  for (uint i = 0; i < itsVDB.numObjects(); ++i) itsListWidget->addItem(itsVDB.getObject(i)->getName().c_str());
  if (itsVDB.numObjects() > 0) itsListWidget->setCurrentRow(0);
}

// ######################################################################
bool VisualObjectDBQt::save()
{
  if (itsFileName.empty()) {
    QString fileName = QFileDialog::getSaveFileName(this, tr("Select VisualObjectDB File to Save"),
                                                  QString(""), tr("VisualObjectDB Files (*.vdb)"));
    itsFileName = fileName.toStdString();
    if (itsFileName.empty()) return false;
  }

  if (itsVDB.saveTo(itsFileName)) {
    LINFO("Saved VisualObjectDB with %u objects.", itsVDB.numObjects());
    itsVDBchanged = false;
    return true;
  }

  QMessageBox::warning(this, tr("VisualObjectDBQt"), tr("Error saving VisualObjectDB"),
                       QMessageBox::Ok | QMessageBox::Default);
  return false;
}

// ######################################################################
bool VisualObjectDBQt::import()
{
  QString fileName = QFileDialog::getOpenFileName(this, tr("Select Image File to Load"),
                                                  QString(""), tr("Image Files (*.png *.pnm *.ppm *.jpg)"));
  itsVDBchanged = importcore(fileName.toLatin1().data());

  refreshList(); 

  return itsVDBchanged;
}

// ######################################################################
bool VisualObjectDBQt::importcore(const char *fil)
{
  statusBar()->showMessage(sformat("Importing image: %s", fil).c_str()); repaint();

  QFileInfo fi(fil);
  std::string path = fi.canonicalPath().toStdString();
  std::string name = fi.completeBaseName().toStdString();
  std::vector<std::string> tokens; split(path, "/", std::back_inserter(tokens));
  if (tokens.size() >= 2) name = tokens[tokens.size()-2] + ':' + tokens[tokens.size()-1] + ':' + name;

  Image< PixRGB<byte> > img = Raster::ReadRGB(fil);

  rutz::shared_ptr<VisualObject> obj(new VisualObject(name, fil, img));
  if (itsVDB.addObject(obj)) {
    LINFO("Added VisualObject '%s'", name.c_str());
    return true;
  }

  QMessageBox::warning(this, tr("VisualObjectDBQt"), tr("Error adding VisualObject"),
                       QMessageBox::Ok | QMessageBox::Default);
  return false;
}

// ######################################################################
bool VisualObjectDBQt::rimport()
{
  QString fileName = QFileDialog::getExistingDirectory(this, tr("Select Directory to Load"));
  statusBar()->showMessage(sformat("Recursively importing images from: %s", fileName.toLatin1().data()).c_str());
  repaint();

  itsVDBchanged = rimportcore(fileName.toLatin1().data());

  refreshList();
  return itsVDBchanged;
}

// ######################################################################
bool VisualObjectDBQt::rimportcore(const char *dir)
{
  LINFO("Considering: %s", dir);

  bool ret = true;
  DIR *dp = opendir(dir); dirent *dirp;
  while( (dirp = readdir(dp)) ) {
    if (dirp->d_name[0] != '.') {
      std::string fil = sformat("%s/%s", dir, dirp->d_name);
      if (isDirectory(dirp)) ret &= rimportcore(fil.c_str()); else ret &= importcore(fil.c_str());
    }
  }
  closedir(dp);

  return ret;
}

// ######################################################################
void VisualObjectDBQt::listChanged(const int idx)
{
  if (itsListWidget->count() && idx >= 0)
    {
      const QListWidgetItem *item = itsListWidget->item(idx);
      std::string oname = item->text().toStdString();

      rutz::shared_ptr<VisualObject> obj = itsVDB.getObject(oname);
      if (obj.is_invalid()) LFATAL("Selected a non-existent object!");

      const Image<PixRGB<byte> > &img = obj->getKeypointImage(1.0F); //getImage();

      // display the image in our widget:
      QPixmap pixmap = convertToQPixmap4(img);
      itsFrameWidget->setPixmap(pixmap);

      statusBar()->showMessage(sformat("%u keypoints, %u features -- %s", obj->numKeypoints(), obj->numFeatures(),
                                       obj->getImageFname().c_str()).c_str());
    }
}

// ######################################################################
bool VisualObjectDBQt::match()
{
  QString fileName = QFileDialog::getOpenFileName(this, tr("Select Image File to Load"),
                                                  QString(""), tr("Image Files (*.png *.pnm *.ppm *.jpg)"));

  statusBar()->showMessage(sformat("Matching image: %s", fileName.toLatin1().data()).c_str()); repaint();

  Image< PixRGB<byte> > img = Raster::ReadRGB(fileName.toLatin1().data());
  rutz::shared_ptr<VisualObject> obj(new VisualObject("Test Object", fileName.toLatin1().data(), img));

  std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;
  if (itsVDB.getObjectMatchesParallel(obj, matches, 50 /* numthreads */, 0.05F, 0.95F, 0.5F, 5U, 9U, false)) {
    std::string msg = "Object matches found:\n\n";

    size_t bestidx = 0; float bestscore = -1.0F;
    for (size_t i = 0; i < matches.size(); ++i) {
      rutz::shared_ptr<VisualObject> ref = matches[i]->getVoTest();
      LINFO("Matched %s, score = %f", ref->getName().c_str(), matches[i]->getScore());
      msg += sformat("%s: score = %f\n", ref->getName().c_str(), matches[i]->getScore());

      if (matches[i]->getScore() > bestscore) { bestscore = matches[i]->getScore(); bestidx = i; }
    }

    msg += sformat("\nShowing image for match %" ZU , bestidx);
    QMessageBox mbox(this);
    mbox.setInformativeText(msg.c_str());
    mbox.addButton(QMessageBox::Ok);
    QPixmap pixmap = convertToQPixmap4(matches[bestidx]->getMatchImage(0.5F));
    mbox.setIconPixmap(pixmap);
    mbox.exec();
    return true;
  } else {
    LINFO("No good object match found.");
    QMessageBox::information(this, tr("VisualObjectDBQt"), tr("No good object match found!"),
                             QMessageBox::Ok | QMessageBox::Default);
    return false;
  }
}

// ######################################################################
void VisualObjectDBQt::asksave()
{
  if (itsVDBchanged) {
    int r = QMessageBox::warning(this, tr("VisualObjectDBQt"),
                                 tr("The VisualObjectDB has been modified.\n"
                                    "Do you want to save your changes?"),
                                 QMessageBox::Yes | QMessageBox::Default,
                                 QMessageBox::No | QMessageBox::Escape);
    if (r == QMessageBox::Yes) itsVDBchanged = save();
    else if (r == QMessageBox::No) itsVDBchanged = false;
  }
}

// ######################################################################
void VisualObjectDBQt::closeEvent(QCloseEvent *event)
{
  asksave(); // do an event->accept() or event->ignore()
  close();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
