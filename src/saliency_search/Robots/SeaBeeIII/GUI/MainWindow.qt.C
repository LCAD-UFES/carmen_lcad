#ifndef MAINWINDOW_C
#define MAINWINDOW_C

#include "Robots/SeaBeeIII/GUI/MainWindow.qt.H"

MainWindow::MainWindow(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsIceCommunicator(new IceCommunicator(mgr))
{
  addSubComponent(itsIceCommunicator);

}

QGridLayout* MainWindow::constructImageDisplays()
{
  QGridLayout * camLayout = new QGridLayout;

  itsFwdLeftCamera  = new CameraDisplay(this);
  itsFwdRightCamera = new CameraDisplay(this);
  itsDwnLeftCamera  = new CameraDisplay(this);
  itsDwnRightCamera = new CameraDisplay(this);

  camLayout->addWidget(itsFwdLeftCamera,  0, 0);
  camLayout->addWidget(itsFwdRightCamera, 0, 1);
  camLayout->addWidget(itsDwnLeftCamera,  1, 0);
  camLayout->addWidget(itsDwnRightCamera, 1, 1);

  connect( itsIceCommunicator.get(), SIGNAL(NewFwdLeftImg(QImage)),  itsFwdLeftCamera,  SLOT(updateImage(QImage)));
  connect( itsIceCommunicator.get(), SIGNAL(NewFwdRightImg(QImage)), itsFwdRightCamera, SLOT(updateImage(QImage)));
  connect( itsIceCommunicator.get(), SIGNAL(NewDwnLeftImg(QImage)),  itsDwnLeftCamera,  SLOT(updateImage(QImage)));
  connect( itsIceCommunicator.get(), SIGNAL(NewDwnRightImg(QImage)), itsDwnRightCamera, SLOT(updateImage(QImage)));

  return camLayout;

}

void MainWindow::initIce(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter)
{
  itsIceCommunicator->init(ic, adapter);
}

void MainWindow::start2()
{
  QVBoxLayout * mainLayout = new QVBoxLayout;

  QGridLayout * imageDisplayLayout = constructImageDisplays();

  mainLayout->addLayout(imageDisplayLayout);

  //Perform some kind of Qt magic
  QWidget *centralWidget = new QWidget;
  setCentralWidget(centralWidget);

  centralWidget->setLayout(mainLayout);

}


#endif //MAINWINDOW_C


