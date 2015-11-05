#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QtGui>

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Robots/SeaBeeIII/GUI/CameraDisplay.qt.H"
#include "Robots/SeaBeeIII/GUI/IceCommunicator.qt.H"

class MainWindow : public QMainWindow, public ModelComponent
{
  Q_OBJECT

  public:
    //! Create a new MainWindow
    MainWindow(OptionManager& mgr,
        const std::string& descrName = "MainWindow",
        const std::string& tagName = "MainWindow");

    void start2();

    void initIce(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter);

   QGridLayout* constructImageDisplays();

  private:
    CameraDisplay *itsFwdLeftCamera;
    CameraDisplay *itsFwdRightCamera;
    CameraDisplay *itsDwnLeftCamera;
    CameraDisplay *itsDwnRightCamera;

    nub::ref<IceCommunicator> itsIceCommunicator;


};

#endif //MAINWINDOW_H

