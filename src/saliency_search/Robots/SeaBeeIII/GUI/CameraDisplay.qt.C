#ifndef CAMERADISPLAY_QT_C
#define CAMERADISPLAY_QT_C

#include "QtUtil/ImageConvert4.H"

CameraDisplay::CameraDisplay(QWidget *parent) :
  QGraphicsView(parent)
{
  setRenderHints(QPainter::Antialiasing);

  itsScene = new QGraphicsScene;

  itsImageDisplay = new ImageGraphicsItem();

  itsScene->addItem(itsImageDisplay);

  this->setScene(itsScene);

}

void CameraDisplay::updateImage(QImage img)
{
  LINFO("Got Image!!");
  itsImageDisplay->setImage(img);
  itsImageDisplay->update();
  this->setSceneRect(itsImageDisplay->getRect());
}



#endif //CAMERADISPLAY_QT_C
