#ifndef CAMERADISPLAY_QT_H
#define CAMERADISPLAY_QT_H

#include <QtGui/QtGui>

#include "Qt4/ImageGraphicsItem.qt.H"

class CameraDisplay : public QGraphicsView
{
  Q_OBJECT

  public:
    CameraDisplay(QWidget *parent=0);

  public slots:
    void updateImage(QImage);

  private:
    //! The Graphics Item which is responsible for drawing the camera image
    ImageGraphicsItem *itsImageDisplay;

    //! This Graphics Scene holds the background image, and all marker objects
    QGraphicsScene *itsScene;

};

#endif //CAMERADISPLAY_QT_H
