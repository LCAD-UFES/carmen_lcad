/*
 * VisionGUI.qt.H
 *
 */

#include <QtGui/QMainWindow>
#include <QtGui/QImage>
#include <QtCore/QString>
#include <QtGui/QFileDialog>
#include "Robots/BeoHawk/gui/ui_VisionGUI.qt.H"

#include "Component/ModelManager.H"
#include "Component/OptionManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/BeoHawk/vision/VisionBrainComponentI.H"
#include "Image/Image.H"
#include "Image/Pixels.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"

#include <IceUtil/Thread.h>

#ifndef VISIONGUI_H_
#define VISIONGUI_H_

class VisionGUI : public QMainWindow, public VisionBrainComponentI{

        Q_OBJECT

public:
        VisionGUI(OptionManager &mgr,
                        const std::string &descrName = "VisionGUI",
                        const std::string &tagName = "VisionGUI");
        virtual ~VisionGUI();

        virtual void registerTopics();
        virtual void updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera);

signals:
    void imageReady();

public slots:
        void updateImageLabel() {
                ui.imageLabel->setPixmap(QPixmap::fromImage(curImage));
        }

        void saveImage() {
                QString filename = QFileDialog::getSaveFileName(this, "Save Frame as...", "",
                                "Images (*.png, *.jpg, *.bmp)");
                curImage.save(filename);
        }


private:
        Image<PixRGB<byte> > cleanImage;
    QImage curImage;
        Ui::VisionGUI ui;

};

#endif /* VISIONGUI_H_ */
