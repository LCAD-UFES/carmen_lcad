/*
 * VisionGUI.qt.C
 *
 *        Description: This class interacts with ICE to receive messages
 *                        from the image processing components.  Right now it only interfaces
 *                        with the Retina (to add other ones in, you need to define your
 *                        ICE message, edit the menu so that you can specify whether or not
 *                        to overlay your stuff).  Also, this is a VisionBrainComponentI as
 *                        we are listening to the RetinaMessages for the images to display.
 *
 *
 *                        TODO: 1) Integrate with ObjectFinder
 *                        TODO: 2) Add a FrameRate submenu to the file menu and have it change
 *                                                the VBCI's framerate variable.
 *
 */

#include "Robots/BeoHawk/gui/VisionGUI.qt.H"
#include "QtUtil/ImageConvert4.H"

VisionGUI::VisionGUI(OptionManager &mgr,
                const std::string &descrName, const std::string &tagName) :
                VisionBrainComponentI(mgr, descrName, tagName) {

        ui.setupUi(this);
        connect (ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
        connect (this, SIGNAL(imageReady()), this, SLOT(updateImageLabel()));
        connect (ui.actionSave_Frame_as_Image, SIGNAL(triggered()), this, SLOT(saveImage()));
}

VisionGUI::~VisionGUI() {
}

void VisionGUI::registerTopics() {

        LINFO("Registering VisionGUI Messages/Subscriptions");
        registerVisionTopics();
}

void VisionGUI::updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera) {

        cleanImage = img;
        if (!ui.actionFreeze_Frame->isChecked()) {
                curImage = convertToQImage4(img);
                emit imageReady();
        }
}
