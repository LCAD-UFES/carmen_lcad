/*
 * ObjectFinder.H
 *
 *        Uses SIFT/SURF? to find specified images in a camera feed.
 */
#include "Component/ModelManager.H"
#include "Component/OptionManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/BeoHawk/vision/VisionBrainComponentI.H"
#include "Image/Image.H"
#include "Image/Pixels.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Robots/BeoHawk/BeoHawkSimEvents.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>

#ifndef OBJECTFINDER_H_
#define OBJECTFINDER_H_

class ObjectFinder : public VisionBrainComponentI {

public:
        ObjectFinder(OptionManager &mgr,
                        const std::string &descrName = "ObjectFinder",
                        const std::string &tagName = "ObjectFinder");
        virtual ~ObjectFinder();

        virtual void registerTopics();
        virtual void updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera);
        void buildFeatureDB();

private:
        OModelParam<std::string> imagesDir;
        FastFeatureDB featureDB;



};

#endif /* OBJECTFINDER_H_ */
