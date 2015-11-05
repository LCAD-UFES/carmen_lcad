/*
 * ObjectFinder.C
 *
 *
 */

#include "Robots/BeoHawk/vision/ObjectFinder.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include <dirent.h>

const ModelOptionCateg MOC_ObjectFinder = {
    MOC_SORTPRI_3, "ObjectFinder Related Options" };

const ModelOptionDef OPT_ImagesDir =
{ MODOPT_ARG(std::string), "ImagesDir", &MOC_ObjectFinder, OPTEXP_CORE,
        "The directory in which we should search for images to build the database for the Object Finder."
        "  To avoid having to check for whether the files are of valid type, ONLY images may be stored"
        "in this directory.",
        "images-dir", '\0', "<string>", "images" };

ObjectFinder::ObjectFinder(OptionManager &mgr,
                const std::string &descrName, const std::string &tagName) :
                VisionBrainComponentI(mgr, descrName, tagName),
                imagesDir(&OPT_ImagesDir, this, 0) {

}

ObjectFinder::~ObjectFinder() {
}

void ObjectFinder::registerTopics() {

        LINFO("Registering ObjectFinder Messages/Subscriptions");
        registerVisionTopics();
}

void ObjectFinder::updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera) {

        LINFO("Image Received: %d", itsFrameCount);
        //rutz::shared_ptr<VisualObject> vo(new VisualObject("CurrentFrame", "CurrentFrame", img));
        featureDB.searchDB(img);
        LINFO("##### Size is %dx%d", img.getWidth(), img.getHeight());
        //analyzer.extractFeatures(img);
}


void ObjectFinder::buildFeatureDB() {

        std::string imageDirName = imagesDir.getVal();
        if (imageDirName.rfind('/') != imageDirName.length() - 1)
                imageDirName = imageDirName + "/";

        DIR * imageDir = opendir(imageDirName.c_str());
        struct dirent * curFile;

        if (imageDir == NULL) {
                LFATAL("Invalid directory passed (%s).", imageDirName.c_str());
                return;
        }

        while ((curFile = readdir(imageDir))) {
                if (curFile->d_name[0] != '.') {
                        std::string fullFilename = std::string(curFile->d_name);
                        std::string truncFilename = std::string(curFile->d_name);
                        if (fullFilename.rfind('.') > 0)
                                truncFilename = truncFilename.substr(0, fullFilename.rfind('.'));

                        //Image<PixRGB<byte> > curImage = Raster::ReadRGB(imageDirName + fullFilename);
                        //rutz::shared_ptr<VisualObject> curVO(new VisualObject(truncFilename,
                        //                                                                                fullFilename, curImage));

                        //if (featureDB.addObject(curVO))
                        //        LINFO("SUCCESS: Added image (%s) to DB.", curFile->d_name);
                        //else
                                LINFO("FAILURE: Could not add image (%s) to DB.", curFile->d_name);
                }
        }


        closedir(imageDir);
}
